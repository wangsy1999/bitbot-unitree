#pragma once

#include "CommonLocoInferenceWorker.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <type_traits>

namespace z {
// G1 DreamWaQ: current frame [1,96], term-major five-frame history [1,480].
// All joint vectors on the scheduler bus are already in policy order.
template<typename SchedulerType, CTString NetName, typename Precision, size_t JointCount>
class DreamWaQInferenceWorker
    : public CommonLocoInferenceWorker<SchedulerType, NetName, Precision, JointCount> {
    static_assert(std::is_same_v<Precision, float> && JointCount == 29);
    using Base = CommonLocoInferenceWorker<SchedulerType, NetName, Precision, JointCount>;
    using MotorVec = math::Vector<Precision, JointCount>;
    using Vec3 = math::Vector<Precision, 3>;
    static constexpr size_t FrameSize = 9 + 3 * JointCount;
    static constexpr size_t HistoryLength = 5;
    using Frame = std::array<Precision, FrameSize>;

public:
    DreamWaQInferenceWorker(typename SchedulerType::Ptr scheduler,
                            const nlohmann::json& net, const nlohmann::json& motor)
        : Base(scheduler, net, motor) {
        frame_scale_ = math::cat(this->Scales_ang_vel, this->Scales_project_gravity,
                                Vec3::ones(), this->Scales_dof_pos,
                                this->Scales_dof_vel, this->Scales_last_action);
        // Detect both the original dual-input export and history-only exports.
        const auto count = this->Session__.GetInputCount();
        if ((count != 1 && count != 2) || this->Session__.GetOutputCount() != 1)
            throw std::runtime_error("DreamWaQ expects one or two inputs and one output");
        this->InputNodeNames__.clear();
        this->OutputNodeNames__.clear();
        bool has_frame = false, has_history = false;
        for (size_t i = 0; i < count; ++i) {
            auto type = this->Session__.GetInputTypeInfo(i);
            auto info = type.GetTensorTypeAndShapeInfo();
            const auto shape = info.GetShape();
            if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
                shape.size() != 2 || shape[0] != 1)
                throw std::runtime_error("DreamWaQ expects float32 [1,96] and/or [1,480]");
            if (shape[1] == FrameSize && !has_frame) {
                has_frame = true;
                this->InputOrtTensors__.push_back(this->WarpOrtTensor(current_));
            } else if (shape[1] == FrameSize * HistoryLength && !has_history) {
                has_history = true;
                this->InputOrtTensors__.push_back(this->WarpOrtTensor(stacked_));
            } else {
                throw std::runtime_error("DreamWaQ input shape is unsupported or duplicated");
            }
            this->InputNodeNames__.emplace_back(
                this->Session__.GetInputNameAllocated(i, this->DefaultAllocator__).get());
        }
        if (!has_history || (count == 2 && !has_frame))
            throw std::runtime_error("DreamWaQ requires a history input");
        auto type = this->Session__.GetOutputTypeInfo(0);
        auto info = type.GetTensorTypeAndShapeInfo();
        if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            info.GetShape() != std::vector<int64_t>{1, JointCount})
            throw std::runtime_error("DreamWaQ expects float32 actions [1,29]");
        this->OutputNodeNames__.emplace_back(
            this->Session__.GetOutputNameAllocated(0, this->DefaultAllocator__).get());
        this->OutputOrtTensors__.push_back(this->WarpOrtTensor(output_));
        current_.Array().fill(0);
        stacked_.Array().fill(0);
        output_.Array().fill(0);
    }

    // The policy thread owns history and last_action; UI events only request a reset.
    void reset() { reset_requested_.store(true); }

    void PreProcess() override {
        start_ = std::chrono::steady_clock::now();
        if (reset_requested_.exchange(false)) {
            initialized_ = false;
            last_action_.fill(0);
            this->Scheduler->template SetData<concat(NetName, "NetLastAction")>(last_action_);
        }
        MotorVec q, dq;
        Vec3 omega, angles, command;
        this->Scheduler->template GetData<"CurrentMotorPosition">(q);
        this->Scheduler->template GetData<"CurrentMotorVelocity">(dq);
        this->Scheduler->template GetData<"AngleVelocityValue">(omega);
        this->Scheduler->template GetData<"AngleValue">(angles);
        this->Scheduler->template GetData<concat(NetName, "NetUserCommand3")>(command);
        // RPY are radians from the pelvis IMU. Equivalent to inverse quaternion
        // rotation of world gravity [0,0,-1] used by g1_deploy.
        const Precision roll = angles[0], pitch = angles[1];
        Vec3 gravity{std::sin(pitch), -std::sin(roll) * std::cos(pitch),
                     -std::cos(roll) * std::cos(pitch)};
        this->Scheduler->template SetData<concat(NetName, "NetProjectedGravity")>(gravity);
        auto frame = math::cat(omega, gravity, command, q - this->JointDefaultPos,
                               dq, last_action_);
        for (size_t i = 0; i < FrameSize; ++i) {
            auto& value = frame[i];
            value *= frame_scale_[i];
            if (!std::isfinite(value)) throw std::runtime_error("Non-finite DreamWaQ observation");
            value = std::clamp(value, -this->ClipObservation, this->ClipObservation);
        }
        current_.Array() = frame;
        if (!initialized_) {
            history_.fill(frame);
            initialized_ = true;
        } else {
            for (size_t h = 0; h + 1 < HistoryLength; ++h) history_[h] = history_[h + 1];
            history_.back() = frame;
        }
        size_t offset = 0, cursor = 0;
        for (size_t size : {size_t{3}, size_t{3}, size_t{3}, JointCount, JointCount, JointCount}) {
            for (const auto& entry : history_)
                for (size_t i = 0; i < size; ++i) stacked_.Array()[cursor++] = entry[offset + i];
            offset += size;
        }
    }

    void PostProcess() override {
        auto raw = output_.toVector();
        for (const auto value : raw)
            if (!std::isfinite(value)) throw std::runtime_error("Non-finite DreamWaQ action");
        last_action_ = MotorVec::clamp(raw, -this->ClipAction, this->ClipAction);
        auto target = last_action_ * this->ActionScale + this->JointDefaultPos;
        target = MotorVec::clamp(target, this->JointClipLower, this->JointClipUpper);
        // Discard a result if an event requested a reset while inference ran.
        if (reset_requested_.load()) return;
        this->Scheduler->template SetData<concat(NetName, "NetLastAction")>(last_action_);
        this->Scheduler->template SetData<concat(NetName, "Action")>(target);
        const Precision elapsed = std::chrono::duration<Precision, std::milli>(
            std::chrono::steady_clock::now() - start_).count();
        this->Scheduler->template SetData<concat(NetName, "InferenceTime")>(elapsed);
    }

private:
    math::Tensor<Precision, 1, FrameSize> current_;
    math::Tensor<Precision, 1, FrameSize * HistoryLength> stacked_;
    math::Tensor<Precision, 1, JointCount> output_;
    Frame frame_scale_;
    std::array<Frame, HistoryLength> history_{};
    MotorVec last_action_{};
    bool initialized_ = false;
    std::atomic<bool> reset_requested_{true};
    std::chrono::steady_clock::time_point start_;
};
} // namespace z
