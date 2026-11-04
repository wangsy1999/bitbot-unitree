/**
 * @file SuperMimicInferenceWorker.hpp
 * @brief Uni-SuperMimic 观测历史、参考动作及 ONNX 推理。
 */
#pragma once

#include "AbstractInferenceWorker.hpp"
#include <algorithm>
#include <atomic>
#include "SuperMimicMotion.hpp"
#include <filesystem>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace z
{
    template<typename SchedulerType, CTString NetName, typename InferencePrecision, size_t JointNumber>
    class SuperMimicInferenceWorker : public AbstractNetInferenceWorker<SchedulerType, NetName, InferencePrecision>
    {
        static_assert(std::is_same_v<InferencePrecision, float>);
        using Base = AbstractNetInferenceWorker<SchedulerType, NetName, InferencePrecision>;
        using MotorVec = math::Vector<InferencePrecision, JointNumber>;
        using Vec3 = math::Vector<InferencePrecision, 3>;
        using Rotation = std::array<InferencePrecision, 9>;
        static constexpr size_t ObservationSize = 3 * JointNumber + 12;
        static constexpr size_t GoalFrameSize = 2 * JointNumber + 15;

    public:
        SuperMimicInferenceWorker(typename SchedulerType::Ptr scheduler, const nlohmann::json& cfg,
            const nlohmann::json&, const std::array<size_t, JointNumber>& joint_id_map)
            : Base(scheduler, cfg)
        {
            const auto& prep = cfg.at("Preprocess");
            this->HistoryLength__ = prep.at("HistoryLength").template get<size_t>();
            this->GoalFrames__ = prep.at("GoalFrames").template get<size_t>();
            this->CycleTime__ = cfg.at("Network").at("Cycle_time").template get<double>();
            if (!this->HistoryLength__ || !this->GoalFrames__ || !std::isfinite(this->CycleTime__) || this->CycleTime__ <= 0)
                throw std::runtime_error("Invalid SuperMimic history or control period");
            const auto ids = prep.at("JointIds").template get<std::vector<size_t>>();
            if (ids.size() != JointNumber)
                throw std::runtime_error("SuperMimic JointIds size mismatch");
            for (size_t i = 0; i < JointNumber; ++i)
            {
                const auto found = std::find(joint_id_map.begin(), joint_id_map.end(), ids[i]);
                if (found == joint_id_map.end() || std::count(ids.begin(), ids.end(), ids[i]) != 1)
                    throw std::runtime_error("SuperMimic joint map mismatch");
                this->JointMap__[i] = std::distance(joint_id_map.begin(), found);
                this->DefaultPosition__[i] = cfg.at("Postprocess").at("DefaultPosition").at(i).template get<float>();
                this->ActionScale__[i] = cfg.at("Postprocess").at("action_scale").at(i).template get<float>();
            }
            std::ifstream file(prep.at("ReferencePath").template get<std::string>());
            if (!file) throw std::runtime_error("Cannot open SuperMimic ReferencePath");
            const auto reference = nlohmann::json::parse(file);
            if (reference.at("SchemaVersion") != 1 || std::abs(reference.at("ControlDt").template get<double>() - this->CycleTime__) > 1e-9)
                throw std::runtime_error("SuperMimic reference schema/control period mismatch");
            const auto directory = std::filesystem::path(
                prep.at("ReferencePath").template get<std::string>()).parent_path();
            this->Reference__.Load(reference, directory, this->CycleTime__);
            this->Motions__ = this->Reference__.Clips;
            const auto initial = prep.value("InitialMotion", this->Motions__.front().Name);
            const auto found_motion = std::find_if(this->Motions__.begin(), this->Motions__.end(),
                [&initial](const MotionClip& motion) { return motion.Name == initial; });
            if (found_motion == this->Motions__.end()) throw std::runtime_error("Unknown SuperMimic InitialMotion: " + initial);
            this->SelectMotion(std::distance(this->Motions__.begin(), found_motion));
            std::cout << "SuperMimic motions:";
            for (const auto& motion : this->Motions__) std::cout << " " << motion.Name;
            std::cout << "\nSelected motion: " << this->GetMotionName() << std::endl;
            this->History__.resize(this->HistoryLength__ * ObservationSize);
            this->Goal__.resize(this->GoalFrames__ * GoalFrameSize);
            const std::vector<int64_t> history_shape{1, static_cast<int64_t>(this->HistoryLength__), ObservationSize};
            const std::vector<int64_t> goal_shape{1, static_cast<int64_t>(this->Goal__.size())};
            const std::vector<int64_t> action_shape{1, JointNumber};
            if (this->Session__.GetInputCount() != 2 || this->Session__.GetOutputCount() != 1)
                throw std::runtime_error("SuperMimic requires two inputs and one output");
            ValidateTensor(this->Session__.GetInputTypeInfo(0), history_shape);
            ValidateTensor(this->Session__.GetInputTypeInfo(1), goal_shape);
            ValidateTensor(this->Session__.GetOutputTypeInfo(0), action_shape);
            this->InputOrtTensors__.push_back(Ort::Value::CreateTensor<float>(this->MemoryInfo__, this->History__.data(), this->History__.size(), history_shape.data(), history_shape.size()));
            this->InputOrtTensors__.push_back(Ort::Value::CreateTensor<float>(this->MemoryInfo__, this->Goal__.data(), this->Goal__.size(), goal_shape.data(), goal_shape.size()));
            this->OutputOrtTensors__.push_back(Ort::Value::CreateTensor<float>(this->MemoryInfo__, this->Action__.data(), JointNumber, action_shape.data(), action_shape.size()));
        }

        void start() { this->Request__.store(2 * this->SelectedMotion__.load() + 1); }
        void reset() { this->Request__.store(2 * this->SelectedMotion__.load()); }

        size_t GetMotionCount() const { return this->Motions__.size(); }
        std::string GetMotionName() const { return this->Motions__[this->SelectedMotion__.load()].Name; }

        void SelectMotion(size_t index)
        {
            if (index >= this->Motions__.size()) throw std::out_of_range("SuperMimic motion index");
            this->SelectedMotion__.store(index);
            reset();
        }

        void NextMotion() { SelectMotion((this->SelectedMotion__.load() + 1) % this->Motions__.size()); }
        void PreviousMotion() { SelectMotion((this->SelectedMotion__.load() + this->Motions__.size() - 1) % this->Motions__.size()); }

        void PreProcess() override
        {
            this->BeginTime__ = std::chrono::steady_clock::now();
            // Pack selection and play/reset into one atomic request. Only the
            // inference thread changes the active clip, history and playback clock.
            const uint64_t request = this->Request__.exchange(NoRequest__);
            if (request != NoRequest__)
            {
                this->ActiveMotion__ = request / 2;
                this->Started__ = request % 2 == 1;
                this->StartTimestamp__ = this->Scheduler->getTimeStamp();
                this->Initialized__ = false;
                this->LastAction__.fill(0);
            }
            const auto timestamp = this->Scheduler->getTimeStamp();
            const double elapsed = this->Started__ && timestamp >= this->StartTimestamp__
                ? (timestamp - this->StartTimestamp__) * this->Scheduler->getSpinOnceTime() : 0;
            const auto& motion = this->Motions__[this->ActiveMotion__];
            const size_t frame = motion.Start + static_cast<size_t>(std::min(std::floor(elapsed / this->CycleTime__ + 1e-7),
                static_cast<double>(motion.Count - 1)));
            MotorVec position, velocity;
            Vec3 angles, omega;
            this->Scheduler->template GetData<"CurrentMotorPosition">(position);
            this->Scheduler->template GetData<"CurrentMotorVelocity">(velocity);
            this->Scheduler->template GetData<"AngleRaw">(angles);
            this->Scheduler->template GetData<"AngleVelocityRaw">(omega);
            if (!AllFinite(position) || !AllFinite(velocity) || !AllFinite(angles) || !AllFinite(omega))
                throw std::runtime_error("Non-finite SuperMimic feedback");
            const Rotation rotation = FromEuler(angles[0], angles[1], angles[2]);
            if (!this->Initialized__)
            {
                const auto& first = this->Reference__.Frames[motion.Start].Orientation;
                this->HeadingOffset__ = FromEuler(0, 0, angles[2] - std::atan2(first[3], first[0]));
            }
            const auto relative = TransposeMultiply(rotation, Multiply(this->HeadingOffset__, this->Reference__.Frames[frame].Orientation));
            std::array<float, ObservationSize> observation{};
            MotorVec ref_position, ref_velocity;
            for (size_t i = 0; i < JointNumber; ++i)
            {
                const size_t index = this->JointMap__[i];
                observation[i] = position[index] - this->DefaultPosition__[i];
                observation[JointNumber + i] = velocity[index];
                observation[2 * JointNumber + 6 + i] = this->LastAction__[i];
                ref_position[index] = this->Reference__.Frames[frame].JointPosition[i];
                ref_velocity[index] = this->Reference__.Frames[frame].JointVelocity[i];
            }
            for (size_t i = 0; i < 3; ++i)
            {
                observation[2 * JointNumber + i] = omega[i];
                observation[2 * JointNumber + 3 + i] = -rotation[6 + i];
                for (size_t j = 0; j < 2; ++j)
                    observation[3 * JointNumber + 6 + 2 * i + j] = relative[3 * i + j];
            }
            if (!this->Initialized__)
            {
                for (size_t i = 0; i < this->HistoryLength__; ++i)
                    std::copy(observation.begin(), observation.end(), this->History__.begin() + i * ObservationSize);
                this->Initialized__ = true;
            }
            else
            {
                std::move(this->History__.begin() + ObservationSize, this->History__.end(), this->History__.begin());
                std::copy(observation.begin(), observation.end(), this->History__.end() - ObservationSize);
            }
            this->Reference__.WriteGoal(frame, motion.Start + motion.Count - 1, this->Goal__);
            this->Scheduler->template SetData<concat(NetName, "RefTraj")>(ref_position);
            this->Scheduler->template SetData<concat(NetName, "RefVel")>(ref_velocity);
        }

        void PostProcess() override
        {
            if (!AllFinite(this->Action__)) throw std::runtime_error("Non-finite SuperMimic action");
            MotorVec target, last;
            for (size_t i = 0; i < JointNumber; ++i)
            {
                const size_t index = this->JointMap__[i];
                this->LastAction__[i] = this->Action__[i];
                target[index] = this->DefaultPosition__[i] + this->ActionScale__[i] * this->Action__[i];
                last[index] = this->Action__[i];
            }
            if (!AllFinite(target)) throw std::runtime_error("Non-finite SuperMimic target");
            this->Scheduler->template SetData<concat(NetName, "NetLastAction")>(last);
            this->Scheduler->template SetData<concat(NetName, "Action")>(target);
            const float milliseconds = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - this->BeginTime__).count();
            this->Scheduler->template SetData<concat(NetName, "InferenceTime")>(milliseconds);
        }

    private:
        static void ValidateTensor(const Ort::TypeInfo& type, const std::vector<int64_t>& shape)
        {
            const auto info = type.GetTensorTypeAndShapeInfo();
            if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || info.GetShape() != shape)
                throw std::runtime_error("SuperMimic ONNX shape/type mismatch; re-export the bundle");
        }

        template<typename Container>
        static bool AllFinite(const Container& values)
        {
            return std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); });
        }

        static Rotation FromEuler(float roll, float pitch, float yaw)
        {
            const float sr = std::sin(roll), cr = std::cos(roll), sp = std::sin(pitch), cp = std::cos(pitch), sy = std::sin(yaw), cy = std::cos(yaw);
            return {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr};
        }

        static Rotation Multiply(const Rotation& first, const Rotation& second)
        {
            Rotation result{};
            for (size_t i = 0; i < 3; ++i)
                for (size_t j = 0; j < 3; ++j)
                    for (size_t k = 0; k < 3; ++k)
                        result[3 * i + j] += first[3 * i + k] * second[3 * k + j];
            return result;
        }

        static Rotation TransposeMultiply(const Rotation& first, const Rotation& second)
        {
            Rotation transpose{};
            for (size_t i = 0; i < 3; ++i)
                for (size_t j = 0; j < 3; ++j)
                    transpose[3 * i + j] = first[3 * j + i];
            return Multiply(transpose, second);
        }

        size_t HistoryLength__ = 0, GoalFrames__ = 0, StartTimestamp__ = 0;
        double CycleTime__ = 0;
        bool Initialized__ = false, Started__ = false;
        using MotionClip = typename SuperMimicMotion<JointNumber>::Clip;
        SuperMimicMotion<JointNumber> Reference__;
        static constexpr uint64_t NoRequest__ = std::numeric_limits<uint64_t>::max();
        std::atomic<uint64_t> Request__{0};
        std::atomic<size_t> SelectedMotion__{0};
        size_t ActiveMotion__ = 0;
        std::vector<MotionClip> Motions__;
        std::array<size_t, JointNumber> JointMap__{};
        std::array<float, JointNumber> DefaultPosition__{}, ActionScale__{}, Action__{}, LastAction__{};
        Rotation HeadingOffset__{};
        std::vector<float> History__, Goal__;
        std::chrono::steady_clock::time_point BeginTime__;
    };
}
