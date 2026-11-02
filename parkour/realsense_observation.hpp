/**
 * @file realsense_observation.hpp
 * @brief 将真机 RealSense 深度历史和关节、torso IMU 组装为 Parkour 观测。
 */
#pragma once

#include "policy.hpp"
#include "realsense_depth_types.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace g1_parkour
{
    template<typename SchedulerType, typename InferenceWorkerType, typename JointType, typename ImuType>
    class RealsenseObservationPublisher
    {
    public:
        RealsenseObservationPublisher(const InferenceWorkerType& inference_worker,
            const std::array<JointType, ModelLayout::JointCount>& joints, ImuType torso_imu, const std::array<size_t, ModelLayout::JointCount>& joint_map)
            : InferenceWorker__(inference_worker), Joints__(joints), TorsoImu__(torso_imu)
        {
            const auto& policy = inference_worker.GetPolicy();
            for (size_t i = 0; i < ModelLayout::JointCount; ++i)
            {
                const auto found = std::find(joint_map.begin(), joint_map.end(), policy.JointIds[i]);
                if (found == joint_map.end())
                {
                    throw std::runtime_error("RealSense observation joint map mismatch");
                }
                this->JointOrder__[i] = found - joint_map.begin();
            }
        }

        void Publish(typename SchedulerType::Ptr scheduler)
        {
            RealsenseDepthFrame frame;
            scheduler->template GetData<"RealsenseDepthFrame">(frame);
            if (!frame.Sequence || frame.Sequence == this->LastSequence__ ||
                frame.Generation != this->InferenceWorker__.GetGeneration())
            {
                return;
            }
            this->LastSequence__ = frame.Sequence;
            const auto& policy = this->InferenceWorker__.GetPolicy();
            Observation observation;
            observation.Sequence = frame.Sequence;
            observation.Generation = frame.Generation;
            observation.Time = frame.CapturedAt;
            observation.DepthHistory = frame.Values;
            for (size_t i = 0; i < ModelLayout::JointCount; ++i)
            {
                observation.JointPosition[i] = policy.JointSign[i] *
                    this->Joints__[this->JointOrder__[i]]->GetActualPosition();
                observation.JointVelocity[i] = policy.JointSign[i] *
                    this->Joints__[this->JointOrder__[i]]->GetActualVelocity();
            }
            const float roll = this->TorsoImu__->GetRoll();
            const float pitch = this->TorsoImu__->GetPitch();
            observation.ProjectedGravity = {std::sin(pitch), -std::sin(roll) * std::cos(pitch),
                -std::cos(roll) * std::cos(pitch)};
            observation.AngularVelocity = {this->TorsoImu__->GetGyroX(),
                this->TorsoImu__->GetGyroY(), this->TorsoImu__->GetGyroZ()};
            // 真机没有可直接读取的绝对 torso 高度，策略也不使用该字段。
            scheduler->template SetData<"ParkourObservation">(observation);
        }

    private:
        const InferenceWorkerType& InferenceWorker__;
        std::array<JointType, ModelLayout::JointCount> Joints__;
        ImuType TorsoImu__;
        std::array<size_t, ModelLayout::JointCount> JointOrder__{};
        uint64_t LastSequence__ = 0;
    };
}
