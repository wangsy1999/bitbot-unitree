/**
 * @file inference_worker.hpp
 * @brief 复用 WalkNet 接口的 DreamWaQ / Parkour 推理 Worker。
 */
#pragma once

#include "policy.hpp"
#include "Workers/NN/DreamWaQInferenceWorker.hpp"
#include <atomic>
#include <chrono>
#include <filesystem>
#include <mutex>
#include <type_traits>

namespace g1_parkour
{
    /**
     * @brief 在原 InferWalkTask 中推理，输出交由原 ActionManager 处理。
     * @details 推理线程独占策略历史；事件线程仅提交复位请求，不访问历史缓存。
     */
    template<typename SchedulerType, z::CTString NetName, typename InferencePrecision, size_t JointNumber>
    class WalkInferenceWorker : public z::AbstractWorker<SchedulerType>
    {
        static_assert(JointNumber == 29 && std::is_same_v<InferencePrecision, float>);
        using MotorVec = z::math::Vector<InferencePrecision, JointNumber>;
        using Vec3 = z::math::Vector<InferencePrecision, 3>;
        using DreamWaQWorkerType = z::DreamWaQInferenceWorker<
            SchedulerType, NetName, InferencePrecision, JointNumber>;

    public:
        WalkInferenceWorker(typename SchedulerType::Ptr scheduler, const nlohmann::json& net_cfg,
            const nlohmann::json& motor_cfg, const std::array<size_t, JointNumber>& joint_map)
            : z::AbstractWorker<SchedulerType>(scheduler), JointIdMap__(joint_map)
        {
            const auto policy_name = net_cfg.value("Policy", std::string("DreamWaQ"));
            if (policy_name == "DreamWaQ")
            {
                this->DreamWaQWorker__ = std::make_unique<DreamWaQWorkerType>(scheduler, net_cfg, motor_cfg);
            }
            else if (policy_name == "Parkour")
            {
                const std::filesystem::path root(PROJECT_ROOT_DIR);
                this->Policy__ = std::make_unique<Policy>(
                    (root / net_cfg.at("ParkourModels").get<std::string>()).string(),
                    (root / net_cfg.at("ParkourConfig").get<std::string>()).string());
                for (size_t i = 0; i < JointNumber; ++i)
                {
                    const auto found = std::find(this->Policy__->JointIds.begin(),
                        this->Policy__->JointIds.end(), this->JointIdMap__[i]);
                    if (found == this->Policy__->JointIds.end())
                    {
                        throw std::runtime_error("Parkour joint map mismatch");
                    }
                    this->PolicyJointOrder__[i] = found - this->Policy__->JointIds.begin();
                }
                PublishDefaultPosition();
            }
            else
            {
                throw std::runtime_error("Unknown WalkNet1.Policy: " + policy_name);
            }
        }

        bool IsParkour() const
        {
            return bool(this->Policy__);
        }

        /** @brief 相机 Worker 仅读取策略配置，不访问推理历史或网络。 */
        const Policy& GetPolicy() const
        {
            return *this->Policy__;
        }

        uint64_t GetGeneration() const
        {
            return this->Generation__.load();
        }

        /** @brief 将复位角和 PD 参数转换为原 scheduler 的关节顺序。 */
        void ConfigureMotor(nlohmann::json& workers) const
        {
            if (!this->Policy__)
            {
                return;
            }
            const auto defaults = this->Policy__->GetDefaultDevicePosition();
            for (size_t i = 0; i < JointNumber; ++i)
            {
                workers["MotorControl"]["DefaultPosition"][i] = defaults[this->JointIdMap__[i]];
            }
            for (auto& properties : workers["ActionManager"]["MotorProperties"])
            {
                if (properties["PolicyNetName"] != NetName.value)
                {
                    continue;
                }
                for (size_t i = 0; i < JointNumber; ++i)
                {
                    properties["Stiffness"][i] = this->Policy__->PositionKp[this->PolicyJointOrder__[i]];
                    properties["Damping"][i] = this->Policy__->VelocityKd[this->PolicyJointOrder__[i]];
                }
            }
        }

        /** @brief 保留原行走 Worker 的 reset 接口，使已有事件回调无需修改。 */
        void reset()
        {
            if (this->DreamWaQWorker__)
            {
                this->DreamWaQWorker__->reset();
            }
            else
            {
                std::lock_guard lock(this->OutputMutex__);
                ++this->Generation__;
                // 首个新结果到达前，ActionManager 使用默认站姿，不复用旧策略目标。
                PublishDefaultPosition();
                this->Scheduler->template SetData<z::concat(NetName, "NetLastAction")>(MotorVec::zeros());
            }
        }

        void TaskCreate() override
        {
            if (this->DreamWaQWorker__)
            {
                this->DreamWaQWorker__->TaskCreate();
            }
        }

        void TaskRun() override
        {
            if (this->DreamWaQWorker__)
            {
                this->DreamWaQWorker__->TaskRun();
                return;
            }

            const auto generation = this->Generation__.load();
            if (generation != this->ProcessedGeneration__)
            {
                this->Policy__->Reset();
                this->ProcessedGeneration__ = generation;
                this->LastSequence__ = 0;
            }

            Observation observation;
            this->Scheduler->template GetData<"ParkourObservation">(observation);
            // 推理任务先于 MainTask 被唤醒时，等待属于本次复位的新快照。
            if (observation.Generation != generation || !observation.Sequence ||
                observation.Sequence == this->LastSequence__)
            {
                return;
            }
            this->LastSequence__ = observation.Sequence;

            Vec3 command;
            this->Scheduler->template GetData<z::concat(NetName, "NetUserCommand3")>(command);
            const auto begin = std::chrono::steady_clock::now();
            const auto device_target = this->Policy__->Step(observation, {command[0], command[1], command[2]});

            std::lock_guard lock(this->OutputMutex__);
            if (this->Generation__.load() != generation)
            {
                return; // 复位请求使在途结果失效。
            }

            MotorVec target;
            MotorVec action;
            for (size_t i = 0; i < JointNumber; ++i)
            {
                target[i] = device_target[this->JointIdMap__[i]];
                action[i] = this->Policy__->GetLastAction()[this->PolicyJointOrder__[i]];
            }
            this->Scheduler->template SetData<z::concat(NetName, "Action")>(target);
            this->Scheduler->template SetData<z::concat(NetName, "NetLastAction")>(action);
            this->Scheduler->template SetData<z::concat(NetName, "NetProjectedGravity")>(
                Vec3{observation.ProjectedGravity[0], observation.ProjectedGravity[1], observation.ProjectedGravity[2]});
            const InferencePrecision elapsed = std::chrono::duration<InferencePrecision, std::milli>(
                std::chrono::steady_clock::now() - begin).count();
            this->Scheduler->template SetData<z::concat(NetName, "InferenceTime")>(elapsed);
        }

    private:
        void PublishDefaultPosition()
        {
            MotorVec position;
            const auto defaults = this->Policy__->GetDefaultDevicePosition();
            for (size_t i = 0; i < JointNumber; ++i)
            {
                position[i] = defaults[this->JointIdMap__[i]];
            }
            this->Scheduler->template SetData<z::concat(NetName, "Action")>(position);
        }

        std::unique_ptr<DreamWaQWorkerType> DreamWaQWorker__;
        std::unique_ptr<Policy> Policy__;
        std::array<size_t, JointNumber> JointIdMap__;
        std::array<size_t, JointNumber> PolicyJointOrder__{};
        uint64_t LastSequence__ = 0;
        uint64_t ProcessedGeneration__ = 0;
        std::atomic<uint64_t> Generation__{1};
        std::mutex OutputMutex__;
    };
}
