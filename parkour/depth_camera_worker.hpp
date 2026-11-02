/**
 * @file depth_camera_worker.hpp
 * @brief 深度观测发布回调：读取设备缓冲、预处理、维护图像历史。
 */
#pragma once

#include "mujoco_depth_camera.h"
#include "policy.hpp"
#include "bitbot_mujoco/bus/mujoco_bus.h"
#include <deque>
#include <limits>

namespace g1_parkour
{
    /**
     * @brief 供 MainTask 的回调 Worker 调用，职责对应参考项目 PublishDepthImageFlat。
     * @details 设备 Input() 负责采集；本类只读取新缓冲，发布含深度历史的完整观测。
     */
    template<typename SchedulerType, typename InferenceWorkerType>
    class DepthObservationPublisher
    {
    public:
        DepthObservationPublisher(const bitbot::MujocoBus& bus, const bitbot::MujocoDepthCamera& camera,
            const InferenceWorkerType& inference_worker)
            : Bus__(bus), Camera__(camera), InferenceWorker__(inference_worker)
        {
        }

        void Publish(typename SchedulerType::Ptr scheduler)
        {
            const auto* model = this->Bus__.GetMujocoModel();
            auto* data = this->Bus__.GetMujocoData();
            const auto& policy = this->InferenceWorker__.GetPolicy();
            const auto generation = this->InferenceWorker__.GetGeneration();
            if (this->Generation__ != generation)
            {
                this->DepthHistory__.clear();
                this->Generation__ = generation;
            }

            const auto sequence = this->Camera__.GetFrameSequence();
            const float* buffer = this->Camera__.GetDepth();
            if (!buffer || !sequence || sequence == this->LastFrameSequence__)
            {
                return;
            }
            this->LastFrameSequence__ = sequence;
            if (this->TorsoId__ < 0)
            {
                ConfigureModel(model, policy);
            }

            // 与参考设备接口一致：GetDepth() 为首行在底部的 OpenGL 非线性深度。
            const double near_plane = model->vis.map.znear * model->stat.extent;
            const double far_plane = model->vis.map.zfar * model->stat.extent;
            RawDepthImage depth;
            for (int y = 0; y < 36; ++y)
            {
                for (int x = 0; x < 64; ++x)
                {
                    const float value = buffer[(35 - y) * 64 + x];
                    depth[y * 64 + x] = value >= 1.f ? std::numeric_limits<float>::infinity()
                        : static_cast<float>(near_plane / (1.0 - value * (1.0 - near_plane / far_plane)));
                }
            }

            const auto image = Policy::Preprocess(depth);
            if (this->DepthHistory__.empty())
            {
                this->DepthHistory__.assign(37, image);
            }
            else
            {
                this->DepthHistory__.pop_front();
                this->DepthHistory__.push_back(image);
            }

            Observation observation;
            observation.Sequence = sequence;
            observation.Generation = generation;
            observation.Time = this->Camera__.GetFrameTime();
            size_t cursor = 0;
            const int stride = policy.GetDepthHistoryStride();
            for (int offset = 7 * stride; offset >= 0; offset -= stride)
            {
                const auto& entry = this->DepthHistory__[
                    this->DepthHistory__.size() - 1 - policy.GetDepthDelay() - offset];
                std::copy(entry.begin(), entry.end(), observation.DepthHistory.begin() + cursor);
                cursor += entry.size();
            }
            for (size_t i = 0; i < 29; ++i)
            {
                observation.JointPosition[i] = policy.JointSign[i] * data->qpos[this->JointPositionAddress__[i]];
                observation.JointVelocity[i] = policy.JointSign[i] * data->qvel[this->JointVelocityAddress__[i]];
            }

            mjtNum velocity[6];
            mj_objectVelocity(model, data, mjOBJ_BODY, this->TorsoId__, velocity, 1);
            for (int i = 0; i < 3; ++i)
            {
                observation.AngularVelocity[i] = velocity[i];
                observation.ProjectedGravity[i] = -data->xmat[9 * this->TorsoId__ + 6 + i];
            }
            observation.Height = data->xpos[3 * this->TorsoId__ + 2];
            scheduler->template SetData<"ParkourObservation">(observation);
        }

    private:
        void ConfigureModel(const mjModel* model, const Policy& policy)
        {
            this->TorsoId__ = mj_name2id(model, mjOBJ_BODY, "torso_link");
            if (this->TorsoId__ < 0)
            {
                throw std::runtime_error("Missing torso_link");
            }
            if (std::abs(model->opt.timestep - 0.002) > 1e-9)
            {
                throw std::runtime_error("Parkour expects 2ms MuJoCo steps");
            }
            for (size_t i = 0; i < 29; ++i)
            {
                const int joint = mj_name2id(model, mjOBJ_JOINT, policy.JointNames[i].c_str());
                if (joint < 0)
                {
                    throw std::runtime_error("Missing Parkour joint");
                }
                this->JointPositionAddress__[i] = model->jnt_qposadr[joint];
                this->JointVelocityAddress__[i] = model->jnt_dofadr[joint];
            }
        }

        const bitbot::MujocoBus& Bus__;
        const bitbot::MujocoDepthCamera& Camera__;
        const InferenceWorkerType& InferenceWorker__;
        std::array<int, 29> JointPositionAddress__{};
        std::array<int, 29> JointVelocityAddress__{};
        std::deque<Image> DepthHistory__;
        int TorsoId__ = -1;
        uint64_t Generation__ = 0;
        uint64_t LastFrameSequence__ = 0;
    };
}
