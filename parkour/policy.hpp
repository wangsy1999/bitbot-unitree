/**
 * @file policy.hpp
 * @brief Parkour 深度编码器、动作网络及观测历史。
 * @details 改编自 g1_deploy/include/g1_parkour_policy.h，许可见 LICENSE。
 */
#pragma once

#include "depth_config.hpp"
#include <onnxruntime_cxx_api.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace g1_parkour
{
    using Joints = std::array<float, ModelLayout::JointCount>;
    using Image = std::array<float, ModelLayout::DepthPixels>;
    using RawDepthImage = std::array<float, MujocoDepthLayout::Pixels>;

    /** @brief 同一物理时刻的完整观测；关节向量使用策略顺序。 */
    struct Observation
    {
        uint64_t Sequence = 0;
        uint64_t Generation = 0; // 本地复位代次，不参与网络输入。
        double Time = 0;
        Joints JointPosition{};
        Joints JointVelocity{};
        std::array<float, 3> AngularVelocity{};
        std::array<float, 3> ProjectedGravity{};
        float Height = 0;
        std::array<float, ModelLayout::DepthHistorySize> DepthHistory{}; // 观测层处理后的深度历史，从旧到新。
    };

    class Policy
    {
    public:
        std::array<int, ModelLayout::JointCount> JointIds{}; // 策略顺序 -> DDS 设备顺序。
        Joints DefaultJointPosition{};
        Joints PositionKp{};
        Joints VelocityKd{};
        Joints ActionScale{};
        Joints JointSign{};
        Joints EffortLimit{};
        std::vector<std::string> JointNames;
        double StepTime = ModelLayout::PolicyStepTime;

        Policy(const std::string& model_directory, const std::string& config_path)
        {
            std::ifstream input(config_path);
            if (!input)
            {
                throw std::runtime_error("Cannot open parkour config: " + config_path);
            }

            const auto cfg = nlohmann::json::parse(input);
            this->DefaultJointPosition = ReadArray<ModelLayout::JointCount>(cfg.at("default_joint_pos"));
            this->PositionKp = ReadArray<ModelLayout::JointCount>(cfg.at("kp"));
            this->VelocityKd = ReadArray<ModelLayout::JointCount>(cfg.at("kd"));
            this->ActionScale = ReadArray<ModelLayout::JointCount>(cfg.at("action_scale"));
            this->JointSign = ReadArray<ModelLayout::JointCount>(cfg.at("joint_sign"));
            this->EffortLimit = ReadArray<ModelLayout::JointCount>(cfg.at("effort_limit"));
            this->DepthDelay__ = cfg.at("depth_delay_frames").get<int>();
            this->StepTime = cfg.at("step_dt").get<double>();
            this->DepthHistoryStride__ = cfg.at("depth_history_skip_frames").get<int>();
            this->DepthHistorySize__ = cfg.at("depth_history_buffer_size").get<size_t>();
            MakeDepthHistoryIndices(this->DepthHistorySize__, this->DepthHistoryStride__, this->DepthDelay__);
            this->AngularVelocityScale__ = cfg.at("observation_scales").at("ang_vel").get<float>();
            this->JointVelocityScale__ = cfg.at("observation_scales").at("dof_vel").get<float>();

            if (!std::isfinite(this->StepTime) ||
                std::abs(this->StepTime - ModelLayout::PolicyStepTime) > 1e-8 ||
                !std::isfinite(this->AngularVelocityScale__) || !std::isfinite(this->JointVelocityScale__))
            {
                throw std::runtime_error("Policy timing/scales do not match the supported model interface");
            }

            this->JointNames = cfg.at("joint_names").get<std::vector<std::string>>();
            ConfigureJointMap();

            const auto& networks = cfg.at("networks");
            Ort::SessionOptions options;
            options.SetIntraOpNumThreads(networks.at("intra_op_threads").get<int>());
            options.SetInterOpNumThreads(networks.at("inter_op_threads").get<int>());
            const auto encoder_path = std::filesystem::path(model_directory) /
                networks.at("depth_encoder").get<std::string>();
            const auto actor_path = std::filesystem::path(model_directory) /
                networks.at("actor").get<std::string>();
            this->Encoder__ = std::make_unique<Ort::Session>(this->Environment__, encoder_path.c_str(), options);
            this->Actor__ = std::make_unique<Ort::Session>(this->Environment__, actor_path.c_str(), options);
            ValidateNetwork(*this->Encoder__, {1, ModelLayout::HistoryFrames, ModelLayout::DepthHeight,
                ModelLayout::DepthWidth}, {1, ModelLayout::DepthLatentSize},
                this->EncoderInputName__, this->EncoderOutputName__);
            ValidateNetwork(*this->Actor__, {1, ModelLayout::ActorInputSize}, {1, ModelLayout::JointCount},
                this->ActorInputName__, this->ActorOutputName__);
        }

        /** @brief 由推理线程清空观测历史及上一动作。 */
        void Reset()
        {
            this->ObservationHistory__.clear();
            this->LastAction__.fill(0);
        }

        const Joints& GetLastAction() const
        {
            return this->LastAction__;
        }

        int GetDepthDelay() const { return this->DepthDelay__; }
        int GetDepthHistoryStride() const { return this->DepthHistoryStride__; }
        size_t GetDepthHistorySize() const { return this->DepthHistorySize__; }

        /** @brief 仿真深度裁剪、3×3 Gaussian 滤波（sigma=1）及归一化。 */
        static Image Preprocess(const RawDepthImage& depth)
        {
            for (float value : depth)
            {
                if (std::isnan(value) || value < 0)
                {
                    throw std::runtime_error("Invalid raw depth");
                }
            }

            Image image{};
            const double side_weight = std::exp(-0.5);
            const double weight_sum = 1 + 2 * side_weight;
            const std::array<double, 3> weights{
                side_weight / weight_sum, 1 / weight_sum, side_weight / weight_sum};
            const auto reflect = [](int index, int size)
            {
                return index < 0 ? -index : (index >= size ? 2 * size - 2 - index : index);
            };

            for (int y = 0; y < static_cast<int>(ModelLayout::DepthHeight); ++y)
            {
                for (int x = 0; x < static_cast<int>(ModelLayout::DepthWidth); ++x)
                {
                    double value = 0;
                    for (int j = -1; j <= 1; ++j)
                    {
                        for (int i = -1; i <= 1; ++i)
                        {
                            const auto source_row = reflect(y + j, ModelLayout::DepthHeight) +
                                MujocoDepthLayout::Height - ModelLayout::DepthHeight;
                            const auto source_column = reflect(x + i, ModelLayout::DepthWidth) +
                                (MujocoDepthLayout::Width - ModelLayout::DepthWidth) / 2;
                            const float distance = depth[source_row * MujocoDepthLayout::Width + source_column];
                            // 无命中像素在滤波和截断后仍按远处处理。
                            value += weights[j + 1] * weights[i + 1] *
                                (std::isinf(distance) ? 1e30 : double(distance));
                        }
                    }
                    image[y * ModelLayout::DepthWidth + x] = std::clamp(value, 0.0,
                        MujocoDepthLayout::MaximumDepth) / MujocoDepthLayout::MaximumDepth;
                }
            }
            return image;
        }

        /** @brief 更新本体历史，按观测项拼接输入，并附加观测层提供的深度历史。 */
        std::pair<std::vector<float>, std::vector<float>> Observe(
            const Observation& observation, const std::array<float, 3>& command)
        {
            std::array<float, ModelLayout::ObservationSize> frame{};
            for (int i = 0; i < 3; ++i)
            {
                frame[i] = observation.AngularVelocity[i] * this->AngularVelocityScale__;
                frame[3 + i] = observation.ProjectedGravity[i];
                frame[6 + i] = command[i];
            }
            for (int i = 0; i < static_cast<int>(ModelLayout::JointCount); ++i)
            {
                frame[9 + i] = observation.JointPosition[i] - this->DefaultJointPosition[i];
                frame[38 + i] = observation.JointVelocity[i] * this->JointVelocityScale__;
                frame[67 + i] = this->LastAction__[i];
            }
            for (float value : frame)
            {
                if (!std::isfinite(value))
                {
                    throw std::runtime_error("Non-finite proprioception");
                }
            }

            if (this->ObservationHistory__.empty())
            {
                this->ObservationHistory__.assign(ModelLayout::HistoryFrames, frame);
            }
            else
            {
                this->ObservationHistory__.pop_front();
                this->ObservationHistory__.push_back(frame);
            }

            std::vector<float> proprioception;
            std::vector<float> depth;
            proprioception.reserve(ModelLayout::ProprioceptionSize);
            depth.reserve(ModelLayout::DepthHistorySize);
            constexpr int SpatialSize = ModelLayout::SpatialDimensions;
            constexpr int PositionStart = 3 * SpatialSize;
            constexpr int VelocityStart = PositionStart + ModelLayout::JointCount;
            constexpr int ActionStart = VelocityStart + ModelLayout::JointCount;
            const std::array<int, 6> starts{0, SpatialSize, 2 * SpatialSize, PositionStart, VelocityStart, ActionStart};
            const std::array<size_t, 6> sizes{SpatialSize, SpatialSize, SpatialSize,
                ModelLayout::JointCount, ModelLayout::JointCount, ModelLayout::JointCount};
            for (int term = 0; term < 6; ++term)
            {
                for (const auto& entry : this->ObservationHistory__)
                {
                    proprioception.insert(proprioception.end(),
                        entry.begin() + starts[term], entry.begin() + starts[term] + sizes[term]);
                }
            }
            depth.assign(observation.DepthHistory.begin(), observation.DepthHistory.end());
            return {proprioception, depth};
        }

        /** @brief 执行双网络推理，返回 DDS 设备顺序的关节目标角。 */
        Joints Step(const Observation& observation, const std::array<float, 3>& command)
        {
            auto inputs = Observe(observation, command);
            const auto latent = RunNetwork(*this->Encoder__, inputs.second,
                {1, ModelLayout::HistoryFrames, ModelLayout::DepthHeight, ModelLayout::DepthWidth},
                this->EncoderInputName__, this->EncoderOutputName__);
            inputs.first.insert(inputs.first.end(), latent.begin(), latent.end());
            const auto actions = RunNetwork(*this->Actor__, inputs.first, {1, ModelLayout::ActorInputSize},
                this->ActorInputName__, this->ActorOutputName__);

            Joints target{};
            for (int i = 0; i < static_cast<int>(ModelLayout::JointCount); ++i)
            {
                this->LastAction__[i] = actions[i];
                target[this->JointIds[i]] = this->JointSign[i] *
                    (this->DefaultJointPosition[i] + this->ActionScale[i] * actions[i]);
                if (!std::isfinite(target[this->JointIds[i]]))
                {
                    throw std::runtime_error("Non-finite target");
                }
            }
            return target;
        }

        Joints GetDefaultDevicePosition() const
        {
            Joints position{};
            for (int i = 0; i < static_cast<int>(ModelLayout::JointCount); ++i)
            {
                position[this->JointIds[i]] = this->JointSign[i] * this->DefaultJointPosition[i];
            }
            return position;
        }

    private:
        template<std::size_t N>
        static std::array<float, N> ReadArray(const nlohmann::json& node)
        {
            if (!node.is_array() || node.size() != N)
            {
                throw std::runtime_error("Invalid parkour configuration array");
            }
            std::array<float, N> values{};
            for (std::size_t i = 0; i < N; ++i)
            {
                values[i] = node[i].get<float>();
                if (!std::isfinite(values[i]))
                {
                    throw std::runtime_error("Non-finite configuration");
                }
            }
            return values;
        }

        void ConfigureJointMap()
        {
            const std::vector<std::string> device_names{
                "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
                "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
                "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
                "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint", "left_shoulder_pitch_joint",
                "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
                "left_wrist_pitch_joint", "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
                "right_wrist_yaw_joint"};
            if (this->JointNames.size() != ModelLayout::JointCount)
            {
                throw std::runtime_error("Expected 29 policy joints");
            }

            std::array<bool, ModelLayout::JointCount> seen{};
            for (int i = 0; i < static_cast<int>(ModelLayout::JointCount); ++i)
            {
                const auto found = std::find(device_names.begin(), device_names.end(), this->JointNames[i]);
                if (found == device_names.end() || seen[found - device_names.begin()])
                {
                    throw std::runtime_error("Unknown or duplicate joint");
                }
                this->JointIds[i] = found - device_names.begin();
                seen[this->JointIds[i]] = true;
                if (this->JointSign[i] != 1 && this->JointSign[i] != -1)
                {
                    throw std::runtime_error("joint_sign must be +1 or -1");
                }
                if (this->PositionKp[i] <= 0 || this->VelocityKd[i] < 0 || this->EffortLimit[i] <= 0)
                {
                    throw std::runtime_error("Invalid PD gains");
                }
            }
        }

        static void ValidateNetwork(Ort::Session& session, const std::vector<int64_t>& input_shape,
            const std::vector<int64_t>& output_shape, std::string& input_name, std::string& output_name)
        {
            if (session.GetInputCount() != 1 || session.GetOutputCount() != 1)
            {
                throw std::runtime_error("Expected single-input single-output network");
            }
            const auto input_type = session.GetInputTypeInfo(0);
            const auto output_type = session.GetOutputTypeInfo(0);
            const auto input_info = input_type.GetTensorTypeAndShapeInfo();
            const auto output_info = output_type.GetTensorTypeAndShapeInfo();
            if (input_info.GetShape() != input_shape || output_info.GetShape() != output_shape ||
                input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
                output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
            {
                throw std::runtime_error("Incompatible parkour ONNX tensor shape/type");
            }
            Ort::AllocatorWithDefaultOptions allocator;
            input_name = session.GetInputNameAllocated(0, allocator).get();
            output_name = session.GetOutputNameAllocated(0, allocator).get();
        }

        static std::vector<float> RunNetwork(Ort::Session& session, std::vector<float>& data,
            const std::vector<int64_t>& shape, const std::string& input_name, const std::string& output_name)
        {
            auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            auto tensor = Ort::Value::CreateTensor<float>(memory, data.data(), data.size(), shape.data(), shape.size());
            const char* input = input_name.c_str();
            const char* output = output_name.c_str();
            auto result = session.Run(Ort::RunOptions{nullptr}, &input, &tensor, 1, &output, 1);
            const auto count = result[0].GetTensorTypeAndShapeInfo().GetElementCount();
            const auto* values = result[0].GetTensorData<float>();
            for (std::size_t i = 0; i < count; ++i)
            {
                if (!std::isfinite(values[i]))
                {
                    throw std::runtime_error("Non-finite network output");
                }
            }
            return {values, values + count};
        }

        Ort::Env Environment__{ORT_LOGGING_LEVEL_WARNING, "g1_parkour"};
        std::unique_ptr<Ort::Session> Encoder__;
        std::unique_ptr<Ort::Session> Actor__;
        std::string EncoderInputName__;
        std::string EncoderOutputName__;
        std::string ActorInputName__;
        std::string ActorOutputName__;
        std::deque<std::array<float, ModelLayout::ObservationSize>> ObservationHistory__;
        Joints LastAction__{};
        int DepthDelay__ = 0;
        int DepthHistoryStride__ = 0;
        size_t DepthHistorySize__ = 0;
        float AngularVelocityScale__ = 0;
        float JointVelocityScale__ = 0;
    };
}
