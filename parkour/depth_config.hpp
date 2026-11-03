/**
 * @file depth_config.hpp
 * @brief Parkour 模型维度约束及可配置深度处理参数。
 */
#pragma once

#include <nlohmann/json.hpp>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace g1_parkour
{
    /** @brief 当前导出模型的固定接口；变更时同时校验实际 ONNX 形状。 */
    struct ModelLayout
    {
        static constexpr size_t JointCount = 29;
        static constexpr size_t SpatialDimensions = 3;
        static constexpr size_t HistoryFrames = 8;
        static constexpr size_t DepthWidth = 32;
        static constexpr size_t DepthHeight = 18;
        static constexpr size_t DepthPixels = DepthWidth * DepthHeight;
        static constexpr size_t DepthHistorySize = HistoryFrames * DepthPixels;
        static constexpr size_t DepthLatentSize = 128;
        static constexpr size_t ObservationSize = 3 * SpatialDimensions + 3 * JointCount;
        static constexpr size_t ProprioceptionSize = HistoryFrames * ObservationSize;
        static constexpr size_t ActorInputSize = ProprioceptionSize + DepthLatentSize;
        static constexpr double PolicyStepTime = 0.02;
    };

    /** @brief 当前 MuJoCo 相机缓冲接口；与 XML 设备尺寸校验保持一致。 */
    struct MujocoDepthLayout
    {
        static constexpr int Width = 64;
        static constexpr int Height = 36;
        static constexpr size_t Pixels = Width * Height;
        static constexpr double MaximumDepth = 2.5;
    };

    /** @brief 从旧到新的历史索引；index_base 控制零基或一基输出。 */
    inline std::vector<size_t> MakeDepthHistoryIndices(size_t buffer_size, int stride, int delay,
        size_t frame_count = ModelLayout::HistoryFrames, size_t index_base = 0)
    {
        if (frame_count == 0 || stride <= 0 || delay < 0 || buffer_size == 0 ||
            (frame_count - 1) * static_cast<size_t>(stride) + static_cast<size_t>(delay) >= buffer_size)
        {
            throw std::invalid_argument("Depth buffer cannot hold the configured history");
        }
        std::vector<size_t> indices;
        indices.reserve(frame_count);
        for (size_t frame = 0; frame < frame_count; ++frame)
        {
            const size_t offset = (frame_count - 1 - frame) * stride + delay;
            indices.push_back(buffer_size - 1 - offset + index_base);
        }
        return indices;
    }

    /** @brief RealSense 的运行参数；尺寸来自 JSON，输出受当前模型接口约束。 */
    struct DepthProcessingConfig
    {
        int ResizeWidth;
        int ResizeHeight;
        int CropWidth;
        int CropHeight;
        int CropX;
        int CropY;
        bool NearestResize;
        bool EnableBlur;
        int BlurKernel;
        double BlurSigma;
        float DepthMin;
        float DepthMax;
        float OutputMin;
        float OutputMax;
        unsigned FrameTimeoutMs;

        explicit DepthProcessingConfig(const nlohmann::json& cfg)
        {
            const auto& resize = cfg.at("ResizeConfig");
            const auto& crop = cfg.at("CropConfig");
            const auto& blur = cfg.at("GaussianBlurConfig");
            const auto& normalize = cfg.at("NormalizeConfig");
            this->ResizeWidth = resize.at("Width").get<int>();
            this->ResizeHeight = resize.at("Height").get<int>();
            this->CropWidth = crop.at("CropWidth").get<int>();
            this->CropHeight = crop.at("CropHeight").get<int>();
            this->CropX = crop.value("X", (this->ResizeWidth - this->CropWidth) / 2);
            this->CropY = crop.value("Y", this->ResizeHeight - this->CropHeight);
            const auto interpolation = resize.at("Interpolation").get<std::string>();
            if (interpolation != "nearest" && interpolation != "linear")
            {
                throw std::invalid_argument("ResizeConfig.Interpolation must be nearest or linear");
            }
            this->NearestResize = interpolation == "nearest";
            this->EnableBlur = blur.at("Enable").get<bool>();
            this->BlurKernel = blur.at("KernelSize").get<int>();
            this->BlurSigma = blur.at("Sigma").get<double>();
            this->DepthMin = normalize.at("DepthMin").get<float>();
            this->DepthMax = normalize.at("DepthMax").get<float>();
            this->OutputMin = normalize.at("NormMin").get<float>();
            this->OutputMax = normalize.at("NormMax").get<float>();
            const int timeout = cfg.at("CameraConfig").at("FrameTimeoutMs").get<int>();
            if (timeout <= 0)
            {
                throw std::invalid_argument("CameraConfig.FrameTimeoutMs must be positive");
            }
            this->FrameTimeoutMs = static_cast<unsigned>(timeout);
            Validate();
        }

        void Validate() const
        {
            if (this->CropWidth != static_cast<int>(ModelLayout::DepthWidth) ||
                this->CropHeight != static_cast<int>(ModelLayout::DepthHeight))
            {
                throw std::invalid_argument("Depth crop does not match the ONNX depth input shape");
            }
            if (this->ResizeWidth < this->CropWidth || this->ResizeHeight < this->CropHeight ||
                this->CropX < 0 || this->CropY < 0 || this->CropX > this->ResizeWidth - this->CropWidth ||
                this->CropY > this->ResizeHeight - this->CropHeight)
            {
                throw std::invalid_argument("Depth crop lies outside the resized image");
            }
            if (this->EnableBlur && (this->BlurKernel < 1 || this->BlurKernel % 2 == 0 ||
                !std::isfinite(this->BlurSigma) || this->BlurSigma <= 0))
            {
                throw std::invalid_argument("Invalid Gaussian blur configuration");
            }
            if (!std::isfinite(this->DepthMin) || !std::isfinite(this->DepthMax) ||
                this->DepthMin < 0 || this->DepthMin >= this->DepthMax ||
                this->OutputMin != 0 || this->OutputMax != 1)
            {
                throw std::invalid_argument("Depth normalization must produce the model's [0,1] input range");
            }
        }
    };
}
