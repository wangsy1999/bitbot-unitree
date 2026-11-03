/**
 * @file realsense_depth_worker.hpp
 * @brief 从 xptt/CtrlZ 的 RealsenseDepthWorker 移植的真机深度采集 Worker。
 * @details 保留 SDK 后处理链及首帧填满历史，输出适配 Parkour 的 8×18×32。
 */
#pragma once

#include "realsense_depth_types.hpp"
#include "Workers/AbstractWorker.hpp"
#include "Utils/MathTypes.hpp"
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

namespace z
{
    /** @brief 仅注册于真机 DepthTask；等待相机帧不会阻塞 MainTask。 */
    template<typename SchedulerType, typename DepthPrecision>
    class RealsenseDepthWorker : public AbstractWorker<SchedulerType>
    {
        static_assert(std::is_same_v<DepthPrecision, float>);
        using DepthMat = cv::Mat_<DepthPrecision>;
        using DepthVector = math::Vector<DepthPrecision, g1_parkour::ModelLayout::DepthHistorySize>;

    public:
        RealsenseDepthWorker(typename SchedulerType::Ptr scheduler, const nlohmann::json& cfg,
            std::function<uint64_t()> generation_provider)
            : AbstractWorker<SchedulerType>(scheduler), Config__(cfg), ProcessingConfig__(cfg), GenerationProvider__(generation_provider)
        {
            this->Enabled__ = cfg.value("Enable", true);
            const auto& buffer = cfg.at("BufferConfig");
            this->BufferSize__ = buffer.at("BufferSize").template get<size_t>();
            this->SelectIndices__ = buffer.at("SelectIndices").template get<std::vector<size_t>>();
            this->IndexBase__ = buffer.value("IndexBase", size_t{1});
            this->Postprocess__ = cfg.at("PostprocessConfig");
            this->SaveConfig__ = cfg.value("SaveConfig", nlohmann::json::object());
            ValidateConfig();
            ConfigureFilters();
            this->PrintSplitLine();
            std::cout << "RealsenseDepthWorker: " << g1_parkour::ModelLayout::HistoryFrames << " x "
                << this->ProcessingConfig__.CropHeight << " x " << this->ProcessingConfig__.CropWidth
                << ", dedicated DepthTask" << std::endl;
            this->PrintSplitLine();
        }

        ~RealsenseDepthWorker() override
        {
            StopPipeline();
        }

        void TaskCreate() override
        {
            if (!this->Enabled__)
            {
                return;
            }
            try
            {
                const auto& camera = this->Config__.at("CameraConfig");
                rs2::config config;
                const auto serial = camera.value("Serial", std::string());
                if (!serial.empty())
                {
                    config.enable_device(serial);
                }
                config.enable_stream(RS2_STREAM_DEPTH, camera.at("Width").template get<int>(),
                    camera.at("Height").template get<int>(), RS2_FORMAT_Z16, camera.at("FPS").template get<int>());
                this->Pipeline__ = std::make_unique<rs2::pipeline>();
                this->Pipeline__->start(config);
                this->PipelineStarted__ = true;
                this->FrameBuffer__.clear();
                this->Sequence__ = 0;
                if (this->SaveConfig__.value("EnableSave", false))
                {
                    std::filesystem::create_directories(GetSavePath());
                }
                std::cout << "RealSense depth pipeline started" << std::endl;
            }
            catch (const std::exception& error)
            {
                ReportError(error.what());
                StopPipeline();
            }
        }

        void TaskRun() override
        {
            if (!this->PipelineStarted__)
            {
                return;
            }
            try
            {
                rs2::frameset frames;
                if (!this->Pipeline__->try_wait_for_frames(&frames, this->ProcessingConfig__.FrameTimeoutMs))
                {
                    return;
                }
                auto frame = frames.get_depth_frame();
                if (!frame)
                {
                    return;
                }
                const double captured_at = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                const auto generation = this->GenerationProvider__();
                if (generation != this->Generation__)
                {
                    this->FrameBuffer__.clear();
                    this->Generation__ = generation;
                }

                if (this->SaveConfig__.value("EnableSave", false) && this->SaveConfig__.value("SaveRaw", false))
                {
                    SaveImage(ConvertFrame(frame), "Raw");
                }
                if (this->Postprocess__.value("EnablePostprocess", true))
                {
                    frame = ProcessFrame(frame);
                }
                auto processed = ConvertFrame(frame);
                SaveImage(processed, "Postprocessed");

                const auto& parameters = this->ProcessingConfig__;
                DepthMat resized;
                const int interpolation = parameters.NearestResize ? cv::INTER_NEAREST : cv::INTER_LINEAR;
                cv::resize(processed, resized, cv::Size(parameters.ResizeWidth, parameters.ResizeHeight),
                    0, 0, interpolation);
                SaveImage(resized, "Resized");

                DepthMat image = resized(cv::Rect(parameters.CropX, parameters.CropY,
                    parameters.CropWidth, parameters.CropHeight)).clone();
                SaveImage(image, "Cropped");
                if (parameters.EnableBlur)
                {
                    cv::GaussianBlur(image, image, cv::Size(parameters.BlurKernel, parameters.BlurKernel),
                        parameters.BlurSigma);
                }
                SaveImage(image, "Blurred");

                cv::min(image, parameters.DepthMax, image);
                cv::max(image, parameters.DepthMin, image);
                image = (image - parameters.DepthMin) / (parameters.DepthMax - parameters.DepthMin) *
                    (parameters.OutputMax - parameters.OutputMin) + parameters.OutputMin;
                SaveImage(image, "Normalized");

                if (this->FrameBuffer__.empty())
                {
                    this->FrameBuffer__.assign(this->BufferSize__, image);
                }
                else
                {
                    this->FrameBuffer__.pop_front();
                    this->FrameBuffer__.push_back(image);
                }
                g1_parkour::RealsenseDepthFrame output;
                output.Sequence = ++this->Sequence__;
                output.Generation = generation;
                output.CapturedAt = captured_at;
                size_t cursor = 0;
                for (const auto index : this->SelectIndices__)
                {
                    const auto& selected = this->FrameBuffer__[index - this->IndexBase__];
                    for (int row = 0; row < this->ProcessingConfig__.CropHeight; ++row)
                    {
                        for (int column = 0; column < this->ProcessingConfig__.CropWidth; ++column)
                        {
                            const float value = selected(row, column);
                            if (!std::isfinite(value))
                            {
                                throw std::runtime_error("Non-finite RealSense depth");
                            }
                            output.Values[cursor++] = value;
                        }
                    }
                }
                DepthVector flattened;
                for (size_t i = 0; i < output.Values.size(); ++i)
                {
                    flattened[i] = output.Values[i];
                }
                this->Scheduler->template SetData<"DepthMapFlattenedValue">(flattened);
                this->Scheduler->template SetData<"RealsenseDepthFrame">(output);
            }
            catch (const std::exception& error)
            {
                ReportError(error.what());
            }
        }

        void TaskDestroy() override
        {
            StopPipeline();
        }

    private:
        void ValidateConfig() const
        {
            if (this->BufferSize__ == 0 || this->SelectIndices__.size() != g1_parkour::ModelLayout::HistoryFrames)
            {
                throw std::invalid_argument("RealSense history must match the model frame count");
            }
            for (const auto index : this->SelectIndices__)
            {
                if (index < this->IndexBase__ || index - this->IndexBase__ >= this->BufferSize__)
                {
                    throw std::out_of_range("RealSense history index out of range");
                }
            }
            const auto& camera = this->Config__.at("CameraConfig");
            if (camera.at("Width").template get<int>() <= 0 || camera.at("Height").template get<int>() <= 0 ||
                camera.at("FPS").template get<int>() <= 0)
            {
                throw std::invalid_argument("Camera resolution and FPS must be positive");
            }
        }

        void ConfigureFilters()
        {
            const auto& decimation = this->Postprocess__.at("DecimationFilter");
            this->DecimationFilter__.set_option(RS2_OPTION_FILTER_MAGNITUDE, decimation.at("Magnitude").template get<float>());
            const auto& spatial = this->Postprocess__.at("SpatialFilter");
            this->SpatialFilter__.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, spatial.at("SmoothAlpha").template get<float>());
            this->SpatialFilter__.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, spatial.at("SmoothDelta").template get<float>());
            this->SpatialFilter__.set_option(RS2_OPTION_HOLES_FILL, spatial.at("HoleFillLevel").template get<float>());
            const auto& temporal = this->Postprocess__.at("TemporalFilter");
            this->TemporalFilter__.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, temporal.at("SmoothAlpha").template get<float>());
            this->TemporalFilter__.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, temporal.at("SmoothDelta").template get<float>());
            this->TemporalFilter__.set_option(RS2_OPTION_HOLES_FILL, temporal.at("PersistencyIndex").template get<float>());
            const auto& threshold = this->Postprocess__.at("ThresholdFilter");
            this->ThresholdFilter__.set_option(RS2_OPTION_MIN_DISTANCE, threshold.at("MinDepth").template get<float>());
            this->ThresholdFilter__.set_option(RS2_OPTION_MAX_DISTANCE, threshold.at("MaxDepth").template get<float>());
            this->HoleFillingFilter__.set_option(RS2_OPTION_HOLES_FILL,
                this->Postprocess__.at("HoleFillingFilter").at("FillMode").template get<float>());
        }

        rs2::depth_frame ProcessFrame(const rs2::depth_frame& frame)
        {
            rs2::frame result = frame;
            if (this->Postprocess__.at("DecimationFilter").value("Enable", false))
            {
                result = this->DecimationFilter__.process(result);
            }
            result = this->DepthToDisparity__.process(result);
            if (this->Postprocess__.at("SpatialFilter").value("Enable", true))
            {
                result = this->SpatialFilter__.process(result);
            }
            if (this->Postprocess__.at("TemporalFilter").value("Enable", true))
            {
                result = this->TemporalFilter__.process(result);
            }
            result = this->DisparityToDepth__.process(result);
            if (this->Postprocess__.at("ThresholdFilter").value("Enable", true))
            {
                result = this->ThresholdFilter__.process(result);
            }
            if (this->Postprocess__.at("HoleFillingFilter").value("Enable", true))
            {
                result = this->HoleFillingFilter__.process(result);
            }
            return result.as<rs2::depth_frame>();
        }

        static DepthMat ConvertFrame(const rs2::depth_frame& frame)
        {
            DepthMat image(frame.get_height(), frame.get_width());
            for (int row = 0; row < image.rows; ++row)
            {
                for (int column = 0; column < image.cols; ++column)
                {
                    image(row, column) = frame.get_distance(column, row);
                }
            }
            return image;
        }

        std::filesystem::path GetSavePath() const
        {
            return std::filesystem::path(PROJECT_ROOT_DIR) /
                this->SaveConfig__.at("SavePath").template get<std::string>();
        }

        void SaveImage(const DepthMat& image, const std::string& stage) const
        {
            if (!this->SaveConfig__.value("EnableSave", false) ||
                !this->SaveConfig__.value("Save" + stage, false))
            {
                return;
            }
            const auto path = GetSavePath() / ("depth_" + stage + "_" + std::to_string(this->Sequence__));
            if (this->SaveConfig__.value("SaveVisualization", true))
            {
                cv::Mat display;
                const double minimum = stage == "Normalized" ? this->ProcessingConfig__.OutputMin : this->ProcessingConfig__.DepthMin;
                const double maximum = stage == "Normalized" ? this->ProcessingConfig__.OutputMax : this->ProcessingConfig__.DepthMax;
                const double scale = 255.0 / (maximum - minimum);
                image.convertTo(display, CV_8U, scale, -minimum * scale);
                cv::imwrite(path.string() + ".png", display);
            }
            if (this->SaveConfig__.value("SaveBin", false))
            {
                const auto packed = image.clone();
                std::ofstream file(path.string() + ".bin", std::ios::binary);
                file.write(reinterpret_cast<const char*>(packed.data), packed.total() * sizeof(DepthPrecision));
            }
        }

        void ReportError(const std::string& message)
        {
            if (message != this->LastError__)
            {
                std::cerr << "RealSense: " << message << std::endl;
                this->LastError__ = message;
            }
        }

        void StopPipeline()
        {
            if (this->PipelineStarted__)
            {
                try
                {
                    this->Pipeline__->stop();
                }
                catch (const std::exception& error)
                {
                    ReportError(error.what());
                }
                this->PipelineStarted__ = false;
            }
        }

        nlohmann::json Config__;
        g1_parkour::DepthProcessingConfig ProcessingConfig__;
        nlohmann::json Postprocess__;
        nlohmann::json SaveConfig__;
        std::function<uint64_t()> GenerationProvider__;
        bool Enabled__ = true;
        bool PipelineStarted__ = false;
        size_t BufferSize__ = 0;
        size_t IndexBase__ = 1;
        std::vector<size_t> SelectIndices__;
        std::deque<DepthMat> FrameBuffer__;
        uint64_t Generation__ = 0;
        uint64_t Sequence__ = 0;
        std::string LastError__;
        std::unique_ptr<rs2::pipeline> Pipeline__;
        rs2::decimation_filter DecimationFilter__;
        rs2::spatial_filter SpatialFilter__;
        rs2::temporal_filter TemporalFilter__;
        rs2::threshold_filter ThresholdFilter__;
        rs2::hole_filling_filter HoleFillingFilter__;
        rs2::disparity_transform DepthToDisparity__{true};
        rs2::disparity_transform DisparityToDepth__{false};
    };
}
