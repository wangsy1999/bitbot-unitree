/**
 * @file realsense_depth_types.hpp
 * @brief RealSense 深度历史快照，供采集与观测 Worker 交换数据。
 */
#pragma once

#include "depth_config.hpp"
#include <array>
#include <cstdint>

namespace g1_parkour
{
    struct RealsenseDepthFrame
    {
        uint64_t Sequence = 0;
        uint64_t Generation = 0;
        double CapturedAt = 0; // 本机 steady_clock 秒，表示接收帧的时刻。
        std::array<float, ModelLayout::DepthHistorySize> Values{};
    };
}
