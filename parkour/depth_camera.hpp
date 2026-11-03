/**
 * @file depth_camera.hpp
 * @brief MuJoCo 离屏深度采集及实时预览。
 */
#pragma once

#include "depth_config.hpp"
#include <mujoco/mujoco.h>
#include <array>
#include <atomic>

struct GLFWwindow;

namespace g1_parkour
{
    /**
     * @brief 参考 vision/CtrlZ 的 MujocoDepthCamera，输出顶部为首行的米制深度图。
     * @details UI 线程创建和销毁窗口；采集及绘制由物理线程执行。
     */
    class DepthCamera
    {
    public:
        using RawDepthImage = std::array<float, MujocoDepthLayout::Pixels>;

        explicit DepthCamera(const mjModel* model);
        ~DepthCamera();
        DepthCamera(const DepthCamera&) = delete;
        DepthCamera& operator=(const DepthCamera&) = delete;

        RawDepthImage Capture(const mjModel* model, mjData* data);
        const float* GetDepth() const { return this->DepthBuffer__.data(); }
        int GetMjCameraId() const { return this->CameraId__; }

    private:
        void ShowPreview(const RawDepthImage& depth);

        GLFWwindow* Window__ = nullptr;
        mjrContext RenderContext__{};
        mjvScene Scene__{};
        mjvOption RenderOption__{};
        int CameraId__ = -1;
        RawDepthImage DepthBuffer__{};
        unsigned int PreviewTexture__ = 0;
        std::atomic<bool> PreviewOpen__{true};
    };
}
