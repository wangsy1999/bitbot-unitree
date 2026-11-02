/**
 * @file mujoco_depth_camera.h
 * @brief 按 vision/CtrlZ 接口封装的 MuJoCo 深度相机设备。
 */
#pragma once

#include "depth_camera.hpp"
#include "bitbot_mujoco/device/mujoco_device.hpp"
#include <memory>

namespace bitbot
{
    /**
     * @brief 在总线 Input() 中采集；观测 Worker 通过 GetDepth() 读取原始缓冲。
     * @details 渲染后端保留策略深度预览。GLFW 资源在 UI 线程初始化和释放。
     */
    class MujocoDepthCamera final : public MujocoDevice
    {
    public:
        static constexpr uint32_t DeviceType = static_cast<uint32_t>(MujocoDeviceType::MUJOCO_FRAMELINVEL) + 1;

        explicit MujocoDepthCamera(const pugi::xml_node& device_node);

        const float* GetDepth() const;
        unsigned GetWidth() const { return this->Width__; }
        unsigned GetHeight() const { return this->Height__; }
        int GetMjCameraId() const;
        uint64_t GetFrameSequence() const { return this->FrameSequence__; }
        double GetFrameTime() const { return this->FrameTime__; }
        void SetSubsampleSteps(unsigned steps);
        void InitializeRender(const mjModel* model);
        void ReleaseRender();

    private:
        void UpdateModel(const mjModel* model, mjData* data) override;
        void Input(const mjModel* model, mjData* data) override;
        void Output(const mjModel* model, mjData* data) override;
        void UpdateRuntimeData() override;

        std::unique_ptr<g1_parkour::DepthCamera> Camera__;
        std::string CameraName__;
        unsigned Width__ = g1_parkour::MujocoDepthLayout::Width;
        unsigned Height__ = g1_parkour::MujocoDepthLayout::Height;
        unsigned SubsampleSteps__ = 1;
        uint64_t InputTick__ = 0;
        uint64_t FrameSequence__ = 0;
        double FrameTime__ = 0;
        double DepthMean__ = 0;
    };
}
