/**
 * @file mujoco_depth_camera.cpp
 * @brief 深度相机设备注册、Input 分频及原始缓冲发布。
 */
#include "mujoco_depth_camera.h"
#include "bitbot_kernel/device/device_factory.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bitbot
{
    namespace
    {
        // 应用侧扩展设备工厂，保留原依赖包和 main 的启动流程。
        DeviceRegistrar<MujocoDevice, MujocoDepthCamera> DepthCameraRegistrar(
            MujocoDepthCamera::DeviceType, "MujocoDepthCamera");
    }

    MujocoDepthCamera::MujocoDepthCamera(const pugi::xml_node& device_node)
        : MujocoDevice(device_node)
    {
        this->basic_type_ = static_cast<uint32_t>(BasicDeviceType::SENSOR);
        this->type_ = DeviceType;
        this->monitor_header_.headers = {"depth_mean"};
        this->monitor_data_.resize(1);
        ConfigParser::ParseAttribute2s(this->CameraName__, device_node.attribute("camera"));
        ConfigParser::ParseAttribute2ui(this->Width__, device_node.attribute("width"));
        ConfigParser::ParseAttribute2ui(this->Height__, device_node.attribute("height"));
        ConfigParser::ParseAttribute2ui(this->SubsampleSteps__, device_node.attribute("subsample"));
        SetSubsampleSteps(this->SubsampleSteps__);
        if (this->CameraName__ != "parkour_depth" || this->Width__ != 64 || this->Height__ != 36)
        {
            throw std::runtime_error("Parkour depth camera requires parkour_depth at 64x36");
        }
    }

    void MujocoDepthCamera::InitializeRender(const mjModel* model)
    {
        this->Camera__ = std::make_unique<g1_parkour::DepthCamera>(model);
        this->InputTick__ = 0;
        this->FrameSequence__ = 0;
        this->DepthMean__ = 0;
    }

    void MujocoDepthCamera::ReleaseRender()
    {
        this->Camera__.reset();
        this->FrameSequence__ = 0;
    }

    const float* MujocoDepthCamera::GetDepth() const
    {
        return this->Camera__ && this->FrameSequence__ ? this->Camera__->GetDepth() : nullptr;
    }

    int MujocoDepthCamera::GetMjCameraId() const
    {
        return this->Camera__ ? this->Camera__->GetMjCameraId() : -1;
    }

    void MujocoDepthCamera::SetSubsampleSteps(unsigned steps)
    {
        this->SubsampleSteps__ = std::max(1u, steps);
        this->InputTick__ = 0;
    }

    void MujocoDepthCamera::UpdateModel(const mjModel* model, mjData*)
    {
        // ConfigFunc 在 UI 线程提前初始化窗口，设备回调只核对模型并重置采集相位。
        if (!this->Camera__ || mj_name2id(model, mjOBJ_CAMERA, this->CameraName__.c_str()) != GetMjCameraId())
        {
            throw std::runtime_error("Depth camera render context/model not initialized");
        }
        this->InputTick__ = 0;
        this->FrameSequence__ = 0;
    }

    void MujocoDepthCamera::Input(const mjModel* model, mjData* data)
    {
        if (!this->Camera__ || this->InputTick__++ % this->SubsampleSteps__ != 0)
        {
            return;
        }
        this->Camera__->Capture(model, data);
        ++this->FrameSequence__;
        this->FrameTime__ = data->time;

        double sum = 0;
        size_t count = 0;
        const float* depth = GetDepth();
        for (size_t i = 0; i < this->Width__ * this->Height__; ++i)
        {
            if (std::isfinite(depth[i]) && depth[i] > 0)
            {
                sum += depth[i];
                ++count;
            }
        }
        this->DepthMean__ = count ? sum / count : 0;
    }

    void MujocoDepthCamera::Output(const mjModel*, mjData*)
    {
    }

    void MujocoDepthCamera::UpdateRuntimeData()
    {
        this->monitor_data_[0] = this->DepthMean__;
    }
}
