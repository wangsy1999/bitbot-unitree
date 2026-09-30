#pragma once

#include "bitbot_kernel/device/device.hpp"
#include "mujoco/mujoco.h"

namespace bitbot
{
  enum class MujocoDeviceType : uint32_t
  {
    MUJOCO_DEVICE = 11000,
    MUJOCO_JOINT,
    MUJOCO_FORCE_SENSOR,
    MUJOCO_IMU,
    MUJOCO_FRAMEPOS,
    MUJOCO_FRAMELINVEL
  };

  class MujocoDevice : public Device
  {
  public:
    MujocoDevice(const pugi::xml_node& device_node)
      : Device(device_node)
    {
    }

    ~MujocoDevice() = default;

    virtual void UpdateModel(const mjModel*m, mjData* mj_d) = 0;

    virtual void Input(const mjModel*m, mjData* mj_d) = 0;
    virtual void Output(const mjModel*m, mjData* mj_d) = 0;
    virtual void UpdateRuntimeData() = 0;
  private:
    /* data */
  };


}
