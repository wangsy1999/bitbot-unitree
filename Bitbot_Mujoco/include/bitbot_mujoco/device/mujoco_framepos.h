#pragma once

#include "bitbot_mujoco/device/mujoco_device.hpp"
#include <array>
#include <optional>

namespace bitbot
{

  class MujocoFramepos final: public MujocoDevice
  {
  public:
    MujocoFramepos(const pugi::xml_node& device_node);
    ~MujocoFramepos();

    inline std::array<double,3> GetPos()
    {
      return pos_;
    }

  private:
    virtual void UpdateModel(const mjModel*m, mjData* mj_d);
    virtual void Input(const mjModel*m, mjData* mj_d) final;
    virtual void Output(const mjModel*m, mjData* mj_d) final;
    virtual void UpdateRuntimeData() final;

    std::string mj_sensor_name_;
    int mj_sensor_id_ = 0;
    int mj_sensor_adr_ = 0;

    std::array<double,3> pos_;
  };


}
