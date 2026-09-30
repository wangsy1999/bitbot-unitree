#pragma once

#include "bitbot_mujoco/device/mujoco_device.hpp"
#include <array>
#include <map>
#include <optional>

namespace bitbot
{

  class MujocoForceSensor final: public MujocoDevice
  {
  public:
    MujocoForceSensor(const pugi::xml_node& device_node);
    ~MujocoForceSensor();

    inline double GetForceX()
    {
      return f_x_;
    }
    inline double GetForceY()
    {
      return f_y_;
    }
    inline double GetForceZ()
    {
      return f_z_;
    }
    inline double GetTorqueX()
    {
      return t_x_;
    }
    inline double GetTorqueY()
    {
      return t_y_;
    }
    inline double GetTorqueZ()
    {
      return t_z_;
    }


  private:
    virtual void UpdateModel(const mjModel*m, mjData* mj_d);
    virtual void Input(const mjModel*m, mjData* mj_d) final;
    virtual void Output(const mjModel*m, mjData* mj_d) final;

    virtual void UpdateRuntimeData() final;

    bool has_force_ = false;
    std::string mj_force_name_;
    int mj_force_id_ = 0;
    int mj_force_adr_ = 0;
    bool has_torque_ = false;
    std::string mj_torque_name_;
    int mj_torque_id_ = 0;
    int mj_torque_adr_ = 0;

    double f_x_ = 0;
    double f_y_ = 0;
    double f_z_ = 0;
    double t_x_ = 0;
    double t_y_ = 0;
    double t_z_ = 0;

  };


}
