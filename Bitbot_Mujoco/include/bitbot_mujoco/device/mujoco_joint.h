#pragma once

#include "bitbot_mujoco/device/mujoco_device.hpp"
#include <array>
#include <map>
#include <optional>

namespace bitbot
{
  enum class MujocoJointMode
  {
    NONE = 0,
    POSITION,
    VELOCITY,
    TORQUE,
    MIT
  };

  class MujocoJoint final : public MujocoDevice
  {
  public:
    MujocoJoint(const pugi::xml_node& device_node);
    ~MujocoJoint();

    static std::string ModeToStr(const MujocoJointMode& mode);

    static MujocoJointMode ModeFromStr(const std::string& mode);

    inline double GetActualPosition()
    {
      return actual_position_;
    }

    inline double GetActualVelocity()
    {
      return actual_velocity_;
    }

    inline double GetActualTorque()
    {
      return actual_torque_;
    }

    inline void SetTargetPosition(double pos)
    {
      target_position_ = pos;
    }

    inline void SetTargetVelocity(double vel)
    {
      target_velocity_ = vel;
    }

    inline void SetTargetTorque(double torque)
    {
      target_torque_ = torque;
    }

    inline void SetMode(MujocoJointMode mode)
    {
      if (mode_ != MujocoJointMode::POSITION)
        pos_i_ = 0;
      mode_ = mode;
    }

    double GetMotionKp() const
    {
      return pos_kp_;
    }

    double GetMotionKd() const
    {
      return pos_kd_;
    }

    void SetMotionKp(double kp)
    {
      pos_kp_ = kp;
    }

    void SetMotionKd(double kd)
    {
      pos_kd_ = kd;
    }

    double GetMotionKi() const
    {
      return pos_ki_;
    }

    void SetMotionKi(double ki)
    {
      pos_ki_ = ki;
    }

    double GetVelocityKp() const
    {
      return vel_kp_;
    }

    double GetVelocityKd() const
    {
      return vel_kd_;
    }

    void SetVelocityKp(double kp)
    {
      vel_kp_ = kp;
    }
    void SetVelocityKd(double kd)
    {
      vel_kd_ = kd;
    }
    double GetVelocityKi() const
    {
      return vel_ki_;
    }
    void SetVelocityKi(double ki)
    {
      vel_ki_ = ki;
    }

    double GetPositionKp() const
    {
      return pos_kp_;
    }

    double GetPositionKd() const
    {
      return pos_kd_;
    }

    void SetPositionKp(double kp)
    {
      pos_kp_ = kp;
    }

    void SetPositionKd(double kd)
    {
      pos_kd_ = kd;
    }

    void SetPositionKi(double ki)
    {
      pos_ki_ = ki;
    }

    double GetPositionKi() const
    {
      return pos_ki_;
    }


  private:
    virtual void UpdateModel(const mjModel* m, mjData* mj_d);
    virtual void Input(const mjModel* m, mjData* mj_d) final;
    virtual void Output(const mjModel* m, mjData* mj_d) final;

    virtual void UpdateRuntimeData() final;

    bool enable_ = true;
    int mj_joint_id_ = 0;
    int mj_jointqpos_adr_ = 0;
    int mj_jointqvel_adr_ = 0;
    int mj_actuator_id_ = 0;
    int mj_control_adr_ = 0;
    int mj_actuatorforce_adr_ = 0;
    bool native_pid_ = false;

    int mj_joint_type_ = mjJNT_HINGE;
    double initial_pos_ = 0;

    MujocoJointMode mode_;
    double actual_position_ = 0;
    double actual_velocity_ = 0;
    double actual_torque_ = 0;
    double target_position_ = 0;
    double target_velocity_ = 0;
    double target_torque_ = 0;

    double pos_i_ = 0; // 位控积分值
    double pos_kp_ = 100;
    double pos_kd_ = 50;
    double pos_ki_ = 10;

    double vel_i_ = 0; // 速度控制积分值
    double vel_kp_ = 100;
    double vel_kd_ = 50;
    double vel_ki_ = 10;
  };


}
