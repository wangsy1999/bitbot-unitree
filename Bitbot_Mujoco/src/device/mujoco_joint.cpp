#include "bitbot_mujoco/device/mujoco_joint.h"
#include <stdexcept>

namespace bitbot{

  MujocoJoint::MujocoJoint(const pugi::xml_node& device_node)
    : MujocoDevice(device_node)
  {
    constexpr double deg2rad = M_PI/180.0;

    basic_type_ = (uint32_t)BasicDeviceType::MOTOR;
    type_ = (uint32_t)MujocoDeviceType::MUJOCO_JOINT;

    monitor_header_.headers = {"mode","actual_position", "target_position", "actual_velocity", "target_velocity", "actual_torque","target_torque"};
    monitor_data_.resize(monitor_header_.headers.size());

    ConfigParser::ParseAttribute2d(initial_pos_, device_node.attribute("initial_pos"));
    initial_pos_ *= deg2rad;
    target_position_ = initial_pos_;

    ConfigParser::ParseAttribute2b(enable_, device_node.attribute("enable"));
    std::string mode_str;
    ConfigParser::ParseAttribute2s(mode_str, device_node.attribute("mode"));
    mode_ = ModeFromStr(mode_str);

    ConfigParser::ParseAttribute2d(pos_kp_, device_node.attribute("pos_kp"));
    ConfigParser::ParseAttribute2d(pos_kd_, device_node.attribute("pos_kd"));
    ConfigParser::ParseAttribute2d(pos_ki_, device_node.attribute("pos_ki"));
    ConfigParser::ParseAttribute2d(vel_kp_, device_node.attribute("vel_kp"));
    ConfigParser::ParseAttribute2d(vel_kd_, device_node.attribute("vel_kd"));
    ConfigParser::ParseAttribute2d(vel_ki_, device_node.attribute("vel_ki"));
  }

  MujocoJoint::~MujocoJoint()
  {
  }

  void MujocoJoint::UpdateModel(const mjModel*m, mjData* mj_d)
  {
    int id = 0;
    if(id = mj_name2id(m, mjtObj::mjOBJ_JOINT, name_.c_str()); id != -1)
    {
      mj_joint_id_ = id;
      mj_jointqpos_adr_ = m->jnt_qposadr[mj_joint_id_];
      mj_jointqvel_adr_ = m->jnt_dofadr[mj_joint_id_];
      logger_->debug("joint {} id:{} pos_adr:{} vel_adr:{}", name_, mj_joint_id_, mj_jointqpos_adr_, mj_jointqvel_adr_);
      mj_d->qpos[mj_jointqpos_adr_] = initial_pos_;
      mj_joint_type_ = m->jnt_type[mj_joint_id_];
      if((mj_joint_type_ != mjJNT_HINGE) && (mj_joint_type_ != mjJNT_SLIDE))
      {
        logger_->error("joint named {} has unsupported type", name_);
      }
    }
    else
      logger_->error("can not find joint named {}", name_);

    if(id = mj_name2id(m, mjtObj::mjOBJ_ACTUATOR, name_.c_str()); id != -1)
    {
      mj_actuator_id_ = id;
      mj_control_adr_ = m->actuator_ctrladr[id];
      mj_actuatorforce_adr_ = m->actuator_outadr[id];
      native_pid_ = m->actuator_gaintype[id] == mjGAIN_PID;
      if (mode_ == MujocoJointMode::MIT)
      {
        if (!native_pid_ || m->actuator_ctrlnum[id] != 3 ||
            m->actuator_ctrlspec[id] != (mjINPUT_POS | mjINPUT_VEL | mjINPUT_FF) ||
            m->actuator_dyntype[id] != mjDYN_NONE ||
            m->actuator_biastype[id] != mjBIAS_AFFINE ||
            m->actuator_outnum[id] != 1 ||
            m->actuator_trntype[id] != mjTRN_JOINT ||
            m->actuator_trnid[2 * id] != mj_joint_id_ ||
            m->actuator_gear[6 * mj_actuatorforce_adr_] != 1)
        {
          throw std::runtime_error(
              "MIT mode requires a stateless joint PID with input='pos vel ff' and gear=1: " + name_);
        }
        Output(m, mj_d);
      }
      else if (native_pid_ || m->actuator_ctrlnum[id] != 1 || m->actuator_outnum[id] != 1)
      {
        throw std::runtime_error("Non-MIT MujocoJoint requires a single-input/output actuator: " + name_);
      }
      logger_->debug("joint {} act_id:{}", name_, mj_actuator_id_);
    }
    else
      logger_->error("can not find actuator named {}", name_);
  }

  void MujocoJoint::Input(const mjModel*m, mjData* mj_d)
  {
    actual_position_ = mj_d->qpos[mj_jointqpos_adr_];
    actual_velocity_ = mj_d->qvel[mj_jointqvel_adr_];
    actual_torque_ = mj_d->actuator_force[mj_actuatorforce_adr_];
  }

  void MujocoJoint::Output(const mjModel*m, mjData* mj_d)
  {
    static double last_velocity = 0;

    if (native_pid_)
    {
      if (mode_ != MujocoJointMode::MIT && mode_ != MujocoJointMode::NONE &&
          mode_ != MujocoJointMode::TORQUE)
      {
        throw std::runtime_error("Native PID supports mit, torque or none mode: " + name_);
      }
      const bool active = enable_ && mode_ != MujocoJointMode::NONE;
      const bool feedback = active && mode_ == MujocoJointMode::MIT;
      // MuJoCo 3.14 PID stores Kp/Kd in the affine bias parameters. Update
      // them on the physics thread together with the command, before mj_step
      // computes actuation. MITMotorControlWorker can change these each cycle.
      m->actuator_biasprm[mjNBIAS * mj_actuator_id_ + 1] = feedback ? -pos_kp_ : 0;
      m->actuator_biasprm[mjNBIAS * mj_actuator_id_ + 2] = feedback ? -pos_kd_ : 0;
      mj_d->ctrl[mj_control_adr_] = feedback ? target_position_ : 0;
      mj_d->ctrl[mj_control_adr_ + 1] = feedback ? target_velocity_ : 0;
      mj_d->ctrl[mj_control_adr_ + 2] = active ? target_torque_ : 0;
      return;
    }

    if(!enable_)
    {
      mj_d->ctrl[mj_control_adr_] = 0;
      return;
    }

    switch (mode_)
    {
    case MujocoJointMode::POSITION:
        pos_i_ += (target_position_-actual_position_);
        mj_d->ctrl[mj_control_adr_] = pos_kp_*(target_position_-actual_position_)-pos_kd_*actual_velocity_+pos_ki_*pos_i_;
      break;
    case MujocoJointMode::VELOCITY:
        mj_d->ctrl[mj_control_adr_] = vel_kp_*(target_velocity_-actual_velocity_)-vel_kd_*(actual_velocity_-last_velocity);
        last_velocity = actual_velocity_;
      break;
    case MujocoJointMode::TORQUE:
        mj_d->ctrl[mj_control_adr_] = target_torque_;
      break;
    case MujocoJointMode::MIT:
      throw std::runtime_error("MIT mode requires a native PID actuator: " + name_);
    default:
      mj_d->ctrl[mj_control_adr_] = 0;
      break;
    }
  }

  void MujocoJoint::UpdateRuntimeData()
  {
    constexpr double rad2deg = 180.0/M_PI;

    monitor_data_[0] = static_cast<int>(mode_);
    if(mj_joint_type_ == mjJNT_HINGE)
    {
      monitor_data_[1] = rad2deg * actual_position_;
      monitor_data_[2] = rad2deg * target_position_;
    }
    else
    {
      monitor_data_[1] = actual_position_;
      monitor_data_[2] = target_position_;
    }
    monitor_data_[3] = actual_velocity_;
    monitor_data_[4] = target_velocity_;
    monitor_data_[5] = actual_torque_;
    monitor_data_[6] = target_torque_;
  }

  std::string MujocoJoint::ModeToStr(const MujocoJointMode& mode)
  {
    static const std::array<std::string, 5> mode_str = {"none", "position", "velocity", "torque", "mit"};

    return mode_str[(int)mode];
  }

  MujocoJointMode MujocoJoint::ModeFromStr(const std::string& mode)
  {
    static std::map<std::string, MujocoJointMode> str_mode;
    static bool init = true;
    if(init)
    {
      str_mode["none"] = MujocoJointMode::NONE;
      str_mode["position"] = MujocoJointMode::POSITION;
      str_mode["velocity"] = MujocoJointMode::VELOCITY;
      str_mode["torque"] = MujocoJointMode::TORQUE;
      str_mode["mit"] = MujocoJointMode::MIT;
      init = false;
    }

    if(auto result = str_mode.find(mode); result != str_mode.end())
    {
      return result->second;
    }
    else
    {
      Logger().ConsoleLogger()->error("unkown mujoco motor mode: {}", mode);
      return MujocoJointMode::NONE;
    }
  }


}
