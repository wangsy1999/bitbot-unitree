#include "bitbot_mujoco/device/mujoco_force_sensor.h"

namespace bitbot{

  MujocoForceSensor::MujocoForceSensor(const pugi::xml_node& device_node)
    : MujocoDevice(device_node)
  {
    basic_type_ = (uint32_t)BasicDeviceType::MOTOR;
    type_ = (uint32_t)MujocoDeviceType::MUJOCO_FORCE_SENSOR;

    monitor_header_.headers = {"f_x", "f_y", "f_z", "t_x", "t_y", "t_z"};
    monitor_data_.resize(monitor_header_.headers.size());

    ConfigParser::ParseAttribute2s(mj_force_name_, device_node.attribute("force"));
    if(!mj_force_name_.empty())
      has_force_ = true;
    ConfigParser::ParseAttribute2s(mj_torque_name_, device_node.attribute("torque"));
    if(!mj_torque_name_.empty())
      has_torque_ = true;
  }

  MujocoForceSensor::~MujocoForceSensor()
  {
  }

  void MujocoForceSensor::UpdateModel(const mjModel*m, mjData* mj_d)
  {
    int id = 0;
    if(has_force_)
    {
      if(id = mj_name2id(m, mjtObj::mjOBJ_SENSOR, mj_force_name_.c_str()); id != -1)
      {
        mj_force_id_ = id;
        mj_force_adr_ = m->sensor_adr[mj_force_id_];
        logger_->debug("force_sensor {} id:{} force_adr:{}", name_, mj_force_id_, mj_force_adr_);
      }
      else
        logger_->error("can not find sensor/force named {}", mj_force_name_);
    }
    if(has_torque_)
    {
      if(id = mj_name2id(m, mjtObj::mjOBJ_SENSOR, mj_torque_name_.c_str()); id != -1)
      {
        mj_torque_id_ = id;
        mj_torque_adr_ = m->sensor_adr[mj_torque_id_];
        logger_->debug("force_sensor {} id:{} torque_adr:{}", name_, mj_torque_id_, mj_torque_adr_);
      }
      else
        logger_->error("can not find sensor/torque named {}", mj_torque_name_);
    }
  }

  void MujocoForceSensor::Input(const mjModel*m, mjData* d)
  {
    if(has_force_)
    {
      f_x_ = d->sensordata[mj_force_adr_];
      f_y_ = d->sensordata[mj_force_adr_+1];
      f_z_ = d->sensordata[mj_force_adr_+2];
    }
    if(has_torque_)
    {
      t_x_ = d->sensordata[mj_torque_adr_];
      t_y_ = d->sensordata[mj_torque_adr_+1];
      t_z_ = d->sensordata[mj_torque_adr_+2];
    }
  }

  void MujocoForceSensor::Output(const mjModel*m, mjData* mj_d)
  {
    return;
  }

  void MujocoForceSensor::UpdateRuntimeData()
  {
    monitor_data_[0] = f_x_;
    monitor_data_[1] = f_y_;
    monitor_data_[2] = f_z_;
    monitor_data_[3] = t_x_;
    monitor_data_[4] = t_y_;
    monitor_data_[5] = t_z_;
  }

}
