#include "bitbot_mujoco/device/mujoco_framepos.h"

namespace bitbot{

  MujocoFramepos::MujocoFramepos(const pugi::xml_node& device_node)
    : MujocoDevice(device_node)
  {
    basic_type_ = (uint32_t)BasicDeviceType::SENSOR;
    type_ = (uint32_t)MujocoDeviceType::MUJOCO_FRAMEPOS;

    monitor_header_.headers = {"framepos_x", "framepos_y", "framepos_z"};
    monitor_data_.resize(monitor_header_.headers.size());

    ConfigParser::ParseAttribute2s(mj_sensor_name_, device_node.attribute("sensor_name"));
    if(mj_sensor_name_.empty())
    {
      mj_sensor_name_ = this->name_;
    }
  }

  MujocoFramepos::~MujocoFramepos()
  {
  }

  void MujocoFramepos::UpdateModel(const mjModel*m, mjData* mj_d)
  {
    int id = 0;
    if(id = mj_name2id(m, mjtObj::mjOBJ_SENSOR, mj_sensor_name_.c_str()); id != -1)
    {
      mj_sensor_id_ = id;
      mj_sensor_adr_ = m->sensor_adr[mj_sensor_id_];
      logger_->debug("framepos {} sensor_id:{}", name_, mj_sensor_id_);
    }
    else
      logger_->error("can not find sensor named {}", mj_sensor_name_);
  }

  void MujocoFramepos::Input(const mjModel*m, mjData* d)
  {
    pos_[0] = d->sensordata[mj_sensor_adr_];
    pos_[1] = d->sensordata[mj_sensor_adr_+1];
    pos_[2] = d->sensordata[mj_sensor_adr_+2];
  }

  void MujocoFramepos::Output(const mjModel*m, mjData* mj_d)
  {
    return;
  }

  void MujocoFramepos::UpdateRuntimeData()
  {
    monitor_data_[0] = pos_[0];
    monitor_data_[1] = pos_[1];
    monitor_data_[2] = pos_[2];
  }
}
