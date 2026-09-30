#include "bitbot_mujoco/device/mujoco_framelinvel.h"

namespace bitbot{

  MujocoFramelinvel::MujocoFramelinvel(const pugi::xml_node& device_node)
    : MujocoDevice(device_node)
  {
    basic_type_ = (uint32_t)BasicDeviceType::SENSOR;
    type_ = (uint32_t)MujocoDeviceType::MUJOCO_FRAMELINVEL;

    monitor_header_.headers = {"linvel_x", "linvel_y", "linvel_z"};
    monitor_data_.resize(monitor_header_.headers.size());

    ConfigParser::ParseAttribute2s(mj_sensor_name_, device_node.attribute("sensor_name"));
    if(mj_sensor_name_.empty())
    {
      mj_sensor_name_ = this->name_;
    }
  }

  MujocoFramelinvel::~MujocoFramelinvel()
  {
  }

  void MujocoFramelinvel::UpdateModel(const mjModel*m, mjData* mj_d)
  {
    int id = 0;
    if(id = mj_name2id(m, mjtObj::mjOBJ_SENSOR, mj_sensor_name_.c_str()); id != -1)
    {
      mj_sensor_id_ = id;
      mj_sensor_adr_ = m->sensor_adr[mj_sensor_id_];
      logger_->debug("framelinvel {} sensor_id:{}", name_, mj_sensor_id_);
    }
    else
      logger_->error("can not find sensor named {}", mj_sensor_name_);
  }

  void MujocoFramelinvel::Input(const mjModel*m, mjData* d)
  {
    linvel_[0] = d->sensordata[mj_sensor_adr_];
    linvel_[1] = d->sensordata[mj_sensor_adr_+1];
    linvel_[2] = d->sensordata[mj_sensor_adr_+2];
  }

  void MujocoFramelinvel::Output(const mjModel*m, mjData* mj_d)
  {
    return;
  }

  void MujocoFramelinvel::UpdateRuntimeData()
  {
    monitor_data_[0] = linvel_[0];
    monitor_data_[1] = linvel_[1];
    monitor_data_[2] = linvel_[2];
  }
}
