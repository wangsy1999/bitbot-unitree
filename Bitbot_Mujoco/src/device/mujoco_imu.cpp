#include "bitbot_mujoco/device/mujoco_imu.h"

namespace bitbot{

  MujocoImu::MujocoImu(const pugi::xml_node& device_node)
    : MujocoDevice(device_node)
  {
    basic_type_ = (uint32_t)BasicDeviceType::IMU;
    type_ = (uint32_t)MujocoDeviceType::MUJOCO_IMU;

    monitor_header_.headers = {"roll", "pitch", "yaw", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"};
    monitor_data_.resize(monitor_header_.headers.size());

    ConfigParser::ParseAttribute2s(mj_site_name_, device_node.attribute("site"));
    if(mj_site_name_.empty())
    {
      logger_->error("Imu id:{} need a \"site\" attribute.", id_);
    }

    ConfigParser::ParseAttribute2s(mj_acc_name_, device_node.attribute("acc"));
    if(!mj_acc_name_.empty())
      has_acc_ = true;
    ConfigParser::ParseAttribute2s(mj_gyro_name_, device_node.attribute("gyro"));
    if(!mj_gyro_name_.empty())
      has_gyro_ = true;
  }

  MujocoImu::~MujocoImu()
  {
  }

  void MujocoImu::UpdateModel(const mjModel*m, mjData* mj_d)
  {
    int id = 0;
    if(id = mj_name2id(m, mjtObj::mjOBJ_SITE, mj_site_name_.c_str()); id != -1)
    {
      mj_site_id_ = id;
      mj_site_xmat_adr_ = mj_site_id_*9;
      logger_->debug("imu {} site_id:{}", name_, mj_site_id_);
    }
    else
      logger_->error("can not find site named {}", mj_site_name_);
    if(has_acc_)
    {
      if(id = mj_name2id(m, mjtObj::mjOBJ_SENSOR, mj_acc_name_.c_str()); id != -1)
      {
        mj_acc_id_ = id;
        mj_acc_adr_ = m->sensor_adr[mj_acc_id_];
        logger_->debug("imu {} id:{} acc_adr:{}", name_, mj_acc_id_, mj_acc_adr_);
      }
      else
        logger_->error("can not find sensor/acc named {}", mj_acc_name_);
    }
    if(has_gyro_)
    {
      if(id = mj_name2id(m, mjtObj::mjOBJ_SENSOR, mj_gyro_name_.c_str()); id != -1)
      {
        mj_gyro_id_ = id;
        mj_gyro_adr_ = m->sensor_adr[mj_gyro_id_];
        logger_->debug("imu {} id:{} gyro_adr:{}", name_, mj_gyro_id_, mj_gyro_adr_);
      }
      else
        logger_->error("can not find sensor/gyro named {}", mj_gyro_name_);
    }
  }

  void MujocoImu::Input(const mjModel*m, mjData* d)
  {
    for(int i=0;i<9;i++)
    {
      rot_mat_[i] = d->site_xmat[mj_site_xmat_adr_ + i];
    }
    Rot2RPY();

    if(has_acc_)
    {
      acc_x_ = d->sensordata[mj_acc_adr_];
      acc_y_ = d->sensordata[mj_acc_adr_+1];
      acc_z_ = d->sensordata[mj_acc_adr_+2];
    }
    if(has_gyro_)
    {
      gyro_x_ = d->sensordata[mj_gyro_adr_];
      gyro_y_ = d->sensordata[mj_gyro_adr_+1];
      gyro_z_ = d->sensordata[mj_gyro_adr_+2];
    }
  }

  void MujocoImu::Output(const mjModel*m, mjData* mj_d)
  {
    return;
  }

  void MujocoImu::UpdateRuntimeData()
  {
    constexpr double rad2deg = 180.0/M_PI;

    monitor_data_[0] = rad2deg * roll_;
    monitor_data_[1] = rad2deg * pitch_;
    monitor_data_[2] = rad2deg * yaw_;
    monitor_data_[3] = acc_x_;
    monitor_data_[4] = acc_y_;
    monitor_data_[5] = acc_z_;
    monitor_data_[6] = gyro_x_;
    monitor_data_[7] = gyro_y_;
    monitor_data_[8] = gyro_z_;
  }

  void MujocoImu::Rot2RPY()
  {
    constexpr int row_num = 3;
    roll_ = std::atan2(rot_mat_[2 * row_num + 1], rot_mat_[2 * row_num + 2]);
    pitch_ = std::atan2(-rot_mat_[2 * row_num], sqrt(rot_mat_[2 * row_num + 1] * rot_mat_[2 * row_num + 1] + rot_mat_[2 * row_num + 2] * rot_mat_[2 * row_num + 2]));
    yaw_ = std::atan2(rot_mat_[1 * row_num], rot_mat_[0]);
  }
}
