#include "bitbot_gz/device/gz_imu.h"

#include <Eigen/Geometry>

namespace bitbot {

GzImu::GzImu(pugi::xml_node const& device_node) : GzDevice(device_node) {
  basic_type_ = (uint32_t)BasicDeviceType::IMU;
  type_ = (uint32_t)GzDeviceType::GZ_IMU;

  monitor_header_.headers = {"roll",  "pitch",  "yaw",    "acc_x", "acc_y",
                             "acc_z", "gyro_x", "gyro_y", "gyro_z"};
  monitor_data_.resize(monitor_header_.headers.size());
}

GzImu::~GzImu() {}

void GzImu::Input(const RosInterface::Ptr ros_interface) {
  auto imu_msg = ros_interface->GetImuData();

  Eigen::Quaterniond q(imu_msg->orientation.w, imu_msg->orientation.x,
                       imu_msg->orientation.y, imu_msg->orientation.z);
  Eigen::Matrix3d rot_mat = q.toRotationMatrix();
  roll_ = atan2(rot_mat(2, 1), rot_mat(2, 2));
  pitch_ = atan2(-rot_mat(2, 0), sqrt(rot_mat(2, 1) * rot_mat(2, 1) +
                                      rot_mat(2, 2) * rot_mat(2, 2)));
  yaw_ = atan2(rot_mat(1, 0), rot_mat(0, 0));

  acc_x_ = imu_msg->linear_acceleration.x;
  acc_y_ = imu_msg->linear_acceleration.y;
  acc_z_ = imu_msg->linear_acceleration.z;

  gyro_x_ = imu_msg->angular_velocity.x;
  gyro_y_ = imu_msg->angular_velocity.y;
  gyro_z_ = imu_msg->angular_velocity.z;
}

void GzImu::Output(const RosInterface::Ptr) {}

void GzImu::UpdateRuntimeData() {
  constexpr double rad2deg = 180.0 / M_PI;

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

void GzImu::UpdateModel(const RosInterface::Ptr ros_interface) {}

}  // namespace bitbot
