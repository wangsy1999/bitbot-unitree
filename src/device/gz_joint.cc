#include "bitbot_gz/device/gz_joint.h"

namespace bitbot {

GzJoint::GzJoint(const pugi::xml_node& device_node) : GzDevice(device_node) {
  basic_type_ = (uint32_t)BasicDeviceType::MOTOR;
  type_ = (uint32_t)GzDeviceType::GZ_JOINT;

  monitor_header_.headers = {"mode",
                             "actual_position",
                             "target_position",
                             "actual_velocity",
                             "target_velocity",
                             "actual_torque",
                             "target_torque"};
  monitor_data_.resize(monitor_header_.headers.size());

  target_position_ = 0.0;

  ConfigParser::ParseAttribute2d(initial_pos_,
                                 device_node.attribute("initial_pos"));
  ConfigParser::ParseAttribute2b(enable_, device_node.attribute("enable"));

  std::string mode_str;
  ConfigParser::ParseAttribute2s(mode_str, device_node.attribute("mode"));
  if (mode_str == "position") {
    joint_type_ = GzJointType::POSITION;
  } else if (mode_str == "velocity") {
    joint_type_ = GzJointType::VELOCITY;
  } else if (mode_str == "torque") {
    joint_type_ = GzJointType::TORQUE;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("bitbot_gz"),
                 "Invalid joint mode: %s. Defaulting to NONE.",
                 mode_str.c_str());
    joint_type_ = GzJointType::NONE;
  }
  ConfigParser::ParseAttribute2d(p_gain_, device_node.attribute("p_gain"));
  ConfigParser::ParseAttribute2d(d_gain_, device_node.attribute("d_gain"));
  ConfigParser::ParseAttribute2d(i_gain_, device_node.attribute("i_gain"));
}

GzJoint::~GzJoint() {}

void GzJoint::Input(const RosInterface::Ptr ros_interface) {
  auto joint_state_msg = ros_interface->GetJointState();
  actual_position_ = joint_state_msg->position[ros_joint_index_];
  actual_velocity_ = joint_state_msg->velocity[ros_joint_index_];
  actual_torque_ = joint_state_msg->effort[ros_joint_index_];
}

void GzJoint::Output(const RosInterface::Ptr ros_interface) {
  auto& joint_command_msg = ros_interface->GetJointCommand();

  switch (joint_type_) {
    case GzJointType::POSITION:
      pos_i_ += (target_position_ - actual_position_);
      joint_command_msg.data[ros_joint_index_] =
          p_gain_ * (target_position_ - actual_position_) -
          d_gain_ * actual_velocity_ + i_gain_ * pos_i_;
      break;
    case GzJointType::VELOCITY:
      joint_command_msg.data[ros_joint_index_] =
          p_gain_ * (target_velocity_ - actual_velocity_) -
          d_gain_ * (actual_velocity_ - last_velocity_);
      break;
    case GzJointType::TORQUE:
      joint_command_msg.data[ros_joint_index_] = target_torque_;
      break;
    default:
      break;
  }
}

void GzJoint::UpdateRuntimeData() {
  constexpr double rad2deg = 180.0 / M_PI;

  monitor_data_[0] = (int)joint_type_;
  monitor_data_[1] = rad2deg * actual_position_;
  monitor_data_[2] = rad2deg * target_position_;
  monitor_data_[3] = actual_velocity_;
  monitor_data_[4] = target_velocity_;
  monitor_data_[5] = actual_torque_;
  monitor_data_[6] = target_torque_;
}
void GzJoint::UpdateModel(const RosInterface::Ptr ros_interface) {
  ros_joint_index_ = ros_interface->GetJointIndex(name_);
}

}  // namespace bitbot
