#ifndef GZ_NODE_HPP
#define GZ_NODE_HPP

#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace bitbot {

using namespace std::chrono_literals;

class RosInterface : public rclcpp::Node {
 public:
  using Ptr = std::shared_ptr<RosInterface>;
  RosInterface() : rclcpp::Node("bitbot_ros_interface") {
    timer_ready_.store(false);
    joint_map_ready_.store(false);

    joint_command_publisher_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/effort_controller/commands", 10);
    joint_state_subscriber_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
              std::lock_guard<std::mutex> lock(this->data_lock_);
              if (!joint_map_ready_.load()) {
                this->joint_name_index_map_.clear();
                for (size_t i = 0; i < msg->name.size(); ++i) {
                  this->joint_name_index_map_[msg->name[i]] = i;
                }
                this->joint_command_msg_.data.resize(msg->name.size());
                joint_map_ready_.store(true);
              }
              this->joint_state_msg_ = msg;
            });
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(this->data_lock_);
          this->imu_msg_ = msg;
        });
    clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 10, [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
          this->clock_count_++;
          // Clock rate is 1000
          timer_ready_.store(true);
        });
  }

  ~RosInterface() {
    if (ros_loop_ && ros_loop_->joinable()) {
      ros_loop_->join();
    }
  }

  void PublishJointCommand() {
    joint_command_publisher_->publish(joint_command_msg_);
  }

  sensor_msgs::msg::JointState::SharedPtr GetJointState() {
    std::lock_guard<std::mutex> lock(data_lock_);
    return joint_state_msg_;
  }

  std_msgs::msg::Float64MultiArray& GetJointCommand() {
    return joint_command_msg_;
  }

  sensor_msgs::msg::Imu::SharedPtr GetImuData() {
    std::lock_guard<std::mutex> lock(data_lock_);
    return imu_msg_;
  }

  bool IsClockReady() {
    if (timer_ready_.load()) {
      timer_ready_.store(false);
      return true;
    }
    return false;
  }

  bool IsSystemReady() {
    std::lock_guard<std::mutex> lock(this->data_lock_);
    if (joint_map_ready_.load() && imu_msg_ != nullptr)
      return true;
    else
      return false;
  }

  size_t GetJointIndex(std::string const& joint_name) {
    std::lock_guard<std::mutex> lock(data_lock_);
    if (joint_map_ready_.load()) {
      auto it = joint_name_index_map_.find(joint_name);
      if (it != joint_name_index_map_.end()) {
        return it->second;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("bitbot_ros_interface"),
                     "Joint name %s not found in map.", joint_name.c_str());
        return static_cast<size_t>(-1);
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("bitbot_ros_interface"),
                   "Joint map is not ready.");
      return static_cast<size_t>(-1);
    }
  }

  static void RunRosSpin(Ptr ptr) {
    RCLCPP_INFO(rclcpp::get_logger("bitbot_ros_interface"),
                "Starting ROS spin loop...");
    ptr->ros_loop_ =
        std::make_shared<std::thread>([ptr]() { rclcpp::spin(ptr); });
  }

 private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      joint_command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;

  size_t clock_count_ = 0;

  std::mutex data_lock_;
  sensor_msgs::msg::JointState::SharedPtr joint_state_msg_;
  std_msgs::msg::Float64MultiArray joint_command_msg_;
  sensor_msgs::msg::Imu::SharedPtr imu_msg_;
  std::atomic_bool timer_ready_;

  std::atomic_bool joint_map_ready_;
  std::map<std::string, size_t> joint_name_index_map_;

  std::shared_ptr<std::thread> ros_loop_;
};

}  // namespace bitbot

#endif  // !GZ_NODE_HPP
