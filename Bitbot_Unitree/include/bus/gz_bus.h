#ifndef GZ_BUS_H
#define GZ_BUS_H

#include "device/gz_device.hpp"
#include "device/gz_imu.h"
#include "device/gz_joint.h"
#include "bitbot_kernel/bus/bus_manager.hpp"

#include "unitree_hg/msg/motor_state.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/main_board_state.hpp"
#include "unitree_hg/msg/bms_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "thread"
#include "mutex"

#include "pugixml.hpp"

namespace bitbot {
  class GzBus : public BusManagerTpl<GzBus, GzDevice> {
  public:
    GzBus();
    ~GzBus();

    void WriteBus();
    void ReadBus();
    void RegisterDevices();
    void Init(pugi::xml_node& bitbot_node);
    void PowerOn();
    void PowerOff();
    bool isSystemReady() { return received.load(); }

  private:
    void LowStateCallback(const unitree_hg::msg::LowState::SharedPtr msg);
    void AlterImuCallback(const unitree_hg::msg::IMUState::SharedPtr msg);
    void MainBoardStateCallback(const unitree_hg::msg::MainBoardState::SharedPtr msg);
    void BmsStateCallback(const unitree_hg::msg::BmsState::SharedPtr msg);

  private:
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr low_command_publisher_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr low_state_subscriber_;
    rclcpp::Subscription<unitree_hg::msg::IMUState>::SharedPtr alter_imu_subscriber_;
    rclcpp::Subscription<unitree_hg::msg::MainBoardState>::SharedPtr main_board_state_subscriber_;
    rclcpp::Subscription<unitree_hg::msg::BmsState>::SharedPtr bms_state_subscriber_;

    rclcpp::Node::SharedPtr node_;

    //注意读写时加锁解锁
    std::mutex motor_state_lock_;
    std::array<unitree_hg::msg::MotorState, 35> motor_states_;
    std::mutex motor_cmd_lock_;
    std::array<unitree_hg::msg::MotorCmd, 35> motor_cmds_;
    std::mutex imu_state_lock_;
    std::array<unitree_hg::msg::IMUState, 2> imu_states_;
    std::mutex main_board_state_lock_;
    unitree_hg::msg::MainBoardState main_board_state_msg_;
    std::mutex bms_state_lock_;
    unitree_hg::msg::BmsState bms_state_msg_;

    // ros bus node spin thread
    std::thread spin_thread_;
    std::atomic<bool> received = false;

    //device list
    std::vector<GzDevice*> joint_devices_;
    std::vector<GzDevice*> imu_devices_;
    // GzDevice motherboard_device_;
    // GzDevice battery_device_;
    //TODO: add motherboard and battery device

  private: //LowCmd config
    uint8_t mode_pr_;
    uint8_t mode_machine_;

  };
}  // namespace bitbot

#endif  // !GZ_BUS_H
