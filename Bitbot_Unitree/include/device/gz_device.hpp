#ifndef GZ_DEVICE_HPP
#define GZ_DEVICE_HPP

#include "variant"
#include "bitbot_kernel/device/device.hpp"
#include "unitree_hg/msg/motor_state.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/main_board_state.hpp"
#include "unitree_hg/msg/bms_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"

namespace bitbot {

  enum class GzDeviceType : uint32_t {
    GZ_DEVICE = 1000,
    GZ_JOINT,
    GZ_IMU,
    GZ_JOYSTICK,
    GZ_BATTERY,
    GZ_MOTHERBOARD
  };

  using IOType = std::variant<unitree_hg::msg::MotorState,
    unitree_hg::msg::IMUState,
    unitree_hg::msg::MainBoardState,
    unitree_hg::msg::BmsState,
    unitree_hg::msg::MotorCmd>;

  class GzDevice : public Device {
  public:
    GzDevice(const pugi::xml_node& device_node) : Device(device_node) {}
    ~GzDevice() = default;

    // Method to w/r gz simulation
    virtual void Input(const IOType& IO) = 0;
    virtual IOType Output() = 0;


    // Inherited from Device base calss
    virtual void UpdateRuntimeData() = 0;

  private:
  };
}  // namespace bitbot

#endif  // !GZ_DEVICE_HPP
