#pragma once

#include "variant"
#include "bitbot_kernel/device/device.hpp"

#include "unitree/idl/hg/MotorState_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/hg/IMUState_.hpp"
#include "unitree/idl/hg/MainBoardState_.hpp"
#include "unitree/idl/hg/BmsState_.hpp"
#include "unitree/idl/hg/MotorCmd_.hpp"
#include "unitree/idl/hg/LowCmd_.hpp"
#include "device/unitree_gamepadheader.h"



namespace bitbot {

  enum class UnitreeDeviceType : uint32_t {
    UNITREE_DEVICE = 1000,
    UNITREE_JOINT,
    UNITREE_IMU,
    UNITREE_GAMEPAD,
    UNITREE_BATTERY,
    UNITREE_MOTHERBOARD
  };


  using IOType = std::variant<unitree_hg::msg::dds_::MotorState_,
    unitree_hg::msg::dds_::IMUState_,
    unitree_hg::msg::dds_::MainBoardState_,
    unitree_hg::msg::dds_::BmsState_,
    unitree_hg::msg::dds_::MotorCmd_,
    REMOTE_DATA_RX>;

  class UnitreeDevice : public Device {
  public:
    UnitreeDevice(const pugi::xml_node& device_node) : Device(device_node) {}
    ~UnitreeDevice() = default;

    // Method to w/r unitree robot
    virtual void Input(const IOType& IO) = 0;
    virtual IOType Output() = 0;


    // Inherited from Device base calss
    virtual void UpdateRuntimeData() = 0;

  };
}  // namespace bitbot

