#include "bitbot_mujoco/bus/mujoco_bus.h"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/device/mujoco_force_sensor.h"
#include "bitbot_mujoco/device/mujoco_imu.h"
#include "bitbot_mujoco/device/mujoco_framepos.h"
#include "bitbot_mujoco/device/mujoco_framelinvel.h"

namespace bitbot{
  MujocoBus::MujocoBus(/* args */)
  {
  }

  MujocoBus::~MujocoBus()
  {
  }

  void MujocoBus::SetMujocoParam(const mjModel *m, mjData *d)
  {
    mj_m_ = m;
    mj_d_ = d;
  }

  void MujocoBus::RegisterDevices()
  {
    static DeviceRegistrar<MujocoDevice, MujocoJoint> mujocojoint((uint32_t)MujocoDeviceType::MUJOCO_JOINT, "MujocoJoint");
    static DeviceRegistrar<MujocoDevice, MujocoForceSensor> mujocoforcesensor((uint32_t)MujocoDeviceType::MUJOCO_FORCE_SENSOR, "MujocoForceSensor");
    static DeviceRegistrar<MujocoDevice, MujocoImu> mujocoimu((uint32_t)MujocoDeviceType::MUJOCO_IMU, "MujocoImu");
    static DeviceRegistrar<MujocoDevice, MujocoFramepos> mujocoframepos((uint32_t)MujocoDeviceType::MUJOCO_FRAMEPOS, "MujocoFramepos");
    static DeviceRegistrar<MujocoDevice, MujocoFramelinvel> mujocoframelinvel((uint32_t)MujocoDeviceType::MUJOCO_FRAMELINVEL, "MujocoFramelinvel");
  }

  void MujocoBus::ReadBus()
  {
    for(auto& device:devices_)
    {
      device->Input(this->mj_m_, this->mj_d_);
    }
  }

  void MujocoBus::WriteBus()
  {
    for(auto& device:devices_)
    {
      device->Output(this->mj_m_, this->mj_d_);
    }
  }

  void MujocoBus::UpdateDevices()
  {
    if(mj_m_ == nullptr)
      return;

    for(auto& device:devices_)
    {
      device->UpdateModel(mj_m_, mj_d_);
    }

  }

  const mjModel* MujocoBus::GetMujocoModel() const
  {
    return mj_m_;
  }

  mjData * MujocoBus::GetMujocoData() const
  {
    return mj_d_;
  }
}
