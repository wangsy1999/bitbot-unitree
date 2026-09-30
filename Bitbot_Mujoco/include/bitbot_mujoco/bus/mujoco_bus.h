#pragma once

#include "bitbot_kernel/bus/bus_manager.hpp"
#include "bitbot_mujoco/device/mujoco_device.hpp"

namespace bitbot{

  class MujocoBus : public BusManagerTpl<MujocoBus, MujocoDevice>
  {
  public:
    MujocoBus(/* args */);
    ~MujocoBus();

    void RegisterDevices();

    void SetMujocoParam(const mjModel *m, mjData *d);

    void ReadBus();
    void WriteBus();

    void UpdateDevices();

    const mjModel* GetMujocoModel() const;
    mjData * GetMujocoData() const;

  private:
    const mjModel *mj_m_ = nullptr;
    mjData *mj_d_ = nullptr;
  };


}
