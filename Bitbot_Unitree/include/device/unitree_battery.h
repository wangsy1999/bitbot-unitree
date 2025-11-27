#pragma once

#include "device/unitree_device.hpp"

namespace bitbot {

    class UnitreeBattery final : public UnitreeDevice {
    public:
        UnitreeBattery(pugi::xml_node const& device_node);
        ~UnitreeBattery();

    private:
        virtual void Input(const IOType& IO) final;
        virtual IOType Output() final;
        virtual void UpdateRuntimeData() final;

        unitree_hg::msg::dds_::BmsState_ battery;
    };

}  // namespace bitbot

