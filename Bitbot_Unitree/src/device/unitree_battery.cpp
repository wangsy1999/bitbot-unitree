#include "device/unitree_battery.h"

namespace bitbot
{
    UnitreeBattery::UnitreeBattery(pugi::xml_node const& device_node)
        :UnitreeDevice(device_node)
    {
        basic_type_ = (uint32_t)BasicDeviceType::SENSOR;
        type_ = (uint32_t)UnitreeDeviceType::UNITREE_BATTERY;
        monitor_header_.headers = { "current","voltage","battery_life","temperature","cycle" };
        monitor_data_.resize(monitor_header_.headers.size());
    }


    UnitreeBattery::~UnitreeBattery() {}

    void UnitreeBattery::Input(const IOType& IO)
    {
        this->battery = std::get<unitree_hg::msg::dds_::BmsState_>(IO);
    }

    IOType UnitreeBattery::Output()
    {
        return IOType();
    }

    void UnitreeBattery::UpdateRuntimeData()
    {
        monitor_data_[0] = this->battery.current();
        monitor_data_[1] = (this->battery.bmsvoltage()[0] + this->battery.bmsvoltage()[1]) / 2.0;
        monitor_data_[2] = this->battery.soc();

        float avg_tmp = 0;
        for (size_t i = 0; i < 4;i++)
        {
            avg_tmp += this->battery.temperature()[i];
        }
        avg_tmp /= 4;
        monitor_data_[3] = avg_tmp;
        monitor_data_[4] = this->battery.cycle();
    }
};
