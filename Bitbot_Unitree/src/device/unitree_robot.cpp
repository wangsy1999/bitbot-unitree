#include "device/unitree_robot.h"

namespace bitbot
{
    UnitreeMotherboard::UnitreeMotherboard(pugi::xml_node const& device_node)
        :UnitreeDevice(device_node)
    {
        basic_type_ = (uint32_t)BasicDeviceType::SENSOR;
        type_ = (uint32_t)UnitreeDeviceType::UNITREE_MOTHERBOARD;
        monitor_header_.headers = { "temperature" };
        monitor_data_.resize(monitor_header_.headers.size());
    }


    UnitreeMotherboard::~UnitreeMotherboard() {}

    void UnitreeMotherboard::Input(const IOType& IO)
    {
        this->Motherboard = std::get<unitree_hg::msg::dds_::MainBoardState_>(IO);
    }

    IOType UnitreeMotherboard::Output()
    {
        return IOType();
    }

    void UnitreeMotherboard::UpdateRuntimeData()
    {
        monitor_data_[0] = this->Motherboard.temperature()[0];
    }
};
