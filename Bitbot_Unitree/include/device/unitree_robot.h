#pragma once

#include "device/unitree_device.hpp"

namespace bitbot {

    class UnitreeMotherboard final : public UnitreeDevice {
    public:
        UnitreeMotherboard(pugi::xml_node const& device_node);
        ~UnitreeMotherboard();

    private:
        virtual void Input(const IOType& IO) final;
        virtual IOType Output() final;
        virtual void UpdateRuntimeData() final;

        unitree_hg::msg::dds_::MainBoardState_ Motherboard;
    };

}  // namespace bitbot

