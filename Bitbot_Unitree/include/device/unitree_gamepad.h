#pragma once

#include "device/unitree_device.hpp"
#include "device/unitree_gamepadheader.h"
#include "unordered_map"

namespace bitbot {
    class KernelInterface;
    class UnitreeGamepad final : public UnitreeDevice {
    public:
        UnitreeGamepad(pugi::xml_node const& device_node);
        ~UnitreeGamepad();

        void init(KernelInterface* interface, const std::unordered_map<std::string, std::string>& map)
        {
            this->KeyEventMap = map;
            this->kernel_interface = interface;
        }

        void updateEventIDMap(const std::unordered_map<std::string, EventId>& map)
        {
            this->EventMap = map;
        }

    private:
        virtual void Input(const IOType& IO) final;
        virtual IOType Output() final;
        virtual void UpdateRuntimeData() final;

        void ProcessButtonEvent(const Button& button, const std::string& key);
        void ProcessJoystickEvent(float joystick, const std::string& key);

    private:
        Gamepad* gamepad_;
        std::unordered_map<std::string, EventId> EventMap;
        std::unordered_map<std::string, std::string> KeyEventMap;
        KernelInterface* kernel_interface;
    };

}  // namespace bitbot

