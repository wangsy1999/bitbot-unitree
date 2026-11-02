#include "device/unitree_gamepadheader.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void Check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void TestAxis(float bitbot::xRockerBtnDataStruct::*input,
              bitbot::Joystick bitbot::Gamepad::*axis)
{
    bitbot::Gamepad pad(0.03f, 0.01f);
    bitbot::xRockerBtnDataStruct packet{};
    float command = 0.0f;
    int events = 0;
    auto step = [&] {
        pad.update(packet);
        const auto& stick = pad.*axis;
        if (stick.moved) { command = stick.value; ++events; }
    };
    // Normal deflection must cross the event threshold despite low-pass filtering.
    packet.*input = 1.0f;
    for (int frame = 0; frame < 300; ++frame) step();
    Check(events > 0 && command > 0.94f, "Filtered deflection did not produce a command");
    packet.*input = -1.0f;
    for (int frame = 0; frame < 300; ++frame) step();
    Check(command < -0.94f, "Direction reversal did not produce a command");
    // Release must deliver an exact zero immediately, even with a filter tail.
    const int before_release = events;
    packet.*input = 0.0f;
    step();
    Check(command == 0.0f && events == before_release + 1, "Release did not emit zero");
    // A later keyboard command must survive an idle remote and dead-zone noise.
    command = 0.3f;
    const int before_idle = events;
    for (int frame = 0; frame < 300; ++frame) {
        packet.*input = frame % 2 ? 0.009f : -0.009f;
        step();
    }
    Check(events == before_idle && command == 0.3f, "Idle stick overwrote keyboard command");
    // Gradual movement must accumulate rather than disappear between samples.
    for (int frame = 0; frame < 500; ++frame) {
        packet.*input = float(frame) / 1000.0f;
        step();
    }
    Check(command > 0.40f, "Slow joystick movement was lost");
    // Tiny movement still gets a release event; held center remains silent.
    packet.*input = 0.0f;
    step();
    Check(command == 0.0f, "Slow movement release did not clear command");
    packet.*input = 0.02f;
    step();
    packet.*input = 0.0f;
    step();
    Check((pad.*axis).moved && (pad.*axis).value == 0.0f, "Sub-threshold release did not emit zero");
    packet.*input = std::numeric_limits<float>::quiet_NaN();
    step();
    Check(std::isfinite((pad.*axis).value), "Invalid input poisoned joystick filter");
}

int main()
{
    try {
        TestAxis(&bitbot::xRockerBtnDataStruct::lx, &bitbot::Gamepad::lx_joystick);
        TestAxis(&bitbot::xRockerBtnDataStruct::rx, &bitbot::Gamepad::rx_joystick);
        TestAxis(&bitbot::xRockerBtnDataStruct::ry, &bitbot::Gamepad::ry_joystick);
        TestAxis(&bitbot::xRockerBtnDataStruct::ly, &bitbot::Gamepad::ly_joystick);
        TestAxis(&bitbot::xRockerBtnDataStruct::L2, &bitbot::Gamepad::l2_joystick);
        bitbot::Gamepad pad;
        bitbot::xRockerBtnDataStruct packet{};
        packet.btn.components.A = 1;
        pad.update(packet);
        Check(pad.A.on_press && pad.A.on_toggle, "Button press changed");
        packet.btn.components.A = 0;
        pad.update(packet);
        Check(pad.A.on_release && pad.A.on_toggle, "Button release changed");
        std::cout << "PASS: five axes, filtered/slow motion, reversal, exact release, keyboard coexistence, button edges\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
