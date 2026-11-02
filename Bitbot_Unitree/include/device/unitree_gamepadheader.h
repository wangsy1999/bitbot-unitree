#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace bitbot
{
    // bytecode mapping for raw joystick data
   // 16b
    typedef union
    {
        struct
        {
            uint8_t R1 : 1;
            uint8_t L1 : 1;
            uint8_t start : 1;
            uint8_t select : 1;
            uint8_t R2 : 1;
            uint8_t L2 : 1;
            uint8_t F1 : 1;
            uint8_t F2 : 1;
            uint8_t A : 1;
            uint8_t B : 1;
            uint8_t X : 1;
            uint8_t Y : 1;
            uint8_t up : 1;
            uint8_t right : 1;
            uint8_t down : 1;
            uint8_t left : 1;
        } components;
        uint16_t value;
    } xKeySwitchUnion;

    // 40 Byte (now used 24B)
    typedef struct
    {
        uint8_t head[2];
        xKeySwitchUnion btn;
        float lx;
        float rx;
        float ry;
        float L2;
        float ly;

        uint8_t idle[16];
    } xRockerBtnDataStruct;

    typedef union
    {
        xRockerBtnDataStruct RF_RX;
        uint8_t buff[40];
    } REMOTE_DATA_RX;

    class Button
    {
    public:
        Button() {}

        void update(bool state)
        {
            on_press = state ? state != pressed : false;
            on_release = state ? false : state != pressed;
            pressed = state;
            on_toggle = on_press || on_release;
        }

        bool pressed = false;
        bool on_press = false;
        bool on_release = false;
        bool on_toggle = false;
    };

    class Joystick
    {
    public:
        Joystick() {}
        void update(float value)
        {
            this->returned_to_center = value == 0 && this->last_value != 0;
            this->moved = value != this->last_value;
            this->last_value = value;
            this->value = value;
        }

        bool moved = false;
        bool returned_to_center = false;
        float value = 0;
        float last_value = 0;
    };

    class Gamepad
    {
    public:
        using Clock = std::chrono::steady_clock;

        Gamepad(float smooth = 0.03, float dead_zone = 0.01, double axis_publish_hz = 50.0)
            :smooth(smooth), dead_zone(dead_zone)
        {
            if (!std::isfinite(smooth) || smooth <= 0 || smooth > 1 ||
                !std::isfinite(dead_zone) || dead_zone < 0 || dead_zone >= 1 ||
                !std::isfinite(axis_publish_hz) || axis_publish_hz <= 0)
            {
                throw std::invalid_argument("Invalid gamepad smoothing, dead zone or axis publication rate");
            }
            this->axis_publish_period_ = std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(1.0 / axis_publish_hz));
            if (this->axis_publish_period_ <= Clock::duration::zero())
            {
                throw std::invalid_argument("Gamepad axis publication rate exceeds clock resolution");
            }
        }

        void update(xRockerBtnDataStruct& key_data, Clock::time_point now = Clock::now())
        {
            this->publish_axes_ = now >= this->next_axis_publish_;
            if (this->publish_axes_)
            {
                this->next_axis_publish_ = now + this->axis_publish_period_;
            }

            lx = FilterAxis(lx, key_data.lx);
            rx = FilterAxis(rx, key_data.rx);
            ry = FilterAxis(ry, key_data.ry);
            l2 = FilterAxis(l2, key_data.L2);
            ly = FilterAxis(ly, key_data.ly);

            lx_joystick.update(lx);
            rx_joystick.update(rx);
            ry_joystick.update(ry);
            l2_joystick.update(l2);
            ly_joystick.update(ly);

            R1.update(key_data.btn.components.R1);
            L1.update(key_data.btn.components.L1);
            start.update(key_data.btn.components.start);
            select.update(key_data.btn.components.select);
            R2.update(key_data.btn.components.R2);
            L2.update(key_data.btn.components.L2);
            F1.update(key_data.btn.components.F1);
            F2.update(key_data.btn.components.F2);
            A.update(key_data.btn.components.A);
            B.update(key_data.btn.components.B);
            X.update(key_data.btn.components.X);
            Y.update(key_data.btn.components.Y);
            up.update(key_data.btn.components.up);
            right.update(key_data.btn.components.right);
            down.update(key_data.btn.components.down);
            left.update(key_data.btn.components.left);
        }

        bool ShouldPublishJoystick(const Joystick& joystick) const
        {
            return this->publish_axes_ || joystick.returned_to_center;
        }

        float lx = 0.;
        float rx = 0.;
        float ry = 0.;
        float l2 = 0.;
        float ly = 0.;

        Joystick lx_joystick;
        Joystick rx_joystick;
        Joystick ry_joystick;
        Joystick l2_joystick;
        Joystick ly_joystick;

        float smooth;
        float dead_zone;

        Button R1;
        Button L1;
        Button start;
        Button select;
        Button R2;
        Button L2;
        Button F1;
        Button F2;
        Button A;
        Button B;
        Button X;
        Button Y;
        Button up;
        Button right;
        Button down;
        Button left;

    private:
        float FilterAxis(float previous, float raw) const
        {
            // 回中和无效输入立即归零，不残留平滑衰减或非有限速度命令。
            if (!std::isfinite(raw) || std::fabs(raw) <= this->dead_zone)
            {
                return 0;
            }
            if (!std::isfinite(previous))
            {
                previous = 0;
            }
            return previous * (1 - this->smooth) + std::clamp(raw, -1.f, 1.f) * this->smooth;
        }

        Clock::duration axis_publish_period_{};
        Clock::time_point next_axis_publish_{};
        bool publish_axes_ = false;
    };
};
