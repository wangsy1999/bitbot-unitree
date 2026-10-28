#include "bus/motor_crc_hg.h"
#include <bit>
#include <cstddef>


// Serialize the HG command explicitly. Reading a float-containing struct through
// uint32_t* violates strict aliasing and can produce wrong CRCs with optimization.
void get_crc(unitree_hg::msg::dds_::LowCmd_& msg) {
    static_assert(sizeof(float) == sizeof(uint32_t));
    constexpr size_t motor_words = 7;
    constexpr size_t reserve_start = 1 + 35 * motor_words;
    std::array<uint32_t, reserve_start + 4> words{};
    words[0] = uint32_t{msg.mode_pr()} | (uint32_t{msg.mode_machine()} << 8);

    for (size_t i = 0; i < msg.motor_cmd().size(); ++i) {
        const auto& motor = msg.motor_cmd()[i];
        const size_t offset = 1 + i * motor_words;
        words[offset] = motor.mode(); // Upper three bytes are protocol padding.
        words[offset + 1] = std::bit_cast<uint32_t>(motor.q());
        words[offset + 2] = std::bit_cast<uint32_t>(motor.dq());
        words[offset + 3] = std::bit_cast<uint32_t>(motor.tau());
        words[offset + 4] = std::bit_cast<uint32_t>(motor.kp());
        words[offset + 5] = std::bit_cast<uint32_t>(motor.kd());
        words[offset + 6] = motor.reserve();
    }
    for (size_t i = 0; i < msg.reserve().size(); ++i) {
        words[reserve_start + i] = msg.reserve()[i];
    }
    msg.crc() = crc32_core(words.data(), static_cast<uint32_t>(words.size()));
}

uint32_t crc32_core(const uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = uint32_t{1} << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit) CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}