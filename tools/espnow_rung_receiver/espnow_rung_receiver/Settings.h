#pragma once
#include <cstdint>

namespace PassiveReceiver {
// Robot ESP32-S3 STA MAC, not the phone/SoftAP MAC. Read it from robot WebUI.
constexpr uint8_t RobotMac[6] = {0xAC, 0x27, 0x6E, 0xD0, 0x3A, 0x24};
constexpr uint8_t RadioChannel = 1;
constexpr uint32_t SerialBaud = 921600;
}
