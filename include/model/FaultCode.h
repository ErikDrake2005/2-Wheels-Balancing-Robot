#pragma once
#include <cstdint>
enum class FaultCode : uint8_t {
    None, ImuInvalid, ImuStale, PulseInvalid, PulseStale, Fallen,
    InvalidControl, MotorFault, EmergencyStop, ArmTimeout
};
