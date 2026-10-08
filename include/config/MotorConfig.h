#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H
#include <cstdint>

constexpr bool MOTOR_EN_ACTIVE_HIGH = false;
constexpr uint32_t MCPWM_TIMER_RESOLUTION_HZ = 1000000;
constexpr uint32_t MCPWM_INITIAL_PERIOD_US = 10000;

#endif