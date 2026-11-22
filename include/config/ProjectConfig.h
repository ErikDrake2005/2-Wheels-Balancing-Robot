#ifndef PROJECTCONFIG_H
#define PROJECTCONFIG_H

#include "ImuProfile.h"

// In cả ở READY và telemetry để đối chiếu đúng bản thực sự đã nạp.
constexpr const char* phienBanFirmware =
    dungLsm6dsv ? "lsm-smooth-f-20261122" : (dungMpu6050 ? "mpu6050-siso-20261121" : "icm-profile-20261117");

#endif
