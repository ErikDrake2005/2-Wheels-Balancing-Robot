#ifndef SAFETYCONFIG_H
#define SAFETYCONFIG_H

#include <cstdint>
constexpr uint32_t tuoiImuToiDa_us = 20000;
constexpr uint32_t tuoiXungToiDa_us = 10000;
constexpr uint32_t tuoiLenhDeBat_us = 10000;
constexpr float gocNga_rad = 0.50f;
constexpr float gocChoPhepBat_rad = 0.10f;
constexpr float tocDoNghiengChoPhepBat_radps = 0.35f;
constexpr uint32_t chuKyTelemetry_ms = 200;
// Các ngưỡng khởi đầu cần đối chiếu log thực tế.
constexpr float gocNgaNgang_rad = 0.50f;
constexpr float gocNgangChoPhepBat_rad = 0.10f;
constexpr float giaTocHopLeToiThieu_mps2 = 2.0f;
constexpr float giaTocHopLeToiDa_mps2 = 30.0f;
constexpr uint32_t thoiGianChoLenhKhoiDong_us = 100000;

#endif
