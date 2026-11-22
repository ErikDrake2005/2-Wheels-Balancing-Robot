#pragma once
#include <cstdint>

// Profile phần cứng riêng cho MPU6050 I2C (environment: lolin_s3_mini_mpu6050):
// - SDA -> GPIO10, SCL -> GPIO11
// - AD0 -> GPIO7 (ESP32 xuất mức LOW cố định -> địa chỉ I2C = 0x68)
// - INT -> để trống (-1): đọc đồng bộ theo cờ DATA_RDY_INT (0x3A bit 0) bằng esp_timer bám pha.
// - LƯU Ý AN TOÀN MẠCH PHỤ XCL/XDA:
//   Theo datasheet InvenSense PS-MPU-6000A-00, XDA (AUX_DA) và XCL (AUX_CL) nên ĐỂ TRỐNG (NC)
//   khi không gắn từ kế phụ. KHÔNG nối cứng XCL lên 3V3 hay XDA xuống GND vì nếu I2C_BYPASS_EN
//   hoặc I2C_MST_EN bật sẽ gây chập bus SDA/SCL hoặc ngắn mạch 3V3-GND qua chân XCL.
//   Driver khóa cứng USER_CTRL (0x6A)=0x00 và INT_PIN_CFG (0x37)=0x00 (I2C_BYPASS_EN=0) để bảo vệ.
namespace mpu6050 {
constexpr int chanSda = 10;
constexpr int chanScl = 11;
constexpr int chanAd0 = 7;
constexpr int chanInt = -1; // Chân INT để trống theo sơ đồ phần cứng của người dùng.

constexpr bool mucAd0Cao = false;
constexpr uint8_t diaChiI2c = mucAd0Cao ? 0x69u : 0x68u;
constexpr uint32_t tanSoI2c_hz = 400000;   // Fast-Mode I2C 400 kHz (~360 us đọc burst 14 byte).
constexpr uint32_t tanSoLayMau_hz = 200;   // 1000 Hz / (1 + SMPLRT_DIV=4) = 200 Hz (chu kỳ 5 ms).

constexpr uint8_t phamViGiaToc_g = 4;      // ±4 g (8192 LSB/g).
constexpr uint16_t phamViGyro_dps = 500;   // ±500 deg/s (65.5 LSB/(deg/s)).
// DLPF_CFG = 3 trên MPU6050 (khác bảng ICM20602):
// Accel BW = 44 Hz (delay 4.9 ms), Gyro BW = 42 Hz (delay 4.8 ms), Fs nội = 1 kHz.
// Hai kênh có độ trễ phần cứng khớp nhau (~0.1 ms) giúp bù gia tốc tịnh tiến chính xác.
constexpr uint8_t maDlpf = 3;
constexpr uint8_t maNhanDang = 0x68;       // WHO_AM_I chuẩn của MPU6050 tại thanh ghi 0x75.

constexpr uint16_t thoiGianChoI2c_ms = 5;
constexpr uint32_t thoiGianOnDinhReset_ms = 100;
constexpr uint32_t timeoutReset_ms = 100;
constexpr uint32_t timeoutBoot_ms = 1500;
constexpr uint16_t soMauBoot = 32;

// Định thời bám pha mẫu mới khi chân INT để trống (chu kỳ danh nghĩa 5000 us):
// Ngủ 4300 us sau mẫu hợp lệ trước, sau đó kiểm tra nhanh thanh ghi 0x3A mỗi 250 us
// để bắt sườn DATA_RDY_INT với độ trễ <= 250 us mà không mất mẫu khi clock MEMS lệch ±2%.
constexpr uint32_t chuKyNguPoll_us = 4300;
constexpr uint32_t chuKyThuLaiPoll_us = 250;
constexpr uint32_t chuKyLoiPoll_us = 1000;

constexpr uint8_t heSoChiaMau = static_cast<uint8_t>(1000u / tanSoLayMau_hz - 1u);
constexpr uint8_t maGyroFs = phamViGyro_dps == 250 ? 0x00u :
    phamViGyro_dps == 500 ? 0x08u : phamViGyro_dps == 1000 ? 0x10u : 0x18u;
constexpr uint8_t maAccelFs = phamViGiaToc_g == 2 ? 0x00u :
    phamViGiaToc_g == 4 ? 0x08u : phamViGiaToc_g == 8 ? 0x10u : 0x18u;
constexpr float lsbMoiG = phamViGiaToc_g == 2 ? 16384.0f :
    phamViGiaToc_g == 4 ? 8192.0f : phamViGiaToc_g == 8 ? 4096.0f : 2048.0f;
constexpr float lsbMoiDps = phamViGyro_dps == 250 ? 131.0f :
    phamViGyro_dps == 500 ? 65.5f : phamViGyro_dps == 1000 ? 32.8f : 16.4f;

static_assert(diaChiI2c == 0x68u || diaChiI2c == 0x69u, "Dia chi I2C MPU6050 phai la 0x68 hoac 0x69");
static_assert(tanSoI2c_hz == 100000u || tanSoI2c_hz == 400000u, "I2C MPU6050 ho tro 100 kHz hoac 400 kHz");
static_assert(maDlpf >= 1u && maDlpf <= 6u, "MPU6050 can DLPF 1..6 (Fs=1kHz) de chia mau 200Hz va chong alias");
static_assert(tanSoLayMau_hz > 0u && tanSoLayMau_hz <= 1000u && 1000u % tanSoLayMau_hz == 0u,
    "Tan so lay mau MPU6050 phai la uoc cua 1000 Hz");
static_assert(phamViGiaToc_g == 2u || phamViGiaToc_g == 4u || phamViGiaToc_g == 8u || phamViGiaToc_g == 16u,
    "Dai gia toc MPU6050 khong hop le");
static_assert(phamViGyro_dps == 250u || phamViGyro_dps == 500u || phamViGyro_dps == 1000u || phamViGyro_dps == 2000u,
    "Dai gyro MPU6050 khong hop le");
}
