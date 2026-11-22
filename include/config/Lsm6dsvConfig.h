#pragma once
#include <cstdint>

// Profile riêng cho LSM6DSV / LSM6DSV16BX SPI; không đổi thanh ghi/dải đo của ICM20602.
namespace lsm6dsv {
constexpr int chanSck = 11, chanMosi = 10, chanMiso = 9, chanCs = 8, chanInt = 7;
constexpr uint32_t tanSoSpi_hz = 4000000; // 4 MHz SPI (chip hỗ trợ tối đa 10 MHz), giảm thời gian đọc xuống ~110 us.
constexpr uint32_t tanSoLayMau_hz = 200;  // HAODR nhóm 12.5/25/50/100/200/400/800 Hz (LSM6DSV/16X).
constexpr uint32_t tanSoLayMau16Bx_hz =
    tanSoLayMau_hz == 25 ? 30 : tanSoLayMau_hz == 50 ? 60 :
    tanSoLayMau_hz == 100 ? 120 : tanSoLayMau_hz == 200 ? 240 :
    tanSoLayMau_hz == 400 ? 480 : 960; // Chế độ High-Performance 240 Hz cho LSM6DSV16B/16BX (WHO=0x71).
constexpr uint8_t phamViGiaToc_g = 4;
constexpr uint16_t phamViGyro_dps = 500;
constexpr uint8_t locGyroLpf1 = 5; // VERY_STRONG; mã filter ST, không phải mã DLPF ICM.
constexpr bool batLocGyroLpf1 = true;
constexpr uint8_t locGiaTocLpf2 = 3; // LSM6DSV_XL_MEDIUM = ODR/45 (~5.3 Hz ở 240 Hz), triệt rung động cơ phần cứng cho accelerometer.
constexpr bool batLocGiaTocLpf2 = true;
constexpr uint32_t thoiGianOnDinh_ms = 200;
constexpr uint32_t timeoutReset_ms = 100;
constexpr uint32_t timeoutBoot_ms = 1500;
constexpr uint16_t soMauBoot = 32;
constexpr uint32_t timeoutGiaoDich_us = 3800; // < 4166 us (240 Hz), tránh timeout giả khi bị task khác xen ngang lúc đọc SPI 1 MHz.
constexpr bool mucNgatCao = true;
constexpr uint8_t maNhanDang = 0x70;
constexpr uint8_t maNhanDang16Bx = 0x71; // LSM6DSV16B / LSM6DSV16BX / ISM330BX

constexpr uint8_t maOdr = tanSoLayMau_hz == 25 ? 4 : tanSoLayMau_hz == 50 ? 5 :
    tanSoLayMau_hz == 100 ? 6 : tanSoLayMau_hz == 200 ? 7 : tanSoLayMau_hz == 400 ? 8 : 9;
constexpr uint8_t maGiaToc = phamViGiaToc_g == 2 ? 0 : phamViGiaToc_g == 4 ? 1 :
    phamViGiaToc_g == 8 ? 2 : 3;
constexpr uint8_t maGyro = phamViGyro_dps == 125 ? 0 : phamViGyro_dps == 250 ? 1 :
    phamViGyro_dps == 500 ? 2 : phamViGyro_dps == 1000 ? 3 : phamViGyro_dps == 2000 ? 4 : 12;
constexpr float giaToc_mgLsb = phamViGiaToc_g == 2 ? .061f : phamViGiaToc_g == 4 ? .122f :
    phamViGiaToc_g == 8 ? .244f : .488f;
constexpr float gyro_dpsLsb = phamViGyro_dps == 125 ? .004375f : phamViGyro_dps == 250 ? .00875f :
    phamViGyro_dps == 500 ? .0175f : phamViGyro_dps == 1000 ? .035f : phamViGyro_dps == 2000 ? .07f : .14f;
static_assert(tanSoSpi_hz > 0 && tanSoSpi_hz <= 10000000, "SPI LSM6DSV toi da 10 MHz");
static_assert(tanSoLayMau_hz == 25 || tanSoLayMau_hz == 50 || tanSoLayMau_hz == 100 ||
    tanSoLayMau_hz == 200 || tanSoLayMau_hz == 400 || tanSoLayMau_hz == 800, "ODR HAODR khong ho tro");
static_assert(phamViGiaToc_g == 2 || phamViGiaToc_g == 4 || phamViGiaToc_g == 8 || phamViGiaToc_g == 16,
    "Dai gia toc LSM6DSV khong hop le");
static_assert(phamViGyro_dps == 125 || phamViGyro_dps == 250 || phamViGyro_dps == 500 ||
    phamViGyro_dps == 1000 || phamViGyro_dps == 2000 || phamViGyro_dps == 4000, "Dai gyro khong hop le");
static_assert(locGyroLpf1 <= 7 && locGiaTocLpf2 <= 7, "Ma bo loc LSM6DSV phai 0..7");
}
