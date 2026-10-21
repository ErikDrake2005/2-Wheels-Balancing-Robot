#ifndef IMUCONFIG_H
#define IMUCONFIG_H
#include <cstdint>
// Vận hành và diễn giải MPU6050
#define diaChiI2cMpu 0x68
#define tanSoI2cMpu_hz 100000
#define tanSoLayMauMpu_hz 200
#define phamViGiaTocMpu_g 4
#define phamViTocDoGocMpu_dps 500
#define cauHinhLocThongThapMpu 2
#define batNgatMpu true
#define mucNgatMpuCao true // true: active HIGH; false: active LOW
#define soMauHieuChinhMpu 500
// Ngưỡng thử ban đầu để phát hiện gyro biến động khi hiệu chuẩn.
// Đơn vị: độ/giây. Cần tinh chỉnh bằng dữ liệu cảm biến thực tế.
constexpr float doLechChuanConQuayToiDa_dps = 0.5f;
// Timeout cho giao dịch I2C, đơn vị millisecond.
// Không bao gồm mọi thời gian chờ khóa bus giữa các task.
constexpr uint16_t thoiGianChoI2c_ms = 2;
// Thời gian chờ cảm biến sau khi đánh thức.
constexpr uint32_t thoiGianKhoiDongMpu_ms = 100;
// Thời gian dự phòng ngoài thời gian thu mẫu hiệu chuẩn.
constexpr uint32_t duPhongThoiGianHieuChinh_ms = 1000;
#endif