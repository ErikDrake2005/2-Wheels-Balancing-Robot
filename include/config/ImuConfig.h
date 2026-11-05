#ifndef IMUCONFIG_H
#define IMUCONFIG_H
#include <cstdint>
// ICM20602 đọc raw accel/gyro + DLPF phần cứng; fusion thực hiện trên ESP32, không DMP.
// Các macro Mpu là cấu hình tương thích bench cũ; runtime dùng các tên Icm bên dưới.
#define diaChiI2cMpu 0x68
#define tanSoI2cMpu_hz 100000
#define tanSoLayMauMpu_hz 200
#define phamViGiaTocMpu_g 4
#define phamViTocDoGocMpu_dps 500
#define cauHinhLocThongThapMpu 3 // ICM20602 gyro BW 41 Hz; không dùng bảng MPU6050.
#define batNgatMpu true
#define mucNgatMpuCao true // true: active HIGH; false: active LOW
#define soMauHieuChinhMpu 500
constexpr uint8_t diaChiI2cIcm = diaChiI2cMpu;
constexpr uint32_t tanSoI2cIcm_hz = tanSoI2cMpu_hz;
constexpr uint32_t tanSoLayMauIcm_hz = tanSoLayMauMpu_hz;
constexpr uint8_t phamViGiaTocIcm_g = phamViGiaTocMpu_g;
constexpr uint16_t phamViTocDoGocIcm_dps = phamViTocDoGocMpu_dps;
constexpr uint8_t locConQuayIcm = cauHinhLocThongThapMpu;
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
constexpr uint8_t locGiaTocIcm = 3; // ICM20602 accel BW 44,8 Hz tại fs đọc 200 Hz.
// Hướng lắp nay dùng phép quay 3D trong ImuMountConfig.h, không chọn riêng X/Y.
constexpr float hangSoThoiGianLocGoc_s = 0.4f;
constexpr float saiLechGiaTocChoPhep_mps2 = 2.5f;
// Giảm tin gia tốc khi góc accel khác nhiều dự đoán gyro (gia tốc tịnh tiến).
constexpr float saiLechGocBatDauGiamTin_rad = 0.08f;
constexpr float saiLechGocMatTin_rad = 0.25f;
constexpr float thoiGianMatTinGiaTocToiDa_s = 0.5f;
static_assert(saiLechGocMatTin_rad > saiLechGocBatDauGiamTin_rad &&
              thoiGianMatTinGiaTocToiDa_s > 0, "Mien tin cay gia toc khong hop le");
static_assert(tanSoLayMauMpu_hz > 0 && tanSoLayMauMpu_hz <= 1000 &&
              1000 % tanSoLayMauMpu_hz == 0, "Tan so IMU phai chia het 1000 Hz");
static_assert(hangSoThoiGianLocGoc_s > 0, "Hang so loc goc phai duong");
#endif
