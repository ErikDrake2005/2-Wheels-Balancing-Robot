#ifndef IMUCONFIG_H
#define IMUCONFIG_H
#include <cstdint>
// ICM20602 đọc raw accel/gyro + DLPF phần cứng; fusion thực hiện trên ESP32, không DMP.
// Các macro Mpu là cấu hình tương thích bench cũ; runtime dùng các tên Icm bên dưới.
#define diaChiI2cMpu 0x68
// Tối ưu 10/11/2026 (L2): nâng I2C từ 100 kHz lên 400 kHz (Fast Mode theo DS-000176).
// Burst INT_STATUS + 14 byte giảm từ ~1,62 ms xuống ~0,41 ms, giảm trễ vòng góc
// và giảm nguy cơ mẫu bị loại do QuaChuKyMau.
#define tanSoI2cMpu_hz 400000
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
// Ngắt giữ mức đến khi đọc INT_STATUS; không dùng xung 50 us dễ bỏ lỡ.
// Bit4=0: chỉ ACK khi đọc status, không xóa ngắt vì đọc thanh ghi chẩn đoán khác.
constexpr uint8_t cauHinhChanNgatIcm = (mucNgatMpuCao ? 0x00u : 0x80u) | 0x20u;
// Ngưỡng phát hiện gyro biến động khi hiệu chuẩn (nới từ 0.5 lên 1.05 dps để không bị lặp lại 70+ lần khi giữ tay).
constexpr float doLechChuanConQuayToiDa_dps = 1.05f;
// Chờ một khoảng dữ liệu ổn định; rung chỉ hủy khoảng đo, không kết thúc ngay calib.
constexpr uint32_t thoiGianChoOnDinhCalib_ms = 30000;
constexpr uint32_t thoiGianLangSauNhanNut_ms = 500;
constexpr uint32_t thoiGianMatMauCalib_ms = 500;
constexpr uint32_t khoangMauCalibToiDa_ms = 50;
constexpr float saiLechNormGiaTocCalib_mps2 = 0.5f;
constexpr float phuongSaiGiaTocCalibToiDa = 0.04f; // Tổng phương sai XYZ, (m/s²)².
constexpr float tocDoGyroTinhCalibToiDa_radps = 0.10f; // Sau trừ bias (~5.7 dps).
constexpr float gioiHanGyroThoCalib_dps = 10.0f; // Trước trừ bias; không chứng minh đứng yên.
constexpr uint16_t soMauKiemTraKhoiDongIcm = 32;
constexpr uint32_t thoiGianKiemTraKhoiDongIcm_ms = 1500;
// Timeout cho giao dịch I2C, đơn vị millisecond.
// Không bao gồm mọi thời gian chờ khóa bus giữa các task.
// Burst status + 14 byte ở 400 kHz cần ~0,41 ms trên dây (ở 100 kHz cần ~1,62 ms).
// Đây là hạn giao dịch, KHÔNG nới chốt mẫu <5 ms hoặc Safety 20 ms.
constexpr uint16_t thoiGianChoI2c_ms = 5;
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
