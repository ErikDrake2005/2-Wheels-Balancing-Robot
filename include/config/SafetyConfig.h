#ifndef SAFETYCONFIG_H
#define SAFETYCONFIG_H

#include <cstdint>
constexpr uint32_t tuoiImuToiDa_us = 20000;
// Tuổi snapshot xung tại lúc kiểm tra (mục tiêu MKS 0,5 ms; TMC_MT 1 ms).
constexpr uint32_t tuoiXungToiDa_us = 20000;
// Chặt khi BẬT: lệnh 0 xác nhận phiên phải thật mới.
constexpr uint32_t tuoiLenhDeBat_us = 10000;
// Tối ưu 10/11/2026 (H1): khi ĐANG CHẠY cho phép mất 1–2 mẫu IMU (5 ms/mẫu) mà
// không dừng khẩn. Vẫn nhỏ hơn hạn lệnh 20 ms của driver và watchdog GPTimer.
constexpr uint32_t tuoiLenhKhiChay_us = 16000;
// Khoảng cách tối đa giữa hai khung xung liên tiếp ControlTask dùng để tính vận tốc.
// Tách khỏi tuoiXungToiDa_us: mất một mẫu IMU làm khoảng cách ~10 ms là bình thường.
constexpr uint32_t khoangCachKhungXungToiDa_us = 25000;
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
static_assert(tuoiLenhKhiChay_us >= tuoiLenhDeBat_us && tuoiLenhKhiChay_us < 20000,
              "Tuoi lenh khi chay phai nam giua nguong bat va han lenh driver");
static_assert(khoangCachKhungXungToiDa_us > 2 * 5000, "Phai chiu duoc mat it nhat mot mau IMU");

#endif
