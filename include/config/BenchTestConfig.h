#ifndef BENCH_TEST_CONFIG_H
#define BENCH_TEST_CONFIG_H

#include <cstdint>

// Giai đoạn đầu đo tín hiệu STEP khi MKS bị disable.
// Chỉ đổi thành true sau khi đã xác nhận:
// - En = L trên cả hai MKS, phù hợp mucBatMotorCao = false;
// - STEP/DIR/EN có mức logic tương thích và nối đúng;
// - hai bánh được nhấc khỏi mặt đất, có thể ngắt nguồn motor.
constexpr bool choPhepBatMotorThu = false;

// Giá trị THỬ cho độ rộng tín hiệu, chưa phải giới hạn đã xác nhận của MKS.
// Đối chiếu manual đúng phiên bản trước khi bật công suất motor.
constexpr uint32_t thoiGianMucCaoStepThu_us = 10;
constexpr uint32_t thoiGianMucThapStepThuToiThieu_us = 10;

// Khoảng chờ sau khi disable/stop và trước/sau đổi DIR trong phép thử.
// Phép thử luôn disable trước khi đảo chiều; không dùng trình tự này
// làm thuật toán đảo chiều khi xe đang cân bằng.
constexpr uint32_t thoiGianBaoVeDirThu_us = 100;

// Đây là xung/giây, KHÔNG phải RPM hay tần số đọc MPU.
constexpr float tanSoStepThu_hz = 200.0f;
constexpr float tanSoStepThuThuHai_hz = 400.0f;

// Mỗi lệnh chạy có hiệu lực tối đa 1 giây; lệnh mới gia hạn phép thử.
constexpr uint32_t thoiGianChayMotorThu_ms = 1000;

// Giá trị thử ở MPU 200 Hz: 20 ms tương đương khoảng 4 chu kỳ mẫu.
// Lỗi I2C hoặc mẫu lỗi sẽ khóa phép thử motor tới lần reset tiếp theo.
constexpr uint32_t thoiGianChoMpuThu_ms = 20;
constexpr uint32_t chuKyKiemTraMotor_ms = 2;
constexpr uint32_t chuKyBaoCaoThu_ms = 1000;

#endif
