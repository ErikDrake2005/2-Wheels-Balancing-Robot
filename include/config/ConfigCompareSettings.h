#ifndef CONFIG_COMPARE_SETTINGS_H
#define CONFIG_COMPARE_SETTINGS_H
#include <cstdint>

// Đặt file này vào include/config/ConfigCompareSettings.h khi tự dùng main mẫu.
// Giữ false cho đến khi đã xác nhận các mức điện và đo STEP.
constexpr bool choPhepChayMotorThu = true;

// Hai phép thử: xung thưa và xung liên tục, đơn vị steps/s.
constexpr float tanSoXungThuaThu_hz = 200.0f;
constexpr float tanSoXungLienTucThu_hz = 2000.0f;

// Chỉ cho main_step_signal_only_v2.cpp: thời gian phát STEP tối đa.
// EN vẫn luôn disable trong chương trình đo tín hiệu này.
constexpr uint32_t thoiGianPhatTinHieuThu_ms = 1000;

// Command Serial chỉ yêu cầu chạy tối đa khoảng này. Main tạo các command
// điều khiển mới mỗi 5 ms trong cửa sổ này, rồi disable khi hết hạn.
constexpr uint32_t thoiGianChayMotorThu_us = 1000000;
constexpr uint32_t chuKyLamMoiLenhThu_us = 5000;

constexpr uint32_t tuoiDuLieuMpuToiDa_us = 20000;
constexpr uint32_t chuKyBaoCao_ms = 1000;
constexpr uint32_t tocDoSerialThu_baud = 115200;
constexpr uint32_t thoiGianKhoiDongConsole_ms = 500;
constexpr uint32_t kichThuocNganXepTaskMpu_byte = 4096;
constexpr uint32_t kichThuocNganXepTaskMotor_byte = 4096;
constexpr unsigned uuTienTaskMpu = 3;
constexpr unsigned uuTienTaskMotor = 4;

// Phần nhập tay để tính RPM/m/s từ xung đầu vào MKS.
// Giá trị 0 là CHƯA BIẾT, main không chia cho 0 hoặc khẳng định RPM.
constexpr uint16_t soBuocDayDuBanhTraiMoiVong = 0;
constexpr uint16_t soBuocDayDuBanhPhaiMoiVong = 0;
constexpr uint16_t soViBuocBanhTrai = 0;
constexpr uint16_t soViBuocBanhPhai = 0;
constexpr float soVongDongCoBanhTraiTrenVongBanh = 0.0f;
constexpr float soVongDongCoBanhPhaiTrenVongBanh = 0.0f;
constexpr float banKinhBanhThu_m = 0.040f;

#endif
