#pragma once

// Bộ tham số điều khiển, khử rung động cơ và bộ điều khiển SISO cho profile MPU6050.
// Lưu độc lập trong namespace NVS "xe_mpu6050", không ảnh hưởng ICM20602 hoặc LSM6DSV.
namespace mpu6050Tune {
// Hệ số vòng góc & vòng ngoài (giảm nhẹ Kd so với ICM để tránh khuếch đại nhiễu MEMS của MPU6050,
// kết hợp lọc Biquad Butterworth bậc 2 + Notch giúp cắt sạch tiếng gằn động cơ bước).
constexpr float kp = 32.0f, ki = 0.0f, kd = 3.1f;
constexpr float kpv = 0.15f, kiv = 0.06f;
constexpr float kpMo = 29.0f, kiMo = 0.0f, kdMo = 2.9f;
constexpr float kpvMo = 0.15f, kivMo = 0.06f;
constexpr float gocCanBang_rad = 0.0f;

// Chuỗi lọc số SISO chống rung động cơ trên nhánh vi phân tốc độ góc (fs = 200 Hz):
// 1) Bộ lọc khía Biquad Notch triệt rung cộng hưởng khung/vi bước (đặt 0.0f nếu muốn tắt Notch).
// 2) Bộ lọc thông thấp Butterworth bậc 2 (-40 dB/decade) cắt nhiễu cao tần mạnh gấp >7 lần PT1.
constexpr float gyroButterworth_hz = 35.0f;
constexpr float gyroNotch_hz = 60.0f;
constexpr float gyroNotch_q = 3.0f;

// Tham số ước lượng tư thế thích nghi (tăng nhẹ hằng số thời gian lọc bù để giảm nhạy rung cơ khí lên accel):
constexpr float locGoc_s = 0.45f;
constexpr float saiLechGiaToc_mps2 = 2.5f;
constexpr float giamTinGoc_rad = 0.08f, matTinGoc_rad = 0.25f, matTin_s = 0.5f;
constexpr bool buGiaTocImu = true;
constexpr bool tangGiaToc = true, tangBamVanToc = true;

// Tham số Bộ điều khiển vòng kín SISO giữ hướng quay ngang (Yaw Heading Hold SISO Lead-PD):
// - 1 Ngõ vào duy nhất: sai lệch góc hướng e_psi = psi_ref - psi [rad]
// - 1 Ngõ ra duy nhất: tốc độ góc quay bù omega_bu [rad/s]
// - Hàm truyền: C_psi(s) = Kp_psi + (Kd_psi * s) / (tau_d * s + 1)
constexpr float kpSisoHuong = 1.8f;
constexpr float kiSisoHuong = 0.0f;
constexpr float kdSisoHuong = 0.15f;
constexpr float tauLocSisoHuong_s = 0.020f;
constexpr float gioiHanRaSisoHuong_radps = 0.45f;
constexpr float gioiHanTichPhanSisoHuong = 0.20f;

// Lần đầu khởi động bắt buộc hiệu chuẩn 3 tư thế qua USB/BOOT để lưu ma trận lắp đúng chiều GY-521 vào NVS.
constexpr bool daXacNhanMaTranLap = false;
constexpr float maTranLap[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
}
