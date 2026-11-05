#ifndef CONTROLCONFIG_H
#define CONTROLCONFIG_H

#include <cstdint>

// Giá trị KHỞI ĐẦU để thử/tinh chỉnh trên xe; chưa được nhận dạng thực nghiệm.
// θ > 0: thân nghiêng trước; a > 0: bánh tăng tốc tiến để đỡ thân.
// PID trả gia tốc m/s²; sau đó tích phân thành vận tốc và đổi sang STEP/s.
constexpr float kpGoc = 30.0f;
constexpr float kiGoc = 0.0f;
constexpr float kdGoc = 3.0f;
constexpr float gioiHanTichPhanGoc_rad_s = 0.1f;
constexpr float giaTocCanBangToiDa_mps2 = 3.0f;
// Góc đặt cơ khí cần xác định riêng theo vị trí trọng tâm. UI_Calib không gán giá trị này.
// 0 chỉ là giá trị khởi đầu, không phải kết quả người dùng dựng xe trong bước hiệu chuẩn.
constexpr float gocCanBang_rad = 0.0f;
constexpr bool batPidMo = true;
constexpr float mienSaiLechMo_rad = 0.15f;
constexpr float mienTocDoMo_radps = 1.5f;

constexpr uint32_t chuKyVongVanToc_us = 20000; // 50 Hz, góc theo nhịp IMU.
constexpr float kpVanToc = 0.15f; // rad / (m/s)
constexpr float kiVanToc = 0.03f; // rad / m
constexpr float gocDatToiDa_rad = 0.10f;
constexpr float vanTocDat_mps = 0.0f;
constexpr float hangSoLocVanToc_s = 0.03f;
// PT1 chỉ ở đường D/đầu vào tốc độ của mờ, không lọc chậm đường tích phân góc.
constexpr float tanSoLocGyroD_hz = 40.0f; // Danh nghĩa liên tục; -3 dB rời rạc xem báo cáo.
constexpr float vanTocDiChuyenToiDa_mps = 0.20f;
constexpr float tocDoQuayToiDa_radps = 0.80f;
constexpr float giaTocLenhDiChuyen_mps2 = 0.50f;
constexpr float giaTocLenhQuay_radps2 = 2.0f;
constexpr float vanTocThuSerial_mps = 0.15f;
constexpr float tocDoQuayThuSerial_radps = 0.60f;
constexpr uint32_t hanLenhDiChuyen_us = 500000;
static_assert(tanSoLocGyroD_hz >= 0 && vanTocDiChuyenToiDa_mps > 0 &&
              tocDoQuayToiDa_radps > 0 && giaTocLenhDiChuyen_mps2 > 0 &&
              giaTocLenhQuay_radps2 > 0 && hanLenhDiChuyen_us > 0, "Cau hinh di chuyen khong hop le");
static_assert(kpGoc >= 0 && kiGoc >= 0 && kdGoc >= 0, "He so PID phai khong am");
static_assert(mienSaiLechMo_rad > 0 && mienTocDoMo_radps > 0, "Mien mo phai duong");
static_assert(giaTocCanBangToiDa_mps2 > 0 && gocDatToiDa_rad > 0 &&
              hangSoLocVanToc_s > 0 && chuKyVongVanToc_us > 0, "Gioi han dieu khien khong hop le");

#endif
