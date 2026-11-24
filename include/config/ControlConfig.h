#ifndef CONTROLCONFIG_H
#define CONTROLCONFIG_H

#include <cstdint>
#include "ImuProfile.h"
#include "Lsm6dsvTuning.h"
#include "Mpu6050Tuning.h"

enum class KieuSuyLuanMo : uint8_t {
    SugenoBac0 = 0,
    MamdaniCentroid = 1
};

// Ba chế độ bộ điều khiển độc lập có thể chọn lúc khởi động hoặc luân chuyển trực tiếp qua Serial/WebUI ('p', 'm', 'u', 'f'):
// - CheDoDieuKhien::PidThuan  : Bộ điều khiển PID thuần kinh điển (hệ số cố định độc lập, không qua suy luận mờ).
// - CheDoDieuKhien::PidMo     : Bộ điều khiển PID mờ Mamdani (chỉnh định thích nghi Kp, Ki, Kd theo 25 luật mờ).
// - CheDoDieuKhien::FuzzySiso : Bộ điều khiển mờ SISO 1 ngõ vào (Direct Mamdani 1D: vào e -> 7 tập ngôn ngữ không đều -> 7 luật -> Centroid -> gia tốc lệnh a_cmd, tương đương P phi tuyến).
enum class CheDoDieuKhien : uint8_t {
    PidThuan = 0,
    PidMo = 1,
    FuzzySiso = 2
};

// Giá trị KHỞI ĐẦU để thử/tinh chỉnh trên xe; chưa được nhận dạng thực nghiệm.
// θ > 0: thân nghiêng trước; a > 0: bánh tăng tốc tiến để đỡ thân.
// PID trả gia tốc m/s²; sau đó tích phân thành vận tốc và đổi sang STEP/s.
// Kd=3 chỉ hợp lý khi chiều dài con lắc hiệu dụng đủ lớn (xem RobotGeometry.h,
// chieuDaiConLacHieuDung_m; đo thực tế bằng chu kỳ dao động khi lật ngược xe).
constexpr float kpGoc = dungLsm6dsv ? lsm6dsvTune::kpMo : (dungMpu6050 ? mpu6050Tune::kpMo : 30.0f);
constexpr float kiGoc = dungLsm6dsv ? lsm6dsvTune::kiMo : (dungMpu6050 ? mpu6050Tune::kiMo : 0.0f);
constexpr float kdGoc = dungLsm6dsv ? lsm6dsvTune::kdMo : (dungMpu6050 ? mpu6050Tune::kdMo : 3.0f);

// PID thuần chọn từ các lượt khảo sát đứng cân bằng trên sàn ngày 15/11/2026.
// Đây là bộ tốt nhất trong phạm vi đã thử; NVS có thể ghi đè các giá trị mặc định này.
// Vòng ngoài giữ .15/.06 của xe đã đo; chưa suy rộng sang chạy/rẽ hoặc tải khác.
constexpr float kpGocPidThuan = dungLsm6dsv ? lsm6dsvTune::kp : (dungMpu6050 ? mpu6050Tune::kp : 33.0f);
constexpr float kiGocPidThuan = dungLsm6dsv ? lsm6dsvTune::ki : (dungMpu6050 ? mpu6050Tune::ki : 0.0f);
constexpr float kdGocPidThuan = dungLsm6dsv ? lsm6dsvTune::kd : (dungMpu6050 ? mpu6050Tune::kd : 3.3f);
constexpr float kpVanTocPidThuan = dungLsm6dsv ? lsm6dsvTune::kpv : (dungMpu6050 ? mpu6050Tune::kpv : 0.15f);
constexpr float kiVanTocPidThuan = dungLsm6dsv ? lsm6dsvTune::kiv : (dungMpu6050 ? mpu6050Tune::kiv : 0.06f);

constexpr float gioiHanTichPhanGoc_rad_s = 0.1f;
// Chưa nâng: cần xác nhận mô-men 17HS4401S/nguồn trước (đánh giá M1).
constexpr float giaTocCanBangToiDa_mps2 = 3.0f;
// Góc đặt cơ khí cần xác định riêng theo vị trí trọng tâm. UI_Calib không gán giá trị này.
// 0 chỉ là giá trị khởi đầu, không phải kết quả người dùng dựng xe trong bước hiệu chuẩn.
constexpr float gocCanBang_rad =
    dungLsm6dsv ? lsm6dsvTune::gocCanBang_rad : (dungMpu6050 ? mpu6050Tune::gocCanBang_rad : 0.0f);
constexpr CheDoDieuKhien cheDoDieuKhienKhoiDong = CheDoDieuKhien::PidThuan;
constexpr bool batPidMo = (cheDoDieuKhienKhoiDong == CheDoDieuKhien::PidMo);
// Hệ số hãm trôi vận tốc khi chạy thử trên giá nhấc bánh không ma sát (Bench Mode):
constexpr float heSoGiamChanVanTocBench_1ps = 8.0f;
// Chọn phương pháp suy luận mờ:
// - KieuSuyLuanMo::MamdaniCentroid : Mamdani chuẩn (AND=MIN, Implication=MIN, Aggregation=MAX, Defuzz=Centroid/CoG).
// - KieuSuyLuanMo::SugenoBac0      : Takagi-Sugeno bậc 0 (AND=Product, Defuzz=Weighted Average).
constexpr KieuSuyLuanMo kieuSuyLuanMo = KieuSuyLuanMo::MamdaniCentroid;
// Tối ưu 10/11/2026 (M1): miền mờ trùng vùng CHƯA bão hòa ±3 m/s².
// Kp tối đa 1,4×30=42 → 42×0,07≈2,94 m/s²; Kd tối đa 1,5×3=4,5 → 4,5×0,7≈3,15 m/s².
// Miền cũ 0,15 rad / 1,5 rad/s làm hàng NB/PB chỉ hoạt động khi đầu ra đã bị cắt.
constexpr float mienSaiLechMo_rad = 0.07f;
constexpr float mienTocDoMo_radps = 0.7f;
// Miền chuẩn hóa và tâm 7 tập mờ ngôn ngữ không đều cho bộ điều khiển mờ SISO 1 vào (e -> 7 luật -> Centroid -> a_cmd):
// 7 tập vào  (NB, NM, NS, Z, PS, PM, PB): [-1.00, -c2, -c1, 0, +c1, +c2, +1.00] * eMax_rad
// 7 tập ra   (NB, NM, NS, Z, PS, PM, PB): [-1.00, -u2, -u1, 0, +u1, +u2, +1.00] * ku_mps2
// Mặc định tối ưu giảm chao đảo nhưng vẫn giữ hệ số P tương đương > g (10,86..14,51 m/s²/rad trên 0,2..10°):
constexpr float mienSaiLechMoSiso_rad = 0.21f;
constexpr float tamVaoMoSiso1 = 0.45f;
constexpr float tamVaoMoSiso2 = 0.76f;
constexpr float tamRaMoSiso1 = 0.39f;
constexpr float tamRaMoSiso2 = 0.73f;
constexpr float giaTocMoSisoToiDa_mps2 = giaTocCanBangToiDa_mps2;

struct CauHinhMoSiso {
    float eMax_rad = mienSaiLechMoSiso_rad;
    float c1 = tamVaoMoSiso1;
    float c2 = tamVaoMoSiso2;
    float u1 = tamRaMoSiso1;
    float u2 = tamRaMoSiso2;
    float ku_mps2 = giaTocMoSisoToiDa_mps2;
};

constexpr uint32_t chuKyVongVanToc_us = 20000; // 50 Hz, góc theo nhịp IMU.
constexpr float kpVanToc = dungLsm6dsv ? lsm6dsvTune::kpvMo : (dungMpu6050 ? mpu6050Tune::kpvMo : 0.15f); // rad / (m/s) - Giảm chấn vận tốc mặt đất.
constexpr float kiVanToc = dungLsm6dsv ? lsm6dsvTune::kivMo : (dungMpu6050 ? mpu6050Tune::kivMo : 0.06f); // rad / m     - Độ cứng giữ vị trí mặt đất (Position Hold).
// Khóa trực tiếp tọa độ từ kế/xung 64-bit kết hợp Mamdani thích nghi độ cứng vị trí:
constexpr bool batKhoaViTriChinhXac = true;
constexpr float kiViTri_radpms = 0.0f;           // Giữ = 0 vì con lắc ngược có zero nửa mặt phẳng phải (non-minimum phase).
constexpr float gioiHanSaiLechViTri_m = 0.50f;   // Chặn sai lệch vị trí tối đa ±0,50 m để chống vọt góc khi bị nhấc xe.
constexpr float gocBuTrongTamToiDa_rad = 0.035f; // Giới hạn góc bù tối đa ±0,035 rad (~±2,0 độ).
constexpr float gocDatToiDa_rad = 0.14f;
constexpr float vanTocDat_mps = 0.0f;
constexpr bool batGiuHuongQuay = true;
constexpr float kpGiuHuongQuay = 1.8f;
constexpr float kdGiuHuongQuay = 0.15f;
constexpr float gioiHanSaiLechHuongQuay = 0.50f;
constexpr float gioiHanTocDoQuayGiuHuong = 0.45f;
constexpr float nguongVanTocTinhBanh = 0.08f;
constexpr float hangSoLocVanToc_s = 0.03f;
// PT1 (ICM/LSM) hoặc Biquad Butterworth bậc 2 + Notch (MPU6050) chỉ ở đường D/đầu vào tốc độ của mờ,
// không lọc chậm đường tích phân góc.
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
constexpr float tanSoLocGyroD_hz = lsm6dsvTune::gyroD_hz;
#elif CAU_HINH_IMU == IMU_MPU6050_I2C
constexpr float tanSoLocGyroD_hz = mpu6050Tune::gyroButterworth_hz;
#else
constexpr float tanSoLocGyroD_hz = 40.0f; // Danh nghĩa liên tục; -3 dB rời rạc xem báo cáo.
#endif
constexpr float vanTocDiChuyenToiDa_mps = 0.50f;
constexpr float tocDoQuayToiDa_radps = 0.80f;
constexpr float giaTocLenhDiChuyen_mps2 = 0.50f;
constexpr float giaTocLenhQuay_radps2 = 2.0f;
constexpr float vanTocThuSerial_mps = 0.25f;
constexpr float tocDoQuayThuSerial_radps = 0.60f;
constexpr uint32_t hanLenhDiChuyen_us = 500000;

// Tối ưu 10/11/2026 (M4): xung đo góc bánh so với THÂN; cộng R*theta, R*thetaDot
// để ra quãng đường/vận tốc mặt đất cho vòng PI vận tốc.
constexpr bool buGocThanVaoQuangDuong = true;
// Tối ưu 10/11/2026 (M5): trừ gia tốc tịnh tiến bánh ĐÃ LỆNH khỏi accel trước khi
// lấy phương trọng lực. Phần R*thetaDDot và cánh tay đòn IMU chưa bù.
constexpr bool buGiaTocLenhVaoImu =
    dungLsm6dsv ? lsm6dsvTune::buGiaTocImu : (dungMpu6050 ? mpu6050Tune::buGiaTocImu : true);
// Chặn biên độ bù để lệnh lỗi không thể làm lệch accel quá mức.
constexpr float giaTocBuImuToiDa_mps2 = 4.0f;

static_assert(tanSoLocGyroD_hz >= 0 && vanTocDiChuyenToiDa_mps > 0 &&
              tocDoQuayToiDa_radps > 0 && giaTocLenhDiChuyen_mps2 > 0 &&
              giaTocLenhQuay_radps2 > 0 && hanLenhDiChuyen_us > 0, "Cau hinh di chuyen khong hop le");
static_assert(kpGoc >= 0 && kiGoc >= 0 && kdGoc >= 0, "He so PID phai khong am");
static_assert(mienSaiLechMo_rad > 0 && mienTocDoMo_radps > 0, "Mien mo phai duong");
static_assert(mienSaiLechMoSiso_rad > 0 && tamVaoMoSiso1 > 0.05f && tamVaoMoSiso2 > tamVaoMoSiso1 &&
              tamVaoMoSiso2 < 0.98f && tamRaMoSiso1 > 0.05f && tamRaMoSiso2 > tamRaMoSiso1 &&
              tamRaMoSiso2 < 0.98f, "Cau hinh mo SISO 1 vao khong hop le");
static_assert(giaTocCanBangToiDa_mps2 > 0 && gocDatToiDa_rad > 0 &&
              hangSoLocVanToc_s > 0 && chuKyVongVanToc_us > 0 &&
              gioiHanSaiLechViTri_m > 0 && gocBuTrongTamToiDa_rad >= 0, "Gioi han dieu khien khong hop le");
static_assert(giaTocBuImuToiDa_mps2 >= giaTocCanBangToiDa_mps2, "Bu IMU phai bao trum gia toc lenh");

#endif
