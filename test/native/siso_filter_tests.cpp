#include "LocBiquad.h"
#include "BoDieuKhienSiso.h"
#include "AnglePidController.h"
#include "CascadeController.h"
#include "DifferentialDrive.h"
#include "config/ControlConfig.h"
#include "config/RobotGeometry.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    static_assert(dungMpu6050, "Test nay kiem tra profile MPU6050 + Biquad + SISO");
    constexpr double soPi = 3.14159265358979323846;

    // 1. Kiểm tra bộ lọc số SISO Biquad (Butterworth bậc 2 + Notch khử cộng hưởng động cơ):
    {
        LocBiquadSiso locButter(LocBiquadSiso::LoaiLoc::ThongThapButterworth);
        LocBiquadSiso locNotch(LocBiquadSiso::LoaiLoc::KhiaNotch);

        // Khởi tạo êm ở mẫu đầu tiên (DC gain = 1, không tạo gai đạo hàm lúc vừa bật motor):
        assert(std::fabs(locButter.capNhatThongThap(1.5f, 0.005f, 35.0f) - 1.5f) < 1e-6f);
        assert(std::fabs(locNotch.capNhatNotch(-0.8f, 0.005f, 60.0f, 3.0f) + 0.8f) < 1e-6f);

        // Loại bỏ NaN mà không làm hỏng trạng thái bộ lọc:
        const float nanVal = std::numeric_limits<float>::quiet_NaN();
        assert(std::isnan(locButter.capNhatThongThap(nanVal, 0.005f, 35.0f)));
        assert(std::fabs(locButter.capNhatThongThap(1.5f, 0.005f, 35.0f) - 1.5f) < 1e-5f);

        // Đo suy giảm tần số thực tế tại fs = 200 Hz:
        // a) Tín hiệu cân bằng 3.34 Hz đi qua nguyên vẹn (> 0.99 biên độ):
        locButter.datLai(); locNotch.datLai();
        double eVaoThap = 0.0, eRaThap = 0.0;
        for (unsigned k = 0; k < 1000; ++k) {
            const float x = static_cast<float>(std::sin(2.0 * soPi * 3.34 * k * 0.005));
            const float y = locButter.capNhatThongThap(locNotch.capNhatNotch(x, 0.005f, 60.0f, 3.0f), 0.005f, 35.0f);
            if (k >= 200) { eVaoThap += x * x; eRaThap += y * y; }
        }
        const double tiLeThap = std::sqrt(eRaThap / eVaoThap);
        assert(tiLeThap > 0.99 && tiLeThap <= 1.001);

        // b) Rung cộng hưởng động cơ 60 Hz bị triệt tiêu hoàn toàn (< 0.005 biên độ, > 46 dB):
        locButter.datLai(); locNotch.datLai();
        double eVao60 = 0.0, eRa60 = 0.0;
        for (unsigned k = 0; k < 1000; ++k) {
            const float x = static_cast<float>(std::sin(2.0 * soPi * 60.0 * k * 0.005));
            const float y = locButter.capNhatThongThap(locNotch.capNhatNotch(x, 0.005f, 60.0f, 3.0f), 0.005f, 35.0f);
            if (k >= 200) { eVao60 += x * x; eRa60 += y * y; }
        }
        assert(std::sqrt(eRa60 / eVao60) < 0.005);

        // c) Nhiễu cao tần 80 Hz suy giảm mạnh hơn 25 dB (< 0.056 biên độ, vượt xa PT1 bậc 1):
        locButter.datLai(); locNotch.datLai();
        double eVao80 = 0.0, eRa80 = 0.0;
        for (unsigned k = 0; k < 1000; ++k) {
            const float x = static_cast<float>(std::sin(2.0 * soPi * 80.0 * k * 0.005));
            const float y = locButter.capNhatThongThap(locNotch.capNhatNotch(x, 0.005f, 60.0f, 3.0f), 0.005f, 35.0f);
            if (k >= 200) { eVao80 += x * x; eRa80 += y * y; }
        }
        assert(std::sqrt(eRa80 / eVao80) < 0.055);
    }

    // 2. Kiểm tra Bộ điều khiển vòng kín SISO chuẩn (1 vào e[k] -> 1 ra u[k], Tustin Lead-PD / PIDF):
    {
        BoDieuKhienSiso boSiso({1.8f, 0.0f, 0.15f, 0.020f, 0.20f, 0.45f});
        float psi_rad = 0.15f;     // Nhiễu lệch hướng ban đầu +0.15 rad (~8.6 độ)
        float omegaThuc_radps = 0.0f;
        float vungQuaDoAm = 0.0f;
        constexpr float dt = 0.005f;
        constexpr float tauMotor_s = 0.015f;

        for (unsigned k = 0; k < 500; ++k) {
            const float saiLechHuong = 0.0f - psi_rad; // 1 ngõ vào duy nhất e_psi
            const float omegaLenh = boSiso.capNhat(saiLechHuong, dt); // 1 ngõ ra duy nhất u_omega
            assert(std::isfinite(omegaLenh) && std::fabs(omegaLenh) <= 0.45f + 1e-6f);
            // Đối tượng SISO bậc 2 loại 1: G_psi(s) = 1 / [s * (tau_m * s + 1)]
            for (unsigned sub = 0; sub < 5; ++sub) {
                constexpr float dts = dt / 5.0f;
                omegaThuc_radps += ((omegaLenh - omegaThuc_radps) / tauMotor_s) * dts;
                psi_rad += omegaThuc_radps * dts;
            }
            if (psi_rad < vungQuaDoAm) vungQuaDoAm = psi_rad;
        }
        assert(std::fabs(psi_rad) < 0.003f);   // Bám triệt tiêu sai lệch xác lập về 0
        assert(vungQuaDoAm > -0.002f);         // Hầu như không vọt lố nhờ khâu sớm pha Tustin Lead-D

        // Kiểm tra chống bão hòa tích phân (Clamping Anti-Windup) khi bật thêm Ki > 0:
        BoDieuKhienSiso sisoPidf({2.0f, 5.0f, 0.1f, 0.02f, 0.50f, 0.30f});
        for (unsigned k = 0; k < 200; ++k) sisoPidf.capNhat(1.0f, dt); // Ép bão hòa trần dương +0.30
        assert(std::fabs(sisoPidf.thanhPhanTichPhan()) < 1e-5f);       // Không tích lũy windup khi đang bão hòa P
    }

    // 3. Mô phỏng khép vòng toàn hệ thống MPU6050 (CascadeController + Biquad chống rung + SISO giữ hướng)
    //    dưới tác động rung động cơ bước mạnh ở 60 Hz & 78 Hz:
    {
        CascadeController boDieuKhien;
        TrangThaiXe xe{};
        xe.gocHopLe = xe.viTriHopLe = xe.vanTocHopLe = true;
        xe.nguonQuangDuong = NguonUocLuongQuangDuong::XungBuoc;

        float theta = 0.030f; // Lệch ban đầu ~1.72 độ
        float omega = 0.0f;
        float sTrai = 0.0f, sPhai = 0.0f;
        float vTrai = 0.0f, vPhai = 0.0f;
        WheelStepCommand lenh{};
        const double mPerStep = DifferentialDrive::metersPerStep();

        // Đồng bộ mốc ban đầu tại t = 1000 us, sau đó tạo nhiễu lệch bánh phải +4 mm (~0.02 rad lệch hướng)
        assert(boDieuKhien.update(xe, 1000, false, lenh));
        sPhai = 0.004f;

        double tongBinhPhuongChatter = 0.0;
        float giaTocLenhTruoc = 0.0f;
        unsigned demMauXacLap = 0;

        for (unsigned k = 1; k <= 800; ++k) {
            const uint64_t t_us = 1000u + k * 5000u;
            const double t_s = k * 0.005;
            // Cộng rung cơ khí 60 Hz + 78 Hz biên độ lớn (0.15 rad/s) vào gyro đo được của MPU6050:
            const float rungDongCo = static_cast<float>(
                0.12 * std::sin(2.0 * soPi * 60.0 * t_s) +
                0.08 * std::sin(2.0 * soPi * 78.0 * t_s));

            xe.thoiDiemGoc_us = xe.thoiDiemQuangDuong_us = t_us;
            xe.gocNghien_rad = theta;
            xe.tocDoNghien_radps = omega + rungDongCo;
            xe.viTriBanhTrai_m = sTrai;
            xe.viTriBanhPhai_m = sPhai;
            xe.vanTocBanhTrai_mps = vTrai;
            xe.vanTocBanhPhai_mps = vPhai;
            xe.viTriDoc_m = 0.5f * (sTrai + sPhai) + banKinhBanh_m * theta;
            xe.vanTocDoc_mps = 0.5f * (vTrai + vPhai) + banKinhBanh_m * omega;

            assert(boDieuKhien.update(xe, t_us, false, lenh));
            const float aCmd = boDieuKhien.giaTocBanhDaLenh_mps2();
            if (k > 250) {
                const double da = aCmd - giaTocLenhTruoc;
                tongBinhPhuongChatter += da * da;
                ++demMauXacLap;
            }
            giaTocLenhTruoc = aCmd;

            const float vLenhTrai = static_cast<float>(lenh.tocDoBuocTrai_buocTrenGiay * mPerStep);
            const float vLenhPhai = static_cast<float>(lenh.tocDoBuocPhai_buocTrenGiay * mPerStep);
            const float aThuc = 0.5f * ((vLenhTrai - vTrai) + (vLenhPhai - vPhai)) / 0.005f;
            vTrai = vLenhTrai;
            vPhai = vLenhPhai;
            for (unsigned sub = 0; sub < 5; ++sub) {
                constexpr float dts = 0.001f;
                const float thDDot = (9.81f * std::sin(theta) - aThuc * std::cos(theta)) / chieuDaiConLacHieuDung_m;
                omega += thDDot * dts;
                theta += omega * dts;
                sTrai += vTrai * dts;
                sPhai += vPhai * dts;
            }
        }

        const double rmsChatter = std::sqrt(tongBinhPhuongChatter / demMauXacLap);
        const float saiLechHuongCuoi = (sPhai - sTrai) / khoangCachTamHaiBanh_m;
        assert(std::fabs(theta) < 0.005f);           // Xe đứng cân bằng vững (< 0.3 độ)
        assert(std::fabs(saiLechHuongCuoi) < 0.002f);// Bộ SISO giữ hướng kéo lệch hướng từ 0.02 rad về ~0
        assert(rmsChatter < 0.08);                   // Độ giật lệnh gia tốc motor cực thấp nhờ lọc Biquad
    }

    std::cout << "PASS: Biquad Butterworth2+Notch vibration suppression & SISO closed-loop controller verified\n";
    return 0;
}
