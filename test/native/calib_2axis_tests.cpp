#include "HieuChuanHaiTruc.h"
#include "ComplementaryEstimator.h"
#include "config/RobotGeometry.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>

namespace {
constexpr float g = 9.80665f, doiRad = 0.01745329252f;
unsigned soDieuKien = 0;
void kiemTra(bool dat, const char* moTa) {
    ++soDieuKien;
    if (!dat) { std::cerr << "FAIL: " << moTa << '\n'; std::exit(1); }
}
Vec3Imu veCamBien(const MaTranQuayImu& lap, const Vec3Imu& than) {
    const auto& r = lap.phanTu;
    return {
        r[0][0] * than.x + r[1][0] * than.y + r[2][0] * than.z,
        r[0][1] * than.x + r[1][1] * than.y + r[2][1] * than.z,
        r[0][2] * than.x + r[1][2] * than.y + r[2][2] * than.z
    };
}
Vec3Imu giaTocPitch(float goc_deg) {
    return {-g * std::sin(goc_deg * doiRad), 0.0f, g * std::cos(goc_deg * doiRad)};
}
Vec3Imu giaTocRoll(float goc_deg) {
    return {0.0f, g * std::sin(goc_deg * doiRad), g * std::cos(goc_deg * doiRad)};
}
HieuChuanHaiTruc taoLuotHaiTruc(const MaTranQuayImu& lap, float gocPitch_deg, float gocRoll_deg, float nhieu = 0.005f) {
    HieuChuanHaiTruc luot;
    const float tongPitch_rad = 2.0f * gocPitch_deg * doiRad;
    const float tongRoll_rad = 2.0f * gocRoll_deg * doiRad;
    for (unsigned i = 0; i < 600; ++i) {
        const float dau = (i % 2) ? 1.0f : -1.0f;
        luot.themMauPitch(veCamBien(lap, {dau * nhieu, -tongPitch_rad / 3.0f, dau * nhieu}), 0.005f);
    }
    for (unsigned i = 0; i < 400; ++i) {
        const float dau = (i % 2) ? 1.0f : -1.0f;
        luot.themMauRoll(veCamBien(lap, {tongRoll_rad / 2.0f, dau * nhieu, dau * nhieu}), 0.005f);
    }
    return luot;
}
}

int main() {
    // 1. Kiểm tra cập nhật chiều cao LSM6DSV (+10 mm so với ICM cũ: 75 mm -> 85 mm, 111 mm -> 121 mm):
    static_assert(dungLsm6dsv, "Test nay bien dich voi IMU_LSM6DSV_SPI");
    kiemTra(std::fabs(khoangCachImuDenTrongTamZ_m - 0.085f) < 1e-6f, "LSM cao hon ICM 10mm -> 0.085m");
    kiemTra(std::fabs(viTriMpuZ_m - 0.121f) < 1e-6f, "Toa do Z LSM so voi truc banh la 0.121m");

    // 2. Quét toàn bộ không gian góc lắp 3D (roll, pitch, yaw) + góc nghiêng Roll 40..70 độ:
    for (float rollLap : {-15.0f, 0.0f, 18.0f})
    for (float pitchLap : {-10.0f, 0.0f, 12.0f})
    for (float yawLap : {-90.0f, -35.0f, 0.0f, 45.0f, 90.0f, 180.0f})
    for (float gocRoll : {40.0f, 65.0f}) {
        const auto lap = HeTrucImu::tuGocLap_deg(rollLap, pitchLap, yawLap);
        const auto luot = taoLuotHaiTruc(lap, 85.0f, gocRoll);
        MaTranQuayImu ketQua{};
        BaoCaoCalibHaiTruc baoCao{};
        kiemTra(luot.tinhMaTran(
            veCamBien(lap, giaTocPitch(4.0f)),
            veCamBien(lap, giaTocPitch(85.0f)),
            veCamBien(lap, giaTocPitch(-85.0f)),
            veCamBien(lap, giaTocRoll(-gocRoll)),
            veCamBien(lap, giaTocRoll(gocRoll)),
            ketQua, &baoCao), "calib 2 truc PASS tren moi goc lap 3D");
        kiemTra(std::strcmp(baoCao.lyDo, "PASS") == 0 && baoCao.lechVuongGoc_rad < 1e-4f, "hai truc vuong goc chinh xac");
        const HeTrucImu heTruc(ketQua);
        const Vec3Imu x = heTruc.chuyenSangXe(veCamBien(lap, {1, 0, 0}));
        const Vec3Imu y = heTruc.chuyenSangXe(veCamBien(lap, {0, 1, 0}));
        const Vec3Imu z = heTruc.chuyenSangXe(veCamBien(lap, {0, 0, 1}));
        kiemTra(std::fabs(x.x - 1) < 1e-4f && std::fabs(x.y) < 1e-4f && std::fabs(x.z) < 1e-4f, "truc X chinh xac");
        kiemTra(std::fabs(y.x) < 1e-4f && std::fabs(y.y - 1) < 1e-4f && std::fabs(y.z) < 1e-4f, "truc Y chinh xac");
        kiemTra(std::fabs(z.x) < 1e-4f && std::fabs(z.y) < 1e-4f && std::fabs(z.z - 1) < 1e-4f, "truc Z chinh xac");
    }

    // 3. Kiểm tra tính trực giao đối xứng: nếu lượt Roll bị vặn lệch 4 độ quanh Z, sai số chia đều 2 độ cho X và 2 độ cho Y:
    {
        const auto lap = HeTrucImu::tuGocLap_deg(0, 0, 0);
        const auto lapLech4 = HeTrucImu::tuGocLap_deg(0, 0, 4.0f);
        const auto luot = taoLuotHaiTruc(lap, 88.0f, 55.0f);
        MaTranQuayImu ketQua{};
        BaoCaoCalibHaiTruc baoCao{};
        kiemTra(luot.tinhMaTran(
            giaTocPitch(0), giaTocPitch(88.0f), giaTocPitch(-88.0f),
            veCamBien(lapLech4, giaTocRoll(-55.0f)),
            veCamBien(lapLech4, giaTocRoll(55.0f)),
            ketQua, &baoCao), "chap nhan lech vuong goc nho 4 do");
        kiemTra(std::fabs(baoCao.lechVuongGoc_rad / doiRad - 4.0f) < 0.05f, "do dung do lech vuong goc 4 do");
        const float saiX_deg = std::acos(std::clamp(ketQua.phanTu[0][0], -1.0f, 1.0f)) / doiRad;
        const float saiY_deg = std::acos(std::clamp(ketQua.phanTu[1][1], -1.0f, 1.0f)) / doiRad;
        kiemTra(std::fabs(saiX_deg - 2.0f) < 0.05f && std::fabs(saiY_deg - 2.0f) < 0.05f,
            "truc giao doi xung chia deu 2 do cho X va 2 do cho Y");
    }

    // 4. Kiểm tra các tình huống từ chối lỗi (đảo chiều Trái/Phải, góc Roll quá nhỏ, lệch vuông góc > 15 độ):
    {
        const auto lap = HeTrucImu::tuGocLap_deg(5, -3, 90);
        const auto luot = taoLuotHaiTruc(lap, 85.0f, 50.0f);
        MaTranQuayImu ketQua{};
        BaoCaoCalibHaiTruc baoCao{};
        // Đảo mốc Trái <-> Phải:
        kiemTra(!luot.tinhMaTran(
            veCamBien(lap, giaTocPitch(0)), veCamBien(lap, giaTocPitch(85)), veCamBien(lap, giaTocPitch(-85)),
            veCamBien(lap, giaTocRoll(50)), veCamBien(lap, giaTocRoll(-50)), ketQua, &baoCao),
            "tu choi dao nguoc TRAI/PHAI");
        // Nghiêng Roll quá nhỏ (10 độ < 25 độ):
        const auto luotNho = taoLuotHaiTruc(lap, 85.0f, 10.0f);
        kiemTra(!luotNho.tinhMaTran(
            veCamBien(lap, giaTocPitch(0)), veCamBien(lap, giaTocPitch(85)), veCamBien(lap, giaTocPitch(-85)),
            veCamBien(lap, giaTocRoll(-10)), veCamBien(lap, giaTocRoll(10)), ketQua, &baoCao),
            "tu choi goc roll qua nho");
        // Không vuông góc (lệch 25 độ):
        const auto lapLech25 = HeTrucImu::tuGocLap_deg(5, -3, 115);
        kiemTra(!luot.tinhMaTran(
            veCamBien(lap, giaTocPitch(0)), veCamBien(lap, giaTocPitch(85)), veCamBien(lap, giaTocPitch(-85)),
            veCamBien(lapLech25, giaTocRoll(-50)), veCamBien(lapLech25, giaTocRoll(50)), ketQua, &baoCao),
            "tu choi hai truc lech vuong goc qua 15 do");
    }

    std::cout << "PASS: 2-axis Pitch+Roll symmetric orthogonalization calibration & LSM +10mm Z geometry (" << soDieuKien << " checks)\n";
    return 0;
}
