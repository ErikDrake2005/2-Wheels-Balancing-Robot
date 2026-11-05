#include "HieuChuanQuayNgang.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
void kiemTra(bool dieuKien, const char* moTa) {
    if (!dieuKien) {
        std::cerr << "FAIL: " << moTa << '\n';
        std::exit(1);
    }
}

Vec3Imu veCamBien(const MaTranQuayImu& maTran, const Vec3Imu& trongXe) {
    const auto& phanTu = maTran.phanTu;
    return {phanTu[0][0]*trongXe.x + phanTu[1][0]*trongXe.y + phanTu[2][0]*trongXe.z,
            phanTu[0][1]*trongXe.x + phanTu[1][1]*trongXe.y + phanTu[2][1]*trongXe.z,
            phanTu[0][2]*trongXe.x + phanTu[1][2]*trongXe.y + phanTu[2][2]*trongXe.z};
}

Vec3Imu giaTocTheoRoll(float roll_rad, float pitch_rad = 0) {
    return {-9.80665f * std::sin(pitch_rad),
            9.80665f * std::cos(pitch_rad) * std::sin(roll_rad),
            9.80665f * std::cos(pitch_rad) * std::cos(roll_rad)};
}
}

int main() {
    const auto lap = HeTrucImu::tuGocLap_deg(-4, 5, 30);
    const Vec3Imu dung = veCamBien(lap, giaTocTheoRoll(0));
    const Vec3Imu trai = veCamBien(lap, giaTocTheoRoll(-0.5f));
    const Vec3Imu phai = veCamBien(lap, giaTocTheoRoll(0.7f));
    HieuChuanQuayNgang luotChuan, nguocChieu, quayNhieuTruc, quayQuaIt;
    for (unsigned mau = 0; mau < 240; ++mau) {
        // Tốc độ 1 rad/s trong 1,2s. Nhiễu ngang nhỏ, đổi dấu luân phiên.
        const float nhieu = (mau % 2 == 0) ? 0.02f : -0.02f;
        luotChuan.themMau(veCamBien(lap, {1, nhieu, nhieu}), 0.005f);
        nguocChieu.themMau(veCamBien(lap, {-1, 0, 0}), 0.005f);
        quayNhieuTruc.themMau(veCamBien(lap, {1, mau % 2 == 0 ? 0.5f : -0.5f, 0}), 0.005f);
        quayQuaIt.themMau(veCamBien(lap, {0.5f, 0, 0}), 0.005f);
    }
    MaTranQuayImu ketQua;
    kiemTra(luotChuan.tinhMaTran(dung, trai, phai, ketQua), "nhan luot trai -> phai co nhieu nho");
    const HeTrucImu heTruc(ketQua);
    const Vec3Imu xuongXe = heTruc.chuyenSangXe(veCamBien(lap, {0, 0.7f, 0}));
    kiemTra(std::fabs(xuongXe.x) < 1e-4f && std::fabs(xuongXe.y - 0.7f) < 1e-4f &&
            std::fabs(xuongXe.z) < 1e-4f, "dung dau gyro pitch sau hieu chuan roll");
    kiemTra(!nguocChieu.tinhMaTran(dung, trai, phai, ketQua), "tu choi gyro nguoc dau hai moc gia toc");
    kiemTra(!quayNhieuTruc.tinhMaTran(dung, trai, phai, ketQua), "tu choi nang luong ngoai truc lon");
    kiemTra(!quayQuaIt.tinhMaTran(dung, trai, phai, ketQua), "tu choi goc gyro tich phan khong khop hai moc");
    kiemTra(!luotChuan.tinhMaTran(dung, dung, phai, ketQua), "tu choi moc trai chua nghieng");
    kiemTra(!luotChuan.tinhMaTran({0, 0, 0}, trai, phai, ketQua), "tu choi moc dung loi");
    for (float pitchNguoiGiu : {-0.10471976f, 0.10471976f}) {
        for (float rollThamChieu : {-0.06981317f, 0.0f, 0.06981317f}) {
            const Vec3Imu mocDung = veCamBien(lap, giaTocTheoRoll(rollThamChieu, pitchNguoiGiu));
            const Vec3Imu mocTrai = veCamBien(lap, giaTocTheoRoll(rollThamChieu - 0.5f, pitchNguoiGiu));
            const Vec3Imu mocPhai = veCamBien(lap, giaTocTheoRoll(rollThamChieu + 0.7f, pitchNguoiGiu));
            kiemTra(luotChuan.tinhMaTran(mocDung, mocTrai, mocPhai, ketQua),
                    "chap nhan nguoi giu lech pitch6do va rollthamchieu4do");
            const HeTrucImu heQuyChieu(ketQua);
            const Vec3Imu gocDung = heQuyChieu.chuyenSangXe(mocDung);
            const float pitchSauCalib = std::atan2(-gocDung.x, std::hypot(gocDung.y, gocDung.z));
            kiemTra(std::fabs(pitchSauCalib - pitchNguoiGiu) < 1e-4f,
                    "khong gan pitch nguoi dung giu bang0");
            const Vec3Imu trucDoc = heQuyChieu.chuyenSangXe(veCamBien(lap, {1, 0, 0}));
            kiemTra(std::fabs(trucDoc.x - 1.0f) < 1e-4f &&
                    std::fabs(trucDoc.y) < 1e-4f && std::fabs(trucDoc.z) < 1e-4f,
                    "giu nguyen truc doc khung xe sau calib");
        }
    }
    std::cout << "PASS: UI_Calib roll axis, signs and quality guards\n";
}
