#include "DifferentialDrive.h"
#include "config/RobotGeometry.h"
#include "config/MotorConfig.h"
#include "config/ControlConfig.h"
#include "config/SafetyConfig.h"
#include <cmath>

double DifferentialDrive::metersPerStep() {
    static_assert(banKinhBanh_m > 0 && soBuocDayDuMoiVongDongCo > 0 &&
        soViBuocDauVaoMks > 0 && soVongDongCoTrenMotVongBanh > 0, "Thong so banh xe khong hop le");
    return 6.283185307179586 * banKinhBanh_m /
        (soBuocDayDuMoiVongDongCo * static_cast<double>(soViBuocDauVaoMks) *
         soVongDongCoTrenMotVongBanh);
}

double DifferentialDrive::metersPerEncoderCount() {
    static_assert(banKinhBanh_m > 0 && soXungEncoderMoiVongDongCo > 0 &&
        soVongDongCoTrenMotVongBanh > 0, "Thong so encoder banh xe khong hop le");
    return 6.283185307179586 * banKinhBanh_m /
        (static_cast<double>(soXungEncoderMoiVongDongCo) * soVongDongCoTrenMotVongBanh);
}

double DifferentialDrive::metersPerFeedbackPulse(NguonUocLuongQuangDuong nguon) {
    return nguon == NguonUocLuongQuangDuong::EncoderBanh
        ? metersPerEncoderCount()
        : metersPerStep();
}

float DifferentialDrive::stepsPerSecond(float vanToc_mps) {
    return static_cast<float>(vanToc_mps / metersPerStep());
}

bool DifferentialDrive::update(const WheelPulseFrame& khungDuLieu, TrangThaiXe& trangThaiXe,
                               NguonUocLuongQuangDuong nguon) {
    trangThaiXe.viTriHopLe = trangThaiXe.vanTocHopLe = false;
    if (nguon == NguonUocLuongQuangDuong::KhongCo) return false;
    const double quangDuongMoiXung_m = metersPerFeedbackPulse(nguon);
    if (!khungDuLieu.hopLe || !khungDuLieu.thoiDiem_us || !std::isfinite(quangDuongMoiXung_m) || quangDuongMoiXung_m <= 0.0) return false;
    if (_khungXungTruoc.hopLe && khungDuLieu.thoiDiem_us <= _khungXungTruoc.thoiDiem_us) return false;

    const double viTriTrai_m = static_cast<double>(khungDuLieu.trai) * quangDuongMoiXung_m;
    const double viTriPhai_m = static_cast<double>(khungDuLieu.phai) * quangDuongMoiXung_m;
    // Stator gắn thân: xung/encoder đo góc bánh SO VỚI THÂN. Quãng đường mặt đất
    // x = R*(phi_tuongDoi + theta), v = R*(phiDot_tuongDoi + thetaDot) (M4, 10/11/2026).
    const bool buGocThan = buGocThanVaoQuangDuong && trangThaiXe.gocHopLe &&
        std::isfinite(trangThaiXe.gocNghien_rad) && std::isfinite(trangThaiXe.tocDoNghien_radps);
    const double viTriDoc_m = (viTriTrai_m + viTriPhai_m) * 0.5 +
        (buGocThan ? static_cast<double>(banKinhBanh_m) * trangThaiXe.gocNghien_rad : 0.0);

    bool vanTocHopLe = false;
    if (_khungXungTruoc.hopLe) {
        const uint64_t khoangCachMau_us = khungDuLieu.thoiDiem_us - _khungXungTruoc.thoiDiem_us;
        if (khoangCachMau_us <= khoangCachKhungXungToiDa_us) {
            const float buocThoiGian_s = khoangCachMau_us * 1e-6f;
            const double deltaTrai = static_cast<double>(khungDuLieu.trai - _khungXungTruoc.trai);
            const double deltaPhai = static_cast<double>(khungDuLieu.phai - _khungXungTruoc.phai);
            const float vanTocTraiTho_mps = static_cast<float>(deltaTrai * quangDuongMoiXung_m / buocThoiGian_s);
            const float vanTocPhaiTho_mps = static_cast<float>(deltaPhai * quangDuongMoiXung_m / buocThoiGian_s);
            const float heSoLoc = buocThoiGian_s / (hangSoLocVanToc_s + buocThoiGian_s);
            _vanTocTraiDaLoc_mps += heSoLoc * (vanTocTraiTho_mps - _vanTocTraiDaLoc_mps);
            _vanTocPhaiDaLoc_mps += heSoLoc * (vanTocPhaiTho_mps - _vanTocPhaiDaLoc_mps);

            const float vanTocChuaLoc_mps = (vanTocTraiTho_mps + vanTocPhaiTho_mps) * 0.5f +
                (buGocThan ? banKinhBanh_m * trangThaiXe.tocDoNghien_radps : 0.0f);
            _vanTocDaLoc_mps += heSoLoc * (vanTocChuaLoc_mps - _vanTocDaLoc_mps);
            vanTocHopLe = true;
        } else {
            _vanTocDaLoc_mps = 0.0f;
            _vanTocTraiDaLoc_mps = 0.0f;
            _vanTocPhaiDaLoc_mps = 0.0f;
        }
    }
    _khungXungTruoc = khungDuLieu;
    trangThaiXe.viTriBanhTrai_m = static_cast<float>(viTriTrai_m);
    trangThaiXe.viTriBanhPhai_m = static_cast<float>(viTriPhai_m);
    trangThaiXe.vanTocBanhTrai_mps = _vanTocTraiDaLoc_mps;
    trangThaiXe.vanTocBanhPhai_mps = _vanTocPhaiDaLoc_mps;
    trangThaiXe.viTriDoc_m = static_cast<float>(viTriDoc_m);
    trangThaiXe.vanTocDoc_mps = _vanTocDaLoc_mps;
    trangThaiXe.thoiDiemQuangDuong_us = khungDuLieu.thoiDiem_us;
    trangThaiXe.nguonQuangDuong = nguon;
    trangThaiXe.viTriHopLe = true;
    trangThaiXe.vanTocHopLe = vanTocHopLe;
    return vanTocHopLe;
}
