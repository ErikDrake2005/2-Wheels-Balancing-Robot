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
float DifferentialDrive::stepsPerSecond(float vanToc_mps) {
    return static_cast<float>(vanToc_mps / metersPerStep());
}
bool DifferentialDrive::update(const WheelPulseFrame& khungDuLieu, TrangThaiXe& trangThaiXe) {
    trangThaiXe.viTriHopLe = trangThaiXe.vanTocHopLe = false;
    const double quangDuongMoiXung_m = metersPerStep();
    if (!khungDuLieu.hopLe || !khungDuLieu.thoiDiem_us || !std::isfinite(quangDuongMoiXung_m) || quangDuongMoiXung_m <= 0.0) return false;
    if (_khungXungTruoc.hopLe && khungDuLieu.thoiDiem_us <= _khungXungTruoc.thoiDiem_us) return false;
    const double viTriDoc_m = (static_cast<double>(khungDuLieu.trai) + khungDuLieu.phai) * 0.5 * quangDuongMoiXung_m;
    bool vanTocHopLe = false;
    if (_khungXungTruoc.hopLe) {
        const uint64_t khoangCachMau_us = khungDuLieu.thoiDiem_us - _khungXungTruoc.thoiDiem_us;
        if (khoangCachMau_us <= tuoiXungToiDa_us) {
            const float buocThoiGian_s = khoangCachMau_us * 1e-6f;
            const double soXungTrungBinh = ((khungDuLieu.trai - _khungXungTruoc.trai) + (khungDuLieu.phai - _khungXungTruoc.phai)) * 0.5;
            const float vanTocChuaLoc_mps = static_cast<float>(soXungTrungBinh * quangDuongMoiXung_m / buocThoiGian_s);
            _vanTocDaLoc_mps += buocThoiGian_s / (hangSoLocVanToc_s + buocThoiGian_s) * (vanTocChuaLoc_mps - _vanTocDaLoc_mps);
            vanTocHopLe = true;
        } else _vanTocDaLoc_mps = 0.0f;
    }
    _khungXungTruoc = khungDuLieu;
    trangThaiXe.viTriDoc_m = static_cast<float>(viTriDoc_m);
    trangThaiXe.vanTocDoc_mps = _vanTocDaLoc_mps;
    trangThaiXe.thoiDiemQuangDuong_us = khungDuLieu.thoiDiem_us;
    trangThaiXe.nguonQuangDuong = NguonUocLuongQuangDuong::XungBuoc;
    trangThaiXe.viTriHopLe = true;
    trangThaiXe.vanTocHopLe = vanTocHopLe;
    return vanTocHopLe;
}
