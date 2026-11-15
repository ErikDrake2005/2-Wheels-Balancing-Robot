#include "SafetyMonitor.h"
#include "config/SafetyConfig.h"
#include "config/ControlConfig.h"
#include <cmath>
FaultCode SafetyMonitor::check(const TrangThaiXe& trangThaiXe, uint64_t thoiDiemHienTai_us, float gocThamChieu_rad) {
    if (!trangThaiXe.gocHopLe || !std::isfinite(trangThaiXe.gocNghien_rad) ||
        !std::isfinite(trangThaiXe.tocDoNghien_radps) ||
        !std::isfinite(trangThaiXe.gocNghiengNgang_rad) ||
        !std::isfinite(trangThaiXe.tocDoNghiengNgang_radps) ||
        !std::isfinite(trangThaiXe.doLonGiaToc_mps2) ||
        trangThaiXe.doLonGiaToc_mps2 < giaTocHopLeToiThieu_mps2 ||
        trangThaiXe.doLonGiaToc_mps2 > giaTocHopLeToiDa_mps2) {
        return FaultCode::ImuInvalid;
    }
    if (trangThaiXe.thoiDiemGoc_us > thoiDiemHienTai_us ||
        thoiDiemHienTai_us - trangThaiXe.thoiDiemGoc_us >= tuoiImuToiDa_us) {
        return FaultCode::ImuStale;
    }
    if (!trangThaiXe.viTriHopLe || !trangThaiXe.vanTocHopLe ||
        !std::isfinite(trangThaiXe.viTriDoc_m) || !std::isfinite(trangThaiXe.vanTocDoc_mps)) {
        return FaultCode::PulseInvalid;
    }
    if (trangThaiXe.thoiDiemQuangDuong_us > thoiDiemHienTai_us ||
        thoiDiemHienTai_us - trangThaiXe.thoiDiemQuangDuong_us >= tuoiXungToiDa_us) {
        return FaultCode::PulseStale;
    }
    if (std::fabs(trangThaiXe.gocNghien_rad - gocThamChieu_rad) >= gocNga_rad ||
        std::fabs(trangThaiXe.gocNghiengNgang_rad) >= gocNgaNgang_rad) return FaultCode::Fallen;
    return FaultCode::None;
}
bool SafetyMonitor::canArm(const TrangThaiXe& trangThaiXe, uint64_t thoiDiemHienTai_us, float gocThamChieu_rad) {
    return check(trangThaiXe, thoiDiemHienTai_us, gocThamChieu_rad) == FaultCode::None &&
        trangThaiXe.giaTocTinCay &&
        std::fabs(trangThaiXe.gocNghiengNgang_rad) <= gocNgangChoPhepBat_rad &&
        std::fabs(trangThaiXe.tocDoNghiengNgang_radps) <= tocDoNghiengChoPhepBat_radps &&
        std::fabs(trangThaiXe.gocNghien_rad - gocThamChieu_rad) <= gocChoPhepBat_rad &&
        std::fabs(trangThaiXe.tocDoNghien_radps) <= tocDoNghiengChoPhepBat_radps;
}
