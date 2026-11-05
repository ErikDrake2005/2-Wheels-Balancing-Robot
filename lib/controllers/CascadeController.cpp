#include "CascadeController.h"
#include "DifferentialDrive.h"
#include "config/ControlConfig.h"
#include "config/MotorConfig.h"
#include "config/RobotGeometry.h"
#include "BoTronLenhBanh.h"
#include <algorithm>
#include <cmath>

void CascadeController::reset() {
    _boPidGoc.reset(); _thoiDiemTruoc_us = 0;
    _thoiGianVongVanToc_s = _tichPhanVanToc_m = _vanTocLenh_mps = 0;
    _gocDat_rad = gocCanBang_rad;
    _vanTocMucTieu_mps = _tocDoQuayMucTieu_radps = 0;
}
bool CascadeController::update(const TrangThaiXe& trangThaiXe, uint64_t thoiDiemHienTai_us,
    bool dungPidMo, WheelStepCommand& lenhBanh, float vanTocYeuCau_mps, float tocDoQuayYeuCau_radps) {
    lenhBanh = {};
    if (!trangThaiXe.gocHopLe || !trangThaiXe.vanTocHopLe || !std::isfinite(trangThaiXe.vanTocDoc_mps) ||
        !std::isfinite(trangThaiXe.gocNghien_rad) || !std::isfinite(trangThaiXe.tocDoNghien_radps) ||
        !std::isfinite(vanTocYeuCau_mps) || !std::isfinite(tocDoQuayYeuCau_radps) ||
        !thoiDiemHienTai_us || (_thoiDiemTruoc_us && thoiDiemHienTai_us <= _thoiDiemTruoc_us)) return false;
    lenhBanh.thoiDiemRaLenh_us = thoiDiemHienTai_us;
    lenhBanh.thoiDiemHetHan_us = thoiDiemHienTai_us + thoiGianHetHanLenhMotor_us;
    if (!_thoiDiemTruoc_us) { _thoiDiemTruoc_us = thoiDiemHienTai_us; return true; }
    const float buocThoiGian_s = (thoiDiemHienTai_us - _thoiDiemTruoc_us)*1e-6f;
    _thoiDiemTruoc_us = thoiDiemHienTai_us;
    if (buocThoiGian_s > 0.02f) { reset(); return false; }
    vanTocYeuCau_mps = std::clamp(vanTocYeuCau_mps, -vanTocDiChuyenToiDa_mps, vanTocDiChuyenToiDa_mps);
    tocDoQuayYeuCau_radps = std::clamp(tocDoQuayYeuCau_radps, -tocDoQuayToiDa_radps, tocDoQuayToiDa_radps);
    _vanTocMucTieu_mps += std::clamp(vanTocYeuCau_mps - _vanTocMucTieu_mps,
        -giaTocLenhDiChuyen_mps2 * buocThoiGian_s, giaTocLenhDiChuyen_mps2 * buocThoiGian_s);
    _tocDoQuayMucTieu_radps += std::clamp(tocDoQuayYeuCau_radps - _tocDoQuayMucTieu_radps,
        -giaTocLenhQuay_radps2 * buocThoiGian_s, giaTocLenhQuay_radps2 * buocThoiGian_s);
    _thoiGianVongVanToc_s += buocThoiGian_s;
    if (_thoiGianVongVanToc_s >= chuKyVongVanToc_us*1e-6f) {
        const float saiLechVanToc_mps = _vanTocMucTieu_mps - trangThaiXe.vanTocDoc_mps;
        const float tichPhanDuKien_m = _tichPhanVanToc_m + saiLechVanToc_mps*_thoiGianVongVanToc_s;
        const float gocChuaGioiHan_rad = kpVanToc*saiLechVanToc_mps + kiVanToc*tichPhanDuKien_m;
        const float gocDaGioiHan_rad = std::clamp(gocChuaGioiHan_rad, -gocDatToiDa_rad, gocDatToiDa_rad);
        if (gocChuaGioiHan_rad == gocDaGioiHan_rad || gocChuaGioiHan_rad*saiLechVanToc_mps < 0) _tichPhanVanToc_m = tichPhanDuKien_m;
        _gocDat_rad = gocCanBang_rad + gocDaGioiHan_rad;
        _thoiGianVongVanToc_s = 0;
    }
    const float vanTocToiDa_mps = static_cast<float>(tanSoStepToiDa_hz * DifferentialDrive::metersPerStep());
    const int huongBaoHoaVanToc = _vanTocLenh_mps >= vanTocToiDa_mps ? 1 :
        (_vanTocLenh_mps <= -vanTocToiDa_mps ? -1 : 0);
    const float giaTocLenh_mps2 = _boPidGoc.update(trangThaiXe.gocNghien_rad, trangThaiXe.tocDoNghien_radps,
                                              _gocDat_rad, buocThoiGian_s, dungPidMo, huongBaoHoaVanToc);
    if (!std::isfinite(giaTocLenh_mps2)) return false;
    _vanTocLenh_mps = std::clamp(_vanTocLenh_mps + giaTocLenh_mps2*buocThoiGian_s, -vanTocToiDa_mps, vanTocToiDa_mps);
    const auto haiBanh = tronUuTienCanBang(_vanTocLenh_mps, _tocDoQuayMucTieu_radps,
        khoangCachTamHaiBanh_m, vanTocToiDa_mps);
    lenhBanh.tocDoBuocTrai_buocTrenGiay = DifferentialDrive::stepsPerSecond(haiBanh.trai_mps);
    lenhBanh.tocDoBuocPhai_buocTrenGiay = DifferentialDrive::stepsPerSecond(haiBanh.phai_mps);
    return std::isfinite(lenhBanh.tocDoBuocTrai_buocTrenGiay);
}
