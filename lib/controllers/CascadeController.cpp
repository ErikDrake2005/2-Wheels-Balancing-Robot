#include "CascadeController.h"
#include "DifferentialDrive.h"
#include "config/ControlConfig.h"
#include "config/MotorConfig.h"
#include "config/RobotGeometry.h"
#include "BoTronLenhBanh.h"
#include <algorithm>
#include <cmath>

void CascadeController::setPidVanTocThuan(float kpV, float kiV) {
    if (std::isfinite(kpV) && kpV >= 0.0f) _kpVanTocPidThuan = kpV;
    if (std::isfinite(kiV) && kiV >= 0.0f) _kiVanTocPidThuan = kiV;
}

void CascadeController::datPidVanTocMo(float kpVanTocMoi, float kiVanTocMoi) {
    if (std::isfinite(kpVanTocMoi) && kpVanTocMoi >= 0.0f) kpVanTocPidMo = kpVanTocMoi;
    if (std::isfinite(kiVanTocMoi) && kiVanTocMoi >= 0.0f) kiVanTocPidMo = kiVanTocMoi;
}

void CascadeController::setGocCanBang_rad(float goc_rad) {
    if (std::isfinite(goc_rad) && std::fabs(goc_rad) <= 0.35f) {
        _gocCanBang_rad = goc_rad;
        if (!_batVongNgoai) _gocDat_rad = _gocCanBang_rad;
    }
}

void CascadeController::setBatVongNgoai(bool bat) {
    if (_batVongNgoai == bat) return;
    _batVongNgoai = bat;
    if (!bat) {
        _tichPhanVanToc_m = 0.0f;
        _gocBuTrongTam_rad = 0.0f;
        _gocDat_rad = _gocCanBang_rad;
    } else {
        _daDongBoViTri = false;
    }
}

void CascadeController::reset() {
    _boPidGoc.reset();
    _boVongKinBanhXe.reset();
    _thoiDiemTruoc_us = 0;
    _thoiGianVongVanToc_s = _tichPhanVanToc_m = _vanTocLenh_mps = 0;
    _viTriMucTieu_m = _gocBuTrongTam_rad = 0;
    _daDongBoViTri = false;
    hieuViTriHaiBanhMucTieu = 0.0f;
    daDongBoHuongQuay = false;
    dangDiChuyenDocTruoc = false;
    coLenhDiChuyenDocTruoc = false;
    _gocDat_rad = _gocCanBang_rad;
    _vanTocMucTieu_mps = _tocDoQuayMucTieu_radps = 0;
    _giaTocBanhDaLenh_mps2 = 0;
}

void CascadeController::chotViTriGiuTaiCho() {
    _daDongBoViTri = false;
    daDongBoHuongQuay = false;
    _tichPhanVanToc_m = 0.0f;
}

bool CascadeController::update(const TrangThaiXe& trangThaiXe, uint64_t thoiDiemHienTai_us,
    bool dungPidMo, WheelStepCommand& lenhBanh, float vanTocYeuCau_mps, float tocDoQuayYeuCau_radps) {
    lenhBanh = {};
    _giaTocBanhDaLenh_mps2 = 0;
    if (!trangThaiXe.gocHopLe || !trangThaiXe.vanTocHopLe || !std::isfinite(trangThaiXe.vanTocDoc_mps) ||
        !std::isfinite(trangThaiXe.gocNghien_rad) || !std::isfinite(trangThaiXe.tocDoNghien_radps) ||
        !std::isfinite(vanTocYeuCau_mps) || !std::isfinite(tocDoQuayYeuCau_radps) ||
        !thoiDiemHienTai_us || (_thoiDiemTruoc_us && thoiDiemHienTai_us <= _thoiDiemTruoc_us)) return false;

    const bool coViTriChinhXac = batKhoaViTriChinhXac &&
        trangThaiXe.viTriHopLe && std::isfinite(trangThaiXe.viTriDoc_m);

    lenhBanh.thoiDiemRaLenh_us = thoiDiemHienTai_us;
    lenhBanh.thoiDiemHetHan_us = thoiDiemHienTai_us + thoiGianHetHanLenhMotor_us;
    if (!_thoiDiemTruoc_us) {
        _thoiDiemTruoc_us = thoiDiemHienTai_us;
        if (coViTriChinhXac) {
            _viTriMucTieu_m = trangThaiXe.viTriDoc_m;
            _daDongBoViTri = true;
            hieuViTriHaiBanhMucTieu = trangThaiXe.viTriBanhPhai_m - trangThaiXe.viTriBanhTrai_m;
            daDongBoHuongQuay = true;
        }
        return true;
    }
    const float buocThoiGian_s = (thoiDiemHienTai_us - _thoiDiemTruoc_us) * 1e-6f;
    _thoiDiemTruoc_us = thoiDiemHienTai_us;
    if (buocThoiGian_s > 0.02f) { reset(); return false; }

    vanTocYeuCau_mps = std::clamp(vanTocYeuCau_mps, -vanTocDiChuyenToiDa_mps, vanTocDiChuyenToiDa_mps);
    tocDoQuayYeuCau_radps = std::clamp(tocDoQuayYeuCau_radps, -tocDoQuayToiDa_radps, tocDoQuayToiDa_radps);
    const float giaTocLenhDoc_mps2 = (std::fabs(_vanTocMucTieu_mps) > 1e-5f)
        ? (2.5f * giaTocLenhDiChuyen_mps2)
        : giaTocLenhDiChuyen_mps2;
    _vanTocMucTieu_mps += std::clamp(vanTocYeuCau_mps - _vanTocMucTieu_mps,
        -giaTocLenhDoc_mps2 * buocThoiGian_s, giaTocLenhDoc_mps2 * buocThoiGian_s);
    _tocDoQuayMucTieu_radps += std::clamp(tocDoQuayYeuCau_radps - _tocDoQuayMucTieu_radps,
        -giaTocLenhQuay_radps2 * buocThoiGian_s, giaTocLenhQuay_radps2 * buocThoiGian_s);

    const bool coLenhDiChuyenDoc = std::fabs(vanTocYeuCau_mps) > 1e-4f;
    const bool dangDiChuyenDoc = coLenhDiChuyenDoc || std::fabs(_vanTocMucTieu_mps) > 1e-4f;
    if (coViTriChinhXac) {
        if (!_daDongBoViTri) {
            _viTriMucTieu_m = trangThaiXe.viTriDoc_m;
            _daDongBoViTri = true;
        } else if (coLenhDiChuyenDoc) {
            // Tích phân vị trí dẫn đường giới hạn ±0,25 m trước thân xe khi tiến/lùi
            // để cả thành phần vị trí và vận tốc cùng kéo xe tăng tốc nhanh mà không windup trên dốc.
            constexpr float cuaSoDanDuong_m = 0.25f;
            _viTriMucTieu_m += _vanTocMucTieu_mps * buocThoiGian_s;
            _viTriMucTieu_m = std::clamp(
                _viTriMucTieu_m,
                trangThaiXe.viTriDoc_m - cuaSoDanDuong_m,
                trangThaiXe.viTriDoc_m + cuaSoDanDuong_m);
        } else {
            if (coLenhDiChuyenDocTruoc) {
                _viTriMucTieu_m = trangThaiXe.viTriDoc_m;
            }
            // Khi nha can ga ve 0: tich phan muot quang duong giam toc roi khoa cung vi tri dung yen.
            _viTriMucTieu_m += _vanTocMucTieu_mps * buocThoiGian_s;
            _viTriMucTieu_m = std::clamp(
                _viTriMucTieu_m,
                trangThaiXe.viTriDoc_m - gioiHanSaiLechViTri_m,
                trangThaiXe.viTriDoc_m + gioiHanSaiLechViTri_m);
        }
    }
    dangDiChuyenDocTruoc = dangDiChuyenDoc;
    coLenhDiChuyenDocTruoc = coLenhDiChuyenDoc;

    _thoiGianVongVanToc_s += buocThoiGian_s;
    if (!_batVongNgoai) {
        if (coViTriChinhXac) _viTriMucTieu_m = trangThaiXe.viTriDoc_m;
        _tichPhanVanToc_m = 0.0f;
        _gocBuTrongTam_rad = 0.0f;
        _gocDat_rad = _gocCanBang_rad;
        _thoiGianVongVanToc_s = 0.0f;
    } else if (_thoiGianVongVanToc_s >= chuKyVongVanToc_us * 1e-6f) {
        const float saiLechVanToc_mps = _vanTocMucTieu_mps - trangThaiXe.vanTocDoc_mps;
        const float heSoBamVanTocDiChuyen = coLenhDiChuyenDoc ? 2.2f : 1.0f;
        if (coViTriChinhXac && _daDongBoViTri) {
            // Khóa vị trí trực tiếp từ tọa độ từ kế/xung 64-bit:
            // - Khi dungPidMo == true  (Chế độ PID Mờ): dùng bộ suy luận Mamdani chỉnh định độ cứng vị trí.
            // - Khi dungPidMo == false (Chế độ PID Thuần): dùng bộ hệ số cố định độc lập _kpVanTocPidThuan, _kiVanTocPidThuan.
            const float saiLechViTri_m = std::clamp(
                _viTriMucTieu_m - trangThaiXe.viTriDoc_m, -gioiHanSaiLechViTri_m, gioiHanSaiLechViTri_m);
            const PidGains heSoVongNgoai = dungPidMo
                ? AnglePidController::scheduledGains(
                      saiLechViTri_m * 0.35f, saiLechVanToc_mps * 3.5f, true,
                      {kiVanTocPidMo, kiViTri_radpms, kpVanTocPidMo})
                : PidGains{_kiVanTocPidThuan, kiViTri_radpms, _kpVanTocPidThuan};
            const bool duDieuKienBuTrongTam =
                !dangDiChuyenDoc && std::fabs(trangThaiXe.vanTocDoc_mps) < nguongVanTocTinhBanh;
            const float buTrongTamDuKien_rad = duDieuKienBuTrongTam
                ? std::clamp(
                      _gocBuTrongTam_rad + kiViTri_radpms * saiLechViTri_m * _thoiGianVongVanToc_s,
                      -gocBuTrongTamToiDa_rad, gocBuTrongTamToiDa_rad)
                : _gocBuTrongTam_rad;
            const float gocChuaGioiHan_rad =
                heSoVongNgoai.heSoViPhan * heSoBamVanTocDiChuyen * saiLechVanToc_mps +
                heSoVongNgoai.heSoTiLe * saiLechViTri_m +
                buTrongTamDuKien_rad;
            const float gocDaGioiHan_rad = std::clamp(gocChuaGioiHan_rad, -gocDatToiDa_rad, gocDatToiDa_rad);
            if (duDieuKienBuTrongTam &&
                (gocChuaGioiHan_rad == gocDaGioiHan_rad || gocChuaGioiHan_rad * saiLechViTri_m < 0.0f)) {
                _gocBuTrongTam_rad = buTrongTamDuKien_rad;
            }
            _tichPhanVanToc_m = saiLechViTri_m;
            _gocDat_rad = _gocCanBang_rad + gocDaGioiHan_rad;
        } else {
            const float kpV = dungPidMo ? kpVanTocPidMo : _kpVanTocPidThuan;
            const float kiV = dungPidMo ? kiVanTocPidMo : _kiVanTocPidThuan;
            const float tichPhanDuKien_m = _tichPhanVanToc_m + saiLechVanToc_mps * _thoiGianVongVanToc_s;
            const float gocChuaGioiHan_rad = kpV * heSoBamVanTocDiChuyen * saiLechVanToc_mps + kiV * tichPhanDuKien_m;
            const float gocDaGioiHan_rad = std::clamp(gocChuaGioiHan_rad, -gocDatToiDa_rad, gocDatToiDa_rad);
            if (gocChuaGioiHan_rad == gocDaGioiHan_rad || gocChuaGioiHan_rad * saiLechVanToc_mps < 0.0f) {
                _tichPhanVanToc_m = tichPhanDuKien_m;
            }
            _gocDat_rad = _gocCanBang_rad + gocDaGioiHan_rad;
        }
        _thoiGianVongVanToc_s = 0;
    }

    const float vanTocToiDa_mps = static_cast<float>(tanSoStepToiDa_hz * DifferentialDrive::metersPerStep());
    const int huongBaoHoaVanToc = _vanTocLenh_mps >= vanTocToiDa_mps ? 1 :
        (_vanTocLenh_mps <= -vanTocToiDa_mps ? -1 : 0);
    const float giaTocLenh_mps2 = _boPidGoc.update(trangThaiXe.gocNghien_rad, trangThaiXe.tocDoNghien_radps,
                                              _gocDat_rad, buocThoiGian_s, dungPidMo, huongBaoHoaVanToc);
    if (!std::isfinite(giaTocLenh_mps2)) return false;
    const float vanTocLenhTruoc_mps = _vanTocLenh_mps;
    // Ở chế độ thử nhấc bánh không ma sát (Bench Mode), thêm lực hãm tỷ lệ vận tốc để bánh
    // quay tương ứng góc nghiêng và tự trở về 0 thay vì tích phân trôi kịch trần 10000 STEP/s.
    const float giaTocHieuDung_mps2 = _cheDoThuKhongMaSat
        ? (giaTocLenh_mps2 - heSoGiamChanVanTocBench_1ps * _vanTocLenh_mps)
        : giaTocLenh_mps2;
    _vanTocLenh_mps = std::clamp(_vanTocLenh_mps + giaTocHieuDung_mps2 * buocThoiGian_s, -vanTocToiDa_mps, vanTocToiDa_mps);
    _giaTocBanhDaLenh_mps2 = (_vanTocLenh_mps - vanTocLenhTruoc_mps) / buocThoiGian_s;

    float tocDoQuayDieuKhien = _tocDoQuayMucTieu_radps;
    const bool coLenhQuay = std::fabs(tocDoQuayYeuCau_radps) > 1e-4f || std::fabs(_tocDoQuayMucTieu_radps) > 1e-4f;
    if (coViTriChinhXac && _batVongNgoai && !_cheDoThuKhongMaSat && khoangCachTamHaiBanh_m > 1e-4f) {
        const float hieuViTriHienTai = trangThaiXe.viTriBanhPhai_m - trangThaiXe.viTriBanhTrai_m;
        if (!daDongBoHuongQuay || coLenhQuay) {
            hieuViTriHaiBanhMucTieu = hieuViTriHienTai;
            daDongBoHuongQuay = true;
        } else if (batGiuHuongQuay) {
            const float saiLechHuongQuay = std::clamp(
                (hieuViTriHaiBanhMucTieu - hieuViTriHienTai) / khoangCachTamHaiBanh_m,
                -gioiHanSaiLechHuongQuay, gioiHanSaiLechHuongQuay);
            const float tocDoQuayDoDuoc =
                (trangThaiXe.vanTocBanhPhai_mps - trangThaiXe.vanTocBanhTrai_mps) / khoangCachTamHaiBanh_m;
            const float tocDoQuayBuGiuHuong = std::clamp(
                kpGiuHuongQuay * saiLechHuongQuay - kdGiuHuongQuay * tocDoQuayDoDuoc,
                -gioiHanTocDoQuayGiuHuong, gioiHanTocDoQuayGiuHuong);
            tocDoQuayDieuKhien += tocDoQuayBuGiuHuong;
        }
    } else {
        daDongBoHuongQuay = false;
    }
    auto haiBanh = tronUuTienCanBang(_vanTocLenh_mps, tocDoQuayDieuKhien,
        khoangCachTamHaiBanh_m, vanTocToiDa_mps);

    // Khi dùng cấu hình TMC2209 + MT6701 (nguồn phản hồi là EncoderBanh), khép vòng vị trí/vận tốc
    // từng bánh trên ESP32 và ngắt an toàn nếu phát hiện kẹt rotor/mất bước kéo dài.
    if (trangThaiXe.nguonQuangDuong == NguonUocLuongQuangDuong::EncoderBanh && batVongKinBanhXeTmcMt) {
        TrangThaiVongKinBanhXe trangThaiVongKin{};
        if (!_boVongKinBanhXe.capNhat(
                haiBanh.trai_mps, haiBanh.phai_mps, trangThaiXe, buocThoiGian_s, vanTocToiDa_mps, trangThaiVongKin)) {
            return false;
        }
        haiBanh.trai_mps = trangThaiVongKin.vanTocLenhTrai_mps;
        haiBanh.phai_mps = trangThaiVongKin.vanTocLenhPhai_mps;
    }

    lenhBanh.tocDoBuocTrai_buocTrenGiay = DifferentialDrive::stepsPerSecond(haiBanh.trai_mps);
    lenhBanh.tocDoBuocPhai_buocTrenGiay = DifferentialDrive::stepsPerSecond(haiBanh.phai_mps);
    return std::isfinite(lenhBanh.tocDoBuocTrai_buocTrenGiay) &&
           std::isfinite(lenhBanh.tocDoBuocPhai_buocTrenGiay);
}
