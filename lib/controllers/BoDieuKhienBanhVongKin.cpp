#include "BoDieuKhienBanhVongKin.h"
#include "config/RobotGeometry.h"
#include <algorithm>
#include <cmath>

void BoDieuKhienBanhVongKin::reset() {
    _banhTrai = {};
    _banhPhai = {};
    _trangThai = {};
    _thoiGianLoiBam_s = 0.0f;
    _daDongBo = false;
}

float BoDieuKhienBanhVongKin::capNhatMotBanh(
    TrucBanh& truc, float vanTocDat_mps, float viTriDo_m, float vanTocDo_mps,
    float buocThoiGian_s, float vanTocToiDa_mps,
    float& saiLechViTri_m, float& saiLechGoc_rad, float& vanTocBu_mps) {
    truc.viTriThamChieu_m += vanTocDat_mps * buocThoiGian_s;
    const float saiLechTho_m = std::clamp(
        truc.viTriThamChieu_m - viTriDo_m,
        -gioiHanSaiLechViTriBanh_m, gioiHanSaiLechViTriBanh_m);
    // Giữ điểm tham chiếu trong miền chống windup vị trí quanh vị trí thực của encoder.
    truc.viTriThamChieu_m = viTriDo_m + saiLechTho_m;

    saiLechViTri_m = saiLechTho_m;
    saiLechGoc_rad = saiLechTho_m / banKinhBanh_m;

    const float saiLechVanToc_mps = vanTocDat_mps - vanTocDo_mps;
    const float tichPhanDuKien_m = truc.tichPhanVanToc_m + saiLechVanToc_mps * buocThoiGian_s;
    const float buChuaChan_mps =
        kpViTriBanh_1ps * saiLechViTri_m +
        kpVanTocBanh * saiLechVanToc_mps +
        kiVanTocBanh_1ps * tichPhanDuKien_m;
    const float buDaChan_mps = std::clamp(
        buChuaChan_mps, -gioiHanBuVanTocBanh_mps, gioiHanBuVanTocBanh_mps);

    const float lenhChuaChan_mps = vanTocDat_mps + buDaChan_mps;
    const float lenhDaChan_mps = std::clamp(lenhChuaChan_mps, -vanTocToiDa_mps, vanTocToiDa_mps);

    // Chỉ tích phân khi cả đầu ra bù và lệnh tổng chưa chạm trần, hoặc khi sai lệch giúp thoát trần.
    const bool khongBaoHoaBu = (buChuaChan_mps == buDaChan_mps) || (buChuaChan_mps * saiLechVanToc_mps < 0.0f);
    const bool khongBaoHoaTong = (lenhChuaChan_mps == lenhDaChan_mps) || (lenhChuaChan_mps * saiLechVanToc_mps < 0.0f);
    if (khongBaoHoaBu && khongBaoHoaTong) {
        truc.tichPhanVanToc_m = std::clamp(
            tichPhanDuKien_m, -gioiHanSaiLechViTriBanh_m, gioiHanSaiLechViTriBanh_m);
    }

    vanTocBu_mps = lenhDaChan_mps - vanTocDat_mps;
    return lenhDaChan_mps;
}

bool BoDieuKhienBanhVongKin::capNhat(
    float vanTocDatTrai_mps, float vanTocDatPhai_mps,
    const TrangThaiXe& trangThaiXe, float buocThoiGian_s,
    float vanTocToiDa_mps, TrangThaiVongKinBanhXe& ketQua) {
    ketQua = {};
    if (!trangThaiXe.viTriHopLe || !trangThaiXe.vanTocHopLe ||
        !std::isfinite(vanTocDatTrai_mps) || !std::isfinite(vanTocDatPhai_mps) ||
        !std::isfinite(trangThaiXe.viTriBanhTrai_m) || !std::isfinite(trangThaiXe.viTriBanhPhai_m) ||
        !std::isfinite(trangThaiXe.vanTocBanhTrai_mps) || !std::isfinite(trangThaiXe.vanTocBanhPhai_mps) ||
        !std::isfinite(buocThoiGian_s) || buocThoiGian_s <= 0.0f || buocThoiGian_s > 0.02f ||
        !std::isfinite(vanTocToiDa_mps) || vanTocToiDa_mps <= 0.0f) {
        reset();
        return false;
    }

    if (!_daDongBo) {
        _banhTrai.viTriThamChieu_m = trangThaiXe.viTriBanhTrai_m;
        _banhPhai.viTriThamChieu_m = trangThaiXe.viTriBanhPhai_m;
        _banhTrai.tichPhanVanToc_m = 0.0f;
        _banhPhai.tichPhanVanToc_m = 0.0f;
        _thoiGianLoiBam_s = 0.0f;
        _daDongBo = true;
    }

    ketQua.vanTocLenhTrai_mps = capNhatMotBanh(
        _banhTrai, vanTocDatTrai_mps, trangThaiXe.viTriBanhTrai_m, trangThaiXe.vanTocBanhTrai_mps,
        buocThoiGian_s, vanTocToiDa_mps,
        ketQua.saiLechViTriTrai_m, ketQua.saiLechGocTrai_rad, ketQua.vanTocBuTrai_mps);
    ketQua.vanTocLenhPhai_mps = capNhatMotBanh(
        _banhPhai, vanTocDatPhai_mps, trangThaiXe.viTriBanhPhai_m, trangThaiXe.vanTocBanhPhai_mps,
        buocThoiGian_s, vanTocToiDa_mps,
        ketQua.saiLechViTriPhai_m, ketQua.saiLechGocPhai_rad, ketQua.vanTocBuPhai_mps);

    const float saiLechGocLonNhat_rad = std::max(
        std::fabs(ketQua.saiLechGocTrai_rad), std::fabs(ketQua.saiLechGocPhai_rad));
    if (saiLechGocLonNhat_rad >= gocLoiBamBanhToiDa_rad) {
        _thoiGianLoiBam_s += buocThoiGian_s;
    } else {
        _thoiGianLoiBam_s = std::max(0.0f, _thoiGianLoiBam_s - buocThoiGian_s);
    }
    ketQua.biKetBanh = _thoiGianLoiBam_s >= thoiGianKetBanhToiDa_s;
    _trangThai = ketQua;
    return !ketQua.biKetBanh;
}
