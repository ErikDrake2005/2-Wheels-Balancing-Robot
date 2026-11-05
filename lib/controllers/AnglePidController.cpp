#include "AnglePidController.h"
#include "config/ControlConfig.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
float membership(float giaTriChuanHoa, int chiSoTapMo) {
    // NB, NS, Z, PS, PB: tâm -1, -0.5, 0, 0.5, 1; hai đầu dạng vai.
    const float tamTapMo = (chiSoTapMo - 2) * 0.5f;
    return std::max(0.0f, 1.0f - std::fabs(giaTriChuanHoa - tamTapMo) * 2.0f);
}
// Sugeno bậc 0, phép AND tích, giải mờ trung bình có trọng số.
// Hàng: e; cột: tốc độ nghiêng. Bảng nhân hệ số cơ sở trong ControlConfig.
constexpr float bangHeSoTiLe[5][5] = {
    {1.40f,1.30f,1.20f,1.10f,1.00f}, {1.25f,1.15f,1.05f,1.00f,1.00f},
    {1.10f,1.05f,1.00f,1.05f,1.10f}, {1.00f,1.00f,1.05f,1.15f,1.25f},
    {1.00f,1.10f,1.20f,1.30f,1.40f}};
constexpr float bangHeSoViPhan[5][5] = {
    {1.10f,1.10f,1.00f,1.30f,1.50f}, {1.20f,1.10f,1.00f,1.30f,1.40f},
    {1.40f,1.20f,1.00f,1.20f,1.40f}, {1.40f,1.30f,1.00f,1.10f,1.20f},
    {1.50f,1.30f,1.00f,1.10f,1.10f}};
constexpr float bangHeSoTichPhan[5][5] = {
    {0,0,0,0,0}, {0,0.2f,0.5f,0.2f,0}, {0,0.5f,1,0.5f,0},
    {0,0.2f,0.5f,0.2f,0}, {0,0,0,0,0}};
}

PidGains AnglePidController::scheduledGains(float saiLechGoc_rad, float tocDoNghien_radps, bool dungPidMo,
                                          PidGains heSoCoSo) {
    PidGains heSoPid = heSoCoSo;
    if (!dungPidMo) return heSoPid;
    const float saiLechChuanHoa = std::clamp(saiLechGoc_rad / mienSaiLechMo_rad, -1.0f, 1.0f);
    const float tocDoChuanHoa = std::clamp(tocDoNghien_radps / mienTocDoMo_radps, -1.0f, 1.0f);
    float tongHeSoTiLe = 0, tongHeSoTichPhan = 0, tongHeSoViPhan = 0, tongTrongSo = 0;
    for (int chiSoHang = 0; chiSoHang < 5; ++chiSoHang) {
        for (int chiSoCot = 0; chiSoCot < 5; ++chiSoCot) {
            const float trongSoLuat = membership(saiLechChuanHoa, chiSoHang) *
                                     membership(tocDoChuanHoa, chiSoCot);
            tongHeSoTiLe += trongSoLuat * bangHeSoTiLe[chiSoHang][chiSoCot];
            tongHeSoTichPhan += trongSoLuat * bangHeSoTichPhan[chiSoHang][chiSoCot];
            tongHeSoViPhan += trongSoLuat * bangHeSoViPhan[chiSoHang][chiSoCot];
            tongTrongSo += trongSoLuat;
        }
    }
    if (tongTrongSo > 0) {
        heSoPid.heSoTiLe *= tongHeSoTiLe / tongTrongSo;
        heSoPid.heSoTichPhan *= tongHeSoTichPhan / tongTrongSo;
        heSoPid.heSoViPhan *= tongHeSoViPhan / tongTrongSo;
    }
    return heSoPid;
}

float AnglePidController::update(float gocNghien_rad, float tocDoNghien_radps, float gocMucTieu_rad, float buocThoiGian_s,
                                bool dungPidMo, int huongBaoHoaVanToc) {
    if (!std::isfinite(gocNghien_rad) || !std::isfinite(tocDoNghien_radps) || !std::isfinite(gocMucTieu_rad) ||
        !std::isfinite(buocThoiGian_s) || buocThoiGian_s <= 0 || buocThoiGian_s > 0.02f ||
        huongBaoHoaVanToc < -1 || huongBaoHoaVanToc > 1) return std::numeric_limits<float>::quiet_NaN();
    const float tocDoDaLoc_radps = _locGyroD.capNhat(tocDoNghien_radps, buocThoiGian_s, tanSoLocGyroD_hz);
    const float saiLechGoc_rad = gocNghien_rad - gocMucTieu_rad;
    _heSoPid = scheduledGains(saiLechGoc_rad, tocDoDaLoc_radps, dungPidMo, _heSoCoSo);
    const float tichPhanDuKien_rad_s = std::clamp(_tichPhanGoc_rad_s + saiLechGoc_rad*buocThoiGian_s,
        -gioiHanTichPhanGoc_rad_s, gioiHanTichPhanGoc_rad_s);
    const float giaTocChuaGioiHan_mps2 =
        _heSoPid.heSoTiLe * saiLechGoc_rad +
        _heSoPid.heSoTichPhan * tichPhanDuKien_rad_s +
        _heSoPid.heSoViPhan * tocDoDaLoc_radps;
    const float giaTocDaGioiHan_mps2 = std::clamp(giaTocChuaGioiHan_mps2, -giaTocCanBangToiDa_mps2, giaTocCanBangToiDa_mps2);
    if ((huongBaoHoaVanToc == 0 || saiLechGoc_rad * huongBaoHoaVanToc < 0) &&
        (giaTocChuaGioiHan_mps2 == giaTocDaGioiHan_mps2 ||
         saiLechGoc_rad * giaTocChuaGioiHan_mps2 < 0)) {
        _tichPhanGoc_rad_s = tichPhanDuKien_rad_s;
    }
    // D trên gyro: thay đổi góc đặt không tạo derivative kick.
    return std::clamp(
        _heSoPid.heSoTiLe * saiLechGoc_rad +
        _heSoPid.heSoTichPhan * _tichPhanGoc_rad_s +
        _heSoPid.heSoViPhan * tocDoDaLoc_radps,
        -giaTocCanBangToiDa_mps2, giaTocCanBangToiDa_mps2);
}
