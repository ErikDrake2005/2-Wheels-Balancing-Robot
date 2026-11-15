#include "AnglePidController.h"
#include "config/ControlConfig.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
float membership(float giaTriChuanHoa, int chiSoTapMo) {
    // NB, NS, Z, PS, PB: tâm -1, -0.5, 0, 0.5, 1; hai đầu dạng vai nhờ clamp đầu vào [-1, 1].
    const float tamTapMo = (chiSoTapMo - 2) * 0.5f;
    return std::max(0.0f, 1.0f - std::fabs(giaTriChuanHoa - tamTapMo) * 2.0f);
}

// ============================================================================
// 1. MÔ HÌNH MAMDANI CHUẨN (AND = MIN, Implication = MIN, Aggregation = MAX, Defuzz = Centroid/CoG)
// ============================================================================
// Năm tập mờ ngôn ngữ đầu ra có hàm liên thuộc hình tam giác cân đối xứng:
// - Chỉ số 0 (S / Z) : tâm chuẩn hóa c_0 = 0.00 (ứng với Kp=1.00*Kp0, Kd=1.000*Kd0, Ki=0.00*Ki0)
// - Chỉ số 1 (MS / S): tâm chuẩn hóa c_1 = 0.25 (ứng với Kp=1.10*Kp0, Kd=1.125*Kd0, Ki=0.25*Ki0)
// - Chỉ số 2 (M)     : tâm chuẩn hóa c_2 = 0.50 (ứng với Kp=1.20*Kp0, Kd=1.250*Kd0, Ki=0.50*Ki0)
// - Chỉ số 3 (MB)    : tâm chuẩn hóa c_3 = 0.75 (ứng với Kp=1.30*Kp0, Kd=1.375*Kd0, Ki=0.75*Ki0)
// - Chỉ số 4 (B)     : tâm chuẩn hóa c_4 = 1.00 (ứng với Kp=1.40*Kp0, Kd=1.500*Kd0, Ki=1.00*Ki0)
enum TapMoDauRa : uint8_t {
    S = 0,
    MS = 1,
    M = 2,
    MB = 3,
    B = 4,
    Z = 0
};

// Hàng: sai lệch góc E (NB, NS, Z, PS, PB); Cột: tốc độ góc EC (NB, NS, Z, PS, PB).
constexpr uint8_t bangLuatMamdaniTiLe[5][5] = {
    {B,  MB, M,  MS, S },
    {MB, M,  MS, S,  S },
    {MS, MS, S,  MS, MS},
    {S,  S,  MS, M,  MB},
    {S,  MS, M,  MB, B }
};

constexpr uint8_t bangLuatMamdaniViPhan[5][5] = {
    {MS, MS, S,  MB, B },
    {M,  MS, S,  MB, MB},
    {MB, M,  S,  M,  MB},
    {MB, MB, S,  MS, M },
    {B,  MB, S,  MS, MS}
};

constexpr uint8_t bangLuatMamdaniTichPhan[5][5] = {
    {Z,  Z,  Z,  Z,  Z },
    {Z,  MS, M,  MS, Z },
    {Z,  M,  B,  M,  Z },
    {Z,  MS, M,  MS, Z },
    {Z,  Z,  Z,  Z,  Z }
};

// Giải mờ Trọng tâm (Centroid / Center of Gravity - CoG) cho hàm liên thuộc hợp Mamdani:
//   mu_agg(u) = max_{k=0..4} min(h_k, mu_out_k(u))
// với mu_out_k(u) = max(0, 1 - |u - 0.25*k| / 0.25) trên miền chuẩn hóa u thuộc [-0.25, +1.25].
// Lưới tích phân 61 nút cách đều du = 0.025 khớp chính xác mọi đỉnh và chân tam giác.
float giaiMoMamdaniCentroid(const float doCaoCatNgon[5], float giaTriToiThieu, float bienDo) {
    float tuSo = 0.0f;
    float mauSo = 0.0f;
    constexpr int soKhoang = 60;
    constexpr float buocU = 0.025f; // (1.25 - (-0.25)) / 60 = 0.025
    for (int chiSoNut = 1; chiSoNut < soKhoang; ++chiSoNut) {
        const float u = -0.25f + static_cast<float>(chiSoNut) * buocU;
        float muHop = 0.0f;
        for (int k = 0; k < 5; ++k) {
            if (doCaoCatNgon[k] <= muHop) continue;
            const float tamTapMo = 0.25f * static_cast<float>(k);
            const float muTamGiac = std::max(0.0f, 1.0f - std::fabs(u - tamTapMo) * 4.0f);
            const float muCatNgon = std::min(doCaoCatNgon[k], muTamGiac); // Implication: MIN
            if (muCatNgon > muHop) muHop = muCatNgon;                    // Aggregation: MAX
        }
        tuSo += u * muHop;
        mauSo += muHop;
    }
    const float uTrongTam = (mauSo > 0.0f) ? std::clamp(tuSo / mauSo, 0.0f, 1.0f) : 0.0f;
    return giaTriToiThieu + bienDo * uTrongTam;
}

// ============================================================================
// 2. MÔ HÌNH TAKAGI-SUGENO BẬC 0 (AND = Product, Defuzz = Weighted Average)
// ============================================================================
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
                                            PidGains heSoCoSo, KieuSuyLuanMo kieuMo) {
    PidGains heSoPid = heSoCoSo;
    if (!dungPidMo) return heSoPid;
    const float saiLechChuanHoa = std::clamp(saiLechGoc_rad / mienSaiLechMo_rad, -1.0f, 1.0f);
    const float tocDoChuanHoa = std::clamp(tocDoNghien_radps / mienTocDoMo_radps, -1.0f, 1.0f);

    float muSaiLech[5];
    float muTocDo[5];
    for (int i = 0; i < 5; ++i) {
        muSaiLech[i] = membership(saiLechChuanHoa, i);
        muTocDo[i] = membership(tocDoChuanHoa, i);
    }

    if (kieuMo == KieuSuyLuanMo::MamdaniCentroid) {
        float catNgonTiLe[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        float catNgonTichPhan[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        float catNgonViPhan[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        for (int chiSoHang = 0; chiSoHang < 5; ++chiSoHang) {
            if (muSaiLech[chiSoHang] <= 0.0f) continue;
            for (int chiSoCot = 0; chiSoCot < 5; ++chiSoCot) {
                if (muTocDo[chiSoCot] <= 0.0f) continue;
                // Toán tử giao Mamdani (AND = MIN):
                const float doKichHoatLuat = std::min(muSaiLech[chiSoHang], muTocDo[chiSoCot]);
                // Toán tử hợp Mamdani (Aggregation = MAX):
                const uint8_t tapTiLe = bangLuatMamdaniTiLe[chiSoHang][chiSoCot];
                const uint8_t tapTichPhan = bangLuatMamdaniTichPhan[chiSoHang][chiSoCot];
                const uint8_t tapViPhan = bangLuatMamdaniViPhan[chiSoHang][chiSoCot];
                if (doKichHoatLuat > catNgonTiLe[tapTiLe]) catNgonTiLe[tapTiLe] = doKichHoatLuat;
                if (doKichHoatLuat > catNgonTichPhan[tapTichPhan]) catNgonTichPhan[tapTichPhan] = doKichHoatLuat;
                if (doKichHoatLuat > catNgonViPhan[tapViPhan]) catNgonViPhan[tapViPhan] = doKichHoatLuat;
            }
        }

        // Giải mờ Trọng tâm (Centroid / Center of Gravity):
        heSoPid.heSoTiLe *= giaiMoMamdaniCentroid(catNgonTiLe, 1.00f, 0.40f);
        heSoPid.heSoTichPhan *= giaiMoMamdaniCentroid(catNgonTichPhan, 0.00f, 1.00f);
        heSoPid.heSoViPhan *= giaiMoMamdaniCentroid(catNgonViPhan, 1.00f, 0.50f);
        return heSoPid;
    }

    float tongHeSoTiLe = 0, tongHeSoTichPhan = 0, tongHeSoViPhan = 0, tongTrongSo = 0;
    for (int chiSoHang = 0; chiSoHang < 5; ++chiSoHang) {
        for (int chiSoCot = 0; chiSoCot < 5; ++chiSoCot) {
            const float trongSoLuat = muSaiLech[chiSoHang] * muTocDo[chiSoCot];
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

void AnglePidController::setHeSoPidThuan(const PidGains& heSo) {
    if (std::isfinite(heSo.heSoTiLe) && heSo.heSoTiLe >= 0.0f) _heSoPidThuan.heSoTiLe = heSo.heSoTiLe;
    if (std::isfinite(heSo.heSoTichPhan) && heSo.heSoTichPhan >= 0.0f) _heSoPidThuan.heSoTichPhan = heSo.heSoTichPhan;
    if (std::isfinite(heSo.heSoViPhan) && heSo.heSoViPhan >= 0.0f) _heSoPidThuan.heSoViPhan = heSo.heSoViPhan;
}

void AnglePidController::setHeSoCoSoMo(const PidGains& heSo) {
    if (std::isfinite(heSo.heSoTiLe) && heSo.heSoTiLe >= 0.0f) _heSoCoSo.heSoTiLe = heSo.heSoTiLe;
    if (std::isfinite(heSo.heSoTichPhan) && heSo.heSoTichPhan >= 0.0f) _heSoCoSo.heSoTichPhan = heSo.heSoTichPhan;
    if (std::isfinite(heSo.heSoViPhan) && heSo.heSoViPhan >= 0.0f) _heSoCoSo.heSoViPhan = heSo.heSoViPhan;
}

float AnglePidController::update(float gocNghien_rad, float tocDoNghien_radps, float gocMucTieu_rad, float buocThoiGian_s,
                                 bool dungPidMo, int huongBaoHoaVanToc) {
    if (!std::isfinite(gocNghien_rad) || !std::isfinite(tocDoNghien_radps) || !std::isfinite(gocMucTieu_rad) ||
        !std::isfinite(buocThoiGian_s) || buocThoiGian_s <= 0 || buocThoiGian_s > 0.02f ||
        huongBaoHoaVanToc < -1 || huongBaoHoaVanToc > 1) return std::numeric_limits<float>::quiet_NaN();
    const float tocDoDaLoc_radps = _locGyroD.capNhat(tocDoNghien_radps, buocThoiGian_s, tanSoLocGyroD_hz);
    const float saiLechGoc_rad = gocNghien_rad - gocMucTieu_rad;
    _heSoPid = dungPidMo
        ? scheduledGains(saiLechGoc_rad, tocDoDaLoc_radps, true, _heSoCoSo)
        : _heSoPidThuan;
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
