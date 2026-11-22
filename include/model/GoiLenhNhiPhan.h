#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

constexpr uint8_t dauKhungNhiPhan = 0xA5u;
constexpr uint8_t maLenhDiChuyenNhiPhan = 0x01u;
constexpr uint8_t maLenhPhimNhiPhan = 0x02u;
constexpr uint8_t maLenhWebKhaoSatNhiPhan = 0x03u;
constexpr int16_t thangDoLenhNhiPhan = 1000;

struct MucLenhWeb {
    int16_t forward_pct = 0, turn_pct = 0;
    uint8_t speed_pct = 0;
    bool valid = false;
};

struct GoiLenhWebKhaoSat {
    uint8_t dauKhung, maLenh;
    int16_t mucTienLui, mucReNgang;
    uint8_t phimHanhDong, reserved;
    int16_t forward_pct, turn_pct;
    uint8_t speed_pct, maKiemTra;
} __attribute__((packed));
static_assert(sizeof(GoiLenhWebKhaoSat) == 14, "Khung Web khao sat phai dung 14 byte");

struct GoiLenhNhiPhan {
    uint8_t dauKhung;
    uint8_t maLenh;
    int16_t mucTienLui;
    int16_t mucReNgang;
    uint8_t phimHanhDong;
    uint8_t maKiemTra;
} __attribute__((packed));

static_assert(sizeof(GoiLenhNhiPhan) == 8u, "GoiLenhNhiPhan phai dung 8 byte");

inline uint8_t tinhMaKiemTraNhiPhan(const uint8_t* duLieuBayByte, size_t doDai = 7u) {
    if (!duLieuBayByte) return 0u;
    uint8_t maXor = 0u;
    for (size_t chiSo = 0u; chiSo < doDai; ++chiSo) {
        maXor ^= duLieuBayByte[chiSo];
    }
    return maXor;
}

inline float noiSuyTangTocKeo(float giaTriKeo, float vungChet = 0.06f, float heSoCong = 0.35f) {
    if (!std::isfinite(giaTriKeo)) return 0.0f;
    const float giaTriKep = std::clamp(giaTriKeo, -1.0f, 1.0f);
    const float doLonTho = std::fabs(giaTriKep);
    const float vungChetHopLe = std::clamp(vungChet, 0.0f, 0.9f);
    const float heSoCongHopLe = std::clamp(heSoCong, 0.0f, 1.0f);
    if (doLonTho <= vungChetHopLe) return 0.0f;
    const float tyLe = (doLonTho - vungChetHopLe) / (1.0f - vungChetHopLe);
    const float doLon = (1.0f - heSoCongHopLe) * tyLe + heSoCongHopLe * tyLe * tyLe;
    return giaTriKep >= 0.0f ? doLon : -doLon;
}

inline GoiLenhNhiPhan dongGoiLenhNhiPhan(float vanTocTienLui,
                                        float tocDoQuayNgang,
                                        float vanTocToiDa,
                                        float tocDoQuayToiDa,
                                        uint8_t phimKemTheo = 0u,
                                        uint8_t loaiLenh = maLenhDiChuyenNhiPhan) {
    GoiLenhNhiPhan goi{};
    goi.dauKhung = dauKhungNhiPhan;
    goi.maLenh = loaiLenh;
    const float tyLeTien = (std::isfinite(vanTocTienLui) && std::isfinite(vanTocToiDa) && vanTocToiDa > 0.0f)
        ? std::clamp(vanTocTienLui / vanTocToiDa, -1.0f, 1.0f)
        : 0.0f;
    const float tyLeQuay = (std::isfinite(tocDoQuayNgang) && std::isfinite(tocDoQuayToiDa) && tocDoQuayToiDa > 0.0f)
        ? std::clamp(tocDoQuayNgang / tocDoQuayToiDa, -1.0f, 1.0f)
        : 0.0f;
    goi.mucTienLui = static_cast<int16_t>(std::lround(tyLeTien * static_cast<float>(thangDoLenhNhiPhan)));
    goi.mucReNgang = static_cast<int16_t>(std::lround(tyLeQuay * static_cast<float>(thangDoLenhNhiPhan)));
    goi.phimHanhDong = phimKemTheo;
    uint8_t boDemByte[sizeof(GoiLenhNhiPhan)]{};
    std::memcpy(boDemByte, &goi, sizeof(GoiLenhNhiPhan));
    goi.maKiemTra = tinhMaKiemTraNhiPhan(boDemByte);
    return goi;
}

inline bool giaiMaGoiLenhNhiPhan(const uint8_t* duLieu,
                                 size_t doDai,
                                 float vanTocToiDa,
                                 float tocDoQuayToiDa,
                                 float& vanTocRa,
                                 float& tocDoQuayRa,
                                 char& phimRa,
                                 bool& coDiChuyenRa,
                                 MucLenhWeb* mucWebRa = nullptr) {
    if (mucWebRa) *mucWebRa = {};
    if (!duLieu || (doDai != sizeof(GoiLenhNhiPhan) && doDai != sizeof(GoiLenhWebKhaoSat))) return false;
    if (!std::isfinite(vanTocToiDa) || vanTocToiDa <= 0.0f ||
        !std::isfinite(tocDoQuayToiDa) || tocDoQuayToiDa <= 0.0f) return false;
    if (duLieu[0] != dauKhungNhiPhan) return false;
    const bool laWebKhaoSat = doDai == sizeof(GoiLenhWebKhaoSat);
    if (laWebKhaoSat) {
        if (duLieu[1] != maLenhWebKhaoSatNhiPhan ||
            tinhMaKiemTraNhiPhan(duLieu, doDai - 1) != duLieu[doDai - 1]) return false;
        GoiLenhWebKhaoSat web{};
        std::memcpy(&web, duLieu, sizeof(web));
        if (web.reserved || web.forward_pct < -100 || web.forward_pct > 100 ||
            web.turn_pct < -100 || web.turn_pct > 100 || web.speed_pct > 100 ||
            web.mucTienLui < -thangDoLenhNhiPhan || web.mucTienLui > thangDoLenhNhiPhan ||
            web.mucReNgang < -thangDoLenhNhiPhan || web.mucReNgang > thangDoLenhNhiPhan ||
            web.phimHanhDong) return false;
        if (mucWebRa) *mucWebRa = {web.forward_pct, web.turn_pct, web.speed_pct, true};
    } else {
        if (duLieu[1] != maLenhDiChuyenNhiPhan && duLieu[1] != maLenhPhimNhiPhan) return false;
        if (tinhMaKiemTraNhiPhan(duLieu) != duLieu[7]) return false;
    }

    GoiLenhNhiPhan goiNhan{};
    std::memcpy(&goiNhan, duLieu, sizeof(GoiLenhNhiPhan));
    const int16_t mucTien = std::clamp(
        goiNhan.mucTienLui, static_cast<int16_t>(-thangDoLenhNhiPhan), thangDoLenhNhiPhan);
    const int16_t mucRe = std::clamp(
        goiNhan.mucReNgang, static_cast<int16_t>(-thangDoLenhNhiPhan), thangDoLenhNhiPhan);
    vanTocRa = (static_cast<float>(mucTien) / static_cast<float>(thangDoLenhNhiPhan)) * vanTocToiDa;
    tocDoQuayRa = (static_cast<float>(mucRe) / static_cast<float>(thangDoLenhNhiPhan)) * tocDoQuayToiDa;
    phimRa = static_cast<char>(goiNhan.phimHanhDong);
    coDiChuyenRa = laWebKhaoSat || (goiNhan.maLenh == maLenhDiChuyenNhiPhan);
    return true;
}
