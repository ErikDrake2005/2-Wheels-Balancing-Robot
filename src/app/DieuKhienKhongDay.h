#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <esp_now.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "model/GoiLenhNhiPhan.h"

struct GoiLenhEspNow {
    uint32_t maNhanDang;
    float vanTocTienLui;
    float tocDoQuayNgang;
    uint8_t maHanhDong;
} __attribute__((packed));

struct ThongTinTrangThaiWeb {
    bool daBat = false;
    bool dangChayPidMo = false;
    bool cheDoNangBanh = false;
    bool batVongViTri = true;
    bool daoChieuDongCo = false;
    bool gocHopLe = false;
    bool heThongSanSang = false;
    float gocNghienDo = 0.0f;
    float gocCanBangDo = 0.0f;
    float gocDatDo = 0.0f;
    float viTriMet = 0.0f;
    float vanTocMetGiay = 0.0f;
    float tocDoGocRadGiay = 0.0f;
    long long xungBanhTrai = 0;
    long long xungBanhPhai = 0;
    long long encoderTrai = 0;
    long long encoderPhai = 0;
    float kpGoc = 0.0f;
    float kiGoc = 0.0f;
    float kdGoc = 0.0f;
    float kpVanToc = 0.0f;
    float kiVanToc = 0.0f;
    float kpHieuDung = 0.0f;
    float kiHieuDung = 0.0f;
    float kdHieuDung = 0.0f;
    unsigned maLoiDongCo = 0;
    char tenLoi[24] = "none";
    char tenSanSang[24] = "none";
    char cheDoDieuKhien[16] = "PID_THUAN";
};

class DieuKhienKhongDay {
public:
    static constexpr size_t soDongNhatKyToiDa = 36u;
    static constexpr size_t doDaiDongNhatKyToiDa = 160u;

    using HamNhanDiChuyen = std::function<void(float vanTocTienLui, float tocDoQuayNgang)>;
    using HamNhanPhimLenh = std::function<void(char kyTuLenh)>;
    using HamNhanDongLenh = std::function<void(const char* dongLenh)>;
    using HamLayTrangThai = std::function<void(ThongTinTrangThaiWeb& trangThaiRa)>;
    using HamNhanChanDoan = std::function<bool(const uint8_t* mac, const uint8_t* data, size_t length)>;

    bool khoiTao(HamNhanDiChuyen hamDiChuyenMoi,
                 HamNhanPhimLenh hamPhimMoi,
                 HamNhanDongLenh hamDongMoi,
                 HamLayTrangThai hamTrangThaiMoi, HamNhanChanDoan hamChanDoanMoi = {});
    void xuLy();
    void ghiDongNhatKy(const char* noiDung);
    void ghiNhatKyDinhDang(const char* dinhDang, ...);

    bool wifiSanSang() const { return daBatWifi; }
    bool espNowSanSang() const { return daBatEspNow; }
    uint32_t tongSoGoiEspNow() const { return demGoiEspNow; }
    uint32_t tongSoGoiNhiPhan() const { return demGoiNhiPhan; }
    const char* diaChiIpAp() const { return chuoiDiaChiIp; }
    const char* diaChiMacXe() const { return chuoiDiaChiMac; }

private:
    static DieuKhienKhongDay* doiTuongHienTai;
    static void khiNhanDuLieuEspNow(const esp_now_recv_info_t* thongTinNhan, const uint8_t* duLieu, int doDaiDuLieu);

    void xuLyTrangChu();
    void xuLyApiDiChuyen();
    void xuLyApiNhiPhan();
    void xuLyApiLenh();
    void xuLyApiTrangThai();
    void xuLyApiNhatKy();
    void xuLyApiLenhNhatKy();
    void xuLyWebSocketNhiPhan();
    void xuLyUdpNhiPhan();
    bool apDungKhungNhiPhan(const uint8_t* duLieu, size_t doDai);
    void thucThiChuoiLenhWeb(const char* chuoiLenh);
    void dayDuLieuEspNow(const uint8_t* duLieu, int doDaiDuLieu);

    HamNhanDiChuyen hamDiChuyen;
    HamNhanPhimLenh hamPhimLenh;
    HamNhanDongLenh hamDongLenh;
    HamLayTrangThai hamLayTrangThai;
    HamNhanChanDoan hamNhanChanDoan;

    bool daBatWifi = false;
    bool daBatEspNow = false;
    uint32_t demGoiEspNow = 0;
    uint32_t demGoiNhiPhan = 0;
    char chuoiDiaChiIp[24] = "0.0.0.0";
    char chuoiDiaChiMac[24] = "00:00:00:00:00:00";

    portMUX_TYPE khoaDuLieuEspNow = portMUX_INITIALIZER_UNLOCKED;
    bool coLenhDiChuyenEspNow = false;
    float vanTocEspNowCho = 0.0f;
    float tocDoQuayEspNowCho = 0.0f;
    char phimHanhDongEspNowCho = '\0';
    bool coChuoiLenhEspNow = false;
    char boDemChuoiEspNow[48]{};
    char boDemDongNhatKy[soDongNhatKyToiDa][doDaiDongNhatKyToiDa]{};
    uint32_t tongSoDongNhatKy = 0u;
};
