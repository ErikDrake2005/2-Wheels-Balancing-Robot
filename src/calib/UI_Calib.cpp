#include "UI_Calib.h"
#include "HieuChuanQuayNgang.h"
#include "config/CalibConfig.h"
#include "config/ControlConfig.h"
#include "config/SafetyConfig.h"
#include "esp32-hal-rgb-led.h"
#include <cmath>

namespace {
constexpr uint32_t locDoiNut_ms = 35;
constexpr uint32_t thoiGianChoMau_ms = 2500;
constexpr uint32_t thoiGianThaoTac_ms = 30000;
constexpr unsigned soMauTinh = 80;

bool dangNhanNut() {
    return digitalRead(chanNutCalib) == (nutCalibMucThap ? LOW : HIGH);
}
Vec3Imu giaTocCua(const KhungDuLieuImu& mau) {
    return {mau.giaTocX_mps2, mau.giaTocY_mps2, mau.giaTocZ_mps2};
}
Vec3Imu gyroCua(const KhungDuLieuImu& mau) {
    return {mau.tocDoGocX_radps, mau.tocDoGocY_radps, mau.tocDoGocZ_radps};
}
bool mauDungYen(const KhungDuLieuImu& mau) {
    const float normGiaToc = doLonVecto(giaTocCua(mau));
    const float normGyro = doLonVecto(gyroCua(mau));
    return mau.daHieuChinhConQuay && std::fabs(normGiaToc - 9.80665f) <= 0.5f && normGyro <= 0.06f;
}
}

void UI_Calib::hienMau(uint8_t doLed, uint8_t xanhLa, uint8_t xanhDuong) const {
    if (chanRgbCalib >= 0) {
        rgbLedWriteOrdered(chanRgbCalib, thuTuRgbCalibLaGRB ? LED_COLOR_ORDER_GRB : LED_COLOR_ORDER_RGB,
                           doLed, xanhLa, xanhDuong);
    }
}
void UI_Calib::baoSanSang() const { hienMau(0, 32, 0); }
void UI_Calib::baoLoi() const { hienMau(40, 0, 0); }

void UI_Calib::dongBoNut() {
    _mucCu = _mucOnDinh = dangNhanNut();
    _lucDoiMuc_ms = millis();
    _daNhan = false;
}
void UI_Calib::khoiTao() {
    pinMode(chanNutCalib, nutCalibMucThap ? INPUT_PULLUP : INPUT_PULLDOWN);
    dongBoNut();
    hienMau(0, 0, 16);
}
bool UI_Calib::daXacNhan() {
    const bool muc = dangNhanNut();
    if (muc != _mucCu) {
        _mucCu = muc;
        _lucDoiMuc_ms = millis();
    }
    if (muc == _mucOnDinh || millis() - _lucDoiMuc_ms < locDoiNut_ms) return false;
    _mucOnDinh = muc;
    if (muc) {
        _daNhan = true;
        return false;
    }
    const bool xacNhan = _daNhan;
    _daNhan = false;
    return xacNhan;
}
void UI_Calib::choXacNhan() {
    dongBoNut();
    while (!daXacNhan()) delay(5);
}

bool UI_Calib::layGiaTocTinh(Vec3Imu& giaToc) {
    delay(150); // Đợi rung do nhả nút lắng xuống.
    const uint32_t batDau_ms = millis();
    double tong[3]{}, tongBinhPhuong = 0;
    unsigned soMau = 0;
    while (soMau < soMauTinh && millis() - batDau_ms < thoiGianChoMau_ms) {
        KhungDuLieuImu mau{};
        if (_camBien.docMauMoi(mau)) {
            if (!mauDungYen(mau)) return false;
            const Vec3Imu vecto = giaTocCua(mau);
            tong[0] += vecto.x;
            tong[1] += vecto.y;
            tong[2] += vecto.z;
            tongBinhPhuong += tichVoHuong(vecto, vecto);
            ++soMau;
        }
        delay(1);
    }
    if (soMau != soMauTinh) return false;
    const double x = tong[0]/soMau, y = tong[1]/soMau, z = tong[2]/soMau;
    if (tongBinhPhuong/soMau - (x*x+y*y+z*z) > 0.04) return false;
    giaToc = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    return true;
}

bool UI_Calib::choVeThang(const HeTrucImu& heTruc) {
    hienMau(24, 24, 24);
    Serial.printf("CALIB: giu yen gan goc dat khoi dong %.4f rad. Can tinh chinh goc can bang rieng.\n", gocCanBang_rad);
    const uint32_t batDau_ms = millis();
    uint32_t lanBaoCao_ms = batDau_ms;
    unsigned soMauThang = 0;
    while (millis() - batDau_ms < thoiGianThaoTac_ms) {
        KhungDuLieuImu mau{};
        if (_camBien.docMauMoi(mau)) {
            const Vec3Imu giaTocXe = heTruc.chuyenSangXe(giaTocCua(mau));
            const float pitch_rad = std::atan2(-giaTocXe.x, std::hypot(giaTocXe.y, giaTocXe.z));
            const float roll_rad = std::atan2(giaTocXe.y, giaTocXe.z);
            if (millis() - lanBaoCao_ms >= 500) {
                lanBaoCao_ms = millis();
                Serial.printf("CALIB_RETURN pitch=%.4f roll=%.4f target=%.4f rad\n", pitch_rad, roll_rad, gocCanBang_rad);
            }
            if (mauDungYen(mau) && std::fabs(pitch_rad - gocCanBang_rad) <= gocChoPhepBat_rad &&
                std::fabs(roll_rad) <= gocNgangChoPhepBat_rad) ++soMauThang;
            else soMauThang = 0;
            if (soMauThang >= soMauTinh) return true;
        }
        delay(1);
    }
    return false;
}

bool UI_Calib::thuMotLuot(MaTranQuayImu& ketQua) {
    hienMau(32, 20, 0);
    Serial.println("CALIB: dang do bias gyro, giu yen.");
    delay(150);
    Vec3Imu giaTocDung, giaTocTrai, giaTocPhai;
    if (!_camBien.hieuChinhConQuay(soMauHieuChinhMpu) || !layGiaTocTinh(giaTocDung)) return false;

    hienMau(0, 28, 28);
    Serial.println("CALIB: nghieng TRAI, giu yen, nhan roi nha nut.");
    choXacNhan();
    hienMau(32, 20, 0);
    if (!layGiaTocTinh(giaTocTrai)) return false;

    hienMau(32, 0, 28);
    Serial.println("CALIB: lat cham quanh truc doc tu TRAI sang PHAI, dung yen, nhan roi nha.");
    HieuChuanQuayNgang luotQuay;
    dongBoNut();
    const uint32_t batDau_ms = millis();
    uint32_t thoiDiemTruoc_us = micros();
    while (!daXacNhan()) {
        if (millis() - batDau_ms >= thoiGianThaoTac_ms) return false;
        KhungDuLieuImu mau{};
        if (_camBien.docMauMoi(mau)) {
            const uint32_t hienTai_us = micros();
            luotQuay.themMau(gyroCua(mau), (hienTai_us - thoiDiemTruoc_us) * 1e-6f);
            thoiDiemTruoc_us = hienTai_us;
        }
        delay(1);
    }
    hienMau(32, 20, 0);
    if (!layGiaTocTinh(giaTocPhai) || !luotQuay.tinhMaTran(giaTocDung, giaTocTrai, giaTocPhai, ketQua))
        return false;
    return choVeThang(HeTrucImu(ketQua));
}

void UI_Calib::chay(MaTranQuayImu& ketQua) {
    hienMau(0, 0, 36);
    Serial.println("CALIB: dat hai banh cung cao, giu khung gan thang, nhan roi nha. Day la tu the tham chieu.");
    choXacNhan();
    while (!thuMotLuot(ketQua)) {
        baoLoi();
        Serial.println("CALIB LOI: dat xe ve thang, giu yen va nhan nha de thu lai.");
        choXacNhan();
    }
}
