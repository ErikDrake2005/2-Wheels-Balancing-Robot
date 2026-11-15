#include "UI_Calib.h"
#include "HieuChuanNghiengDoc.h"
#include "ThongKeMauImu.h"
#include "config/CalibConfig.h"
#include "config/ControlConfig.h"
#include "config/SafetyConfig.h"
#include "esp32-hal-rgb-led.h"
#include <cmath>

namespace {
constexpr uint32_t locDoiNut_ms = 35;
constexpr uint32_t thoiGianThaoTac_ms = 30000;
constexpr unsigned soMauTinh = 80;
constexpr float doiSangDo = 57.295779513f;

bool dangNhanNut() {
    const int chanKiemTra = chanNutCalibPhu >= 0 ? chanNutCalibPhu : chanNutCalib;
    return digitalRead(chanKiemTra) == (nutCalibMucThap ? LOW : HIGH);
}
Vec3Imu giaTocCua(const KhungDuLieuImu& mau) {
    return {mau.giaTocX_mps2, mau.giaTocY_mps2, mau.giaTocZ_mps2};
}
Vec3Imu gyroCua(const KhungDuLieuImu& mau) {
    return {mau.tocDoGocX_radps, mau.tocDoGocY_radps, mau.tocDoGocZ_radps};
}
bool mauDungYen(const KhungDuLieuImu& mau) {
    return mau.daHieuChinhConQuay && mauImuGanTinh(mau, tocDoGyroTinhCalibToiDa_radps);
}
}

void UI_Calib::hienMau(uint8_t doLed, uint8_t xanhLa, uint8_t xanhDuong) const {
    if (chanRgbCalib >= 0) {
        rgbLedWriteOrdered(chanRgbCalib, thuTuRgbCalibLaGRB ? LED_COLOR_ORDER_GRB : LED_COLOR_ORDER_RGB,
                           doLed, xanhLa, xanhDuong);
    }
}
void UI_Calib::baoSanSang() const { hienMau(0, 32, 0); }
void UI_Calib::baoChoDuLieu() const { hienMau(32, 8, 0); }
void UI_Calib::baoLoi() const { hienMau(40, 0, 0); }

void UI_Calib::dongBoNut() {
    _mucCu = _mucOnDinh = dangNhanNut();
    _lucDoiMuc_ms = millis();
    _daNhan = false;
}
void UI_Calib::khoiTao() {
    const int chanKiemTra = chanNutCalibPhu >= 0 ? chanNutCalibPhu : chanNutCalib;
    pinMode(chanKiemTra, nutCalibMucThap ? INPUT_PULLUP : INPUT_PULLDOWN);
    dongBoNut();
    hienMau(32, 8, 0); // Cam: đang khởi tạo/kiểm tra ICM, chưa nhận thao tác calib.
}
bool UI_Calib::daXacNhan() {
    while (Serial.available()) {
        const int kyTu = Serial.read();
        if (kyTu == '\n' || kyTu == '\r' || kyTu == 'c' || kyTu == 'C' || kyTu == 'y' || kyTu == 'Y') {
            return true;
        }
    }
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
    uint32_t lanNhac_ms = millis();
    while (!daXacNhan()) {
        if (millis() - lanNhac_ms >= 3000) {
            lanNhac_ms = millis();
            const int chanKiemTra = chanNutCalibPhu >= 0 ? chanNutCalibPhu : chanNutCalib;
            Serial.printf("CALIB_WAIT_CONFIRM: nhan nut GPIO%d (BOOT) hoac gui Enter/'c' qua Serial de xac nhan...\n", chanKiemTra);
        }
        delay(5);
    }
}

bool UI_Calib::kiemTraLoiDoc(const KhungDuLieuImu& mau, uint32_t lanCoMau_ms, const char* buoc) const {
    if (mau.trangThai == TrangThaiKhungDuLieuImu::LoiIO) {
        Serial.printf("CALIB FAIL buoc=%s loi_i2c=%u reg=0x%02X\n", buoc,
            static_cast<unsigned>(_camBien.maLoiI2cCuoi()), _camBien.thanhGhiCuoi());
        return false;
    }
    if (millis() - lanCoMau_ms >= thoiGianMatMauCalib_ms) {
        Serial.printf("CALIB FAIL buoc=%s mat_mau_moi\n", buoc);
        return false;
    }
    return true;
}

bool UI_Calib::layGiaTocTinh(Vec3Imu& giaToc, const char* buoc) {
    delay(thoiGianLangSauNhanNut_ms);
    const uint32_t batDau_ms = millis();
    uint32_t lanCoMau_ms = batDau_ms, lanBaoCao_ms = batDau_ms;
    ThongKeMauImu thongKe;
    unsigned soLanThuLai = 0;
    while (millis() - batDau_ms < thoiGianChoOnDinhCalib_ms) {
        KhungDuLieuImu mau{};
        if (_camBien.docMauMoi(mau)) {
            if (millis() - lanCoMau_ms > khoangMauCalibToiDa_ms) thongKe.xoa();
            lanCoMau_ms = millis();
            if (!mauDungYen(mau)) {
                thongKe.xoa();
                ++soLanThuLai;
            } else {
                thongKe.themMau(mau);
                if (thongKe.soMau() >= soMauTinh) {
                    if (thongKe.onDinh()) {
                        giaToc = {thongKe.trungBinh(0), thongKe.trungBinh(1), thongKe.trungBinh(2)};
                        Serial.printf("CALIB_STATIC PASS buoc=%s retry=%u\n", buoc, soLanThuLai);
                        return true;
                    }
                    Serial.printf("CALIB_WAIT %s: rung; std_dps=%.3f var_acc=%.4f; tu thu lai\n",
                        buoc, thongKe.doLechChuanGyroLonNhat_dps(), thongKe.phuongSaiGiaToc());
                    thongKe.xoa();
                    ++soLanThuLai;
                }
            }
        } else if (!kiemTraLoiDoc(mau, lanCoMau_ms, buoc)) return false;
        if (millis() - lanBaoCao_ms >= 1000) {
            lanBaoCao_ms = millis();
            Serial.printf("CALIB_WAIT %s: giu yen, mau=%u/%u retry=%u\n",
                buoc, thongKe.soMau(), soMauTinh, soLanThuLai);
        }
        delay(1);
    }
    Serial.printf("CALIB_STATIC FAIL buoc=%s het_30s_cho_on_dinh\n", buoc);
    return false;
}

bool UI_Calib::choVeThang(const HeTrucImu& heTruc) {
    hienMau(24, 24, 24);
    Serial.printf("CALIB: dung xe lai, hai banh cung cao. Goc khoi dong %.2f deg. DAU xuong=duong, DUOI xuong=am.\n",
        gocCanBang_rad * doiSangDo);
    const uint32_t batDau_ms = millis();
    uint32_t lanBaoCao_ms = batDau_ms - 500u; // In ngay mẫu đầu, kể cả khi qua bước trắng trước 500 ms.
    uint32_t lanCoMau_ms = batDau_ms;
    unsigned soMauThang = 0;
    while (millis() - batDau_ms < thoiGianThaoTac_ms) {
        KhungDuLieuImu mau{};
        if (_camBien.docMauMoi(mau)) {
            if (millis() - lanCoMau_ms > khoangMauCalibToiDa_ms) soMauThang = 0;
            lanCoMau_ms = millis();
            const Vec3Imu giaTocXe = heTruc.chuyenSangXe(giaTocCua(mau));
            const float gocDoc_rad = std::atan2(-giaTocXe.x, giaTocXe.z);
            const float gocNgang_rad = std::atan2(giaTocXe.y, std::hypot(giaTocXe.x,giaTocXe.z));
            if (millis() - lanBaoCao_ms >= 500) {
                lanBaoCao_ms = millis();
                Serial.printf("CALIB_RETURN doc=%.2f deg ngang=%.2f deg target=%.2f deg (doc_rad=%.4f)\n",
                    gocDoc_rad*doiSangDo, gocNgang_rad*doiSangDo, gocCanBang_rad*doiSangDo, gocDoc_rad);
            }
            if (mauDungYen(mau) && std::fabs(gocDoc_rad - gocCanBang_rad) <= gocChoPhepBat_rad &&
                std::fabs(gocNgang_rad) <= gocNgangChoPhepBat_rad) ++soMauThang;
            else soMauThang = 0;
            if (soMauThang >= soMauTinh) return true;
        } else if (!kiemTraLoiDoc(mau, lanCoMau_ms, "ve_goc_dat")) return false;
        delay(1);
    }
    Serial.println("CALIB_RETURN FAIL het_30s_cho_tu_the");
    return false;
}

bool UI_Calib::thuMotLuot(MaTranQuayImu& ketQua) {
    hienMau(32, 20, 0);
    Serial.println("CALIB: dang do bias gyro, giu yen.");
    delay(thoiGianLangSauNhanNut_ms);
    Vec3Imu giaTocDung, giaTocTruoc, giaTocSau;
    if (!_camBien.hieuChinhConQuay(soMauHieuChinhMpu) || !layGiaTocTinh(giaTocDung, "moc_dung")) return false;

    hienMau(0, 28, 28);
    Serial.println("CALIB: HA DAU XE xuong, dat nam tren mat do cung (~90 deg), DUOI huong len. Giu yen, nhan roi nha.");
    choXacNhan();
    hienMau(32, 20, 0);
    if (!layGiaTocTinh(giaTocTruoc, "moc_truoc")) return false;

    hienMau(32, 0, 28);
    Serial.println("CALIB: nang DAU qua tu the DUNG, roi HA DUOI nam phia sau (~90 deg), DAU huong len. Di cham 4-8s, giu yen, nhan nha.");
    HieuChuanNghiengDoc luotQuay;
    dongBoNut();
    const uint32_t batDau_ms = millis();
    uint32_t lanCoMau_ms = batDau_ms;
    uint32_t thoiDiemTruoc_us = micros();
    while (!daXacNhan()) {
        if (millis() - batDau_ms >= thoiGianThaoTac_ms) {
            Serial.println("CALIB_DOC FAIL het_30s_cho_xac_nhan");
            return false;
        }
        KhungDuLieuImu mau{};
        if (_camBien.docMauMoi(mau)) {
            if (millis() - lanCoMau_ms > khoangMauCalibToiDa_ms) {
                Serial.println("CALIB_DOC FAIL gian_doan_mau");
                return false;
            }
            lanCoMau_ms = millis();
            const uint32_t hienTai_us = micros();
            luotQuay.themMau(gyroCua(mau), (hienTai_us - thoiDiemTruoc_us) * 1e-6f);
            thoiDiemTruoc_us = hienTai_us;
        } else if (!kiemTraLoiDoc(mau, lanCoMau_ms, "dau_qua_dung_ve_duoi")) return false;
        delay(1);
    }
    hienMau(32, 20, 0);
    if (!layGiaTocTinh(giaTocSau, "moc_sau")) return false;
    BaoCaoCalibDoc baoCao;
    const bool dat = luotQuay.tinhMaTran(giaTocDung, giaTocTruoc, giaTocSau, ketQua, &baoCao);
    Serial.printf("CALIB_DOC %s truoc=%.2f deg sau=%.2f deg gyro=%.2f deg energy=%.4f vectorErr=%.2f deg\n",
        baoCao.lyDo, baoCao.gocMocTruoc_rad*doiSangDo, baoCao.gocMocSau_rad*doiSangDo,
        baoCao.gocQuayGyro_rad*doiSangDo, baoCao.tyLeNangLuongTruc, baoCao.lechVector_rad*doiSangDo);
    if (!dat) {
        Serial.printf("CALIB_MOC a_dung=%.4f,%.4f,%.4f a_truoc=%.4f,%.4f,%.4f a_sau=%.4f,%.4f,%.4f mps2\n",
            giaTocDung.x,giaTocDung.y,giaTocDung.z,giaTocTruoc.x,giaTocTruoc.y,giaTocTruoc.z,
            giaTocSau.x,giaTocSau.y,giaTocSau.z);
        return false;
    }
    for (unsigned truc = 0; truc < 3; ++truc)
        Serial.printf("CALIB_AXIS %c_sensor=%.6f,%.6f,%.6f\n", "XYZ"[truc],
            ketQua.phanTu[truc][0],ketQua.phanTu[truc][1],ketQua.phanTu[truc][2]);
    return choVeThang(HeTrucImu(ketQua));
}

void UI_Calib::chay(MaTranQuayImu& ketQua) {
    hienMau(0, 0, 36);
    Serial.println("CALIB: danh dau DAU XE la huong tien, DUOI doi dien. Khong lat sang hai ben banh.");
    Serial.println("CALIB: dung gan thang, hai banh cung cao, giu yen, nhan roi nha. Tu the nay KHONG duoc gan pitch=0.");
    choXacNhan();
    while (!thuMotLuot(ketQua)) {
        baoLoi();
        Serial.println("CALIB LOI: xem ly do Serial; dung xe, hai banh cung cao, giu yen va nhan nha de thu lai tu bias.");
        choXacNhan();
    }
}
