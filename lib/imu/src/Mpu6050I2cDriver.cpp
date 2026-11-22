#include "config/ImuProfile.h"
#if CAU_HINH_IMU == IMU_MPU6050_I2C
#include "Mpu6050I2cDriver.h"
#include "config/ImuConfig.h"
#include "driver/gpio.h"
#include "ThongKeMauImu.h"
#include <cmath>

namespace {
constexpr uint8_t regSmplrtDiv    = 0x19;
constexpr uint8_t regConfig       = 0x1A;
constexpr uint8_t regGyroConfig   = 0x1B;
constexpr uint8_t regAccelConfig  = 0x1C;
constexpr uint8_t regFifoEn       = 0x23;
constexpr uint8_t regIntPinCfg    = 0x37;
constexpr uint8_t regIntEnable    = 0x38;
constexpr uint8_t regIntStatus    = 0x3A;
constexpr uint8_t regAccelXoutH   = 0x3B;
constexpr uint8_t regSigPathReset = 0x68;
constexpr uint8_t regUserCtrl     = 0x6A;
constexpr uint8_t regPwrMgmt1     = 0x6B;
constexpr uint8_t regPwrMgmt2     = 0x6C;
constexpr uint8_t regWhoAmI       = 0x75;

int16_t ghepSo16Bit(uint8_t byteCao, uint8_t byteThap) {
    const uint16_t giaTri = (static_cast<uint16_t>(byteCao) << 8) | byteThap;
    return static_cast<int16_t>(giaTri < 0x8000u
        ? static_cast<int32_t>(giaTri)
        : static_cast<int32_t>(giaTri) - 65536);
}
}

bool Mpu6050I2cDriver::ghiThanhGhi(uint8_t diaChi, uint8_t giaTri) {
    _thanhGhiCuoi = diaChi;
    _busI2c.beginTransmission(mpu6050::diaChiI2c);
    _busI2c.write(diaChi);
    _busI2c.write(giaTri);
    _maLoiI2cCuoi = static_cast<IcmI2cStatus>(_busI2c.endTransmission());
    return _maLoiI2cCuoi == IcmI2cStatus::OK;
}

IcmI2cStatus Mpu6050I2cDriver::docNhieuByte(uint8_t diaChiBatDau, uint8_t* duLieu, size_t soByte) {
    const uint32_t batDau_us = micros();
    _thanhGhiCuoi = diaChiBatDau;
    _giaoDichDocCuoi = {};
    _giaoDichDocCuoi.thanhGhi = diaChiBatDau;
    _giaoDichDocCuoi.soByteYeuCau = static_cast<uint8_t>(soByte <= 255 ? soByte : 255);
    const auto ketThuc = [&](IcmI2cStatus trangThai) {
        _maLoiI2cCuoi = trangThai;
        _giaoDichDocCuoi.maLoi = static_cast<uint8_t>(trangThai);
        _giaoDichDocCuoi.thoiGian_us = static_cast<uint32_t>(micros() - batDau_us);
        if (trangThai != IcmI2cStatus::OK) _giaoDichDocLoiCuoi = _giaoDichDocCuoi;
        return trangThai;
    };
    if (duLieu == nullptr || soByte == 0 || soByte > 15) return ketThuc(IcmI2cStatus::INVALID_PARAM);
    _busI2c.beginTransmission(mpu6050::diaChiI2c);
    _busI2c.write(diaChiBatDau);
    const uint8_t trangThaiTruyen = _busI2c.endTransmission(false);
    if (trangThaiTruyen != 0) return ketThuc(static_cast<IcmI2cStatus>(trangThaiTruyen));
    const size_t daNhan = _busI2c.requestFrom(mpu6050::diaChiI2c, soByte, true);
    _giaoDichDocCuoi.soByteNhan = static_cast<uint8_t>(daNhan <= 255 ? daNhan : 255);
    if (daNhan != soByte) return ketThuc(IcmI2cStatus::READ_SIZE_MISMATCH);
    for (size_t chiSo = 0; chiSo < soByte; ++chiSo) {
        const int giaTriDoc = _busI2c.read();
        if (giaTriDoc < 0) return ketThuc(IcmI2cStatus::READ_SIZE_MISMATCH);
        duLieu[chiSo] = static_cast<uint8_t>(giaTriDoc);
        ++_giaoDichDocCuoi.soByteDaDoc;
    }
    return ketThuc(IcmI2cStatus::OK);
}

bool Mpu6050I2cDriver::cauHinhHopLe() {
    if (docNhieuByte(regWhoAmI, &_maNhanDang, 1) != IcmI2cStatus::OK) return false;
    if (_maNhanDang != mpu6050::maNhanDang) {
        _maLoiI2cCuoi = IcmI2cStatus::OTHER_ERROR;
        _thanhGhiCuoi = regWhoAmI;
        return false;
    }
    uint8_t cauHinh[4]{}, ngat[2]{}, quanLy[3]{};
    if (docNhieuByte(regSmplrtDiv, cauHinh, sizeof(cauHinh)) != IcmI2cStatus::OK ||
        docNhieuByte(regIntPinCfg, ngat, sizeof(ngat)) != IcmI2cStatus::OK ||
        docNhieuByte(regUserCtrl, quanLy, sizeof(quanLy)) != IcmI2cStatus::OK) {
        return false;
    }
    if (cauHinh[0] != mpu6050::heSoChiaMau) { _maLoiI2cCuoi = IcmI2cStatus::OTHER_ERROR; _thanhGhiCuoi = regSmplrtDiv; return false; }
    if (cauHinh[1] != mpu6050::maDlpf)      { _maLoiI2cCuoi = IcmI2cStatus::OTHER_ERROR; _thanhGhiCuoi = regConfig; return false; }
    if (cauHinh[2] != mpu6050::maGyroFs)    { _maLoiI2cCuoi = IcmI2cStatus::OTHER_ERROR; _thanhGhiCuoi = regGyroConfig; return false; }
    if (cauHinh[3] != mpu6050::maAccelFs)   { _maLoiI2cCuoi = IcmI2cStatus::OTHER_ERROR; _thanhGhiCuoi = regAccelConfig; return false; }
    // Bảo vệ an toàn mạch: INT_PIN_CFG phải tắt I2C_BYPASS_EN (bit 1 = 0), USER_CTRL phải tắt I2C_MST_EN (= 0x00).
    if (ngat[0] != 0x00u)                   { _maLoiI2cCuoi = IcmI2cStatus::OTHER_ERROR; _thanhGhiCuoi = regIntPinCfg; return false; }
    if (ngat[1] != 0x01u)                   { _maLoiI2cCuoi = IcmI2cStatus::OTHER_ERROR; _thanhGhiCuoi = regIntEnable; return false; }
    if (quanLy[0] != 0x00u)                 { _maLoiI2cCuoi = IcmI2cStatus::OTHER_ERROR; _thanhGhiCuoi = regUserCtrl; return false; }
    if (quanLy[1] != 0x01u)                 { _maLoiI2cCuoi = IcmI2cStatus::OTHER_ERROR; _thanhGhiCuoi = regPwrMgmt1; return false; }
    if (quanLy[2] != 0x00u)                 { _maLoiI2cCuoi = IcmI2cStatus::OTHER_ERROR; _thanhGhiCuoi = regPwrMgmt2; return false; }
    return true;
}

bool Mpu6050I2cDriver::begin() {
    _daKhoiTao = false;
    _daHieuChinhConQuay = false;
    _maNhanDang = 0;
    _maLoiI2cCuoi = IcmI2cStatus::OK;
    _giaoDichDocCuoi = _giaoDichDocLoiCuoi = {};
    for (float& doLech : _doLechConQuay_dps) doLech = 0.0f;

    // Chốt cứng chân AD0 (GPIO7) trước khi mở bus I2C để địa chỉ 0x68/0x69 xác định tuyệt đối.
    if (mpu6050::chanAd0 >= 0) {
        const auto chanAd0Gpio = static_cast<gpio_num_t>(mpu6050::chanAd0);
        if (gpio_set_level(chanAd0Gpio, mpu6050::mucAd0Cao ? 1 : 0) != ESP_OK ||
            gpio_set_direction(chanAd0Gpio, GPIO_MODE_OUTPUT) != ESP_OK) {
            return false;
        }
        delay(2);
    }

    if (!_busI2c.begin(mpu6050::chanSda, mpu6050::chanScl)) {
        _maLoiI2cCuoi = IcmI2cStatus::OTHER_ERROR;
        return false;
    }
    _busI2c.setClock(mpu6050::tanSoI2c_hz);
    _busI2c.setTimeOut(mpu6050::thoiGianChoI2c_ms);

    if (docNhieuByte(regWhoAmI, &_maNhanDang, 1) != IcmI2cStatus::OK) return false;
    if (_maNhanDang != mpu6050::maNhanDang) {
        _maLoiI2cCuoi = IcmI2cStatus::OTHER_ERROR;
        _thanhGhiCuoi = regWhoAmI;
        return false;
    }

    // Reset toàn bộ thanh ghi nội bộ (RM-MPU-6000A-00 §4.28) và chờ bit DEVICE_RESET tự xóa.
    if (!ghiThanhGhi(regPwrMgmt1, 0x80u)) return false;
    const uint32_t batDauReset_ms = millis();
    uint8_t pwr1 = 0x80u;
    do {
        delay(2);
        if (docNhieuByte(regPwrMgmt1, &pwr1, 1) != IcmI2cStatus::OK) return false;
        if (millis() - batDauReset_ms >= mpu6050::timeoutReset_ms) {
            _maLoiI2cCuoi = IcmI2cStatus::TIMEOUT;
            _thanhGhiCuoi = regPwrMgmt1;
            return false;
        }
    } while (pwr1 & 0x80u);

    // Đánh thức chip với PLL khóa theo trục X Gyro (0x01), reset đường tín hiệu số cảm biến.
    if (!ghiThanhGhi(regPwrMgmt1, 0x01u)) return false;
    delay(15);
    if (!ghiThanhGhi(regSigPathReset, 0x07u)) return false;
    delay(mpu6050::thoiGianOnDinhReset_ms);
    if (!ghiThanhGhi(regPwrMgmt2, 0x00u)) return false;

    // Tắt hoàn toàn FIFO, I2C Master phụ (XCL/XDA) và cầu nối I2C_BYPASS_EN.
    if (!ghiThanhGhi(regFifoEn, 0x00u)) return false;
    if (!ghiThanhGhi(regUserCtrl, 0x00u)) return false;
    if (!ghiThanhGhi(regIntPinCfg, 0x00u)) return false;

    // Cấu hình bộ lọc chống alias DLPF_CFG=3 (~42/44 Hz), chia mẫu 200 Hz, ±500 dps, ±4 g.
    if (!ghiThanhGhi(regConfig, mpu6050::maDlpf)) return false;
    if (!ghiThanhGhi(regSmplrtDiv, mpu6050::heSoChiaMau)) return false;
    if (!ghiThanhGhi(regGyroConfig, mpu6050::maGyroFs)) return false;
    if (!ghiThanhGhi(regAccelConfig, mpu6050::maAccelFs)) return false;
    // Bật cờ DATA_RDY_EN trong thanh ghi trạng thái 0x3A để đọc đồng bộ dù chân INT để trống.
    if (!ghiThanhGhi(regIntEnable, 0x01u)) return false;

    if (!cauHinhHopLe()) return false;

    // Xóa cờ DATA_RDY tồn đọng đầu tiên sau cấu hình.
    uint8_t xoaNgat = 0;
    if (docNhieuByte(regIntStatus, &xoaNgat, 1) != IcmI2cStatus::OK) return false;

    _daKhoiTao = true;
    return true;
}

bool Mpu6050I2cDriver::giaiMaMau(const uint8_t* boDem, KhungDuLieuImu& khungDuLieu) const {
    const int16_t giaTocThoX = ghepSo16Bit(boDem[0],  boDem[1]);
    const int16_t giaTocThoY = ghepSo16Bit(boDem[2],  boDem[3]);
    const int16_t giaTocThoZ = ghepSo16Bit(boDem[4],  boDem[5]);
    const int16_t nhietDoTho = ghepSo16Bit(boDem[6],  boDem[7]);
    const int16_t tocDoGocThoX = ghepSo16Bit(boDem[8],  boDem[9]);
    const int16_t tocDoGocThoY = ghepSo16Bit(boDem[10], boDem[11]);
    const int16_t tocDoGocThoZ = ghepSo16Bit(boDem[12], boDem[13]);

    constexpr float heSoDoiGiaToc = 9.80665f / mpu6050::lsbMoiG;
    constexpr float heSoDoiDoSangRad = 0.01745329252f;

    khungDuLieu.giaTocX_mps2 = giaTocThoX * heSoDoiGiaToc;
    khungDuLieu.giaTocY_mps2 = giaTocThoY * heSoDoiGiaToc;
    khungDuLieu.giaTocZ_mps2 = giaTocThoZ * heSoDoiGiaToc;
    khungDuLieu.tocDoGocX_radps =
        (tocDoGocThoX / mpu6050::lsbMoiDps - _doLechConQuay_dps[0]) * heSoDoiDoSangRad;
    khungDuLieu.tocDoGocY_radps =
        (tocDoGocThoY / mpu6050::lsbMoiDps - _doLechConQuay_dps[1]) * heSoDoiDoSangRad;
    khungDuLieu.tocDoGocZ_radps =
        (tocDoGocThoZ / mpu6050::lsbMoiDps - _doLechConQuay_dps[2]) * heSoDoiDoSangRad;
    // Công thức nhiệt độ chuẩn MPU6050 (RM-MPU-6000A-00 §4.18): T = TEMP_OUT / 340 + 36.53.
    khungDuLieu.nhietDoCamBien_C = nhietDoTho / 340.0f + 36.53f;
    khungDuLieu.daHieuChinhConQuay = _daHieuChinhConQuay;
    khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::HopLe;
    return true;
}

bool Mpu6050I2cDriver::read(KhungDuLieuImu& khungDuLieu) {
    khungDuLieu = KhungDuLieuImu{};
    if (!_daKhoiTao) return false;
    uint8_t boDem[14]{};
    if (docNhieuByte(regAccelXoutH, boDem, sizeof(boDem)) != IcmI2cStatus::OK) {
        khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::LoiIO;
        return false;
    }
    return giaiMaMau(boDem, khungDuLieu);
}

bool Mpu6050I2cDriver::docMauMoi(KhungDuLieuImu& khungDuLieu) {
    khungDuLieu = KhungDuLieuImu{};
    if (!_daKhoiTao) return false;
    const uint32_t batDau_us = micros();
    uint8_t trangThaiNgat = 0;
    // Kiểm tra nhanh 1 byte INT_STATUS (~100 us ở 400 kHz): nếu chưa có mẫu mới thì trả về ngay,
    // nếu có DATA_RDY_INT (bit 0 = 1) thì đọc trọn khối 14 byte dữ liệu và ghi nhận tổng thời gian.
    if (docNhieuByte(regIntStatus, &trangThaiNgat, 1) != IcmI2cStatus::OK) {
        khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::LoiIO;
        return false;
    }
    if ((trangThaiNgat & 0x01u) == 0u) {
        khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::DuLieuCu;
        return false;
    }
    uint8_t boDem[14]{};
    if (docNhieuByte(regAccelXoutH, boDem, sizeof(boDem)) != IcmI2cStatus::OK) {
        khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::LoiIO;
        return false;
    }
    _giaoDichDocCuoi.thoiGian_us = static_cast<uint32_t>(micros() - batDau_us);
    return giaiMaMau(boDem, khungDuLieu);
}

bool Mpu6050I2cDriver::docMauTheoNgat(KhungDuLieuImu& khungDuLieu) {
    khungDuLieu = KhungDuLieuImu{};
    if (!_daKhoiTao) return false;
    uint8_t boDem[15]{};
    if (docNhieuByte(regIntStatus, boDem, sizeof(boDem)) != IcmI2cStatus::OK) {
        khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::LoiIO;
        return false;
    }
    if ((boDem[0] & 0x01u) == 0u) {
        khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::DuLieuCu;
        return false;
    }
    return giaiMaMau(boDem + 1, khungDuLieu);
}

bool Mpu6050I2cDriver::kiemTraSauKhoiDong(BaoCaoKiemTraIcm& baoCao) {
    baoCao = BaoCaoKiemTraIcm{};
    const uint32_t batDau_ms = millis();
    const auto ketThuc = [&](const char* ketQua, bool dat = false) {
        baoCao.ketQua = ketQua;
        baoCao.thoiGian_ms = millis() - batDau_ms;
        return dat;
    };
    if (!_daKhoiTao) return ketThuc("chua_khoi_tao");
    if (!cauHinhHopLe()) {
        if (_maLoiI2cCuoi == IcmI2cStatus::OTHER_ERROR && _thanhGhiCuoi == regWhoAmI)
            return ketThuc("sai_who_am_i");
        if (_maLoiI2cCuoi == IcmI2cStatus::OTHER_ERROR)
            return ketThuc("sai_cau_hinh");
        return ketThuc("loi_i2c");
    }

    uint8_t trangThaiNgat = 0;
    if (docNhieuByte(regIntStatus, &trangThaiNgat, 1) != IcmI2cStatus::OK) return ketThuc("loi_i2c");

    uint32_t mauDau_us = 0;
    while (millis() - batDau_ms < mpu6050::timeoutBoot_ms) {
        KhungDuLieuImu mau{};
        if (docMauMoi(mau)) {
            baoCao.mauCuoi = mau;
            const float normGiaToc = normGiaTocImu(mau);
            if (mauImuHuuHan(mau) && normGiaToc >= 2.0f && normGiaToc <= 30.0f &&
                mau.nhietDoCamBien_C >= -40.0f && mau.nhietDoCamBien_C <= 85.0f) {
                if (baoCao.soMauTot == 0) mauDau_us = micros();
                if (++baoCao.soMauTot >= mpu6050::soMauBoot) {
                    const uint32_t daTroi_us = micros() - mauDau_us;
                    const uint32_t duKien_us = (baoCao.soMauTot - 1u) * 1000000u / mpu6050::tanSoLayMau_hz;
                    // Kiểm tra tần số thực tế của chuỗi DATA_RDY quanh 200 Hz (±15%) vì không dùng chân INT.
                    if (daTroi_us < duKien_us * 85u / 100u || daTroi_us > duKien_us * 115u / 100u) {
                        return ketThuc("sai_nhip_mau");
                    }
                    return ketThuc("PASS", true);
                }
            } else {
                ++baoCao.soMauLoi;
                baoCao.soMauTot = 0;
            }
        } else if (mau.trangThai == TrangThaiKhungDuLieuImu::LoiIO) {
            return ketThuc("loi_i2c");
        }
        delay(1);
    }
    return ketThuc(baoCao.soMauLoi ? "du_lieu_bat_thuong" : "khong_du_mau_moi");
}

void Mpu6050I2cDriver::datDoLechConQuay_dps(float bx, float by, float bz) {
    _daHieuChinhConQuay = std::isfinite(bx) && std::isfinite(by) && std::isfinite(bz);
    _doLechConQuay_dps[0] = _daHieuChinhConQuay ? bx : 0.0f;
    _doLechConQuay_dps[1] = _daHieuChinhConQuay ? by : 0.0f;
    _doLechConQuay_dps[2] = _daHieuChinhConQuay ? bz : 0.0f;
}

void Mpu6050I2cDriver::layDoLechConQuay_dps(float& bx, float& by, float& bz, bool& daHieuChinh) const {
    bx = _doLechConQuay_dps[0];
    by = _doLechConQuay_dps[1];
    bz = _doLechConQuay_dps[2];
    daHieuChinh = _daHieuChinhConQuay;
}

bool Mpu6050I2cDriver::hieuChinhConQuay(uint16_t soMau) {
    if (!_daKhoiTao || soMau < 2) return false;
    _daHieuChinhConQuay = false;
    for (float& doLech : _doLechConQuay_dps) doLech = 0.0f;

    ThongKeMauImu thongKe;
    const uint32_t batDau_ms = millis();
    uint32_t lanCoMau_ms = batDau_ms, lanBaoCao_ms = batDau_ms;
    unsigned soLanThuLai = 0;
    while (millis() - batDau_ms < thoiGianChoOnDinhCalib_ms) {
        KhungDuLieuImu mau{};
        if (docMauMoi(mau)) {
            const uint32_t hienTai_ms = millis();
            if (hienTai_ms - lanCoMau_ms > khoangMauCalibToiDa_ms) thongKe.xoa();
            lanCoMau_ms = hienTai_ms;
            if (!mauImuGanTinh(mau, gioiHanGyroThoCalib_dps * 0.01745329252f)) {
                thongKe.xoa();
                ++soLanThuLai;
            } else {
                thongKe.themMau(mau);
                if (thongKe.soMau() >= soMau) {
                    if (thongKe.onDinh()) {
                        datDoLechConQuay_dps(
                            thongKe.trungBinh(3) / 0.01745329252f,
                            thongKe.trungBinh(4) / 0.01745329252f,
                            thongKe.trungBinh(5) / 0.01745329252f);
                        Serial.printf("CALIB_GYRO PASS bias_dps=%.4f,%.4f,%.4f retry=%u\n",
                            _doLechConQuay_dps[0], _doLechConQuay_dps[1], _doLechConQuay_dps[2], soLanThuLai);
                        return true;
                    }
                    Serial.printf("CALIB_WAIT gyro: rung; std_dps=%.3f var_acc=%.4f; tu thu lai\n",
                        thongKe.doLechChuanGyroLonNhat_dps(), thongKe.phuongSaiGiaToc());
                    thongKe.xoa();
                    ++soLanThuLai;
                }
            }
        } else if (mau.trangThai == TrangThaiKhungDuLieuImu::LoiIO) {
            Serial.printf("CALIB_GYRO FAIL loi_i2c=%u reg=0x%02X\n",
                static_cast<unsigned>(_maLoiI2cCuoi), _thanhGhiCuoi);
            return false;
        }
        if (millis() - lanCoMau_ms >= thoiGianMatMauCalib_ms) {
            Serial.println("CALIB_GYRO FAIL mat_mau_moi");
            return false;
        }
        if (millis() - lanBaoCao_ms >= 1000) {
            lanBaoCao_ms = millis();
            Serial.printf("CALIB_WAIT gyro: giu yen, mau=%u/%u retry=%u\n",
                thongKe.soMau(), soMau, soLanThuLai);
        }
        delay(1);
    }
    Serial.println("CALIB_GYRO FAIL het_30s_cho_on_dinh");
    return false;
}
#endif
