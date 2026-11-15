#include "Icm20602Driver.h"
#include "driver/gpio.h"
#include "ThongKeMauImu.h"
namespace {
    int16_t ghepSo16Bit(uint8_t byteCao, uint8_t byteThap) {
        uint16_t giaTri = (static_cast<uint16_t>(byteCao) << 8) | byteThap;
        int32_t giaTriCoDau = giaTri;
        if (giaTri >= 0x8000u) giaTriCoDau -= 65536;
        return static_cast<int16_t>(giaTriCoDau);
    }
}
Icm20602Driver::Icm20602Driver(TwoWire& busI2c): _busI2c(busI2c){}

bool Icm20602Driver::ghiThanhGhi(uint8_t diaChi, uint8_t giaTri) {
    _thanhGhiCuoi = diaChi;
    _busI2c.beginTransmission(diaChiI2cIcm);
    _busI2c.write(diaChi);
    _busI2c.write(giaTri);
    _maLoiI2cCuoi = static_cast<IcmI2cStatus>(_busI2c.endTransmission());
    return _maLoiI2cCuoi == IcmI2cStatus::OK;
}

IcmI2cStatus Icm20602Driver::docNhieuByte(uint8_t diachiBatDau, uint8_t* duLieu, size_t soByte){
    const uint32_t batDau_us = micros();
    _thanhGhiCuoi = diachiBatDau;
    _giaoDichDocCuoi = {};
    _giaoDichDocCuoi.thanhGhi = diachiBatDau;
    _giaoDichDocCuoi.soByteYeuCau = static_cast<uint8_t>(soByte <= 255 ? soByte : 255);
    const auto ketThuc = [&](IcmI2cStatus trangThai) {
        _maLoiI2cCuoi = trangThai;
        _giaoDichDocCuoi.maLoi = static_cast<uint8_t>(trangThai);
        _giaoDichDocCuoi.thoiGian_us = static_cast<uint32_t>(micros() - batDau_us);
        if (trangThai != IcmI2cStatus::OK) _giaoDichDocLoiCuoi = _giaoDichDocCuoi;
        return trangThai;
    };
    if (duLieu == nullptr || soByte == 0 || soByte > 15) return ketThuc(IcmI2cStatus::INVALID_PARAM);
    _busI2c.beginTransmission(diaChiI2cIcm);
    _busI2c.write(diachiBatDau);
    uint8_t trangThaiTruyen = _busI2c.endTransmission(false);
    if (trangThaiTruyen != 0) return ketThuc(static_cast<IcmI2cStatus>(trangThaiTruyen));
    size_t daNhan = _busI2c.requestFrom(
        static_cast<uint8_t>(diaChiI2cIcm),
        soByte,
        true
    );
    _giaoDichDocCuoi.soByteNhan = static_cast<uint8_t>(daNhan <= 255 ? daNhan : 255);
    if (daNhan != soByte) return ketThuc(IcmI2cStatus::READ_SIZE_MISMATCH);
    for (size_t chiSoByte = 0; chiSoByte < soByte; ++chiSoByte) {
        int giaTriDoc = _busI2c.read();
        if (giaTriDoc < 0) {
            return ketThuc(IcmI2cStatus::READ_SIZE_MISMATCH);
        }
        duLieu[chiSoByte] = static_cast<uint8_t>(giaTriDoc);
        ++_giaoDichDocCuoi.soByteDaDoc;
    }
    return ketThuc(IcmI2cStatus::OK);
}

bool Icm20602Driver::begin() {
    _daKhoiTao = false;
    _daHieuChinhConQuay = false;
    _maNhanDang = 0;
    _maLoiI2cCuoi = IcmI2cStatus::OK;
    _giaoDichDocCuoi = _giaoDichDocLoiCuoi = {};
    for (float& doLech : _doLechConQuay_dps) doLech = 0.0f;
    if (phamViGiaTocIcm_g != 4 ||
        phamViTocDoGocIcm_dps != 500) {
        return false;
    }
    if (locConQuayIcm < 1 ||
        locConQuayIcm > 6) {
        return false;
    }
    const uint32_t tanSoLayMau_hz = tanSoLayMauIcm_hz;
    if (tanSoLayMau_hz == 0 || tanSoLayMau_hz > 1000 ||
        1000 % tanSoLayMau_hz != 0) {
        return false;
    }
    const uint32_t heSoChiaTanSo = 1000 / tanSoLayMau_hz - 1;
    if (heSoChiaTanSo > 255) {
        return false;
    }
    static_assert(diaChiI2cIcm == 0x68 || diaChiI2cIcm == 0x69, "Dia chi ICM20602 phai la 0x68/0x69");
    // Sơ đồ MKS nối CS và SA0 về ESP32; sơ đồ TMC_MT dùng GPIO8/9 cho encoder trái (CS/SA0 = -1).
    if (chanCsIcm >= 0 &&
        (gpio_set_level(static_cast<gpio_num_t>(chanCsIcm), 1) != ESP_OK ||
         gpio_set_direction(static_cast<gpio_num_t>(chanCsIcm), GPIO_MODE_OUTPUT) != ESP_OK)) return false;
    if (chanSa0Icm >= 0 &&
        (gpio_set_level(static_cast<gpio_num_t>(chanSa0Icm), diaChiI2cIcm == 0x69 ? 1 : 0) != ESP_OK ||
         gpio_set_direction(static_cast<gpio_num_t>(chanSa0Icm), GPIO_MODE_OUTPUT) != ESP_OK)) return false;
    if (!_busI2c.begin(chanSdaMpu, chanSclMpu)) {
        return false;
    }
    _busI2c.setClock(tanSoI2cIcm_hz);
    _busI2c.setTimeOut(thoiGianChoI2c_ms); // Timeout I2C thử ban đầu, đơn vị ms
    // WHO_AM_I: đọc một byte để kiểm tra đúng loại cảm biến.
    const IcmI2cStatus ketQuaDoc = docNhieuByte(0x75, &_maNhanDang, 1);
    if (ketQuaDoc != IcmI2cStatus::OK) {
        return false;
    }
    if (_maNhanDang != 0x12) {
        return false;
    }
    if (!ghiThanhGhi(0x6B, 0x80)) return false; // DEVICE_RESET
    delay(thoiGianKhoiDongMpu_ms);
    if (!ghiThanhGhi(0x6B, 0x01) || !ghiThanhGhi(0x6C, 0x00)) return false;
    delay(thoiGianKhoiDongMpu_ms);                               // Chỉ lúc khởi tạo
    if (!ghiThanhGhi(0x1A, locConQuayIcm))
        return false;                         // DLPF
    if (!ghiThanhGhi(0x19, static_cast<uint8_t>(heSoChiaTanSo)))
        return false;                         // Tần số mẫu
    if (!ghiThanhGhi(0x1B, 0x08))
        return false;                         // Gyro ±500 °/s
    if (!ghiThanhGhi(0x1C, 0x08))
        return false;                         // Accel ±4 g
    if (!ghiThanhGhi(0x37, cauHinhChanNgatIcm)) return false; // Latch, ACK bằng INT_STATUS.
    if (!ghiThanhGhi(0x38, batNgatMpu ? 0x01 : 0x00))
        return false;                     // Data-ready interrupt
    if (locGiaTocIcm > 6 || !ghiThanhGhi(0x1D, locGiaTocIcm)) return false;
    uint8_t cauHinh[5]{};
    if (docNhieuByte(0x19, cauHinh, sizeof(cauHinh)) != IcmI2cStatus::OK) return false;
    if (cauHinh[0] != static_cast<uint8_t>(heSoChiaTanSo) ||
        cauHinh[1] != locConQuayIcm ||
        cauHinh[2] != 0x08 ||   // Gyro ±500 °/s
        cauHinh[3] != 0x08 || cauHinh[4] != locGiaTocIcm) {  // Accel ±4 g
        return false;
    }
    _daKhoiTao = true;
    return true;
}
bool Icm20602Driver::read(KhungDuLieuImu& khungDuLieu) {
    // Mỗi lần gọi bắt đầu với một khungDuLieu không hợp lệ.
    khungDuLieu = KhungDuLieuImu{};
    if (!_daKhoiTao) return false;
    uint8_t boDem[14]{};
    // Đọc một khối: accel XYZ, nhiệt độ, gyro XYZ.
    if (docNhieuByte(0x3B, boDem, sizeof(boDem)) != IcmI2cStatus::OK) {
        khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::LoiIO;
        return false;
    }

    return giaiMaMau(boDem, khungDuLieu);
}

bool Icm20602Driver::docMauTheoNgat(KhungDuLieuImu& khungDuLieu) {
    khungDuLieu = KhungDuLieuImu{};
    if (!_daKhoiTao) return false;
    uint8_t boDem[15]{};
    if (docNhieuByte(0x3A, boDem, sizeof(boDem)) != IcmI2cStatus::OK) {
        khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::LoiIO;
        return false;
    }
    if ((boDem[0] & 0x01u) == 0) {
        khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::DuLieuCu;
        return false;
    }
    return giaiMaMau(boDem + 1, khungDuLieu);
}

bool Icm20602Driver::giaiMaMau(const uint8_t* boDem, KhungDuLieuImu& khungDuLieu) const {
    const int16_t giaTocThoX = ghepSo16Bit(boDem[0],  boDem[1]);
    const int16_t giaTocThoY = ghepSo16Bit(boDem[2],  boDem[3]);
    const int16_t giaTocThoZ = ghepSo16Bit(boDem[4],  boDem[5]);
    const int16_t nhietDoTho  = ghepSo16Bit(boDem[6],  boDem[7]);
    const int16_t tocDoGocThoX = ghepSo16Bit(boDem[8],  boDem[9]);
    const int16_t tocDoGocThoY = ghepSo16Bit(boDem[10], boDem[11]);
    const int16_t tocDoGocThoZ = ghepSo16Bit(boDem[12], boDem[13]);

    // Hệ số cho accel ±4 g.
    constexpr float heSoDoiGiaToc = 9.80665f / 8192.0f;

    // Hệ số cho gyro ±500 °/s.
    constexpr float soDonViThoMoiDoTrenGiay = 65.5f;
    constexpr float heSoDoiDoSangRad = 0.01745329252f;

    khungDuLieu.giaTocX_mps2 = giaTocThoX * heSoDoiGiaToc;
    khungDuLieu.giaTocY_mps2 = giaTocThoY * heSoDoiGiaToc;
    khungDuLieu.giaTocZ_mps2 = giaTocThoZ * heSoDoiGiaToc;
    khungDuLieu.tocDoGocX_radps =
        (tocDoGocThoX / soDonViThoMoiDoTrenGiay - _doLechConQuay_dps[0]) * heSoDoiDoSangRad;
    khungDuLieu.tocDoGocY_radps =
        (tocDoGocThoY / soDonViThoMoiDoTrenGiay - _doLechConQuay_dps[1]) * heSoDoiDoSangRad;
    khungDuLieu.tocDoGocZ_radps =
        (tocDoGocThoZ / soDonViThoMoiDoTrenGiay - _doLechConQuay_dps[2]) * heSoDoiDoSangRad;
    khungDuLieu.nhietDoCamBien_C = nhietDoTho / 326.8f + 25.0f;
    khungDuLieu.daHieuChinhConQuay = _daHieuChinhConQuay;
    khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::HopLe;
    return true;
}
bool Icm20602Driver::docMauMoi(KhungDuLieuImu& khungDuLieu) {
    khungDuLieu = KhungDuLieuImu{};
    if (!_daKhoiTao) return false;
    uint8_t trangThaiNgat = 0;
    if (docNhieuByte(0x3A, &trangThaiNgat, 1) != IcmI2cStatus::OK) {
        khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::LoiIO;
        return false;
    }
    if ((trangThaiNgat & 0x01u) == 0) {
        khungDuLieu.trangThai = TrangThaiKhungDuLieuImu::DuLieuCu;
        return false;
    }
    return read(khungDuLieu);
}

bool Icm20602Driver::kiemTraSauKhoiDong(BaoCaoKiemTraIcm& baoCao) {
    baoCao = BaoCaoKiemTraIcm{};
    const uint32_t batDau_ms = millis();
    const auto ketThuc = [&](const char* ketQua, bool dat = false) {
        baoCao.ketQua = ketQua;
        baoCao.thoiGian_ms = millis() - batDau_ms;
        return dat;
    };
    if (!_daKhoiTao) return ketThuc("chua_khoi_tao");
    if (docNhieuByte(0x75, &_maNhanDang, 1) != IcmI2cStatus::OK) return ketThuc("loi_i2c");
    if (_maNhanDang != 0x12) return ketThuc("sai_who_am_i");
    uint8_t cauHinh[5]{}, nguon[2]{}, ngat[2]{};
    if (docNhieuByte(0x19, cauHinh, 5) != IcmI2cStatus::OK ||
        docNhieuByte(0x6B, nguon, 2) != IcmI2cStatus::OK ||
        docNhieuByte(0x37, ngat, 2) != IcmI2cStatus::OK) return ketThuc("loi_i2c");
    if (cauHinh[0] != 1000u/tanSoLayMauIcm_hz - 1u || cauHinh[1] != locConQuayIcm ||
        cauHinh[2] != 0x08 || cauHinh[3] != 0x08 || cauHinh[4] != locGiaTocIcm ||
        nguon[0] != 0x01 || nguon[1] != 0 ||
        ngat[0] != cauHinhChanNgatIcm || ngat[1] != (batNgatMpu ? 1 : 0))
        return ketThuc("sai_cau_hinh");
    // INT_STATUS là read-to-clear: bỏ cờ tồn đọng rồi chờ các mẫu mới sau reset.
    uint8_t trangThaiNgat = 0;
    if (docNhieuByte(0x3A, &trangThaiNgat, 1) != IcmI2cStatus::OK) return ketThuc("loi_i2c");
    while (millis() - batDau_ms < thoiGianKiemTraKhoiDongIcm_ms) {
        KhungDuLieuImu mau{};
        if (docMauMoi(mau)) {
            baoCao.mauCuoi = mau;
            const float normGiaToc = normGiaTocImu(mau);
            // Giới hạn rộng, không phải kiểm tra tư thế hay độ yên của người giữ.
            if (mauImuHuuHan(mau) && normGiaToc >= 2.0f && normGiaToc <= 30.0f &&
                mau.nhietDoCamBien_C >= -45.0f && mau.nhietDoCamBien_C <= 90.0f) {
                ++baoCao.soMauTot;
                if (baoCao.soMauTot >= soMauKiemTraKhoiDongIcm) return ketThuc("PASS", true);
            } else {
                ++baoCao.soMauLoi;
                baoCao.soMauTot = 0; // Cần chuỗi mẫu tốt, không cộng dồn qua đoạn lỗi.
            }
        } else if (mau.trangThai == TrangThaiKhungDuLieuImu::LoiIO) {
            return ketThuc("loi_i2c");
        }
        delay(1);
    }
    return ketThuc(baoCao.soMauLoi ? "du_lieu_bat_thuong" : "khong_du_mau_moi");
}

void Icm20602Driver::datDoLechConQuay_dps(float bx, float by, float bz) {
    _doLechConQuay_dps[0] = bx;
    _doLechConQuay_dps[1] = by;
    _doLechConQuay_dps[2] = bz;
    _daHieuChinhConQuay = true;
}

void Icm20602Driver::layDoLechConQuay_dps(float& bx, float& by, float& bz, bool& daHieuChinh) const {
    bx = _doLechConQuay_dps[0];
    by = _doLechConQuay_dps[1];
    bz = _doLechConQuay_dps[2];
    daHieuChinh = _daHieuChinhConQuay;
}

bool Icm20602Driver::hieuChinhConQuay(uint16_t soMau) {
    if (!_daKhoiTao || soMau < 2) return false;
    _daHieuChinhConQuay = false;
    // read() cần trả gyro thô khi đang đo lại; không trừ bias của lượt trước.
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
                        for (unsigned truc = 0; truc < 3; ++truc)
                            _doLechConQuay_dps[truc] = thongKe.trungBinh(truc + 3) / 0.01745329252f;
                        _daHieuChinhConQuay = true;
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
