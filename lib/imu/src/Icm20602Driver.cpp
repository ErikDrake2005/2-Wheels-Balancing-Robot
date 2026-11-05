#include "Icm20602Driver.h"
#include "driver/gpio.h"
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
    _busI2c.beginTransmission(diaChiI2cIcm);
    _busI2c.write(diaChi);
    _busI2c.write(giaTri);
    return _busI2c.endTransmission() == 0;
}

IcmI2cStatus Icm20602Driver::docNhieuByte(uint8_t diachiBatDau, uint8_t* duLieu, size_t soByte){
    if (duLieu == nullptr || soByte == 0 || soByte > 14) return IcmI2cStatus::INVALID_PARAM;
    _busI2c.beginTransmission(diaChiI2cIcm);
    _busI2c.write(diachiBatDau);
    uint8_t trangThaiTruyen = _busI2c.endTransmission(false);
    if (trangThaiTruyen != 0) return static_cast<IcmI2cStatus>(trangThaiTruyen);
    size_t daNhan = _busI2c.requestFrom(
        static_cast<uint8_t>(diaChiI2cIcm),
        soByte,
        true
    );
    if (daNhan != soByte) return IcmI2cStatus::READ_SIZE_MISMATCH;
    for (size_t chiSoByte = 0; chiSoByte < soByte; ++chiSoByte) {
        int giaTriDoc = _busI2c.read();
        if (giaTriDoc < 0) {
            return IcmI2cStatus::READ_SIZE_MISMATCH;
        }
        duLieu[chiSoByte] = static_cast<uint8_t>(giaTriDoc);
    }
    return IcmI2cStatus::OK;
}

bool Icm20602Driver::begin() {
    _daKhoiTao = false;
    _daHieuChinhConQuay = false;
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
    // Sơ đồ nối CS và SA0 về ESP32. Chốt chế độ I2C/địa chỉ trước khi mở bus.
    if (gpio_set_level(static_cast<gpio_num_t>(chanCsIcm), 1) != ESP_OK ||
        gpio_set_direction(static_cast<gpio_num_t>(chanCsIcm), GPIO_MODE_OUTPUT) != ESP_OK ||
        gpio_set_level(static_cast<gpio_num_t>(chanSa0Icm), diaChiI2cIcm == 0x69 ? 1 : 0) != ESP_OK ||
        gpio_set_direction(static_cast<gpio_num_t>(chanSa0Icm), GPIO_MODE_OUTPUT) != ESP_OK) return false;
    if (!_busI2c.begin(chanSdaMpu, chanSclMpu)) {
        return false;
    }
    _busI2c.setClock(tanSoI2cIcm_hz);
    _busI2c.setTimeOut(thoiGianChoI2c_ms); // Timeout I2C thử ban đầu, đơn vị ms
    // WHO_AM_I: đọc một byte để kiểm tra đúng loại cảm biến.
    uint8_t maNhanDang = 0;
    const IcmI2cStatus ketQuaDoc = docNhieuByte(0x75, &maNhanDang, 1);
    if (ketQuaDoc != IcmI2cStatus::OK) {
        return false;
    }
    if (maNhanDang != 0x12) {
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
    const uint8_t cauHinhChanNgat = mucNgatMpuCao ? 0x00 : 0x80;
    if (!ghiThanhGhi(0x37, cauHinhChanNgat)) return false;                   // INT active HIGH
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
    uint8_t trangThaiNgat = 0;
    if (!_daKhoiTao || docNhieuByte(0x3A, &trangThaiNgat, 1) != IcmI2cStatus::OK ||
        (trangThaiNgat & 0x01u) == 0) return false;
    return read(khungDuLieu);
}

bool Icm20602Driver::hieuChinhConQuay(uint16_t soMau) {
    // Cần ít nhất 2 mẫu để đánh giá độ biến thiên.
    if (!_daKhoiTao || soMau < 2) return false;
    _daHieuChinhConQuay = false;
    // Tần số lấy mẫu ICM20602, đơn vị mẫu/giây.
    const uint32_t tanSoLayMau_hz = tanSoLayMauIcm_hz;
    // Thời gian dự kiến thu đủ mẫu, làm tròn lên millisecond.
    const uint32_t thoiGianThuMau_ms = (static_cast<uint32_t>(soMau) * 1000u + tanSoLayMau_hz - 1u) / tanSoLayMau_hz;
    // Cộng thời gian dự phòng từ Config.
    const uint32_t thoiGianChoToiDa_ms = thoiGianThuMau_ms + duPhongThoiGianHieuChinh_ms;
    // Thời điểm bắt đầu để kiểm tra timeout trong vòng while.
    const uint32_t thoiDiemBatDau_ms = millis();
    // trungBinh: trung bình gyro theo độ/giây.
    // tongBinhPhuongLech: giá trị tích lũy để tính phương sai.
    float trungBinh[3]{};
    float tongBinhPhuongLech[3]{};
    uint16_t soMauDaThu = 0;
    // Đọc INT_STATUS để xóa sự kiện đã có trước khi hiệu chuẩn.
    uint8_t trangThaiNgat = 0;
    if (docNhieuByte(0x3A, &trangThaiNgat, 1) != IcmI2cStatus::OK) return false;
    while (soMauDaThu < soMau) {
        if (static_cast<uint32_t>(millis() - thoiDiemBatDau_ms) >= thoiGianChoToiDa_ms) return false;
        // Bit 0 của INT_STATUS báo có dữ liệu mới.
        // Đọc thanh ghi này cũng xóa trạng thái ngắt.
        if (docNhieuByte(0x3A, &trangThaiNgat, 1) != IcmI2cStatus::OK) return false;
        if ((trangThaiNgat & 0x01u) == 0) {
            delay(1);
            continue;
        }
        // Khi hiệu chuẩn chỉ cần 6 byte gyro từ GYRO_XOUT_H.
        uint8_t boDem[6]{};

        if (docNhieuByte(0x43, boDem, sizeof(boDem)) != IcmI2cStatus::OK) return false;
        const float tocDoGoc_dps[3] = {
            ghepSo16Bit(boDem[0], boDem[1]) / 65.5f,
            ghepSo16Bit(boDem[2], boDem[3]) / 65.5f,
            ghepSo16Bit(boDem[4], boDem[5]) / 65.5f
        };
        ++soMauDaThu;
        // Cập nhật trung bình và độ biến thiên từng trục.
        for (uint8_t chiSoTruc = 0; chiSoTruc < 3; ++chiSoTruc) {
            const float saiLechTruoc = tocDoGoc_dps[chiSoTruc] - trungBinh[chiSoTruc];
            trungBinh[chiSoTruc] += saiLechTruoc / static_cast<float>(soMauDaThu);
            const float doLechSau = tocDoGoc_dps[chiSoTruc] - trungBinh[chiSoTruc];
            tongBinhPhuongLech[chiSoTruc] += saiLechTruoc * doLechSau;
        }
    }
    const float gioiHanPhuongSai =doLechChuanConQuayToiDa_dps * doLechChuanConQuayToiDa_dps;
    // Nếu gyro biến động lớn, không chấp nhận bias vừa đo.
    for (uint8_t chiSoTruc = 0; chiSoTruc < 3; ++chiSoTruc) {
        const float phuongSai = tongBinhPhuongLech[chiSoTruc] / static_cast<float>(soMauDaThu - 1);
        if (phuongSai > gioiHanPhuongSai) return false;
    }
    // Chỉ ghi bias sau khi cả 3 trục đều vượt qua kiểm tra.
    for (uint8_t chiSoTruc = 0; chiSoTruc < 3; ++chiSoTruc) _doLechConQuay_dps[chiSoTruc] = trungBinh[chiSoTruc];
    _daHieuChinhConQuay = true;
    return true;
}
