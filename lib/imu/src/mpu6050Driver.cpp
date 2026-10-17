#include "mpu6050Driver.h"
namespace {
    int16_t ghepSo16Bit(uint8_t byteCao, uint8_t byteThap) {
        uint16_t giaTri = (static_cast<uint16_t>(byteCao) << 8) | byteThap;
        int32_t giaTriCoDau = giaTri;
        if (giaTri >= 0x8000u) giaTriCoDau -= 65536;
        return static_cast<int16_t>(giaTriCoDau);
    }
}
mpu6050Drv::mpu6050Drv(TwoWire& bus): _bus(bus){}

bool mpu6050Drv::ghiThanhGhi(uint8_t diaChi, uint8_t giaTri) {
    _bus.beginTransmission(diaChiI2cMpu);
    _bus.write(diaChi);
    _bus.write(giaTri);
    return _bus.endTransmission() == 0;
}

I2C_Status mpu6050Drv::docNhieuByte(uint8_t diachiBatDau, uint8_t* duLieu, size_t soByte){
    if (duLieu == nullptr || soByte == 0 || soByte > 14) return I2C_Status::INVALID_PARAM;
    _bus.beginTransmission(diaChiI2cMpu);
    _bus.write(diachiBatDau);
    uint8_t txStatus = _bus.endTransmission(false);
    if (txStatus != 0) return static_cast<I2C_Status>(txStatus);
    size_t daNhan = _bus.requestFrom(
        static_cast<uint8_t>(diaChiI2cMpu),
        soByte,
        true
    );
    if (daNhan != soByte) return I2C_Status::READ_SIZE_MISMATCH;
    for (size_t i = 0; i < soByte; ++i) {
        int value = _bus.read();
        if (value < 0) {
            return I2C_Status::READ_SIZE_MISMATCH;
        }
        duLieu[i] = static_cast<uint8_t>(value);
    }
    return I2C_Status::OK;
}

bool mpu6050Drv::begin() {
    _daKhoiTao = false;
    _daHieuChinhConQuay = false;
    for (float& bias : _doLechConQuay_dps) bias = 0.0f;
    if (phamViGiaTocMpu_g != 4 ||
        phamViTocDoGocMpu_dps != 500) {
        return false;
    }
    if (cauHinhLocThongThapMpu < 1 ||
        cauHinhLocThongThapMpu > 6) {
        return false;
    }
    const uint32_t tanSoLayMau_hz = tanSoLayMauMpu_hz;
    if (tanSoLayMau_hz == 0 || tanSoLayMau_hz > 1000 ||
        1000 % tanSoLayMau_hz != 0) {
        return false;
    }
    const uint32_t divider = 1000 / tanSoLayMau_hz - 1;
    if (divider > 255) {
        return false;
    }
    if (!_bus.begin(chanSdaMpu, chanSclMpu)) {
        return false;
    }
    _bus.setClock(tanSoI2cMpu_hz);
    _bus.setTimeOut(thoiGianChoI2c_ms); // Timeout I2C thử ban đầu, đơn vị ms
    uint8_t id = 0;
    if (docNhieuByte(0x75, &id, 1) != I2C_Status::OK || id != 0x68) return false;
    if (!ghiThanhGhi(0x6B, 0x01)) return false; // Đánh thức MPU
    delay(thoiGianKhoiDongMpu_ms);                               // Chỉ lúc khởi tạo
    if (!ghiThanhGhi(0x1A, cauHinhLocThongThapMpu))
        return false;                         // DLPF
    if (!ghiThanhGhi(0x19, static_cast<uint8_t>(divider)))
        return false;                         // Tần số mẫu
    if (!ghiThanhGhi(0x1B, 0x08))
        return false;                         // Gyro ±500 °/s
    if (!ghiThanhGhi(0x1C, 0x08))
        return false;                         // Accel ±4 g
    const uint8_t intPinConfig = mucNgatMpuCao ? 0x00 : 0x80;
    if (!ghiThanhGhi(0x37, intPinConfig)) return false;                   // INT active HIGH
    if (!ghiThanhGhi(0x38, batNgatMpu ? 0x01 : 0x00))
        return false;                     // Data-ready interrupt
    uint8_t cauHinh[4]{};
    if (docNhieuByte(0x19, cauHinh, sizeof(cauHinh)) != I2C_Status::OK) return false;
    if (cauHinh[0] != static_cast<uint8_t>(divider) ||
        cauHinh[1] != cauHinhLocThongThapMpu ||
        cauHinh[2] != 0x08 ||   // Gyro ±500 °/s
        cauHinh[3] != 0x08) {  // Accel ±4 g
        return false;
    }
    _daKhoiTao = true;
    return true;
}
bool mpu6050Drv::read(KhungDuLieuMPU& khungDuLieu) {
    // Mỗi lần gọi bắt đầu với một khungDuLieu không hợp lệ.
    khungDuLieu = KhungDuLieuMPU{};
    if (!_daKhoiTao) return false;
    uint8_t boDem[14]{};
    // Đọc một khối: accel XYZ, nhiệt độ, gyro XYZ.
    if (docNhieuByte(0x3B, boDem, sizeof(boDem)) != I2C_Status::OK) {
        khungDuLieu.trangThai = TrangThaiKhungDuLieuMpu::LoiIO;
        return false;
    }

    const int16_t ax = ghepSo16Bit(boDem[0],  boDem[1]);
    const int16_t ay = ghepSo16Bit(boDem[2],  boDem[3]);
    const int16_t az = ghepSo16Bit(boDem[4],  boDem[5]);
    const int16_t t  = ghepSo16Bit(boDem[6],  boDem[7]);
    const int16_t gx = ghepSo16Bit(boDem[8],  boDem[9]);
    const int16_t gy = ghepSo16Bit(boDem[10], boDem[11]);
    const int16_t gz = ghepSo16Bit(boDem[12], boDem[13]);

    // Hệ số cho accel ±4 g.
    constexpr float accelScale = 9.80665f / 8192.0f;

    // Hệ số cho gyro ±500 °/s.
    constexpr float gyroLsbPerDps = 65.5f;
    constexpr float degToRad = 0.01745329252f;

    khungDuLieu.giaTocX_mps2 = ax * accelScale;
    khungDuLieu.giaTocY_mps2 = ay * accelScale;
    khungDuLieu.giaTocZ_mps2 = az * accelScale;
    khungDuLieu.tocDoGocX_radps =(gx / gyroLsbPerDps - _doLechConQuay_dps[0]) * degToRad;
    khungDuLieu.tocDoGocY_radps =(gy / gyroLsbPerDps - _doLechConQuay_dps[1]) * degToRad;
    khungDuLieu.tocDoGocZ_radps =(gz / gyroLsbPerDps - _doLechConQuay_dps[2]) * degToRad;
    khungDuLieu.nhietDoCamBien_C = t / 340.0f + 36.53f;
    // Chưa triển khai hiệu chuẩn gyro.
    khungDuLieu.daHieuChinhConQuay = _daHieuChinhConQuay;
    khungDuLieu.trangThai = TrangThaiKhungDuLieuMpu::HopLe;
    return true;
}
bool mpu6050Drv::hieuChinhConQuay(uint16_t soMau) {
    // Cần ít nhất 2 mẫu để đánh giá độ biến thiên.
    if (!_daKhoiTao || soMau < 2) return false;
    _daHieuChinhConQuay = false;
    // Tần số lấy mẫu MPU, đơn vị mẫu/giây.
    const uint32_t tanSoLayMau_hz = tanSoLayMauMpu_hz;
    // Thời gian dự kiến thu đủ mẫu, làm tròn lên millisecond.
    const uint32_t thoiGianThuMau_ms = (static_cast<uint32_t>(soMau) * 1000u + tanSoLayMau_hz - 1u) / tanSoLayMau_hz;
    // Cộng thời gian dự phòng từ Config.
    const uint32_t timeoutMs = thoiGianThuMau_ms + duPhongThoiGianHieuChinh_ms;
    // Thời điểm bắt đầu để kiểm tra timeout trong vòng while.
    const uint32_t thoiDiemBatDau_ms = millis();
    // trungBinh: trung bình gyro theo độ/giây.
    // tongBinhPhuongLech: giá trị tích lũy để tính phương sai.
    float trungBinh[3]{};
    float tongBinhPhuongLech[3]{};
    uint16_t soMauDaThu = 0;
    // Đọc INT_STATUS để xóa sự kiện đã có trước khi hiệu chuẩn.
    uint8_t trangThaiNgat = 0;
    if (docNhieuByte(0x3A, &trangThaiNgat, 1) != I2C_Status::OK) return false;
    while (soMauDaThu < soMau) {
        if (static_cast<uint32_t>(millis() - thoiDiemBatDau_ms) >= timeoutMs) return false;
        // Bit 0 của INT_STATUS báo có dữ liệu mới.
        // Đọc thanh ghi này cũng xóa trạng thái ngắt.
        if (docNhieuByte(0x3A, &trangThaiNgat, 1) != I2C_Status::OK) return false;
        if ((trangThaiNgat & 0x01u) == 0) {
            delay(1);
            continue;
        }
        // Khi hiệu chuẩn chỉ cần 6 byte gyro từ GYRO_XOUT_H.
        uint8_t boDem[6]{};

        if (docNhieuByte(0x43, boDem, sizeof(boDem)) != I2C_Status::OK) return false;
        const float tocDoGoc_dps[3] = {
            ghepSo16Bit(boDem[0], boDem[1]) / 65.5f,
            ghepSo16Bit(boDem[2], boDem[3]) / 65.5f,
            ghepSo16Bit(boDem[4], boDem[5]) / 65.5f
        };
        ++soMauDaThu;
        // Cập nhật trung bình và độ biến thiên từng trục.
        for (uint8_t axis = 0; axis < 3; ++axis) {
            const float delta = tocDoGoc_dps[axis] - trungBinh[axis];
            trungBinh[axis] += delta / static_cast<float>(soMauDaThu);
            const float doLechSau = tocDoGoc_dps[axis] - trungBinh[axis];
            tongBinhPhuongLech[axis] += delta * doLechSau;
        }
    }
    const float gioiHanPhuongSai =doLechChuanConQuayToiDa_dps * doLechChuanConQuayToiDa_dps;
    // Nếu gyro biến động lớn, không chấp nhận bias vừa đo.
    for (uint8_t axis = 0; axis < 3; ++axis) {
        const float phuongSai = tongBinhPhuongLech[axis] / static_cast<float>(soMauDaThu - 1);
        if (phuongSai > gioiHanPhuongSai) return false;
    }
    // Chỉ ghi bias sau khi cả 3 trục đều vượt qua kiểm tra.
    for (uint8_t axis = 0; axis < 3; ++axis) _doLechConQuay_dps[axis] = trungBinh[axis];
    _daHieuChinhConQuay = true;
    return true;
}