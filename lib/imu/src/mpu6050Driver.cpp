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

bool mpu6050Drv::ghiThanhghi(uint8_t DiaChi, uint8_t GiaTri) {
    _bus.beginTransmission(MPU_I2C_ADRESS);
    _bus.write(DiaChi);
    _bus.write(GiaTri);
    return _bus.endTransmission() == 0;
}

I2C_Status mpu6050Drv::docNhieuByte(uint8_t diachiBatDau, uint8_t* duLieu, size_t soByte){
    if (duLieu == nullptr || soByte == 0 || soByte > 14) return I2C_Status::INVALID_PARAM;
    _bus.beginTransmission(MPU_I2C_ADRESS);
    _bus.write(diachiBatDau);
    uint8_t txStatus = _bus.endTransmission(false);
    if (txStatus != 0) return static_cast<I2C_Status>(txStatus);
    size_t daNhan = _bus.requestFrom(
        static_cast<uint8_t>(MPU_I2C_ADRESS),
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
    _trangthaiKhoiTao = false;
    _trangthaiKhoiTao = false;
    _daHieuChinhGyro = false;
    for (float& bias : _gyroBiasDps) bias = 0.0f;
    if (MPU_AccelRangeChoPhep_g != 4 ||
        MPU_GyroRangeChoPhep_dps != 500) {
        return false;
    }
    if (MPU_DigitalLowPassFilterConfig < 1 ||
        MPU_DigitalLowPassFilterConfig > 6) {
        return false;
    }
    const uint32_t thoigianLayMau_Hz = MPU_thoigianLayMau_Hz;
    if (thoigianLayMau_Hz == 0 || thoigianLayMau_Hz > 1000 ||
        1000 % thoigianLayMau_Hz != 0) {
        return false;
    }
    const uint32_t divider = 1000 / thoigianLayMau_Hz - 1;
    if (divider > 255) {
        return false;
    }
    if (!_bus.begin(PIN_SDA_MPU6050, PIN_SCL_MPU6050)) {
        return false;
    }
    _bus.setClock(MPU_CLK_MSR);
    _bus.setTimeOut(2); // Timeout I2C thử ban đầu, đơn vị ms
    uint8_t id = 0;
    if (docNhieuByte(0x75, &id, 1) != I2C_Status::OK || id != 0x68) return false;
    if (!ghiThanhghi(0x6B, 0x01)) return false; // Đánh thức MPU
    delay(100);                               // Chỉ lúc khởi tạo
    if (!ghiThanhghi(0x1A, MPU_DigitalLowPassFilterConfig))
        return false;                         // DLPF
    if (!ghiThanhghi(0x19, static_cast<uint8_t>(divider)))
        return false;                         // Tần số mẫu
    if (!ghiThanhghi(0x1B, 0x08))
        return false;                         // Gyro ±500 °/s
    if (!ghiThanhghi(0x1C, 0x08))
        return false;                         // Accel ±4 g
    const uint8_t intPinConfig = MPU_ActiveSignal ? 0x00 : 0x80;
    if (!ghiThanhghi(0x37, intPinConfig)) return false;                   // INT active HIGH
    if (!ghiThanhghi(0x38, MPU_NgatIMU ? 0x01 : 0x00))
        return false;                     // Data-ready interrupt
    uint8_t cauHinh[4]{};
    if (docNhieuByte(0x19, cauHinh, sizeof(cauHinh)) != I2C_Status::OK) return false;
    if (cauHinh[0] != static_cast<uint8_t>(divider) ||
        cauHinh[1] != MPU_DigitalLowPassFilterConfig ||
        cauHinh[2] != 0x08 ||   // Gyro ±500 °/s
        cauHinh[3] != 0x08) {  // Accel ±4 g
        return false;
    }
    _trangthaiKhoiTao = true;
    return true;
}
bool mpu6050Drv::read(KhungDuLieuMPU& frame) {
    // Mỗi lần gọi bắt đầu với một frame không hợp lệ.
    frame = KhungDuLieuMPU{};
    if (!_trangthaiKhoiTao) return false;
    uint8_t buffer[14]{};
    // Đọc một khối: accel XYZ, nhiệt độ, gyro XYZ.
    if (docNhieuByte(0x3B, buffer, sizeof(buffer)) != I2C_Status::OK) {
        frame.trangthai = trangthaiKhungDuLieuMPU::LoiIO;
        return false;
    }

    const int16_t ax = ghepSo16Bit(buffer[0],  buffer[1]);
    const int16_t ay = ghepSo16Bit(buffer[2],  buffer[3]);
    const int16_t az = ghepSo16Bit(buffer[4],  buffer[5]);
    const int16_t t  = ghepSo16Bit(buffer[6],  buffer[7]);
    const int16_t gx = ghepSo16Bit(buffer[8],  buffer[9]);
    const int16_t gy = ghepSo16Bit(buffer[10], buffer[11]);
    const int16_t gz = ghepSo16Bit(buffer[12], buffer[13]);

    // Hệ số cho accel ±4 g.
    constexpr float accelScale = 9.80665f / 8192.0f;

    // Hệ số cho gyro ±500 °/s.
    constexpr float gyroLsbPerDps = 65.5f;
    constexpr float degToRad = 0.01745329252f;

    frame.giatocX_mps2 = ax * accelScale;
    frame.giatocY_mps2 = ay * accelScale;
    frame.giatocZ_mps2 = az * accelScale;
    frame.tocdoGocX_radps =(gx / gyroLsbPerDps - _gyroBiasDps[0]) * degToRad;
    frame.tocdoGocY_radps =(gy / gyroLsbPerDps - _gyroBiasDps[1]) * degToRad;
    frame.tocdoGocZ_radps =(gz / gyroLsbPerDps - _gyroBiasDps[2]) * degToRad;
    frame.nhietdoCamBien_C = t / 340.0f + 36.53f;
    // Chưa triển khai hiệu chuẩn gyro.
    frame.trangthaiCalib = _daHieuChinhGyro;
    frame.trangthai = trangthaiKhungDuLieuMPU::HopLe;
    return true;
}
bool mpu6050Drv::HieuChinhGyro(uint16_t Somau) {
    // Cần ít nhất 2 mẫu để đánh giá độ biến thiên.
    if (!_trangthaiKhoiTao || Somau < 2) return false;
    _daHieuChinhGyro = false;
    // Tính thời gian dự kiến từ số mẫu và tần số MPU.
    // Thêm 1 giây dự phòng để hàm không chờ vô hạn
    // khi cảm biến không tạo thêm dữ liệu.
    const uint32_t thoigianLayMau_Hz = MPU_thoigianLayMau_Hz;
    const uint32_t timeoutMs =(static_cast<uint32_t>(Somau) * 1000u + thoigianLayMau_Hz - 1u)/thoigianLayMau_Hz + 1000u;
    const uint32_t batDauMs = millis();
    // mean: trung bình gyro theo độ/giây.
    // m2: tổng dùng để tính phương sai bằng thuật toán Welford.
    float mean[3]{};
    float m2[3]{};
    uint16_t daThu = 0;
    // Đọc INT_STATUS để xóa sự kiện đã có trước khi hiệu chuẩn.
    uint8_t intStatus = 0;
    if (docNhieuByte(0x3A, &intStatus, 1) != I2C_Status::OK) return false;
    while (daThu < Somau) {
        if (static_cast<uint32_t>(millis() - batDauMs) >= timeoutMs) return false;
        // Bit 0 của INT_STATUS báo có dữ liệu mới.
        // Đọc thanh ghi này cũng xóa trạng thái ngắt.
        if (docNhieuByte(0x3A, &intStatus, 1) != I2C_Status::OK) return false;
        if ((intStatus & 0x01u) == 0) {
            delay(1);
            continue;
        }
        // Khi hiệu chuẩn chỉ cần 6 byte gyro từ GYRO_XOUT_H.
        uint8_t buffer[6]{};

        if (docNhieuByte(0x43, buffer, sizeof(buffer)) != I2C_Status::OK) return false;
        const float gyroDps[3] = {
            ghepSo16Bit(buffer[0], buffer[1]) / 65.5f,
            ghepSo16Bit(buffer[2], buffer[3]) / 65.5f,
            ghepSo16Bit(buffer[4], buffer[5]) / 65.5f
        };
        ++daThu;
        // Cập nhật trung bình và độ biến thiên từng trục.
        for (uint8_t axis = 0; axis < 3; ++axis) {
            const float delta = gyroDps[axis] - mean[axis];
            mean[axis] += delta / static_cast<float>(daThu);
            const float deltaSau = gyroDps[axis] - mean[axis];
            m2[axis] += delta * deltaSau;
        }
    }
    const float varianceLimit =MPU_GYRO_CALIB_MAX_STD_DPS * MPU_GYRO_CALIB_MAX_STD_DPS;
    // Nếu gyro biến động lớn, không chấp nhận bias vừa đo.
    for (uint8_t axis = 0; axis < 3; ++axis) {
        const float variance = m2[axis] / static_cast<float>(daThu - 1);
        if (variance > varianceLimit) return false;
    }
    // Chỉ ghi bias sau khi cả 3 trục đều vượt qua kiểm tra.
    for (uint8_t axis = 0; axis < 3; ++axis) _gyroBiasDps[axis] = mean[axis];
    _daHieuChinhGyro = true;
    return true;
}