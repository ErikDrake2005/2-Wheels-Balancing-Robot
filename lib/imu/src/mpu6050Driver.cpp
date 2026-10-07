#include "mpu6050Driver.h"

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
    if (MPU_AccelRangeChoPhep_g != 4 ||
        MPU_GyroRangeChoPhep_dps != 500) {
        return false;
    }
    if (MPU_DigitalLowPassFilterConfig < 1 ||
        MPU_DigitalLowPassFilterConfig > 6) {
        return false;
    }
    const uint32_t sampleHz = MPU_thoigianLayMau_Hz;
    if (sampleHz == 0 || sampleHz > 1000 ||
        1000 % sampleHz != 0) {
        return false;
    }
    const uint32_t divider = 1000 / sampleHz - 1;
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
    if (!ghiThanhghi(0x37, 0x00))
        return false;                         // INT active HIGH
    if (!ghiThanhghi(0x38, MPU_NgatIMU ? 0x01 : 0x00))
        return false;                         // Data-ready interrupt
    _trangthaiKhoiTao = true;
    return true;
}