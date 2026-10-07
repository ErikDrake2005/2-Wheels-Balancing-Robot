#ifndef MPU6050DRIVER_H
#define MPU6050DRIVER_H
#include <Arduino.h>
#include <Wire.h>
#include "model/SensorFrame.h"
#include "config/ImuConfig.h"
#include "config/PinConfig.h"

enum class I2C_Status : uint8_t {
    OK = 0,
    BUFFER_OVERFLOW = 1,
    ADDR_NACK = 2,
    DATA_NACK = 3,
    OTHER_ERROR = 4,
    TIMEOUT = 5,
    READ_SIZE_MISMATCH = 6,
    INVALID_PARAM = 7
};
class mpu6050Drv {
public:
    explicit mpu6050Drv(TwoWire& bus);
    bool begin();
    bool read(KhungDuLieuMPU& KhungDuLieu);
    // Chỉ gọi trước khi cân bằng, khi xe đứng yên và motor đã disable.
    // Hàm thu nhiều mẫu nên có thể mất vài giây.
    // Không gọi từ ISR hoặc trong vòng PID.
    bool HieuChinhGyro(uint16_t Somau);
private:

    TwoWire& _bus;
    float _gyroBiasDps[3]={0.0f, 0.0f, 0.0f};
    bool _trangthaiKhoiTao = false;
    bool _daHieuChinhGyro = false;
    bool ghiThanhghi(uint8_t DiaChi, uint8_t GiaTri);
    I2C_Status docNhieuByte(uint8_t diachiBatDau, uint8_t* duLieu, size_t soByte);
};
#endif