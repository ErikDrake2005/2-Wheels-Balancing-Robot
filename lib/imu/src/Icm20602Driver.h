#ifndef ICM20602DRIVER_H
#define ICM20602DRIVER_H
#include <Arduino.h>
#include <Wire.h>
#include "model/SensorFrame.h"
#include "config/ImuConfig.h"
#include "config/PinConfig.h"

enum class IcmI2cStatus : uint8_t {
    OK = 0,
    BUFFER_OVERFLOW = 1,
    ADDR_NACK = 2,
    DATA_NACK = 3,
    OTHER_ERROR = 4,
    TIMEOUT = 5,
    READ_SIZE_MISMATCH = 6,
    INVALID_PARAM = 7
};
class Icm20602Driver {
public:
    explicit Icm20602Driver(TwoWire& busI2c);
    bool begin();
    // Đọc một mẫu accel, gyro và nhiệt độ.
    //
    // Điều kiện gọi:
    // - Chỉ một task được sử dụng driver tại một thời điểm.
    // - Không gọi từ ISR.
    //
    // Kết quả:
    // - true: đọc đủ dữ liệu; khungDuLieu.trangThai = HopLe.
    // - false: mẫu không hợp lệ; bên gọi phải bỏ mẫu.
    //
    // Quy ước dữ liệu:
    // - Accel: m/s², vẫn bao gồm ảnh hưởng của trọng lực.
    // - Gyro: rad/s, theo trục ICM20602.
    // - Chưa ánh xạ dữ liệu sang hệ trục của xe.
    //
    // Trách nhiệm của tầng gọi:
    // - Điền timestamp và số thứ tự mẫu sau khi đọc thành công.
    // - Kiểm tra tuổi dữ liệu trước khi đưa vào bộ điều khiển.
    bool read(KhungDuLieuImu& khungDuLieu);
    // Poll data-ready trong UI trước khi gắn ISR, chỉ trả mẫu mới.
    bool docMauMoi(KhungDuLieuImu& khungDuLieu);
    // Chỉ gọi trước khi cân bằng, khi xe đứng yên và motor đã disable.
    // Hàm thu nhiều mẫu nên có thể mất vài giây.
    // Không gọi từ ISR hoặc trong vòng PID.
    bool hieuChinhConQuay(uint16_t soMau);
private:

    TwoWire& _busI2c;
    float _doLechConQuay_dps[3]={0.0f, 0.0f, 0.0f};
    bool _daKhoiTao = false;
    bool _daHieuChinhConQuay = false;
    bool ghiThanhGhi(uint8_t diaChi, uint8_t giaTri);
    IcmI2cStatus docNhieuByte(uint8_t diachiBatDau, uint8_t* duLieu, size_t soByte);
};
#endif
