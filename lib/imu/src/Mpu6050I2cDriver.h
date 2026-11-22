#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "ImuDriverTypes.h"
#include "config/Mpu6050Config.h"

// Driver I2C chuẩn công nghiệp cho MPU6050 (tương thích hoàn toàn giao diện SelectedImuDriver):
// - Khóa cứng AD0 trên GPIO7 (mức LOW -> địa chỉ 0x68).
// - Khóa cứng USER_CTRL (0x6A)=0x00 và INT_PIN_CFG (0x37)=0x00 (tắt I2C_MST và I2C_BYPASS)
//   để bảo vệ phần cứng nếu chân phụ XCL/XDA bị nối ngoài ý muốn.
// - Hỗ trợ vận hành không cần chân INT vật lý (INT để trống) thông qua kiểm tra cờ DATA_RDY_INT (0x3A).
class Mpu6050I2cDriver {
public:
    explicit Mpu6050I2cDriver(TwoWire& busI2c) : _busI2c(busI2c) {}
    bool begin();
    bool kiemTraSauKhoiDong(BaoCaoKiemTraIcm& baoCao);
    IcmI2cStatus maLoiI2cCuoi() const { return _maLoiI2cCuoi; }
    uint8_t thanhGhiCuoi() const { return _thanhGhiCuoi; }
    uint8_t maNhanDang() const { return _maNhanDang; }
    ChanDoanGiaoDichI2c giaoDichDocCuoi() const { return _giaoDichDocCuoi; }
    ChanDoanGiaoDichI2c giaoDichDocLoiCuoi() const { return _giaoDichDocLoiCuoi; }

    bool read(KhungDuLieuImu& khungDuLieu);
    bool docMauMoi(KhungDuLieuImu& khungDuLieu);
    bool docMauTheoNgat(KhungDuLieuImu& khungDuLieu);

    bool hieuChinhConQuay(uint16_t soMau);
    void datDoLechConQuay_dps(float bx, float by, float bz);
    void layDoLechConQuay_dps(float& bx, float& by, float& bz, bool& daHieuChinh) const;

private:
    TwoWire& _busI2c;
    float _doLechConQuay_dps[3]{0.0f, 0.0f, 0.0f};
    bool _daKhoiTao = false;
    bool _daHieuChinhConQuay = false;
    IcmI2cStatus _maLoiI2cCuoi = IcmI2cStatus::OK;
    uint8_t _thanhGhiCuoi = 0;
    uint8_t _maNhanDang = 0;
    ChanDoanGiaoDichI2c _giaoDichDocCuoi{}, _giaoDichDocLoiCuoi{};

    bool ghiThanhGhi(uint8_t diaChi, uint8_t giaTri);
    IcmI2cStatus docNhieuByte(uint8_t diaChiBatDau, uint8_t* duLieu, size_t soByte);
    bool cauHinhHopLe();
    bool giaiMaMau(const uint8_t* boDem, KhungDuLieuImu& khungDuLieu) const;
};
