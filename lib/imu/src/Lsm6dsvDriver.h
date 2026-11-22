#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "ImuDriverTypes.h"
#include "config/Lsm6dsvConfig.h"

class Lsm6dsvDriver {
public:
    explicit Lsm6dsvDriver(SPIClass& bus = SPI) : _bus(bus) {}
    bool begin();
    bool kiemTraSauKhoiDong(BaoCaoKiemTraIcm& baoCao);
    // Giữ kiểu API chẩn đoán dùng chung; với driver này các số đo là SPI.
    IcmI2cStatus maLoiI2cCuoi() const { return _loiCuoi; }
    uint8_t thanhGhiCuoi() const { return _thanhGhiCuoi; }
    uint8_t maNhanDang() const { return _maNhanDang; }
    bool laBienThe16Bx() const { return _laBienThe16Bx; }
    uint32_t tanSoLayMauThucTe_hz() const {
        return _laBienThe16Bx ? lsm6dsv::tanSoLayMau16Bx_hz : lsm6dsv::tanSoLayMau_hz;
    }
    ChanDoanGiaoDichI2c giaoDichDocCuoi() const { return _giaoDichCuoi; }
    ChanDoanGiaoDichI2c giaoDichDocLoiCuoi() const { return _giaoDichLoi; }
    bool read(KhungDuLieuImu& mau);
    bool docMauMoi(KhungDuLieuImu& mau);
    bool docMauTheoNgat(KhungDuLieuImu& mau);
    bool hieuChinhConQuay(uint16_t soMau);
    void datDoLechConQuay_dps(float x, float y, float z);
    void layDoLechConQuay_dps(float& x, float& y, float& z, bool& hopLe) const;
    float normGiaTocTinh_mps2() const { return _normGiaTocTinh_mps2; }
private:
    SPIClass& _bus;
    bool _daKhoiTao = false, _daHieuChinh = false, _laBienThe16Bx = false;
    float _bias[3]{};
    float _normGiaTocTinh_mps2 = 9.80665f;
    uint8_t _maNhanDang = 0, _thanhGhiCuoi = 0;
    IcmI2cStatus _loiCuoi = IcmI2cStatus::OK;
    ChanDoanGiaoDichI2c _giaoDichCuoi{}, _giaoDichLoi{};
    uint8_t maOdrHoatDong() const;
    bool doc(uint8_t reg, uint8_t* data, size_t n);
    bool ghi(uint8_t reg, uint8_t value);
    bool ghiKiemTra(uint8_t reg, uint8_t value);
    bool cauHinhHopLe();
    bool loi(IcmI2cStatus status, uint8_t reg);
    bool giaiMa(const uint8_t* data, KhungDuLieuImu& mau) const;
    bool docMau(KhungDuLieuImu& mau, bool theoNgat);
};
