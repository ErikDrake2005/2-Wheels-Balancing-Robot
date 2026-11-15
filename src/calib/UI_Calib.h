#pragma once
#include "HeTrucImu.h"
#include "Icm20602Driver.h"

// UI đồng bộ trong setup, trước khi gắn ngắt IMU và tạo các task cân bằng.
class UI_Calib {
public:
    explicit UI_Calib(Icm20602Driver& camBien) : _camBien(camBien) {}
    void khoiTao();
    void chay(MaTranQuayImu& ketQua);
    void baoSanSang() const;
    void baoChoDuLieu() const;
    void baoLoi() const;
private:
    Icm20602Driver& _camBien;
    bool _mucCu = false, _mucOnDinh = false, _daNhan = false;
    uint32_t _lucDoiMuc_ms = 0;
    void hienMau(uint8_t doLed, uint8_t xanhLa, uint8_t xanhDuong) const;
    void dongBoNut();
    bool daXacNhan();
    void choXacNhan();
    bool kiemTraLoiDoc(const KhungDuLieuImu& mau, uint32_t lanCoMau_ms, const char* buoc) const;
    bool layGiaTocTinh(Vec3Imu& giaToc, const char* buoc);
    bool choVeThang(const HeTrucImu& heTruc);
    bool thuMotLuot(MaTranQuayImu& ketQua);
};
