#pragma once
#include "HeTrucImu.h"
#include "SelectedImuDriver.h"

// UI đồng bộ trong setup, trước khi gắn ngắt IMU và tạo các task cân bằng.
class UI_Calib {
public:
    explicit UI_Calib(SelectedImuDriver& camBien) : _camBien(camBien) {}
    void khoiTao();
    void chay(MaTranQuayImu& ketQua);
    void baoSanSang() const;
    void baoChoDuLieu() const;
    void baoLoi() const;
private:
    struct TrangThaiNut {
        int chan = -1;
        bool mucCu = false, mucOnDinh = false, daNhan = false;
        uint32_t lucDoi_ms = 0;
    };
    SelectedImuDriver& _camBien;
    TrangThaiNut _nutChinh{}, _nutPhu{};
    void hienMau(uint8_t doLed, uint8_t xanhLa, uint8_t xanhDuong) const;
    static void dongBoMotNut(TrangThaiNut& nut);
    static bool docSuKienMotNut(TrangThaiNut& nut);
    void dongBoNut();
    bool daXacNhan();
    void choXacNhan();
    bool kiemTraLoiDoc(const KhungDuLieuImu& mau, uint32_t lanCoMau_ms, const char* buoc) const;
    bool layGiaTocTinh(Vec3Imu& giaToc, const char* buoc);
    bool choVeThang(const HeTrucImu& heTruc);
    bool thuMotLuot(MaTranQuayImu& ketQua);
    bool thuMotLuotHaiTruc(MaTranQuayImu& ketQua);
};
