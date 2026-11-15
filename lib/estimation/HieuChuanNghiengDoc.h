#pragma once
#include "HeTrucImu.h"

struct BaoCaoCalibDoc {
    const char* lyDo = "chua_tinh";
    float gocMocTruoc_rad = 0, gocMocSau_rad = 0, gocQuayGyro_rad = 0;
    float tyLeNangLuongTruc = 0, lechVector_rad = 0;
};

// Hạ ĐẦU (pitch dương) -> qua đứng -> hạ ĐUÔI (pitch âm), gyro đã trừ bias.
// Hai mặt nằm phải có chuẩn cơ khí; không lấy tư thế người giữ ban đầu làm pitch 0.
class HieuChuanNghiengDoc {
public:
    void themMau(const Vec3Imu& gyro, float khoangThoiGian_s);
    bool tinhMaTran(const Vec3Imu& giaTocDung, const Vec3Imu& giaTocTruoc,
                    const Vec3Imu& giaTocSau, MaTranQuayImu& ketQua,
                    BaoCaoCalibDoc* baoCao = nullptr) const;
private:
    float _moment[3][3]{};
    Vec3Imu _tongGoc{};
    unsigned _soMau = 0;
};
