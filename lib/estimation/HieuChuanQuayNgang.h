#pragma once
#include "HeTrucImu.h"

// Tích lũy một lượt nghiêng từ trái sang phải, với gyro đã trừ bias (rad/s).
class HieuChuanQuayNgang {
public:
    void themMau(const Vec3Imu& gyro, float khoangThoiGian_s);
    bool tinhMaTran(const Vec3Imu& giaTocDung, const Vec3Imu& giaTocTrai,
                    const Vec3Imu& giaTocPhai, MaTranQuayImu& ketQua) const;
private:
    float _moment[3][3]{};
    Vec3Imu _tongGoc{};
    unsigned _soMau = 0;
};
