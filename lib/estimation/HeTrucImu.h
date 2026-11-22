#pragma once
#include <cmath>

struct Vec3Imu { float x=0, y=0, z=0; };
struct MaTranQuayImu {
    float phanTu[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
};
inline float tichVoHuong(const Vec3Imu& vectoThuNhat, const Vec3Imu& vectoThuHai) {
    return vectoThuNhat.x*vectoThuHai.x+vectoThuNhat.y*vectoThuHai.y+vectoThuNhat.z*vectoThuHai.z;
}
inline Vec3Imu tichCoHuong(const Vec3Imu& vectoThuNhat, const Vec3Imu& vectoThuHai) {
    return {vectoThuNhat.y*vectoThuHai.z-vectoThuNhat.z*vectoThuHai.y,
            vectoThuNhat.z*vectoThuHai.x-vectoThuNhat.x*vectoThuHai.z,
            vectoThuNhat.x*vectoThuHai.y-vectoThuNhat.y*vectoThuHai.x};
}
inline bool vectoHuuHan(const Vec3Imu& vecto) {
    return std::isfinite(vecto.x) && std::isfinite(vecto.y) && std::isfinite(vecto.z);
}
inline float doLonVecto(const Vec3Imu& vecto) {
    return std::sqrt(tichVoHuong(vecto,vecto));
}
inline bool chuanHoaVecto(Vec3Imu& vecto) {
    const float doLon=doLonVecto(vecto);
    if (!vectoHuuHan(vecto) || !std::isfinite(doLon) || doLon < 1e-6f) return false;
    vecto.x/=doLon; vecto.y/=doLon; vecto.z/=doLon;
    return true;
}

class HeTrucImu {
public:
    HeTrucImu(); // Tính R một lần từ ImuMountConfig.h.
    explicit HeTrucImu(const MaTranQuayImu& maTran);
    bool hopLe() const { return _hopLe; }
    const MaTranQuayImu& maTran() const { return _maTran; }
    Vec3Imu chuyenSangXe(const Vec3Imu& vectoCamBien) const;
    static MaTranQuayImu tuGocLap_deg(float roll_deg, float pitch_deg, float yaw_deg);
    static bool kiemTraMaTran(const MaTranQuayImu& maTran);
    static bool tuTuTheDung(const Vec3Imu& giaTocDung, Vec3Imu huongTienSensor, MaTranQuayImu& ketQua);
    // Accel trung bình lúc KHUNG XE thẳng + gyro của đoạn nghiêng TIẾN đã trừ bias.
    static bool tuHaiVector(const Vec3Imu& giaTocDung, const Vec3Imu& gyroNghiengTien,
                           MaTranQuayImu& ketQua);
private:
    MaTranQuayImu _maTran;
    bool _hopLe = false;
};
