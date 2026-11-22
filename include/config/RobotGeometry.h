#ifndef ROBOTGEOMETRY_H
#define ROBOTGEOMETRY_H

#include "config/ImuProfile.h"

constexpr float khoiLuongXeKhongTai_kg = 1.45f;

constexpr float toaDoTrongTamX_m = 0.000f;
constexpr float toaDoTrongTamY_m = 0.000f;
// Người dùng xác nhận: trọng tâm xe cách tâm trục bánh xe 3,6 cm (0.036 m).
constexpr float toaDoTrongTamZ_m = 0.036f;
// Khoảng cách từ cảm biến IMU đến trọng tâm xe:
// - ICM20602 cũ / MPU6050: 7,5 cm (0.075 m)
// - LSM6DSV lắp cao hơn ICM cũ 10 mm (+0.010 m): 8,5 cm (0.085 m)
constexpr float khoangCachImuDenTrongTamZ_m = dungLsm6dsv ? 0.085f : 0.075f;
// Chiều dài con lắc hiệu dụng L_eff = (I_c + m*l^2)/(m*l) + R (mục H3 đánh giá).
// Giá trị 0,12 m là ước lượng khi tính cả quán tính khay/pin và phản lực stator;
// cần đo lại bằng chu kỳ dao động tự nhiên khi treo ngược: L_eff = g*(T/(2*pi))^2.
constexpr float chieuDaiConLacHieuDung_m = 0.120f;

constexpr float banKinhBanh_m = 0.040f;
constexpr float khoangCachTamHaiBanh_m = 0.200f;

constexpr float beDayBanh_m = 0.035f;
constexpr float beRongTiepXucBanh_m = 0.030f;

constexpr float chieuDaiKhay_m = 0.150f;
constexpr float chieuRongKhay_m = 0.080f;

constexpr float viTriMpuX_m = -0.005f;
constexpr float viTriMpuY_m = -0.010f;
// Tọa độ Z của IMU so với tâm trục bánh xe:
// - ICM20602 / MPU6050: 3,6 cm + 7,5 cm = 11,1 cm (0.111f m)
// - LSM6DSV (+10 mm)  : 3,6 cm + 8,5 cm = 12,1 cm (0.121f m)
constexpr float viTriMpuZ_m = toaDoTrongTamZ_m + khoangCachImuDenTrongTamZ_m;

constexpr float soVongDongCoTrenMotVongBanh = 1.0f;

#endif
