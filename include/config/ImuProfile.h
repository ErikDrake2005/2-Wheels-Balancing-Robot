#pragma once

// Chọn tại build; đổi cảm biến cần nạp đúng environment rồi khởi động lại.
#define IMU_ICM20602_I2C 0
#define IMU_LSM6DSV_SPI  1
#define IMU_MPU6050_I2C  2

#ifndef CAU_HINH_IMU
#define CAU_HINH_IMU IMU_ICM20602_I2C
#endif

#if CAU_HINH_IMU != IMU_ICM20602_I2C && CAU_HINH_IMU != IMU_LSM6DSV_SPI && CAU_HINH_IMU != IMU_MPU6050_I2C
#error "CAU_HINH_IMU khong hop le"
#endif

constexpr bool dungLsm6dsv = (CAU_HINH_IMU == IMU_LSM6DSV_SPI);
constexpr bool dungMpu6050 = (CAU_HINH_IMU == IMU_MPU6050_I2C);
constexpr bool dungProfileImuRieng = dungLsm6dsv || dungMpu6050;

constexpr const char* tenImu =
    dungLsm6dsv ? "LSM6DSV" : (dungMpu6050 ? "MPU6050" : "ICM20602");
constexpr const char* giaoTiepImu =
    dungLsm6dsv ? "SPI" : "I2C";
constexpr const char* nvsCauHinhImu =
    dungLsm6dsv ? "xe_lsm6dsv" : (dungMpu6050 ? "xe_mpu6050" : "xecanbang");
