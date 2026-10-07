#ifndef IMUCONFIG_H
#define IMUCONFIG_H

// Vận hành và diễn giải MPU6050
#define MPU_I2C_ADRESS 0x68
#define MPU_CLK_MSR 400000
#define MPU_thoigianLayMau_Hz 200
#define MPU_AccelRangeChoPhep_g 4
#define MPU_GyroRangeChoPhep_dps 500
#define MPU_DigitalLowPassFilterConfig 3
#define MPU_NgatIMU true
#define MPU_ActiveSignal true // true: active HIGH; false: active LOW
#define MPU_laymauCalib 500
// Ngưỡng thử ban đầu để phát hiện gyro biến động khi hiệu chuẩn.
// Đơn vị: độ/giây. Cần tinh chỉnh bằng dữ liệu cảm biến thực tế.
constexpr float MPU_GYRO_CALIB_MAX_STD_DPS = 0.5f;
#endif