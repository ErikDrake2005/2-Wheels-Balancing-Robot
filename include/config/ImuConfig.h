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
#define MPU_ActiveSignal true // cạnh lên
#define MPU_laymauCalib 500

#endif