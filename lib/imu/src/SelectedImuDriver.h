#pragma once
#include "config/ImuProfile.h"
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
#include "Lsm6dsvDriver.h"
using SelectedImuDriver = Lsm6dsvDriver;
#elif CAU_HINH_IMU == IMU_MPU6050_I2C
#include "Mpu6050I2cDriver.h"
using SelectedImuDriver = Mpu6050I2cDriver;
#else
#include "Icm20602Driver.h"
using SelectedImuDriver = Icm20602Driver;
#endif
