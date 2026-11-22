#ifndef PINCONFIG_H
#define PINCONFIG_H

#include "config/MotorConfig.h"
#include "config/ImuProfile.h"
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
#include "config/Lsm6dsvConfig.h"
#elif CAU_HINH_IMU == IMU_MPU6050_I2C
#include "config/Mpu6050Config.h"
#endif

// Sơ đồ GPIO ESP32-S3 SUPER MINI cho hai cấu hình phần cứng:
// 1. LoaiPhanCungDongCo::MKS (MKS Servo42C V1.1 / Servo42D, khớp phần cứng thực tế trên xe):
//    - GPIO1  = TX chung nối tới chân RX của cả hai module MKS Servo42C V1.1
//    - GPIO3  = RX1 nối tới chân TX của MKS 1 (DIR=12, STEP=13 -> Bánh TRÁI thực tế, 0xE1)
//    - GPIO4  = RX2 nối tới chân TX của MKS 2 (DIR=5,  STEP=6  -> Bánh PHẢI thực tế, 0xE2)
//    - GPIO0  = Nút BOOT tích hợp sẵn trên ESP32-S3 Super Mini dùng cho UI_Calib
//    - GPIO8  = SA0 ICM20602, GPIO9 = CS ICM20602
// 2. LoaiPhanCungDongCo::TMC_MT (TMC2209 + MT6701 theo sơ đồ pcb.pdf):
//    - GPIO1  = A · encoder phải (MT6701)
//    - GPIO2  = ENN · cả hai TMC2209
//    - GPIO3  = S3 · nút hiệu chuẩn
//    - GPIO4  = B · encoder phải (MT6701)
//    - GPIO5  = DIR · TMC trái
//    - GPIO6  = STEP · TMC trái
//    - GPIO7  = INT · chân ngắt thật của ICM20602
//    - GPIO8  = A · encoder trái (MT6701)
//    - GPIO9  = B · encoder trái (MT6701)
//    - GPIO10 = SDA · ICM20602
//    - GPIO11 = SCL · ICM20602
//    - GPIO12 = DIR · TMC phải
//    - GPIO13 = STEP · TMC phải

#if CAU_HINH_IMU == IMU_MPU6050_I2C
#define chanSdaMpu mpu6050::chanSda
#define chanSclMpu mpu6050::chanScl
#define chanNgatMpu mpu6050::chanInt
constexpr int chanAd0Mpu = mpu6050::chanAd0;
constexpr int chanSa0Icm = -1;
constexpr int chanCsIcm  = -1;
#else
#define chanSdaMpu 10
#define chanSclMpu 11
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
#define chanNgatMpu lsm6dsv::chanInt
#else
#define chanNgatMpu 7
#endif
constexpr int chanAd0Mpu = -1;
// Khi dùng TMC_MT, GPIO8/9 thuộc về encoder trái; CS/SA0 của ICM20602 được nối cứng trên PCB (-1).
constexpr int chanSa0Icm = dungPhanCungTmcMt ? -1 : 8;
constexpr int chanCsIcm  = dungPhanCungTmcMt ? -1 : 9;
#endif

// Ba chân UART đọc cảm biến từ 14-bit tích hợp trên 2 module MKS Servo42C V1.1 (chế độ MKS):
// - IO1 (GPIO1): TX chung của ESP32-S3 -> nối tới chân RX của cả 2 module MKS 1 và MKS 2
// - IO3 (GPIO3): RX của động cơ MKS 1 (Bánh Trái thực tế: DIR=GPIO12, STEP=GPIO13, 0xE1)
// - IO4 (GPIO4): RX của động cơ MKS 2 (Bánh Phải thực tế: DIR=GPIO5, STEP=GPIO6, 0xE2)
constexpr int chanUartTxMks     = (!dungPhanCungTmcMt && batUartEncoderMks) ? 1 : -1;
constexpr int chanUartRxMksTrai = (!dungPhanCungTmcMt && batUartEncoderMks) ? 3 : -1;
constexpr int chanUartRxMksPhai = (!dungPhanCungTmcMt && batUartEncoderMks) ? 4 : -1;
constexpr int chanUartRxMks1    = chanUartRxMksTrai; // IO3 = RX của động cơ MKS 1 (Bánh Trái, 0xE1)
constexpr int chanUartRxMks2    = chanUartRxMksPhai; // IO4 = RX của động cơ MKS 2 (Bánh Phải, 0xE2)

// Chân AB của hai encoder từ tính MT6701 (chỉ kích hoạt khi dungPhanCungTmcMt = true).
constexpr int chanEncoderTraiA = dungPhanCungTmcMt ? 8 : -1;
constexpr int chanEncoderTraiB = dungPhanCungTmcMt ? 9 : -1;
constexpr int chanEncoderPhaiA = dungPhanCungTmcMt ? 1 : -1;
constexpr int chanEncoderPhaiB = dungPhanCungTmcMt ? 4 : -1;

// Nút hiệu chuẩn: GPIO3 dùng đa hợp theo thời gian (Time-Division Multiplexing):
// - Giai đoạn 1 (Hiệu chuẩn ICM20602 lúc khởi động): GPIO3 là nút nhấn xác nhận (INPUT_PULLUP).
// - Giai đoạn 2 (Sau khi xong calib, bước vào cân bằng): GPIO3 chuyển thành UART RX đọc hồi tiếp từ kế MKS 1.
constexpr int chanNutCalib = 3;
constexpr int chanNutCalibPhu = 0; // Nút BOOT trên ESP32-S3 Super Mini hỗ trợ song song lúc calib.
constexpr bool nutCalibMucThap = true; // Nút nối GND khi nhấn.
constexpr int chanRgbCalib = 48; // Mặc định cho Super Mini, -1 để tắt RGB.

// Chân điều khiển hai driver: với MKS, bánh TRÁI thực tế (0xE1, RX=IO3) là DIR12/STEP13 và bánh PHẢI (0xE2, RX=IO4) là DIR5/STEP6.
constexpr int chanDirTrai  = dungPhanCungTmcMt ? 5 : 12;
constexpr int chanStepTrai = dungPhanCungTmcMt ? 6 : 13;
constexpr int chanDirPhai  = dungPhanCungTmcMt ? 12 : 5;
constexpr int chanStepPhai = dungPhanCungTmcMt ? 13 : 6;
#define chanBatMotor 2

constexpr bool cacChanChucNangKhongTrung() {
    // chanNutCalib (GPIO3) và chanUartRxMksTrai (GPIO3) hoạt động ở 2 giai đoạn tách biệt
    // (calib ICM20602 trước -> chuyển sang UART RX vòng hồi tiếp khi vào cân bằng) nên được phép dùng chung GPIO3.
    const int cacChan[] = {
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
        lsm6dsv::chanMosi, lsm6dsv::chanSck, lsm6dsv::chanInt, lsm6dsv::chanMiso, lsm6dsv::chanCs,
#elif CAU_HINH_IMU == IMU_MPU6050_I2C
        mpu6050::chanSda, mpu6050::chanScl, mpu6050::chanAd0, mpu6050::chanInt,
#else
        chanSdaMpu, chanSclMpu, chanNgatMpu, chanSa0Icm, chanCsIcm,
#endif
        chanUartTxMks, chanUartRxMksTrai, chanUartRxMksPhai,
        chanEncoderTraiA, chanEncoderTraiB, chanEncoderPhaiA, chanEncoderPhaiB,
        chanDirTrai, chanStepTrai, chanDirPhai, chanStepPhai,
        chanBatMotor, chanRgbCalib
    };
    for (unsigned chiSo = 0; chiSo < sizeof(cacChan)/sizeof(cacChan[0]); ++chiSo) {
        if (cacChan[chiSo] < 0) continue;
        for (unsigned chiSoTruoc = 0; chiSoTruoc < chiSo; ++chiSoTruoc) {
            if (cacChan[chiSo] == cacChan[chiSoTruoc]) return false;
        }
    }
    return true;
}
static_assert(cacChanChucNangKhongTrung(), "Trung GPIO giua IMU, UART MKS, encoder, motor, nut hoac RGB");

#endif
