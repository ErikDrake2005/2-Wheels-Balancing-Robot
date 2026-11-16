#ifndef PINCONFIG_H
#define PINCONFIG_H

#include "config/MotorConfig.h"

// Sơ đồ GPIO ESP32-S3 SUPER MINI cho hai cấu hình phần cứng:
// 1. LoaiPhanCungDongCo::MKS (MKS Servo42C V1.1 / Servo42D, khớp chiều tiến +X thực tế trên xe):
//    - GPIO1  = TX chung nối tới chân RX của cả hai module MKS Servo42C V1.1
//    - GPIO4  = RX1 nối tới chân TX của MKS 2 (DIR=12, STEP=13 -> Bánh TRÁI thực tế)
//    - GPIO3  = RX2 nối tới chân TX của MKS 1 (DIR=5,  STEP=6  -> Bánh PHẢI thực tế)
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

#define chanSdaMpu 10
#define chanSclMpu 11
#define chanNgatMpu 7

// Khi dùng TMC_MT, GPIO8/9 thuộc về encoder trái; CS/SA0 của ICM20602 được nối cứng trên PCB (-1).
constexpr int chanSa0Icm = dungPhanCungTmcMt ? -1 : 8;
constexpr int chanCsIcm  = dungPhanCungTmcMt ? -1 : 9;

// Ba chân UART đọc cảm biến từ 14-bit tích hợp trên 2 module MKS Servo42C V1.1 (chế độ MKS):
// - IO1 (GPIO1): TX chung của ESP32-S3 -> nối tới chân RX của cả 2 module MKS 1 và MKS 2
// - IO4 (GPIO4): RX của động cơ MKS 2 (Bánh Trái thực tế: GPIO12/13)
// - IO3 (GPIO3): RX của động cơ MKS 1 (Bánh Phải thực tế: GPIO5/6)
constexpr int chanUartTxMks     = (!dungPhanCungTmcMt && batUartEncoderMks) ? 1 : -1;
constexpr int chanUartRxMksTrai = (!dungPhanCungTmcMt && batUartEncoderMks) ? 4 : -1;
constexpr int chanUartRxMksPhai = (!dungPhanCungTmcMt && batUartEncoderMks) ? 3 : -1;
constexpr int chanUartRxMks1    = chanUartRxMksPhai; // IO3 = RX của động cơ MKS 1 (Bánh Phải)
constexpr int chanUartRxMks2    = chanUartRxMksTrai; // IO4 = RX của động cơ MKS 2 (Bánh Trái)

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

// Chân điều khiển hai driver: với MKS hoán đổi cặp kênh trái/phải theo trục tiến +X thực tế của xe.
constexpr int chanDirTrai  = dungPhanCungTmcMt ? 5 : 12;
constexpr int chanStepTrai = dungPhanCungTmcMt ? 6 : 13;
constexpr int chanDirPhai  = dungPhanCungTmcMt ? 12 : 5;
constexpr int chanStepPhai = dungPhanCungTmcMt ? 13 : 6;
#define chanBatMotor 2

constexpr bool cacChanChucNangKhongTrung() {
    // chanNutCalib (GPIO3) và chanUartRxMksPhai (GPIO3) hoạt động ở 2 giai đoạn tách biệt
    // (calib ICM20602 trước -> chuyển sang UART RX vòng hồi tiếp khi vào cân bằng) nên được phép dùng chung GPIO3.
    const int cacChan[] = {
        chanSdaMpu, chanSclMpu, chanNgatMpu, chanSa0Icm, chanCsIcm,
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
