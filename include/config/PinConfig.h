#ifndef PINCONFIG_H
#define PINCONFIG_H

// Số GPIO theo sơ đồ ESP32S3 SUPER MINI người dùng cung cấp.

#define chanSdaMpu 10
#define chanSclMpu 11
#define chanNgatMpu 7
constexpr int chanSa0Icm = 8;
constexpr int chanCsIcm = 9;

// Header 4P: IO1, IO3, IO4, GND. GPIO3 là nút hiệu chuẩn.
constexpr int chanNutCalib = 3;
constexpr bool nutCalibMucThap = true; // Nút nối GPIO3 với GND khi nhấn.
constexpr int chanRgbCalib = 48; // Mặc định thử cho Super Mini, -1 để tắt RGB.

// Người dùng xác nhận motor 1 (DIR1/ST1) là phải; motor 2 (DIR2/ST2) là trái.
#define chanDirTrai 5
#define chanStepTrai 6
#define chanDirPhai 12
#define chanStepPhai 13
#define chanBatMotor 2

constexpr bool cacChanChucNangKhongTrung() {
    const int cacChan[] = {chanSdaMpu, chanSclMpu, chanNgatMpu, chanSa0Icm, chanCsIcm,
                          chanDirTrai, chanStepTrai, chanDirPhai, chanStepPhai,
                          chanBatMotor, chanNutCalib, chanRgbCalib};
    for (unsigned chiSo = 0; chiSo < sizeof(cacChan)/sizeof(cacChan[0]); ++chiSo) {
        if (cacChan[chiSo] < 0) continue;
        for (unsigned chiSoTruoc = 0; chiSoTruoc < chiSo; ++chiSoTruoc) {
            if (cacChan[chiSo] == cacChan[chiSoTruoc]) return false;
        }
    }
    return true;
}
static_assert(cacChanChucNangKhongTrung(), "Trung GPIO giua IMU, motor, nut hoac RGB");

#endif
