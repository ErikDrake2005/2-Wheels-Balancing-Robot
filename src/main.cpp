#include <Arduino.h>
#include <Wire.h>

#include "config/PinConfig.h"
#include "config/ImuConfig.h"
#include "config/ConfigCompareSettings.h"

bool daKhoiTaoBus = false;
bool daDanhThucCamBien = false;

// Ghi một giá trị vào thanh ghi cảm biến.
bool ghiThanhGhiThu(uint8_t diaChiThanhGhi, uint8_t giaTri) {
    Wire.beginTransmission(diaChiI2cMpu);
    Wire.write(diaChiThanhGhi);
    Wire.write(giaTri);

    return Wire.endTransmission(true) == 0;
}

// Đọc liên tiếp nhiều byte bắt đầu từ một thanh ghi.
bool docKhoiThu(
    uint8_t diaChiThanhGhi,
    uint8_t* duLieu,
    size_t soByte
) {
    if (duLieu == nullptr || soByte == 0) {
        return false;
    }

    Wire.beginTransmission(diaChiI2cMpu);
    Wire.write(diaChiThanhGhi);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    const size_t soByteNhan = Wire.requestFrom(
        static_cast<uint8_t>(diaChiI2cMpu),
        soByte,
        true
    );

    if (soByteNhan != soByte) {
        return false;
    }

    for (size_t i = 0; i < soByte; ++i) {
        const int giaTri = Wire.read();

        if (giaTri < 0) {
            return false;
        }

        duLieu[i] = static_cast<uint8_t>(giaTri);
    }

    return true;
}

// Ghép hai byte thành số nguyên có dấu 16 bit.
int16_t ghepSo16BitThu(uint8_t byteCao, uint8_t byteThap) {
    const uint16_t giaTri =
        (static_cast<uint16_t>(byteCao) << 8) | byteThap;

    int32_t giaTriCoDau = giaTri;

    if (giaTri >= 0x8000u) {
        giaTriCoDau -= 65536;
    }

    return static_cast<int16_t>(giaTriCoDau);
}

void setup() {
    Serial.begin(tocDoSerialThu_baud);

    daKhoiTaoBus = Wire.begin(
        chanSdaMpu,
        chanSclMpu,
        tanSoI2cMpu_hz
    );

    if (!daKhoiTaoBus) {
        return;
    }

    Wire.setTimeOut(thoiGianChoI2c_ms);

    // PWR_MGMT_1: thoát sleep, chọn nguồn clock.
    const bool ghiNguonThanhCong =
        ghiThanhGhiThu(0x6B, 0x01);

    delay(thoiGianKhoiDongMpu_ms);

    // PWR_MGMT_2: bật các trục gia tốc và con quay.
    const bool batCacTrucThanhCong =
        ghiThanhGhiThu(0x6C, 0x00);

    daDanhThucCamBien =
        ghiNguonThanhCong && batCacTrucThanhCong;
}

void loop() {
    static uint32_t lanBaoCaoCuoi_ms = 0;
    const uint32_t hienTai_ms = millis();

    if (hienTai_ms - lanBaoCaoCuoi_ms < chuKyBaoCao_ms) {
        delay(1);
        return;
    }

    lanBaoCaoCuoi_ms = hienTai_ms;

    uint8_t maNhanDang = 0;
    uint8_t duLieuTho[14]{};

    const bool docIdThanhCong =
        daKhoiTaoBus &&
        docKhoiThu(0x75, &maNhanDang, 1);

    const bool docDuLieuThanhCong =
        daKhoiTaoBus &&
        daDanhThucCamBien &&
        docKhoiThu(0x3B, duLieuTho, sizeof(duLieuTho));

    Serial.printf(
        "THU_MPU: bus=%d danhThuc=%d docID=%d "
        "ID=0x%02X docDuLieu=%d\n",
        static_cast<int>(daKhoiTaoBus),
        static_cast<int>(daDanhThucCamBien),
        static_cast<int>(docIdThanhCong),
        static_cast<unsigned>(maNhanDang),
        static_cast<int>(docDuLieuThanhCong)
    );

    if (!docDuLieuThanhCong) {
        return;
    }

    // Byte 6 và 7 là nhiệt độ; phép thử chưa sử dụng.
    Serial.printf(
        "THO: A=%d,%d,%d G=%d,%d,%d\n",
        static_cast<int>(ghepSo16BitThu(duLieuTho[0], duLieuTho[1])),
        static_cast<int>(ghepSo16BitThu(duLieuTho[2], duLieuTho[3])),
        static_cast<int>(ghepSo16BitThu(duLieuTho[4], duLieuTho[5])),
        static_cast<int>(ghepSo16BitThu(duLieuTho[8], duLieuTho[9])),
        static_cast<int>(ghepSo16BitThu(duLieuTho[10], duLieuTho[11])),
        static_cast<int>(ghepSo16BitThu(duLieuTho[12], duLieuTho[13]))
    );
}