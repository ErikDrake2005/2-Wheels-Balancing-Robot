#ifndef BOGIAIMAUARTMKS_H
#define BOGIAIMAUARTMKS_H

#include <cmath>
#include <cstdint>

// Bộ đóng gói và giải mã khung truyền UART cho MKS Servo42C V1.1:
// - Giao thức V1.1 sử dụng mã kiểm tra CRC8 = (tổng các byte trước đó) & 0xFF.
// - Lệnh 0x30 (Read encoder carry + value):
//   + Khung hỏi (3 byte): [diaChi, 0x30, crc8]
//   + Khung trả lời V1.1 chuẩn (8 byte): [diaChi, carry(4B big-endian), value(2B big-endian), crc8]
//   + Hỗ trợ cả biến thể 9 byte có lặp lại mã lệnh: [diaChi, 0x30, carry(4B), value(2B), crc8]
//   + Trong đó value thuộc [0 .. 0x3FFF] (16384 xung từ kế / 1 vòng động cơ).
struct MauEncoderMksUart {
    int32_t soVongCarry = 0;
    uint16_t gocTrongVong = 0;
    int64_t tongXungEncoder = 0;
    bool hopLe = false;
};

class BoGiaiMaUartMks {
public:
    static constexpr uint8_t maLenhDocEncoderCarry = 0x30;
    static constexpr uint8_t maLenhDocSaiLechGoc = 0x39;
    static constexpr uint8_t maLenhDocTrangThaiKet = 0x3E;

    static constexpr uint8_t tinhCrc8(const uint8_t* duLieu, unsigned soByte) {
        uint8_t tong = 0;
        for (unsigned i = 0; i < soByte; ++i) {
            tong = static_cast<uint8_t>(tong + duLieu[i]);
        }
        return tong;
    }

    static constexpr void taoKhungHoi(uint8_t diaChi, uint8_t maLenh, uint8_t khungRa[3]) {
        khungRa[0] = diaChi;
        khungRa[1] = maLenh;
        khungRa[2] = static_cast<uint8_t>(diaChi + maLenh);
    }

    static constexpr MauEncoderMksUart giaiMaEncoder0x30(
        const uint8_t* khung, unsigned doDai, uint8_t diaChiMongDoi, uint32_t soXungMoiVong = 65536u) {
        MauEncoderMksUart ketQua{};
        if (!khung || soXungMoiVong == 0u || (doDai != 8u && doDai != 9u)) return ketQua;
        if (khung[0] != diaChiMongDoi) return ketQua;
        if (tinhCrc8(khung, doDai - 1u) != khung[doDai - 1u]) return ketQua;

        unsigned viTriCarry = 1u;
        if (doDai == 9u) {
            if (khung[1] != maLenhDocEncoderCarry) return ketQua;
            viTriCarry = 2u;
        }

        const uint32_t carryThuan =
            (static_cast<uint32_t>(khung[viTriCarry]) << 24) |
            (static_cast<uint32_t>(khung[viTriCarry + 1u]) << 16) |
            (static_cast<uint32_t>(khung[viTriCarry + 2u]) << 8) |
            static_cast<uint32_t>(khung[viTriCarry + 3u]);
        const int32_t carry = static_cast<int32_t>(carryThuan);
        const uint16_t value = static_cast<uint16_t>(
            (static_cast<uint16_t>(khung[viTriCarry + 4u]) << 8) |
            static_cast<uint16_t>(khung[viTriCarry + 5u]));

        if (value >= soXungMoiVong) return ketQua;

        ketQua.soVongCarry = carry;
        ketQua.gocTrongVong = value;
        ketQua.tongXungEncoder =
            static_cast<int64_t>(carry) * static_cast<int64_t>(soXungMoiVong) +
            static_cast<int64_t>(value);
        ketQua.hopLe = true;
        return ketQua;
    }

    // Quy đổi độ dịch chuyển cảm biến từ MKS (mặc định 65536 đơn vị/vòng cho lệnh 0x30) sang số xung vi bước tương đương
    // để hợp nhất không trễ pha với bộ đếm phần cứng PCNT.
    static int64_t quyDoiSangXungBuoc(int64_t deltaXungEncoder,
                                      uint32_t soXungBuocMoiVong,
                                      uint32_t soXungEncoderMoiVong = 65536u) {
        if (soXungEncoderMoiVong == 0u) return 0;
        const double tiLe = static_cast<double>(soXungBuocMoiVong) /
                            static_cast<double>(soXungEncoderMoiVong);
        return static_cast<int64_t>(std::llround(static_cast<double>(deltaXungEncoder) * tiLe));
    }
};

#endif
