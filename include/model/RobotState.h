#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <cstdint>

enum class NguonUocLuongQuangDuong : uint8_t {
    KhongCo = 0,
    XungBuoc,
    EncoderBanh
};

struct TrangThaiXe {
    uint64_t thoiDiemGoc_us = 0;
    uint64_t thoiDiemQuangDuong_us = 0;

    float gocNghien_rad = 0.0f;
    float tocDoNghien_radps = 0.0f;
    float gocNghiengNgang_rad = 0.0f; // Góc từ gia tốc, dùng bảo vệ tư thế.
    float tocDoNghiengNgang_radps = 0.0f;
    float doLonGiaToc_mps2 = 0.0f;
    bool giaTocTinCay = false; // Đủ gần trọng lực để khởi tạo/cho phép bật.
    float gocTuGiaToc_rad = 0.0f;
    float saiLechGiaToc_rad = 0.0f;
    float doTinCayGiaToc = 0.0f;
    float gocNghienTong_rad = 0.0f; // Độ lệch trục Z xe khỏi phương đứng, không có dấu.
    float huongNghien_rad = 0.0f; // Hướng nghiêng trong XY xe; 0 khi đứng thẳng (không xác định vật lý).
    float viTriDoc_m = 0.0f;
    float vanTocDoc_mps = 0.0f; // Vận tốc mặt đất ước lượng (đã cộng R*thetaDot nếu bật bù).
    float giaTocDaBu_mps2 = 0.0f; // Gia tốc tịnh tiến đã trừ khỏi accel ở mẫu này (M5).

    // Quãng đường và vận tốc dài từng bánh so với khung xe (R*phi, R*phiDot),
    // dùng cho bộ điều khiển vòng kín bánh xe TMC2209 + MT6701.
    float viTriBanhTrai_m = 0.0f;
    float viTriBanhPhai_m = 0.0f;
    float vanTocBanhTrai_mps = 0.0f;
    float vanTocBanhPhai_mps = 0.0f;

    NguonUocLuongQuangDuong nguonQuangDuong =
        NguonUocLuongQuangDuong::KhongCo;

    bool gocHopLe = false;
    bool viTriHopLe = false;
    bool vanTocHopLe = false;
};

#endif
