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
    float viTriDoc_m = 0.0f;
    float vanTocDoc_mps = 0.0f;

    NguonUocLuongQuangDuong nguonQuangDuong =
        NguonUocLuongQuangDuong::KhongCo;

    bool gocHopLe = false;
    bool viTriHopLe = false;
    bool vanTocHopLe = false;
};

#endif
