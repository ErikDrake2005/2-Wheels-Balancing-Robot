#ifndef SENSORFRAME_H
#define SENSORFRAME_H

#include <cstdint>

enum class trangthaiKhungDuLieuMPU : uint8_t {
    KhongHopLe = 0,
    HopLe,
    DuLieuCu,
    LoiIO,
    CanTinhChinhLai
};

struct KhungDuLieuMPU {
    uint64_t ThoiDiemLayMau_us = 0;
    uint32_t SoMauLayDuoc = 0;
    float KhoangThoiGianLayMau_dt = 0.0f;

    float giatocX_mps2 = 0.0f;
    float giatocY_mps2 = 0.0f;
    float giatocZ_mps2 = 0.0f;

    float giatocgocX_rps = 0.0f;
    float giatocgocY_rps = 0.0f;
    float giatocgocZ_rps = 0.0f;

    float nhietdoCamBien_C = 0.0f;
    bool trangthaiCalib = false;
    bool SENSOR_OK = false;

    trangthaiKhungDuLieuMPU trangthai =
        trangthaiKhungDuLieuMPU::KhongHopLe;
};

#endif