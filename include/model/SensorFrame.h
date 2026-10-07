#ifndef SENSORFRAME_H
#define SENSORFRAME_H

#include <cstdint>

enum class trangthaiKhungDuLieuMPU:uint8_t{
    KhongHopLe=0,
    HopLe,
    DuLieuCu,
    LoiIO,
    CanTinhChinhLai
};

struct KhungDuLieuMPU{
    uint64_t ThoiDiemLayMau_us;
    uint32_t SoMauLayDuoc;
    float KhoangThoiGianLayMau_dt;

    float giatocX_mps2;
    float giatocY_mps2;
    float giatocZ_mps2;

    float giatocgocX_rps;
    float giatocgocY_rps;
    float giatocgocZ_rps;

    float nhietdoCamBien_C;

    bool trangthaiCalib;

    bool SENSOR_OK;

    trangthaiKhungDuLieuMPU trangthai;
};

#endif