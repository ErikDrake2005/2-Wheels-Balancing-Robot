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
    // Thời điểm của mẫu, đơn vị microsecond.
    uint64_t ThoiDiemLayMau_us = 0;

    // Số thứ tự, tăng sau mỗi lần đọc thành công.
    uint32_t SoMauLayDuoc = 0;

    // Gia tốc đo bởi accelerometer, đơn vị m/s².
    // Dữ liệu này vẫn bao gồm ảnh hưởng của trọng lực.
    float giatocX_mps2 = 0.0f;
    float giatocY_mps2 = 0.0f;
    float giatocZ_mps2 = 0.0f;

    // Tốc độ góc đo bởi gyro, đơn vị rad/s.
    float tocdoGocX_radps = 0.0f;
    float tocdoGocY_radps = 0.0f;
    float tocdoGocZ_radps = 0.0f;

    // Nhiệt độ của cảm biến, đơn vị °C.
    float nhietdoCamBien_C = 0.0f;

    // Gyro đã được trừ bias từ hiệu chuẩn hay chưa.
    bool trangthaiCalib = false;

    // Mặc định chưa có mẫu sử dụng được.
    trangthaiKhungDuLieuMPU trangthai = trangthaiKhungDuLieuMPU::KhongHopLe;
};

#endif