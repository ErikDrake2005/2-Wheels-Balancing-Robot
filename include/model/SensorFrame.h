#ifndef SENSORFRAME_H
#define SENSORFRAME_H

#include <cstdint>

enum class TrangThaiKhungDuLieuMpu : uint8_t {
    KhongHopLe = 0,
    HopLe,
    DuLieuCu,
    LoiIO,
    CanTinhChinhLai
};

struct KhungDuLieuMPU {
    // Thời điểm của mẫu, đơn vị microsecond.
    uint64_t thoiDiemLayMau_us = 0;

    // Số thứ tự, tăng sau mỗi lần đọc thành công.
    uint32_t soMauLayDuoc = 0;

    // Gia tốc đo bởi accelerometer, đơn vị m/s².
    // Dữ liệu này vẫn bao gồm ảnh hưởng của trọng lực.
    float giaTocX_mps2 = 0.0f;
    float giaTocY_mps2 = 0.0f;
    float giaTocZ_mps2 = 0.0f;

    // Tốc độ góc đo bởi gyro, đơn vị rad/s.
    float tocDoGocX_radps = 0.0f;
    float tocDoGocY_radps = 0.0f;
    float tocDoGocZ_radps = 0.0f;

    // Nhiệt độ của cảm biến, đơn vị °C.
    float nhietDoCamBien_C = 0.0f;

    // Gyro đã được trừ bias từ hiệu chuẩn hay chưa.
    bool daHieuChinhConQuay = false;

    // Mặc định chưa có mẫu sử dụng được.
    TrangThaiKhungDuLieuMpu trangThai = TrangThaiKhungDuLieuMpu::KhongHopLe;
};

#endif
