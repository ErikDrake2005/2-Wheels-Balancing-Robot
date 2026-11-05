#ifndef SENSORFRAME_H
#define SENSORFRAME_H

#include <cstdint>

enum class TrangThaiKhungDuLieuImu : uint8_t {
    KhongHopLe = 0,
    HopLe,
    DuLieuCu,
    LoiIO,
    CanTinhChinhLai
};

struct KhungDuLieuImu {
    // Thời điểm của mẫu, đơn vị microsecond.
    uint64_t thoiDiemLayMau_us = 0;

    // Chương trình chính dùng số thứ tự ngắt data-ready; có thể nhảy khi bỏ lỡ mẫu.
    uint32_t soMauLayDuoc = 0;
    uint64_t soMauBoLo = 0;
    uint32_t soThongBaoTrung = 0;
    uint32_t soLanDocChongNgat = 0;

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
    TrangThaiKhungDuLieuImu trangThai = TrangThaiKhungDuLieuImu::KhongHopLe;
};
// Tương thích chương trình bench MPU6050 cũ; không mang bất kỳ dữ liệu DMP nào.
using KhungDuLieuMPU = KhungDuLieuImu;
using TrangThaiKhungDuLieuMpu = TrangThaiKhungDuLieuImu;

#endif
