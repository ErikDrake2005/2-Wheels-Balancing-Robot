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

struct ChanDoanGiaoDichI2c {
    uint8_t thanhGhi = 0, soByteYeuCau = 0, soByteNhan = 0, soByteDaDoc = 0, maLoi = 0;
    uint32_t thoiGian_us = 0;
};

struct ChanDoanLuongImu {
    uint32_t soThuTuNgat = 0;
    uint64_t thoiDiemNgatCuoi_us = 0, thoiDiemDocTotCuoi_us = 0;
    uint32_t soMauTot = 0, soLanHetChoNgat = 0, soLoiI2c = 0, soLanKhongCoDataReady = 0;
    uint32_t soLanAckPhucHoi = 0;
    uint32_t soLanDocQuaChuKy = 0, soLanDocChongNgat = 0;
    uint8_t maLoiI2cCuoi = 0, thanhGhiLoiCuoi = 0;
    TrangThaiKhungDuLieuImu trangThaiDocCuoi = TrangThaiKhungDuLieuImu::KhongHopLe;
    ChanDoanGiaoDichI2c giaoDichCuoi{}, giaoDichLoiCuoi{};
    uint32_t thoiGianDocLonNhat_us = 0;
};

struct KhungDuLieuImu {
    // Thời điểm của mẫu, đơn vị microsecond.
    uint64_t thoiDiemLayMau_us = 0;

    // Chương trình chính dùng số thứ tự ngắt data-ready; có thể nhảy khi bỏ lỡ mẫu.
    uint32_t soMauLayDuoc = 0;
    uint64_t soMauBoLo = 0;
    uint32_t soThongBaoTrung = 0;
    uint32_t soLanDocChongNgat = 0;
    // Bộ đếm runtime vẫn cập nhật trên frame lỗi, không nhầm miss với sequence.
    ChanDoanLuongImu chanDoan{};

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
