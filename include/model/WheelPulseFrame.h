#pragma once
#include <cstdint>

// Xung tại GPIO STEP, có dấu theo DIR. Đây là ước lượng chuyển động khi MKS bám xung.
struct WheelPulseFrame {
    int64_t trai = 0;
    int64_t phai = 0;
    uint64_t thoiDiem_us = 0;
    bool hopLe = false;
    // Chẩn đoán hồi tiếp từ kế MKS qua UART (0x30):
    int64_t encoderTrai = 0;
    int64_t encoderPhai = 0;
    uint32_t soKhungUartTrai = 0;
    uint32_t soKhungUartPhai = 0;
    uint8_t diaChiUartTrai = 0;
    uint8_t diaChiUartPhai = 0;
    bool coUartTrai = false;
    bool coUartPhai = false;
    // Tính từ reset odometry. soKhungUart* đếm khung hợp lệ mới nhất đã chọn cho mỗi lần đọc FIFO.
    uint32_t soLanHoiUartTrai = 0;
    uint32_t soLanHoiUartPhai = 0;
    uint32_t soLoiGhiUart = 0;
    uint64_t soKheUartBoLo = 0;
    // Đồng hồ MCU: lúc driver nhận đủ 3 byte TX, và lúc MCU đọc/chấp nhận RX (khác lúc cảm biến lấy mẫu).
    uint64_t thoiDiemHoiUartTrai_us = 0;
    uint64_t thoiDiemHoiUartPhai_us = 0;
    uint64_t thoiDiemNhanUartTrai_us = 0;
    uint64_t thoiDiemNhanUartPhai_us = 0;
};
