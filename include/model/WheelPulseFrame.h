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
};
