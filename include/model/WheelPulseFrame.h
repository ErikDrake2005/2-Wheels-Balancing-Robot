#pragma once
#include <cstdint>

// Xung tại GPIO STEP, có dấu theo DIR. Đây là ước lượng chuyển động khi MKS bám xung.
struct WheelPulseFrame {
    int64_t trai = 0;
    int64_t phai = 0;
    uint64_t thoiDiem_us = 0;
    bool hopLe = false;
};
