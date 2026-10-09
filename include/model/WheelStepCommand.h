#ifndef WHEEL_STEP_COMMAND_H
#define WHEEL_STEP_COMMAND_H
#include <cstdint>

// Một lệnh mới nhất cho cả cặp bánh.
struct WheelStepCommand {
    // Dương: xe tiến; âm: xe lùi. Mỗi bánh có ánh xạ DIR riêng.
    // Đơn vị steps/s theo microstep đầu vào của MKS, không phải RPM.
    float leftStepsPerSecond = 0.0f;
    float rightStepsPerSecond = 0.0f;
    uint64_t issuedAtUs = 0;
    uint64_t expiresAtUs = 0;
};
#endif
