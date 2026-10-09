#ifndef CONFIG_COMPARE_SETTINGS_H
#define CONFIG_COMPARE_SETTINGS_H
#include <cstdint>

// Đặt file này vào include/config/ConfigCompareSettings.h khi tự dùng main mẫu.
// Giữ false cho đến khi đã xác nhận các mức điện và đo STEP.
constexpr bool COMPARE_ALLOW_MOTOR_RUN = false;

// Hai phép thử: xung thưa và xung liên tục, đơn vị steps/s.
constexpr float COMPARE_SPARSE_RATE_HZ = 200.0f;
constexpr float COMPARE_CONTINUOUS_RATE_HZ = 2000.0f;

// Chỉ cho main_step_signal_only_v2.cpp: thời gian phát STEP tối đa.
// EN vẫn luôn disable trong chương trình đo tín hiệu này.
constexpr uint32_t COMPARE_SIGNAL_RUN_MS = 1000;

// Command Serial chỉ yêu cầu chạy tối đa khoảng này. Main tạo các command
// điều khiển mới mỗi 5 ms trong cửa sổ này, rồi disable khi hết hạn.
constexpr uint32_t COMPARE_RUN_WINDOW_US = 1000000;
constexpr uint32_t COMPARE_COMMAND_REFRESH_US = 5000;

constexpr uint32_t COMPARE_IMU_STALE_US = 20000;
constexpr uint32_t COMPARE_REPORT_MS = 1000;
constexpr uint32_t COMPARE_SERIAL_BAUD = 115200;
constexpr uint32_t COMPARE_CONSOLE_STARTUP_MS = 500;
constexpr uint32_t COMPARE_IMU_TASK_STACK_BYTES = 4096;
constexpr uint32_t COMPARE_MOTOR_TASK_STACK_BYTES = 4096;
constexpr unsigned COMPARE_IMU_TASK_PRIORITY = 3;
constexpr unsigned COMPARE_MOTOR_TASK_PRIORITY = 4;

// Phần nhập tay để tính RPM/m/s từ xung đầu vào MKS.
// Giá trị 0 là CHƯA BIẾT, main không chia cho 0 hoặc khẳng định RPM.
constexpr uint16_t COMPARE_LEFT_FULL_STEPS_PER_REV = 0;
constexpr uint16_t COMPARE_RIGHT_FULL_STEPS_PER_REV = 0;
constexpr uint16_t COMPARE_LEFT_MICROSTEP = 0;
constexpr uint16_t COMPARE_RIGHT_MICROSTEP = 0;
constexpr float COMPARE_LEFT_MOTOR_REVS_PER_WHEEL_REV = 0.0f;
constexpr float COMPARE_RIGHT_MOTOR_REVS_PER_WHEEL_REV = 0.0f;
constexpr float COMPARE_WHEEL_RADIUS_M = 0.040f;

#endif
