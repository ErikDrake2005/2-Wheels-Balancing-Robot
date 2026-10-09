#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H
#include <cstdint>

// MẪU CẤU HÌNH V2: các giá trị tín hiệu dưới đây cần đối chiếu MKS thực tế.
// begin() vẫn chuẩn bị phần cứng; enable() chỉ được chấp nhận khi true.
constexpr bool MOTOR_HARDWARE_CONFIG_CONFIRMED = false;
constexpr bool MOTOR_EN_ACTIVE_HIGH = false; // Phải khớp menu EN = L.
constexpr bool MOTOR_LEFT_FORWARD_DIR_HIGH = true;  // Chưa xác nhận chiều.
constexpr bool MOTOR_RIGHT_FORWARD_DIR_HIGH = false; // Chưa xác nhận chiều.

constexpr uint32_t MOTOR_STEP_HIGH_US = 10;       // Giá trị thử, không phải datasheet.
constexpr uint32_t MOTOR_STEP_MIN_LOW_US = 10;    // Giá trị thử.
constexpr uint32_t MOTOR_DIR_HOLD_US = 10;        // Giá trị thử.
constexpr uint32_t MOTOR_DIR_SETUP_US = 10;       // Giá trị thử.
constexpr uint32_t MOTOR_ENABLE_SETUP_US = 100;  // Giá trị thử.
constexpr float MOTOR_MAX_STEP_RATE_HZ = 10000.0f; // Trần phần mềm thử nghiệm.

constexpr int MCPWM_GROUP_ID = 0;
constexpr uint32_t MCPWM_TIMER_RESOLUTION_HZ = 1000000;
constexpr uint32_t MCPWM_INITIAL_PERIOD_US = 10000;
constexpr uint32_t MCPWM_PAUSE_GUARD_US = 2;

// >= 1000 steps/s: dùng xung MCPWM liên tục. Chu kỳ <= 1 ms.
// < 1000 steps/s: tích lũy phần bước và phát từng xung MCPWM hoàn chỉnh.
// Không nâng lệnh nhỏ thành một tốc độ tối thiểu lớn hơn.
constexpr float MOTOR_CONTINUOUS_MIN_RATE_HZ = 1000.0f;

// service() cần được task sở hữu motor gọi đều, mục tiêu 250 us.
// Đây là yêu cầu tích hợp, class không tự tạo task hay timer đánh thức.
constexpr uint32_t MOTOR_SERVICE_TARGET_US = 250;
constexpr uint32_t MOTOR_SERVICE_MAX_GAP_US = 2000;
constexpr uint32_t MOTOR_COMMAND_TIMEOUT_US = 20000;

// Không cho tồn đọng cả cụm xung để phát bù sau khi task bị chậm.
constexpr double MOTOR_SPARSE_BACKLOG_LIMIT_STEPS = 2.0;
#endif
