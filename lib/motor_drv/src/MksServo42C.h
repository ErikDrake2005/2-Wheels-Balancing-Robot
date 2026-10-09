#ifndef MKS_SERVO42C_H
#define MKS_SERVO42C_H
#include <cstdint>
#include "McpwmStepOutput.h"
#include "model/WheelStepCommand.h"

enum class MksResult : uint8_t {
    Ok, Saturated, Pending, Disabled, NotInitialized, ConfigInvalid,
    HardwareConfigUnconfirmed, FaultLatched
};
enum class MksFault : uint8_t {
    None, InvalidCommand, CommandExpired, ServiceLate, BackendError, SparseBacklog,
    EmergencyStop
};

struct MksSnapshot {
    bool initialized = false;
    bool enabled = false; // Mức EN mà phần mềm đã đặt, không phải phản hồi MKS.
    bool pending = false;
    bool saturated = false;
    MksFault fault = MksFault::None;
    float requestedLeftStepsPerSecond = 0.0f;
    float requestedRightStepsPerSecond = 0.0f;
    double continuousLeftHz = 0.0;
    double continuousRightHz = 0.0;
    uint64_t expiresAtUs = 0;
};

// Chủ duy nhất của hai STEP, hai DIR và EN chung.
// Gọi API từ cùng một MotorTask; không gọi từ ISR.
// ISR/điều khiển/safety khác gửi lệnh hoặc notification tới task sở hữu.
class MksServo42CPair {
public:
    bool begin();
    MksResult enable();
    bool disable();
    bool isEnabled() const { return _enabled; }

    // Kiểm tra toàn bộ lệnh trước khi cập nhật hai mục tiêu.
    // Không tự enable. Lệnh mới thay lệnh cũ, không có queue trong class.
    MksResult apply(const WheelStepCommand& command);

    // Dừng chuyển động bình thường, giữ EN/lực giữ.
    MksResult stopMotion();

    // Gọi định kỳ theo MOTOR_SERVICE_TARGET_US và sau khi có lệnh mới.
    // Tiến trình đổi DIR/STEP và phát bước thưa đều không dùng delay().
    MksResult service();

    // Ngắt EN và STEP ngay; fault giữ lại tới khi clearFault().
    bool emergencyStop();
    bool clearFault(); // Chỉ khi disabled; không tự enable lại.
    MksSnapshot snapshot() const;

private:
    enum class Phase : uint8_t { Idle, Running, Pausing, DirSetup };
    enum class Mode : uint8_t { None, Continuous, Sparse };
    struct Axis {
        McpwmStepOutput step;
        int dirPin = -1;
        bool forwardDirHigh = true;
        bool forward = true;
        float target = 0.0f;
        Phase phase = Phase::Idle;
        Mode mode = Mode::None;
        uint64_t waitUntilUs = 0;
        // Bước chưa đủ một xung, tích lũy riêng cho mỗi bánh.
        double credit = 0.0;
        float configuredRateHz = 0.0f; // Tránh gọi API MCPWM lặp khi mục tiêu không đổi.
    };
    bool advance(Axis& axis, double dtSeconds, uint64_t nowUs);
    bool requestPause(Axis& axis);
    bool writeDirection(Axis& axis, bool forward);
    void trip(MksFault reason);
    bool pending() const;

    Axis _left;
    Axis _right;
    bool _initialized = false;
    bool _enabled = false;
    bool _saturated = false;
    MksFault _fault = MksFault::None;
    uint64_t _deadlineUs = 0;
    uint64_t _enableReadyUs = 0;
    uint64_t _lastServiceUs = 0;
    uint64_t _lastCommandIssuedUs = 0;
};
#endif
