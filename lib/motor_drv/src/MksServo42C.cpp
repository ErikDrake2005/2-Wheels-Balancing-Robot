#include "MksServo42C.h"
#include "config/MotorConfig.h"
#include "config/PinConfig.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <cmath>
#include <initializer_list>

namespace {
uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
}

bool MksServo42CPair::begin() {
    if (_initialized) return false;
    if (!std::isfinite(MOTOR_MAX_STEP_RATE_HZ) || MOTOR_MAX_STEP_RATE_HZ <= 0.0f ||
        !std::isfinite(MOTOR_CONTINUOUS_MIN_RATE_HZ) || MOTOR_CONTINUOUS_MIN_RATE_HZ <= 0.0f ||
        MOTOR_CONTINUOUS_MIN_RATE_HZ > MOTOR_MAX_STEP_RATE_HZ ||
        MOTOR_SERVICE_TARGET_US == 0 || MOTOR_COMMAND_TIMEOUT_US == 0 ||
        !std::isfinite(MOTOR_SPARSE_BACKLOG_LIMIT_STEPS) || MOTOR_SPARSE_BACKLOG_LIMIT_STEPS <= 1.0 ||
        MOTOR_SERVICE_MAX_GAP_US < MOTOR_SERVICE_TARGET_US ||
        MOTOR_CONTINUOUS_MIN_RATE_HZ * MOTOR_SERVICE_TARGET_US / 1000000.0f >= 1.0f) return false;

    const int pins[] = {PIN_StepL, PIN_DirL, PIN_StepR, PIN_DirR, PIN_EnMotor};
    for (unsigned i = 0; i < 5; ++i) {
        if (pins[i] < 0 || !GPIO_IS_VALID_OUTPUT_GPIO(pins[i])) return false;
        for (unsigned j = 0; j < i; ++j) if (pins[i] == pins[j]) return false;
    }

    const auto enPin = static_cast<gpio_num_t>(PIN_EnMotor);
    if (gpio_set_level(enPin, MOTOR_EN_ACTIVE_HIGH ? 0 : 1) != ESP_OK ||
        gpio_set_direction(enPin, GPIO_MODE_OUTPUT) != ESP_OK) return false;

    _left.dirPin = PIN_DirL;
    _right.dirPin = PIN_DirR;
    _left.forwardDirHigh = MOTOR_LEFT_FORWARD_DIR_HIGH;
    _right.forwardDirHigh = MOTOR_RIGHT_FORWARD_DIR_HIGH;
    for (Axis* a : {&_left, &_right}) {
        const auto pin = static_cast<gpio_num_t>(a->dirPin);
        if (gpio_set_level(pin, a->forwardDirHigh ? 1 : 0) != ESP_OK ||
            gpio_set_direction(pin, GPIO_MODE_OUTPUT) != ESP_OK) return false;
    }

    const bool leftOk = _left.step.begin(PIN_StepL, MOTOR_STEP_HIGH_US, MOTOR_STEP_MIN_LOW_US);
    const bool rightOk = _right.step.begin(PIN_StepR, MOTOR_STEP_HIGH_US, MOTOR_STEP_MIN_LOW_US);
    if (!leftOk || !rightOk) {
        if (leftOk) _left.step.stop();
        if (rightOk) _right.step.stop();
        _fault = MksFault::BackendError;
        return false; // Không bật EN; reset sau khi sửa nguyên nhân.
    }
    if (!_left.step.supportsFrequency(MOTOR_CONTINUOUS_MIN_RATE_HZ) ||
        !_left.step.supportsFrequency(MOTOR_MAX_STEP_RATE_HZ) ||
        !_right.step.supportsFrequency(MOTOR_CONTINUOUS_MIN_RATE_HZ) ||
        !_right.step.supportsFrequency(MOTOR_MAX_STEP_RATE_HZ)) {
        _left.step.stop(); _right.step.stop();
        return false;
    }
    _initialized = true;
    return true;
}

MksResult MksServo42CPair::enable() {
    if (!_initialized) return MksResult::NotInitialized;
    if (_fault != MksFault::None) return MksResult::FaultLatched;
    if (!MOTOR_HARDWARE_CONFIG_CONFIRMED) return MksResult::HardwareConfigUnconfirmed;
    if (_enabled) return MksResult::Ok;
    if (!_left.step.isPauseReady() || !_right.step.isPauseReady()) return MksResult::Pending;
    if (gpio_set_level(static_cast<gpio_num_t>(PIN_EnMotor), MOTOR_EN_ACTIVE_HIGH ? 1 : 0) != ESP_OK) {
        trip(MksFault::BackendError);
        return MksResult::FaultLatched;
    }
    const uint64_t now = nowUs();
    _enabled = true;
    _enableReadyUs = now + MOTOR_ENABLE_SETUP_US;
    _deadlineUs = now + MOTOR_COMMAND_TIMEOUT_US;
    _lastServiceUs = now;
    _lastCommandIssuedUs = 0;
    return MksResult::Ok;
}

bool MksServo42CPair::disable() {
    // EN off trước khi có thể cắt xung HIGH: thao tác bảo vệ, không phải đảo chiều.
    const bool enOk = gpio_set_level(static_cast<gpio_num_t>(PIN_EnMotor),
                                   MOTOR_EN_ACTIVE_HIGH ? 0 : 1) == ESP_OK;
    bool stepOk = true;
    if (_initialized) {
        const bool a = _left.step.stop();
        const bool b = _right.step.stop();
        stepOk = a && b;
    }
    _enabled = false;
    _deadlineUs = 0;
    _lastServiceUs = 0;
    _saturated = false;
    for (Axis* a : {&_left, &_right}) {
        a->target = 0.0f;
        a->credit = 0.0;
        a->phase = Phase::Idle;
        a->mode = Mode::None;
        a->configuredRateHz = 0.0f;
    }
    if (!enOk || !stepOk) _fault = MksFault::BackendError;
    return enOk && stepOk;
}

void MksServo42CPair::trip(MksFault reason) {
    // Giữ nguyên nguyên nhân đầu tiên, trừ khi ngay thao tác disable cũng lỗi.
    if (_fault == MksFault::None) _fault = reason;
    disable();
}

bool MksServo42CPair::emergencyStop() {
    if (_fault == MksFault::None) _fault = MksFault::EmergencyStop;
    return disable();
}

bool MksServo42CPair::clearFault() {
    if (!_initialized || _enabled) return false;
    if (!_left.step.stop() || !_right.step.stop()) return false;
    _fault = MksFault::None;
    _lastCommandIssuedUs = 0;
    return true;
}

bool MksServo42CPair::requestPause(Axis& a) {
    if (a.phase == Phase::Pausing) return true;
    if (!a.step.pauseAfterPulse()) return false;
    // Đếm từ SAU khi đã cấm cạnh lên, bảo đảm DIR hold tối thiểu.
    a.waitUntilUs = nowUs() + MOTOR_DIR_HOLD_US;
    a.phase = Phase::Pausing;
    return true;
}

bool MksServo42CPair::writeDirection(Axis& a, bool forward) {
    const bool high = forward ? a.forwardDirHigh : !a.forwardDirHigh;
    if (gpio_set_level(static_cast<gpio_num_t>(a.dirPin), high ? 1 : 0) != ESP_OK) return false;
    a.forward = forward;
    a.waitUntilUs = nowUs() + MOTOR_DIR_SETUP_US;
    a.phase = Phase::DirSetup;
    return true;
}

bool MksServo42CPair::advance(Axis& a, double dtSeconds, uint64_t now) {
    const float hz = std::fabs(a.target);
    const bool desiredForward = a.target >= 0.0f;
    const Mode desiredMode = hz >= MOTOR_CONTINUOUS_MIN_RATE_HZ ? Mode::Continuous : Mode::Sparse;

    if (a.phase == Phase::Running &&
        (hz == 0.0f || desiredForward != a.forward || desiredMode != a.mode)) {
        return requestPause(a);
    }
    if (a.phase == Phase::Pausing) {
        if (!a.step.isPauseReady() || now < a.waitUntilUs) return true;
        a.phase = Phase::Idle;
        a.mode = Mode::None;
        a.configuredRateHz = 0.0f;
        // Không phát bù phần bước cũ sau khi đổi chiều/chế độ.
        a.credit = 0.0;
    }
    if (hz == 0.0f) {
        a.credit = 0.0;
        a.phase = Phase::Idle;
        a.mode = Mode::None;
        return true;
    }
    if (desiredForward != a.forward) return writeDirection(a, desiredForward);
    if (a.phase == Phase::DirSetup) {
        if (now < a.waitUntilUs) return true;
        a.phase = Phase::Idle;
    }
    // Kể cả mục tiêu đã về 0 rồi tăng lại trong lúc DirSetup,
    // vẫn không bỏ qua khoảng ổn định DIR đã đặt trước đó.
    if (now < _enableReadyUs || now < a.waitUntilUs) return true;

    if (a.phase == Phase::Idle) {
        if (!a.step.isPauseReady()) return true;
        a.mode = desiredMode;
        a.phase = Phase::Running;
        if (a.mode == Mode::Continuous) {
            if (!a.step.setFrequency(hz)) return false;
            a.configuredRateHz = hz;
            return true;
        }
        a.credit = 0.0;
        return true; // Bắt đầu tích lũy từ service tiếp theo.
    }
    if (a.mode == Mode::Continuous) {
        if (hz == a.configuredRateHz) return true;
        if (!a.step.setFrequency(hz)) return false;
        a.configuredRateHz = hz;
        return true;
    }

    // Xung thưa: 0.25 step + 0.25 step + 0.25 step + 0.25 step => một STEP.
    // Không bỏ các tốc độ nhỏ, không tăng tốc độ lên min frequency của timer.
    a.credit += static_cast<double>(hz) * dtSeconds;
    if (a.credit >= MOTOR_SPARSE_BACKLOG_LIMIT_STEPS) {
        trip(MksFault::SparseBacklog); // Task bị chậm; không phát một cụm bù giật.
        return false;
    }
    if (a.credit >= 1.0 && a.step.isPauseReady()) {
        if (!a.step.emitOnePulse()) return false;
        a.credit -= 1.0;
    }
    return true;
}

bool MksServo42CPair::pending() const {
    for (const Axis* a : {&_left, &_right}) {
        if (a->phase == Phase::Pausing || a->phase == Phase::DirSetup ||
            (a->phase == Phase::Idle && a->target != 0.0f)) return true;
    }
    return false;
}

MksResult MksServo42CPair::service() {
    if (!_initialized) return MksResult::NotInitialized;
    if (_fault != MksFault::None) return MksResult::FaultLatched;
    if (!_enabled) return MksResult::Disabled;
    const uint64_t now = nowUs();
    if (now >= _deadlineUs) {
        trip(MksFault::CommandExpired);
        return MksResult::FaultLatched;
    }
    const uint64_t gapUs = now - _lastServiceUs;
    if (gapUs > MOTOR_SERVICE_MAX_GAP_US) {
        trip(MksFault::ServiceLate);
        return MksResult::FaultLatched;
    }
    _lastServiceUs = now;
    const double dt = static_cast<double>(gapUs) / 1000000.0;
    if (!advance(_left, dt, now) || !advance(_right, dt, now)) {
        trip(MksFault::BackendError);
        return MksResult::FaultLatched;
    }
    return pending() ? MksResult::Pending : MksResult::Ok;
}

MksResult MksServo42CPair::apply(const WheelStepCommand& c) {
    if (!_initialized) return MksResult::NotInitialized;
    if (_fault != MksFault::None) return MksResult::FaultLatched;
    if (!_enabled) return MksResult::Disabled;

    // Tích lũy khoảng vừa qua theo mục tiêu CŨ trước khi nhận mục tiêu mới.
    if (service() == MksResult::FaultLatched) return MksResult::FaultLatched;
    const uint64_t now = nowUs();
    if (!std::isfinite(c.leftStepsPerSecond) || !std::isfinite(c.rightStepsPerSecond) ||
        c.issuedAtUs > now || c.expiresAtUs <= c.issuedAtUs ||
        c.issuedAtUs < _lastCommandIssuedUs) {
        trip(MksFault::InvalidCommand);
        return MksResult::FaultLatched;
    }
    const uint64_t age = now - c.issuedAtUs;
    if (c.expiresAtUs <= now || age >= MOTOR_COMMAND_TIMEOUT_US) {
        trip(MksFault::CommandExpired);
        return MksResult::FaultLatched;
    }

    // Giới hạn cả cặp theo cùng tỷ lệ, tránh làm lệch tỷ lệ trái/phải.
    float left = c.leftStepsPerSecond;
    float right = c.rightStepsPerSecond;
    const float largest = std::fmax(std::fabs(left), std::fabs(right));
    _saturated = largest > MOTOR_MAX_STEP_RATE_HZ;
    if (_saturated) {
        const float ratio = MOTOR_MAX_STEP_RATE_HZ / largest;
        left *= ratio;
        right *= ratio;
    }
    _left.target = left;
    _right.target = right;
    _lastCommandIssuedUs = c.issuedAtUs;
    // Hạn tính từ issuedAt, không gia hạn mãi bằng cách gửi lại cùng lệnh cũ.
    const uint64_t remaining = MOTOR_COMMAND_TIMEOUT_US - age;
    const uint64_t configuredDeadline = now + remaining;
    _deadlineUs = c.expiresAtUs < configuredDeadline ? c.expiresAtUs : configuredDeadline;
    const MksResult progress = service();
    if (progress == MksResult::FaultLatched) return progress;
    return _saturated ? MksResult::Saturated : progress;
}

MksResult MksServo42CPair::stopMotion() {
    if (!_initialized) return MksResult::NotInitialized;
    if (_fault != MksFault::None) return MksResult::FaultLatched;
    _left.target = 0.0f;
    _right.target = 0.0f;
    return service(); // Không gia hạn watchdog; controller vẫn phải gửi lệnh mới.
}

MksSnapshot MksServo42CPair::snapshot() const {
    MksSnapshot s{};
    s.initialized = _initialized;
    s.enabled = _enabled;
    s.pending = pending();
    s.saturated = _saturated;
    s.fault = _fault;
    s.requestedLeftStepsPerSecond = _left.target;
    s.requestedRightStepsPerSecond = _right.target;
    s.continuousLeftHz = _left.step.actualFrequencyHz();
    s.continuousRightHz = _right.step.actualFrequencyHz();
    s.expiresAtUs = _deadlineUs;
    return s;
}
