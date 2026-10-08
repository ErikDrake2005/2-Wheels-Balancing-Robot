#include "MksServo42C.h"
#include <Arduino.h>
#include "config/PinConfig.h"
#include "config/MotorConfig.h"
#include "driver/gpio.h"

bool MksServo42CPair::begin() {
    _initialized = false;
    _enabled = false;
    const gpio_num_t enPin = static_cast<gpio_num_t>(PIN_EnMotor);
    const int disableLevel = MOTOR_EN_ACTIVE_HIGH ? 0 : 1;
    // Ghi trực tiếp mức đầu ra trước khi bật chế độ OUTPUT.
    if (gpio_set_level(enPin, disableLevel) != ESP_OK) return false;
    // Đăng ký chân với Arduino để các digitalWrite() sau hoạt động.
    pinMode(PIN_EnMotor, OUTPUT);
    // Đặt lại mức disable sau khi chân đã được cấu hình.
    digitalWrite(PIN_EnMotor, disableLevel ? HIGH : LOW);
    // Chuẩn bị STEP ở LOW, chưa phát xung.
    digitalWrite(PIN_StepL, LOW);
    digitalWrite(PIN_StepR, LOW);
    pinMode(PIN_StepL, OUTPUT);
    pinMode(PIN_StepR, OUTPUT);

    // Chiều mặc định lúc khởi tạo.
    digitalWrite(PIN_DirL, LOW);
    digitalWrite(PIN_DirR, LOW);
    pinMode(PIN_DirL, OUTPUT);
    pinMode(PIN_DirR, OUTPUT);
    _initialized = true;
    return true;
}

void MksServo42CPair::enable() {
    if (!_initialized) return;
    digitalWrite(PIN_EnMotor,MOTOR_EN_ACTIVE_HIGH ? HIGH : LOW);
    _enabled = true;
}

void MksServo42CPair::disable() {
    if (!_initialized) return;
    // Tắt công suất driver.
    digitalWrite(PIN_EnMotor,MOTOR_EN_ACTIVE_HIGH ? LOW : HIGH);
    digitalWrite(PIN_StepL, LOW);
    digitalWrite(PIN_StepR, LOW);
    _enabled = false;
}