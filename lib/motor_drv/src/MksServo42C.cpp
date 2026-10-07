#include "MksServo42C.h"
#include <Arduino.h>
#include "config/PinConfig.h"
#include "config/MotorConfig.h"

bool MksServo42CPair::begin() {
    _initialized = false;
    _enabled = false;
    // Đặt mức disable trước khi chuyển EN thành OUTPUT.
    digitalWrite(PIN_EnMotor,MOTOR_EN_ACTIVE_HIGH ? LOW : HIGH);
    pinMode(PIN_EnMotor, OUTPUT);

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