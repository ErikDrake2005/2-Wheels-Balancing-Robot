// MAIN ĐO GPIO STEP CHO BACKEND V2, EN LUÔN DISABLE.
// Bạn tự dùng nội dung này làm src/main.cpp khi đo tín hiệu bằng logic analyzer.
// Không chạy đồng thời với main_config_compare_v2.cpp.
// Không tạo MksServo42CPair trong phép thử này: backend trực tiếp sở hữu STEP.

#include <Arduino.h>
#include "driver/gpio.h"
#include "McpwmStepOutput.h"
#include "config/MotorConfig.h"
#include "config/PinConfig.h"
#include "config/ConfigCompareSettings.h"

McpwmStepOutput leftPulse;
McpwmStepOutput rightPulse;
bool ready = false;
bool active = false;
bool leftActive = false;
bool rightActive = false;
bool singlePending = false;
uint32_t runStartedMs = 0;

void stopAllSignals() {
    // Dừng khẩn cấp chỉ cho tín hiệu điện; EN đã disable.
    const bool a = leftPulse.stop();
    const bool b = rightPulse.stop();
    active = false;
    singlePending = false;
    if (!a || !b) Serial.println("STOP API error");
}

void setup() {
    const auto en = static_cast<gpio_num_t>(PIN_EnMotor);
    // Cần menu EN H/L thật sự khớp; Hold không cho GPIO disable công suất.
    if (gpio_set_level(en, MOTOR_EN_ACTIVE_HIGH ? 0 : 1) != ESP_OK ||
        gpio_set_direction(en, GPIO_MODE_OUTPUT) != ESP_OK) return;
    Serial.begin(COMPARE_SERIAL_BAUD);
    delay(COMPARE_CONSOLE_STARTUP_MS);
    const auto dl = static_cast<gpio_num_t>(PIN_DirL);
    const auto dr = static_cast<gpio_num_t>(PIN_DirR);
    if (gpio_set_level(dl, MOTOR_LEFT_FORWARD_DIR_HIGH ? 1 : 0) != ESP_OK ||
        gpio_set_direction(dl, GPIO_MODE_OUTPUT) != ESP_OK ||
        gpio_set_level(dr, MOTOR_RIGHT_FORWARD_DIR_HIGH ? 1 : 0) != ESP_OK ||
        gpio_set_direction(dr, GPIO_MODE_OUTPUT) != ESP_OK) return;
    const bool a = leftPulse.begin(PIN_StepL, MOTOR_STEP_HIGH_US, MOTOR_STEP_MIN_LOW_US);
    const bool b = rightPulse.begin(PIN_StepR, MOTOR_STEP_HIGH_US, MOTOR_STEP_MIN_LOW_US);
    if (!a || !b) {
        if (a) leftPulse.stop();
        if (b) rightPulse.stop();
        Serial.println("FAIL: backend begin");
        return;
    }
    ready = true;
    Serial.println("EN DISABLE: l/r/b=continuous trai/phai/hai; p=1 xung trai; z=pause; x=force stop");
    Serial.println("u/v=doi tan so continuous len/xuong; waveform can do bang logic analyzer.");
}

void loop() {
    if (!ready) { delay(100); return; }
    // EN disable suốt chương trình: không có lệnh enable ở đây.
    if (active && static_cast<uint32_t>(millis() - runStartedMs) >= COMPARE_SIGNAL_RUN_MS) {
        stopAllSignals();
        Serial.println("Signal window expired");
    }
    if (singlePending && leftPulse.isPauseReady()) {
        const bool ok = leftPulse.emitOnePulse();
        singlePending = false;
        Serial.println(ok ? "Requested ONE pulse: count rising edge" : "ONE pulse API failed");
    }
    for (uint8_t n = 0; n < 16 && Serial.available(); ++n) {
        const char c = static_cast<char>(Serial.read());
        if (c == 'x') { stopAllSignals(); }
        else if (c == 'z' || c == 'p') {
            const bool a = leftPulse.pauseAfterPulse();
            const bool b = rightPulse.pauseAfterPulse();
            active = false;
            singlePending = c == 'p' && a && b;
            Serial.println(a && b ? "Pause requested: finish HIGH" : "Pause API failed");
        } else if (c == 'l' || c == 'r' || c == 'b') {
            // Sau pause(), phải đợi isPauseReady(); không đổi DIR ở đây.
            if (!leftPulse.isPauseReady() || !rightPulse.isPauseReady()) {
                Serial.println("Not paused yet: send z, wait, then command again");
                continue;
            }
            singlePending = false;
            leftActive = c != 'r';
            rightActive = c != 'l';
            const bool a = leftActive ? leftPulse.setFrequency(COMPARE_CONTINUOUS_RATE_HZ) : true;
            const bool b = rightActive ? rightPulse.setFrequency(COMPARE_CONTINUOUS_RATE_HZ) : true;
            if (!a || !b) { stopAllSignals(); continue; }
            active = true;
            runStartedMs = millis();
            Serial.printf("Requested continuous %.0f Hz; EN DISABLE\n", COMPARE_CONTINUOUS_RATE_HZ);
        } else if ((c == 'u' || c == 'v') && active) {
            const float hz = c == 'u' ? COMPARE_CONTINUOUS_RATE_HZ * 2.0f
                                     : COMPARE_CONTINUOUS_RATE_HZ;
            const bool a = leftActive ? leftPulse.setFrequency(hz) : true;
            const bool b = rightActive ? rightPulse.setFrequency(hz) : true;
            if (!a || !b) stopAllSignals();
            else Serial.printf("New requested continuous %.0f Hz\n", hz);
        }
    }
    delay(1); // Console seulement: độ rộng/tần số continuous do hardware.
}
