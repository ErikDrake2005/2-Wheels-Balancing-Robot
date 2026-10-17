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

McpwmStepOutput xungTrai;
McpwmStepOutput xungPhai;
bool ready = false;
bool active = false;
bool dangChayTrai = false;
bool dangChayPhai = false;
bool dangChoMotXung = false;
uint32_t thoiDiemBatDauChay_ms = 0;

void stopAllSignals() {
    // Dừng khẩn cấp chỉ cho tín hiệu điện; EN đã disable.
    const bool a = xungTrai.stop();
    const bool b = xungPhai.stop();
    active = false;
    dangChoMotXung = false;
    if (!a || !b) Serial.println("STOP API error");
}

void setup() {
    const auto en = static_cast<gpio_num_t>(chanBatMotor);
    // Cần menu EN H/L thật sự khớp; Hold không cho GPIO disable công suất.
    if (gpio_set_level(en, mucBatMotorCao ? 0 : 1) != ESP_OK ||
        gpio_set_direction(en, GPIO_MODE_OUTPUT) != ESP_OK) return;
    Serial.begin(tocDoSerialThu_baud);
    delay(thoiGianKhoiDongConsole_ms);
    const auto dl = static_cast<gpio_num_t>(chanDirTrai);
    const auto dr = static_cast<gpio_num_t>(chanDirPhai);
    if (gpio_set_level(dl, mucDirTienBanhTraiCao ? 1 : 0) != ESP_OK ||
        gpio_set_direction(dl, GPIO_MODE_OUTPUT) != ESP_OK ||
        gpio_set_level(dr, mucDirTienBanhPhaiCao ? 1 : 0) != ESP_OK ||
        gpio_set_direction(dr, GPIO_MODE_OUTPUT) != ESP_OK) return;
    const bool a = xungTrai.begin(chanStepTrai, thoiGianMucCaoStep_us, thoiGianMucThapToiThieuStep_us);
    const bool b = xungPhai.begin(chanStepPhai, thoiGianMucCaoStep_us, thoiGianMucThapToiThieuStep_us);
    if (!a || !b) {
        if (a) xungTrai.stop();
        if (b) xungPhai.stop();
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
    if (active && static_cast<uint32_t>(millis() - thoiDiemBatDauChay_ms) >= thoiGianPhatTinHieuThu_ms) {
        stopAllSignals();
        Serial.println("Signal window expired");
    }
    if (dangChoMotXung && xungTrai.isPauseReady()) {
        const bool ok = xungTrai.emitOnePulse();
        dangChoMotXung = false;
        Serial.println(ok ? "Requested ONE pulse: count rising edge" : "ONE pulse API failed");
    }
    for (uint8_t n = 0; n < 16 && Serial.available(); ++n) {
        const char c = static_cast<char>(Serial.read());
        if (c == 'x') { stopAllSignals(); }
        else if (c == 'z' || c == 'p') {
            const bool a = xungTrai.pauseAfterPulse();
            const bool b = xungPhai.pauseAfterPulse();
            active = false;
            dangChoMotXung = c == 'p' && a && b;
            Serial.println(a && b ? "Pause requested: finish HIGH" : "Pause API failed");
        } else if (c == 'l' || c == 'r' || c == 'b') {
            // Sau pause(), phải đợi isPauseReady(); không đổi DIR ở đây.
            if (!xungTrai.isPauseReady() || !xungPhai.isPauseReady()) {
                Serial.println("Not paused yet: send z, wait, then lenh again");
                continue;
            }
            dangChoMotXung = false;
            dangChayTrai = c != 'r';
            dangChayPhai = c != 'l';
            const bool a = dangChayTrai ? xungTrai.setFrequency(tanSoXungLienTucThu_hz) : true;
            const bool b = dangChayPhai ? xungPhai.setFrequency(tanSoXungLienTucThu_hz) : true;
            if (!a || !b) { stopAllSignals(); continue; }
            active = true;
            thoiDiemBatDauChay_ms = millis();
            Serial.printf("Requested continuous %.0f Hz; EN DISABLE\n", tanSoXungLienTucThu_hz);
        } else if ((c == 'u' || c == 'v') && active) {
            const float hz = c == 'u' ? tanSoXungLienTucThu_hz * 2.0f
                                     : tanSoXungLienTucThu_hz;
            const bool a = dangChayTrai ? xungTrai.setFrequency(hz) : true;
            const bool b = dangChayPhai ? xungPhai.setFrequency(hz) : true;
            if (!a || !b) stopAllSignals();
            else Serial.printf("New requested continuous %.0f Hz\n", hz);
        }
    }
    delay(1); // Console seulement: độ rộng/tần số continuous do hardware.
}
