#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include "core/GiamSatLenhTuXa.h"

class LienKetChanDoanEspNow {
public:
    bool begin(); // WiFi/esp_now đã khởi tạo bởi DieuKhienKhongDay.
    bool setPeer(const uint8_t mac[6]); // Chỉ RobotApp gọi khi motor tắt.
    bool enabled() {
        portENTER_CRITICAL(&mux); const bool value = configured; portEXIT_CRITICAL(&mux); return value;
    }
    const char* peerText() const { return macText; }
    bool receive(const uint8_t* mac, const uint8_t* data, size_t size, uint64_t at);
    GiamSatLenhTuXa::Decision serviceMotor(uint64_t now, bool en);
    bool desired(uint64_t now, float& v, float& yaw);
    void cancel();
    void sendTelemetry(ChanDoanEspNow::GoiTelemetry& packet);
private:
    static void onSend(const wifi_tx_info_t*, esp_now_send_status_t status);
    static LienKetChanDoanEspNow* instance;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    GiamSatLenhTuXa state;
    uint8_t peer[6]{};
    char macText[18] = "unset";
    bool ready = false, configured = false, motorOff = true, busy = false;
    uint32_t txSeq = 0, dropped = 0, failed = 0, rejectedPackets = 0;
};
