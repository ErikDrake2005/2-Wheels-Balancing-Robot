#include "LienKetChanDoanEspNow.h"
#include "config/WifiConfig.h"
#include <Preferences.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <cstring>
#include <cstdio>

LienKetChanDoanEspNow* LienKetChanDoanEspNow::instance = nullptr;

bool LienKetChanDoanEspNow::begin() {
    uint32_t boot;
    do { boot = esp_random(); } while (!boot);
    portENTER_CRITICAL(&mux); state = GiamSatLenhTuXa(boot); portEXIT_CRITICAL(&mux);
    instance = this;
    ready = esp_now_register_send_cb(onSend) == ESP_OK;
    Preferences prefs;
    uint8_t saved[6]{};
    bool exists = false;
    if (prefs.begin("radiodiag", true)) {
        exists = prefs.getBytesLength("peer") == sizeof(saved) && prefs.getBytes("peer", saved, sizeof(saved)) == sizeof(saved);
        prefs.end();
    }
    return ready && (!exists || setPeer(saved));
}

bool LienKetChanDoanEspNow::setPeer(const uint8_t mac[6]) {
    if (!ready || !mac || (mac[0] & 1u)) return false;
    uint8_t combined = 0;
    for (unsigned i = 0; i < 6; ++i) combined |= mac[i];
    if (!combined) return false;
    esp_now_peer_info_t entry{};
    std::memcpy(entry.peer_addr, mac, 6);
    entry.channel = kenhWifiAp;
    entry.ifidx = WIFI_IF_STA;
    if (!esp_now_is_peer_exist(mac) && esp_now_add_peer(&entry) != ESP_OK) return false;
    portENTER_CRITICAL(&mux);
    const bool idle = motorOff && !busy;
    if (idle) {
        std::memcpy(peer, mac, 6);
        configured = true;
        uint32_t boot = 0;
        ChanDoanEspNow::GoiTelemetry t{}; state.telemetry(t); boot = t.boot_id;
        state = GiamSatLenhTuXa(boot);
    }
    portEXIT_CRITICAL(&mux);
    if (!idle) return false;
    std::snprintf(macText, sizeof(macText), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Preferences prefs;
    if (!prefs.begin("radiodiag", false)) return false;
    const bool saved = prefs.putBytes("peer", mac, 6) == 6;
    prefs.end();
    return saved;
}

bool LienKetChanDoanEspNow::receive(const uint8_t* mac, const uint8_t* data, size_t size, uint64_t at) {
    if (!data) return enabled();
    uint32_t magic = 0;
    if (size >= sizeof(magic)) std::memcpy(&magic, data, sizeof(magic));
    // Một khi ghép cầu nối, không cho gói ESP-NOW cũ đi vòng qua kiểm tra phiên/lease.
    uint8_t expected[6]; bool active;
    portENTER_CRITICAL(&mux); active = configured; std::memcpy(expected, peer, 6); portEXIT_CRITICAL(&mux);
    if (!active) return magic == ChanDoanEspNow::magic;
    if (!mac || std::memcmp(mac, expected, 6)) {
        portENTER_CRITICAL(&mux); ++rejectedPackets; portEXIT_CRITICAL(&mux); return true;
    }
    ChanDoanEspNow::GoiHello h{};
    ChanDoanEspNow::GoiLenh c{};
    if (ChanDoanEspNow::giaiMa(data, size, h)) {
        portENTER_CRITICAL(&mux); state.hello(h, motorOff); portEXIT_CRITICAL(&mux);
    } else if (ChanDoanEspNow::giaiMa(data, size, c)) {
        portENTER_CRITICAL(&mux); state.receive(c, at); portEXIT_CRITICAL(&mux);
    } else {
        portENTER_CRITICAL(&mux); ++rejectedPackets; portEXIT_CRITICAL(&mux);
    }
    return true;
}

GiamSatLenhTuXa::Decision LienKetChanDoanEspNow::serviceMotor(uint64_t now, bool en) {
    portENTER_CRITICAL(&mux);
    motorOff = !en;
    auto result = state.service(now, en);
    portEXIT_CRITICAL(&mux);
    return result;
}

bool LienKetChanDoanEspNow::desired(uint64_t now, float& v, float& yaw) {
    portENTER_CRITICAL(&mux);
    const bool takeOver = configured && state.getSession();
    state.desired(now, v, yaw);
    portEXIT_CRITICAL(&mux);
    return takeOver;
}

void LienKetChanDoanEspNow::cancel() {
    portENTER_CRITICAL(&mux); state.cancel(); portEXIT_CRITICAL(&mux);
}

void LienKetChanDoanEspNow::onSend(const wifi_tx_info_t*, esp_now_send_status_t status) {
    if (!instance) return;
    portENTER_CRITICAL(&instance->mux);
    instance->busy = false;
    if (status != ESP_NOW_SEND_SUCCESS) ++instance->failed;
    portEXIT_CRITICAL(&instance->mux);
}

void LienKetChanDoanEspNow::sendTelemetry(ChanDoanEspNow::GoiTelemetry& packet) {
    if (!ready || !enabled()) return;
    // RobotApp chọn nhịp 20 ms. Không chặn thời gian lần hai ở đây vì thời gian
    // chuẩn bị snapshot biến thiên vài us có thể vô tình biến 50 Hz thành 25 Hz.
    portENTER_CRITICAL(&mux);
    state.telemetry(packet);
    packet.rx_reject += rejectedPackets;
    const uint32_t session = state.getSession();
    const bool canSend = session && !busy;
    if (session && busy) ++dropped;
    if (canSend) busy = true;
    packet.tx_drop = dropped; packet.tx_fail = failed;
    portEXIT_CRITICAL(&mux);
    if (!canSend) return;
    if (!++txSeq) ++txSeq;
    ChanDoanEspNow::dongGoi(packet, session, txSeq);
    if (esp_now_send(peer, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet)) != ESP_OK) {
        portENTER_CRITICAL(&mux); busy = false; ++failed; portEXIT_CRITICAL(&mux);
    }
}
