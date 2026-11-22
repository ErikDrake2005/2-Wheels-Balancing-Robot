#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include "Settings.h"
#include "GiaoThucKhaoSatRung.h"

namespace PassiveReceiver {
constexpr size_t MaxLineBytes = 1024;
constexpr size_t OutputSlots = 16;
constexpr size_t RadioQueueSlots = 32;
constexpr size_t UsbWriteChunk = 256;
constexpr size_t UsbWriteBudget = 1024;
constexpr char PassiveWheelFirmwarePrefix[] = "lsm-smooth-";

struct RadioFrame {
    uint8_t mac[6]{};
    uint64_t receivedUs = 0;
    int8_t rssi = -128;
    uint16_t length = 0;
    uint8_t payload[250]{};
};

// Formatting lives in the loop. Wi-Fi callback only copies into radioQueue.
QueueHandle_t radioQueue = nullptr;
portMUX_TYPE callbackMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t callbackOverflow = 0;
uint32_t receiverReject = 0, packetGap = 0, serialDrop = 0, subscriptionErrors = 0;
uint32_t receiverSession = 0, subscriptionSeq = 0, lastSubscriptionMs = 0;
uint32_t currentBoot = 0, lastPacketSeq = 0;
uint32_t retiredBoots[8]{};
size_t retiredBootPos = 0;
uint64_t lastRadioUs = 0;
uint32_t activeLog = 0, lastWheelSeq = 0, wheelSession = 0, wheelGap = 0, helloSeq = 0, lastHelloMs = 0;
uint32_t passiveWheelLog = 0;
uint64_t lastSampleUs = 0, lastSampleRxUs = 0, lastWheelRxUs = 0;
bool motorOffInSample = false;
bool ready = false, headersPending = true;
char output[OutputSlots][MaxLineBytes]{};
uint16_t outputLength[OutputSlots]{};
size_t outputRead = 0, outputWrite = 0, outputCount = 0, outputOffset = 0;
char input[32]{};
size_t inputLength = 0;
bool inputOverflow = false;

const char VrHeader[] =
    "VR_HEADER,bridge_rx_us,boot_id,bridge_session,packet_seq,log_id,sample_index,sample_hz,imu,queue_drop,tx_drop,"
    "t_us,command_age_us,sample_seq,cfg,forward_pct,turn_pct,flags,mode,diag_flags,source,speed_pct,"
    "ax,ay,az,gx,gy,gz,temp_c,theta,rate,target,gyro_d,vref,yawref,cmd_l,cmd_r,a_cmd,v,v_request,yaw_request,"
    "rssi_dbm,receiver_queue_drop,receiver_reject,receiver_packet_gap,serial_drop,serial_crc16\n";
const char ViHeader[] =
    "VI_HEADER,bridge_rx_us,boot_id,bridge_session,packet_seq,log_id,t_us,produced,queue_drop,tx_drop,tx_fail,"
    "spi_hz,imu_bus_errors,imu_missed,imu_late,sample_hz,enabled,imu,sck,mosi,miso,cs,interrupt,"
    "gyro_lpf,accel_lpf,filter_flags,accel_range_g,gyro_range_dps,eq,kp,ki,kd,kpv,kiv,fusion_tau,d_lpf_hz,"
    "cfg,gyro_bias_x_dps,gyro_bias_y_dps,gyro_bias_z_dps,trust_norm,trust_start,trust_end,trust_timeout,"
    "mount00,mount01,mount02,mount10,mount11,mount12,mount20,mount21,mount22,firmware,rssi_dbm,"
    "receiver_queue_drop,receiver_reject,receiver_packet_gap,serial_drop,serial_crc16\n";
const char VwHeader[] =
    "VW_HEADER,bridge_rx_us,boot_id,bridge_session,diag_session,packet_seq,log_id,t_us,motor_us,sample_seq,cfg,"
    "motor_session,en,angle_valid,cmd_l,cmd_r,steps_l,steps_r,enc_l,enc_r,uart_rx_l,uart_rx_r,"
    "uart_age_l_us,uart_age_r_us,diag_packet_gap,rssi_dbm,serial_drop,serial_crc16\n";

uint64_t timeUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
uint32_t queueDrops() {
    portENTER_CRITICAL(&callbackMux);
    const uint32_t count = callbackOverflow;
    portEXIT_CRITICAL(&callbackMux);
    return count;
}

bool queueText(const char* text) {
    const size_t size = std::strlen(text);
    if (!size) return true;
    if (size >= MaxLineBytes || outputCount == OutputSlots) {
        ++serialDrop;
        return false;
    }
    std::memcpy(output[outputWrite], text, size);
    outputLength[outputWrite] = static_cast<uint16_t>(size);
    outputWrite = (outputWrite + 1) % OutputSlots;
    ++outputCount;
    return true;
}

class CsvLine {
public:
    void add(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        if (!valid) return;
        va_list args;
        va_start(args, fmt);
        const int written = std::vsnprintf(text + used, sizeof(text) - used, fmt, args);
        va_end(args);
        if (written < 0 || static_cast<size_t>(written) >= sizeof(text) - used) {
            valid = false;
            return;
        }
        used += static_cast<size_t>(written);
    }
    void send() {
        if (!valid) { ++serialDrop; return; }
        const uint16_t crc = ChanDoanEspNow::tinhCrc16(reinterpret_cast<const uint8_t*>(text), used);
        add(",%u\n", static_cast<unsigned>(crc));
        if (valid) queueText(text);
        else ++serialDrop;
    }
private:
    char text[MaxLineBytes]{};
    size_t used = 0;
    bool valid = true;
};

void flushUsb() {
    uint8_t chunk[UsbWriteChunk];
    size_t budget = UsbWriteBudget;
    for (unsigned write = 0; outputCount && budget && write < 8; ++write) {
        const int available = Serial.availableForWrite();
        if (available <= 0) return;
        const size_t limit = std::min({budget, sizeof(chunk), static_cast<size_t>(available)});
        size_t size = 0;
        // Free CDC space can span several CSV rows. Preserve byte order and CRCs.
        for (size_t i = 0; i < outputCount && size < limit; ++i) {
            const size_t slot = (outputRead + i) % OutputSlots;
            const size_t offset = i == 0 ? outputOffset : 0;
            const size_t bytes = std::min(limit - size, outputLength[slot] - offset);
            std::memcpy(chunk + size, output[slot] + offset, bytes);
            size += bytes;
        }
        const size_t written = Serial.write(chunk, size);
        size_t remaining = written;
        while (remaining) {
            const size_t bytes = std::min(remaining, outputLength[outputRead] - outputOffset);
            outputOffset += bytes;
            remaining -= bytes;
            if (outputOffset == outputLength[outputRead]) {
                outputOffset = 0;
                outputRead = (outputRead + 1) % OutputSlots;
                --outputCount;
            }
        }
        budget -= written;
        if (written < size) return;
    }
}

void onReceive(const esp_now_recv_info_t* info, const uint8_t* bytes, int length) {
    if (!info || !bytes || !radioQueue ||
        (length != static_cast<int>(sizeof(KhaoSatRung::Samples)) &&
         length != static_cast<int>(sizeof(KhaoSatRung::Info)) &&
         length != static_cast<int>(sizeof(ChanDoanEspNow::GoiTelemetry)))) return;
    RadioFrame item;
    std::memcpy(item.mac, info->src_addr, sizeof(item.mac));
    item.receivedUs = timeUs();
    item.rssi = info->rx_ctrl ? info->rx_ctrl->rssi : -128;
    item.length = static_cast<uint16_t>(length);
    std::memcpy(item.payload, bytes, static_cast<size_t>(length));
    if (xQueueSend(radioQueue, &item, 0) != pdTRUE) {
        portENTER_CRITICAL(&callbackMux);
        ++callbackOverflow;
        portEXIT_CRITICAL(&callbackMux);
    }
}

void subscribe() {
    KhaoSatRung::Subscribe packet;
    if (!++subscriptionSeq) ++subscriptionSeq;
    KhaoSatRung::pack(packet, receiverSession, subscriptionSeq);
    if (esp_now_send(RobotMac, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet)) != ESP_OK)
        ++subscriptionErrors;
    lastSubscriptionMs = millis();
}

bool acceptOrder(uint32_t boot, const KhaoSatRung::Header& h) {
    if (h.session != receiverSession) return false;
    if (boot != currentBoot) {
        for (uint32_t old : retiredBoots) if (old && old == boot) return false;
        if (currentBoot) {
            retiredBoots[retiredBootPos] = currentBoot;
            retiredBootPos = (retiredBootPos + 1) % 8;
        }
        currentBoot = boot;
        lastPacketSeq = 0;
        activeLog = passiveWheelLog = wheelSession = lastWheelSeq = 0;
        lastWheelRxUs = lastSampleUs = lastSampleRxUs = 0;
        motorOffInSample = false;
    }
    if (lastPacketSeq && !ChanDoanEspNow::thuTuMoiHon(h.seq, lastPacketSeq)) return false;
    if (lastPacketSeq) packetGap += h.seq - lastPacketSeq - 1u;
    lastPacketSeq = h.seq;
    return true;
}

void addQuality(CsvLine& line, const RadioFrame& frame) {
    line.add(",%d,%lu,%lu,%lu,%lu", static_cast<int>(frame.rssi),
             static_cast<unsigned long>(queueDrops()), static_cast<unsigned long>(receiverReject),
             static_cast<unsigned long>(packetGap), static_cast<unsigned long>(serialDrop));
}

void printSamples(const KhaoSatRung::Samples& p, const RadioFrame& frame) {
    const auto& latest = p.sample[p.count - 1];
    activeLog = p.log_id;
    lastSampleUs = latest.t_us;
    lastSampleRxUs = frame.receivedUs;
    motorOffInSample = !(latest.flags & KhaoSatRung::MotorOn);
    for (unsigned i = 0; i < p.count; ++i) {
        const auto& s = p.sample[i];
        CsvLine line;
        line.add("VR,%llu,%lu,%lu,%lu,%lu,%u,%u,%u,%lu,%lu,%llu,%lu,%lu,%lu,%d,%d,%u,%u,%u,%u,%u",
            static_cast<unsigned long long>(frame.receivedUs), static_cast<unsigned long>(p.boot_id),
            static_cast<unsigned long>(p.h.session), static_cast<unsigned long>(p.h.seq),
            static_cast<unsigned long>(p.log_id), i, p.sample_hz, p.imu,
            static_cast<unsigned long>(p.queue_drop), static_cast<unsigned long>(p.tx_drop),
            static_cast<unsigned long long>(s.t_us), static_cast<unsigned long>(s.command_age_us),
            static_cast<unsigned long>(s.sample_seq), static_cast<unsigned long>(s.cfg),
            s.forward_pct, s.turn_pct, s.flags, s.mode, s.diag_flags, s.source, s.speed_pct);
        line.add(",%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g",
            s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.temp_c, s.theta, s.rate, s.target,
            s.gyro_d, s.vref, s.yawref, s.cmd_l, s.cmd_r, s.a_cmd, s.v, s.v_request, s.yaw_request);
        addQuality(line, frame);
        line.send();
    }
}

// Legacy robots treat HELLO as a remote-control session and suppress WebUI drive.
// Wait for a supported firmware's Info for this exact log before requesting wheels.
void requestWheelTelemetry() {
    const auto now = timeUs();
    if (!activeLog || activeLog != passiveWheelLog || !motorOffInSample || !lastSampleRxUs ||
        now - lastSampleRxUs > 250000 ||
        (lastWheelRxUs && now - lastWheelRxUs < 2000000) || millis() - lastHelloMs < 2000) return;
    ChanDoanEspNow::GoiHello hello{};
    if (!++helloSeq) ++helloSeq;
    ChanDoanEspNow::dongGoi(hello, receiverSession, helloSeq);
    if (esp_now_send(RobotMac, reinterpret_cast<const uint8_t*>(&hello), sizeof(hello)) != ESP_OK)
        ++subscriptionErrors;
    lastHelloMs = millis();
}

void receiveWheels(const RadioFrame& frame) {
    ChanDoanEspNow::GoiTelemetry p{};
    if (!ChanDoanEspNow::giaiMa(frame.payload, frame.length, p) || !ChanDoanEspNow::telemetryHopLe(p)) {
        ++receiverReject;
        return;
    }
    if (p.boot_id != currentBoot) return;
    // A still-active diagnostic session for this peer can already be streaming;
    // listen to it directly without replacing its session when the robot moves.
    if (wheelSession != p.h.session) {
        wheelSession = p.h.session;
        lastWheelSeq = wheelGap = 0;
    }
    if (lastWheelSeq && !ChanDoanEspNow::thuTuMoiHon(p.h.seq, lastWheelSeq)) return;
    if (lastWheelSeq) wheelGap += p.h.seq - lastWheelSeq - 1u;
    lastWheelSeq = p.h.seq;
    lastWheelRxUs = frame.receivedUs;
    const uint64_t delta = p.t_us > lastSampleUs ? p.t_us - lastSampleUs : lastSampleUs - p.t_us;
    if (!activeLog || !lastSampleUs || delta > 100000u) return;
    CsvLine line;
    line.add("VW,%llu,%lu,%lu,%lu,%lu,%lu,%llu,%llu,%lu,%lu,%lu,%u,%u,%.7g,%.7g",
             static_cast<unsigned long long>(frame.receivedUs), static_cast<unsigned long>(p.boot_id),
             static_cast<unsigned long>(receiverSession), static_cast<unsigned long>(wheelSession),
             static_cast<unsigned long>(p.h.seq), static_cast<unsigned long>(activeLog),
             static_cast<unsigned long long>(p.t_us), static_cast<unsigned long long>(p.motor_us),
             static_cast<unsigned long>(p.sample_seq), static_cast<unsigned long>(p.cfg),
             static_cast<unsigned long>(p.motor_session), p.en, p.angle_valid, p.cmd_l, p.cmd_r);
    line.add(",%lld,%lld,%lld,%lld,%lu,%lu,%lu,%lu,%lu,%d,%lu",
             static_cast<long long>(p.steps_l), static_cast<long long>(p.steps_r),
             static_cast<long long>(p.enc_l), static_cast<long long>(p.enc_r),
             static_cast<unsigned long>(p.uart_rx_l), static_cast<unsigned long>(p.uart_rx_r),
             static_cast<unsigned long>(p.uart_age_l_us), static_cast<unsigned long>(p.uart_age_r_us),
             static_cast<unsigned long>(wheelGap), static_cast<int>(frame.rssi), static_cast<unsigned long>(serialDrop));
    line.send();
}

bool validInfo(const KhaoSatRung::Info& p) {
    if (!KhaoSatRung::valid(p)) return false;
    const float parameters[] = {p.eq, p.kp, p.ki, p.kd, p.kpv, p.kiv, p.fusion_tau, p.d_lpf_hz,
        p.trust_norm, p.trust_start, p.trust_end, p.trust_timeout};
    for (float v : parameters) if (!std::isfinite(v)) return false;
    for (float v : p.gyro_bias) if (!std::isfinite(v)) return false;
    for (float v : p.mount) if (!std::isfinite(v)) return false;
    for (const char* c = p.firmware; *c; ++c) {
        if (!((*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') ||
              (*c >= '0' && *c <= '9') || *c == '-' || *c == '_' || *c == '.')) return false;
    }
    return p.firmware[0] != '\0';
}

void printInfo(const KhaoSatRung::Info& p, const RadioFrame& frame) {
    CsvLine line;
    line.add("VI,%llu,%lu,%lu,%lu,%lu,%llu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u",
        static_cast<unsigned long long>(frame.receivedUs), static_cast<unsigned long>(p.boot_id),
        static_cast<unsigned long>(p.h.session), static_cast<unsigned long>(p.h.seq),
        static_cast<unsigned long>(p.log_id), static_cast<unsigned long long>(p.t_us),
        static_cast<unsigned long>(p.produced), static_cast<unsigned long>(p.queue_drop),
        static_cast<unsigned long>(p.tx_drop), static_cast<unsigned long>(p.tx_fail),
        static_cast<unsigned long>(p.spi_hz), static_cast<unsigned long>(p.imu_bus_errors),
        static_cast<unsigned long>(p.imu_missed), static_cast<unsigned long>(p.imu_late),
        p.sample_hz, p.enabled, p.imu, p.sck, p.mosi, p.miso, p.cs, p.interrupt,
        p.gyro_lpf, p.accel_lpf, p.filter_flags, p.accel_range_g, p.gyro_range_dps);
    line.add(",%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g", p.eq, p.kp, p.ki, p.kd, p.kpv, p.kiv,
             p.fusion_tau, p.d_lpf_hz);
    line.add(",%lu", static_cast<unsigned long>(p.cfg));
    for (float v : p.gyro_bias) line.add(",%.7g", v);
    line.add(",%.7g,%.7g,%.7g,%.7g", p.trust_norm, p.trust_start, p.trust_end, p.trust_timeout);
    for (float v : p.mount) line.add(",%.7g", v);
    line.add(",%s", p.firmware);
    addQuality(line, frame);
    line.send();
}

void receiveFrames() {
    RadioFrame frame;
    for (unsigned count = 0; count < 8 && xQueueReceive(radioQueue, &frame, 0) == pdTRUE; ++count) {
        if (std::memcmp(frame.mac, RobotMac, sizeof(RobotMac))) { ++receiverReject; continue; }
        if (frame.length == sizeof(ChanDoanEspNow::GoiTelemetry)) {
            receiveWheels(frame);
        } else if (frame.length == sizeof(KhaoSatRung::Samples)) {
            KhaoSatRung::Samples p;
            if (!KhaoSatRung::decode(frame.payload, frame.length, p) || !KhaoSatRung::valid(p) ||
                !acceptOrder(p.boot_id, p.h)) { ++receiverReject; continue; }
            lastRadioUs = frame.receivedUs;
            printSamples(p, frame);
        } else {
            KhaoSatRung::Info p;
            if (!KhaoSatRung::decode(frame.payload, frame.length, p) || !validInfo(p) ||
                !acceptOrder(p.boot_id, p.h)) { ++receiverReject; continue; }
            lastRadioUs = frame.receivedUs;
            passiveWheelLog = p.enabled &&
                std::strncmp(p.firmware, PassiveWheelFirmwarePrefix, sizeof(PassiveWheelFirmwarePrefix) - 1u) == 0
                ? p.log_id : 0;
            if (!p.enabled || p.log_id != activeLog) {
                activeLog = 0;
                lastSampleUs = 0;
                motorOffInSample = false;
            }
            printInfo(p, frame);
        }
        flushUsb();
    }
}

void printStatus() {
    char line[256];
    std::snprintf(line, sizeof(line),
        "RECEIVER_STATUS,ready=%u,mac=%s,session=%lu,boot=%lu,last_rx_us=%llu,subscribe_errors=%lu,rx_drop=%lu,serial_drop=%lu\n",
        static_cast<unsigned>(ready), WiFi.macAddress().c_str(), static_cast<unsigned long>(receiverSession),
        static_cast<unsigned long>(currentBoot), static_cast<unsigned long long>(lastRadioUs),
        static_cast<unsigned long>(subscriptionErrors), static_cast<unsigned long>(queueDrops()),
        static_cast<unsigned long>(serialDrop));
    queueText(line);
}

void readCommands() {
    for (unsigned n = 0; n < 64 && Serial.available(); ++n) {
        const int c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            input[inputLength] = '\0';
            if (!inputOverflow) {
                if (!std::strcmp(input, "headers")) headersPending = true;
                else if (!std::strcmp(input, "status")) printStatus();
            }
            inputLength = 0;
            inputOverflow = false;
        } else if (c >= 0) {
            if (!inputOverflow && inputLength + 1 < sizeof(input)) input[inputLength++] = static_cast<char>(c);
            else inputOverflow = true;
        }
    }
}

void begin() {
    Serial.begin(SerialBaud);
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    do { receiverSession = esp_random(); } while (!receiverSession);
    radioQueue = xQueueCreate(RadioQueueSlots, sizeof(RadioFrame));
    ready = radioQueue && esp_wifi_set_channel(RadioChannel, WIFI_SECOND_CHAN_NONE) == ESP_OK &&
        esp_now_init() == ESP_OK;
    if (ready) {
        esp_now_peer_info_t peer{};
        std::memcpy(peer.peer_addr, RobotMac, sizeof(RobotMac));
        peer.channel = RadioChannel;
        peer.ifidx = WIFI_IF_STA;
        ready = esp_now_add_peer(&peer) == ESP_OK && esp_now_register_recv_cb(onReceive) == ESP_OK;
    }
    printStatus();
    if (ready) subscribe();
}

void service() {
    readCommands();
    if (ready && millis() - lastSubscriptionMs >= 1000) subscribe();
    if (headersPending && outputCount <= OutputSlots - 3) {
        queueText(VrHeader);
        queueText(ViHeader);
        queueText(VwHeader);
        headersPending = false;
    }
    if (ready && !headersPending) receiveFrames();
    if (ready) requestWheelTelemetry();
    flushUsb();
    delay(1);
}
} // namespace PassiveReceiver
