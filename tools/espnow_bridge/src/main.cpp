#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "model/GiaoThucChanDoanEspNow.h"

using namespace ChanDoanEspNow;

namespace {
constexpr uint32_t hostTimeoutMs = 250;
constexpr uint32_t telemetryTimeoutMs = 200;
constexpr uint32_t helloIntervalMs = 500;
constexpr uint32_t stopIntervalMs = 50;
constexpr uint8_t radioChannel = 1;

struct Received {
    uint8_t mac[6];
    uint64_t bridge_rx_us;
    int8_t rssi_dbm;
    GoiTelemetry packet;
};
QueueHandle_t rxQueue = nullptr;
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t rxOverflow = 0;
uint32_t rejected = 0, missedRadio = 0, txErrors = 0, usbDropped = 0;
uint8_t robotMac[6]{};
bool radioReady = false, connected = false, monitoring = false, haveTelemetry = false;
bool latched = true, armRequested = false, armConfirmed = false, stopping = false, disabling = false;
uint32_t bridgeSession = 0, txSequence = 0, robotBoot = 0, lastTelemetrySequence = 0;
uint32_t lastHostMs = 0, lastTelemetryMs = 0, lastSourceMs = 0, lastHelloMs = 0, lastStopMs = 0;
uint32_t pendingArmSeq = 0, firstStopSeq = 0, moveDeadlineMs = 0, helloAttempts = 0;
uint32_t armRequestMs = 0;
GoiTelemetry telemetry{};
uint64_t telemetryRxUs = 0;
int8_t telemetryRssi = -128;
bool telemetryPending = false;
char input[96]{};
size_t inputLength = 0;
bool inputOverflow = false;

// Một dòng được gửi từng đoạn tùy sức chứa USB. Tối đa bốn dòng, mất dòng được đếm riêng.
constexpr size_t lineCapacity = 1152;
char output[4][lineCapacity]{};
size_t outputLength[4]{}, outputHead = 0, outputTail = 0, outputCount = 0, outputOffset = 0;

uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
bool due(uint32_t now, uint32_t deadline) { return static_cast<int32_t>(now-deadline) >= 0; }

void queueText(const char* text) {
    const size_t length = std::strlen(text);
    if (length >= lineCapacity || outputCount == 4) { ++usbDropped; return; }
    std::memcpy(output[outputTail], text, length);
    outputLength[outputTail] = length;
    outputTail = (outputTail+1) % 4;
    ++outputCount;
}

void event(const char* message) {
    char line[160];
    std::snprintf(line, sizeof(line), "BRIDGE_EVENT,%lu,%s\n", static_cast<unsigned long>(millis()), message);
    queueText(line);
}

void flushUsb() {
    if (!outputCount) return;
    const int freeBytes = Serial.availableForWrite();
    if (freeBytes <= 0) return;
    const size_t remain = outputLength[outputHead]-outputOffset;
    const size_t chunk = std::min(remain, std::min(static_cast<size_t>(freeBytes), size_t{256}));
    outputOffset += Serial.write(reinterpret_cast<const uint8_t*>(output[outputHead])+outputOffset, chunk);
    if (outputOffset == outputLength[outputHead]) {
        outputOffset = 0;
        outputHead = (outputHead+1) % 4;
        --outputCount;
    }
}

void onReceive(const esp_now_recv_info_t* info, const uint8_t* bytes, int size) {
    if (!info || !bytes || size != sizeof(GoiTelemetry)) return;
    Received item{};
    std::memcpy(item.mac, info->src_addr, sizeof(item.mac));
    item.bridge_rx_us = nowUs();
    item.rssi_dbm = info->rx_ctrl ? info->rx_ctrl->rssi : -128;
    std::memcpy(&item.packet, bytes, sizeof(item.packet));
    if (xQueueSend(rxQueue, &item, 0) != pdTRUE) {
        portENTER_CRITICAL(&rxMux);
        ++rxOverflow;
        portEXIT_CRITICAL(&rxMux);
    }
}

uint32_t nextSequence() {
    ++txSequence;
    if (!txSequence) ++txSequence;
    return txSequence;
}

uint32_t sendCommand(MaLenh opcode, float speed = 0, float yaw = 0, uint16_t lease = 0) {
    if (!connected || !robotBoot) return 0;
    GoiLenh command{};
    command.boot_id = robotBoot;
    command.opcode = opcode;
    command.lease_ms = lease;
    command.v_mps = speed;
    command.yaw_radps = yaw;
    const uint32_t seq = nextSequence();
    dongGoi(command, bridgeSession, seq);
    if (esp_now_send(robotMac, reinterpret_cast<const uint8_t*>(&command), sizeof(command)) != ESP_OK) {
        ++txErrors;
        return 0;
    }
    return seq; // Enqueue thành công. ACK ứng dụng và EN nằm trong telemetry.
}

void stopNow(const char* reason) {
    latched = true;
    armRequested = armConfirmed = false;
    pendingArmSeq = 0;
    moveDeadlineMs = 0;
    if (!stopping) firstStopSeq = 0;
    stopping = connected && robotBoot;
    if (stopping) {
        const uint32_t seq = sendCommand(disabling ? MaLenh::Disable : MaLenh::Stop);
        if (!firstStopSeq && seq) firstStopSeq = seq;
        lastStopMs = millis();
    }
    event(reason);
}

void sendHello() {
    GoiHello hello{};
    hello.period_ms = 20;
    dongGoi(hello, bridgeSession, nextSequence());
    if (esp_now_send(robotMac, reinterpret_cast<const uint8_t*>(&hello), sizeof(hello)) != ESP_OK) ++txErrors;
    lastHelloMs = millis();
    ++helloAttempts;
}

bool telemetryFresh() {
    const uint32_t now = millis();
    return haveTelemetry && now-lastTelemetryMs <= telemetryTimeoutMs && now-lastSourceMs <= telemetryTimeoutMs;
}

void receiveTelemetry() {
    Received item{};
    for (unsigned count = 0; count < 8 && xQueueReceive(rxQueue, &item, 0) == pdTRUE; ++count) {
        GoiTelemetry packet{};
        if (!connected || std::memcmp(item.mac, robotMac, 6) ||
            !giaiMa(reinterpret_cast<const uint8_t*>(&item.packet), sizeof(item.packet), packet) ||
            !telemetryHopLe(packet) ||
            packet.h.session != bridgeSession || !packet.boot_id ||
            (lastTelemetrySequence && static_cast<int32_t>(packet.h.seq-lastTelemetrySequence) <= 0)) {
            ++rejected;
            continue;
        }
        if (robotBoot && packet.boot_id != robotBoot) {
            robotBoot = packet.boot_id;
            stopNow("robot_reboot_reconnect_required");
            haveTelemetry = false;
            continue;
        }
        if (lastTelemetrySequence) missedRadio += packet.h.seq-lastTelemetrySequence-1;
        lastTelemetrySequence = packet.h.seq;
        const uint32_t receivedMs = static_cast<uint32_t>(item.bridge_rx_us/1000);
        if (!haveTelemetry || (packet.t_us > telemetry.t_us && packet.control_seq != telemetry.control_seq))
            lastSourceMs = receivedMs;
        lastTelemetryMs = receivedMs;
        robotBoot = packet.boot_id;
        telemetry = packet;
        telemetryRxUs = item.bridge_rx_us;
        telemetryRssi = item.rssi_dbm;
        haveTelemetry = true;
        telemetryPending = true;
        if (pendingArmSeq && (packet.ack_seq == pendingArmSeq || thuTuMoiHon(packet.ack_seq, pendingArmSeq))
            && (packet.remote_state & ControlLease) && packet.en && packet.control_phase == 2 && packet.motor_phase == 2) {
            armConfirmed = true;
            armRequested = false;
            pendingArmSeq = 0;
            event("arm_applied_EN1");
        }
        if (stopping && firstStopSeq && (packet.ack_seq == firstStopSeq || thuTuMoiHon(packet.ack_seq, firstStopSeq)) &&
            (disabling ? (!packet.en && packet.motor_phase == 0) :
             (!(packet.remote_state & ControlLease) && std::fabs(packet.vref) < 0.0001f && std::fabs(packet.yawref) < 0.0001f))) {
            stopping = false;
            event(disabling ? "disarm_applied_EN0" : "stop_applied_balance_hold");
            disabling = false;
        }
    }
}

void printHeader() {
    queueText("BN_HEADER,bridge_rx_us,boot_id,bridge_session,radio_seq,t_us,motor_us,sample_seq,control_seq,cfg,"
        "control_session,motor_session,received_seq,ack_seq,ack_opcode,ack_result,remote_state,en,mode,control_fault,"
        "motor_fault,control_phase,motor_phase,angle_valid,feedback,theta,rate,target,eq,v,vref,yawref,cmd_l,cmd_r,"
        "a_cmd,gyro_d,acc_norm,innov,trust,kp,ki,kd,steps_l,steps_r,enc_l,enc_r,uart_age_l_us,uart_age_r_us,"
        "uart_rx_l,uart_rx_r,imu_miss,imu_i2c,imu_late,robot_tx_drop,robot_tx_fail,robot_rx_reject,"
        "robot_rx_duplicate,bridge_rx_overflow,bridge_reject,bridge_missed_radio,bridge_tx_errors,usb_dropped,rssi_dbm,diag_flags,serial_crc16\n");
}

void printTelemetry() {
    if (!monitoring || !telemetryPending) return;
    telemetryPending = false;
    uint32_t overflow;
    portENTER_CRITICAL(&rxMux);
    overflow = rxOverflow;
    portEXIT_CRITICAL(&rxMux);
    const auto& t = telemetry;
    char line[lineCapacity];
    const int size = std::snprintf(line, sizeof(line),
        "BN,%llu,%lu,%lu,%lu,%llu,%llu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
        "%.7g,%.7g,%.7g,%.7g,%.6g,%.6g,%.6g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.6g,%.6g,%.6g,%.6g,"
        "%lld,%lld,%lld,%lld,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%d,%u\n",
        static_cast<unsigned long long>(telemetryRxUs), static_cast<unsigned long>(t.boot_id),
        static_cast<unsigned long>(t.h.session), static_cast<unsigned long>(t.h.seq),
        static_cast<unsigned long long>(t.t_us), static_cast<unsigned long long>(t.motor_us),
        static_cast<unsigned long>(t.sample_seq), static_cast<unsigned long>(t.control_seq),
        static_cast<unsigned long>(t.cfg), static_cast<unsigned long>(t.control_session),
        static_cast<unsigned long>(t.motor_session), static_cast<unsigned long>(t.received_seq),
        static_cast<unsigned long>(t.ack_seq), t.ack_opcode, t.ack_result, t.remote_state, t.en, t.mode,
        t.control_fault, t.motor_fault, t.control_phase, t.motor_phase, t.angle_valid, t.feedback,
        t.theta, t.rate, t.target, t.eq, t.v, t.vref, t.yawref, t.cmd_l, t.cmd_r,
        t.a_cmd, t.gyro_d, t.acc_norm, t.innov, t.trust, t.kp, t.ki, t.kd,
        static_cast<long long>(t.steps_l), static_cast<long long>(t.steps_r),
        static_cast<long long>(t.enc_l), static_cast<long long>(t.enc_r),
        static_cast<unsigned long>(t.uart_age_l_us), static_cast<unsigned long>(t.uart_age_r_us),
        static_cast<unsigned long>(t.uart_rx_l), static_cast<unsigned long>(t.uart_rx_r),
        static_cast<unsigned long>(t.imu_miss), static_cast<unsigned long>(t.imu_i2c),
        static_cast<unsigned long>(t.imu_late), static_cast<unsigned long>(t.tx_drop),
        static_cast<unsigned long>(t.tx_fail), static_cast<unsigned long>(t.rx_reject),
        static_cast<unsigned long>(t.rx_duplicate), static_cast<unsigned long>(overflow),
        static_cast<unsigned long>(rejected), static_cast<unsigned long>(missedRadio),
        static_cast<unsigned long>(txErrors), static_cast<unsigned long>(usbDropped), static_cast<int>(telemetryRssi), static_cast<unsigned>(t.diag_flags));
    if (size > 0 && static_cast<size_t>(size)+8 < sizeof(line)) {
        const size_t payloadLength = static_cast<size_t>(size)-1; // bỏ newline để tính CRC.
        const uint16_t crc = tinhCrc16(reinterpret_cast<const uint8_t*>(line), payloadLength);
        std::snprintf(line+payloadLength,sizeof(line)-payloadLength,",%u\n",static_cast<unsigned>(crc));
        queueText(line);
    } else ++usbDropped;
}

void safetyCheck() {
    const uint32_t now = millis();
    if (armRequested || armConfirmed) {
        if (now-lastHostMs > hostTimeoutMs) stopNow("host_timeout_latched");
        else if (!telemetryFresh()) stopNow("telemetry_timeout_latched");
        else if (telemetry.control_fault || telemetry.motor_fault || !telemetry.angle_valid)
            stopNow("robot_fault_latched");
        else if (armConfirmed && !telemetry.en) stopNow("robot_disabled_latched");
        else if (armRequested && now-armRequestMs > 1000) stopNow("arm_timeout_latched");
        else if (moveDeadlineMs && due(now, moveDeadlineMs)) stopNow("move_lease_expired_latched");
    }
    if (stopping && now-lastStopMs >= stopIntervalMs) {
        const uint32_t seq = sendCommand(disabling ? MaLenh::Disable : MaLenh::Stop);
        if (!firstStopSeq && seq) firstStopSeq = seq;
        lastStopMs = now;
    }
    if (connected && !haveTelemetry && !robotBoot && helloAttempts < 10 && now-lastHelloMs >= helloIntervalMs)
        sendHello();
}

bool parseMac(const char* text, uint8_t* out) {
    unsigned parts[6];
    int end = 0;
    if (std::sscanf(text, "%2x:%2x:%2x:%2x:%2x:%2x%n", &parts[0], &parts[1], &parts[2],
                    &parts[3], &parts[4], &parts[5], &end) != 6 || text[end]) return false;
    for (unsigned i = 0; i < 6; ++i) out[i] = static_cast<uint8_t>(parts[i]);
    return !(out[0]&1) && (parts[0]|parts[1]|parts[2]|parts[3]|parts[4]|parts[5]);
}

void command(const char* text) {
    const uint32_t now = millis();
    if (!std::strcmp(text, "help")) {
        queueText("BRIDGE_HELP,connect MAC | monitor | hb | arm | move v_mps yaw_radps lease_ms | stop(hold) | disarm | status\n");
        return;
    }
    if (!std::strcmp(text, "status")) {
        char line[220];
        std::snprintf(line, sizeof(line), "BRIDGE_STATUS,mac=%s,connected=%d,session=%lu,boot=%lu,EN=%d,"
            "arm_confirmed=%d,latched=%d,telemetry_fresh=%d,tx_errors=%lu,usb_crc=1\n",
            WiFi.macAddress().c_str(), connected, static_cast<unsigned long>(bridgeSession),
            static_cast<unsigned long>(robotBoot), haveTelemetry ? telemetry.en : -1,
            armConfirmed, latched, telemetryFresh(), static_cast<unsigned long>(txErrors));
        queueText(line);
        return;
    }
    if (!std::strncmp(text, "connect ", 8)) {
        uint8_t nextMac[6];
        if (!radioReady || !parseMac(text+8, nextMac)) { event("invalid_mac_or_radio_not_ready"); return; }
        if (armRequested || armConfirmed || (stopping && telemetryFresh())) { event("stop_before_reconnect"); return; }
        if (connected) esp_now_del_peer(robotMac);
        esp_now_peer_info_t peer{};
        std::memcpy(peer.peer_addr, nextMac, 6);
        peer.channel = radioChannel;
        peer.ifidx = WIFI_IF_STA;
        if (esp_now_add_peer(&peer) != ESP_OK) { connected = false; event("add_peer_failed"); return; }
        std::memcpy(robotMac, nextMac, 6);
        do { bridgeSession = esp_random(); } while (!bridgeSession);
        txSequence = robotBoot = lastTelemetrySequence = helloAttempts = 0;
        firstStopSeq = pendingArmSeq = moveDeadlineMs = 0;
        haveTelemetry = armRequested = armConfirmed = stopping = disabling = false;
        telemetryPending = false;
        telemetry = GoiTelemetry{};
        telemetryRxUs = 0;
        telemetryRssi = -128;
        latched = false;
        connected = monitoring = true;
        lastHostMs = now;
        printHeader();
        sendHello();
        event("hello_sent_wait_boot_and_binding");
        return;
    }
    if (!std::strcmp(text, "monitor")) {
        monitoring = true;
        lastHostMs = now;
        printHeader();
        return;
    }
    if (!std::strcmp(text, "stop")) { disabling = false; stopNow("operator_stop_latched"); return; }
    if (!std::strcmp(text, "disarm")) { disabling = true; stopNow("operator_disarm_requested"); return; }
    if (!std::strcmp(text, "hb")) {
        lastHostMs = now;
        if (!latched && (armConfirmed || armRequested) && !sendCommand(MaLenh::Heartbeat, 0, 0, 150))
            stopNow("heartbeat_enqueue_failed");
        return;
    }
    if (!std::strcmp(text, "arm")) {
        if (!telemetryFresh() || !robotBoot || !(telemetry.remote_state&Bound) ||
            telemetry.mode != 0 || telemetry.control_fault || telemetry.motor_fault || !telemetry.angle_valid ||
            pendingArmSeq || stopping || (telemetry.motor_phase != 0 && telemetry.motor_phase != 2)) {
            event("arm_rejected_reconnect_or_wait_ready");
            return;
        }
        lastHostMs = now;
        latched = false; disabling = false;
        pendingArmSeq = sendCommand(MaLenh::Arm);
        armRequestMs = now;
        armRequested = pendingArmSeq != 0;
        if (!armRequested) stopNow("arm_enqueue_failed");
        else event("arm_requested_wait_applied_ack_and_EN");
        return;
    }
    if (!std::strncmp(text, "move ", 5)) {
        float speed, yaw;
        unsigned lease;
        char extra;
        if (std::sscanf(text+5, "%f %f %u %c", &speed, &yaw, &lease, &extra) != 3 ||
            !std::isfinite(speed) || !std::isfinite(yaw) || std::fabs(speed) > 0.1f ||
            std::fabs(yaw) > 0.3f || !lease || lease > 150) {
            stopNow("invalid_move_latched"); return;
        }
        if (latched || !armConfirmed || !telemetryFresh()) { event("move_rejected_arm_not_confirmed"); return; }
        lastHostMs = now;
        if (!sendCommand(MaLenh::Move, speed, yaw, static_cast<uint16_t>(lease))) {
            stopNow("move_enqueue_failed"); return;
        }
        moveDeadlineMs = now + lease;
        return;
    }
    event("unknown_command");
}
}

void setup() {
    Serial.begin(921600);
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    rxQueue = xQueueCreate(8, sizeof(Received));
    radioReady = rxQueue && esp_wifi_set_channel(radioChannel, WIFI_SECOND_CHAN_NONE) == ESP_OK
        && esp_now_init() == ESP_OK && esp_now_register_recv_cb(onReceive) == ESP_OK;
    event(radioReady ? "ready_unarmed_channel1" : "radio_initialization_failed");
}

void loop() {
    receiveTelemetry();
    safetyCheck();
    for (unsigned count = 0; count < 64 && Serial.available(); ++count) {
        const int ch = Serial.read();
        if (ch == '\r') continue;
        if (ch == '\n') {
            input[inputLength] = '\0';
            if (inputOverflow) stopNow("serial_line_overflow_latched");
            else if (inputLength) {
                if (input[0]=='@') {
                    char* checksum=std::strrchr(input,'#');
                    unsigned received=0;int used=0;
                    if (!checksum || std::sscanf(checksum+1,"%4x%n",&received,&used)!=1 || used!=4 || checksum[1+used]) {
                        stopNow("usb_command_frame_invalid_hold");
                    } else {
                        *checksum='\0';
                        const uint16_t expected=tinhCrc16(reinterpret_cast<const uint8_t*>(input+1),std::strlen(input+1));
                        if (expected!=received) stopNow("usb_command_crc_invalid_hold");
                        else command(input+1);
                    }
                } else command(input);
            }
            inputLength = 0;
            inputOverflow = false;
        } else if (ch >= 0) {
            if (inputLength+1 < sizeof(input) && !inputOverflow) input[inputLength++] = static_cast<char>(ch);
            else inputOverflow = true;
        }
    }
    safetyCheck();
    printTelemetry();
    flushUsb();
    delay(1);
}
