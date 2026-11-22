#include "model/GiaoThucChanDoanEspNow.h"
#include <array>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace ChanDoanEspNow;
namespace {
unsigned checks = 0;
void check(bool ok, const char* label) {
    ++checks;
    if (!ok) {
        std::cerr << "FAIL: " << label << '\n';
        std::exit(1);
    }
}

template<class T> bool decode(const T& src, T& dst) {
    return giaiMa(reinterpret_cast<const uint8_t*>(&src), sizeof(src), dst);
}
template<class T> void reseal(T& packet) {
    packet.crc = tinhCrc16(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet) - 2);
}

void frameIntegrity() {
    const uint8_t vector[] = {'1','2','3','4','5','6','7','8','9'};
    check(tinhCrc16(vector, sizeof(vector)) == 0x29B1, "CCITT-FALSE known check value");
    GoiHello hello{}, out{};
    dongGoi(hello, 0x10203040, 1);
    check(decode(hello, out) && helloHopLe(out), "HELLO round trip at 50 Hz");
    const auto* bytes = reinterpret_cast<const uint8_t*>(&hello);
    check(bytes[0] == 'X' && bytes[1] == 'C' && bytes[2] == 'N' && bytes[3] == '1',
          "magic byte order matches ESP32 wire format");
    check(!giaiMa(nullptr, sizeof(hello), out), "null packet rejected");
    check(!giaiMa(bytes, sizeof(hello)-1, out), "truncated packet rejected");
    check(!giaiMa(bytes, sizeof(hello)+1, out), "oversize packet rejected before read");
    for (size_t byte = 0; byte < sizeof(hello); ++byte) {
        for (unsigned bit = 0; bit < 8; ++bit) {
            std::array<uint8_t, sizeof(GoiHello)> corrupt{};
            std::memcpy(corrupt.data(), &hello, sizeof(hello));
            corrupt[byte] ^= static_cast<uint8_t>(1u << bit);
            check(!giaiMa(corrupt.data(), corrupt.size(), out), "each single-bit corruption rejected");
        }
    }
    GoiHello bad = hello;
    bad.h.version = version+1; reseal(bad);
    check(!decode(bad, out), "unknown version rejected even with correct CRC");
    bad = hello; bad.h.length = 1; reseal(bad);
    check(!decode(bad, out), "embedded size must agree with payload");
    bad = hello; bad.h.type = LoaiGoi::Lenh; reseal(bad);
    check(!decode(bad, out), "type confusion rejected");
    bad = hello; bad.h.session = 0; reseal(bad);
    check(!decode(bad, out), "zero session rejected");
    bad = hello; bad.h.seq = 0; reseal(bad);
    check(!decode(bad, out), "zero sequence rejected");
    bad = hello; bad.reserved = 1;
    check(!helloHopLe(bad), "reserved HELLO flags rejected");
    bad = hello; bad.period_ms = 1;
    check(!helloHopLe(bad), "unsupported telemetry rate rejected");
    check(out.h.session == hello.h.session, "failed decoding preserves last valid output");
}

void commands() {
    GoiLenh cmd{}, out{};
    cmd.boot_id = 0xDEADBEEF;
    for (const auto op : {MaLenh::Stop, MaLenh::Arm, MaLenh::Disable}) {
        cmd.opcode = op;
        dongGoi(cmd, 7, 2);
        check(decode(cmd, out) && lenhHopLe(out, cmd.boot_id), "explicit stationary action accepted");
        cmd.lease_ms = 150;
        check(!lenhHopLe(cmd, cmd.boot_id), "stationary action cannot carry lease");
        cmd.lease_ms = 0;
    }
    cmd.opcode = MaLenh::Move;
    cmd.lease_ms = 150;
    cmd.v_mps = 0.10f;
    cmd.yaw_radps = -0.30f;
    dongGoi(cmd, 7, 3);
    check(decode(cmd, out) && lenhHopLe(out, cmd.boot_id), "MOVE exact positive/negative bounds");
    check(!lenhHopLe(cmd, cmd.boot_id+1) && !lenhHopLe(cmd, 0), "boot change invalidates commands");
    cmd.v_mps = 0.10001f;
    check(!lenhHopLe(cmd, cmd.boot_id), "overspeed rejected");
    cmd.v_mps = -0.10f;
    cmd.yaw_radps = 0.30001f;
    check(!lenhHopLe(cmd, cmd.boot_id), "excess yaw rejected");
    cmd.yaw_radps = 0;
    cmd.lease_ms = 151;
    check(!lenhHopLe(cmd, cmd.boot_id), "lease above 150 ms rejected");
    cmd.lease_ms = 0;
    check(!lenhHopLe(cmd, cmd.boot_id), "MOVE needs positive lease");
    cmd.lease_ms = 150;
    cmd.v_mps = std::numeric_limits<float>::quiet_NaN();
    check(!lenhHopLe(cmd, cmd.boot_id), "NaN rejected");
    cmd.v_mps = 0;
    cmd.yaw_radps = std::numeric_limits<float>::infinity();
    check(!lenhHopLe(cmd, cmd.boot_id), "infinity rejected");
    cmd.yaw_radps = 0;
    cmd.opcode = MaLenh::Heartbeat;
    check(lenhHopLe(cmd, cmd.boot_id), "HEARTBEAT accepts bounded lease with zero request");
    cmd.v_mps = 0.01f;
    check(!lenhHopLe(cmd, cmd.boot_id), "HEARTBEAT cannot smuggle movement");
    cmd.v_mps = 0;
    cmd.flags = 1;
    check(!lenhHopLe(cmd, cmd.boot_id), "unknown flags rejected");
    cmd.flags = 0;
    cmd.opcode = static_cast<MaLenh>(99);
    check(!lenhHopLe(cmd, cmd.boot_id), "unknown opcode rejected");
}

void sequenceAndLease() {
    BoLocThuTu order;
    check(!order.chapNhan(7, 1), "commands require established session");
    order.reset(7);
    check(order.chapNhan(7, 100), "first fresh command");
    check(!order.chapNhan(7, 100) && !order.chapNhan(7, 99), "duplicate and older command rejected");
    check(!order.chapNhan(8, 1000) && order.soThuTuCuoi() == 100, "wrong session cannot advance replay filter");
    check(order.chapNhan(7, 101), "new command after rejected replay");
    order.reset(8);
    check(!order.chapNhan(7, 102) && order.chapNhan(8, 1), "new session invalidates old command stream");
    order.reset(9);
    check(order.chapNhan(9, UINT32_MAX-1) && order.chapNhan(9, UINT32_MAX) &&
          !order.chapNhan(9, 0) && order.chapNhan(9, 1), "serial arithmetic wraps while skipping zero");
    check(!order.chapNhan(9, UINT32_MAX) && !order.chapNhan(9, 0x80000001u),
          "old and ambiguous half-range sequence rejected");
    const uint64_t received = 0x1'0000'0000ULL;
    check(leaseConHieuLuc(150, received, received+149999), "lease valid just before deadline");
    check(!leaseConHieuLuc(150, received, received+150000), "exact deadline expires");
    check(!leaseConHieuLuc(150, received, received-1), "future receive timestamp rejected");
    check(!leaseConHieuLuc(150, received, received+200000), "queue delay cannot renew stale motion");
    check(!leaseConHieuLuc(0, received, received) && !leaseConHieuLuc(151, received, received),
          "invalid lease rejected");
}

void telemetry() {
    GoiTelemetry packet{}, out{};
    packet.boot_id = 123;
    packet.t_us = 0x1'2345'6789ULL;
    packet.motor_us = packet.t_us + 4100;
    packet.steps_l = -0x1'0000'0001LL;
    packet.steps_r = 0x1'0000'0001LL;
    packet.enc_l = std::numeric_limits<int64_t>::min()+1;
    packet.enc_r = std::numeric_limits<int64_t>::max();
    packet.received_seq = 12;
    packet.ack_seq = 11;
    packet.remote_state = Bound | Expired;
    packet.theta = -0.0123f;
    packet.rate = 0.67f;
    packet.tx_fail = 9;
    dongGoi(packet, 42, 300);
    check(sizeof(packet) == 224 && sizeof(packet) <= 250, "telemetry fits ESP-NOW v1 frame");
    check(decode(packet, out) && out.t_us == packet.t_us && out.motor_us == packet.motor_us,
          "independent source clocks survive transmission");
    check(out.steps_l == packet.steps_l && out.steps_r == packet.steps_r &&
          out.enc_l == packet.enc_l && out.enc_r == packet.enc_r, "signed 64-bit counts survive transmission");
    check(out.theta == packet.theta && out.rate == packet.rate && out.tx_fail == 9,
          "float telemetry and callback drop counters survive transmission");
    check(out.ack_seq == 11 && out.received_seq == 12 && out.remote_state == (Bound | Expired),
          "receipt, application ACK and expired authority remain distinct");
    check(out.uart_age_l_us == UINT32_MAX, "no encoder sample uses explicit unknown age");
    out.boot_id = 77; out.motor_us = 12345; out.t_us = 12000;
    check(telemetryHopLe(out), "finite bounded telemetry accepted semantically");
    out.diag_flags=7;
    check(telemetryHopLe(out), "recorded A/B switches accepted");
    out.diag_flags=8;
    check(!telemetryHopLe(out), "unknown A/B flags rejected");
    out.diag_flags=0;
    out.theta = std::numeric_limits<float>::quiet_NaN();
    check(!telemetryHopLe(out), "valid CRC does not make NaN telemetry usable");
    out.theta = 0; out.angle_valid = 2;
    check(!telemetryHopLe(out), "invalid boolean telemetry rejected");
}
}

int main() {
    frameIntegrity();
    commands();
    sequenceAndLease();
    telemetry();
    std::cout << "PASS: ESP-NOW v1 CRC, bounds, boot/session replay, lease and telemetry (" << checks << " checks)\n";
}
