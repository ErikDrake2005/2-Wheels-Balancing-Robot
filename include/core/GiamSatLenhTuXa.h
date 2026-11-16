#pragma once
#include "model/GiaoThucChanDoanEspNow.h"

// Gọi dưới cùng một khóa ở lớp ứng dụng. Không tự bật/tắt phần cứng.
class GiamSatLenhTuXa {
public:
    struct Decision { bool arm = false, disable = false; uint32_t seq = 0; };
    explicit GiamSatLenhTuXa(uint32_t boot = 1) : bootId(boot) {}
    bool hello(const ChanDoanEspNow::GoiHello& p, bool motorOff) {
        if (!ChanDoanEspNow::helloHopLe(p)) return reject();
        if (session != p.h.session) {
            if (!motorOff || authority || pendingArm || armAwaiting) return reject();
            session = p.h.session; order.reset(session); ack = 0; expired = false;
            pendingStop = pendingDisable = false; received = 0;
        }
        if (!order.chapNhan(session, p.h.seq)) { ++duplicates; return false; }
        return true;
    }
    bool receive(const ChanDoanEspNow::GoiLenh& p, uint64_t at) {
        using namespace ChanDoanEspNow;
        if (!session || p.h.session != session || !lenhHopLe(p, bootId)) return reject();
        if (!order.chapNhan(session, p.h.seq)) {
            ++duplicates; result = KetQuaLenh::Duplicate; return false;
        }
        received = p.h.seq; opcode = p.opcode; result = KetQuaLenh::Accepted;
        if (p.opcode == MaLenh::Stop || p.opcode == MaLenh::Disable) {
            authority = false; expired = true; v = yaw = 0; pendingArm = armAwaiting = 0;
            if (p.opcode == MaLenh::Disable) pendingDisable = p.h.seq;
            else pendingStop = p.h.seq;
            return true;
        }
        if (pendingStop || pendingDisable) return reject();
        if (p.opcode == MaLenh::Arm) {
            pendingArm = p.h.seq; armAt = at; return true;
        }
        // HB chỉ giữ quyền điều khiển; không làm mới hạn của vận tốc khác 0.
        if (!authority || expired || at >= heartbeatUntil) {
            authority = false; expired = true; v = yaw = 0; return reject();
        }
        heartbeatUntil = at + static_cast<uint64_t>(p.lease_ms) * 1000;
        if (p.opcode == MaLenh::Move) {
            v = p.v_mps; yaw = p.yaw_radps; motionUntil = heartbeatUntil;
            pendingMove = p.h.seq;
        } else {
            pendingHeartbeat = p.h.seq;
        }
        return true;
    }
    Decision service(uint64_t now, bool motorEnabled) {
        Decision d;
        if (pendingDisable) {
            d.disable = true; d.seq = pendingDisable; disableAwaiting = pendingDisable;
            pendingDisable = pendingStop = 0; pendingMove = pendingHeartbeat = 0;
            return d;
        }
        if (pendingStop) {
            applied(pendingStop); pendingStop = 0; pendingMove = pendingHeartbeat = 0;
        }
        if (pendingArm) {
            const uint32_t seq = pendingArm; pendingArm = 0;
            if (now < armAt || now - armAt >= 150000) {
                authority = false; expired = true; result = ChanDoanEspNow::KetQuaLenh::Rejected;
            } else {
                authority = true; expired = false; v = yaw = 0; motionUntil = 0;
                heartbeatUntil = armAt + 150000; armAwaiting = seq;
                d.arm = !motorEnabled; d.seq = seq;
            }
        }
        if (authority && now >= heartbeatUntil) {
            authority = false; expired = true; v = yaw = 0;
            armAwaiting = pendingMove = pendingHeartbeat = 0;
        }
        if (motionUntil && now >= motionUntil) { v = yaw = 0; motionUntil = 0; }
        if (motorEnabled && authority && armAwaiting) { applied(armAwaiting); armAwaiting = 0; }
        if (!motorEnabled && disableAwaiting) { applied(disableAwaiting); disableAwaiting = 0; }
        if (pendingMove) { applied(pendingMove); pendingMove = 0; }
        if (pendingHeartbeat) { applied(pendingHeartbeat); pendingHeartbeat = 0; }
        return d;
    }
    void desired(uint64_t now, float& speed, float& turn) const {
        speed = turn = 0;
        if (authority && !expired && now < heartbeatUntil && motionUntil && now < motionUntil) {
            speed = v; turn = yaw;
        }
    }
    void cancel() {
        authority = false; expired = true; v = yaw = 0;
        pendingArm = armAwaiting = pendingMove = pendingHeartbeat = 0;
    }
    void telemetry(ChanDoanEspNow::GoiTelemetry& t) const {
        t.boot_id = bootId; t.h.session = session; t.ack_seq = ack; t.received_seq = received;
        t.ack_opcode = static_cast<uint8_t>(opcode); t.ack_result = static_cast<uint8_t>(result);
        t.remote_state = (session ? ChanDoanEspNow::Bound : 0) |
            (authority ? ChanDoanEspNow::ControlLease : 0) | (expired ? ChanDoanEspNow::Expired : 0);
        t.rx_reject = rejects; t.rx_duplicate = duplicates;
    }
    uint32_t getSession() const { return session; }
private:
    bool reject() { ++rejects; result = ChanDoanEspNow::KetQuaLenh::Rejected; return false; }
    void applied(uint32_t seq) { if (!ack || ChanDoanEspNow::thuTuMoiHon(seq, ack)) ack = seq; }
    uint32_t bootId, session = 0, received = 0, ack = 0, rejects = 0, duplicates = 0;
    ChanDoanEspNow::BoLocThuTu order;
    ChanDoanEspNow::MaLenh opcode = ChanDoanEspNow::MaLenh::Stop;
    ChanDoanEspNow::KetQuaLenh result = ChanDoanEspNow::KetQuaLenh::None;
    uint32_t pendingArm = 0, pendingStop = 0, pendingDisable = 0, pendingMove = 0, pendingHeartbeat = 0;
    uint32_t armAwaiting = 0, disableAwaiting = 0;
    uint64_t armAt = 0, heartbeatUntil = 0, motionUntil = 0;
    float v = 0, yaw = 0;
    bool authority = false, expired = false;
};
