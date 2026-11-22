#pragma once
#include "GiaoThucChanDoanEspNow.h"

// Passive logger protocol. Subscription never grants motor/control authority.
namespace KhaoSatRung {
constexpr uint32_t magic = 0x31525658u; // XVR1
constexpr uint8_t version = 1;
constexpr uint16_t leaseMs = 3000;
enum class Type : uint8_t { Subscribe = 1, Samples = 2, Info = 3 };
enum Flags : uint8_t { MotorOn = 1, AngleValid = 2, CommandValid = 4, UiValid = 8 };
enum Source : uint8_t { None = 0, Web = 1, Other = 2, Remote = 3 };
#pragma pack(push, 1)
struct Header {
    uint32_t magic = 0;
    uint8_t version = 0;
    Type type = Type::Subscribe;
    uint16_t length = 0;
    uint32_t session = 0, seq = 0;
};
struct Subscribe {
    Header h{};
    uint16_t lease_ms = leaseMs;
    uint16_t crc = 0;
};
struct Sample {
    uint64_t t_us = 0;
    uint32_t command_age_us = UINT32_MAX, sample_seq = 0, cfg = 0;
    int16_t forward_pct = 0, turn_pct = 0; // UI before shaping; +forward / +left.
    uint8_t flags = 0, mode = 0, diag_flags = 0, source = None, speed_pct = 0;
    float ax = 0, ay = 0, az = 0; // sensor frame, m/s²; gravity included
    float gx = 0, gy = 0, gz = 0, temp_c = 0; // sensor frame, rad/s; calibrated gyro
    float theta = 0, rate = 0, target = 0, gyro_d = 0;
    float vref = 0, yawref = 0, cmd_l = 0, cmd_r = 0, a_cmd = 0, v = 0;
    float v_request = 0, yaw_request = 0; // actual request consumed by cascade
};
struct Samples {
    Header h{};
    uint32_t boot_id = 0, log_id = 0, queue_drop = 0, tx_drop = 0;
    uint16_t sample_hz = 0;
    uint8_t count = 0, imu = 1; // 1 = LSM6DSV_SPI
    Sample sample[2]{};
    uint16_t crc = 0;
};
struct Info {
    Header h{};
    uint32_t boot_id = 0, log_id = 0;
    uint64_t t_us = 0;
    uint32_t produced = 0, queue_drop = 0, tx_drop = 0, tx_fail = 0;
    uint32_t spi_hz = 0, imu_bus_errors = 0, imu_missed = 0, imu_late = 0;
    uint16_t sample_hz = 0;
    uint8_t enabled = 0, imu = 1;
    int8_t sck = -1, mosi = -1, miso = -1, cs = -1, interrupt = -1;
    uint8_t gyro_lpf = 0, accel_lpf = 0, filter_flags = 0, reserved = 0;
    uint16_t accel_range_g = 0, gyro_range_dps = 0;
    float eq = 0, kp = 0, ki = 0, kd = 0, kpv = 0, kiv = 0;
    float fusion_tau = 0, d_lpf_hz = 0;
    uint32_t cfg = 0;
    float gyro_bias[3]{};
    float trust_norm = 0, trust_start = 0, trust_end = 0, trust_timeout = 0;
    float mount[9]{};
    char firmware[32]{};
    uint16_t crc = 0;
};
#pragma pack(pop)
static_assert(sizeof(Sample) == 105 && sizeof(Samples) == 248 && sizeof(Info) <= 250,
              "Passive logging frame exceeds ESP-NOW legacy payload");
template<class T> constexpr Type typeOf() {
    static_assert(std::is_same<T, Subscribe>::value || std::is_same<T, Samples>::value ||
                  std::is_same<T, Info>::value, "Unsupported logger frame");
    if constexpr (std::is_same<T, Subscribe>::value) return Type::Subscribe;
    if constexpr (std::is_same<T, Samples>::value) return Type::Samples;
    return Type::Info;
}
template<class T> void pack(T& p, uint32_t session, uint32_t seq) {
    p.h = {magic, version, typeOf<T>(), static_cast<uint16_t>(sizeof(T)), session, seq};
    p.crc = ChanDoanEspNow::tinhCrc16(reinterpret_cast<const uint8_t*>(&p), sizeof(T)-2);
}
template<class T> bool decode(const uint8_t* data, size_t length, T& p) {
    if (!data || length != sizeof(T)) return false;
    T tmp{};
    std::memcpy(&tmp, data, sizeof(T));
    if (tmp.h.magic != magic || tmp.h.version != version || tmp.h.type != typeOf<T>() ||
        tmp.h.length != sizeof(T) || !tmp.h.session || !tmp.h.seq ||
        tmp.crc != ChanDoanEspNow::tinhCrc16(data, length-2)) return false;
    p = tmp;
    return true;
}
inline bool valid(const Sample& s) {
    if (!s.t_us || s.forward_pct < -100 || s.forward_pct > 100 || s.turn_pct < -100 ||
        s.turn_pct > 100 || s.speed_pct > 100 || s.source > Remote || s.mode > 1 ||
        (s.flags & ~15u) || (s.diag_flags & ~7u)) return false;
    const float values[] = {s.ax,s.ay,s.az,s.gx,s.gy,s.gz,s.temp_c,s.theta,s.rate,s.target,
        s.gyro_d,s.vref,s.yawref,s.cmd_l,s.cmd_r,s.a_cmd,s.v,s.v_request,s.yaw_request};
    for (float v : values) if (!std::isfinite(v)) return false;
    return true;
}
inline bool valid(const Samples& p) {
    if (!p.boot_id || !p.log_id || p.count < 1 || p.count > 2 || p.imu != 1 || !p.sample_hz) return false;
    for (unsigned i = 0; i < p.count; ++i) if (!valid(p.sample[i])) return false;
    return p.count < 2 || p.sample[1].t_us > p.sample[0].t_us;
}
inline bool valid(const Info& p) {
    return p.boot_id && p.imu == 1 && p.t_us && p.sample_hz && p.enabled <= 1 && !p.reserved &&
        std::memchr(p.firmware, '\0', sizeof(p.firmware));
}
} // namespace KhaoSatRung
