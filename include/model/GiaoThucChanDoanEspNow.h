#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// ESP32 và chương trình cầu nối dùng little endian, float IEEE-754 32-bit.
// Gói <=250 byte tương thích payload ESP-NOW v1/v2. CRC kiểm tra lỗi dữ liệu,
// việc xác thực MAC/peer và thay phiên chỉ được quyết định ở lớp ứng dụng.
namespace ChanDoanEspNow {
constexpr uint32_t magic = 0x314E4358u; // Các byte trên dây là "XCN1".
constexpr uint8_t version = 1u;
constexpr size_t kichThuocToiDa = 250u;
constexpr uint16_t chuKyTelemetry_ms = 20u;
constexpr uint16_t leaseToiDa_ms = 150u;
constexpr float vanTocToiDa_mps = 0.10f;
constexpr float tocDoQuayToiDa_radps = 0.30f;
constexpr uint32_t tuoiUartKhongCo = UINT32_MAX;

enum class LoaiGoi : uint8_t { Hello = 1, Lenh = 2, Telemetry = 3 };
// Stop đưa vận tốc/yaw về 0 và khóa quyền di chuyển. Disable yêu cầu tắt motor.
enum class MaLenh : uint8_t { Stop = 0, Arm = 1, Move = 2, Heartbeat = 3, Disable = 4 };
enum class KetQuaLenh : uint8_t { None = 0, Accepted = 1, Duplicate = 2, Rejected = 3 };
enum TrangThaiTuXa : uint8_t { Bound = 1u, ControlLease = 2u, Expired = 4u };

#pragma pack(push, 1)
struct Header {
    uint32_t magic = 0;
    uint8_t version = 0;
    LoaiGoi type = LoaiGoi::Hello;
    uint16_t length = 0;
    uint32_t session = 0; // Số ngẫu nhiên khác 0 do cầu nối tạo cho mỗi lần kết nối.
    uint32_t seq = 0;     // Tăng cho từng gói gửi, bỏ qua 0 khi quay vòng.
};

struct GoiHello {
    Header h{};
    uint16_t period_ms = chuKyTelemetry_ms;
    uint16_t reserved = 0;
    uint16_t crc = 0;
};

struct GoiLenh {
    Header h{};
    uint32_t boot_id = 0; // Nhận từ telemetry mới của đúng robot.
    MaLenh opcode = MaLenh::Stop;
    uint8_t flags = 0;
    uint16_t lease_ms = 0;
    float v_mps = 0;
    float yaw_radps = 0;
    uint16_t crc = 0;
};

struct GoiTelemetry {
    Header h{};
    uint64_t t_us = 0;     // Thời điểm mẫu IMU nguồn của trạng thái điều khiển.
    uint64_t motor_us = 0; // Thời điểm snapshot motor, lấy độc lập với IMU.
    uint32_t boot_id = 0;
    uint32_t sample_seq = 0;
    uint32_t control_seq = 0;
    uint32_t cfg = 0;
    uint32_t control_session = 0;
    uint32_t motor_session = 0;
    uint32_t ack_seq = 0;      // Lệnh đã áp dụng. ARM chỉ ACK sau xác nhận MotorTask.
    uint32_t received_seq = 0; // Lệnh mới hợp lệ đã nhận, có thể còn chờ áp dụng.
    uint8_t en = 0;
    uint8_t mode = 0;
    uint8_t control_fault = 0;
    uint8_t motor_fault = 0;
    uint8_t control_phase = 0;
    uint8_t motor_phase = 0;
    uint8_t angle_valid = 0;
    uint8_t feedback = 0;
    uint8_t ack_opcode = 0;
    uint8_t ack_result = 0; // KetQuaLenh của lần nhận lệnh gần nhất.
    uint8_t remote_state = 0; // Bitmask TrangThaiTuXa.
    uint16_t reserved = 0;
    float theta = 0, rate = 0, target = 0, eq = 0;
    float v = 0, vref = 0, yawref = 0;
    float cmd_l = 0, cmd_r = 0, a_cmd = 0;
    float gyro_d = 0, acc_norm = 0, innov = 0, trust = 0;
    float kp = 0, ki = 0, kd = 0;
    int64_t steps_l = 0, steps_r = 0, enc_l = 0, enc_r = 0;
    uint32_t uart_age_l_us = tuoiUartKhongCo, uart_age_r_us = tuoiUartKhongCo;
    uint32_t uart_rx_l = 0, uart_rx_r = 0;
    uint32_t imu_miss = 0, imu_i2c = 0, imu_late = 0;
    uint32_t tx_drop = 0, tx_fail = 0, rx_reject = 0, rx_duplicate = 0;
    uint16_t crc = 0;
};
#pragma pack(pop)

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "ESP-NOW requires float32 IEEE-754");
static_assert(sizeof(Header) == 16 && sizeof(GoiHello) == 22 && sizeof(GoiLenh) == 34,
              "ESP-NOW v1 command layout changed");
static_assert(sizeof(GoiTelemetry) == 223 && sizeof(GoiTelemetry) <= kichThuocToiDa,
              "ESP-NOW v1 telemetry layout changed or exceeds 250 bytes");
static_assert(offsetof(GoiHello, crc) == sizeof(GoiHello) - 2 &&
              offsetof(GoiLenh, crc) == sizeof(GoiLenh) - 2 &&
              offsetof(GoiTelemetry, crc) == sizeof(GoiTelemetry) - 2, "CRC must terminate each packet");

// CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, không reflect, xorout 0.
inline uint16_t tinhCrc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8u;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = static_cast<uint16_t>((crc & 0x8000u) ? (crc << 1u) ^ 0x1021u : crc << 1u);
        }
    }
    return crc;
}

template<class T> constexpr LoaiGoi loaiGoi() {
    static_assert(std::is_same<T, GoiHello>::value || std::is_same<T, GoiLenh>::value ||
                  std::is_same<T, GoiTelemetry>::value, "Unsupported ESP-NOW packet");
    if constexpr (std::is_same<T, GoiHello>::value) return LoaiGoi::Hello;
    if constexpr (std::is_same<T, GoiLenh>::value) return LoaiGoi::Lenh;
    return LoaiGoi::Telemetry;
}

template<class T> inline void dongGoi(T& goi, uint32_t session, uint32_t seq) {
    goi.h = {magic, version, loaiGoi<T>(), static_cast<uint16_t>(sizeof(T)), session, seq};
    goi.crc = tinhCrc16(reinterpret_cast<const uint8_t*>(&goi), sizeof(T) - sizeof(goi.crc));
}

template<class T> inline bool giaiMa(const uint8_t* data, size_t length, T& goi) {
    if (!data || length != sizeof(T)) return false;
    T tam{};
    std::memcpy(&tam, data, sizeof(tam));
    if (tam.h.magic != magic || tam.h.version != version || tam.h.type != loaiGoi<T>() ||
        tam.h.length != sizeof(T) || tam.h.session == 0 || tam.h.seq == 0 ||
        tam.crc != tinhCrc16(data, length - sizeof(tam.crc))) return false;
    goi = tam;
    return true;
}

inline bool helloHopLe(const GoiHello& goi) {
    return goi.reserved == 0 && goi.period_ms == chuKyTelemetry_ms;
}

inline bool telemetryHopLe(const GoiTelemetry& t) {
    if (!t.boot_id || !t.t_us || !t.motor_us || t.reserved || t.en > 1 || t.mode > 1 ||
        t.angle_valid > 1 || t.control_phase > 2 || t.motor_phase > 2 || t.feedback > 2 ||
        (t.remote_state & ~(Bound | ControlLease | Expired))) return false;
    const float values[] = {t.theta,t.rate,t.target,t.eq,t.v,t.vref,t.yawref,t.cmd_l,t.cmd_r,t.a_cmd,
        t.gyro_d,t.acc_norm,t.innov,t.trust,t.kp,t.ki,t.kd};
    for (float v : values) if (!std::isfinite(v)) return false;
    return t.trust >= 0 && t.trust <= 1;
}

inline bool lenhHopLe(const GoiLenh& goi, uint32_t bootHienTai) {
    if (!bootHienTai || goi.boot_id != bootHienTai || goi.flags != 0 ||
        !std::isfinite(goi.v_mps) || !std::isfinite(goi.yaw_radps)) return false;
    const bool coLease = goi.lease_ms > 0 && goi.lease_ms <= leaseToiDa_ms;
    const bool dungYen = goi.v_mps == 0 && goi.yaw_radps == 0;
    switch (goi.opcode) {
    case MaLenh::Stop:
    case MaLenh::Arm:
    case MaLenh::Disable:
        return dungYen && goi.lease_ms == 0;
    case MaLenh::Heartbeat:
        return dungYen && coLease;
    case MaLenh::Move:
        return coLease && std::fabs(goi.v_mps) <= vanTocToiDa_mps &&
               std::fabs(goi.yaw_radps) <= tocDoQuayToiDa_radps;
    }
    return false;
}

// Tính hạn từ thời điểm callback nhận gói để thời gian chờ xử lý không kéo dài lệnh.
inline bool leaseConHieuLuc(uint16_t lease_ms, uint64_t received_us, uint64_t now_us) {
    return lease_ms > 0 && lease_ms <= leaseToiDa_ms && now_us >= received_us &&
           now_us - received_us < static_cast<uint64_t>(lease_ms) * 1000u;
}

inline bool thuTuMoiHon(uint32_t seq, uint32_t truoc) {
    const uint32_t delta = seq - truoc;
    return seq != 0 && delta != 0 && delta < 0x80000000u;
}

class BoLocThuTu {
public:
    void reset(uint32_t session) { phien = session; thuTu = 0; }
    bool chapNhan(uint32_t session, uint32_t seq) {
        if (!phien || session != phien || !seq || (thuTu && !thuTuMoiHon(seq, thuTu))) return false;
        thuTu = seq;
        return true;
    }
    uint32_t soThuTuCuoi() const { return thuTu; }
private:
    uint32_t phien = 0, thuTu = 0;
};
} // namespace ChanDoanEspNow
