#ifndef MKS_SERVO42C_H
#define MKS_SERVO42C_H
#include <cstdint>
#include "McpwmStepOutput.h"
#include "GiamSatDongCo.h"
#include "model/WheelStepCommand.h"

enum class MksResult : uint8_t {
    Ok, Saturated, Pending, Disabled, NotInitialized, ConfigInvalid,
    HardwareConfigUnconfirmed, FaultLatched
};
enum class MksFault : uint8_t {
    None, InvalidCommand, CommandExpired, ServiceLate, BackendError, SparseBacklog,
    EmergencyStop, WatchdogExpired
};

struct MksSnapshot {
    bool daKhoiTao = false;
    bool daBat = false; // Mức EN mà phần mềm đã đặt, không phải phản hồi MKS.
    bool dangChoXuLy = false;
    bool biGioiHan = false;
    MksFault loi = MksFault::None;
    float tocDoBuocYeuCauTrai_buocTrenGiay = 0.0f;
    float tocDoBuocYeuCauPhai_buocTrenGiay = 0.0f;
    double tanSoLienTucTrai_hz = 0.0;
    double tanSoLienTucPhai_hz = 0.0;
    uint64_t thoiDiemHetHan_us = 0;
};

// Chủ của STEP/DIR. EN được cấp qua GiamSatDongCo; ISR giám sát chỉ có quyền tắt EN.
// Gọi API từ cùng một MotorTask; không gọi từ ISR.
// ISR/điều khiển/safety khác gửi lệnh hoặc notification tới task sở hữu.
class MksServo42CPair {
public:
    bool begin();
    MksResult enable();
    bool disable();
    bool isEnabled() const { return _daBat; }

    // Kiểm tra toàn bộ lệnh trước khi cập nhật hai mục tiêu.
    // Không tự enable. Lệnh mới thay lệnh cũ, không có queue trong class.
    MksResult apply(const WheelStepCommand& lenh);

    // Dừng chuyển động bình thường, giữ EN/lực giữ.
    MksResult stopMotion();

    // Gọi định kỳ theo chuKyPhucVuMotor_us và sau khi có lệnh mới.
    // Tiến trình đổi DIR/STEP và phát bước thưa đều không dùng delay().
    MksResult service();

    // Ngắt EN và STEP ngay; loi giữ lại tới khi clearFault().
    bool emergencyStop();
    bool clearFault(); // Chỉ khi disabled; không tự enable lại.
    MksSnapshot snapshot() const;

private:
    enum class Phase : uint8_t { Idle, Running, Pausing, DirSetup };
    enum class Mode : uint8_t { None, Continuous, Sparse };
    struct Axis {
        McpwmStepOutput step;
        int chanDir = -1;
        bool mucDirTienCao = true;
        bool dangTien = true;
        float tocDoMucTieu = 0.0f;
        Phase giaiDoan = Phase::Idle;
        Mode cheDo = Mode::None;
        uint64_t choDen_us = 0;
        // Bước chưa đủ một xung, tích lũy riêng cho mỗi bánh.
        double tichLuyBuoc = 0.0;
        float tanSoDaCauHinh_hz = 0.0f; // Tránh gọi API MCPWM lặp khi mục tiêu không đổi.
    };
    bool advance(Axis& axis, double dtSeconds, uint64_t nowUs);
    bool requestPause(Axis& axis);
    bool writeDirection(Axis& axis, bool dangTien);
    void trip(MksFault reason);
    bool dangChoXuLy() const;

    Axis _banhTrai;
    Axis _banhPhai;
    GiamSatDongCo _giamSat;
    bool _daKhoiTao = false;
    bool _daBat = false;
    bool _biGioiHan = false;
    MksFault _loi = MksFault::None;
    uint64_t _hanLenh_us = 0;
    uint64_t _sanSangBat_us = 0;
    uint64_t _lanPhucVuCuoi_us = 0;
    uint64_t _thoiDiemRaLenhCuoi_us = 0;
};
#endif
