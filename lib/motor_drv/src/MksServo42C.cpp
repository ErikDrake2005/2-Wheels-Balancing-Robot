#include "MksServo42C.h"
#include "config/MotorConfig.h"
#include "config/PinConfig.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <cmath>
#include <initializer_list>

namespace {
uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
}

bool MksServo42CPair::begin() {
    if (_daKhoiTao) return false;
    if (!std::isfinite(tanSoStepToiDa_hz) || tanSoStepToiDa_hz <= 0.0f ||
        !std::isfinite(tanSoChayLienTucToiThieu_hz) || tanSoChayLienTucToiThieu_hz <= 0.0f ||
        tanSoChayLienTucToiThieu_hz > tanSoStepToiDa_hz ||
        chuKyPhucVuMotor_us == 0 || thoiGianHetHanLenhMotor_us == 0 ||
        !std::isfinite(gioiHanTonDongXungThua_buoc) || gioiHanTonDongXungThua_buoc <= 1.0 ||
        khoangCachPhucVuMotorToiDa_us < chuKyPhucVuMotor_us ||
        tanSoChayLienTucToiThieu_hz * chuKyPhucVuMotor_us / 1000000.0f >= 1.0f) return false;

    const int pins[] = {chanStepTrai, chanDirTrai, chanStepPhai, chanDirPhai, chanBatMotor};
    for (unsigned i = 0; i < 5; ++i) {
        if (pins[i] < 0 || !GPIO_IS_VALID_OUTPUT_GPIO(pins[i])) return false;
        for (unsigned j = 0; j < i; ++j) if (pins[i] == pins[j]) return false;
    }

    const auto enPin = static_cast<gpio_num_t>(chanBatMotor);
    if (gpio_set_level(enPin, mucBatMotorCao ? 0 : 1) != ESP_OK ||
        gpio_set_direction(enPin, GPIO_MODE_OUTPUT) != ESP_OK) return false;

    _banhTrai.chanDir = chanDirTrai;
    _banhPhai.chanDir = chanDirPhai;
    _banhTrai.mucDirTienCao = mucDirTienBanhTraiCao;
    _banhPhai.mucDirTienCao = mucDirTienBanhPhaiCao;
    for (Axis* a : {&_banhTrai, &_banhPhai}) {
        const auto pin = static_cast<gpio_num_t>(a->chanDir);
        if (gpio_set_level(pin, a->mucDirTienCao ? 1 : 0) != ESP_OK ||
            gpio_set_direction(pin, GPIO_MODE_OUTPUT) != ESP_OK) return false;
    }

    const bool leftOk = _banhTrai.step.begin(chanStepTrai, thoiGianMucCaoStep_us, thoiGianMucThapToiThieuStep_us);
    const bool rightOk = _banhPhai.step.begin(chanStepPhai, thoiGianMucCaoStep_us, thoiGianMucThapToiThieuStep_us);
    if (!leftOk || !rightOk) {
        if (leftOk) _banhTrai.step.stop();
        if (rightOk) _banhPhai.step.stop();
        _loi = MksFault::BackendError;
        return false; // Không bật EN; reset sau khi sửa nguyên nhân.
    }
    if (!_banhTrai.step.supportsFrequency(tanSoChayLienTucToiThieu_hz) ||
        !_banhTrai.step.supportsFrequency(tanSoStepToiDa_hz) ||
        !_banhPhai.step.supportsFrequency(tanSoChayLienTucToiThieu_hz) ||
        !_banhPhai.step.supportsFrequency(tanSoStepToiDa_hz)) {
        _banhTrai.step.stop(); _banhPhai.step.stop();
        return false;
    }
    _daKhoiTao = true;
    return true;
}

MksResult MksServo42CPair::enable() {
    if (!_daKhoiTao) return MksResult::NotInitialized;
    if (_loi != MksFault::None) return MksResult::FaultLatched;
    if (!daXacNhanCauHinhPhanCungMotor) return MksResult::HardwareConfigUnconfirmed;
    if (_daBat) return MksResult::Ok;
    if (!_banhTrai.step.isPauseReady() || !_banhPhai.step.isPauseReady()) return MksResult::Pending;
    if (gpio_set_level(static_cast<gpio_num_t>(chanBatMotor), mucBatMotorCao ? 1 : 0) != ESP_OK) {
        trip(MksFault::BackendError);
        return MksResult::FaultLatched;
    }
    const uint64_t now = nowUs();
    _daBat = true;
    _sanSangBat_us = now + thoiGianOnDinhEnable_us;
    _hanLenh_us = now + thoiGianHetHanLenhMotor_us;
    _lanPhucVuCuoi_us = now;
    _thoiDiemRaLenhCuoi_us = 0;
    return MksResult::Ok;
}

bool MksServo42CPair::disable() {
    // EN off trước khi có thể cắt xung HIGH: thao tác bảo vệ, không phải đảo chiều.
    const bool enOk = gpio_set_level(static_cast<gpio_num_t>(chanBatMotor),
                                   mucBatMotorCao ? 0 : 1) == ESP_OK;
    bool stepOk = true;
    if (_daKhoiTao) {
        const bool a = _banhTrai.step.stop();
        const bool b = _banhPhai.step.stop();
        stepOk = a && b;
    }
    _daBat = false;
    _hanLenh_us = 0;
    _lanPhucVuCuoi_us = 0;
    _biGioiHan = false;
    for (Axis* a : {&_banhTrai, &_banhPhai}) {
        a->tocDoMucTieu = 0.0f;
        a->tichLuyBuoc = 0.0;
        a->giaiDoan = Phase::Idle;
        a->cheDo = Mode::None;
        a->tanSoDaCauHinh_hz = 0.0f;
    }
    if (!enOk || !stepOk) _loi = MksFault::BackendError;
    return enOk && stepOk;
}

void MksServo42CPair::trip(MksFault reason) {
    // Giữ nguyên nguyên nhân đầu tiên, trừ khi ngay thao tác disable cũng lỗi.
    if (_loi == MksFault::None) _loi = reason;
    disable();
}

bool MksServo42CPair::emergencyStop() {
    if (_loi == MksFault::None) _loi = MksFault::EmergencyStop;
    return disable();
}

bool MksServo42CPair::clearFault() {
    if (!_daKhoiTao || _daBat) return false;
    if (!_banhTrai.step.stop() || !_banhPhai.step.stop()) return false;
    _loi = MksFault::None;
    _thoiDiemRaLenhCuoi_us = 0;
    return true;
}

bool MksServo42CPair::requestPause(Axis& a) {
    if (a.giaiDoan == Phase::Pausing) return true;
    if (!a.step.pauseAfterPulse()) return false;
    // Đếm từ SAU khi đã cấm cạnh lên, bảo đảm DIR hold tối thiểu.
    a.choDen_us = nowUs() + thoiGianGiuDir_us;
    a.giaiDoan = Phase::Pausing;
    return true;
}

bool MksServo42CPair::writeDirection(Axis& a, bool dangTien) {
    const bool high = dangTien ? a.mucDirTienCao : !a.mucDirTienCao;
    if (gpio_set_level(static_cast<gpio_num_t>(a.chanDir), high ? 1 : 0) != ESP_OK) return false;
    a.dangTien = dangTien;
    a.choDen_us = nowUs() + thoiGianOnDinhDir_us;
    a.giaiDoan = Phase::DirSetup;
    return true;
}

bool MksServo42CPair::advance(Axis& a, double dtSeconds, uint64_t now) {
    const float hz = std::fabs(a.tocDoMucTieu);
    const bool desiredForward = a.tocDoMucTieu >= 0.0f;
    const Mode desiredMode = hz >= tanSoChayLienTucToiThieu_hz ? Mode::Continuous : Mode::Sparse;

    if (a.giaiDoan == Phase::Running &&
        (hz == 0.0f || desiredForward != a.dangTien || desiredMode != a.cheDo)) {
        return requestPause(a);
    }
    if (a.giaiDoan == Phase::Pausing) {
        if (!a.step.isPauseReady() || now < a.choDen_us) return true;
        a.giaiDoan = Phase::Idle;
        a.cheDo = Mode::None;
        a.tanSoDaCauHinh_hz = 0.0f;
        // Không phát bù phần bước cũ sau khi đổi chiều/chế độ.
        a.tichLuyBuoc = 0.0;
    }
    if (hz == 0.0f) {
        a.tichLuyBuoc = 0.0;
        a.giaiDoan = Phase::Idle;
        a.cheDo = Mode::None;
        return true;
    }
    if (desiredForward != a.dangTien) return writeDirection(a, desiredForward);
    if (a.giaiDoan == Phase::DirSetup) {
        if (now < a.choDen_us) return true;
        a.giaiDoan = Phase::Idle;
    }
    // Kể cả mục tiêu đã về 0 rồi tăng lại trong lúc DirSetup,
    // vẫn không bỏ qua khoảng ổn định DIR đã đặt trước đó.
    if (now < _sanSangBat_us || now < a.choDen_us) return true;

    if (a.giaiDoan == Phase::Idle) {
        if (!a.step.isPauseReady()) return true;
        a.cheDo = desiredMode;
        a.giaiDoan = Phase::Running;
        if (a.cheDo == Mode::Continuous) {
            if (!a.step.setFrequency(hz)) return false;
            a.tanSoDaCauHinh_hz = hz;
            return true;
        }
        a.tichLuyBuoc = 0.0;
        return true; // Bắt đầu tích lũy từ service tiếp theo.
    }
    if (a.cheDo == Mode::Continuous) {
        if (hz == a.tanSoDaCauHinh_hz) return true;
        if (!a.step.setFrequency(hz)) return false;
        a.tanSoDaCauHinh_hz = hz;
        return true;
    }

    // Xung thưa: 0.25 step + 0.25 step + 0.25 step + 0.25 step => một STEP.
    // Không bỏ các tốc độ nhỏ, không tăng tốc độ lên min frequency của timer.
    a.tichLuyBuoc += static_cast<double>(hz) * dtSeconds;
    if (a.tichLuyBuoc >= gioiHanTonDongXungThua_buoc) {
        trip(MksFault::SparseBacklog); // Task bị chậm; không phát một cụm bù giật.
        return false;
    }
    if (a.tichLuyBuoc >= 1.0 && a.step.isPauseReady()) {
        if (!a.step.emitOnePulse()) return false;
        a.tichLuyBuoc -= 1.0;
    }
    return true;
}

bool MksServo42CPair::dangChoXuLy() const {
    for (const Axis* a : {&_banhTrai, &_banhPhai}) {
        if (a->giaiDoan == Phase::Pausing || a->giaiDoan == Phase::DirSetup ||
            (a->giaiDoan == Phase::Idle && a->tocDoMucTieu != 0.0f)) return true;
    }
    return false;
}

MksResult MksServo42CPair::service() {
    if (!_daKhoiTao) return MksResult::NotInitialized;
    if (_loi != MksFault::None) return MksResult::FaultLatched;
    if (!_daBat) return MksResult::Disabled;
    const uint64_t now = nowUs();
    if (now >= _hanLenh_us) {
        trip(MksFault::CommandExpired);
        return MksResult::FaultLatched;
    }
    const uint64_t khoangCachMauUs = now - _lanPhucVuCuoi_us;
    if (khoangCachMauUs > khoangCachPhucVuMotorToiDa_us) {
        trip(MksFault::ServiceLate);
        return MksResult::FaultLatched;
    }
    _lanPhucVuCuoi_us = now;
    const double dt = static_cast<double>(khoangCachMauUs) / 1000000.0;
    if (!advance(_banhTrai, dt, now) || !advance(_banhPhai, dt, now)) {
        trip(MksFault::BackendError);
        return MksResult::FaultLatched;
    }
    return dangChoXuLy() ? MksResult::Pending : MksResult::Ok;
}

MksResult MksServo42CPair::apply(const WheelStepCommand& c) {
    if (!_daKhoiTao) return MksResult::NotInitialized;
    if (_loi != MksFault::None) return MksResult::FaultLatched;
    if (!_daBat) return MksResult::Disabled;

    // Tích lũy khoảng vừa qua theo mục tiêu CŨ trước khi nhận mục tiêu mới.
    if (service() == MksResult::FaultLatched) return MksResult::FaultLatched;
    const uint64_t now = nowUs();
    if (!std::isfinite(c.tocDoBuocTrai_buocTrenGiay) || !std::isfinite(c.tocDoBuocPhai_buocTrenGiay) ||
        c.thoiDiemRaLenh_us > now || c.thoiDiemHetHan_us <= c.thoiDiemRaLenh_us ||
        c.thoiDiemRaLenh_us < _thoiDiemRaLenhCuoi_us) {
        trip(MksFault::InvalidCommand);
        return MksResult::FaultLatched;
    }
    const uint64_t age = now - c.thoiDiemRaLenh_us;
    if (c.thoiDiemHetHan_us <= now || age >= thoiGianHetHanLenhMotor_us) {
        trip(MksFault::CommandExpired);
        return MksResult::FaultLatched;
    }

    // Giới hạn cả cặp theo cùng tỷ lệ, tránh làm lệch tỷ lệ trái/phải.
    float left = c.tocDoBuocTrai_buocTrenGiay;
    float right = c.tocDoBuocPhai_buocTrenGiay;
    const float largest = std::fmax(std::fabs(left), std::fabs(right));
    _biGioiHan = largest > tanSoStepToiDa_hz;
    if (_biGioiHan) {
        const float ratio = tanSoStepToiDa_hz / largest;
        left *= ratio;
        right *= ratio;
    }
    _banhTrai.tocDoMucTieu = left;
    _banhPhai.tocDoMucTieu = right;
    _thoiDiemRaLenhCuoi_us = c.thoiDiemRaLenh_us;
    // Hạn tính từ issuedAt, không gia hạn mãi bằng cách gửi lại cùng lệnh cũ.
    const uint64_t remaining = thoiGianHetHanLenhMotor_us - age;
    const uint64_t configuredDeadline = now + remaining;
    _hanLenh_us = c.thoiDiemHetHan_us < configuredDeadline ? c.thoiDiemHetHan_us : configuredDeadline;
    const MksResult tienDo = service();
    if (tienDo == MksResult::FaultLatched) return tienDo;
    return _biGioiHan ? MksResult::Saturated : tienDo;
}

MksResult MksServo42CPair::stopMotion() {
    if (!_daKhoiTao) return MksResult::NotInitialized;
    if (_loi != MksFault::None) return MksResult::FaultLatched;
    _banhTrai.tocDoMucTieu = 0.0f;
    _banhPhai.tocDoMucTieu = 0.0f;
    return service(); // Không gia hạn watchdog; controller vẫn phải gửi lệnh mới.
}

MksSnapshot MksServo42CPair::snapshot() const {
    MksSnapshot s{};
    s.daKhoiTao = _daKhoiTao;
    s.daBat = _daBat;
    s.dangChoXuLy = dangChoXuLy();
    s.biGioiHan = _biGioiHan;
    s.loi = _loi;
    s.tocDoBuocYeuCauTrai_buocTrenGiay = _banhTrai.tocDoMucTieu;
    s.tocDoBuocYeuCauPhai_buocTrenGiay = _banhPhai.tocDoMucTieu;
    s.tanSoLienTucTrai_hz = _banhTrai.step.actualFrequencyHz();
    s.tanSoLienTucPhai_hz = _banhPhai.step.actualFrequencyHz();
    s.thoiDiemHetHan_us = _hanLenh_us;
    return s;
}
