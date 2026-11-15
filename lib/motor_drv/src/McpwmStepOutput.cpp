#include "McpwmStepOutput.h"
#include "config/MotorConfig.h"
#include "driver/gpio.h"
#include "hal/mcpwm_ll.h"
#include "esp_timer.h"
#include <cmath>

// Chỉ phát xung bằng peripheral; không đảo STEP bằng vòng lặp phần mềm.

namespace {
uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
uint64_t usToTicks(uint32_t us, uint32_t hz) {
    return (static_cast<uint64_t>(us) * hz + 999999ULL) / 1000000ULL;
}
}

bool IRAM_ATTR McpwmStepOutput::onTimerStop(
    mcpwm_timer_handle_t, const mcpwm_timer_event_data_t*, void* context) {
    auto* self = static_cast<McpwmStepOutput*>(context);
    portENTER_CRITICAL_ISR(&self->_pulseMux);
    self->_dangPhatMotXung = false;
    portEXIT_CRITICAL_ISR(&self->_pulseMux);
    return false; // Callback không đánh thức task nào.
}

bool McpwmStepOutput::oneShotBusy() const {
    portENTER_CRITICAL(&_pulseMux);
    const bool busy = _dangPhatMotXung;
    portEXIT_CRITICAL(&_pulseMux);
    return busy;
}

void McpwmStepOutput::setOneShotBusy(bool busy) {
    portENTER_CRITICAL(&_pulseMux);
    _dangPhatMotXung = busy;
    portEXIT_CRITICAL(&_pulseMux);
}

bool McpwmStepOutput::setEmptyAction(mcpwm_generator_action_t action) {
    return mcpwm_generator_set_action_on_timer_event(
        _generator, MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, action)) == ESP_OK;
}

bool McpwmStepOutput::cleanupInit() {
    // Chỉ gọi khi begin() chưa phát xung. Giữ handle nếu API xóa thất bại.
    _daKhoiTao = false;
    if (_generator) {
        if (mcpwm_del_generator(_generator) != ESP_OK) return false;
        _generator = nullptr;
    }
    if (_comparator) {
        if (mcpwm_del_comparator(_comparator) != ESP_OK) return false;
        _comparator = nullptr;
    }
    if (_operator) {
        if (mcpwm_del_operator(_operator) != ESP_OK) return false;
        _operator = nullptr;
    }
    if (_timer && _sync) {
        mcpwm_timer_sync_phase_config_t detach{};
        detach.direction = MCPWM_TIMER_DIRECTION_UP;
        if (mcpwm_timer_set_phase_on_sync(_timer, &detach) != ESP_OK) return false;
    }
    if (_sync) {
        if (mcpwm_del_sync_src(_sync) != ESP_OK) return false;
        _sync = nullptr;
    }
    if (_daBatBoDem) {
        if (mcpwm_timer_disable(_timer) != ESP_OK) return false;
        _daBatBoDem = false;
    }
    if (_timer) {
        if (mcpwm_del_timer(_timer) != ESP_OK) return false;
        _timer = nullptr;
    }
    return false;
}

bool McpwmStepOutput::begin(int stepPin, uint32_t pulseHighUs, uint32_t minLowUs) {
    if (_daKhoiTao || _timer || _operator || _comparator || _generator || _sync) return false;
    if (stepPin < 0 || !GPIO_IS_VALID_OUTPUT_GPIO(stepPin)) return false;
    _dophangiaiHz = doPhanGiaiBoDemMcpwm_hz;
    if (!_dophangiaiHz || !pulseHighUs || !minLowUs) return false;

    const uint64_t high = usToTicks(pulseHighUs, _dophangiaiHz);
    const uint64_t low = usToTicks(minLowUs, _dophangiaiHz);
    const uint64_t period = usToTicks(chuKyKhoiTaoMcpwm_us, _dophangiaiHz);
    // Dung count_value = 1 khi sync de tranh trung su kien EMPTY (count = 0),
    // nen nguong CMP phai dat o (high + 1) de dam bao be rong xung HIGH du pulseHighUs.
    if (high == 0 || low == 0 || high + 1u + low > period ||
        period >= MCPWM_LL_MAX_COUNT_VALUE) return false;
    _soTickMucCao = static_cast<uint32_t>(high) + 1u;
    _soTickMucThap = static_cast<uint32_t>(low);
    _soTickChuKy = static_cast<uint32_t>(period);

    // Khoảng chờ sau khi cấm cạnh lên: đủ cho HIGH còn lại và LOW tối thiểu.
    // Làm tròn lên theo tick thực sự đã chọn, không giả định 1 tick = 1 us.
    const uint64_t quietUs = ((high + low) * 1000000ULL + _dophangiaiHz - 1ULL)
                            / _dophangiaiHz + thoiGianBaoVeTamDungMcpwm_us;
    if (quietUs > UINT32_MAX) return false;
    _thoiGianChoYen_us = static_cast<uint32_t>(quietUs);

    const auto gpio = static_cast<gpio_num_t>(stepPin);
    if (gpio_set_level(gpio, 0) != ESP_OK ||
        gpio_set_direction(gpio, GPIO_MODE_OUTPUT) != ESP_OK) return false;

    mcpwm_timer_config_t tc{};
    tc.group_id = nhomMcpwm;
    tc.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    tc.resolution_hz = _dophangiaiHz;
    tc.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    tc.period_ticks = _soTickChuKy;
    tc.flags.update_period_on_empty = true;
    tc.flags.update_period_on_sync = true;
    if (mcpwm_new_timer(&tc, &_timer) != ESP_OK) return cleanupInit();

    mcpwm_operator_config_t oc{};
    oc.group_id = nhomMcpwm;
    if (mcpwm_new_operator(&oc, &_operator) != ESP_OK) return cleanupInit();
    if (mcpwm_operator_connect_timer(_operator, _timer) != ESP_OK) return cleanupInit();

    mcpwm_comparator_config_t cc{};
    cc.flags.update_cmp_on_tez = true;
    cc.flags.update_cmp_on_sync = true;
    if (mcpwm_new_comparator(_operator, &cc, &_comparator) != ESP_OK) return cleanupInit();
    if (mcpwm_comparator_set_compare_value(_comparator, _soTickMucCao) != ESP_OK) return cleanupInit();

    mcpwm_generator_config_t gc{};
    gc.gen_gpio_num = stepPin;
    if (mcpwm_new_generator(_operator, &gc, &_generator) != ESP_OK) return cleanupInit();
    if (mcpwm_generator_set_force_level(_generator, 0, true) != ESP_OK) return cleanupInit();
    if (!setEmptyAction(MCPWM_GEN_ACTION_LOW)) return cleanupInit();
    if (mcpwm_generator_set_action_on_compare_event(
        _generator, MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, _comparator, MCPWM_GEN_ACTION_LOW)) != ESP_OK) return cleanupInit();

    // Sync dùng một lần lúc phát lại để không chờ chu kỳ cũ dài kết thúc.
    mcpwm_soft_sync_config_t sc{};
    if (mcpwm_new_soft_sync_src(&sc, &_sync) != ESP_OK) return cleanupInit();
    mcpwm_timer_sync_phase_config_t phase{};
    phase.sync_src = _sync;
    phase.count_value = 1;
    phase.direction = MCPWM_TIMER_DIRECTION_UP;
    if (mcpwm_timer_set_phase_on_sync(_timer, &phase) != ESP_OK) return cleanupInit();
    if (mcpwm_generator_set_action_on_sync_event(
        _generator, MCPWM_GEN_SYNC_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, _sync, MCPWM_GEN_ACTION_HIGH)) != ESP_OK) return cleanupInit();

    // Chỉ on_stop: không tạo interrupt cho từng xung liên tục.
    mcpwm_timer_event_callbacks_t callbacks{};
    callbacks.on_stop = onTimerStop;
    if (mcpwm_timer_register_event_callbacks(_timer, &callbacks, this) != ESP_OK) return cleanupInit();
    if (mcpwm_timer_enable(_timer) != ESP_OK) return cleanupInit();
    _daBatBoDem = true;
    _daKhoiTao = true;
    _daYeuCauTamDung = true;
    _tamDungSanSang_us = 0;
    return true;
}

bool McpwmStepOutput::supportsFrequency(float hz) const {
    if (!_daKhoiTao || !std::isfinite(hz) || hz <= 0.0f) return false;
    const double ticks = std::ceil(static_cast<double>(_dophangiaiHz) / hz);
    return std::isfinite(ticks) && ticks >= static_cast<double>(_soTickMucCao + _soTickMucThap)
        && ticks < MCPWM_LL_MAX_COUNT_VALUE;
}

bool McpwmStepOutput::pauseAfterPulse() {
    if (!_daKhoiTao) return false;
    if (_daYeuCauTamDung) return true; // Không gia hạn thời gian chờ mỗi lần gọi.
    // Ngăn cạnh lên tương lai nhưng không cắt HIGH đang diễn ra.
    if (!setEmptyAction(MCPWM_GEN_ACTION_LOW)) return false;
    _tamDungSanSang_us = nowUs() + _thoiGianChoYen_us;
    _daYeuCauTamDung = true;
    _dangChayLienTuc = false;
    return true;
}

bool McpwmStepOutput::isPauseReady() const {
    return _daKhoiTao && _daYeuCauTamDung && nowUs() >= _tamDungSanSang_us
        && !oneShotBusy();
}

bool McpwmStepOutput::prepareStart(uint32_t periodTicks) {
    if (!isPauseReady()) return false;
    if (!setEmptyAction(MCPWM_GEN_ACTION_LOW)) return false;
    if (mcpwm_generator_set_force_level(_generator, 0, true) != ESP_OK) return false;
    if (mcpwm_generator_set_force_level(_generator, 0, false) != ESP_OK) return false;
    if (mcpwm_timer_set_period(_timer, periodTicks) != ESP_OK) return false;
    if (mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) return false;
    // EMPTY/CMP giữ LOW. Sync sắp tới là sự kiện duy nhất tạo cạnh lên đầu tiên.
    if (mcpwm_generator_set_force_level(_generator, -1, true) != ESP_OK) return false;
    _soTickChuKy = periodTicks;
    return true;
}

bool McpwmStepOutput::setFrequency(float hz) {
    if (!_daKhoiTao) return false;
    if (hz == 0.0f) return pauseAfterPulse();
    if (!supportsFrequency(hz)) { stop(); return false; }
    const uint32_t ticks = static_cast<uint32_t>(
        std::ceil(static_cast<double>(_dophangiaiHz) / hz));
    if (_dangChayLienTuc && !_daYeuCauTamDung) {
        if (ticks == _soTickChuKy) return true;
        if (mcpwm_timer_set_period(_timer, ticks) != ESP_OK) { stop(); return false; }
        _soTickChuKy = ticks;
        return true; // Có hiệu lực ở EMPTY, không làm ngắn HIGH hiện tại.
    }
    if (!prepareStart(ticks)) { stop(); return false; }
    if (mcpwm_soft_sync_activate(_sync) != ESP_OK) { stop(); return false; }
    if (!setEmptyAction(MCPWM_GEN_ACTION_HIGH)) { stop(); return false; }
    _dangChayLienTuc = true;
    _daYeuCauTamDung = false;
    return true;
}

bool McpwmStepOutput::emitOnePulse() {
    if (!_daKhoiTao || !isPauseReady()) return false;
    const uint32_t ticks = _soTickMucCao + _soTickMucThap;
    if (!prepareStart(ticks)) { stop(); return false; }
    setOneShotBusy(true);
    if (mcpwm_soft_sync_activate(_sync) != ESP_OK) {
        setOneShotBusy(false);
        stop();
        return false;
    }
    // EMPTY vẫn LOW: chỉ sync tạo đúng một HIGH, CMP kết thúc HIGH.
    // Nếu task bị preempt trước lệnh dừng, timer không tạo thêm cạnh lên.
    if (mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_STOP_FULL) != ESP_OK) {
        stop();
        return false;
    }
    _tamDungSanSang_us = nowUs() + _thoiGianChoYen_us;
    _daYeuCauTamDung = true;
    _dangChayLienTuc = false;
    return true;
}

bool McpwmStepOutput::stop() {
    if (!_daKhoiTao) return false;
    // Khẩn cấp, được phép cắt HIGH; lớp MKS đã đặt EN disable.
    const bool forced = mcpwm_generator_set_force_level(_generator, 0, true) == ESP_OK;
    const bool inhibited = setEmptyAction(MCPWM_GEN_ACTION_LOW);
    _dangChayLienTuc = false;
    _daYeuCauTamDung = true;
    _tamDungSanSang_us = nowUs() + _thoiGianChoYen_us;
    return forced && inhibited;
}

double McpwmStepOutput::actualFrequencyHz() const {
    return _dangChayLienTuc && _soTickChuKy ? static_cast<double>(_dophangiaiHz) / _soTickChuKy : 0.0;
}
