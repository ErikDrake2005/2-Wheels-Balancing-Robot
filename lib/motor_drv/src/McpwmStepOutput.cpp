#include "McpwmStepOutput.h"
#include "config/MotorConfig.h"
#include "driver/gpio.h"
#include "hal/mcpwm_ll.h"
#include <cmath>
namespace {
    uint64_t doiUsSangTicks(uint32_t thoiGianUs, uint32_t resolutionHz) {
        // Số tick = thời gian × số tick trong một giây.
        // Chia 1.000.000 vì đầu vào là microsecond.
        //
        // Cộng 999999 trước khi chia để làm tròn lên:
        // thời gian xung thực tế không ngắn hơn yêu cầu.
        return (static_cast<uint64_t>(thoiGianUs) * resolutionHz + 999999ULL) / 1000000ULL;
    }
}
bool McpwmStepOutput::begin( int stepPin, uint32_t pulseHighUs, uint32_t minLowUs) {
    // Không cấp phát lại tài nguyên đang tồn tại.

    if (_initialized || _timer || _operator || _comparator || _generator) return false;
    // Lấy độ phân giải từ Config.
    _dophangiaiHz = MCPWM_TIMER_RESOLUTION_HZ;
    if (_dophangiaiHz == 0 || pulseHighUs == 0 || minLowUs == 0) return false;
    // Chỉ dùng GPIO có khả năng xuất tín hiệu.
    if (stepPin < 0 || !GPIO_IS_VALID_OUTPUT_GPIO(stepPin)) return false;
    // Chuyển mọi thời gian về cùng đơn vị tick.
    const uint64_t highTicks = doiUsSangTicks(pulseHighUs, _dophangiaiHz);
    const uint64_t lowTicks = doiUsSangTicks(minLowUs, _dophangiaiHz);
    const uint64_t initialPeriodTicks = doiUsSangTicks(MCPWM_INITIAL_PERIOD_US, _dophangiaiHz);

    // Chu kỳ phải chứa đủ HIGH và LOW.
    // API cập nhật chu kỳ yêu cầu số tick NHỎ HƠN giới hạn này.
    if (highTicks == 0 || lowTicks == 0 || highTicks + lowTicks > initialPeriodTicks || initialPeriodTicks >= MCPWM_LL_MAX_COUNT_VALUE) return false;
    const gpio_num_t gpio = static_cast<gpio_num_t>(stepPin);

    // Chuẩn bị GPIO ở LOW trước khi MCPWM tiếp quản.
    if (gpio_set_level(gpio, 0) != ESP_OK ||gpio_set_direction(gpio, GPIO_MODE_OUTPUT) != ESP_OK) return false;

    // Hàm nhỏ dùng khi khởi tạo thất bại giữa chừng.
    // Xóa tài nguyên theo thứ tự ngược lại với lúc tạo.
    // Nếu xóa thất bại, giữ handle để không cấp phát trùng.
    auto huyKhoiTao = [this]() -> bool {
        if (_generator &&
            mcpwm_del_generator(_generator) == ESP_OK) {
            _generator = nullptr;
        }

        if (_comparator &&
            mcpwm_del_comparator(_comparator) == ESP_OK) {
            _comparator = nullptr;
        }

        if (_operator &&
            mcpwm_del_operator(_operator) == ESP_OK) {
            _operator = nullptr;
        }

        if (_timer &&
            mcpwm_del_timer(_timer) == ESP_OK) {
            _timer = nullptr;
        }

        return false;
    };

    // A. Tạo timer: đếm tăng từ 0 tới giới hạn chu kỳ.
    // Tạo một biến cấu hình timer, các trường ban đầu bằng 0.
    mcpwm_timer_config_t timerConfig{};
    timerConfig.group_id = 0;
    timerConfig.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    // Độ phân giải đã lấy từ MotorConfig.h.
    timerConfig.resolution_hz = _dophangiaiHz;
    timerConfig.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    // Chu kỳ đã đổi từ microsecond sang tick.
    timerConfig.period_ticks = static_cast<uint32_t>(initialPeriodTicks);
    timerConfig.flags.update_period_on_empty = true;
    if (mcpwm_new_timer(&timerConfig, &_timer) != ESP_OK) return huyKhoiTao();

    // B. Tạo operator và nối nó với timer.
    mcpwm_operator_config_t operatorConfig{};
    operatorConfig.group_id = 0;
    if (mcpwm_new_operator(&operatorConfig, &_operator) != ESP_OK) return huyKhoiTao();
    if (mcpwm_operator_connect_timer(_operator, _timer) != ESP_OK) return huyKhoiTao();

    // C. Comparator xác định lúc kết thúc phần HIGH.
    mcpwm_comparator_config_t comparatorConfig{};
    comparatorConfig.flags.update_cmp_on_tez = true;
    if (mcpwm_new_comparator(_operator, &comparatorConfig, &_comparator) != ESP_OK) return huyKhoiTao();

    // 1 tick = 1 us nên số tick HIGH bằng pulseHighUs.
    if (mcpwm_comparator_set_compare_value(_comparator,static_cast<uint32_t>(highTicks)) != ESP_OK) return huyKhoiTao();

    // D. Generator xuất tín hiệu ra chân STEP.
    mcpwm_generator_config_t generatorConfig{};
    generatorConfig.gen_gpio_num = stepPin;

    if (mcpwm_new_generator(_operator, &generatorConfig, &_generator) != ESP_OK) return huyKhoiTao();

    // Ép STEP LOW và giữ mức đó.
    // Mọi sự kiện tạo sóng bên dưới tạm thời bị ghi đè.
    if (mcpwm_generator_set_force_level(_generator, 0, true) != ESP_OK) return huyKhoiTao();


    // Khi timer về đầu chu kỳ: STEP lên HIGH.
    if (mcpwm_generator_set_action_on_timer_event(
            _generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH
            )) != ESP_OK) {
        return huyKhoiTao();
    }

    // Khi timer đạt giá trị comparator: STEP xuống LOW.
    if (mcpwm_generator_set_action_on_compare_event(
            _generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                _comparator,
                MCPWM_GEN_ACTION_LOW
            )) != ESP_OK) {
        return huyKhoiTao();
    }

    // Enable tài nguyên timer, nhưng CHƯA bắt đầu đếm.
    if (mcpwm_timer_enable(_timer) != ESP_OK) return huyKhoiTao();
    _pulseHighUs = pulseHighUs;
    _minLowUs = minLowUs;
    _timerStarted = false;
    _outputStopped = true;
    _periodTicks = static_cast<uint32_t>(initialPeriodTicks);
    _initialized = true;
    return true;
}
bool McpwmStepOutput::stop() {
    if (!_initialized || _generator == nullptr) return false;

    // Giữ STEP LOW ngay lập tức.
    // Timer vẫn có thể tiếp tục đếm ở bên trong.
    if (mcpwm_generator_set_force_level(_generator, 0, true) != ESP_OK) return false;
    _outputStopped = true;
    return true;
}
bool McpwmStepOutput::setFrequency(float frequencyHz) {
    if (!_initialized ||
        _timer == nullptr ||
        _generator == nullptr) {
        return false;
    }

    // Không chấp nhận NaN, vô cực hoặc tần số âm.
    // Lớp MKS sẽ xử lý dấu tốc độ để chọn DIR.
    if (!std::isfinite(frequencyHz) || frequencyHz < 0.0f) {
        stop();
        return false;
    }

    // Tần số 0 có nghĩa là dừng đầu ra.
    if (frequencyHz == 0.0f) {
        return stop();
    }

    const uint64_t highTicks =
        doiUsSangTicks(_pulseHighUs, _dophangiaiHz);

    const uint64_t lowTicks =
        doiUsSangTicks(_minLowUs, _dophangiaiHz);

    // Chu kỳ = độ phân giải / tần số.
    // Làm tròn lên để tần số thực không cao hơn yêu cầu.
    const double ticksCan = std::ceil(
        static_cast<double>(_dophangiaiHz) / frequencyHz
    );

    // Kiểm tra trước khi chuyển về số nguyên.
    //
    // Quá ít tick: không đủ HIGH và LOW.
    // Quá nhiều tick: vượt phạm vi timer.
    if (!std::isfinite(ticksCan) ||
        ticksCan < static_cast<double>(highTicks + lowTicks) ||
        ticksCan >= static_cast<double>(MCPWM_LL_MAX_COUNT_VALUE)) {
        stop();
        return false;
    }

    const uint32_t periodTicks =
        static_cast<uint32_t>(ticksCan);

    // Đang chạy với cùng chu kỳ thì không cần cập nhật phần cứng.
    if (!_outputStopped && periodTicks == _periodTicks) {
        return true;
    }

    // begin() đã chọn update_period_on_empty:
    // chu kỳ mới có hiệu lực tại đầu một chu kỳ.
    if (mcpwm_timer_set_period(_timer, periodTicks) != ESP_OK) {
        stop();
        return false;
    }

    _periodTicks = periodTicks;

    // Nếu đang phát xung, việc cập nhật chu kỳ đã đủ.
    if (!_outputStopped) {
        return true;
    }

    // Từ đây là trình tự phát lần đầu hoặc phát lại sau stop().
    //
    // Tạm đổi sự kiện đầu chu kỳ thành LOW:
    // trong lúc chuẩn bị, không sự kiện nào được đưa STEP lên HIGH.
    if (mcpwm_generator_set_action_on_timer_event(
            _generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_LOW
            )) != ESP_OK) {
        stop();
        return false;
    }

    // Đưa trạng thái tạo sóng về LOW bằng ép mức tạm thời.
    // Ép LOW liên tục của stop() vẫn đang giữ đầu ra an toàn.
    if (mcpwm_generator_set_force_level(
            _generator, 0, false) != ESP_OK) {
        stop();
        return false;
    }

    // Chỉ bắt đầu timer một lần.
    // Sau stop(), timer vẫn đếm nên không cần bắt đầu lại.
    if (!_timerStarted) {
        if (mcpwm_timer_start_stop(
                _timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) {
            stop();
            return false;
        }

        _timerStarted = true;
    }

    // Bỏ chế độ ép LOW liên tục.
    // Các sự kiện hiện tại vẫn giữ đầu ra LOW.
    if (mcpwm_generator_set_force_level(
            _generator, -1, true) != ESP_OK) {
        stop();
        return false;
    }

    // Cho phép HIGH tại sự kiện đầu chu kỳ tiếp theo.
    // Comparator đã cấu hình sẽ kết thúc phần HIGH.
    if (mcpwm_generator_set_action_on_timer_event(
            _generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH
            )) != ESP_OK) {
        stop();
        return false;
    }

    _outputStopped = false;
    return true;
}
