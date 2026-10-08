#include "McpwmStepOutput.h"
#include "config/MotorConfig.h"
#include "driver/gpio.h"
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
    if (_initialized || _timer || _operator ||  _comparator || _generator)  return false;
    // Lấy cấu hình TRƯỚC khi dùng nó để đổi thời gian sang tick.
    _dophangiaiHz = MCPWM_TIMER_RESOLUTION_HZ;
    if (_dophangiaiHz == 0 || pulseHighUs == 0 || minLowUs == 0) return false;
    const uint64_t highTicks = doiUsSangTicks(pulseHighUs, _dophangiaiHz);
    const uint64_t lowTicks = doiUsSangTicks(minLowUs, _dophangiaiHz);
    const uint64_t initialPeriodTicks = doiUsSangTicks(MCPWM_INITIAL_PERIOD_US, _dophangiaiHz);
    if (highTicks + lowTicks > initialPeriodTicks || initialPeriodTicks > UINT32_MAX) return false;
    // STEP phải dùng chân có khả năng xuất tín hiệu.
    if (stepPin < 0 || !GPIO_IS_VALID_OUTPUT_GPIO(stepPin)) return false;
    if (pulseHighUs == 0 || minLowUs == 0) return false;
    // Phiên bản đầu dùng 1 tick = 1 microsecond.
    _dophangiaiHz = MCPWM_TIMER_RESOLUTION_HZ;
    // Độ phân giải bằng 0 không có ý nghĩa.
    if (_dophangiaiHz == 0) return false;
    // Chu kỳ phải đủ chứa phần HIGH và thời gian LOW tối thiểu.
    if (static_cast<uint64_t>(pulseHighUs) + minLowUs > initialPeriodTicks) return false;
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
    _initialized = true;
    return true;
}
