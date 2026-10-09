#ifndef MCPWM_STEP_OUTPUT_H
#define MCPWM_STEP_OUTPUT_H
#include <cstdint>
#include "esp_attr.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"

// ĐỀ XUẤT V2. Giữ tên begin/setFrequency/stop để đối chiếu với bản hiện tại.
// Một task gọi API của một đối tượng; callback on_stop chỉ đổi cờ có bảo vệ.
// Đối tượng phải tồn tại suốt thời gian phần cứng/callback đang hoạt động.
class McpwmStepOutput {
public:
    McpwmStepOutput() = default;
    McpwmStepOutput(const McpwmStepOutput&) = delete;
    McpwmStepOutput& operator=(const McpwmStepOutput&) = delete;

    bool begin(int stepPin, uint32_t pulseHighUs, uint32_t minLowUs);
    bool setFrequency(float frequencyHz);

    // Bình thường: ngăn cạnh STEP lên tiếp theo, để HIGH hiện tại kết thúc.
    // Không chờ trong hàm. Kiểm tra isPauseReady() trước khi đổi DIR/phát lại.
    bool pauseAfterPulse();
    bool isPauseReady() const;

    // Phát đúng một xung, dùng cho lệnh nhỏ hơn dải xung liên tục.
    // Generator HIGH ở sync, LOW ở comparator; EMPTY được giữ LOW.
    // Timer dừng ở cuối chu kỳ. on_stop chỉ chạy khi dừng, không ở mọi STEP.
    bool emitOnePulse();

    // Khẩn cấp: ép LOW tức thì, có thể cắt xung HIGH. Lớp MKS tắt EN trước.
    bool stop();
    bool supportsFrequency(float hz) const;
    double actualFrequencyHz() const;

private:
    static bool IRAM_ATTR onTimerStop(
        mcpwm_timer_handle_t, const mcpwm_timer_event_data_t*, void* context);
    bool setEmptyAction(mcpwm_generator_action_t action);
    bool prepareStart(uint32_t periodTicks);
    bool cleanupInit();
    bool oneShotBusy() const;
    void setOneShotBusy(bool busy);

    mcpwm_timer_handle_t _timer = nullptr;
    mcpwm_oper_handle_t _operator = nullptr;
    mcpwm_cmpr_handle_t _comparator = nullptr;
    mcpwm_gen_handle_t _generator = nullptr;
    mcpwm_sync_handle_t _sync = nullptr;

    // Bảo vệ duy nhất cờ giao tiếp giữa ISR và task; không giữ khóa khi gọi MCPWM.
    mutable portMUX_TYPE _pulseMux = portMUX_INITIALIZER_UNLOCKED;
    bool _oneShotBusy = false;
    bool _initialized = false;
    bool _continuous = false;
    bool _pauseRequested = true;
    bool _timerEnabled = false;
    uint32_t _dophangiaiHz = 0;
    uint32_t _highTicks = 0;
    uint32_t _lowTicks = 0;
    uint32_t _periodTicks = 0;
    uint32_t _quietWaitUs = 0;
    uint64_t _pauseReadyAtUs = 0;
};
#endif
