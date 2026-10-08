#ifndef MCPWM_STEP_OUTPUT_H
#define MCPWM_STEP_OUTPUT_H

#include <cstdint>
#include "driver/mcpwm_prelude.h"

class McpwmStepOutput {
public:
    // Chuẩn bị phần cứng cho MỘT chân STEP.
    //
    // stepPin: GPIO nối STEP của MKS.
    // pulseHighUs: độ rộng phần HIGH của mỗi xung.
    // minLowUs: thời gian LOW tối thiểu giữa các xung.
    //
    // Sau khi thành công, đầu ra vẫn LOW, chưa phát xung.
    bool begin(int stepPin, uint32_t pulseHighUs, uint32_t minLowUs);

    // Đặt tần số xung, đơn vị Hz = xung/giây.
    //
    // frequencyHz > 0: yêu cầu phát xung.
    // frequencyHz = 0: yêu cầu ngừng phát xung.
    // Từ chối giá trị âm, NaN hoặc vượt khả năng phần cứng.
    //
    // Chiều quay DIR do MksServo42CPair quản lý.
    bool setFrequency(float frequencyHz);

    // Ép STEP về LOW để ngừng phát ngay.
    // Có thể gọi nhiều lần.
    // Không dùng riêng hàm này để đảm bảo đảo chiều đúng timing;
    // trình tự DIR hold/setup sẽ thuộc lớp MKS.
    bool stop();
    McpwmStepOutput() = default;
    // Mỗi đối tượng sở hữu riêng tài nguyên MCPWM.
    // Ngăn sao chép để tránh hai đối tượng cùng quản lý một timer.
    McpwmStepOutput(const McpwmStepOutput&) = delete;
    McpwmStepOutput& operator=(const McpwmStepOutput&) = delete;
private:
    // Handle là biến tham chiếu tới tài nguyên phần cứng
    // do driver MCPWM tạo khi begin() chạy.
    mcpwm_timer_handle_t _timer = nullptr;
    mcpwm_oper_handle_t _operator = nullptr;
    mcpwm_cmpr_handle_t _comparator = nullptr;
    mcpwm_gen_handle_t _generator = nullptr;
    bool _initialized = false;
    uint32_t _pulseHighUs = 0;
    uint32_t _minLowUs = 0;
    uint32_t _dophangiaiHz = 1000000;
    // Timer đã được gửi lệnh bắt đầu đếm hay chưa.
    bool _timerStarted = false;
    // STEP đang bị giữ LOW hay đang phát xung.
    bool _outputStopped = true;
    // Chu kỳ gần nhất được yêu cầu, dùng tránh ghi lặp.
    uint32_t _periodTicks = 0;

};

#endif