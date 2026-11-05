#pragma once
#include "driver/pulse_cnt.h"
#include "model/WheelPulseFrame.h"

// Khởi tạo SAU MCPWM khi motor disabled; đọc bởi duy nhất MotorTask.
// PCNT quan sát chính STEP/DIR qua đường input của GPIO, không cần dây thêm.
class WheelPulseCounter {
public:
    bool begin();
    WheelPulseFrame read();
private:
    struct Axis {
        pcnt_unit_handle_t boDem = nullptr;
        pcnt_channel_handle_t kenhDem = nullptr;
        uint32_t soDemTruoc = 0;
        int64_t tongSoXung = 0;
    };
    bool beginAxis(Axis& trucBanh, int chanXung, int chanChieu, bool mucTienCao);
    bool readAxis(Axis& trucBanh);
    Axis _banhTrai, _banhPhai;
    bool _daSanSang = false;
};
