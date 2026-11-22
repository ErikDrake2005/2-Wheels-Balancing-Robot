#pragma once
#include "driver/pulse_cnt.h"
#include "model/WheelPulseFrame.h"

// Giải mã hai encoder từ tính MT6701 ở chế độ AB quadrature 4x bằng phần cứng PCNT.
// Sơ đồ chân (PinConfig.h khi dungPhanCungTmcMt = true):
// - Encoder trái : A = GPIO8, B = GPIO9
// - Encoder phải : A = GPIO1, B = GPIO4
// Tương thích hoàn toàn với giao diện begin()/read() của WheelPulseCounter.
class Mt6701EncoderCounter {
public:
    bool begin();
    bool beginUartMks() { return true; }
    void datTamDungUart(bool) {}
    void reset();
    WheelPulseFrame read();
private:
    struct Axis {
        pcnt_unit_handle_t boDem = nullptr;
        pcnt_channel_handle_t kenhA = nullptr;
        pcnt_channel_handle_t kenhB = nullptr;
        uint32_t soDemTruoc = 0;
        int64_t tongSoXung = 0;
    };
    bool beginAxis(Axis& trucBanh, int chanA, int chanB, bool chieuTienDuong);
    bool readAxis(Axis& trucBanh);
    Axis _banhTrai, _banhPhai;
    bool _daSanSang = false;
};
