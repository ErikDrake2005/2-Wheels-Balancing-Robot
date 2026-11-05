#pragma once
#include <cstdint>
#include "esp_attr.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"

// Đối tượng tồn tại suốt chương trình. Chỉ lớp này được bật EN sau khi begin().
// ISR chỉ tắt EN và giữ cờ lỗi; MotorTask dọn STEP/trạng thái khi chạy lại.
class GiamSatDongCo {
public:
    bool begin(int chanEn, bool mucBatCao, uint32_t hanPhucVu_us, uint32_t chuKyKiemTra_us);
    bool bat(uint64_t hanLenh_us);
    bool giaHan(uint64_t hanLenh_us);
    bool tat();
    bool xoaLoi();
    bool daNgat() const;
    bool daKhoiTao() const { return _daKhoiTao; }
private:
    static bool IRAM_ATTR khiBaoThuc(gptimer_handle_t,
        const gptimer_alarm_event_data_t*, void* nguCanh);
    void IRAM_ATTR ngatTrongKhoa();
    bool IRAM_ATTR quaHanTrongKhoa(uint64_t thoiDiem_us) const;
    mutable portMUX_TYPE _khoa = portMUX_INITIALIZER_UNLOCKED;
    gptimer_handle_t _boHen = nullptr;
    int _chanEn = -1;
    bool _mucBatCao = false, _daKhoiTao = false, _dangBat = false, _daNgat = false;
    uint32_t _hanPhucVu_us = 0;
    uint64_t _hanPhucVuTuyetDoi_us = 0, _hanLenh_us = 0;
};
