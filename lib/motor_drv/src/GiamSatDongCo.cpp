#include "GiamSatDongCo.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"

bool GiamSatDongCo::begin(int chanEn, bool mucBatCao,
                         uint32_t hanPhucVu_us, uint32_t chuKyKiemTra_us) {
    if (_boHen || chanEn < 0 || !GPIO_IS_VALID_OUTPUT_GPIO(chanEn) ||
        !chuKyKiemTra_us || chuKyKiemTra_us >= hanPhucVu_us) return false;
    _chanEn = chanEn;
    _mucBatCao = mucBatCao;
    _hanPhucVu_us = hanPhucVu_us;
    if (gpio_set_level(static_cast<gpio_num_t>(chanEn), mucBatCao ? 0 : 1) != ESP_OK ||
        gpio_set_direction(static_cast<gpio_num_t>(chanEn), GPIO_MODE_OUTPUT) != ESP_OK) return false;
    gptimer_config_t cauHinh{};
    cauHinh.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    cauHinh.direction = GPTIMER_COUNT_UP;
    cauHinh.resolution_hz = 1000000;
    cauHinh.intr_priority = 3;
    if (gptimer_new_timer(&cauHinh, &_boHen) != ESP_OK) return false;
    gptimer_event_callbacks_t suKien{};
    suKien.on_alarm = khiBaoThuc;
    gptimer_alarm_config_t baoThuc{};
    baoThuc.alarm_count = chuKyKiemTra_us;
    baoThuc.flags.auto_reload_on_alarm = true;
    if (gptimer_register_event_callbacks(_boHen, &suKien, this) != ESP_OK ||
        gptimer_set_alarm_action(_boHen, &baoThuc) != ESP_OK ||
        gptimer_enable(_boHen) != ESP_OK) {
        gptimer_del_timer(_boHen); _boHen = nullptr;
        return false;
    }
    if (gptimer_start(_boHen) != ESP_OK) {
        gptimer_disable(_boHen);
        gptimer_del_timer(_boHen); _boHen = nullptr;
        return false;
    }
    _daKhoiTao = true;
    return true;
}

bool GiamSatDongCo::quaHanTrongKhoa(uint64_t thoiDiem_us) const {
    return _dangBat && (thoiDiem_us >= _hanPhucVuTuyetDoi_us || thoiDiem_us >= _hanLenh_us);
}
void GiamSatDongCo::ngatTrongKhoa() {
    gpio_ll_set_level(&GPIO, _chanEn, _mucBatCao ? 0 : 1);
    _dangBat = false;
    _daNgat = true;
}
bool GiamSatDongCo::khiBaoThuc(gptimer_handle_t,
    const gptimer_alarm_event_data_t*, void* nguCanh) {
    auto* boGiamSat = static_cast<GiamSatDongCo*>(nguCanh);
    portENTER_CRITICAL_ISR(&boGiamSat->_khoa);
    if (boGiamSat->quaHanTrongKhoa(static_cast<uint64_t>(esp_timer_get_time())))
        boGiamSat->ngatTrongKhoa();
    portEXIT_CRITICAL_ISR(&boGiamSat->_khoa);
    return false;
}
bool GiamSatDongCo::bat(uint64_t hanLenh_us) {
    if (!_daKhoiTao) return false;
    portENTER_CRITICAL(&_khoa);
    const uint64_t thoiDiem_us = static_cast<uint64_t>(esp_timer_get_time());
    const bool choPhep = !_dangBat && !_daNgat && hanLenh_us > thoiDiem_us;
    if (choPhep) {
        _hanLenh_us = hanLenh_us;
        _hanPhucVuTuyetDoi_us = thoiDiem_us + _hanPhucVu_us;
        _dangBat = true;
        gpio_ll_set_level(&GPIO, _chanEn, _mucBatCao ? 1 : 0);
    }
    portEXIT_CRITICAL(&_khoa);
    return choPhep;
}
bool GiamSatDongCo::giaHan(uint64_t hanLenh_us) {
    if (!_daKhoiTao) return false;
    portENTER_CRITICAL(&_khoa);
    const uint64_t thoiDiem_us = static_cast<uint64_t>(esp_timer_get_time());
    if (quaHanTrongKhoa(thoiDiem_us) || (_dangBat && hanLenh_us <= thoiDiem_us))
        ngatTrongKhoa();
    const bool choPhep = _dangBat && !_daNgat;
    if (choPhep) {
        _hanLenh_us = hanLenh_us; // Giữ deadline lệnh gốc, không gia hạn bằng now + timeout.
        _hanPhucVuTuyetDoi_us = thoiDiem_us + _hanPhucVu_us;
    }
    portEXIT_CRITICAL(&_khoa);
    return choPhep;
}
bool GiamSatDongCo::tat() {
    if (!_daKhoiTao) return false;
    portENTER_CRITICAL(&_khoa);
    gpio_ll_set_level(&GPIO, _chanEn, _mucBatCao ? 0 : 1);
    _dangBat = false; // Không xóa lỗi đã giữ lại.
    portEXIT_CRITICAL(&_khoa);
    return true;
}
bool GiamSatDongCo::xoaLoi() {
    if (!_daKhoiTao) return false;
    portENTER_CRITICAL(&_khoa);
    const bool choPhep = !_dangBat;
    if (choPhep) _daNgat = false;
    portEXIT_CRITICAL(&_khoa);
    return choPhep;
}
bool GiamSatDongCo::daNgat() const {
    portENTER_CRITICAL(&_khoa);
    const bool daNgat = _daNgat;
    portEXIT_CRITICAL(&_khoa);
    return daNgat;
}
