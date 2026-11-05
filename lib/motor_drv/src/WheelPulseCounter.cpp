#include "WheelPulseCounter.h"
#include "config/MotorConfig.h"
#include "config/PinConfig.h"
#include "driver/gpio.h"
#include "esp_timer.h"

bool WheelPulseCounter::beginAxis(Axis& trucBanh, int chanXung, int chanChieu, bool mucTienCao) {
    pcnt_unit_config_t cauHinhBoDem{};
    cauHinhBoDem.low_limit = -30000;
    cauHinhBoDem.high_limit = 30000;
    cauHinhBoDem.flags.accum_count = true;
    if (pcnt_new_unit(&cauHinhBoDem, &trucBanh.boDem) != ESP_OK) return false;
    pcnt_chan_config_t cauHinhKenhDem{};
    cauHinhKenhDem.edge_gpio_num = chanXung;
    cauHinhKenhDem.level_gpio_num = chanChieu;
    // Giữ output đã cấu hình bởi MCPWM/GPIO và bật thêm input path.
    if (pcnt_new_channel(trucBanh.boDem, &cauHinhKenhDem, &trucBanh.kenhDem) != ESP_OK ||
        gpio_input_enable(static_cast<gpio_num_t>(chanXung)) != ESP_OK ||
        gpio_input_enable(static_cast<gpio_num_t>(chanChieu)) != ESP_OK ||
        pcnt_channel_set_edge_action(trucBanh.kenhDem, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                    PCNT_CHANNEL_EDGE_ACTION_HOLD) != ESP_OK) return false;
    const auto giuChieuDem = PCNT_CHANNEL_LEVEL_ACTION_KEEP;
    const auto daoChieuDem = PCNT_CHANNEL_LEVEL_ACTION_INVERSE;
    if (pcnt_channel_set_level_action(trucBanh.kenhDem, mucTienCao ? giuChieuDem : daoChieuDem,
                                     mucTienCao ? daoChieuDem : giuChieuDem) != ESP_OK ||
        pcnt_unit_add_watch_point(trucBanh.boDem, cauHinhBoDem.high_limit) != ESP_OK ||
        pcnt_unit_add_watch_point(trucBanh.boDem, cauHinhBoDem.low_limit) != ESP_OK ||
        pcnt_unit_enable(trucBanh.boDem) != ESP_OK ||
        pcnt_unit_clear_count(trucBanh.boDem) != ESP_OK ||
        pcnt_unit_start(trucBanh.boDem) != ESP_OK) return false;
    return true;
}

bool WheelPulseCounter::begin() {
    if (_daSanSang || _banhTrai.boDem || _banhPhai.boDem) return false;
    _daSanSang = beginAxis(_banhTrai, chanStepTrai, chanDirTrai, mucDirTienBanhTraiCao) &&
             beginAxis(_banhPhai, chanStepPhai, chanDirPhai, mucDirTienBanhPhaiCao);
    return _daSanSang; // Khởi tạo lỗi: giữ motor disabled, reset sau khi sửa.
}

bool WheelPulseCounter::readAxis(Axis& trucBanh) {
    int soDemDocDuoc = 0;
    if (pcnt_unit_get_count(trucBanh.boDem, &soDemDocDuoc) != ESP_OK) return false;
    const uint32_t soDemHienTai = static_cast<uint32_t>(soDemDocDuoc);
    const uint32_t chenhLechSoDem = soDemHienTai - trucBanh.soDemTruoc;
    trucBanh.tongSoXung += chenhLechSoDem < 0x80000000u ? static_cast<int64_t>(chenhLechSoDem)
                                 : static_cast<int64_t>(chenhLechSoDem) - 0x100000000LL;
    trucBanh.soDemTruoc = soDemHienTai;
    return true;
}

WheelPulseFrame WheelPulseCounter::read() {
    WheelPulseFrame khungDuLieu{};
    if (!_daSanSang || !readAxis(_banhTrai) || !readAxis(_banhPhai)) return khungDuLieu;
    khungDuLieu.trai = _banhTrai.tongSoXung;
    khungDuLieu.phai = _banhPhai.tongSoXung;
    khungDuLieu.thoiDiem_us = static_cast<uint64_t>(esp_timer_get_time());
    khungDuLieu.hopLe = true;
    return khungDuLieu;
}
