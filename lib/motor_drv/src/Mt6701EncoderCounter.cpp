#include "Mt6701EncoderCounter.h"
#include "config/MotorConfig.h"
#include "config/PinConfig.h"
#include "driver/gpio.h"
#include "esp_timer.h"

bool Mt6701EncoderCounter::beginAxis(Axis& trucBanh, int chanA, int chanB, bool chieuTienDuong) {
    if (chanA < 0 || chanB < 0 || chanA == chanB ||
        !GPIO_IS_VALID_GPIO(chanA) || !GPIO_IS_VALID_GPIO(chanB)) return false;

    const auto gpioA = static_cast<gpio_num_t>(chanA);
    const auto gpioB = static_cast<gpio_num_t>(chanB);
    if (gpio_set_direction(gpioA, GPIO_MODE_INPUT) != ESP_OK ||
        gpio_set_direction(gpioB, GPIO_MODE_INPUT) != ESP_OK ||
        gpio_pullup_en(gpioA) != ESP_OK || gpio_pulldown_dis(gpioA) != ESP_OK ||
        gpio_pullup_en(gpioB) != ESP_OK || gpio_pulldown_dis(gpioB) != ESP_OK) return false;

    pcnt_unit_config_t cauHinhBoDem{};
    cauHinhBoDem.low_limit = -30000;
    cauHinhBoDem.high_limit = 30000;
    cauHinhBoDem.flags.accum_count = true;
    if (pcnt_new_unit(&cauHinhBoDem, &trucBanh.boDem) != ESP_OK) return false;

    if (thoiGianLocNhieuEncoder_ns > 0) {
        pcnt_glitch_filter_config_t cauHinhLocNhieu{};
        cauHinhLocNhieu.max_glitch_ns = thoiGianLocNhieuEncoder_ns;
        if (pcnt_unit_set_glitch_filter(trucBanh.boDem, &cauHinhLocNhieu) != ESP_OK) return false;
    }

    // Hoán đổi kênh A/B khi cần đảo chiều đếm dương theo chiều tiến của xe.
    const int chanKenh0 = chieuTienDuong ? chanA : chanB;
    const int chanKenh1 = chieuTienDuong ? chanB : chanA;

    pcnt_chan_config_t cauHinhKenhA{};
    cauHinhKenhA.edge_gpio_num = chanKenh0;
    cauHinhKenhA.level_gpio_num = chanKenh1;
    pcnt_chan_config_t cauHinhKenhB{};
    cauHinhKenhB.edge_gpio_num = chanKenh1;
    cauHinhKenhB.level_gpio_num = chanKenh0;

    // Giải mã quadrature 4x chuẩn: đếm cả 2 cạnh của cả 2 kênh A và B.
    if (pcnt_new_channel(trucBanh.boDem, &cauHinhKenhA, &trucBanh.kenhA) != ESP_OK ||
        pcnt_new_channel(trucBanh.boDem, &cauHinhKenhB, &trucBanh.kenhB) != ESP_OK ||
        pcnt_channel_set_edge_action(trucBanh.kenhA,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE) != ESP_OK ||
        pcnt_channel_set_level_action(trucBanh.kenhA,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK ||
        pcnt_channel_set_edge_action(trucBanh.kenhB,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE) != ESP_OK ||
        pcnt_channel_set_level_action(trucBanh.kenhB,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK ||
        pcnt_unit_add_watch_point(trucBanh.boDem, cauHinhBoDem.high_limit) != ESP_OK ||
        pcnt_unit_add_watch_point(trucBanh.boDem, cauHinhBoDem.low_limit) != ESP_OK ||
        pcnt_unit_enable(trucBanh.boDem) != ESP_OK ||
        pcnt_unit_clear_count(trucBanh.boDem) != ESP_OK ||
        pcnt_unit_start(trucBanh.boDem) != ESP_OK) return false;

    return true;
}

bool Mt6701EncoderCounter::begin() {
    if (_daSanSang || _banhTrai.boDem || _banhPhai.boDem) return false;
    _daSanSang = beginAxis(_banhTrai, chanEncoderTraiA, chanEncoderTraiB, chieuTienEncoderTraiDuong) &&
                 beginAxis(_banhPhai, chanEncoderPhaiA, chanEncoderPhaiB, chieuTienEncoderPhaiDuong);
    if (!_daSanSang) return false;
    const WheelPulseFrame khungKiemTra = read();
    _daSanSang = khungKiemTra.hopLe;
    return _daSanSang;
}

void Mt6701EncoderCounter::reset() {
    if (!_daSanSang) return;
    if (_banhTrai.boDem) pcnt_unit_clear_count(_banhTrai.boDem);
    if (_banhPhai.boDem) pcnt_unit_clear_count(_banhPhai.boDem);
    _banhTrai.soDemTruoc = 0;
    _banhTrai.tongSoXung = 0;
    _banhPhai.soDemTruoc = 0;
    _banhPhai.tongSoXung = 0;
}

bool Mt6701EncoderCounter::readAxis(Axis& trucBanh) {
    int soDemDocDuoc = 0;
    if (pcnt_unit_get_count(trucBanh.boDem, &soDemDocDuoc) != ESP_OK) return false;
    const uint32_t soDemHienTai = static_cast<uint32_t>(soDemDocDuoc);
    const uint32_t chenhLechSoDem = soDemHienTai - trucBanh.soDemTruoc;
    trucBanh.tongSoXung += chenhLechSoDem < 0x80000000u
        ? static_cast<int64_t>(chenhLechSoDem)
        : static_cast<int64_t>(chenhLechSoDem) - 0x100000000LL;
    trucBanh.soDemTruoc = soDemHienTai;
    return true;
}

WheelPulseFrame Mt6701EncoderCounter::read() {
    WheelPulseFrame khungDuLieu{};
    if (!_daSanSang || !readAxis(_banhTrai) || !readAxis(_banhPhai)) return khungDuLieu;
    khungDuLieu.trai = _banhTrai.tongSoXung;
    khungDuLieu.phai = _banhPhai.tongSoXung;
    khungDuLieu.thoiDiem_us = static_cast<uint64_t>(esp_timer_get_time());
    khungDuLieu.hopLe = true;
    return khungDuLieu;
}
