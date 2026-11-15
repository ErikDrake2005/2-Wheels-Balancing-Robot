#pragma once
using gpio_num_t = int;
constexpr int ESP_OK = 0, GPIO_MODE_OUTPUT = 1;
inline int mucGpioGia[64]{};
inline int gpio_set_level(gpio_num_t chan, int muc) { mucGpioGia[chan] = muc; return ESP_OK; }
inline int gpio_set_direction(gpio_num_t, int) { return ESP_OK; }
