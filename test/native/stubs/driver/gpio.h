#pragma once
#include "driver/gptimer.h"
using gpio_num_t = int;
constexpr int GPIO_MODE_OUTPUT = 1;
#define GPIO_IS_VALID_OUTPUT_GPIO(chan) ((chan)>=0 && (chan)<49)
esp_err_t gpio_set_level(gpio_num_t,uint32_t);
esp_err_t gpio_set_direction(gpio_num_t,int);
