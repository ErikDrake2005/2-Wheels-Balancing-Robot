#pragma once
#include "soc/gpio_struct.h"
#include <cstdint>
inline void gpio_ll_set_level(gpio_dev_t* phanCung,uint32_t chan,uint32_t muc) {
    phanCung->mucChan[chan]=muc;
}
