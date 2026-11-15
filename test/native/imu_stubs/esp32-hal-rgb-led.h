#pragma once
#include "Arduino.h"
constexpr int LED_COLOR_ORDER_GRB = 0, LED_COLOR_ORDER_RGB = 1;
inline std::function<void(uint8_t, uint8_t, uint8_t)> doiMauGia;
inline void rgbLedWriteOrdered(int, int, uint8_t doLed, uint8_t xanhLa, uint8_t xanhDuong) {
    if (doiMauGia) doiMauGia(doLed, xanhLa, xanhDuong);
}
