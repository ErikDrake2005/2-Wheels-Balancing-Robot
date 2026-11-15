#pragma once
// Giao diện điều khiển cặp TMC2209 nhận xung STEP/DIR và chân ENN chung (GPIO2).
// Dùng chung bộ phát xung MCPWM phần cứng (liên tục >=1000 Hz / xung thưa <1000 Hz)
// và bộ giám sát độc lập GPTimer (GiamSatDongCo) với MksServo42CPair.
#include "MksServo42C.h"
using Tmc2209Pair = MksServo42CPair;
