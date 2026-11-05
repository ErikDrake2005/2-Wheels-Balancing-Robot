#pragma once
#include "model/WheelPulseFrame.h"
#include "model/RobotState.h"

class DifferentialDrive {
public:
    static double metersPerStep();
    static float stepsPerSecond(float vanToc_mps);
    bool update(const WheelPulseFrame& khungDuLieu, TrangThaiXe& trangThaiXe);
private:
    WheelPulseFrame _khungXungTruoc{};
    float _vanTocDaLoc_mps = 0.0f;
};
