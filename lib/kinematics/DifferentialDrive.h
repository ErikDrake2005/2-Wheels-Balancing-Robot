#pragma once
#include "model/WheelPulseFrame.h"
#include "model/RobotState.h"
#include "config/MotorConfig.h"

class DifferentialDrive {
public:
    static double metersPerStep();
    static double metersPerEncoderCount();
    static double metersPerFeedbackPulse(NguonUocLuongQuangDuong nguon);
    static float stepsPerSecond(float vanToc_mps);
    bool update(const WheelPulseFrame& khungDuLieu, TrangThaiXe& trangThaiXe,
                NguonUocLuongQuangDuong nguon = dungPhanCungTmcMt
                    ? NguonUocLuongQuangDuong::EncoderBanh
                    : NguonUocLuongQuangDuong::XungBuoc);
    void reset() {
        _khungXungTruoc = {};
        _vanTocDaLoc_mps = 0.0f;
        _vanTocTraiDaLoc_mps = 0.0f;
        _vanTocPhaiDaLoc_mps = 0.0f;
    }
private:
    WheelPulseFrame _khungXungTruoc{};
    float _vanTocDaLoc_mps = 0.0f;
    float _vanTocTraiDaLoc_mps = 0.0f;
    float _vanTocPhaiDaLoc_mps = 0.0f;
};
