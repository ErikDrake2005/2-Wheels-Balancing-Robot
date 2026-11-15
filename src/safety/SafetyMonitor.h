#pragma once
#include "model/RobotState.h"
#include "model/FaultCode.h"
#include "config/ControlConfig.h"
class SafetyMonitor {
public:
    static FaultCode check(const TrangThaiXe& trangThaiXe, uint64_t thoiDiemHienTai_us,
                           float gocThamChieu_rad = gocCanBang_rad);
    static bool canArm(const TrangThaiXe& trangThaiXe, uint64_t thoiDiemHienTai_us,
                       float gocThamChieu_rad = gocCanBang_rad);
};
