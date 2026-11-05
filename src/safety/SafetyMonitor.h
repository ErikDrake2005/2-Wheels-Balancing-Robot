#pragma once
#include "model/RobotState.h"
#include "model/FaultCode.h"
class SafetyMonitor {
public:
    static FaultCode check(const TrangThaiXe& trangThaiXe, uint64_t thoiDiemHienTai_us);
    static bool canArm(const TrangThaiXe& trangThaiXe, uint64_t thoiDiemHienTai_us);
};
