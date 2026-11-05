#pragma once
#include "AnglePidController.h"
#include "model/RobotState.h"
#include "model/WheelStepCommand.h"

class CascadeController {
public:
    void reset();
    bool update(const TrangThaiXe& trangThaiXe, uint64_t thoiDiemHienTai_us, bool dungPidMo,
                WheelStepCommand& lenhBanh, float vanTocYeuCau_mps = vanTocDat_mps,
                float tocDoQuayYeuCau_radps = 0);
    float gocDat_rad() const { return _gocDat_rad; }
    PidGains heSoPid() const { return _boPidGoc.heSoPid(); }
    float tocDoDaLoc_radps() const { return _boPidGoc.tocDoDaLoc_radps(); }
    float vanTocMucTieu_mps() const { return _vanTocMucTieu_mps; }
    float tocDoQuayMucTieu_radps() const { return _tocDoQuayMucTieu_radps; }
private:
    AnglePidController _boPidGoc;
    uint64_t _thoiDiemTruoc_us = 0;
    float _thoiGianVongVanToc_s = 0, _tichPhanVanToc_m = 0, _gocDat_rad = 0, _vanTocLenh_mps = 0;
    float _vanTocMucTieu_mps = 0, _tocDoQuayMucTieu_radps = 0;
};
