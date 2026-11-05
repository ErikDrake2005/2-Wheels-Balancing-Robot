#pragma once
#include "config/ControlConfig.h"
#include "LocThongThap.h"
struct PidGains { float heSoTiLe, heSoTichPhan, heSoViPhan; };
class AnglePidController {
public:
    explicit AnglePidController(PidGains heSoCoSo = {kpGoc, kiGoc, kdGoc}) : _heSoCoSo(heSoCoSo) {}
    void reset() { _tichPhanGoc_rad_s = 0.0f; _heSoPid = {}; _locGyroD.datLai(); }
    float update(float gocNghien_rad, float tocDoNghien_radps, float gocMucTieu_rad, float buocThoiGian_s, bool dungPidMo,
                 int huongBaoHoaVanToc = 0); // -1: trần âm, 0: chưa bão hòa, +1: trần dương.
    PidGains heSoPid() const { return _heSoPid; }
    float tocDoDaLoc_radps() const { return _locGyroD.giaTri(); }
    static PidGains scheduledGains(float saiLechGoc_rad, float tocDoNghien_radps, bool dungPidMo,
                                 PidGains heSoCoSo = {kpGoc, kiGoc, kdGoc});
private:
    float _tichPhanGoc_rad_s = 0.0f;
    PidGains _heSoPid{};
    PidGains _heSoCoSo;
    LocThongThap _locGyroD;
};
