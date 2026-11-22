#pragma once
#include "config/ControlConfig.h"
#include "LocThongThap.h"
#if CAU_HINH_IMU == IMU_MPU6050_I2C
#include "LocBiquad.h"
#endif
struct PidGains { float heSoTiLe, heSoTichPhan, heSoViPhan; };
class AnglePidController {
public:
    AnglePidController()
        : _heSoCoSo{kpGoc, kiGoc, kdGoc},
          _heSoPidThuan{kpGocPidThuan, kiGocPidThuan, kdGocPidThuan} {}
    explicit AnglePidController(PidGains heSoCoSo)
        : _heSoCoSo(heSoCoSo), _heSoPidThuan(heSoCoSo) {}
    AnglePidController(PidGains heSoCoSoMo, PidGains heSoPidThuan)
        : _heSoCoSo(heSoCoSoMo), _heSoPidThuan(heSoPidThuan) {}
    void reset() {
        _tichPhanGoc_rad_s = 0.0f;
        _heSoPid = {};
        _locGyroD.datLai();
#if CAU_HINH_IMU == IMU_MPU6050_I2C
        _locNotchGyroD.datLai();
        _locButterworthGyroD.datLai();
#endif
    }
    float update(float gocNghien_rad, float tocDoNghien_radps, float gocMucTieu_rad, float buocThoiGian_s, bool dungPidMo,
                 int huongBaoHoaVanToc = 0, float heSoTyLeKp = 1.0f, float heSoTyLeKd = 1.0f); // -1: trần âm, 0: chưa bão hòa, +1: trần dương.
    PidGains heSoPid() const { return _heSoPid; }
    PidGains heSoPidThuan() const { return _heSoPidThuan; }
    PidGains heSoCoSoMo() const { return _heSoCoSo; }
    void setHeSoPidThuan(const PidGains& heSo);
    void setHeSoCoSoMo(const PidGains& heSo);
#if CAU_HINH_IMU == IMU_MPU6050_I2C
    float tocDoDaLoc_radps() const { return _locButterworthGyroD.giaTri(); }
#else
    float tocDoDaLoc_radps() const { return _locGyroD.giaTri(); }
#endif
    static PidGains scheduledGains(float saiLechGoc_rad, float tocDoNghien_radps, bool dungPidMo,
                                   PidGains heSoCoSo = {kpGoc, kiGoc, kdGoc},
                                   KieuSuyLuanMo kieuMo = kieuSuyLuanMo);
private:
    float _tichPhanGoc_rad_s = 0.0f;
    PidGains _heSoPid{};
    PidGains _heSoCoSo;
    PidGains _heSoPidThuan;
    LocThongThap _locGyroD;
#if CAU_HINH_IMU == IMU_MPU6050_I2C
    LocBiquadSiso _locNotchGyroD{LocBiquadSiso::LoaiLoc::KhiaNotch};
    LocBiquadSiso _locButterworthGyroD{LocBiquadSiso::LoaiLoc::ThongThapButterworth};
#endif
};
