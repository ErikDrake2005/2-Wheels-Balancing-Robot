#pragma once
#include <cmath>
#include <limits>

// PT1 backward-Euler, công thức chuẩn được triển khai độc lập.
// Đối chiếu PX4 AlphaFilter (BSD-3); xem docs/NGUON_THAM_KHAO_LOC.md.
class LocThongThap {
public:
    void datLai() { _daCoMau = false; _giaTri = 0; }
    float capNhat(float giaTri, float buocThoiGian_s, float tanSoDanhNghia_hz) {
        if (!std::isfinite(giaTri) || !std::isfinite(buocThoiGian_s) ||
            !std::isfinite(tanSoDanhNghia_hz) || buocThoiGian_s <= 0 || tanSoDanhNghia_hz < 0)
            return std::numeric_limits<float>::quiet_NaN();
        if (!_daCoMau || tanSoDanhNghia_hz == 0) {
            _giaTri = giaTri;
            _daCoMau = true;
        } else {
            const float hangSoThoiGian_s = 1.0f / (6.28318530718f * tanSoDanhNghia_hz);
            const float heSo = buocThoiGian_s / (hangSoThoiGian_s + buocThoiGian_s);
            _giaTri += heSo * (giaTri - _giaTri);
        }
        return _giaTri;
    }
    float giaTri() const { return _giaTri; }
private:
    float _giaTri = 0;
    bool _daCoMau = false;
};
