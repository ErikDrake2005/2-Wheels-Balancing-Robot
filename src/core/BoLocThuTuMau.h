#pragma once
#include <cstdint>

// Chỉ ImuTask sử dụng. Phép trừ unsigned xử lý vòng lại UINT32_MAX -> 0.
class BoLocThuTuMau {
public:
    bool chapNhan(uint32_t soThuTu) {
        if (_daCoMau) {
            const uint32_t chenhLech = soThuTu - _soThuTuTruoc;
            if (chenhLech == 0 || chenhLech >= 0x80000000u) {
                ++_soThongBaoTrung;
                return false;
            }
            _soMauBoLo += chenhLech - 1u;
        }
        _soThuTuTruoc = soThuTu;
        _daCoMau = true;
        return true;
    }
    uint64_t soMauBoLo() const { return _soMauBoLo; }
    uint32_t soThongBaoTrung() const { return _soThongBaoTrung; }
private:
    bool _daCoMau = false;
    uint32_t _soThuTuTruoc = 0, _soThongBaoTrung = 0;
    uint64_t _soMauBoLo = 0;
};
