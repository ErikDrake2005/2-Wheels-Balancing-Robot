#pragma once
#include "model/WheelStepCommand.h"
#include "config/SafetyConfig.h"

enum class CheDoCanBang : uint8_t { Tat, ChoLenhMoi, DangChay };

// MotorTask sở hữu; ControlTask nhận bản sao mã phiên/chế độ qua queue.
class PhienCanBang {
public:
    void yeuCauBat(uint64_t thoiDiem_us) {
        ++_maPhien;
        _thoiDiemYeuCau_us = thoiDiem_us;
        _cheDo = CheDoCanBang::ChoLenhMoi;
    }
    void tat() { ++_maPhien; _cheDo = CheDoCanBang::Tat; }
    bool xacNhanDaBat() {
        if (_cheDo != CheDoCanBang::ChoLenhMoi) return false;
        _cheDo = CheDoCanBang::DangChay;
        return true;
    }
    bool quaHanCho(uint64_t thoiDiem_us) const {
        return _cheDo == CheDoCanBang::ChoLenhMoi &&
            (thoiDiem_us < _thoiDiemYeuCau_us ||
             thoiDiem_us - _thoiDiemYeuCau_us >= thoiGianChoLenhKhoiDong_us);
    }
    bool lenhKhoiDongHopLe(uint32_t maPhien, const WheelStepCommand& lenh,
                         uint64_t thoiDiem_us) const {
        return _cheDo == CheDoCanBang::ChoLenhMoi && !quaHanCho(thoiDiem_us) &&
            maPhien == _maPhien && lenh.thoiDiemRaLenh_us >= _thoiDiemYeuCau_us &&
            lenh.thoiDiemRaLenh_us <= thoiDiem_us &&
            thoiDiem_us - lenh.thoiDiemRaLenh_us < tuoiLenhDeBat_us &&
            lenh.thoiDiemHetHan_us > thoiDiem_us &&
            lenh.tocDoBuocTrai_buocTrenGiay == 0.0f &&
            lenh.tocDoBuocPhai_buocTrenGiay == 0.0f;
    }
    uint32_t maPhien() const { return _maPhien; }
    CheDoCanBang cheDo() const { return _cheDo; }
private:
    uint32_t _maPhien = 0;
    uint64_t _thoiDiemYeuCau_us = 0;
    CheDoCanBang _cheDo = CheDoCanBang::Tat;
};
