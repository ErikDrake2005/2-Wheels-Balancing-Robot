#pragma once
#include "model/SensorFrame.h"
#include "model/RobotState.h"
#include "HeTrucImu.h"

class ComplementaryEstimator {
public:
    explicit ComplementaryEstimator(const HeTrucImu& heTruc = HeTrucImu()) : _heTruc(heTruc) {}
    // giaTocTinhTien_mps2: gia tốc tịnh tiến bánh theo phương ngang (+ tiến) đã ra lệnh
    // trong chu kỳ vừa qua. 0 = không bù (motor tắt, test cũ).
    bool update(const KhungDuLieuImu& khungDuLieu, TrangThaiXe& trangThaiXe, float giaTocTinhTien_mps2 = 0.0f);
    void reset() { _daKhoiTao = false; _thoiDiemMauTruoc_us = 0; _thoiGianMatTin_s = 0; }
    bool heTrucHopLe() const { return _heTruc.hopLe(); }
private:
    HeTrucImu _heTruc;
    Vec3Imu _huongLen{0,0,1};
    uint64_t _thoiDiemMauTruoc_us = 0;
    bool _daKhoiTao = false;
    float _thoiGianMatTin_s = 0;
};
