#pragma once
#include "model/SensorFrame.h"
#include "model/RobotState.h"
#include "HeTrucImu.h"
#include <cmath>

class ComplementaryEstimator {
public:
    explicit ComplementaryEstimator(const HeTrucImu& heTruc = HeTrucImu()) : _heTruc(heTruc) {}
    // giaTocTinhTien_mps2: gia tốc tịnh tiến bánh theo phương ngang (+ tiến) đã ra lệnh
    // trong chu kỳ vừa qua. 0 = không bù (motor tắt, test cũ).
    bool update(const KhungDuLieuImu& khungDuLieu, TrangThaiXe& trangThaiXe, float giaTocTinhTien_mps2 = 0.0f);
    void reset() { _daKhoiTao = false; _thoiDiemMauTruoc_us = 0; _thoiGianMatTin_s = 0; }
    bool heTrucHopLe() const { return _heTruc.hopLe(); }
    void datTrongTruongThamChieu_mps2(float giaTri_mps2) {
        if (std::isfinite(giaTri_mps2) && giaTri_mps2 >= 9.0f && giaTri_mps2 <= 10.6f) {
            _trongTruongThamChieu_mps2 = giaTri_mps2;
        }
    }
    float trongTruongThamChieu_mps2() const { return _trongTruongThamChieu_mps2; }
private:
    HeTrucImu _heTruc;
    Vec3Imu _huongLen{0,0,1};
    float _trongTruongThamChieu_mps2 = 9.80665f;
    uint64_t _thoiDiemMauTruoc_us = 0;
    bool _daKhoiTao = false;
    float _thoiGianMatTin_s = 0;
};
