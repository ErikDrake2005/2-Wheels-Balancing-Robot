#pragma once
#include <cmath>
#include "model/SensorFrame.h"
#include "config/ImuConfig.h"

inline bool mauImuHuuHan(const KhungDuLieuImu& mau) {
    return mau.trangThai == TrangThaiKhungDuLieuImu::HopLe &&
        std::isfinite(mau.giaTocX_mps2) && std::isfinite(mau.giaTocY_mps2) &&
        std::isfinite(mau.giaTocZ_mps2) && std::isfinite(mau.tocDoGocX_radps) &&
        std::isfinite(mau.tocDoGocY_radps) && std::isfinite(mau.tocDoGocZ_radps) &&
        std::isfinite(mau.nhietDoCamBien_C);
}
inline float normGiaTocImu(const KhungDuLieuImu& mau) {
    return std::sqrt(mau.giaTocX_mps2*mau.giaTocX_mps2 + mau.giaTocY_mps2*mau.giaTocY_mps2 +
                     mau.giaTocZ_mps2*mau.giaTocZ_mps2);
}
inline bool mauImuGanTinh(const KhungDuLieuImu& mau, float gioiHanGyro_radps) {
    const float binhPhuongGyro = mau.tocDoGocX_radps*mau.tocDoGocX_radps +
        mau.tocDoGocY_radps*mau.tocDoGocY_radps + mau.tocDoGocZ_radps*mau.tocDoGocZ_radps;
    return mauImuHuuHan(mau) &&
        std::fabs(normGiaTocImu(mau) - 9.80665f) <= saiLechNormGiaTocCalib_mps2 &&
        binhPhuongGyro <= gioiHanGyro_radps*gioiHanGyro_radps;
}

// Welford: không cần mảng lưu 500 mẫu, kiểm cả gyro và gia tốc để tránh đo bias khi lắc.
class ThongKeMauImu {
public:
    void xoa() { *this = ThongKeMauImu{}; }
    void themMau(const KhungDuLieuImu& mau) {
        const double thanhPhan[] = {mau.giaTocX_mps2, mau.giaTocY_mps2, mau.giaTocZ_mps2,
            mau.tocDoGocX_radps, mau.tocDoGocY_radps, mau.tocDoGocZ_radps};
        ++_soMau;
        for (unsigned truc = 0; truc < 6; ++truc) {
            const double lech = thanhPhan[truc] - _trungBinh[truc];
            _trungBinh[truc] += lech / _soMau;
            _tongBinhPhuongLech[truc] += lech * (thanhPhan[truc] - _trungBinh[truc]);
        }
    }
    unsigned soMau() const { return _soMau; }
    float trungBinh(unsigned truc) const { return static_cast<float>(_trungBinh[truc]); }
    float phuongSaiGiaToc() const {
        if (_soMau < 2) return INFINITY;
        return static_cast<float>((_tongBinhPhuongLech[0] + _tongBinhPhuongLech[1] +
            _tongBinhPhuongLech[2]) / (_soMau - 1));
    }
    float doLechChuanGyroLonNhat_dps() const {
        if (_soMau < 2) return INFINITY;
        double lonNhat = 0;
        for (unsigned truc = 3; truc < 6; ++truc)
            if (_tongBinhPhuongLech[truc] > lonNhat) lonNhat = _tongBinhPhuongLech[truc];
        return static_cast<float>(std::sqrt(lonNhat / (_soMau - 1)) / 0.01745329252);
    }
    bool onDinh() const {
        return _soMau >= 2 && phuongSaiGiaToc() <= phuongSaiGiaTocCalibToiDa &&
            doLechChuanGyroLonNhat_dps() <= doLechChuanConQuayToiDa_dps;
    }
private:
    unsigned _soMau = 0;
    double _trungBinh[6]{}, _tongBinhPhuongLech[6]{};
};
