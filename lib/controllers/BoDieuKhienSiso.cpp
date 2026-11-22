#include "BoDieuKhienSiso.h"
#include <algorithm>
#include <cmath>
#include <limits>

void BoDieuKhienSiso::reset() {
    _daCoMauTruoc = false;
    _saiLechTruoc = 0.0f;
    _tichPhan = 0.0f;
    _viPhan = 0.0f;
    _ngoRaCuoi = 0.0f;
}

void BoDieuKhienSiso::datThamSo(const ThamSoSiso& thamSo) {
    if (!std::isfinite(thamSo.kp) || !std::isfinite(thamSo.ki) || !std::isfinite(thamSo.kd) ||
        !std::isfinite(thamSo.tauLocViPhan_s) || !std::isfinite(thamSo.gioiHanTichPhan) ||
        !std::isfinite(thamSo.gioiHanNgora)) {
        return;
    }
    if (thamSo.kp < 0.0f || thamSo.ki < 0.0f || thamSo.kd < 0.0f ||
        thamSo.tauLocViPhan_s < 0.0f || thamSo.gioiHanTichPhan < 0.0f || thamSo.gioiHanNgora <= 0.0f) {
        return;
    }
    _thamSo = thamSo;
    _tichPhan = std::clamp(_tichPhan, -_thamSo.gioiHanTichPhan, _thamSo.gioiHanTichPhan);
}

float BoDieuKhienSiso::capNhat(float saiLech, float buocThoiGian_s) {
    if (!std::isfinite(saiLech) || !std::isfinite(buocThoiGian_s) ||
        buocThoiGian_s <= 0.0f || _thamSo.gioiHanNgora <= 0.0f) {
        reset();
        return std::numeric_limits<float>::quiet_NaN();
    }

    const float uTiLe = _thamSo.kp * saiLech;

    float uViPhanMoi = 0.0f;
    if (_daCoMauTruoc && _thamSo.kd > 0.0f) {
        const float mauSo = 2.0f * _thamSo.tauLocViPhan_s + buocThoiGian_s;
        if (mauSo > 1e-6f) {
            const float heSoGiu = (2.0f * _thamSo.tauLocViPhan_s - buocThoiGian_s) / mauSo;
            const float heSoViPhan = (2.0f * _thamSo.kd) / mauSo;
            uViPhanMoi = heSoGiu * _viPhan + heSoViPhan * (saiLech - _saiLechTruoc);
        }
    }

    float tichPhanDuKien = _tichPhan;
    if (_thamSo.ki > 0.0f) {
        const float buocTichPhan = _daCoMauTruoc
            ? (0.5f * _thamSo.ki * buocThoiGian_s * (saiLech + _saiLechTruoc))
            : (_thamSo.ki * buocThoiGian_s * saiLech);
        tichPhanDuKien = std::clamp(
            _tichPhan + buocTichPhan,
            -_thamSo.gioiHanTichPhan,
            _thamSo.gioiHanTichPhan);
    } else {
        tichPhanDuKien = 0.0f;
    }

    const float uChuaGioiHan = uTiLe + tichPhanDuKien + uViPhanMoi;
    const float uDaGioiHan = std::clamp(uChuaGioiHan, -_thamSo.gioiHanNgora, _thamSo.gioiHanNgora);

    // Chống bão hòa tích phân (Clamping Anti-Windup): chỉ tích lũy khi ngõ ra chưa bão hòa
    // hoặc sai lệch ngược dấu giúp kéo ngõ ra thoát khỏi vùng bão hòa.
    if (_thamSo.ki <= 0.0f) {
        _tichPhan = 0.0f;
    } else if (uChuaGioiHan == uDaGioiHan || uChuaGioiHan * saiLech < 0.0f) {
        _tichPhan = tichPhanDuKien;
    }

    _viPhan = uViPhanMoi;
    _saiLechTruoc = saiLech;
    _daCoMauTruoc = true;
    _ngoRaCuoi = uDaGioiHan;
    return _ngoRaCuoi;
}
