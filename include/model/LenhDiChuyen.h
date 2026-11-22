#pragma once
#include <cstdint>
#include <cmath>
#include "config/ControlConfig.h"
#include "model/GoiLenhNhiPhan.h"

struct LenhDiChuyen {
    float vanToc_mps = 0;
    float tocDoQuay_radps = 0; // Dương: quay trái theo quy ước vPhai > vTrai.
    uint64_t thoiDiem_us = 0, hetHan_us = 0;
    uint32_t maPhien = 0;
    MucLenhWeb mucWeb{};
    bool hopLe(uint32_t maPhienHienTai, uint64_t hienTai_us) const {
        return maPhien == maPhienHienTai && std::isfinite(vanToc_mps) &&
            std::isfinite(tocDoQuay_radps) && thoiDiem_us <= hienTai_us &&
            hetHan_us > hienTai_us && hetHan_us > thoiDiem_us &&
            hetHan_us - thoiDiem_us <= hanLenhDiChuyen_us;
    }
};
