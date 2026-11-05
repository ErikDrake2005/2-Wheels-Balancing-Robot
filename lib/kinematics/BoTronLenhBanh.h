#pragma once
#include <algorithm>
#include <cmath>

struct VanTocHaiBanh { float trai_mps, phai_mps; };
inline VanTocHaiBanh tronUuTienCanBang(float vanTocCanBang_mps, float tocDoQuay_radps,
                                      float khoangCachBanh_m, float vanTocBanhToiDa_mps) {
    const float vanTocChung = std::clamp(vanTocCanBang_mps, -vanTocBanhToiDa_mps, vanTocBanhToiDa_mps);
    const float duDia = std::max(0.0f, vanTocBanhToiDa_mps - std::fabs(vanTocChung));
    const float chenhLech = std::clamp(tocDoQuay_radps * khoangCachBanh_m * 0.5f, -duDia, duDia);
    return {vanTocChung - chenhLech, vanTocChung + chenhLech};
}
