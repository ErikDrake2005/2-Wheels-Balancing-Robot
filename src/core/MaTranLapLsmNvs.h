#pragma once
#include "HeTrucImu.h"
#include <cstdint>
#include <cstring>

namespace mountLsm {
// Đổi phiên bản khóa nếu thay quy ước trục sau khi giải mã thanh ghi.
constexpr const char* khoa = "mount_lsm_v1";
struct BanGhi {
    uint32_t who;
    MaTranQuayImu maTran;
    float ref_rad;
};
static_assert(sizeof(BanGhi) == 11 * sizeof(uint32_t), "Layout NVS LSM thay doi");

inline bool hopLe(const BanGhi& banGhi, uint8_t who) {
    return (who == 0x70 || who == 0x71) && banGhi.who == who &&
        HeTrucImu::kiemTraMaTran(banGhi.maTran) &&
        std::isfinite(banGhi.ref_rad) && std::fabs(banGhi.ref_rad) <= 0.35f;
}

template <class Nvs>
bool doc(Nvs& nvs, uint8_t who, BanGhi& ketQua) {
    BanGhi ungVien{};
    if (!nvs.isKey(khoa) || nvs.getBytesLength(khoa) != sizeof(ungVien) ||
        nvs.getBytes(khoa, &ungVien, sizeof(ungVien)) != sizeof(ungVien) ||
        !hopLe(ungVien, who)) return false;
    ketQua = ungVien;
    return true;
}

template <class Nvs>
bool luu(Nvs& nvs, uint8_t who, const MaTranQuayImu& maTran, float ref_rad) {
    const BanGhi moi{who, maTran, ref_rad};
    if (!hopLe(moi, who)) return false;
    BanGhi daLuu{};
    if (doc(nvs, who, daLuu) && std::memcmp(&daLuu, &moi, sizeof(moi)) == 0) return true;
    return nvs.putBytes(khoa, &moi, sizeof(moi)) == sizeof(moi) &&
        doc(nvs, who, daLuu) && std::memcmp(&daLuu, &moi, sizeof(moi)) == 0;
}
}
