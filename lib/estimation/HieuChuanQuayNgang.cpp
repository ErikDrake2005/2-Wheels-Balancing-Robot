#include "HieuChuanQuayNgang.h"
#include <initializer_list>

void HieuChuanQuayNgang::themMau(const Vec3Imu& gyro, float khoangThoiGian_s) {
    const float tocDo = doLonVecto(gyro);
    if (!std::isfinite(tocDo) || tocDo < 0.07f || tocDo > 2.5f ||
        !std::isfinite(khoangThoiGian_s) || khoangThoiGian_s <= 0 || khoangThoiGian_s > 0.02f) return;

    const float thanhPhan[] = {gyro.x, gyro.y, gyro.z};
    for (unsigned hang = 0; hang < 3; ++hang) {
        for (unsigned cot = 0; cot < 3; ++cot) {
            _moment[hang][cot] += thanhPhan[hang] * thanhPhan[cot] * khoangThoiGian_s;
        }
    }
    _tongGoc.x += gyro.x * khoangThoiGian_s;
    _tongGoc.y += gyro.y * khoangThoiGian_s;
    _tongGoc.z += gyro.z * khoangThoiGian_s;
    ++_soMau;
}

bool HieuChuanQuayNgang::tinhMaTran(const Vec3Imu& giaTocDung, const Vec3Imu& giaTocTrai,
                                    const Vec3Imu& giaTocPhai, MaTranQuayImu& ketQua) const {
    for (const auto* giaToc : {&giaTocDung, &giaTocTrai, &giaTocPhai}) {
        if (!vectoHuuHan(*giaToc) || std::fabs(doLonVecto(*giaToc) - 9.80665f) > 0.5f) return false;
    }
    if (_soMau < 40 || doLonVecto(_tongGoc) < 0.50f) return false;

    // Power iteration tìm trục năng lượng gyro chính. Tổng góc có dấu chọn chiều X tiến.
    Vec3Imu trucTien = _tongGoc;
    if (!chuanHoaVecto(trucTien)) return false;
    const auto nhanMoment = [this](const Vec3Imu& vecto) -> Vec3Imu {
        return {_moment[0][0]*vecto.x + _moment[0][1]*vecto.y + _moment[0][2]*vecto.z,
                _moment[1][0]*vecto.x + _moment[1][1]*vecto.y + _moment[1][2]*vecto.z,
                _moment[2][0]*vecto.x + _moment[2][1]*vecto.y + _moment[2][2]*vecto.z};
    };
    for (unsigned lan = 0; lan < 12; ++lan) {
        trucTien = nhanMoment(trucTien);
        if (!chuanHoaVecto(trucTien)) return false;
    }
    const float nangLuongTong = _moment[0][0] + _moment[1][1] + _moment[2][2];
    if (tichVoHuong(trucTien, nhanMoment(trucTien)) < 0.95f * nangLuongTong) return false;
    if (tichVoHuong(trucTien, _tongGoc) < 0.0f) trucTien = {-trucTien.x, -trucTien.y, -trucTien.z};

    Vec3Imu huongTrongLuc = giaTocDung;
    if (!chuanHoaVecto(huongTrongLuc) || std::fabs(tichVoHuong(trucTien, huongTrongLuc)) > 0.15f)
        return false;
    // Giữ X đo từ chuyển động của KHUNG XE. Không ép X nằm ngang bằng gravity lúc người giữ.
    // Nhờ vậy pitch ban đầu vẫn đo được, không bị gán bằng 0 rồi nhầm thành góc cân bằng.
    Vec3Imu trucTrai = tichCoHuong(huongTrongLuc, trucTien);
    if (!chuanHoaVecto(trucTrai)) return false;
    Vec3Imu trucLen = tichCoHuong(trucTien, trucTrai);
    if (!chuanHoaVecto(trucLen)) return false;
    const MaTranQuayImu maTran = {{{trucTien.x, trucTien.y, trucTien.z},
                                 {trucTrai.x, trucTrai.y, trucTrai.z},
                                 {trucLen.x, trucLen.y, trucLen.z}}};
    if (!HeTrucImu::kiemTraMaTran(maTran)) return false;
    const HeTrucImu heTruc(maTran);
    const Vec3Imu trai = heTruc.chuyenSangXe(giaTocTrai);
    const Vec3Imu phai = heTruc.chuyenSangXe(giaTocPhai);
    // Hai mốc phải nghiêng trái/phải thật và không lẫn pitch quá ~10 độ.
    if (std::fabs(trai.x) > 1.70f || std::fabs(phai.x) > 1.70f) return false;
    const float rollTrai = std::atan2(trai.y, trai.z);
    const float rollPhai = std::atan2(phai.y, phai.z);
    if (rollTrai > -0.35f || rollTrai < -1.75f || rollPhai < 0.35f || rollPhai > 1.75f) return false;
    const float gocQuayDo = tichVoHuong(trucTien, _tongGoc);
    if (std::fabs(gocQuayDo - (rollPhai - rollTrai)) > 0.35f) return false;
    ketQua = maTran;
    return true;
}
