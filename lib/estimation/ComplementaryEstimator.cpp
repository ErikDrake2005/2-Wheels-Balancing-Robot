#include "ComplementaryEstimator.h"
#include "config/ImuConfig.h"
#include "config/SafetyConfig.h"
#include <cmath>
#include <algorithm>

bool ComplementaryEstimator::update(const KhungDuLieuImu& khungDuLieu, TrangThaiXe& trangThaiXe) {
    trangThaiXe.gocHopLe = false;
    trangThaiXe.giaTocTinCay = false;
    trangThaiXe.doTinCayGiaToc = 0;
    trangThaiXe.saiLechGiaToc_rad = 0;
    if (!_heTruc.hopLe() || khungDuLieu.trangThai != TrangThaiKhungDuLieuImu::HopLe ||
        !khungDuLieu.daHieuChinhConQuay || khungDuLieu.thoiDiemLayMau_us == 0) return false;

    // Cả accel và gyro được xoay cùng một R sau khi driver đổi đơn vị/trừ bias.
    const Vec3Imu giaToc = _heTruc.chuyenSangXe(
        {khungDuLieu.giaTocX_mps2,khungDuLieu.giaTocY_mps2,khungDuLieu.giaTocZ_mps2});
    const Vec3Imu gyro = _heTruc.chuyenSangXe(
        {khungDuLieu.tocDoGocX_radps,khungDuLieu.tocDoGocY_radps,khungDuLieu.tocDoGocZ_radps});
    const float doLonGiaToc = doLonVecto(giaToc);
    if (!vectoHuuHan(giaToc) || !vectoHuuHan(gyro) || !std::isfinite(doLonGiaToc)) return false;
    if (doLonGiaToc < giaTocHopLeToiThieu_mps2 || doLonGiaToc > giaTocHopLeToiDa_mps2) {
        reset();
        return false;
    }
    Vec3Imu huongLenTuGiaToc = giaToc;
    if (!chuanHoaVecto(huongLenTuGiaToc)) return false;
    const bool tinCayGiaToc = std::fabs(doLonGiaToc-9.80665f) <= saiLechGiaTocChoPhep_mps2;
    trangThaiXe.doLonGiaToc_mps2 = doLonGiaToc;
    // Giữ roll từ accel làm đường bảo vệ thô, không dùng góc sensor chưa bù gá.
    trangThaiXe.gocNghiengNgang_rad = std::atan2(giaToc.y,giaToc.z);
    trangThaiXe.gocTuGiaToc_rad = std::atan2(-giaToc.x,std::hypot(giaToc.y,giaToc.z));
    trangThaiXe.doTinCayGiaToc = std::clamp(
        1.0f-std::fabs(doLonGiaToc-9.80665f)/saiLechGiaTocChoPhep_mps2,0.0f,1.0f);

    if (!_daKhoiTao) {
        if (!tinCayGiaToc) return false;
        // Khởi tạo TƯ THẾ hiện tại, không hiệu chuẩn lại ma trận lắp lúc boot.
        _huongLen = huongLenTuGiaToc;
        _daKhoiTao = true;
    } else {
        if (khungDuLieu.thoiDiemLayMau_us <= _thoiDiemMauTruoc_us) return false;
        const uint64_t khoangCachMau_us = khungDuLieu.thoiDiemLayMau_us-_thoiDiemMauTruoc_us;
        if (khoangCachMau_us > tuoiImuToiDa_us) { reset(); return false; }
        const float buocThoiGian_s = khoangCachMau_us*1e-6f;
        const float doLonGyro = doLonVecto(gyro);
        if (!std::isfinite(doLonGyro)) { reset(); return false; }
        Vec3Imu duDoan = _huongLen;
        if (doLonGyro > 1e-6f) {
            const Vec3Imu trucQuay{gyro.x/doLonGyro,gyro.y/doLonGyro,gyro.z/doLonGyro};
            const float gocQuay = doLonGyro*buocThoiGian_s;
            const float cosGoc=std::cos(gocQuay),sinGoc=std::sin(gocQuay);
            const Vec3Imu tichCheo=tichCoHuong(trucQuay,_huongLen);
            const float thanhPhanDocTruc=tichVoHuong(trucQuay,_huongLen)*(1.0f-cosGoc);
            // Rodrigues với -omega*dt: vector phương đứng biểu diễn trong body đang quay.
            duDoan={_huongLen.x*cosGoc-tichCheo.x*sinGoc+trucQuay.x*thanhPhanDocTruc,
                    _huongLen.y*cosGoc-tichCheo.y*sinGoc+trucQuay.y*thanhPhanDocTruc,
                    _huongLen.z*cosGoc-tichCheo.z*sinGoc+trucQuay.z*thanhPhanDocTruc};
        }
        if (!chuanHoaVecto(duDoan)) { reset(); return false; }
        const float saiLechHuong=std::acos(std::clamp(tichVoHuong(duDoan,huongLenTuGiaToc),-1.0f,1.0f));
        trangThaiXe.saiLechGiaToc_rad = saiLechHuong;
        const float tinCaySaiLech=std::clamp(
            (saiLechGocMatTin_rad-saiLechHuong)/(saiLechGocMatTin_rad-saiLechGocBatDauGiamTin_rad),0.0f,1.0f);
        trangThaiXe.doTinCayGiaToc *= tinCaySaiLech;
        if (trangThaiXe.doTinCayGiaToc < 0.1f) _thoiGianMatTin_s += buocThoiGian_s;
        else _thoiGianMatTin_s = 0;
        if (_thoiGianMatTin_s >= thoiGianMatTinGiaTocToiDa_s) { reset(); return false; }
        const float trongSo=trangThaiXe.doTinCayGiaToc*buocThoiGian_s/(hangSoThoiGianLocGoc_s+buocThoiGian_s);
        _huongLen={duDoan.x+trongSo*(huongLenTuGiaToc.x-duDoan.x),
                   duDoan.y+trongSo*(huongLenTuGiaToc.y-duDoan.y),
                   duDoan.z+trongSo*(huongLenTuGiaToc.z-duDoan.z)};
        if (!chuanHoaVecto(_huongLen)) { reset(); return false; }
    }
    const float chieuDaiYZ=std::hypot(_huongLen.y,_huongLen.z);
    // Pitch Euler suy biến gần +/-90 độ; xe đã ở ngoài vùng cân bằng.
    if (chieuDaiYZ < 1e-4f) { reset(); return false; }
    trangThaiXe.gocNghien_rad = std::atan2(-_huongLen.x,chieuDaiYZ);
    trangThaiXe.tocDoNghien_radps = (_huongLen.z*gyro.y-_huongLen.y*gyro.z)/chieuDaiYZ;
    trangThaiXe.tocDoNghiengNgang_radps = gyro.x-
        _huongLen.x*(_huongLen.y*gyro.y+_huongLen.z*gyro.z)/(chieuDaiYZ*chieuDaiYZ);
    const float chieuDaiXY=std::hypot(_huongLen.x,_huongLen.y);
    trangThaiXe.gocNghienTong_rad = std::atan2(chieuDaiXY,_huongLen.z);
    trangThaiXe.huongNghien_rad = chieuDaiXY>1e-6f ? std::atan2(-_huongLen.y,-_huongLen.x) : 0.0f;
    _thoiDiemMauTruoc_us = khungDuLieu.thoiDiemLayMau_us;
    trangThaiXe.giaTocTinCay = tinCayGiaToc && trangThaiXe.doTinCayGiaToc>=0.5f;
    trangThaiXe.thoiDiemGoc_us = _thoiDiemMauTruoc_us;
    trangThaiXe.gocHopLe = true;
    return true;
}
