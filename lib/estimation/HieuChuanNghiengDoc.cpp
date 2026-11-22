#include "HieuChuanNghiengDoc.h"
#include "config/CalibConfig.h"
#include "config/ImuConfig.h"
#include <algorithm>
#include <initializer_list>

void HieuChuanNghiengDoc::themMau(const Vec3Imu& gyro, float khoangThoiGian_s) {
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

bool HieuChuanNghiengDoc::tinhMaTran(const Vec3Imu& giaTocDung, const Vec3Imu& giaTocTruoc,
                                    const Vec3Imu& giaTocSau, MaTranQuayImu& ketQua,
                                    BaoCaoCalibDoc* baoCao) const {
    BaoCaoCalibDoc chanDoan;
    const auto ketThuc = [&](const char* lyDo, bool dat = false) {
        chanDoan.lyDo = lyDo;
        if (baoCao) *baoCao = chanDoan;
        return dat;
    };
    for (const auto* giaToc : {&giaTocDung, &giaTocTruoc, &giaTocSau}) {
        if (!vectoHuuHan(*giaToc) || std::fabs(doLonVecto(*giaToc) - 9.80665f) > saiLechNormGiaTocCalib_mps2)
            return ketThuc("gia_toc_moc_khong_hop_le");
    }
    if (_soMau < 40 || doLonVecto(_tongGoc) < 2.5f) return ketThuc("luot_quay_qua_ngan");

    // Tìm trục quay ngang qua HAI BÁNH bằng đủ gyro XYZ, không chọn riêng sensor Y.
    Vec3Imu trucTrai = _tongGoc;
    if (!chuanHoaVecto(trucTrai)) return ketThuc("truc_gyro_rong");
    const auto nhanMoment = [this](const Vec3Imu& vecto) -> Vec3Imu {
        return {_moment[0][0]*vecto.x + _moment[0][1]*vecto.y + _moment[0][2]*vecto.z,
                _moment[1][0]*vecto.x + _moment[1][1]*vecto.y + _moment[1][2]*vecto.z,
                _moment[2][0]*vecto.x + _moment[2][1]*vecto.y + _moment[2][2]*vecto.z};
    };
    for (unsigned lan = 0; lan < 12; ++lan) {
        trucTrai = nhanMoment(trucTrai);
        if (!chuanHoaVecto(trucTrai)) return ketThuc("moment_gyro_loi");
    }
    const float nangLuongTong = _moment[0][0] + _moment[1][1] + _moment[2][2];
    if (!std::isfinite(nangLuongTong) || nangLuongTong <= 0) return ketThuc("nang_luong_gyro_loi");
    chanDoan.tyLeNangLuongTruc = tichVoHuong(trucTrai, nhanMoment(trucTrai)) / nangLuongTong;
    if (chanDoan.tyLeNangLuongTruc < 0.82f) return ketThuc("quay_lan_nhieu_truc");
    // ĐẦU xuống -> ĐUÔI xuống có pitch GIẢM, nên tổng góc phải ngược chiều +Y thân.
    if (tichVoHuong(trucTrai, _tongGoc) > 0.0f) trucTrai = {-trucTrai.x, -trucTrai.y, -trucTrai.z};

    Vec3Imu hieuGiaToc{giaTocSau.x-giaTocTruoc.x, giaTocSau.y-giaTocTruoc.y,
                       giaTocSau.z-giaTocTruoc.z};
    const float docTruc = tichVoHuong(hieuGiaToc, trucTrai);
    if (std::fabs(docTruc) > 0.15f * doLonVecto(hieuGiaToc)) return ketThuc("hai_moc_lech_truc_quay");
    Vec3Imu trucTien{hieuGiaToc.x-docTruc*trucTrai.x, hieuGiaToc.y-docTruc*trucTrai.y,
                    hieuGiaToc.z-docTruc*trucTrai.z};
    if (!chuanHoaVecto(trucTien)) return ketThuc("hai_moc_khong_tach_biet");
    Vec3Imu trucLen = tichCoHuong(trucTien, trucTrai);
    if (!chuanHoaVecto(trucLen)) return ketThuc("khong_dung_duoc_truc_len");
    const MaTranQuayImu maTran = {{{trucTien.x, trucTien.y, trucTien.z},
                                 {trucTrai.x, trucTrai.y, trucTrai.z},
                                 {trucLen.x, trucLen.y, trucLen.z}}};
    if (!HeTrucImu::kiemTraMaTran(maTran)) return ketThuc("ma_tran_khong_hop_le");
    const HeTrucImu heTruc(maTran);
    const Vec3Imu dung = heTruc.chuyenSangXe(giaTocDung);
    const Vec3Imu truoc = heTruc.chuyenSangXe(giaTocTruoc);
    const Vec3Imu sau = heTruc.chuyenSangXe(giaTocSau);
    // Gravity lúc người giữ chỉ kiểm hướng lên, KHÔNG lấy làm Z và không ép pitch=0.
    if (dung.z <= 0 || std::fabs(std::atan2(-dung.x, dung.z)) > 0.27f ||
        std::fabs(std::atan2(dung.y, std::hypot(dung.x,dung.z))) > 0.18f)
        return ketThuc("nguoc_chieu_hoac_moc_dung_qua_lech");
    if (std::fabs(truoc.y) > 1.70f || std::fabs(sau.y) > 1.70f)
        return ketThuc("hai_moc_bi_nghieng_sang_ben");
    // Dùng vector trong mặt phẳng XZ; không dùng Euler pitch đang suy biến ở ±90°.
    chanDoan.gocMocTruoc_rad = std::atan2(-truoc.x,truoc.z);
    chanDoan.gocMocSau_rad = std::atan2(-sau.x,sau.z);
    chanDoan.gocQuayGyro_rad = -tichVoHuong(trucTrai, _tongGoc);
    if (chanDoan.gocMocTruoc_rad < gocNamCalibToiThieu_rad || chanDoan.gocMocTruoc_rad > gocNamCalibToiDa_rad ||
        chanDoan.gocMocSau_rad > -gocNamCalibToiThieu_rad || chanDoan.gocMocSau_rad < -gocNamCalibToiDa_rad)
        return ketThuc("hai_moc_chua_dat_nam_ngang");
    if (std::fabs(chanDoan.gocQuayGyro_rad - (chanDoan.gocMocTruoc_rad-chanDoan.gocMocSau_rad)) > saiSoGocQuayCalib_rad)
        return ketThuc("goc_gyro_khong_khop_hai_moc");
    // Quay gravity ở mốc sau ngược lại tới mốc trước; vẫn tốt tại đúng 180°.
    const float goc = -chanDoan.gocQuayGyro_rad;
    const float cosGoc = std::cos(goc), sinGoc = std::sin(goc);
    const Vec3Imu cheo = tichCoHuong(trucTrai, giaTocSau);
    const float doc = tichVoHuong(trucTrai, giaTocSau) * (1-cosGoc);
    Vec3Imu duDoan{giaTocSau.x*cosGoc + cheo.x*sinGoc + trucTrai.x*doc,
                  giaTocSau.y*cosGoc + cheo.y*sinGoc + trucTrai.y*doc,
                  giaTocSau.z*cosGoc + cheo.z*sinGoc + trucTrai.z*doc};
    Vec3Imu mocTruoc = giaTocTruoc;
    if (!chuanHoaVecto(duDoan) || !chuanHoaVecto(mocTruoc)) return ketThuc("vector_moc_loi");
    chanDoan.lechVector_rad = std::acos(std::clamp(tichVoHuong(duDoan,mocTruoc),-1.0f,1.0f));
    if (chanDoan.lechVector_rad > saiSoGocQuayCalib_rad) return ketThuc("vector_gyro_khong_khop");
    ketQua = maTran;
    return ketThuc("PASS", true);
}
