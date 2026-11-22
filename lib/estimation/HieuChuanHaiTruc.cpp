#include "HieuChuanHaiTruc.h"
#include "config/CalibConfig.h"
#include "config/ImuConfig.h"
#include <algorithm>
#include <cmath>
#include <initializer_list>

void HieuChuanHaiTruc::BoTichLuyGyro::them(const Vec3Imu& gyro, float dt_s) {
    const float tocDo = doLonVecto(gyro);
    if (!std::isfinite(tocDo) || tocDo < 0.07f || tocDo > 2.5f ||
        !std::isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.02f) {
        return;
    }
    const float tp[] = {gyro.x, gyro.y, gyro.z};
    for (unsigned h = 0; h < 3; ++h) {
        for (unsigned c = 0; c < 3; ++c) {
            moment[h][c] += tp[h] * tp[c] * dt_s;
        }
    }
    tongGoc.x += gyro.x * dt_s;
    tongGoc.y += gyro.y * dt_s;
    tongGoc.z += gyro.z * dt_s;
    ++soMau;
}

bool HieuChuanHaiTruc::BoTichLuyGyro::timTrucChinh(Vec3Imu& truc, float& tyLeNangLuong) const {
    truc = tongGoc;
    if (!chuanHoaVecto(truc)) return false;
    const auto nhanMoment = [this](const Vec3Imu& v) -> Vec3Imu {
        return {
            moment[0][0] * v.x + moment[0][1] * v.y + moment[0][2] * v.z,
            moment[1][0] * v.x + moment[1][1] * v.y + moment[1][2] * v.z,
            moment[2][0] * v.x + moment[2][1] * v.y + moment[2][2] * v.z
        };
    };
    for (unsigned lan = 0; lan < 12; ++lan) {
        truc = nhanMoment(truc);
        if (!chuanHoaVecto(truc)) return false;
    }
    const float nangLuongTong = moment[0][0] + moment[1][1] + moment[2][2];
    if (!std::isfinite(nangLuongTong) || nangLuongTong <= 0.0f) return false;
    tyLeNangLuong = tichVoHuong(truc, nhanMoment(truc)) / nangLuongTong;
    return std::isfinite(tyLeNangLuong);
}

void HieuChuanHaiTruc::themMauPitch(const Vec3Imu& gyro, float khoangThoiGian_s) {
    _pitch.them(gyro, khoangThoiGian_s);
}

void HieuChuanHaiTruc::themMauRoll(const Vec3Imu& gyro, float khoangThoiGian_s) {
    _roll.them(gyro, khoangThoiGian_s);
}

bool HieuChuanHaiTruc::tinhMaTran(const Vec3Imu& giaTocDung,
                                  const Vec3Imu& giaTocTruoc, const Vec3Imu& giaTocSau,
                                  const Vec3Imu& giaTocTrai, const Vec3Imu& giaTocPhai,
                                  MaTranQuayImu& ketQua,
                                  BaoCaoCalibHaiTruc* baoCao) const {
    BaoCaoCalibHaiTruc chanDoan{};
    const auto ketThuc = [&](const char* lyDo, bool dat = false) {
        chanDoan.lyDo = lyDo;
        if (baoCao) *baoCao = chanDoan;
        return dat;
    };

    for (const auto* giaToc : {&giaTocDung, &giaTocTruoc, &giaTocSau, &giaTocTrai, &giaTocPhai}) {
        if (!vectoHuuHan(*giaToc) || std::fabs(doLonVecto(*giaToc) - 9.80665f) > saiLechNormGiaTocCalib_mps2) {
            return ketThuc("gia_toc_moc_khong_hop_le");
        }
    }
    if (_pitch.soMau < 40 || doLonVecto(_pitch.tongGoc) < 2.2f) {
        return ketThuc("luot_pitch_qua_ngan");
    }
    if (_roll.soMau < 25 || doLonVecto(_roll.tongGoc) < 0.70f) {
        return ketThuc("luot_roll_qua_ngan");
    }

    // 1. Trích xuất 2 trục quay độc lập từ gyro (Y_gyro từ lượt Pitch, X_gyro từ lượt Roll).
    Vec3Imu yGyro{}, xGyro{};
    if (!_pitch.timTrucChinh(yGyro, chanDoan.tyLeNangLuongPitch)) return ketThuc("moment_pitch_loi");
    if (chanDoan.tyLeNangLuongPitch < 0.80f) return ketThuc("pitch_lan_nhieu_truc");
    // Hạ ĐẦU -> ĐUÔI làm pitch giảm => tổng góc quay gyro ngược chiều +Y (trục trái).
    if (tichVoHuong(yGyro, _pitch.tongGoc) > 0.0f) yGyro = {-yGyro.x, -yGyro.y, -yGyro.z};

    if (!_roll.timTrucChinh(xGyro, chanDoan.tyLeNangLuongRoll)) return ketThuc("moment_roll_loi");
    if (chanDoan.tyLeNangLuongRoll < 0.80f) return ketThuc("roll_lan_nhieu_truc");
    // Nghiêng TRÁI -> PHẢI làm roll tăng quanh +X => tổng góc quay gyro cùng chiều +X (trục tiến).
    if (tichVoHuong(xGyro, _roll.tongGoc) < 0.0f) xGyro = {-xGyro.x, -xGyro.y, -xGyro.z};

    // 2. Trích xuất trực tiếp 2 trục X và Y từ trọng lực tĩnh của 4 mốc đối xứng:
    //    - X_acc = normalize(a_sau - a_truoc)  (vì a_truoc ~ -g*X, a_sau ~ +g*X)
    //    - Y_acc = normalize(a_phai - a_trai)  (vì a_trai ~ -g*sin(phi)*Y, a_phai ~ +g*sin(phi)*Y)
    Vec3Imu xAcc{giaTocSau.x - giaTocTruoc.x, giaTocSau.y - giaTocTruoc.y, giaTocSau.z - giaTocTruoc.z};
    Vec3Imu yAcc{giaTocPhai.x - giaTocTrai.x, giaTocPhai.y - giaTocTrai.y, giaTocPhai.z - giaTocTrai.z};
    if (doLonVecto(xAcc) < 12.0f || !chuanHoaVecto(xAcc)) return ketThuc("moc_dau_duoi_khong_tach_biet");
    if (doLonVecto(yAcc) < 6.0f || !chuanHoaVecto(yAcc)) return ketThuc("moc_trai_phai_khong_tach_biet");

    // 3. Kiểm tra chéo đồng nhất giữa Accel tĩnh và Gyro quay trên cả 2 trục (ngưỡng <= 18 độ).
    constexpr float nguongLechAccelGyro_rad = 0.31415927f; // ~18 deg
    chanDoan.lechTrucXGyro_rad = std::acos(std::clamp(tichVoHuong(xAcc, xGyro), -1.0f, 1.0f));
    chanDoan.lechTrucYGyro_rad = std::acos(std::clamp(tichVoHuong(yAcc, yGyro), -1.0f, 1.0f));
    if (chanDoan.lechTrucXGyro_rad > nguongLechAccelGyro_rad) return ketThuc("lech_truc_x_accel_gyro");
    if (chanDoan.lechTrucYGyro_rad > nguongLechAccelGyro_rad) return ketThuc("lech_truc_y_accel_gyro");

    // 4. Kiểm tra độ vuông góc thực giữa X_acc và Y_acc (góc nằm trong 75..105 độ).
    const float cosXy = std::clamp(tichVoHuong(xAcc, yAcc), -1.0f, 1.0f);
    constexpr float nuaPi = 1.57079632679f;
    chanDoan.lechVuongGoc_rad = std::fabs(nuaPi - std::acos(cosXy));
    if (chanDoan.lechVuongGoc_rad > 0.26179939f) return ketThuc("hai_truc_khong_vuong_goc"); // > 15 deg

    // 5. Trực giao hóa đối xứng (Wahba / Polar Decomposition 2 vector đơn vị):
    //    u = normalize(xAcc + yAcc), v = normalize(xAcc - yAcc) => u ⊥ v chính xác.
    //    xOpt = normalize(u + v), yOpt = normalize(u - v) chia đều sai số lệch vuông góc cho cả X và Y.
    Vec3Imu u{xAcc.x + yAcc.x, xAcc.y + yAcc.y, xAcc.z + yAcc.z};
    Vec3Imu v{xAcc.x - yAcc.x, xAcc.y - yAcc.y, xAcc.z - yAcc.z};
    if (!chuanHoaVecto(u) || !chuanHoaVecto(v)) return ketThuc("truc_giao_doi_xung_loi");
    Vec3Imu trucTien{u.x + v.x, u.y + v.y, u.z + v.z};
    Vec3Imu trucTrai{u.x - v.x, u.y - v.y, u.z - v.z};
    if (!chuanHoaVecto(trucTien) || !chuanHoaVecto(trucTrai)) return ketThuc("truc_giao_doi_xung_loi");
    Vec3Imu trucLen = tichCoHuong(trucTien, trucTrai);
    if (!chuanHoaVecto(trucLen)) return ketThuc("khong_dung_duoc_truc_len");

    const MaTranQuayImu maTran = {{
        {trucTien.x, trucTien.y, trucTien.z},
        {trucTrai.x, trucTrai.y, trucTrai.z},
        {trucLen.x,  trucLen.y,  trucLen.z}
    }};
    if (!HeTrucImu::kiemTraMaTran(maTran)) return ketThuc("ma_tran_khong_hop_le");

    // 6. Hậu kiểm toàn bộ 5 mốc tĩnh và góc tích phân gyro của cả hai lượt.
    const HeTrucImu heTruc(maTran);
    const Vec3Imu dung  = heTruc.chuyenSangXe(giaTocDung);
    const Vec3Imu truoc = heTruc.chuyenSangXe(giaTocTruoc);
    const Vec3Imu sau   = heTruc.chuyenSangXe(giaTocSau);
    const Vec3Imu trai  = heTruc.chuyenSangXe(giaTocTrai);
    const Vec3Imu phai  = heTruc.chuyenSangXe(giaTocPhai);

    if (dung.z <= 0.0f ||
        std::fabs(std::atan2(-dung.x, dung.z)) > 0.27f ||
        std::fabs(std::atan2(dung.y, std::hypot(dung.x, dung.z))) > 0.18f) {
        return ketThuc("nguoc_chieu_hoac_moc_dung_qua_lech");
    }
    if (std::fabs(truoc.y) > 1.70f || std::fabs(sau.y) > 1.70f) {
        return ketThuc("moc_dau_duoi_bi_nghieng_ngang");
    }
    if (std::fabs(trai.x) > 2.20f || std::fabs(phai.x) > 2.20f) {
        return ketThuc("moc_trai_phai_bi_nghieng_doc");
    }

    chanDoan.gocMocTruoc_rad = std::atan2(-truoc.x, truoc.z);
    chanDoan.gocMocSau_rad   = std::atan2(-sau.x, sau.z);
    chanDoan.gocQuayPitchGyro_rad = -tichVoHuong(trucTrai, _pitch.tongGoc);
    if (chanDoan.gocMocTruoc_rad < gocNamCalibToiThieu_rad || chanDoan.gocMocTruoc_rad > gocNamCalibToiDa_rad ||
        chanDoan.gocMocSau_rad > -gocNamCalibToiThieu_rad || chanDoan.gocMocSau_rad < -gocNamCalibToiDa_rad) {
        return ketThuc("moc_dau_duoi_chua_dat_nam_ngang");
    }
    if (std::fabs(chanDoan.gocQuayPitchGyro_rad - (chanDoan.gocMocTruoc_rad - chanDoan.gocMocSau_rad)) > saiSoGocQuayCalib_rad) {
        return ketThuc("goc_gyro_pitch_khong_khop");
    }

    chanDoan.gocMocTrai_rad = std::atan2(trai.y, trai.z);
    chanDoan.gocMocPhai_rad = std::atan2(phai.y, phai.z);
    chanDoan.gocQuayRollGyro_rad = tichVoHuong(trucTien, _roll.tongGoc);
    if (chanDoan.gocMocTrai_rad > -gocNghiengNgangCalibToiThieu_rad ||
        chanDoan.gocMocTrai_rad < -gocNghiengNgangCalibToiDa_rad ||
        chanDoan.gocMocPhai_rad < gocNghiengNgangCalibToiThieu_rad ||
        chanDoan.gocMocPhai_rad > gocNghiengNgangCalibToiDa_rad) {
        return ketThuc("moc_trai_phai_chua_du_goc");
    }
    if (std::fabs(chanDoan.gocQuayRollGyro_rad - (chanDoan.gocMocPhai_rad - chanDoan.gocMocTrai_rad)) > 0.30f) {
        return ketThuc("goc_gyro_roll_khong_khop");
    }

    ketQua = maTran;
    return ketThuc("PASS", true);
}
