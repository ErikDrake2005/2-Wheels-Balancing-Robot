#include "HieuChuanNghiengDoc.h"
#include "ComplementaryEstimator.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
constexpr float g = 9.80665f, doiRad = 0.01745329252f;
unsigned soDieuKien = 0;
void kiemTra(bool dat, const char* moTa) {
    ++soDieuKien;
    if (!dat) { std::cerr << "FAIL: " << moTa << '\n'; std::exit(1); }
}
Vec3Imu veCamBien(const MaTranQuayImu& lap, const Vec3Imu& than) {
    const auto& r = lap.phanTu;
    return {r[0][0]*than.x+r[1][0]*than.y+r[2][0]*than.z,
            r[0][1]*than.x+r[1][1]*than.y+r[2][1]*than.z,
            r[0][2]*than.x+r[1][2]*than.y+r[2][2]*than.z};
}
Vec3Imu giaTocDoc(float goc_deg) {
    return {-g*std::sin(goc_deg*doiRad),0,g*std::cos(goc_deg*doiRad)};
}
HieuChuanNghiengDoc taoLuot(const MaTranQuayImu& lap, float gocQuay_rad, float nhieu = 0.005f) {
    HieuChuanNghiengDoc luot;
    for (unsigned i = 0; i < 600; ++i) {
        const float dau = i % 2 ? 1.0f : -1.0f;
        luot.themMau(veCamBien(lap,{dau*nhieu,-gocQuay_rad/3.0f,dau*nhieu}),0.005f);
    }
    return luot;
}
void kiemTraCacGocLap() {
    for (float rollLap : {-12.0f,0.0f,17.0f})
    for (float pitchLap : {-8.0f,0.0f,11.0f})
    for (float yawLap : {-45.0f,0.0f,30.0f,90.0f,180.0f})
    for (float gocNam : {80.0f,85.0f,90.0f}) {
        const auto lap = HeTrucImu::tuGocLap_deg(rollLap,pitchLap,yawLap);
        const auto luot = taoLuot(lap,2*gocNam*doiRad);
        for (float gocNguoiGiu : {-6.0f,0.0f,6.0f}) {
            MaTranQuayImu ketQua;
            BaoCaoCalibDoc baoCao;
            kiemTra(luot.tinhMaTran(veCamBien(lap,giaTocDoc(gocNguoiGiu)),
                veCamBien(lap,giaTocDoc(gocNam)),veCamBien(lap,giaTocDoc(-gocNam)),ketQua,&baoCao),
                "nhan DAU->DUOI voi gyro tron XYZ va moc nam80..90");
            const HeTrucImu heTruc(ketQua);
            const Vec3Imu x = heTruc.chuyenSangXe(veCamBien(lap,{1,0,0}));
            const Vec3Imu y = heTruc.chuyenSangXe(veCamBien(lap,{0,1,0}));
            kiemTra(std::fabs(x.x-1)<1e-4f && std::fabs(x.y)<1e-4f && std::fabs(x.z)<1e-4f,
                "tim dung truc DAU/DUOI khong ep gravity tham chieu thanhZ");
            kiemTra(std::fabs(y.x)<1e-4f && std::fabs(y.y-1)<1e-4f && std::fabs(y.z)<1e-4f,
                "tim dung truc quay quanh hai banh va dau duong");
            kiemTra(std::fabs(baoCao.gocMocTruoc_rad/doiRad-gocNam)<.005f &&
                std::fabs(baoCao.gocMocSau_rad/doiRad+gocNam)<.005f,
                "hai moc vector co dau dung tai90do");
            ComplementaryEstimator boLoc(heTruc);
            KhungDuLieuImu mau;
            const auto a = veCamBien(lap,giaTocDoc(gocNguoiGiu));
            const auto w = veCamBien(lap,{0,.2f,0});
            mau.giaTocX_mps2=a.x; mau.giaTocY_mps2=a.y; mau.giaTocZ_mps2=a.z;
            mau.tocDoGocX_radps=w.x; mau.tocDoGocY_radps=w.y; mau.tocDoGocZ_radps=w.z;
            mau.thoiDiemLayMau_us=10000;
            mau.trangThai=TrangThaiKhungDuLieuImu::HopLe; mau.daHieuChinhConQuay=true;
            TrangThaiXe trangThai;
            kiemTra(boLoc.update(mau,trangThai) &&
                std::fabs(trangThai.gocNghien_rad-gocNguoiGiu*doiRad)<1e-4f &&
                std::fabs(trangThai.tocDoNghien_radps-.2f)<1e-4f,
                "estimator dung ma tran: giu pitch nguoi giu va toc do duong");
        }
    }
}
void kiemTraLoaiLuotSai() {
    const auto lap=HeTrucImu::tuGocLap_deg(-4,5,30);
    const auto dung=veCamBien(lap,giaTocDoc(0));
    const auto truoc=veCamBien(lap,giaTocDoc(90));
    const auto sau=veCamBien(lap,giaTocDoc(-90));
    auto luot=taoLuot(lap,180*doiRad);
    MaTranQuayImu ketQua;
    BaoCaoCalibDoc baoCao;
    kiemTra(luot.tinhMaTran(dung,truoc,sau,ketQua,&baoCao), "quay dung180do khong suy bien");
    kiemTra(!taoLuot(lap,-180*doiRad).tinhMaTran(dung,truoc,sau,ketQua,&baoCao), "tu choi quay nguoc qua lung");
    kiemTra(!taoLuot(lap,180*doiRad,.4f).tinhMaTran(dung,truoc,sau,ketQua,&baoCao), "tu choi quay lan nhieu truc");
    kiemTra(!taoLuot(lap,160*doiRad).tinhMaTran(dung,truoc,sau,ketQua,&baoCao), "tu choi matgoc gyro du moc180");
    kiemTra(!luot.tinhMaTran(dung,truoc,truoc,ketQua,&baoCao), "tu choi moc khong doi ben");
    kiemTra(!luot.tinhMaTran({0,0,0},truoc,sau,ketQua,&baoCao), "tu choi gia toc0");
    kiemTra(!luot.tinhMaTran({std::numeric_limits<float>::quiet_NaN(),0,g},truoc,sau,ketQua), "tu choi NaN");
    kiemTra(!taoLuot(lap,90*doiRad).tinhMaTran(dung,veCamBien(lap,giaTocDoc(45)),
        veCamBien(lap,giaTocDoc(-45)),ketQua), "khong nhan thao tac30..45cu");
    kiemTra(!taoLuot(lap,200*doiRad).tinhMaTran(dung,veCamBien(lap,giaTocDoc(100)),
        veCamBien(lap,giaTocDoc(-100)),ketQua), "tu choi dat qua92do");
    kiemTra(!luot.tinhMaTran(veCamBien(lap,giaTocDoc(30)),truoc,sau,ketQua), "moc dung qua lech");
    // Khoang80/90 khong the tu phan biet sai moc X5do: kiem thu gioi han quan sat.
    const auto luot170=taoLuot(lap,170*doiRad);
    kiemTra(luot170.tinhMaTran(dung,veCamBien(lap,giaTocDoc(80)),sau,ketQua,&baoCao),
        "nhan cua so80..90 nhung khong duoc tuyen bo zero tuyet doi");
    const auto goc = HeTrucImu(ketQua).chuyenSangXe(dung);
    kiemTra(std::fabs(std::atan2(-goc.x,goc.z)/doiRad-5)<.01f,
        "moc80/90 gay sai zero5do, can mat do co khi");
    HieuChuanNghiengDoc mauHong;
    for (unsigned i=0;i<100;++i) {
        mauHong.themMau({0,-10,0},.005f);
        mauHong.themMau({0,-1,0},.1f);
    }
    kiemTra(!mauHong.tinhMaTran(dung,truoc,sau,ketQua), "bo gyro qua nhanh va dt gian doan");
}
}
int main() {
    kiemTraCacGocLap();
    kiemTraLoaiLuotSai();
    std::cout << "PASS: DAU/DUOI pitch calibration, mounting and reference ambiguity " << soDieuKien << " checks\n";
}
