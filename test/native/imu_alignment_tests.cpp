#include "HeTrucImu.h"
#include "ComplementaryEstimator.h"
#include "safety/SafetyMonitor.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <initializer_list>

unsigned soKiemTra=0;
void kiemTra(bool dieuKien,const char* moTa) {
    ++soKiemTra;
    if (!dieuKien) { std::printf("FAIL: %s\n",moTa); std::exit(1); }
}
bool gan(float thuNhat,float thuHai,float saiSo=1e-4f) { return std::fabs(thuNhat-thuHai)<saiSo; }
Vec3Imu veHeCamBien(const MaTranQuayImu& maTran,const Vec3Imu& trongXe) {
    const auto& r=maTran.phanTu;
    return {r[0][0]*trongXe.x+r[1][0]*trongXe.y+r[2][0]*trongXe.z,
            r[0][1]*trongXe.x+r[1][1]*trongXe.y+r[2][1]*trongXe.z,
            r[0][2]*trongXe.x+r[1][2]*trongXe.y+r[2][2]*trongXe.z};
}
Vec3Imu huongLen(float pitch,float roll) {
    return {-std::sin(pitch),std::cos(pitch)*std::sin(roll),std::cos(pitch)*std::cos(roll)};
}
KhungDuLieuImu taoMau(const MaTranQuayImu& lap,float pitch,float roll,const Vec3Imu& gyro,uint64_t thoiDiem) {
    Vec3Imu len=huongLen(pitch,roll);
    Vec3Imu giaToc=veHeCamBien(lap,{len.x*9.80665f,len.y*9.80665f,len.z*9.80665f});
    Vec3Imu conQuay=veHeCamBien(lap,gyro);
    KhungDuLieuImu mau{};
    mau.thoiDiemLayMau_us=thoiDiem; mau.daHieuChinhConQuay=true;
    mau.trangThai=TrangThaiKhungDuLieuImu::HopLe;
    mau.giaTocX_mps2=giaToc.x; mau.giaTocY_mps2=giaToc.y; mau.giaTocZ_mps2=giaToc.z;
    mau.tocDoGocX_radps=conQuay.x; mau.tocDoGocY_radps=conQuay.y; mau.tocDoGocZ_radps=conQuay.z;
    return mau;
}
int main() {
    constexpr float doiRad=0.01745329252f;
    kiemTra(HeTrucImu().hopLe(),"cau hinh lap hien tai hop le");
    HeTrucImu yaw30(HeTrucImu::tuGocLap_deg(0,0,30));
    const auto trucY=yaw30.chuyenSangXe({0.5f,0.8660254f,0});
    kiemTra(gan(trucY.x,0) && gan(trucY.y,1),"mau tinh tay yaw30 phai ra gyroY duong");
    for(float rollLap : {-10.0f,0.0f,7.0f}) for(float pitchLap : {-8.0f,0.0f,5.0f})
    for(float yawLap : {-90.0f,-45.0f,0.0f,30.0f,90.0f,180.0f}) {
        const auto lap=HeTrucImu::tuGocLap_deg(rollLap,pitchLap,yawLap);
        HeTrucImu heTruc(lap);
        kiemTra(heTruc.hopLe(),"Euler tao proper rotation");
        const Vec3Imu batKy{2,-3,4};
        const auto sensor=veHeCamBien(lap,batKy),khoiPhuc=heTruc.chuyenSangXe(sensor);
        kiemTra(gan(khoiPhuc.x,2)&&gan(khoiPhuc.y,-3)&&gan(khoiPhuc.z,4),"khoi phuc vector XYZ");
        kiemTra(gan(doLonVecto(sensor),doLonVecto(batKy)),"bao toan norm");
        MaTranQuayImu hieuChuan;
        kiemTra(HeTrucImu::tuHaiVector(veHeCamBien(lap,{0,0,9.80665f}),
                                     veHeCamBien(lap,{0,0.3f,0}),hieuChuan),"hieu chuan hai vector");
        for(unsigned hang=0;hang<3;++hang) for(unsigned cot=0;cot<3;++cot)
            kiemTra(gan(hieuChuan.phanTu[hang][cot],lap.phanTu[hang][cot]),"ma tran hieu chuan khop goc lap");
        for(float pitch : {-0.2f,0.0f,0.15f}) for(float roll : {-0.1f,0.0f,0.2f}) {
            ComplementaryEstimator boLoc(heTruc);
            TrangThaiXe xe{};
            kiemTra(boLoc.update(taoMau(lap,pitch,roll,{0,0.3f,0.8f},1000),xe),"doc tu the sau bu lap");
            kiemTra(gan(xe.gocNghien_rad,pitch),"pitch dung du roll va mounting ket hop");
            kiemTra(gan(xe.gocNghiengNgang_rad,roll),"roll bao ve trong he xe");
            kiemTra(gan(xe.tocDoNghien_radps,0.3f*std::cos(roll)-0.8f*std::sin(roll)),
                    "pitch Euler rate ket hop q va r");
        }
        ComplementaryEstimator baoVe(heTruc);
        TrangThaiXe xe{};
        baoVe.update(taoMau(lap,0,85*doiRad,{0,0,0},2000),xe);
        xe.viTriHopLe=xe.vanTocHopLe=true; xe.thoiDiemQuangDuong_us=2000;
        kiemTra(SafetyMonitor::check(xe,2001)==FaultCode::Fallen && !SafetyMonitor::canArm(xe,2001),
                "bu mounting khong zero tu the xe nga luc boot");
    }
    const auto lap=HeTrucImu::tuGocLap_deg(-4,5,30);
    const float pitch=10*doiRad,roll=20*doiRad;
    Vec3Imu len=huongLen(pitch,roll);
    Vec3Imu gyroQuayDung{0.8f*len.x,0.8f*len.y,0.8f*len.z};
    ComplementaryEstimator loc3D{HeTrucImu(lap)};
    TrangThaiXe xe{};
    for(unsigned mau=0;mau<200;++mau) {
        kiemTra(loc3D.update(taoMau(lap,pitch,roll,gyroQuayDung,1000+mau*5000),xe),"quay quanh dung voi roll pitch co san");
        kiemTra(gan(xe.gocNghien_rad,pitch)&&gan(xe.tocDoNghien_radps,0),"yaw dung khong tao pitch gia");
    }
    MaTranQuayImu sai;
    for (unsigned truongHop=0;truongHop<4;++truongHop) {
        const float pitchMau=truongHop==0 ? 0.1f : (truongHop==1 ? -0.1f : 0);
        const float rollMau=truongHop==2 ? 0.1f : 0;
        ComplementaryEstimator nghiengTong{HeTrucImu(lap)};
        const bool docDuoc=nghiengTong.update(taoMau(lap,pitchMau,rollMau,{},1000),xe);
        const float huongMongDoi=truongHop==1 ? 3.14159265359f : (truongHop==2 ? -1.57079632679f : 0);
        const float saiSoHuong=std::atan2(std::sin(xe.huongNghien_rad-huongMongDoi),
                                         std::cos(xe.huongNghien_rad-huongMongDoi));
        kiemTra(docDuoc && gan(xe.gocNghienTong_rad,truongHop==3 ? 0.0f : 0.1f) && gan(saiSoHuong,0),
                "do nghieng tong va huong XY phan biet tien lui ngang");
    }
    sai.phanTu[1][1]=-1;
    kiemTra(!HeTrucImu(sai).hopLe(),"tu choi reflection det-1");
    sai={};sai.phanTu[0][1]=0.1f;
    kiemTra(!HeTrucImu(sai).hopLe(),"tu choi shear");
    sai={};sai.phanTu[0][0]=std::numeric_limits<float>::quiet_NaN();
    kiemTra(!HeTrucImu(sai).hopLe(),"tu choi NaN");
    ComplementaryEstimator locSai{HeTrucImu(sai)};
    kiemTra(!locSai.update(taoMau({},0,0,{},1000),xe)&&!xe.gocHopLe,"ma tran loi khong duoc xuat goc hop le");
    MaTranQuayImu ketQua;
    kiemTra(!HeTrucImu::tuHaiVector({0,0,0},{0,0.3f,0},ketQua),"loai gravity rong");
    kiemTra(!HeTrucImu::tuHaiVector({0,0,9.80665f},{0,0,0},ketQua),"loai gyro qua nho");
    kiemTra(!HeTrucImu::tuHaiVector({0,0,9.80665f},{0,0,0.3f},ketQua),"loai hai vector song song");
    kiemTra(!HeTrucImu::tuHaiVector({0,0,9.80665f},{0,0.3f,0.2f},ketQua),"loai thao tac quay yaw qua nhieu");
    std::printf("PASS: IMU mounting/vector estimator %u checks\n",soKiemTra);
}
