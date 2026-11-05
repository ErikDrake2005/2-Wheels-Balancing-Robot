#include "LocThongThap.h"
#include "ComplementaryEstimator.h"
#include "AnglePidController.h"
#include "CascadeController.h"
#include "BoTronLenhBanh.h"
#include "model/LenhDiChuyen.h"
#include "config/RobotGeometry.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

unsigned soKiemTra = 0;
void kiemTra(bool dieuKien, const char* moTa) {
    ++soKiemTra;
    if (!dieuKien) { std::printf("FAIL: %s\n", moTa); std::exit(1); }
}
KhungDuLieuImu taoMau(uint64_t thoiDiem_us, float giaTocTien = 0) {
    KhungDuLieuImu mau{};
    mau.thoiDiemLayMau_us = thoiDiem_us;
    mau.giaTocX_mps2 = giaTocTien;
    mau.giaTocZ_mps2 = 9.80665f;
    mau.daHieuChinhConQuay = true;
    mau.trangThai = TrangThaiKhungDuLieuImu::HopLe;
    return mau;
}
int main() {
    LocThongThap loc;
    kiemTra(loc.capNhat(2,0.005f,40)==2, "loc khoi tao tu mau dau, khong xung gia");
    const float loi = std::numeric_limits<float>::quiet_NaN();
    kiemTra(std::isnan(loc.capNhat(loi,0.005f,40)), "loai NaN");
    kiemTra(loc.capNhat(2,0.005f,40)==2, "NaN khong lam hong trang thai loc");
    loc.datLai(); loc.capNhat(0,0.005f,40);
    double nangLuongVao=0,nangLuongRa=0;
    for (unsigned mau=1;mau<=1200;++mau) {
        const float dauVao=std::sin(2.0*3.141592653589793*50*mau/200.0);
        const float dauRa=loc.capNhat(dauVao,0.005f,40);
        if (mau>200) { nangLuongVao+=dauVao*dauVao; nangLuongRa+=dauRa*dauRa; }
    }
    const double tyLe=std::sqrt(nangLuongRa/nangLuongVao);
    kiemTra(tyLe>0.49 && tyLe<0.52, "suy giam 50Hz PT1 khop tinh toan");
    kiemTra(loc.capNhat(3,0.005f,0)==3, "fc0 bypass");

    ComplementaryEstimator uocLuong{HeTrucImu{MaTranQuayImu{}}};
    TrangThaiXe trangThai{};
    kiemTra(uocLuong.update(taoMau(1000),trangThai), "init IMU");
    for(unsigned mau=1;mau<=40;++mau) {
        kiemTra(uocLuong.update(taoMau(1000+mau*5000,3),trangThai), "cho phep du doan gyro trong nhieu ngan");
        kiemTra(std::fabs(trangThai.gocNghien_rad)<0.001f, "khong keo goc theo gia toc ngang3mps2");
        kiemTra(trangThai.doTinCayGiaToc<0.01f, "innovation loai accel du norm gan g");
    }
    kiemTra(uocLuong.update(taoMau(206000),trangThai) && trangThai.giaTocTinCay,
            "phuc hoi trong so khi gia toc binh thuong");
    bool daBaoLoi=false;
    for(unsigned mau=1;mau<=110;++mau) {
        if (!uocLuong.update(taoMau(206000+mau*5000,3),trangThai)) { daBaoLoi=true; break; }
    }
    kiemTra(daBaoLoi,"mat tin cay keo dai phai bao loi cho app giu latch");

    for(int huong : {-1,1}) {
        AnglePidController pid({30,20,0});
        for(unsigned mau=0;mau<100;++mau) pid.update(huong*0.01f,0,0,0.01f,false);
        const float truoc=pid.update(-huong*0.001f,0,0,0.005f,false,huong);
        float sau=truoc;
        for(unsigned mau=0;mau<100;++mau) sau=pid.update(-huong*0.001f,0,0,0.005f,false,huong);
        kiemTra(huong*(sau-truoc)<-0.009f,"Ki xa tich phan o ca hai tran van toc");
    }
    for(float vanToc : {-0.392699f,-0.3f,0.0f,0.3f,0.392699f}) {
        for(float quay : {-2.0f,0.0f,2.0f}) {
            auto banh=tronUuTienCanBang(vanToc,quay,0.2f,0.392699f);
            kiemTra(std::fabs((banh.trai_mps+banh.phai_mps)*0.5f-vanToc)<1e-6f,
                    "mixer giu nguyen thanh phan can bang khi quay");
            kiemTra(std::fabs(banh.trai_mps)<=0.3927f && std::fabs(banh.phai_mps)<=0.3927f,
                    "hai banh khong vuot tran");
        }
    }
    LenhDiChuyen lenh{0.15f,0,1000,501000,9};
    kiemTra(lenh.hopLe(9,1000) && !lenh.hopLe(10,1000),"lenh di chuyen dung phien");
    kiemTra(!lenh.hopLe(9,999) && !lenh.hopLe(9,501000),"timestamp va timeout di chuyen");
    lenh.hetHan_us=900000;
    kiemTra(!lenh.hopLe(9,1000),"khong chap nhan lease qua500ms");
    CascadeController tang;
    WheelStepCommand buoc{};
    trangThai={}; trangThai.gocHopLe=trangThai.vanTocHopLe=true;
    kiemTra(tang.update(trangThai,1000,false,buoc,0.2f,0.8f) && buoc.tocDoBuocTrai_buocTrenGiay==0,
            "moi phien van khoi dong bang0");
    kiemTra(tang.update(trangThai,6000,false,buoc,0.2f,0.8f),"nhan muc tieu di chuyen");
    kiemTra(std::fabs(tang.vanTocMucTieu_mps()-0.0025f)<1e-6f &&
            std::fabs(tang.tocDoQuayMucTieu_radps()-0.01f)<1e-6f,"slew chi ap lenh nguoi dung");
    for(unsigned mau=2;mau<=100;++mau) tang.update(trangThai,1000+mau*5000,false,buoc,3,3);
    kiemTra(tang.vanTocMucTieu_mps()<=vanTocDiChuyenToiDa_mps &&
            tang.tocDoQuayMucTieu_radps()<=tocDoQuayToiDa_radps,"chan muc tieu di chuyen");
    for(unsigned mau=101;mau<=200;++mau) tang.update(trangThai,1000+mau*5000,false,buoc,0,0);
    kiemTra(std::fabs(tang.vanTocMucTieu_mps())<1e-6f &&
            std::fabs(tang.tocDoQuayMucTieu_radps())<1e-6f,"het lenh giam muc tieu ve0");
    tang.reset();
    kiemTra(tang.vanTocMucTieu_mps()==0 && tang.tocDoQuayMucTieu_radps()==0,"reset xoa muc tieu cu");
    std::printf("PASS: filter/motion/anti-windup %u checks; PT1 50Hz amplitude=%.6f\n",soKiemTra,tyLe);
}
