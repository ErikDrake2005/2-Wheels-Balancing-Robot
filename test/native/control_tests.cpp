#include "AnglePidController.h"
#include "CascadeController.h"
#include "ComplementaryEstimator.h"
#include "DifferentialDrive.h"
#include "safety/SafetyMonitor.h"
#include "config/ControlConfig.h"
#include "config/MotorConfig.h"
#include "config/RobotGeometry.h"
#include "config/SafetyConfig.h"
#include "core/PhienCanBang.h"
#include "core/BoLocThuTuMau.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
int soPhepKiemTra = 0;
void check(bool dieuKien, const char* moTa) {
    ++soPhepKiemTra;
    if (!dieuKien) { std::cerr << "FAIL: " << moTa << '\n'; std::exit(1); }
}
bool near(double giaTriThuNhat, double giaTriThuHai, double saiSoChoPhep = 1e-5) { return std::fabs(giaTriThuNhat-giaTriThuHai) < saiSoChoPhep; }
KhungDuLieuMPU imu(uint64_t thoiDiemMau_us, float gocNghien_rad, float tocDoNghien_radps = 0) {
    KhungDuLieuMPU khungDuLieu{};
    khungDuLieu.thoiDiemLayMau_us = thoiDiemMau_us;
    khungDuLieu.giaTocX_mps2 = -9.80665f*std::sin(gocNghien_rad);
    khungDuLieu.giaTocZ_mps2 = 9.80665f*std::cos(gocNghien_rad);
    khungDuLieu.tocDoGocY_radps = tocDoNghien_radps;
    khungDuLieu.daHieuChinhConQuay = true;
    khungDuLieu.trangThai = TrangThaiKhungDuLieuMpu::HopLe;
    return khungDuLieu;
}
TrangThaiXe validState(uint64_t thoiDiemMau_us) {
    TrangThaiXe trangThaiXe{};
    trangThaiXe.gocHopLe = trangThaiXe.viTriHopLe = trangThaiXe.vanTocHopLe = true;
    trangThaiXe.doLonGiaToc_mps2 = 9.80665f;
    trangThaiXe.giaTocTinCay = true;
    trangThaiXe.thoiDiemGoc_us = trangThaiXe.thoiDiemQuangDuong_us = thoiDiemMau_us;
    return trangThaiXe;
}
void odometryTests() {
    check(near(DifferentialDrive::metersPerStep()*6400, 2*3.141592653589793*0.04),
          "6400 pulses equal one wheel circumference");
    check(near(DifferentialDrive::stepsPerSecond(0.1f)*DifferentialDrive::metersPerStep(), 0.1),
          "velocity / step frequency units");
    DifferentialDrive boQuyDoiBanhXe;
    TrangThaiXe trangThaiXe{};
    check(!boQuyDoiBanhXe.update({0,0,1000,true}, trangThaiXe), "first count has no measured velocity yet");
    for (int chiSoMau = 1; chiSoMau <= 100; ++chiSoMau) {
        check(boQuyDoiBanhXe.update({chiSoMau*10,chiSoMau*10,1000u+chiSoMau*5000u,true}, trangThaiXe), "forward samples accepted");
    }
    const double vanTocMongDoi_mps = 10*DifferentialDrive::metersPerStep()/0.005;
    check(near(trangThaiXe.vanTocDoc_mps, vanTocMongDoi_mps), "filtered speed converges to actual pulse delta");
    check(trangThaiXe.nguonQuangDuong == NguonUocLuongQuangDuong::XungBuoc, "source remains pulse estimate");
    check(boQuyDoiBanhXe.update({990,990,506000,true}, trangThaiXe), "reverse delta accepted");
    check(near(trangThaiXe.viTriDoc_m, 990*DifferentialDrive::metersPerStep()), "reversal subtracts distance");
    check(!boQuyDoiBanhXe.update({990,990,506000,true}, trangThaiXe) && !trangThaiXe.vanTocHopLe, "duplicate pulse sample rejected");
    check(!boQuyDoiBanhXe.update({990,990,550000,true}, trangThaiXe), "long counter gap invalidates speed");
    check(boQuyDoiBanhXe.update({990,990,555000,true}, trangThaiXe) && trangThaiXe.vanTocDoc_mps == 0,
          "stationary sample after gap cannot reuse old speed");
    DifferentialDrive boQuyDoiXungThua;
    check(!boQuyDoiXungThua.update({0,0,1000,true}, trangThaiXe), "sparse baseline");
    check(boQuyDoiXungThua.update({1,1,6000,true}, trangThaiXe) && trangThaiXe.viTriDoc_m > 0, "one sparse pulse is retained");
    check(boQuyDoiXungThua.update({2,0,11000,true}, trangThaiXe), "opposite wheels accepted");
    check(near(trangThaiXe.viTriDoc_m, DifferentialDrive::metersPerStep()), "turn-in-place has no longitudinal increment");
}
void estimatorTests() {
    ComplementaryEstimator boUocLuongGoc{HeTrucImu{MaTranQuayImu{}}};
    TrangThaiXe trangThaiXe{};
    check(boUocLuongGoc.update(imu(1000, 0.1f), trangThaiXe) && near(trangThaiXe.gocNghien_rad, 0.1),
          "positive forward lean and radians");
    for (int chiSoMau = 1; chiSoMau <= 100; ++chiSoMau)
        check(boUocLuongGoc.update(imu(1000+chiSoMau*5000, 0.1f+chiSoMau*0.001f, 0.2f), trangThaiXe), "rotating IMU samples");
    check(near(trangThaiXe.gocNghien_rad, 0.2, 0.001), "gyro and accelerometer agree during rotation");
    auto khungDuLieuLoi = imu(506000, 0);
    khungDuLieuLoi.giaTocX_mps2 = std::numeric_limits<float>::quiet_NaN();
    check(!boUocLuongGoc.update(khungDuLieuLoi, trangThaiXe) && !trangThaiXe.gocHopLe, "NaN IMU rejected");
    check(!boUocLuongGoc.update(imu(600000, 0.1f), trangThaiXe), "IMU gap invalidates angle");
    check(boUocLuongGoc.update(imu(605000, 0.1f), trangThaiXe), "angle reinitialized after gap");
    check(!boUocLuongGoc.update(imu(605000, 0.1f), trangThaiXe), "duplicate IMU timestamp rejected");
    boUocLuongGoc.reset();
    khungDuLieuLoi = imu(610000, 0);
    khungDuLieuLoi.daHieuChinhConQuay = false;
    check(!boUocLuongGoc.update(khungDuLieuLoi, trangThaiXe), "uncalibrated gyro cannot initialize estimator");
}
void safetyTests() {
    auto trangThaiXe = validState(100000);
    check(SafetyMonitor::canArm(trangThaiXe, 100001), "upright fresh state can arm");
    trangThaiXe.gocNghien_rad = gocChoPhepBat_rad + 0.01f;
    check(!SafetyMonitor::canArm(trangThaiXe, 100001), "arm rejects excess initial lean");
    trangThaiXe.gocNghien_rad = gocNga_rad;
    check(SafetyMonitor::check(trangThaiXe, 100001) == FaultCode::Fallen, "fall threshold trips");
    trangThaiXe = validState(100000);
    check(SafetyMonitor::check(trangThaiXe, 100000+tuoiImuToiDa_us) == FaultCode::ImuStale, "IMU deadline is enforced");
    trangThaiXe = validState(100000);
    trangThaiXe.thoiDiemQuangDuong_us = 80000;
    check(SafetyMonitor::check(trangThaiXe, 100001) == FaultCode::PulseStale, "pulse deadline is enforced");
    trangThaiXe = validState(100000);
    check(SafetyMonitor::check(trangThaiXe, 99999) == FaultCode::ImuStale, "future timestamps rejected");
    trangThaiXe.vanTocDoc_mps = std::numeric_limits<float>::infinity();
    check(SafetyMonitor::check(trangThaiXe, 100000) == FaultCode::PulseInvalid, "infinite velocity rejected");
}
void kiemTraTuTheVaDuLieuLoi() {
    for (float gocNgang_rad : {-1.48353f, 1.48353f}) {
        ComplementaryEstimator boUocLuong{HeTrucImu{MaTranQuayImu{}}};
        auto trangThaiXe = validState(100000);
        auto khungDuLieu = imu(100000, 0);
        khungDuLieu.giaTocY_mps2 = 9.80665f * std::sin(gocNgang_rad);
        khungDuLieu.giaTocZ_mps2 = 9.80665f * std::cos(gocNgang_rad);
        check(boUocLuong.update(khungDuLieu, trangThaiXe), "doc duoc tu the nghieng ngang");
        check(SafetyMonitor::check(trangThaiXe,100001) == FaultCode::Fallen,
              "nghieng ngang 85 do phai bao nga");
        check(!SafetyMonitor::canArm(trangThaiXe,100001), "khong bat motor khi nam nghieng");
    }
    ComplementaryEstimator boUocLuong{HeTrucImu{MaTranQuayImu{}}};
    auto trangThaiXe = validState(200000);
    check(boUocLuong.update(imu(200000,0),trangThaiXe), "khoi tao dung thang");
    auto khungLoi = imu(205000,0);
    khungLoi.giaTocZ_mps2 = 0;
    check(!boUocLuong.update(khungLoi,trangThaiXe), "loai mau toan 0 sau khoi tao");
    check(!SafetyMonitor::canArm(trangThaiXe,205001), "mau toan 0 khong duoc arm");
    check(boUocLuong.update(imu(210000,0),trangThaiXe), "phuc hoi uoc luong sau mau loi");
    trangThaiXe = validState(300000);
    trangThaiXe.giaTocTinCay = false;
    check(!SafetyMonitor::canArm(trangThaiXe,300001), "arm can gia toc tin cay");
    trangThaiXe = validState(300000);
    trangThaiXe.tocDoNghiengNgang_radps = 1;
    check(!SafetyMonitor::canArm(trangThaiXe,300001), "arm tu choi dang quay ngang");
}
void kiemTraPhienVaThuTuMau() {
    BoLocThuTuMau boLoc;
    check(boLoc.chapNhan(UINT32_MAX-1), "nhan mau dau");
    check(!boLoc.chapNhan(UINT32_MAX-1), "bo thong bao trung");
    check(boLoc.chapNhan(0), "so thu tu quay vong");
    check(boLoc.soMauBoLo()==1 && boLoc.soThongBaoTrung()==1, "thong ke mat/trung mau");
    check(!boLoc.chapNhan(UINT32_MAX), "tu choi mau cu sau quay vong");
    check(boLoc.chapNhan(1), "mau moi sau thong bao trung van hop le");

    PhienCanBang phien;
    WheelStepCommand lenh{};
    phien.yeuCauBat(100000);
    const uint32_t maPhienCu = phien.maPhien();
    lenh.thoiDiemRaLenh_us = 101000;
    lenh.thoiDiemHetHan_us = 121000;
    check(!phien.lenhKhoiDongHopLe(maPhienCu-1,lenh,101001), "tu choi queue tu phien cu");
    lenh.tocDoBuocTrai_buocTrenGiay = 100;
    check(!phien.lenhKhoiDongHopLe(maPhienCu,lenh,101001), "lenh khoi dong phai bang 0");
    lenh.tocDoBuocTrai_buocTrenGiay = 0;
    check(phien.lenhKhoiDongHopLe(maPhienCu,lenh,101001), "nhan lenh 0 dung phien");
    check(phien.xacNhanDaBat(), "chuyen sang dang chay");
    phien.tat();
    phien.yeuCauBat(101500); // Tắt/bật giữa hai chu kỳ điều khiển 5 ms.
    check(!phien.lenhKhoiDongHopLe(maPhienCu,lenh,101501), "bat nhanh khong dung lai lenh cu");
    check(!phien.lenhKhoiDongHopLe(phien.maPhien(),lenh,101501), "tu choi timestamp truoc yeu cau");
    CascadeController boDieuKhien;
    auto trangThaiXe = validState(100000);
    boDieuKhien.update(trangThaiXe,100000,true,lenh);
    trangThaiXe.gocNghien_rad = 0.03f;
    boDieuKhien.update(trangThaiXe,105000,true,lenh);
    check(lenh.tocDoBuocTrai_buocTrenGiay>0, "tao lenh dang chay truoc reset");
    boDieuKhien.reset();
    check(boDieuKhien.update(trangThaiXe,110000,true,lenh), "reset tao lenh khoi dong");
    check(phien.lenhKhoiDongHopLe(phien.maPhien(),lenh,110001), "ack 0 sau reset duoc chap nhan");
    check(phien.quaHanCho(201500), "cho ack qua han");
    check(!phien.lenhKhoiDongHopLe(phien.maPhien(),lenh,201500), "khong bat khi ack het han");
    phien.tat();
    check(!phien.xacNhanDaBat(), "khong tu bat khi khong cho");
}
void controllerTests() {
    AnglePidController boPidGoc;
    check(boPidGoc.update(0.01f,0,0,0.005f,true) > 0, "forward lean commands forward acceleration");
    check(boPidGoc.update(-0.01f,0,0,0.005f,true) < 0, "backward lean commands reverse acceleration");
    for (int chiSoSaiLech = -30; chiSoSaiLech <= 30; ++chiSoSaiLech) for (int chiSoTocDo = -30; chiSoTocDo <= 30; ++chiSoTocDo) {
        auto heSoPid = AnglePidController::scheduledGains(chiSoSaiLech*0.02f, chiSoTocDo*0.2f, true);
        check(heSoPid.heSoTiLe >= kpGoc-1e-5f && heSoPid.heSoTiLe <= kpGoc*1.4f+1e-5f &&
              heSoPid.heSoViPhan >= kdGoc-1e-5f && heSoPid.heSoViPhan <= kdGoc*1.5f+1e-5f,
              "fuzzy gains bounded over input domain and shoulders");
        check(std::fabs(boPidGoc.update(chiSoSaiLech*0.02f,chiSoTocDo*0.2f,0,0.005f,true)) <= giaTocCanBangToiDa_mps2,
              "acceleration stays bounded");
    }
    check(std::isnan(boPidGoc.update(0,0,0,0,true)), "invalid dt cannot emit acceleration");
    CascadeController boDieuKhienTang;
    WheelStepCommand lenhBanh{};
    auto trangThaiXe = validState(1000);
    check(boDieuKhienTang.update(trangThaiXe,1000,true,lenhBanh), "first cascade sample");
    trangThaiXe.gocNghien_rad = 0.4f;
    for (unsigned chiSoMau=1; chiSoMau<=1000; ++chiSoMau) {
        check(boDieuKhienTang.update(trangThaiXe,1000+chiSoMau*5000u,true,lenhBanh), "cascade regular cadence");
        check(std::fabs(lenhBanh.tocDoBuocTrai_buocTrenGiay) <= tanSoStepToiDa_hz+0.01f,
              "integrated velocity never exceeds driver pulse ceiling");
    }
    boDieuKhienTang.reset();
    check(boDieuKhienTang.update(validState(10000000),10000000,true,lenhBanh) &&
          lenhBanh.tocDoBuocTrai_buocTrenGiay == 0, "rearm resets accumulated wheel command");
    check(!boDieuKhienTang.update(trangThaiXe,10000000,true,lenhBanh), "repeated controller time rejected");
    check(!boDieuKhienTang.update(trangThaiXe,11000000,true,lenhBanh), "control timing gap rejected");
}
// Ideal point-mass inverted pendulum; checks units/sign and recovery under this model only.
void plantTest(bool dungPidMo) {
    AnglePidController boPidGoc;
    double gocNghien_rad=0.02, tocDoNghien_radps=0;
    constexpr double buocThoiGian_s=0.005, buocMoPhong_s=buocThoiGian_s/10;
    for (int chiSoMau=0; chiSoMau<400; ++chiSoMau) {
        const double giaToc_mps2=boPidGoc.update(static_cast<float>(gocNghien_rad),static_cast<float>(tocDoNghien_radps),0,buocThoiGian_s,dungPidMo);
        for(int chiSoBuocNho=0;chiSoBuocNho<10;++chiSoBuocNho) {
            tocDoNghien_radps += (9.80665*std::sin(gocNghien_rad)-giaToc_mps2*std::cos(gocNghien_rad))/toaDoTrongTamZ_m*buocMoPhong_s;
            gocNghien_rad += tocDoNghien_radps*buocMoPhong_s;
        }
        check(std::isfinite(gocNghien_rad) && std::fabs(gocNghien_rad)<0.05, "ideal pendulum remains bounded");
    }
    check(std::fabs(gocNghien_rad)<0.0001 && std::fabs(tocDoNghien_radps)<0.001, "ideal pendulum recovers from initial lean");
}
void cascadePlantTest(bool dungPidMo) {
    CascadeController boDieuKhienTang;
    DifferentialDrive boQuyDoiBanhXe;
    TrangThaiXe trangThaiXe{};
    WheelStepCommand lenhBanh{};
    double gocNghien_rad=0.01, tocDoNghien_radps=0, vanToc_mps=0, viTri_m=0;
    constexpr double buocThoiGian_s=0.005, buocMoPhong_s=buocThoiGian_s/10;
    for (unsigned chiSoMau=0; chiSoMau<1000; ++chiSoMau) {
        const uint64_t thoiDiemMau_us=1000+chiSoMau*5000u;
        trangThaiXe.gocHopLe=true;
        trangThaiXe.gocNghien_rad=static_cast<float>(gocNghien_rad);
        trangThaiXe.tocDoNghien_radps=static_cast<float>(tocDoNghien_radps);
        const int64_t soXung=static_cast<int64_t>(std::llround(viTri_m/boQuyDoiBanhXe.metersPerStep()));
        if (!boQuyDoiBanhXe.update({soXung,soXung,thoiDiemMau_us,true},trangThaiXe)) continue;
        check(boDieuKhienTang.update(trangThaiXe,thoiDiemMau_us,dungPidMo,lenhBanh), "cascade accepts quantized pulse odometry");
        const double vanTocTiepTheo_mps=lenhBanh.tocDoBuocTrai_buocTrenGiay*boQuyDoiBanhXe.metersPerStep();
        const double giaToc_mps2=(vanTocTiepTheo_mps-vanToc_mps)/buocThoiGian_s;
        for(int chiSoBuocNho=0;chiSoBuocNho<10;++chiSoBuocNho) {
            tocDoNghien_radps += (9.80665*std::sin(gocNghien_rad)-giaToc_mps2*std::cos(gocNghien_rad))/toaDoTrongTamZ_m*buocMoPhong_s;
            gocNghien_rad += tocDoNghien_radps*buocMoPhong_s;
        }
        viTri_m += (vanToc_mps+vanTocTiepTheo_mps)*0.5*buocThoiGian_s;
        vanToc_mps=vanTocTiepTheo_mps;
        check(std::isfinite(gocNghien_rad) && std::fabs(gocNghien_rad)<0.1, "cascade remains upright in ideal plant");
    }
    check(std::fabs(gocNghien_rad)<0.01 && std::fabs(vanToc_mps)<0.05, "cascade limits drift after initial lean");
}
// H3/H1/M4: con lắc hiệu dụng L_eff có trễ truyền 10 ms (2 chu kỳ mẫu), chịu mất mẫu lẻ và bù R*thetaDot.
void delayedEffectivePendulumTest(bool dungPidMo) {
    AnglePidController boPidGoc;
    double gocNghien_rad = 0.03, tocDoNghien_radps = 0;
    double hangDoiGiaToc_mps2[2] = {0.0, 0.0}; // Trễ 2 chu kỳ = 10 ms.
    constexpr double buocThoiGian_s = 0.005, buocMoPhong_s = buocThoiGian_s / 10;
    for (int chiSoMau = 0; chiSoMau < 600; ++chiSoMau) {
        const double giaTocLenh_mps2 = boPidGoc.update(
            static_cast<float>(gocNghien_rad), static_cast<float>(tocDoNghien_radps), 0, buocThoiGian_s, dungPidMo);
        const double giaTocThuc_mps2 = hangDoiGiaToc_mps2[0];
        hangDoiGiaToc_mps2[0] = hangDoiGiaToc_mps2[1];
        hangDoiGiaToc_mps2[1] = giaTocLenh_mps2;
        for (int chiSoBuocNho = 0; chiSoBuocNho < 10; ++chiSoBuocNho) {
            tocDoNghien_radps += (9.80665 * std::sin(gocNghien_rad) - giaTocThuc_mps2 * std::cos(gocNghien_rad))
                / chieuDaiConLacHieuDung_m * buocMoPhong_s;
            gocNghien_rad += tocDoNghien_radps * buocMoPhong_s;
        }
        check(std::isfinite(gocNghien_rad) && std::fabs(gocNghien_rad) < 0.08,
              "delayed L_eff pendulum remains bounded");
    }
    check(std::fabs(gocNghien_rad) < 0.001 && std::fabs(tocDoNghien_radps) < 0.01,
          "delayed L_eff pendulum recovers from initial lean");

    // H1 + M4: mất 1–2 mẫu IMU (khoảng cách 10–15 ms) vẫn giữ vận tốc hợp lệ; bù R*thetaDot đúng dấu.
    DifferentialDrive boQuyDoi;
    TrangThaiXe trangThai{};
    trangThai.gocHopLe = true;
    trangThai.gocNghien_rad = 0.05f;
    trangThai.tocDoNghien_radps = 1.0f;
    check(!boQuyDoi.update({0, 0, 1000, true}, trangThai), "baseline pulse frame");
    check(near(trangThai.viTriDoc_m, banKinhBanh_m * 0.05f, 1e-6), "M4 body angle added to ground position");
    check(boQuyDoi.update({0, 0, 13000, true}, trangThai) && trangThai.vanTocHopLe,
          "H1 12 ms gap (dropped 1 IMU sample + jitter) keeps velocity valid");
    check(trangThai.vanTocDoc_mps > 0.0f, "M4 body pitch rate contributes positive ground speed when stator rotates with body");
}
}
int main() {
    odometryTests(); estimatorTests(); safetyTests(); controllerTests();
    kiemTraTuTheVaDuLieuLoi(); kiemTraPhienVaThuTuMau();
    plantTest(false); plantTest(true); cascadePlantTest(false); cascadePlantTest(true);
    delayedEffectivePendulumTest(false); delayedEffectivePendulumTest(true);
    std::cout << "PASS: odometry, IMU fusion, safety, fuzzy PID, cascade, ideal/delayed plant (fixed/fuzzy); "
              << soPhepKiemTra << " checks\n";
}
