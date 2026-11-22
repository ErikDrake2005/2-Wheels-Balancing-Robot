#include "LocThongThap.h"
#include "ComplementaryEstimator.h"
#include "AnglePidController.h"
#include "BoDieuKhienBanhVongKin.h"
#include "core/BoGiaiMaUartMks.h"
#include "CascadeController.h"
#include "DifferentialDrive.h"
#include "BoTronLenhBanh.h"
#include "model/GoiLenhNhiPhan.h"
#include "model/LenhDiChuyen.h"
#include "config/ControlConfig.h"
#include <cstring>
#include "config/MotorConfig.h"
#include "config/PinConfig.h"
#include "config/RobotGeometry.h"
#include "config/SafetyConfig.h"
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
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
void kiemTraChuyenGiuHuongLSM(unsigned chuKy_us) {
    CascadeController boRe, boDiThang;
    TrangThaiXe xe{};
    xe.gocHopLe = xe.viTriHopLe = xe.vanTocHopLe = true;
    uint64_t t = 1000;
    const float buocQuay = giaTocLenhQuay_radps2 * chuKy_us * 1e-6f;
    const double metMoiBuoc = DifferentialDrive::metersPerStep();
    const float vMax = tanSoStepToiDa_hz * metMoiBuoc;
    float quayTruoc = 0;
    auto chay = [&](float re, bool coTheMatDuDia = false) {
        t += chuKy_us;
        WheelStepCommand banh{}, thang{};
        TrangThaiXe xeThang = xe;
        xeThang.vanTocBanhPhai_mps = xeThang.vanTocBanhTrai_mps = 0;
        kiemTra(boRe.update(xe,t,false,banh,0,re) && boDiThang.update(xeThang,t,false,thang),
                "hai bo dieu khien cung nhan mau");
        const double vl = banh.tocDoBuocTrai_buocTrenGiay * metMoiBuoc;
        const double vr = banh.tocDoBuocPhai_buocTrenGiay * metMoiBuoc;
        const double vCanBang = thang.tocDoBuocTrai_buocTrenGiay * metMoiBuoc;
        kiemTra(std::fabs((vl + vr) / 2 - vCanBang) < 1e-6 &&
                std::max(std::fabs(vl), std::fabs(vr)) <= vMax + 1e-6,
                "giu nguyen tac dong PID pitch va tran van toc tung banh khi re");
        const float quay = static_cast<float>((vr-vl) / khoangCachTamHaiBanh_m);
        if (!coTheMatDuDia)
            kiemTra(std::fabs(quay-quayTruoc) <= buocQuay + 2e-6f,
                    "gioi han gia toc quay tai dau ra khi chuyen giu huong/re");
        quayTruoc = quay;
        return quay;
    };

    chay(0);
    xe.vanTocBanhPhai_mps = 0.4f;
    xe.vanTocBanhTrai_mps = -0.4f;
    for (unsigned i=0; i<75; ++i) chay(0);
    kiemTra(std::fabs(quayTruoc + gioiHanTocDoQuayGiuHuong) < 2e-6f, "dat muc bu giu huong");
    for (unsigned i=0; i<150; ++i) chay(0.5f);
    kiemTra(std::fabs(quayTruoc - .5f) < 2e-6f, "lenh re nguoi dung dat chinh xac");
    for (unsigned i=0; i<150; ++i) chay(0); // Nhả cần hoặc lease hết hạn.
    kiemTra(std::fabs(quayTruoc + gioiHanTocDoQuayGiuHuong) < 2e-6f, "nhan lai giu huong");

    xe.gocNghien_rad = .25f; // Tạo bão hòa vận tốc chung rồi trả lại dư địa rẽ.
    for (unsigned i=0; i<90; ++i) chay(.5f,true);
    kiemTra(std::fabs(quayTruoc) < 2e-6f, "nhuong het du dia cho pitch");
    xe.gocNghien_rad = -.25f;
    for (unsigned i=0; i<12; ++i) chay(.5f);

    boRe.reset();
    boDiThang.reset();
    xe = {};
    xe.gocHopLe = xe.viTriHopLe = xe.vanTocHopLe = true;
    quayTruoc = 0;
    chay(0);
    kiemTra(std::fabs(chay(.5f) - buocQuay) < 2e-6f, "phien moi re tu zero, khong them tre cho lenh tay");
}
#endif
int main() {
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
    kiemTraChuyenGiuHuongLSM(4167);
    kiemTraChuyenGiuHuongLSM(5000);
#endif
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

    // Sai lệch clock cảm biến quanh 200 Hz vẫn phải giữ lưới vòng ngoài 50 Hz.
    // Kiv * sai lệch vận tốc cố định cho phép quan sát từng lần cập nhật góc đặt.
    if (dungProfileImuRieng) for (unsigned chuKyImu_us : {4999u, 5000u, 5001u}) {
        CascadeController boThu;
        boThu.setPidVanTocThuan(0.0f, 0.01f);
        TrangThaiXe xe{};
        xe.gocHopLe = xe.vanTocHopLe = true;
        xe.vanTocDoc_mps = -0.01f;
        WheelStepCommand lenh{};
        uint64_t t = 1000, mocVongNgoai = t;
        float gocTruoc = 0;
        unsigned soLanVongNgoai = 0;
        boThu.update(xe, t, false, lenh);
        for (unsigned i = 0; i < 400; ++i) {
            t += chuKyImu_us;
            kiemTra(boThu.update(xe, t, false, lenh), "nhan mau IMU lech clock nhe");
            if (boThu.gocDat_rad() != gocTruoc) {
                ++soLanVongNgoai;
                mocVongNgoai = t;
                gocTruoc = boThu.gocDat_rad();
            }
        }
        kiemTra(soLanVongNgoai == (t - 1000) / chuKyVongVanToc_us,
                "vong ngoai giu du so ky 50Hz, khong mat phan du");
        const float gocTinhToan = 0.0001f * (mocVongNgoai - 1000) * 1e-6f;
        kiemTra(std::fabs(gocTruoc - gocTinhToan) < 1e-7f,
                "tich phan vong ngoai dung dt thuc sau khi giu luoi thoi gian");
    }

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

    // A/B chỉ tác động tạo lệnh ngoài, không sửa PID góc hoặc gia tốc cứu cân bằng.
    for (bool tangBam : {false,true}) {
        CascadeController thu;
        thu.setThuNghiemDiChuyen(false,tangBam);
        thu.setPidVanTocThuan(0.15f,0);
        auto xe=trangThai; xe.vanTocDoc_mps=0;
        for (unsigned k=0;k<=4;++k)
            kiemTra(thu.update(xe,1000+k*5000,false,buoc,0.1f,0),"A/B update hop le");
        kiemTra(std::fabs(thu.vanTocMucTieu_mps()-.01f)<1e-6f,"tat accelboost giu ramp 0.5m/s2 sau mau dau");
        const float factor=tangBam?2.2f:1.0f;
        kiemTra(std::fabs(thu.gocDat_rad()-(gocCanBang_rad+.15f*factor*.01f))<1e-6f,
                "co vgainboost doi dung thanh phan bam van toc");
        thu.reset();
        kiemTra(!thu.tangGiaTocDiChuyen() && thu.tangBamVanTocDiChuyen()==tangBam,"reset khong doi lua chon A/B");
    }

    // M5: khi truyền gia tốc tịnh tiến đã lệnh vào bộ ước lượng, accel không bị kéo lệch góc.
    ComplementaryEstimator uocLuongBu{HeTrucImu{MaTranQuayImu{}}};
    TrangThaiXe trangThaiBu{};
    kiemTra(uocLuongBu.update(taoMau(1000,0),trangThaiBu,0),"init estimator bu gia toc");
    for(unsigned mau=1;mau<=60;++mau) {
        kiemTra(uocLuongBu.update(taoMau(1000+mau*5000,3.0f),trangThaiBu,3.0f),
                "M5 bu gia toc 3 m/s2 khong mat tin cay");
        kiemTra(trangThaiBu.giaTocTinCay && trangThaiBu.doTinCayGiaToc>0.95f,
                "M5 giu do tin cay accel cao khi gia toc lenh khop");
        kiemTra(std::fabs(trangThaiBu.gocTuGiaToc_rad)<0.001f && std::fabs(trangThaiBu.gocNghien_rad)<0.001f,
                "M5 khong lech goc khi tang toc 3 m/s2");
    }

    // Nếu gia tốc lệnh khác chuyển động thật, phần bù có thể tạo lệch góc dù gyro đứng yên.
    ComplementaryEstimator buSai{HeTrucImu{MaTranQuayImu{}}}, khongBu{HeTrucImu{MaTranQuayImu{}}};
    TrangThaiXe xeBuSai{}, xeKhongBu{};
    for (unsigned k=0;k<=200;++k) {
        auto mau=taoMau(1000+k*5000,0);
        kiemTra(buSai.update(mau,xeBuSai,.2f) && khongBu.update(mau,xeKhongBu,0),"mo phong sai so gia toc lenh hop le");
    }
    kiemTra(std::fabs(xeBuSai.gocNghien_rad)>.01f && std::fabs(xeKhongBu.gocNghien_rad)<.001f,
            "bai tong hop: bu sai gia toc lenh khong duoc coi la do goc doc lap");

    // Kiểm tra bộ điều khiển Mamdani (MIN - MAX - Centroid/CoG):
    {
        const PidGains coSo{30.0f, 10.0f, 3.0f};
        const auto mamdaniTam = AnglePidController::scheduledGains(0.0f, 0.0f, true, coSo, KieuSuyLuanMo::MamdaniCentroid);
        kiemTra(std::fabs(mamdaniTam.heSoTiLe - 30.0f) < 1e-4f &&
                std::fabs(mamdaniTam.heSoTichPhan - 10.0f) < 1e-4f &&
                std::fabs(mamdaniTam.heSoViPhan - 3.0f) < 1e-4f,
                "Mamdani tai (Z, Z) tra ve dung he so co so (Kp=1.0x, Ki=1.0x, Kd=1.0x)");
        const auto mamdaniNga = AnglePidController::scheduledGains(0.07f, 0.70f, true, coSo, KieuSuyLuanMo::MamdaniCentroid);
        kiemTra(std::fabs(mamdaniNga.heSoTiLe - 42.0f) < 1e-4f &&
                std::fabs(mamdaniNga.heSoTichPhan - 0.0f) < 1e-4f &&
                std::fabs(mamdaniNga.heSoViPhan - 3.375f) < 1e-4f,
                "Mamdani tai (PB, PB) tang Kp len 1.4x=42, cat Ki=0, Kd=1.125x=3.375");
        const auto mamdaniPhanh = AnglePidController::scheduledGains(0.07f, -0.70f, true, coSo, KieuSuyLuanMo::MamdaniCentroid);
        kiemTra(std::fabs(mamdaniPhanh.heSoTiLe - 30.0f) < 1e-4f &&
                std::fabs(mamdaniPhanh.heSoViPhan - 4.5f) < 1e-4f,
                "Mamdani tai (PB, NB) tang Kd len 1.5x=4.5 de phanh som");

        // Kiểm tra 2 chế độ độc lập: PID Mờ (dungPidMo=true) vs PID Thuần (dungPidMo=false) và luân chuyển êm (bumpless):
        AnglePidController boPidHaiCheDo({30.0f, 0.0f, 3.0f}, {28.0f, 0.5f, 2.5f});
        boPidHaiCheDo.update(0.07f, 0.70f, 0.0f, 0.005f, false);
        kiemTra(std::fabs(boPidHaiCheDo.heSoPid().heSoTiLe - 28.0f) < 1e-4f &&
                std::fabs(boPidHaiCheDo.heSoPid().heSoTichPhan - 0.5f) < 1e-4f &&
                std::fabs(boPidHaiCheDo.heSoPid().heSoViPhan - 2.5f) < 1e-4f,
                "Che do PID Thuan giu nguyen bo he so doc lap (28.0, 0.5, 2.5) khong qua suy luan mo");
        boPidHaiCheDo.update(0.07f, 0.70f, 0.0f, 0.005f, true);
        kiemTra(boPidHaiCheDo.heSoPid().heSoTiLe > 35.0f,
                "Luan chuyen sang che do PID Mo kich hoat ngay suy luan Mamdani");
    }

    // Kiểm tra giải mã UART MKS Servo42C V1.1 (0x30 carry + value 14-bit 16384 xung/vòng):
    {
        uint8_t khungHoi[3]{};
        BoGiaiMaUartMks::taoKhungHoi(0xE0, 0x30, khungHoi);
        kiemTra(khungHoi[0] == 0xE0 && khungHoi[1] == 0x30 && khungHoi[2] == 0x10,
                "MKS V1.1 lenh 0x30 CRC8 dung [E0 30 10]");
        // Giả lập trả lời 8 byte: carry = 2 vòng, value = 4096 (0x1000 = 1/4 vòng) -> tổng = 2*16384 + 4096 = 36864
        uint8_t khungTraLoi[8] = {0xE0, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x00};
        khungTraLoi[7] = BoGiaiMaUartMks::tinhCrc8(khungTraLoi, 7);
        const auto kqMks = BoGiaiMaUartMks::giaiMaEncoder0x30(khungTraLoi, 8, 0xE0, 16384);
        kiemTra(kqMks.hopLe && kqMks.soVongCarry == 2 && kqMks.gocTrongVong == 4096 &&
                kqMks.tongXungEncoder == 36864,
                "Giai ma MKS V1.1 0x30 carry=2 value=4096 chinh xac");
        // Quy đổi 2.25 vòng từ kế (36864 / 16384) sang xung bước MStep=32 (6400 xung/vòng) = 14400 xung
        kiemTra(BoGiaiMaUartMks::quyDoiSangXungBuoc(kqMks.tongXungEncoder, 6400, 16384) == 14400,
                "Quy doi 16384 xung tu ke MKS sang 6400 xung buoc chinh xac");

        // Đổi vi bước đầu vào không đổi định dạng encoder 0x30 (65536 đơn vị/vòng).
        kiemTra(soXungEncoderMksMoiVong == 65536u && diaChiUartMksTrai == 0xE1u &&
                diaChiUartMksPhai == 0xE2u && soViBuocDauVaoMks == 64u && soViBuocTmc2209 == 32u &&
                cheDoDieuKhienKhoiDong == CheDoDieuKhien::PidThuan,
                "MKS 64 vi buoc, TMC 32 vi buoc, encoder MKS van 65536/vong va PID_THUAN");
        const uint32_t soBuocMoiVongMks = soBuocDayDuMoiVongDongCo * soViBuocDauVaoMks;
        kiemTra(BoGiaiMaUartMks::quyDoiSangXungBuoc(65536, soBuocMoiVongMks) == 12800 &&
                BoGiaiMaUartMks::quyDoiSangXungBuoc(-16384, soBuocMoiVongMks) == -3200,
                "Encoder MKS mot vong va am mot phan tu vong doi dung sang STEP 64 vi buoc");
        const double soBuocMoiVongDangDung = dungPhanCungTmcMt ? 6400.0 : 12800.0;
        const double tanSoThamChieu_hz = dungPhanCungTmcMt ? 10000.0 : 20000.0;
        kiemTra(std::fabs(DifferentialDrive::metersPerStep() * soBuocMoiVongDangDung -
                        2.0 * 3.141592653589793 * banKinhBanh_m) < 1e-7 &&
                std::fabs(tanSoThamChieu_hz * DifferentialDrive::metersPerStep() - 0.3926990817) < 1e-7,
                "Moi cau hinh giu dung chu vi banh va moc quy doi van toc 0.392699 m/s");
        uint8_t khungThuc[8] = {0xE1, 0xFF, 0xFF, 0xFF, 0xFF, 0xC6, 0x66, 0x09};
        const auto kqThuc = BoGiaiMaUartMks::giaiMaEncoder0x30(khungThuc, 8, 0xE1);
        kiemTra(kqThuc.hopLe && kqThuc.soVongCarry == -1 && kqThuc.gocTrongVong == 0xC666u &&
                kqThuc.tongXungEncoder == -14746 &&
                BoGiaiMaUartMks::quyDoiSangXungBuoc(-16384, 6400) == -1600,
                "Giai ma khung thuc te MKS V1.1 E1 FF FF FF FF C6 66 09 (65536/vong) chinh xac");
    }

    // Kiểm tra chế độ nhấc bánh không ma sát (Bench Mode) + cập nhật hệ số PID_THUAN runtime:
    {
        CascadeController boBench;
        DifferentialDrive odoBench;
        boBench.setPidGocThuan({40.0f, 0.0f, 3.0f});
        boBench.setGocCanBang_rad(0.02f);
        boBench.setCheDoThuKhongMaSat(true);
        boBench.setBatVongNgoai(false);

        TrangThaiXe xe{};
        WheelStepCommand lenh{};
        double viTriThuc_m = 0.0;
        const auto nguonOdo = dungPhanCungTmcMt
            ? NguonUocLuongQuangDuong::EncoderBanh
            : NguonUocLuongQuangDuong::XungBuoc;
        const double metMoiXungFeedback = DifferentialDrive::metersPerFeedbackPulse(nguonOdo);
        const double metMoiStep = DifferentialDrive::metersPerStep();
        // Giữ nghiêng +0.03 rad so với điểm cân bằng 0.02 rad (theta = 0.05 rad) trong 1s:
        for (unsigned k = 0; k < 200; ++k) {
            const uint64_t t_us = 1000u + k * 5000u;
            xe.gocHopLe = true;
            xe.gocNghien_rad = 0.05f;
            xe.tocDoNghien_radps = 0.0f;
            const int64_t tongXung = static_cast<int64_t>(std::llround(viTriThuc_m / metMoiXungFeedback));
            if (!odoBench.update({tongXung, tongXung, t_us, true}, xe, nguonOdo)) continue;
            kiemTra(boBench.update(xe, t_us, false, lenh), "Bench Mode update hop le");
            viTriThuc_m += static_cast<double>(lenh.tocDoBuocTrai_buocTrenGiay) * metMoiStep * 0.005;
        }
        const double vanTocBench_mps = lenh.tocDoBuocTrai_buocTrenGiay * metMoiStep;
        kiemTra(std::fabs(boBench.gocDat_rad() - 0.02f) < 1e-5f &&
                vanTocBench_mps > 0.0589 && vanTocBench_mps < 0.2357 &&
                std::fabs(lenh.tocDoBuocTrai_buocTrenGiay) < tanSoStepToiDa_hz,
                "Bench Mode giu van toc vat ly ti le goc nghieng va khong bao hoa STEP");

        // Trả xe về đúng gocCanBang_rad = 0.02f: bánh phải tự hãm dần về 0 STEP/s:
        for (unsigned k = 200; k < 420; ++k) {
            const uint64_t t_us = 1000u + k * 5000u;
            xe.gocNghien_rad = 0.02f;
            const int64_t tongXung = static_cast<int64_t>(std::llround(viTriThuc_m / metMoiXungFeedback));
            odoBench.update({tongXung, tongXung, t_us, true}, xe, nguonOdo);
            kiemTra(boBench.update(xe, t_us, false, lenh), "Bench Mode return update hop le");
            viTriThuc_m += static_cast<double>(lenh.tocDoBuocTrai_buocTrenGiay) * metMoiStep * 0.005;
        }
        kiemTra(std::fabs(lenh.tocDoBuocTrai_buocTrenGiay * metMoiStep) < 0.001,
                "Bench Mode tu ham banh ve duoi 1 mm/s khi tro ve goc can bang");
    }

    // Kiểm tra chống trôi vị trí khi lệch trọng tâm cơ khí (CoM Auto-Trim + Mamdani Position Lock):
    {
        CascadeController boDieuKhien;
        DifferentialDrive boQuyDoi;
        TrangThaiXe xe{};
        WheelStepCommand lenhBanh{};
        const double gocTrongTamThuc_rad = 0.008; // Trọng tâm thực lệch ~0,46 độ so với gocCanBang_rad=0
        double gocNghien_rad = gocTrongTamThuc_rad, tocDoNghien_radps = 0.0, vanToc_mps = 0.0, viTri_m = 0.0;
        constexpr double dt_s = 0.005, dtCon_s = dt_s / 10.0;
        const auto nguonOdo = dungPhanCungTmcMt
            ? NguonUocLuongQuangDuong::EncoderBanh : NguonUocLuongQuangDuong::XungBuoc;
        const double metMoiXungFeedback = DifferentialDrive::metersPerFeedbackPulse(nguonOdo);
        for (unsigned mau = 0; mau < 2400; ++mau) { // 12 giây mô phỏng
            const uint64_t t_us = 1000u + mau * 5000u;
            xe.gocHopLe = true;
            xe.gocNghien_rad = static_cast<float>(gocNghien_rad);
            xe.tocDoNghien_radps = static_cast<float>(tocDoNghien_radps);
            const int64_t soXung = static_cast<int64_t>(std::llround(viTri_m / metMoiXungFeedback));
            if (!boQuyDoi.update({soXung, soXung, t_us, true}, xe)) continue;
            kiemTra(boDieuKhien.update(xe, t_us, true, lenhBanh), "Mamdani CoM Auto-Trim update hop le");
            const double vanTocTiep_mps = lenhBanh.tocDoBuocTrai_buocTrenGiay * boQuyDoi.metersPerStep();
            const double giaToc_mps2 = (vanTocTiep_mps - vanToc_mps) / dt_s;
            for (int k = 0; k < 10; ++k) {
                tocDoNghien_radps += (9.80665 * std::sin(gocNghien_rad - gocTrongTamThuc_rad) -
                                      giaToc_mps2 * std::cos(gocNghien_rad - gocTrongTamThuc_rad)) /
                                     toaDoTrongTamZ_m * dtCon_s;
                gocNghien_rad += tocDoNghien_radps * dtCon_s;
            }
            viTri_m += 0.5 * (vanToc_mps + vanTocTiep_mps) * dt_s;
            vanToc_mps = vanTocTiep_mps;
        }
        // Nhờ khóa vị trí trực tiếp + Mamdani tăng độ cứng vị trí, vận tốc xác lập hội tụ về 0 (không trôi tự do)
        // và độ lệch vị trí tĩnh bị ghìm chặt trong phạm vi vài cm quanh điểm gốc.
        kiemTra(std::fabs(viTri_m) < 0.12 && std::fabs(vanToc_mps) < 0.005,
                "Khoa vi tri Mamdani triet tieu troi van toc khi lech trong tam co khi");
    }

    // TMC2209 + MT6701: kiểm tra quy đổi 4096 xung/vòng và bộ điều khiển bánh xe vòng kín.
    kiemTra(std::fabs(DifferentialDrive::metersPerEncoderCount()*4096.0 - 2.0*3.141592653589793*banKinhBanh_m) < 1e-6,
            "MT6701 4096 xung/vong bang chu vi banh xe");
    DifferentialDrive odoEncoder;
    TrangThaiXe xeEnc{};
    kiemTra(!odoEncoder.update({1000,1000,1000,true}, xeEnc, NguonUocLuongQuangDuong::EncoderBanh),
            "mau encoder dau chi khoi tao vi tri");
    kiemTra(odoEncoder.update({1000,1000,6000,true}, xeEnc, NguonUocLuongQuangDuong::EncoderBanh) &&
            xeEnc.nguonQuangDuong == NguonUocLuongQuangDuong::EncoderBanh,
            "mau encoder thu hai co van toc hop le");

    BoDieuKhienBanhVongKin vongKin;
    TrangThaiVongKinBanhXe kqVongKin{};
    // Dù bánh đã bị lăn tay tới vị trí khác 0 khi motor tắt, lần đầu cập nhật phải đồng bộ sai lệch = 0.
    kiemTra(vongKin.capNhat(0.0f, 0.0f, xeEnc, 0.005f, 0.3927f, kqVongKin),
            "dong bo vi tri tham chieu theo encoder khi bat dau");
    kiemTra(std::fabs(kqVongKin.saiLechViTriTrai_m) < 1e-6f && std::fabs(kqVongKin.vanTocLenhTrai_mps) < 1e-6f,
            "khong giat banh khi arm sau khi xe bi day tay");

    // Khi bánh trái bị hụt bước (encoder đứng yên trong khi lệnh yêu cầu tiến 0.10 m/s),
    // bộ vòng kín phải tăng vận tốc lệnh bánh trái cao hơn feedforward để bù góc.
    kiemTra(vongKin.capNhat(0.10f, 0.10f, xeEnc, 0.005f, 0.3927f, kqVongKin),
            "cap nhat bu truot buoc ngan han");
    kiemTra(kqVongKin.vanTocLenhTrai_mps > 0.10f && kqVongKin.vanTocBuTrai_mps > 0.0f,
            "vong kin tang xung STEP khi encoder cham hon lenh");

    // Nếu rotor bị kẹt cứng kéo dài quá thoiGianKetBanhToiDa_s (150 ms), phải báo biKetBanh để ngắt ENN.
    bool daPhatHienKet = false;
    for (unsigned mau = 0; mau < 80; ++mau) {
        if (!vongKin.capNhat(0.20f, 0.20f, xeEnc, 0.005f, 0.3927f, kqVongKin) && kqVongKin.biKetBanh) {
            daPhatHienKet = true;
            break;
        }
    }
    kiemTra(daPhatHienKet, "phat hien ket rotor / mat buoc keo dai tren TMC2209+MT6701");

    // 1. Kiem tra bo loc goc nghieng ngang chong ngat gia fault=fallen do xung rung 5ms:
    {
        ComplementaryEstimator uocLuongRung{HeTrucImu{MaTranQuayImu{}}};
        TrangThaiXe trangThaiRung{};
        kiemTra(uocLuongRung.update(taoMau(1000), trangThaiRung), "khoi tao uoc luong chong rung");
        KhungDuLieuImu mauXoc = taoMau(6000);
        mauXoc.giaTocY_mps2 = 9.0f;
        mauXoc.giaTocZ_mps2 = 9.0f;
        kiemTra(uocLuongRung.update(mauXoc, trangThaiRung), "cap nhat mau co xung rung ngang");
        kiemTra(std::fabs(trangThaiRung.gocNghiengNgang_rad) < 0.05f,
                "goc nghieng ngang duoc loc gyro chong ngat gia do xung rung 5ms");
    }

    // 2. Kiem tra giu vi tri dung tai cho che do PID_THUAN khi lech trong tam co khi:
    {
        CascadeController boGiuPidThuan;
        DifferentialDrive odoGiuPidThuan;
        TrangThaiXe trangThaiGiu{};
        WheelStepCommand lenhGiu{};
        const double gocLechTrongTam = 0.008;
        double gocNghienThuc = gocLechTrongTam;
        double tocDoNghienThuc = 0.0;
        double vanTocThucMetGiay = 0.0;
        double viTriThucMet = 0.0;
        constexpr double buocThoiGian = 0.005;
        constexpr double buocNho = buocThoiGian / 10.0;
        const auto nguonOdo = dungPhanCungTmcMt
            ? NguonUocLuongQuangDuong::EncoderBanh
            : NguonUocLuongQuangDuong::XungBuoc;
        const double metMoiXung = DifferentialDrive::metersPerFeedbackPulse(nguonOdo);
        const double metMoiBuoc = DifferentialDrive::metersPerStep();
        for (unsigned mau = 0; mau < 2400; ++mau) {
            const uint64_t thoiDiem = 1000u + mau * 5000u;
            trangThaiGiu.gocHopLe = true;
            trangThaiGiu.gocNghien_rad = static_cast<float>(gocNghienThuc);
            trangThaiGiu.tocDoNghien_radps = static_cast<float>(tocDoNghienThuc);
            const int64_t soXungDo = static_cast<int64_t>(std::llround(viTriThucMet / metMoiXung));
            if (!odoGiuPidThuan.update({soXungDo, soXungDo, thoiDiem, true}, trangThaiGiu, nguonOdo)) continue;
            kiemTra(boGiuPidThuan.update(trangThaiGiu, thoiDiem, false, lenhGiu),
                    "PID Thuan giu vi tri tai cho hop le");
            const double vanTocMoi = lenhGiu.tocDoBuocTrai_buocTrenGiay * metMoiBuoc;
            const double giaTocThuc = (vanTocMoi - vanTocThucMetGiay) / buocThoiGian;
            for (int buoc = 0; buoc < 10; ++buoc) {
                tocDoNghienThuc += (9.80665 * std::sin(gocNghienThuc - gocLechTrongTam) -
                                    giaTocThuc * std::cos(gocNghienThuc - gocLechTrongTam)) /
                                   toaDoTrongTamZ_m * buocNho;
                gocNghienThuc += tocDoNghienThuc * buocNho;
            }
            viTriThucMet += 0.5 * (vanTocThucMetGiay + vanTocMoi) * buocThoiGian;
            vanTocThucMetGiay = vanTocMoi;
        }
        // Góc cân bằng bị cố ý để sai 0,008 rad: nhánh vị trí P cần một sai lệch
        // hữu hạn để tạo góc đỡ đúng. Không ép x=0 hoặc giữ ngưỡng chỉ hợp với Kiv cũ.
        const double viTriCanBangDuKien = -gocLechTrongTam / kiVanTocPidThuan;
        kiemTra(std::fabs(viTriThucMet - viTriCanBangDuKien) < 0.005 &&
                std::fabs(vanTocThucMetGiay) < 0.005 &&
                std::fabs(gocNghienThuc - gocLechTrongTam) < 0.001,
                "PID Thuan hoi tu dung vi tri can bang khi co lech goc trong tam chua bu");
    }

    // 3. Kiem tra bam vi tri khi di chuyen dia hinh va khoa vi tri dung moi khi nha ga:
    {
        CascadeController boDiaHinh;
        DifferentialDrive odoDiaHinh;
        TrangThaiXe trangThaiDiaHinh{};
        WheelStepCommand lenhDiaHinh{};
        double gocNghienDiaHinh = 0.0;
        double tocDoNghienDiaHinh = 0.0;
        double vanTocDiaHinh = 0.0;
        double quangDuongMet = 0.0;
        constexpr double buocThoiGian = 0.005;
        constexpr double buocNho = buocThoiGian / 10.0;
        const auto nguonOdo = dungPhanCungTmcMt
            ? NguonUocLuongQuangDuong::EncoderBanh
            : NguonUocLuongQuangDuong::XungBuoc;
        const double metMoiXung = DifferentialDrive::metersPerFeedbackPulse(nguonOdo);
        const double metMoiBuoc = DifferentialDrive::metersPerStep();
        for (unsigned mau = 0; mau < 400; ++mau) {
            const uint64_t thoiDiem = 1000u + mau * 5000u;
            trangThaiDiaHinh.gocHopLe = true;
            trangThaiDiaHinh.gocNghien_rad = static_cast<float>(gocNghienDiaHinh);
            trangThaiDiaHinh.tocDoNghien_radps = static_cast<float>(tocDoNghienDiaHinh);
            const int64_t soXung = static_cast<int64_t>(std::llround(quangDuongMet / metMoiXung));
            if (!odoDiaHinh.update({soXung, soXung, thoiDiem, true}, trangThaiDiaHinh, nguonOdo)) continue;
            kiemTra(boDiaHinh.update(trangThaiDiaHinh, thoiDiem, false, lenhDiaHinh, 0.12f, 0.0f),
                    "Cap nhat khi dang di chuyen dia hinh");
            const double vanTocMoi = lenhDiaHinh.tocDoBuocTrai_buocTrenGiay * metMoiBuoc;
            const double giaTocThuc = (vanTocMoi - vanTocDiaHinh) / buocThoiGian;
            for (int buoc = 0; buoc < 10; ++buoc) {
                tocDoNghienDiaHinh += (9.80665 * std::sin(gocNghienDiaHinh) -
                                       giaTocThuc * std::cos(gocNghienDiaHinh)) /
                                      toaDoTrongTamZ_m * buocNho;
                gocNghienDiaHinh += tocDoNghienDiaHinh * buocNho;
            }
            quangDuongMet += 0.5 * (vanTocDiaHinh + vanTocMoi) * buocThoiGian;
            vanTocDiaHinh = vanTocMoi;
        }
        const float viTriMucTieuLucChay = boDiaHinh.viTriMucTieu_m();
        for (unsigned mau = 400; mau < 800; ++mau) {
            const uint64_t thoiDiem = 1000u + mau * 5000u;
            trangThaiDiaHinh.gocHopLe = true;
            trangThaiDiaHinh.gocNghien_rad = static_cast<float>(gocNghienDiaHinh);
            trangThaiDiaHinh.tocDoNghien_radps = static_cast<float>(tocDoNghienDiaHinh);
            const int64_t soXung = static_cast<int64_t>(std::llround(quangDuongMet / metMoiXung));
            odoDiaHinh.update({soXung, soXung, thoiDiem, true}, trangThaiDiaHinh, nguonOdo);
            kiemTra(boDiaHinh.update(trangThaiDiaHinh, thoiDiem, false, lenhDiaHinh, 0.0f, 0.0f),
                    "Cap nhat sau khi nha ga ve 0");
            const double vanTocMoi = lenhDiaHinh.tocDoBuocTrai_buocTrenGiay * metMoiBuoc;
            const double giaTocThuc = (vanTocMoi - vanTocDiaHinh) / buocThoiGian;
            for (int buoc = 0; buoc < 10; ++buoc) {
                tocDoNghienDiaHinh += (9.80665 * std::sin(gocNghienDiaHinh) -
                                       giaTocThuc * std::cos(gocNghienDiaHinh)) /
                                      toaDoTrongTamZ_m * buocNho;
                gocNghienDiaHinh += tocDoNghienDiaHinh * buocNho;
            }
            quangDuongMet += 0.5 * (vanTocDiaHinh + vanTocMoi) * buocThoiGian;
            vanTocDiaHinh = vanTocMoi;
        }
        kiemTra(viTriMucTieuLucChay > 0.08f && boDiaHinh.viTriMucTieu_m() > 0.08f &&
                std::fabs(vanTocDiaHinh) < 0.02,
                "Khoa vi tri moi tai diem dung ma khong keo lui ve goc 0");
    }

#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
    // Kiem tra nha ga em tren LSM6DSV khong tao buoc nhay goc dat dot ngot (ca khi bat va tat vgainboost):
    for (bool batTangBam : {false, true}) {
        CascadeController boNhaGa;
        boNhaGa.setThuNghiemDiChuyen(false, batTangBam);
        boNhaGa.setPidVanTocThuan(lsm6dsvTune::kpv, lsm6dsvTune::kiv);
        TrangThaiXe xeNha{};
        WheelStepCommand lenhNha{};
        xeNha.gocHopLe = xeNha.viTriHopLe = xeNha.vanTocHopLe = true;
        xeNha.viTriDoc_m = 0.0f;
        xeNha.vanTocDoc_mps = 0.0f;
        boNhaGa.update(xeNha, 1000u, false, lenhNha, 0.25f, 0.0f);
        for (unsigned mau = 1u; mau <= 120u; ++mau) {
            xeNha.viTriDoc_m += 0.0005f;
            xeNha.vanTocDoc_mps = 0.10f;
            boNhaGa.update(xeNha, 1000u + mau * 5000u, false, lenhNha, 0.25f, 0.0f);
        }
        float gocDatTruoc_rad = boNhaGa.gocDat_rad();
        float buocNhayToiDa_rad = 0.0f;
        for (unsigned mau = 121u; mau <= 200u; ++mau) {
            xeNha.viTriDoc_m += 0.0002f;
            boNhaGa.update(xeNha, 1000u + mau * 5000u, false, lenhNha, 0.0f, 0.0f);
            const float delta = std::fabs(boNhaGa.gocDat_rad() - gocDatTruoc_rad);
            if (delta > buocNhayToiDa_rad) buocNhayToiDa_rad = delta;
            gocDatTruoc_rad = boNhaGa.gocDat_rad();
        }
        kiemTra(buocNhayToiDa_rad < 0.015f,
                "LSM6DSV nha ga khong tao buoc nhay goc dat qua 0.015 rad");
    }

    // Kiem tra bo tham so lap lich gain theo |v|, bien du tru on dinh khi chay va loc muot goc dat 240 Hz tren LSM6DSV:
    {
        kiemTra(lsm6dsvTune::kp >= 24.0f && lsm6dsvTune::kd <= 2.4f &&
                lsm6dsvTune::kd * lsm6dsvTune::heSoGiamChayKd <= 1.85f &&
                lsm6dsvTune::kdMo * lsm6dsvTune::heSoGiamChayKd * 1.5f <= 2.30f &&
                lsm6dsvTune::gyroD_hz >= 50.0f && lsm6dsvTune::locGocDat_s > lsm6dsvTune::locGocDatTinh_s &&
                lsm6dsvTune::tangBamVanToc,
                "LSM6DSV giu do cung dung yen cao, giam an toan khi chay, bat tang bam van toc va loc muot goc dat");
        // Kiem tra du goc dat de vuot nguong ma sat lan khi keo tien muc vua (0.325 m/s ~65%):
        {
            CascadeController boDayTien;
            boDayTien.setThuNghiemDiChuyen(lsm6dsvTune::tangGiaToc, lsm6dsvTune::tangBamVanToc);
            boDayTien.setPidVanTocThuan(lsm6dsvTune::kpv, lsm6dsvTune::kiv);
            TrangThaiXe xeDung{};
            WheelStepCommand lenhDay{};
            xeDung.gocHopLe = xeDung.viTriHopLe = xeDung.vanTocHopLe = true;
            uint64_t tDay_us = 1000u;
            boDayTien.update(xeDung, tDay_us, false, lenhDay, 0.325f, 0.0f);
            for (unsigned m = 1u; m <= 240u; ++m) {
                tDay_us += 4167u;
                boDayTien.update(xeDung, tDay_us, false, lenhDay, 0.325f, 0.0f);
            }
            kiemTra(boDayTien.gocDat_rad() >= 0.105f && boDayTien.gocDatHieuDung_rad() >= 0.105f,
                    "LSM6DSV keo tien 65% dat goc dat >= 6.0 deg de vuot nguong lan banh");
        }
        CascadeController boLocGocDat;
        boLocGocDat.setThuNghiemDiChuyen(false, false);
        boLocGocDat.setPidVanTocThuan(lsm6dsvTune::kpv, lsm6dsvTune::kiv);
        boLocGocDat.setGocCanBang_rad(0.05f);
        TrangThaiXe xeLoc{};
        WheelStepCommand lenhLoc{};
        xeLoc.gocHopLe = xeLoc.viTriHopLe = xeLoc.vanTocHopLe = true;
        uint64_t thoiDiem_us = 1000u;
        boLocGocDat.update(xeLoc, thoiDiem_us, false, lenhLoc, 0.0f, 0.0f);
        // 1. Dung yen (|v|=0): giu nguyen Kp/Kd dung yen.
        for (unsigned mau = 1u; mau <= 10u; ++mau) {
            thoiDiem_us += 4167u;
            boLocGocDat.update(xeLoc, thoiDiem_us, false, lenhLoc, 0.0f, 0.0f);
        }
        kiemTra(std::fabs(boLocGocDat.heSoPid().heSoTiLe - kpGocPidThuan) < 1e-4f &&
                std::fabs(boLocGocDat.heSoPid().heSoViPhan - kdGocPidThuan) < 1e-4f,
                "LSM6DSV dung yen giu dung Kp/Kd do cung cao");
        for (unsigned mau = 1u; mau <= 240u; ++mau) {
            thoiDiem_us += 4167u;
            xeLoc.vanTocDoc_mps = 0.15f;
            boLocGocDat.update(xeLoc, thoiDiem_us, false, lenhLoc, 0.40f, 0.0f);
        }
        // 2. Khi chay (|v|=0.15 m/s): giam em xuong heSoGiamChayKp / heSoGiamChayKd.
        kiemTra(std::fabs(boLocGocDat.heSoPid().heSoTiLe - kpGocPidThuan * lsm6dsvTune::heSoGiamChayKp) < 0.05f &&
                std::fabs(boLocGocDat.heSoPid().heSoViPhan - kdGocPidThuan * lsm6dsvTune::heSoGiamChayKd) < 0.01f,
                "LSM6DSV khi chay giam em Kp/Kd de triet dao dong 10 Hz");
        float gocHieuDungTruoc = boLocGocDat.gocDatHieuDung_rad();
        float deltaHieuDungMax = 0.0f;
        float chenhLechTruoc = std::fabs(boLocGocDat.gocDat_rad() - gocHieuDungTruoc);
        bool thayTreSauStep = false;
        bool hoiTuDonDieu = true;
        float gocThoTruoc = boLocGocDat.gocDat_rad();
        for (unsigned mau = 1u; mau <= 120u; ++mau) {
            thoiDiem_us += 4167u;
            boLocGocDat.update(xeLoc, thoiDiem_us, false, lenhLoc, -0.40f, 0.0f);
            const float gocTho = boLocGocDat.gocDat_rad();
            const float gocHieuDung = boLocGocDat.gocDatHieuDung_rad();
            const float delta = std::fabs(gocHieuDung - gocHieuDungTruoc);
            if (delta > deltaHieuDungMax) deltaHieuDungMax = delta;
            if (std::fabs(gocTho - gocHieuDung) > 1e-4f) thayTreSauStep = true;
            const float chenhLech = std::fabs(gocTho - gocHieuDung);
            if (std::fabs(gocTho - gocThoTruoc) < 1e-6f) {
                if (chenhLech > chenhLechTruoc + 1e-6f) hoiTuDonDieu = false;
            }
            chenhLechTruoc = chenhLech;
            gocThoTruoc = gocTho;
            gocHieuDungTruoc = gocHieuDung;
        }
        kiemTra(deltaHieuDungMax < 0.004f && thayTreSauStep && hoiTuDonDieu,
                "LSM6DSV loc muot goc dat tai 240Hz khi dao chieu dot ngot va hoi tu don dieu");

        // 3. Tro lai dung yen (|v|=0): phuc hoi Kp/Kd dung yen; Bench Mode giu nguyen he so goc.
        xeLoc.vanTocDoc_mps = 0.0f;
        for (unsigned mau = 1u; mau <= 240u; ++mau) {
            thoiDiem_us += 4167u;
            boLocGocDat.update(xeLoc, thoiDiem_us, false, lenhLoc, 0.0f, 0.0f);
        }
        kiemTra(std::fabs(boLocGocDat.heSoPid().heSoTiLe - kpGocPidThuan) < 0.05f &&
                std::fabs(boLocGocDat.heSoPid().heSoViPhan - kdGocPidThuan) < 0.01f,
                "LSM6DSV phuc hoi Kp/Kd dung yen khi xe dung lai");
        boLocGocDat.setCheDoThuKhongMaSat(true);
        xeLoc.vanTocDoc_mps = 0.25f;
        for (unsigned mau = 1u; mau <= 60u; ++mau) {
            thoiDiem_us += 4167u;
            boLocGocDat.update(xeLoc, thoiDiem_us, false, lenhLoc, 0.0f, 0.0f);
        }
        kiemTra(std::fabs(boLocGocDat.heSoPid().heSoTiLe - kpGocPidThuan) < 1e-4f,
                "Bench Mode khong giam gain khi banh quay trong khong khi");
        boLocGocDat.setCheDoThuKhongMaSat(false);
        xeLoc.vanTocDoc_mps = 0.0f;

        boLocGocDat.reset();
        kiemTra(std::fabs(boLocGocDat.gocDatHieuDung_rad() - boLocGocDat.gocCanBangThucTe_rad()) < 1e-6f,
                "LSM6DSV reset dua gocDatHieuDung ve goc can bang");
        for (unsigned mau = 1u; mau <= 20u; ++mau) {
            thoiDiem_us += 4167u;
            boLocGocDat.update(xeLoc, thoiDiem_us, false, lenhLoc, 0.40f, 0.0f);
        }
        kiemTra(std::fabs(boLocGocDat.gocDatHieuDung_rad() - boLocGocDat.gocCanBangThucTe_rad()) > 1e-4f,
                "LSM6DSV gocDatHieuDung thay doi khi vong ngoai hoat dong");
        boLocGocDat.setBatVongNgoai(false);
        kiemTra(std::fabs(boLocGocDat.gocDatHieuDung_rad() - boLocGocDat.gocCanBangThucTe_rad()) < 1e-6f,
                "LSM6DSV tat vong ngoai dua gocDatHieuDung ve goc can bang");
    }
#endif

    // Kiem tra trong truong tham chieu tuy chinh (vd 9.42 m/s^2 tu static LSM6DSV):
    {
        ComplementaryEstimator uocLuongRef{HeTrucImu{MaTranQuayImu{}}};
        uocLuongRef.datTrongTruongThamChieu_mps2(9.42f);
        TrangThaiXe xeRef{};
        KhungDuLieuImu mau942 = taoMau(1000u, 0.0f);
        mau942.giaTocZ_mps2 = 9.42f;
        kiemTra(uocLuongRef.update(mau942, xeRef) && xeRef.doTinCayGiaToc > 0.99f && xeRef.giaTocTinCay,
                "Trong truong tham chieu 9.42 m/s^2 giup trust tinh dat >0.99");
        mau942 = taoMau(6000u, 3.0f);
        mau942.giaTocZ_mps2 = 9.42f;
        kiemTra(uocLuongRef.update(mau942, xeRef) && xeRef.doTinCayGiaToc < 0.01f,
                "Trong truong tham chieu 9.42 van loai xung gia toc ngang 3 m/s^2 qua innovation");
    }

    // 4. Kiem tra khoa huong quay ngang giu cho hai banh khong xoay lech khi dung tai cho:
    {
        CascadeController boGiuHuong;
        DifferentialDrive odoGiuHuong;
        TrangThaiXe trangThaiHuong{};
        WheelStepCommand lenhHuong{};
        const auto nguonOdo = dungPhanCungTmcMt
            ? NguonUocLuongQuangDuong::EncoderBanh
            : NguonUocLuongQuangDuong::XungBuoc;
        const double metMoiXung = DifferentialDrive::metersPerFeedbackPulse(nguonOdo);
        const int64_t xungLechPhai = static_cast<int64_t>(std::llround(0.02 / metMoiXung));
        trangThaiHuong.gocHopLe = true;
        odoGiuHuong.update({0, 0, 1000u, true}, trangThaiHuong, nguonOdo);
        odoGiuHuong.update({0, 0, 6000u, true}, trangThaiHuong, nguonOdo);
        boGiuHuong.update(trangThaiHuong, 6000u, false, lenhHuong, 0.0f, 0.0f);
        odoGiuHuong.update({0, xungLechPhai, 11000u, true}, trangThaiHuong, nguonOdo);
        kiemTra(boGiuHuong.update(trangThaiHuong, 11000u, false, lenhHuong, 0.0f, 0.0f),
                "Cap nhat giu huong quay ngang");
        kiemTra(lenhHuong.tocDoBuocTrai_buocTrenGiay > lenhHuong.tocDoBuocPhai_buocTrenGiay,
                "Khoa huong bu toc do vi sai de keo thang huong khi mot banh bi lech");
    }

    // 5. Kiem tra 2 che do PID Thuan va PID Mo hoat dong doc lap va chuyen doi em (bumpless):
    {
        CascadeController boThuThuan;
        CascadeController boThuMo;
        DifferentialDrive odoThuThuan;
        DifferentialDrive odoThuMo;
        boThuThuan.setPidGocThuan({28.0f, 0.0f, 2.8f});
        boThuThuan.setPidVanTocThuan(0.14f, 0.07f);
        boThuMo.setPidGocMoCoSo({32.0f, 0.0f, 3.2f});
        boThuMo.datPidVanTocMo(0.22f, 0.11f);
        kiemTra(std::fabs(boThuMo.kpVanTocMo() - 0.22f) < 1e-6f &&
                std::fabs(boThuMo.kiVanTocMo() - 0.11f) < 1e-6f &&
                std::fabs(boThuThuan.kpVanTocThuan() - 0.14f) < 1e-6f &&
                std::fabs(boThuThuan.kiVanTocThuan() - 0.07f) < 1e-6f,
                "Hai bo he so vong ngoai PID Thuan va PID Mo doc lap");

        TrangThaiXe xeVongThuan{};
        TrangThaiXe xeVongMo{};
        WheelStepCommand lenhVongThuan{};
        WheelStepCommand lenhVongMo{};
        xeVongThuan.gocHopLe = true;
        xeVongMo.gocHopLe = true;
        odoThuThuan.update({0, 0, 1000u, true}, xeVongThuan);
        odoThuMo.update({0, 0, 1000u, true}, xeVongMo);
        odoThuThuan.update({0, 0, 6000u, true}, xeVongThuan);
        odoThuMo.update({0, 0, 6000u, true}, xeVongMo);
        boThuThuan.update(xeVongThuan, 6000u, false, lenhVongThuan);
        boThuMo.update(xeVongMo, 6000u, true, lenhVongMo);
        for (unsigned mau = 1u; mau <= 5u; ++mau) {
            const uint64_t thoiDiem = 6000u + mau * 5000u;
            const int64_t xungMau = static_cast<int64_t>(mau * 15u);
            odoThuThuan.update({xungMau, xungMau, thoiDiem, true}, xeVongThuan);
            odoThuMo.update({xungMau, xungMau, thoiDiem, true}, xeVongMo);
            boThuThuan.update(xeVongThuan, thoiDiem, false, lenhVongThuan);
            boThuMo.update(xeVongMo, thoiDiem, true, lenhVongMo);
        }
        kiemTra(std::fabs(boThuMo.gocDat_rad()) > std::fabs(boThuThuan.gocDat_rad()) + 1e-4f,
                "Vong ngoai PID Mo va PID Thuan dung bo he so doc lap");

        CascadeController boHaiCheDo;
        DifferentialDrive odoHaiCheDo;
        TrangThaiXe xeHaiMode{};
        WheelStepCommand lenhHaiMode{};
        const double gocLech = 0.008;
        double gocHienTai = gocLech;
        double tocDoGocHienTai = 0.0;
        double vanTocHienTai = 0.0;
        double viTriHienTai = 0.0;
        constexpr double dtMoPhong = 0.005;
        constexpr double dtTichPhan = dtMoPhong / 10.0;
        bool daThayThuanCoDinh = false;
        bool daThayMoTangKp = false;
        bool daThayMoTangKd = false;
        const auto nguonOdo = dungPhanCungTmcMt
            ? NguonUocLuongQuangDuong::EncoderBanh
            : NguonUocLuongQuangDuong::XungBuoc;
        const double metMoiXung = DifferentialDrive::metersPerFeedbackPulse(nguonOdo);
        const double metMoiBuoc = DifferentialDrive::metersPerStep();

        for (unsigned mau = 0u; mau < 2400u; ++mau) {
            const uint64_t thoiDiem = 1000u + mau * 5000u;
            const bool dungMo = (mau >= 600u && mau < 1400u);
            if (mau == 700u) gocHienTai += 0.025;
            xeHaiMode.gocHopLe = true;
            xeHaiMode.gocNghien_rad = static_cast<float>(gocHienTai);
            xeHaiMode.tocDoNghien_radps = static_cast<float>(tocDoGocHienTai);
            const int64_t xungDo = static_cast<int64_t>(std::llround(viTriHienTai / metMoiXung));
            if (!odoHaiCheDo.update({xungDo, xungDo, thoiDiem, true}, xeHaiMode, nguonOdo)) continue;
            kiemTra(boHaiCheDo.update(xeHaiMode, thoiDiem, dungMo, lenhHaiMode),
                    "Cap nhat chuyen doi hai che do hop le");
            const PidGains heSoDangChay = boHaiCheDo.heSoPid();
            if (!dungMo && mau >= 5u) {
                if (std::fabs(heSoDangChay.heSoTiLe - kpGocPidThuan) < 1e-4f &&
                    std::fabs(heSoDangChay.heSoViPhan - kdGocPidThuan) < 1e-4f) {
                    daThayThuanCoDinh = true;
                }
            }
            if (dungMo && mau >= 700u && mau <= 740u) {
                if (heSoDangChay.heSoTiLe > kpGoc + 0.5f) daThayMoTangKp = true;
                if (heSoDangChay.heSoViPhan > kdGoc + 0.05f) daThayMoTangKd = true;
            }
            const double vanTocMoi = lenhHaiMode.tocDoBuocTrai_buocTrenGiay * metMoiBuoc;
            const double giaTocThuc = (vanTocMoi - vanTocHienTai) / dtMoPhong;
            for (int buoc = 0; buoc < 10; ++buoc) {
                tocDoGocHienTai += (9.80665 * std::sin(gocHienTai - gocLech) -
                                    giaTocThuc * std::cos(gocHienTai - gocLech)) /
                                   toaDoTrongTamZ_m * dtTichPhan;
                gocHienTai += tocDoGocHienTai * dtTichPhan;
            }
            viTriHienTai += 0.5 * (vanTocHienTai + vanTocMoi) * dtMoPhong;
            vanTocHienTai = vanTocMoi;
            kiemTra(std::isfinite(gocHienTai) && std::fabs(gocHienTai) < 0.08,
                    "Khong vot lo hoac mat on dinh khi doi che do giua chung");
        }
        kiemTra(daThayThuanCoDinh && daThayMoTangKp && daThayMoTangKd,
                "PID Thuan giu he so co dinh va PID Mo tu tang Kp/Kd khi nghieng");
        const double viTriCanBangSauChuyenMode = -gocLech / kiVanTocPidThuan;
        kiemTra(std::fabs(viTriHienTai - viTriCanBangSauChuyenMode) < 0.005 &&
                std::fabs(vanTocHienTai) < 0.005 && std::fabs(gocHienTai - gocLech) < 0.001,
                "Xe hoi tu dung diem can bang sau khi chuyen mode voi lech trong tam chua bu");
    }

    // 6. Kiem tra goi lenh nhi phan 8 byte (WebSocket/UDP/ESP-NOW) va duong cong keo tang toc 2 joystick:
    {
        const GoiLenhNhiPhan goiHopLe = dongGoiLenhNhiPhan(0.15f, -0.40f, 0.20f, 0.80f, static_cast<uint8_t>('a'));
        uint8_t mangByte[sizeof(GoiLenhNhiPhan)]{};
        std::memcpy(mangByte, &goiHopLe, sizeof(GoiLenhNhiPhan));
        kiemTra(sizeof(GoiLenhNhiPhan) == 8u && mangByte[0] == dauKhungNhiPhan &&
                mangByte[1] == maLenhDiChuyenNhiPhan && mangByte[7] == tinhMaKiemTraNhiPhan(mangByte),
                "Khung nhi phan dung 8 byte, co header 0xA5 va XOR hop le");

        float vanTocGiaiMa = 0.0f;
        float tocDoQuayGiaiMa = 0.0f;
        char phimGiaiMa = '\0';
        bool coDiChuyen = false;
        kiemTra(giaiMaGoiLenhNhiPhan(mangByte, sizeof(mangByte), 0.20f, 0.80f,
                                     vanTocGiaiMa, tocDoQuayGiaiMa, phimGiaiMa, coDiChuyen) &&
                coDiChuyen && phimGiaiMa == 'a' &&
                std::fabs(vanTocGiaiMa - 0.15f) < 0.001f &&
                std::fabs(tocDoQuayGiaiMa - (-0.40f)) < 0.002f,
                "Giai ma chinh xac 2 kenh tien/lui, re va phim hanh dong");

        const GoiLenhNhiPhan goiPhim = dongGoiLenhNhiPhan(0.0f, 0.0f, 0.20f, 0.80f, static_cast<uint8_t>('p'), maLenhPhimNhiPhan);
        std::memcpy(mangByte, &goiPhim, sizeof(GoiLenhNhiPhan));
        kiemTra(giaiMaGoiLenhNhiPhan(mangByte, sizeof(mangByte), 0.20f, 0.80f,
                                     vanTocGiaiMa, tocDoQuayGiaiMa, phimGiaiMa, coDiChuyen) &&
                !coDiChuyen && phimGiaiMa == 'p',
                "Goi phim lenh nhi phan 0x02 giu coDiChuyen=false va dung ky tu phim");

        uint8_t goiSaiDau[sizeof(GoiLenhNhiPhan)]{};
        std::memcpy(goiSaiDau, mangByte, sizeof(goiSaiDau));
        goiSaiDau[0] = 0x5Au;
        kiemTra(!giaiMaGoiLenhNhiPhan(goiSaiDau, sizeof(goiSaiDau), 0.20f, 0.80f,
                                      vanTocGiaiMa, tocDoQuayGiaiMa, phimGiaiMa, coDiChuyen),
                "Tu choi khung nhi phan sai dau khung");

        uint8_t goiSaiXor[sizeof(GoiLenhNhiPhan)]{};
        std::memcpy(goiSaiXor, mangByte, sizeof(goiSaiXor));
        goiSaiXor[7] ^= 0x01u;
        kiemTra(!giaiMaGoiLenhNhiPhan(goiSaiXor, sizeof(goiSaiXor), 0.20f, 0.80f,
                                      vanTocGiaiMa, tocDoQuayGiaiMa, phimGiaiMa, coDiChuyen),
                "Tu choi khung nhi phan sai ma kiem tra XOR");
        kiemTra(!giaiMaGoiLenhNhiPhan(mangByte, 7u, 0.20f, 0.80f,
                                      vanTocGiaiMa, tocDoQuayGiaiMa, phimGiaiMa, coDiChuyen),
                "Tu choi khung nhi phan thieu do dai");

        const float mucNhe = noiSuyTangTocKeo(0.30f);
        const float mucVua = noiSuyTangTocKeo(0.60f);
        const float mucToiDa = noiSuyTangTocKeo(1.0f);
        const float mucLuiToiDa = noiSuyTangTocKeo(-1.0f);
        kiemTra(noiSuyTangTocKeo(0.05f) == 0.0f &&
                noiSuyTangTocKeo(-0.05f) == 0.0f &&
                mucNhe > 0.0f && mucVua > mucNhe &&
                (mucToiDa - mucVua) > (mucVua - mucNhe) &&
                std::fabs(mucToiDa - 1.0f) < 1e-6f &&
                std::fabs(mucLuiToiDa + 1.0f) < 1e-6f,
                "Duong cong keo tang toc triet vung chet, tang phi tuyen va dat +/-1.0");
    }

    std::printf("PASS: filter/motion/anti-windup/TMC_MT_closed_loop %u checks; PT1 50Hz amplitude=%.6f\n",soKiemTra,tyLe);
}
