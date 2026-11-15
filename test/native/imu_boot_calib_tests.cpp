#include "Icm20602Driver.h"
#include "ThongKeMauImu.h"
#include "calib/UI_Calib.h"
#include "esp32-hal-rgb-led.h"
#include "driver/gpio.h"
#include "core/ChotMauImu.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
unsigned soDieuKien = 0;
void kiemTra(bool dat, const char* moTa) {
    ++soDieuKien;
    if (!dat) { std::cerr << "FAIL: " << moTa << '\n' << Serial.noiDung; std::exit(1); }
}
void datLaiMoPhong() {
    dongHoGia_ms = 0;
    Serial.noiDung.clear();
    docNutGia = {};
    doiMauGia = {};
}
bool coLog(const char* dong) { return Serial.noiDung.find(dong) != std::string::npos; }

void kiemTraBoot() {
    datLaiMoPhong();
    TwoWire bus;
    Icm20602Driver camBien(bus);
    BaoCaoKiemTraIcm baoCao;
    kiemTra(!camBien.kiemTraSauKhoiDong(baoCao), "khong bo qua begin");
    kiemTra(camBien.begin(), "begin thanh cong");
    kiemTra(bus.chanSda == 10 && bus.chanScl == 11 && mucGpioGia[9] == 1 && mucGpioGia[8] == 0,
        "dung so do chan va che do I2C");
    kiemTra(camBien.kiemTraSauKhoiDong(baoCao), "32 mau dung yen giong nhau van hop le");
    kiemTra(baoCao.soMauTot == 32 && baoCao.soMauLoi == 0 && baoCao.thoiGian_ms >= 155,
        "can 32 lan data-ready moi, khong doc lap thanh ghi");
    kiemTra(!baoCao.mauCuoi.daHieuChinhConQuay && std::fabs(baoCao.mauCuoi.giaTocZ_mps2 - 9.80665f) < .001f,
        "du lieu truoc calib co gia toc dung don vi");
    bus.thanhGhi[0x6B] = 0x40;
    kiemTra(!camBien.kiemTraSauKhoiDong(baoCao) && std::string(baoCao.ketQua) == "sai_cau_hinh",
        "phat hien cam bien ngu sau reset");
    kiemTra(camBien.begin() && camBien.kiemTraSauKhoiDong(baoCao) && bus.soLanReset == 2,
        "reset ESP voi ICM con nguon: begin reset va cau hinh lai");
    bus.thanhGhi[0x75] = 0x68;
    kiemTra(!camBien.kiemTraSauKhoiDong(baoCao) && std::string(baoCao.ketQua) == "sai_who_am_i",
        "tu choi chip sai ID");
    kiemTra(!camBien.begin() && camBien.maNhanDang() == 0x68, "begin cung tu choi chip sai");
}

void kiemTraLoiBusVaMau() {
    datLaiMoPhong();
    TwoWire bus;
    Icm20602Driver camBien(bus);
    BaoCaoKiemTraIcm baoCao;
    kiemTra(camBien.begin(), "begin de tiem loi");
    bus.coDataReady = false;
    KhungDuLieuImu mau;
    mau.giaTocX_mps2 = 123;
    mau.trangThai = TrangThaiKhungDuLieuImu::HopLe;
    kiemTra(!camBien.docMauMoi(mau) && mau.trangThai == TrangThaiKhungDuLieuImu::DuLieuCu &&
        mau.giaTocX_mps2 == 0, "khong co mau moi phai xoa frame cu");
    kiemTra(!camBien.kiemTraSauKhoiDong(baoCao) && std::string(baoCao.ketQua) == "khong_du_mau_moi" &&
        baoCao.thoiGian_ms == thoiGianKiemTraKhoiDongIcm_ms, "timeout khi khong co data-ready");
    bus.coDataReady = true;
    bus.thanhGhiNack = 0x3A;
    kiemTra(!camBien.docMauMoi(mau) && mau.trangThai == TrangThaiKhungDuLieuImu::LoiIO &&
        camBien.maLoiI2cCuoi() == IcmI2cStatus::ADDR_NACK, "phan biet NACK voi chua co mau");
    kiemTra(!camBien.kiemTraSauKhoiDong(baoCao) && std::string(baoCao.ketQua) == "loi_i2c",
        "boot tu choi NACK thanh ghi status");
    bus.thanhGhiNack = 0x3B;
    kiemTra(!camBien.kiemTraSauKhoiDong(baoCao) && camBien.thanhGhiCuoi() == 0x3B,
        "boot tu choi NACK khoi accel gyro");
    bus.thanhGhiNack = -1;
    bus.thanhGhiDocThieu = 0x3B;
    kiemTra(!camBien.kiemTraSauKhoiDong(baoCao) && camBien.maLoiI2cCuoi() == IcmI2cStatus::READ_SIZE_MISMATCH,
        "doc thieu 14 byte khong duoc PASS");
    bus.thanhGhiDocThieu = -1;
    bus.taoMau = [](uint32_t) { return std::array<int16_t,7>{}; };
    kiemTra(!camBien.kiemTraSauKhoiDong(baoCao) && baoCao.soMauLoi > 0 &&
        std::string(baoCao.ketQua) == "du_lieu_bat_thuong", "bus ACK nhung du lieu toan0 phai tu choi");
    bus.taoMau = [](uint32_t) { return std::array<int16_t,7>{-1,-1,-1,-1,-1,-1,-1}; };
    kiemTra(!camBien.kiemTraSauKhoiDong(baoCao), "du lieu toanFF phai tu choi");
    bus.taoMau = [](uint32_t t) { return std::array<int16_t,7>{0, static_cast<int16_t>((t/5%2)*80),8192,0,100,0,0}; };
    kiemTra(camBien.kiemTraSauKhoiDong(baoCao), "boot van PASS khi co rung nhe, chua can bias");
}

void kiemTraThuLaiGyro() {
    datLaiMoPhong();
    TwoWire bus;
    Icm20602Driver camBien(bus);
    kiemTra(camBien.begin(), "begin thu gyro");
    const uint32_t batDau = millis();
    bus.taoMau = [=](uint32_t t) {
        const int16_t nhieu = t - batDau < 800 ? (t/5%2 ? 131 : -131) : 0;
        return std::array<int16_t,7>{0,0,8192,0,static_cast<int16_t>(98+nhieu),-33,16};
    };
    kiemTra(camBien.hieuChinhConQuay(500), "rung dau luot sau do yen: tu thu lai thanh cong");
    kiemTra(coLog("CALIB_WAIT gyro: rung") && coLog("CALIB_GYRO PASS"), "co log ly do thu lai va PASS");
    kiemTra(millis() - batDau >= 4900, "khong nhan cua so dau bi rung");
    KhungDuLieuImu mau;
    kiemTra(camBien.read(mau) && mau.daHieuChinhConQuay && std::fabs(mau.tocDoGocX_radps) < .0001f &&
        std::fabs(mau.tocDoGocY_radps) < .0001f, "bias chi lay tu khoang on dinh");
    bus.taoMau = [](uint32_t) { return std::array<int16_t,7>{0,0,8192,0,131,-20,5}; };
    kiemTra(camBien.hieuChinhConQuay(500) && camBien.read(mau) && std::fabs(mau.tocDoGocX_radps) < .0001f,
        "do lai bias khong tru nham bias cu");
    bus.taoMau = [](uint32_t t) { return std::array<int16_t,7>{0,0,8192,0,static_cast<int16_t>(t/5%2 ? 131 : -131),0,0}; };
    const uint32_t lucRung = millis();
    kiemTra(!camBien.hieuChinhConQuay(500) && millis()-lucRung == thoiGianChoOnDinhCalib_ms,
        "rung lien tuc: cho co han roi tu choi, khong chap nhan bias sai");
    kiemTra(camBien.read(mau) && !mau.daHieuChinhConQuay, "loi calib khong de lai co bias hop le");
    bus.coDataReady = false;
    const uint32_t lucMatMau = millis();
    kiemTra(!camBien.hieuChinhConQuay(500) && millis()-lucMatMau == thoiGianMatMauCalib_ms &&
        coLog("mat_mau_moi"), "mat data-ready la loi du lieu, khong doi du30s nhu rung");
    bus.coDataReady = true;
    bus.thanhGhiNack = 0x3B;
    kiemTra(!camBien.hieuChinhConQuay(500) && coLog("loi_i2c=2"), "loi I2C khong bi che boi retry on dinh");
}

void kiemTraUIVoiRung() {
    datLaiMoPhong();
    TwoWire bus;
    Icm20602Driver camBien(bus);
    kiemTra(camBien.begin(), "begin UI");
    UI_Calib ui(camBien);
    unsigned buoc = 0, lanVang = 0, soMauMoc = 0;
    uint32_t dauBuoc = millis();
    bool daRungMocDung = false, daRungMocTruoc = false, daRungMocSau = false;
    doiMauGia = [&](uint8_t r, uint8_t g, uint8_t b) {
        kiemTra(!(r == 40 && g == 0 && b == 0), "UI khong duoc do vi mot mau rung roi yen");
        dauBuoc = millis();
        soMauMoc = 0;
        if (r == 32 && g == 8) buoc = 0;
        else if (r == 0 && g == 0 && b == 36) buoc = 1;
        else if (r == 32 && g == 20) { buoc = 2; ++lanVang; }
        else if (r == 0 && g == 28) buoc = 3;
        else if (r == 32 && b == 28) buoc = 4;
        else if (r == 24 && g == 24) buoc = 5;
    };
    docNutGia = [&](int) {
        const auto t = millis() - dauBuoc;
        const auto nhanLuc = buoc == 4 ? 3400u : 200u;
        return (buoc == 1 || buoc == 3 || buoc == 4) && t >= nhanLuc && t < nhanLuc + 100 ? LOW : HIGH;
    };
    bus.taoMau = [&](uint32_t t) {
        const uint32_t dt = t - dauBuoc;
        constexpr float gocNam = 1.570796327f;
        float gocDoc = 0, gyroY = 0;
        if (buoc == 3 || (buoc == 2 && lanVang == 2)) gocDoc = gocNam;
        if (buoc == 4) {
            gocDoc = dt < 100 ? gocNam : (dt < 3100 ? gocNam - (dt-100)*(2*gocNam/3000) : -gocNam);
            gyroY = dt >= 100 && dt < 3100 ? -2*gocNam/3 : 0;
        }
        if (buoc == 2 && lanVang == 3) gocDoc = -gocNam;
        if (buoc == 2 && (lanVang != 1 || coLog("CALIB_GYRO PASS"))) {
            ++soMauMoc;
            if (soMauMoc == 10) {
                gyroY = .12f; // Rung do nut / tac dong ngan, roi lai yen.
                if (lanVang == 1) daRungMocDung = true;
                if (lanVang == 2) daRungMocTruoc = true;
                if (lanVang == 3) daRungMocSau = true;
            }
        }
        return std::array<int16_t,7>{static_cast<int16_t>(-8192*std::sin(gocDoc)),0,
            static_cast<int16_t>(8192*std::cos(gocDoc)),0,0,static_cast<int16_t>(gyroY/0.01745329252f*65.5f),0};
    };
    ui.khoiTao();
    kiemTra(buoc == 0, "cam trong khi boot, xanh duong chi khi cho xac nhan");
    MaTranQuayImu ketQua;
    ui.chay(ketQua);
    kiemTra(daRungMocDung && daRungMocTruoc && daRungMocSau, "da tiem rung vao ca ba moc tinh");
    kiemTra(buoc == 5 && HeTrucImu::kiemTraMaTran(ketQua), "UI hoan tat tat ca buoc voi ma tran hop le");
    kiemTra(coLog("buoc=moc_dung retry=1") && coLog("buoc=moc_truoc retry=1") &&
        coLog("buoc=moc_sau retry=1"), "tu thu lai tai dung buoc, khong bat nguoi dung calib tu dau");
    kiemTra(coLog("CALIB_DOC PASS") && coLog("CALIB_AXIS Y_sensor=") && coLog("HA DAU XE") && coLog("CALIB_RETURN doc="),
        "UI in huong dau duoi va chan doan truc");
    doiMauGia = {};
    docNutGia = {};
}

void kiemTraNgatVaMauRuntime() {
    datLaiMoPhong();
    TwoWire bus;
    Icm20602Driver camBien(bus);
    kiemTra(camBien.begin() && (bus.thanhGhi[0x37] & 0x30) == 0x20,
        "INT latch va chi ACK bang status");
    kiemTra(camBien.hieuChinhConQuay(500), "bias truoc khi test runtime");
    bus.taoMau=[](uint32_t) { return std::array<int16_t,7>{-100,200,8192,327,655,-131,66}; };
    delay(5);
    kiemTra(bus.intDangGiuMuc(), "data-ready ton dong truoc attach van giu INT");
    KhungDuLieuImu mau;
    kiemTra(camBien.docMauTheoNgat(mau) && !bus.intDangGiuMuc() && bus.soLanBurstStatusMau == 1,
        "burst15 ACK INT va doc accel/temp/gyro");
    kiemTra(std::fabs(mau.giaTocX_mps2-(-100*9.80665f/8192))<1e-5f &&
        std::fabs(mau.tocDoGocX_radps-10*0.01745329252f)<1e-5f &&
        std::fabs(mau.nhietDoCamBien_C-26)<.01f && mau.daHieuChinhConQuay,
        "decode15 co offsetstatus dung, khong nhambyte");
    kiemTra(chotMauImu(mau,true,1,1,10000,(10000)+1500u,5000) == KetQuaChotMauImu::ChapNhan &&
        mau.thoiDiemLayMau_us == 10000 && mau.soMauLayDuoc == 1,
        "mau moi gan timestamp tu ISR");
    kiemTra(!camBien.docMauTheoNgat(mau) && mau.trangThai == TrangThaiKhungDuLieuImu::DuLieuCu &&
        mau.thoiDiemLayMau_us == 0 && mau.giaTocZ_mps2 == 0,
        "cung data-ready khong duoc tai su dung hai lan");
    for (unsigned i=2;i<=202;++i) {
        delay(5);
        kiemTra(bus.intDangGiuMuc() && camBien.docMauTheoNgat(mau) &&
            chotMauImu(mau,true,i,i,i*5000,(i*5000)+1500u,5000) == KetQuaChotMauImu::ChapNhan,
            "ACK lien tuc nhan du lieu moi hon1giay200Hz");
    }
    delay(5);
    kiemTra(camBien.docMauTheoNgat(mau) &&
        chotMauImu(mau,false,202,202,1010000,(1010000)+1500u,5000) == KetQuaChotMauImu::ThieuNgat &&
        mau.trangThai == TrangThaiKhungDuLieuImu::DuLieuCu && mau.thoiDiemLayMau_us == 0,
        "pollACK khi matIRQ khong duoc bien thanh mau moi");
    delay(5);
    kiemTra(camBien.docMauTheoNgat(mau) &&
        chotMauImu(mau,true,0,0,0,(0)+1500u,5000) == KetQuaChotMauImu::ThieuNgat,
        "kick khoi dong ACK pending nhung khong dung timestampgia");
    delay(5);
    kiemTra(camBien.docMauTheoNgat(mau) &&
        chotMauImu(mau,true,203,204,1015000,(1015000)+1500u,5000) == KetQuaChotMauImu::NgatChong &&
        mau.giaTocX_mps2 == 0, "ngat moi trong luc doc khong duocganthoigianmaucu");
    delay(5);
    kiemTra(camBien.docMauTheoNgat(mau) &&
        chotMauImu(mau,true,205,205,0,(0)+1500u,5000) == KetQuaChotMauImu::ThieuNgat,
        "khong chapnhan timestamp0");
    delay(5);
    bus.thanhGhiDocThieu=0x3A;
    kiemTra(!camBien.docMauTheoNgat(mau) && mau.trangThai == TrangThaiKhungDuLieuImu::LoiIO &&
        chotMauImu(mau,true,206,206,1030000,(1030000)+1500u,5000) == KetQuaChotMauImu::KhongCoMau &&
        camBien.maLoiI2cCuoi() == IcmI2cStatus::READ_SIZE_MISMATCH,
        "burst thieu1byte khong duocdecodehaypublish");
    bus.thanhGhiDocThieu=-1;
    bus.thanhGhiNack=0x3A;
    delay(5);
    kiemTra(!camBien.docMauTheoNgat(mau) && camBien.maLoiI2cCuoi() == IcmI2cStatus::ADDR_NACK &&
        bus.intDangGiuMuc(), "I2CNACK co ma rieng INT congiulevel");
    bus.thanhGhiNack=-1;
    kiemTra(camBien.docMauTheoNgat(mau) && !bus.intDangGiuMuc() &&
        chotMauImu(mau,false,206,206,1030000,(1030000)+1500u,5000) == KetQuaChotMauImu::ThieuNgat,
        "ACKphuchoi sauNACK chigiai phongINT khonggoimaufresh");
    delay(5);
    kiemTra(camBien.docMauTheoNgat(mau) &&
        chotMauImu(mau,true,207,207,1035000,(1035000)+1500u,5000) == KetQuaChotMauImu::ChapNhan,
        "sauphuchoi canngatmoi desudungmau");
    delay(5);
    kiemTra(camBien.docMauTheoNgat(mau) &&
        chotMauImu(mau,true,208,208,1040000,1046000,5000) == KetQuaChotMauImu::QuaChuKyMau &&
        mau.thoiDiemLayMau_us == 0, "seq khong doi vi latch vanphai loaiIRQcu hon1chuky");
    delay(5);
    kiemTra(camBien.docMauTheoNgat(mau) &&
        chotMauImu(mau,true,209,209,1045000,1050000,5000) == KetQuaChotMauImu::QuaChuKyMau,
        "doc dungbien5ms cungbi loai");
    delay(5);
    kiemTra(camBien.docMauTheoNgat(mau) &&
        chotMauImu(mau,true,210,210,1050000,1049999,5000) == KetQuaChotMauImu::QuaChuKyMau,
        "loai timestampIRQtuonglai");
    delay(5);
    kiemTra(camBien.docMauTheoNgat(mau) &&
        chotMauImu(mau,true,0,0,1055000,1056500,5000) == KetQuaChotMauImu::ChapNhan,
        "sequence0sauwrapvan hop le khi timestampISRton tai");
    bus.coDataReady=false;
    delay(5);
    kiemTra(!camBien.docMauTheoNgat(mau) && mau.trangThai == TrangThaiKhungDuLieuImu::DuLieuCu,
        "khongcongACK nhungkhongDATA_RDY");
}

void kiemTraTimeoutVaThongKeGiaoDich() {
    datLaiMoPhong();
    TwoWire bus;
    Icm20602Driver camBien(bus);
    kiemTra(camBien.begin() && bus.timeoutDaDat_ms == 5 && thoiGianChoI2c_ms == 5,
        "timeout5ms thuc su duoc truyen xuong Wire");
    kiemTra(camBien.hieuChinhConQuay(500), "bias truoc test thoi gian");
    bus.treDoc_ms = 4;
    delay(5);
    const uint64_t irq_us = dongHoGia_ms*1000;
    KhungDuLieuImu mau;
    kiemTra(camBien.docMauTheoNgat(mau), "doc4ms thanh cong voi timeout5");
    auto gd = camBien.giaoDichDocCuoi();
    kiemTra(gd.thanhGhi == 0x3A && gd.soByteYeuCau == 15 && gd.soByteNhan == 15 &&
        gd.soByteDaDoc == 15 && gd.thoiGian_us == 4000 && gd.maLoi == 0,
        "thong ke du15byte va4ms");
    kiemTra(chotMauImu(mau,true,1,1,irq_us,dongHoGia_ms*1000,5000) == KetQuaChotMauImu::ChapNhan,
        "mau4ms duoc nhan khi timestampIRQconmoi");
    delay(5);
    const uint64_t irqTre_us = dongHoGia_ms*1000;
    delay(2); // Task chờ 2 ms rồi giao dịch 4 ms: tuổi nguồn 6 ms.
    kiemTra(camBien.docMauTheoNgat(mau) && camBien.giaoDichDocCuoi().thoiGian_us == 4000 &&
        chotMauImu(mau,true,2,2,irqTre_us,dongHoGia_ms*1000,5000) == KetQuaChotMauImu::QuaChuKyMau,
        "timeout5khongnoichotmau5ms");
    bus.setTimeOut(2); // Tái tạo khác biệt cấu hình cũ trong mô hình bus giả.
    delay(5);
    kiemTra(!camBien.docMauTheoNgat(mau) && camBien.giaoDichDocCuoi().soByteNhan == 0 &&
        camBien.giaoDichDocCuoi().thoiGian_us == 2000 && camBien.maLoiI2cCuoi() == IcmI2cStatus::READ_SIZE_MISMATCH,
        "mau4ms voi timeout2 bi Wire tra0byte, khong doi code6 thanh5");
    bus.setTimeOut(5);
    bus.traRong = true;
    delay(5);
    kiemTra(!camBien.docMauTheoNgat(mau) && camBien.giaoDichDocLoiCuoi().soByteNhan == 0 &&
        camBien.giaoDichDocLoiCuoi().soByteDaDoc == 0,
        "phan biet read0byte va readthieu14byte");
    bus.traRong = false;
    bus.thanhGhiDocThieu = 0x3A;
    delay(5);
    kiemTra(!camBien.docMauTheoNgat(mau), "tu choi14trong15byte");
    const auto loi = camBien.giaoDichDocLoiCuoi();
    kiemTra(loi.soByteYeuCau == 15 && loi.soByteNhan == 14 && loi.soByteDaDoc == 0 && loi.maLoi == 6,
        "requestFromtra14bytes duocghi chinhxac");
    bus.thanhGhiDocThieu = -1;
    delay(5);
    kiemTra(camBien.docMauTheoNgat(mau) && camBien.giaoDichDocCuoi().maLoi == 0 &&
        camBien.giaoDichDocLoiCuoi().soByteNhan == 14 && camBien.giaoDichDocLoiCuoi().maLoi == 6,
        "lan doc tot khong xoa snapshotloi");
    bus.hutKhiDocByte = 7;
    delay(5);
    kiemTra(!camBien.docMauTheoNgat(mau) && camBien.giaoDichDocCuoi().soByteNhan == 15 &&
        camBien.giaoDichDocCuoi().soByteDaDoc == 7 && camBien.giaoDichDocCuoi().maLoi == 6,
        "readhut sau requestFromdu15duoc phanbietbangused");
    bus.hutKhiDocByte = -1;
    // micros() uint32 quay vòng trong giao dịch, nhưng thời gian vẫn là 4 ms.
    dongHoGia_ms = 4294966;
    kiemTra(camBien.docMauTheoNgat(mau) && camBien.giaoDichDocCuoi().thoiGian_us == 4000,
        "do thoi gian dung khi microswrap");
    bus.treDoc_ms = 0;
    delay(5);
    kiemTra(camBien.docMauTheoNgat(mau) && camBien.giaoDichDocCuoi().thoiGian_us == 0 &&
        camBien.giaoDichDocLoiCuoi().soByteDaDoc == 7,
        "giao dich hien tai reset, snapshotloi van giu");
}
}

int main() {
    kiemTraBoot();
    kiemTraLoiBusVaMau();
    kiemTraThuLaiGyro();
    kiemTraUIVoiRung();
    kiemTraNgatVaMauRuntime();
    kiemTraTimeoutVaThongKeGiaoDich();
    std::cout << "PASS: ICM boot, bus faults and calibration retry " << soDieuKien << " checks\n";
}
