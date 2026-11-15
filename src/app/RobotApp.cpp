#include "RobotApp.h"
#include "config/ControlConfig.h"
#include "config/CalibConfig.h"
#include "config/MotorConfig.h"
#include "config/PinConfig.h"
#include "config/ProjectConfig.h"
#include "config/SafetyConfig.h"
#include "config/WifiConfig.h"
#include "safety/SafetyMonitor.h"
#include "driver/gpio.h"
#include "calib/UI_Calib.h"
#include "core/ChotMauImu.h"
#include "ThongKeMauImu.h"
#include <Preferences.h>
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
constexpr uint32_t suKienDinhKy = 1u, suKienBatDongCo = 2u, suKienTatDongCo = 4u, suKienDungKhan = 8u,
                   suKienXoaLoi = 16u, suKienDoiCheDoMo = 32u, suKienChonPidMo = 64u, suKienChonPidThuan = 128u,
                   suKienResetXung = 256u;
constexpr char nvsNamespace[] = "xecanbang";
constexpr uint32_t nvsMagic = 0x58434232u; // "XCB2"
constexpr uint16_t soMauCalibImuTask = 400; // 2.0s @ 200Hz
constexpr uint16_t soMauCalibBoot = 300;    // 1.5s @ 200Hz
constexpr float doiRadSangDo = 57.295779513f;
constexpr float doiDoSangRad = 0.01745329252f;
uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
const char* faultName(FaultCode maLoi) {
    switch (maLoi) {
    case FaultCode::None: return "none";
    case FaultCode::ImuInvalid: return "imu_invalid";
    case FaultCode::ImuStale: return "imu_stale";
    case FaultCode::PulseInvalid: return "pulse_invalid";
    case FaultCode::PulseStale: return "pulse_stale";
    case FaultCode::Fallen: return "fallen";
    case FaultCode::InvalidControl: return "control_invalid";
    case FaultCode::MotorFault: return "motor_fault";
    case FaultCode::EmergencyStop: return "emergency";
    case FaultCode::ArmTimeout: return "arm_timeout";
    }
    return "unknown";
}
}

void IRAM_ATTR RobotApp::imuInterrupt(void* nguCanh) {
    auto* ungDung = static_cast<RobotApp*>(nguCanh);
    const uint64_t thoiDiemMau_us = static_cast<uint64_t>(esp_timer_get_time());
    portENTER_CRITICAL_ISR(&ungDung->_khoaDuLieuNgatImu);
    ungDung->_thoiDiemNgat_us = thoiDiemMau_us;
    ++ungDung->_soThuTuNgat;
    portEXIT_CRITICAL_ISR(&ungDung->_khoaDuLieuNgatImu);
    BaseType_t canChuyenTacVu = pdFALSE;
    vTaskNotifyGiveFromISR(ungDung->_tacVuImu, &canChuyenTacVu);
    if (canChuyenTacVu) portYIELD_FROM_ISR();
}
void RobotApp::motorTimer(void* nguCanh) {
    auto* ungDung = static_cast<RobotApp*>(nguCanh);
    xTaskNotify(ungDung->_tacVuDongCo, suKienDinhKy, eSetBits);
}
void RobotApp::imuEntry(void* nguCanh) { static_cast<RobotApp*>(nguCanh)->imuLoop(); }
void RobotApp::controlEntry(void* nguCanh) { static_cast<RobotApp*>(nguCanh)->controlLoop(); }
void RobotApp::motorEntry(void* nguCanh) { static_cast<RobotApp*>(nguCanh)->motorLoop(); }

void RobotApp::cleanupStartup() {
    if (_boHenDongCo) { esp_timer_stop(_boHenDongCo); esp_timer_delete(_boHenDongCo); _boHenDongCo = nullptr; }
    detachInterrupt(chanNgatMpu);
    if (_tacVuDongCo) { vTaskDelete(_tacVuDongCo); _tacVuDongCo = nullptr; }
    if (_tacVuDieuKhien) { vTaskDelete(_tacVuDieuKhien); _tacVuDieuKhien = nullptr; }
    if (_tacVuImu) { vTaskDelete(_tacVuImu); _tacVuImu = nullptr; }
    _capDongCo.disable();
    if (_hangDoiImu) { vQueueDelete(_hangDoiImu); _hangDoiImu = nullptr; }
    if (_hangDoiTrangThaiDongCo) { vQueueDelete(_hangDoiTrangThaiDongCo); _hangDoiTrangThaiDongCo = nullptr; }
    if (_hangDoiTrangThaiDieuKhien) { vQueueDelete(_hangDoiTrangThaiDieuKhien); _hangDoiTrangThaiDieuKhien = nullptr; }
    if (_hangDoiDiChuyen) { vQueueDelete(_hangDoiDiChuyen); _hangDoiDiChuyen = nullptr; }
    if (_hangDoiChanDoanImu) { vQueueDelete(_hangDoiChanDoanImu); _hangDoiChanDoanImu = nullptr; }
    if (_hangDoiCauHinh) { vQueueDelete(_hangDoiCauHinh); _hangDoiCauHinh = nullptr; }
    if (_hangDoiYeuCauCalib) { vQueueDelete(_hangDoiYeuCauCalib); _hangDoiYeuCauCalib = nullptr; }
    if (_hangDoiKetQuaCalib) { vQueueDelete(_hangDoiKetQuaCalib); _hangDoiKetQuaCalib = nullptr; }
}

bool RobotApp::begin() {
    Serial.begin(115200);
    _cauHinhChay = CauHinhChay{};
    bool daNapNvs = false;
    {
        Preferences prefs;
        if (prefs.begin(nvsNamespace, true)) {
            if (prefs.getUInt("magic", 0u) == nvsMagic) {
                _cauHinhChay.kpGocThuan = prefs.getFloat("kp_p", kpGocPidThuan);
                _cauHinhChay.kiGocThuan = prefs.getFloat("ki_p", kiGocPidThuan);
                _cauHinhChay.kdGocThuan = prefs.getFloat("kd_p", kdGocPidThuan);
                _cauHinhChay.kpVanTocThuan = prefs.getFloat("kpv_p", kpVanTocPidThuan);
                _cauHinhChay.kiVanTocThuan = prefs.getFloat("kiv_p", kiVanTocPidThuan);
                _cauHinhChay.kpGocMo = prefs.getFloat("kp_m", kpGoc);
                _cauHinhChay.kiGocMo = prefs.getFloat("ki_m", kiGoc);
                _cauHinhChay.kdGocMo = prefs.getFloat("kd_m", kdGoc);
                _cauHinhChay.kpVanTocMo = prefs.getFloat("kpv_m", kpVanToc);
                _cauHinhChay.kiVanTocMo = prefs.getFloat("kiv_m", kiVanToc);
                _cauHinhChay.gocCanBang_rad = std::clamp(prefs.getFloat("ref_rad", ::gocCanBang_rad), -0.35f, 0.35f);
                _cauHinhChay.dungPidMo = prefs.getBool("mode_mo", batPidMo);
                _cauHinhChay.daoChieuDongCo = prefs.getBool("inv_dir", false);
                _coBiasLuu = prefs.getBool("gb_ok", false);
                if (_coBiasLuu) {
                    _biasX_dps = prefs.getFloat("gb_x", 0.0f);
                    _biasY_dps = prefs.getFloat("gb_y", 0.0f);
                    _biasZ_dps = prefs.getFloat("gb_z", 0.0f);
                    if (!std::isfinite(_biasX_dps) || !std::isfinite(_biasY_dps) || !std::isfinite(_biasZ_dps)) {
                        _coBiasLuu = false;
                        _biasX_dps = _biasY_dps = _biasZ_dps = 0.0f;
                    }
                }
                daNapNvs = true;
            }
            prefs.end();
        }
    }
    const auto chanChoPhepDongCo = static_cast<gpio_num_t>(chanBatMotor);
    // Kéo nội bộ về mức tắt (L1); vẫn cần điện trở kéo phần cứng cho giai đoạn bootloader.
    if ((mucBatMotorCao ? gpio_pulldown_en(chanChoPhepDongCo) : gpio_pullup_en(chanChoPhepDongCo)) != ESP_OK ||
        (mucBatMotorCao ? gpio_pullup_dis(chanChoPhepDongCo) : gpio_pulldown_dis(chanChoPhepDongCo)) != ESP_OK ||
        gpio_set_level(chanChoPhepDongCo, mucBatMotorCao ? 0 : 1) != ESP_OK ||
        gpio_set_direction(chanChoPhepDongCo, GPIO_MODE_OUTPUT) != ESP_OK) return false;
    UI_Calib uiCalib(_camBienGoc);
    uiCalib.khoiTao();
    if (!batUiCalib && !_boUocLuongGoc.heTrucHopLe()) {
        Serial.println("BOOT ERROR: ma tran lap IMU khong phai phep quay hop le");
        return false;
    }
    if (!batNgatMpu || chuKyPhucVuMotor_us >= khoangCachPhucVuMotorToiDa_us ||
        1000000u / tanSoLayMauIcm_hz >= thoiGianHetHanLenhMotor_us) return false;
    if (!_capDongCo.begin() || !_boDemXung.begin()) {
        uiCalib.baoLoi();
        Serial.println("BOOT ERROR: MCPWM/PCNT"); return false;
    }
    if (!_camBienGoc.begin()) {
        uiCalib.baoLoi();
        Serial.printf("BOOT ERROR: ICM20602 init WHO=0x%02X expected=0x12 i2c=%u reg=0x%02X\n",
            _camBienGoc.maNhanDang(), static_cast<unsigned>(_camBienGoc.maLoiI2cCuoi()), _camBienGoc.thanhGhiCuoi());
        return false;
    }
    Serial.println("ICM_BOOT: kiem tra ID, cau hinh va 32 mau moi; motor tat.");
    BaoCaoKiemTraIcm baoCaoImu;
    const bool camBienDat = _camBienGoc.kiemTraSauKhoiDong(baoCaoImu);
    Serial.printf("ICM_BOOT %s WHO=0x%02X good=%u bad=%u time_ms=%lu i2c=%u reg=0x%02X\n",
        baoCaoImu.ketQua, _camBienGoc.maNhanDang(), baoCaoImu.soMauTot, baoCaoImu.soMauLoi,
        static_cast<unsigned long>(baoCaoImu.thoiGian_ms),
        static_cast<unsigned>(_camBienGoc.maLoiI2cCuoi()), _camBienGoc.thanhGhiCuoi());
    const auto& mauKiemTra = baoCaoImu.mauCuoi;
    Serial.printf("ICM_BOOT_SAMPLE a_mps2=%.4f,%.4f,%.4f gyro_radps=%.4f,%.4f,%.4f T_C=%.2f\n",
        mauKiemTra.giaTocX_mps2, mauKiemTra.giaTocY_mps2, mauKiemTra.giaTocZ_mps2,
        mauKiemTra.tocDoGocX_radps, mauKiemTra.tocDoGocY_radps, mauKiemTra.tocDoGocZ_radps,
        mauKiemTra.nhietDoCamBien_C);
    if (!camBienDat) {
        uiCalib.baoLoi();
        Serial.println("BOOT ERROR: kiem tra du lieu ICM20602 khong dat; khong vao calib/can bang.");
        return false;
    }
    // Dùng nút BOOT (chanNutCalibPhu = GPIO0) thay vì GPIO3 vì GPIO3 là RX1 từ MKS 1.
    bool giuNutDeCalib3TuThe = false;
    if (chanNutCalibPhu >= 0) {
        pinMode(chanNutCalibPhu, nutCalibMucThap ? INPUT_PULLUP : INPUT_PULLDOWN);
        delay(5);
        giuNutDeCalib3TuThe = (digitalRead(chanNutCalibPhu) == (nutCalibMucThap ? LOW : HIGH));
    }
    if (batUiCalib || giuNutDeCalib3TuThe) {
        MaTranQuayImu maTranLap;
        uiCalib.chay(maTranLap);
        _heTrucImu = HeTrucImu(maTranLap);
        _boUocLuongGoc = ComplementaryEstimator(_heTrucImu);
        _camBienGoc.layDoLechConQuay_dps(_biasX_dps, _biasY_dps, _biasZ_dps, _coBiasLuu);
    } else {
        _heTrucImu = HeTrucImu();
        _boUocLuongGoc = ComplementaryEstimator(_heTrucImu);
        Serial.printf("FAST_BOOT: Su dung ma tran lap IMU trong ImuMountConfig.h (giu GPIO%d luc bat nguon neu muon calib 3 tu the).\n", chanNutCalibPhu);
        Serial.println("FAST_BOOT: Dang do bias gyro (1.5s)...");
        if (_camBienGoc.hieuChinhConQuay(soMauCalibBoot)) {
            _camBienGoc.layDoLechConQuay_dps(_biasX_dps, _biasY_dps, _biasZ_dps, _coBiasLuu);
        } else {
            if (_coBiasLuu) {
                _camBienGoc.datDoLechConQuay_dps(_biasX_dps, _biasY_dps, _biasZ_dps);
                Serial.printf("WARN: gyro boot rung; tam nap bias tu NVS (%.3f,%.3f,%.3f dps). Gui 'c' khi xe yen de do lai.\n",
                              _biasX_dps, _biasY_dps, _biasZ_dps);
            } else {
                _camBienGoc.datDoLechConQuay_dps(0.0f, 0.0f, 0.0f);
                Serial.println("WARN: gyro boot rung; tam dung bias=0. Hay dat xe yen va gui 'c' qua Serial de calib khoa hoc.");
            }
        }
    }
    if (!_boDemXung.beginUartMks()) {
        if (batUiCalib) uiCalib.baoLoi();
        Serial.println("BOOT ERROR: UART MKS feedback init failed");
        return false;
    }
    if (!dungPhanCungTmcMt && batUartEncoderMks) {
        Serial.printf("MKS_UART san sang: TX=IO%d, RX1_L=IO%d (0x%02X), RX2_R=IO%d (0x%02X), %lubps, 65536/rev.\n",
                      chanUartTxMks, chanUartRxMksTrai, diaChiUartMksTrai,
                      chanUartRxMksPhai, diaChiUartMksPhai,
                      static_cast<unsigned long>(tocDoBaudUartMks_bps));
    }
    _hangDoiImu = xQueueCreate(1, sizeof(KhungDuLieuImu));
    _hangDoiTrangThaiDongCo = xQueueCreate(1, sizeof(MotorStatus));
    _hangDoiTrangThaiDieuKhien = xQueueCreate(1, sizeof(ControlStatus));
    _hangDoiDiChuyen = xQueueCreate(1, sizeof(LenhDiChuyen));
    _hangDoiChanDoanImu = xQueueCreate(1, sizeof(ChanDoanLuongImu));
    _hangDoiCauHinh = xQueueCreate(1, sizeof(CauHinhChay));
    _hangDoiYeuCauCalib = xQueueCreate(1, sizeof(YeuCauCalibImu));
    _hangDoiKetQuaCalib = xQueueCreate(1, sizeof(KetQuaCalibImu));
    if (!_hangDoiImu || !_hangDoiTrangThaiDongCo || !_hangDoiTrangThaiDieuKhien || !_hangDoiDiChuyen ||
        !_hangDoiChanDoanImu || !_hangDoiCauHinh || !_hangDoiYeuCauCalib || !_hangDoiKetQuaCalib) {
        if (batUiCalib) uiCalib.baoLoi();
        cleanupStartup(); return false;
    }
    apDungCauHinh(_cauHinhChay);
    xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
    MotorStatus trangThaiBanDau{};
    trangThaiBanDau.trangThaiBoDieuKhienDongCo = _capDongCo.snapshot();
    trangThaiBanDau.khungXung = _boDemXung.read();
    trangThaiBanDau.dungPidMo = _cauHinhChay.dungPidMo;
    xQueueOverwrite(_hangDoiTrangThaiDongCo, &trangThaiBanDau);
    if (xTaskCreatePinnedToCore(imuEntry, "Imu", 4096, this, 4, &_tacVuImu, 1) != pdPASS ||
        xTaskCreatePinnedToCore(controlEntry, "Control", 6144, this, 3, &_tacVuDieuKhien, 1) != pdPASS ||
        xTaskCreatePinnedToCore(motorEntry, "Motor", 4096, this, 5, &_tacVuDongCo, 1) != pdPASS) {
        if (batUiCalib) uiCalib.baoLoi();
        cleanupStartup(); return false;
    }
    pinMode(chanNgatMpu, INPUT);
    attachInterruptArg(chanNgatMpu, imuInterrupt, this, mucNgatMpuCao ? RISING : FALLING);
    // Nếu INT đã giữ mức trước attach, chưa có cạnh mới. Chỉ ImuTask ACK cờ tồn
    // để mở lại cạnh kế tiếp; không xuất mẫu hợp lệ với timestamp giả ở lần đánh thức này.
    xTaskNotifyGive(_tacVuImu);
    esp_timer_create_args_t cauHinhBoHen{};
    cauHinhBoHen.callback = motorTimer;
    cauHinhBoHen.arg = this;
    cauHinhBoHen.dispatch_method = ESP_TIMER_TASK;
    cauHinhBoHen.name = "MotorWake";
    cauHinhBoHen.skip_unhandled_events = true;
    if (esp_timer_create(&cauHinhBoHen, &_boHenDongCo) != ESP_OK ||
        esp_timer_start_periodic(_boHenDongCo, chuKyPhucVuMotor_us) != ESP_OK) {
        if (batUiCalib) uiCalib.baoLoi();
        cleanupStartup(); return false;
    }
    // Không phát lại phím Serial đã gửi trong khi đang làm hiệu chuẩn.
    const int soByteCho = Serial.available();
    for (int chiSo = 0; chiSo < soByteCho; ++chiSo) Serial.read();
    _daSanSang = true;
    if (batUiCalib) uiCalib.baoChoDuLieu();
    const bool khongDayOk = dieuKhienKhongDay.khoiTao(
        [this](float vanTocTienLui, float tocDoQuayNgang) {
            nhanLenhDiChuyenTuXa(vanTocTienLui, tocDoQuayNgang);
        },
        [this](char kyTuLenh) { xuLyPhimDonSerial(kyTuLenh); },
        [this](const char* dongLenh) { xuLyDongLenhSerial(dongLenh); },
        [this](ThongTinTrangThaiWeb& trangThaiRa) { dienTrangThaiWeb(trangThaiRa); });
    inDinhDangSerial("FIRMWARE %s; INT latched, runtime burst INT_STATUS+14 byte; I2C=%luHz timeout=%ums; NVS=%s; mode=%s\n",
        phienBanFirmware,static_cast<unsigned long>(tanSoI2cIcm_hz),thoiGianChoI2c_ms,
        daNapNvs ? "LOADED" : "DEFAULT", _cauHinhChay.dungPidMo ? "PID_MO" : "PID_THUAN");
    if (khongDayOk || dieuKhienKhongDay.wifiSanSang() || dieuKhienKhongDay.espNowSanSang()) {
        inDinhDangSerial("WIFI_AP SSID=%s PASS=%s URL=http://%s | ESP_NOW=%s MAC=%s CH=%u\n",
                      tenMangWifiAp, matKhauWifiAp, dieuKhienKhongDay.diaChiIpAp(),
                      dieuKhienKhongDay.espNowSanSang() ? "ON" : "OFF",
                      dieuKhienKhongDay.diaChiMacXe(), static_cast<unsigned>(kenhWifiAp));
    }
    if (dungPhanCungTmcMt) {
        inDinhDangSerial("READY ICM20602 %uHz; odometry=MT6701_AB_4x(%lu); driver=TMC2209_ClosedLoop; hardwareConfirmed=%d\n",
                      tanSoLayMauIcm_hz, static_cast<unsigned long>(soXungEncoderMoiVongDongCo),
                      daXacNhanCauHinhPhanCungMotor);
    } else {
        inDinhDangSerial("READY ICM20602 %uHz; odometry=STEP/DIR_PCNT+MKS_V1.1_UART(TX=IO%d,RX1=IO%d,RX2=IO%d,%lubps); MKS=42C_V1.1 MStep=%lu; hardwareConfirmed=%d\n",
                      tanSoLayMauIcm_hz, chanUartTxMks, chanUartRxMksTrai, chanUartRxMksPhai,
                      static_cast<unsigned long>(tocDoBaudUartMks_bps),
                      static_cast<unsigned long>(soViBuocDauVaoMks), daXacNhanCauHinhPhanCungMotor);
    }
    inTroGiup();
    return true;
}

void RobotApp::imuLoop() {
    BoLocThuTuMau boLocThuTu;
    uint32_t soLanDocChongNgat = 0;
    ChanDoanLuongImu chanDoan;
    ThongKeMauImu thongKeCalib;
    bool dangCalib = false;
    uint32_t maYeuCauCalibDangXuLy = 0;
    uint32_t lucBatDauCalib_ms = 0;
    uint16_t soLanThuLaiCalib = 0;
    float cuBiasX_dps = 0.0f, cuBiasY_dps = 0.0f, cuBiasZ_dps = 0.0f;

    for (;;) {
        YeuCauCalibImu yeuCauCalib{};
        if (xQueuePeek(_hangDoiYeuCauCalib, &yeuCauCalib, 0) == pdTRUE &&
            yeuCauCalib.maYeuCau != 0 && yeuCauCalib.maYeuCau != maYeuCauCalibDangXuLy) {
            maYeuCauCalibDangXuLy = yeuCauCalib.maYeuCau;
            dangCalib = true;
            lucBatDauCalib_ms = millis();
            soLanThuLaiCalib = 0;
            thongKeCalib.xoa();
            bool daHieuChinhTmp = false;
            _camBienGoc.layDoLechConQuay_dps(cuBiasX_dps, cuBiasY_dps, cuBiasZ_dps, daHieuChinhTmp);
        }

        KhungDuLieuImu khungDuLieu{};
        const bool coThongBao = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) != 0;
        portENTER_CRITICAL(&_khoaDuLieuNgatImu);
        const uint64_t thoiDiemMau_us = _thoiDiemNgat_us;
        const uint32_t soThuTuMau = _soThuTuNgat;
        portEXIT_CRITICAL(&_khoaDuLieuNgatImu);
        chanDoan.soThuTuNgat = soThuTuMau;
        chanDoan.thoiDiemNgatCuoi_us = thoiDiemMau_us;
        if (coThongBao && thoiDiemMau_us && !boLocThuTu.chapNhan(soThuTuMau)) {
            xQueueOverwrite(_hangDoiChanDoanImu,&chanDoan);
            continue;
        }
        if (!coThongBao) {
            ++chanDoan.soLanHetChoNgat;
            ++chanDoan.soLanAckPhucHoi;
        }
        // Cùng một burst ACK status và đọc mẫu. Khi hết chờ, ACK chỉ để giải
        // phóng đường INT nếu lần đọc trước lỗi; chotMauImu không cho dùng mẫu poll.
        _camBienGoc.docMauTheoNgat(khungDuLieu);
        chanDoan.giaoDichCuoi = _camBienGoc.giaoDichDocCuoi();
        if (chanDoan.giaoDichCuoi.thoiGian_us > chanDoan.thoiGianDocLonNhat_us)
            chanDoan.thoiGianDocLonNhat_us = chanDoan.giaoDichCuoi.thoiGian_us;
        if (khungDuLieu.trangThai == TrangThaiKhungDuLieuImu::LoiIO) {
            ++chanDoan.soLoiI2c;
            chanDoan.maLoiI2cCuoi = static_cast<uint8_t>(_camBienGoc.maLoiI2cCuoi());
            chanDoan.thanhGhiLoiCuoi = _camBienGoc.thanhGhiCuoi();
            chanDoan.giaoDichLoiCuoi = _camBienGoc.giaoDichDocLoiCuoi();
        } else if (coThongBao && thoiDiemMau_us && khungDuLieu.trangThai == TrangThaiKhungDuLieuImu::DuLieuCu)
            ++chanDoan.soLanKhongCoDataReady;
        portENTER_CRITICAL(&_khoaDuLieuNgatImu);
        const uint32_t soThuTuSauDoc = _soThuTuNgat;
        portEXIT_CRITICAL(&_khoaDuLieuNgatImu);
        const auto ketQuaMau = chotMauImu(khungDuLieu,coThongBao,soThuTuMau,soThuTuSauDoc,
            thoiDiemMau_us,nowUs(),1000000u/tanSoLayMauIcm_hz);
        if (ketQuaMau == KetQuaChotMauImu::NgatChong) ++soLanDocChongNgat;
        if (ketQuaMau == KetQuaChotMauImu::QuaChuKyMau) ++chanDoan.soLanDocQuaChuKy;
        if (ketQuaMau == KetQuaChotMauImu::ChapNhan) {
            ++chanDoan.soMauTot;
            chanDoan.thoiDiemDocTotCuoi_us = nowUs();
        }
        chanDoan.soLanDocChongNgat = soLanDocChongNgat;
        chanDoan.trangThaiDocCuoi = khungDuLieu.trangThai;
        xQueueOverwrite(_hangDoiChanDoanImu,&chanDoan);
        // Bỏ một mẫu không chắc timestamp, giữ cơ chế giám sát tuổi dữ liệu hiện hành.
        // Queue chẩn đoán ở trên vẫn có nhịp nếu mọi mẫu đều bị overlap/đọc trễ.
        if (ketQuaMau == KetQuaChotMauImu::NgatChong || ketQuaMau == KetQuaChotMauImu::QuaChuKyMau) continue;

        if (dangCalib) {
            if (ketQuaMau == KetQuaChotMauImu::ChapNhan) {
                KhungDuLieuImu mauTho = khungDuLieu;
                mauTho.tocDoGocX_radps += cuBiasX_dps * doiDoSangRad;
                mauTho.tocDoGocY_radps += cuBiasY_dps * doiDoSangRad;
                mauTho.tocDoGocZ_radps += cuBiasZ_dps * doiDoSangRad;
                if (!mauImuGanTinh(mauTho, gioiHanGyroThoCalib_dps * doiDoSangRad)) {
                    thongKeCalib.xoa();
                    ++soLanThuLaiCalib;
                } else {
                    thongKeCalib.themMau(mauTho);
                    if (thongKeCalib.soMau() >= soMauCalibImuTask) {
                        if (thongKeCalib.onDinh()) {
                            const float bx = thongKeCalib.trungBinh(3) / doiDoSangRad;
                            const float by = thongKeCalib.trungBinh(4) / doiDoSangRad;
                            const float bz = thongKeCalib.trungBinh(5) / doiDoSangRad;
                            const Vec3Imu aXe = _heTrucImu.chuyenSangXe({
                                thongKeCalib.trungBinh(0),
                                thongKeCalib.trungBinh(1),
                                thongKeCalib.trungBinh(2)});
                            const float gocCanBangDo_rad = std::atan2(-aXe.x, std::hypot(aXe.y, aXe.z));
                            KetQuaCalibImu ketQua{};
                            ketQua.maYeuCau = maYeuCauCalibDangXuLy;
                            ketQua.hoanTat = true;
                            ketQua.doLechChuanGyro_dps = thongKeCalib.doLechChuanGyroLonNhat_dps();
                            ketQua.phuongSaiGiaToc = thongKeCalib.phuongSaiGiaToc();
                            ketQua.soMauDaThu = static_cast<uint16_t>(thongKeCalib.soMau());
                            ketQua.soLanThuLai = soLanThuLaiCalib;
                            if (std::isfinite(gocCanBangDo_rad) && std::fabs(gocCanBangDo_rad) <= 0.35f) {
                                _camBienGoc.datDoLechConQuay_dps(bx, by, bz);
                                ketQua.thanhCong = true;
                                ketQua.biasX_dps = bx;
                                ketQua.biasY_dps = by;
                                ketQua.biasZ_dps = bz;
                                ketQua.gocCanBangDo_rad = gocCanBangDo_rad;
                            }
                            xQueueOverwrite(_hangDoiKetQuaCalib, &ketQua);
                            dangCalib = false;
                        } else {
                            thongKeCalib.xoa();
                            ++soLanThuLaiCalib;
                        }
                    }
                }
            }
            if (dangCalib && (millis() - lucBatDauCalib_ms >= 15000u)) {
                KetQuaCalibImu ketQua{};
                ketQua.maYeuCau = maYeuCauCalibDangXuLy;
                ketQua.hoanTat = true;
                ketQua.thanhCong = false;
                ketQua.soMauDaThu = static_cast<uint16_t>(thongKeCalib.soMau());
                ketQua.soLanThuLai = soLanThuLaiCalib;
                xQueueOverwrite(_hangDoiKetQuaCalib, &ketQua);
                dangCalib = false;
            }
        }

        khungDuLieu.chanDoan = chanDoan;
        khungDuLieu.soMauBoLo = boLocThuTu.soMauBoLo();
        khungDuLieu.soThongBaoTrung = boLocThuTu.soThongBaoTrung();
        khungDuLieu.soLanDocChongNgat = soLanDocChongNgat;
        xQueueOverwrite(_hangDoiImu, &khungDuLieu);
    }
}

void RobotApp::controlLoop() {
    TrangThaiXe trangThaiXe{};
    uint32_t soThuTuLenh = 0;
    uint32_t maPhienTruoc = 0;
    uint32_t phienBanCauHinhTruoc = 0;
    uint32_t soLanChotViTriTruoc = 0;
    uint32_t maResetXungTruoc = 0;
    uint64_t thoiDiemXungDaXuLy_us = 0;
    for (;;) {
        KhungDuLieuImu khungDuLieu{};
        xQueueReceive(_hangDoiImu, &khungDuLieu, portMAX_DELAY);
        CauHinhChay cauHinhMoi{};
        if (xQueuePeek(_hangDoiCauHinh, &cauHinhMoi, 0) == pdTRUE &&
            cauHinhMoi.phienBan != phienBanCauHinhTruoc) {
            apDungCauHinh(cauHinhMoi);
            if (cauHinhMoi.soLanChotViTri != soLanChotViTriTruoc) {
                _boDieuKhienTang.chotViTriGiuTaiCho();
                soLanChotViTriTruoc = cauHinhMoi.soLanChotViTri;
            }
            phienBanCauHinhTruoc = cauHinhMoi.phienBan;
        }
        MotorStatus trangThaiDongCo{};
        xQueuePeek(_hangDoiTrangThaiDongCo, &trangThaiDongCo, 0);
        if (trangThaiDongCo.maResetXung != maResetXungTruoc) {
            _boQuyDoiBanhXe.reset();
            _boDieuKhienTang.reset();
            thoiDiemXungDaXuLy_us = 0;
            maResetXungTruoc = trangThaiDongCo.maResetXung;
        }
        // M5: lệnh của chu kỳ trước đang được bánh thực hiện trong khoảng mẫu này.
        // Controller bị reset khi motor không chạy nên phần bù khi đó bằng 0.
        // Ở Bench Mode (nhấc bánh khỏi sàn), bánh quay không tạo gia tốc tịnh tiến thân xe nên không trừ vào accel IMU.
        const float giaTocTinhTien_mps2 =
            (buGiaTocLenhVaoImu && !_boDieuKhienTang.cheDoThuKhongMaSat())
                ? _boDieuKhienTang.giaTocBanhDaLenh_mps2()
                : 0.0f;
        _boUocLuongGoc.update(khungDuLieu, trangThaiXe, giaTocTinhTien_mps2);
        // Chỉ gọi _boQuyDoiBanhXe.update khi có mẫu xung mới (hoặc chưa có vận tốc/mẫu lỗi),
        // tránh việc hai chu kỳ IMU đọc cùng một snapshot xung gây xóa cờ vanTocHopLe.
        if (!trangThaiDongCo.khungXung.hopLe || !trangThaiXe.vanTocHopLe ||
            trangThaiDongCo.khungXung.thoiDiem_us != thoiDiemXungDaXuLy_us) {
            _boQuyDoiBanhXe.update(trangThaiDongCo.khungXung, trangThaiXe);
            if (trangThaiDongCo.khungXung.hopLe) {
                thoiDiemXungDaXuLy_us = trangThaiDongCo.khungXung.thoiDiem_us;
            }
        }
        ControlStatus baoCaoDieuKhien{};
        baoCaoDieuKhien.trangThaiXe = trangThaiXe;
        baoCaoDieuKhien.khungCamBien = khungDuLieu;
        baoCaoDieuKhien.maPhien = trangThaiDongCo.maPhien;
        baoCaoDieuKhien.soMauBoLo = khungDuLieu.soMauBoLo;
        baoCaoDieuKhien.soThongBaoTrung = khungDuLieu.soThongBaoTrung;
        baoCaoDieuKhien.soLanDocChongNgat = khungDuLieu.soLanDocChongNgat;
        baoCaoDieuKhien.gocCanBang_rad = _boDieuKhienTang.gocCanBangThucTe_rad();
        const uint64_t thoiDiemHienTai_us = nowUs();
        baoCaoDieuKhien.loi = SafetyMonitor::check(
            trangThaiXe, thoiDiemHienTai_us, _boDieuKhienTang.gocCanBangThucTe_rad());
        // Cho phép luân chuyển êm (bumpless transfer) giữa PID_MO và PID_THUAN ngay cả khi đang cân bằng.
        if (trangThaiDongCo.cheDo != CheDoCanBang::DangChay ||
            trangThaiDongCo.maPhien != maPhienTruoc ||
            baoCaoDieuKhien.loi != FaultCode::None)
            _boDieuKhienTang.reset();
        maPhienTruoc = trangThaiDongCo.maPhien;
        LenhDiChuyen lenhDiChuyen{};
        xQueuePeek(_hangDoiDiChuyen, &lenhDiChuyen, 0);
        const bool choDiChuyen = trangThaiDongCo.cheDo == CheDoCanBang::DangChay &&
            lenhDiChuyen.hopLe(trangThaiDongCo.maPhien, thoiDiemHienTai_us);
        if (baoCaoDieuKhien.loi == FaultCode::None &&
            !_boDieuKhienTang.update(trangThaiXe, thoiDiemHienTai_us, trangThaiDongCo.dungPidMo,
                baoCaoDieuKhien.lenhBanh, choDiChuyen ? lenhDiChuyen.vanToc_mps : 0,
                choDiChuyen ? lenhDiChuyen.tocDoQuay_radps : 0))
            baoCaoDieuKhien.loi = FaultCode::InvalidControl;
        baoCaoDieuKhien.heSoPid = _boDieuKhienTang.heSoPid();
        baoCaoDieuKhien.vongKinBanh = _boDieuKhienTang.trangThaiVongKinBanh();
        baoCaoDieuKhien.gocDat_rad = _boDieuKhienTang.gocDat_rad();
        baoCaoDieuKhien.gyroDaLoc_radps = _boDieuKhienTang.tocDoDaLoc_radps();
        baoCaoDieuKhien.vanTocMucTieu_mps = _boDieuKhienTang.vanTocMucTieu_mps();
        baoCaoDieuKhien.tocDoQuayMucTieu_radps = _boDieuKhienTang.tocDoQuayMucTieu_radps();
        baoCaoDieuKhien.soThuTuLenh = ++soThuTuLenh;
        xQueueOverwrite(_hangDoiTrangThaiDieuKhien, &baoCaoDieuKhien);
    }
}

void RobotApp::motorLoop() {
    MotorStatus baoCaoDongCo{};
    PhienCanBang phienCanBang;
    CauHinhChay cauHinhBanDau{};
    if (xQueuePeek(_hangDoiCauHinh, &cauHinhBanDau, 0) == pdTRUE) {
        baoCaoDongCo.dungPidMo = cauHinhBanDau.dungPidMo;
    } else {
        baoCaoDongCo.dungPidMo = batPidMo;
    }
    bool daoChieuDongCoTruoc = cauHinhBanDau.daoChieuDongCo;
    uint32_t soThuTuLenhDaApDung = 0;
    uint64_t thoiDiemPhucVuTruoc_us = 0;
    uint64_t thoiDiemDocXungTruoc_us = 0;
    for (;;) {
        uint32_t cacSuKien = 0;
        xTaskNotifyWait(0, UINT32_MAX, &cacSuKien, pdMS_TO_TICKS(1));
        // Nhận lỗi ISR giám sát trước khi xét bất kỳ yêu cầu bật lại nào.
        if (_capDongCo.service() == MksResult::FaultLatched) {
            if (baoCaoDongCo.loi == FaultCode::None) baoCaoDongCo.loi = FaultCode::MotorFault;
            if (phienCanBang.cheDo() != CheDoCanBang::Tat) phienCanBang.tat();
        }
        CauHinhChay cauHinhHienTai{};
        const bool coCauHinhHienTai = (xQueuePeek(_hangDoiCauHinh, &cauHinhHienTai, 0) == pdTRUE);
        const float gocThamChieu_rad = coCauHinhHienTai
            ? cauHinhHienTai.gocCanBang_rad
            : ::gocCanBang_rad;
        const bool daoChieuDongCoHienTai = coCauHinhHienTai
            ? cauHinhHienTai.daoChieuDongCo
            : daoChieuDongCoTruoc;
        if (daoChieuDongCoHienTai != daoChieuDongCoTruoc) {
            daoChieuDongCoTruoc = daoChieuDongCoHienTai;
            if (phienCanBang.cheDo() == CheDoCanBang::Tat) {
                _boDemXung.reset();
                ++baoCaoDongCo.maResetXung;
                thoiDiemDocXungTruoc_us = 0;
            }
        }
        ControlStatus baoCaoDieuKhien{};
        const bool coDuLieuDieuKhien =
            xQueuePeek(_hangDoiTrangThaiDieuKhien, &baoCaoDieuKhien, 0) == pdTRUE;
        const uint64_t thoiDiemHienTai_us = nowUs();
        if (thoiDiemPhucVuTruoc_us &&
            thoiDiemHienTai_us - thoiDiemPhucVuTruoc_us > baoCaoDongCo.khoangCachPhucVuLonNhat_us)
            baoCaoDongCo.khoangCachPhucVuLonNhat_us =
                static_cast<uint32_t>(thoiDiemHienTai_us - thoiDiemPhucVuTruoc_us);
        thoiDiemPhucVuTruoc_us = thoiDiemHienTai_us;
        const bool lenhHopLeThoiGian = coDuLieuDieuKhien &&
            baoCaoDieuKhien.lenhBanh.thoiDiemRaLenh_us <= thoiDiemHienTai_us &&
            baoCaoDieuKhien.lenhBanh.thoiDiemHetHan_us > thoiDiemHienTai_us;
        const uint64_t tuoiLenh_us = lenhHopLeThoiGian
            ? thoiDiemHienTai_us - baoCaoDieuKhien.lenhBanh.thoiDiemRaLenh_us : UINT64_MAX;
        // Bật: ngưỡng chặt. Đang chạy: chịu được 1–2 mẫu IMU bị bỏ (H1, 10/11/2026).
        const bool lenhConMoi = tuoiLenh_us < tuoiLenhDeBat_us;
        const bool lenhConHieuLucKhiChay = tuoiLenh_us < tuoiLenhKhiChay_us;
        FaultCode loiAnToan = coDuLieuDieuKhien
            ? SafetyMonitor::check(baoCaoDieuKhien.trangThaiXe, thoiDiemHienTai_us, gocThamChieu_rad)
            : FaultCode::ImuInvalid;
        if (loiAnToan == FaultCode::None) loiAnToan = baoCaoDieuKhien.loi;

        // Luân chuyển giữa 2 chế độ độc lập: PID Mờ (Mamdani) và PID Thuần (cố định)
        // có hiệu lực ngay cả khi động cơ đang tắt hoặc đang chạy cân bằng (bumpless).
        if (cacSuKien & suKienDoiCheDoMo) {
            baoCaoDongCo.dungPidMo = !baoCaoDongCo.dungPidMo;
        } else if (cacSuKien & suKienChonPidMo) {
            baoCaoDongCo.dungPidMo = true;
        } else if (cacSuKien & suKienChonPidThuan) {
            baoCaoDongCo.dungPidMo = false;
        }

        // Cùng một lượt nhận lệnh: dừng khẩn/tắt luôn ưu tiên hơn bật.
        if (cacSuKien & suKienDungKhan) {
            baoCaoDongCo.loi = FaultCode::EmergencyStop;
            phienCanBang.tat();
            _capDongCo.emergencyStop();
        } else if (cacSuKien & suKienTatDongCo) {
            phienCanBang.tat();
            _capDongCo.disable();
        } else if (cacSuKien & suKienXoaLoi) {
            if (!_capDongCo.isEnabled() && _capDongCo.clearFault()) {
                phienCanBang.tat();
                baoCaoDongCo.loi = FaultCode::None;
            }
        } else if ((cacSuKien & suKienBatDongCo) &&
                   phienCanBang.cheDo() == CheDoCanBang::Tat &&
                   baoCaoDongCo.loi == FaultCode::None && lenhConMoi &&
                   loiAnToan == FaultCode::None &&
                   SafetyMonitor::canArm(baoCaoDieuKhien.trangThaiXe, thoiDiemHienTai_us, gocThamChieu_rad)) {
            phienCanBang.yeuCauBat(thoiDiemHienTai_us);
        }

        if ((cacSuKien & suKienResetXung) && phienCanBang.cheDo() == CheDoCanBang::Tat) {
            _boDemXung.reset();
            ++baoCaoDongCo.maResetXung;
            thoiDiemDocXungTruoc_us = 0;
        }

        if (phienCanBang.cheDo() == CheDoCanBang::ChoLenhMoi) {
            if (phienCanBang.quaHanCho(thoiDiemHienTai_us)) {
                baoCaoDongCo.loi = FaultCode::ArmTimeout;
                phienCanBang.tat();
                _capDongCo.disable();
            } else if (loiAnToan != FaultCode::None ||
                       !SafetyMonitor::canArm(baoCaoDieuKhien.trangThaiXe, thoiDiemHienTai_us, gocThamChieu_rad)) {
                phienCanBang.tat(); // Tư thế thay đổi trong khi chờ: phải yêu cầu bật lại.
            } else if (phienCanBang.lenhKhoiDongHopLe(
                           baoCaoDieuKhien.maPhien, baoCaoDieuKhien.lenhBanh, thoiDiemHienTai_us)) {
                baoCaoDongCo.ketQuaCuoi = _capDongCo.enable();
                if (baoCaoDongCo.ketQuaCuoi == MksResult::Ok) {
                    phienCanBang.xacNhanDaBat();
                    baoCaoDongCo.khoangCachPhucVuLonNhat_us = 0;
                    thoiDiemPhucVuTruoc_us = nowUs();
                    soThuTuLenhDaApDung = baoCaoDieuKhien.soThuTuLenh - 1u;
                } else if (baoCaoDongCo.ketQuaCuoi != MksResult::Pending) {
                    phienCanBang.tat();
                }
            }
        }
        if (phienCanBang.cheDo() == CheDoCanBang::DangChay) {
            if (loiAnToan != FaultCode::None || !lenhConHieuLucKhiChay ||
                baoCaoDieuKhien.maPhien != phienCanBang.maPhien()) {
                baoCaoDongCo.loi = loiAnToan == FaultCode::None ? FaultCode::InvalidControl : loiAnToan;
                phienCanBang.tat();
                _capDongCo.emergencyStop();
            } else if (baoCaoDieuKhien.soThuTuLenh != soThuTuLenhDaApDung) {
                WheelStepCommand lenhBanhApDung = baoCaoDieuKhien.lenhBanh;
                if (daoChieuDongCoHienTai) {
                    lenhBanhApDung.tocDoBuocTrai_buocTrenGiay = -lenhBanhApDung.tocDoBuocTrai_buocTrenGiay;
                    lenhBanhApDung.tocDoBuocPhai_buocTrenGiay = -lenhBanhApDung.tocDoBuocPhai_buocTrenGiay;
                }
                baoCaoDongCo.ketQuaCuoi = _capDongCo.apply(lenhBanhApDung);
                soThuTuLenhDaApDung = baoCaoDieuKhien.soThuTuLenh;
            }
        }
        if (_capDongCo.service() == MksResult::FaultLatched) {
            if (baoCaoDongCo.loi == FaultCode::None) baoCaoDongCo.loi = FaultCode::MotorFault;
            if (phienCanBang.cheDo() != CheDoCanBang::Tat) phienCanBang.tat();
        }
        const uint64_t thoiDiemTruocDocXung_us = nowUs();
        if (!baoCaoDongCo.khungXung.hopLe || thoiDiemDocXungTruoc_us == 0 ||
            thoiDiemTruocDocXung_us - thoiDiemDocXungTruoc_us >= 1000u) {
            baoCaoDongCo.khungXung = _boDemXung.read();
            if (daoChieuDongCoHienTai && baoCaoDongCo.khungXung.hopLe) {
                baoCaoDongCo.khungXung.trai = -baoCaoDongCo.khungXung.trai;
                baoCaoDongCo.khungXung.phai = -baoCaoDongCo.khungXung.phai;
                baoCaoDongCo.khungXung.encoderTrai = -baoCaoDongCo.khungXung.encoderTrai;
                baoCaoDongCo.khungXung.encoderPhai = -baoCaoDongCo.khungXung.encoderPhai;
            }
            thoiDiemDocXungTruoc_us = thoiDiemTruocDocXung_us;
            if (_capDongCo.service() == MksResult::FaultLatched) {
                if (baoCaoDongCo.loi == FaultCode::None) baoCaoDongCo.loi = FaultCode::MotorFault;
                if (phienCanBang.cheDo() != CheDoCanBang::Tat) phienCanBang.tat();
            }
        }
        if (!baoCaoDongCo.khungXung.hopLe && _capDongCo.isEnabled()) {
            baoCaoDongCo.loi = FaultCode::PulseInvalid;
            phienCanBang.tat();
            _capDongCo.emergencyStop();
        }
        baoCaoDongCo.trangThaiBoDieuKhienDongCo = _capDongCo.snapshot();
        baoCaoDongCo.maPhien = phienCanBang.maPhien();
        baoCaoDongCo.cheDo = phienCanBang.cheDo();
        xQueueOverwrite(_hangDoiTrangThaiDongCo, &baoCaoDongCo);
    }
}

void RobotApp::apDungCauHinh(const CauHinhChay& cauHinh) {
    _boDieuKhienTang.setPidGocThuan({cauHinh.kpGocThuan, cauHinh.kiGocThuan, cauHinh.kdGocThuan});
    _boDieuKhienTang.setPidGocMoCoSo({cauHinh.kpGocMo, cauHinh.kiGocMo, cauHinh.kdGocMo});
    _boDieuKhienTang.setPidVanTocThuan(cauHinh.kpVanTocThuan, cauHinh.kiVanTocThuan);
    _boDieuKhienTang.datPidVanTocMo(cauHinh.kpVanTocMo, cauHinh.kiVanTocMo);
    _boDieuKhienTang.setGocCanBang_rad(cauHinh.gocCanBang_rad);
    _boDieuKhienTang.setCheDoThuKhongMaSat(cauHinh.cheDoThuKhongMaSat);
    _boDieuKhienTang.setBatVongNgoai(cauHinh.batVongNgoai);
}

void RobotApp::inDongSerial(const char* noiDung) {
    if (!noiDung) return;
    Serial.println(noiDung);
    dieuKhienKhongDay.ghiDongNhatKy(noiDung);
}

void RobotApp::inDinhDangSerial(const char* dinhDang, ...) {
    if (!dinhDang) return;
    char boDemTam[320]{};
    va_list danhSachThamSo;
    va_start(danhSachThamSo, dinhDang);
    std::vsnprintf(boDemTam, sizeof(boDemTam), dinhDang, danhSachThamSo);
    va_end(danhSachThamSo);
    Serial.print(boDemTam);
    dieuKhienKhongDay.ghiDongNhatKy(boDemTam);
}

void RobotApp::luuCauHinhVaoNvs() {
    MotorStatus baoCaoDongCo{};
    if (_hangDoiTrangThaiDongCo &&
        xQueuePeek(_hangDoiTrangThaiDongCo, &baoCaoDongCo, 0) == pdTRUE &&
        baoCaoDongCo.cheDo != CheDoCanBang::Tat) {
        _canLuuNvs = true;
        return;
    }
    _canLuuNvs = false;
    Preferences prefs;
    if (!prefs.begin(nvsNamespace, false)) {
        inDongSerial("[NVS] ERROR: Khong mo duoc NVS de luu.");
        return;
    }
    prefs.putUInt("magic", nvsMagic);
    prefs.putFloat("kp_p", _cauHinhChay.kpGocThuan);
    prefs.putFloat("ki_p", _cauHinhChay.kiGocThuan);
    prefs.putFloat("kd_p", _cauHinhChay.kdGocThuan);
    prefs.putFloat("kpv_p", _cauHinhChay.kpVanTocThuan);
    prefs.putFloat("kiv_p", _cauHinhChay.kiVanTocThuan);
    prefs.putFloat("kp_m", _cauHinhChay.kpGocMo);
    prefs.putFloat("ki_m", _cauHinhChay.kiGocMo);
    prefs.putFloat("kd_m", _cauHinhChay.kdGocMo);
    prefs.putFloat("kpv_m", _cauHinhChay.kpVanTocMo);
    prefs.putFloat("kiv_m", _cauHinhChay.kiVanTocMo);
    prefs.putFloat("ref_rad", _cauHinhChay.gocCanBang_rad);
    prefs.putBool("mode_mo", _cauHinhChay.dungPidMo);
    prefs.putBool("inv_dir", _cauHinhChay.daoChieuDongCo);
    prefs.putBool("gb_ok", _coBiasLuu);
    if (_coBiasLuu) {
        prefs.putFloat("gb_x", _biasX_dps);
        prefs.putFloat("gb_y", _biasY_dps);
        prefs.putFloat("gb_z", _biasZ_dps);
    }
    prefs.end();
}

void RobotApp::inTroGiup() {
    inDinhDangSerial("FIRMWARE %s\n", phienBanFirmware);
    inDongSerial("Dieu khien: a=arm; d=disable; x=emergency; k=clear fault; p=PID_THUAN; m=PID_MO; f=dao mode");
    inDongSerial("Di chuyen (khi arm): w/s=tien/lui; j/l=trai/phai; z=dung; g=chot giu vi tri tai cho");
    inDongSerial("Calib & Bench: c=calib khoa hoc 2s (gyro+pitch+NVS); o=zero nhanh pitch+odo; [/]=trim -/+0.1deg");
    inDongSerial("               b=bat/tat Bench Mode (nhac banh khong ma sat); v=bat/tat vong van toc/vi tri");
    inDongSerial("Chinh PID (kem Enter): kp X; ki X; kd X; kpv X; kiv X; kpv_p X; kpv_m X; ref X(rad); refdeg X(deg); bench 0|1; vloop 0|1; daochieu [0|1]");
    inDongSerial("Lenh khac: pid|fuzzy|mode pid|fuzzy; giuvitri; save=luu NVS; resetcfg=xoa NVS; status=xem cau hinh; t=stream; tw=web stream; i=RAW_IMU");
    inDinhDangSerial("WiFi/ESP-NOW: SSID=%s PASS=%s URL=http://%s WS=ws://%s:%u UDP=%u MAC=%s\n",
                  tenMangWifiAp, matKhauWifiAp, dieuKhienKhongDay.diaChiIpAp(),
                  dieuKhienKhongDay.diaChiIpAp(), static_cast<unsigned>(congWebSocketNhiPhan),
                  static_cast<unsigned>(congUdpNhiPhan), dieuKhienKhongDay.diaChiMacXe());
}

void RobotApp::inTrangThaiChiTiet() {
    MotorStatus baoCaoDongCo{};
    ControlStatus baoCaoDieuKhien{};
    xQueuePeek(_hangDoiTrangThaiDongCo, &baoCaoDongCo, 0);
    const bool coBaoCao = xQueuePeek(_hangDoiTrangThaiDieuKhien, &baoCaoDieuKhien, 0) == pdTRUE;
    inDongSerial("=== TRANG THAI HE THONG ===");
    inDinhDangSerial("Mode=%s EN=%d fault=%s bench=%d vloop=%d invDir=%d telemetry=%d tw=%d\n",
                  _cauHinhChay.dungPidMo ? "PID_MO" : "PID_THUAN",
                  baoCaoDongCo.trangThaiBoDieuKhienDongCo.daBat,
                  faultName(baoCaoDongCo.loi),
                  _cauHinhChay.cheDoThuKhongMaSat ? 1 : 0,
                  _cauHinhChay.batVongNgoai ? 1 : 0,
                  _cauHinhChay.daoChieuDongCo ? 1 : 0,
                  _batTelemetry ? 1 : 0,
                  batTelemetryWeb ? 1 : 0);
    inDinhDangSerial("PID_THUAN: Kp=%.3f Ki=%.3f Kd=%.3f | Kpv=%.4f Kiv=%.4f\n",
                  _cauHinhChay.kpGocThuan, _cauHinhChay.kiGocThuan, _cauHinhChay.kdGocThuan,
                  _cauHinhChay.kpVanTocThuan, _cauHinhChay.kiVanTocThuan);
    inDinhDangSerial("PID_MO base: Kp=%.3f Ki=%.3f Kd=%.3f | Kpv=%.4f Kiv=%.4f\n",
                  _cauHinhChay.kpGocMo, _cauHinhChay.kiGocMo, _cauHinhChay.kdGocMo,
                  _cauHinhChay.kpVanTocMo, _cauHinhChay.kiVanTocMo);
    inDinhDangSerial("Equilibrium: ref=%.5frad (%.2fdeg) | IMU_biasOk=%d (%.3f,%.3f,%.3f dps)\n",
                  _cauHinhChay.gocCanBang_rad, _cauHinhChay.gocCanBang_rad * doiRadSangDo,
                  _coBiasLuu ? 1 : 0, _biasX_dps, _biasY_dps, _biasZ_dps);
    if (coBaoCao) {
        inDinhDangSerial("Live: theta=%.2fdeg rate=%.3frad/s target=%.2fdeg K_active=(%.2f,%.2f,%.2f) STEP=%lld,%lld\n",
                      baoCaoDieuKhien.trangThaiXe.gocNghien_rad * doiRadSangDo,
                      baoCaoDieuKhien.trangThaiXe.tocDoNghien_radps,
                      baoCaoDieuKhien.gocDat_rad * doiRadSangDo,
                      baoCaoDieuKhien.heSoPid.heSoTiLe,
                      baoCaoDieuKhien.heSoPid.heSoTichPhan,
                      baoCaoDieuKhien.heSoPid.heSoViPhan,
                      static_cast<long long>(baoCaoDongCo.khungXung.trai),
                      static_cast<long long>(baoCaoDongCo.khungXung.phai));
    }
    inDinhDangSerial("MKS_UART (vFOC MStep=%lu, 65536/rev, baud=%lu, vFOC_fusion=%d): L(ok=%d addr=0x%02X rx=%lu encStep=%lld) R(ok=%d addr=0x%02X rx=%lu encStep=%lld)\n",
                  static_cast<unsigned long>(soViBuocDauVaoMks),
                  static_cast<unsigned long>(tocDoBaudUartMks_bps),
                  batBuTruotUartMksVaoXung ? 1 : 0,
                  baoCaoDongCo.khungXung.coUartTrai ? 1 : 0,
                  baoCaoDongCo.khungXung.diaChiUartTrai,
                  static_cast<unsigned long>(baoCaoDongCo.khungXung.soKhungUartTrai),
                  static_cast<long long>(baoCaoDongCo.khungXung.encoderTrai),
                  baoCaoDongCo.khungXung.coUartPhai ? 1 : 0,
                  baoCaoDongCo.khungXung.diaChiUartPhai,
                  static_cast<unsigned long>(baoCaoDongCo.khungXung.soKhungUartPhai),
                  static_cast<long long>(baoCaoDongCo.khungXung.encoderPhai));
    inDinhDangSerial("WiFi_AP=%s URL=http://%s WS=ws://%s:%u UDP=%u rxNhiPhan=%lu | ESP_NOW=%s MAC=%s rxGoi=%lu\n",
                  dieuKhienKhongDay.wifiSanSang() ? tenMangWifiAp : "OFF",
                  dieuKhienKhongDay.diaChiIpAp(),
                  dieuKhienKhongDay.diaChiIpAp(),
                  static_cast<unsigned>(congWebSocketNhiPhan),
                  static_cast<unsigned>(congUdpNhiPhan),
                  static_cast<unsigned long>(dieuKhienKhongDay.tongSoGoiNhiPhan()),
                  dieuKhienKhongDay.espNowSanSang() ? "ON" : "OFF",
                  dieuKhienKhongDay.diaChiMacXe(),
                  static_cast<unsigned long>(dieuKhienKhongDay.tongSoGoiEspNow()));
}

void RobotApp::xuLyPhimDonSerial(char kyTu) {
    MotorStatus trangThaiDongCo{};
    xQueuePeek(_hangDoiTrangThaiDongCo, &trangThaiDongCo, 0);
    if (kyTu == 'w' || kyTu == 's' || kyTu == 'j' || kyTu == 'l' || kyTu == 'z') {
        if (trangThaiDongCo.cheDo == CheDoCanBang::DangChay) {
            LenhDiChuyen lenh{};
            lenh.maPhien = trangThaiDongCo.maPhien;
            lenh.thoiDiem_us = nowUs();
            lenh.hetHan_us = lenh.thoiDiem_us + hanLenhDiChuyen_us;
            if (kyTu == 'w') lenh.vanToc_mps = vanTocThuSerial_mps;
            if (kyTu == 's') lenh.vanToc_mps = -vanTocThuSerial_mps;
            if (kyTu == 'j') lenh.tocDoQuay_radps = tocDoQuayThuSerial_radps;
            if (kyTu == 'l') lenh.tocDoQuay_radps = -tocDoQuayThuSerial_radps;
            xQueueOverwrite(_hangDoiDiChuyen, &lenh);
        }
        return;
    }

    switch (kyTu) {
    case 'i': _inDuLieuImu = !_inDuLieuImu; break;
    case 't':
        _batTelemetry = !_batTelemetry;
        inDinhDangSerial("[UI] Telemetry stream=%s\n", _batTelemetry ? "ON" : "OFF");
        break;
    case 'a': xTaskNotify(_tacVuDongCo, suKienBatDongCo, eSetBits); break;
    case 'd':
    case '!': xTaskNotify(_tacVuDongCo, suKienTatDongCo, eSetBits); break;
    case 'x': xTaskNotify(_tacVuDongCo, suKienDungKhan, eSetBits); break;
    case 'k': xTaskNotify(_tacVuDongCo, suKienXoaLoi, eSetBits); break;
    case 'f':
        _cauHinhChay.dungPidMo = !_cauHinhChay.dungPidMo;
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        xTaskNotify(_tacVuDongCo,
                    _cauHinhChay.dungPidMo ? suKienChonPidMo : suKienChonPidThuan,
                    eSetBits);
        inDinhDangSerial("[CTRL] Dao che do -> %s\n", _cauHinhChay.dungPidMo ? "PID_MO" : "PID_THUAN");
        break;
    case 'm':
        _cauHinhChay.dungPidMo = true;
        _cauHinhChay.cheDoThuKhongMaSat = false;
        _cauHinhChay.batVongNgoai = true;
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        xTaskNotify(_tacVuDongCo, suKienChonPidMo, eSetBits);
        inDongSerial("[CTRL] Chon che do: PID_MO (Can bang banh xe, Bench=OFF, VongViTri=ON)");
        break;
    case 'p':
        _cauHinhChay.dungPidMo = false;
        _cauHinhChay.cheDoThuKhongMaSat = false;
        _cauHinhChay.batVongNgoai = true;
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        xTaskNotify(_tacVuDongCo, suKienChonPidThuan, eSetBits);
        inDongSerial("[CTRL] Chon che do: PID_THUAN (Can bang banh xe, Bench=OFF, VongViTri=ON)");
        break;
    case 'b':
        _cauHinhChay.cheDoThuKhongMaSat = !_cauHinhChay.cheDoThuKhongMaSat;
        _cauHinhChay.batVongNgoai = !_cauHinhChay.cheDoThuKhongMaSat;
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        inDinhDangSerial("[BENCH] BenchMode=%s (damping=%.1f/s, outerLoop=%s)\n",
                      _cauHinhChay.cheDoThuKhongMaSat ? "ON" : "OFF",
                      heSoGiamChanVanTocBench_1ps,
                      _cauHinhChay.batVongNgoai ? "ON" : "OFF");
        break;
    case 'v':
        _cauHinhChay.batVongNgoai = !_cauHinhChay.batVongNgoai;
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        inDinhDangSerial("[CTRL] OuterLoop (v/x)=%s\n", _cauHinhChay.batVongNgoai ? "ON" : "OFF");
        break;
    case 'g':
        ++_cauHinhChay.soLanChotViTri;
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        inDongSerial("[CTRL] Da chot vi tri va huong giu tai cho");
        break;
    case '[':
    case ']': {
        const float buoc_rad = 0.10f * doiDoSangRad;
        _cauHinhChay.gocCanBang_rad = std::clamp(
            _cauHinhChay.gocCanBang_rad + (kyTu == ']' ? buoc_rad : -buoc_rad), -0.35f, 0.35f);
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        inDinhDangSerial("[CAL] Trim eqPitch=%.5frad (%.2fdeg)\n",
                      _cauHinhChay.gocCanBang_rad, _cauHinhChay.gocCanBang_rad * doiRadSangDo);
        break;
    }
    case 'o': {
        if (trangThaiDongCo.cheDo != CheDoCanBang::Tat) {
            inDongSerial("[CAL] Hay tat motor ('d') truoc khi zero!");
            break;
        }
        ControlStatus baoCaoDieuKhien{};
        if (xQueuePeek(_hangDoiTrangThaiDieuKhien, &baoCaoDieuKhien, 0) == pdTRUE &&
            baoCaoDieuKhien.trangThaiXe.gocHopLe &&
            std::fabs(baoCaoDieuKhien.trangThaiXe.gocNghien_rad) <= 0.35f) {
            _cauHinhChay.gocCanBang_rad = baoCaoDieuKhien.trangThaiXe.gocNghien_rad;
            ++_cauHinhChay.phienBan;
            xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
            xTaskNotify(_tacVuDongCo, suKienResetXung, eSetBits);
            inDinhDangSerial("[CAL] Quick zero: eqPitch=%.5frad (%.2fdeg) + reset odometry\n",
                          _cauHinhChay.gocCanBang_rad, _cauHinhChay.gocCanBang_rad * doiRadSangDo);
        } else {
            inDongSerial("[CAL] Chua co goc IMU hop le hoac nghieng >20deg.");
        }
        break;
    }
    case 'c': {
        if (trangThaiDongCo.cheDo != CheDoCanBang::Tat) {
            inDongSerial("[CAL] Hay tat motor ('d') truoc khi calib khoa hoc!");
            break;
        }
        YeuCauCalibImu yeuCau{};
        yeuCau.maYeuCau = ++_maYeuCauCalib;
        _dangChoCalib = true;
        xQueueOverwrite(_hangDoiYeuCauCalib, &yeuCau);
        inDongSerial("[CAL] Giu xe DUNG YEN tai tu the can bang ~2.0 giay (400 mau)...");
        break;
    }
    case 'h':
    case '?':
        inTroGiup();
        break;
    default: break;
    }
}

void RobotApp::xuLyDongLenhSerial(const char* dongLenh) {
    if (!dongLenh) return;
    while (*dongLenh == ' ' || *dongLenh == '\t') ++dongLenh;
    if (*dongLenh == '\0') return;

    char banSao[96]{};
    std::strncpy(banSao, dongLenh, sizeof(banSao) - 1u);
    size_t len = std::strlen(banSao);
    while (len > 0 && (banSao[len - 1u] == ' ' || banSao[len - 1u] == '\t' ||
                       banSao[len - 1u] == '\r' || banSao[len - 1u] == '\n')) {
        banSao[--len] = '\0';
    }
    if (len == 0u) return;

    if (len == 1u) {
        xuLyPhimDonSerial(banSao[0]);
        return;
    }

    // Chuyển phần lệnh về chữ thường để dễ gõ.
    for (size_t i = 0; i < len && banSao[i] != ' '; ++i) {
        if (banSao[i] >= 'A' && banSao[i] <= 'Z') banSao[i] = static_cast<char>(banSao[i] - 'A' + 'a');
    }

    MotorStatus trangThaiDongCo{};
    xQueuePeek(_hangDoiTrangThaiDongCo, &trangThaiDongCo, 0);

    if (std::strcmp(banSao, "help") == 0) { inTroGiup(); return; }
    if (std::strcmp(banSao, "status") == 0) { inTrangThaiChiTiet(); return; }
    if (std::strcmp(banSao, "arm") == 0) { xuLyPhimDonSerial('a'); return; }
    if (std::strcmp(banSao, "disarm") == 0 || std::strcmp(banSao, "stop") == 0) { xuLyPhimDonSerial('d'); return; }
    if (std::strcmp(banSao, "clear") == 0) { xuLyPhimDonSerial('k'); return; }
    if (std::strcmp(banSao, "calib") == 0) { xuLyPhimDonSerial('c'); return; }
    if (std::strcmp(banSao, "zero") == 0) { xuLyPhimDonSerial('o'); return; }
    if (std::strcmp(banSao, "giuvitri") == 0) { xuLyPhimDonSerial('g'); return; }
    if (std::strcmp(banSao, "pid") == 0 || std::strcmp(banSao, "pid_thuan") == 0) { xuLyPhimDonSerial('p'); return; }
    if (std::strcmp(banSao, "fuzzy") == 0 || std::strcmp(banSao, "pid_mo") == 0) { xuLyPhimDonSerial('m'); return; }
    if (std::strcmp(banSao, "banh") == 0 || std::strcmp(banSao, "banhxe") == 0) {
        _cauHinhChay.cheDoThuKhongMaSat = false;
        _cauHinhChay.batVongNgoai = true;
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        inDongSerial("[CTRL] Che do can bang banh xe: BenchMode=OFF, OuterLoop=ON");
        return;
    }
    if (std::strcmp(banSao, "daochieu") == 0) {
        if (trangThaiDongCo.cheDo != CheDoCanBang::Tat) {
            inDongSerial("[CTRL] Hay tat motor ('d') truoc khi dao chieu dong co!");
            return;
        }
        _cauHinhChay.daoChieuDongCo = !_cauHinhChay.daoChieuDongCo;
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        xTaskNotify(_tacVuDongCo, suKienResetXung, eSetBits);
        inDinhDangSerial("[CTRL] daoChieuDongCo=%s (nho 'save' neu muon luu Flash)\n",
                         _cauHinhChay.daoChieuDongCo ? "DAO (ON)" : "MAC DINH (OFF)");
        return;
    }
    if (std::strcmp(banSao, "tw") == 0) {
        batTelemetryWeb = !batTelemetryWeb;
        inDinhDangSerial("[UI] Web Terminal telemetry stream=%s\n", batTelemetryWeb ? "ON" : "OFF");
        return;
    }
    if (std::strcmp(banSao, "save") == 0) {
        luuCauHinhVaoNvs();
        if (_canLuuNvs) {
            inDongSerial("[NVS] Dang can bang: da hen luu NVS va se ghi Flash ngay khi tat motor.");
        } else {
            inDongSerial("[NVS] Da luu PID_THUAN, PID_MO, eqPitch, mode va IMU bias vao Flash.");
        }
        return;
    }
    if (std::strcmp(banSao, "resetcfg") == 0) {
        _canLuuNvs = false;
        Preferences prefs;
        if (prefs.begin(nvsNamespace, false)) {
            prefs.clear();
            prefs.end();
        }
        _cauHinhChay = CauHinhChay{};
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        xTaskNotify(_tacVuDongCo, _cauHinhChay.dungPidMo ? suKienChonPidMo : suKienChonPidThuan, eSetBits);
        inDongSerial("[NVS] Da xoa NVS va khoi phuc thong so mac dinh ControlConfig.h.");
        return;
    }

    char thamSo[32]{};
    float giaTri = 0.0f;
    if (std::sscanf(banSao, "mode %31s", thamSo) == 1) {
        if (std::strcmp(thamSo, "pid") == 0 || std::strcmp(thamSo, "p") == 0 || std::strcmp(thamSo, "pid_thuan") == 0) {
            xuLyPhimDonSerial('p');
        } else if (std::strcmp(thamSo, "fuzzy") == 0 || std::strcmp(thamSo, "m") == 0 || std::strcmp(thamSo, "pid_mo") == 0) {
            xuLyPhimDonSerial('m');
        } else {
            inDongSerial("[CMD] Dung: mode pid | mode fuzzy");
        }
        return;
    }
    if (std::sscanf(banSao, "bench %f", &giaTri) == 1) {
        _cauHinhChay.cheDoThuKhongMaSat = (giaTri != 0.0f);
        _cauHinhChay.batVongNgoai = !_cauHinhChay.cheDoThuKhongMaSat;
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        inDinhDangSerial("[BENCH] BenchMode=%s (outerLoop=%s)\n",
                      _cauHinhChay.cheDoThuKhongMaSat ? "ON" : "OFF",
                      _cauHinhChay.batVongNgoai ? "ON" : "OFF");
        return;
    }
    if (std::sscanf(banSao, "vloop %f", &giaTri) == 1) {
        _cauHinhChay.batVongNgoai = (giaTri != 0.0f);
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        inDinhDangSerial("[CTRL] OuterLoop=%s\n", _cauHinhChay.batVongNgoai ? "ON" : "OFF");
        return;
    }
    if (std::sscanf(banSao, "daochieu %f", &giaTri) == 1) {
        if (trangThaiDongCo.cheDo != CheDoCanBang::Tat) {
            inDongSerial("[CTRL] Hay tat motor ('d') truoc khi dao chieu dong co!");
            return;
        }
        _cauHinhChay.daoChieuDongCo = (giaTri != 0.0f);
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        xTaskNotify(_tacVuDongCo, suKienResetXung, eSetBits);
        inDinhDangSerial("[CTRL] daoChieuDongCo=%s (nho 'save' neu muon luu Flash)\n",
                         _cauHinhChay.daoChieuDongCo ? "DAO (ON)" : "MAC DINH (OFF)");
        return;
    }
    if (std::sscanf(banSao, "refdeg %f", &giaTri) == 1 && std::isfinite(giaTri)) {
        _cauHinhChay.gocCanBang_rad = std::clamp(giaTri * doiDoSangRad, -0.35f, 0.35f);
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        inDinhDangSerial("[CAL] eqPitch=%.5frad (%.2fdeg)\n",
                      _cauHinhChay.gocCanBang_rad, _cauHinhChay.gocCanBang_rad * doiRadSangDo);
        return;
    }
    if (std::sscanf(banSao, "ref %f", &giaTri) == 1 && std::isfinite(giaTri)) {
        _cauHinhChay.gocCanBang_rad = std::clamp(giaTri, -0.35f, 0.35f);
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        inDinhDangSerial("[CAL] eqPitch=%.5frad (%.2fdeg)\n",
                      _cauHinhChay.gocCanBang_rad, _cauHinhChay.gocCanBang_rad * doiRadSangDo);
        return;
    }

    auto capNhatThongSo = [&](const char* nhan, float& dich, float minV, float maxV) {
        if (!std::isfinite(giaTri) || giaTri < minV || giaTri > maxV) {
            inDinhDangSerial("[PID] Gia tri %s ngoai mien [%.3f..%.3f]\n", nhan, minV, maxV);
            return;
        }
        dich = giaTri;
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        inDinhDangSerial("[PID] %s = %.4f\n", nhan, dich);
    };

    if (std::sscanf(banSao, "kpv_p %f", &giaTri) == 1) { capNhatThongSo("PID_THUAN Kpv", _cauHinhChay.kpVanTocThuan, 0.0f, 5.0f); return; }
    if (std::sscanf(banSao, "kiv_p %f", &giaTri) == 1) { capNhatThongSo("PID_THUAN Kiv", _cauHinhChay.kiVanTocThuan, 0.0f, 5.0f); return; }
    if (std::sscanf(banSao, "kpv_m %f", &giaTri) == 1) { capNhatThongSo("PID_MO Kpv", _cauHinhChay.kpVanTocMo, 0.0f, 5.0f); return; }
    if (std::sscanf(banSao, "kiv_m %f", &giaTri) == 1) { capNhatThongSo("PID_MO Kiv", _cauHinhChay.kiVanTocMo, 0.0f, 5.0f); return; }
    if (std::sscanf(banSao, "kp_p %f", &giaTri) == 1) { capNhatThongSo("PID_THUAN Kp", _cauHinhChay.kpGocThuan, 0.0f, 300.0f); return; }
    if (std::sscanf(banSao, "ki_p %f", &giaTri) == 1) { capNhatThongSo("PID_THUAN Ki", _cauHinhChay.kiGocThuan, 0.0f, 100.0f); return; }
    if (std::sscanf(banSao, "kd_p %f", &giaTri) == 1) { capNhatThongSo("PID_THUAN Kd", _cauHinhChay.kdGocThuan, 0.0f, 50.0f); return; }
    if (std::sscanf(banSao, "kp_m %f", &giaTri) == 1) { capNhatThongSo("PID_MO Kp_base", _cauHinhChay.kpGocMo, 0.0f, 300.0f); return; }
    if (std::sscanf(banSao, "ki_m %f", &giaTri) == 1) { capNhatThongSo("PID_MO Ki_base", _cauHinhChay.kiGocMo, 0.0f, 100.0f); return; }
    if (std::sscanf(banSao, "kd_m %f", &giaTri) == 1) { capNhatThongSo("PID_MO Kd_base", _cauHinhChay.kdGocMo, 0.0f, 50.0f); return; }

    if (std::sscanf(banSao, "kpv %f", &giaTri) == 1) {
        if (_cauHinhChay.dungPidMo) capNhatThongSo("PID_MO Kpv", _cauHinhChay.kpVanTocMo, 0.0f, 5.0f);
        else capNhatThongSo("PID_THUAN Kpv", _cauHinhChay.kpVanTocThuan, 0.0f, 5.0f);
        return;
    }
    if (std::sscanf(banSao, "kiv %f", &giaTri) == 1) {
        if (_cauHinhChay.dungPidMo) capNhatThongSo("PID_MO Kiv", _cauHinhChay.kiVanTocMo, 0.0f, 5.0f);
        else capNhatThongSo("PID_THUAN Kiv", _cauHinhChay.kiVanTocThuan, 0.0f, 5.0f);
        return;
    }
    if (std::sscanf(banSao, "kp %f", &giaTri) == 1) {
        if (_cauHinhChay.dungPidMo) capNhatThongSo("PID_MO Kp_base", _cauHinhChay.kpGocMo, 0.0f, 300.0f);
        else capNhatThongSo("PID_THUAN Kp", _cauHinhChay.kpGocThuan, 0.0f, 300.0f);
        return;
    }
    if (std::sscanf(banSao, "ki %f", &giaTri) == 1) {
        if (_cauHinhChay.dungPidMo) capNhatThongSo("PID_MO Ki_base", _cauHinhChay.kiGocMo, 0.0f, 100.0f);
        else capNhatThongSo("PID_THUAN Ki", _cauHinhChay.kiGocThuan, 0.0f, 100.0f);
        return;
    }
    if (std::sscanf(banSao, "kd %f", &giaTri) == 1) {
        if (_cauHinhChay.dungPidMo) capNhatThongSo("PID_MO Kd_base", _cauHinhChay.kdGocMo, 0.0f, 50.0f);
        else capNhatThongSo("PID_THUAN Kd", _cauHinhChay.kdGocThuan, 0.0f, 50.0f);
        return;
    }

    inDinhDangSerial("[CMD] Khong hieu: '%s' (go 'help' hoac 'status')\n", banSao);
}

void RobotApp::nhanLenhDiChuyenTuXa(float vanTocTienLui, float tocDoQuayNgang) {
    MotorStatus trangThaiDongCo{};
    xQueuePeek(_hangDoiTrangThaiDongCo, &trangThaiDongCo, 0);
    if (trangThaiDongCo.cheDo != CheDoCanBang::DangChay) return;
    LenhDiChuyen lenhMoi{};
    lenhMoi.maPhien = trangThaiDongCo.maPhien;
    lenhMoi.thoiDiem_us = nowUs();
    lenhMoi.hetHan_us = lenhMoi.thoiDiem_us + hanLenhDiChuyen_us;
    lenhMoi.vanToc_mps = std::clamp(vanTocTienLui, -vanTocDiChuyenToiDa_mps, vanTocDiChuyenToiDa_mps);
    lenhMoi.tocDoQuay_radps = std::clamp(tocDoQuayNgang, -tocDoQuayToiDa_radps, tocDoQuayToiDa_radps);
    xQueueOverwrite(_hangDoiDiChuyen, &lenhMoi);
}

void RobotApp::dienTrangThaiWeb(ThongTinTrangThaiWeb& trangThaiRa) {
    MotorStatus baoCaoDongCo{};
    ControlStatus baoCaoDieuKhien{};
    xQueuePeek(_hangDoiTrangThaiDongCo, &baoCaoDongCo, 0);
    const bool coBaoCao = xQueuePeek(_hangDoiTrangThaiDieuKhien, &baoCaoDieuKhien, 0) == pdTRUE;
    const uint64_t thoiDiemHienTai = nowUs();
    FaultCode loiHienTai = coBaoCao
        ? SafetyMonitor::check(baoCaoDieuKhien.trangThaiXe, thoiDiemHienTai, _cauHinhChay.gocCanBang_rad)
        : FaultCode::ImuInvalid;
    if (loiHienTai == FaultCode::None) loiHienTai = baoCaoDieuKhien.loi;
    const bool gocConMoi = baoCaoDieuKhien.trangThaiXe.gocHopLe &&
        baoCaoDieuKhien.trangThaiXe.thoiDiemGoc_us &&
        baoCaoDieuKhien.trangThaiXe.thoiDiemGoc_us <= thoiDiemHienTai &&
        thoiDiemHienTai - baoCaoDieuKhien.trangThaiXe.thoiDiemGoc_us < tuoiImuToiDa_us;

    const bool dangMo = _cauHinhChay.dungPidMo;
    trangThaiRa.daBat = baoCaoDongCo.trangThaiBoDieuKhienDongCo.daBat;
    trangThaiRa.dangChayPidMo = dangMo;
    trangThaiRa.cheDoNangBanh = _cauHinhChay.cheDoThuKhongMaSat;
    trangThaiRa.batVongViTri = _cauHinhChay.batVongNgoai;
    trangThaiRa.daoChieuDongCo = _cauHinhChay.daoChieuDongCo;
    trangThaiRa.gocHopLe = gocConMoi;
    trangThaiRa.heThongSanSang = (loiHienTai == FaultCode::None && baoCaoDongCo.loi == FaultCode::None);
    trangThaiRa.gocNghienDo = baoCaoDieuKhien.trangThaiXe.gocNghien_rad * doiRadSangDo;
    trangThaiRa.gocCanBangDo = _cauHinhChay.gocCanBang_rad * doiRadSangDo;
    trangThaiRa.gocDatDo = baoCaoDieuKhien.gocDat_rad * doiRadSangDo;
    trangThaiRa.viTriMet = baoCaoDieuKhien.trangThaiXe.viTriDoc_m;
    trangThaiRa.vanTocMetGiay = baoCaoDieuKhien.trangThaiXe.vanTocDoc_mps;
    trangThaiRa.tocDoGocRadGiay = baoCaoDieuKhien.trangThaiXe.tocDoNghien_radps;
    trangThaiRa.xungBanhTrai = baoCaoDongCo.khungXung.trai;
    trangThaiRa.xungBanhPhai = baoCaoDongCo.khungXung.phai;
    trangThaiRa.encoderTrai = baoCaoDongCo.khungXung.encoderTrai;
    trangThaiRa.encoderPhai = baoCaoDongCo.khungXung.encoderPhai;
    trangThaiRa.kpGoc = dangMo ? _cauHinhChay.kpGocMo : _cauHinhChay.kpGocThuan;
    trangThaiRa.kiGoc = dangMo ? _cauHinhChay.kiGocMo : _cauHinhChay.kiGocThuan;
    trangThaiRa.kdGoc = dangMo ? _cauHinhChay.kdGocMo : _cauHinhChay.kdGocThuan;
    trangThaiRa.kpVanToc = dangMo ? _cauHinhChay.kpVanTocMo : _cauHinhChay.kpVanTocThuan;
    trangThaiRa.kiVanToc = dangMo ? _cauHinhChay.kiVanTocMo : _cauHinhChay.kiVanTocThuan;
    trangThaiRa.kpHieuDung = baoCaoDieuKhien.heSoPid.heSoTiLe;
    trangThaiRa.kiHieuDung = baoCaoDieuKhien.heSoPid.heSoTichPhan;
    trangThaiRa.kdHieuDung = baoCaoDieuKhien.heSoPid.heSoViPhan;
    trangThaiRa.maLoiDongCo = static_cast<unsigned>(baoCaoDongCo.trangThaiBoDieuKhienDongCo.loi);
    std::snprintf(trangThaiRa.tenLoi, sizeof(trangThaiRa.tenLoi), "%s", faultName(baoCaoDongCo.loi));
    std::snprintf(trangThaiRa.tenSanSang, sizeof(trangThaiRa.tenSanSang), "%s", faultName(loiHienTai));
    std::snprintf(trangThaiRa.cheDoDieuKhien, sizeof(trangThaiRa.cheDoDieuKhien), "%s",
                  dangMo ? "PID_MO" : "PID_THUAN");
}

void RobotApp::loop() {
    if (!_daSanSang) {
        static uint32_t lanBaoLoiBoot_ms = 0;
        if (millis() - lanBaoLoiBoot_ms >= 2000) {
            lanBaoLoiBoot_ms = millis();
            Serial.printf("BOOT ERROR: khoi tao phan cung that bai (ICM20602 WHO=0x%02X i2c=%u reg=0x%02X). Kiem tra day I2C/nguon va nhan RST.\n",
                _camBienGoc.maNhanDang(), static_cast<unsigned>(_camBienGoc.maLoiI2cCuoi()), _camBienGoc.thanhGhiCuoi());
        }
        delay(100);
        return;
    }

    if (_dangChoCalib) {
        KetQuaCalibImu kqCalib{};
        if (xQueuePeek(_hangDoiKetQuaCalib, &kqCalib, 0) == pdTRUE &&
            kqCalib.hoanTat && kqCalib.maYeuCau == _maYeuCauCalib &&
            kqCalib.maYeuCau != _maKetQuaCalibDaXuLy) {
            _maKetQuaCalibDaXuLy = kqCalib.maYeuCau;
            _dangChoCalib = false;
            if (kqCalib.thanhCong) {
                _biasX_dps = kqCalib.biasX_dps;
                _biasY_dps = kqCalib.biasY_dps;
                _biasZ_dps = kqCalib.biasZ_dps;
                _coBiasLuu = true;
                _cauHinhChay.gocCanBang_rad = kqCalib.gocCanBangDo_rad;
                ++_cauHinhChay.phienBan;
                xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
                xTaskNotify(_tacVuDongCo, suKienResetXung, eSetBits);
                luuCauHinhVaoNvs();
                inDinhDangSerial("[CAL] PASS samples=%u retry=%u stdGyro=%.4fdps varAcc=%.5f bias_dps=%.4f,%.4f,%.4f eqPitch=%.5frad (%.2fdeg) [SAVED_NVS]\n",
                              kqCalib.soMauDaThu, kqCalib.soLanThuLai, kqCalib.doLechChuanGyro_dps,
                              kqCalib.phuongSaiGiaToc, _biasX_dps, _biasY_dps, _biasZ_dps,
                              _cauHinhChay.gocCanBang_rad, _cauHinhChay.gocCanBang_rad * doiRadSangDo);
            } else {
                inDinhDangSerial("[CAL] FAIL (khong du on dinh hoac nghieng >20deg, retry=%u). Giu yen xe va thu 'c' lai.\n",
                              kqCalib.soLanThuLai);
            }
        }
    }

    for (unsigned chiSoKyTu = 0; chiSoKyTu < 32 && Serial.available(); ++chiSoKyTu) {
        const int kyTu = Serial.read();
        if (kyTu < 0) break;
        const char ch = static_cast<char>(kyTu);
        if (ch == '\r' || ch == '\n') {
            if (_doDaiLenhSerial > 0u) {
                _boDemLenhSerial[_doDaiLenhSerial] = '\0';
                xuLyDongLenhSerial(_boDemLenhSerial);
                _doDaiLenhSerial = 0u;
            }
            continue;
        }
        if (_doDaiLenhSerial == 0u &&
            (ch == '!' || ch == '[' || ch == ']' || ch == '?')) {
            xuLyPhimDonSerial(ch);
            continue;
        }
        if (_doDaiLenhSerial + 1u < sizeof(_boDemLenhSerial)) {
            _boDemLenhSerial[_doDaiLenhSerial++] = ch;
            _lucNhanKyTuCuoi_ms = millis();
        }
    }
    // Hỗ trợ nhấn đơn các phím chữ cái không cần Enter (sau 60 ms không có ký tự tiếp theo)
    // mà không cắt ngang ký tự đầu của các lệnh đầy đủ như daochieu, disarm, arm, zero, help.
    if (_doDaiLenhSerial == 1u && !Serial.available() && (millis() - _lucNhanKyTuCuoi_ms >= 60u)) {
        const char ch = _boDemLenhSerial[0];
        _doDaiLenhSerial = 0u;
        xuLyPhimDonSerial(ch);
    } else if (_doDaiLenhSerial > 1u && !Serial.available() && (millis() - _lucNhanKyTuCuoi_ms >= 2000u)) {
        _doDaiLenhSerial = 0u;
    }

    dieuKhienKhongDay.xuLy();

    if (_canLuuNvs) {
        MotorStatus trangThaiDongCoNvs{};
        if (xQueuePeek(_hangDoiTrangThaiDongCo, &trangThaiDongCoNvs, 0) == pdTRUE &&
            trangThaiDongCoNvs.cheDo == CheDoCanBang::Tat) {
            luuCauHinhVaoNvs();
            if (!_canLuuNvs) {
                inDongSerial("[NVS] Da ghi Flash cau hinh dang cho sau khi tat motor.");
            }
        }
    }

    static uint32_t thoiDiemBaoCaoTruoc_ms = 0;
    if (millis() - thoiDiemBaoCaoTruoc_ms >= chuKyTelemetry_ms) {
        thoiDiemBaoCaoTruoc_ms = millis();
        MotorStatus baoCaoDongCo{};
        ControlStatus baoCaoDieuKhien{};
        xQueuePeek(_hangDoiTrangThaiDongCo, &baoCaoDongCo, 0);
        const bool coBaoCao = xQueuePeek(_hangDoiTrangThaiDieuKhien, &baoCaoDieuKhien, 0) == pdTRUE;
        ChanDoanLuongImu chanDoanMoi;
        xQueuePeek(_hangDoiChanDoanImu,&chanDoanMoi,0);
        const uint64_t lucBaoCao_us = nowUs();
        FaultCode loiHienTai = coBaoCao
            ? SafetyMonitor::check(baoCaoDieuKhien.trangThaiXe, lucBaoCao_us, _cauHinhChay.gocCanBang_rad)
            : FaultCode::ImuInvalid;
        if (loiHienTai == FaultCode::None) loiHienTai = baoCaoDieuKhien.loi;
        const bool gocConMoi = baoCaoDieuKhien.trangThaiXe.gocHopLe &&
            baoCaoDieuKhien.trangThaiXe.thoiDiemGoc_us && baoCaoDieuKhien.trangThaiXe.thoiDiemGoc_us <= lucBaoCao_us &&
            lucBaoCao_us-baoCaoDieuKhien.trangThaiXe.thoiDiemGoc_us < tuoiImuToiDa_us;
        const auto tuoiMs = [lucBaoCao_us](uint64_t moc_us) -> long long {
            return moc_us && moc_us <= lucBaoCao_us ? static_cast<long long>((lucBaoCao_us-moc_us)/1000u) : -1;
        };
        if (chanRgbCalib >= 0) {
            UI_Calib denTrangThai(_camBienGoc); // Chỉ ghi RGB; không đọc bus hoặc chạy lại calib.
            if (!coBaoCao) denTrangThai.baoChoDuLieu();
            else if (loiHienTai != FaultCode::None || baoCaoDongCo.loi != FaultCode::None)
                denTrangThai.baoLoi();
            else denTrangThai.baoSanSang();
        }
        if (_inDuLieuImu && baoCaoDongCo.cheDo != CheDoCanBang::Tat) _inDuLieuImu = false;
        if (_inDuLieuImu && gocConMoi && baoCaoDieuKhien.khungCamBien.trangThai == TrangThaiKhungDuLieuImu::HopLe) {
            const auto& mau = baoCaoDieuKhien.khungCamBien;
            // Gia tốc còn chứa trọng lực; gyro đã trừ bias. Tất cả vẫn ở hệ sensor.
            Serial.printf("RAW_IMU,%llu,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n",
                static_cast<unsigned long long>(mau.thoiDiemLayMau_us),
                mau.giaTocX_mps2,mau.giaTocY_mps2,mau.giaTocZ_mps2,
                mau.tocDoGocX_radps,mau.tocDoGocY_radps,mau.tocDoGocZ_radps);
        }
        if (batTelemetryWeb && (millis() - mocTelemetryWebTruocMs >= 400u)) {
            mocTelemetryWebTruocMs = millis();
            dieuKhienKhongDay.ghiNhatKyDinhDang(
                "EN=%d ctrl=%s fault=%s thetaDeg=%.2f targetDeg=%.2f x=%.4f v=%.4f STEP=%lld,%lld mksL=0x%02X:%lu:%lld mksR=0x%02X:%lu:%lld",
                baoCaoDongCo.trangThaiBoDieuKhienDongCo.daBat,
                baoCaoDongCo.dungPidMo ? "PID_MO" : "PID_THUAN",
                faultName(baoCaoDongCo.loi),
                baoCaoDieuKhien.trangThaiXe.gocNghien_rad * doiRadSangDo,
                baoCaoDieuKhien.gocDat_rad * doiRadSangDo,
                baoCaoDieuKhien.trangThaiXe.viTriDoc_m,
                baoCaoDieuKhien.trangThaiXe.vanTocDoc_mps,
                static_cast<long long>(baoCaoDongCo.khungXung.trai),
                static_cast<long long>(baoCaoDongCo.khungXung.phai),
                baoCaoDongCo.khungXung.diaChiUartTrai,
                static_cast<unsigned long>(baoCaoDongCo.khungXung.soKhungUartTrai),
                static_cast<long long>(baoCaoDongCo.khungXung.encoderTrai),
                baoCaoDongCo.khungXung.diaChiUartPhai,
                static_cast<unsigned long>(baoCaoDongCo.khungXung.soKhungUartPhai),
                static_cast<long long>(baoCaoDongCo.khungXung.encoderPhai));
        }
        if (!_batTelemetry) {
            delay(1);
            return;
        }
        Serial.printf("EN=%d fuzzy=%d ctrl=%s fault=%s ready=%s motorFault=%u last=%u "
            "theta=%.4f rate=%.4f target=%.4f x=%.4f v=%.4f STEP=%lld,%lld "
            "cmd=%.1f,%.1f K=%.2f,%.2f,%.2f serviceMax=%luus "
            "roll=%.4f accel=%.2f phase=%u epoch=%lu miss=%llu dup=%lu overlap=%lu "
            "gyroD=%.4f innov=%.4f trust=%.3f vref=%.3f yawref=%.3f tilt=%.4f tiltDir=%.4f thetaDeg=%.2f targetDeg=%.2f "
            "fw=%s frame=%u angleValid=%d biasOk=%d rawSeq=%lu irqSeq=%lu irqAgeMs=%lld imuAgeMs=%lld okAgeMs=%lld "
            "imuOk=%lu imuWait=%lu imuI2c=%lu imuEmpty=%lu ackRecovery=%lu imuLate=%lu readStatus=%u "
            "i2cLast=%u regLast=0x%02X rx=%u/%u used=%u readUs=%lu readMaxUs=%lu "
            "rxErr=%u/%u usedErr=%u errReadUs=%lu hw=%s wErrRad=%.4f,%.4f wBu=%.3f,%.3f "
            "eqDeg=%.2f bench=%d vloop=%d mksL=0x%02X:%lu:%lld mksR=0x%02X:%lu:%lld\n",
            baoCaoDongCo.trangThaiBoDieuKhienDongCo.daBat, baoCaoDongCo.dungPidMo,
            baoCaoDongCo.dungPidMo ? "PID_MO" : "PID_THUAN",
            faultName(baoCaoDongCo.loi), faultName(loiHienTai),
            static_cast<unsigned>(baoCaoDongCo.trangThaiBoDieuKhienDongCo.loi), static_cast<unsigned>(baoCaoDongCo.ketQuaCuoi),
            baoCaoDieuKhien.trangThaiXe.gocNghien_rad, baoCaoDieuKhien.trangThaiXe.tocDoNghien_radps, baoCaoDieuKhien.gocDat_rad,
            baoCaoDieuKhien.trangThaiXe.viTriDoc_m, baoCaoDieuKhien.trangThaiXe.vanTocDoc_mps,
            static_cast<long long>(baoCaoDongCo.khungXung.trai), static_cast<long long>(baoCaoDongCo.khungXung.phai),
            baoCaoDongCo.trangThaiBoDieuKhienDongCo.tocDoBuocYeuCauTrai_buocTrenGiay,
            baoCaoDongCo.trangThaiBoDieuKhienDongCo.tocDoBuocYeuCauPhai_buocTrenGiay,
            baoCaoDieuKhien.heSoPid.heSoTiLe, baoCaoDieuKhien.heSoPid.heSoTichPhan,
            baoCaoDieuKhien.heSoPid.heSoViPhan,
            static_cast<unsigned long>(baoCaoDongCo.khoangCachPhucVuLonNhat_us),
            baoCaoDieuKhien.trangThaiXe.gocNghiengNgang_rad,
            baoCaoDieuKhien.trangThaiXe.doLonGiaToc_mps2,
            static_cast<unsigned>(baoCaoDongCo.cheDo),
            static_cast<unsigned long>(baoCaoDongCo.maPhien),
            static_cast<unsigned long long>(baoCaoDieuKhien.soMauBoLo),
            static_cast<unsigned long>(baoCaoDieuKhien.soThongBaoTrung),
            static_cast<unsigned long>(chanDoanMoi.soLanDocChongNgat),
            baoCaoDieuKhien.gyroDaLoc_radps, baoCaoDieuKhien.trangThaiXe.saiLechGiaToc_rad,
            baoCaoDieuKhien.trangThaiXe.doTinCayGiaToc, baoCaoDieuKhien.vanTocMucTieu_mps,
            baoCaoDieuKhien.tocDoQuayMucTieu_radps, baoCaoDieuKhien.trangThaiXe.gocNghienTong_rad,
            baoCaoDieuKhien.trangThaiXe.huongNghien_rad,
            baoCaoDieuKhien.trangThaiXe.gocNghien_rad*57.295779513f, baoCaoDieuKhien.gocDat_rad*57.295779513f,
            phienBanFirmware, static_cast<unsigned>(baoCaoDieuKhien.khungCamBien.trangThai),
            gocConMoi, baoCaoDieuKhien.khungCamBien.daHieuChinhConQuay,
            static_cast<unsigned long>(baoCaoDieuKhien.khungCamBien.soMauLayDuoc),
            static_cast<unsigned long>(chanDoanMoi.soThuTuNgat),
            tuoiMs(chanDoanMoi.thoiDiemNgatCuoi_us),
            tuoiMs(baoCaoDieuKhien.trangThaiXe.thoiDiemGoc_us),
            tuoiMs(chanDoanMoi.thoiDiemDocTotCuoi_us),
            static_cast<unsigned long>(chanDoanMoi.soMauTot),
            static_cast<unsigned long>(chanDoanMoi.soLanHetChoNgat),
            static_cast<unsigned long>(chanDoanMoi.soLoiI2c),
            static_cast<unsigned long>(chanDoanMoi.soLanKhongCoDataReady),
            static_cast<unsigned long>(chanDoanMoi.soLanAckPhucHoi),
            static_cast<unsigned long>(chanDoanMoi.soLanDocQuaChuKy), static_cast<unsigned>(chanDoanMoi.trangThaiDocCuoi),
            chanDoanMoi.maLoiI2cCuoi, chanDoanMoi.thanhGhiLoiCuoi,
            chanDoanMoi.giaoDichCuoi.soByteNhan,chanDoanMoi.giaoDichCuoi.soByteYeuCau,
            chanDoanMoi.giaoDichCuoi.soByteDaDoc,static_cast<unsigned long>(chanDoanMoi.giaoDichCuoi.thoiGian_us),
            static_cast<unsigned long>(chanDoanMoi.thoiGianDocLonNhat_us),
            chanDoanMoi.giaoDichLoiCuoi.soByteNhan,chanDoanMoi.giaoDichLoiCuoi.soByteYeuCau,
            chanDoanMoi.giaoDichLoiCuoi.soByteDaDoc,static_cast<unsigned long>(chanDoanMoi.giaoDichLoiCuoi.thoiGian_us),
            dungPhanCungTmcMt ? "TMC_MT" : "MKS",
            baoCaoDieuKhien.vongKinBanh.saiLechGocTrai_rad,
            baoCaoDieuKhien.vongKinBanh.saiLechGocPhai_rad,
            baoCaoDieuKhien.vongKinBanh.vanTocBuTrai_mps,
            baoCaoDieuKhien.vongKinBanh.vanTocBuPhai_mps,
            _cauHinhChay.gocCanBang_rad * doiRadSangDo,
            _cauHinhChay.cheDoThuKhongMaSat ? 1 : 0,
            _cauHinhChay.batVongNgoai ? 1 : 0,
            baoCaoDongCo.khungXung.diaChiUartTrai,
            static_cast<unsigned long>(baoCaoDongCo.khungXung.soKhungUartTrai),
            static_cast<long long>(baoCaoDongCo.khungXung.encoderTrai),
            baoCaoDongCo.khungXung.diaChiUartPhai,
            static_cast<unsigned long>(baoCaoDongCo.khungXung.soKhungUartPhai),
            static_cast<long long>(baoCaoDongCo.khungXung.encoderPhai));
    }
    delay(1);
}
