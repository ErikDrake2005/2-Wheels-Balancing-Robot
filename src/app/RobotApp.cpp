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
#include "core/MaTranLapLsmNvs.h"
#include "ThongKeMauImu.h"
#include <Preferences.h>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
constexpr uint32_t suKienDinhKy = 1u, suKienBatDongCo = 2u, suKienTatDongCo = 4u, suKienDungKhan = 8u,
                   suKienXoaLoi = 16u, suKienDoiCheDoMo = 32u, suKienChonPidMo = 64u, suKienChonPidThuan = 128u,
                   suKienResetXung = 256u;
constexpr const char* nvsNamespace = nvsCauHinhImu;
constexpr uint32_t nvsMagic = 0x58434232u; // "XCB2"
constexpr uint16_t soMauCalibImuTask = 2u * tanSoLayMauImu_hz;
constexpr uint16_t soMauCalibBoot = 3u * tanSoLayMauImu_hz / 2u;
constexpr float doiRadSangDo = 57.295779513f;
constexpr float doiDoSangRad = 0.01745329252f;
uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
bool luuMaTranMoi(uint8_t who, const MaTranQuayImu& maTran) {
    Preferences prefs;
    if (!prefs.begin(nvsNamespace, false)) return false;
    if (dungLsm6dsv)
        return mountLsm::luu(prefs, who, maTran, ::gocCanBang_rad);
    return prefs.putBytes("mount", &maTran, sizeof(maTran)) == sizeof(maTran);
}
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
void RobotApp::imuPollTimer(void* nguCanh) {
    auto* ungDung = static_cast<RobotApp*>(nguCanh);
    if (ungDung->_tacVuImu) xTaskNotifyGive(ungDung->_tacVuImu);
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
    if (_boHenImu) { esp_timer_stop(_boHenImu); esp_timer_delete(_boHenImu); _boHenImu = nullptr; }
    if (dungNgatImu && chanNgatMpu >= 0) detachInterrupt(chanNgatMpu);
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

uint32_t RobotApp::tanSoImuHienTai_hz() const {
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
    return _camBienGoc.tanSoLayMauThucTe_hz();
#else
    return tanSoLayMauImu_hz;
#endif
}

bool RobotApp::begin() {
    Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(10);
#endif
    _cauHinhChay = CauHinhChay{};
    bool daNapNvs = false;
    {
        Preferences prefs;
        if (prefs.begin(nvsNamespace, true)) {
            if (prefs.getUInt("magic", 0u) == nvsMagic) {
                const auto docHeSo = [&](const char* key, float macDinh) {
                    const float value = prefs.getFloat(key, macDinh);
                    if (!dungProfileImuRieng) return value;
                    if (std::isfinite(value) && value >= 0.0f) return value;
                    Serial.printf("NVS WARN: %s khong hop le, dung mac dinh %.5g\n", key, macDinh);
                    return macDinh;
                };
                _cauHinhChay.kpGocThuan = docHeSo("kp_p", kpGocPidThuan);
                _cauHinhChay.kiGocThuan = docHeSo("ki_p", kiGocPidThuan);
                _cauHinhChay.kdGocThuan = docHeSo("kd_p", kdGocPidThuan);
                _cauHinhChay.kpVanTocThuan = docHeSo("kpv_p", kpVanTocPidThuan);
                _cauHinhChay.kiVanTocThuan = docHeSo("kiv_p", kiVanTocPidThuan);
                _cauHinhChay.kpGocMo = docHeSo("kp_m", kpGoc);
                _cauHinhChay.kiGocMo = docHeSo("ki_m", kiGoc);
                _cauHinhChay.kdGocMo = docHeSo("kd_m", kdGoc);
                _cauHinhChay.kpVanTocMo = docHeSo("kpv_m", kpVanToc);
                _cauHinhChay.kiVanTocMo = docHeSo("kiv_m", kiVanToc);
                const float refLuu = dungLsm6dsv ? ::gocCanBang_rad : prefs.getFloat("ref_rad", ::gocCanBang_rad);
                _cauHinhChay.gocCanBang_rad = dungProfileImuRieng && !std::isfinite(refLuu)
                    ? ::gocCanBang_rad : std::clamp(refLuu, -0.35f, 0.35f);
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
                if (dungMpu6050 && prefs.isKey("mount")) {
                    MaTranQuayImu mount;
                    if (prefs.getBytesLength("mount") == sizeof(mount) &&
                        prefs.getBytes("mount", &mount, sizeof(mount)) == sizeof(mount) &&
                        HeTrucImu::kiemTraMaTran(mount)) {
                        _heTrucImu = HeTrucImu(mount);
                        _coMaTranLapLsm = true;
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
    if (!dungProfileImuRieng && !batUiCalib && !_boUocLuongGoc.heTrucHopLe()) {
        Serial.println("BOOT ERROR: ma tran lap IMU khong phai phep quay hop le");
        return false;
    }
    if ((!dungNgatImu && !dungMpu6050) || chuKyPhucVuMotor_us >= khoangCachPhucVuMotorToiDa_us ||
        1000000u / tanSoLayMauImu_hz >= thoiGianHetHanLenhMotor_us) return false;
    if (!_capDongCo.begin() || !_boDemXung.begin()) {
        uiCalib.baoLoi();
        Serial.println("BOOT ERROR: MCPWM/PCNT"); return false;
    }
    if (!_camBienGoc.begin()) {
        uiCalib.baoLoi();
        Serial.printf("BOOT ERROR: %s init WHO=0x%02X expected=0x%02X busError=%u reg=0x%02X\n",
            tenImu, _camBienGoc.maNhanDang(), maNhanDangImuDuKien, static_cast<unsigned>(_camBienGoc.maLoiI2cCuoi()), _camBienGoc.thanhGhiCuoi());
        return false;
    }
    Serial.printf("IMU_BOOT %s bus=%s hz=%lu NVS=%s: kiem tra ID/cau hinh/mau moi; motor tat.\n",
        tenImu, giaoTiepImu, static_cast<unsigned long>(tanSoImuHienTai_hz()), nvsNamespace);
    BaoCaoKiemTraIcm baoCaoImu;
    const bool camBienDat = _camBienGoc.kiemTraSauKhoiDong(baoCaoImu);
    constexpr const char* bootTag = dungLsm6dsv ? "LSM_BOOT" : (dungMpu6050 ? "MPU_BOOT" : "ICM_BOOT");
    Serial.printf("%s %s WHO=0x%02X good=%u bad=%u time_ms=%lu %s=%u reg=0x%02X\n",
        bootTag, baoCaoImu.ketQua, _camBienGoc.maNhanDang(), baoCaoImu.soMauTot, baoCaoImu.soMauLoi,
        static_cast<unsigned long>(baoCaoImu.thoiGian_ms), dungLsm6dsv ? "spi" : "i2c",
        static_cast<unsigned>(_camBienGoc.maLoiI2cCuoi()), _camBienGoc.thanhGhiCuoi());
    const auto& mauKiemTra = baoCaoImu.mauCuoi;
    Serial.printf("%s_SAMPLE a_mps2=%.4f,%.4f,%.4f gyro_radps=%.4f,%.4f,%.4f T_C=%.2f\n",
        bootTag, mauKiemTra.giaTocX_mps2, mauKiemTra.giaTocY_mps2, mauKiemTra.giaTocZ_mps2,
        mauKiemTra.tocDoGocX_radps, mauKiemTra.tocDoGocY_radps, mauKiemTra.tocDoGocZ_radps,
        mauKiemTra.nhietDoCamBien_C);
    if (!camBienDat) {
        uiCalib.baoLoi();
        Serial.println("BOOT ERROR: kiem tra du lieu IMU khong dat; khong vao calib/can bang.");
        return false;
    }
    if (dungLsm6dsv) {
        Preferences prefs;
        mountLsm::BanGhi banGhi{};
        if (prefs.begin(nvsNamespace, true) && mountLsm::doc(prefs, _camBienGoc.maNhanDang(), banGhi)) {
            _heTrucImu = HeTrucImu(banGhi.maTran);
            _cauHinhChay.gocCanBang_rad = banGhi.ref_rad;
            _coMaTranLapLsm = true;
        }
    }
    // Lúc bật nguồn: chỉ đọc nút BOOT (GPIO0) để tránh nhiễu khi MKS chưa cấp nguồn kéo thấp GPIO3;
    // khi đã khởi động vào wizard calib 2 trục, cả GPIO3 (S3), GPIO0 (BOOT) và Web đều dùng được.
    bool giuNutDeCalib3TuThe = false;
    if (chanNutCalibPhu >= 0) {
        pinMode(chanNutCalibPhu, nutCalibMucThap ? INPUT_PULLUP : INPUT_PULLDOWN);
        delay(5);
        giuNutDeCalib3TuThe = (digitalRead(chanNutCalibPhu) == (nutCalibMucThap ? LOW : HIGH));
    }
    const bool daXacNhanLapCode = dungLsm6dsv ? lsm6dsvTune::daXacNhanMaTranLap
                                              : (dungMpu6050 ? mpu6050Tune::daXacNhanMaTranLap : false);
    if (dungProfileImuRieng && !_coMaTranLapLsm && daXacNhanLapCode && _heTrucImu.hopLe())
        _coMaTranLapLsm = true;
    const bool canCalibLapRieng = dungProfileImuRieng && !_coMaTranLapLsm;
    if (canCalibLapRieng && dungLsm6dsv)
        Serial.println("LSM_CALIB_REQUIRED: giu xe dung yen va gui c de calib dung theo huong tien da cau hinh; motor OFF.");
    else if (canCalibLapRieng)
        Serial.printf("%s_CALIB_REQUIRED: chua co ma tran lap; mo WiFi Web hoac dung nut S3 (GPIO%d) / BOOT (GPIO%d), motor khoa OFF.\n",
            tenImu, chanNutCalib, chanNutCalibPhu);
    if (dungProfileImuRieng && !batUiCalib && (canCalibLapRieng || giuNutDeCalib3TuThe)) {
        _cauHinhChay.khoaMotorCalib = true;
        _cauHinhChay.maTranLap = _heTrucImu.maTran();
        _batDauCalib2TrucKhiMo = !dungLsm6dsv || giuNutDeCalib3TuThe;
        if (_coMaTranLapLsm && _camBienGoc.hieuChinhConQuay(soMauCalibBoot)) {
            _camBienGoc.layDoLechConQuay_dps(_biasX_dps, _biasY_dps, _biasZ_dps, _coBiasLuu);
        } else {
            _biasX_dps = _biasY_dps = _biasZ_dps = 0.0f;
            _coBiasLuu = false;
        }
    } else if (batUiCalib || giuNutDeCalib3TuThe || canCalibLapRieng) {
        MaTranQuayImu maTranLap;
        uiCalib.chay(maTranLap);
        _heTrucImu = HeTrucImu(maTranLap);
        _cauHinhChay.maTranLap = maTranLap;
        _cauHinhChay.khoaMotorCalib = false;
        _camBienGoc.layDoLechConQuay_dps(_biasX_dps, _biasY_dps, _biasZ_dps, _coBiasLuu);
        if (dungProfileImuRieng) {
            _coMaTranLapLsm = _coBiasLuu && _heTrucImu.hopLe();
            if (!_coMaTranLapLsm) return false;
            // Store mounting only when calibrated; a single NVS blob is updated atomically.
            if (!luuMaTranMoi(_camBienGoc.maNhanDang(), maTranLap)) {
                Serial.printf("BOOT ERROR: khong luu duoc ma tran lap %s\n", tenImu);
                return false;
            }
            if (dungLsm6dsv) _cauHinhChay.gocCanBang_rad = ::gocCanBang_rad;
            if (!luuCauHinhVaoNvs()) return false;
        }
    } else {
        if (!dungProfileImuRieng) _heTrucImu = HeTrucImu();
        _cauHinhChay.maTranLap = _heTrucImu.maTran();
        _cauHinhChay.khoaMotorCalib = false;
        Serial.printf("FAST_BOOT: %s mount=%s; giu GPIO%d khi bat nguon de calib 3 tu the.\n",
            tenImu,
            dungLsm6dsv ? "LSM_PROFILE/NVS" : (dungMpu6050 ? "MPU_PROFILE/NVS" : "ImuMountConfig.h"),
            chanNutCalibPhu);
        Serial.println("FAST_BOOT: Dang do bias gyro (1.5s)...");
        if (_camBienGoc.hieuChinhConQuay(soMauCalibBoot)) {
            _camBienGoc.layDoLechConQuay_dps(_biasX_dps, _biasY_dps, _biasZ_dps, _coBiasLuu);
        } else {
            if (dungProfileImuRieng) {
                Serial.printf("BOOT ERROR: %s gyro calibration failed; giu yen va RST. Khong dung bias cu/0 de arm.\n", tenImu);
                return false;
            }
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
    _boDemXung.datTamDungUart(_cauHinhChay.khoaMotorCalib);
    if (!dungPhanCungTmcMt && batUartEncoderMks) {
        Serial.printf("MKS_UART san sang: TX=IO%d, RX1_L=IO%d (0x%02X), RX2_R=IO%d (0x%02X), %lubps, 65536/rev.\n",
                      chanUartTxMks, chanUartRxMksTrai, diaChiUartMksTrai,
                      chanUartRxMksPhai, diaChiUartMksPhai,
                      static_cast<unsigned long>(tocDoBaudUartMks_bps));
    }
    auto& biasBoot = _cauHinhChay.biasGyro_dps;
    _camBienGoc.layDoLechConQuay_dps(biasBoot.x, biasBoot.y, biasBoot.z, _cauHinhChay.coBiasGyro);
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
    _cauHinhChay.normGiaTocTinh_mps2 = _camBienGoc.normGiaTocTinh_mps2();
#endif
    _hangDoiImu = xQueueCreate(1, sizeof(MauImuDaDoc));
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
    trangThaiBanDau.thoiDiemSnapshot_us = nowUs();
    xQueueOverwrite(_hangDoiTrangThaiDongCo, &trangThaiBanDau);
    if (xTaskCreatePinnedToCore(imuEntry, "Imu", 4096, this, 4, &_tacVuImu, 1) != pdPASS ||
        xTaskCreatePinnedToCore(controlEntry, "Control", 6144, this, 3, &_tacVuDieuKhien, 1) != pdPASS ||
        xTaskCreatePinnedToCore(motorEntry, "Motor", 4096, this, 5, &_tacVuDongCo, 1) != pdPASS) {
        if (batUiCalib) uiCalib.baoLoi();
        cleanupStartup(); return false;
    }
    if (dungNgatImu) {
        pinMode(chanNgatMpu, INPUT);
        attachInterruptArg(chanNgatMpu, imuInterrupt, this, ngatImuCao ? RISING : FALLING);
        // Nếu INT đã giữ mức trước attach, chưa có cạnh mới. Chỉ ImuTask ACK cờ tồn
        // để mở lại cạnh kế tiếp; không xuất mẫu hợp lệ với timestamp giả ở lần đánh thức này.
        xTaskNotifyGive(_tacVuImu);
    } else {
        esp_timer_create_args_t cauHinhBoHenImu{};
        cauHinhBoHenImu.callback = imuPollTimer;
        cauHinhBoHenImu.arg = this;
        cauHinhBoHenImu.dispatch_method = ESP_TIMER_TASK;
        cauHinhBoHenImu.name = "ImuPoll";
        cauHinhBoHenImu.skip_unhandled_events = true;
        if (esp_timer_create(&cauHinhBoHenImu, &_boHenImu) != ESP_OK) {
            if (batUiCalib) uiCalib.baoLoi();
            cleanupStartup(); return false;
        }
        xTaskNotifyGive(_tacVuImu);
    }
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
        [this](float vanTocTienLui, float tocDoQuayNgang, const MucLenhWeb& mucWeb) {
            nhanLenhDiChuyenTuXa(vanTocTienLui, tocDoQuayNgang, mucWeb);
        },
        [this](char kyTuLenh) { xuLyPhimDonSerial(kyTuLenh); },
        [this](const char* dongLenh) { xuLyDongLenhSerial(dongLenh); },
        [this](ThongTinTrangThaiWeb& trangThaiRa) { dienTrangThaiWeb(trangThaiRa); },
        [this](const uint8_t* mac, const uint8_t* data, size_t length) {
            return lienKetChanDoan.receive(mac, data, length, nowUs());
        });
    if (dieuKhienKhongDay.espNowSanSang()) lienKetChanDoan.begin();
    if (dungLsm6dsv && !logRung.begin(lienKetChanDoan, static_cast<uint16_t>(tanSoImuHienTai_hz())))
        inDongSerial("VIBLOG ERROR: khong tao duoc bo ghi log.");
    inDinhDangSerial("FIRMWARE %s; IMU=%s bus=%s busHz=%lu; NVS=%s; mode=%s\n",
        phienBanFirmware,tenImu,giaoTiepImu,static_cast<unsigned long>(tanSoBusImu_hz),
        daNapNvs ? "LOADED" : "DEFAULT", _cauHinhChay.dungPidMo ? "PID_MO" : "PID_THUAN");
    if (khongDayOk || dieuKhienKhongDay.wifiSanSang() || dieuKhienKhongDay.espNowSanSang()) {
        inDinhDangSerial("WIFI_AP SSID=%s PASS=%s URL=http://%s | ESP_NOW=%s MAC=%s CH=%u\n",
                      tenMangWifiAp, matKhauWifiAp, dieuKhienKhongDay.diaChiIpAp(),
                      dieuKhienKhongDay.espNowSanSang() ? "ON" : "OFF",
                      dieuKhienKhongDay.diaChiMacXe(), static_cast<unsigned>(kenhWifiAp));
    }
    if (dungPhanCungTmcMt) {
        inDinhDangSerial("READY %s %luHz; odometry=MT6701_AB_4x(%lu); driver=TMC2209_ClosedLoop; hardwareConfirmed=%d\n",
                      tenImu, static_cast<unsigned long>(tanSoImuHienTai_hz()),
                      static_cast<unsigned long>(soXungEncoderMoiVongDongCo),
                      daXacNhanCauHinhPhanCungMotor);
    } else {
        inDinhDangSerial("READY %s %luHz; odometry=STEP/DIR_PCNT+MKS_V1.1_UART(TX=IO%d,RX1=IO%d,RX2=IO%d,%lubps); MKS=42C_V1.1 MStep=%lu; hardwareConfirmed=%d\n",
                      tenImu, static_cast<unsigned long>(tanSoImuHienTai_hz()),
                      chanUartTxMks, chanUartRxMksTrai, chanUartRxMksPhai,
                      static_cast<unsigned long>(tocDoBaudUartMks_bps),
                      static_cast<unsigned long>(soViBuocDauVaoMks), daXacNhanCauHinhPhanCungMotor);
    }
    _nutCalibChinh.chan = chanNutCalib;
    _nutCalibPhu.chan = chanNutCalibPhu;
    dongBoNutVatLyCalib();
    inTroGiup();
    if (_batDauCalib2TrucKhiMo) {
        batDauCalib2Truc(true);
    }
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
    YeuCauCalibImu yeuCauCalibTinh{};
    uint32_t phienBanBiasDangDung = 0;

    uint32_t phienCalib2 = 0, daXuLyXacNhan2 = 0;
    PhaCalib2Truc pha2 = PhaCalib2Truc::KhongHoatDong;
    uint32_t lucChoSettle2_ms = 0, hanBuoc2_ms = 0;
    uint64_t mocGyroQuay2_us = 0;
    float cu2BiasX_dps = 0.0f, cu2BiasY_dps = 0.0f, cu2BiasZ_dps = 0.0f;
    float moi2BiasX_dps = 0.0f, moi2BiasY_dps = 0.0f, moi2BiasZ_dps = 0.0f;
    Vec3Imu a2Dung{}, a2Truoc{}, a2Sau{};
#if CAU_HINH_IMU == IMU_MPU6050_I2C
    uint32_t soThuTuMauPoll = 0;
    uint64_t soMauBoLoPoll = 0;
    uint64_t mocMauTotTruoc_us = 0;
#elif CAU_HINH_IMU == IMU_LSM6DSV_SPI
    uint16_t soLoiIoLienTiep = 0;
    uint32_t lucThuPhucHoiCuoi_ms = 0;
#endif

    for (;;) {
        {
            portENTER_CRITICAL(&_khoaCalib2Truc);
            const uint32_t phienYeuCau = _calib2Truc.phienYeuCau;
            const uint32_t cXacNhan = _calib2Truc.lenhXacNhan;
            const bool batDau = _calib2Truc.dangYeuCauCalib;
            portEXIT_CRITICAL(&_khoaCalib2Truc);

            if (phienYeuCau != phienCalib2 && !batDau) {
                phienCalib2 = phienYeuCau;
                daXuLyXacNhan2 = cXacNhan;
                pha2 = PhaCalib2Truc::KhongHoatDong;
                _thongKeCalib2Truc.xoa();
                portENTER_CRITICAL(&_khoaCalib2Truc);
                _calib2Truc.pha = pha2;
                _calib2Truc.soMauHienTai = 0;
                _calib2Truc.tongMauCan = 0;
                std::snprintf(_calib2Truc.lyDoLoi, sizeof(_calib2Truc.lyDoLoi), "huy");
                portEXIT_CRITICAL(&_khoaCalib2Truc);
            } else if (phienYeuCau != phienCalib2) {
                phienCalib2 = phienYeuCau;
                daXuLyXacNhan2 = cXacNhan;
                pha2 = PhaCalib2Truc::ChoDungYen;
                _luotCalibDoc = HieuChuanNghiengDoc{};
                _thongKeCalib2Truc.xoa();
                hanBuoc2_ms = millis() + 60000u;
                portENTER_CRITICAL(&_khoaCalib2Truc);
                _calib2Truc.pha = pha2;
                _calib2Truc.soMauHienTai = 0;
                _calib2Truc.tongMauCan = soMauCalibBoot;
                _calib2Truc.coKetQuaMoi = false;
                _calib2Truc.thanhCong = false;
                std::snprintf(_calib2Truc.lyDoLoi, sizeof(_calib2Truc.lyDoLoi), "none");
                portEXIT_CRITICAL(&_khoaCalib2Truc);
            } else if (cXacNhan != daXuLyXacNhan2) {
                daXuLyXacNhan2 = cXacNhan;
                const uint32_t bayGio_ms = millis();
                if (pha2 == PhaCalib2Truc::ChoDungYen) {
                    bool tmpBiasOk = false;
                    _camBienGoc.layDoLechConQuay_dps(cu2BiasX_dps, cu2BiasY_dps, cu2BiasZ_dps, tmpBiasOk);
                    pha2 = PhaCalib2Truc::DangDoDung;
                    lucChoSettle2_ms = bayGio_ms + thoiGianLangSauNhanNut_ms;
                    hanBuoc2_ms = lucChoSettle2_ms + 20000u;
                    _thongKeCalib2Truc.xoa();
                } else if (pha2 == PhaCalib2Truc::ChoHaDau) {
                    pha2 = PhaCalib2Truc::DangDoTruoc;
                    lucChoSettle2_ms = bayGio_ms + thoiGianLangSauNhanNut_ms;
                    hanBuoc2_ms = lucChoSettle2_ms + 20000u;
                    _thongKeCalib2Truc.xoa();
                } else if (pha2 == PhaCalib2Truc::DangQuayPitch) {
                    pha2 = PhaCalib2Truc::DangDoSau;
                    lucChoSettle2_ms = bayGio_ms + thoiGianLangSauNhanNut_ms;
                    hanBuoc2_ms = lucChoSettle2_ms + 20000u;
                    _thongKeCalib2Truc.xoa();
                }
                if (pha2 == PhaCalib2Truc::DangDoDung || pha2 == PhaCalib2Truc::DangDoTruoc ||
                    pha2 == PhaCalib2Truc::DangDoSau) {
                    portENTER_CRITICAL(&_khoaCalib2Truc);
                    _calib2Truc.pha = pha2;
                    _calib2Truc.soMauHienTai = 0;
                    _calib2Truc.tongMauCan = (pha2 == PhaCalib2Truc::DangDoDung) ? soMauCalibBoot : 80u;
                    portEXIT_CRITICAL(&_khoaCalib2Truc);
                }
            } else if ((pha2 == PhaCalib2Truc::DangDoDung || pha2 == PhaCalib2Truc::DangDoTruoc ||
                        pha2 == PhaCalib2Truc::DangQuayPitch || pha2 == PhaCalib2Truc::DangDoSau) &&
                       millis() >= hanBuoc2_ms) {
                pha2 = PhaCalib2Truc::Loi;
                portENTER_CRITICAL(&_khoaCalib2Truc);
                _calib2Truc.pha = pha2;
                _calib2Truc.thanhCong = false;
                _calib2Truc.coKetQuaMoi = true;
                _calib2Truc.phienKetQua = phienCalib2;
                std::snprintf(_calib2Truc.lyDoLoi, sizeof(_calib2Truc.lyDoLoi), "het_gio_buoc_calib");
                portEXIT_CRITICAL(&_khoaCalib2Truc);
            }
        }

        YeuCauCalibImu yeuCauCalib{};
        if (xQueuePeek(_hangDoiYeuCauCalib, &yeuCauCalib, 0) == pdTRUE &&
            yeuCauCalib.maYeuCau != 0 && yeuCauCalib.maYeuCau != maYeuCauCalibDangXuLy) {
            maYeuCauCalibDangXuLy = yeuCauCalib.maYeuCau;
            yeuCauCalibTinh = yeuCauCalib;
            dangCalib = true;
            lucBatDauCalib_ms = millis();
            soLanThuLaiCalib = 0;
            thongKeCalib.xoa();
            bool daHieuChinhTmp = false;
            _camBienGoc.layDoLechConQuay_dps(cuBiasX_dps, cuBiasY_dps, cuBiasZ_dps, daHieuChinhTmp);
        }

        CauHinhChay cauHinhMau{};
        if (xQueuePeek(_hangDoiCauHinh, &cauHinhMau, 0) == pdTRUE &&
            phienBanBiasDangDung != cauHinhMau.phienBanMaTranLap) {
            if (cauHinhMau.coBiasGyro) {
                const auto& b = cauHinhMau.biasGyro_dps;
                _camBienGoc.datDoLechConQuay_dps(b.x, b.y, b.z);
            }
            phienBanBiasDangDung = cauHinhMau.phienBanMaTranLap;
        }
        MauImuDaDoc mauDoc{};
        mauDoc.phienBanMaTranLap = phienBanBiasDangDung;
        KhungDuLieuImu& khungDuLieu = mauDoc.mau;
#if CAU_HINH_IMU == IMU_MPU6050_I2C
        // Chế độ không nối chân INT (INT để trống): dùng esp_timer bám pha cờ DATA_RDY_INT (0x3A bit 0).
        const auto henDocTiep = [&](uint64_t cho_us) {
            if (_boHenImu) {
                esp_timer_stop(_boHenImu);
                esp_timer_start_once(_boHenImu, cho_us);
            }
        };
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        const uint64_t lucBatDauDoc_us = nowUs();
        const bool coMauMoi = _camBienGoc.docMauMoi(khungDuLieu);
        const uint64_t lucDocXong_us = nowUs();
        const uint32_t chuKyMau_us = 1000000u / tanSoLayMauImu_hz;
        chanDoan.giaoDichCuoi = _camBienGoc.giaoDichDocCuoi();
        if (coMauMoi && chanDoan.giaoDichCuoi.thoiGian_us > chanDoan.thoiGianDocLonNhat_us)
            chanDoan.thoiGianDocLonNhat_us = chanDoan.giaoDichCuoi.thoiGian_us;

        if (!coMauMoi) {
            if (khungDuLieu.trangThai == TrangThaiKhungDuLieuImu::LoiIO) {
                ++chanDoan.soLoiI2c;
                chanDoan.maLoiI2cCuoi = static_cast<uint8_t>(_camBienGoc.maLoiI2cCuoi());
                chanDoan.thanhGhiLoiCuoi = _camBienGoc.thanhGhiCuoi();
                chanDoan.giaoDichLoiCuoi = _camBienGoc.giaoDichDocLoiCuoi();
                chanDoan.trangThaiDocCuoi = khungDuLieu.trangThai;
                xQueueOverwrite(_hangDoiChanDoanImu, &chanDoan);
                khungDuLieu.chanDoan = chanDoan;
                khungDuLieu.soMauBoLo = soMauBoLoPoll;
                xQueueOverwrite(_hangDoiImu, &mauDoc);
                henDocTiep(mpu6050::chuKyLoiPoll_us);
                continue;
            }
            // Nếu quá 2 chu kỳ mẫu (>=10 ms) vẫn chưa có DATA_RDY mới, báo cho ControlTask hạ cờ gocHopLe.
            if (mocMauTotTruoc_us != 0 && lucDocXong_us - mocMauTotTruoc_us >= 2u * chuKyMau_us) {
                ++chanDoan.soLanKhongCoDataReady;
                chanDoan.trangThaiDocCuoi = TrangThaiKhungDuLieuImu::DuLieuCu;
                xQueueOverwrite(_hangDoiChanDoanImu, &chanDoan);
                khungDuLieu.chanDoan = chanDoan;
                khungDuLieu.soMauBoLo = soMauBoLoPoll;
                xQueueOverwrite(_hangDoiImu, &mauDoc);
            }
            henDocTiep(mpu6050::chuKyThuLaiPoll_us);
            continue;
        }

        if (lucDocXong_us < lucBatDauDoc_us || lucDocXong_us - lucBatDauDoc_us >= chuKyMau_us) {
            ++chanDoan.soLanDocQuaChuKy;
            chanDoan.trangThaiDocCuoi = TrangThaiKhungDuLieuImu::DuLieuCu;
            xQueueOverwrite(_hangDoiChanDoanImu, &chanDoan);
            henDocTiep(mpu6050::chuKyThuLaiPoll_us);
            continue;
        }

        if (mocMauTotTruoc_us != 0 && lucBatDauDoc_us > mocMauTotTruoc_us) {
            const uint64_t khoangCach_us = lucBatDauDoc_us - mocMauTotTruoc_us;
            if (khoangCach_us >= chuKyMau_us + (chuKyMau_us * 3u) / 5u) {
                const uint64_t soKy = (khoangCach_us + chuKyMau_us / 2u) / chuKyMau_us;
                if (soKy > 1u) soMauBoLoPoll += (soKy - 1u);
            }
        }
        mocMauTotTruoc_us = lucBatDauDoc_us;
        ++soThuTuMauPoll;
        khungDuLieu.thoiDiemLayMau_us = lucBatDauDoc_us;
        khungDuLieu.soMauLayDuoc = soThuTuMauPoll;
        chanDoan.soThuTuNgat = soThuTuMauPoll;
        chanDoan.thoiDiemNgatCuoi_us = lucBatDauDoc_us;
        ++chanDoan.soMauTot;
        chanDoan.thoiDiemDocTotCuoi_us = lucDocXong_us;
        chanDoan.trangThaiDocCuoi = TrangThaiKhungDuLieuImu::HopLe;
        xQueueOverwrite(_hangDoiChanDoanImu, &chanDoan);

        const uint64_t daTroi_us = nowUs() - lucBatDauDoc_us;
        const uint64_t henTiep_us = (daTroi_us < mpu6050::chuKyNguPoll_us)
            ? (mpu6050::chuKyNguPoll_us - daTroi_us)
            : mpu6050::chuKyThuLaiPoll_us;
        henDocTiep(henTiep_us);
        const auto ketQuaMau = KetQuaChotMauImu::ChapNhan;
#else
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
        // Driver ACK theo cảm biến (ICM: status; LSM: đọc output).
        // Poll phục hồi không có timestamp IRQ vẫn bị chotMauImu loại.
        _camBienGoc.docMauTheoNgat(khungDuLieu);
        chanDoan.giaoDichCuoi = _camBienGoc.giaoDichDocCuoi();
        if (chanDoan.giaoDichCuoi.thoiGian_us > chanDoan.thoiGianDocLonNhat_us)
            chanDoan.thoiGianDocLonNhat_us = chanDoan.giaoDichCuoi.thoiGian_us;
        if (khungDuLieu.trangThai == TrangThaiKhungDuLieuImu::LoiIO) {
            ++chanDoan.soLoiI2c;
            chanDoan.maLoiI2cCuoi = static_cast<uint8_t>(_camBienGoc.maLoiI2cCuoi());
            chanDoan.thanhGhiLoiCuoi = _camBienGoc.thanhGhiCuoi();
            chanDoan.giaoDichLoiCuoi = _camBienGoc.giaoDichDocLoiCuoi();
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
            if (soLoiIoLienTiep < UINT16_MAX) ++soLoiIoLienTiep;
            if (soLoiIoLienTiep >= 10u && !_motorDangBat.load() &&
                (millis() - lucThuPhucHoiCuoi_ms >= 500u)) {
                lucThuPhucHoiCuoi_ms = millis();
                float bxTmp = 0.0f, byTmp = 0.0f, bzTmp = 0.0f;
                bool coBiasTmp = false;
                _camBienGoc.layDoLechConQuay_dps(bxTmp, byTmp, bzTmp, coBiasTmp);
                if (_camBienGoc.begin()) {
                    if (coBiasTmp) _camBienGoc.datDoLechConQuay_dps(bxTmp, byTmp, bzTmp);
                    soLoiIoLienTiep = 0;
                    boLocThuTu = BoLocThuTuMau{};
                }
            }
#endif
        } else {
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
            if (khungDuLieu.trangThai == TrangThaiKhungDuLieuImu::HopLe) soLoiIoLienTiep = 0;
#endif
            if (coThongBao && thoiDiemMau_us && khungDuLieu.trangThai == TrangThaiKhungDuLieuImu::DuLieuCu)
                ++chanDoan.soLanKhongCoDataReady;
        }
        portENTER_CRITICAL(&_khoaDuLieuNgatImu);
        const uint32_t soThuTuSauDoc = _soThuTuNgat;
        portEXIT_CRITICAL(&_khoaDuLieuNgatImu);
        const auto ketQuaMau = chotMauImu(khungDuLieu,coThongBao,soThuTuMau,soThuTuSauDoc,
            thoiDiemMau_us,nowUs(),1000000u/tanSoImuHienTai_hz());
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
#endif

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
                            KetQuaCalibImu ketQua{};
                            ketQua.dungMotTuThe = yeuCauCalibTinh.dungMotTuThe;
                            ketQua.maTranLap = yeuCauCalibTinh.maTranLap;
                            const Vec3Imu aTB{thongKeCalib.trungBinh(0), thongKeCalib.trungBinh(1), thongKeCalib.trungBinh(2)};
                            const bool dungTruc = !ketQua.dungMotTuThe || HeTrucImu::tuTuTheDung(
                                aTB, yeuCauCalibTinh.huongTienSensor, ketQua.maTranLap);
                            const Vec3Imu aXe = HeTrucImu(ketQua.maTranLap).chuyenSangXe(aTB);
                            const float gocCanBangDo_rad = ketQua.dungMotTuThe ? 0.0f : std::atan2(-aXe.x, std::hypot(aXe.y, aXe.z));
                            ketQua.maYeuCau = maYeuCauCalibDangXuLy;
                            ketQua.hoanTat = true;
                            ketQua.doLechChuanGyro_dps = thongKeCalib.doLechChuanGyroLonNhat_dps();
                            ketQua.phuongSaiGiaToc = thongKeCalib.phuongSaiGiaToc();
                            ketQua.soMauDaThu = static_cast<uint16_t>(thongKeCalib.soMau());
                            ketQua.soLanThuLai = soLanThuLaiCalib;
                            if (dungTruc && aXe.z > 0 && std::isfinite(gocCanBangDo_rad) && std::fabs(gocCanBangDo_rad) <= 0.35f) {
                                ketQua.thanhCong = true;
                                ketQua.biasX_dps = bx;
                                ketQua.biasY_dps = by;
                                ketQua.biasZ_dps = bz;
                                ketQua.gocCanBangDo_rad = gocCanBangDo_rad;
                                const float normCal = doLonVecto(aTB);
                                if (dungLsm6dsv && std::isfinite(normCal) && normCal >= 9.0f && normCal <= 10.6f) {
                                    ketQua.normGiaTocTinh_mps2 = normCal;
                                }
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
                ketQua.dungMotTuThe = yeuCauCalibTinh.dungMotTuThe;
                ketQua.hoanTat = true;
                ketQua.thanhCong = false;
                ketQua.soMauDaThu = static_cast<uint16_t>(thongKeCalib.soMau());
                ketQua.soLanThuLai = soLanThuLaiCalib;
                xQueueOverwrite(_hangDoiKetQuaCalib, &ketQua);
                dangCalib = false;
            }
        }

        if (pha2 != PhaCalib2Truc::KhongHoatDong && pha2 != PhaCalib2Truc::HoanTat &&
            pha2 != PhaCalib2Truc::Loi && ketQuaMau == KetQuaChotMauImu::ChapNhan) {
            const uint32_t bayGio2_ms = millis();
            // Bias ứng viên chỉ dùng cho các mẫu của wizard. Motor/estimator vẫn thấy bias
            // đã xác nhận cho đến khi ma trận và bias mới cùng được chấp nhận.
            KhungDuLieuImu mauCalib = khungDuLieu;
            mauCalib.tocDoGocX_radps += (cu2BiasX_dps - moi2BiasX_dps) * doiDoSangRad;
            mauCalib.tocDoGocY_radps += (cu2BiasY_dps - moi2BiasY_dps) * doiDoSangRad;
            mauCalib.tocDoGocZ_radps += (cu2BiasZ_dps - moi2BiasZ_dps) * doiDoSangRad;
            if (pha2 == PhaCalib2Truc::DangDoDung && bayGio2_ms >= lucChoSettle2_ms) {
                KhungDuLieuImu mauTho = khungDuLieu;
                mauTho.tocDoGocX_radps += cu2BiasX_dps * doiDoSangRad;
                mauTho.tocDoGocY_radps += cu2BiasY_dps * doiDoSangRad;
                mauTho.tocDoGocZ_radps += cu2BiasZ_dps * doiDoSangRad;
                if (!mauImuGanTinh(mauTho, gioiHanGyroThoCalib_dps * doiDoSangRad)) {
                    _thongKeCalib2Truc.xoa();
                } else {
                    _thongKeCalib2Truc.themMau(mauTho);
                    if (_thongKeCalib2Truc.soMau() >= soMauCalibBoot) {
                        if (_thongKeCalib2Truc.onDinh()) {
                            moi2BiasX_dps = _thongKeCalib2Truc.trungBinh(3) / doiDoSangRad;
                            moi2BiasY_dps = _thongKeCalib2Truc.trungBinh(4) / doiDoSangRad;
                            moi2BiasZ_dps = _thongKeCalib2Truc.trungBinh(5) / doiDoSangRad;
                            a2Dung = {_thongKeCalib2Truc.trungBinh(0),
                                      _thongKeCalib2Truc.trungBinh(1),
                                      _thongKeCalib2Truc.trungBinh(2)};
                            _luotCalibDoc = HieuChuanNghiengDoc{};
                            pha2 = PhaCalib2Truc::ChoHaDau;
                            _thongKeCalib2Truc.xoa();
                            portENTER_CRITICAL(&_khoaCalib2Truc);
                            _calib2Truc.pha = pha2;
                            _calib2Truc.soMauHienTai = 0;
                            _calib2Truc.tongMauCan = 0;
                            portEXIT_CRITICAL(&_khoaCalib2Truc);
                        } else {
                            _thongKeCalib2Truc.xoa();
                        }
                    } else {
                        portENTER_CRITICAL(&_khoaCalib2Truc);
                        _calib2Truc.soMauHienTai = static_cast<uint16_t>(_thongKeCalib2Truc.soMau());
                        portEXIT_CRITICAL(&_khoaCalib2Truc);
                    }
                }
            } else if ((pha2 == PhaCalib2Truc::DangDoTruoc || pha2 == PhaCalib2Truc::DangDoSau) &&
                       bayGio2_ms >= lucChoSettle2_ms) {
                if (!mauImuGanTinh(mauCalib, tocDoGyroTinhCalibToiDa_radps)) {
                    _thongKeCalib2Truc.xoa();
                } else {
                    _thongKeCalib2Truc.themMau(mauCalib);
                    if (_thongKeCalib2Truc.soMau() >= 80u) {
                        if (_thongKeCalib2Truc.onDinh()) {
                            const Vec3Imu aMoc{_thongKeCalib2Truc.trungBinh(0),
                                               _thongKeCalib2Truc.trungBinh(1),
                                               _thongKeCalib2Truc.trungBinh(2)};
                            _thongKeCalib2Truc.xoa();
                            if (pha2 == PhaCalib2Truc::DangDoTruoc) {
                                a2Truoc = aMoc;
                                mocGyroQuay2_us = 0;
                                hanBuoc2_ms = millis() + 45000u;
                                pha2 = PhaCalib2Truc::DangQuayPitch;
                                portENTER_CRITICAL(&_khoaCalib2Truc);
                                _calib2Truc.pha = pha2;
                                _calib2Truc.soMauHienTai = 0;
                                _calib2Truc.tongMauCan = 0;
                                portEXIT_CRITICAL(&_khoaCalib2Truc);
                            } else {
                                a2Sau = aMoc;
                                BaoCaoCalibDoc baoCaoDoc{};
                                MaTranQuayImu maTranDoc{};
                                const bool datDoc = _luotCalibDoc.tinhMaTran(
                                    a2Dung, a2Truoc, a2Sau, maTranDoc, &baoCaoDoc);
                                pha2 = datDoc ? PhaCalib2Truc::HoanTat : PhaCalib2Truc::Loi;
                                portENTER_CRITICAL(&_khoaCalib2Truc);
                                _calib2Truc.pha = pha2;
                                _calib2Truc.soMauHienTai = 80u;
                                _calib2Truc.tongMauCan = 80u;
                                _calib2Truc.maTranKetQua = maTranDoc;
                                _calib2Truc.biasX_dps = moi2BiasX_dps;
                                _calib2Truc.biasY_dps = moi2BiasY_dps;
                                _calib2Truc.biasZ_dps = moi2BiasZ_dps;
                                _calib2Truc.baoCao = baoCaoDoc;
                                _calib2Truc.aDung = a2Dung;
                                _calib2Truc.aTruoc = a2Truoc;
                                _calib2Truc.aSau = a2Sau;
                                {
                                    const float norm2 = doLonVecto(a2Dung);
                                    if (dungLsm6dsv && std::isfinite(norm2) && norm2 >= 9.0f && norm2 <= 10.6f) {
                                        _calib2Truc.normGiaTocTinh_mps2 = norm2;
                                    }
                                }
                                _calib2Truc.thanhCong = datDoc;
                                _calib2Truc.coKetQuaMoi = true;
                                _calib2Truc.phienKetQua = phienCalib2;
                                std::snprintf(_calib2Truc.lyDoLoi, sizeof(_calib2Truc.lyDoLoi), "%s",
                                              baoCaoDoc.lyDo ? baoCaoDoc.lyDo : "loi");
                                portEXIT_CRITICAL(&_khoaCalib2Truc);
                            }
                        } else {
                            _thongKeCalib2Truc.xoa();
                        }
                    } else {
                        portENTER_CRITICAL(&_khoaCalib2Truc);
                        _calib2Truc.soMauHienTai = static_cast<uint16_t>(_thongKeCalib2Truc.soMau());
                        portEXIT_CRITICAL(&_khoaCalib2Truc);
                    }
                }
            } else if (pha2 == PhaCalib2Truc::DangQuayPitch) {
                if (mocGyroQuay2_us != 0 && khungDuLieu.thoiDiemLayMau_us > mocGyroQuay2_us) {
                    const float dt_s = static_cast<float>(khungDuLieu.thoiDiemLayMau_us - mocGyroQuay2_us) * 1e-6f;
                    if (dt_s > 0.0f && dt_s <= 0.05f) {
                        const Vec3Imu gMau{mauCalib.tocDoGocX_radps, mauCalib.tocDoGocY_radps,
                                           mauCalib.tocDoGocZ_radps};
                        _luotCalibDoc.themMau(gMau, dt_s);
                    }
                }
                mocGyroQuay2_us = khungDuLieu.thoiDiemLayMau_us;
            }
        }

        khungDuLieu.chanDoan = chanDoan;
#if CAU_HINH_IMU == IMU_MPU6050_I2C
        khungDuLieu.soMauBoLo = soMauBoLoPoll;
        khungDuLieu.soThongBaoTrung = 0;
        khungDuLieu.soLanDocChongNgat = 0;
#else
        khungDuLieu.soMauBoLo = boLocThuTu.soMauBoLo();
        khungDuLieu.soThongBaoTrung = boLocThuTu.soThongBaoTrung();
        khungDuLieu.soLanDocChongNgat = soLanDocChongNgat;
#endif
        xQueueOverwrite(_hangDoiImu, &mauDoc);
    }
}

void RobotApp::controlLoop() {
    TrangThaiXe trangThaiXe{};
    uint32_t soThuTuLenh = 0;
    uint32_t maPhienTruoc = 0;
    uint32_t phienBanCauHinhTruoc = 0;
    CauHinhChay cauHinhDangDung{};
    uint32_t phienBanMaTranTruoc = 0;
    uint32_t soLanChotViTriTruoc = 0;
    uint32_t maResetXungTruoc = 0;
    uint64_t thoiDiemXungDaXuLy_us = 0;
    for (;;) {
        MauImuDaDoc mauDoc{};
        xQueueReceive(_hangDoiImu, &mauDoc, portMAX_DELAY);
        const KhungDuLieuImu& khungDuLieu = mauDoc.mau;
        CauHinhChay cauHinhMoi{};
        if (xQueuePeek(_hangDoiCauHinh, &cauHinhMoi, 0) == pdTRUE &&
            cauHinhMoi.phienBan != phienBanCauHinhTruoc) {
            if (cauHinhMoi.phienBanMaTranLap != phienBanMaTranTruoc) {
                _boUocLuongGoc = ComplementaryEstimator(HeTrucImu(cauHinhMoi.maTranLap));
                _boDieuKhienTang.reset();
                phienBanMaTranTruoc = cauHinhMoi.phienBanMaTranLap;
            }
            apDungCauHinh(cauHinhMoi);
            cauHinhDangDung = cauHinhMoi;
            if (cauHinhMoi.soLanChotViTri != soLanChotViTriTruoc) {
                _boDieuKhienTang.chotViTriGiuTaiCho();
                soLanChotViTriTruoc = cauHinhMoi.soLanChotViTri;
            }
            phienBanCauHinhTruoc = cauHinhMoi.phienBan;
        }
        if (mauDoc.phienBanMaTranLap != phienBanMaTranTruoc) continue;
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
            (cauHinhDangDung.buGiaTocImu && !_boDieuKhienTang.cheDoThuKhongMaSat())
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
        baoCaoDieuKhien.phaDieuKhien = trangThaiDongCo.cheDo;
        baoCaoDieuKhien.dungPidMo = trangThaiDongCo.dungPidMo;
        baoCaoDieuKhien.phienBanCauHinh = phienBanCauHinhTruoc;
        baoCaoDieuKhien.bench = _boDieuKhienTang.cheDoThuKhongMaSat();
        baoCaoDieuKhien.vongNgoai = _boDieuKhienTang.batVongNgoai();
        baoCaoDieuKhien.coThuNghiem = (cauHinhDangDung.buGiaTocImu ? 1u : 0u) |
            (_boDieuKhienTang.tangGiaTocDiChuyen() ? 2u : 0u) |
            (_boDieuKhienTang.tangBamVanTocDiChuyen() ? 4u : 0u);
        baoCaoDieuKhien.heSoGocCoSo = trangThaiDongCo.dungPidMo ?
            _boDieuKhienTang.pidGocMoCoSo() : _boDieuKhienTang.pidGocThuan();
        baoCaoDieuKhien.kpVanTocCoSo = trangThaiDongCo.dungPidMo ?
            _boDieuKhienTang.kpVanTocMo() : _boDieuKhienTang.kpVanTocThuan();
        baoCaoDieuKhien.kiVanTocCoSo = trangThaiDongCo.dungPidMo ?
            _boDieuKhienTang.kiVanTocMo() : _boDieuKhienTang.kiVanTocThuan();
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
        float vanTocTuXa = choDiChuyen ? lenhDiChuyen.vanToc_mps : 0;
        float quayTuXa = choDiChuyen ? lenhDiChuyen.tocDoQuay_radps : 0;
        float vRadio = 0, yawRadio = 0;
        const bool radioDieuKhien = lienKetChanDoan.desired(thoiDiemHienTai_us, vRadio, yawRadio);
        if (radioDieuKhien) {
            vanTocTuXa = trangThaiDongCo.cheDo == CheDoCanBang::DangChay ? vRadio : 0;
            quayTuXa = trangThaiDongCo.cheDo == CheDoCanBang::DangChay ? yawRadio : 0;
        }
        if (baoCaoDieuKhien.loi == FaultCode::None &&
            !_boDieuKhienTang.update(trangThaiXe, thoiDiemHienTai_us, trangThaiDongCo.dungPidMo,
                baoCaoDieuKhien.lenhBanh, vanTocTuXa, quayTuXa))
            baoCaoDieuKhien.loi = FaultCode::InvalidControl;
        baoCaoDieuKhien.heSoPid = _boDieuKhienTang.heSoPid();
        baoCaoDieuKhien.giaTocLenh_mps2 = _boDieuKhienTang.giaTocBanhDaLenh_mps2();
        baoCaoDieuKhien.vongKinBanh = _boDieuKhienTang.trangThaiVongKinBanh();
        baoCaoDieuKhien.gocDat_rad = _boDieuKhienTang.gocDatHieuDung_rad();
        baoCaoDieuKhien.gyroDaLoc_radps = _boDieuKhienTang.tocDoDaLoc_radps();
        baoCaoDieuKhien.vanTocMucTieu_mps = _boDieuKhienTang.vanTocMucTieu_mps();
        baoCaoDieuKhien.tocDoQuayMucTieu_radps = _boDieuKhienTang.tocDoQuayMucTieu_radps();
        baoCaoDieuKhien.soThuTuLenh = ++soThuTuLenh;
        xQueueOverwrite(_hangDoiTrangThaiDieuKhien, &baoCaoDieuKhien);
        if (dungLsm6dsv && logRung.enabled()) {
            KhaoSatRung::Sample sample{};
            sample.t_us = khungDuLieu.thoiDiemLayMau_us;
            sample.sample_seq = khungDuLieu.soMauLayDuoc;
            sample.cfg = phienBanCauHinhTruoc;
            sample.mode = trangThaiDongCo.dungPidMo;
            sample.diag_flags = baoCaoDieuKhien.coThuNghiem;
            sample.flags = (trangThaiDongCo.trangThaiBoDieuKhienDongCo.daBat ? KhaoSatRung::MotorOn : 0) |
                (trangThaiXe.gocHopLe ? KhaoSatRung::AngleValid : 0);
            if (lenhDiChuyen.thoiDiem_us && thoiDiemHienTai_us >= lenhDiChuyen.thoiDiem_us &&
                thoiDiemHienTai_us - lenhDiChuyen.thoiDiem_us < UINT32_MAX)
                sample.command_age_us = thoiDiemHienTai_us - lenhDiChuyen.thoiDiem_us;
            const bool webConMoi = lenhDiChuyen.mucWeb.valid && sample.command_age_us < hanLenhDiChuyen_us;
            if (radioDieuKhien) sample.source = KhaoSatRung::Remote;
            else if (webConMoi) {
                sample.source = KhaoSatRung::Web;
                sample.forward_pct = lenhDiChuyen.mucWeb.forward_pct;
                sample.turn_pct = lenhDiChuyen.mucWeb.turn_pct;
                sample.speed_pct = lenhDiChuyen.mucWeb.speed_pct;
                sample.flags |= KhaoSatRung::UiValid;
            } else sample.source = choDiChuyen ? KhaoSatRung::Other : KhaoSatRung::None;
            if (choDiChuyen && !radioDieuKhien) sample.flags |= KhaoSatRung::CommandValid;
            sample.ax = khungDuLieu.giaTocX_mps2; sample.ay = khungDuLieu.giaTocY_mps2;
            sample.az = khungDuLieu.giaTocZ_mps2;
            sample.gx = khungDuLieu.tocDoGocX_radps; sample.gy = khungDuLieu.tocDoGocY_radps;
            sample.gz = khungDuLieu.tocDoGocZ_radps; sample.temp_c = khungDuLieu.nhietDoCamBien_C;
            sample.theta = trangThaiXe.gocNghien_rad; sample.rate = trangThaiXe.tocDoNghien_radps;
            sample.target = baoCaoDieuKhien.gocDat_rad; sample.gyro_d = baoCaoDieuKhien.gyroDaLoc_radps;
            sample.vref = baoCaoDieuKhien.vanTocMucTieu_mps; sample.yawref = baoCaoDieuKhien.tocDoQuayMucTieu_radps;
            sample.cmd_l = baoCaoDieuKhien.lenhBanh.tocDoBuocTrai_buocTrenGiay;
            sample.cmd_r = baoCaoDieuKhien.lenhBanh.tocDoBuocPhai_buocTrenGiay;
            sample.a_cmd = baoCaoDieuKhien.giaTocLenh_mps2; sample.v = trangThaiXe.vanTocDoc_mps;
            sample.v_request = vanTocTuXa; sample.yaw_request = quayTuXa;
            logRung.record(sample);
        }
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
    bool tamDungUartTruoc = cauHinhBanDau.khoaMotorCalib;
    uint32_t soThuTuLenhDaApDung = 0;
    uint64_t thoiDiemPhucVuTruoc_us = 0;
    uint64_t thoiDiemDocXungTruoc_us = 0;
    uint8_t giaiDoanTest = 0;
    uint32_t maTestDangChay = 0;
    uint64_t hanTest_us = 0;
    float testTrai_hz = 0.0f, testPhai_hz = 0.0f;
    int64_t stepTraiDau = 0, stepPhaiDau = 0, encTraiDau = 0, encPhaiDau = 0;
    char nhanTest[8]{};
    const auto xuatKetQuaTest = [&](bool biHuy) {
        KetQuaTestBanh kq{};
        kq.maYeuCau = maTestDangChay;
        kq.hoanTat = true;
        kq.biHuy = biHuy;
        kq.dStepTrai = baoCaoDongCo.khungXung.trai - stepTraiDau;
        kq.dStepPhai = baoCaoDongCo.khungXung.phai - stepPhaiDau;
        kq.dEncTrai = baoCaoDongCo.khungXung.encoderTrai - encTraiDau;
        kq.dEncPhai = baoCaoDongCo.khungXung.encoderPhai - encPhaiDau;
        kq.coUartTrai = baoCaoDongCo.khungXung.coUartTrai;
        kq.coUartPhai = baoCaoDongCo.khungXung.coUartPhai;
        std::memcpy(kq.nhan, nhanTest, sizeof(kq.nhan));
        portENTER_CRITICAL(&_khoaTestBanh);
        _ketQuaTestBanh = kq;
        portEXIT_CRITICAL(&_khoaTestBanh);
    };
    for (;;) {
        uint32_t cacSuKien = 0;
        xTaskNotifyWait(0, UINT32_MAX, &cacSuKien, pdMS_TO_TICKS(1));
        const auto lenhRadio = lienKetChanDoan.serviceMotor(nowUs(), _capDongCo.isEnabled());
        if (lenhRadio.arm) cacSuKien |= suKienBatDongCo;
        if (lenhRadio.disable) cacSuKien |= suKienTatDongCo;
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
        const bool khoaMotorCalib = coCauHinhHienTai && cauHinhHienTai.khoaMotorCalib;
        if (khoaMotorCalib != tamDungUartTruoc) {
            tamDungUartTruoc = khoaMotorCalib;
            _boDemXung.datTamDungUart(tamDungUartTruoc);
        }
        if (khoaMotorCalib && phienCanBang.cheDo() != CheDoCanBang::Tat) {
            phienCanBang.tat();
            _capDongCo.disable();
        }
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
        const bool dungCauHinhDeBat = coCauHinhHienTai && coDuLieuDieuKhien &&
            baoCaoDieuKhien.phienBanCauHinh == cauHinhHienTai.phienBan;
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

        if (giaiDoanTest != 0) {
            cacSuKien &= ~suKienBatDongCo;
        }

        // Cùng một lượt nhận lệnh: dừng khẩn/tắt luôn ưu tiên hơn bật.
        if (cacSuKien & suKienDungKhan) {
            lienKetChanDoan.cancel();
            baoCaoDongCo.loi = FaultCode::EmergencyStop;
            phienCanBang.tat();
            _capDongCo.emergencyStop();
        } else if (cacSuKien & suKienTatDongCo) {
            lienKetChanDoan.cancel();
            phienCanBang.tat();
            _capDongCo.disable();
        } else if (cacSuKien & suKienXoaLoi) {
            if (!_capDongCo.isEnabled() && _capDongCo.clearFault()) {
                phienCanBang.tat();
                baoCaoDongCo.loi = FaultCode::None;
            }
        } else if ((cacSuKien & suKienBatDongCo) &&
                   !khoaMotorCalib && dungCauHinhDeBat &&
                   phienCanBang.cheDo() == CheDoCanBang::Tat &&
                   baoCaoDongCo.loi == FaultCode::None && lenhConMoi &&
                   loiAnToan == FaultCode::None &&
                   SafetyMonitor::canArm(baoCaoDieuKhien.trangThaiXe, thoiDiemHienTai_us, gocThamChieu_rad)) {
            phienCanBang.yeuCauBat(thoiDiemHienTai_us);
        }

        if ((cacSuKien & suKienResetXung) && phienCanBang.cheDo() == CheDoCanBang::Tat && giaiDoanTest == 0) {
            _boDemXung.reset();
            ++baoCaoDongCo.maResetXung;
            thoiDiemDocXungTruoc_us = 0;
        }

        if (phienCanBang.cheDo() == CheDoCanBang::ChoLenhMoi) {
            if (phienCanBang.quaHanCho(thoiDiemHienTai_us)) {
                baoCaoDongCo.loi = FaultCode::ArmTimeout;
                phienCanBang.tat();
                _capDongCo.disable();
            } else if (!dungCauHinhDeBat || loiAnToan != FaultCode::None ||
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
        if (giaiDoanTest != 0) {
            if ((cacSuKien & (suKienTatDongCo | suKienDungKhan)) ||
                khoaMotorCalib || !coCauHinhHienTai || !cauHinhHienTai.cheDoThuKhongMaSat ||
                baoCaoDongCo.loi != FaultCode::None || phienCanBang.cheDo() != CheDoCanBang::Tat) {
                _capDongCo.disable();
                xuatKetQuaTest(true);
                giaiDoanTest = 0;
            } else if (giaiDoanTest == 1) {
                const uint64_t lucTest_us = nowUs();
                if (lucTest_us < hanTest_us) {
                    WheelStepCommand lenhTest{};
                    lenhTest.tocDoBuocTrai_buocTrenGiay = daoChieuDongCoHienTai ? -testTrai_hz : testTrai_hz;
                    lenhTest.tocDoBuocPhai_buocTrenGiay = daoChieuDongCoHienTai ? -testPhai_hz : testPhai_hz;
                    lenhTest.thoiDiemRaLenh_us = lucTest_us;
                    lenhTest.thoiDiemHetHan_us = lucTest_us + thoiGianHetHanLenhMotor_us;
                    baoCaoDongCo.ketQuaCuoi = _capDongCo.apply(lenhTest);
                    if (baoCaoDongCo.ketQuaCuoi == MksResult::FaultLatched) {
                        if (baoCaoDongCo.loi == FaultCode::None) baoCaoDongCo.loi = FaultCode::MotorFault;
                        _capDongCo.disable();
                        xuatKetQuaTest(true);
                        giaiDoanTest = 0;
                    }
                } else {
                    _capDongCo.disable();
                    hanTest_us = lucTest_us + 30000ULL;
                    giaiDoanTest = 2;
                }
            }
        } else {
            YeuCauTestBanh yeuCauTest{};
            portENTER_CRITICAL(&_khoaTestBanh);
            yeuCauTest = _yeuCauTestBanh;
            portEXIT_CRITICAL(&_khoaTestBanh);
            if (yeuCauTest.maYeuCau != 0 && yeuCauTest.maYeuCau != maTestDangChay) {
                const bool duDieuKien = phienCanBang.cheDo() == CheDoCanBang::Tat && !khoaMotorCalib &&
                    baoCaoDongCo.loi == FaultCode::None && coCauHinhHienTai &&
                    cauHinhHienTai.cheDoThuKhongMaSat && !_capDongCo.isEnabled();
                if (!duDieuKien) {
                    maTestDangChay = yeuCauTest.maYeuCau;
                    stepTraiDau = baoCaoDongCo.khungXung.trai;
                    stepPhaiDau = baoCaoDongCo.khungXung.phai;
                    encTraiDau = baoCaoDongCo.khungXung.encoderTrai;
                    encPhaiDau = baoCaoDongCo.khungXung.encoderPhai;
                    std::memcpy(nhanTest, yeuCauTest.nhan, sizeof(nhanTest));
                    xuatKetQuaTest(true);
                } else {
                    const MksResult ketQuaBat = _capDongCo.enable();
                    baoCaoDongCo.ketQuaCuoi = ketQuaBat;
                    if (ketQuaBat == MksResult::Ok) {
                        maTestDangChay = yeuCauTest.maYeuCau;
                        testTrai_hz = yeuCauTest.tocDoTrai_hz;
                        testPhai_hz = yeuCauTest.tocDoPhai_hz;
                        std::memcpy(nhanTest, yeuCauTest.nhan, sizeof(nhanTest));
                        stepTraiDau = baoCaoDongCo.khungXung.trai;
                        stepPhaiDau = baoCaoDongCo.khungXung.phai;
                        encTraiDau = baoCaoDongCo.khungXung.encoderTrai;
                        encPhaiDau = baoCaoDongCo.khungXung.encoderPhai;
                        const uint64_t lucBatTest_us = nowUs();
                        hanTest_us = lucBatTest_us + static_cast<uint64_t>(yeuCauTest.thoiGian_ms) * 1000ULL;
                        giaiDoanTest = 1;
                        baoCaoDongCo.khoangCachPhucVuLonNhat_us = 0;
                        thoiDiemPhucVuTruoc_us = lucBatTest_us;
                    } else if (ketQuaBat != MksResult::Pending) {
                        maTestDangChay = yeuCauTest.maYeuCau;
                        stepTraiDau = baoCaoDongCo.khungXung.trai;
                        stepPhaiDau = baoCaoDongCo.khungXung.phai;
                        encTraiDau = baoCaoDongCo.khungXung.encoderTrai;
                        encPhaiDau = baoCaoDongCo.khungXung.encoderPhai;
                        std::memcpy(nhanTest, yeuCauTest.nhan, sizeof(nhanTest));
                        xuatKetQuaTest(true);
                    }
                }
            }
        }
        if (_capDongCo.service() == MksResult::FaultLatched) {
            if (baoCaoDongCo.loi == FaultCode::None) baoCaoDongCo.loi = FaultCode::MotorFault;
            if (phienCanBang.cheDo() != CheDoCanBang::Tat) phienCanBang.tat();
        }
        const uint64_t thoiDiemTruocDocXung_us = nowUs();
        if (!baoCaoDongCo.khungXung.hopLe || thoiDiemDocXungTruoc_us == 0 ||
            thoiDiemTruocDocXung_us - thoiDiemDocXungTruoc_us >= chuKyDocPhanHoiBanh_us) {
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
        if (giaiDoanTest == 2 && nowUs() >= hanTest_us) {
            xuatKetQuaTest(false);
            giaiDoanTest = 0;
        }
        if (!baoCaoDongCo.khungXung.hopLe && _capDongCo.isEnabled()) {
            baoCaoDongCo.loi = FaultCode::PulseInvalid;
            phienCanBang.tat();
            _capDongCo.emergencyStop();
        }
        baoCaoDongCo.trangThaiBoDieuKhienDongCo = _capDongCo.snapshot();
        _motorDangBat.store(baoCaoDongCo.trangThaiBoDieuKhienDongCo.daBat ||
                            phienCanBang.cheDo() != CheDoCanBang::Tat);
        baoCaoDongCo.maPhien = phienCanBang.maPhien();
        baoCaoDongCo.cheDo = phienCanBang.cheDo();
        baoCaoDongCo.thoiDiemSnapshot_us = nowUs();
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
    _boDieuKhienTang.setThuNghiemDiChuyen(cauHinh.tangGiaTocDiChuyen, cauHinh.tangBamVanTocDiChuyen);
    _boUocLuongGoc.datTrongTruongThamChieu_mps2(cauHinh.normGiaTocTinh_mps2);
}

void RobotApp::inDongSerial(const char* noiDung) {
    if (!noiDung) return;
    if (_dangPhatNghienCuu) _nhatKyNghienCuu.them(noiDung, std::strlen(noiDung), true);
    else Serial.println(noiDung);
    dieuKhienKhongDay.ghiDongNhatKy(noiDung);
}

void RobotApp::inDinhDangSerial(const char* dinhDang, ...) {
    if (!dinhDang) return;
    char boDemTam[320]{};
    va_list danhSachThamSo;
    va_start(danhSachThamSo, dinhDang);
    std::vsnprintf(boDemTam, sizeof(boDemTam), dinhDang, danhSachThamSo);
    va_end(danhSachThamSo);
    if (_dangPhatNghienCuu) _nhatKyNghienCuu.them(boDemTam, std::strlen(boDemTam));
    else Serial.print(boDemTam);
    dieuKhienKhongDay.ghiDongNhatKy(boDemTam);
}

bool RobotApp::luuCauHinhVaoNvs() {
    MotorStatus baoCaoDongCo{};
    if (_hangDoiTrangThaiDongCo &&
        xQueuePeek(_hangDoiTrangThaiDongCo, &baoCaoDongCo, 0) == pdTRUE &&
        baoCaoDongCo.cheDo != CheDoCanBang::Tat) {
        _canLuuNvs = true;
        return false;
    }
    _canLuuNvs = false;
    Preferences prefs;
    if (!prefs.begin(nvsNamespace, false)) {
        inDongSerial("[NVS] ERROR: Khong mo duoc NVS de luu.");
        return false;
    }
    bool ok = prefs.putUInt("magic", nvsMagic) == sizeof(nvsMagic);
    const auto luuFloat = [&](const char* key, float value) { ok &= prefs.putFloat(key, value) == sizeof(value); };
    luuFloat("kp_p", _cauHinhChay.kpGocThuan);
    luuFloat("ki_p", _cauHinhChay.kiGocThuan);
    luuFloat("kd_p", _cauHinhChay.kdGocThuan);
    luuFloat("kpv_p", _cauHinhChay.kpVanTocThuan);
    luuFloat("kiv_p", _cauHinhChay.kiVanTocThuan);
    luuFloat("kp_m", _cauHinhChay.kpGocMo);
    luuFloat("ki_m", _cauHinhChay.kiGocMo);
    luuFloat("kd_m", _cauHinhChay.kdGocMo);
    luuFloat("kpv_m", _cauHinhChay.kpVanTocMo);
    luuFloat("kiv_m", _cauHinhChay.kiVanTocMo);
    if (dungLsm6dsv) {
        if (_coMaTranLapLsm)
            ok &= mountLsm::luu(prefs, _camBienGoc.maNhanDang(), _cauHinhChay.maTranLap, _cauHinhChay.gocCanBang_rad);
    } else {
        luuFloat("ref_rad", _cauHinhChay.gocCanBang_rad);
    }
    ok &= prefs.putBool("mode_mo", _cauHinhChay.dungPidMo) == 1;
    ok &= prefs.putBool("inv_dir", _cauHinhChay.daoChieuDongCo) == 1;
    ok &= prefs.putBool("gb_ok", _coBiasLuu) == 1;
    if (_coBiasLuu) {
        luuFloat("gb_x", _biasX_dps);
        luuFloat("gb_y", _biasY_dps);
        luuFloat("gb_z", _biasZ_dps);
    }
    prefs.end();
    if (!ok) inDongSerial("[NVS] ERROR: ghi cau hinh chua day du.");
    return ok;
}

void RobotApp::inTroGiup() {
    inDinhDangSerial("FIRMWARE %s\n", phienBanFirmware);
    inDongSerial("Dieu khien: a=arm; d=disable; x=emergency; k=clear fault; p=PID_THUAN; m=PID_MO; f=dao mode");
    inDongSerial("Di chuyen (khi arm): w/s=tien/lui; j/l=trai/phai; z=dung; g=chot giu vi tri tai cho");
    inDongSerial("Calib & Bench: c=calib khoa hoc 2s (gyro+pitch+NVS); o=zero nhanh pitch+odo; [/]=trim -/+0.1deg");
    if (dungLsm6dsv) inDongSerial("Calib dung LSM: cal1 +x/-x/+y/-y/+z/-z (truc sensor chi ve DAU xe), giu dung yen.");
    if (dungProfileImuRieng)
        inDongSerial("Calib 3 tu the (Web/S3/BOOT): cal3 start | cal3 next | cal3 cancel");
    inDongSerial("Test banh cham (bench 1, nhac banh): mtest 0 | l+ | l- | r+ | r- | + | -");
    inDongSerial("               b=bat/tat Bench Mode (nhac banh khong ma sat); v=bat/tat vong van toc/vi tri");
    inDongSerial("Chinh PID (kem Enter): kp X; ki X; kd X; kpv X; kiv X; kpv_p X; kpv_m X; ref X(rad); refdeg X(deg); bench 0|1; vloop 0|1; daochieu [0|1]");
    inDongSerial("Lenh khac: pid|fuzzy|mode pid|fuzzy; giuvitri; save=luu NVS; resetcfg=xoa NVS; status=xem cau hinh; t=stream; tw=web stream; i=RAW_IMU");
    inDongSerial("research 1 = bat CSV NC 50 Hz; research 0 = tat CSV. Thoi gian nguon IMU tinh bang micro giay.");
    inDongSerial("radio peer AA:BB:CC:DD:EE:FF = ghep cau noi ESP-NOW (motor tat); radio status = xem peer.");
    inDongSerial("A/B RAM (motor tat): imucomp 0|1; accelboost 0|1; vgainboost 0|1. Khoi dong lai ve mac dinh; save khong luu cac co nay.");
    if (dungLsm6dsv) inDongSerial("Log rung ESP-NOW: viblog 1=bat, viblog 0=tat. Thu dong, theo nhip IMU.");
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
    inDinhDangSerial("IMU_PROFILE=%s bus=%s busHz=%lu sampleHz=%lu NVS=%s mountReady=%d WHO=0x%02X\n",
        tenImu, giaoTiepImu, static_cast<unsigned long>(tanSoBusImu_hz),
        static_cast<unsigned long>(tanSoImuHienTai_hz()), nvsNamespace, dungProfileImuRieng ? _coMaTranLapLsm : 1,
        static_cast<unsigned>(_camBienGoc.maNhanDang()));
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
    inDinhDangSerial("LSM6DSV_CFG LPF1_G=%u(fcD=%.1fHz) LPF2_XL=%u(en=%d) gRef=%.4f\n",
        static_cast<unsigned>(lsm6dsv::locGyroLpf1), static_cast<double>(tanSoLocGyroD_hz),
        static_cast<unsigned>(lsm6dsv::locGiaTocLpf2), lsm6dsv::batLocGiaTocLpf2 ? 1 : 0,
        static_cast<double>(_cauHinhChay.normGiaTocTinh_mps2));
#elif CAU_HINH_IMU == IMU_MPU6050_I2C
    inDinhDangSerial("MPU6050_CFG AD0=IO%d(0x%02X) SDA=IO%d SCL=IO%d INT=NONE(poll) DLPF=%u Notch=%.1fHz(Q=%.1f) Butterworth2=%.1fHz | SISO_Yaw(Kp=%.2f,Kd=%.2f,tau=%.3fs)\n",
        mpu6050::chanAd0, static_cast<unsigned>(mpu6050::diaChiI2c), mpu6050::chanSda, mpu6050::chanScl,
        static_cast<unsigned>(mpu6050::maDlpf),
        static_cast<double>(mpu6050Tune::gyroNotch_hz), static_cast<double>(mpu6050Tune::gyroNotch_q),
        static_cast<double>(mpu6050Tune::gyroButterworth_hz),
        static_cast<double>(mpu6050Tune::kpSisoHuong), static_cast<double>(mpu6050Tune::kdSisoHuong),
        static_cast<double>(mpu6050Tune::tauLocSisoHuong_s));
#endif
    inDinhDangSerial("AB_RAM imucomp=%d accelboost=%d vgainboost=%d\n", _cauHinhChay.buGiaTocImu,
        _cauHinhChay.tangGiaTocDiChuyen, _cauHinhChay.tangBamVanTocDiChuyen);
    inDinhDangSerial("Mode=%s EN=%d fault=%s bench=%d vloop=%d invDir=%d telemetry=%d tw=%d research=%d\n",
                  _cauHinhChay.dungPidMo ? "PID_MO" : "PID_THUAN",
                  baoCaoDongCo.trangThaiBoDieuKhienDongCo.daBat,
                  faultName(baoCaoDongCo.loi),
                  _cauHinhChay.cheDoThuKhongMaSat ? 1 : 0,
                  _cauHinhChay.batVongNgoai ? 1 : 0,
                  _cauHinhChay.daoChieuDongCo ? 1 : 0,
                  _batTelemetry ? 1 : 0,
                  batTelemetryWeb ? 1 : 0, _batNghienCuu ? 1 : 0);
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
    inDinhDangSerial("MKS_UART (vFOC MStep=%lu, stepMaxHz=%.0f, 65536/rev, baud=%lu, vFOC_fusion=%d): L(ok=%d addr=0x%02X rx=%lu encStep=%lld) R(ok=%d addr=0x%02X rx=%lu encStep=%lld)\n",
                  static_cast<unsigned long>(soViBuocDauVaoMks),
                  static_cast<double>(tanSoStepToiDaMks_hz),
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
    inDinhDangSerial("TIMING fw=%s cpuMHz=%lu mcpwmHz=%lu imuHz=%lu motorServiceUs=%lu feedbackUs=%lu uartWheelUs=%lu\n",
                  phienBanFirmware,
                  static_cast<unsigned long>(getCpuFrequencyMhz()),
                  static_cast<unsigned long>(doPhanGiaiBoDemMcpwm_hz),
                  static_cast<unsigned long>(tanSoImuHienTai_hz()),
                  static_cast<unsigned long>(chuKyPhucVuMotor_us),
                  static_cast<unsigned long>(chuKyDocPhanHoiBanh_us),
                  static_cast<unsigned long>(chuKyDocUartMks_us));
    if (!dungPhanCungTmcMt) {
        // Tuổi tại chính snapshot motor, không cộng thời gian in các dòng Serial phía trên.
        const uint64_t lucXem_us = baoCaoDongCo.thoiDiemSnapshot_us;
        const auto tuoiNhan_us = [lucXem_us](uint64_t moc_us) -> long long {
            return moc_us && moc_us <= lucXem_us ? static_cast<long long>(lucXem_us - moc_us) : -1;
        };
        inDinhDangSerial("MKS_UART_DIAG snapshotUs=%llu L(query=%lu rxAgeSnapshotUs=%lld) R(query=%lu rxAgeSnapshotUs=%lld) skippedSlots=%llu txErrors=%lu\n",
            static_cast<unsigned long long>(lucXem_us),
            static_cast<unsigned long>(baoCaoDongCo.khungXung.soLanHoiUartTrai),
            tuoiNhan_us(baoCaoDongCo.khungXung.thoiDiemNhanUartTrai_us),
            static_cast<unsigned long>(baoCaoDongCo.khungXung.soLanHoiUartPhai),
            tuoiNhan_us(baoCaoDongCo.khungXung.thoiDiemNhanUartPhai_us),
            static_cast<unsigned long long>(baoCaoDongCo.khungXung.soKheUartBoLo),
            static_cast<unsigned long>(baoCaoDongCo.khungXung.soLoiGhiUart));
    }
    inDinhDangSerial("WiFi_AP=%s sta=%u txPwr=%s URL=http://%s WS=ws://%s:%u UDP=%u rx=%lu | ESP_NOW=%s MAC=%s rxGoi=%lu\n",
                  dieuKhienKhongDay.wifiSanSang() ? tenMangWifiAp : "OFF",
                  static_cast<unsigned>(dieuKhienKhongDay.soKhachWifiAp()),
                  dieuKhienKhongDay.mucCongSuatWifi(),
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
        if (_cauHinhChay.khoaMotorCalib || trangThaiDongCo.cheDo != CheDoCanBang::Tat) {
            inDongSerial("[CAL] Hay tat motor ('d') truoc khi zero!");
            break;
        }
        ControlStatus baoCaoDieuKhien{};
        if (xQueuePeek(_hangDoiTrangThaiDieuKhien, &baoCaoDieuKhien, 0) == pdTRUE &&
            baoCaoDieuKhien.phienBanCauHinh == _cauHinhChay.phienBan &&
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
        const auto& f = lsm6dsvTune::huongTienSensor;
        const Vec3Imu tien{f[0], f[1], f[2]};
        batDauCalibDung(dungLsm6dsv && !_coMaTranLapLsm ? &tien : nullptr);
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
    if (dungLsm6dsv && std::strncmp(banSao, "cal1", 4) == 0) {
        const char dau = len == 7 ? banSao[5] : 0;
        const char truc = len == 7 ? std::tolower(static_cast<unsigned char>(banSao[6])) : 0;
        if (len != 7 || banSao[4] != ' ' || (dau != '+' && dau != '-') ||
            (truc != 'x' && truc != 'y' && truc != 'z')) {
            inDongSerial("[CAL1] Chon truc sensor chi ve DAU xe: cal1 +x/-x/+y/-y/+z/-z.");
            return;
        }
        const float heSo = dau == '+' ? 1.f : -1.f;
        const Vec3Imu tien{truc == 'x' ? heSo : 0.f, truc == 'y' ? heSo : 0.f, truc == 'z' ? heSo : 0.f};
        batDauCalibDung(&tien);
        return;
    }
    if (std::strcmp(banSao, "viblog 1") == 0 || std::strcmp(banSao, "viblog 0") == 0) {
        if (!dungLsm6dsv) { inDongSerial("VIBLOG ERROR: can profile LSM6DSV."); return; }
        if (!logRung.setEnabled(banSao[7] == '1'))
            inDongSerial("VIBLOG ERROR: bo ghi chua san sang.");
        else inDinhDangSerial("VIBLOG enabled=%d id=%lu; receiver=%d\n", logRung.enabled(),
            static_cast<unsigned long>(logRung.logId()), lienKetChanDoan.logReceiverReady(nowUs()));
        capNhatThongTinLogRung();
        return;
    }
    const bool lenhBuImu = std::strcmp(banSao,"imucomp 0")==0 || std::strcmp(banSao,"imucomp 1")==0;
    const bool lenhTangGiaToc = std::strcmp(banSao,"accelboost 0")==0 || std::strcmp(banSao,"accelboost 1")==0;
    const bool lenhTangBam = std::strcmp(banSao,"vgainboost 0")==0 || std::strcmp(banSao,"vgainboost 1")==0;
    if (lenhBuImu || lenhTangGiaToc || lenhTangBam) {
        if (trangThaiDongCo.cheDo != CheDoCanBang::Tat) {
            inDongSerial("AB ERROR: tat motor truoc khi thay doi"); return;
        }
        const bool bat = banSao[len-1]=='1';
        if (lenhBuImu) _cauHinhChay.buGiaTocImu=bat;
        if (lenhTangGiaToc) _cauHinhChay.tangGiaTocDiChuyen=bat;
        if (lenhTangBam) _cauHinhChay.tangBamVanTocDiChuyen=bat;
        ++_cauHinhChay.phienBan;
        xQueueOverwrite(_hangDoiCauHinh,&_cauHinhChay);
        inDinhDangSerial("AB_RAM imucomp=%d accelboost=%d vgainboost=%d [not saved]\n", _cauHinhChay.buGiaTocImu,
            _cauHinhChay.tangGiaTocDiChuyen,_cauHinhChay.tangBamVanTocDiChuyen);
        return;
    }
    if (std::strcmp(banSao, "radio status") == 0) {
        inDinhDangSerial("RADIO peer=%s enabled=%d\n", lienKetChanDoan.peerText(), lienKetChanDoan.enabled());
        return;
    }
    if (std::strncmp(banSao, "radio peer ", 11) == 0) {
        unsigned b[6]; int n = 0;
        if (trangThaiDongCo.cheDo != CheDoCanBang::Tat ||
            std::sscanf(banSao + 11, "%2x:%2x:%2x:%2x:%2x:%2x%n", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5], &n) != 6 || banSao[11+n]) {
            inDongSerial("RADIO ERROR: motor phai tat, cu phap radio peer AA:BB:CC:DD:EE:FF"); return;
        }
        uint8_t mac[6]; for (unsigned i = 0; i < 6; ++i) mac[i] = static_cast<uint8_t>(b[i]);
        inDongSerial(lienKetChanDoan.setPeer(mac) ? "RADIO peer saved; restart bridge connect" : "RADIO ERROR: cannot save peer");
        return;
    }
    if (std::strcmp(banSao, "research 1") == 0) {
        _batNghienCuu = _dangPhatNghienCuu = true;
        _canTieuDeNghienCuu = _canCauHinhNghienCuu = true;
        _soMauNghienCuuBo = _thuTuDaGhi = 0;
        _henMauNghienCuu_us = nowUs();
#if ARDUINO_USB_CDC_ON_BOOT
        Serial.setTxTimeoutMs(0);
#endif
        return;
    }
    if (std::strcmp(banSao, "research 0") == 0) {
        _batNghienCuu = false;
        return;
    }
    if (std::strcmp(banSao, "arm") == 0) { xuLyPhimDonSerial('a'); return; }
    if (std::strcmp(banSao, "disarm") == 0 || std::strcmp(banSao, "stop") == 0) { xuLyPhimDonSerial('d'); return; }
    if (std::strcmp(banSao, "clear") == 0) { xuLyPhimDonSerial('k'); return; }
    if (std::strcmp(banSao, "calib") == 0) { xuLyPhimDonSerial('c'); return; }
    if (std::strcmp(banSao, "cal3 start") == 0 || std::strcmp(banSao, "calib3") == 0 ||
        std::strcmp(banSao, "cal2 start") == 0 || std::strcmp(banSao, "calib2truc") == 0) {
        batDauCalib2Truc(false);
        return;
    }
    if (std::strcmp(banSao, "cal3 next") == 0 || std::strcmp(banSao, "cal2 next") == 0 ||
        std::strcmp(banSao, "calnext") == 0) {
        xacNhanBuocCalib2Truc();
        return;
    }
    if (std::strcmp(banSao, "cal3 cancel") == 0 || std::strcmp(banSao, "cal2 cancel") == 0) {
        huyCalib2Truc();
        return;
    }
    if (std::strncmp(banSao, "mtest ", 6) == 0 || std::strcmp(banSao, "mtest") == 0) {
        if (trangThaiDongCo.cheDo != CheDoCanBang::Tat || trangThaiDongCo.trangThaiBoDieuKhienDongCo.daBat) {
            inDongSerial("[MTEST] Hay tat motor ('d') truoc khi chay mtest.");
            return;
        }
        if (_cauHinhChay.khoaMotorCalib || trangThaiDongCo.loi != FaultCode::None ||
            !_cauHinhChay.cheDoThuKhongMaSat || _dangChoTestBanh) {
            inDongSerial("[MTEST] Can bench=1 (nhac banh), fault=none (gui 'k' neu can), khong khoa calib va khong dang chay test.");
            return;
        }
        const char* thamSoTest = (len > 6) ? (banSao + 6) : "";
        while (*thamSoTest == ' ' || *thamSoTest == '\t') ++thamSoTest;
        char khoaTest[8]{};
        for (size_t i = 0; i < sizeof(khoaTest) - 1u && thamSoTest[i] != '\0'; ++i) {
            khoaTest[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(thamSoTest[i])));
        }
        constexpr float tocDoTest_hz =
            0.25f * static_cast<float>(soBuocDayDuMoiVongDongCo) * static_cast<float>(soViBuocDauVaoDriver);
        struct MucTest { const char* ten; float trai, phai; };
        const MucTest cacMucTest[] = {
            {"0",  0.0f,         0.0f},
            {"l+", tocDoTest_hz, 0.0f},
            {"l-", -tocDoTest_hz, 0.0f},
            {"r+", 0.0f,         tocDoTest_hz},
            {"r-", 0.0f,         -tocDoTest_hz},
            {"+",  tocDoTest_hz, tocDoTest_hz},
            {"-",  -tocDoTest_hz, -tocDoTest_hz},
        };
        const MucTest* mucChon = nullptr;
        for (const auto& muc : cacMucTest) {
            if (std::strcmp(khoaTest, muc.ten) == 0) { mucChon = &muc; break; }
        }
        if (!mucChon) {
            inDongSerial("[MTEST] Cu phap: mtest <0|l+|l-|r+|r-|+|->");
            return;
        }
        YeuCauTestBanh yeuCau{};
        portENTER_CRITICAL(&_khoaTestBanh);
        yeuCau.maYeuCau = _yeuCauTestBanh.maYeuCau + 1u;
        yeuCau.tocDoTrai_hz = mucChon->trai;
        yeuCau.tocDoPhai_hz = mucChon->phai;
        yeuCau.thoiGian_ms = 1000u;
        std::snprintf(yeuCau.nhan, sizeof(yeuCau.nhan), "%s", mucChon->ten);
        _yeuCauTestBanh = yeuCau;
        portEXIT_CRITICAL(&_khoaTestBanh);
        _dangChoTestBanh = true;
        inDinhDangSerial("[MTEST] Bat dau '%s' (1.0s @ %.0f Hz = 0.25 vong; gui 'd'/'x' de dung ngay)...\n",
            mucChon->ten, static_cast<double>(tocDoTest_hz));
        return;
    }
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
        const bool daLuu = luuCauHinhVaoNvs();
        if (_canLuuNvs) {
            inDongSerial("[NVS] Dang can bang: da hen luu NVS va se ghi Flash ngay khi tat motor.");
        } else if (daLuu) {
            inDongSerial("[NVS] Da luu PID_THUAN, PID_MO, eqPitch, mode va IMU bias vao Flash.");
        }
        return;
    }
    if (std::strcmp(banSao, "resetcfg") == 0) {
        if (_dangChoCalib) {
            inDongSerial("[NVS] Dang do bias/zero. Cho ket qua truoc khi resetcfg.");
            return;
        }
        if (dungProfileImuRieng && trangThaiDongCo.cheDo != CheDoCanBang::Tat) {
            inDongSerial("[NVS] Hay tat motor ('d') truoc khi resetcfg.");
            return;
        }
        _canLuuNvs = false;
        Preferences prefs;
        if (!prefs.begin(nvsNamespace, false) || !prefs.clear()) {
            inDongSerial("[NVS] ERROR: khong xoa duoc cau hinh.");
            return;
        }
        prefs.end();
        portENTER_CRITICAL(&_khoaCalib2Truc);
        ++_calib2Truc.phienYeuCau;
        _calib2Truc.dangYeuCauCalib = false;
        _calib2Truc.coKetQuaMoi = false;
        portEXIT_CRITICAL(&_khoaCalib2Truc);
        const bool daXacNhanLapCode = dungLsm6dsv ? lsm6dsvTune::daXacNhanMaTranLap
                                                  : (dungMpu6050 ? mpu6050Tune::daXacNhanMaTranLap : false);
        _heTrucImu = HeTrucImu();
        _coMaTranLapLsm = dungProfileImuRieng ? (daXacNhanLapCode && _heTrucImu.hopLe()) : true;
        const uint32_t phienBanMoi = _cauHinhChay.phienBan + 1u;
        const uint32_t phienBanMaTranMoi = _cauHinhChay.phienBanMaTranLap + 1u;
        _cauHinhChay = CauHinhChay{};
        _cauHinhChay.phienBan = phienBanMoi;
        _cauHinhChay.phienBanMaTranLap = phienBanMaTranMoi;
        _cauHinhChay.maTranLap = _heTrucImu.maTran();
        _cauHinhChay.coBiasGyro = _coBiasLuu;
        _cauHinhChay.biasGyro_dps = {_biasX_dps, _biasY_dps, _biasZ_dps};
        _cauHinhChay.khoaMotorCalib = dungProfileImuRieng && !(_coMaTranLapLsm && _coBiasLuu);
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
    int txDbm = 0;
    if (std::sscanf(banSao, "wifipwr %d", &txDbm) == 1) {
        if (dieuKhienKhongDay.datCongSuatWifi(txDbm)) {
            inDinhDangSerial("[WIFI] txPwr=%s\n", dieuKhienKhongDay.mucCongSuatWifi());
        } else {
            inDongSerial("[WIFI] Dung: wifipwr <8|11|13|15|19>");
        }
        return;
    }

    inDinhDangSerial("[CMD] Khong hieu: '%s' (go 'help' hoac 'status')\n", banSao);
}

void RobotApp::nhanLenhDiChuyenTuXa(float vanTocTienLui, float tocDoQuayNgang, const MucLenhWeb& mucWeb) {
    MotorStatus trangThaiDongCo{};
    xQueuePeek(_hangDoiTrangThaiDongCo, &trangThaiDongCo, 0);
    if (!dungLsm6dsv && trangThaiDongCo.cheDo != CheDoCanBang::DangChay) return;
    LenhDiChuyen lenhMoi{};
    lenhMoi.maPhien = trangThaiDongCo.maPhien;
    lenhMoi.thoiDiem_us = nowUs();
    lenhMoi.hetHan_us = lenhMoi.thoiDiem_us + hanLenhDiChuyen_us;
    lenhMoi.vanToc_mps = std::clamp(vanTocTienLui, -vanTocDiChuyenToiDa_mps, vanTocDiChuyenToiDa_mps);
    lenhMoi.tocDoQuay_radps = std::clamp(tocDoQuayNgang, -tocDoQuayToiDa_radps, tocDoQuayToiDa_radps);
    lenhMoi.mucWeb = mucWeb;
    xQueueOverwrite(_hangDoiDiChuyen, &lenhMoi);
}

void RobotApp::dienTrangThaiWeb(ThongTinTrangThaiWeb& trangThaiRa) {
    trangThaiRa.logRungSupported = dungLsm6dsv;
    trangThaiRa.logRungEnabled = logRung.enabled();
    trangThaiRa.logRungReceiver = lienKetChanDoan.logReceiverReady(nowUs());
    trangThaiRa.logRungId = logRung.logId();
    trangThaiRa.logRungQueueDrop = logRung.queueDrop();
    trangThaiRa.logRungTxDrop = logRung.txDrop();
    trangThaiRa.logRungProduced = logRung.produced();
    trangThaiRa.logRungHz = static_cast<uint16_t>(tanSoImuHienTai_hz());
    trangThaiRa.hoTroCalib2Truc = dungProfileImuRieng;
    trangThaiRa.canCalibLap = dungProfileImuRieng && !_coMaTranLapLsm;
    portENTER_CRITICAL(&_khoaCalib2Truc);
    trangThaiRa.phaCalib2Truc = static_cast<uint8_t>(_calib2Truc.pha);
    trangThaiRa.mauCalib2Truc = _calib2Truc.soMauHienTai;
    trangThaiRa.tongMauCalib2Truc = _calib2Truc.tongMauCan;
    std::snprintf(trangThaiRa.loiCalib2Truc, sizeof(trangThaiRa.loiCalib2Truc), "%s", _calib2Truc.lyDoLoi);
    portEXIT_CRITICAL(&_khoaCalib2Truc);
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
    trangThaiRa.heThongSanSang = !_cauHinhChay.khoaMotorCalib &&
        baoCaoDieuKhien.phienBanCauHinh == _cauHinhChay.phienBan &&
        (loiHienTai == FaultCode::None && baoCaoDongCo.loi == FaultCode::None);
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
    const char* tenSanSangChuoi = _cauHinhChay.khoaMotorCalib ? "need_mount_calib" : faultName(loiHienTai);
    std::snprintf(trangThaiRa.tenSanSang, sizeof(trangThaiRa.tenSanSang), "%s", tenSanSangChuoi);
    std::snprintf(trangThaiRa.cheDoDieuKhien, sizeof(trangThaiRa.cheDoDieuKhien), "%s",
                  dangMo ? "PID_MO" : "PID_THUAN");
}

void RobotApp::guiChanDoanEspNow() {
    if (!lienKetChanDoan.enabled()) return;
    static uint64_t lanGui_us = 0;
    const uint64_t lucHen = nowUs();
    if (lucHen - lanGui_us < 20000) return;
    lanGui_us = lucHen;
    ControlStatus c{}; MotorStatus m{};
    if (xQueuePeek(_hangDoiTrangThaiDieuKhien, &c, 0) != pdTRUE ||
        xQueuePeek(_hangDoiTrangThaiDongCo, &m, 0) != pdTRUE) return;
    const uint64_t now = nowUs(); // Sau khi đọc queue để snapshot mới không có timestamp "tương lai" giả.
    ChanDoanEspNow::GoiTelemetry t{};
    t.t_us = c.khungCamBien.thoiDiemLayMau_us; t.motor_us = m.thoiDiemSnapshot_us;
    t.sample_seq = c.khungCamBien.soMauLayDuoc; t.control_seq = c.soThuTuLenh; t.cfg = c.phienBanCauHinh;
    t.control_session = c.maPhien; t.motor_session = m.maPhien;
    t.en = m.trangThaiBoDieuKhienDongCo.daBat; t.mode = c.dungPidMo;
    t.control_fault = static_cast<uint8_t>(c.loi); t.motor_fault = static_cast<uint8_t>(m.loi);
    t.control_phase = static_cast<uint8_t>(c.phaDieuKhien); t.motor_phase = static_cast<uint8_t>(m.cheDo);
    t.angle_valid = c.trangThaiXe.gocHopLe && c.trangThaiXe.thoiDiemGoc_us &&
        now >= c.trangThaiXe.thoiDiemGoc_us && now-c.trangThaiXe.thoiDiemGoc_us < tuoiImuToiDa_us;
    t.feedback = static_cast<uint8_t>(c.trangThaiXe.nguonQuangDuong);
    t.diag_flags = c.coThuNghiem;
    t.theta = c.trangThaiXe.gocNghien_rad; t.rate = c.trangThaiXe.tocDoNghien_radps;
    t.target = c.gocDat_rad; t.eq = c.gocCanBang_rad; t.v = c.trangThaiXe.vanTocDoc_mps;
    t.vref = c.vanTocMucTieu_mps; t.yawref = c.tocDoQuayMucTieu_radps;
    t.cmd_l = c.lenhBanh.tocDoBuocTrai_buocTrenGiay; t.cmd_r = c.lenhBanh.tocDoBuocPhai_buocTrenGiay;
    t.a_cmd = c.giaTocLenh_mps2; t.gyro_d = c.gyroDaLoc_radps;
    t.acc_norm = c.trangThaiXe.doLonGiaToc_mps2; t.innov = c.trangThaiXe.saiLechGiaToc_rad;
    t.trust = c.trangThaiXe.doTinCayGiaToc;
    t.kp = c.heSoPid.heSoTiLe; t.ki = c.heSoPid.heSoTichPhan; t.kd = c.heSoPid.heSoViPhan;
    t.steps_l = m.khungXung.trai; t.steps_r = m.khungXung.phai;
    t.enc_l = m.khungXung.encoderTrai; t.enc_r = m.khungXung.encoderPhai;
    const auto age = [now](uint64_t at) -> uint32_t {
        if (!at || at > now || now-at >= UINT32_MAX) return UINT32_MAX;
        return static_cast<uint32_t>(now-at);
    };
    t.uart_age_l_us = age(m.khungXung.thoiDiemNhanUartTrai_us);
    t.uart_age_r_us = age(m.khungXung.thoiDiemNhanUartPhai_us);
    t.uart_rx_l = m.khungXung.soKhungUartTrai; t.uart_rx_r = m.khungXung.soKhungUartPhai;
    t.imu_miss = static_cast<uint32_t>(std::min<uint64_t>(c.soMauBoLo, UINT32_MAX));
    t.imu_i2c = c.khungCamBien.chanDoan.soLoiI2c; t.imu_late = c.khungCamBien.chanDoan.soLanDocQuaChuKy;
    lienKetChanDoan.sendTelemetry(t);
}

void RobotApp::capNhatThongTinLogRung() {
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
    if (!dungLsm6dsv) return;
    static uint64_t previous = 0;
    static uint32_t previousCfg = 0, previousId = UINT32_MAX;
    static bool previousEnabled = false;
    const uint64_t now = nowUs();
    if (now - previous < 100000u && previousCfg == _cauHinhChay.phienBan &&
        previousId == logRung.logId() && previousEnabled == logRung.enabled()) return;
    previous = now; previousCfg = _cauHinhChay.phienBan;
    previousId = logRung.logId(); previousEnabled = logRung.enabled();
    ControlStatus c{};
    xQueuePeek(_hangDoiTrangThaiDieuKhien, &c, 0);
    KhaoSatRung::Info info{};
    info.t_us = now; info.log_id = logRung.logId(); info.enabled = logRung.enabled();
    info.cfg = _cauHinhChay.phienBan;
    info.sample_hz = tanSoImuHienTai_hz(); info.spi_hz = lsm6dsv::tanSoSpi_hz;
    info.sck = lsm6dsv::chanSck; info.mosi = lsm6dsv::chanMosi;
    info.miso = lsm6dsv::chanMiso; info.cs = lsm6dsv::chanCs; info.interrupt = lsm6dsv::chanInt;
    info.gyro_lpf = lsm6dsv::locGyroLpf1; info.accel_lpf = lsm6dsv::locGiaTocLpf2;
    info.filter_flags = (lsm6dsv::batLocGyroLpf1 ? 1u : 0u) | (lsm6dsv::batLocGiaTocLpf2 ? 2u : 0u);
    info.accel_range_g = lsm6dsv::phamViGiaToc_g; info.gyro_range_dps = lsm6dsv::phamViGyro_dps;
    info.eq = _cauHinhChay.gocCanBang_rad;
    info.kp = _cauHinhChay.dungPidMo ? _cauHinhChay.kpGocMo : _cauHinhChay.kpGocThuan;
    info.ki = _cauHinhChay.dungPidMo ? _cauHinhChay.kiGocMo : _cauHinhChay.kiGocThuan;
    info.kd = _cauHinhChay.dungPidMo ? _cauHinhChay.kdGocMo : _cauHinhChay.kdGocThuan;
    info.kpv = _cauHinhChay.dungPidMo ? _cauHinhChay.kpVanTocMo : _cauHinhChay.kpVanTocThuan;
    info.kiv = _cauHinhChay.dungPidMo ? _cauHinhChay.kiVanTocMo : _cauHinhChay.kiVanTocThuan;
    info.fusion_tau = hangSoThoiGianLocGoc_s; info.d_lpf_hz = tanSoLocGyroD_hz;
    info.gyro_bias[0] = _biasX_dps; info.gyro_bias[1] = _biasY_dps; info.gyro_bias[2] = _biasZ_dps;
    info.trust_norm = saiLechGiaTocChoPhep_mps2;
    info.trust_start = saiLechGocBatDauGiamTin_rad; info.trust_end = saiLechGocMatTin_rad;
    info.trust_timeout = thoiGianMatTinGiaTocToiDa_s;
    const auto& mount = _heTrucImu.maTran();
    std::memcpy(info.mount, mount.phanTu, sizeof(info.mount));
    info.imu_bus_errors = c.khungCamBien.chanDoan.soLoiI2c;
    info.imu_missed = static_cast<uint32_t>(c.soMauBoLo);
    info.imu_late = c.khungCamBien.chanDoan.soLanDocQuaChuKy;
    std::snprintf(info.firmware, sizeof(info.firmware), "%s", phienBanFirmware);
    logRung.updateInfo(info);
#endif
}

void RobotApp::ghiMauNghienCuu() {
    _nhatKyNghienCuu.phat(Serial);
    if (!_batNghienCuu) {
        if (!_nhatKyNghienCuu.conDuLieu()) {
            _dangPhatNghienCuu = false;
#if ARDUINO_USB_CDC_ON_BOOT
            // Giữ TX timeout ngắn (10 ms) để Serial không chặn loop/Wi-Fi khi rút cáp USB.
            Serial.setTxTimeoutMs(10);
#endif
        }
        return;
    }
    constexpr uint64_t chuKy_us = 20000; // 50 Hz, giữ lưới thời gian khi loop bị trễ.
    const uint64_t lucHen_us = nowUs();
    if (lucHen_us < _henMauNghienCuu_us) return;
    const uint64_t soKyTre = (lucHen_us - _henMauNghienCuu_us) / chuKy_us;
    _soMauNghienCuuBo += static_cast<uint32_t>(soKyTre);
    _henMauNghienCuu_us += (soKyTre + 1u) * chuKy_us;
    if (_nhatKyNghienCuu.conDuLieu()) {
        ++_soMauNghienCuuBo;
        return;
    }
    if (_canTieuDeNghienCuu) {
        static constexpr char tieuDe[] =
            "NC_HEADER,t_us,seq,cseq,cfg,csess,cphase,mode,cfault,valid,m_us,msess,mphase,en,mfault,"
            "theta,rate,target,eq,e,v,vref,yawref,cmdL,cmdR,kp,ki,kd,miss,age_us,drop,i2c,late,a_cmd,feedback\n";
        _nhatKyNghienCuu.them(tieuDe, sizeof(tieuDe) - 1);
        _canTieuDeNghienCuu = false;
    }
    ControlStatus c{};
    MotorStatus m{};
    if (xQueuePeek(_hangDoiTrangThaiDieuKhien, &c, 0) != pdTRUE ||
        xQueuePeek(_hangDoiTrangThaiDongCo, &m, 0) != pdTRUE || c.soThuTuLenh == _thuTuDaGhi) {
        ++_soMauNghienCuuBo;
        return;
    }

    // Mọi hệ số/đích thuộc đúng mẫu điều khiển. Cột m_* là snapshot motor riêng.
    char dong[448];
    const auto ghiDong = [&](int doDai) {
        return doDai > 0 && static_cast<size_t>(doDai) < sizeof(dong) &&
               _nhatKyNghienCuu.them(dong, static_cast<size_t>(doDai));
    };
    if (_canCauHinhNghienCuu || c.phienBanCauHinh != _phienBanDaGhi || c.dungPidMo != _cheDoMoDaGhi) {
        const int doDai = std::snprintf(dong, sizeof(dong), "NC_CFG,%lu,%d,%.7g,%.7g,%.7g,%.7g,%.7g,%d,%d\n",
            static_cast<unsigned long>(c.phienBanCauHinh), c.dungPidMo,
            c.heSoGocCoSo.heSoTiLe, c.heSoGocCoSo.heSoTichPhan, c.heSoGocCoSo.heSoViPhan,
            c.kpVanTocCoSo, c.kiVanTocCoSo, c.bench, c.vongNgoai);
        if (!ghiDong(doDai)) { ++_soMauNghienCuuBo; return; }
        _canCauHinhNghienCuu = false;
        _phienBanDaGhi = c.phienBanCauHinh;
        _cheDoMoDaGhi = c.dungPidMo;
    }
    const auto& xe = c.trangThaiXe;
    const auto& imu = c.khungCamBien;
    const uint64_t lucLay_us = nowUs();
    const long long tuoi_us = xe.thoiDiemGoc_us && lucLay_us >= xe.thoiDiemGoc_us
        ? static_cast<long long>(lucLay_us - xe.thoiDiemGoc_us) : -1;
    const int doDai = std::snprintf(dong, sizeof(dong),
        "NC,%llu,%lu,%lu,%lu,%lu,%u,%d,%u,%d,%llu,%lu,%u,%d,%u,"
        "%.7g,%.7g,%.7g,%.7g,%.7g,%.6g,%.6g,%.6g,%.7g,%.7g,%.6g,%.6g,%.6g,%llu,%lld,%lu,%lu,%lu,%.7g,%u\n",
        static_cast<unsigned long long>(imu.thoiDiemLayMau_us), static_cast<unsigned long>(imu.soMauLayDuoc),
        static_cast<unsigned long>(c.soThuTuLenh), static_cast<unsigned long>(c.phienBanCauHinh),
        static_cast<unsigned long>(c.maPhien), static_cast<unsigned>(c.phaDieuKhien), c.dungPidMo,
        static_cast<unsigned>(c.loi), xe.gocHopLe && tuoi_us >= 0 && tuoi_us < tuoiImuToiDa_us,
        static_cast<unsigned long long>(m.thoiDiemSnapshot_us), static_cast<unsigned long>(m.maPhien),
        static_cast<unsigned>(m.cheDo), m.trangThaiBoDieuKhienDongCo.daBat, static_cast<unsigned>(m.loi),
        xe.gocNghien_rad, xe.tocDoNghien_radps, c.gocDat_rad, c.gocCanBang_rad, xe.gocNghien_rad - c.gocDat_rad,
        xe.vanTocDoc_mps, c.vanTocMucTieu_mps, c.tocDoQuayMucTieu_radps,
        c.lenhBanh.tocDoBuocTrai_buocTrenGiay, c.lenhBanh.tocDoBuocPhai_buocTrenGiay,
        c.heSoPid.heSoTiLe, c.heSoPid.heSoTichPhan, c.heSoPid.heSoViPhan,
        static_cast<unsigned long long>(c.soMauBoLo), tuoi_us, static_cast<unsigned long>(_soMauNghienCuuBo),
        static_cast<unsigned long>(imu.chanDoan.soLoiI2c), static_cast<unsigned long>(imu.chanDoan.soLanDocQuaChuKy),
        c.giaTocLenh_mps2, static_cast<unsigned>(xe.nguonQuangDuong));
    if (ghiDong(doDai)) _thuTuDaGhi = c.soThuTuLenh;
    else ++_soMauNghienCuuBo;
}

void RobotApp::dongBoNutVatLyCalib() {
    const auto dongBo = [](BoLocNutCalib& nut) {
        if (nut.chan < 0) return;
        const bool nhan = gpio_get_level(static_cast<gpio_num_t>(nut.chan)) == (nutCalibMucThap ? 0 : 1);
        nut.mucCu = nut.mucOnDinh = nhan;
        nut.lucDoi_ms = millis();
        nut.daNhan = false;
    };
    dongBo(_nutCalibChinh);
    dongBo(_nutCalibPhu);
}

void RobotApp::batDauCalibDung(const Vec3Imu* huongTien) {
    MotorStatus motor{};
    if (xQueuePeek(_hangDoiTrangThaiDongCo, &motor, 0) != pdTRUE || motor.cheDo != CheDoCanBang::Tat ||
        _dangChoCalib || (!huongTien && _cauHinhChay.khoaMotorCalib)) {
        inDongSerial("[CAL] Can motor tat; dung cal1 khi chua co he truc LSM.");
        return;
    }
    if (huongTien) {
        portENTER_CRITICAL(&_khoaCalib2Truc);
        ++_calib2Truc.phienYeuCau;
        _calib2Truc.dangYeuCauCalib = false;
        _calib2Truc.coKetQuaMoi = false;
        portEXIT_CRITICAL(&_khoaCalib2Truc);
    }
    _cauHinhChay.khoaMotorCalib = true;
    ++_cauHinhChay.phienBan;
    xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
    YeuCauCalibImu yeuCau{};
    yeuCau.maYeuCau = ++_maYeuCauCalib;
    yeuCau.maTranLap = _cauHinhChay.maTranLap;
    yeuCau.dungMotTuThe = huongTien != nullptr;
    if (huongTien) yeuCau.huongTienSensor = *huongTien;
    _dangChoCalib = true;
    xQueueOverwrite(_hangDoiYeuCauCalib, &yeuCau);
    inDongSerial("[CAL] Giu xe dung yen tai tu the can bang, hai banh cung cao.");
}

void RobotApp::batDauCalib2Truc(bool tuDongKhoiDong) {
    if (_dangChoCalib) {
        inDongSerial("[CALIB_3T] Dang do bias/zero, cho ket qua truoc khi bat dau.");
        return;
    }
    if (!dungProfileImuRieng) {
        inDongSerial("[CALIB_2TRUC] Profile hien tai dung ImuMountConfig.h; khong ho tro wizard nay.");
        return;
    }
    MotorStatus trangThaiDongCo{};
    if (_hangDoiTrangThaiDongCo && xQueuePeek(_hangDoiTrangThaiDongCo, &trangThaiDongCo, 0) == pdTRUE &&
        trangThaiDongCo.cheDo != CheDoCanBang::Tat) {
        inDongSerial("[CALIB_2TRUC] Hay tat motor ('d') truoc khi bat dau calib 2 truc!");
        return;
    }
    _cauHinhChay.khoaMotorCalib = true;
    ++_cauHinhChay.phienBan;
    xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
    dongBoNutVatLyCalib();
    _phaCalib2TrucDaIn = PhaCalib2Truc::KhongHoatDong;
    portENTER_CRITICAL(&_khoaCalib2Truc);
    ++_calib2Truc.phienYeuCau;
    _calib2Truc.dangYeuCauCalib = true;
    _calib2Truc.coKetQuaMoi = false;
    _calib2Truc.thanhCong = false;
    portEXIT_CRITICAL(&_khoaCalib2Truc);
    if (tuDongKhoiDong) {
        inDinhDangSerial("CALIB_3T_WIZARD: Mo http://%s -> Che do & Calib HOAC nhan nut S3 (GPIO%d) / BOOT (GPIO%d) / gui 'cal3 next'.\n",
            dieuKhienKhongDay.diaChiIpAp(), chanNutCalib, chanNutCalibPhu);
    }
}

void RobotApp::xacNhanBuocCalib2Truc() {
    if (!dungProfileImuRieng) return;
    portENTER_CRITICAL(&_khoaCalib2Truc);
    const auto pha = _calib2Truc.pha;
    const bool choXacNhan = (pha == PhaCalib2Truc::ChoDungYen || pha == PhaCalib2Truc::ChoHaDau ||
                             pha == PhaCalib2Truc::DangQuayPitch);
    if (choXacNhan) ++_calib2Truc.lenhXacNhan;
    portEXIT_CRITICAL(&_khoaCalib2Truc);
    if (choXacNhan) {
        dongBoNutVatLyCalib();
    } else {
        inDongSerial("[CALIB_3T] Hien khong o buoc cho xac nhan (go 'cal3 start' de bat dau).");
    }
}

void RobotApp::huyCalib2Truc() {
    if (!dungProfileImuRieng || _dangChoCalib) return;
    portENTER_CRITICAL(&_khoaCalib2Truc);
    ++_calib2Truc.phienYeuCau;
    _calib2Truc.dangYeuCauCalib = false;
    _calib2Truc.coKetQuaMoi = false;
    portEXIT_CRITICAL(&_khoaCalib2Truc);
    _cauHinhChay.khoaMotorCalib = !(_coMaTranLapLsm && _coBiasLuu);
    ++_cauHinhChay.phienBan;
    xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
    inDongSerial("[CALIB_3T] Da huy calib 3 tu the.");
}

void RobotApp::docNutVatLyCalib2Truc() {
    if (!dungProfileImuRieng) return;
    portENTER_CRITICAL(&_khoaCalib2Truc);
    const auto pha = _calib2Truc.pha;
    portEXIT_CRITICAL(&_khoaCalib2Truc);
    const bool choXacNhan = (pha == PhaCalib2Truc::ChoDungYen || pha == PhaCalib2Truc::ChoHaDau ||
                             pha == PhaCalib2Truc::DangQuayPitch);
    if (!choXacNhan) return;

    const auto kiemTraNut = [](BoLocNutCalib& nut) -> bool {
        if (nut.chan < 0) return false;
        const bool muc = gpio_get_level(static_cast<gpio_num_t>(nut.chan)) == (nutCalibMucThap ? 0 : 1);
        if (muc != nut.mucCu) {
            nut.mucCu = muc;
            nut.lucDoi_ms = millis();
        }
        if (muc == nut.mucOnDinh || millis() - nut.lucDoi_ms < 35u) return false;
        nut.mucOnDinh = muc;
        if (muc) {
            nut.daNhan = true;
            return false;
        }
        const bool kichHoat = nut.daNhan;
        nut.daNhan = false;
        return kichHoat;
    };
    if (kiemTraNut(_nutCalibChinh) || kiemTraNut(_nutCalibPhu)) {
        xacNhanBuocCalib2Truc();
    }
}

void RobotApp::xuLyCalib2TrucTrongLoop() {
    if (!dungProfileImuRieng) return;
    PhaCalib2Truc pha = PhaCalib2Truc::KhongHoatDong;
    bool coKetQua = false;
    bool thanhCong = false;
    MaTranQuayImu maTranMoi{};
    float bx = 0, by = 0, bz = 0, normCalib2 = 9.80665f;
    BaoCaoCalibDoc baoCao{};
    Vec3Imu aD{}, aT{}, aS{};
    char lyDo[32]{};

    portENTER_CRITICAL(&_khoaCalib2Truc);
    pha = _calib2Truc.pha;
    coKetQua = _calib2Truc.coKetQuaMoi && _calib2Truc.dangYeuCauCalib &&
        _calib2Truc.phienKetQua == _calib2Truc.phienYeuCau;
    if (coKetQua) {
        _calib2Truc.dangYeuCauCalib = false;
        _calib2Truc.coKetQuaMoi = false;
        thanhCong = _calib2Truc.thanhCong;
        maTranMoi = _calib2Truc.maTranKetQua;
        bx = _calib2Truc.biasX_dps;
        by = _calib2Truc.biasY_dps;
        bz = _calib2Truc.biasZ_dps;
        normCalib2 = _calib2Truc.normGiaTocTinh_mps2;
        baoCao = _calib2Truc.baoCao;
        aD = _calib2Truc.aDung;
        aT = _calib2Truc.aTruoc;
        aS = _calib2Truc.aSau;
        std::snprintf(lyDo, sizeof(lyDo), "%s", _calib2Truc.lyDoLoi);
    }
    portEXIT_CRITICAL(&_khoaCalib2Truc);

    if (pha != _phaCalib2TrucDaIn) {
        _phaCalib2TrucDaIn = pha;
        dongBoNutVatLyCalib();
        if (pha == PhaCalib2Truc::ChoDungYen) {
            inDongSerial("CALIB_3T [Buoc 1/3]: Dung xe thang dung, hai banh cung cao, giu yen roi nhan Xac nhan (Web / S3 / BOOT).");
        } else if (pha == PhaCalib2Truc::ChoHaDau) {
            inDongSerial("CALIB_3T [Buoc 2/3]: Nghieng DAU xe 90 deg ve TRUOC, giu yen -> nhan Xac nhan.");
        } else if (pha == PhaCalib2Truc::DangQuayPitch) {
            inDongSerial("CALIB_3T [Buoc 3/3]: Nang DAU qua thang dung roi nghieng 90 deg ve SAU (4-8s), giu yen -> nhan Xac nhan.");
        }
    }

    if (coKetQua) {
        if (thanhCong) {
            const char* loi = nullptr;
            if (!HeTrucImu::kiemTraMaTran(maTranMoi) || !vectoHuuHan({bx, by, bz})) loi = "calib_invalid";
            else if (!luuMaTranMoi(_camBienGoc.maNhanDang(), maTranMoi)) loi = "mount_nvs_write";
            if (loi) {
                thanhCong = false;
                std::snprintf(lyDo, sizeof(lyDo), "%s", loi);
                portENTER_CRITICAL(&_khoaCalib2Truc);
                _calib2Truc.pha = PhaCalib2Truc::Loi;
                _calib2Truc.thanhCong = false;
                std::snprintf(_calib2Truc.lyDoLoi, sizeof(_calib2Truc.lyDoLoi), "%s", loi);
                portEXIT_CRITICAL(&_khoaCalib2Truc);
            }
        }
        if (thanhCong) {
            _heTrucImu = HeTrucImu(maTranMoi);
            _biasX_dps = bx;
            _biasY_dps = by;
            _biasZ_dps = bz;
            _coBiasLuu = true;
            _coMaTranLapLsm = true;
            _cauHinhChay.biasGyro_dps = {bx, by, bz};
            _cauHinhChay.coBiasGyro = true;
            if (dungLsm6dsv) _cauHinhChay.gocCanBang_rad = ::gocCanBang_rad;
            if (dungLsm6dsv && std::isfinite(normCalib2) && normCalib2 >= 9.0f && normCalib2 <= 10.6f) {
                _cauHinhChay.normGiaTocTinh_mps2 = normCalib2;
            }
            _cauHinhChay.maTranLap = maTranMoi;
            ++_cauHinhChay.phienBanMaTranLap;
            _cauHinhChay.khoaMotorCalib = false;
            ++_cauHinhChay.phienBan;
            const bool daLuuCauHinh = luuCauHinhVaoNvs();
            xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
            xTaskNotify(_tacVuDongCo, suKienResetXung, eSetBits);
            inDinhDangSerial("CALIB_DOC PASS truoc=%.2f deg sau=%.2f deg gyro=%.2f deg energy=%.4f vectorErr=%.2f deg [%s]\n",
                baoCao.gocMocTruoc_rad * doiRadSangDo, baoCao.gocMocSau_rad * doiRadSangDo,
                baoCao.gocQuayGyro_rad * doiRadSangDo, baoCao.tyLeNangLuongTruc,
                baoCao.lechVector_rad * doiRadSangDo, daLuuCauHinh ? "SAVED_NVS" : "MOUNT_SAVED_CONFIG_INCOMPLETE");
            for (unsigned truc = 0; truc < 3; ++truc) {
                inDinhDangSerial("CALIB_AXIS %c_sensor=%.6f,%.6f,%.6f\n", "XYZ"[truc],
                    maTranMoi.phanTu[truc][0], maTranMoi.phanTu[truc][1], maTranMoi.phanTu[truc][2]);
            }
        } else {
            inDinhDangSerial("CALIB_DOC FAIL %s truoc=%.2f deg sau=%.2f deg gyro=%.2f deg energy=%.4f vectorErr=%.2f deg\n",
                lyDo,
                baoCao.gocMocTruoc_rad * doiRadSangDo, baoCao.gocMocSau_rad * doiRadSangDo,
                baoCao.gocQuayGyro_rad * doiRadSangDo, baoCao.tyLeNangLuongTruc,
                baoCao.lechVector_rad * doiRadSangDo);
            inDinhDangSerial("CALIB_MOC a_dung=%.4f,%.4f,%.4f a_truoc=%.4f,%.4f,%.4f a_sau=%.4f,%.4f,%.4f mps2\n",
                aD.x, aD.y, aD.z, aT.x, aT.y, aT.z, aS.x, aS.y, aS.z);
            _cauHinhChay.khoaMotorCalib = !(_coMaTranLapLsm && _coBiasLuu);
            ++_cauHinhChay.phienBan;
            xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
        }
    }
}

void RobotApp::loop() {
    if (!_daSanSang) {
        static uint32_t lanBaoLoiBoot_ms = 0;
        if (millis() - lanBaoLoiBoot_ms >= 2000) {
            lanBaoLoiBoot_ms = millis();
            Serial.printf("BOOT ERROR: khoi tao that bai (%s/%s WHO=0x%02X busError=%u reg=0x%02X). Kiem tra nguon/day/calib va RST.\n",
                tenImu, giaoTiepImu, _camBienGoc.maNhanDang(), static_cast<unsigned>(_camBienGoc.maLoiI2cCuoi()), _camBienGoc.thanhGhiCuoi());
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
            _cauHinhChay.khoaMotorCalib = dungProfileImuRieng && !(_coMaTranLapLsm && _coBiasLuu);
            if (kqCalib.thanhCong && kqCalib.dungMotTuThe) {
                kqCalib.thanhCong = luuMaTranMoi(_camBienGoc.maNhanDang(), kqCalib.maTranLap);
                if (kqCalib.thanhCong) {
                    _heTrucImu = HeTrucImu(kqCalib.maTranLap);
                    _cauHinhChay.maTranLap = kqCalib.maTranLap;
                    _coMaTranLapLsm = true;
                } else inDongSerial("[CAL1] Khong luu duoc ma tran vao NVS.");
            }
            if (kqCalib.thanhCong) {
                _biasX_dps = kqCalib.biasX_dps;
                _biasY_dps = kqCalib.biasY_dps;
                _biasZ_dps = kqCalib.biasZ_dps;
                _coBiasLuu = true;
                _cauHinhChay.biasGyro_dps = {_biasX_dps, _biasY_dps, _biasZ_dps};
                _cauHinhChay.coBiasGyro = true;
                _cauHinhChay.khoaMotorCalib = false;
                ++_cauHinhChay.phienBanMaTranLap;
                _cauHinhChay.gocCanBang_rad = kqCalib.gocCanBangDo_rad;
                if (dungLsm6dsv && std::isfinite(kqCalib.normGiaTocTinh_mps2) &&
                    kqCalib.normGiaTocTinh_mps2 >= 9.0f && kqCalib.normGiaTocTinh_mps2 <= 10.6f) {
                    _cauHinhChay.normGiaTocTinh_mps2 = kqCalib.normGiaTocTinh_mps2;
                }
                ++_cauHinhChay.phienBan;
                xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
                xTaskNotify(_tacVuDongCo, suKienResetXung, eSetBits);
                const bool daLuu = luuCauHinhVaoNvs();
                inDinhDangSerial("[CAL] PASS samples=%u retry=%u stdGyro=%.4fdps varAcc=%.5f bias_dps=%.4f,%.4f,%.4f eqPitch=%.5frad (%.2fdeg) [%s]\n",
                              kqCalib.soMauDaThu, kqCalib.soLanThuLai, kqCalib.doLechChuanGyro_dps,
                              kqCalib.phuongSaiGiaToc, _biasX_dps, _biasY_dps, _biasZ_dps,
                              _cauHinhChay.gocCanBang_rad, _cauHinhChay.gocCanBang_rad * doiRadSangDo, daLuu ? "SAVED_NVS" : "RAM_ONLY");
            } else {
                ++_cauHinhChay.phienBan;
                xQueueOverwrite(_hangDoiCauHinh, &_cauHinhChay);
                inDinhDangSerial("[CAL] FAIL retry=%u. Kiem tra dung yen, tu the va huong truc; thu lai %s.\n",
                              kqCalib.soLanThuLai, kqCalib.dungMotTuThe ? "cal1" : "c");
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
    xuLyCalib2TrucTrongLoop();
    docNutVatLyCalib2Truc();
    if (_dangChoTestBanh) {
        KetQuaTestBanh kqTest{};
        portENTER_CRITICAL(&_khoaTestBanh);
        kqTest = _ketQuaTestBanh;
        portEXIT_CRITICAL(&_khoaTestBanh);
        if (kqTest.hoanTat && kqTest.maYeuCau != _maKetQuaTestBanhDaIn) {
            _maKetQuaTestBanhDaIn = kqTest.maYeuCau;
            _dangChoTestBanh = false;
            inDinhDangSerial("[MTEST] %s %s | dSTEP=(%lld,%lld) dENC=(%lld,%lld) uartOk=(%d,%d) invDir=%d\n",
                kqTest.nhan, kqTest.biHuy ? "ABORT" : "DONE",
                static_cast<long long>(kqTest.dStepTrai), static_cast<long long>(kqTest.dStepPhai),
                static_cast<long long>(kqTest.dEncTrai), static_cast<long long>(kqTest.dEncPhai),
                kqTest.coUartTrai ? 1 : 0, kqTest.coUartPhai ? 1 : 0,
                _cauHinhChay.daoChieuDongCo ? 1 : 0);
        }
    }
    guiChanDoanEspNow();
    capNhatThongTinLogRung();
    if (_dangPhatNghienCuu) ghiMauNghienCuu();

    if (_canLuuNvs) {
        MotorStatus trangThaiDongCoNvs{};
        if (xQueuePeek(_hangDoiTrangThaiDongCo, &trangThaiDongCoNvs, 0) == pdTRUE &&
            trangThaiDongCoNvs.cheDo == CheDoCanBang::Tat) {
            if (luuCauHinhVaoNvs()) {
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
        if (_inDuLieuImu && baoCaoDongCo.cheDo != CheDoCanBang::Tat && !_cauHinhChay.cheDoThuKhongMaSat)
            _inDuLieuImu = false;
        if (!_dangPhatNghienCuu && _inDuLieuImu && gocConMoi && baoCaoDieuKhien.khungCamBien.trangThai == TrangThaiKhungDuLieuImu::HopLe) {
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
        if (!_batTelemetry || _dangPhatNghienCuu || Serial.availableForWrite() == 0) {
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
