#include "RobotApp.h"
#include "config/ControlConfig.h"
#include "config/CalibConfig.h"
#include "config/MotorConfig.h"
#include "config/PinConfig.h"
#include "config/SafetyConfig.h"
#include "safety/SafetyMonitor.h"
#include "driver/gpio.h"
#include "calib/UI_Calib.h"

namespace {
constexpr uint32_t suKienDinhKy = 1u, suKienBatDongCo = 2u, suKienTatDongCo = 4u, suKienDungKhan = 8u,
                   suKienXoaLoi = 16u, suKienDoiCheDoMo = 32u;
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
}

bool RobotApp::begin() {
    Serial.begin(115200);
    const auto chanChoPhepDongCo = static_cast<gpio_num_t>(chanBatMotor);
    if (gpio_set_level(chanChoPhepDongCo, mucBatMotorCao ? 0 : 1) != ESP_OK ||
        gpio_set_direction(chanChoPhepDongCo, GPIO_MODE_OUTPUT) != ESP_OK) return false;
    UI_Calib uiCalib(_camBienGoc);
    if (batUiCalib) uiCalib.khoiTao();
    if (!batUiCalib && !_boUocLuongGoc.heTrucHopLe()) {
        Serial.println("BOOT ERROR: ma tran lap IMU khong phai phep quay hop le");
        return false;
    }
    if (!batNgatMpu || chuKyPhucVuMotor_us >= khoangCachPhucVuMotorToiDa_us ||
        1000000u / tanSoLayMauIcm_hz >= thoiGianHetHanLenhMotor_us) return false;
    if (!_capDongCo.begin() || !_boDemXung.begin()) {
        if (batUiCalib) uiCalib.baoLoi();
        Serial.println("BOOT ERROR: MCPWM/PCNT"); return false;
    }
    if (!_camBienGoc.begin()) {
        if (batUiCalib) uiCalib.baoLoi();
        Serial.println("BOOT ERROR: ICM20602, WHO_AM_I phai la 0x12"); return false;
    }
    if (batUiCalib) {
        MaTranQuayImu maTranLap;
        uiCalib.chay(maTranLap);
        _boUocLuongGoc = ComplementaryEstimator(HeTrucImu(maTranLap));
    } else if (!_camBienGoc.hieuChinhConQuay(soMauHieuChinhMpu)) {
        Serial.println("BOOT ERROR: gyro calibration"); return false;
    }
    _hangDoiImu = xQueueCreate(1, sizeof(KhungDuLieuImu));
    _hangDoiTrangThaiDongCo = xQueueCreate(1, sizeof(MotorStatus));
    _hangDoiTrangThaiDieuKhien = xQueueCreate(1, sizeof(ControlStatus));
    _hangDoiDiChuyen = xQueueCreate(1, sizeof(LenhDiChuyen));
    if (!_hangDoiImu || !_hangDoiTrangThaiDongCo || !_hangDoiTrangThaiDieuKhien || !_hangDoiDiChuyen) {
        if (batUiCalib) uiCalib.baoLoi();
        cleanupStartup(); return false;
    }
    MotorStatus trangThaiBanDau{};
    trangThaiBanDau.trangThaiBoDieuKhienDongCo = _capDongCo.snapshot();
    trangThaiBanDau.khungXung = _boDemXung.read();
    trangThaiBanDau.dungPidMo = batPidMo;
    xQueueOverwrite(_hangDoiTrangThaiDongCo, &trangThaiBanDau);
    if (xTaskCreate(imuEntry, "Imu", 4096, this, 4, &_tacVuImu) != pdPASS ||
        xTaskCreate(controlEntry, "Control", 6144, this, 3, &_tacVuDieuKhien) != pdPASS ||
        xTaskCreate(motorEntry, "Motor", 4096, this, 5, &_tacVuDongCo) != pdPASS) {
        if (batUiCalib) uiCalib.baoLoi();
        cleanupStartup(); return false;
    }
    pinMode(chanNgatMpu, INPUT);
    attachInterruptArg(chanNgatMpu, imuInterrupt, this, mucNgatMpuCao ? RISING : FALLING);
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
    if (batUiCalib) uiCalib.baoSanSang();
    Serial.printf("READY ICM20602 %uHz; odometry=STEP/DIR PCNT; MKS=CR_vFOC; hardwareConfirmed=%d\n",
                  tanSoLayMauIcm_hz, daXacNhanCauHinhPhanCungMotor);
    Serial.println("a=arm; d=disable; x=emergency; k=clear fault; f=toggle fuzzy khi disabled; h=help");
    Serial.println("Sau arm: w/s=tien/lui; j/l=quay trai/phai; z=ve muc tieu0; lap lenh <500ms.");
    Serial.println("i=bat/tat RAW_IMU 5Hz khi motor tat; ma tran lap trong ImuMountConfig.h.");
    return true;
}

void RobotApp::imuLoop() {
    BoLocThuTuMau boLocThuTu;
    uint32_t soLanDocChongNgat = 0;
    for (;;) {
        KhungDuLieuImu khungDuLieu{};
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20))) {
            portENTER_CRITICAL(&_khoaDuLieuNgatImu);
            const uint64_t thoiDiemMau_us = _thoiDiemNgat_us;
            const uint32_t soThuTuMau = _soThuTuNgat;
            portEXIT_CRITICAL(&_khoaDuLieuNgatImu);
            // Bỏ thông báo trùng, không biến nó thành một frame lỗi.
            if (!boLocThuTu.chapNhan(soThuTuMau)) continue;
            if (_camBienGoc.read(khungDuLieu)) {
                portENTER_CRITICAL(&_khoaDuLieuNgatImu);
                const uint32_t soThuTuSauDoc = _soThuTuNgat;
                portEXIT_CRITICAL(&_khoaDuLieuNgatImu);
                // Bỏ mẫu không chắc timestamp nếu có ngắt mới trong lúc đọc I2C.
                if (soThuTuSauDoc != soThuTuMau) {
                    ++soLanDocChongNgat;
                    continue;
                }
                khungDuLieu.thoiDiemLayMau_us = thoiDiemMau_us;
                khungDuLieu.soMauLayDuoc = soThuTuMau;
            }
        }
        khungDuLieu.soMauBoLo = boLocThuTu.soMauBoLo();
        khungDuLieu.soThongBaoTrung = boLocThuTu.soThongBaoTrung();
        khungDuLieu.soLanDocChongNgat = soLanDocChongNgat;
        xQueueOverwrite(_hangDoiImu, &khungDuLieu);
    }
}

void RobotApp::controlLoop() {
    TrangThaiXe trangThaiXe{};
    uint32_t soThuTuLenh = 0;
    bool dungPidMoTruoc = batPidMo;
    uint32_t maPhienTruoc = 0;
    for (;;) {
        KhungDuLieuImu khungDuLieu{};
        xQueueReceive(_hangDoiImu, &khungDuLieu, portMAX_DELAY);
        MotorStatus trangThaiDongCo{};
        xQueuePeek(_hangDoiTrangThaiDongCo, &trangThaiDongCo, 0);
        _boUocLuongGoc.update(khungDuLieu, trangThaiXe);
        _boQuyDoiBanhXe.update(trangThaiDongCo.khungXung, trangThaiXe);
        ControlStatus baoCaoDieuKhien{};
        baoCaoDieuKhien.trangThaiXe = trangThaiXe;
        baoCaoDieuKhien.khungCamBien = khungDuLieu;
        baoCaoDieuKhien.maPhien = trangThaiDongCo.maPhien;
        baoCaoDieuKhien.soMauBoLo = khungDuLieu.soMauBoLo;
        baoCaoDieuKhien.soThongBaoTrung = khungDuLieu.soThongBaoTrung;
        baoCaoDieuKhien.soLanDocChongNgat = khungDuLieu.soLanDocChongNgat;
        const uint64_t thoiDiemHienTai_us = nowUs();
        baoCaoDieuKhien.loi = SafetyMonitor::check(trangThaiXe, thoiDiemHienTai_us);
        if (trangThaiDongCo.cheDo != CheDoCanBang::DangChay ||
            trangThaiDongCo.maPhien != maPhienTruoc ||
            baoCaoDieuKhien.loi != FaultCode::None || trangThaiDongCo.dungPidMo != dungPidMoTruoc)
            _boDieuKhienTang.reset();
        maPhienTruoc = trangThaiDongCo.maPhien;
        dungPidMoTruoc = trangThaiDongCo.dungPidMo;
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
    baoCaoDongCo.dungPidMo = batPidMo;
    uint32_t soThuTuLenhDaApDung = 0;
    uint64_t thoiDiemPhucVuTruoc_us = 0;
    for (;;) {
        uint32_t cacSuKien = 0;
        xTaskNotifyWait(0, UINT32_MAX, &cacSuKien, pdMS_TO_TICKS(5));
        // Nhận lỗi ISR giám sát trước khi xét bất kỳ yêu cầu bật lại nào.
        if (_capDongCo.service() == MksResult::FaultLatched) {
            if (baoCaoDongCo.loi == FaultCode::None) baoCaoDongCo.loi = FaultCode::MotorFault;
            if (phienCanBang.cheDo() != CheDoCanBang::Tat) phienCanBang.tat();
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
        const bool lenhConMoi = coDuLieuDieuKhien &&
            baoCaoDieuKhien.lenhBanh.thoiDiemRaLenh_us <= thoiDiemHienTai_us &&
            thoiDiemHienTai_us - baoCaoDieuKhien.lenhBanh.thoiDiemRaLenh_us < tuoiLenhDeBat_us &&
            baoCaoDieuKhien.lenhBanh.thoiDiemHetHan_us > thoiDiemHienTai_us;
        FaultCode loiAnToan = coDuLieuDieuKhien
            ? SafetyMonitor::check(baoCaoDieuKhien.trangThaiXe, thoiDiemHienTai_us)
            : FaultCode::ImuInvalid;
        if (loiAnToan == FaultCode::None) loiAnToan = baoCaoDieuKhien.loi;

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
        } else if (cacSuKien & suKienDoiCheDoMo) {
            if (!_capDongCo.isEnabled()) {
                phienCanBang.tat();
                baoCaoDongCo.dungPidMo = !baoCaoDongCo.dungPidMo;
            }
        } else if ((cacSuKien & suKienBatDongCo) &&
                   phienCanBang.cheDo() == CheDoCanBang::Tat &&
                   baoCaoDongCo.loi == FaultCode::None && lenhConMoi &&
                   loiAnToan == FaultCode::None &&
                   SafetyMonitor::canArm(baoCaoDieuKhien.trangThaiXe, thoiDiemHienTai_us)) {
            phienCanBang.yeuCauBat(thoiDiemHienTai_us);
        }

        if (phienCanBang.cheDo() == CheDoCanBang::ChoLenhMoi) {
            if (phienCanBang.quaHanCho(thoiDiemHienTai_us)) {
                baoCaoDongCo.loi = FaultCode::ArmTimeout;
                phienCanBang.tat();
                _capDongCo.disable();
            } else if (loiAnToan != FaultCode::None ||
                       !SafetyMonitor::canArm(baoCaoDieuKhien.trangThaiXe, thoiDiemHienTai_us)) {
                phienCanBang.tat(); // Tư thế thay đổi trong khi chờ: phải yêu cầu bật lại.
            } else if (phienCanBang.lenhKhoiDongHopLe(
                           baoCaoDieuKhien.maPhien, baoCaoDieuKhien.lenhBanh, thoiDiemHienTai_us)) {
                baoCaoDongCo.ketQuaCuoi = _capDongCo.enable();
                if (baoCaoDongCo.ketQuaCuoi == MksResult::Ok) {
                    phienCanBang.xacNhanDaBat();
                    soThuTuLenhDaApDung = baoCaoDieuKhien.soThuTuLenh - 1u;
                } else if (baoCaoDongCo.ketQuaCuoi != MksResult::Pending) {
                    phienCanBang.tat();
                }
            }
        }
        if (phienCanBang.cheDo() == CheDoCanBang::DangChay) {
            if (loiAnToan != FaultCode::None || !lenhConMoi ||
                baoCaoDieuKhien.maPhien != phienCanBang.maPhien()) {
                baoCaoDongCo.loi = loiAnToan == FaultCode::None ? FaultCode::InvalidControl : loiAnToan;
                phienCanBang.tat();
                _capDongCo.emergencyStop();
            } else if (baoCaoDieuKhien.soThuTuLenh != soThuTuLenhDaApDung) {
                baoCaoDongCo.ketQuaCuoi = _capDongCo.apply(baoCaoDieuKhien.lenhBanh);
                soThuTuLenhDaApDung = baoCaoDieuKhien.soThuTuLenh;
            }
        }
        if (_capDongCo.service() == MksResult::FaultLatched) {
            if (baoCaoDongCo.loi == FaultCode::None) baoCaoDongCo.loi = FaultCode::MotorFault;
            if (phienCanBang.cheDo() != CheDoCanBang::Tat) phienCanBang.tat();
        }
        baoCaoDongCo.khungXung = _boDemXung.read();
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

void RobotApp::loop() {
    if (!_daSanSang) { delay(100); return; }
    static bool inDuLieuImu = false;
    uint32_t cacSuKien = 0;
    for (unsigned chiSoKyTu = 0; chiSoKyTu < 16 && Serial.available(); ++chiSoKyTu) {
        const int kyTu = Serial.read();
        if (kyTu == 'w' || kyTu == 's' || kyTu == 'j' || kyTu == 'l' || kyTu == 'z') {
            MotorStatus trangThaiDongCo{};
            if (xQueuePeek(_hangDoiTrangThaiDongCo, &trangThaiDongCo, 0) == pdTRUE &&
                trangThaiDongCo.cheDo == CheDoCanBang::DangChay) {
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
        }
        switch (kyTu) {
        case 'i': inDuLieuImu = !inDuLieuImu; break;
        case 'a': cacSuKien |= suKienBatDongCo; break;
        case 'd': cacSuKien |= suKienTatDongCo; break;
        case 'x': cacSuKien |= suKienDungKhan; break;
        case 'k': cacSuKien |= suKienXoaLoi; break;
        case 'f': cacSuKien |= suKienDoiCheDoMo; break;
        case 'h': Serial.println("a=arm; d=disable; x=emergency; k=clear; f=fuzzy; w/s=tien/lui; j/l=trai/phai; z=dung; i=raw IMU khi tat. Lap lenh <500ms"); break;
        default: break;
        }
    }
    if (cacSuKien) xTaskNotify(_tacVuDongCo, cacSuKien, eSetBits);
    static uint32_t thoiDiemBaoCaoTruoc_ms = 0;
    if (millis() - thoiDiemBaoCaoTruoc_ms >= chuKyTelemetry_ms) {
        thoiDiemBaoCaoTruoc_ms = millis();
        MotorStatus baoCaoDongCo{};
        ControlStatus baoCaoDieuKhien{};
        xQueuePeek(_hangDoiTrangThaiDongCo, &baoCaoDongCo, 0);
        xQueuePeek(_hangDoiTrangThaiDieuKhien, &baoCaoDieuKhien, 0);
        if (inDuLieuImu && baoCaoDongCo.cheDo != CheDoCanBang::Tat) inDuLieuImu = false;
        if (inDuLieuImu && baoCaoDieuKhien.khungCamBien.trangThai == TrangThaiKhungDuLieuImu::HopLe) {
            const auto& mau = baoCaoDieuKhien.khungCamBien;
            // Gia tốc còn chứa trọng lực; gyro đã trừ bias. Tất cả vẫn ở hệ sensor.
            Serial.printf("RAW_IMU,%llu,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n",
                static_cast<unsigned long long>(mau.thoiDiemLayMau_us),
                mau.giaTocX_mps2,mau.giaTocY_mps2,mau.giaTocZ_mps2,
                mau.tocDoGocX_radps,mau.tocDoGocY_radps,mau.tocDoGocZ_radps);
        }
        Serial.printf("EN=%d fuzzy=%d fault=%s ready=%s motorFault=%u last=%u "
            "theta=%.4f rate=%.4f target=%.4f x=%.4f v=%.4f STEP=%lld,%lld "
            "cmd=%.1f,%.1f K=%.2f,%.2f,%.2f serviceMax=%luus "
            "roll=%.4f accel=%.2f phase=%u epoch=%lu miss=%llu dup=%lu overlap=%lu "
            "gyroD=%.4f innov=%.4f trust=%.3f vref=%.3f yawref=%.3f tilt=%.4f tiltDir=%.4f\n",
            baoCaoDongCo.trangThaiBoDieuKhienDongCo.daBat, baoCaoDongCo.dungPidMo,
            faultName(baoCaoDongCo.loi), faultName(baoCaoDieuKhien.loi),
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
            static_cast<unsigned long>(baoCaoDieuKhien.soLanDocChongNgat),
            baoCaoDieuKhien.gyroDaLoc_radps, baoCaoDieuKhien.trangThaiXe.saiLechGiaToc_rad,
            baoCaoDieuKhien.trangThaiXe.doTinCayGiaToc, baoCaoDieuKhien.vanTocMucTieu_mps,
            baoCaoDieuKhien.tocDoQuayMucTieu_radps, baoCaoDieuKhien.trangThaiXe.gocNghienTong_rad,
            baoCaoDieuKhien.trangThaiXe.huongNghien_rad);
    }
    delay(1);
}
