#pragma once
#include <Arduino.h>
#include "Icm20602Driver.h"
#include "MksServo42C.h"
#include "WheelPulseCounter.h"
#include "ComplementaryEstimator.h"
#include "DifferentialDrive.h"
#include "CascadeController.h"
#include "model/FaultCode.h"
#include "model/LenhDiChuyen.h"
#include "core/PhienCanBang.h"
#include "core/BoLocThuTuMau.h"
#include "freertos/queue.h"
#include "esp_timer.h"

class RobotApp {
public:
    RobotApp() : _camBienGoc(Wire) {}
    bool begin();
    void loop();
private:
    struct MotorStatus {
        MksSnapshot trangThaiBoDieuKhienDongCo{};
        WheelPulseFrame khungXung{};
        FaultCode loi = FaultCode::None;
        MksResult ketQuaCuoi = MksResult::Disabled;
        bool dungPidMo = true;
        uint32_t khoangCachPhucVuLonNhat_us = 0;
        uint32_t maPhien = 0;
        CheDoCanBang cheDo = CheDoCanBang::Tat;
    };
    struct ControlStatus {
        TrangThaiXe trangThaiXe{};
        WheelStepCommand lenhBanh{};
        FaultCode loi = FaultCode::ImuInvalid;
        PidGains heSoPid{};
        float gocDat_rad = 0;
        float gyroDaLoc_radps = 0, vanTocMucTieu_mps = 0, tocDoQuayMucTieu_radps = 0;
        KhungDuLieuImu khungCamBien{}; // Bản raw hệ sensor phục vụ chẩn đoán gá lắp.
        uint32_t soThuTuLenh = 0;
        uint32_t maPhien = 0;
        uint64_t soMauBoLo = 0;
        uint32_t soThongBaoTrung = 0, soLanDocChongNgat = 0;
    };
    static void IRAM_ATTR imuInterrupt(void* nguCanh);
    static void motorTimer(void* nguCanh);
    static void imuEntry(void* nguCanh);
    static void controlEntry(void* nguCanh);
    static void motorEntry(void* nguCanh);
    void imuLoop();
    void controlLoop();
    void motorLoop();
    void cleanupStartup();
    Icm20602Driver _camBienGoc;
    MksServo42CPair _capDongCo;
    WheelPulseCounter _boDemXung;
    ComplementaryEstimator _boUocLuongGoc;
    DifferentialDrive _boQuyDoiBanhXe;
    CascadeController _boDieuKhienTang;
    QueueHandle_t _hangDoiImu = nullptr, _hangDoiTrangThaiDongCo = nullptr, _hangDoiTrangThaiDieuKhien = nullptr;
    QueueHandle_t _hangDoiDiChuyen = nullptr;
    TaskHandle_t _tacVuImu = nullptr, _tacVuDieuKhien = nullptr, _tacVuDongCo = nullptr;
    esp_timer_handle_t _boHenDongCo = nullptr;
    portMUX_TYPE _khoaDuLieuNgatImu = portMUX_INITIALIZER_UNLOCKED;
    uint64_t _thoiDiemNgat_us = 0;
    uint32_t _soThuTuNgat = 0;
    bool _daSanSang = false;
};
