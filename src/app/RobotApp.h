#pragma once
#include <Arduino.h>
#include <type_traits>
#include "Icm20602Driver.h"
#include "MksServo42C.h"
#include "Tmc2209Pair.h"
#include "WheelPulseCounter.h"
#include "Mt6701EncoderCounter.h"
#include "ComplementaryEstimator.h"
#include "DifferentialDrive.h"
#include "CascadeController.h"
#include "model/FaultCode.h"
#include "model/LenhDiChuyen.h"
#include "core/PhienCanBang.h"
#include "core/BoLocThuTuMau.h"
#include "config/ControlConfig.h"
#include "telemetry/NhatKyNghienCuu.h"
#include "HeTrucImu.h"
#include "DieuKhienKhongDay.h"
#include "LienKetChanDoanEspNow.h"
#include "freertos/queue.h"
#include "esp_timer.h"

class RobotApp {
public:
    RobotApp() : _camBienGoc(Wire) {}
    bool begin();
    void loop();
private:
    using CapDriverDongCo = std::conditional_t<dungPhanCungTmcMt, Tmc2209Pair, MksServo42CPair>;
    using BoDemPhanHoiBanh = std::conditional_t<dungPhanCungTmcMt, Mt6701EncoderCounter, WheelPulseCounter>;

    struct CauHinhChay {
        float kpGocThuan = kpGocPidThuan;
        float kiGocThuan = kiGocPidThuan;
        float kdGocThuan = kdGocPidThuan;
        float kpVanTocThuan = kpVanTocPidThuan;
        float kiVanTocThuan = kiVanTocPidThuan;
        float kpGocMo = kpGoc;
        float kiGocMo = kiGoc;
        float kdGocMo = kdGoc;
        float kpVanTocMo = kpVanToc;
        float kiVanTocMo = kiVanToc;
        float gocCanBang_rad = ::gocCanBang_rad;
        bool cheDoThuKhongMaSat = false;
        bool batVongNgoai = true;
        bool daoChieuDongCo = false;
        bool dungPidMo = batPidMo;
        uint32_t phienBan = 1;
        uint32_t soLanChotViTri = 0;
    };
    struct YeuCauCalibImu {
        uint32_t maYeuCau = 0;
    };
    struct KetQuaCalibImu {
        uint32_t maYeuCau = 0;
        bool hoanTat = false;
        bool thanhCong = false;
        float biasX_dps = 0.0f, biasY_dps = 0.0f, biasZ_dps = 0.0f;
        float gocCanBangDo_rad = 0.0f;
        float doLechChuanGyro_dps = 0.0f;
        float phuongSaiGiaToc = 0.0f;
        uint16_t soMauDaThu = 0;
        uint16_t soLanThuLai = 0;
    };

    struct MotorStatus {
        uint64_t thoiDiemSnapshot_us = 0;
        MksSnapshot trangThaiBoDieuKhienDongCo{};
        WheelPulseFrame khungXung{};
        FaultCode loi = FaultCode::None;
        MksResult ketQuaCuoi = MksResult::Disabled;
        bool dungPidMo = batPidMo;
        uint32_t khoangCachPhucVuLonNhat_us = 0;
        uint32_t maPhien = 0;
        uint32_t maResetXung = 0;
        CheDoCanBang cheDo = CheDoCanBang::Tat;
    };
    struct ControlStatus {
        TrangThaiXe trangThaiXe{};
        WheelStepCommand lenhBanh{};
        TrangThaiVongKinBanhXe vongKinBanh{};
        FaultCode loi = FaultCode::ImuInvalid;
        PidGains heSoPid{};
        float gocDat_rad = 0;
        float gocCanBang_rad = ::gocCanBang_rad;
        float gyroDaLoc_radps = 0, vanTocMucTieu_mps = 0, tocDoQuayMucTieu_radps = 0;
        KhungDuLieuImu khungCamBien{}; // Bản raw hệ sensor phục vụ chẩn đoán gá lắp.
        uint32_t soThuTuLenh = 0;
        uint32_t maPhien = 0;
        uint32_t phienBanCauHinh = 0;
        CheDoCanBang phaDieuKhien = CheDoCanBang::Tat;
        bool dungPidMo = false, bench = false, vongNgoai = false;
        PidGains heSoGocCoSo{};
        float kpVanTocCoSo = 0, kiVanTocCoSo = 0, giaTocLenh_mps2 = 0;
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
    void apDungCauHinh(const CauHinhChay& cauHinh);
    void xuLyDongLenhSerial(const char* dongLenh);
    void xuLyPhimDonSerial(char kyTu);
    void inTrangThaiChiTiet();
    void inTroGiup();
    void luuCauHinhVaoNvs();
    void nhanLenhDiChuyenTuXa(float vanTocTienLui, float tocDoQuayNgang);
    void dienTrangThaiWeb(ThongTinTrangThaiWeb& trangThaiRa);
    void inDongSerial(const char* noiDung);
    void inDinhDangSerial(const char* dinhDang, ...);
    void ghiMauNghienCuu();
    void guiChanDoanEspNow();

    DieuKhienKhongDay dieuKhienKhongDay;
    LienKetChanDoanEspNow lienKetChanDoan;

    Icm20602Driver _camBienGoc;
    CapDriverDongCo _capDongCo;
    BoDemPhanHoiBanh _boDemXung;
    HeTrucImu _heTrucImu;
    ComplementaryEstimator _boUocLuongGoc;
    DifferentialDrive _boQuyDoiBanhXe;
    CascadeController _boDieuKhienTang;
    CauHinhChay _cauHinhChay{};
    float _biasX_dps = 0.0f, _biasY_dps = 0.0f, _biasZ_dps = 0.0f;
    bool _coBiasLuu = false;
    uint32_t _maYeuCauCalib = 0, _maKetQuaCalibDaXuLy = 0;
    bool _dangChoCalib = false;
    bool _canLuuNvs = false;
    bool _inDuLieuImu = false;
    bool _batTelemetry = true;
    bool _batNghienCuu = false, _dangPhatNghienCuu = false;
    bool _canTieuDeNghienCuu = false, _canCauHinhNghienCuu = false;
    NhatKyNghienCuu _nhatKyNghienCuu;
    uint64_t _henMauNghienCuu_us = 0;
    uint32_t _soMauNghienCuuBo = 0, _thuTuDaGhi = 0, _phienBanDaGhi = 0;
    bool _cheDoMoDaGhi = false;
    bool batTelemetryWeb = false;
    uint32_t mocTelemetryWebTruocMs = 0u;
    char _boDemLenhSerial[96]{};
    size_t _doDaiLenhSerial = 0;
    uint32_t _lucNhanKyTuCuoi_ms = 0;

    QueueHandle_t _hangDoiImu = nullptr, _hangDoiTrangThaiDongCo = nullptr, _hangDoiTrangThaiDieuKhien = nullptr;
    QueueHandle_t _hangDoiDiChuyen = nullptr;
    QueueHandle_t _hangDoiChanDoanImu = nullptr; // Vẫn cập nhật khi các mẫu điều khiển bị bỏ.
    QueueHandle_t _hangDoiCauHinh = nullptr;
    QueueHandle_t _hangDoiYeuCauCalib = nullptr;
    QueueHandle_t _hangDoiKetQuaCalib = nullptr;
    TaskHandle_t _tacVuImu = nullptr, _tacVuDieuKhien = nullptr, _tacVuDongCo = nullptr;
    esp_timer_handle_t _boHenDongCo = nullptr;
    portMUX_TYPE _khoaDuLieuNgatImu = portMUX_INITIALIZER_UNLOCKED;
    uint64_t _thoiDiemNgat_us = 0;
    uint32_t _soThuTuNgat = 0;
    bool _daSanSang = false;
};
