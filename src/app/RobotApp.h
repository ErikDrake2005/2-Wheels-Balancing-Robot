#pragma once
#include <Arduino.h>
#include <atomic>
#include <type_traits>
#include "SelectedImuDriver.h"
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
#include "HieuChuanNghiengDoc.h"
#include "ThongKeMauImu.h"
#include "DieuKhienKhongDay.h"
#include "LienKetChanDoanEspNow.h"
#include "BoGhiRungEspNow.h"
#include "freertos/queue.h"
#include "esp_timer.h"

class RobotApp {
public:
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
    RobotApp() = default;
#else
    RobotApp() : _camBienGoc(Wire) {}
#endif
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
        bool dungMoSiso = false;
        CauHinhMoSiso cauHinhMoSiso{};
        // A/B trong RAM, mặc định giữ hành vi cũ; chỉ đổi khi motor tắt.
        bool buGiaTocImu = buGiaTocLenhVaoImu;
        bool tangGiaTocDiChuyen = dungLsm6dsv ? lsm6dsvTune::tangGiaToc : (dungMpu6050 ? mpu6050Tune::tangGiaToc : true);
        bool tangBamVanTocDiChuyen = dungLsm6dsv ? lsm6dsvTune::tangBamVanToc : (dungMpu6050 ? mpu6050Tune::tangBamVanToc : true);
        bool khoaMotorCalib = false;
        float normGiaTocTinh_mps2 = 9.80665f;
        MaTranQuayImu maTranLap{};
        Vec3Imu biasGyro_dps{};
        bool coBiasGyro = false;
        // Cùng một phiên bản cho bias trên ImuTask và hệ trục trên ControlTask.
        uint32_t phienBanMaTranLap = 1;
        uint32_t phienBan = 1;
        uint32_t soLanChotViTri = 0;
    };
    enum class PhaCalib2Truc : uint8_t {
        KhongHoatDong = 0,
        ChoDungYen = 1,
        DangDoDung = 2,
        ChoHaDau = 3,
        DangDoTruoc = 4,
        DangQuayPitch = 5,
        DangDoSau = 6,
        HoanTat = 7,
        Loi = 8
    };
    struct TrangThaiCalib2Truc {
        PhaCalib2Truc pha = PhaCalib2Truc::KhongHoatDong;
        bool dangYeuCauCalib = false;
        uint32_t phienYeuCau = 0, phienKetQua = 0, lenhXacNhan = 0;
        uint16_t soMauHienTai = 0, tongMauCan = 0;
        bool coKetQuaMoi = false, thanhCong = false;
        MaTranQuayImu maTranKetQua{};
        float biasX_dps = 0, biasY_dps = 0, biasZ_dps = 0;
        BaoCaoCalibDoc baoCao{};
        Vec3Imu aDung{}, aTruoc{}, aSau{};
        float normGiaTocTinh_mps2 = 9.80665f;
        char lyDoLoi[32] = "none";
    };
    struct BoLocNutCalib {
        int chan = -1;
        bool mucCu = false, mucOnDinh = false, daNhan = false;
        uint32_t lucDoi_ms = 0;
    };
    struct YeuCauCalibImu {
        uint32_t maYeuCau = 0;
        MaTranQuayImu maTranLap{};
        Vec3Imu huongTienSensor{};
        bool dungMotTuThe = false;
    };
    struct MauImuDaDoc {
        KhungDuLieuImu mau{};
        uint32_t phienBanMaTranLap = 0;
    };
    struct KetQuaCalibImu {
        uint32_t maYeuCau = 0;
        bool dungMotTuThe = false;
        MaTranQuayImu maTranLap{};
        bool hoanTat = false;
        bool thanhCong = false;
        float biasX_dps = 0.0f, biasY_dps = 0.0f, biasZ_dps = 0.0f;
        float gocCanBangDo_rad = 0.0f;
        float normGiaTocTinh_mps2 = 9.80665f;
        float doLechChuanGyro_dps = 0.0f;
        float phuongSaiGiaToc = 0.0f;
        uint16_t soMauDaThu = 0;
        uint16_t soLanThuLai = 0;
    };
    struct YeuCauTestBanh {
        uint32_t maYeuCau = 0;
        float tocDoTrai_hz = 0.0f;
        float tocDoPhai_hz = 0.0f;
        uint32_t thoiGian_ms = 1000u;
        char nhan[8]{};
    };
    struct KetQuaTestBanh {
        uint32_t maYeuCau = 0;
        bool hoanTat = false;
        bool biHuy = false;
        int64_t dStepTrai = 0, dStepPhai = 0;
        int64_t dEncTrai = 0, dEncPhai = 0;
        bool coUartTrai = false, coUartPhai = false;
        char nhan[8]{};
    };
    struct YeuCauBuocThu {
        uint32_t maYeuCau = 0;
        uint32_t maPhien = 0;
        uint8_t loai = 0; // 0=huy, 1=vstep, 2=angstep
        float giaTri = 0.0f; // m/s (vstep) hoac rad (angstep)
        uint32_t giu_ms = 2000u;
    };
    struct KetQuaBuocThu {
        uint32_t maYeuCau = 0;
        bool hoanTat = false;
        bool biHuy = false;
        uint8_t loai = 0;
        float giaTri = 0.0f;
    };

    struct MotorStatus {
        uint64_t thoiDiemSnapshot_us = 0;
        MksSnapshot trangThaiBoDieuKhienDongCo{};
        WheelPulseFrame khungXung{};
        FaultCode loi = FaultCode::None;
        MksResult ketQuaCuoi = MksResult::Disabled;
        bool dungPidMo = batPidMo;
        bool dungMoSiso = false;
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
        bool dungPidMo = false, dungMoSiso = false, bench = false, vongNgoai = false;
        uint8_t coThuNghiem = 0;
        PidGains heSoGocCoSo{};
        float kpVanTocCoSo = 0, kiVanTocCoSo = 0, giaTocLenh_mps2 = 0;
        uint64_t soMauBoLo = 0;
        uint32_t soThongBaoTrung = 0, soLanDocChongNgat = 0;
    };
    static void IRAM_ATTR imuInterrupt(void* nguCanh);
    static void imuPollTimer(void* nguCanh);
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
    bool luuCauHinhVaoNvs();
    void nhanLenhDiChuyenTuXa(float vanTocTienLui, float tocDoQuayNgang, const MucLenhWeb& mucWeb);
    void dienTrangThaiWeb(ThongTinTrangThaiWeb& trangThaiRa);
    void inDongSerial(const char* noiDung);
    void inDinhDangSerial(const char* dinhDang, ...);
    void ghiMauNghienCuu();
    void guiChanDoanEspNow();
    void capNhatThongTinLogRung();
    uint32_t tanSoImuHienTai_hz() const;
    void batDauCalib2Truc(bool tuDongKhoiDong = false);
    void batDauCalibDung(const Vec3Imu* huongTien);
    void xacNhanBuocCalib2Truc();
    void huyCalib2Truc();
    void xuLyCalib2TrucTrongLoop();
    void docNutVatLyCalib2Truc();
    void dongBoNutVatLyCalib();

    DieuKhienKhongDay dieuKhienKhongDay;
    LienKetChanDoanEspNow lienKetChanDoan;
    BoGhiRungEspNow logRung;

    SelectedImuDriver _camBienGoc;
    bool _coMaTranLapLsm = false;
    bool _batDauCalib2TrucKhiMo = false;
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
    uint8_t _cheDoDieuKhienDaGhi = 0;
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
    esp_timer_handle_t _boHenImu = nullptr;
    portMUX_TYPE _khoaDuLieuNgatImu = portMUX_INITIALIZER_UNLOCKED;
    portMUX_TYPE _khoaCalib2Truc = portMUX_INITIALIZER_UNLOCKED;
    portMUX_TYPE _khoaTestBanh = portMUX_INITIALIZER_UNLOCKED;
    YeuCauTestBanh _yeuCauTestBanh{};
    KetQuaTestBanh _ketQuaTestBanh{};
    uint32_t _maKetQuaTestBanhDaIn = 0;
    bool _dangChoTestBanh = false;
    portMUX_TYPE _khoaBuocThu = portMUX_INITIALIZER_UNLOCKED;
    YeuCauBuocThu _yeuCauBuocThu{};
    KetQuaBuocThu _ketQuaBuocThu{};
    uint32_t _maKetQuaBuocThuDaIn = 0;
    bool _dangChoBuocThu = false;
    TrangThaiCalib2Truc _calib2Truc{};
    HieuChuanNghiengDoc _luotCalibDoc;
    ThongKeMauImu _thongKeCalib2Truc;
    BoLocNutCalib _nutCalibChinh{}, _nutCalibPhu{};
    PhaCalib2Truc _phaCalib2TrucDaIn = PhaCalib2Truc::KhongHoatDong;
    uint64_t _thoiDiemNgat_us = 0;
    uint32_t _soThuTuNgat = 0;
    std::atomic<bool> _motorDangBat{false};
    bool _daSanSang = false;
};
