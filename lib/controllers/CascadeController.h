#pragma once
#include "AnglePidController.h"
#include "BoDieuKhienBanhVongKin.h"
#include "model/RobotState.h"
#include "model/WheelStepCommand.h"

class CascadeController {
public:
    void reset();
    void chotViTriGiuTaiCho();
    bool update(const TrangThaiXe& trangThaiXe, uint64_t thoiDiemHienTai_us, bool dungPidMo,
                WheelStepCommand& lenhBanh, float vanTocYeuCau_mps = vanTocDat_mps,
                float tocDoQuayYeuCau_radps = 0);
    float gocDat_rad() const { return _gocDat_rad; }
    float gocBuTrongTam_rad() const { return _gocBuTrongTam_rad; }
    float viTriMucTieu_m() const { return _viTriMucTieu_m; }
    PidGains heSoPid() const { return _boPidGoc.heSoPid(); }
    PidGains pidGocThuan() const { return _boPidGoc.heSoPidThuan(); }
    PidGains pidGocMoCoSo() const { return _boPidGoc.heSoCoSoMo(); }
    void setPidGocThuan(const PidGains& heSo) { _boPidGoc.setHeSoPidThuan(heSo); }
    void setPidGocMoCoSo(const PidGains& heSo) { _boPidGoc.setHeSoCoSoMo(heSo); }
    void setPidVanTocThuan(float kpV, float kiV);
    float kpVanTocThuan() const { return _kpVanTocPidThuan; }
    float kiVanTocThuan() const { return _kiVanTocPidThuan; }
    void datPidVanTocMo(float kpVanTocMoi, float kiVanTocMoi);
    float kpVanTocMo() const { return kpVanTocPidMo; }
    float kiVanTocMo() const { return kiVanTocPidMo; }
    void setGocCanBang_rad(float goc_rad);
    float gocCanBangThucTe_rad() const { return _gocCanBang_rad; }
    void setBatVongNgoai(bool bat);
    bool batVongNgoai() const { return _batVongNgoai; }
    void setCheDoThuKhongMaSat(bool bat) { _cheDoThuKhongMaSat = bat; }
    bool cheDoThuKhongMaSat() const { return _cheDoThuKhongMaSat; }
    bool dangDiChuyenDoc() const { return dangDiChuyenDocTruoc; }
    float tocDoDaLoc_radps() const { return _boPidGoc.tocDoDaLoc_radps(); }
    float vanTocMucTieu_mps() const { return _vanTocMucTieu_mps; }
    float tocDoQuayMucTieu_radps() const { return _tocDoQuayMucTieu_radps; }
    // Gia tốc trung bình hai bánh THỰC SỰ đã lệnh ở lần update gần nhất (sau mọi giới hạn).
    float giaTocBanhDaLenh_mps2() const { return _giaTocBanhDaLenh_mps2; }
    const TrangThaiVongKinBanhXe& trangThaiVongKinBanh() const { return _boVongKinBanhXe.trangThai(); }
private:
    AnglePidController _boPidGoc;
    BoDieuKhienBanhVongKin _boVongKinBanhXe;
    uint64_t _thoiDiemTruoc_us = 0;
    float _thoiGianVongVanToc_s = 0, _tichPhanVanToc_m = 0, _gocDat_rad = 0, _vanTocLenh_mps = 0;
    float _viTriMucTieu_m = 0, _gocBuTrongTam_rad = 0;
    bool _daDongBoViTri = false;
    float _vanTocMucTieu_mps = 0, _tocDoQuayMucTieu_radps = 0, _giaTocBanhDaLenh_mps2 = 0;
    float _gocCanBang_rad = 0.0f;
    float _kpVanTocPidThuan = 0.15f;
    float _kiVanTocPidThuan = 0.08f;
    float kpVanTocPidMo = 0.15f;
    float kiVanTocPidMo = 0.06f;
    bool _batVongNgoai = true;
    bool _cheDoThuKhongMaSat = false;
    float hieuViTriHaiBanhMucTieu = 0.0f;
    bool daDongBoHuongQuay = false;
    bool dangDiChuyenDocTruoc = false;
};
