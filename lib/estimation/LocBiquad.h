#pragma once
#include <cmath>
#include <cstdint>
#include <limits>

// Bộ lọc số IIR Biquad SISO bậc 2 (Direct Form II Transposed):
//   H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
// Hỗ trợ hai cấu hình khử nhiễu & khử rung động cơ cho xe tự cân bằng:
// 1. Butterworth thông thấp bậc 2 (-40 dB/decade, biến đổi song tuyến Tustin có pre-warping).
// 2. Bộ lọc khía Notch (Band-Stop) bậc 2 triệt tần số cộng hưởng cơ khí/vi bước.
class LocBiquadSiso {
public:
    enum class LoaiLoc : uint8_t {
        ThongThapButterworth = 0,
        KhiaNotch = 1
    };

    explicit LocBiquadSiso(LoaiLoc loai = LoaiLoc::ThongThapButterworth) : _loai(loai) {}

    void datLai() {
        _daKhoiTaoMau = false;
        _daCoHeSo = false;
        _z1 = 0.0f;
        _z2 = 0.0f;
        _giaTri = 0.0f;
    }

    float capNhatThongThap(float mauVao, float buocThoiGian_s, float tanSoCat_hz) {
        _loai = LoaiLoc::ThongThapButterworth;
        return capNhatNoiBo(mauVao, buocThoiGian_s, tanSoCat_hz, 0.70710678f);
    }

    float capNhatNotch(float mauVao, float buocThoiGian_s, float tanSoTam_hz, float heSoPhamChatQ) {
        _loai = LoaiLoc::KhiaNotch;
        return capNhatNoiBo(mauVao, buocThoiGian_s, tanSoTam_hz, heSoPhamChatQ);
    }

    float giaTri() const { return _giaTri; }

private:
    LoaiLoc _loai = LoaiLoc::ThongThapButterworth;
    bool _daKhoiTaoMau = false;
    bool _daCoHeSo = false;
    float _b0 = 1.0f, _b1 = 0.0f, _b2 = 0.0f;
    float _a1 = 0.0f, _a2 = 0.0f;
    float _z1 = 0.0f, _z2 = 0.0f;
    float _giaTri = 0.0f;
    float _dtCu_s = 0.0f, _tanSoCu_hz = -1.0f, _qCu = -1.0f;

    void tinhHeSo(float buocThoiGian_s, float tanSo_hz, float q) {
        const float tanSoLayMau_hz = 1.0f / buocThoiGian_s;
        // Chặn an toàn dưới tần số Nyquist (0.45 * fs) để tránh cực suy biến sát z = -1.
        const float fGioiHan_hz = (tanSo_hz < 0.45f * tanSoLayMau_hz)
            ? tanSo_hz
            : (0.45f * tanSoLayMau_hz);
        constexpr float soPi = 3.14159265358979323846f;

        if (_loai == LoaiLoc::ThongThapButterworth) {
            constexpr float canHai = 1.4142135623730951f;
            const float k = std::tan(soPi * fGioiHan_hz * buocThoiGian_s);
            const float k2 = k * k;
            const float chuanHoa = 1.0f / (1.0f + canHai * k + k2);
            _b0 = k2 * chuanHoa;
            _b1 = 2.0f * _b0;
            _b2 = _b0;
            _a1 = 2.0f * (k2 - 1.0f) * chuanHoa;
            _a2 = (1.0f - canHai * k + k2) * chuanHoa;
        } else {
            const float qHopLe = (q > 0.1f) ? q : 0.1f;
            const float w0 = 2.0f * soPi * fGioiHan_hz * buocThoiGian_s;
            const float cosW0 = std::cos(w0);
            const float alpha = std::sin(w0) / (2.0f * qHopLe);
            const float chuanHoa = 1.0f / (1.0f + alpha);
            _b0 = chuanHoa;
            _b1 = -2.0f * cosW0 * chuanHoa;
            _b2 = chuanHoa;
            _a1 = -2.0f * cosW0 * chuanHoa;
            _a2 = (1.0f - alpha) * chuanHoa;
        }
        _dtCu_s = buocThoiGian_s;
        _tanSoCu_hz = tanSo_hz;
        _qCu = q;
        _daCoHeSo = true;
    }

    float capNhatNoiBo(float mauVao, float buocThoiGian_s, float tanSo_hz, float q) {
        if (!std::isfinite(mauVao) || !std::isfinite(buocThoiGian_s) ||
            !std::isfinite(tanSo_hz) || !std::isfinite(q) ||
            buocThoiGian_s <= 0.0f || tanSo_hz < 0.0f) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        // Tần số = 0 Hz -> chế độ đi thẳng (bypass) không lọc.
        if (tanSo_hz == 0.0f) {
            _giaTri = mauVao;
            _daKhoiTaoMau = true;
            return _giaTri;
        }
        if (!_daCoHeSo ||
            std::fabs(buocThoiGian_s - _dtCu_s) > 1e-5f ||
            std::fabs(tanSo_hz - _tanSoCu_hz) > 1e-3f ||
            std::fabs(q - _qCu) > 1e-3f) {
            tinhHeSo(buocThoiGian_s, tanSo_hz, q);
        }
        if (!_daKhoiTaoMau) {
            // Khởi tạo trạng thái xác lập DC (DC gain = 1) để tránh gai đạo hàm mẫu đầu tiên.
            _giaTri = mauVao;
            _z2 = (_b2 - _a2) * mauVao;
            _z1 = (_b1 - _a1) * mauVao + _z2;
            _daKhoiTaoMau = true;
            return _giaTri;
        }
        const float mauRa = _b0 * mauVao + _z1;
        _z1 = _b1 * mauVao - _a1 * mauRa + _z2;
        _z2 = _b2 * mauVao - _a2 * mauRa;
        _giaTri = mauRa;
        return _giaTri;
    }
};
