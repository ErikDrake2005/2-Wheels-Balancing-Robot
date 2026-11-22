#pragma once

// ============================================================================
// BỘ ĐIỀU KHIỂN VÒNG KÍN SISO CHUẨN (Single-Input Single-Output Controller)
// ============================================================================
// Thiết kế phục vụ điều khiển thực tế và trình bày báo cáo Lý thuyết Điều khiển SISO:
// - Ngõ vào duy nhất : sai lệch e(t) = r(t) - y(t)
// - Ngõ ra duy nhất  : tín hiệu điều khiển u(t)
// - Hàm truyền liên tục miền Laplace:
//     C(s) = U(s) / E(s) = Kp + Ki / s + (Kd * s) / (tau_d * s + 1)
// - Rời rạc hóa bằng phép biến đổi song tuyến Tustin (s = (2/Ts)*(1 - z^-1)/(1 + z^-1)):
//     u_P[k] = Kp * e[k]
//     u_I[k] = u_I[k-1] + Ki * (Ts / 2) * (e[k] + e[k-1])          (có khóa chống windup)
//     u_D[k] = ((2*tau_d - Ts)/(2*tau_d + Ts)) * u_D[k-1]
//            + (2*Kd / (2*tau_d + Ts)) * (e[k] - e[k-1])
//     u[k]   = clamp(u_P[k] + u_I[k] + u_D[k], -gioiHanRa, +gioiHanRa)
struct ThamSoSiso {
    float kp = 1.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float tauLocViPhan_s = 0.020f;
    float gioiHanTichPhan = 1.0f;
    float gioiHanNgora = 1.0f;
};

class BoDieuKhienSiso {
public:
    BoDieuKhienSiso() = default;
    explicit BoDieuKhienSiso(const ThamSoSiso& thamSo) : _thamSo(thamSo) {}

    void reset();
    void datThamSo(const ThamSoSiso& thamSo);
    const ThamSoSiso& thamSo() const { return _thamSo; }

    // Thực thi đúng quy luật SISO: nhận 1 sai lệch e[k], trả về 1 tín hiệu điều khiển u[k].
    float capNhat(float saiLech, float buocThoiGian_s);

    float ngoRaCuoi() const { return _ngoRaCuoi; }
    float thanhPhanTichPhan() const { return _tichPhan; }
    float thanhPhanViPhan() const { return _viPhan; }

private:
    ThamSoSiso _thamSo{};
    bool _daCoMauTruoc = false;
    float _saiLechTruoc = 0.0f;
    float _tichPhan = 0.0f;
    float _viPhan = 0.0f;
    float _ngoRaCuoi = 0.0f;
};
