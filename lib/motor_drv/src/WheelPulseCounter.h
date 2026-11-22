#pragma once
#include "driver/pulse_cnt.h"
#include "model/WheelPulseFrame.h"
#include "core/BoGiaiMaUartMks.h"
#include "LichHoiUartMks.h"

// Khởi tạo SAU MCPWM khi motor disabled; đọc bởi duy nhất MotorTask.
// PCNT đếm STEP/DIR. Hai UART theo dõi encoder 0x30, quy đổi theo soXungEncoderMksMoiVong.
// Chỉ hợp nhất số đo UART vào PCNT khi cấu hình batBuTruotUartMksVaoXung bật.
class WheelPulseCounter {
public:
    bool begin();
    // Chuyển GPIO3 sang chân RX1 (UART1) đọc vòng hồi tiếp từ kế MKS sau giai đoạn khởi tạo.
    bool beginUartMks();
    // Tạm dừng giao tiếp UART MKS (ví dụ khi đang dùng nút nhấn GPIO3 để calib lúc motor tắt).
    void datTamDungUart(bool tamDung);
    void reset();
    WheelPulseFrame read();
    bool coPhanHoiUartTrai() const { return _banhTrai.daCoMauUart; }
    bool coPhanHoiUartPhai() const { return _banhPhai.daCoMauUart; }
    uint8_t diaChiUartTrai() const { return _banhTrai.diaChiHoatDong; }
    uint8_t diaChiUartPhai() const { return _banhPhai.diaChiHoatDong; }
private:
    struct Axis {
        pcnt_unit_handle_t boDem = nullptr;
        pcnt_channel_handle_t kenhDem = nullptr;
        uint32_t soDemTruoc = 0;
        int64_t tongSoXung = 0;
        // Trạng thái hợp nhất / chẩn đoán từ kế MKS Servo42C V1.1 qua UART:
        bool daCoMauUart = false;
        bool daKhoaDiaChi = false;
        uint8_t diaChiHoatDong = 0;
        uint8_t chiSoThuDiaChi = 0;
        int64_t xungEncoderGoc = 0;
        int64_t xungEncoderTuongDuongBuoc = 0;
        int64_t xungPcntLucHoi = 0;
        int64_t buTuKeMucTieu = 0;
        double buTuKeDaLoc = 0.0;
        int64_t buTuKe = 0;
        uint32_t soKhungUart = 0;
        uint32_t soLanHoiUart = 0;
        uint64_t thoiDiemHoiUart_us = 0;
        uint64_t thoiDiemNhanUart_us = 0;
    };
    bool beginAxis(Axis& trucBanh, int chanXung, int chanChieu, bool mucTienCao);
    bool readAxis(Axis& trucBanh);
    void resetAxis(Axis& trucBanh);
    void capNhatLocBuTuKe(Axis& trucBanh, double dt_s);
    void serviceUartMks();
    void docKenhUartMks(int congUart, Axis& trucBanh, uint8_t diaChiUuTien, uint8_t diaChiDuPhong,
                        bool chieuTienDuong);

    Axis _banhTrai, _banhPhai;
    LichHoiUartMks _lichHoiUart;
    uint32_t _soLoiGhiUart = 0;
    uint64_t _thoiDiemDocTruoc_us = 0;
    bool _uartSanSang = false;
    volatile bool _tamDungUart = false;
    bool _daSanSang = false;
};
