#pragma once
#include <cstdint>
#include "config/MotorConfig.h"

class LichHoiUartMks {
public:
    enum class Luot { KhongHoi, Trai, Phai };

    static constexpr uint32_t chuKyKhe_us = chuKyDocUartMks_us / 2;
    // UART 8N1: lệnh 3 byte, phản hồi dài nhất mà parser nhận là 9 byte.
    static constexpr uint32_t thoiGianTrenDay_us =
        ((3u + 9u) * 10u * 1000000u + tocDoBaudUartMks_bps - 1u) / tocDoBaudUartMks_bps;
    // Dành tối thiểu 2 ms giữa hai yêu cầu để driver còn thời gian xử lý sau phần truyền trên dây.
    static constexpr uint32_t khoangGhiToiThieu_us = thoiGianTrenDay_us < 2000u ? 2000u : thoiGianTrenDay_us;
    static_assert(chuKyDocUartMks_us % 2 == 0 && chuKyKhe_us >= khoangGhiToiThieu_us,
                  "Khe UART khong du dai cho mot luot hoi va dap");

    Luot chonLuot(uint64_t hienTai_us) {
        if (_dangGhi || hienTai_us < _duocGhiTiep_us) return Luot::KhongHoi;
        if (!_daKhoiTao) {
            _mocKhe_us = hienTai_us; // Lượt trái đầu tiên đi ngay.
            _daKhoiTao = true;
        } else {
            const uint64_t soKheDaQua = (hienTai_us - _mocKhe_us) / chuKyKhe_us;
            if (soKheDaQua == 0) return Luot::KhongHoi;
            // Tiến đến khe mới nhất, bỏ các khe cũ trong O(1); mốc luôn bám lưới ban đầu.
            _mocKhe_us += soKheDaQua * chuKyKhe_us;
            _soKheBoLo += soKheDaQua - 1;
        }
        _dangGhi = true;
        return _tiepTheo;
    }

    // Guard tính từ sau uart_write_bytes để cả khoảng bị preempt và lần ghi thiếu byte đều được bảo vệ.
    void ghiNhanKetQua(uint64_t ghiXong_us, bool daGuiDuLenh) {
        _duocGhiTiep_us = ghiXong_us + khoangGhiToiThieu_us;
        _dangGhi = false;
        if (daGuiDuLenh) _tiepTheo = _tiepTheo == Luot::Trai ? Luot::Phai : Luot::Trai;
    }
    void reset() {
        // Zero odometry vẫn dùng cùng đồng hồ MCU; giữ hạn chờ của giao dịch vừa thực hiện.
        _mocKhe_us = 0;
        _soKheBoLo = 0;
        _tiepTheo = Luot::Trai;
        _daKhoiTao = false;
    }
    uint64_t soKheBoLo() const { return _soKheBoLo; }

private:
    uint64_t _mocKhe_us = 0;
    uint64_t _duocGhiTiep_us = 0;
    uint64_t _soKheBoLo = 0;
    Luot _tiepTheo = Luot::Trai;
    bool _daKhoiTao = false;
    bool _dangGhi = false;
};
