#include "WheelPulseCounter.h"
#include "config/MotorConfig.h"
#include "config/PinConfig.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr uart_port_t congUartMksTrai = UART_NUM_1;
constexpr uart_port_t congUartMksPhai = UART_NUM_2;
constexpr int kichThuocBoDemUart = 256;
constexpr uint32_t soXungBuocMoiVongMks =
    static_cast<uint32_t>(soBuocDayDuMoiVongDongCo) * static_cast<uint32_t>(soViBuocDauVaoMks);
}

bool WheelPulseCounter::beginAxis(Axis& trucBanh, int chanXung, int chanChieu, bool mucTienCao) {
    pcnt_unit_config_t cauHinhBoDem{};
    cauHinhBoDem.low_limit = -30000;
    cauHinhBoDem.high_limit = 30000;
    cauHinhBoDem.flags.accum_count = true;
    if (pcnt_new_unit(&cauHinhBoDem, &trucBanh.boDem) != ESP_OK) return false;
    pcnt_chan_config_t cauHinhKenhDem{};
    cauHinhKenhDem.edge_gpio_num = chanXung;
    cauHinhKenhDem.level_gpio_num = chanChieu;
    // Giữ output đã cấu hình bởi MCPWM/GPIO và bật thêm input path.
    if (pcnt_new_channel(trucBanh.boDem, &cauHinhKenhDem, &trucBanh.kenhDem) != ESP_OK ||
        gpio_input_enable(static_cast<gpio_num_t>(chanXung)) != ESP_OK ||
        gpio_input_enable(static_cast<gpio_num_t>(chanChieu)) != ESP_OK ||
        pcnt_channel_set_edge_action(trucBanh.kenhDem, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                    PCNT_CHANNEL_EDGE_ACTION_HOLD) != ESP_OK) return false;
    const auto giuChieuDem = PCNT_CHANNEL_LEVEL_ACTION_KEEP;
    const auto daoChieuDem = PCNT_CHANNEL_LEVEL_ACTION_INVERSE;
    if (pcnt_channel_set_level_action(trucBanh.kenhDem, mucTienCao ? giuChieuDem : daoChieuDem,
                                     mucTienCao ? daoChieuDem : giuChieuDem) != ESP_OK ||
        pcnt_unit_add_watch_point(trucBanh.boDem, cauHinhBoDem.high_limit) != ESP_OK ||
        pcnt_unit_add_watch_point(trucBanh.boDem, cauHinhBoDem.low_limit) != ESP_OK ||
        pcnt_unit_enable(trucBanh.boDem) != ESP_OK ||
        pcnt_unit_clear_count(trucBanh.boDem) != ESP_OK ||
        pcnt_unit_start(trucBanh.boDem) != ESP_OK) return false;
    return true;
}

bool WheelPulseCounter::beginUartMks() {
    if (_uartSanSang) return true;
    if (!batUartEncoderMks || chanUartTxMks < 0 || chanUartRxMksTrai < 0 || chanUartRxMksPhai < 0) {
        return true;
    }
    uart_config_t cauHinhUart{};
    cauHinhUart.baud_rate = static_cast<int>(tocDoBaudUartMks_bps);
    cauHinhUart.data_bits = UART_DATA_8_BITS;
    cauHinhUart.parity = UART_PARITY_DISABLE;
    cauHinhUart.stop_bits = UART_STOP_BITS_1;
    cauHinhUart.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cauHinhUart.source_clk = UART_SCLK_DEFAULT;

    // UART1: sở hữu GPIO1 (TX chung phát lệnh tới cả 2 MKS) và GPIO3 (RX1 nhận từ MKS 1).
    if (uart_param_config(congUartMksTrai, &cauHinhUart) != ESP_OK ||
        uart_set_pin(congUartMksTrai, chanUartTxMks, chanUartRxMksTrai,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK ||
        uart_driver_install(congUartMksTrai, kichThuocBoDemUart, kichThuocBoDemUart, 0, nullptr, 0) != ESP_OK) {
        return false;
    }

    // UART2: chỉ nhận trên GPIO4 (RX2 nhận từ MKS 2), không chiếm chân TX.
    if (uart_param_config(congUartMksPhai, &cauHinhUart) != ESP_OK ||
        uart_set_pin(congUartMksPhai, UART_PIN_NO_CHANGE, chanUartRxMksPhai,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK ||
        uart_driver_install(congUartMksPhai, kichThuocBoDemUart, 0, 0, nullptr, 0) != ESP_OK) {
        uart_driver_delete(congUartMksTrai);
        return false;
    }

    gpio_pulldown_dis(static_cast<gpio_num_t>(chanUartRxMksTrai));
    gpio_pullup_en(static_cast<gpio_num_t>(chanUartRxMksTrai));
    gpio_pulldown_dis(static_cast<gpio_num_t>(chanUartRxMksPhai));
    gpio_pullup_en(static_cast<gpio_num_t>(chanUartRxMksPhai));
    uart_flush_input(congUartMksTrai);
    uart_flush_input(congUartMksPhai);

    _uartSanSang = true;
    return true;
}

void WheelPulseCounter::datTamDungUart(bool tamDung) {
    if (_tamDungUart == tamDung) return;
    _tamDungUart = tamDung;
    if (!tamDung && _uartSanSang) {
        gpio_pulldown_dis(static_cast<gpio_num_t>(chanUartRxMksTrai));
        gpio_pullup_en(static_cast<gpio_num_t>(chanUartRxMksTrai));
        gpio_pulldown_dis(static_cast<gpio_num_t>(chanUartRxMksPhai));
        gpio_pullup_en(static_cast<gpio_num_t>(chanUartRxMksPhai));
        uart_flush_input(congUartMksTrai);
        uart_flush_input(congUartMksPhai);
    }
}

bool WheelPulseCounter::begin() {
    if (_daSanSang || _banhTrai.boDem || _banhPhai.boDem) return false;
    // Chỉ khởi tạo PCNT trước để giữ nguyên GPIO3 làm nút xác nhận trong suốt quá trình Calib ICM20602.
    // Ngay sau khi Calib hoàn tất, RobotApp::begin() sẽ gọi beginUartMks() để chuyển GPIO3 sang UART1 RX.
    _daSanSang = beginAxis(_banhTrai, chanStepTrai, chanDirTrai, mucDirTienBanhTraiCao) &&
                 beginAxis(_banhPhai, chanStepPhai, chanDirPhai, mucDirTienBanhPhaiCao);
    if (!_daSanSang) return false;
    // Kiểm tra đọc thử PCNT ngay sau khởi tạo khi EN vẫn đang tắt (mục M3).
    const WheelPulseFrame khungKiemTra = read();
    _daSanSang = khungKiemTra.hopLe && khungKiemTra.trai == 0 && khungKiemTra.phai == 0;
    return _daSanSang; // Khởi tạo lỗi: giữ motor disabled, reset sau khi sửa.
}

bool WheelPulseCounter::readAxis(Axis& trucBanh) {
    int soDemDocDuoc = 0;
    if (pcnt_unit_get_count(trucBanh.boDem, &soDemDocDuoc) != ESP_OK) return false;
    const uint32_t soDemHienTai = static_cast<uint32_t>(soDemDocDuoc);
    const uint32_t chenhLechSoDem = soDemHienTai - trucBanh.soDemTruoc;
    trucBanh.tongSoXung += chenhLechSoDem < 0x80000000u ? static_cast<int64_t>(chenhLechSoDem)
                                 : static_cast<int64_t>(chenhLechSoDem) - 0x100000000LL;
    trucBanh.soDemTruoc = soDemHienTai;
    return true;
}

void WheelPulseCounter::resetAxis(Axis& trucBanh) {
    if (trucBanh.boDem) {
        pcnt_unit_clear_count(trucBanh.boDem);
    }
    trucBanh.soDemTruoc = 0;
    trucBanh.tongSoXung = 0;
    trucBanh.daCoMauUart = false;
    trucBanh.xungEncoderGoc = 0;
    trucBanh.xungEncoderTuongDuongBuoc = 0;
    trucBanh.xungPcntLucHoi = 0;
    trucBanh.soKhungUart = 0;
    trucBanh.soLanHoiUart = 0;
    trucBanh.thoiDiemHoiUart_us = 0;
    trucBanh.thoiDiemNhanUart_us = 0;
    trucBanh.buTuKeMucTieu = 0;
    trucBanh.buTuKeDaLoc = 0.0;
    trucBanh.buTuKe = 0;
}

void WheelPulseCounter::reset() {
    if (!_daSanSang) return;
    resetAxis(_banhTrai);
    resetAxis(_banhPhai);
    _lichHoiUart.reset();
    _soLoiGhiUart = 0;
    _thoiDiemDocTruoc_us = 0;
    if (_uartSanSang) {
        uart_flush_input(congUartMksTrai);
        uart_flush_input(congUartMksPhai);
    }
}

namespace {
uint8_t chonDiaChiHoiMks(bool daKhoa, uint8_t diaChiHoatDong, uint8_t& chiSoThu,
                         uint8_t uuTien, uint8_t duPhong) {
    if (daKhoa && diaChiHoatDong != 0) return diaChiHoatDong;
    const uint8_t danhSach[3] = {uuTien, duPhong, static_cast<uint8_t>(0xE0)};
    const uint8_t ketQua = danhSach[chiSoThu % 3u];
    chiSoThu = static_cast<uint8_t>((chiSoThu + 1u) % 3u);
    return ketQua;
}
} // namespace

void WheelPulseCounter::docKenhUartMks(int congUart, Axis& trucBanh,
                                       uint8_t diaChiUuTien, uint8_t diaChiDuPhong,
                                       bool chieuTienDuong) {
    const auto port = static_cast<uart_port_t>(congUart);
    size_t soByteCoSan = 0;
    if (uart_get_buffered_data_len(port, &soByteCoSan) != ESP_OK || soByteCoSan < 8u) return;

    uint8_t boDem[64]{};
    const int soByteDoc = uart_read_bytes(
        port, boDem, soByteCoSan > sizeof(boDem) ? sizeof(boDem) : soByteCoSan, 0);
    if (soByteDoc < 8) return;

    const uint8_t ungVien[3] = {diaChiUuTien, diaChiDuPhong, static_cast<uint8_t>(0xE0)};
    const unsigned soUngVien = trucBanh.daKhoaDiaChi ? 1u : 3u;

    // Tìm khung trả lời 0x30 hợp lệ mới nhất trong bộ đệm FIFO.
    for (int viTri = soByteDoc - 8; viTri >= 0; --viTri) {
        for (unsigned i = 0; i < soUngVien; ++i) {
            const uint8_t diaChiThu = trucBanh.daKhoaDiaChi ? trucBanh.diaChiHoatDong : ungVien[i];
            MauEncoderMksUart mau = BoGiaiMaUartMks::giaiMaEncoder0x30(
                &boDem[viTri], 8u, diaChiThu, soXungEncoderMksMoiVong);
            if (!mau.hopLe && (soByteDoc - viTri) >= 9) {
                mau = BoGiaiMaUartMks::giaiMaEncoder0x30(
                    &boDem[viTri], 9u, diaChiThu, soXungEncoderMksMoiVong);
            }
            if (mau.hopLe) {
                if (!trucBanh.daKhoaDiaChi) {
                    trucBanh.daKhoaDiaChi = true;
                    trucBanh.diaChiHoatDong = diaChiThu;
                }
                ++trucBanh.soKhungUart;
                trucBanh.thoiDiemNhanUart_us = static_cast<uint64_t>(esp_timer_get_time());
                const int64_t xungCoHuong = chieuTienDuong ? mau.tongXungEncoder : -mau.tongXungEncoder;
                if (!trucBanh.daCoMauUart) {
                    trucBanh.xungEncoderGoc = xungCoHuong;
                    trucBanh.xungEncoderTuongDuongBuoc = 0;
                    trucBanh.buTuKeMucTieu = 0;
                    trucBanh.buTuKeDaLoc = 0.0;
                    trucBanh.buTuKe = 0;
                    trucBanh.daCoMauUart = true;
                } else {
                    const int64_t deltaEncoder = xungCoHuong - trucBanh.xungEncoderGoc;
                    const int64_t xungTuKeQuyDoi = BoGiaiMaUartMks::quyDoiSangXungBuoc(
                        deltaEncoder, soXungBuocMoiVongMks, soXungEncoderMksMoiVong);
                    trucBanh.xungEncoderTuongDuongBuoc = xungTuKeQuyDoi;
                    if (batBuTruotUartMksVaoXung) {
                        trucBanh.buTuKeMucTieu = xungTuKeQuyDoi - trucBanh.tongSoXung;
                    }
                }
                return;
            }
        }
    }
}

void WheelPulseCounter::serviceUartMks() {
    if (!_uartSanSang || _tamDungUart) return;
    docKenhUartMks(congUartMksTrai, _banhTrai, diaChiUartMksTrai, diaChiUartMksPhai, chieuTienEncoderMksTraiDuong);
    docKenhUartMks(congUartMksPhai, _banhPhai, diaChiUartMksPhai, diaChiUartMksTrai, chieuTienEncoderMksPhaiDuong);

    const auto luot = _lichHoiUart.chonLuot(static_cast<uint64_t>(esp_timer_get_time()));
    if (luot == LichHoiUartMks::Luot::KhongHoi) return;
    const bool hoiTrai = luot == LichHoiUartMks::Luot::Trai;
    Axis& banh = hoiTrai ? _banhTrai : _banhPhai;
    const uint8_t diaChi = chonDiaChiHoiMks(banh.daKhoaDiaChi, banh.diaChiHoatDong, banh.chiSoThuDiaChi,
        hoiTrai ? diaChiUartMksTrai : diaChiUartMksPhai, hoiTrai ? diaChiUartMksPhai : diaChiUartMksTrai);
    uint8_t khungHoi[3]{};
    BoGiaiMaUartMks::taoKhungHoi(diaChi, BoGiaiMaUartMks::maLenhDocEncoderCarry, khungHoi);
    const int daGhi = uart_write_bytes(congUartMksTrai, khungHoi, sizeof(khungHoi));
    const uint64_t ghiXong_us = static_cast<uint64_t>(esp_timer_get_time());
    const bool ghiDuLenh = daGhi == static_cast<int>(sizeof(khungHoi));
    _lichHoiUart.ghiNhanKetQua(ghiXong_us, ghiDuLenh);
    if (ghiDuLenh) {
        ++banh.soLanHoiUart;
        banh.thoiDiemHoiUart_us = ghiXong_us;
        banh.xungPcntLucHoi = banh.tongSoXung;
    } else {
        ++_soLoiGhiUart;
    }
}

void WheelPulseCounter::capNhatLocBuTuKe(Axis& trucBanh, double dt_s) {
    if (!batBuTruotUartMksVaoXung || !trucBanh.daCoMauUart || dt_s <= 0.0) return;
    const double alpha = dt_s / (static_cast<double>(hangSoLocBuUartMks_s) + dt_s);
    double buMoi = trucBanh.buTuKeDaLoc +
        alpha * (static_cast<double>(trucBanh.buTuKeMucTieu) - trucBanh.buTuKeDaLoc);
    const double buocToiDa = static_cast<double>(tocDoBuUartToiDa_buocTrenGiay) * dt_s;
    buMoi = std::clamp(buMoi, trucBanh.buTuKeDaLoc - buocToiDa, trucBanh.buTuKeDaLoc + buocToiDa);
    trucBanh.buTuKeDaLoc = buMoi;
    trucBanh.buTuKe = static_cast<int64_t>(std::llround(trucBanh.buTuKeDaLoc));
}

WheelPulseFrame WheelPulseCounter::read() {
    WheelPulseFrame khungDuLieu{};
    if (!_daSanSang || !readAxis(_banhTrai) || !readAxis(_banhPhai)) return khungDuLieu;
    const uint64_t thoiDiem_us = static_cast<uint64_t>(esp_timer_get_time());
    serviceUartMks();
    double dt_s = 0.0;
    if (_thoiDiemDocTruoc_us != 0 && thoiDiem_us > _thoiDiemDocTruoc_us) {
        dt_s = std::clamp(static_cast<double>(thoiDiem_us - _thoiDiemDocTruoc_us) * 1e-6, 0.0, 0.05);
    }
    _thoiDiemDocTruoc_us = thoiDiem_us;
    capNhatLocBuTuKe(_banhTrai, dt_s);
    capNhatLocBuTuKe(_banhPhai, dt_s);
    khungDuLieu.trai = _banhTrai.tongSoXung + _banhTrai.buTuKe;
    khungDuLieu.phai = _banhPhai.tongSoXung + _banhPhai.buTuKe;
    khungDuLieu.thoiDiem_us = thoiDiem_us;
    khungDuLieu.encoderTrai = _banhTrai.xungEncoderTuongDuongBuoc;
    khungDuLieu.encoderPhai = _banhPhai.xungEncoderTuongDuongBuoc;
    khungDuLieu.soKhungUartTrai = _banhTrai.soKhungUart;
    khungDuLieu.soKhungUartPhai = _banhPhai.soKhungUart;
    khungDuLieu.diaChiUartTrai = _banhTrai.diaChiHoatDong;
    khungDuLieu.diaChiUartPhai = _banhPhai.diaChiHoatDong;
    khungDuLieu.coUartTrai = _banhTrai.daCoMauUart;
    khungDuLieu.coUartPhai = _banhPhai.daCoMauUart;
    khungDuLieu.soLanHoiUartTrai = _banhTrai.soLanHoiUart;
    khungDuLieu.soLanHoiUartPhai = _banhPhai.soLanHoiUart;
    khungDuLieu.soLoiGhiUart = _soLoiGhiUart;
    khungDuLieu.soKheUartBoLo = _lichHoiUart.soKheBoLo();
    khungDuLieu.thoiDiemHoiUartTrai_us = _banhTrai.thoiDiemHoiUart_us;
    khungDuLieu.thoiDiemHoiUartPhai_us = _banhPhai.thoiDiemHoiUart_us;
    khungDuLieu.thoiDiemNhanUartTrai_us = _banhTrai.thoiDiemNhanUart_us;
    khungDuLieu.thoiDiemNhanUartPhai_us = _banhPhai.thoiDiemNhanUart_us;
    khungDuLieu.hopLe = true;
    return khungDuLieu;
}
