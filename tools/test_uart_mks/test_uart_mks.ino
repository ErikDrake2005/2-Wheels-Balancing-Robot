/*
 ==============================================================================
  CHƯƠNG TRÌNH KIỂM TRA GIAO TIẾP DUAL-UART MKS SERVO42C V1.1 (ESP32-S3)
 ==============================================================================
  Sơ đồ kết nối phần cứng 3 chân UART:
  - ESP32-S3 GPIO1 (TX chung)  --> Nối chân RX của CẢ HAI module MKS 1 & MKS 2
  - ESP32-S3 GPIO3 (RX1)       --> Nối chân TX của MKS 1 (Bánh Trái)
  - ESP32-S3 GPIO4 (RX2)       --> Nối chân TX của MKS 2 (Bánh Phải)
  - ESP32-S3 GND               --> Nối chung GND với nguồn cấp của 2 module MKS!

  Cấu hình menu trên màn hình MKS Servo42C V1.1 (nhấn giữ nút xoay):
  - Mode     : CR_vFOC
  - Baud     : 115200
  - UartAddr : 0xE0 (cả 2 module để chung 0xE0, vì 2 chân RX1/RX2 đã tách biệt)
  - MStep    : 32 (nếu dùng chung bước điều khiển)
 ==============================================================================
*/

#include <Arduino.h>
#include "driver/uart.h"
#include "esp_timer.h"

// ----------------------------------------------------------------------------
// 1. CẤU HÌNH CHÂN VÀ THAM SỐ GIAO TIẾP
// ----------------------------------------------------------------------------
constexpr int PIN_UART_TX_CHUNG = 1; // GPIO1: TX phát lệnh tới cả 2 MKS
constexpr int PIN_UART_RX_MKS1   = 3; // GPIO3: RX1 nhận từ MKS 1 (Trái)
constexpr int PIN_UART_RX_MKS2   = 4; // GPIO4: RX2 nhận từ MKS 2 (Phải)

constexpr uart_port_t PORT_UART_MKS1 = UART_NUM_1;
constexpr uart_port_t PORT_UART_MKS2 = UART_NUM_2;

constexpr uint32_t BAUDRATE_MKS = 115200;
constexpr uint8_t DIA_CHI_MKS   = 0xE0;
constexpr int UART_BUF_SIZE     = 256;

constexpr uint32_t SO_XUNG_ENCODER_MOI_VONG = 16384; // 14-bit MKS V1.1
constexpr uint32_t SO_XUNG_BUOC_MOI_VONG    = 6400;  // 200 bước * 32 vi bước

// ----------------------------------------------------------------------------
// 2. CẤU TRÚC DỮ LIỆU VÀ GIẢI MÃ KHUNG TRUYỀN (LỆNH 0x30)
// ----------------------------------------------------------------------------
struct KetQuaMksUart {
    int32_t carry = 0;          // Số vòng nguyên (âm / dương)
    uint16_t value = 0;         // Góc tuyệt đối trong 1 vòng (0..16383)
    int64_t tongXung = 0;       // Tổng xung encoder 14-bit: carry*16384 + value
    float gocDo = 0.0f;         // Góc trong 1 vòng (0.00° .. 359.98°)
    float tongVong = 0.0f;      // Tổng số vòng quay (dạng số thực)
    int64_t xungViBuoc = 0;     // Quy đổi tương đương vi bước 1/32 (6400 xung/vòng)
    uint8_t rawBytes[16]{};     // Lưu chuỗi byte thô nhận được
    uint8_t soByteNhan = 0;
    bool hopLe = false;
    const char* thongBaoLoi = "OK";
};

// Tính checksum CRC8: tổng các byte trước & 0xFF
uint8_t tinhCrc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc = static_cast<uint8_t>(crc + data[i]);
    }
    return crc;
}

// Giải mã khung trả lời lệnh 0x30 từ buffer FIFO
KetQuaMksUart giaiMaKhung0x30(const uint8_t* buf, int len, uint8_t diaChiMongDoi) {
    KetQuaMksUart res;
    res.soByteNhan = static_cast<uint8_t>(min(len, 16));
    memcpy(res.rawBytes, buf, res.soByteNhan);

    if (len < 8) {
        res.hopLe = false;
        res.thongBaoLoi = (len == 0) ? "TIMEOUT_KHONG_CO_DU_LIEU" : "KHUNG_CHUA_DU_8_BYTE";
        return res;
    }

    // Tìm vị trí frame hợp lệ (hỗ trợ cả frame 8-byte và 9-byte)
    for (int i = len - 8; i >= 0; --i) {
        const uint8_t* p = &buf[i];
        if (p[0] != diaChiMongDoi) continue;

        // Thử dạng chuẩn 8-byte: [E0, carry_B3, carry_B2, carry_B1, carry_B0, val_H, val_L, CRC]
        uint8_t crc8 = tinhCrc8(p, 7);
        if (crc8 == p[7]) {
            uint32_t cU = (static_cast<uint32_t>(p[1]) << 24) |
                          (static_cast<uint32_t>(p[2]) << 16) |
                          (static_cast<uint32_t>(p[3]) << 8)  |
                          static_cast<uint32_t>(p[4]);
            res.carry = static_cast<int32_t>(cU);
            res.value = (static_cast<uint16_t>(p[5]) << 8) | static_cast<uint16_t>(p[6]);

            if (res.value < SO_XUNG_ENCODER_MOI_VONG) {
                res.tongXung = static_cast<int64_t>(res.carry) * SO_XUNG_ENCODER_MOI_VONG + res.value;
                res.gocDo = (static_cast<float>(res.value) * 360.0f) / SO_XUNG_ENCODER_MOI_VONG;
                res.tongVong = static_cast<float>(res.carry) + static_cast<float>(res.value) / SO_XUNG_ENCODER_MOI_VONG;
                res.xungViBuoc = (res.tongXung * SO_XUNG_BUOC_MOI_VONG) / SO_XUNG_ENCODER_MOI_VONG;
                res.hopLe = true;
                res.thongBaoLoi = "OK_8BYTE";
                return res;
            }
        }

        // Thử dạng 9-byte: [E0, 0x30, carry_B3, carry_B2, carry_B1, carry_B0, val_H, val_L, CRC]
        if ((len - i) >= 9 && p[1] == 0x30) {
            uint8_t crc9 = tinhCrc8(p, 8);
            if (crc9 == p[8]) {
                uint32_t cU = (static_cast<uint32_t>(p[2]) << 24) |
                              (static_cast<uint32_t>(p[3]) << 16) |
                              (static_cast<uint32_t>(p[4]) << 8)  |
                              static_cast<uint32_t>(p[5]);
                res.carry = static_cast<int32_t>(cU);
                res.value = (static_cast<uint16_t>(p[6]) << 8) | static_cast<uint16_t>(p[7]);

                if (res.value < SO_XUNG_ENCODER_MOI_VONG) {
                    res.tongXung = static_cast<int64_t>(res.carry) * SO_XUNG_ENCODER_MOI_VONG + res.value;
                    res.gocDo = (static_cast<float>(res.value) * 360.0f) / SO_XUNG_ENCODER_MOI_VONG;
                    res.tongVong = static_cast<float>(res.carry) + static_cast<float>(res.value) / SO_XUNG_ENCODER_MOI_VONG;
                    res.xungViBuoc = (res.tongXung * SO_XUNG_BUOC_MOI_VONG) / SO_XUNG_ENCODER_MOI_VONG;
                    res.hopLe = true;
                    res.thongBaoLoi = "OK_9BYTE";
                    return res;
                }
            }
        }
    }

    res.hopLe = false;
    res.thongBaoLoi = "SAI_CRC8_HOAC_HEADER";
    return res;
}

// ----------------------------------------------------------------------------
// 3. THỐNG KÊ VÀ BIẾN TOÀN CỤC
// ----------------------------------------------------------------------------
struct ThongKeDongCo {
    uint32_t soLanGui = 0;
    uint32_t soGoiNhanTot = 0;
    uint32_t soGoiLoiCrc = 0;
    uint32_t soGoiTimeout = 0;
    int64_t gocGocZero = 0;
    bool daDatZero = false;
};

ThongKeDongCo tkMks1;
ThongKeDongCo tkMks2;

uint32_t chuKyDoc_ms = 100; // Mặc định 100ms (10 Hz)
uint32_t thoiDiemDocTruoc_ms = 0;
bool cheDoInChiTiet = true;

// ----------------------------------------------------------------------------
// 4. KHỞI TẠO UART PHẦN CỨNG
// ----------------------------------------------------------------------------
bool khoiTaoDualUart() {
    uart_config_t config{};
    config.baud_rate = BAUDRATE_MKS;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;

    // UART1: TX = GPIO1, RX = GPIO3 (MKS 1 - Bánh Trái)
    if (uart_param_config(PORT_UART_MKS1, &config) != ESP_OK ||
        uart_set_pin(PORT_UART_MKS1, PIN_UART_TX_CHUNG, PIN_UART_RX_MKS1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK ||
        uart_driver_install(PORT_UART_MKS1, UART_BUF_SIZE, UART_BUF_SIZE, 0, nullptr, 0) != ESP_OK) {
        Serial.println("[LOI] Khong the khoi tao UART1 (GPIO1 TX, GPIO3 RX)!");
        return false;
    }

    // UART2: TX = không dùng (-1), RX = GPIO4 (MKS 2 - Bánh Phải)
    if (uart_param_config(PORT_UART_MKS2, &config) != ESP_OK ||
        uart_set_pin(PORT_UART_MKS2, UART_PIN_NO_CHANGE, PIN_UART_RX_MKS2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK ||
        uart_driver_install(PORT_UART_MKS2, UART_BUF_SIZE, 0, 0, nullptr, 0) != ESP_OK) {
        Serial.println("[LOI] Khong the khoi tao UART2 (GPIO4 RX)!");
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
// 5. GỬI LỆNH VÀ THU THẬP PHẢN HỒI
// ----------------------------------------------------------------------------
void docThuHaiDongCo(bool inChiTiet) {
    ++tkMks1.soLanGui;
    ++tkMks2.soLanGui;

    // 1. Xóa sạch bộ đệm rác trước khi gửi
    uart_flush_input(PORT_UART_MKS1);
    uart_flush_input(PORT_UART_MKS2);

    // 2. Khung lệnh 0x30: [diaChi=0xE0, maLenh=0x30, crc8=0x10]
    const uint8_t khungHoi[3] = {DIA_CHI_MKS, 0x30, 0x10};
    // Phát đồng thời qua GPIO1 tới cả 2 motor
    uart_write_bytes(PORT_UART_MKS1, khungHoi, sizeof(khungHoi));

    // 3. Chờ phản hồi (ở 115200 baud, 8 byte truyền mất ~0.7 ms, chờ tối đa 8 ms)
    delay(8);

    // 4. Đọc dữ liệu từ MKS 1 (UART1 - GPIO3)
    uint8_t buf1[32]{};
    size_t len1 = 0;
    uart_get_buffered_data_len(PORT_UART_MKS1, &len1);
    int read1 = uart_read_bytes(PORT_UART_MKS1, buf1, sizeof(buf1), 0);
    KetQuaMksUart kq1 = giaiMaKhung0x30(buf1, read1, DIA_CHI_MKS);

    // 5. Đọc dữ liệu từ MKS 2 (UART2 - GPIO4)
    uint8_t buf2[32]{};
    size_t len2 = 0;
    uart_get_buffered_data_len(PORT_UART_MKS2, &len2);
    int read2 = uart_read_bytes(PORT_UART_MKS2, buf2, sizeof(buf2), 0);
    KetQuaMksUart kq2 = giaiMaKhung0x30(buf2, read2, DIA_CHI_MKS);

    // Cập nhật thống kê MKS 1
    if (kq1.hopLe) {
        ++tkMks1.soGoiNhanTot;
        if (!tkMks1.daDatZero) {
            tkMks1.gocGocZero = kq1.tongXung;
            tkMks1.daDatZero = true;
        }
    } else {
        if (read1 == 0) ++tkMks1.soGoiTimeout;
        else ++tkMks1.soGoiLoiCrc;
    }

    // Cập nhật thống kê MKS 2
    if (kq2.hopLe) {
        ++tkMks2.soGoiNhanTot;
        if (!tkMks2.daDatZero) {
            tkMks2.gocGocZero = kq2.tongXung;
            tkMks2.daDatZero = true;
        }
    } else {
        if (read2 == 0) ++tkMks2.soGoiTimeout;
        else ++tkMks2.soGoiLoiCrc;
    }

    // ------------------------------------------------------------------------
    // IN KẾT QUẢ RA SERIAL MONITOR
    // ------------------------------------------------------------------------
    if (inChiTiet) {
        Serial.printf("======================= [MẪU #%lu] =======================\n", tkMks1.soLanGui);

        // --- ĐỘNG CƠ 1 (TRÁI - GPIO3) ---
        Serial.printf("[MKS 1 - BÁNH TRÁI (RX=IO3)]: %s\n", kq1.hopLe ? ">>> OK <<<" : ">>> LOI <<<");
        Serial.print("  HEX nhan: ");
        if (read1 == 0) {
            Serial.println("(0 byte - Khong phan hoi!)");
        } else {
            for (int i = 0; i < read1; ++i) Serial.printf("%02X ", buf1[i]);
            Serial.printf(" (%d bytes) -> %s\n", read1, kq1.thongBaoLoi);
        }
        if (kq1.hopLe) {
            int64_t relTicks = kq1.tongXung - tkMks1.gocGocZero;
            float relRevs = static_cast<float>(relTicks) / SO_XUNG_ENCODER_MOI_VONG;
            Serial.printf("  So vong (Carry)   : %+ld vong\n", kq1.carry);
            Serial.printf("  Goc 1 vong (Value): %u / 16384 (%.2f deg)\n", kq1.value, kq1.gocDo);
            Serial.printf("  Tong xung (14-bit): %lld ticks (Tu goc 0: %+lld ticks = %+.2f vong)\n",
                          (long long)kq1.tongXung, (long long)relTicks, relRevs);
            Serial.printf("  Quy doi xung STEP : %+lld steps (o 6400 xung/vong)\n", (long long)kq1.xungViBuoc);
        }
        Serial.printf("  Thong ke: Tot=%lu | LoiCRC=%lu | Timeout=%lu (Ty le dat: %.1f%%)\n",
                      tkMks1.soGoiNhanTot, tkMks1.soGoiLoiCrc, tkMks1.soGoiTimeout,
                      tkMks1.soLanGui ? (tkMks1.soGoiNhanTot * 100.0f / tkMks1.soLanGui) : 0.0f);

        Serial.println("---------------------------------------------------------");

        // --- ĐỘNG CƠ 2 (PHẢI - GPIO4) ---
        Serial.printf("[MKS 2 - BÁNH PHẢI (RX=IO4)]: %s\n", kq2.hopLe ? ">>> OK <<<" : ">>> LOI <<<");
        Serial.print("  HEX nhan: ");
        if (read2 == 0) {
            Serial.println("(0 byte - Khong phan hoi!)");
        } else {
            for (int i = 0; i < read2; ++i) Serial.printf("%02X ", buf2[i]);
            Serial.printf(" (%d bytes) -> %s\n", read2, kq2.thongBaoLoi);
        }
        if (kq2.hopLe) {
            int64_t relTicks = kq2.tongXung - tkMks2.gocGocZero;
            float relRevs = static_cast<float>(relTicks) / SO_XUNG_ENCODER_MOI_VONG;
            Serial.printf("  So vong (Carry)   : %+ld vong\n", kq2.carry);
            Serial.printf("  Goc 1 vong (Value): %u / 16384 (%.2f deg)\n", kq2.value, kq2.gocDo);
            Serial.printf("  Tong xung (14-bit): %lld ticks (Tu goc 0: %+lld ticks = %+.2f vong)\n",
                          (long long)kq2.tongXung, (long long)relTicks, relRevs);
            Serial.printf("  Quy doi xung STEP : %+lld steps (o 6400 xung/vong)\n", (long long)kq2.xungViBuoc);
        }
        Serial.printf("  Thong ke: Tot=%lu | LoiCRC=%lu | Timeout=%lu (Ty le dat: %.1f%%)\n",
                      tkMks2.soGoiNhanTot, tkMks2.soGoiLoiCrc, tkMks2.soGoiTimeout,
                      tkMks2.soLanGui ? (tkMks2.soGoiNhanTot * 100.0f / tkMks2.soLanGui) : 0.0f);
        Serial.println();
    } else {
        // Dạng in bảng ngắn gọn (Compact table view)
        Serial.printf("#%05lu | M1(IO3): %-7s %4.1f deg | %+6.2fv | %8lld t | M2(IO4): %-7s %4.1f deg | %+6.2fv | %8lld t\n",
                      tkMks1.soLanGui,
                      kq1.hopLe ? "OK" : kq1.thongBaoLoi, kq1.gocDo, kq1.tongVong, (long long)kq1.tongXung,
                      kq2.hopLe ? "OK" : kq2.thongBaoLoi, kq2.gocDo, kq2.tongVong, (long long)kq2.tongXung);
    }
}

// ----------------------------------------------------------------------------
// 6. SETUP & LOOP
// ----------------------------------------------------------------------------
void inTroGiup() {
    Serial.println("\n--- CAC PHIM CHUC NANG (GO VAO SERIAL MONITOR) ---");
    Serial.println("  [?] hoac [h] : In bang tro giup nay");
    Serial.println("  [1]          : Doc 1 lan duy nhat (Single-shot query)");
    Serial.println("  [z]          : Dat lai vi tri goc 0 (Reset zero reference)");
    Serial.println("  [s]          : Dao che do in (Chi tiet <-> Bang ngan gon)");
    Serial.println("  [+]          : Tang toc do doc (giam chu ky ms)");
    Serial.println("  [-]          : Giam toc do doc (tang chu ky ms)");
    Serial.println("  [p]          : Tam dung / Tiep tuc doc dinh ky");
    Serial.println("--------------------------------------------------\n");
}

void setup() {
    Serial.begin(115200);
    delay(1500); // Đợi kết nối USB CDC trên ESP32-S3

    Serial.println("\n\n========================================================");
    Serial.println("   CHUONG TRINH TEST UART DUAL MKS SERVO42C V1.1       ");
    Serial.println("   Board: ESP32-S3 Super Mini                          ");
    Serial.println("========================================================");
    Serial.println("So do ket noi phan cung:");
    Serial.println("  - GPIO1 (TX chung) -> RX cua Ca 2 Motor MKS 1 & 2");
    Serial.println("  - GPIO3 (RX1)      -> TX cua Motor MKS 1 (Banh Trai)");
    Serial.println("  - GPIO4 (RX2)      -> TX cua Motor MKS 2 (Banh Phai)");
    Serial.println("  - GND ESP32-S3     -> GND Nguon 12V/24V cua 2 Motor MKS");
    Serial.println("Cau hinh menu tren MKS Servo42C V1.1:");
    Serial.println("  - Mode: CR_vFOC  |  Baud: 115200  |  UartAddr: 0xE0");
    Serial.println("========================================================");

    if (!khoiTaoDualUart()) {
        Serial.println("[LOI NGHIEP TRONG] Khong the khoi tao Dual UART! Dung chuong trinh.");
        while (true) delay(1000);
    }

    Serial.println("[OK] Da khoi tao Dual-UART 115200 bps thanh cong!");
    inTroGiup();
    thoiDiemDocTruoc_ms = millis();
}

bool tamDungDoc = false;

void loop() {
    // 1. Xử lý phím gõ từ bàn phím Serial
    if (Serial.available()) {
        char k = Serial.read();
        switch (k) {
        case '?':
        case 'h':
            inTroGiup();
            break;
        case '1':
            Serial.println("\n>>> GUI LENH DOC DON (SINGLE SHOT) <<<");
            docThuHaiDongCo(true);
            break;
        case 'z':
            tkMks1.daDatZero = false;
            tkMks2.daDatZero = false;
            Serial.println("\n>>> DA RESET GOC TOA DO VE 0 CHO CA 2 DONG CO! <<<");
            break;
        case 's':
            cheDoInChiTiet = !cheDoInChiTiet;
            Serial.printf("\n>>> CHE DO IN: %s <<<\n", cheDoInChiTiet ? "CHI TIET" : "BANG NGAN GON");
            break;
        case 'p':
            tamDungDoc = !tamDungDoc;
            Serial.printf("\n>>> %s DOC DINH KY <<<\n", tamDungDoc ? "TAM DUNG" : "TIEP TUC");
            break;
        case '+':
            if (chuKyDoc_ms > 20) chuKyDoc_ms -= 20;
            else if (chuKyDoc_ms > 5) chuKyDoc_ms -= 5;
            Serial.printf("\n>>> CHU KY DOC MOI: %lu ms (%.1f Hz) <<<\n", chuKyDoc_ms, 1000.0f / chuKyDoc_ms);
            break;
        case '-':
            chuKyDoc_ms += 20;
            Serial.printf("\n>>> CHU KY DOC MOI: %lu ms (%.1f Hz) <<<\n", chuKyDoc_ms, 1000.0f / chuKyDoc_ms);
            break;
        default:
            break;
        }
    }

    // 2. Định kỳ gửi lệnh đọc
    if (!tamDungDoc && (millis() - thoiDiemDocTruoc_ms >= chuKyDoc_ms)) {
        thoiDiemDocTruoc_ms = millis();
        docThuHaiDongCo(cheDoInChiTiet);
    }

    delay(1);
}
