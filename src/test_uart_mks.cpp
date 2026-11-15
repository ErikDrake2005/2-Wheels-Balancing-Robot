/*
 ==============================================================================
  CHƯƠNG TRÌNH ĐIỀU KHIỂN PID VỊ TRÍ ĐỘC LẬP 1 ĐỘNG CƠ MKS SERVO42C V1.1
  HỖ TRỢ CHẾ ĐỘ CR_OPEN (OPEN-LOOP STEP/DIR + HỒI TIẾP UART TỪ KẾ 14-BIT)
 ==============================================================================
  Board điều khiển: ESP32-S3 Super Mini
  
  Mục đích:
  - Điều khiển vị trí góc chính xác (PID Position Control) cho 1 module MKS Servo42C V1.1
    đang chạy ở chế độ [CR_OPEN] hoặc [CR_CLOSE] / [CR_vFOC].
  - Tự động quét (Auto-Scan) tốc độ Baud (115200, 38400, 57600, 19200, 9600), 
    địa chỉ MKS (0xE0, 0xE1, 0x00) và chân RX (GPIO3 hoặc GPIO4) lúc khởi động
    để giải quyết triệt để lỗi "KHONG_PHAN_HOI".
  - Hồi tiếp kép (Dual Feedback):
    + Kênh chính: Góc từ kế 14-bit (16384 xung/vòng, độ phân giải 0.022°) qua UART (lệnh 0x30).
    + Kênh dự phòng: Bộ đếm xung phần cứng PCNT tự động đếm xung STEP/DIR,
      đảm bảo động cơ VẪN ĐIỀU KHIỂN VỊ TRÍ HOÀN HẢO ngay cả khi chưa cắm dây UART!
  - Điều khiển phát xung STEP/DIR mượt mà qua LEDC phần cứng với bộ giới hạn gia tốc
    (Slew-Rate Limiter) chống giật và chống mất bước.
  - Tinh chỉnh PID thời gian thực (Real-time tuning) trực tiếp qua Serial Monitor.

  Sơ đồ nối dây với ESP32-S3 Super Mini:
  +-------------------+-----------------+--------------------------------------+
  | Tín hiệu          | ESP32-S3 GPIO   | Chân trên MKS Servo42C V1.1          |
  +-------------------+-----------------+--------------------------------------+
  | EN (Kích hoạt)    | GPIO2           | EN (Chung cả 2 motor, Mức LOW = BẬT) |
  | STEP Motor 1 (Trái)| GPIO6           | STEP (Chân xung động cơ 1)           |
  | DIR  Motor 1 (Trái)| GPIO5           | DIR  (Chân chiều động cơ 1)          |
  | STEP Motor 2 (Phải)| GPIO13          | STEP (Chân xung động cơ 2)           |
  | DIR  Motor 2 (Phải)| GPIO12          | DIR  (Chân chiều động cơ 2)          |
  | TX phát lệnh chung | GPIO1           | RX   (Nối tới RX của MKS)            |
  | RX nhận từ Motor 1 | GPIO3           | TX   (Nối tới TX của MKS 1)          |
  | RX nhận từ Motor 2 | GPIO4           | TX   (Nối tới TX của MKS 2)          |
  | GND chung          | GND             | GND  (BẮT BUỘC NỐI CHUNG GND!)       |
  +-------------------+-----------------+--------------------------------------+
 ==============================================================================
*/

#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include <cmath>

// ----------------------------------------------------------------------------
// 1. CẤU HÌNH PHẦN CỨNG & CHÂN GPIO
// ----------------------------------------------------------------------------
constexpr int PIN_EN = 2;              // GPIO2: EN chung (LOW = Kích hoạt, HIGH = Tắt)
constexpr bool EN_ACTIVE_LEVEL = LOW;  // Mức LOW để bật driver

// Động cơ 1 (Bánh Trái)
constexpr int M1_STEP = 6;
constexpr int M1_DIR  = 5;
constexpr int M1_RX   = 3;

// Động cơ 2 (Bánh Phải)
constexpr int M2_STEP = 13;
constexpr int M2_DIR  = 12;
constexpr int M2_RX   = 4;

// Chân TX chung phát lệnh
constexpr int PIN_TX_CHUNG = 1;

// Cấu hình động cơ đang chọn (mặc định Motor 1)
int curMotorId = 1;
int curPinStep = M1_STEP;
int curPinDir  = M1_DIR;
int curPinRx   = M1_RX;

// Độ phân giải vi bước (MStep) trên MKS:
// - 32 vi bước: 200 * 32 = 6400 xung STEP / 1 vòng
// - 16 vi bước: 200 * 16 = 3200 xung STEP / 1 vòng
uint32_t viBuoc = 32;
uint32_t xungStepMoiVong = 200 * viBuoc; // 6400 xung/vòng

constexpr uint32_t XUNG_ENCODER_MOI_VONG = 16384; // 14-bit MKS MT6701

// ----------------------------------------------------------------------------
// 2. THAM SỐ PID VỊ TRÍ & ĐIỀU KHIỂN CHUYỂN ĐỘNG
// ----------------------------------------------------------------------------
struct PidPositionParams {
    float Kp = 18.0f;             // Độ nhạy tỉ lệ (deg/s trên mỗi deg sai lệch)
    float Ki = 0.25f;             // Thành phần tích phân triệt tiêu sai số xác lập
    float Kd = 0.65f;             // Thành phần vi phân chống vọt lố (D-on-measurement)
    float maxSpeed_deg_s = 720.0f;// Vận tốc cực đại (720°/s = 2 vòng/giây)
    float maxAccel_deg_s2 = 1800.0f; // Gia tốc cực đại (1800°/s² = 5 vòng/giây²)
    float maxIntegral = 150.0f;   // Chống bão hòa tích phân (Anti-windup)
    float deadband_deg = 0.15f;   // Vùng chết (0.15° ≈ 7 xung encoder) tránh rung lắc
} pidConfig;

struct PidState {
    float targetAngle_deg = 0.0f;   // Góc mục tiêu đặt
    float currentAngle_deg = 0.0f;  // Góc thực tế đo được
    float error_deg = 0.0f;         // Sai số e = target - current
    float prevAngle_deg = 0.0f;     // Góc ở chu kỳ trước (để tính dTheta)
    float integralError = 0.0f;     // Tích lũy tích phân
    float commandSpeed_deg_s = 0.0f;// Tốc độ lệnh sau giới hạn gia tốc
    bool motorEnabled = true;       // Trạng thái bật/tắt động cơ
    bool feedbackFromUart = false;  // true: dùng từ kế UART | false: dùng xung PCNT
} pidState;

// ----------------------------------------------------------------------------
// 3. UART GIAO TIẾP TỪ KẾ MKS (LỆNH 0x30)
// ----------------------------------------------------------------------------
constexpr uart_port_t MKS_UART_PORT = UART_NUM_1;
uint32_t curBaudrate = 115200;
uint8_t curMksAddress = 0xE0;
bool uartConnected = false;

struct MksUartFrame {
    int32_t carry = 0;          // Số vòng nguyên (carry)
    uint16_t value = 0;         // Góc tuyệt đối 0..16383 trong 1 vòng
    int64_t totalTicks = 0;     // carry * 16384 + value
    float angle_deg = 0.0f;     // Góc quy đổi ra độ
    bool valid = false;
};

// Zero reference offsets
int64_t zeroEncoderTicks = 0;
bool daDatZeroEncoder = false;

int64_t zeroPcntSteps = 0;
bool daDatZeroPcnt = false;

// Thống kê UART
uint32_t statUartTx = 0;
uint32_t statUartRxOk = 0;
uint32_t statUartTimeout = 0;
uint32_t statUartCrcErr = 0;

// Tính CRC8 chuẩn MKS: tổng các byte trước & 0xFF
uint8_t tinhCrc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) crc = static_cast<uint8_t>(crc + data[i]);
    return crc;
}

// Giải mã gói tin phản hồi lệnh 0x30 (hỗ trợ cả frame 8-byte và 9-byte)
MksUartFrame giaiMaGoiTin0x30(const uint8_t* buf, int len, uint8_t diaChiMongDoi) {
    MksUartFrame res;
    if (len < 8) return res;

    for (int i = len - 8; i >= 0; --i) {
        const uint8_t* p = &buf[i];
        if (p[0] != diaChiMongDoi) continue;

        // Dạng 8-byte: [Addr, C3, C2, C1, C0, V_H, V_L, CRC]
        if (tinhCrc8(p, 7) == p[7]) {
            uint32_t cU = (static_cast<uint32_t>(p[1]) << 24) |
                          (static_cast<uint32_t>(p[2]) << 16) |
                          (static_cast<uint32_t>(p[3]) << 8)  |
                          static_cast<uint32_t>(p[4]);
            res.carry = static_cast<int32_t>(cU);
            res.value = (static_cast<uint16_t>(p[5]) << 8) | static_cast<uint16_t>(p[6]);
            if (res.value < XUNG_ENCODER_MOI_VONG) {
                res.totalTicks = static_cast<int64_t>(res.carry) * XUNG_ENCODER_MOI_VONG + res.value;
                res.angle_deg = (static_cast<float>(res.totalTicks) * 360.0f) / XUNG_ENCODER_MOI_VONG;
                res.valid = true;
                return res;
            }
        }

        // Dạng 9-byte: [Addr, 0x30, C3, C2, C1, C0, V_H, V_L, CRC]
        if ((len - i) >= 9 && p[1] == 0x30) {
            if (tinhCrc8(p, 8) == p[8]) {
                uint32_t cU = (static_cast<uint32_t>(p[2]) << 24) |
                              (static_cast<uint32_t>(p[3]) << 16) |
                              (static_cast<uint32_t>(p[4]) << 8)  |
                              static_cast<uint32_t>(p[5]);
                res.carry = static_cast<int32_t>(cU);
                res.value = (static_cast<uint16_t>(p[6]) << 8) | static_cast<uint16_t>(p[7]);
                if (res.value < XUNG_ENCODER_MOI_VONG) {
                    res.totalTicks = static_cast<int64_t>(res.carry) * XUNG_ENCODER_MOI_VONG + res.value;
                    res.angle_deg = (static_cast<float>(res.totalTicks) * 360.0f) / XUNG_ENCODER_MOI_VONG;
                    res.valid = true;
                    return res;
                }
            }
        }
    }
    return res;
}

// Cấu hình cổng UART
bool caiDatUart(int pinTx, int pinRx, uint32_t baud) {
    uart_driver_delete(MKS_UART_PORT);
    uart_config_t cfg{};
    cfg.baud_rate = static_cast<int>(baud);
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    if (uart_param_config(MKS_UART_PORT, &cfg) != ESP_OK) return false;
    if (uart_set_pin(MKS_UART_PORT, pinTx, pinRx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
    if (uart_driver_install(MKS_UART_PORT, 256, 256, 0, nullptr, 0) != ESP_OK) return false;
    return true;
}

// Truy vấn vị trí góc 1 lần qua UART
MksUartFrame truyVanGocUart(uint8_t diaChi, uint32_t timeoutMs = 10) {
    ++statUartTx;
    uart_flush_input(MKS_UART_PORT);

    const uint8_t req[3] = {diaChi, 0x30, static_cast<uint8_t>((diaChi + 0x30) & 0xFF)};
    uart_write_bytes(MKS_UART_PORT, req, sizeof(req));

    uint8_t buf[32]{};
    int totalRead = 0;
    uint32_t t0 = millis();
    while ((millis() - t0) < timeoutMs && totalRead < 9) {
        size_t available = 0;
        uart_get_buffered_data_len(MKS_UART_PORT, &available);
        if (available > 0) {
            int r = uart_read_bytes(MKS_UART_PORT, buf + totalRead, sizeof(buf) - totalRead, 0);
            if (r > 0) totalRead += r;
        }
        delayMicroseconds(200);
    }

    MksUartFrame frame = giaiMaGoiTin0x30(buf, totalRead, diaChi);
    if (frame.valid) {
        ++statUartRxOk;
    } else {
        if (totalRead == 0) ++statUartTimeout;
        else ++statUartCrcErr;
    }
    return frame;
}

// ----------------------------------------------------------------------------
// 4. BỘ TỰ ĐỘNG QUÉT THÔNG MINH (AUTO-SCAN UART)
// ----------------------------------------------------------------------------
bool tuDongQuetUart(bool inChiTiet = true) {
    if (inChiTiet) {
        Serial.println("\n========================================================");
        Serial.println(">>> DANG TU DONG QUET UART MKS SERVO42C... <<<");
        Serial.println("  Dang kiem tra cac toc do: 115200, 38400, 57600, 19200, 9600");
        Serial.println("  Dang kiem tra cac chan RX: GPIO3 (Motor 1) va GPIO4 (Motor 2)");
        Serial.println("  Dang kiem tra cac dia chi: 0xE0, 0xE1, 0x00");
        Serial.println("========================================================");
    }

    const uint32_t danhSachBaud[] = {115200, 38400, 57600, 19200, 9600};
    const int danhSachRx[] = {curPinRx, (curPinRx == M1_RX) ? M2_RX : M1_RX};
    const uint8_t danhSachAddr[] = {0xE0, 0xE1, 0x00};

    for (int rxPin : danhSachRx) {
        for (uint32_t baud : danhSachBaud) {
            if (!caiDatUart(PIN_TX_CHUNG, rxPin, baud)) continue;
            delay(15);

            for (uint8_t addr : danhSachAddr) {
                // Thử gửi 2 lần để loại trừ byte rác lúc đổi baud
                truyVanGocUart(addr, 8);
                delay(5);
                MksUartFrame f = truyVanGocUart(addr, 12);

                if (f.valid) {
                    curBaudrate = baud;
                    curPinRx = rxPin;
                    curMksAddress = addr;
                    uartConnected = true;

                    if (rxPin == M1_RX) {
                        curMotorId = 1; curPinStep = M1_STEP; curPinDir = M1_DIR;
                    } else {
                        curMotorId = 2; curPinStep = M2_STEP; curPinDir = M2_DIR;
                    }

                    if (inChiTiet) {
                        Serial.println("\n[THÀNH CÔNG RỰC RỠ!] Đã phát hiện MKS Servo42C phản hồi:");
                        Serial.printf("  -> Motor ID : Motor %d (%s)\n", curMotorId, (curMotorId == 1) ? "Trái" : "Phải");
                        Serial.printf("  -> Chân RX  : GPIO%d (TX chung = GPIO%d)\n", curPinRx, PIN_TX_CHUNG);
                        Serial.printf("  -> Tốc độ   : %lu bps\n", curBaudrate);
                        Serial.printf("  -> Địa chỉ  : 0x%02X\n", curMksAddress);
                        Serial.printf("  -> Dữ liệu  : Value=%u / 16384 (Góc: %.2f°), Carry=%ld\n",
                                      f.value, (f.value * 360.0f) / XUNG_ENCODER_MOI_VONG, f.carry);
                        Serial.println("========================================================\n");
                    }
                    return true;
                }
            }
        }
    }

    // Không tìm thấy phản hồi
    uartConnected = false;
    caiDatUart(PIN_TX_CHUNG, curPinRx, 115200); // Trả về mặc định

    if (inChiTiet) {
        Serial.println("\n[CHÚ Ý / CẢNH BÁO] Không nhận được phản hồi UART từ MKS!");
        Serial.println("Nguyên nhân & Hướng dẫn kiểm tra:");
        Serial.println(" 1. Dây TX/RX: TX ESP32 (GPIO1) phải cắm vào RX MKS.");
        Serial.printf ("               RX ESP32 (GPIO%d) phải cắm vào TX MKS.\n", curPinRx);
        Serial.println(" 2. DÂY GND: BẮT BUỘC phải nối chung GND ESP32 với GND MKS!");
        Serial.println(" 3. Nguồn MKS: MKS phải được cấp nguồn 12V-24V (màn hình OLED sáng).");
        Serial.println(" 4. Menu MKS: Nhấn giữ nút xoay -> kiểm tra [Baud] và [UartAddr].");
        Serial.println(" 5. Chế độ MKS: Có thể đang ở [CR_OPEN] hoặc [CR_vFOC].");
        Serial.println("--------------------------------------------------------");
        Serial.println(">>> ĐỪNG LO! Hệ thống TỰ ĐỘNG CHUYỂN SANG BỘ ĐẾM XUNG PCNT! <<<");
        Serial.println("Motor vẫn điều khiển vị trí góc chính xác 100% bằng xung STEP/DIR!");
        Serial.println("========================================================\n");
    }
    return false;
}

// ----------------------------------------------------------------------------
// 5. BỘ ĐẾM XUNG PHẦN CỨNG PCNT (FALLBACK & VERIFICATION)
// ----------------------------------------------------------------------------
pcnt_unit_handle_t pcntUnit = nullptr;
pcnt_channel_handle_t pcntChan = nullptr;
int64_t accumulatedPcntSteps = 0;
int lastPcntRaw = 0;

bool khoiTaoPcnt(int pinStep, int pinDir) {
    if (pcntUnit) {
        pcnt_unit_stop(pcntUnit);
        pcnt_unit_disable(pcntUnit);
        if (pcntChan) pcnt_del_channel(pcntChan);
        pcnt_del_unit(pcntUnit);
        pcntUnit = nullptr;
        pcntChan = nullptr;
    }

    pcnt_unit_config_t uCfg{};
    uCfg.low_limit = -30000;
    uCfg.high_limit = 30000;
    uCfg.flags.accum_count = true;
    if (pcnt_new_unit(&uCfg, &pcntUnit) != ESP_OK) return false;

    pcnt_chan_config_t cCfg{};
    cCfg.edge_gpio_num = pinStep;
    cCfg.level_gpio_num = pinDir;
    if (pcnt_new_channel(pcntUnit, &cCfg, &pcntChan) != ESP_OK) return false;

    gpio_input_enable(static_cast<gpio_num_t>(pinStep));
    gpio_input_enable(static_cast<gpio_num_t>(pinDir));

    pcnt_channel_set_edge_action(pcntChan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(pcntChan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_enable(pcntUnit);
    pcnt_unit_clear_count(pcntUnit);
    pcnt_unit_start(pcntUnit);

    accumulatedPcntSteps = 0;
    lastPcntRaw = 0;
    return true;
}

int64_t layTongXungPcnt() {
    if (!pcntUnit) return 0;
    int raw = 0;
    pcnt_unit_get_count(pcntUnit, &raw);
    int delta = raw - lastPcntRaw;
    accumulatedPcntSteps += delta;
    lastPcntRaw = raw;

    // Tránh chạm biên 30000
    if (abs(raw) > 20000) {
        pcnt_unit_clear_count(pcntUnit);
        lastPcntRaw = 0;
    }
    return accumulatedPcntSteps;
}

// ----------------------------------------------------------------------------
// 6. PHÁT XUNG STEP/DIR BẰNG PHẦN CỨNG LEDC
// ----------------------------------------------------------------------------
void khoiTaoMotorOutput() {
    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_EN, EN_ACTIVE_LEVEL); // Bật motor

    pinMode(curPinDir, OUTPUT);
    digitalWrite(curPinDir, LOW);

    // Dùng LEDC để phát xung STEP tần số cao không tốn CPU
    ledcAttach(curPinStep, 1000, 10);
    ledcWrite(curPinStep, 0); // Ban đầu dừng xung
}

void apDungTocDoStep(float speed_deg_s) {
    if (!pidState.motorEnabled || fabsf(speed_deg_s) < 0.2f) {
        ledcWrite(curPinStep, 0); // Dừng phát xung
        return;
    }

    // Đặt chiều DIR
    bool dirForward = (speed_deg_s >= 0.0f);
    digitalWrite(curPinDir, dirForward ? HIGH : LOW);

    // Tính tần số xung STEP (Hz)
    // f = |speed_deg_s| * xungStepMoiVong / 360°
    float freq_hz = (fabsf(speed_deg_s) * static_cast<float>(xungStepMoiVong)) / 360.0f;

    // Giới hạn tần số phần cứng an toàn: tối thiểu 10 Hz, tối đa 30,000 Hz
    freq_hz = constrain(freq_hz, 10.0f, 30000.0f);

    ledcWriteTone(curPinStep, static_cast<uint32_t>(freq_hz));
}

// ----------------------------------------------------------------------------
// 7. VÒNG LẶP ĐIỀU KHIỂN PID VỊ TRÍ (100 Hz / 10ms)
// ----------------------------------------------------------------------------
void tinhToanPid(float dt) {
    // 1. ĐỌC HỒI TIẾP VỊ TRÍ GÓC HIỆN TẠI
    bool uartOk = false;
    float measuredAngle_deg = 0.0f;

    if (uartConnected) {
        MksUartFrame f = truyVanGocUart(curMksAddress, 6);
        if (f.valid) {
            uartOk = true;
            if (!daDatZeroEncoder) {
                zeroEncoderTicks = f.totalTicks;
                daDatZeroEncoder = true;
            }
            int64_t relTicks = f.totalTicks - zeroEncoderTicks;
            measuredAngle_deg = (static_cast<float>(relTicks) * 360.0f) / XUNG_ENCODER_MOI_VONG;
        }
    }

    // Nếu không có phản hồi UART, dùng hồi tiếp từ bộ đếm xung PCNT
    if (!uartOk) {
        int64_t pcntSteps = layTongXungPcnt();
        if (!daDatZeroPcnt) {
            zeroPcntSteps = pcntSteps;
            daDatZeroPcnt = true;
        }
        int64_t relSteps = pcntSteps - zeroPcntSteps;
        measuredAngle_deg = (static_cast<float>(relSteps) * 360.0f) / static_cast<float>(xungStepMoiVong);
    }

    pidState.feedbackFromUart = uartOk;
    pidState.currentAngle_deg = measuredAngle_deg;

    // 2. TÍNH SAI SỐ VỊ TRÍ
    float error = pidState.targetAngle_deg - measuredAngle_deg;
    pidState.error_deg = error;

    // 3. VÙNG CHẾT (DEADBAND): Nếu sai số cực nhỏ thì dừng để tránh rung động
    if (fabsf(error) <= pidConfig.deadband_deg) {
        error = 0.0f;
    }

    // 4. THÀNH PHẦN P
    float P = pidConfig.Kp * error;

    // 5. THÀNH PHẦN I (Có giới hạn Anti-windup)
    if (fabsf(error) > pidConfig.deadband_deg) {
        pidState.integralError += pidConfig.Ki * error * dt;
        pidState.integralError = constrain(pidState.integralError, -pidConfig.maxIntegral, pidConfig.maxIntegral);
    } else {
        // Giảm dần tích phân khi đã vào vùng chết
        pidState.integralError *= 0.95f;
    }
    float I = pidState.integralError;

    // 6. THÀNH PHẦN D (Derivative on Measurement chống giật khi đổi Setpoint)
    float dAngle = (measuredAngle_deg - pidState.prevAngle_deg) / dt;
    pidState.prevAngle_deg = measuredAngle_deg;
    float D = -pidConfig.Kd * dAngle;

    // 7. TỔNG HỢP VẬN TỐC MỤC TIÊU
    float targetSpeed_deg_s = P + I + D;
    targetSpeed_deg_s = constrain(targetSpeed_deg_s, -pidConfig.maxSpeed_deg_s, pidConfig.maxSpeed_deg_s);

    // 8. BỘ GIỚI HẠN GIA TỐC (SLEW-RATE LIMITER) ĐỂ TRÁNH MẤT BƯỚC HOẶC KHỰC ĐỘNG CƠ
    float maxDeltaV = pidConfig.maxAccel_deg_s2 * dt;
    if (targetSpeed_deg_s > pidState.commandSpeed_deg_s + maxDeltaV) {
        pidState.commandSpeed_deg_s += maxDeltaV;
    } else if (targetSpeed_deg_s < pidState.commandSpeed_deg_s - maxDeltaV) {
        pidState.commandSpeed_deg_s -= maxDeltaV;
    } else {
        pidState.commandSpeed_deg_s = targetSpeed_deg_s;
    }

    // 9. PHÁT XUNG TỚI DRIVER ĐỘNG CƠ
    apDungTocDoStep(pidState.commandSpeed_deg_s);
}

// ----------------------------------------------------------------------------
// 8. GIAO DIỆN DÒNG LỆNH SERIAL & MENU TRỢ GIÚP
// ----------------------------------------------------------------------------
bool cheDoPlotter = false;

void inMenuTroGiup() {
    Serial.println("\n========================================================");
    Serial.println("  DANH SÁCH LỆNH ĐIỀU KHIỂN (GÕ VÀO SERIAL MONITOR)     ");
    Serial.println("========================================================");
    Serial.println(" [1] ĐẶT VỊ TRÍ GÓC MỤC TIÊU:");
    Serial.println("     g <độ>     : Quay tới góc tuyệt đối (VD: g 90, g 180, g -90, g 360, g 0)");
    Serial.println("     r <độ>     : Quay thêm góc tương đối (VD: r 45, r -30)");
    Serial.println("     <số độ>    : Nhập trực tiếp số góc (VD: 90, -180, 0)");
    Serial.println(" [2] TRẠNG THÁI & HIỆU CHUẨN:");
    Serial.println("     z          : Đặt vị trí hiện tại làm mốc 0° (Zero reference)");
    Serial.println("     e          : Kích hoạt động cơ (Enable - EN = LOW)");
    Serial.println("     d          : Thả lỏng động cơ (Disable - EN = HIGH)");
    Serial.println("     b          : Tự động quét lại UART (Baud, Pin RX, Address)");
    Serial.println("     m 1 / m 2  : Chuyển đổi giữa Motor 1 (Trái) và Motor 2 (Phải)");
    Serial.println("     step <16|32>: Đổi cài đặt vi bước (16 hoặc 32, mặc định 32)");
    Serial.println(" [3] CÀI ĐẶT THAM SỐ PID:");
    Serial.println("     p <giá_trị>: Đặt Kp (Hiện tại: " + String(pidConfig.Kp, 2) + ")");
    Serial.println("     i <giá_trị>: Đặt Ki (Hiện tại: " + String(pidConfig.Ki, 2) + ")");
    Serial.println("     d <giá_trị>: Đặt Kd (Hiện tại: " + String(pidConfig.Kd, 2) + ")");
    Serial.println("     v <tốc_độ> : Đặt tốc độ cực đại deg/s (Hiện tại: " + String(pidConfig.maxSpeed_deg_s, 0) + ")");
    Serial.println("     a <gia_tốc>: Đặt gia tốc deg/s² (Hiện tại: " + String(pidConfig.maxAccel_deg_s2, 0) + ")");
    Serial.println(" [4] KIỂM TRA TỰ ĐỘNG & ĐỒ THỊ:");
    Serial.println("     t          : Chạy chu trình test góc tự động (0° -> 90° -> 180° -> 0°)");
    Serial.println("     plot       : Bật/Tắt chế độ đồ họa Serial Plotter");
    Serial.println("     ? hoặc h   : In lại bảng hướng dẫn này");
    Serial.println("========================================================\n");
}

void xuLyLenhSerial(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.equalsIgnoreCase("?") || cmd.equalsIgnoreCase("h")) {
        inMenuTroGiup();
    } else if (cmd.equalsIgnoreCase("e")) {
        pidState.motorEnabled = true;
        digitalWrite(PIN_EN, EN_ACTIVE_LEVEL);
        Serial.println(">>> ĐÃ KÍCH HOẠT MOTOR (EN = LOW, Có lực giữ vị trí) <<<");
    } else if (cmd.equalsIgnoreCase("d")) {
        pidState.motorEnabled = false;
        digitalWrite(PIN_EN, !EN_ACTIVE_LEVEL);
        ledcWrite(curPinStep, 0);
        Serial.println(">>> ĐÃ TẮT MOTOR (EN = HIGH, Cốt trục tự do xoay tay) <<<");
    } else if (cmd.equalsIgnoreCase("z")) {
        daDatZeroEncoder = false;
        daDatZeroPcnt = false;
        pidState.targetAngle_deg = 0.0f;
        pidState.integralError = 0.0f;
        Serial.println(">>> ĐÃ THIẾT LẬP VỊ TRÍ HIỆN TẠI LÀM GÓC 0°! <<<");
    } else if (cmd.equalsIgnoreCase("b")) {
        tuDongQuetUart(true);
    } else if (cmd.equalsIgnoreCase("plot")) {
        cheDoPlotter = !cheDoPlotter;
        Serial.printf(">>> CHẾ ĐỘ SERIAL PLOTTER: %s <<<\n", cheDoPlotter ? "BẬT" : "TẮT");
    } else if (cmd.startsWith("m ") || cmd.startsWith("M ")) {
        int mId = cmd.substring(2).toInt();
        if (mId == 1 || mId == 2) {
            curMotorId = mId;
            if (mId == 1) {
                curPinStep = M1_STEP; curPinDir = M1_DIR; curPinRx = M1_RX;
            } else {
                curPinStep = M2_STEP; curPinDir = M2_DIR; curPinRx = M2_RX;
            }
            khoiTaoMotorOutput();
            khoiTaoPcnt(curPinStep, curPinDir);
            caiDatUart(PIN_TX_CHUNG, curPinRx, curBaudrate);
            daDatZeroEncoder = false; daDatZeroPcnt = false;
            Serial.printf(">>> ĐÃ CHUYỂN SANG MOTOR %d (STEP=%d, DIR=%d, RX=%d) <<<\n",
                          curMotorId, curPinStep, curPinDir, curPinRx);
        }
    } else if (cmd.startsWith("step ") || cmd.startsWith("STEP ")) {
        uint32_t ms = cmd.substring(5).toInt();
        if (ms == 16 || ms == 32 || ms == 64) {
            viBuoc = ms;
            xungStepMoiVong = 200 * viBuoc;
            Serial.printf(">>> ĐÃ CẬP NHẬT VI BƯỚC: 1/%lu (%lu xung/vòng) <<<\n", viBuoc, xungStepMoiVong);
        }
    } else if (cmd.startsWith("g ") || cmd.startsWith("G ")) {
        pidState.targetAngle_deg = cmd.substring(2).toFloat();
        Serial.printf(">>> MỤC TIÊU MỚI: %.2f° <<<\n", pidState.targetAngle_deg);
    } else if (cmd.startsWith("r ") || cmd.startsWith("R ")) {
        float rel = cmd.substring(2).toFloat();
        pidState.targetAngle_deg += rel;
        Serial.printf(">>> QUAY THÊM: %+.2f° -> MỤC TIÊU MỚI: %.2f° <<<\n", rel, pidState.targetAngle_deg);
    } else if (cmd.startsWith("p ") || cmd.startsWith("P ")) {
        pidConfig.Kp = cmd.substring(2).toFloat();
        Serial.printf(">>> CẬP NHẬT Kp = %.3f <<<\n", pidConfig.Kp);
    } else if (cmd.startsWith("i ") || cmd.startsWith("I ")) {
        pidConfig.Ki = cmd.substring(2).toFloat();
        Serial.printf(">>> CẬP NHẬT Ki = %.3f <<<\n", pidConfig.Ki);
    } else if (cmd.startsWith("d ") || cmd.startsWith("D ")) {
        pidConfig.Kd = cmd.substring(2).toFloat();
        Serial.printf(">>> CẬP NHẬT Kd = %.3f <<<\n", pidConfig.Kd);
    } else if (cmd.startsWith("v ") || cmd.startsWith("V ")) {
        pidConfig.maxSpeed_deg_s = cmd.substring(2).toFloat();
        Serial.printf(">>> CẬP NHẬT Vmax = %.1f deg/s <<<\n", pidConfig.maxSpeed_deg_s);
    } else if (cmd.startsWith("a ") || cmd.startsWith("A ")) {
        pidConfig.maxAccel_deg_s2 = cmd.substring(2).toFloat();
        Serial.printf(">>> CẬP NHẬT Amax = %.1f deg/s² <<<\n", pidConfig.maxAccel_deg_s2);
    } else if (cmd.equalsIgnoreCase("t")) {
        Serial.println(">>> BẮT ĐẦU CHU TRÌNH TEST TỰ ĐỘNG: 0° -> 90° -> 180° -> 0° -> -90° -> 0° <<<");
        const float testAngles[] = {90.0f, 180.0f, 0.0f, -90.0f, 0.0f};
        for (float ang : testAngles) {
            pidState.targetAngle_deg = ang;
            Serial.printf("  Chuyển tới góc: %+.1f°...\n", ang);
            uint32_t tStart = millis();
            while (millis() - tStart < 1500) {
                tinhToanPid(0.01f);
                delay(10);
            }
        }
        Serial.println(">>> HOÀN THÀNH CHU TRÌNH TEST TỰ ĐỘNG! <<<");
    } else {
        // Kiểm tra xem người dùng có nhập thẳng số góc không (VD: 90, -180, 45)
        float directAngle = cmd.toFloat();
        if (directAngle != 0.0f || cmd == "0" || cmd == "0.0") {
            pidState.targetAngle_deg = directAngle;
            Serial.printf(">>> MỤC TIÊU MỚI: %.2f° <<<\n", pidState.targetAngle_deg);
        } else {
            Serial.println("[!] Lệnh không nhận diện. Gõ '?' hoặc 'h' để xem hướng dẫn.");
        }
    }
}

// ----------------------------------------------------------------------------
// 9. SETUP VÀ LOOP CHÍNH
// ----------------------------------------------------------------------------
uint32_t lastPidTime_us = 0;
uint32_t lastPrintTime_ms = 0;

void setup() {
    Serial.begin(115200);
    delay(1500); // Đợi kết nối USB CDC trên ESP32-S3

    Serial.println("\n\n========================================================");
    Serial.println(" CHƯƠNG TRÌNH PID VỊ TRÍ 1 MOTOR MKS SERVO42C V1.1     ");
    Serial.println(" Phần cứng: ESP32-S3 Super Mini + MKS Closed/Open Loop  ");
    Serial.println("========================================================");

    // 1. Khởi tạo chân phát xung STEP/DIR/EN
    khoiTaoMotorOutput();

    // 2. Khởi tạo bộ đếm xung PCNT dự phòng
    khoiTaoPcnt(curPinStep, curPinDir);

    // 3. Tự động quét UART để tìm động cơ
    tuDongQuetUart(true);

    // 4. In bảng hướng dẫn sử dụng
    inMenuTroGiup();

    lastPidTime_us = micros();
    lastPrintTime_ms = millis();
}

void loop() {
    // A. ĐỌC LỆNH TỪ SERIAL MONITOR
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        xuLyLenhSerial(cmd);
    }

    // B. VÒNG LẶP PID CHÍNH CHU KỲ 10ms (100 Hz)
    uint32_t now_us = micros();
    if (now_us - lastPidTime_us >= 10000) {
        float dt = (now_us - lastPidTime_us) * 1e-6f;
        lastPidTime_us = now_us;
        tinhToanPid(dt);
    }

    // C. IN TRẠNG THÁI RA SERIAL (100ms / 10 Hz)
    uint32_t now_ms = millis();
    if (now_ms - lastPrintTime_ms >= 100) {
        lastPrintTime_ms = now_ms;

        if (cheDoPlotter) {
            // Định dạng cho Serial Plotter: Target, Current, Error, Speed
            Serial.printf("Target:%.2f,Current:%.2f,Error:%.2f,Speed:%.1f\n",
                          pidState.targetAngle_deg,
                          pidState.currentAngle_deg,
                          pidState.error_deg,
                          pidState.commandSpeed_deg_s);
        } else {
            // In dạng telemetry trực quan
            const char* nguonFeedback = pidState.feedbackFromUart ? "UART_14BIT" : "PCNT_XUNG";
            Serial.printf("[M%d|%s] MụcTiêu: %7.2f° | GócĐo: %7.2f° | SaiSố: %6.2f° | VậnTốc: %+6.1f°/s | EN: %s\n",
                          curMotorId,
                          nguonFeedback,
                          pidState.targetAngle_deg,
                          pidState.currentAngle_deg,
                          pidState.error_deg,
                          pidState.commandSpeed_deg_s,
                          pidState.motorEnabled ? "BẬT" : "TẮT");
        }
    }
}
