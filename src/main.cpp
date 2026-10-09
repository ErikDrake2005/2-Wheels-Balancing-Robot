// MAIN THAM KHẢO CHO API MKS V2 ĐÃ CÓ TRONG DỰ ÁN.
// Có chú thích để đọc hiểu; bạn tự đặt vào src/main.cpp khi muốn thử.
// Chưa có PID, xe chỉ được thử khi hai bánh nhấc khỏi đất.
// Không tạo McpwmStepOutput riêng: MksServo42CPair đã sở hữu hai backend.

#include <Arduino.h>
#include <cmath>
#include <limits>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mpu6050Driver.h"
#include "MksServo42C.h"
#include "config/ImuConfig.h"
#include "config/MotorConfig.h"
#include "config/PinConfig.h"
#include "config/ConfigCompareSettings.h"

mpu6050Drv imu(Wire);
MksServo42CPair motors;

uint64_t clockUs() {
    return static_cast<uint64_t>(esp_timer_get_time());
}

// Chỉ chạy ở setup, trước khi task IMU được tạo: không tranh I2C với task.
bool readRegister(uint8_t reg, uint8_t& value) {
    Wire.beginTransmission(static_cast<uint8_t>(MPU_I2C_ADRESS));
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(static_cast<uint8_t>(MPU_I2C_ADRESS),
                         static_cast<size_t>(1), true) != 1) return false;
    const int byte = Wire.read();
    if (byte < 0) return false;
    value = static_cast<uint8_t>(byte);
    return true;
}

bool compareRegister(const char* name, uint8_t reg, uint8_t expected, uint8_t mask) {
    uint8_t value = 0;
    const bool readOk = readRegister(reg, value);
    const bool match = readOk && ((value & mask) == (expected & mask));
    Serial.printf("MPU %-13s reg=0x%02X got=0x%02X expected=0x%02X mask=0x%02X %s\n",
        name, static_cast<unsigned>(reg), static_cast<unsigned>(value),
        static_cast<unsigned>(expected), static_cast<unsigned>(mask),
        !readOk ? "READ_ERROR" : (match ? "MATCH" : "MISMATCH"));
    return match;
}

bool auditImuConfig() {
    // Bộ driver hiện tại chỉ hỗ trợ +/-4g và +/-500 dps.
    // WHO_AM_I vẫn 0x68 ngay cả khi AD0 làm địa chỉ bus thành 0x69.
    bool passed = true;
    passed = compareRegister("WHO_AM_I", 0x75, 0x68, 0x7E) && passed;
    passed = compareRegister("PWR_MGMT_1", 0x6B, 0x01, 0x7F) && passed;
    passed = compareRegister("SMPLRT_DIV", 0x19,
        static_cast<uint8_t>(1000u / MPU_thoigianLayMau_Hz - 1u), 0xFF) && passed;
    passed = compareRegister("DLPF_CFG", 0x1A, MPU_DigitalLowPassFilterConfig, 0x07) && passed;
    passed = compareRegister("GYRO_CONFIG", 0x1B, 0x08, 0x18) && passed;
    passed = compareRegister("ACCEL_CONFIG", 0x1C, 0x08, 0x18) && passed;
    passed = compareRegister("INT_PIN_CFG", 0x37, MPU_ActiveSignal ? 0x00 : 0x80, 0xF0) && passed;
    passed = compareRegister("INT_ENABLE", 0x38, MPU_NgatIMU ? 0x01 : 0x00, 0x01) && passed;
    return passed;
}

// Chỉ tính từ số liệu bạn nhập tay, không đọc được menu/encoder MKS qua STEP/DIR.
void printMotorConversion(const char* name, uint16_t steps, uint16_t microstep, float ratio) {
    if (steps == 0 || microstep == 0 || !std::isfinite(ratio) || ratio <= 0.0f) {
        Serial.printf("%s: steps/rev, MStep hoac gear ratio chua nhap; khong suy ra RPM.\n", name);
        return;
    }
    const double inputStepsPerWheelRev = static_cast<double>(steps) * microstep * ratio;
    const double hz = COMPARE_CONTINUOUS_RATE_HZ;
    const double wheelRpm = hz * 60.0 / inputStepsPerWheelRev;
    const double velocity = hz / inputStepsPerWheelRev * 6.28318530718 * COMPARE_WHEEL_RADIUS_M;
    Serial.printf("%s: %.0f input steps/wheel-rev; %.0f steps/s -> %.2f wheel RPM, %.4f m/s\n",
        name, inputStepsPerWheelRev, hz, wheelRpm, velocity);
}

void printMotorConfig() {
    Serial.printf("MKS PIN L(step/dir)=%d/%d R=%d/%d EN=%d\n",
        PIN_StepL, PIN_DirL, PIN_StepR, PIN_DirR, PIN_EnMotor);
    Serial.printf("MKS EN active=%s, forward DIR L=%s R=%s, confirmed=%d, benchPermission=%d\n",
        MOTOR_EN_ACTIVE_HIGH ? "HIGH" : "LOW", MOTOR_LEFT_FORWARD_DIR_HIGH ? "HIGH" : "LOW",
        MOTOR_RIGHT_FORWARD_DIR_HIGH ? "HIGH" : "LOW", MOTOR_HARDWARE_CONFIG_CONFIRMED,
        COMPARE_ALLOW_MOTOR_RUN);
    Serial.printf("MKS HIGH=%luus minLOW=%luus hold=%luus setup=%luus enableWait=%luus\n",
        static_cast<unsigned long>(MOTOR_STEP_HIGH_US), static_cast<unsigned long>(MOTOR_STEP_MIN_LOW_US),
        static_cast<unsigned long>(MOTOR_DIR_HOLD_US), static_cast<unsigned long>(MOTOR_DIR_SETUP_US),
        static_cast<unsigned long>(MOTOR_ENABLE_SETUP_US));
    const uint64_t high = (static_cast<uint64_t>(MOTOR_STEP_HIGH_US) * MCPWM_TIMER_RESOLUTION_HZ + 999999ULL) / 1000000ULL;
    const uint64_t low = (static_cast<uint64_t>(MOTOR_STEP_MIN_LOW_US) * MCPWM_TIMER_RESOLUTION_HZ + 999999ULL) / 1000000ULL;
    const double maxSignalHz = high + low ? static_cast<double>(MCPWM_TIMER_RESOLUTION_HZ) / (high + low) : 0.0;
    Serial.printf("MCPWM resolution=%luHz, min continuous backend~%.2fHz, max by waveform~%.2fHz\n",
        static_cast<unsigned long>(MCPWM_TIMER_RESOLUTION_HZ),
        static_cast<double>(MCPWM_TIMER_RESOLUTION_HZ) / 65535.0, maxSignalHz);
    Serial.printf("PAIR threshold=%.0f steps/s max=%.0f steps/s service=%luus deadline=%luus\n",
        MOTOR_CONTINUOUS_MIN_RATE_HZ, MOTOR_MAX_STEP_RATE_HZ,
        static_cast<unsigned long>(MOTOR_SERVICE_TARGET_US), static_cast<unsigned long>(MOTOR_COMMAND_TIMEOUT_US));
    printMotorConversion("LEFT", COMPARE_LEFT_FULL_STEPS_PER_REV, COMPARE_LEFT_MICROSTEP,
        COMPARE_LEFT_MOTOR_REVS_PER_WHEEL_REV);
    printMotorConversion("RIGHT", COMPARE_RIGHT_FULL_STEPS_PER_REV, COMPARE_RIGHT_MICROSTEP,
        COMPARE_RIGHT_MOTOR_REVS_PER_WHEEL_REV);
    Serial.println("MKS config tren chi la ky vong ESP32; doi chieu menu va logic analyzer rieng.");
}

const char* resultName(MksResult value) {
    switch (value) {
        case MksResult::Ok: return "Ok";
        case MksResult::Saturated: return "Saturated";
        case MksResult::Pending: return "Pending";
        case MksResult::Disabled: return "Disabled";
        case MksResult::NotInitialized: return "NotInitialized";
        case MksResult::ConfigInvalid: return "ConfigInvalid";
        case MksResult::HardwareConfigUnconfirmed: return "HardwareConfigUnconfirmed";
        case MksResult::FaultLatched: return "FaultLatched";
    }
    return "Unknown";
}

const char* faultName(MksFault value) {
    switch (value) {
        case MksFault::None: return "None";
        case MksFault::InvalidCommand: return "InvalidCommand";
        case MksFault::CommandExpired: return "CommandExpired";
        case MksFault::ServiceLate: return "ServiceLate";
        case MksFault::BackendError: return "BackendError";
        case MksFault::SparseBacklog: return "SparseBacklog";
        case MksFault::EmergencyStop: return "EmergencyStop";
    }
    return "Unknown";
}

// Dữ liệu snapshot đi qua queue để loop không đọc thành viên class từ task khác.
struct ImuStats {
    KhungDuLieuMPU frame{};
    uint32_t ok = 0, errors = 0, missed = 0;
    uint32_t dtMinUs = UINT32_MAX, dtMaxUs = 0;
    uint32_t readMaxUs = 0, wakeMaxUs = 0;
    uint64_t lastOkUs = 0;
    bool fault = false;
};
struct MotorStats {
    MksSnapshot state{};
    MksResult lastResult = MksResult::Disabled;
    uint32_t serviceMaxUs = 0, serviceCallMaxUs = 0;
    bool blockedByImu = false;
};
struct BenchCommand { char code = 'd'; uint64_t validUntilUs = 0; };
QueueHandle_t imuQueue = nullptr, motorQueue = nullptr, commandQueue = nullptr;
TaskHandle_t imuHandle = nullptr, motorHandle = nullptr;
esp_timer_handle_t serviceTimer = nullptr;
portMUX_TYPE sampleMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t sampleSequence = 0;
volatile uint64_t sampleTimeUs = 0;
bool bootOk = false;

void ARDUINO_ISR_ATTR onImuDataReady() {
    const uint64_t now = clockUs();
    portENTER_CRITICAL_ISR(&sampleMux);
    sampleSequence = sampleSequence + 1u;
    sampleTimeUs = now;
    portEXIT_CRITICAL_ISR(&sampleMux);
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(imuHandle, &wake);
    if (wake == pdTRUE) portYIELD_FROM_ISR();
}

void imuWorker(void*) {
    ImuStats stats{};
    uint32_t previousSequence = 0;
    uint64_t previousUs = 0;
    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(
                (COMPARE_IMU_STALE_US + 999u) / 1000u)) == 0) {
            stats.fault = true;
            stats.frame = KhungDuLieuMPU{};
            stats.frame.trangthai = trangthaiKhungDuLieuMPU::DuLieuCu;
            xQueueOverwrite(imuQueue, &stats);
            vTaskSuspend(nullptr);
            continue;
        }
        uint32_t sequence;
        uint64_t timestamp;
        portENTER_CRITICAL(&sampleMux);
        sequence = sampleSequence;
        timestamp = sampleTimeUs;
        portEXIT_CRITICAL(&sampleMux);
        const uint32_t gap = sequence - previousSequence;
        if (!gap) continue;
        if (gap > 1) stats.missed += gap - 1;
        previousSequence = sequence;
        if (previousUs) {
            const uint32_t dt = static_cast<uint32_t>(timestamp - previousUs);
            if (dt < stats.dtMinUs) stats.dtMinUs = dt;
            if (dt > stats.dtMaxUs) stats.dtMaxUs = dt;
        }
        previousUs = timestamp;
        const uint64_t startUs = clockUs();
        const uint32_t wakeUs = static_cast<uint32_t>(startUs - timestamp);
        if (wakeUs > stats.wakeMaxUs) stats.wakeMaxUs = wakeUs;
        KhungDuLieuMPU frame{};
        const bool readOk = imu.read(frame);
        const uint64_t endUs = clockUs();
        const uint32_t readUs = static_cast<uint32_t>(endUs - startUs);
        if (readUs > stats.readMaxUs) stats.readMaxUs = readUs;
        uint32_t after;
        portENTER_CRITICAL(&sampleMux);
        after = sampleSequence;
        portEXIT_CRITICAL(&sampleMux);
        // Nghiêm ngặt: nếu data-ready khác tới trong lúc đọc, bỏ mẫu
        // vì không chắc timestamp trước đọc gắn đúng dữ liệu vừa nhận.
        if (!readOk || after != sequence || stats.missed) {
            stats.fault = true;
            ++stats.errors;
            frame.trangthai = readOk ? trangthaiKhungDuLieuMPU::DuLieuCu : trangthaiKhungDuLieuMPU::LoiIO;
        } else {
            ++stats.ok;
            frame.ThoiDiemLayMau_us = timestamp;
            frame.SoMauLayDuoc = stats.ok;
            stats.lastOkUs = endUs;
        }
        stats.frame = frame;
        xQueueOverwrite(imuQueue, &stats);
        if (stats.fault) vTaskSuspend(nullptr);
    }
}

// Callback esp_timer chạy từ task timer. Nó chỉ đánh thức MotorTask.
// Chu kỳ 250 us là mục tiêu; code đo gap thực, không giả định đúng tuyệt đối.
void motorTick(void*) { xTaskNotifyGive(motorHandle); }

void motorWorker(void*) {
    MotorStats stats{};
    float targetLeft = 0.0f, targetRight = 0.0f;
    uint64_t runUntilUs = 0, lastApplyUs = 0, lastServiceUs = 0;
    bool running = false, refreshCommands = true;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        uint64_t now = clockUs();
        ImuStats imuStats{};
        const bool imuHealthy = xQueuePeek(imuQueue, &imuStats, 0) == pdTRUE &&
            !imuStats.fault && imuStats.frame.trangthaiCalib && imuStats.lastOkUs &&
            now - imuStats.lastOkUs < COMPARE_IMU_STALE_US;

        BenchCommand command{};
        const bool got = xQueueReceive(commandQueue, &command, 0) == pdTRUE;
        if (got) {
            if (command.code == 'd') { motors.disable(); running = false; }
            else if (command.code == 'x') { motors.emergencyStop(); running = false; }
            else if (command.code == 'k') {
                motors.disable(); running = false;
                stats.lastResult = motors.clearFault() ? MksResult::Ok : MksResult::FaultLatched;
            } else if (command.code == 'f') {
                refreshCommands = false; // service vẫn chạy, chờ CommandExpired.
            } else if (command.code == 's' && running) {
                targetLeft = targetRight = 0.0f; // Giữ EN, gửi command 0 có heartbeat.
                lastApplyUs = 0;
            } else if (COMPARE_ALLOW_MOTOR_RUN && imuHealthy && now < command.validUntilUs) {
                const char c = command.code;
                const bool movement = c == 'l' || c == 'L' || c == 'r' || c == 'R' ||
                    c == 'b' || c == 'B' || c == 'c' || c == 'C';
                if (movement) {
                    // Nếu đang chạy, KHÔNG disable để thử trình tự đổi dấu V2.
                    stats.lastResult = motors.enable();
                    if (stats.lastResult == MksResult::Ok) {
                        const bool highRate = c == 'c' || c == 'C';
                        const float magnitude = highRate ? COMPARE_CONTINUOUS_RATE_HZ : COMPARE_SPARSE_RATE_HZ;
                        const bool reverse = c == 'L' || c == 'R' || c == 'B' || c == 'C';
                        const float signedRate = reverse ? -magnitude : magnitude;
                        targetLeft = c == 'r' || c == 'R' ? 0.0f : signedRate;
                        targetRight = c == 'l' || c == 'L' ? 0.0f : signedRate;
                        runUntilUs = command.validUntilUs;
                        running = true;
                        refreshCommands = true;
                        lastApplyUs = 0;
                        stats.blockedByImu = false;
                    }
                }
            } else if (!COMPARE_ALLOW_MOTOR_RUN) {
                // Các lệnh chuyển động không vượt qua cổng xác nhận phần cứng.
                stats.lastResult = MksResult::HardwareConfigUnconfirmed;
            }
        }

        now = clockUs();
        if (running && (!imuHealthy || now >= runUntilUs)) {
            if (!imuHealthy) { motors.emergencyStop(); stats.blockedByImu = true; }
            else motors.disable();
            running = false;
        }
        if (running && refreshCommands &&
            (lastApplyUs == 0 || now - lastApplyUs >= COMPARE_COMMAND_REFRESH_US)) {
            WheelStepCommand c{};
            c.leftStepsPerSecond = targetLeft;
            c.rightStepsPerSecond = targetRight;
            c.issuedAtUs = clockUs();
            c.expiresAtUs = c.issuedAtUs + MOTOR_COMMAND_TIMEOUT_US;
            stats.lastResult = motors.apply(c);
            lastApplyUs = c.issuedAtUs;
            if (stats.lastResult == MksResult::FaultLatched) running = false;
        }
        const uint64_t beforeService = clockUs();
        if (lastServiceUs && motors.isEnabled()) {
            const uint32_t gap = static_cast<uint32_t>(beforeService - lastServiceUs);
            if (gap > stats.serviceMaxUs) stats.serviceMaxUs = gap;
        }
        lastServiceUs = beforeService;
        const MksResult progress = motors.service();
        const uint32_t callUs = static_cast<uint32_t>(clockUs() - beforeService);
        if (callUs > stats.serviceCallMaxUs) stats.serviceCallMaxUs = callUs;
        if (progress == MksResult::FaultLatched) { running = false; stats.lastResult = progress; }
        stats.state = motors.snapshot();
        xQueueOverwrite(motorQueue, &stats);
    }
}

void printHelp() {
    Serial.println("l/L=trai +/- sparse; r/R=phai +/- sparse; b/B=hai banh +/- sparse");
    Serial.println("c/C=hai banh +/- continuous; s=command 0 giu EN; d=disable");
    Serial.println("f=ngung refresh de thu timeout; x=emergency; k=disable+clear fault; h=help");
    Serial.println("Dau +/- theo Config forward DIR, KHONG mac dinh la CW/CCW.");
}

void setup() {
    // Đặt EN disable trước khi startup chậm. Không có motor tự chạy sau reset.
    const auto enPin = static_cast<gpio_num_t>(PIN_EnMotor);
    if (gpio_set_level(enPin, MOTOR_EN_ACTIVE_HIGH ? 0 : 1) != ESP_OK ||
        gpio_set_direction(enPin, GPIO_MODE_OUTPUT) != ESP_OK) return;
    Serial.begin(COMPARE_SERIAL_BAUD);
    delay(COMPARE_CONSOLE_STARTUP_MS);
    printMotorConfig();

    if (!MPU_NgatIMU || COMPARE_COMMAND_REFRESH_US >= MOTOR_COMMAND_TIMEOUT_US ||
        COMPARE_SPARSE_RATE_HZ <= 0 || COMPARE_SPARSE_RATE_HZ >= MOTOR_CONTINUOUS_MIN_RATE_HZ ||
        COMPARE_CONTINUOUS_RATE_HZ < MOTOR_CONTINUOUS_MIN_RATE_HZ ||
        COMPARE_CONTINUOUS_RATE_HZ > MOTOR_MAX_STEP_RATE_HZ) {
        Serial.println("FAIL: config main tham khao khong phu hop."); return;
    }
    if (!motors.begin()) { Serial.println("FAIL: MKS begin; EN disable."); return; }
    if (!imu.begin() || !auditImuConfig()) { Serial.println("FAIL: MPU/init/readback."); motors.disable(); return; }
    Serial.println("Hieu chuan: giu yen, motor disable.");
    if (!imu.HieuChinhGyro(MPU_laymauCalib)) { Serial.println("FAIL: gyro calibration."); return; }

    imuQueue = xQueueCreate(1, sizeof(ImuStats));
    motorQueue = xQueueCreate(1, sizeof(MotorStats));
    commandQueue = xQueueCreate(1, sizeof(BenchCommand));
    if (!imuQueue || !motorQueue || !commandQueue) { Serial.println("FAIL: queues."); return; }
    ImuStats initial{};
    xQueueOverwrite(imuQueue, &initial);
    if (xTaskCreate(imuWorker, "ImuCompare", COMPARE_IMU_TASK_STACK_BYTES, nullptr,
        COMPARE_IMU_TASK_PRIORITY, &imuHandle) != pdPASS) { Serial.println("FAIL: IMU task."); return; }
    pinMode(PIN_INT_MPU6050, INPUT);
    attachInterrupt(PIN_INT_MPU6050, onImuDataReady, MPU_ActiveSignal ? RISING : FALLING);
    if (xTaskCreate(motorWorker, "MotorCompare", COMPARE_MOTOR_TASK_STACK_BYTES, nullptr,
        COMPARE_MOTOR_TASK_PRIORITY, &motorHandle) != pdPASS) {
        detachInterrupt(PIN_INT_MPU6050); vTaskDelete(imuHandle); Serial.println("FAIL: motor task."); return;
    }
    esp_timer_create_args_t timerArgs{};
    timerArgs.callback = motorTick;
    timerArgs.dispatch_method = ESP_TIMER_TASK;
    timerArgs.name = "MotorWake";
    timerArgs.skip_unhandled_events = true;
    if (esp_timer_create(&timerArgs, &serviceTimer) != ESP_OK ||
        esp_timer_start_periodic(serviceTimer, MOTOR_SERVICE_TARGET_US) != ESP_OK) {
        vTaskDelete(motorHandle);
        detachInterrupt(PIN_INT_MPU6050); vTaskDelete(imuHandle);
        Serial.println("FAIL: motor wake timer."); return;
    }
    bootOk = true;
    Serial.println("READY: doi chieu REGISTER la tu dong; menu MKS phai nhap/kiem tra thu cong.");
    printHelp();
}

void loop() {
    if (!bootOk) { delay(100); return; }
    for (uint8_t count = 0; count < 16 && Serial.available(); ++count) {
        const char c = static_cast<char>(Serial.read());
        if (c == 'h') { printHelp(); continue; }
        if (c == 'l' || c == 'L' || c == 'r' || c == 'R' || c == 'b' || c == 'B' ||
            c == 'c' || c == 'C' || c == 's' || c == 'd' || c == 'f' || c == 'x' || c == 'k') {
            BenchCommand command{};
            command.code = c;
            command.validUntilUs = clockUs() + COMPARE_RUN_WINDOW_US;
            xQueueOverwrite(commandQueue, &command);
        }
    }
    static uint64_t lastReport = clockUs();
    static uint32_t previousOk = 0;
    const uint64_t now = clockUs();
    if (now - lastReport >= COMPARE_REPORT_MS * 1000ULL) {
        ImuStats a{};
        MotorStats b{};
        if (xQueuePeek(imuQueue, &a, 0) == pdTRUE) {
            const float hz = (a.ok - previousOk) * 1000000.0f / (now - lastReport);
            const float norm = std::sqrt(a.frame.giatocX_mps2 * a.frame.giatocX_mps2 +
                a.frame.giatocY_mps2 * a.frame.giatocY_mps2 + a.frame.giatocZ_mps2 * a.frame.giatocZ_mps2);
            Serial.printf("IMU %.1fHz ok=%lu err=%lu miss=%lu dt=%lu..%luus readMax=%lu wakeMax=%lu fault=%d\n",
                hz, static_cast<unsigned long>(a.ok), static_cast<unsigned long>(a.errors),
                static_cast<unsigned long>(a.missed),
                static_cast<unsigned long>(a.dtMinUs == UINT32_MAX ? 0 : a.dtMinUs),
                static_cast<unsigned long>(a.dtMaxUs), static_cast<unsigned long>(a.readMaxUs),
                static_cast<unsigned long>(a.wakeMaxUs), a.fault);
            Serial.printf("IMU |A|=%.2fm/s2 gyro=%.4f,%.4f,%.4frad/s calib=%d\n",
                norm, a.frame.tocdoGocX_radps, a.frame.tocdoGocY_radps, a.frame.tocdoGocZ_radps, a.frame.trangthaiCalib);
            previousOk = a.ok;
        }
        if (xQueuePeek(motorQueue, &b, 0) == pdTRUE) {
            Serial.printf("PAIR EN=%d target=%.2f,%.2f backendHz=%.1f,%.1f pending=%d sat=%d "
                          "fault=%s last=%s gapMax=%luus callMax=%luus imuBlock=%d\n",
                b.state.enabled, b.state.requestedLeftStepsPerSecond, b.state.requestedRightStepsPerSecond,
                b.state.continuousLeftHz, b.state.continuousRightHz, b.state.pending, b.state.saturated,
                faultName(b.state.fault), resultName(b.lastResult),
                static_cast<unsigned long>(b.serviceMaxUs), static_cast<unsigned long>(b.serviceCallMaxUs), b.blockedByImu);
        }
        lastReport = now;
    }
    delay(1); // Chỉ console task; không dùng dòng này để định thời service 250 us.
}
