// MAIN MẪU KIỂM THỬ MPU + MCPWM + MKS STEP/DIR.
// Bạn tự đặt vào src/main.cpp khi muốn thử tích hợp.
// Chưa có estimator/PID và KHÔNG cho xe tự đứng cân bằng.
// Không cần sửa API các thư viện hiện tại để chạy mẫu này.

#include <Arduino.h>
#include <esp_timer.h>
#include <cmath>
#include <limits>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <mpu6050Driver.h>
#include <McpwmStepOutput.h>
#include <MksServo42C.h>
#include "config/PinConfig.h"
#include "config/ImuConfig.h"
#include "config/MotorConfig.h"
#include "config/BenchTestConfig.h"

mpu6050Drv imu(Wire);
MksServo42CPair motors;
McpwmStepOutput stepTrai;
McpwmStepOutput stepPhai;

// Một queue dài 1 giữ snapshot mới nhất; không tích lũy mẫu cũ.
struct ImuReport {
    KhungDuLieuMPU frame{};
    uint32_t ok = 0;
    uint32_t errors = 0;
    uint32_t missed = 0;
    uint32_t readMaxUs = 0;
    uint32_t wakeMaxUs = 0;
    uint64_t readSumUs = 0;
    uint64_t lastOkUs = 0;
    uint32_t dtMinUs = UINT32_MAX;
    uint32_t dtMaxUs = 0;
    bool fault = false;
};

enum class MotorEvent : uint8_t {
    Ready, Running, Stopped, Expired, ImuFault, DriverError, ApiTestOk, ApiTestFailed
};

struct MotorReport {
    bool ready = false;
    bool running = false;
    bool enabled = false; // Lệnh EN do ESP32 đặt, không phải phản hồi MKS.
    bool fault = false;
    float requestedHz = 0.0f;
    char mode = '-';
    MotorEvent event = MotorEvent::Ready;
};

struct TestCommand {
    char code = 's';
    uint64_t expiresAtUs = 0;
};

QueueHandle_t imuReports = nullptr;
QueueHandle_t motorReports = nullptr;
QueueHandle_t motorCommands = nullptr;
TaskHandle_t imuTaskHandle = nullptr;
bool bootOk = false;

// Bảo vệ cặp sequence/timestamp 64-bit giữa ISR và task trên hai core.
portMUX_TYPE intMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t intSequence = 0;
volatile uint64_t latestIntUs = 0;

void ARDUINO_ISR_ATTR mpuDataReady() {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());

    portENTER_CRITICAL_ISR(&intMux);
    intSequence = intSequence + 1u;
    latestIntUs = now;
    portEXIT_CRITICAL_ISR(&intMux);

    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(imuTaskHandle, &wake);
    if (wake == pdTRUE) portYIELD_FROM_ISR();
}

// Sau setup, chỉ task này được gọi imu.read(). Không in Serial ở đây.
void imuTask(void*) {
    ImuReport report{};
    uint32_t previousSequence = 0;
    uint64_t previousIntUs = 0;

    for (;;) {
        // Task ngủ tới khi có INT; timeout giúp phát hiện mất tín hiệu INT.
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TEST_IMU_TIMEOUT_MS)) == 0) {
            report.fault = true;
            report.frame = KhungDuLieuMPU{};
            report.frame.trangthai = trangthaiKhungDuLieuMPU::DuLieuCu;
            xQueueOverwrite(imuReports, &report);
            vTaskSuspend(nullptr); // Reset để bắt đầu lại sau lỗi.
            continue;
        }

        uint32_t sequence;
        uint64_t sampleUs;
        portENTER_CRITICAL(&intMux);
        sequence = intSequence;
        sampleUs = latestIntUs;
        portEXIT_CRITICAL(&intMux);

        // Notification có thể gộp; sequence giúp phát hiện đã bỏ sự kiện.
        const uint32_t gap = sequence - previousSequence;
        if (gap == 0) continue;
        if (gap > 1) report.missed += gap - 1;
        previousSequence = sequence;

        if (previousIntUs != 0) {
            const uint32_t dt = static_cast<uint32_t>(sampleUs - previousIntUs);
            if (dt < report.dtMinUs) report.dtMinUs = dt;
            if (dt > report.dtMaxUs) report.dtMaxUs = dt;
        }
        previousIntUs = sampleUs;

        const uint64_t readStartUs = static_cast<uint64_t>(esp_timer_get_time());
        const uint32_t wakeUs = static_cast<uint32_t>(readStartUs - sampleUs);
        if (wakeUs > report.wakeMaxUs) report.wakeMaxUs = wakeUs;

        KhungDuLieuMPU frame{};
        const bool ok = imu.read(frame);
        const uint64_t readEndUs = static_cast<uint64_t>(esp_timer_get_time());
        const uint32_t readUs = static_cast<uint32_t>(readEndUs - readStartUs);
        if (readUs > report.readMaxUs) report.readMaxUs = readUs;

        uint32_t sequenceAfterRead;
        portENTER_CRITICAL(&intMux);
        sequenceAfterRead = intSequence;
        portEXIT_CRITICAL(&intMux);

        // Nếu có INT khác trong lúc đọc thì timestamp gắn với dữ liệu
        // không còn chắc chắn. Phép thử nghiêm ngặt này bỏ mẫu và báo lỗi.
        if (!ok || sequenceAfterRead != sequence) {
            ++report.errors;
            report.fault = true;
            frame.trangthai = ok ? trangthaiKhungDuLieuMPU::DuLieuCu
                                 : trangthaiKhungDuLieuMPU::LoiIO;
        } else {
            frame.ThoiDiemLayMau_us = sampleUs;
            frame.SoMauLayDuoc = ++report.ok;
            report.readSumUs += readUs;
            report.lastOkUs = readEndUs;
        }

        // Mất sự kiện là lý do dừng phép thử để kiểm tra lại tải/timing.
        if (report.missed != 0) report.fault = true;
        report.frame = frame;
        xQueueOverwrite(imuReports, &report);

        if (report.fault) vTaskSuspend(nullptr);
    }
}

// Chỉ MotorTask gọi hàm này sau khi các peripheral đã được tạo.
bool stopBenchMotors() {
    // Disable EN trước: nếu một API STEP lỗi, MKS vẫn nhận lệnh disable.
    motors.disable();
    const bool leftStopped = stepTrai.stop();
    const bool rightStopped = stepPhai.stop();
    return leftStopped && rightStopped;
}

// Task này quản lý STEP/DIR/EN và thời hạn chạy độc lập Serial/I2C.
void motorTask(void*) {
    MotorReport report{};
    report.ready = true;
    uint64_t deadlineUs = 0;

    for (;;) {
        TestCommand command{};
        const bool gotCommand = xQueueReceive(
            motorCommands, &command, pdMS_TO_TICKS(TEST_MOTOR_CHECK_MS)
        ) == pdTRUE;

        const uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());
        ImuReport imuReport{};
        const bool imuHealthy = xQueuePeek(imuReports, &imuReport, 0) == pdTRUE &&
            !imuReport.fault && imuReport.frame.trangthaiCalib &&
            imuReport.lastOkUs != 0 &&
            nowUs - imuReport.lastOkUs < TEST_IMU_TIMEOUT_MS * 1000ULL;

        if (!imuHealthy && report.running) {
            stopBenchMotors();
            report.running = false;
            report.fault = true;
            report.event = MotorEvent::ImuFault;
        }
        if (report.running && nowUs >= deadlineUs) {
            if (!stopBenchMotors()) report.fault = true;
            report.running = false;
            report.event = MotorEvent::Expired;
        }

        if (gotCommand) {
            const char c = command.code;
            if (c == 's' || c == 'd') {
                if (!stopBenchMotors()) report.fault = true;
                report.running = false;
                report.event = MotorEvent::Stopped;
            } else if (c == 't') {
                // Kiểm tra API với EN disable; không tự phát xung.
                const bool stopped = stopBenchMotors();
                report.running = false;
                const bool negativeRejected = !stepTrai.setFrequency(-1.0f);
                const bool nanRejected = !stepTrai.setFrequency(
                    std::numeric_limits<float>::quiet_NaN());
                const bool infinityRejected = !stepTrai.setFrequency(
                    std::numeric_limits<float>::infinity());
                const bool tooSlowRejected = !stepTrai.setFrequency(0.000001f);
                const bool tooFastRejected = !stepTrai.setFrequency(
                    std::numeric_limits<float>::max());
                const bool zeroAccepted = stepTrai.setFrequency(0.0f);
                const bool passed = stopped && negativeRejected && nanRejected &&
                    infinityRejected && tooSlowRejected && tooFastRejected && zeroAccepted;
                report.event = passed ? MotorEvent::ApiTestOk : MotorEvent::ApiTestFailed;
                if (!passed) report.fault = true;
            } else if (imuHealthy && !report.fault && nowUs < command.expiresAtUs) {
                if (c == 'u' || c == 'v') {
                    // Đổi tần số khi đang chạy: không đổi DIR, không stop trước.
                    if (report.running) {
                        const float hz = c == 'u' ? TEST_STEP_SECOND_FREQUENCY_HZ
                                                  : TEST_STEP_FREQUENCY_HZ;
                        const bool leftActive = report.mode != 'r' && report.mode != 'R';
                        const bool rightActive = report.mode != 'l' && report.mode != 'L';
                        const bool a = stepTrai.setFrequency(leftActive ? hz : 0.0f);
                        const bool b = stepPhai.setFrequency(rightActive ? hz : 0.0f);
                        if (a && b) {
                            report.requestedHz = hz;
                            deadlineUs = command.expiresAtUs;
                        } else {
                            stopBenchMotors();
                            report.running = false;
                            report.fault = true;
                            report.event = MotorEvent::DriverError;
                        }
                    }
                } else if (c == 'l' || c == 'L' || c == 'r' || c == 'R' || c == 'b' || c == 'B') {
                    // Đổi DIR chỉ khi đã disable công suất và dừng STEP.
                    if (stopBenchMotors()) {
                        delayMicroseconds(TEST_DIR_GUARD_US);
                        const bool dirHigh = c == 'l' || c == 'r' || c == 'b';
                        digitalWrite(PIN_DirL, dirHigh ? HIGH : LOW);
                        digitalWrite(PIN_DirR, dirHigh ? HIGH : LOW);
                        delayMicroseconds(TEST_DIR_GUARD_US);

                        if (TEST_ALLOW_MOTOR_ENABLE) motors.enable();
                        const bool leftActive = c != 'r' && c != 'R';
                        const bool rightActive = c != 'l' && c != 'L';
                        const bool a = stepTrai.setFrequency(
                            leftActive ? TEST_STEP_FREQUENCY_HZ : 0.0f);
                        const bool b = stepPhai.setFrequency(
                            rightActive ? TEST_STEP_FREQUENCY_HZ : 0.0f);

                        if (a && b) {
                            report.running = true;
                            report.mode = c;
                            report.requestedHz = TEST_STEP_FREQUENCY_HZ;
                            deadlineUs = command.expiresAtUs;
                            report.event = MotorEvent::Running;
                        } else {
                            stopBenchMotors();
                            report.running = false;
                            report.fault = true;
                            report.event = MotorEvent::DriverError;
                        }
                    } else {
                        report.running = false;
                        report.fault = true;
                        report.event = MotorEvent::DriverError;
                    }
                }
            }
        }

        report.enabled = motors.isEnabled();
        if (!report.running) report.requestedHz = 0.0f;
        xQueueOverwrite(motorReports, &report);
    }
}

void printHelp() {
    Serial.println("l/L: STEP trai, DIR HIGH/LOW; r/R: STEP phai, DIR HIGH/LOW");
    Serial.println("b/B: ca hai STEP, DIR HIGH/LOW; u: doi sang tan so 2; v: tan so 1");
    Serial.println("s/d: stop + disable; t: test input va gioi han timer; h: huong dan");
    Serial.println("Moi lenh chay tu dung sau thoi han Config; khong co PID.");
}

void setup() {
    // EN disable trước các bước khởi tạo chậm và trước khi có MCPWM.
    if (!motors.begin()) return;
    motors.disable();
    Serial.begin(115200);
    delay(500);

    // Không đụng GPIO STEP bằng pinMode() sau khi MCPWM đã tiếp quản.
    const bool a = stepTrai.begin(PIN_StepL, TEST_STEP_HIGH_US, TEST_STEP_MIN_LOW_US);
    const bool b = stepPhai.begin(PIN_StepR, TEST_STEP_HIGH_US, TEST_STEP_MIN_LOW_US);
    if (!a || !b) {
        motors.disable();
        if (a) stepTrai.stop();
        if (b) stepPhai.stop();
        Serial.println("FAIL: MCPWM begin. EN giu disable.");
        return;
    }

    if (!imu.begin()) {
        stopBenchMotors();
        Serial.println("FAIL: MPU begin.");
        return;
    }
    Serial.println("Hieu chuan gyro: giu xe dung yen, motor disable...");
    if (!imu.HieuChinhGyro(MPU_laymauCalib)) {
        stopBenchMotors();
        Serial.println("FAIL: calibration. Giu yen va reset de thu lai.");
        return;
    }

    imuReports = xQueueCreate(1, sizeof(ImuReport));
    motorReports = xQueueCreate(1, sizeof(MotorReport));
    motorCommands = xQueueCreate(1, sizeof(TestCommand));
    if (!imuReports || !motorReports || !motorCommands) {
        stopBenchMotors();
        Serial.println("FAIL: queue allocation.");
        return;
    }

    // Khởi tạo dữ liệu để motor không chạy trước mẫu IMU hợp lệ đầu tiên.
    ImuReport initialImu{};
    xQueueOverwrite(imuReports, &initialImu);

    if (xTaskCreate(imuTask, "ImuBench", 4096, nullptr, 3, &imuTaskHandle) != pdPASS) {
        stopBenchMotors();
        Serial.println("FAIL: IMU task.");
        return;
    }
    pinMode(PIN_INT_MPU6050, INPUT);
    attachInterrupt(PIN_INT_MPU6050, mpuDataReady,
                    MPU_ActiveSignal ? RISING : FALLING);

    if (xTaskCreate(motorTask, "MotorBench", 4096, nullptr, 4, nullptr) != pdPASS) {
        detachInterrupt(PIN_INT_MPU6050);
        vTaskDelete(imuTaskHandle);
        stopBenchMotors();
        Serial.println("FAIL: motor task.");
        return;
    }

    bootOk = true;
    Serial.printf("READY: motor enable permission = %d\n", TEST_ALLOW_MOTOR_ENABLE);
    printHelp();
}

void loop() {
    if (!bootOk) {
        delay(100);
        return;
    }

    // Không dùng readString()/parseFloat(): các hàm đó có thể chờ lâu.
    // Mỗi lượt xử lý tối đa 16 ký tự để Serial không chiếm hết loop().
    for (uint8_t n = 0; n < 16 && Serial.available() > 0; ++n) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r' || c == '\n' || c == ' ') continue;
        if (c == 'h') {
            printHelp();
            continue;
        }
        if (c == 'l' || c == 'L' || c == 'r' || c == 'R' || c == 'b' || c == 'B' ||
            c == 'u' || c == 'v' || c == 's' || c == 'd' || c == 't') {
            TestCommand command{};
            command.code = c;
            command.expiresAtUs = static_cast<uint64_t>(esp_timer_get_time())
                + TEST_MOTOR_RUN_MS * 1000ULL;
            xQueueOverwrite(motorCommands, &command);
        }
    }

    static uint64_t lastReportUs = static_cast<uint64_t>(esp_timer_get_time());
    static uint32_t previousOk = 0;
    const uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());
    if (nowUs - lastReportUs >= TEST_REPORT_PERIOD_MS * 1000ULL) {
        ImuReport a{};
        MotorReport b{};
        if (xQueuePeek(imuReports, &a, 0) == pdTRUE) {
            const float rateHz = (a.ok - previousOk) * 1000000.0f
                / static_cast<float>(nowUs - lastReportUs);
            const float avgUs = a.ok ? static_cast<float>(a.readSumUs) / a.ok : 0.0f;
            const float normA = std::sqrt(a.frame.giatocX_mps2 * a.frame.giatocX_mps2 +
                a.frame.giatocY_mps2 * a.frame.giatocY_mps2 +
                a.frame.giatocZ_mps2 * a.frame.giatocZ_mps2);
            Serial.printf("IMU %.1f Hz, ok=%lu err=%lu miss=%lu dt=%lu..%lu us "
                          "read(avg/max)=%.0f/%lu us wakeMax=%lu us fault=%d\n",
                rateHz, static_cast<unsigned long>(a.ok), static_cast<unsigned long>(a.errors),
                static_cast<unsigned long>(a.missed),
                static_cast<unsigned long>(a.dtMinUs == UINT32_MAX ? 0 : a.dtMinUs),
                static_cast<unsigned long>(a.dtMaxUs), avgUs,
                static_cast<unsigned long>(a.readMaxUs), static_cast<unsigned long>(a.wakeMaxUs), a.fault);
            Serial.printf("|A|=%.2f m/s2, gyro=%.4f %.4f %.4f rad/s, calib=%d\n",
                normA, a.frame.tocdoGocX_radps, a.frame.tocdoGocY_radps,
                a.frame.tocdoGocZ_radps, a.frame.trangthaiCalib);
            previousOk = a.ok;
        }
        if (xQueuePeek(motorReports, &b, 0) == pdTRUE) {
            Serial.printf("MOTOR pulse=%d EN=%d cmdHz=%.0f mode=%c event=%u fault=%d\n",
                b.running, b.enabled, b.requestedHz, b.mode,
                static_cast<unsigned>(b.event), b.fault);
        }
        lastReportUs = nowUs;
    }

    delay(1); // Chỉ task Serial nghỉ; IMU/MotorTask vẫn chạy độc lập.
}
