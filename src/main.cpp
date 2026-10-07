#include <Arduino.h>
#include <esp_timer.h>
#include <cmath>
#include <mpu6050Driver.h>

mpu6050Drv imu(Wire);

bool imuSanSang = false;
uint32_t soMau = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    imuSanSang = imu.begin();

    Serial.println(
        imuSanSang ? "MPU san sang" : "Khoi tao MPU that bai"
    );
}

void loop() {
    if (!imuSanSang) {
        delay(1000);
        return;
    }

    KhungDuLieuMPU frame{};

    // Đo thời gian thực hiện hàm read().
    const uint64_t batDau_us =
        static_cast<uint64_t>(esp_timer_get_time());

    const bool docThanhCong = imu.read(frame);

    const uint32_t thoiGianDoc_us =
        static_cast<uint32_t>(esp_timer_get_time() - batDau_us);

    if (docThanhCong) {
        frame.ThoiDiemLayMau_us = batDau_us;
        frame.SoMauLayDuoc = ++soMau;

        const float doLonGiaToc = std::sqrt(
            frame.giatocX_mps2 * frame.giatocX_mps2 +
            frame.giatocY_mps2 * frame.giatocY_mps2 +
            frame.giatocZ_mps2 * frame.giatocZ_mps2
        );

        Serial.printf(
            "A[m/s2]: %.2f %.2f %.2f | |A|: %.2f | "
            "G[rad/s]: %.3f %.3f %.3f | read: %lu us\n",
            frame.giatocX_mps2,
            frame.giatocY_mps2,
            frame.giatocZ_mps2,
            doLonGiaToc,
            frame.tocdoGocX_radps,
            frame.tocdoGocY_radps,
            frame.tocdoGocZ_radps,
            static_cast<unsigned long>(thoiGianDoc_us)
        );
    } else {
        Serial.printf(
            "Loi doc MPU | read: %lu us\n",
            static_cast<unsigned long>(thoiGianDoc_us)
        );
    }

    delay(100); // Chỉ dành cho phép thử đọc và quan sát.
}