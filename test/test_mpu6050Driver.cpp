#include <Arduino.h>
#include <esp_timer.h>
#include <cmath>
#include <mpu6050Driver.h>

mpu6050Drv camBienMpu(Wire);

bool camBienMpuSanSang = false;
uint32_t soMau = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    camBienMpuSanSang = camBienMpu.begin();

    Serial.println(
        camBienMpuSanSang ? "MPU san sang" : "Khoi tao MPU that bai"
    );
}

void loop() {
    if (!camBienMpuSanSang) {
        delay(1000);
        return;
    }

    KhungDuLieuMPU khungDuLieu{};

    // Đo thời gian thực hiện hàm read().
    const uint64_t batDau_us =
        static_cast<uint64_t>(esp_timer_get_time());

    const bool docThanhCong = camBienMpu.read(khungDuLieu);

    const uint32_t thoiGianDoc_us =
        static_cast<uint32_t>(esp_timer_get_time() - batDau_us);

    if (docThanhCong) {
        khungDuLieu.thoiDiemLayMau_us = batDau_us;
        khungDuLieu.soMauLayDuoc = ++soMau;

        const float doLonGiaToc = std::sqrt(
            khungDuLieu.giaTocX_mps2 * khungDuLieu.giaTocX_mps2 +
            khungDuLieu.giaTocY_mps2 * khungDuLieu.giaTocY_mps2 +
            khungDuLieu.giaTocZ_mps2 * khungDuLieu.giaTocZ_mps2
        );

        Serial.printf(
            "A[m/s2]: %.2f %.2f %.2f | |A|: %.2f | "
            "G[rad/s]: %.3f %.3f %.3f | read: %lu us\n",
            khungDuLieu.giaTocX_mps2,
            khungDuLieu.giaTocY_mps2,
            khungDuLieu.giaTocZ_mps2,
            doLonGiaToc,
            khungDuLieu.tocDoGocX_radps,
            khungDuLieu.tocDoGocY_radps,
            khungDuLieu.tocDoGocZ_radps,
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