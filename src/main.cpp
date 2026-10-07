#include <Arduino.h>
#include <mpu6050Driver.h>

// Tạo driver, dùng bus I2C Wire.
mpu6050Drv imu(Wire);

// Lưu kết quả khởi tạo.
bool imuSanSang = false;

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Khởi tạo một lần khi ESP32 bắt đầu chạy.
    imuSanSang = imu.begin();
}

void loop() {
    if (imuSanSang) {
        Serial.println("MPU6050: khoi tao thanh cong");
    } else {
        Serial.println("MPU6050: khoi tao that bai");
    }

    delay(1000); // Chỉ dùng trong chương trình thử này.
}