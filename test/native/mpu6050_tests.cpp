#include "SelectedImuDriver.h"
#include "config/ControlConfig.h"
#include "config/ImuConfig.h"
#include "config/PinConfig.h"
#include "driver/gpio.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

int main() {
    static_assert(dungMpu6050 && !dungLsm6dsv);
    static_assert(tanSoLayMauImu_hz == 200 && tanSoBusImu_hz == 400000);
    static_assert(!dungNgatImu && chanNgatMpu == -1);
    static_assert(chanSdaMpu == 10 && chanSclMpu == 11 && chanAd0Mpu == 7);
    static_assert(cacChanChucNangKhongTrung());
    assert(std::strcmp(nvsCauHinhImu, "xe_mpu6050") == 0);
    assert(std::strcmp(tenImu, "MPU6050") == 0);

    TwoWire bus;
    bus.thanhGhi[0x75] = 0x68; // WHO_AM_I chuẩn của MPU6050
    mucGpioGia[7] = 1;         // Giả lập trước đó GPIO7 đang ở mức cao

    SelectedImuDriver imu(bus);
    assert(imu.begin());

    // 1. Kiểm tra chân phần cứng theo sơ đồ người dùng: SDA=10, SCL=11, AD0=IO7 kéo xuống LOW (0x68).
    assert(bus.chanSda == 10 && bus.chanScl == 11);
    assert(mucGpioGia[7] == 0);

    // 2. Kiểm tra thanh ghi cấu hình & khóa bảo vệ mạch phụ XCL/XDA (tắt I2C_MST và I2C_BYPASS_EN).
    assert(bus.soLanReset == 1);
    assert(bus.thanhGhi[0x19] == 4);    // SMPLRT_DIV = 4 -> 200 Hz
    assert(bus.thanhGhi[0x1A] == 3);    // DLPF_CFG = 3 (Gyro 42 Hz, Accel 44 Hz)
    assert(bus.thanhGhi[0x1B] == 0x08); // ±500 dps
    assert(bus.thanhGhi[0x1C] == 0x08); // ±4 g
    assert(bus.thanhGhi[0x23] == 0x00); // FIFO tắt
    assert(bus.thanhGhi[0x37] == 0x00); // INT_PIN_CFG = 0x00 (I2C_BYPASS_EN = 0!)
    assert(bus.thanhGhi[0x38] == 0x01); // DATA_RDY_EN = 1 để đọc đồng bộ qua 0x3A dù INT trống
    assert(bus.thanhGhi[0x6A] == 0x00); // USER_CTRL = 0x00 (I2C_MST_EN = 0!)
    assert(bus.thanhGhi[0x6B] == 0x01); // CLKSEL = PLL trục X Gyro
    assert(bus.thanhGhi[0x6C] == 0x00);

    // 3. Kiểm tra sau khởi động (32 mẫu liên tiếp đúng nhịp 200 Hz = 5 ms/mẫu).
    BaoCaoKiemTraIcm baoCao{};
    assert(imu.kiemTraSauKhoiDong(baoCao) && std::strcmp(baoCao.ketQua, "PASS") == 0);
    assert(baoCao.soMauTot == 32 && baoCao.soMauLoi == 0);

    // 4. Kiểm tra hệ số quy đổi đơn vị gia tốc (±4g = 8192 LSB/g), gyro (±500 dps = 65.5 LSB/dps),
    //    nhiệt độ chuẩn MPU6050 (TEMP/340 + 36.53) và trừ bias tĩnh.
    bus.taoMau = [](uint32_t) -> std::array<int16_t, 7> {
        // ax = -8192 (-1g), ay = 0, az = 0, temp = -3400 (-10 + 36.53 = 26.53 C), gx = 655 (10 dps)
        return {0, 0, 8192, -3400, 65, -131, 0}; // ~ +0.992 dps, -2.0 dps, 0 dps
    };
    assert(imu.hieuChinhConQuay(32));
    float bx = 0, by = 0, bz = 0;
    bool biasOk = false;
    imu.layDoLechConQuay_dps(bx, by, bz, biasOk);
    assert(biasOk && std::fabs(bx - (65.0f / 65.5f)) < 1e-3f && std::fabs(by + 2.0f) < 1e-3f);

    bus.taoMau = [](uint32_t) -> std::array<int16_t, 7> {
        return {-8192, 0, 0, -3400, 655 + 65, -131, 0}; // Sau trừ bias gx còn đúng +10.0 dps
    };
    dongHoGia_ms += 5;
    KhungDuLieuImu mau{};
    assert(imu.docMauMoi(mau) && mau.trangThai == TrangThaiKhungDuLieuImu::HopLe);
    assert(mau.daHieuChinhConQuay);
    assert(std::fabs(mau.giaTocX_mps2 + 9.80665f) < 1e-3f);
    assert(std::fabs(mau.nhietDoCamBien_C - 26.53f) < 1e-3f);
    assert(std::fabs(mau.tocDoGocX_radps - 10.0f * 0.01745329252f) < 1e-4f);
    assert(std::fabs(mau.tocDoGocY_radps) < 1e-4f);

    // Đọc lặp trong cùng chu kỳ 5 ms khi chưa có DATA_RDY mới phải trả về DuLieuCu.
    assert(!imu.docMauMoi(mau) && mau.trangThai == TrangThaiKhungDuLieuImu::DuLieuCu);

    // 5. Kiểm tra từ chối an toàn nếu cầu nối I2C_BYPASS_EN hoặc I2C_MST_EN bị bật ngoài ý muốn.
    bus.thanhGhi[0x37] = 0x02; // Giả lập lỗi bật I2C_BYPASS_EN (nguy hiểm khi XCL/XDA nối cứng)
    assert(!imu.kiemTraSauKhoiDong(baoCao) && std::strcmp(baoCao.ketQua, "sai_cau_hinh") == 0);
    bus.thanhGhi[0x37] = 0x00;
    bus.thanhGhi[0x6A] = 0x20; // Giả lập lỗi bật I2C_MST_EN
    assert(!imu.kiemTraSauKhoiDong(baoCao) && std::strcmp(baoCao.ketQua, "sai_cau_hinh") == 0);
    bus.thanhGhi[0x6A] = 0x00;

    // 6. Kiểm tra phát hiện sai nhịp lấy mẫu (ví dụ clock chạy nhanh gấp 2.5 lần = 2 ms/mẫu).
    bus.chuKyMau_ms = 2;
    assert(!imu.kiemTraSauKhoiDong(baoCao) && std::strcmp(baoCao.ketQua, "sai_nhip_mau") == 0);

    // 7. Kiểm tra từ chối sai mã WHO_AM_I (ví dụ gắn nhầm ICM20602 0x12 hoặc clone lạ).
    TwoWire busSaiId;
    busSaiId.thanhGhi[0x75] = 0x12;
    SelectedImuDriver imuSaiId(busSaiId);
    assert(!imuSaiId.begin() && imuSaiId.maNhanDang() == 0x12);

    std::cout << "PASS: MPU6050 I2C profile (AD0=IO7, SDA=10, SCL=11, INT=none), XCL/XDA safety lock, scaling & rate checks\n";
    return 0;
}
