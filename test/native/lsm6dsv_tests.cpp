#include "SelectedImuDriver.h"
#include "config/ControlConfig.h"
#include "config/ImuConfig.h"
#include "config/PinConfig.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

#ifdef CHECK_ST_REFERENCE
// Interpret the hardware register image written by our actual driver using
// ST's independent C implementation, pinned in the bring-up document.
#include "lsm6dsv_reg.h"
static void checkStReference(SPIClass& bus) {
    stmdev_ctx_t ctx{};
    ctx.handle = &bus;
    ctx.read_reg = [](void* h, uint8_t r, uint8_t* out, uint16_t n) -> int32_t {
        const auto& regs = static_cast<SPIClass*>(h)->reg;
        if (r + n > regs.size()) return -1;
        std::memcpy(out, regs.data() + r, n);
        return 0;
    };
    lsm6dsv_data_rate_t rate;
    assert(lsm6dsv_get_xl_data_rate(&ctx, &rate) == 0 && rate == LSM6DSV_ODR_HA02_AT_200Hz);
    assert(lsm6dsv_get_gy_data_rate(&ctx, &rate) == 0 && rate == LSM6DSV_ODR_HA02_AT_200Hz);
    lsm6dsv_xl_mode_t xlMode;
    lsm6dsv_gy_mode_t gyMode;
    assert(lsm6dsv_get_xl_mode(&ctx, &xlMode) == 0 && xlMode == LSM6DSV_XL_HIGH_ACCURACY_ODR_MD);
    assert(lsm6dsv_get_gy_mode(&ctx, &gyMode) == 0 && gyMode == LSM6DSV_GY_HIGH_ACCURACY_ODR_MD);
    lsm6dsv_xl_full_scale_t xl;
    lsm6dsv_gy_full_scale_t gy;
    assert(lsm6dsv_get_xl_full_scale(&ctx, &xl) == 0 && xl == LSM6DSV_4g);
    assert(lsm6dsv_get_gy_full_scale(&ctx, &gy) == 0 && gy == LSM6DSV_500dps);
    lsm6dsv_filt_gy_lp1_bandwidth_t bw;
    assert(lsm6dsv_get_filt_gy_lp1_bandwidth(&ctx, &bw) == 0 && bw == LSM6DSV_GY_VERY_STRONG);
    lsm6dsv_filt_xl_lp2_bandwidth_t xlBw;
    assert(lsm6dsv_get_filt_xl_lp2_bandwidth(&ctx, &xlBw) == 0 && xlBw == LSM6DSV_XL_MEDIUM);
    uint8_t xlLp2En = 0;
    assert(lsm6dsv_get_filt_xl_lp2(&ctx, &xlLp2En) == 0 && xlLp2En == 1);
    uint8_t enabled;
    assert(lsm6dsv_get_block_data_update(&ctx, &enabled) == 0 && enabled);
    std::cout << "PASS: ST lsm6dsv-pid confirms gyro/accel mode, ODR, ranges, BDU and gyro/accel bandwidth\n";
}
#endif

int main() {
    static_assert(dungLsm6dsv && tanSoLayMauImu_hz == 200);
    static_assert(lsm6dsv::tanSoSpi_hz == 4000000 && lsm6dsv::locGiaTocLpf2 == 3 && lsm6dsv::batLocGiaTocLpf2);
    static_assert(cacChanChucNangKhongTrung() && chanNgatMpu == 7);
    assert(std::strcmp(nvsCauHinhImu, "xe_lsm6dsv") == 0);
    SPIClass bus;
    SelectedImuDriver imu(bus);
    assert(imu.begin());
#ifdef CHECK_ST_REFERENCE
    checkStReference(bus);
#endif
    assert(bus.reg[0x10] == 0x17 && bus.reg[0x11] == 0x17 && bus.reg[0x62] == 2);
    assert(bus.reg[0x17] == ((3u << 5) | 1u) && bus.reg[0x18] == 8u);
    assert(bus.reg[0x0D] == 2 && bus.reg[0x12] == 0x44);
    BaoCaoKiemTraIcm report;
    assert(imu.kiemTraSauKhoiDong(report) && report.soMauTot == 32);
    assert(imu.hieuChinhConQuay(32));
    assert(std::fabs(report.mauCuoi.giaTocZ_mps2 - 9.80665f) < .01f);

    KhungDuLieuImu m;
    bus.sampling = false; bus.ready = 3;
    bus.setWord(0x20, -256);
    bus.setWord(0x22, -1000);
    bus.setWord(0x28, -8197);
    assert(imu.docMauTheoNgat(m));
    assert(m.nhietDoCamBien_C == 24 && m.daHieuChinhConQuay && m.thoiDiemLayMau_us == 0);
    assert(std::fabs(m.tocDoGocX_radps + 17.5f * .01745329252f) < .00001f);
    assert(std::fabs(m.giaTocX_mps2 + 9.80665f) < .01f);
#ifdef CHECK_ST_REFERENCE
    assert(std::fabs(m.nhietDoCamBien_C - lsm6dsv_from_lsb_to_celsius(-256)) < .0001f);
    assert(std::fabs(m.tocDoGocX_radps - lsm6dsv_from_fs500_to_mdps(-1000) * .001f * .01745329252f) < .00001f);
    assert(std::fabs(m.giaTocX_mps2 - lsm6dsv_from_fs4_to_mg(-8197) * .001f * 9.80665f) < .0001f);
#endif
    assert(!imu.docMauTheoNgat(m) && m.trangThai == TrangThaiKhungDuLieuImu::DuLieuCu);
    bus.ready = 2;
    assert(!imu.docMauMoi(m) && bus.ready == 2);
    assert(!imu.docMauTheoNgat(m) && bus.ready == 0);
    bus.ready = 2; bus.readsUntilAccel = 3;
    assert(imu.docMauTheoNgat(m) && bus.ready == 0);
    bus.ready = 3; bus.forceAfterBurst = 3;
    assert(!imu.docMauTheoNgat(m) && m.trangThai == TrangThaiKhungDuLieuImu::DuLieuCu);
    bus.connected = false;
    assert(!imu.docMauTheoNgat(m) && m.trangThai == TrangThaiKhungDuLieuImu::LoiIO);

    SPIClass bad;
    SelectedImuDriver failing(bad);
    bad.reg[0x0F] = 0x12;
    assert(!failing.begin());
    bad.reg[0x0F] = 0x70; bad.corruptConfig = true;
    assert(!failing.begin() && failing.thanhGhiCuoi() == 0x62);
    bad.corruptConfig = false; bad.periodMs = 4;
    assert(failing.begin() && !failing.kiemTraSauKhoiDong(report));
    assert(std::strcmp(report.ketQua, "sai_nhip_mau") == 0);
    bad.periodMs = 5; bad.setWord(0x22, 1200);
    assert(!failing.hieuChinhConQuay(32));
    bad.sampling = false; bad.ready = 0;
    assert(!failing.kiemTraSauKhoiDong(report) && !failing.hieuChinhConQuay(32));

    // Kiem tra bien the thuc te LSM6DSV16BX (WHO_AM_I=0x71, ZYX accel order, 240 Hz HP, khong HAODR 0x62)
    // va ghi nhan static gravity norm thuc te (~9.42 m/s^2 tu 7875 LSB).
    dongHoGia_ms = 0;
    dongHoGia_subUs = 0;
    SPIClass bus16bx;
    bus16bx.reg[0x0F] = 0x71;
    bus16bx.setWord(0x2C, 0);
    bus16bx.setWord(0x28, 7875); // Z = ~9.42 m/s^2 tai OUTZ_L_A (0x28).
    bus16bx.periodUs = 4167;     // 240 Hz.
    SelectedImuDriver imu16bx(bus16bx);
    assert(imu16bx.begin());
    assert(imu16bx.laBienThe16Bx() && imu16bx.tanSoLayMauThucTe_hz() == 240);
    assert(bus16bx.reg[0x10] == 0x07 && bus16bx.reg[0x11] == 0x07 && bus16bx.reg[0x62] == 0);
    BaoCaoKiemTraIcm report16bx;
    assert(imu16bx.kiemTraSauKhoiDong(report16bx) && report16bx.soMauTot == 32);
    assert(std::fabs(imu16bx.normGiaTocTinh_mps2() - 9.421f) < 0.05f);
    assert(imu16bx.hieuChinhConQuay(32));
    assert(std::fabs(imu16bx.normGiaTocTinh_mps2() - 9.421f) < 0.05f);

    // Sai nhip (200 Hz thay vi 240 Hz) phai bi phat hien.
    bus16bx.periodUs = 5000;
    assert(!imu16bx.kiemTraSauKhoiDong(report16bx) && std::strcmp(report16bx.ketQua, "sai_nhip_mau") == 0);

    // Kiem tra thu tu Z, Y, X (0x28=Z, 0x2A=Y, 0x2C=X).
    bus16bx.sampling = false;
    bus16bx.ready = 3;
    bus16bx.setWord(0x28, 1000);  // Z
    bus16bx.setWord(0x2A, -2000); // Y
    bus16bx.setWord(0x2C, -8197); // X
    KhungDuLieuImu m16bx{};
    assert(imu16bx.docMauTheoNgat(m16bx));
    constexpr float aScale = lsm6dsv::giaToc_mgLsb * .001f * 9.80665f;
    assert(std::fabs(m16bx.giaTocZ_mps2 - 1000.0f * aScale) < 1e-4f);
    assert(std::fabs(m16bx.giaTocY_mps2 + 2000.0f * aScale) < 1e-4f);
    assert(std::fabs(m16bx.giaTocX_mps2 + 8197.0f * aScale) < 1e-4f);

    // A completed read can still be too late. Preserve its diagnostics, never
    // publish its payload, and accept the next independently ready sample.
    bus16bx.ready = 3;
    bus16bx.payloadDelayUs = lsm6dsv::timeoutGiaoDich_us;
    assert(!imu16bx.docMauTheoNgat(m16bx));
    assert(m16bx.trangThai == TrangThaiKhungDuLieuImu::LoiIO && m16bx.giaTocX_mps2 == 0);
    const auto diag = imu16bx.giaoDichDocLoiCuoi();
    assert(diag.maLoi == static_cast<uint8_t>(IcmI2cStatus::TIMEOUT) && diag.thanhGhi == 0x20);
    assert(diag.soByteNhan == 14 && diag.thoiGian_us == lsm6dsv::timeoutGiaoDich_us);
    assert(!imu16bx.docMauTheoNgat(m16bx) && m16bx.trangThai == TrangThaiKhungDuLieuImu::DuLieuCu);
    bus16bx.ready = 3;
    bus16bx.payloadDelayUs = 0;
    assert(imu16bx.docMauTheoNgat(m16bx) && imu16bx.maLoiI2cCuoi() == IcmI2cStatus::OK);

    // Kiem tra bus treo 0xFF tren STATUS_REG (bits 0x78) bi phat hien la LoiIO.
    bus16bx.connected = false;
    assert(!imu16bx.docMauTheoNgat(m16bx) && m16bx.trangThai == TrangThaiKhungDuLieuImu::LoiIO);

    std::cout << "PASS: LSM6DSV/16BX SPI profile, config/ODR, XYZ/ZYX scaling, DRDY, faults and calibration\n";
}
