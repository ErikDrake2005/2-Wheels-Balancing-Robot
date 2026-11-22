#include "config/ImuProfile.h"
#if CAU_HINH_IMU == IMU_LSM6DSV_SPI
#include "Lsm6dsvDriver.h"
#include "ThongKeMauImu.h"
#include <cmath>
#include <cstring>

// ST DS13476 Rev 5, AN5922; register fields cross-checked against
// STMicroelectronics/lsm6dsv-pid @ ba9ea4ea331184416c0be47616e64a51200f0c23.
// See test/native/lsm6dsv_tests.cpp for the check against ST's reference functions.
namespace {
constexpr uint8_t who = 0x0F, ctrl1 = 0x10, ctrl2 = 0x11, ctrl3 = 0x12;
constexpr uint8_t statusReg = 0x1E, outTemp = 0x20;
constexpr uint8_t configOdr = 0x10u | lsm6dsv::maOdr; // HAODR + chosen ODR.
constexpr uint8_t configGyro = (lsm6dsv::locGyroLpf1 << 4) | lsm6dsv::maGyro;
constexpr uint8_t configAccel = (lsm6dsv::locGiaTocLpf2 << 5) | lsm6dsv::maGiaToc;
struct RegValue { uint8_t reg, value; };
constexpr RegValue settings[] = {
    {0x03, static_cast<uint8_t>(0x01u | (lsm6dsv::mucNgatCao ? 0u : 0x10u))}, // SPI only, 4-wire, push-pull.
    {ctrl3, 0x44}, // BDU and IF_INC; no reset/boot.
    {0x13, 0x08}, // Mask DRDY until filters settle; latched (DRDY_PULSED=0).
    {0x15, configGyro}, {0x16, lsm6dsv::batLocGyroLpf1 ? 1u : 0u},
    {0x17, configAccel}, {0x18, lsm6dsv::batLocGiaTocLpf2 ? 8u : 0u},
    {0x62, 0x02}, // HAODR set 12.5/25/.../200/... Hz, before enabling sensors.
    {0x0A, 0x00}, // FIFO bypass: no queued old samples in balance loop.
    {0x0E, 0x00}, {0x0D, 0x02}, // INT1 gyro-ready only, INT2 unused.
    {ctrl1, configOdr}, {ctrl2, configOdr}
};
int16_t le16(const uint8_t* p) {
    const uint16_t u = static_cast<uint16_t>(p[0]) | static_cast<uint16_t>(p[1]) << 8;
    return static_cast<int16_t>(u < 0x8000 ? static_cast<int32_t>(u) : static_cast<int32_t>(u) - 65536);
}
}

bool Lsm6dsvDriver::loi(IcmI2cStatus s, uint8_t reg) {
    _loiCuoi = s; _thanhGhiCuoi = reg;
    _giaoDichCuoi.maLoi = static_cast<uint8_t>(s);
    _giaoDichCuoi.thanhGhi = reg;
    _giaoDichLoi = _giaoDichCuoi;
    return false;
}

bool Lsm6dsvDriver::doc(uint8_t reg, uint8_t* data, size_t n) {
    _giaoDichCuoi = {};
    _thanhGhiCuoi = reg; _giaoDichCuoi.thanhGhi = reg;
    _giaoDichCuoi.soByteYeuCau = static_cast<uint8_t>(n);
    if (!data || !n || n > 16) return loi(IcmI2cStatus::INVALID_PARAM, reg);
    // Send address and dummy bytes in one hardware exchange so the task cannot
    // be preempted between them with CS held low. ESP32 SPI HAL accesses words.
    uint32_t frame[5]{}; // 1 address + up to 16 payload bytes, rounded up.
    auto* bytes = reinterpret_cast<uint8_t*>(frame);
    bytes[0] = reg | 0x80u;
    const uint32_t start = micros();
    _bus.beginTransaction(SPISettings(lsm6dsv::tanSoSpi_hz, MSBFIRST, SPI_MODE3));
    digitalWrite(lsm6dsv::chanCs, LOW);
    _bus.transferBytes(bytes, bytes, n + 1);
    digitalWrite(lsm6dsv::chanCs, HIGH);
    _bus.endTransaction();
    _giaoDichCuoi.thoiGian_us = micros() - start;
    _giaoDichCuoi.soByteNhan = _giaoDichCuoi.soByteDaDoc = static_cast<uint8_t>(n);
    if (_giaoDichCuoi.thoiGian_us >= lsm6dsv::timeoutGiaoDich_us) return loi(IcmI2cStatus::TIMEOUT, reg);
    std::memcpy(data, bytes + 1, n);
    _loiCuoi = IcmI2cStatus::OK;
    return true; // SPI has no ACK/CRC; identity/config checks detect stuck/disconnected/reset devices.
}

bool Lsm6dsvDriver::ghi(uint8_t reg, uint8_t value) {
    const uint32_t start = micros();
    _bus.beginTransaction(SPISettings(lsm6dsv::tanSoSpi_hz, MSBFIRST, SPI_MODE3));
    digitalWrite(lsm6dsv::chanCs, LOW);
    _bus.transfer(reg & 0x7fu); _bus.transfer(value);
    digitalWrite(lsm6dsv::chanCs, HIGH);
    _bus.endTransaction();
    _thanhGhiCuoi = reg;
    if (micros() - start >= lsm6dsv::timeoutGiaoDich_us) return loi(IcmI2cStatus::TIMEOUT, reg);
    _loiCuoi = IcmI2cStatus::OK;
    return true;
}

bool Lsm6dsvDriver::ghiKiemTra(uint8_t reg, uint8_t value) {
    uint8_t actual = 0;
    if (!ghi(reg, value) || !doc(reg, &actual, 1)) return false;
    return actual == value || loi(IcmI2cStatus::OTHER_ERROR, reg);
}

bool Lsm6dsvDriver::cauHinhHopLe() {
    uint8_t id = 0;
    if (!doc(who, &id, 1)) return false;
    if (id != _maNhanDang || (_maNhanDang != lsm6dsv::maNhanDang && _maNhanDang != lsm6dsv::maNhanDang16Bx)) {
        return loi(IcmI2cStatus::OTHER_ERROR, who);
    }
    for (const auto& entry : settings) {
        if (_laBienThe16Bx && entry.reg == 0x62u) continue; // LSM6DSV16BX không có HAODR_CFG (0x62).
        const uint8_t expected = (entry.reg == ctrl1 || entry.reg == ctrl2) ? maOdrHoatDong() : entry.value;
        uint8_t value = 0;
        if (!doc(entry.reg, &value, 1)) return false;
        if (value != expected) return loi(IcmI2cStatus::OTHER_ERROR, entry.reg);
    }
    return true;
}

uint8_t Lsm6dsvDriver::maOdrHoatDong() const {
    return _laBienThe16Bx ? lsm6dsv::maOdr : configOdr;
}

bool Lsm6dsvDriver::begin() {
    _daKhoiTao = _daHieuChinh = _laBienThe16Bx = false;
    _maNhanDang = 0; _loiCuoi = IcmI2cStatus::OK;
    _giaoDichCuoi = _giaoDichLoi = {};
    _normGiaTocTinh_mps2 = 9.80665f;
    for (float& b : _bias) b = 0;
    pinMode(lsm6dsv::chanCs, OUTPUT);
    digitalWrite(lsm6dsv::chanCs, HIGH);
    pinMode(lsm6dsv::chanMiso, INPUT); // IMU drives SDO/SA0 in SPI mode.
    pinMode(lsm6dsv::chanInt, INPUT);
    if (!_bus.begin(lsm6dsv::chanSck, lsm6dsv::chanMiso, lsm6dsv::chanMosi, lsm6dsv::chanCs))
        return loi(IcmI2cStatus::OTHER_ERROR, who);
    delay(10);
    if (!doc(who, &_maNhanDang, 1)) return false;
    if (_maNhanDang != lsm6dsv::maNhanDang && _maNhanDang != lsm6dsv::maNhanDang16Bx) {
        delay(2);
        if (!doc(who, &_maNhanDang, 1)) return false;
    }
    if (_maNhanDang == lsm6dsv::maNhanDang16Bx) _laBienThe16Bx = true;
    else if (_maNhanDang != lsm6dsv::maNhanDang) return loi(IcmI2cStatus::OTHER_ERROR, who);
    if (!ghi(ctrl3, 0x01)) return false;
    const uint32_t start = millis();
    uint8_t reset = 1;
    do {
        delay(1);
        if (!doc(ctrl3, &reset, 1)) return false;
        if (millis() - start >= lsm6dsv::timeoutReset_ms) return loi(IcmI2cStatus::TIMEOUT, ctrl3);
    } while (reset & 1u);
    if (!ghiKiemTra(ctrl1, 0) || !ghiKiemTra(ctrl2, 0)) return false;
    for (const auto& entry : settings) {
        if (_laBienThe16Bx && entry.reg == 0x62u) continue;
        const uint8_t target = (entry.reg == ctrl1 || entry.reg == ctrl2) ? maOdrHoatDong() : entry.value;
        if (!ghiKiemTra(entry.reg, target)) return false;
    }
    delay(lsm6dsv::thoiGianOnDinh_ms);
    if (!cauHinhHopLe()) return false;
    // Consume pending output (reading gyro MSB ACKs latched DRDY, NOT reading STATUS).
    uint8_t discarded[14]{};
    if (!doc(outTemp, discarded, sizeof discarded)) return false;
    _daKhoiTao = true;
    return true;
}

bool Lsm6dsvDriver::giaiMa(const uint8_t* b, KhungDuLieuImu& m) const {
    constexpr float rad = 0.01745329252f;
    constexpr float aScale = lsm6dsv::giaToc_mgLsb * .001f * 9.80665f;
    m.nhietDoCamBien_C = le16(b) / 256.0f + 25.0f;
    m.tocDoGocX_radps = (le16(b + 2) * lsm6dsv::gyro_dpsLsb - _bias[0]) * rad;
    m.tocDoGocY_radps = (le16(b + 4) * lsm6dsv::gyro_dpsLsb - _bias[1]) * rad;
    m.tocDoGocZ_radps = (le16(b + 6) * lsm6dsv::gyro_dpsLsb - _bias[2]) * rad;
    if (_laBienThe16Bx) {
        // ST lsm6dsv16bx_reg.h: OUTZ_L_A=0x28, OUTY_L_A=0x2A, OUTX_L_A=0x2C (thứ tự Z, Y, X).
        m.giaTocZ_mps2 = le16(b + 8) * aScale;
        m.giaTocY_mps2 = le16(b + 10) * aScale;
        m.giaTocX_mps2 = le16(b + 12) * aScale;
    } else {
        m.giaTocX_mps2 = le16(b + 8) * aScale;
        m.giaTocY_mps2 = le16(b + 10) * aScale;
        m.giaTocZ_mps2 = le16(b + 12) * aScale;
    }
    m.daHieuChinhConQuay = _daHieuChinh;
    m.trangThai = TrangThaiKhungDuLieuImu::HopLe;
    return true;
}

bool Lsm6dsvDriver::read(KhungDuLieuImu& m) {
    // Unlike an unconditional snapshot, every public read requires new accel AND gyro.
    return docMauMoi(m);
}

bool Lsm6dsvDriver::docMauMoi(KhungDuLieuImu& m) {
    return docMau(m, false);
}

bool Lsm6dsvDriver::docMauTheoNgat(KhungDuLieuImu& m) {
    return docMau(m, true);
}

bool Lsm6dsvDriver::docMau(KhungDuLieuImu& m, bool theoNgat) {
    m = {};
    if (!_daKhoiTao) return false;
    const uint32_t start = micros();
    uint8_t identityAndOdr[3]{}, status = 0, after = 0, data[14]{};
    // WHO_AM_I+CTRL1+CTRL2 are contiguous. Detect removed sensor, stuck MISO and reset.
    if (!doc(who, identityAndOdr, sizeof identityAndOdr)) {
        m.trangThai = TrangThaiKhungDuLieuImu::LoiIO; return false;
    }
    const uint8_t odrHoatDong = maOdrHoatDong();
    if (identityAndOdr[0] != _maNhanDang || identityAndOdr[1] != odrHoatDong || identityAndOdr[2] != odrHoatDong) {
        loi(IcmI2cStatus::OTHER_ERROR, who); m.trangThai = TrangThaiKhungDuLieuImu::LoiIO; return false;
    }
    if (!doc(statusReg, &status, 1)) { m.trangThai = TrangThaiKhungDuLieuImu::LoiIO; return false; }
    const uint8_t matNaLoiStatus = _laBienThe16Bx ? 0x78u : 0x48u;
    if (status & matNaLoiStatus) { // Reserved status bits signal floating/stuck bus (e.g. FF).
        loi(IcmI2cStatus::OTHER_ERROR, statusReg); m.trangThai = TrangThaiKhungDuLieuImu::LoiIO; return false;
    }
    // A gyro edge can arrive just before the accel update. Give it a bounded
    // 200 us window in ImuTask, never in the ISR. If still partial, drain the
    // gyro to release its latch, but discard the pair. Polling can simply retry.
    for (unsigned retry = 0; theoNgat && (status & 3u) == 2u && retry < 4; ++retry) {
        delayMicroseconds(50);
        if (!doc(statusReg, &status, 1) || (status & matNaLoiStatus)) {
            if (_loiCuoi == IcmI2cStatus::OK) loi(IcmI2cStatus::OTHER_ERROR, statusReg);
            m.trangThai = TrangThaiKhungDuLieuImu::LoiIO;
            return false;
        }
    }
    if ((status & 3u) != 3u && !(theoNgat && (status & 2u))) {
        m.trangThai = TrangThaiKhungDuLieuImu::DuLieuCu;
        return false;
    }
    if (!doc(outTemp, data, sizeof data)) {
        m.trangThai = TrangThaiKhungDuLieuImu::LoiIO; return false;
    }
    const auto payloadDiag = _giaoDichCuoi;
    if (!doc(statusReg, &after, 1)) {
        m.trangThai = TrangThaiKhungDuLieuImu::LoiIO; return false;
    }
    if (after & matNaLoiStatus) {
        loi(IcmI2cStatus::OTHER_ERROR, statusReg); m.trangThai = TrangThaiKhungDuLieuImu::LoiIO; return false;
    }
    // Successful sample diagnostics report payload bytes and end-to-end read
    // duration (including status checks), not just the final one-byte transfer.
    _giaoDichCuoi = payloadDiag;
    _giaoDichCuoi.thoiGian_us = micros() - start;
    const uint32_t chuKyMau_us = 1000000u / tanSoLayMauThucTe_hz();
    if ((status & 3u) != 3u || (after & 3u) || _giaoDichCuoi.thoiGian_us >= chuKyMau_us) {
        m.trangThai = TrangThaiKhungDuLieuImu::DuLieuCu; return false;
    }
    return giaiMa(data, m); // Timestamp/sequence are assigned only by ImuTask from IRQ.
}

bool Lsm6dsvDriver::kiemTraSauKhoiDong(BaoCaoKiemTraIcm& report) {
    report = {}; const uint32_t start = millis();
    const auto finish = [&](const char* msg, bool ok = false) {
        report.ketQua = msg; report.thoiGian_ms = millis() - start; return ok;
    };
    if (!_daKhoiTao) return finish("chua_khoi_tao");
    if (!cauHinhHopLe()) return finish("sai_id_hoac_cau_hinh_spi");
    uint32_t mauDau_us = 0;
    double tongNorm = 0.0;
    while (millis() - start < lsm6dsv::timeoutBoot_ms) {
        KhungDuLieuImu m{};
        if (docMauMoi(m)) {
            report.mauCuoi = m;
            const float norm = normGiaTocImu(m);
            if (mauImuHuuHan(m) && norm >= 2 && norm <= 30 && m.nhietDoCamBien_C >= -45 && m.nhietDoCamBien_C <= 90) {
                if (report.soMauTot == 0) { mauDau_us = micros(); tongNorm = 0.0; }
                tongNorm += static_cast<double>(norm);
                if (++report.soMauTot >= lsm6dsv::soMauBoot) {
                    // Detect e.g. 240 Hz vs HAODR 200 Hz mismatch.
                    const uint32_t elapsed = micros() - mauDau_us;
                    const uint32_t expected = (report.soMauTot - 1u) * 1000000u / tanSoLayMauThucTe_hz();
                    if (elapsed < expected * 9 / 10 || elapsed > expected * 11 / 10)
                        return finish("sai_nhip_mau");
                    const float normTB = static_cast<float>(tongNorm / report.soMauTot);
                    if (std::isfinite(normTB) && normTB >= 9.0f && normTB <= 10.6f) {
                        _normGiaTocTinh_mps2 = normTB;
                    }
                    return finish("PASS", true);
                }
            } else { ++report.soMauLoi; report.soMauTot = 0; tongNorm = 0.0; }
        } else if (m.trangThai == TrangThaiKhungDuLieuImu::LoiIO) return finish("loi_spi");
        delayMicroseconds(200);
    }
    return finish(report.soMauLoi ? "du_lieu_bat_thuong" : "khong_du_mau_moi");
}

void Lsm6dsvDriver::datDoLechConQuay_dps(float x, float y, float z) {
    _daHieuChinh = std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    _bias[0] = _daHieuChinh ? x : 0; _bias[1] = _daHieuChinh ? y : 0; _bias[2] = _daHieuChinh ? z : 0;
}
void Lsm6dsvDriver::layDoLechConQuay_dps(float& x, float& y, float& z, bool& ok) const {
    x = _bias[0]; y = _bias[1]; z = _bias[2]; ok = _daHieuChinh;
}
bool Lsm6dsvDriver::hieuChinhConQuay(uint16_t required) {
    if (!_daKhoiTao || required < 2) return false;
    _daHieuChinh = false; for (float& b : _bias) b = 0;
    ThongKeMauImu statistics;
    const uint32_t start = millis(); uint32_t last = start;
    while (millis() - start < thoiGianChoOnDinhCalib_ms) {
        KhungDuLieuImu m{};
        if (docMauMoi(m)) {
            if (millis() - last > khoangMauCalibToiDa_ms) statistics.xoa();
            last = millis();
            if (!mauImuGanTinh(m, gioiHanGyroThoCalib_dps * .01745329252f)) statistics.xoa();
            else {
                statistics.themMau(m);
                if (statistics.soMau() >= required) {
                    if (statistics.onDinh()) {
                        const float ax = statistics.trungBinh(0);
                        const float ay = statistics.trungBinh(1);
                        const float az = statistics.trungBinh(2);
                        const float gNorm = std::sqrt(ax * ax + ay * ay + az * az);
                        if (std::isfinite(gNorm) && gNorm >= 9.0f && gNorm <= 10.6f) {
                            _normGiaTocTinh_mps2 = gNorm;
                        }
                        datDoLechConQuay_dps(statistics.trungBinh(3)/.01745329252f,
                            statistics.trungBinh(4)/.01745329252f, statistics.trungBinh(5)/.01745329252f);
                        return true;
                    }
                    statistics.xoa();
                }
            }
        } else if (m.trangThai == TrangThaiKhungDuLieuImu::LoiIO) return false;
        if (millis() - last >= thoiGianMatMauCalib_ms) return false;
        delayMicroseconds(200);
    }
    return false;
}

#endif
