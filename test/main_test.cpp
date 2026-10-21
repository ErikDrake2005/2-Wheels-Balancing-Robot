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
#include <cstring>
mpu6050Drv camBienMpu(Wire);
MksServo42CPair capDongCo;
uint8_t soThietBiI2c = 0;
uint8_t diaChiI2cPhatHien[8]{};
int whoAmITheoDiaChiCauHinh = -1;

uint64_t layThoiGian_us() {
    return static_cast<uint64_t>(esp_timer_get_time());
}

// Chỉ chạy ở setup, trước khi task IMU được tạo: không tranh I2C với task.
bool docThanhGhi(uint8_t reg, uint8_t& value) {
    Wire.beginTransmission(static_cast<uint8_t>(diaChiI2cMpu));
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(static_cast<uint8_t>(diaChiI2cMpu),
                         static_cast<size_t>(1), true) != 1) return false;
    const int byte = Wire.read();
    if (byte < 0) return false;
    value = static_cast<uint8_t>(byte);
    return true;
}

// Máy quét tạm thời cho bench: xác định thiết bị nào thực sự trả lời trên
// cùng hai chân SDA/SCL mà mpu6050Drv sẽ sử dụng. Hàm này không điều khiển motor.
void quetDiaChiI2c() {
    Serial.printf("I2C_SCAN: SDA=%d SCL=%d baud=%lu\n",
        chanSdaMpu, chanSclMpu, static_cast<unsigned long>(tanSoI2cMpu_hz));
    Wire.begin(chanSdaMpu, chanSclMpu);
    Wire.setClock(tanSoI2cMpu_hz);

    soThietBiI2c = 0;
    whoAmITheoDiaChiCauHinh = -1;
    for (uint8_t diaChi = 1; diaChi < 127; ++diaChi) {
        Wire.beginTransmission(diaChi);
        const uint8_t maLoi = Wire.endTransmission();
        if (maLoi == 0) {
            Serial.printf("I2C_SCAN: tim thay dia chi 0x%02X\n", diaChi);
            if (soThietBiI2c < sizeof(diaChiI2cPhatHien)) {
                diaChiI2cPhatHien[soThietBiI2c] = diaChi;
            }
            ++soThietBiI2c;
        }
    }
    Wire.beginTransmission(static_cast<uint8_t>(diaChiI2cMpu));
    Wire.write(0x75);
    if (Wire.endTransmission(false) == 0 &&
        Wire.requestFrom(static_cast<uint8_t>(diaChiI2cMpu), static_cast<size_t>(1), true) == 1) {
        whoAmITheoDiaChiCauHinh = Wire.read();
    }
    if (soThietBiI2c == 0) {
        Serial.println("I2C_SCAN: khong tim thay thiet bi");
    } else {
        Serial.printf("I2C_SCAN: tong cong %u thiet bi\n", soThietBiI2c);
    }
}
void inTrangThaiI2c() {
    Serial.printf(
        "I2C_STATE: soThietBi=%u cauHinh=0x%02X WHO_AM_I=",
        static_cast<unsigned>(soThietBiI2c),
        static_cast<unsigned>(diaChiI2cMpu)
    );

    if (whoAmITheoDiaChiCauHinh < 0) {
        Serial.print("READ_ERROR");
    } else {
        Serial.printf(
            "0x%02X",
            static_cast<unsigned>(whoAmITheoDiaChiCauHinh)
        );
    }

    Serial.print(" diaChi=");

    const size_t soChoLuu =
        sizeof(diaChiI2cPhatHien) / sizeof(diaChiI2cPhatHien[0]);

    for (size_t i = 0; i < soThietBiI2c && i < soChoLuu; ++i) {
        if (i > 0) Serial.print(",");

        Serial.printf(
            "0x%02X",
            static_cast<unsigned>(diaChiI2cPhatHien[i])
        );
    }

    if (soThietBiI2c == 0) Serial.print("khong_co");

    Serial.println();
}

bool soSanhThanhGhi(const char* name, uint8_t reg, uint8_t expected, uint8_t mask) {
    uint8_t value = 0;
    const bool docThanhCong = docThanhGhi(reg, value);
    const bool match = docThanhCong && ((value & mask) == (expected & mask));
    Serial.printf("MPU %-13s reg=0x%02X daNhanLenh=0x%02X expected=0x%02X mask=0x%02X %s\n",
        name, static_cast<unsigned>(reg), static_cast<unsigned>(value),
        static_cast<unsigned>(expected), static_cast<unsigned>(mask),
        !docThanhCong ? "READ_ERROR" : (match ? "MATCH" : "MISMATCH"));
    return match;
}

bool kiemTraCauHinhMpu() {
    // Bộ driver hiện tại chỉ hỗ trợ +/-4g và +/-500 dps.
    // WHO_AM_I vẫn 0x68 ngay cả khi AD0 làm địa chỉ bus thành 0x69.
    bool passed = true;
    passed = soSanhThanhGhi("WHO_AM_I", 0x75, 0x68, 0x7E) && passed;
    passed = soSanhThanhGhi("PWR_MGMT_1", 0x6B, 0x01, 0x7F) && passed;
    passed = soSanhThanhGhi("SMPLRT_DIV", 0x19,
        static_cast<uint8_t>(1000u / tanSoLayMauMpu_hz - 1u), 0xFF) && passed;
    passed = soSanhThanhGhi("DLPF_CFG", 0x1A, cauHinhLocThongThapMpu, 0x07) && passed;
    passed = soSanhThanhGhi("GYRO_CONFIG", 0x1B, 0x08, 0x18) && passed;
    passed = soSanhThanhGhi("ACCEL_CONFIG", 0x1C, 0x08, 0x18) && passed;
    passed = soSanhThanhGhi("INT_PIN_CFG", 0x37, mucNgatMpuCao ? 0x00 : 0x80, 0xF0) && passed;
    passed = soSanhThanhGhi("INT_ENABLE", 0x38, batNgatMpu ? 0x01 : 0x00, 0x01) && passed;
    return passed;
}

// Chỉ tính từ số liệu bạn nhập tay, không đọc được menu/encoder MKS qua STEP/DIR.
void inQuyDoiMotor(const char* name, uint16_t steps, uint16_t microstep, float ratio) {
    if (steps == 0 || microstep == 0 || !std::isfinite(ratio) || ratio <= 0.0f) {
        Serial.printf("%s: steps/rev, MStep hoac gear ratio chua nhap; khong suy ra RPM.\n", name);
        return;
    }
    const double inputStepsPerWheelRev = static_cast<double>(steps) * microstep * ratio;
    const double hz = tanSoXungLienTucThu_hz;
    const double wheelRpm = hz * 60.0 / inputStepsPerWheelRev;
    const double velocity = hz / inputStepsPerWheelRev * 6.28318530718 * banKinhBanhThu_m;
    Serial.printf("%s: %.0f input steps/wheel-rev; %.0f steps/s -> %.2f wheel RPM, %.4f m/s\n",
        name, inputStepsPerWheelRev, hz, wheelRpm, velocity);
}

void inCauHinhMotor() {
    Serial.printf("MKS PIN L(step/dir)=%d/%d R=%d/%d EN=%d\n",
        chanStepTrai, chanDirTrai, chanStepPhai, chanDirPhai, chanBatMotor);
    Serial.printf("MKS EN active=%s, forward DIR L=%s R=%s, confirmed=%d, benchPermission=%d\n",
        mucBatMotorCao ? "HIGH" : "LOW", mucDirTienBanhTraiCao ? "HIGH" : "LOW",
        mucDirTienBanhPhaiCao ? "HIGH" : "LOW", daXacNhanCauHinhPhanCungMotor,
        choPhepChayMotorThu);
    Serial.printf("MKS HIGH=%luus minLOW=%luus hold=%luus setup=%luus enableWait=%luus\n",
        static_cast<unsigned long>(thoiGianMucCaoStep_us), static_cast<unsigned long>(thoiGianMucThapToiThieuStep_us),
        static_cast<unsigned long>(thoiGianGiuDir_us), static_cast<unsigned long>(thoiGianOnDinhDir_us),
        static_cast<unsigned long>(thoiGianOnDinhEnable_us));
    const uint64_t high = (static_cast<uint64_t>(thoiGianMucCaoStep_us) * doPhanGiaiBoDemMcpwm_hz + 999999ULL) / 1000000ULL;
    const uint64_t low = (static_cast<uint64_t>(thoiGianMucThapToiThieuStep_us) * doPhanGiaiBoDemMcpwm_hz + 999999ULL) / 1000000ULL;
    const double maxSignalHz = high + low ? static_cast<double>(doPhanGiaiBoDemMcpwm_hz) / (high + low) : 0.0;
    Serial.printf("MCPWM resolution=%luHz, min continuous backend~%.2fHz, max by waveform~%.2fHz\n",
        static_cast<unsigned long>(doPhanGiaiBoDemMcpwm_hz),
        static_cast<double>(doPhanGiaiBoDemMcpwm_hz) / 65535.0, maxSignalHz);
    Serial.printf("PAIR threshold=%.0f steps/s max=%.0f steps/s service=%luus deadline=%luus\n",
        tanSoChayLienTucToiThieu_hz, tanSoStepToiDa_hz,
        static_cast<unsigned long>(chuKyPhucVuMotor_us), static_cast<unsigned long>(thoiGianHetHanLenhMotor_us));
    inQuyDoiMotor("LEFT", soBuocDayDuBanhTraiMoiVong, soViBuocBanhTrai,
        soVongDongCoBanhTraiTrenVongBanh);
    inQuyDoiMotor("RIGHT", soBuocDayDuBanhPhaiMoiVong, soViBuocBanhPhai,
        soVongDongCoBanhPhaiTrenVongBanh);
    Serial.println("MKS config tren chi la ky vong ESP32; doi chieu menu va logic analyzer rieng.");
}

const char* tenKetQua(MksResult value) {
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

const char* tenLoi(MksFault value) {
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
struct ThongKeMpu {
    KhungDuLieuMPU khungDuLieu{};
    uint32_t soMauTot = 0, soLoi = 0, soMauBoLo = 0;
    uint32_t khoangCachMauNhoNhat_us = UINT32_MAX, khoangCachMauLonNhat_us = 0;
    uint32_t thoiGianDocLonNhat_us = 0, thoiGianDanhThucLonNhat_us = 0;
    uint64_t thoiDiemMauTotCuoi_us = 0;
    bool loi = false;
};
struct ThongKeMotor {
    MksSnapshot trangThai{};
    MksResult ketQuaCuoi = MksResult::Disabled;
    uint32_t khoangCachPhucVuLonNhat_us = 0, thoiGianGoiLonNhat_us = 0;
    bool biChanDoMpu = false;
};
struct LenhThu { char code = 'd'; uint64_t validUntilUs = 0; };
QueueHandle_t hangDoiMpu = nullptr, hangDoiMotor = nullptr, hangDoiLenh = nullptr;
TaskHandle_t tayCamTaskMpu = nullptr, tayCamTaskMotor = nullptr;
esp_timer_handle_t boHenPhucVuMotor = nullptr;
portMUX_TYPE khoaMau = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t thuTuMau = 0;
volatile uint64_t thoiDiemMau_us = 0;
bool daKhoiDong = false;
// Giữ lại bước khởi động cuối cùng để Serial Monitor mở trễ vẫn biết lỗi nằm ở đâu.
const char* giaiDoanKhoiDong = "chua_bat_dau";
const char* loiKhoiDong = "khong_co";

void ghiNhanGiaiDoanKhoiDong(const char* tenGiaiDoan) {
    giaiDoanKhoiDong = tenGiaiDoan;
    Serial.printf("STARTUP: %s\n", giaiDoanKhoiDong);
}

void ghiNhanLoiKhoiDong(const char* tenLoi) {
    loiKhoiDong = tenLoi;
    Serial.printf("FAIL: %s\n", loiKhoiDong);
}

void ARDUINO_ISR_ATTR onImuDataReady() {
    const uint64_t now = layThoiGian_us();
    portENTER_CRITICAL_ISR(&khoaMau);
    thuTuMau = thuTuMau + 1u;
    thoiDiemMau_us = now;
    portEXIT_CRITICAL_ISR(&khoaMau);
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(tayCamTaskMpu, &wake);
    if (wake == pdTRUE) portYIELD_FROM_ISR();
}

void taskMpu(void*) {
    ThongKeMpu thongKe{};
    uint32_t thuTuMauTruoc = 0;
    uint64_t thoiDiemMauTruoc_us = 0;
    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(
                (tuoiDuLieuMpuToiDa_us + 999u) / 1000u)) == 0) {
            thongKe.loi = true;
            thongKe.khungDuLieu = KhungDuLieuMPU{};
            thongKe.khungDuLieu.trangThai = TrangThaiKhungDuLieuMpu::DuLieuCu;
            xQueueOverwrite(hangDoiMpu, &thongKe);
            vTaskSuspend(nullptr);
            continue;
        }
        uint32_t thuTuMauHienTai;
        uint64_t thoiDiemMauHienTai_us;
        portENTER_CRITICAL(&khoaMau);
        thuTuMauHienTai = thuTuMau;
        thoiDiemMauHienTai_us = thoiDiemMau_us;
        portEXIT_CRITICAL(&khoaMau);
        const uint32_t khoangCachMau = thuTuMauHienTai - thuTuMauTruoc;
        if (!khoangCachMau) continue;
        if (khoangCachMau > 1) thongKe.soMauBoLo += khoangCachMau - 1;
        thuTuMauTruoc = thuTuMauHienTai;
        if (thoiDiemMauTruoc_us) {
            const uint32_t dt = static_cast<uint32_t>(thoiDiemMauHienTai_us - thoiDiemMauTruoc_us);
            if (dt < thongKe.khoangCachMauNhoNhat_us) thongKe.khoangCachMauNhoNhat_us = dt;
            if (dt > thongKe.khoangCachMauLonNhat_us) thongKe.khoangCachMauLonNhat_us = dt;
        }
        thoiDiemMauTruoc_us = thoiDiemMauHienTai_us;
        const uint64_t batDauDoc_us = layThoiGian_us();
        const uint32_t thoiGianDanhThuc_us = static_cast<uint32_t>(batDauDoc_us - thoiDiemMauHienTai_us);
        if (thoiGianDanhThuc_us > thongKe.thoiGianDanhThucLonNhat_us) thongKe.thoiGianDanhThucLonNhat_us = thoiGianDanhThuc_us;
        KhungDuLieuMPU khungDuLieu{};
        const bool docThanhCong = camBienMpu.read(khungDuLieu);
        const uint64_t ketThucDoc_us = layThoiGian_us();
        const uint32_t thoiGianDoc_us = static_cast<uint32_t>(ketThucDoc_us - batDauDoc_us);
        if (thoiGianDoc_us > thongKe.thoiGianDocLonNhat_us) thongKe.thoiGianDocLonNhat_us = thoiGianDoc_us;
        uint32_t thuTuSauDoc;
        portENTER_CRITICAL(&khoaMau);
        thuTuSauDoc = thuTuMau;
        portEXIT_CRITICAL(&khoaMau);
        // Nghiêm ngặt: nếu data-ready khác tới trong lúc đọc, bỏ mẫu
        // vì không chắc thoiDiemMau_us trước đọc gắn đúng dữ liệu vừa nhận.
        if (!docThanhCong || thuTuSauDoc != thuTuMauHienTai || thongKe.soMauBoLo) {
            thongKe.loi = true;
            ++thongKe.soLoi;
            khungDuLieu.trangThai = docThanhCong ? TrangThaiKhungDuLieuMpu::DuLieuCu : TrangThaiKhungDuLieuMpu::LoiIO;
        } else {
            ++thongKe.soMauTot;
            khungDuLieu.thoiDiemLayMau_us = thoiDiemMauHienTai_us;
            khungDuLieu.soMauLayDuoc = thongKe.soMauTot;
            thongKe.thoiDiemMauTotCuoi_us = ketThucDoc_us;
        }
        thongKe.khungDuLieu = khungDuLieu;
        xQueueOverwrite(hangDoiMpu, &thongKe);
        if (thongKe.loi) vTaskSuspend(nullptr);
    }
}

// Callback esp_timer chạy từ task timer. Nó chỉ đánh thức MotorTask.
// Chu kỳ 250 us là mục tiêu; code đo khoangCachMau thực, không giả định đúng tuyệt đối.
void goiTaskMotor(void*) { xTaskNotifyGive(tayCamTaskMotor); }

void taskMotor(void*) {
    ThongKeMotor thongKe{};
    float tocDoMucTieuTrai = 0.0f, tocDoMucTieuPhai = 0.0f;
    uint64_t thoiDiemDung_us = 0, lanApDungCuoi_us = 0, lanPhucVuCuoi_us = 0;
    bool dangChay = false, lamMoiLenh = true;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        uint64_t now = layThoiGian_us();
        ThongKeMpu camBienMpuStats{};
        const bool camBienMpuHealthy = xQueuePeek(hangDoiMpu, &camBienMpuStats, 0) == pdTRUE &&
            !camBienMpuStats.loi && camBienMpuStats.khungDuLieu.daHieuChinhConQuay && camBienMpuStats.thoiDiemMauTotCuoi_us &&
            now - camBienMpuStats.thoiDiemMauTotCuoi_us < tuoiDuLieuMpuToiDa_us;

        LenhThu lenh{};
        const bool daNhanLenh = xQueueReceive(hangDoiLenh, &lenh, 0) == pdTRUE;
        if (daNhanLenh) {
            if (lenh.code == 'd') { capDongCo.disable(); dangChay = false; }
            else if (lenh.code == 'x') { capDongCo.emergencyStop(); dangChay = false; }
            else if (lenh.code == 'k') {
                capDongCo.disable(); dangChay = false;
                thongKe.ketQuaCuoi = capDongCo.clearFault() ? MksResult::Ok : MksResult::FaultLatched;
            } else if (lenh.code == 'f') {
                lamMoiLenh = false; // service vẫn chạy, chờ CommandExpired.
            } else if (lenh.code == 's' && dangChay) {
                tocDoMucTieuTrai = tocDoMucTieuPhai = 0.0f; // Giữ EN, gửi lenh 0 có heartbeat.
                lanApDungCuoi_us = 0;
            } else if (choPhepChayMotorThu && camBienMpuHealthy && now < lenh.validUntilUs) {
                const char c = lenh.code;
                const bool laLenhChuyenDong = c == 'l' || c == 'L' || c == 'r' || c == 'R' ||
                    c == 'b' || c == 'B' || c == 'c' || c == 'C';
                if (laLenhChuyenDong) {
                    // Nếu đang chạy, KHÔNG disable để thử trình tự đổi dấu V2.
                    thongKe.ketQuaCuoi = capDongCo.enable();
                    if (thongKe.ketQuaCuoi == MksResult::Ok) {
                        const bool tanSoCao = c == 'c' || c == 'C';
                        const float doLonTocDo = tanSoCao ? tanSoXungLienTucThu_hz : tanSoXungThuaThu_hz;
                        const bool daoChieu = c == 'L' || c == 'R' || c == 'B' || c == 'C';
                        const float tocDoCoDau = daoChieu ? -doLonTocDo : doLonTocDo;
                        tocDoMucTieuTrai = c == 'r' || c == 'R' ? 0.0f : tocDoCoDau;
                        tocDoMucTieuPhai = c == 'l' || c == 'L' ? 0.0f : tocDoCoDau;
                        thoiDiemDung_us = lenh.validUntilUs;
                        dangChay = true;
                        lamMoiLenh = true;
                        lanApDungCuoi_us = 0;
                        thongKe.biChanDoMpu = false;
                    }
                }
            } else if (!choPhepChayMotorThu) {
                // Các lệnh chuyển động không vượt qua cổng xác nhận phần cứng.
                thongKe.ketQuaCuoi = MksResult::HardwareConfigUnconfirmed;
            }
        }

        now = layThoiGian_us();
        if (dangChay && (!camBienMpuHealthy || now >= thoiDiemDung_us)) {
            if (!camBienMpuHealthy) { capDongCo.emergencyStop(); thongKe.biChanDoMpu = true; }
            else capDongCo.disable();
            dangChay = false;
        }
        if (dangChay && lamMoiLenh &&
            (lanApDungCuoi_us == 0 || now - lanApDungCuoi_us >= chuKyLamMoiLenhThu_us)) {
            WheelStepCommand c{};
            c.tocDoBuocTrai_buocTrenGiay = tocDoMucTieuTrai;
            c.tocDoBuocPhai_buocTrenGiay = tocDoMucTieuPhai;
            c.thoiDiemRaLenh_us = layThoiGian_us();
            c.thoiDiemHetHan_us = c.thoiDiemRaLenh_us + thoiGianHetHanLenhMotor_us;
            thongKe.ketQuaCuoi = capDongCo.apply(c);
            lanApDungCuoi_us = c.thoiDiemRaLenh_us;
            if (thongKe.ketQuaCuoi == MksResult::FaultLatched) dangChay = false;
        }
        const uint64_t truocPhucVu_us = layThoiGian_us();
        if (lanPhucVuCuoi_us && capDongCo.isEnabled()) {
            const uint32_t khoangCachMau = static_cast<uint32_t>(truocPhucVu_us - lanPhucVuCuoi_us);
            if (khoangCachMau > thongKe.khoangCachPhucVuLonNhat_us) thongKe.khoangCachPhucVuLonNhat_us = khoangCachMau;
        }
        lanPhucVuCuoi_us = truocPhucVu_us;
        const MksResult tienDo = capDongCo.service();
        const uint32_t thoiGianGoi_us = static_cast<uint32_t>(layThoiGian_us() - truocPhucVu_us);
        if (thoiGianGoi_us > thongKe.thoiGianGoiLonNhat_us) thongKe.thoiGianGoiLonNhat_us = thoiGianGoi_us;
        if (tienDo == MksResult::FaultLatched) { dangChay = false; thongKe.ketQuaCuoi = tienDo; }
        thongKe.trangThai = capDongCo.snapshot();
        xQueueOverwrite(hangDoiMotor, &thongKe);
    }
}

void inHuongDan() {
    Serial.println("l/L=trai +/- sparse; r/R=phai +/- sparse; b/B=hai banh +/- sparse");
    Serial.println("c/C=hai banh +/- continuous; s=lenh 0 giu EN; d=disable");
    Serial.println("f=ngung refresh de thu timeout; x=emergency; k=disable+clear loi; h=help");
    Serial.println("Dau +/- theo Config forward DIR, KHONG mac dinh la CW/CCW.");
}

void setup() {
    // USB CDC của ESP32-S3 có thể nối lại sau reset; khởi tạo Serial trước để
    // các bước bên dưới có thể được ghi lại. EN vẫn được đặt về trạng thái tắt
    // trước khi khởi tạo driver motor.
    Serial.begin(tocDoSerialThu_baud);
    delay(thoiGianKhoiDongConsole_ms);
    ghiNhanGiaiDoanKhoiDong("bat_dau");

    // Đặt EN disable trước khi khởi tạo driver; không cho motor tự chạy sau reset.
    const auto enPin = static_cast<gpio_num_t>(chanBatMotor);
    ghiNhanGiaiDoanKhoiDong("tat_en_motor");
    if (gpio_set_direction(enPin, GPIO_MODE_OUTPUT) != ESP_OK ||
        gpio_set_level(enPin, mucBatMotorCao ? 0 : 1) != ESP_OK) {
        ghiNhanLoiKhoiDong("gpio_en_that_bai");
        return;
    }
    inCauHinhMotor();

    ghiNhanGiaiDoanKhoiDong("kiem_tra_config");
    if (!batNgatMpu || chuKyLamMoiLenhThu_us >= thoiGianHetHanLenhMotor_us ||
        tanSoXungThuaThu_hz <= 0 || tanSoXungThuaThu_hz >= tanSoChayLienTucToiThieu_hz ||
        tanSoXungLienTucThu_hz < tanSoChayLienTucToiThieu_hz ||
        tanSoXungLienTucThu_hz > tanSoStepToiDa_hz) {
        ghiNhanLoiKhoiDong("config_main_khong_phu_hop");
        return;
    }
    ghiNhanGiaiDoanKhoiDong("khoi_tao_mks");
    if (!capDongCo.begin()) {
        ghiNhanLoiKhoiDong("mks_begin_that_bai");
        return;
    }
    ghiNhanGiaiDoanKhoiDong("khoi_tao_mpu");
    quetDiaChiI2c();
    if (!camBienMpu.begin()) {
        ghiNhanLoiKhoiDong("mpu_begin_that_bai");
        capDongCo.disable();
        return;
    }
    ghiNhanGiaiDoanKhoiDong("kiem_tra_thanh_ghi_mpu");
    if (!kiemTraCauHinhMpu()) {
        ghiNhanLoiKhoiDong("mpu_readback_that_bai");
        capDongCo.disable();
        return;
    }
    Serial.println("Hieu chuan: giu yen, motor disable.");
    ghiNhanGiaiDoanKhoiDong("hieu_chinh_con_quay");
    if (!camBienMpu.hieuChinhConQuay(soMauHieuChinhMpu)) {
        ghiNhanLoiKhoiDong("hieu_chinh_con_quay_that_bai");
        return;
    }

    ghiNhanGiaiDoanKhoiDong("tao_queue");
    hangDoiMpu = xQueueCreate(1, sizeof(ThongKeMpu));
    hangDoiMotor = xQueueCreate(1, sizeof(ThongKeMotor));
    hangDoiLenh = xQueueCreate(1, sizeof(LenhThu));
    if (!hangDoiMpu || !hangDoiMotor || !hangDoiLenh) {
        ghiNhanLoiKhoiDong("tao_queue_that_bai");
        return;
    }
    ThongKeMpu initial{};
    xQueueOverwrite(hangDoiMpu, &initial);
    ghiNhanGiaiDoanKhoiDong("tao_task_mpu");
    if (xTaskCreate(taskMpu, "ImuCompare", kichThuocNganXepTaskMpu_byte, nullptr,
        uuTienTaskMpu, &tayCamTaskMpu) != pdPASS) {
        ghiNhanLoiKhoiDong("tao_task_mpu_that_bai");
        return;
    }
    pinMode(chanNgatMpu, INPUT);
    attachInterrupt(chanNgatMpu, onImuDataReady, mucNgatMpuCao ? RISING : FALLING);
    ghiNhanGiaiDoanKhoiDong("tao_task_motor");
    if (xTaskCreate(taskMotor, "MotorCompare", kichThuocNganXepTaskMotor_byte, nullptr,
        uuTienTaskMotor, &tayCamTaskMotor) != pdPASS) {
        detachInterrupt(chanNgatMpu);
        vTaskDelete(tayCamTaskMpu);
        ghiNhanLoiKhoiDong("tao_task_motor_that_bai");
        return;
    }
    ghiNhanGiaiDoanKhoiDong("tao_timer_motor");
    esp_timer_create_args_t timerArgs{};
    timerArgs.callback = goiTaskMotor;
    timerArgs.dispatch_method = ESP_TIMER_TASK;
    timerArgs.name = "MotorWake";
    timerArgs.skip_unhandled_events = true;
    if (esp_timer_create(&timerArgs, &boHenPhucVuMotor) != ESP_OK ||
        esp_timer_start_periodic(boHenPhucVuMotor, chuKyPhucVuMotor_us) != ESP_OK) {
        vTaskDelete(tayCamTaskMotor);
        detachInterrupt(chanNgatMpu); vTaskDelete(tayCamTaskMpu);
        ghiNhanLoiKhoiDong("tao_timer_motor_that_bai");
        return;
    }
    daKhoiDong = true;
    ghiNhanGiaiDoanKhoiDong("hoan_tat");
    Serial.println("READY: doi chieu REGISTER la tu dong; menu MKS phai nhap/kiem tra thu cong.");
    inHuongDan();
}

void loop() {
    static uint32_t thoiDiemBaoCuoi_ms = 0;
    const uint32_t thoiDiemHienTai_ms = millis();

    if (thoiDiemHienTai_ms - thoiDiemBaoCuoi_ms >= chuKyBaoCao_ms) {
        thoiDiemBaoCuoi_ms = thoiDiemHienTai_ms;

        Serial.printf(
            "KIEM_TRA_LOOP: daKhoiDong=%d giaiDoan=%s loi=%s\n",
            static_cast<int>(daKhoiDong), giaiDoanKhoiDong, loiKhoiDong
        );
        // In kết quả quét đã lưu, kể cả khi setup() khởi tạo thất bại.
        // Chỉ đọc khi begin() của MPU đã thất bại.
// Ở giai đoạn này task đọc MPU chưa được tạo.
        if (!daKhoiDong &&
            std::strcmp(loiKhoiDong, "mpu_begin_that_bai") == 0) {

            static uint32_t soLanDoc = 0;
            uint8_t maNhanDang = 0;

            const bool docThanhCong = docThanhGhi(0x75, maNhanDang);
            ++soLanDoc;

            Serial.printf(
                "MPU_DOC_MOI: lan=%lu docThanhCong=%d ID=0x%02X\n",
                static_cast<unsigned long>(soLanDoc),
                static_cast<int>(docThanhCong),
                static_cast<unsigned>(maNhanDang)
            );
        }
        inTrangThaiI2c();
    }

    if (!daKhoiDong) {
        delay(100);
        return;
    }
    for (uint8_t count = 0; count < 16 && Serial.available(); ++count) {
        const char c = static_cast<char>(Serial.read());
        if (c == 'h') { inHuongDan(); continue; }
        if (c == 'l' || c == 'L' || c == 'r' || c == 'R' || c == 'b' || c == 'B' ||
            c == 'c' || c == 'C' || c == 's' || c == 'd' || c == 'f' || c == 'x' || c == 'k') {
            LenhThu lenh{};
            lenh.code = c;
            lenh.validUntilUs = layThoiGian_us() + thoiGianChayMotorThu_us;
            xQueueOverwrite(hangDoiLenh, &lenh);
        }
    }
    static uint64_t lanBaoCaoCuoi_us = layThoiGian_us();
    static uint32_t soMauTotTruoc = 0;
    const uint64_t now = layThoiGian_us();
    if (now - lanBaoCaoCuoi_us >= chuKyBaoCao_ms * 1000ULL) {
        ThongKeMpu a{};
        ThongKeMotor b{};
        if (xQueuePeek(hangDoiMpu, &a, 0) == pdTRUE) {
            const float hz = (a.soMauTot - soMauTotTruoc) * 1000000.0f / (now - lanBaoCaoCuoi_us);
            const float doLonGiaToc = std::sqrt(a.khungDuLieu.giaTocX_mps2 * a.khungDuLieu.giaTocX_mps2 +
                a.khungDuLieu.giaTocY_mps2 * a.khungDuLieu.giaTocY_mps2 + a.khungDuLieu.giaTocZ_mps2 * a.khungDuLieu.giaTocZ_mps2);
            Serial.printf("IMU %.1fHz soMauTot=%lu err=%lu miss=%lu dt=%lu..%luus readMax=%lu wakeMax=%lu loi=%d\n",
                hz, static_cast<unsigned long>(a.soMauTot), static_cast<unsigned long>(a.soLoi),
                static_cast<unsigned long>(a.soMauBoLo),
                static_cast<unsigned long>(a.khoangCachMauNhoNhat_us == UINT32_MAX ? 0 : a.khoangCachMauNhoNhat_us),
                static_cast<unsigned long>(a.khoangCachMauLonNhat_us), static_cast<unsigned long>(a.thoiGianDocLonNhat_us),
                static_cast<unsigned long>(a.thoiGianDanhThucLonNhat_us), a.loi);
            Serial.printf("IMU |A|=%.2fm/s2 gyro=%.4f,%.4f,%.4frad/s calib=%d\n",
                doLonGiaToc, a.khungDuLieu.tocDoGocX_radps, a.khungDuLieu.tocDoGocY_radps, a.khungDuLieu.tocDoGocZ_radps, a.khungDuLieu.daHieuChinhConQuay);
            soMauTotTruoc = a.soMauTot;
        }
        if (xQueuePeek(hangDoiMotor, &b, 0) == pdTRUE) {
            Serial.printf("PAIR EN=%d target=%.2f,%.2f backendHz=%.1f,%.1f dangChoXuLy=%d sat=%d "
                          "loi=%s last=%s khoangCachMauMax=%luus callMax=%luus camBienMpuBlock=%d\n",
                b.trangThai.daBat, b.trangThai.tocDoBuocYeuCauTrai_buocTrenGiay, b.trangThai.tocDoBuocYeuCauPhai_buocTrenGiay,
                b.trangThai.tanSoLienTucTrai_hz, b.trangThai.tanSoLienTucPhai_hz, b.trangThai.dangChoXuLy, b.trangThai.biGioiHan,
                tenLoi(b.trangThai.loi), tenKetQua(b.ketQuaCuoi),
                static_cast<unsigned long>(b.khoangCachPhucVuLonNhat_us), static_cast<unsigned long>(b.thoiGianGoiLonNhat_us), b.biChanDoMpu);
        }
        lanBaoCaoCuoi_us = now;
    }
    delay(1); // Chỉ console task; không dùng dòng này để định thời service 250 us.
}
