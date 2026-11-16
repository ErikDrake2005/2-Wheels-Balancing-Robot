#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H
#include <cstdint>

// Lựa chọn phần cứng truyền động bánh xe:
// - LoaiPhanCungDongCo::MKS    : MKS Servo42C V1.1 / Servo42D chạy CR_vFOC (STEP/DIR/EN + đọc từ kế 14-bit qua UART 3 chân).
// - LoaiPhanCungDongCo::TMC_MT : TMC2209 (STEP/DIR/ENN) + Encoder từ MT6701 AB (hồi tiếp vòng kín trên ESP32).
enum class LoaiPhanCungDongCo : uint8_t {
    MKS = 0,
    TMC_MT = 1
};

#ifndef CAU_HINH_PHAN_CUNG_DONG_CO
#define CAU_HINH_PHAN_CUNG_DONG_CO LoaiPhanCungDongCo::MKS
#endif

constexpr LoaiPhanCungDongCo loaiPhanCungDongCo = CAU_HINH_PHAN_CUNG_DONG_CO;
constexpr bool dungPhanCungTmcMt = (loaiPhanCungDongCo == LoaiPhanCungDongCo::TMC_MT);

// Động cơ 1,8 độ (17HS4401S), MKS dùng 64 vi bước (12800 xung STEP/vòng khi tỉ số 1:1).
// - Với MKS Servo42C V1.1: đặt cả hai driver Mode = CR_vFOC, MStep = 64, En = L, Baud = 115200.
//   MStep trên driver phải khớp firmware; MPlyer nội suy nội bộ là một thiết lập riêng.
// - Với TMC2209: đặt MS1/MS2 (hoặc UART) = 32 vi bước, chân ENN active LOW (mucBatMotorCao = false).
constexpr uint16_t soBuocDayDuMoiVongDongCo = 200;
constexpr uint16_t soViBuocDauVaoMks = 64;
constexpr uint16_t soViBuocTmc2209 = 32;
constexpr uint16_t soViBuocDauVaoDriver = dungPhanCungTmcMt ? soViBuocTmc2209 : soViBuocDauVaoMks;

// Cấu hình đọc hồi tiếp cảm biến từ 14-bit tích hợp sẵn trên MKS Servo42C V1.1 qua 3 chân UART
// (IO1 = TX chung, IO3 = RX động cơ MKS 1, IO4 = RX động cơ MKS 2):
// - Theo tài liệu MKS Servo42C V1.1, lệnh 0x30 trả về carry (int32_t) + value (0..0x3FFF = 16384 xung/vòng).
// - Nhờ tách riêng RX1 (UART1 trên IO3) và RX2 (UART2 trên IO4), cả hai module MKS để chung địa chỉ 0xE0
//   nhận lệnh đồng thời qua IO1 và phản hồi song song ở tốc độ 115200 bps (< 1 ms trên đường dây)!
constexpr bool batUartEncoderMks = !dungPhanCungTmcMt;
constexpr uint32_t soXungEncoderMksMoiVong = 65536;  // Lệnh 0x30 trên MKS Servo42C V1.1 trả carry (int32) + value (0..65535 = 1 vòng).
constexpr uint32_t tocDoBaudUartMks_bps = 115200;     // Tốc độ UART tối đa 115200 bps trên MKS Servo42C V1.1.
constexpr uint8_t diaChiUartMksTrai = 0xE1;           // Địa chỉ UART ưu tiên khi quét động cơ trái (tự khóa sang 0xE2 trên IO4 nếu phản hồi 0xE2).
constexpr uint8_t diaChiUartMksPhai = 0xE2;           // Địa chỉ UART ưu tiên khi quét động cơ phải (tự khóa sang 0xE1 trên IO3 nếu phản hồi 0xE1).
constexpr uint32_t chuKyDocUartMks_us = 5000;         // Đọc từ kế MKS mỗi 5 ms (luân phiên 2.5 ms trái / 2.5 ms phải).
constexpr bool chieuTienEncoderMksTraiDuong = true;   // Khớp bánh TRÁI thực tế (GPIO 12/13, UART RX=IO4).
constexpr bool chieuTienEncoderMksPhaiDuong = false;  // Khớp bánh PHẢI thực tế (GPIO 5/6, UART RX=IO3).
constexpr bool batBuTruotUartMksVaoXung = false;      // Tắt hợp nhất UART 0x30 vào PCNT điều khiển vì hai kênh hỏi lệch pha 2,5 ms; vẫn đọc encoder để giám sát.
constexpr float hangSoLocBuUartMks_s = 0.05f;         // Hằng số lọc PT1 (50 ms) hợp nhất bù trượt UART 0x30 êm vào PCNT.
constexpr float tocDoBuUartToiDa_buocTrenGiay = 1200.0f; // Giới hạn tốc độ bù trượt tối đa (steps/s) tránh gai đạo hàm vận tốc.

// Cấu hình encoder từ tính MT6701 ở chế độ AB quadrature (dùng khi dungPhanCungTmcMt = true):
// Mặc định MT6701 xuất 1024 xung/vòng trên mỗi kênh A/B -> giải mã 4 cạnh (x4) = 4096 cạnh/vòng.
constexpr uint32_t soXungEncoderMoiVongDongCo = 4096;
constexpr uint32_t thoiGianLocNhieuEncoder_ns = 1000; // Bộ lọc phần cứng PCNT 1 us chống nhiễu PWM TMC2209.
constexpr bool chieuTienEncoderTraiDuong = false;     // Khớp chiều tiến thực tế với +X IMU.
constexpr bool chieuTienEncoderPhaiDuong = true;      // Khớp chiều tiến thực tế với +X IMU.

// Tham số bộ điều khiển bánh xe vòng kín trên ESP32 (BoDieuKhienBanhVongKin):
// v_lenhStep = v_dat + clamp(kpViTri*(s_ref - s_do) + kpVanToc*(v_dat - v_do) + kiVanToc*int(v_dat - v_do), ±gioiHanBu)
constexpr bool batVongKinBanhXeTmcMt = true;
constexpr float kpViTriBanh_1ps = 8.0f;               // Bù lệch vị trí/góc bánh (1/s): lệch 1 cm bù 0,08 m/s.
constexpr float kpVanTocBanh = 0.35f;                 // Bù lệch vận tốc tức thời (không thứ nguyên).
constexpr float kiVanTocBanh_1ps = 2.0f;              // Tích phân vận tốc bánh (1/s).
constexpr float gioiHanSaiLechViTriBanh_m = 0.030f;   // Chặn sai lệch tích lũy s_ref - s_do tối đa ±3 cm.
constexpr float gioiHanBuVanTocBanh_mps = 0.15f;      // Trần phần vận tốc bù thêm vào lệnh feedforward (±0,15 m/s).
constexpr float gocLoiBamBanhToiDa_rad = 0.45f;       // ~25,8 độ: vượt ngưỡng này liên tục thì tính là trượt bước/kẹt.
constexpr float thoiGianKetBanhToiDa_s = 0.15f;       // Kẹt/mất bước quá 150 ms -> ngắt lỗi MotorFault để bảo vệ.

// MẪU CẤU HÌNH V2: các giá trị tín hiệu dưới đây cần đối chiếu phần cứng thực tế.
// begin() vẫn chuẩn bị phần cứng; enable() chỉ được chấp nhận khi true.
constexpr bool daXacNhanCauHinhPhanCungMotor = true;
constexpr bool mucBatMotorCao = false; // Khớp EN = L của MKS và ENN active-low của TMC2209.
constexpr bool mucDirTienBanhTraiCao = dungPhanCungTmcMt ? false : true;  // Bánh TRÁI MKS (GPIO 12/13) tiến ở mức HIGH để khớp +X IMU.
constexpr bool mucDirTienBanhPhaiCao = dungPhanCungTmcMt ? true : false;  // Bánh PHẢI MKS (GPIO 5/6) tiến ở mức LOW để khớp +X IMU.

constexpr uint32_t thoiGianMucCaoStep_us = 10;       // Giá trị thử, dư rộng cho cả MKS và TMC2209 (TMC cần >= 100 ns).
constexpr uint32_t thoiGianMucThapToiThieuStep_us = 10;    // Giá trị thử.
constexpr uint32_t thoiGianGiuDir_us = 10;        // Giá trị thử (TMC2209 cần >= 20 ns).
constexpr uint32_t thoiGianOnDinhDir_us = 10;       // Giá trị thử.
constexpr uint32_t thoiGianOnDinhEnable_us = 100;  // Giá trị thử.
// MKS: người dùng đang thử 40 kHz ở MStep 64 (trần quy đổi ~0,785 m/s).
// TMC_MT vẫn dùng 32/10 kHz. MCPWM thêm 1 tick bảo vệ khi sync.
// MKS dùng clock 5 MHz: HIGH 51 tick; 40 kHz có chu kỳ 125 tick.
// Đây là trần phát lệnh, không chứng minh mô-men/khả năng bám của motor dưới tải.
constexpr float tanSoStepToiDaMks_hz = 40000.0f;
constexpr float tanSoStepToiDaTmc_hz = 10000.0f;
constexpr float tanSoStepToiDa_hz = dungPhanCungTmcMt ? tanSoStepToiDaTmc_hz : tanSoStepToiDaMks_hz;

constexpr int nhomMcpwm = 0;
// ESP-IDF 5.5.4 trên S3: PLL160M / group prescale 2 / timer prescale 16 = 5 MHz chính xác.
// Chu kỳ khởi tạo 10 ms = 50000 tick < 65536; 10 MHz sẽ vượt bộ đếm.
// Nhánh TMC_MT giữ clock 1 MHz đã dùng trước đây.
constexpr uint32_t doPhanGiaiBoDemMcpwm_hz = dungPhanCungTmcMt ? 1000000u : 5000000u;
constexpr uint32_t chuKyKhoiTaoMcpwm_us = 10000;
constexpr uint32_t thoiGianBaoVeTamDungMcpwm_us = 2;
static_assert(80000000u % doPhanGiaiBoDemMcpwm_hz == 0,
              "Clock MCPWM phai chia het group 80 MHz cua ESP-IDF 5.5.4 tren ESP32-S3");
static_assert((static_cast<uint64_t>(chuKyKhoiTaoMcpwm_us) * doPhanGiaiBoDemMcpwm_hz + 999999u) / 1000000u < 65536u,
              "Chu ky khoi tao MCPWM vuot bo dem 16-bit");

// >= 200 steps/s: dùng xung MCPWM liên tục. Chu kỳ <= 5 ms (khớp nhịp điều khiển 200 Hz).
// < 200 steps/s: tích lũy phần bước và phát từng xung MCPWM hoàn chỉnh.
// Không nâng lệnh nhỏ thành một tốc độ tối thiểu lớn hơn.
constexpr float tanSoChayLienTucToiThieu_hz = 200.0f;

// service() cần được task sở hữu motor gọi đều, mục tiêu 250 us.
// Đây là yêu cầu tích hợp, class không tự tạo task hay timer đánh thức.
constexpr uint32_t chuKyPhucVuMotor_us = 250;
// MKS: phục vụ PCNT/UART tối đa 2 kHz để lịch hỏi 2,5 ms không bị lượng tử theo 1 ms.
// UART vẫn chỉ gửi tổng 400 lượt/s; TMC_MT giữ nhịp đọc encoder 1 ms.
constexpr uint32_t chuKyDocPhanHoiBanh_us = dungPhanCungTmcMt ? 1000u : 500u;
constexpr uint32_t khoangCachPhucVuMotorToiDa_us = 15000;
constexpr uint32_t thoiGianHetHanLenhMotor_us = 25000;
// GPTimer ISR riêng kiểm tra deadline motor, không phụ thuộc MotorTask/esp_timer task.
constexpr uint32_t chuKyGiamSatDocLap_us = 250;

// Không cho tồn đọng cả cụm xung để phát bù sau khi task bị chậm.
constexpr double gioiHanTonDongXungThua_buoc = 10.0;

static_assert(soXungEncoderMoiVongDongCo > 0 && soXungEncoderMksMoiVong > 0, "So xung encoder phai duong");
static_assert(kpViTriBanh_1ps >= 0.0f && kpVanTocBanh >= 0.0f && kiVanTocBanh_1ps >= 0.0f &&
              gioiHanSaiLechViTriBanh_m > 0.0f && gioiHanBuVanTocBanh_mps >= 0.0f &&
              gocLoiBamBanhToiDa_rad > 0.0f && thoiGianKetBanhToiDa_s > 0.0f,
              "Cau hinh vong kin banh xe TMC_MT khong hop le");
#endif
