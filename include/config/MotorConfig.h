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

// Động cơ 1,8 độ (17HS4401S), vi bước đầu vào driver = 32 (6400 xung STEP/vòng khi tỉ số 1:1).
// - Với MKS Servo42C V1.1: đặt Mode = CR_vFOC, MStep = 32, En = L, Baud = 115200, UartAddr = 0xE0.
// - Với TMC2209: đặt MS1/MS2 (hoặc UART) = 32 vi bước, chân ENN active LOW (mucBatMotorCao = false).
constexpr uint16_t soBuocDayDuMoiVongDongCo = 200;
constexpr uint16_t soViBuocDauVaoMks = 32;
constexpr uint16_t soViBuocTmc2209 = soViBuocDauVaoMks;

// Cấu hình đọc hồi tiếp cảm biến từ 14-bit tích hợp sẵn trên MKS Servo42C V1.1 qua 3 chân UART
// (IO1 = TX chung, IO3 = RX động cơ MKS 1, IO4 = RX động cơ MKS 2):
// - Theo tài liệu MKS Servo42C V1.1, lệnh 0x30 trả về carry (int32_t) + value (0..0x3FFF = 16384 xung/vòng).
// - Nhờ tách riêng RX1 (UART1 trên IO3) và RX2 (UART2 trên IO4), cả hai module MKS để chung địa chỉ 0xE0
//   nhận lệnh đồng thời qua IO1 và phản hồi song song ở tốc độ 115200 bps (< 1 ms trên đường dây)!
constexpr bool batUartEncoderMks = !dungPhanCungTmcMt;
constexpr uint32_t soXungEncoderMksMoiVong = 65536;  // Lệnh 0x30 trên MKS Servo42C V1.1 trả carry (int32) + value (0..65535 = 1 vòng).
constexpr uint32_t tocDoBaudUartMks_bps = 115200;     // Tốc độ UART tối đa 115200 bps trên MKS Servo42C V1.1.
constexpr uint8_t diaChiUartMksTrai = 0xE1;           // Địa chỉ UART động cơ MKS trái (0xE1).
constexpr uint8_t diaChiUartMksPhai = 0xE2;           // Địa chỉ UART động cơ MKS phải (0xE2, nhận riêng trên IO4).
constexpr uint32_t chuKyDocUartMks_us = 5000;         // Đọc từ kế MKS mỗi 5 ms (luân phiên 2.5 ms trái / 2.5 ms phải).
constexpr bool chieuTienEncoderMksTraiDuong = false;  // Đảo ngược dấu cũ để khớp chiều tiến thực tế với +X IMU.
constexpr bool chieuTienEncoderMksPhaiDuong = true;   // Đảo ngược dấu cũ để khớp chiều tiến thực tế với +X IMU.
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
constexpr bool mucDirTienBanhTraiCao = false;  // Đã đảo so với cấu hình đầu để bánh quay đỡ đúng chiều nghiêng IMU.
constexpr bool mucDirTienBanhPhaiCao = true;  // Đã đảo so với cấu hình đầu để bánh quay đỡ đúng chiều nghiêng IMU.

constexpr uint32_t thoiGianMucCaoStep_us = 10;       // Giá trị thử, dư rộng cho cả MKS và TMC2209 (TMC cần >= 100 ns).
constexpr uint32_t thoiGianMucThapToiThieuStep_us = 10;    // Giá trị thử.
constexpr uint32_t thoiGianGiuDir_us = 10;        // Giá trị thử (TMC2209 cần >= 20 ns).
constexpr uint32_t thoiGianOnDinhDir_us = 10;       // Giá trị thử.
constexpr uint32_t thoiGianOnDinhEnable_us = 100;  // Giá trị thử.
// Trần phần mềm mặc định 10 kHz (~0,393 m/s ở MStep 32). MCPWM với 10 us HIGH + 10 us LOW
// hỗ trợ tới 50 kHz; sau khi xác nhận đáp ứng driver có thể nâng lên 20000 Hz (~0,785 m/s)
// hoặc giảm MStep xuống 16 để mở rộng dư địa tốc độ (mục M2).
constexpr float tanSoStepToiDa_hz = 10000.0f;

constexpr int nhomMcpwm = 0;
constexpr uint32_t doPhanGiaiBoDemMcpwm_hz = 1000000;
constexpr uint32_t chuKyKhoiTaoMcpwm_us = 10000;
constexpr uint32_t thoiGianBaoVeTamDungMcpwm_us = 2;

// >= 200 steps/s: dùng xung MCPWM liên tục. Chu kỳ <= 5 ms (khớp nhịp điều khiển 200 Hz).
// < 200 steps/s: tích lũy phần bước và phát từng xung MCPWM hoàn chỉnh.
// Không nâng lệnh nhỏ thành một tốc độ tối thiểu lớn hơn.
constexpr float tanSoChayLienTucToiThieu_hz = 200.0f;

// service() cần được task sở hữu motor gọi đều, mục tiêu 250 us.
// Đây là yêu cầu tích hợp, class không tự tạo task hay timer đánh thức.
constexpr uint32_t chuKyPhucVuMotor_us = 250;
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
