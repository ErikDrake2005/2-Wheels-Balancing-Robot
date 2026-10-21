#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H
#include <cstdint>

// Dong co 1,8 do, vi buoc dau vao MKS = 32.
constexpr uint16_t soBuocDayDuMoiVongDongCo = 200;
constexpr uint16_t soViBuocDauVaoMks = 32;

// MẪU CẤU HÌNH V2: các giá trị tín hiệu dưới đây cần đối chiếu MKS thực tế.
// begin() vẫn chuẩn bị phần cứng; enable() chỉ được chấp nhận khi true.
constexpr bool daXacNhanCauHinhPhanCungMotor = false;
constexpr bool mucBatMotorCao = false; // Phải khớp menu EN = L.
constexpr bool mucDirTienBanhTraiCao = true;  // Chưa xác nhận chiều.
constexpr bool mucDirTienBanhPhaiCao = false; // Chưa xác nhận chiều.

constexpr uint32_t thoiGianMucCaoStep_us = 10;       // Giá trị thử, không phải datasheet.
constexpr uint32_t thoiGianMucThapToiThieuStep_us = 10;    // Giá trị thử.
constexpr uint32_t thoiGianGiuDir_us = 10;        // Giá trị thử.
constexpr uint32_t thoiGianOnDinhDir_us = 10;       // Giá trị thử.
constexpr uint32_t thoiGianOnDinhEnable_us = 100;  // Giá trị thử.
constexpr float tanSoStepToiDa_hz = 10000.0f; // Trần phần mềm thử nghiệm.

constexpr int nhomMcpwm = 0;
constexpr uint32_t doPhanGiaiBoDemMcpwm_hz = 1000000;
constexpr uint32_t chuKyKhoiTaoMcpwm_us = 10000;
constexpr uint32_t thoiGianBaoVeTamDungMcpwm_us = 2;

// >= 1000 steps/s: dùng xung MCPWM liên tục. Chu kỳ <= 1 ms.
// < 1000 steps/s: tích lũy phần bước và phát từng xung MCPWM hoàn chỉnh.
// Không nâng lệnh nhỏ thành một tốc độ tối thiểu lớn hơn.
constexpr float tanSoChayLienTucToiThieu_hz = 1000.0f;

// service() cần được task sở hữu motor gọi đều, mục tiêu 250 us.
// Đây là yêu cầu tích hợp, class không tự tạo task hay timer đánh thức.
constexpr uint32_t chuKyPhucVuMotor_us = 250;
constexpr uint32_t khoangCachPhucVuMotorToiDa_us = 2000;
constexpr uint32_t thoiGianHetHanLenhMotor_us = 20000;

// Không cho tồn đọng cả cụm xung để phát bù sau khi task bị chậm.
constexpr double gioiHanTonDongXungThua_buoc = 2.0;
#endif
