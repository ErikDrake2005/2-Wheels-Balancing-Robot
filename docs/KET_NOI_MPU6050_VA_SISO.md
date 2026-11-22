# Cấu hình phần cứng MPU6050, Lọc khử rung động cơ và Ứng dụng SISO

Ngày cập nhật: 21/11/2026. Môi trường PlatformIO: **`lolin_s3_mini_mpu6050`** (`-DCAU_HINH_IMU=IMU_MPU6050_I2C`).
Các profile hiện có (`lolin_s3_mini` cho ICM20602 và `lolin_s3_mini_lsm6dsv` cho LSM6DSV) được giữ nguyên độc lập 100%.

---

## 1. Đối chiếu sơ đồ chân & Các lưu ý kỹ thuật quan trọng

| Chân MPU6050 | Nối vào ESP32-S3 | Trạng thái đánh giá & xử lý trong Firmware |
|---|---|---|
| `VCC` / `GND` | `3V3` / `GND` | Nguồn 3,3 V ổn định, chung mass với ESP32-S3. |
| `SDA` | `GPIO10` (`IO10`) | Hợp lệ — I²C Fast-Mode `400 kHz` (~360 µs đọc 14 byte dữ liệu). |
| `SCL` | `GPIO11` (`IO11`) | Hợp lệ — I²C Fast-Mode `400 kHz` (module GY-521 đã có sẵn trở kéo `4,7 kΩ` lên 3V3). |
| `AD0` | `GPIO7` (`IO7`) | Hợp lệ — Driver ép cứng `GPIO7 = OUTPUT LOW` trước `Wire.begin()` để khóa địa chỉ I²C cố định `0x68` (tránh trôi địa chỉ sang `0x69`). |
| `INT` | **Để trống** (`-1`) | **Cần xử lý đặc biệt (Đã giải quyết):** Vì không có ngắt phần cứng `INT`, firmware dùng bộ hẹn giờ `esp_timer` bám pha (`4300 µs` ngủ + `250 µs` dò sườn) kiểm tra cờ `DATA_RDY_INT` tại thanh ghi `0x3A`, giữ đúng nhịp **200 Hz** mà không treo task hay trùng chân `GPIO7` của `AD0`. |
| `XCL` (`AUX_CL`)<br>`XDA` (`AUX_DA`) | *Đang dự kiến: `XCL -> 3V3`, `XDA -> GND`* | **SAI SÓT PHẦN CỨNG CẦN SỬA DÂY:** Theo datasheet InvenSense PS-MPU-6000A-00 (Mục 7.1), `XDA` và `XCL` là bus I²C Master phụ nối từ kế ngoài; khi không sử dụng **PHẢI ĐỂ TRỐNG (NC)**. Nếu nối cứng `XCL` vào `3V3` và `XDA` xuống `GND`:<br>1. Khi `I2C_BYPASS_EN = 1` (`0x37` bit 1), công tắc nội bộ nối thẳng `SDA` xuống `GND` và `SCL` cứng lên `3V3` (treo chết bus I²C và gây ngắn mạch `3V3–GND` khi `IO11` kéo thấp xung clock).<br>2. Khi `I2C_MST_EN = 1` (`0x6A` bit 5), khối Master phụ kéo `XCL` xuống thấp gây chập thẳng `3V3–GND` làm hỏng chân IC.<br>**Bảo vệ kép trong code:** `Mpu6050I2cDriver` khóa cứng `USER_CTRL (0x6A) = 0x00` và `INT_PIN_CFG (0x37) = 0x00` rồi đọc kiểm tra lại mỗi lần khởi động; tuy nhiên bạn **nên tháo dây ở `XCL` và `XDA` (để trống cả hai chân)** trước khi cấp nguồn. |

### Lưu ý thêm khi vận hành thực tế với module GY-521 (MPU6050)
1. **Kiểm tra mã `WHO_AM_I (0x75)`:** Chip MPU6050 chuẩn trả về `0x68`. Một số bo GY-521 trôi nổi trên thị trường gắn chip clone/MPU6500 trả về `0x70`, `0x72` hoặc `0x98`; nếu gặp trường hợp này Serial sẽ in rõ `BOOT ERROR: MPU6050 init WHO=0x.. expected=0x68`.
2. **Hiệu chuẩn hướng lắp lần đầu (`xe_mpu6050`):** Tương tự LSM6DSV, lần đầu nạp firmware `lolin_s3_mini_mpu6050`, hệ thống yêu cầu hiệu chuẩn 3 tư thế qua USB Serial / nút `BOOT (GPIO0)` để lưu ma trận xoay đúng chiều gá lắp module GY-521 vào phân vùng NVS riêng `"xe_mpu6050"`.

---

## 2. Chuỗi thuật toán lọc nhiễu & khử rung động cơ cho MPU6050

MPU6050 có mật độ nhiễu gyro cao hơn LSM6DSV/ICM20602 và không có thanh ghi lọc số bậc hai riêng cho gia tốc kế, nên rất nhạy với rung vi bước động cơ bước (17HS4401S ở dải `40–85 Hz`). Firmware trang bị **4 tầng lọc phối hợp**:

1. **Tầng 1 — Lọc chống răng cưa phần cứng (`DLPF_CFG = 3` trên chip MPU6050):**
   - Khóa PLL theo trục X Gyro (`PWR_MGMT_1 = 0x01`), đặt `CONFIG (0x1A) = 3` cho băng thông Gyro `42 Hz` (trễ `4,8 ms`) và Accel `44 Hz` (trễ `4,9 ms`) trước khi chia mẫu `SMPLRT_DIV = 4` xuống `200 Hz`. Độ trễ phần cứng giữa Accel và Gyro lệch nhau chỉ `0,1 ms`, giúp khâu trừ bù gia tốc bánh xe chính xác.
2. **Tầng 2 — Bộ lọc khía SISO Biquad Notch (`60 Hz, Q = 3.0`, [LocBiquad.h](../lib/estimation/LocBiquad.h)):**
   - Đặt trên nhánh tốc độ góc `tocDoNghien_radps` trước khâu D để triệt tiêu đỉnh cộng hưởng khung cơ khí / sóng hài vi bước động cơ mà chỉ làm tăng trễ nhóm ở vùng cân bằng `1–5 Hz` thêm `~0,6 ms`. (Có thể chỉnh tần số hoặc đặt `0.0f` trong [Mpu6050Tuning.h](../include/config/Mpu6050Tuning.h) để tắt).
3. **Tầng 3 — Bộ lọc thông thấp SISO Butterworth bậc 2 (`35 Hz`, `-40 dB/decade`, [LocBiquad.h](../lib/estimation/LocBiquad.h)):**
   - Rời rạc hóa bằng phép biến đổi song tuyến Tustin có hiệu chỉnh tần số trước (*frequency pre-warping*) và khởi tạo xác lập êm (*bumpless init*). Ở `80 Hz`, chuỗi Notch + Butterworth bậc 2 suy giảm nhiễu tới **`-28,2 dB`** (mạnh gấp ~25 lần về biên độ so với `-7,9 dB` của lọc bậc 1 PT1), cắt hẳn tiếng kêu rè/giật xung `STEP` của động cơ khi đứng yên.
4. **Tầng 4 — Lọc bù 3D trọng số thích nghi ([ComplementaryEstimator.cpp](../lib/estimation/ComplementaryEstimator.cpp)):**
   - Đặt `locGoc_s = 0,45 s`, tự động hạ trọng số tin cậy gia tốc khi xuất hiện xung rung cơ khí hoặc gia tốc ngang lớn.

---

## 3. Khối điều khiển SISO trong hệ thống để viết báo cáo môn học

Bên cạnh việc đóng khung vòng điều khiển góc dọc thành hệ SISO (như mô hình $G_p(s) = \frac{-1}{L_{\text{eff}}s^2 - g}$), hệ thống nay có một module C++ **SISO thuần túy 100% (1 Ngõ vào $\rightarrow$ 1 Ngõ ra)** tách riêng tại [BoDieuKhienSiso.h](../lib/controllers/BoDieuKhienSiso.h) và [BoDieuKhienSiso.cpp](../lib/controllers/BoDieuKhienSiso.cpp), trực tiếp điều khiển **Vòng giữ hướng quay ngang (Yaw Heading Hold)** của xe trong [CascadeController.cpp](../lib/controllers/CascadeController.cpp):

- **Tín hiệu đặt $r_\psi(t)$:** Góc hướng mục tiêu $\psi_{\text{ref}}(t) = \frac{\Delta s_{\text{ref}}}{b}$ (`rad`, với $b = 0{,}20\text{ m}$ là khoảng cách tâm hai bánh).
- **Ngõ ra đối tượng $y_\psi(t)$:** Góc hướng thực đo $\psi(t) = \frac{s_R(t) - s_L(t)}{b}$ (`rad`).
- **1 Ngõ vào duy nhất vào `BoDieuKhienSiso`:** Sai lệch góc hướng $e_\psi(t) = r_\psi(t) - y_\psi(t)$ (`saiLechHuongQuay`).
- **1 Ngõ ra duy nhất từ `BoDieuKhienSiso`:** Tốc độ góc quay bù $\omega_{\text{bu}}(t)$ (`tocDoQuayBuGiuHuong`, giới hạn `±0,45 rad/s`).
- **Hàm truyền đối tượng SISO bậc 2 loại 1:**
  $$G_\psi(s) = \frac{\Psi(s)}{\Omega_{\text{cmd}}(s)} = \frac{1}{s(\tau_m s + 1)}$$
- **Hàm truyền bộ điều khiển SISO bù sớm pha (Lead-PD / PIDF) & Rời rạc hóa Tustin:**
  $$C_\psi(s) = \frac{\Omega_{\text{bu}}(s)}{E_\psi(s)} = K_p + \frac{K_i}{s} + \frac{K_d s}{\tau_d s + 1}$$
  với phương trình sai phân chạy ở $T_s = 0{,}005\text{ s}$ (`Kp = 1.8`, `Ki = 0.0`, `Kd = 0.15`, `tau_d = 0.020 s`):
  $$u_D[k] = \frac{2\tau_d - T_s}{2\tau_d + T_s} u_D[k-1] + \frac{2K_d}{2\tau_d + T_s}\big(e_\psi[k] - e_\psi[k-1]\big), \qquad \omega_{\text{bu}}[k] = \operatorname{clamp}\big(K_p e_\psi[k] + u_I[k] + u_D[k],\ \pm 0{,}45\big)$$

---

## 4. Lệnh biên dịch & Nạp Firmware

```powershell
# Biên dịch kiểm tra profile MPU6050:
pio run -e lolin_s3_mini_mpu6050

# Nạp lên ESP32-S3 Super Mini:
pio run -e lolin_s3_mini_mpu6050 -t upload
```
