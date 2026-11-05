# Hiệu chuẩn và khởi động bằng nút, LED RGB

Cập nhật 05/11/2026 theo sơ đồ người dùng và xác nhận **motor 1 phải, motor 2 trái**.

`UI_Calib` đo bias gyro rồi xác định cách IMU được xoay so với thân xe. Quy trình
chạy trước các task cân bằng và giữ motor tắt. Hoàn tất calib không có nghĩa xe
đã được thử nghiệm cân bằng thành công.

## 1. Phân biệt ba việc cần hiệu chỉnh

| Việc | Cách xác định | Giới hạn |
| --- | --- | --- |
| Bias gyro | Giữ xe đứng yên, trung bình 500 mẫu gyro | Không lấy chuyển động lật xe làm bias |
| Hướng lắp IMU so với thân xe | Tư thế tham chiếu và lượt lật trái → phải quanh trục dọc thân | Không tự biết trục thân nếu thao tác quay sai trục |
| Góc cân bằng cơ học `gocCanBang_rad` | Vị trí trọng tâm và kiểm chứng trên xe thực | Không suy ra chỉ từ tư thế người đang đỡ xe |

Ví dụ bạn giữ xe chúi trước 6° khi xác nhận bước đầu: thuật toán mới giữ phép đo
pitch khoảng 6°, không lấy tư thế đó làm pitch 0°. Tuy nhiên, mốc **roll 0** vẫn
tham chiếu theo tư thế ban đầu. Vì vậy cần đặt hai bánh cùng độ cao, giữ thân xe
không nghiêng trái/phải khi lấy mốc. Giữ gần thẳng trước/sau là đủ; không cần
tìm đúng tư thế xe tự cân bằng ở bước này.

## 2. Chân sử dụng

| Tín hiệu | GPIO | Ghi chú |
| --- | --- | --- |
| ICM20602 SDA / SCL | 10 / 11 | Theo sơ đồ mới |
| ICM20602 SA0 / CS | 8 / 9 | Firmware đặt LOW / HIGH để dùng I²C tại `0x68` |
| ICM20602 INT | 7 | Ngắt data-ready sau khi UI hoàn tất |
| Motor 1 phải DIR / STEP | 12 / 13 | Net `DIR1` / `ST1` |
| Motor 2 trái DIR / STEP | 5 / 6 | Net `DIR2` / `ST2` |
| EN chung | 2 | Cấu hình hiện tại active LOW, phải khớp MKS |
| Nút xác nhận | 3 | Hiện cấu hình nút nối xuống GND, `INPUT_PULLUP` |
| LED RGB | 48 | Mặc định thử WS2812, thứ tự GRB; sơ đồ chưa thể hiện LED |

Header 4P theo thứ tự chân 1–4 là GPIO1, GPIO3, GPIO4, GND. Với cấu hình nút hiện
tại, nối nút giữa chân 2 và chân 4 của header. Sơ đồ chỉ cho thấy header, chưa
cho thấy mạch nút. Nếu nút thực tế kéo lên 3,3 V khi nhấn, đổi
`nutCalibMucThap = false`; không đưa 5 V vào GPIO.

`chanRgbCalib` và `thuTuRgbCalibLaGRB` cần khớp phiên bản Super Mini. Có thể đặt
`chanRgbCalib = -1` để dùng hướng dẫn Serial khi chưa xác minh LED.
Nhả nút lúc bật nguồn/reset: GPIO3 tham gia chọn nguồn JTAG ở giai đoạn khởi động,
tùy eFuse. [Datasheet ESP32-S3](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)

## 3. Chuẩn bị

1. Cố định PCB/IMU vào thân xe; nếu thay vị trí lắp sau calib phải reset và calib lại.
2. Xác định đầu xe. Trái/phải tính khi nhìn từ đuôi về đầu xe. Trục dọc là đường
   từ đuôi đến đầu, khác trục bánh chạy từ trái sang phải.
3. Mở Serial 115200 để thấy hướng dẫn và lỗi. Giữ cờ
   `daXacNhanCauHinhPhanCungMotor = false` trong lần kiểm tra cảm biến đầu tiên.
4. Nhả nút, cấp nguồn/reset. Đợi LED xanh dương và dòng `CALIB`.

## 4. Các bước theo màu

| Bước, màu LED | Thao tác người dùng | Firmware làm gì / điều kiện chuyển bước |
| --- | --- | --- |
| 1. Xanh dương | Đặt hai bánh cùng cao; giữ thân gần thẳng, không nghiêng bên; **nhấn rồi nhả** | Chờ xác nhận hoàn chỉnh, chưa lấy tư thế này làm góc cân bằng |
| 2. Vàng | Giữ hoàn toàn yên khoảng vài giây | Đo bias gyro 500 mẫu; lấy 80 mẫu gia tốc làm mốc tham chiếu |
| 3. Xanh lơ | Nghiêng thân sang **trái 30–45°**, giữ yên, **nhấn rồi nhả** | Ghi nhận yêu cầu lấy mốc trái |
| 4. Vàng | Tiếp tục giữ yên bên trái | Đợi 150 ms cho rung nút lắng xuống, lấy 80 mẫu gia tốc |
| 5. Tím | Lật chậm từ trái qua giữa sang **phải 30–45°**, giữ hướng đầu xe; dừng yên rồi **nhấn, nhả** | Thu gyro suốt lượt quay; kết thúc và xác nhận trong 30 s |
| 6. Vàng | Giữ yên bên phải | Lấy 80 mẫu gia tốc phải, tính và kiểm tra ma trận lắp |
| 7. Trắng | Đưa xe về gần góc đặt khởi động; giữ yên, **không cần bấm nút** | Chờ 80 mẫu liên tiếp đạt điều kiện tư thế, tối đa 30 s |
| 8. Xanh lá | Xem `READY` và telemetry Serial | Khởi tạo task xong; sẵn sàng nhận yêu cầu `a`, motor chưa tự bật |
| Đỏ + `CALIB LOI` | Đưa xe về tư thế bước 1, giữ yên, **nhấn rồi nhả** | Làm lại từ phép đo bias; lần thử lại chuyển thẳng sang vàng |
| Đỏ + `BOOT ERROR` | Xem lỗi Serial, xử lý kết nối/cấu hình rồi reset | Lỗi khởi tạo không được giải quyết bằng nút xác nhận |

Trong bước tím, hãy lật **thân xe**, không xoay hai bánh để chúi trước/sau, không
quay đầu xe để đổi hướng. Nên mất khoảng 2–4 s để chuyển trái sang phải. Giữ
nguyên cách đỡ thân để trục quay bám theo trục dọc. Tránh kéo dây hoặc bấm nút
làm thân xoay thêm. Các khoảng thời gian ở trên là hướng dẫn thao tác, không phải
bằng chứng phép đo đã đạt.

Nút lọc dội 35 ms. Một xác nhận cần nhấn và nhả đều ổn định; giữ nút qua bước
kế tiếp không tự xác nhận bước đó. Sau `READY`, nút hiện chỉ kết thúc vai trò
calib: **chưa có chức năng arm, disarm hoặc dừng khẩn bằng nút này**.

LED xanh lá hiện chỉ biểu thị khởi tạo hoàn tất, **không đổi theo lỗi hay trạng
thái arm trong lúc chạy**. Theo dõi Serial để biết trạng thái thực.

## 5. Nếu dừng ở LED trắng hoặc báo lỗi

LED trắng kiểm tra:

```text
abs(pitch - gocCanBang_rad) <= 0,10 rad ≈ 5,73°
abs(roll)                 <= 0,10 rad ≈ 5,73°
norm(gyro đã trừ bias)     <= 0,06 rad/s
abs(norm(accel) - g)       <= 0,50 m/s²
80 mẫu liên tiếp đạt điều kiện
```

Serial in `CALIB_RETURN pitch=... roll=... target=... rad` mỗi khoảng 500 ms.
Hãy dựa vào giá trị này để đưa thân về gần `target`. Nếu ban đầu giữ lệch 6°,
thuật toán giữ lệch đó nên có thể cần đưa xe thẳng hơn mới qua bước trắng.
Đây chỉ là kiểm tra tư thế khởi động; nó không xác nhận trọng tâm đã ở trên trục bánh.

Các phép lấy mốc tĩnh đợi 150 ms, cần 80 mẫu trong 2,5 s, phương sai gia tốc tổng
không vượt `0,04 (m/s²)²`. Lượt quay cần ít nhất 40 mẫu chuyển động hợp lệ;
trục gyro chính phải chiếm ít nhất 95% năng lượng; góc tích phân phải khớp hai
mốc trong 0,35 rad. Tư thế đầu cần pitch trong khoảng ±8,63° theo phép kiểm
tra hình học. Các ngưỡng là giá trị khởi đầu đang dùng trong code.

Nếu báo lỗi sau lượt tím: kiểm tra có lật đúng trái → phải, giữ đúng trục dọc,
hai đầu đủ nghiêng và đã giữ yên trong các bước vàng. Nếu `BOOT ERROR: ICM20602`,
kiểm tra SDA10, SCL11, CS9 ở HIGH, SA08 ở LOW, nguồn, GND và điện trở kéo lên I²C.
Driver yêu cầu `WHO_AM_I = 0x12`.

## 6. Kích hoạt cân bằng sau calib

1. Xác minh cấu hình MKS: chế độ STEP/DIR vòng kín, vi bước 32, EN cùng cực tính
   và chiều quay từng bên. Mapping motor trái/phải đúng chưa chứng minh mức DIR
   tiến đã đúng: `mucDirTienBanhTraiCao = true`,
   `mucDirTienBanhPhaiCao = false` vẫn cần kiểm chứng trên xe.
2. Khi đã xác minh phần cứng, đặt `daXacNhanCauHinhPhanCungMotor = true` trong
   `MotorConfig.h`, build/nạp lại. Không sửa cờ chỉ để vượt lỗi khi chưa kiểm tra.
3. Chạy lại calib, chờ xanh lá + `READY`. Giữ xe gần góc đặt, ít rung, trên giá
   thử có thể đỡ thân nếu chiều điều khiển sai.
4. Gửi riêng `a` qua Serial **sau READY**. Byte Serial còn chờ trong giai đoạn
   calib bị xóa trước khi READY để không phát lại lệnh cũ.
5. Kiểm tra `EN=1`, `phase=2`, `fault=none`. `phase=1` là giai đoạn chờ lệnh 0
   của phiên khởi động; `phase=2` mới là chạy điều khiển. `EN` trong telemetry
   là trạng thái cho phép logic, không phải mức điện áp GPIO2.

Arm còn kiểm tra dữ liệu IMU/PCNT mới và hợp lệ, độ tin cậy gia tốc, pitch gần
`gocCanBang_rad`, roll nhỏ, tốc độ nghiêng nhỏ, lỗi motor và đồng bộ phiên.
Nếu từ chối, đọc `last`, `fault`, `phase`; xem bảng trong
[hướng dẫn sử dụng](HUONG_DAN_SU_DUNG.md). Cờ phần cứng vẫn `false` ở bản hiện tại.

`d` tắt motor; `x` dừng khẩn có chốt lỗi; `k` yêu cầu xóa lỗi khi trạng thái đã
hợp lệ. `w/s` tiến/lùi; `j/l` quay trái/phải; `z` đưa mục tiêu về 0 theo giới hạn
gia tốc. Lệnh di chuyển cần lặp trước 500 ms, hết hạn sẽ đưa mục tiêu về 0.
Chỉ thử di chuyển sau khi xe đã giữ cân bằng ổn định.

## 7. Chỉnh góc cân bằng và sửa chương trình

`gocCanBang_rad` nằm trong `include/config/ControlConfig.h`. Giá trị mặc định 0
là giả định khởi đầu. Với trọng tâm có tọa độ `(x, z)` so với trục bánh và quy
ước pitch dương chúi trước, mô hình tĩnh cho `theta = -atan2(x, z)`.
Ví dụ **giả định** trọng tâm lệch trước 2 mm, cao 25 mm: góc khoảng −4,57°.
Con số này không phải góc đã đo cho xe của bạn.

Xác định chiều điều khiển trước khi chỉnh PID. Khi xe đã có thể giữ cân bằng
trên giá thử, ghi log góc, tốc độ, lệnh motor ở mục tiêu tốc độ 0; kiểm tra ảnh
hưởng ma sát, mặt sàn và tải rồi chỉnh góc đặt từng bước nhỏ. Hiện chưa có thuật
toán tự tìm góc cân bằng hay lưu góc vừa tìm bằng nút.

| Muốn sửa | File / hàm |
| --- | --- |
| Chân, cực tính nút, chân LED | `include/config/PinConfig.h` |
| Bật UI, thứ tự màu RGB | `include/config/CalibConfig.h` |
| Debounce, thời hạn, số mẫu tĩnh, màu, lời nhắc | `src/calib/UI_Calib.cpp` |
| Điều kiện nhận lượt lật, PCA và dựng ma trận | `lib/estimation/HieuChuanQuayNgang.cpp` |
| Góc cơ học mong muốn | `include/config/ControlConfig.h` |
| Giới hạn góc arm/ngã | `include/config/SafetyConfig.h` |
| Cực tính DIR/EN, vi bước và xác nhận phần cứng | `include/config/MotorConfig.h` |

Bias và ma trận từ UI chỉ tồn tại trong RAM, mỗi reset chạy lại. Nếu đặt
`batUiCalib = false`, hệ thống vẫn hiệu chuẩn bias lúc khởi động nhưng dùng
ma trận khai báo tại `ImuMountConfig.h`; cần chuẩn bị đúng ma trận trước.
Công cụ offline dùng gravity ở tư thế chuẩn có giả thiết khác, được giải thích
trong [báo cáo lắp IMU](BAO_CAO_BU_LECH_LAP_IMU.md).

Xem [báo cáo sơ đồ, flow và tính toán](DOI_CHIEU_SO_DO_VA_FLOW_CALIB.md) và
[danh mục hàm](DANH_MUC_HAM.md) để đi sâu vào chương trình.

