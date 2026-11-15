# Calib gặp rung và kiểm tra ICM20602 sau reset

**Bổ sung 08/11/2026:** kiểm tra boot và tự thu lại khi rung vẫn áp dụng. Mốc
trái/phải, test roll và số liệu build bên dưới mô tả bản 05/11. UI hiện dùng
[mốc ĐẦU/ĐUÔI gần 90°](CALIB_DAU_DUOI_VA_KIEM_CHUNG_GOC_2026-11-08.md).

## Hiện tượng và nguyên nhân trong code

Người dùng xác nhận LED đỏ xuất hiện **sau khi nhấn rồi nhả nút** ở bước xanh
dương. Bước xanh dương chỉ chờ nút; sau xác nhận, firmware chuyển sang vàng để
đo gyro và mốc gia tốc. Code trước đây thử gyro một lượt, rồi từ chối cả lượt
calib nếu một mẫu gia tốc/gyro ở mốc tĩnh vượt ngưỡng. Khoảng đợi sau thao tác
nút chỉ 150 ms, nên rung còn sót lại dễ kết thúc phép đo.

Đây là nguyên nhân nhạy với rung tìm được trong phần mềm. Chưa có log xe thật
để khẳng định mọi lần đỏ đều do rung; dây/nguồn/I²C cũng có thể gây lỗi. Bản sửa
in nguyên nhân cụ thể để phân biệt các trường hợp.

## Hành vi đã sửa

- Đợi **500 ms** sau thao tác trước khi đo bias hoặc mốc tĩnh.
- Gyro cần **500 mẫu ổn định**. Khoảng đo không đạt sẽ bị bỏ và tự thu lại,
  tối đa **30 s**, không bắt người dùng nhấn nút lại vì một lần rung.
- Mỗi mốc gia tốc cần **80 mẫu ổn định**. Một mẫu chuyển động hoặc một khoảng
  có phương sai cao chỉ làm thu lại mốc hiện tại; thời hạn **30 s mỗi mốc**.
- Giữ ngưỡng độ lệch chuẩn gyro **0,5°/s**, phương sai gia tốc XYZ tổng
  **0,04 (m/s²)²**, norm gia tốc lệch g tối đa **0,5 m/s²**. Mốc tĩnh kiểm cả
  phương sai gyro và gia tốc, ngoài ngưỡng gyro tức thời sau trừ bias **0,06 rad/s**.
- Khoảng thiếu mẫu trên **50 ms** làm xóa dữ liệu tĩnh đang tích lũy. Không có
  mẫu mới **500 ms** là lỗi dữ liệu; không chờ đủ 30 s rồi gọi nhầm là rung.
- Trong lượt lật, khoảng mẫu trên 50 ms bị từ chối vì tích phân góc đã mất thông
  tin. Lỗi I²C ở mọi bước được báo ngay, không bị che bởi cơ chế thu lại khi rung.
- Đo lại gyro xóa bias cũ trước khi thu. Bias mới chỉ có hiệu lực sau khi phép
  đo đạt; không lấy gyro đã trừ bias cũ làm bias mới.

Vẫn phải đỡ/cố định thân khi đo bias. Tự thu lại không có nghĩa có thể hiệu chuẩn
trong lúc lật xe. Một chuyển động quay đều rất chậm có thể khó phân biệt với
bias chỉ bằng dữ liệu IMU; kiểm tra thống kê không thay thế việc giữ xe yên.

## Kiểm tra tự động mỗi lần ESP32 khởi động

```text
EN tắt → LED cam → begin ICM20602/reset/cấu hình
       → đọc lại WHO_AM_I và cấu hình
       → chờ 32 mẫu data-ready mới
       → PASS: LED xanh dương, chờ nút calib
       → FAIL: LED đỏ, BOOT ERROR; không tạo các task cân bằng
```

`kiemTraSauKhoiDong()` chạy cả khi UI bị tắt. Nó kiểm tra:

1. ID `0x12` sau khi cấu hình/reset ICM.
2. Tần số chia mẫu, DLPF, thang accel/gyro, chế độ nguồn, cấu hình chân ngắt và
   cho phép data-ready khớp giá trị đã yêu cầu.
3. Xóa cờ data-ready tồn đọng; đọc đủ **32 mẫu hợp lệ liên tiếp** trong **1,5 s**.
   Mỗi mẫu cần cờ mới và đủ 14 byte accel/nhiệt độ/gyro.
4. Dữ liệu hữu hạn; norm gia tốc trong **2–30 m/s²**, nhiệt độ **−45 đến 90°C**.
   Đây là ngưỡng kiểm tra dữ liệu rộng trong firmware, không phải miền bảo đảm
   độ chính xác hoặc giới hạn vận hành thay cho datasheet.

Kiểm tra này không yêu cầu góc dựng xe chính xác hay đã có bias. Dữ liệu tĩnh có
giá trị giống nhau vẫn được chấp nhận nếu có các sự kiện data-ready mới. Chuỗi
toàn 0/0xFF không vượt kiểm tra norm gia tốc. Một lỗi bus làm dừng kiểm tra.

Bit data-ready của `INT_STATUS` được xóa khi đọc, vì vậy phép kiểm tra chờ các
lần báo dữ liệu mới thay vì đọc lặp một frame. Đây là kiểm tra giao tiếp/dữ liệu,
**không phải MEMS self-test** và chưa kiểm chứng dây INT vật lý đến GPIO7;
runtime còn giám sát ngắt và tuổi mẫu.
[ICM20602 datasheet, mục 9.27–9.32](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000176-icm-20602-v1.1.pdf)

## Cách đọc Serial 115200

Nạp firmware, mở monitor rồi reset ESP32. Khi giao tiếp đạt, cần thấy dạng:

```text
ICM_BOOT PASS WHO=0x12 good=32 bad=0 time_ms=... i2c=0 reg=0x3B
ICM_BOOT_SAMPLE a_mps2=... gyro_radps=... T_C=...
CALIB: dat hai banh cung cao, giu khung gan thang, nhan roi nha...
```

Các dấu `...` là giá trị đo trên board, không phải log đã thu trên xe. `gyro_radps`
ở bước boot chưa trừ bias; vector còn trong hệ cảm biến.

| Dòng/nhãn | Ý nghĩa | Thao tác |
| --- | --- | --- |
| `CALIB_WAIT ... rung ... tu thu lai` | Bus vẫn đọc được, khoảng đo chưa đủ ổn định | Tiếp tục giữ yên, không bấm lại; LED vẫn vàng |
| `CALIB_GYRO PASS bias_dps=...` | Đã có bias hợp lệ cho ba trục | Tiếp tục theo màu LED |
| `CALIB_STATIC PASS buoc=... retry=...` | Mốc tĩnh đã đạt; retry đếm các lần bỏ dữ liệu/thu lại | Không phải lỗi cần reset |
| `het_30s_cho_on_dinh` | Không có khoảng dữ liệu đủ yên trong 30 s | Cố định thân tốt hơn rồi xác nhận lại theo hướng dẫn LED đỏ |
| `loi_i2c=2` | NACK địa chỉ/giao dịch theo mã Wire | Kiểm tra nguồn, GND, SDA10/SCL11, CS9/SA08 |
| `loi_i2c=5` | Timeout giao dịch theo mã Wire | Kiểm tra bus và nguồn; cần log/đo thực tế trước khi chỉnh timeout |
| `i2c=6` hoặc `loi_i2c=6` | Không nhận đủ số byte yêu cầu | Kiểm tra kết nối và nhiễu bus |
| `mat_mau_moi` / `khong_du_mau_moi` | Không nhận đủ sự kiện data-ready | Kiểm tra ICM còn chạy, cấu hình và nguồn |
| `sai_who_am_i` | ID không đúng ICM20602 | Đối chiếu chip/địa chỉ/kết nối |
| `sai_cau_hinh` | Thanh ghi đọc lại khác cấu hình mong muốn | Kiểm tra nguồn/reset và giao tiếp |
| `du_lieu_bat_thuong` | Có frame nhưng không đạt kiểm tra dữ liệu | Xem `ICM_BOOT_SAMPLE`, không coi ACK là đã đo đúng |

Nếu lỗi ngay trong `begin()`, dòng `BOOT ERROR: ICM20602 init` in ID đã đọc,
mã I²C và thanh ghi giao dịch cuối. `reg` là thanh ghi cuối được truy cập, không
luôn là thanh ghi sai cấu hình. Xanh lá vẫn chỉ READY; nút không arm và motor
chưa tự bật. Quy trình sau đó xem [UI_Calib](HUONG_DAN_UI_CALIB.md).

## Code cần đọc hoặc chỉnh

| File / thành phần | Vai trò |
| --- | --- |
| `include/config/ImuConfig.h` | Thời gian đợi, số mẫu boot, ngưỡng dữ liệu/tĩnh |
| `Icm20602Driver::kiemTraSauKhoiDong()` | Test giao tiếp/cấu hình/chuỗi mẫu sau reset |
| `Icm20602Driver::docMauMoi()` | Phân biệt chưa có mẫu (`DuLieuCu`) với lỗi bus (`LoiIO`); xóa frame cũ |
| `Icm20602Driver::hieuChinhConQuay()` | Tự thử lại khoảng đo gyro, kiểm tra gia tốc cùng lúc |
| `lib/imu/src/ThongKeMauImu.h` | Welford sáu thành phần; tính phương sai ổn định mà không lưu mảng 500 mẫu |
| `UI_Calib::layGiaTocTinh()` | Tự thu lại mốc tham chiếu/trái/phải khi rung |
| `UI_Calib::kiemTraLoiDoc()` | Tách lỗi I²C và mất mẫu khỏi lỗi thao tác |
| `RobotApp::begin()` | Gọi test boot trước UI và in báo cáo; dừng khởi động nếu không đạt |

## Kiểm thử

Build `lolin_s3_mini`: PASS, RAM **57.044 byte**, flash **474.070 byte**.
Bản ghi kiểm thử và SHA-256 firmware nằm tại
`docs/du_lieu_tinh_toan/PHIEN_KIEM_THU_ICM_BOOT.json`.
Sáu nhóm native: PASS. Nhóm mới `imu_boot_calib_tests.cpp` chạy chính driver và
UI với bus, đồng hồ, nút, RGB giả; **43 điều kiện** gồm:

- Boot thành công và reset lại khi ICM vẫn còn nguồn.
- Sai ID/cấu hình, NACK ở status và frame, đọc thiếu byte.
- Không có data-ready, frame cũ phải bị xóa, dữ liệu toàn 0/0xFF.
- Rung nhẹ vẫn qua kiểm tra boot; dữ liệu tĩnh không bắt buộc phải thay đổi giá trị.
- Rung ở đầu phép đo gyro rồi yên: tự thử lại và đo đúng bias; đo lại không dùng bias cũ.
- Rung kéo dài: từ chối sau 30 s; mất mẫu và I²C báo lỗi riêng.
- Chạy trọn UI với một mẫu rung ở từng mốc đứng/trái/phải: thu lại tại chỗ, không
  chuyển đỏ, vẫn dựng được ma trận hợp lệ.

Chưa upload hoặc đo I²C/LED/cảm biến thật trong lần sửa này. Để xác nhận bản sửa
trên xe, lưu log từ `ICM_BOOT` đến lúc qua các mốc calib. Các ngưỡng cân bằng,
mapping motor và góc cân bằng cơ học không bị thay đổi bởi bản sửa này.
