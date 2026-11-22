# Thu log LSM6DSV trong lúc tự điều khiển xe

Firmware xe dùng profile `lolin_s3_mini_lsm6dsv`. Bộ thu dùng ESP32 DevKit /
WROOM-32 nối USB máy tính. Bạn điều khiển xe bằng WebUI trên điện thoại và
bật/tắt **Log rung ESP-NOW** tại đó. Máy tính lưu liên tục đến khi nhấn Ctrl+C,
kể cả khoảng chờ khi chưa bật log hoặc lúc xe vừa khởi động lại.

## Chuẩn bị firmware

Gói nạp Arduino và chương trình máy tính có sẵn tại
[`bo_thu_log_lsm6dsv.zip`](../outputs/LSM6DSV_LOGGER/bo_thu_log_lsm6dsv.zip).
Sau giải nén, xem `HUONG_DAN.md`. Có thể chạy file Python trong gói trực tiếp.

1. Build và nạp firmware xe có tính năng log bằng environment
   `lolin_s3_mini_lsm6dsv`. Hoàn tất hiệu chuẩn hướng lắp LSM6DSV theo
   [hướng dẫn kết nối](KET_NOI_LSM6DSV_ESP32S3.md).
2. Mở thư mục sketch
   `tools/espnow_rung_receiver/espnow_rung_receiver/espnow_rung_receiver.ino`
   bằng Arduino IDE. Cài **esp32 by Espressif Systems 3.3.9**, chọn
   **ESP32 Dev Module**, rồi nạp cho DevKit/WROOM-32. Giữ các header và
   `Settings.h` cùng thư mục `.ino`. Giá trị `RobotMac` trong `Settings.h`
   phải là MAC STA của xe. [Hướng dẫn bộ thu](../tools/espnow_rung_receiver/README.md).
3. Xem MAC DevKit bằng lệnh `status` trên Serial Monitor ở 921600 baud.
   Khi motor tắt, gửi `radio peer AA:BB:CC:DD:EE:FF` trong Terminal WebUI xe,
   dùng MAC vừa đọc. Bộ thu và xe dùng cùng kênh Wi-Fi 1.
4. Đóng Serial Monitor để chương trình thu CSV mở được cổng USB.

## Thu một phiên liên tục

Cài Python và pyserial nếu máy chưa có, sau đó chạy tại thư mục gốc dự án.

```powershell
py -m pip install pyserial
py tools/thu_rung_lsm6dsv.py --port COM5 --out outputs/THU_RUNG_LSM_01
```

Đổi `COM5` theo cổng DevKit trong Device Manager. Tên thư mục `--out` phải
mới để tránh ghi đè phiên cũ. Có thể thêm `--seconds 600` để tự kết thúc sau
10 phút. Mặc định chương trình chạy đến Ctrl+C.

Trên điện thoại, vào WebUI xe và bật **Log rung ESP-NOW**. Theo dõi trạng thái
bộ thu trên giao diện rồi bắt đầu điều khiển. Có thể tắt/bật log nhiều lần
trong khi chương trình máy tính vẫn mở. Mỗi lượt bật có `log_id` riêng.
Chương trình ghi cả lúc đứng yên và khi chạy trong lượt log đang bật.

Kết thúc lượt thử, tắt log trên WebUI rồi đợi khoảng 2 giây để nhận trạng thái
cuối phiên. Nhấn Ctrl+C tại máy tính để đóng CSV và hoàn tất metadata. Nếu USB
bị rút, chương trình thử mở lại cùng cổng mỗi giây. Nếu Windows đổi số COM,
dừng chương trình và tạo thư mục thu mới với số COM mới.

## Dữ liệu lưu trong thư mục phiên

- `samples.csv` chứa thời gian mẫu trên xe, vị trí điều khiển −100…100, lệnh
  thực đưa vào bộ điều khiển, gia tốc/gyro ba trục, nhiệt độ, góc và đầu ra motor.
- `config.csv` chứa ID phiên, trạng thái bật/tắt, PID, bộ lọc, dải đo, tần số,
  ma trận lắp, bias gyro, ngưỡng tin cậy và bộ đếm lỗi.
- `serial.raw.log` giữ toàn bộ byte USB nhận được, bao gồm dòng sai CRC và
  thông báo khởi động. Dòng dữ liệu sai CRC bị loại khỏi CSV đã giải mã.
- `metadata.json` ghi số mẫu, lần mất USB, mẫu thiếu/trùng/sai thứ tự và từng
  nhóm `boot_id + bridge_session + log_id`. Metadata được cập nhật trong lúc
  thu và đánh dấu hoàn tất khi kết thúc bình thường hoặc Ctrl+C.

Nén và gửi **cả thư mục phiên** để phân tích. Kèm mô tả ngắn về mức pin/tải,
mặt sàn, chiều chạy rung nhiều hơn, thời điểm chạm/đỡ xe và các thay đổi cơ khí
hoặc thông số giữa các lượt. Ghi lại sự kiện bằng mốc giây tương đối nếu có thể.

## Ý nghĩa các cột chính

`t_us` là đồng hồ microsecond trên xe tính từ khởi động, dùng để tính khoảng
mẫu và phổ rung. `bridge_rx_us` là thời điểm nhận ở DevKit và `host_utc` là
thời gian UTC lúc máy tính xử lý dòng. Ba đồng hồ có gốc riêng.

`forward_pct` dương là tiến, âm là lùi. `turn_pct` dương là trái, âm là phải.
Đây là mức điều khiển trên WebUI trước đường cong và hệ số tốc độ.
`speed_pct` ghi mức thanh tốc độ 0…100. Đọc cờ `UiValid` để xác nhận hai mức
phần trăm là dữ liệu từ WebUI. `v_request` và `yaw_request` ghi lệnh vật lý
được bộ điều khiển sử dụng. `vref` và `yawref` ghi giá trị sau ramp. Khi lệnh
đã hết hạn, `CommandValid` bằng 0 và lệnh thực có thể đã về 0 dù vị trí nhận
cuối của cần gạt còn lưu trong log.

Khi log bật và trang đang mở, WebUI gửi cả vị trí trung tính để ghi được các
đoạn đứng yên. Nếu trang ẩn hoặc mất kết nối quá hạn lệnh, `UiValid` tắt.
Khi đó `speed_pct=0` biểu thị thiếu dữ liệu WebUI mới. Máy tính vẫn thu log
cảm biến nếu kết nối ESP-NOW còn hoạt động.

`flags` là bitmask. Bit 1 nghĩa motor bật, bit 2 nghĩa góc hợp lệ, bit 4 nghĩa
lệnh còn hiệu lực và bit 8 nghĩa có thông tin vị trí WebUI. `mode=0` là PID
thuần, `mode=1` là PID mờ. `source` lần lượt 0/1/2/3 cho nguồn none/web/other/remote.

`ax/ay/az` có đơn vị m/s² và bao gồm trọng lực. `gx/gy/gz` có đơn vị rad/s,
đã trừ bias gyro. Các cột này ở **hệ trục cảm biến**, qua bộ lọc phần cứng
đang cấu hình. `mount00`…`mount22` trong config đổi vectơ từ hệ cảm biến sang
hệ xe. `theta`, `target` ở radian, `rate` và `gyro_d` ở rad/s. `cmd_l/cmd_r`
ở STEP/s, `a_cmd` ở m/s², `v` ở m/s và `temp_c` ở °C.

`sample_hz` là tần số cấu hình, mặc định 200 Hz. Nhịp thực cần tính từ `t_us`.
Tần số Nyquist của log 200 Hz là 100 Hz. Dữ liệu đã qua bộ lọc LSM6DSV hiện
tại nên việc đánh giá rung cao tần còn phụ thuộc cấu hình đó.

## Cách thử để có dữ liệu so sánh

Giữ cùng PID, pin/tải và bộ lọc trong một lượt so sánh. Có thể ghi theo trình
tự đứng yên, tiến đều, về đứng yên, lùi cùng mức, sau đó thử rẽ ở cùng mức
tiến/lùi. Bắt đầu bằng các mức nhỏ và tăng theo điều kiện thực tế của xe.
Ghi rõ các đoạn chạm thân hoặc đỡ xe. Khi cần so sánh thay đổi thông số,
dùng lượt log mới và ghi tên cấu hình trong ghi chú.

Khi phân tích sẽ so sánh RMS gia tốc/gyro theo cửa sổ, phổ rung, bám góc và
lệnh motor ở các đoạn có cùng mức điều khiển. Để đánh giá rung tăng theo thời
gian cần so sánh các cửa sổ cùng điều kiện, tránh gộp mọi loại chuyển động.
Mẫu thiếu trong một nhóm có thể đến từ IMU, task điều khiển, queue, radio hoặc
USB. Bộ đếm lỗi và log gốc giúp phân biệt các nguồn này.
