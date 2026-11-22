# Bộ thu log rung LSM6DSV qua ESP-NOW

Mở `espnow_rung_receiver/espnow_rung_receiver.ino` bằng Arduino IDE. Thư mục
sketch đi kèm `Settings.h`, `ReceiverImpl.h` và hai header giao thức, có thể sao
chép nguyên thư mục sang máy khác. Cài gói **esp32 by Espressif Systems 3.3.9**,
chọn **ESP32 Dev Module** cho DevKit/WROOM-32 và nạp qua USB.
Với ESP32-S3 nối USB trực tiếp, chọn đúng bo ESP32-S3, bật **USB CDC On Boot**
và chọn **USB-OTG (TinyUSB)** nếu menu bo có mục **USB Mode**.

Trong `Settings.h`, sửa `RobotMac` thành MAC STA của ESP32-S3 trên xe nếu thay
bo. Giá trị mặc định là `AC:27:6E:D0:3A:24`. Cầu nối dùng kênh Wi-Fi 1 như xe,
USB Serial **921600 baud**. Gửi `status` trong Serial Monitor để xem MAC của bộ
thu, sau đó ghép MAC đó trên xe bằng `radio peer AA:BB:CC:DD:EE:FF` khi motor
tắt. Mỗi lần sửa header giao thức trong dự án, chạy `py sync_protocol.py` trước
khi đóng gói lại sketch.

Bộ thu tự đăng ký nhận log mỗi giây. Công tắc WebUI trên xe quyết định bật/tắt
luồng dữ liệu rung. Các lệnh Serial hỗ trợ là `status` và `headers`. Thông tin
trạng thái `RECEIVER_STATUS` giúp kiểm tra kết nối. `VR_HEADER`/`VR` là các mẫu
IMU ở nhịp nguồn. `VI_HEADER`/`VI` ghi cấu hình, trạng thái công tắc và bộ đếm.
Mỗi dòng dữ liệu có CRC-16/CCITT-FALSE ở cột `serial_crc16`.

Dữ liệu `forward_pct` và `turn_pct` theo thang −100…100, trước đường cong tạo
lệnh. Giá trị dương lần lượt là tiến và rẽ trái. `speed_pct` ghi hệ số tốc độ
trên WebUI. Bit `UiValid` trong `flags` xác nhận lệnh có thông tin vị trí cần
gạt, bit `CommandValid` xác nhận lệnh còn hiệu lực. Gyro `gx/gy/gz` ở rad/s đã
trừ bias, gia tốc `ax/ay/az` ở m/s² còn trọng lực. Các trục này thuộc hệ cảm
biến. Ma trận trong `VI` đổi sang hệ xe. Thời gian `t_us` là thời gian mẫu trên
xe, `bridge_rx_us` là thời gian nhận trên bộ thu.

Với firmware xe `lsm-smooth-c-20261122`, bật Log rung trong WebUI khi motor
còn tắt và đợi `sample_rows` cùng `wheel_rows` tăng rồi điều khiển xe.
Bộ thu tự yêu cầu kênh encoder khi nhận được mã firmware này trong cấu hình
`VI` và một mẫu mới báo motor tắt. Với firmware cũ, bộ thu vẫn ghi `VR` và
`VI`, đồng thời có thể ghi encoder nếu kênh chẩn đoán đã phát sẵn tới đúng MAC
bộ thu. Trên các bản cũ `wheel_rows` có thể bằng 0 trong khi log IMU vẫn chạy.
Bộ thu xuất `VW` vào `wheels.csv` cùng thời gian nguồn và số thứ tự mẫu IMU.
Nhịp mục tiêu của kênh bánh là 50 Hz. `steps_l/r` là xung STEP tích lũy có dấu,
`enc_l/r` là encoder MKS đã quy đổi về xung STEP. Kiểm tra bộ đếm UART tăng
và tuổi từng mẫu `uart_age_l/r_us` trước khi tính tốc độ. Tuổi 4294967295 µs
nghĩa là thiếu dấu thời gian. Trên các phiên khác nhau hãy tách dữ liệu theo
`boot_id,bridge_session,log_id`, rồi ghép qua `t_us,sample_seq`. `motor_us`
là thời gian chụp trạng thái bánh. Nếu thiếu log bánh, tệp IMU vẫn được ghi.

`queue_drop`/`tx_drop` đến từ xe. `receiver_queue_drop`, `receiver_packet_gap`,
`receiver_reject` và `serial_drop` đánh dấu chất lượng truyền/thu. Giữ các cột
này khi gửi CSV và kiểm tra nhịp thực từ timestamp trước khi so các lần chạy.

Để máy tính lưu CSV, dùng chương trình `tools/thu_rung_lsm6dsv.py` của dự án và đóng
Serial Monitor trước khi mở cùng cổng COM bằng chương trình đó. Thêm
`--native-usb` cho ESP32-S3 dùng USB trực tiếp và lấy số COM hiện tại sau khi
nạp. Nạp `.ino` mới này thay firmware cầu nối debug cũ trên DevKit.

Có thể build tự động bằng `pio run -d tools/espnow_rung_receiver -e esp32devkitv1`.

## Đối chiếu sau khi chạy

Từ thư mục gốc dự án, chạy `py tools/phan_tich_rung_banh.py outputs/THU_RUNG_01 --windows-csv outputs/THU_RUNG_01/windows.csv`. Chương trình in JSON tổng hợp và ghi chi tiết từng giây. Tham số mặc định là 12.800 xung mỗi vòng bánh và bán kính 0,040 m. Đổi bằng `--steps-per-wheel-rev` và `--radius-m` nếu xe dùng cấu hình khác.

Các nhóm phân theo phiên, cấu hình, PID, chiều đặt và dải tốc độ đặt 0,05 m/s. Chặng thẳng, tăng/giảm ga và rẽ phân theo lệnh người điều khiển. Chỉ số rung là RMS tốc độ nghiêng sau khi trừ trung bình từng giây. Xem thêm `encoder_l/r_mps` và `step_l/r_mps` để đối chiếu tốc độ thực tế tại trục với xung đã xuất. Sai số có dấu là encoder trừ STEP. `block_error_rms_l/r_mps` tính RMS sai số theo các khoảng từ 50 ms (thường 60 ms ở 50 Hz), có trọng số thời gian, để phát hiện sai lệch đổi dấu đã triệt tiêu trong trung bình một giây. Hai kênh này không đo mô-men.

Phép tính bỏ đoạn IMU thiếu mẫu và ghi nguyên nhân khi phản hồi bánh thiếu, quá tuổi 15 ms hoặc mất thứ tự. Hai chiều encoder và hệ số bước/vòng phải phù hợp với dây và cài đặt MKS. Khi còn nghi ngờ chiều hoặc MStep, kết quả cho biết hai phép đo chưa khớp và cần xác nhận trên xe.
