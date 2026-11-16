# Cầu nối ESP-NOW cho xe hai bánh cân bằng

ESP32 DevKit/WROOM-32 nối USB với máy tính; ESP32-S3 trên xe hoạt động bằng pin.
Hai thiết bị cùng kênh Wi-Fi 1. Firmware mới trên xe là `radio-20261116`.
Gói telemetry 223 byte ở nhịp mục tiêu 50 Hz; callback Wi-Fi chỉ chép vào queue.

## Nạp và ghép cặp lần đầu

1. Nạp cầu nối bằng PlatformIO trong thư mục này, environment `esp32devkitv1`.
   Serial Monitor dùng **921600 baud**. `status` in MAC cầu nối.
2. Cắm USB vào xe một lần để nạp firmware robot mới. Khi motor tắt, gửi:
   `radio peer AA:BB:CC:DD:EE:FF` với MAC của cầu nối. Xe lưu peer trong NVS.
   `radio status` kiểm tra giá trị. Không cần đổi địa chỉ MKS hoặc PID.
3. Trên cầu nối gửi `connect AC:27:6E:D0:3A:24`. Đây là MAC STA của xe đã ghi
   nhận trong phiên trước; phải đọc lại nếu thay ESP32 trên xe.
4. Đầu tiên chỉ `monitor`, xác nhận boot ID, timestamp nguồn, trạng thái motor
   tắt và dữ liệu góc/encoder còn mới. Chưa đo được tầm hoạt động 2–5 m trên
   phần cứng cho đến khi có cả hai thiết bị.

`connect` tạo session ngẫu nhiên mới; robot chỉ nhận đúng MAC đã ghép, đúng
boot ID và thứ tự gói mới. Ghép mới khi robot đang tắt motor. CRC kiểm lỗi,
không phải cơ chế mã hóa/chống giả mạo. Giữ hệ thống thử trong môi trường kiểm soát.

## Điều khiển

- `hb`: máy tính gửi đều khoảng 25 lần/s khi đang thử, để cầu nối biết máy tính
  còn xử lý. Heartbeat không kéo dài lệnh chuyển động khác 0 đã hết hạn.
- `arm`: thao tác rõ ràng của người vận hành; đợi ACK áp dụng và `en=1`, phase2.
  Chỉ cho PID thuần trong công cụ thử ban đầu. Không tự arm khi khởi động lại.
- `move 0.05 0 100`: tiến 0,05 m/s với lease 100 ms. Muốn giữ chuyển động phải
  có lệnh mới; không tự phát lại từ hàng đợi. Giao thức giới hạn tối đa 0,10 m/s,
  0,30 rad/s và lease150 ms; công cụ thử ban đầu chỉ dùng 0,05 m/s, không rẽ.
- `stop`: đưa lệnh tiến/rẽ về0, khóa di chuyển và giữ bộ cân bằng chạy. Cần
  `arm` mới để cấp lại quyền. Không hiểu STOP là đã tắt điện motor.
- `disarm`: tắt motor rõ ràng; người vận hành phải sẵn sàng đỡ xe.

Mất heartbeat máy tính250 ms hoặc telemetry200 ms làm cầu nối gửi STOP và khóa.
Robot cũng kiểm hạn150 ms từ **thời điểm nhận** trên xe; chờ xử lý không tạo lease
mới. Các trường `received_seq`, `ack_seq`, `en`, `vref`, `yawref` được lưu riêng:
ACK nhận gói không chứng minh bánh đã thực hiện đúng tốc độ.
CSV cầu nối có `rssi_dbm` đo lúc nhận gói; giá trị -128 nghĩa là chưa có số đo.
Lỗi ACK tầng MAC được ghi vào bộ đếm, không đồng nghĩa gói ứng dụng đã bị mất.
Mỗi dòng `BN` có cột cuối `serial_crc16` (CRC-16/CCITT-FALSE trên phần trước
dấu phẩy cuối, không gồm newline). Công cụ loại dòng USB thiếu/sai byte, ghi
`usb_bad_frames`, và ngắt về STOP nếu không còn mẫu hợp lệ mới trong200 ms.
Lệnh do công cụ gửi dùng khung `@noi_dung#CCCC`; CRC được kiểm trước khi thực thi.
Lệnh ASCII thông thường vẫn dùng được trong Serial Monitor khi thao tác thủ công.

## Thu log từ máy tính

Tại thư mục gốc dự án:

```powershell
py tools/thu_log_espnow.py --port COMX --out outputs/radio_monitor_01 --seconds 20
```

Mặc định chỉ quan sát và gửi heartbeat, không arm.
Chế độ `--arm` chờ tối đa 20 giây để người vận hành dựng xe: sai lệch góc dưới
3°, tốc độ nghiêng dưới 0,2 rad/s và ít nhất 0,5 giây mẫu mới ổn định. Nếu chưa
đạt thì không gửi ARM. Thời lượng đo bắt đầu sau xác nhận ACK/EN và hai phiên
điều khiển–motor khớp nhau. Đây là kiểm tra tư thế, không tự xác nhận bánh chạm sàn.
Với cầu nối S3 dùng USB native, thêm `--native-usb`; DevKit/WROOM giữ mặc định
để không kéo DTR vào mạch boot của USB-UART.

Thử chuyển động có người quan sát, sau khi đã kiểm tra mất sóng/STOP ở chế độ
nhấc bánh:

```powershell
py tools/thu_log_espnow.py --port COMX --out outputs/radio_forward_01 --seconds 12 --arm --pulse-v 0.05 --pulse-seconds 0.3 --disarm-at-end
```

Lệnh trên có thể bật motor. Chỉ chạy sau xác nhận người vận hành đã dựng xe.
Nó chờ cân bằng3 giây, tạo một xung tiến0,3 giây, rồi STOP. Tùy chọn cuối tắt
motor khi kết thúc bình thường. Nếu phép thử bị ngắt do lỗi/ngưỡng giám sát,
chương trình gửi STOP và giữ cân bằng, không tự DISARM sớm; cần xem trạng thái
thực và có người đỡ xe trước khi tắt motor. Nếu thiếu tiêu đề USB, công cụ yêu
cầu `monitor` lại và chưa arm; các byte dư trước tiêu đề không được dùng làm mẫu.
Bỏ `--disarm-at-end` thì kết thúc
bình thường cũng dùng STOP/giữ cân bằng.
Để tách thao tác đỡ xe khỏi dữ liệu xung, dùng `--release-gate <file_chua_ton_tai>`:
xe giữ cân bằng chờ tối đa 60 giây, chỉ khi người vận hành xác nhận đã thả tay
và file tín hiệu được tạo thì mới bắt đầu 3 giây ổn định rồi phát xung. Nếu hết
thời gian chờ, chỉ STOP/giữ cân bằng; không tự tắt motor sớm.
Các ngưỡng thử gồm nghiêng5°, tốc độ nghiêng1 rad/s, vận tốc0,15 m/s, quãng
đường STEP ước lượng0,10 m và dữ liệu nguồn mới. Chưa dùng công cụ cho chuỗi
tiến/lùi/rẽ dài khi chưa xác minh trên phần cứng.

ESP-NOW không đo vị trí tuyệt đối. Encoder/STEP có thể sai khi trượt; các giới
hạn trên không bảo đảm xe nằm trong hình vuông1 m². Cần người quan sát và khu
vực trống như đã thống nhất. Không dùng log đứng yên để kết luận hết rung khi chạy.
