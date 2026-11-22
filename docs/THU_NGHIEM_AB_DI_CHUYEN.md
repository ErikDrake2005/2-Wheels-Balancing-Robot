# So sánh A/B cho hiện tượng nghiêng thân nhưng bánh dịch chuyển ít

## Dữ liệu trước thay đổi

Các lần đã xác nhận thả tay bằng file tín hiệu cho thấy PID đứng tại chỗ hoạt
động, nhưng mức đặt0,05 m/s trong1 giây và0,10 m/s trong0,5 giây chưa tạo được
chuyển động duy trì. Encoder và xung phát đều có quãng đường thuần nhỏ. Đây
chưa phải bằng chứng động cơ mất bước, và chưa tái hiện được rung tăng khi chạy lâu.
Người vận hành xác nhận chỉ hai bánh chạm sàn, không tì thanh đỡ hoặc vướng dây;
quan sát thân nghiêng nhưng bánh quay ít.

Một lần lùi còn giữ thân tới khi bắt đầu xung đã bị loại khỏi so sánh. Các phiên
ngắt do lỗi USB/không arm cũng không dùng để suy đặc tính đối tượng. Log gốc và
ghi chú từng phiên nằm trong `outputs/RADIO_TEST_20261116`.

## Mục đích bản radio-ab-20261116

Giữ PID thuần33/0/3,3, vòng ngoài0,15/0,06, góc cân bằng đã kiểm chứng khoảng
0,1016°. Ba lựa chọn A/B khởi động ở trạng thái ban đầu để không âm thầm đổi
điều khiển. Chỉ đổi khi motor đã tắt bằng Terminal Serial trên WebUI/USB:

- `imucomp 0` / `imucomp 1`: tắt/bật việc lấy gia tốc lệnh chu kỳ trước để trừ
  khỏi accelerometer trước khi fusion. Lệnh khác gia tốc thật có thể làm góc
  ước lượng lệch; các lần đo hiện tại chưa chứng minh đây là nguyên nhân thực tế.
- `accelboost 0` / `accelboost 1`: ramp đặt cố định0,50 m/s² hoặc hành vi cũ
  tăng lên1,25 m/s² khi vận tốc đặt đã khác0. Không đổi gia tốc cứu cân bằng.
- `vgainboost 0` / `vgainboost 1`: hệ số bám vận tốc khi có lệnh chạy giữ1,0
  hoặc hành vi cũ2,2. Phép tắt này không phải bảo đảm cho đáp ứng tốt hơn.

Lựa chọn chỉ giữ trong RAM; `save` không lưu các cờ. Khởi động lại trở về ba
giá trị mặc định ban đầu. Đọc `status` xem `AB_RAM`; source áp dụng cùng version
cấu hình và gửi `diag_flags` trong từng mẫu ESP-NOW:

- bit0=imucomp, bit1=accelboost, bit2=vgainboost.
- 7=ba lựa chọn gốc;6=chỉ tắt bù gia tốc IMU;5=chỉ tắt accelboost;
  3=chỉ tắt vgainboost. Các tổ hợp khác phải được ghi rõ nếu đã thử.

Giao thức ứng dụng ESP-NOW tăng version lên2, gói telemetry224 byte, vẫn nằm
trong giới hạn250 byte của ESP-NOW v1. Cần nạp robot và cầu nối cùng version;
firmware hai version khác nhau sẽ không ghép phiên điều khiển.

## Quy trình so sánh

1. Đọc log khi motor tắt, xác nhậnPID/góc đặt và `diag_flags=7`, kiểm tra radio/USB.
2. Giữ cấu hình gốc, kiểm tra lại đứng cân bằng. Chỉ phát xung sau xác nhận người
  vận hành đã thả tay rồi chờ3 giây. Giữ cùng sàn, pin/tải và biên độ xung.
3. Tắt motor. Đổi duy nhất `imucomp 0`, kiểm tra `diag_flags=6` và thực hiện lại
  phép thử. Phục hồi bằng `imucomp 1` để xác minh tính lặp lại nếu kết quả khác.
4. Sau đó mới thử riêng accelboost hoặc vgainboost. Không đổi cả ba cùng lúc
  rồi kết luận lựa chọn nào gây cải thiện.
5. So sánh diện tích vref thực, quãng đường STEP/encoder, tốc độ encoder theo
  cửa sổ, góc/rate/RMS, thời điểm nhả lệnh và chất lượng dữ liệu. Vận tốc encoder
  tương đối với thân cần cộngR×thetaDot nếu dùng để ước lượng vận tốc mặt đất.

Dùng `--expect-diag-flags 6` trong công cụ thu khi thử imucomp0; công cụ không
arm nếu cờ chưa đúng, và dừng phép thử nếu cờ/cấu hình đổi giữa chừng. Cột `phase`
ghi giai đoạn host dự kiến; `vref` mới là giá trị đặt đã ramp trên robot.

## Giới hạn kết luận

Tắt bù IMU là phép thử nhận dạng, không mặc định là bản sửa cuối. Accelerometer
đo cả trọng lực và gia tốc chuyển động; bỏ hết bù có thể tốt lúc đứng yên nhưng
xấu khi tăng tốc thật. Giữ mọi ngưỡng bảo vệ và chỉ thử chuyển động ngắn có
người quan sát. Không cam kết không rung hoặc luôn ở trong1 m² từ các phép thử này.

Log với `wait_operator_release`/`settle_after_release` không được gộp thành dữ
liệu di chuyển không chạm tay. Bộ CSV PID nghiên cứu60 giây trước đó vẫn tách
riêng, không ghi đè bằng các lượt chỉnh định/A-B.
