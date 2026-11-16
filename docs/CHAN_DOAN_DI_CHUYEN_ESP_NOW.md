# Chẩn đoán tiến/lùi và rung khi chạy qua ESP-NOW

## Phạm vi và trạng thái

Firmware robot `radio-20261116`, cầu nối ESP32 DevKit/WROOM-32 và công cụ
`tools/thu_log_espnow.py` chuẩn bị đường log hai chiều không cần dây USB trên xe.
Cần nạp robot qua USB một lần và lưu MAC cầu nối khi motor tắt. Hướng dẫn ở
`tools/espnow_bridge/README.md`. Chưa đo liên kết hai thiết bị hoặc khoảng cách
2–5 m cho đến khi có thiết bị cầu nối.

Không tự lái thử chỉ từ lệnh build. Người vận hành xác nhận xe trên sàn, vùng
thử có chỗ trống và có người sẵn sàng can thiệp trước từng lần arm. ESP-NOW không
cung cấp tọa độ tuyệt đối; STEP/encoder có thể sai khi bánh trượt nên không bảo
đảm một geofence 1 m² chỉ bằng dữ liệu hiện có.

## Những nguyên nhân cần phân biệt

Kết quả PID đứng tại chỗ không xác nhận chế độ chạy. Source hiện còn gia tốc
lệnh tăng 2,5 lần khi vận tốc đặt khác 0 và hệ số bám vận tốc đổi 2,2 ↔ 1 ở
biên nhả cần. Các chuyển tiếp này có thể kích thích thân xe khi đảo chiều.
Tham chiếu vị trí đang chạy còn giữ sai lệch có giới hạn khi đổi tiến/lùi.
Đây là các cơ chế có trong mã; chưa có log di chuyển xác nhận nguyên nhân thật.

Để giải thích tiến chậm hơn lùi, cần đo cùng biên độ lệnh ở cả hai hướng và so:

- Góc nghiêng, góc đặt, tốc độ gyro và độ tin cậy fusion; góc cân bằng cơ học
  đặt sai hoặc trọng tâm lệch có thể làm dư địa hai hướng khác nhau.
- STEP lệnh và vị trí/độ biến thiên encoder thật của từng bánh, tuổi mẫu UART,
  lệch bám trước/sau đảo chiều. Phản hồi điều khiển MKS hiện vẫn là STEP PCNT,
  UART chỉ giám sát; không xem PCNT là bằng chứng bánh đã thực hiện lệnh.
- Vận tốc đặt, mức gia tốc lệnh, yaw đặt và thời gian kể từ khi bắt đầu chạy.
  Không sửa đồng thời PID, filter, MPlyer và mô hình dự báo trong lần so sánh đầu.
- Điện áp pin dưới tải, nhiệt độ motor/driver, mặt sàn và tải cơ khí. Những đại
  lượng chưa có cảm biến không được điền bằng số giả trong CSV.

## Trình tự thử khi phần cứng đã sẵn sàng

1. Kiểm tra log khi motor tắt; đo tỷ lệ gói, tuổi nguồn và phản hồi trạng thái.
   Thử STOP/mất heartbeat trên giá đỡ có người giữ trước khi di chuyển trên sàn.
2. Thu cân bằng tại chỗ 10–20 giây bằng PID thuần 33/0/3,3, vòng ngoài0,15/0,06.
3. Mỗi lượt chỉ tạo một xung +0,05 m/s trong0,30 giây, sau đó STOP và đợi ổn
   định. Quãng đường đặt lý tưởng0,015 m không bảo đảm quãng đường thật. Thu
   ít nhất ba lượt riêng, ghi rõ mọi lần người chạm xe.
4. Làm tương tự -0,05 m/s, cùng pin/tải/mặt sàn. So sánh độ trễ và encoder thay
   vì chỉ nhìn vận tốc PCNT. Chỉ ghép chuỗi tiến–dừng–lùi khi cả hai hướng ổn.
5. Thử rẽ riêng sau đó. Người quan sát dừng ngay khi xe gần biên vùng thử;
   không kéo dài tới ngã để tạo dữ liệu học máy.

Lệnh MOVE chỉ sống tối đa150 ms từ lúc nhận trên robot, cần lệnh mới để giữ
chuyển động. Heartbeat không gia hạn vận tốc cũ. Mất liên lạc đưa lệnh về0 và
khóa chuyển động, giữ bộ cân bằng chạy. DISARM là lệnh riêng cần người đỡ xe.

## Máy học hoặc nơ-ron

Bộ CSV đứng yên ngày15/11 không đại diện rung tăng khi chạy lâu; chưa thể dùng
nó để khẳng định đã huấn luyện bộ dự báo đúng cho hiện tượng mới. Trước tiên
ghi nhiều lượt tiến/lùi/đổi chiều, gồm thời gian nguồn, góc/rate, lệnh bánh,
encoder, yaw, sai số bám và sự kiện có người chạm xe.

Bước phân tích đầu là RMS/p95 của sai số góc và tốc độ nghiêng theo cửa sổ
0,5–1 giây, năng lượng dao động quanh dải rung đo được và xu hướng biên độ.
Đánh giá một mô hình đơn giản như dự báo tự hồi quy trước, rồi so với MLP/GRU
nhỏ nếu có đủ dữ liệu. Mục tiêu có thể là RMS rung trong0,5 giây tiếp theo;
phân biệt rõ dự báo liên tục với nhãn cảnh báo.

Tách train/validation/test **theo lượt chạy hoặc phiên pin/tải**, không trộn
ngẫu nhiên các cửa sổ chồng lấn của cùng lượt. Báo tỷ lệ báo giả, bỏ sót và thời
gian cảnh báo sớm trên các lượt chưa dùng để chỉnh mô hình. So với baseline
RMS/xu hướng đơn giản trước khi cho rằng mạng nơ-ron tốt hơn.

Ban đầu mô hình chỉ chạy offline hoặc cảnh báo, không thay PID 200 Hz và không
tự tăng Kp/Kd khi dự báo rung. Mô hình dự báo sai có thể làm vòng kín mất ổn định.
Độ mượt khi chạy cần được xác nhận sau từng thay đổi điều khiển có thể giải thích.
