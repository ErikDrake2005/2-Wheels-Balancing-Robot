# Bộ thu log rung LSM6DSV qua ESP-NOW

Giải nén toàn bộ gói. Mở espnow_rung_receiver/espnow_rung_receiver.ino bằng Arduino IDE.
Cài esp32 by Espressif Systems phiên bản 3.3.9. Chọn ESP32 Dev Module cho DevKit/WROOM-32.
Với ESP32-S3 nối USB trực tiếp, chọn đúng bo ESP32-S3, bật USB CDC On Boot và chọn USB-OTG (TinyUSB) nếu menu bo có mục USB Mode.
Settings.h chứa MAC STA của xe, mặc định AC:27:6E:D0:3A:24 và kênh Wi-Fi 1.
Giữ các header cùng thư mục sketch khi nạp. Bộ thu cần cắm USB vào máy tính.

Firmware xe cần hỗ trợ log rung LSM6DSV XVR1. Bản xe `lsm-smooth-c-20261122` hỗ trợ mở thêm kênh encoder từ bộ thu.
Sau khi nạp ESP thu, mở Serial Monitor 921600 baud, gửi status để đọc MAC ESP thu.
Khi motor xe tắt, gửi radio peer AA:BB:CC:DD:EE:FF trong Terminal WebUI xe với MAC ESP thu.
Đóng Serial Monitor trước khi mở chương trình thu CSV.

Cài pyserial một lần bằng py -m pip install pyserial.
Tại thư mục giải nén, chạy py thu_rung_lsm6dsv.py --port COM5 --out THU_RUNG_01.
Đổi COM5 theo cổng ESP thu đang hiện trên máy. Thêm --native-usb khi dùng ESP32-S3 nối USB trực tiếp. Thư mục kết quả phải là tên mới.
Mở WebUI trên điện thoại và bật Log rung ESP-NOW khi motor tắt. Với bản xe `lsm-smooth-c-20261122`, đợi cả sample_rows và wheel_rows tăng trước khi tự điều khiển.
Firmware xe cũ vẫn thu được samples.csv và config.csv. wheels.csv chỉ có dữ liệu nếu xe đã phát sẵn kênh chẩn đoán tới đúng MAC bộ thu. Có thể chạy thu IMU khi wheel_rows bằng 0.
Chu kỳ IMU phụ thuộc biến thể chip và cấu hình. Dùng t_us cùng sample_seq để đo nhịp thực và phát hiện mẫu bị thiếu.
Mỗi lần tắt/bật log có log_id riêng. Máy tính tiếp tục chờ khi công tắc tắt.
Kết thúc, tắt log, đợi 2 giây rồi nhấn Ctrl+C trên máy tính.

Gửi cả thư mục THU_RUNG_01, gồm samples.csv, config.csv, wheels.csv, serial.raw.log và metadata.json.
Trong wheels.csv, steps_l/r là xung đã phát, enc_l/r là giá trị encoder thực quy đổi về bước.
Kiểm tra uart_rx_l/r và uart_age_l/r_us để phát hiện số đo cũ. Tuổi 4294967295 nghĩa là thiếu dữ liệu.
Ghép wheels.csv với samples.csv theo boot_id, bridge_session, log_id và thời điểm t_us/số mẫu sample_seq.
Sau khi kết thúc thu, chạy `py phan_tich_rung_banh.py THU_RUNG_01 --windows-csv THU_RUNG_01/windows.csv` để phân nhóm tốc độ đặt, mức rung thân và sai số tốc độ rotor so với STEP.
Chỉ số `rate_rms_ac_radps` là RMS tốc độ nghiêng sau khi trừ trung bình mỗi giây. `block_error_rms_l_mps` và `block_error_rms_r_mps` chỉ sai số bám trên các khoảng ngắn từng bánh. Phép đo này không xác định trực tiếp mô-men.
Kèm ghi chú sàn, pin/tải, đoạn nào chạm xe và điều kiện khi rung tăng.

+forward_pct là tiến, -forward_pct là lùi. +turn_pct là trái, -turn_pct là phải.
Các giá trị phần trăm đo vị trí cần trước đường cong ga. v_request/yaw_request là lệnh thực vào điều khiển.
flags bit8 xác nhận thông tin UI mới. Gia tốc m/s² chứa trọng lực và gyro rad/s đã trừ bias, ở hệ cảm biến.
t_us là thời gian mẫu trên xe. host_utc là thời gian xử lý trên máy tính.

Kiểm tra bộ đếm mất mẫu trong từng phiên trước khi phân tích rung.
