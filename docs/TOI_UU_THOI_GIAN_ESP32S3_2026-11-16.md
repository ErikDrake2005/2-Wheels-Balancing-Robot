# Ngân sách thời gian ESP32-S3 và cấu hình thử ngày 16/11/2026

Mốc so sánh là firmware `research-20261115`, PID thuần 33/0/3,3, vòng ngoài
0,15/0,06, MKS Servo42C V1.1 MStep64/40 kHz và MPlyer Enable. Bộ CSV PID
60 giây ở `outputs/PID_IO_20261115` thuộc mốc này; không ghi đè bằng kết quả
của firmware tối ưu thời gian.

## Phần đã đạt giới hạn thiết bị hoặc đã có dữ liệu ổn định

- CPU của cấu hình SDK đang là 240 MHz. Tần số CPU không phải clock MCPWM.
  Chưa đo runtime từng task để khẳng định mức sử dụng CPU hoặc phần CPU còn dư.
- IMU và vòng góc 200 Hz, chu kỳ 5 ms. I2C đang 400 kHz. Các log trước có thời
  gian giao dịch gần đây khoảng 694–826 µs: 13,9–16,5% ngân sách thời gian 5 ms,
  không đồng nghĩa 13,9–16,5% CPU. Bản ghi PID 60 giây không có I2C/late/miss.
- Vòng vận tốc 50 Hz. MotorTask đặt mục tiêu phục vụ 250 µs. Thời gian thức
  thực tế có jitter, nên không coi 250 µs là độ trễ chấp hành đã đo.
- UART42C V1.1 đang 115200 bps, mức cao nhất trong danh sách lệnh baud 0x8A
  của Makerbase. Không thay baud lên mức của dòng Servo42D.
- Native USB đã ghi được 50 Hz không mất mẫu trong lần thử đứng cân bằng.
  Giá trị baud của Serial Monitor không phải giới hạn 115200 bps trên dây USB.
- Wi-Fi đã tắt sleep trong source. Không tăng ưu tiên Web/USB lên trên Motor,
  IMU và Control chỉ để giao diện phản hồi nhanh hơn.

## Tăng độ phân giải STEP

ESP32-S3 dùng PLL160 MHz cho MCPWM. ESP-IDF 5.5.4 mặc định chia nhóm 2,
cho clock nhóm 80 MHz. Chọn clock bộ đếm 5 MHz chia hết 80 MHz với prescale 16,
tránh trường hợp SDK tự làm tròn nhưng code vẫn tính theo giá trị yêu cầu.

- Tick mới 0,2 µs thay vì 1 µs.
- STEP 40 kHz: chu kỳ 125 tick thay vì 25 tick.
- Nấc ngay dưới 40 kHz: 39682,540 Hz thay vì 38461,538 Hz.
- Với R=0,04 m và 12800 STEP/vòng, khoảng nhảy vận tốc quy đổi ở đầu dải giảm
  từ 0,030208 xuống 0,006233 m/s, mịn hơn khoảng 4,85 lần.
- HIGH 10 µs cộng 1 tick bảo vệ: 51 tick. Ở 40 kHz còn 74 tick LOW = 14,8 µs,
  vượt LOW tối thiểu 10 µs đang cấu hình.
- Chu kỳ khởi tạo 10 ms = 50000 tick và chu kỳ liên tục thấp nhất 200 Hz = 25000
  tick, đều nhỏ hơn 65536. Chỉ đổi sang 10 MHz sẽ làm khởi tạo 10 ms vượt 16 bit.

Nhánh TMC_MT giữ clock 1 MHz. Không tăng trần STEP 40 kHz hay microstep; đổi clock
bộ đếm giúp giảm lượng tử tần số. Chưa đo dạng xung thật bằng oscilloscope;
không coi tính toán này là bằng chứng motor bám mọi lệnh dưới tải.

## Lịch hỏi UART

Mốc trước đo khoảng 151–154 khung hợp lệ mới nhất/s/bánh. Lịch cũ dời mốc theo
thời gian thực tế mỗi lần gửi; khoảng 2,5 ms bị làm tròn lên bởi nhịp đọc khoảng
1 ms và cộng dồn jitter. Đây là nguyên nhân trong phần mềm có thể cải thiện.

Mục tiêu mỗi bánh 200 lượt hỏi/s, luân phiên một yêu cầu mỗi 2,5 ms. Ở 115200/8N1:

- TX chung 400 × 3 × 10 = 12000 bit/s, khoảng 10,42% băng thông.
- RXriêng mỗi bánh 200×8×10=16000 bit/s, khoảng 13,89%; phản hồi 9 byte là 15,63%.
- Một hỏi 3 byte + đáp 8/9 byte cần khoảng 0,955/1,042 ms trên dây, chưa gồm thời
  gian xử lý MKS. Vì vậy tăng baud không phải ưu tiên đầu tiên.

Lịch mới bám mốc cố định, bỏ khe đã qua thay vì phát bù, giữ khoảng cách tối
thiểu giữa các lần enqueue và chỉ đổi bánh khi gửi đủ 3 byte. MKS đọc PCNT/UART
với mục tiêu 500 µs để có cơ hội phục vụ đều mốc 2,5 ms; TMC_MT giữ 1 ms.

`status` thêm `TIMING` và `MKS_UART_DIAG`: số lượt hỏi, tuổi phản hồi hợp lệ tại
snapshot motor, số khe bị bỏ và lỗi ghi TX. Timestamp TX là lúc enqueue; RX là
lúc MCU đọc/chấp nhận khung. Không coi đó là thời điểm lấy mẫu từ kế hoặc độ
trễ chính xác từng giao dịch vì giao thức không có ID truy vấn.

UART vẫn dùng giám sát, chưa hợp nhất vào PCNT. Thay đổi này không tự biến
STEP thành phép đo chuyển động bánh thực. Cần đồng bộ và đánh giá độ trễ trước
khi bật bù trượt từ UART vào vòng điều khiển.

## Xác minh

Kiểm thử native bao gồm lưới UART, jitter, lỗi TX, trễ dài không phát dồn, reset,
clock/timing STEP và các nhóm điều khiển/an toàn cũ đều PASS. Build cả MKS và
TMC_MT PASS; hai nhóm Python thu/xuất dữ liệu cũng PASS.

Đã nạp bản `timing-20261116` qua USB bootloader COM9 và xác nhận hash flash.
Ứng dụng trở về COM11. `status` đọc lại `cpuMHz=240`, `mcpwmHz=5000000`,
`imuHz=200`, `motorServiceUs=250`, `feedbackUs=500`, `uartWheelUs=5000`.
PID thuần 33/0/3,3 và vòng ngoài 0,15/0,06 vẫn đúng; motor tắt, không có lỗi.

Đo chẩn đoán với motor tắt trên khoảng 9,516944 s theo timestamp snapshot:

- Khung RX hợp lệ mới nhất: trái 200,064 Hz, phải 199,959 Hz.
- Request enqueue: trái 199,959 Hz, phải 200,064 Hz.
- Không có khe bị bỏ hoặc lỗi ghi TX trong khoảng đo.
- Tuổi phản hồi tại snapshot lớn nhất trong các lần hỏi status: 5423 µs.
  Đây là giá trị quan sát thưa theo status, không phải max của mọi giao dịch.
- So với các log cũ 151–154 Hz/bánh, nhịp phản hồi tăng khoảng 30%.

Log gốc ở `.cache/motion_debug/timing_uart_after_20261116.json`. Giá trị trên
đo được khi motor tắt; chưa chứng minh mất gói bằng 0 trên dây hoặc tốc độ lấy
mẫu nội bộ encoder. Không suy khả năng giữ cân bằng/chạy/rẽ từ đoạn này.

Kiểm tra trên sàn sau nạp, người vận hành xác nhận xe tự cân bằng và không
bị giữ/chạm thân trong cửa sổ đánh giá:

- PID thuần 33/0/3,3; vòng ngoài 0,15/0,06. Phiên yêu cầu 33 giây,
  tách 3 giây ổn định, còn 30,017667 giây và 1.502 mẫu để đánh giá.
- RMSE bám góc đặt 0,324348°; p95 |sai số| 0,541059°; max |sai số| 0,666753°.
- RMS tốc độ nghiêng 0,085350 rad/s, không mất mẫu/đọc IMU trễ/lỗi I2C hoặc
  bão hòa lệnh bánh; đã xác nhận motor tắt cuối phiên.
- So với lần xác nhận cũ 0,326757°, kết quả gần tương đương; chưa đủ bằng chứng
  quy mức thay đổi nhỏ này riêng cho tối ưu clock hay lịch UART.
- Hai snapshot status bao quanh lượt thử ghi nhận 3 khe UART bị bỏ, 0 lỗi TX,
  nhịp RX chấp nhận khoảng 199,96 Hz mỗi bánh trên khoảng 33,20 giây.
  Các khe trễ được bỏ thay vì phát dồn; không đồng nghĩa 3 gói bị mất trên dây.

Log ở `.cache/motion_debug/timing_pid_validation_20261116_01`. Bản CSV PID 60 giây
trước tối ưu vẫn nguyên vẹn. Chưa đo dạng xung tại chân hoặc thử chạy/rẽ ở tốc
độ cao, nên lợi ích giảm lượng tử ở 40 kHz vẫn là tính toán, không phải kết quả
đo rung ở vận tốc cao.

## Nguồn kiểm tra

- [Makerbase 42C V1.1, lệnh baud 0x8A](https://github.com/makerbase-mks/MKS-SERVO42C/wiki/Serial-communication-description)
- [ESP-IDF 5.5.4: bộ chia nhóm MCPWM](https://github.com/espressif/esp-idf/blob/v5.5.4/components/esp_driver_mcpwm/src/mcpwm_private.h)
- [ESP-IDF 5.5.4: tính prescale](https://github.com/espressif/esp-idf/blob/v5.5.4/components/esp_driver_mcpwm/src/mcpwm_com.c)
- [ESP-IDF 5.5.4: timer 16 bit](https://github.com/espressif/esp-idf/blob/v5.5.4/components/esp_driver_mcpwm/src/mcpwm_timer.c)
