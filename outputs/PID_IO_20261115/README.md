# Dữ liệu ngõ vào–ngõ ra của xe cân bằng chạy PID thuần

Đo thực nghiệm ngày 15/11/2026, Asia/Saigon. Người vận hành xác nhận hai bánh
trên mặt đất, xe tự cân bằng liên tục và không bị giữ/chạm thân trong 60 giây đo.

## File dùng cho mô phỏng

- `data/pid_u_y.csv`: 10 cột số, 3.001 mẫu; phù hợp nhập Scilab/MATLAB.
- `data/data_pid_io.csv`: dữ liệu đầy đủ, gồm thời gian nguồn, trạng thái,
  chế độ PID, thông số đã khóa và bộ đếm chất lượng.
- `data/metadata.json`: định nghĩa đại lượng, SHA-256 và nguồn dữ liệu gốc.
- `raw/`: log Serial nguyên bản, toàn bộ mẫu từ trước arm đến sau disarm,
  sự kiện và metadata phiên thu. Ba giây ổn định đầu đã được tách khỏi CSV mô phỏng.

## Ý nghĩa đại lượng

- `t_s`: thời gian mẫu IMU, đặt mốc 0 ở mẫu đo đầu tiên; chưa nội suy hoặc lấy
  mẫu lại. Nhịp danh nghĩa 50 Hz; dùng timestamp thực để xử lý độ trễ.
- `u_left_steps_per_s`, `u_right_steps_per_s`: lệnh tần số STEP có dấu.
- `u_common_steps_per_s`: trung bình hai lệnh bánh, dùng cho chuyển động dọc.
- `a_cmd_mps2`: gia tốc dọc đã lệnh sau giới hạn vận tốc; không phải gia tốc
  motor hoặc thân xe đo độc lập.
- `y_pitch_rad`, `y_pitch_deg`: góc nghiêng thân từ bộ ước lượng IMU.
- `target_pitch_rad`: góc đặt cuối cùng của vòng góc, có bù từ vòng vận tốc/vị trí.
- `e_rad`: `y_pitch_rad - target_pitch_rad`; `pitch_rate_rad_per_s`: tốc độ nghiêng.

Lệnh u trong một hàng được tính từ mẫu IMU y của chu kỳ tương ứng rồi gửi tới
motor; không mặc định u tác động lên y ngay trong cùng hàng. Phải xét động học
và độ trễ khi nhận dạng. Đây là dữ liệu **vòng kín quanh điểm đứng cân bằng**,
không phải phép khảo sát hở hoặc đặc tuyến tĩnh trên toàn miền hoạt động.

Đối tượng ở đây là góc nghiêng thân xe. Mô hình tham khảo RPM → góc quay bánh
trong thư mục YC2_KHAO_SAT_MO_HINH của người dùng là đối tượng khác; không dùng
lại gain, tau hoặc số đo của mô hình đó cho bộ dữ liệu này.

## Điều kiện và kết quả

- PID thuần: Kp=33, Ki=0, Kd=3,3. Vòng ngoài: Kpv=0,15, Kiv=0,06.
- Góc cân bằng cơ học đã lưu: khoảng 0,1016°; không hiệu chuẩn/zero lại giữa các lượt.
- Firmware đang chạy: `research-20261115`; IMU/control 200 Hz, ghi dữ liệu 50 Hz.
- MKS Servo42C V1.1, MStep64, trần STEP40 kHz, UART115200. MPlyer Enable cả hai
  driver theo xác nhận đã sửa của người vận hành.
- Vận tốc/x của nhánh MKS vẫn dựa trên STEP PCNT; UART từ kế dùng giám sát,
  chưa hợp nhất vào vòng điều khiển. Điện áp nguồn và mặt sàn chưa được đo định lượng.
- Thời gian đo thực 59,998155 s; 3.001 mẫu; nhịp trung bình 50,00154 Hz.
- RMSE bám góc đặt 0,36980°; p95 |sai số| 0,58383°; max |sai số| 0,71702°.
- RMS tốc độ nghiêng 0,09738 rad/s. Không mất mẫu, lỗi I²C, đọc IMU trễ hoặc
  bão hòa lệnh trong phiên đo. Đã xác nhận motor tắt cuối phiên.

Bộ thông số được chọn từ các lần thử PID riêng trước đó; các lần chỉnh thông số
không được ghép vào CSV này. Ki=0 nghĩa là nhánh góc đang hoạt động như PD trong
cấu trúc PID; vòng ngoài có giữ vị trí. Kết quả chưa xác nhận khả năng chạy/rẽ,
tốc độ tối đa hoặc đáp ứng bước đặt; không suy thời gian xác lập/vọt lố từ đoạn
giữ cân bằng không có bước đặt. Không coi đây là bảo đảm tối ưu trên mọi tải.
