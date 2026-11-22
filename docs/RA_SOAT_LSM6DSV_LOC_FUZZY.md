# Rà soát LSM6DSV, lọc nhiễu và Fuzzy

Ngày 17/11/2026. Phạm vi ưu tiên là profile `lolin_s3_mini_lsm6dsv`, SPI SCK8,
MOSI9, CS10, MISO11, INT1 GPIO7. Kiểm tra dựa trên source, bus giả lập và tính
toán. Dữ liệu chuyển động ICM trước đó dùng để nêu giả thuyết, không chứng minh
hiệu năng LSM6DSV.

Sau sửa, các kiểm thử native đã qua, gồm 12.629 điều kiện ở bộ filter/motion
cho LSM và đối chiếu cấu hình SPI với mã C của ST. Ba environment firmware
đều build thành công. Sửa nhịp vòng ngoài và các kiểm tra giá trị NVS/resetcfg
được chọn theo profile LSM6DSV. Chưa nạp lên xe hoặc đo với LSM thật.

## Lỗi tái hiện và sửa trong profile LSM6DSV

- Vòng ngoài đặt 20 ms bị mất phần dư mỗi lần cập nhật. Source cũ với nhịp
  IMU 4999 µs tạo 80 lần cập nhật trong 1,9996 s, khoảng 40 Hz. Đã dùng deadline
  microsecond cho LSM và giữ dt thực riêng cho tích phân. Kiểm thử 4999, 5000,
  5001 µs đếm số kỳ vòng ngoài và đối chiếu giá trị tích phân.
- `resetcfg` trước đây có thể xóa Flash và đổi hệ số khi motor đang chạy.
  Nhánh LSM yêu cầu motor ở trạng thái tắt trước khi xóa. Nhánh xử lý lỗi xóa
  trả thông báo trước khi thay cấu hình RAM.
- PID và góc tham chiếu nạp từ NVS có thể chứa số âm hoặc NaN/Inf từ dữ liệu
  ngoài giao diện hiện tại. LSM kiểm số hữu hạn, hệ số không âm và dùng mặc
  định khi giá trị lỗi. Góc tham chiếu lỗi trở về mặc định của LSM. Kiểm tra
  này tránh việc giá trị hiển thị khác hệ số thực được setter chấp nhận.

## Những điểm còn cần thử khi có module

- Bộ ước lượng giảm tin gia tốc theo độ lệch norm và innovation. Nếu độ tin
  cậy dưới 0,1 liên tục khoảng 0,5 s, nó reset và báo góc không hợp lệ. Gia tốc
  kéo dài hoặc bù gia tốc sai cũng có thể kích hoạt điều kiện này dù gyro còn
  dữ liệu mới. Đây là chính sách phát hiện mất tin cần đánh giá bằng mô phỏng
  và log, chưa thay ngưỡng trong lượt rà soát.
- Bù accelerometer lấy từ gia tốc bánh đã lệnh. Sai lệch bám motor, trượt bánh,
  gia tốc góc thân và vị trí IMU tạo sai số mô hình. Đổi bộ lọc vẫn cần xử lý
  đầu vào bù này. Khi quay còn có gia tốc hướng tâm.
- Hệ số tăng bám vận tốc 2,2 lần tắt theo lệnh thô khi nhả cần, trước khi ramp
  về 0. Với ví dụ Kpv=0,15, Kiv=0, v đo=0, lệnh 0,1 m/s, sau nhả 20 ms vref
  còn 0,075 m/s nhưng góc đặt chuyển từ 1,891° về 0,645°. Cần đo ảnh hưởng
  trước khi thay bằng chuyển hệ số liên tục hoặc ramp giới hạn jerk.
- SPI có kiểm tra ID/cấu hình/mẫu mới nhưng giao thức thanh ghi không có CRC
  để nhận ra mọi bit dữ liệu sai. Mock SPI kiểm logic, tín hiệu dây cần đo trên
  module thật. Nếu gyro/accel chưa đồng thời ready, driver có thể bỏ mẫu.

## Đánh giá bộ lọc đang dùng

LSM dùng gyro LPF1 mã 5 (`VERY_STRONG`), accel LPF2 ODR/4, fusion vector trọng
lực với trọng số gia tốc thích nghi và PT1 trên đường D danh nghĩa 40 Hz.
Accel LPF2 danh nghĩa 50 Hz ở ODR 200 Hz. Bảng gyro của ST ghi 53 Hz cho mã 5
ở ODR 240 Hz, không ghi trực tiếp điểm HAODR 200 Hz. Cần đo hoặc đối chiếu
thêm trước khi gán một cutoff chính xác cho toàn chuỗi gyro ở HAODR 200 Hz.

PT1 phần mềm dùng backward Euler. Ở fs=200 Hz và fc danh nghĩa 40 Hz, alpha
bằng 0,5568627 và cutoff rời rạc thực khoảng 27,472 Hz. Riêng tầng này suy giảm
50 Hz khoảng 5,86 dB, trễ pha 5 Hz khoảng 3,90 ms và trễ pha 10 Hz khoảng
3,70 ms. Bộ lọc trong chip cộng thêm độ trễ. Tau fusion 0,4 s tương ứng điểm
trộn gần 0,4 Hz khi gia tốc được tin hoàn toàn, không phải độ trễ cố định 0,4 s
của đầu ra góc vì đường gyro dự đoán góc theo từng mẫu.

Giữ bộ hiện tại làm mốc A/B. Giảm cutoff làm giảm nhiễu nhưng cần kiểm tra pha
và đáp ứng cứu cân bằng. Ở fs=200 Hz, phổ quan sát chỉ dưới Nyquist 100 Hz.
Log radio 50 Hz có Nyquist 25 Hz, cần kênh ghi ở nhịp IMU hoặc phổ/RMS tính tại
xe để khảo sát rung nhanh. Lọc chống alias cần xảy ra trước bước giảm mẫu.

## Các hướng tối ưu có điều kiện

- **Notch hẹp cộng LPF nhẹ** phù hợp khi phổ có đỉnh cộng hưởng rõ. Ví dụ lý
  thuyết giả định đỉnh 50 Hz, Q=5, PT1 danh nghĩa 60 Hz cộng notch có trễ pha
  5 Hz khoảng 3,12 ms. Ví dụ này chỉ minh họa cấu trúc, chưa chọn notch 50 Hz
  cho xe. Nếu rung nằm trong dải động học cân bằng thì cần xử lý điều khiển/cơ
  khí, tránh xóa tín hiệu chuyển động thực.
- **Butterworth bậc hai** giảm cao tần mạnh hơn. Ví dụ cutoff 40 Hz ở fs200
  suy giảm 80 Hz khoảng 25,09 dB, PT1 hiện tại khoảng 7,90 dB. Đổi lại trễ pha
  10 Hz tăng từ 3,70 ms lên 4,98 ms. Bậc lọc cao hơn cần đánh giá trong vòng kín.
- **Mahony hoặc x-io Fusion** đáng thử cho ước lượng tư thế, đặc biệt với xử lý
  bias và gia tốc bất thường. Cần khớp quy ước trục, dt, startup và phục hồi.
- **Kalman góc+bias** là lựa chọn nghiên cứu nếu cần mô hình nhiễu/covariance.
  Hai file `KalmanEstimator.*` trong dự án hiện trống. Bộ này cần triển khai và
  kiểm thử riêng. Gia tốc chuyển động kéo dài vẫn cần mô hình hoặc nguồn bổ trợ.

## Fuzzy hiện tại và các ứng dụng phù hợp

Fuzzy-PID đang lập lịch gain từ sai lệch góc và gyro. Kp được nhân 1–1,4 và
Kd nhân 1–1,5. Base hiện tại 30/0/3 cho miền Kp30–42, Kd3–4,5. Ki vẫn 0.
Vòng ngoài cũng dùng Fuzzy để đổi độ cứng vị trí/vận tốc. Chuyển PID/Fuzzy vì
vậy thay cả hai vòng. Input của bộ luật chưa có RMS rung hoặc phổ nhiễu.

Không có bằng chứng hiện tại rằng các luật này tối ưu cho LSM. Ưu tiên thử
Fuzzy ở tầng tạo lệnh tiến/rẽ, điều chỉnh giới hạn gia tốc theo RMS rung, sai
lệch vận tốc và góc. Sau đó có thể thử lập lịch gain vòng vận tốc theo tải/tốc
độ hoặc lập lịch độ tin gia tốc theo norm/innovation/RMS. Mỗi đầu ra cần giới
hạn biên độ và tốc độ thay đổi. So sánh với phương án ngưỡng cố định trên cùng
phiên thử và công bố cả lỗi góc, RMS gyro/lệnh motor, bám vận tốc, thời gian
ổn định, bão hòa và lỗi thời gian.

## Tài liệu đối chiếu

- [ST LSM6DSV DS13476 Rev5, phần bộ lọc và HAODR](https://www.st.com/resource/en/datasheet/lsm6dsv.pdf).
- [ST lsm6dsv-pid tại commit dùng đối chiếu](https://github.com/STMicroelectronics/lsm6dsv-pid/tree/ba9ea4ea331184416c0be47616e64a51200f0c23).
- [Analog Devices AN-1057 về gia tốc chuyển động và góc nghiêng](https://www.analog.com/en/resources/app-notes/an-1057.html).
- [ArduPilot về notch, lọc thấp và độ trễ pha](https://ardupilot.org/copter/docs/common-imu-notch-filtering.html).
- [x-io Fusion và xử lý acceleration rejection/recovery](https://github.com/xioTechnologies/Fusion).
- [Mahony, Hamel, Pflimlin 2008, kho ANU](https://openresearch-repository.anu.edu.au/items/db91f02d-9a17-4ead-bc64-86ed5bc9c6f4/full).
- [W3C Audio EQ Cookbook, công thức biquad dùng cho tính toán so sánh](https://www.w3.org/TR/audio-eq-cookbook/).
