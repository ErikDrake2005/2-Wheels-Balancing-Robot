# Thử cân bằng và thu dữ liệu ngõ vào–ngõ ra

Ngày chuẩn bị: 15/11/2026. Hai công việc thực hiện theo thứ tự riêng:

1. Thử các bộ thông số quanh cấu hình xe đang chạy được, chọn bộ tốt nhất trong
   các lần đo. Log bước này phục vụ chỉnh định, không phải bộ dữ liệu nghiệm thu.
2. Chốt cấu hình rồi mở phiên thu mới ở chế độ **PID thuần**, giữ cấu hình cố
   định để lấy CSV khảo sát tương quan ngõ vào–ngõ ra theo yêu cầu người dùng.
   Không ghép những đoạn đã đổi hệ số vào cùng một phép thử cố định.

## Bước 1: chỉnh định trên xe

- Điều kiện đo: hai bánh trên mặt đất, không giữ thân trong đoạn đánh giá, không
  gửi lệnh tiến/lùi/rẽ. Ghi mặt sàn, nguồn, tải và mọi lần người chạm xe.
- Đọc và lưu `status` trước khi sửa. Cấu hình trong NVS có thể khác mặc định
  trong source. Khởi đầu với bộ đã chạy tốt trên xe, không tự xóa NVS hoặc zero
  lại IMU để làm nhỏ sai số.
- Mỗi lần đo bỏ ít nhất 3 giây đầu rồi đo 20–30 giây. Lặp bộ gốc hai lần trước
  khi so sánh những thay đổi nhỏ. Chỉ đổi một hệ số mỗi lần, ban đầu khoảng
  10% quanh giá trị gốc; không tìm dao động tới hạn bằng cách tăng tới khi ngã.
- Đánh giá sai số góc với góc đặt thực của vòng trong, tốc độ nghiêng, trôi
  chuyển động và mức sử dụng lệnh bánh. Một RMSE góc nhỏ hơn nhưng bánh giật hơn
  chưa đủ để chọn bộ mới. Lặp lại bộ được chọn để so với độ biến thiên của bộ gốc.
- Bộ thông số được chọn chỉ được xác nhận cho điều kiện và phạm vi đã thử.
  Đo đứng yên không xác nhận tốc độ tối đa hoặc chống rung khi chạy kết hợp rẽ.

## Bước 2: dữ liệu cho Yêu cầu 2 và Yêu cầu 3

Đối chiếu dữ liệu khảo sát Yêu cầu 2 do người dùng cung cấp trước khi chọn biên
độ và kiểu kích thích. Mỗi phép thử có thư mục riêng, metadata riêng và một
điều kiện được xác nhận bởi người vận hành. Không tạo số liệu thực nghiệm giả.
Chỉ thu PID thuần cho bộ dữ liệu này. Nếu mode chuyển sang PID mờ hoặc cấu hình
thay đổi trong phiên thu, kết thúc và đánh dấu phiên chưa hợp lệ; không âm thầm
loại vài dòng để biến một phiên pha trộn thành phiên PID thuần.

Những đại lượng cần phân biệt trong CSV:

- `theta`: góc nghiêng thân do bộ ước lượng IMU tính, đơn vị rad.
- `target`: góc đặt cuối cùng của vòng góc, rad. Vòng ngoài có thể làm giá trị
  này thay đổi ngay cả khi góc cân bằng cơ học được giữ cố định.
- `eq`: góc cân bằng cơ học đã cấu hình, rad. Lệnh `refdeg` sửa đại lượng này;
  không đồng nghĩa tạo bước đặt cố định cho tổng góc vòng trong.
- `e`: `theta - target`, rad; đúng quy ước dấu của bộ điều khiển hiện tại.
- `rate`: tốc độ nghiêng từ gyro, rad/s.
- `cmdL`, `cmdR`: lệnh STEP/s có dấu của hai bánh, không phải tốc độ bánh đo thật.
- `a_cmd`: gia tốc chung đã lệnh sau giới hạn vận tốc, m/s²; không phải gia tốc
  thân đo độc lập. `v`, `vref`, `yawref` là vận tốc ước lượng và các giá trị đặt.
- Timestamp nguồn IMU, số thứ tự mẫu, phiên điều khiển, phiên bản cấu hình,
  hệ số hiệu dụng và trạng thái/lỗi được lưu để kiểm tra đồng bộ và chất lượng.

Ngõ vào chấp hành của đối tượng toàn xe có thể biểu diễn bằng hai lệnh bánh,
ngõ ra là góc nghiêng. Góc đặt là đầu vào của hệ kín. Những đại lượng này không
được đổi tên lẫn nhau khi đánh giá mô phỏng. Đồ thị tương quan từ dữ liệu vòng
kín có phản hồi và động học; không tự chứng minh quan hệ nhân quả hoặc mô hình
ngõ vào–ngõ ra tĩnh một–một. Chú ý độ trễ trước khi ghép cặp dữ liệu.

Tách dữ liệu dùng chỉnh định/nhận dạng khỏi dữ liệu dùng kiểm chứng mô phỏng.
Không báo thời gian xác lập hoặc vọt lố từ một đoạn đứng yên không có sự kiện
thay đổi giá trị đặt xác định. Chỉ ghi các chỉ tiêu đó khi đã thực hiện đúng
phép thử bám và có đủ dữ liệu trước/sau sự kiện.

Ki góc hiện có thể bằng 0: khi đó nhánh góc là PD được chỉnh định bằng mờ trong
cấu trúc PID. Nêu đúng bộ điều khiển đã thực hiện khi trình bày Yêu cầu 3.
CSV và log ở đây không thay phần trình bày tập mờ, miền giá trị, bảng luật và
phương pháp giải mờ trong báo cáo thiết kế.

## Điều kiện bắt đầu thử từ xa

Chỉ bắt đầu arm khi người vận hành đã xác nhận nguồn, xe trên mặt đất và sẵn
sàng theo dõi/đỡ xe. Phần mềm giữ các ngưỡng bảo vệ firmware; chương trình thu
có thêm giới hạn cho phiên thử tại chỗ và gửi disarm khi kết thúc hoặc bị lỗi.
Không tự chuyển từ một lần đã ngắt vì lỗi sang lần arm tiếp theo.

Khi chưa có xác nhận sẵn sàng, chỉ chuẩn bị/build/kiểm thử và thu motor tắt.
Những phiên motor tắt hoặc xe được giữ bằng tay phải được ghi đúng điều kiện,
không dùng để kết luận xe tự cân bằng tốt.
