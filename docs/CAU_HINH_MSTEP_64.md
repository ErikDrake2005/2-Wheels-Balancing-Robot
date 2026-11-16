# Cấu hình MKS 64 vi bước và trần STEP 20 kHz

Cập nhật ngày 15/11/2026. Cấu hình phần mềm nằm trong
[`MotorConfig.h`](../include/config/MotorConfig.h). Thiết lập thực tế trên
hai driver đã được người dùng xác nhận đặt `MStep = 64`; firmware không ghi
thay đổi `MStep` vào menu driver. Khả năng giảm dao động khi tiến/lùi kết hợp
rẽ vẫn cần kiểm chứng trên mặt đất.

## Thông số quy đổi

- Nhánh **MKS**: 200 bước đầy đủ/vòng × 64 vi bước = **12.800 xung STEP/vòng**;
  trần phát xung **20.000 STEP/s**.
- Nhánh **TMC2209**: giữ 32 vi bước, **6.400 xung STEP/vòng** và trần
  **10.000 STEP/s**.
- Với bán kính bánh 0,04 m và truyền động 1:1, cả hai cấu hình có trần vận tốc
  quy đổi `2 × π × 0,04 × 20.000 / 12.800 ≈ 0,392699 m/s`.

MKS64 giảm quãng đường ứng với một xung xuống khoảng **19,635 µm**.
Tăng đồng thời trần STEP lên 20 kHz giữ nguyên trần vận tốc quy đổi so với
MStep32/10 kHz; không tạo thêm dư địa tốc độ để cứu cân bằng. Trần quy đổi
không bảo đảm motor đạt đủ mô-men hay bám được lệnh trên xe thực.

## Đặt thủ công trên cả hai driver MKS

1. Tắt motor bằng lệnh `d`, xác nhận motor đã tắt và đỡ chắc thân xe trước khi
   thao tác menu. Không đổi vi bước khi xe đang cân bằng.
2. Trên **từng driver**, đối chiếu đúng model/phiên bản rồi đặt:
   `Mode = CR_vFOC`, `MStep = 64`, `En = L`, `Baud = 115200`.
   Giữ địa chỉ UART và chiều quay đang hoạt động đúng.
3. Lưu/cập nhật theo hướng dẫn của đúng phiên bản driver; đọc lại menu của cả
   hai bên để xác nhận `MStep = 64`. Không suy ra giá trị này chỉ từ firmware.
4. Nạp firmware có cấu hình MKS64 sau khi đối chiếu thiết lập. Nếu driver còn
   MStep32 nhưng firmware tính MStep64, số xung cho cùng một lệnh sẽ làm bánh
   quay nhanh gấp đôi dự kiến; hai bên đặt khác nhau còn gây lệch tốc độ bánh.
5. Khi thử lại có người giám sát, bắt đầu bằng mức di chuyển thấp. Kiểm tra
   chiều và mức quay của từng bánh trước khi thử tiến/lùi, rồi mới kết hợp rẽ.
   Quy trình này không tự gửi lệnh chạy hoặc arm xe.

## Phân biệt MStep và MPlyer

`MStep` là số vi bước của **xung STEP đầu vào**, quyết định số xung cho một
vòng bánh. `MPlyer` là nội suy nội bộ lên 256 vi bước của driver; không thay
giá trị 12.800 xung STEP/vòng dùng trong firmware khi đặt MStep64.

Trong lần so sánh đầu tiên, giữ nguyên trạng thái `MPlyer` đang dùng, kể cả
nếu đang bật. Nếu đang tắt và muốn thử bật, đối chiếu tài liệu đúng model và
phiên bản rồi đánh giá như một thay đổi riêng; chưa xác nhận trạng thái hay
khả năng của mục này trên phần cứng hiện tại. Không đổi đồng thời MPlyer và
hệ số PID khi đang đánh giá riêng ảnh hưởng của MStep64.

Các báo cáo cũ ghi MStep32/10 kHz mô tả cấu hình trước thay đổi này.

## Kiểm chứng ngày 15/11/2026

- Build `lolin_s3_mini` và `lolin_s3_mini_tmc_mt`: PASS.
- Toàn bộ native suite: 43.580 kiểm tra PASS. Chạy thêm control suite với
  `TMC_MT`: 15.720 kiểm tra PASS. Kiểm tra quy đổi một vòng, encoder và trần
  vận tốc vật lý cho cả hai cấu hình.
- Đã nạp bản `mstep64-20261115` qua COM9 (USB bootloader), công cụ nạp xác
  nhận hash dữ liệu flash. Khi chạy ứng dụng, cổng trở về COM11.
- Serial `status` đọc lại `MStep=64`, `stepMaxHz=20000`, PID thuần
  `30/0/3`, vòng ngoài `0,15/0,06`. Hai kênh UART encoder đều có phản hồi.
  Đây là thông số firmware; MStep trên menu driver do người dùng xác nhận.
- Sau nạp thu 50 dòng telemetry với motor tắt (`EN=0`):
  `fw=mstep64-20261115`, `fault=none`, `imuI2c=0`, `miss=0`.
  Lỗi ICM20602 `WHO=0x00` có trong log trước nạp không tái xuất hiện trong
  đoạn kiểm tra này. Log lưu tại `.cache/motion_debug/post_mstep64_status.txt`.

Chưa phát lệnh chạy motor để đo dạng xung 20 kHz hoặc đo dao động khi xe chạy
trên mặt đất. Các kiểm tra trên chưa chứng minh vấn đề rung đã được khắc phục.
