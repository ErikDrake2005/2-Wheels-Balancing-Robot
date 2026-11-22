# 2-Wheels-Balacing-Robot

Xe MKS có ba cấu hình IMU: chọn `lolin_s3_mini` cho ICM20602 I²C, `lolin_s3_mini_lsm6dsv` cho LSM6DSV SPI, hoặc `lolin_s3_mini_mpu6050` cho MPU6050 I²C (kèm Biquad khử rung + SISO). Xem [Sơ đồ chân, cấu hình và hiệu chuẩn LSM6DSV](docs/KET_NOI_LSM6DSV_ESP32S3.md) và [Cấu hình MPU6050, lọc khử rung & SISO](docs/KET_NOI_MPU6050_VA_SISO.md).

Profile LSM có công tắc **Log rung ESP-NOW** trên WebUI. ESP thu dùng
[sketch Arduino](tools/espnow_rung_receiver/espnow_rung_receiver/espnow_rung_receiver.ino),
máy tính ghi CSV liên tục với `tools/thu_rung_lsm6dsv.py`.
[Cách nạp, ghép MAC và thu phiên thử](docs/THU_RUNG_LSM6DSV_ESP_NOW.md).

**Chuẩn bị chẩn đoán không dây:** firmware `radio-20261116` bổ sung telemetry
ESP-NOW hai chiều qua ESP32 DevKit/WROOM-32 cắm USB vào máy tính. Ghép MAC một
lần khi xe tắt motor; lệnh có boot/session/sequence và lease tối đa 150 ms.
STOP đưa lệnh về 0 và giữ cân bằng; DISARM tắt motor riêng. Chưa đo liên kết
thực tế 2–5 m và chưa kết luận hết rung khi chạy.
[Cầu nối và cách nạp](tools/espnow_bridge/README.md) ·
[Trình tự chẩn đoán tiến/lùi](docs/CHAN_DOAN_DI_CHUYEN_ESP_NOW.md).

**Tối ưu thời gian 16/11/2026:** đã nạp `timing-20261116`, clock MCPWM 5 MHz,
STEP tối đa 40 kHz, lịch UART cố định 2,5 ms và phục vụ phản hồi MKS 500 µs.
Đo motor tắt đạt khoảng 200 khung UART/s/bánh (trước 151–154), không có khe bỏ/lỗi TX
trong đoạn đo 9,5 s. Giữ PID 33/0/3.3 và vòng ngoài 0.15/0.06.
Đã kiểm tra tự cân bằng trên sàn 30 giây sau nạp: RMSE bám góc đặt 0,324°,
không mất mẫu/lỗi trong phiên này. Chưa đánh giá chạy/rẽ tốc độ cao.
[Tính toán và phạm vi kiểm chứng](docs/TOI_UU_THOI_GIAN_ESP32S3_2026-11-16.md).

**Đo PID thực nghiệm 15/11/2026:** đã chọn PID thuần `33/0/3.3`, vòng ngoài
`0.15/0.06` trong phạm vi khảo sát đứng tại chỗ và lưu NVS trên xe. Phiên CSV
độc lập 60 giây có 3.001 mẫu, 50 Hz, RMSE bám góc đặt 0,370°; không mất mẫu/lỗi
trong phiên này. Xe đang dùng MKS64/40 kHz, MPlyer Enable. Xem
[dữ liệu PID ngõ vào–ngõ ra](outputs/PID_IO_20261115/README.md) và
[cách thu lại](docs/THU_NGHIEM_CAN_BANG_VA_THU_DU_LIEU.md).

**Cấu hình vi bước 15/11/2026:** nhánh MKS dùng `MStep = 64` và trần STEP
20 kHz; nhánh TMC2209 giữ 32 vi bước và 10 kHz. Cả hai vẫn có trần quy đổi
khoảng 0,393 m/s. Phải đặt `MStep = 64` trên **cả hai driver MKS** cho khớp
firmware; firmware không tự đổi menu driver. Người dùng đã xác nhận cả hai
driver đặt 64. Đã nạp bản `mstep64-20261115` và đọc lại `MStep=64`,
`stepMaxHz=20000` qua Serial; khả năng giảm rung khi chạy vẫn cần đo trên mặt đất.
[Cách đặt driver và kiểm tra](docs/CAU_HINH_MSTEP_64.md).

**Chẩn đoán runtime 09/11/2026:** source hiện mang mã `imuAck-20261109-t5`, dùng ngắt
ICM giữ mức và đọc một burst `INT_STATUS + 14 byte` để ACK/nhận mẫu. Chốt mẫu
cần đọc xong trước chu kỳ kế tiếp (5 ms ở 200 Hz); chẩn đoán có queue riêng để
vẫn cập nhật khi mẫu góc bị loại. Sau calib,
LED cam chờ dữ liệu runtime, xanh khi dữ liệu/trạng thái an toàn còn hợp lệ,
đỏ khi có lỗi; xanh chưa bảo đảm đủ điều kiện arm. Telemetry có số thứ tự, tuổi
mẫu và bộ đếm I²C/timeout. Bản t5 tăng timeout giao dịch I²C từ 2 lên 5 ms, giữ
100 kHz và guard chốt mẫu dưới 5 ms/Safety 20 ms; thêm số byte nhận/đã đọc và
thời gian giao dịch để phân biệt lỗi đọc thiếu. [Cách kiểm tra log](docs/CHAN_DOAN_IMU_RUNTIME_2026-11-09.md).

Kết quả lưu của bản `imuAck-20261109` trước t5: build và sáu nhóm native PASS,
nhóm boot/UI/runtime 265 kiểm tra. Log xe sau đó có 431 lỗi I²C dù vẫn nhận IRQ
và chốt được một phần mẫu; chưa thể coi phép đọc thiếu byte là nguyên nhân gốc
đã xác định. Bản t5 cần kiểm thử và log xe riêng, chưa xác nhận hết lỗi phần cứng.


**UI calib 08/11/2026:** dán nhãn ĐẦU/ĐUÔI, hạ ĐẦU đặt nằm gần 90°, rồi chuyển
qua tư thế đứng sang hạ ĐUÔI đặt nằm gần 90°. Quy trình này thay thao tác nghiêng
trái/phải 30–45° trước đây. Dùng mặt đỡ cứng có hình học chuẩn để giảm sai lệch mốc.
[Hướng dẫn mới](docs/HUONG_DAN_UI_CALIB.md) ·
[Báo cáo hình học và kiểm chứng góc](docs/CALIB_DAU_DUOI_VA_KIEM_CHUNG_GOC_2026-11-08.md).

**Sửa calib nhạy rung:** tự thu lại dữ liệu ổn định và kiểm tra 32 mẫu ICM20602
sau mỗi lần reset. [Hướng dẫn đọc log và báo cáo kiểm thử](docs/SUA_LOI_CALIB_VA_KIEM_TRA_ICM_KHOI_DONG.md).

**Sơ đồ và UI hiệu chuẩn 05/11/2026:** ICM20602 dùng SDA GPIO10, SCL GPIO11,
SA0 GPIO8, CS GPIO9; nút xác nhận ở GPIO3. Motor 1 là bánh phải (DIR12/STEP13),
motor 2 là bánh trái (DIR5/STEP6), EN chung GPIO2.
Làm theo màu LED để đo bias gyro và bù góc gá trước khi bật cân bằng.
[Hướng dẫn UI_Calib](docs/HUONG_DAN_UI_CALIB.md) ·
[Đối chiếu sơ đồ và luồng hiệu chuẩn](docs/DOI_CHIEU_SO_DO_VA_FLOW_CALIB.md).
RGB GPIO48/GRB vẫn là mặc định cần đối chiếu board vì không có trên sơ đồ đã gửi.

**Cập nhật 05/11/2026:** [bù lệch gá ICM20602 bằng phép quay3D và vector nghiêng](docs/BAO_CAO_BU_LECH_LAP_IMU.md).
Góc gá có cấu hình Euler/ma trận, công cụ hiệu chuẩn hai vector và UI nghiêng ĐẦU–ĐUÔI.
Mặc định UI đo ma trận mới mỗi lần khởi động, chỉ giữ trong RAM. Nếu tắt UI,
`ImuMountConfig.h` hiện còn identity cho tới khi nhập số đo thực. UI tìm Y từ
chuyển động quanh trục bánh và X từ hai mốc nằm. Tư thế người giữ ở bước xanh
dương không tự trở thành pitch 0 hoặc góc cân bằng; hai mốc nằm phải có chuẩn cơ khí.
`gocCanBang_rad` là góc cân bằng cơ học cần xác định riêng.

ESP32-S3 + ICM20602 + hai MKS Servo42C, điều khiển STEP/DIR và ước lượng chuyển động
bánh từ số xung đã phát. Cập nhật tài liệu/rà soát ngày **03/11/2026**.

Motor người dùng: **17HS4401S**, trọng tâm **0,025 m từ trục bánh**. Runtime dùng
raw ICM20602, không DMP; lọc DLPF + fusion thích nghi + PT1 nhánh D trên ESP32.

- [Báo cáo thiết kế và tính lọc nhiễu](docs/BAO_CAO_ICM20602_LOC_NHIEU.md)
- [Báo cáo Servo42C, motor và di chuyển](docs/BAO_CAO_MKS42C_17HS4401S.md)
- [Nguồn mã nguồn mở và cách áp dụng](docs/NGUON_THAM_KHAO_LOC.md)
- [Số liệu tái lập từ config](docs/du_lieu_tinh_toan/KET_QUA_TINH_TOAN.md)

## Bắt đầu ở đây

1. [Hướng dẫn sử dụng](docs/HUONG_DAN_SU_DUNG.md): chân, cấu hình, build/nạp, Serial, đọc log và thử trên xe.
2. [Hướng dẫn lập trình](docs/HUONG_DAN_LAP_TRINH.md): đọc code từng bước, ý nghĩa biến và ví dụ tính toán.
3. [Checklist tính năng](docs/CHECKLIST_TINH_NANG.md): đã có mã, đã test PC, chưa làm và còn phải đo thực.

## Tra cứu sâu hơn

- [Logic toàn bộ chương trình](docs/LOGIC_CHUONG_TRINH.md)
- [Danh mục và công dụng toàn bộ hàm](docs/DANH_MUC_HAM.md)
- [Hướng dẫn sửa đổi và kiểm thử](docs/HUONG_DAN_SUA_VA_KIEM_THU.md)
- [Báo cáo rà soát 03/11/2026](docs/KET_QUA_RA_SOAT_2026-11-03.md)
- [Tổng quan firmware và cấu hình](docs/BALANCING.md)

Kết quả lưu của bản **08/11/2026** (trước sửa runtime 09/11): build và sáu nhóm native PASS. Nhóm calib pitch mới
đạt 2.038 kiểm tra (45 góc gá × 3 mức mốc 80°/85°/90° × 3 góc tham chiếu), nhóm boot/UI
đạt 44 kiểm tra. RAM 57.044 byte, flash 476.206 byte. Chưa upload/chạy trên xe thật.
Cửa sổ 80–92° là góc sau fit, không chứng minh góc thật; giới hạn sai mốc ±5° chỉ
áp dụng khi cả hai góc thật đều nằm trong 80–90°. Ki góc mặc định bằng0; R01 đã có
sửa và regression test, vẫn cần tinh chỉnh vật lý.
