# 2-Wheels-Balacing-Robot

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
