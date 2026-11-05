# Báo cáo rà soát chương trình — 03/11/2026

> Đây là snapshot trước đợt nâng cấp lọc/di chuyển cùng ngày. R01 sau đó đã sửa và test; nguồn/hàm/code hiện hành xem [báo cáo lọc](BAO_CAO_ICM20602_LOC_NHIEU.md) và [báo cáo motor](BAO_CAO_MKS42C_17HS4401S.md). SHA256 dưới đây thuộc bản trước.

## 1. Phạm vi và kết luận

Đã đọc các module của firmware chính: ứng dụng/task, IMU, ước lượng, kinematics,
PID mờ/PI, kiểm tra an toàn, phiên bật, lọc số mẫu, MCPWM/MKS/PCNT và giám sát EN.
Đối chiếu thêm cấu hình, native tests, các bench cũ và tài liệu hiện có.

Chưa tìm thấy lỗi điều khiển nghiêm trọng mới có bằng chứng chắc chắn ở cấu hình
mặc định. Bốn sửa đổi trước có mặt và được nối đúng trong mã: watchdog độc lập,
kiểm tra tư thế/norm, phiên bật xác nhận lệnh 0 và xử lý thông báo IMU trùng.
Đây là kết quả rà soát và kiểm thử phần mềm, không phải nghiệm thu xe tự cân bằng.

Đợt này cập nhật tài liệu; các vấn đề R01/R02 dưới đây **còn mở**, chưa sửa thuật toán
hoặc thêm chức năng vận hành. Hệ số điều khiển vẫn là giá trị thử, cờ phần cứng vẫn false.

## 2. Phiên bản và bằng chứng đã chạy

- Nền git: `bf4a2f6`, cộng các thay đổi chưa commit trong working tree. HEAD riêng không
  mô tả toàn bộ firmware đang kiểm tra.
- Môi trường: `lolin_s3_mini`, Arduino; profile 4 MB Flash, có `BOARD_HAS_PSRAM`
  và USB CDC lúc boot. Chưa xác nhận profile khớp module Super Mini vật lý.
- Nền tảng build: Espressif32/pioarduino 55.3.39; Arduino 3.3.9;
  thư viện IDF 5.5.4+sha.735507283d; Xtensa GCC 14.2.0+20260121.
- Native: GCC 15.2.0 trên máy hiện tại, qua script với C++17 và warnings-as-errors.
- `control_tests`: PASS, 14507 assertion.
- `watchdog_tests`: PASS, 19 assertion.
- Build: SUCCESS; RAM 56964/327680 byte; ứng dụng Flash 461478/1310720 byte.
- Kiểm tra ELF: `khiBaoThuc`, `ngatTrongKhoa`, `quaHanTrongKhoa` của GiamSatDongCo
  ở `.iram0.text`.
- Firmware SHA256:

```text
52439ad59b7e288f7b094a572236ba7bf5d5bedf7e624ecee6410bc67c95605f
```

Lệnh kiểm tra:

```powershell
./test/run_native.ps1 -TrinhBienDich C:\msys64\ucrt64\bin\g++.exe
pio run -e lolin_s3_mini
git diff --check
```

## 3. Các điểm còn mở

### R01 — P2, có điều kiện: xả tích phân khi chạm trần vận tốc

Vị trí: [AnglePidController.cpp](../lib/controllers/AnglePidController.cpp), nhánh
`if (!vanTocBiGioiHan && ...)`; [CascadeController.cpp](../lib/controllers/CascadeController.cpp)
chỉ truyền một bool bão hòa.

Khi vận tốc lệnh đã chạm trần, mọi cập nhật tích phân góc bị khóa, kể cả sai lệch đang có hướng làm giảm phần I. Nếu bật `kiGoc > 0`, phần I dương còn lại có thể giữ gia tốc dương và trì hoãn việc rời trần dù sai lệch đã âm.

Ví dụ số học với mờ tắt, Kp=30, Ki=20, tích phân=0,01 rad·s, sai lệch=-0,001 rad,
gyro=0: gia tốc là `30*(-0,001) + 20*0,01 = +0,17 m/s²`. Nếu đang ở trần dương,
điều kiện hiện tại ngăn phần I giảm. Đây là phân tích nhánh code; chưa chạy một
bản firmware đã chỉnh Ki=20 trên xe. Cấu hình hiện tại Ki=0 nên phần I không tác động đầu ra.

Đề xuất tiếp theo: truyền hướng bão hòa hoặc một tín hiệu cho phép xả tích phân,
chỉ chặn tích phân làm tăng bão hòa. Thêm regression test hai chiều với Ki khác 0,
bao gồm trường hợp đầu ra gia tốc và vận tốc cùng bão hòa.

### R02 — P2 chẩn đoán: có thể mất thông báo lỗi khởi động

Vị trí: [RobotApp.cpp](../src/app/RobotApp.cpp), `begin()` và nhánh đầu `loop()`.
Lỗi setup in một lần; khi `_daSanSang=false`, loop chỉ delay. Nếu USB console chưa
kết nối lúc lỗi được in, mở monitor muộn có thể chỉ thấy im lặng.

Cách sử dụng hiện tại: mở monitor trước, giữ xe yên rồi reset để xem log setup.
Đề xuất cải thiện: lưu bước/mã lỗi khởi động và in lại có giới hạn tần suất, hoặc
cho phép lệnh `h`/truy vấn trạng thái ngay cả khi setup thất bại.

### R03 — Giới hạn hiệu chuẩn cần ghi đúng

Vị trí: [Icm20602Driver.cpp](../lib/imu/src/Icm20602Driver.cpp), `hieuChinhConQuay()`.
Thuật toán kiểm tra phương sai gyro, không tự chứng minh thân xe đứng yên. Quay
đều có thể cho phương sai nhỏ và bị hấp thụ vào bias. Hướng dẫn phải yêu cầu giữ
xe yên; checklist ghi “thu bias, kiểm tra phương sai”, không ghi “tự xác nhận đứng yên”.

### R04 — Khoảng trống kiểm chứng I/O và tích hợp

Native tests không chạy driver Wire/ICM, MCPWM, PCNT hoặc toàn bộ RobotApp/FreeRTOS.
Các hàm quản lý phiên/thứ tự được test riêng, chưa chứng minh mọi thứ tự lập lịch
trên ESP32. Watchdog dùng mã production nhưng GPIO/GPTimer và critical-section là stub.
Cần test bus/peripheral hoặc đo trên board; không thay các phép đo bằng số assertion.

### R05 — Khả năng tái lập và cấu hình board

`platformio.ini` dùng URL phát hành `stable`; lần cài tương lai có thể nhận bộ công
cụ khác. Nên chốt phiên bản khi đã đo ổn định. Profile LOLIN S3 Mini cũng cần đối
chiếu flash/PSRAM của board thực tế trước khi coi file firmware là phù hợp để nạp.

## 4. Những điểm đã đối chiếu lại

- `maPhien` truyền từ MotorStatus sang ControlStatus rồi được MotorTask kiểm tra;
  zero-ack mới là điều kiện bật, không áp lại lệnh của phiên cũ.
- Watchdog khóa chung EN/deadline, giữ lỗi sau ISR; `giaHan()` kiểm tra hạn cũ trước
  khi cập nhật nên task quay lại muộn không hồi sinh EN.
- `service()` nhận lỗi watchdog; RobotApp gọi service trước khi xét yêu cầu bật.
- IMU có dedup số thứ tự và đối chiếu ngắt trước/sau giao dịch đọc.
- Safety xét cả góc ngang, norm gia tốc, tuổi mẫu; test tái hiện ±85° và mẫu toàn 0 đã đạt.
- Vị trí/vận tốc vẫn là ước lượng từ xung. Không có bằng chứng xác nhận bám xung,
  trượt bánh hoặc đáp ứng motor thật trong đợt này.

## 5. Thứ tự công việc tiếp theo

1. Đọc [hướng dẫn sử dụng](HUONG_DAN_SU_DUNG.md), xác nhận board/pin/mức điện và IMU khi EN tắt.
2. Đo STEP/DIR/PCNT/EN, thử bật lại và timeout với bánh kê; lưu log và dấu vết analyzer.
3. Chỉnh PD cố định trước, ghép PI vận tốc rồi so sánh luật mờ trong cùng điều kiện.
4. Hoàn tất R01 trước khi dựa vào Ki góc khác 0; cải thiện R02 để dễ chẩn đoán.
5. Đánh dấu nghiệm thu tại [checklist](CHECKLIST_TINH_NANG.md) bằng kết quả đo thực.
