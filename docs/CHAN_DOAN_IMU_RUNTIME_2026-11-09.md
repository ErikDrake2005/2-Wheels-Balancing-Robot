# Chẩn đoán log IMU sau hiệu chuẩn

Ngày **09/11/2026**. Hồ sơ có hai bộ log khác nhau: log cũ 67 dòng được người dùng
đính chính, và log sau nạp có 71 dòng telemetry mang `imuAck-20261109`. Không trộn
kết luận của hai bộ dữ liệu. Source hiện được chỉnh tiếp thành **`imuAck-20261109-t5`**
để thử timeout I²C 5 ms và ghi số byte/thời gian giao dịch; chưa có bằng chứng
bản t5 đã hết lỗi trên xe.

## 1. Phân biệt hai bộ log

### 1.1 Log cũ 67 dòng

Tệp có 67 dòng không rỗng. Các trường chính lặp lại:

```text
EN=0 phase=0 epoch=0 last=3 fault=none ready=imu_invalid
theta=-0.0135 roll=-0.0539 accel=9.78 trust=0.000
miss=22 dup=0 overlap=1
```

| Trường | Cách hiểu đúng |
| --- | --- |
| `EN=0`, `phase=0` | Motor đang tắt, chưa vào điều khiển cân bằng |
| `last=3` | Enum `MksResult::Disabled`, không phải mã lỗi motor 3 |
| `fault=none` | Chưa có lỗi chốt trong trạng thái motor; chưa chứng minh IMU đủ điều kiện bật |
| `ready=imu_invalid`, `trust=0` | Trạng thái góc không hợp lệ; không được dùng các số góc đang in làm phép đo tư thế hiện tại |
| `theta=-0.0135`, `roll=-0.0539` | Tương ứng khoảng −0,77°/−3,09° nhưng có thể là giá trị lưu từ trước khi mất mẫu hợp lệ |
| `accel=9.78` | Gần g ở lần số liệu được lưu; không chứng minh cảm biến đang tiếp tục cập nhật |
| `miss=22` không tăng | Đây là số mẫu bỏ lỡ tích lũy, không phải số thứ tự ngắt; không đủ phân biệt mất IRQ với đọc I²C lỗi ở mọi IRQ |
| `serviceMax=536..596us` | Khoảng phục vụ motor quan sát được; không thay cho tình trạng luồng IMU |

Estimator xóa các cờ hợp lệ/trust ở đầu `update()`, nhưng giữ một số giá trị
góc cũ khi frame không đạt. Vì vậy dòng log vẫn có số góc hữu hạn trong khi
`ready=imu_invalid`. Không có cơ sở từ log này để chỉnh Kp/Kd hoặc gán lại góc
cân bằng nhằm vượt điều kiện arm.

### 1.2 Log `imuAck-20261109` sau nạp

Tệp tiếp theo có **76 dòng không rỗng**, gồm banner/hướng dẫn và **71 dòng
telemetry**. Dòng cuối ghi `irqSeq=1394`, `imuOk=802`, `imuI2c=431`,
`imuWait=59`, `imuLate=92`, `overlap=24`, `i2cLast=6`, `regLast=0x3A`.
Tuổi góc từng đạt **623 ms**; có các khoảng `ready=none`, `fallen` và
`imu_invalid`, không phải mọi mẫu đều đứng nguyên như log cũ.

Các số này chứng minh ngắt vẫn đến và một phần frame đã chốt được, nhưng đọc
runtime gián đoạn với nhiều lỗi I²C. Không tính tỷ lệ lỗi bằng cách lấy 431
chia IRQ: lần ACK phục hồi cũng có giao dịch, còn một số mẫu bị loại vì thời gian.
Mã 6 chỉ cho biết số byte trả về/đọc không khớp số yêu cầu, chưa chứng minh
backend cụ thể bị timeout, NACK hay nguyên nhân điện nào.

Người dùng xác nhận tư thế ngả ĐUÔI ở cuối là thao tác thật. Dòng cuối in
`thetaDeg=-86.33` nhưng `angleValid=0`, nên không dùng riêng số góc lưu đó để
đánh giá độ chính xác calib. Cần giải quyết chất lượng luồng dữ liệu trước.

## 2. Điểm thiếu tìm được trong source

Trước đợt sửa, `INT_PIN_CFG` đặt `INT_RD_CLEAR=0`: cờ ngắt được xóa khi đọc
`INT_STATUS`. UI polling có đọc thanh ghi đó, nhưng runtime chỉ đọc 14 byte
từ `ACCEL_XOUT_H` (0x3B), chưa ACK trạng thái ngắt theo cấu hình đã chọn.

Đây là thiếu nhất quán có bằng chứng trong code. Nó chưa chứng minh là nguyên
nhân duy nhất của log cũ: cần phân biệt GPIO/IRQ, I²C NACK/timeout, mẫu đọc trễ
và mẫu bị bỏ do ngắt chồng. Datasheet mô tả chế độ pulse/latch và cách clear
status; không kết luận mọi chế độ pulse sẽ ngừng xung chỉ vì chưa ACK.
[Datasheet ICM20602, mục INT_PIN_CFG và INT_STATUS](https://invensense.tdk.com/wp-content/uploads/2020/11/DS-000176-ICM-20602-v1.1.pdf)

## 3. Bản sửa runtime

- Cấu hình INT giữ mức tới khi ACK: `LATCH_INT_EN=1`, `INT_RD_CLEAR=0`, mức tác
  động theo config; active HIGH dùng 0x20, active LOW dùng 0xA0.
- `docMauTheoNgat()` đọc **15 byte từ 0x3A**: status + accel + nhiệt độ + gyro.
  Cùng giao dịch đọc status để ACK, chỉ giải mã frame khi có bit data-ready.
- Sau khi gắn ISR, đánh thức ImuTask một lần để xử lý cờ tồn đọng từ lúc calib.
  Lần đánh thức không có timestamp IRQ không được coi là mẫu góc hợp lệ.
- Nếu hết chờ IRQ 20 ms, ImuTask thử đọc/ACK để giải phóng INT sau lỗi bus.
  **Mẫu poll này không được xuất thành mẫu điều khiển mới.** Muốn dùng dữ liệu,
  phải có timestamp IRQ và điều kiện chốt mẫu hợp lệ.
- `chotMauImu()` loại mẫu khi không có ngắt nguồn, timestamp 0, ngắt đổi trong
  lúc đọc hoặc tuổi IRQ tới cuối burst **từ một chu kỳ mẫu trở lên** — 5 ms với
  cấu hình 200 Hz. Trong chế độ latch, thanh ghi vẫn có thể cập nhật dù chưa có
  cạnh IRQ mới; không chỉ dựa vào sequence không đổi để gán timestamp.
- Một mẫu overlap/đọc trễ được bỏ theo cơ chế giám sát tuổi dữ liệu hiện có.
  Queue chẩn đoán riêng vẫn xuất bộ đếm, tránh log đóng băng khi mọi mẫu bị bỏ.
- `ready` và `angleValid` ở console được đánh giá lại theo tuổi dữ liệu tại thời
  điểm in; không để một trạng thái cũ còn hiện sẵn sàng mãi.

Không nới ngưỡng Safety 20 ms, không tự bật motor, không tự chuyển polling thành
nguồn dữ liệu thay ISR. Bản `imuAck-20261109` dùng timeout I²C 2 ms. Sau log
thiếu byte thực tế, bản **t5** tăng riêng timeout giao dịch lên **5 ms**, giữ
**100 kHz**, guard chốt mẫu **dưới 5 ms** ở 200 Hz và Safety **20 ms**. Một giao
dịch chờ quá lâu vẫn bị loại khỏi vòng điều khiển, không gán timestamp cũ cho
mẫu mới hơn.

Bản t5 thêm `ChanDoanGiaoDichI2c`: thanh ghi, số byte yêu cầu, số byte nhận từ
`requestFrom()`, số byte thực sự lấy bằng `read()`, mã lỗi và thời gian giao dịch
microsecond. `giaoDichDocCuoi()` và `giaoDichDocLoiCuoi()` trả snapshot theo giá
trị; bản lỗi giữ lại sau lần đọc tốt. ImuTask sao chép chẩn đoán sang queue;
không bật Serial verbose trong ISR/ImuTask và không sửa SDK để in bus.

## 4. Nhận đúng firmware và màu LED

Sau nạp/reset, Serial phải có:

```text
FIRMWARE imuAck-20261109-t5; INT latched, runtime burst INT_STATUS+14 byte; I2C=100000Hz timeout=5ms
```

Phím `h` in lại phiên bản. Mỗi dòng telemetry có `fw=imuAck-20261109-t5` để nhận
diện ngay cả khi monitor mở sau boot. Một tên thư mục source hoặc một lần build
thành công chưa xác nhận board đã được nạp đúng tệp mới.

Quy trình calib ĐẦU/ĐUÔI giữ như hướng dẫn 08/11. Sau khi calib xong:

| Màu | Ý nghĩa runtime |
| --- | --- |
| Cam | Đang chờ bản trạng thái đầu tiên |
| Xanh lá | Trạng thái hiện tại qua `SafetyMonitor::check()` và chưa có fault chốt |
| Đỏ | Dữ liệu/trạng thái runtime không đạt hoặc có fault |

Xanh lá không tự bật motor, chưa thay toàn bộ điều kiện `canArm()` và cờ xác
nhận phần cứng. LED được cập nhật theo nhịp console 200 ms. Bản cũ chỉ để xanh
sau khởi tạo nên có thể vẫn xanh trong log `imu_invalid` đã gửi.

## 5. Các trường chẩn đoán mới

| Trường | Diễn giải |
| --- | --- |
| `fw` | Mã firmware đang in log |
| `frame` | Trạng thái frame trong snapshot Control: 0 không hợp lệ, 1 hợp lệ, 2 dữ liệu cũ, 3 lỗi I/O, 4 cần tinh chỉnh |
| `readStatus` | Trạng thái lần đọc mới nhất từ queue chẩn đoán riêng; cùng enum với frame |
| `angleValid` | Cờ góc và kiểm tuổi mẫu tại lúc in; không chỉ là số góc còn hữu hạn |
| `biasOk` | Cờ đã hiệu chuẩn trên frame Control đang có; frame lỗi bị xóa cờ không tự chứng minh bias lưu trong driver mất |
| `rawSeq` | Số ngắt gắn với frame được xuất cho Control |
| `irqSeq` | Số thứ tự IRQ mới nhất ImuTask quan sát, khác `miss` tích lũy |
| `irqAgeMs` | Tuổi IRQ gần nhất; −1 nếu chưa có mốc hợp lệ |
| `imuAgeMs` | Tuổi góc hợp lệ gần nhất ở state; −1 nếu chưa có |
| `okAgeMs` | Thời gian từ lần đọc được chấp nhận gần nhất; −1 nếu chưa có |
| `imuOk` | Số frame đã chốt hợp lệ, không tính poll phục hồi |
| `imuWait` | Số lần hết chờ IRQ 20 ms |
| `imuI2c` | Tổng lỗi đọc I²C runtime |
| `imuEmpty` | Có thông báo IRQ nhưng status không có data-ready |
| `overlap` | Số mẫu bị loại vì ISR sequence thay đổi trong lúc đọc, lấy từ queue chẩn đoán mới |
| `imuLate` | Số mẫu bị loại vì timestamp/đọc quá chu kỳ mẫu |
| `ackRecovery` | Số lần thử ACK khi hết chờ IRQ; không phải số lần phục hồi thành công |
| `i2cLast`, `regLast` | Mã lỗi và thanh ghi của lỗi I²C runtime gần nhất; giữ lại sau khi bus đã đọc tốt |
| `rx` | Byte `requestFrom()` trả về / byte yêu cầu của giao dịch đọc gần nhất; runtime mong `15/15` |
| `used` | Số byte thực sự lấy bằng `Wire.read()`; đủ burst runtime là 15 |
| `readUs`, `readMaxUs` | Thời gian toàn giao dịch đọc gần nhất / lớn nhất từ khi runtime chạy, µs |
| `rxErr`, `usedErr`, `errReadUs` | Byte nhận/yêu cầu, byte đã đọc và thời gian của giao dịch runtime lỗi gần nhất, giữ sau lần đọc tốt |

Các queue là snapshot cập nhật độc lập, nên `readStatus` mới nhất có thể khác
`frame` Control khi một mẫu đã bị bỏ. Dùng số tăng theo thời gian, tuổi mẫu và
trạng thái cùng nhau. `i2cLast` khác 0 không tự có nghĩa bus vẫn đang lỗi nếu
`imuI2c` đã đứng yên và `imuOk` tiếp tục tăng. Tương tự, `rxErr` là lần lỗi lưu
lại, khác `rx` mới nhất.

Ví dụ `rx=15/15 used=15` là nhận và lấy đủ byte, chưa tự chứng minh timestamp
hợp lệ. `rxErr=14/15 usedErr=0` nghĩa driver đã loại giao dịch thiếu ngay khi
`requestFrom()` trả về, chưa tiêu thụ buffer để giải mã. Nếu `rx=15/15` nhưng
`used` nhỏ hơn 15, số byte được báo nhận đủ nhưng một lần `read()` không lấy
được byte như mong đợi. `readUs` đo toàn giao dịch, không chỉ thời gian tín hiệu
điện trên dây; phép trừ microsecond dùng số unsigned để qua được vòng quay timer.
Bản lỗi được đưa vào thống kê runtime chỉ khi có frame I/O lỗi, không lấy lỗi
calib cũ làm lỗi runtime mới.

## 6. Phân biệt các nhánh khi thử trên xe

| Dấu hiệu qua nhiều dòng | Hướng kiểm tra |
| --- | --- |
| `irqSeq` và `imuOk` đều tăng, `ready=none`, `angleValid=1`, tuổi mẫu nhỏ | Luồng dữ liệu đang hoạt động; kiểm riêng dấu/độ chính xác góc và điều kiện arm |
| `irqSeq` đứng, `irqAgeMs` tăng, `imuWait` tăng, không tăng lỗi I²C | Nghi đường INT hoặc cấu hình/phát ngắt; kiểm GPIO7 và tình trạng data-ready, chưa kết luận ngay dây hỏng |
| `imuI2c` tăng | Xem `i2cLast/regLast` cùng `rxErr/usedErr/errReadUs`, nguồn, GND và bus; mã 2 NACK, 5 timeout, 6 số byte không khớp, không đồng nghĩa mã 5 |
| `irqSeq` tăng nhưng `imuOk` đứng, `overlap/imuLate` tăng | Kiểm thời gian đọc I²C, lịch task và tính đúng của tín hiệu INT |
| `imuEmpty` tăng liên tục | Có thông báo nhưng không thấy data-ready; kiểm nhiễu/ngắt khác hoặc thứ tự ACK |
| `ready=imu_stale`, tuổi mẫu tăng dù một số số góc không đổi | Góc in là giá trị cũ; không coi là phép đo hiện tại hoặc yêu cầu arm |

Trước khi thử bật motor, làm xong calib và giữ xe yên, xem 10–20 dòng log có
đúng tag mới và luồng mẫu tiến triển. Với 200 Hz và log 200 ms, `irqSeq` thường
tăng khoảng 40 mỗi dòng; không đòi đúng 40 tuyệt đối vì lịch in và mất dòng.
Giữ cờ phần cứng false nếu đang kiểm riêng cảm biến.

Nếu còn lỗi, lưu banner phiên bản, `ICM_BOOT`, `CALIB_DOC`, `CALIB_AXIS` và các
dòng runtime mới. Bản t5 cần log mới có đúng hậu tố `-t5` để biết thay timeout đã tác động thế nào;
log `imuAck-20261109` chưa có các trường byte/thời gian mới.

## 7. Phạm vi kiểm chứng

### 7.1 Kết quả lưu của bản trước t5

Build `lolin_s3_mini` PASS, RAM **57.052 byte**, flash **478.170 byte**. Sáu nhóm
native PASS: điều khiển 14.507; watchdog 19; lọc/di chuyển 169; góc gá/vector
3.114; calib ĐẦU/ĐUÔI 2.038; boot/UI/runtime 265 điều kiện.

Test native chạy driver thật với bus/đồng hồ giả, gồm burst ACK, thứ tự byte,
data-ready không bị dùng hai lần, chuỗi 200 Hz hơn một giây, kick/poll không
thành mẫu mới, ngắt chồng, timestamp quá chu kỳ, đọc thiếu byte, NACK/phục hồi
và sequence quay vòng. Các nhóm calib/estimator/controller trước đó cũng được chạy lại.

Chưa mô phỏng đầy đủ lịch FreeRTOS, độ trễ ISR/I²C thật hoặc đo GPIO7 trên board.
Đợt này chưa upload hay xác nhận hết lỗi trên xe. Kết quả cuối của build/test
được ghi trong `docs/du_lieu_tinh_toan/PHIEN_KIEM_THU_IMU_RUNTIME_2026-11-09.json`.

### 7.2 Bản t5 cần kiểm chứng riêng

Số liệu build/test ở mục 7.1 thuộc `imuAck-20261109` trước khi tăng timeout và
thêm chẩn đoán giao dịch. Không dùng chúng để tuyên bố t5 đã PASS hoặc đã hết
lỗi phần cứng. Bản t5 đang được kiểm thử phần mềm; cần log sau nạp trên xe để
đánh giá số byte, thời gian đọc và tốc độ tăng các bộ đếm. Giữ nguyên việc chưa
bật motor khi luồng IMU còn không hợp lệ.

Code liên quan: [Icm20602Driver.cpp](../lib/imu/src/Icm20602Driver.cpp),
[ChotMauImu.h](../src/core/ChotMauImu.h),
[RobotApp.cpp](../src/app/RobotApp.cpp),
[SensorFrame.h](../include/model/SensorFrame.h),
[hướng dẫn sử dụng](HUONG_DAN_SU_DUNG.md).
