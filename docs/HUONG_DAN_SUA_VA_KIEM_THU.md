# Hướng dẫn sửa chương trình và kiểm tra kết quả

> Cập nhật lọc và di chuyển 03/11/2026: [ICM20602](BAO_CAO_ICM20602_LOC_NHIEU.md), [MKS42C/17HS4401S](BAO_CAO_MKS42C_17HS4401S.md). Runtime không DMP; PT1 chỉ nhánh D; có lệnh di chuyển có hạn; R01 đã sửa.

Cập nhật rà soát ngày 08/11/2026. Các điểm chưa hoàn tất được theo dõi trong
[báo cáo](KET_QUA_RA_SOAT_2026-11-03.md) và [checklist](CHECKLIST_TINH_NANG.md).

Tài liệu này dùng cùng [logic tổng thể](LOGIC_CHUONG_TRINH.md) và
[danh mục hàm](DANH_MUC_HAM.md). Mục tiêu là giúp bạn xác định đúng module, hiểu dữ
liệu đi vào/đi ra, rồi kiểm tra thay đổi bằng kết quả đo được.

## 1. Cách tiếp cận mỗi lần sửa

1. Ghi hành vi mong muốn: ví dụ “nghiêng ngang phải bị từ chối bật”.
2. Tìm nơi quyết định hành vi trong bảng ở phần 2.
3. Xác định biến đầu vào, đơn vị và cờ hợp lệ. Viết một trường hợp bình thường,
   một trường hợp biên, một trường hợp lỗi.
4. Sửa một ý chính, dùng biến tiếng Việt không dấu và hậu tố đơn vị.
5. Chạy native test và build firmware; xem log/đo phần cứng cho phần phụ thuộc chip.
6. Ghi lại thông số và kết quả trước/sau. Cập nhật docs nếu đổi giao diện/luồng.

Ví dụ tên dễ đọc:

```cpp
float gocNghiengNgang_rad = 0.0f;
uint64_t thoiDiemPhucVuCuoi_us = 0;
uint32_t maPhien = 0;
bool duLieuHopLe = false;
```

Tên biến diễn tả đại lượng; tên hàm mới diễn tả hành động như `yeuCauBat()`, `tat()`,
`chapNhan()`. Giữ API thư viện nguyên tên. Số giây và microsecond phải đổi đơn vị
rõ ràng (`us * 1e-6f`); không truyền độ vào hàm đang dùng radian.

## 2. Muốn thay đổi gì thì sửa ở đâu?

| Bạn muốn sửa | File/điểm vào | Cần kiểm tra lại |
| --- | --- | --- |
| Chân I²C/INT/SA0/CS/STEP/DIR/EN/nút/RGB | `include/config/PinConfig.h` | Không trùng chân; phù hợp board/flash/USB; mức logic thực tế. Sơ đồ: SDA10/SCL11, SA0=8/CS=9, motor 1 phải12/13, motor 2 trái5/6 |
| Địa chỉ I²C hoặc tần số bus | `ImuConfig.h` và `Icm20602Driver::begin()` | ID, đọc đủ 14 byte, timeout, pull-up/chất lượng bus |
| Dải gia tốc/gyro | `ImuConfig.h`, driver begin/read/calibration | Hiện driver chỉ nhận ±4 g/±500 dps. Đổi cả thanh ghi, hệ số chuyển đổi và hệ số trong hiệu chuẩn |
| Hướng lắp cảm biến | `UI_Calib`, `HieuChuanNghiengDoc`; nếu tắt UI: `ImuMountConfig.h`, `HeTrucImu` | Góc dọc dương khi nghiêng trước, gyro cùng dấu; giữ pitch ban đầu khác0 và nhận đúng nghiêng ngang |
| Nút, LED, thời gian hoặc ngưỡng calib | `CalibConfig.h`, `UI_Calib.cpp` | Nhấn–nhả, chống dội35ms, giữ nút qua bước, dữ liệu rung/timeout, lỗi và thử lại; RGB48/GRB cần xác minh board |
| Góc cân bằng cơ học | `ControlConfig.h::gocCanBang_rad` | Xác định riêng trên xe; không lấy tư thế người đỡ lúc calib làm giá trị này |
| Độ lọc góc | `hangSoThoiGianLocGoc_s`, cấu hình DLPF | Nhiễu, trễ, phản ứng khi bánh tăng tốc, độ trôi |
| Ngưỡng tư thế khi bật/ngã | `SafetyConfig.h`, `SafetyMonitor` | Hai chiều dọc/ngang, số không hợp lệ và biên ngưỡng |
| Điều kiện nhận mẫu IMU | `ComplementaryEstimator::update()` | Trước/sau init, toàn 0, NaN, norm bất thường, hồi phục sau mất mẫu |
| Trùng/mất thông báo ngắt | `BoLocThuTuMau`, `RobotApp::imuLoop()` | Số thứ tự trùng, nhảy, quay vòng, ngắt mới trong lúc I²C |
| Bán kính/vi bước/tỉ số truyền | `RobotGeometry.h`, `MotorConfig.h` | Số xung/vòng, quãng đường/vận tốc, khớp menu MKS |
| Chiều chạy mỗi bánh | `mucDirTienBanhTraiCao`, `mucDirTienBanhPhaiCao` | Cả bánh thực và dấu PCNT phải cùng quy ước tiến |
| Hệ số PID/PI | `ControlConfig.h` | Đơn vị đầu ra gia tốc; so sánh đáp ứng, bão hòa và thời gian ổn định |
| Luật mờ | Ba bảng trong `AnglePidController.cpp` | 5×5, đối xứng cần thiết, miền chuẩn hóa, hệ số không âm/hữu hạn |
| Giới hạn tốc độ/gia tốc | `ControlConfig.h`, `MotorConfig.h` | Công suất thực và khả năng bám; test thoát bão hòa |
| Trình tự bật/tắt | `PhienCanBang`, `RobotApp::motorLoop/controlLoop()` | Lệnh cũ, tắt-bật nhanh, mất xác nhận, đổi mờ, lỗi giữa lúc bật |
| Hạn giám sát motor | `MotorConfig.h`, `GiamSatDongCo` | Thời gian phục vụ thực và phép đo EN khi task ngừng |
| Log/chữ Serial | `RobotApp::loop()`/`faultName()` | Không in trong ISR/đường điều khiển nhanh; tránh quá tải console |
| Đổi board PlatformIO | `platformio.ini` | Flash/PSRAM/chân thực tế, tài nguyên MCPWM/PCNT/GPTimer và build lại |

## 3. Ví dụ đọc một hàm để tự sửa

Với `SafetyMonitor::canArm()`:

```text
Đầu vào: TrangThaiXe + thời điểm hiện tại
Điều kiện 1: check() không báo lỗi
Điều kiện 2: gia tốc đủ tin cậy
Điều kiện 3: góc dọc/ngang trong miền bật
Điều kiện 4: gyro dọc/ngang nhỏ
Đầu ra: true/false, chưa trực tiếp bật EN
```

Muốn cho phép một góc khởi đầu lớn hơn, điểm thay đổi là ngưỡng config. Tuy nhiên,
trước khi tăng ngưỡng hãy kiểm tra xe có đủ khả năng đỡ thân tại góc đó không. Không
thể suy ra khả năng bắt cân bằng chỉ từ một biểu thức điều kiện.

Khi thêm điều kiện mới, bổ sung dữ liệu vào `TrangThaiXe`, gán nó ở estimator, kiểm
tra tại SafetyMonitor, thêm test và cập nhật telemetry nếu cần quan sát. Việc chỉ
thêm biến nhưng quên gán sẽ khiến giá trị mặc định có thể chặn bật hoặc gây hiểu sai.

## 4. Những ràng buộc phải giữ khi mở rộng

- ISR IMU chỉ ghi timestamp/số thứ tự và notification; I²C chạy trong ImuTask.
- Chỉ MotorTask gọi API thay đổi MksServo42CPair sau setup.
- GiamSatDongCo là nơi bật EN; ISR giám sát chỉ tắt EN. Không viết trực tiếp EN để
  “bỏ qua lỗi”, vì sẽ phá quan hệ giữa chân EN và cờ lỗi/deadline.
- Lệnh bật mới phải có phiên mới, reset controller và xác nhận tốc độ 0.
- `thoiDiemRaLenh_us` không được gán lại để làm một lệnh cũ trông như lệnh mới.
- Tắt/xóa lỗi không tự bật lại. Sau khi xóa lỗi vẫn cần `a` riêng.
- Đổi DIR phải giữ pause/hold/setup; không đổi DIR khi STEP đang phát cạnh lên.
- Không thêm Serial, cấp phát heap hoặc chờ I/O vào ISR giám sát.
- UI chạy trong setup trước các task cân bằng. Nếu chuyển sang hiệu chuẩn khi đang
  chạy, phải thiết kế lại quyền sở hữu I²C và trạng thái motor; hiện chưa hỗ trợ.
- Khi sửa dựng ma trận UI, giữ Y từ trục quay quanh hai bánh, X từ hiệu hai mốc
  ĐẦU/ĐUÔI đã chiếu vuông góc Y, Z=X×Y. Test phải kiểm pitch ban đầu khác0 khi
  hai mốc chuẩn; không ép Z trùng gravity tư thế người giữ. Cần có ca bất đối xứng
  80°/90° để thể hiện sai số mốc không quan sát được chỉ bằng hai mốc fit.
- `service()` cần chạy đều để phát xung thưa, đổi chiều và gia hạn heartbeat.
- Mã phiên và số thứ tự dùng kiểu unsigned; so sánh quá hạn/timestamp theo đúng
  ý nghĩa từng biến. Đếm xung bánh có dấu dùng `int64_t`.

## 5. Chạy kiểm thử trên PC và build

Từ thư mục dự án, PowerShell:

```powershell
./test/run_native.ps1 -TrinhBienDich C:\msys64\ucrt64\bin\g++.exe
pio run -e lolin_s3_mini
```

Script chạy sáu nhóm C++: điều khiển/an toàn/phiên, watchdog, lọc/di chuyển,
góc gá/estimator vector, calib pitch ĐẦU/ĐUÔI và boot/I²C/thu lại calib khi rung.
`calib_pitch_tests.cpp` kiểm ma trận gá xéo, chiều quay trước → sau, hai mốc gần
±90°, pitch tham chiếu khác0 và các trường hợp không đạt. Đợt 08/11 đã PASS sáu
nhóm native và build: calib pitch 2.038 kiểm tra, boot/UI 44 kiểm tra; RAM 57.044 byte,
flash 476.206 byte. Chưa upload/chạy xe thật. Bản ghi 05/11 trước đó thuộc UI roll cũ.
Các test số học không
thay cho thử nút/RGB, Wire, timing và chuyển động thật. Sau khi đổi UI, làm lại
[quy trình calib](HUONG_DAN_UI_CALIB.md) trên board với EN tắt, ghi log trục/ma trận,
hai mốc, tổng góc gyro và CALIB_RETURN. Dùng mặt đỡ cơ khí chuẩn; kiểm góc độc lập
khác hai mốc fit, chúi ĐẦU dương/ngả ĐUÔI âm và độ lặp khi trở lại cùng tư thế.

Có thể dùng `-TrinhBienDich g++` nếu compiler ở PATH. Script tạo sáu executable
trong `.cache/`. Python tool có test riêng trong `test/python/`. Compiler lỗi hoặc assertion
thất bại làm script dừng. Nếu shell chưa nhận `pio`, gọi đường dẫn đầy đủ tới
PlatformIO trong môi trường Python đã cài.

| Kiểm tra | Lỗi được phát hiện | Giới hạn |
| --- | --- | --- |
| Native estimator/safety | Góc, norm, NaN, ngã ngang, mẫu 0/quá hạn | Không kiểm tra cảm biến và bus thật |
| Native odometry/controller | Đơn vị, dấu, giới hạn, luật mờ, reset, mô hình lý tưởng | Không chứng minh motor bám lệnh hay xe thật cân bằng |
| Native phiên/mẫu | Tắt-bật nhanh, lệnh khác phiên, zero-ack, trùng/quay vòng số mẫu | Chưa chạy lịch FreeRTOS thật |
| Native watchdog | Mã giám sát thật tắt EN giả khi không còn feed, giữ lỗi, đúng cực tính | Stub không mô phỏng độ trễ/khóa đa core hoặc register GPIO thật |
| Native boot/UI/ICM | Chính driver/UI với Wire giả: reset, ID, cấu hình, data-ready, NACK/thiếu byte, rung rồi yên và timeout | Chưa đo bus/nút/LED vật lý; xem `imu_boot_calib_tests.cpp` |
| Build firmware | Header/API/kiểu dữ liệu/link tài nguyên phù hợp bộ SDK cài đặt | Không đo timing/điện áp trên board |

Không coi số lượng assertion là số kịch bản độc lập: nhiều assertion đến từ vòng
quét miền mờ và mô phỏng nhiều bước thời gian.

## 6. Trình tự kiểm tra phần cứng

### A. IMU khi EN tắt

Giữ cờ `daXacNhanCauHinhPhanCungMotor=false` lúc kiểm tra ban đầu. Nạp firmware,
giữ xe yên lúc hiệu chuẩn. Kiểm tra ID, log `theta/rate/roll/accel`, nghiêng từng
chiều. Đứng yên Z hướng lên thì norm gần 9,81 m/s²; pitch/roll gần vị trí lắp mong
muốn. Nghiêng ngang lớn phải cho `ready=fallen`; mất dữ liệu phải báo lỗi/quá hạn.

### B. STEP/DIR và số đếm

Đối chiếu menu MKS `CR_vFOC`, vi bước 32, motor 1,8°, EN=L theo config. Phép thử
`test/stepMeansure.cpp` có thể dùng làm main riêng để đo xung khi EN luôn tắt; xem
[danh mục bench](DANH_MUC_HAM.md#16-các-chương-trình-bench-cũ-trong-test).
Không biên dịch đồng thời nhiều `setup()/loop()`.

Trên bản ứng dụng, sau khi mức điện/timing đã xác nhận và cờ phần cứng cho phép:
kê bánh, giữ thân xe, thử arm và chiều tác động. Đối chiếu `STEP` với logic analyzer.
Thử sparse, continuous, qua ngưỡng 1000 STEP/s và đảo chiều. Với cấu hình hiện tại,
6400 cạnh có dấu tương ứng một vòng motor/bánh khi bám lệnh. Log `cmd` có thể khác
tần số tức thời trong khoảng pause/setup; dùng số cạnh đo thực để đối chiếu PCNT.

### C. Tắt-bật và mã phiên

Thử `d`, rồi `a` nhanh, quan sát `epoch` tăng và `phase` đi qua chờ lệnh mới.
Phải thấy lần bật mới bắt đầu từ lệnh 0, không khôi phục tốc độ phiên trước.
Thử `x`, sau đó `a`: phải giữ tắt; cần `k` rồi `a` riêng. Thử thay đổi tư thế trong
lúc chờ bật: yêu cầu bị hủy. `last=6` nghĩa cờ cấu hình phần cứng chưa xác nhận.

### D. Kiểm tra giám sát độc lập

Thực hiện với bánh được kê và khả năng cắt nguồn motor. Dùng bản thử có một lệnh
cố ý chặn MotorTask khoảng 10 ms bằng `vTaskDelay` sau khi EN đã bật; đặt điểm chặn
ngoài critical-section để ngắt GPTimer vẫn chạy. Đo trực tiếp chân EN bằng logic
analyzer: EN phải tắt sau khi quá hạn phục vụ, không đợi task kết thúc chờ 10 ms.
Khi task tiếp tục, lỗi driver phải là `WatchdogExpired` và STEP dừng.

Bỏ đoạn gây chậm sau thử. Không đặt breakpoint/khóa ngắt toàn CPU rồi kết luận
GPTimer bảo vệ được mọi trạng thái treo. Hạn phản ứng thực còn gồm chu kỳ kiểm tra
250 µs và độ trễ phục vụ ISR. Phải đo bản firmware/board bạn thực sự dùng.

### E. Tinh chỉnh cân bằng

1. Xác nhận hướng góc/gyro và hướng hai bánh. Nghiêng trước → lệnh tăng tốc tiến.
2. Khi motor tắt, dùng `f` để chọn PID cố định. Có thể tạm đặt hệ số PI vận tốc về
   0 khi kiểm tra riêng vòng góc; ghi lại cấu hình trước khi thay.
3. Chỉnh Kp/Kd cơ sở với giá đỡ. Ghi góc cực đại, dao động, thời gian hồi phục và bão hòa.
4. Ghép lại PI vận tốc để hạn chế trôi. Không tăng đồng thời mọi hệ số.
5. Bật mờ, thử cùng điều kiện để so sánh. Khi thêm Ki góc, kiểm tra tích phân và
   thoát bão hòa riêng; mặc định hiện tại Ki=0.

## 7. Đọc log và tìm lỗi

| Trường | Ý nghĩa |
| --- | --- |
| `EN` | Trạng thái bật theo driver, có loại trừ ISR đã ngắt; không phải phản hồi UART MKS |
| `fuzzy` | 1 dùng mờ, 0 dùng hệ số cố định |
| `fault` | Lỗi ứng dụng giữ lại; thường cần `k` khi motor đã tắt |
| `ready` | Kết quả kiểm tra mẫu gần nhất; `none` chưa đồng nghĩa với đủ mọi điều kiện arm |
| `motorFault` | Lỗi thấp hơn trong driver; bảng phía dưới |
| `last` | Kết quả API motor gần nhất |
| `theta`, `rate`, `target` | Góc dọc, gyro dọc, góc mục tiêu; rad/rad/s/rad |
| `roll`, `accel` | Góc ngang từ gia tốc và norm gia tốc; rad/m/s² |
| `x`, `v` | Vị trí/vận tốc ước lượng từ xung; m/m/s |
| `STEP` | Tổng xung trái/phải có dấu, không phải tốc độ |
| `cmd` | Tốc độ STEP/s mục tiêu của driver sau giới hạn |
| `K` | Kp/Ki/Kd hiện hành; khi reset chưa cập nhật có thể là 0 |
| `serviceMax` | Khoảng cách MotorTask lớn nhất từ khi bắt đầu task, gồm cả lúc tắt |
| `phase` | 0=tắt, 1=chờ lệnh 0, 2=chạy |
| `epoch` | Mã phiên bật/tắt; thay đổi để vô hiệu lệnh cũ |
| `miss` | Số mẫu data-ready nhảy số thứ tự trước khi được xử lý |
| `dup` | Số thông báo trùng/lùi đã bỏ |
| `overlap` | Số lần bỏ kết quả đọc vì có ngắt mới trong lúc đọc I²C |

| `motorFault` | Tên | Kiểm tra |
| --- | --- | --- |
| 0 | None | Không có lỗi driver |
| 1 | InvalidCommand | NaN/Inf, timestamp tương lai/lùi, hạn không hợp lệ |
| 2 | CommandExpired | ControlTask ngừng gửi lệnh mới hoặc deadline hết |
| 3 | ServiceLate | MotorTask phục vụ chậm quá cấu hình |
| 4 | BackendError | API GPIO/MCPWM/GPTimer lỗi, xem init/config |
| 5 | SparseBacklog | Task chậm làm tích lũy quá nhiều xung thưa |
| 6 | EmergencyStop | App yêu cầu dừng vì `x` hoặc lỗi dữ liệu/tư thế; xem `fault` |
| 7 | WatchdogExpired | Giám sát GPTimer đã ngắt EN vì hạn phục vụ/lệnh |

Các mã trên theo `MksFault` hiện tại. Khi thêm enum hãy cập nhật bảng và công cụ đọc log.

`MksResult` (`last`): Ok=0, Saturated=1, Pending=2, Disabled=3, NotInitialized=4,
ConfigInvalid=5, HardwareConfigUnconfirmed=6, FaultLatched=7.

| Tình huống | Hướng tìm nguyên nhân |
| --- | --- |
| Không READY | Mở monitor rồi reset để xem lại BOOT ERROR in một lần; kiểm tra bus/ID, gyro có đứng yên, tài nguyên timer/queue |
| `ready=imu_invalid` | Cờ frame/bias, norm ngoài miền, dữ liệu NaN, timestamp trùng/lùi |
| `ready=pulse_invalid` lúc mới khởi động | Mẫu đầu chưa có vận tốc; nếu kéo dài kiểm tra PCNT và timestamp |
| `ready=fallen` dù nhìn xe đứng | Kiểm tra trục/dấu IMU, điểm đặt góc, cách lắp |
| Gửi `a` không chạy | Kiểm tra cờ phần cứng, góc/gyro cả hai trục, `fault`, `phase`, `last` |
| `arm_timeout` | ControlTask không tạo được zero-ack đúng phiên trong hạn |
| `overlap/miss` tăng nhiều | Đo thời gian I²C và lịch task trước khi thay tần số/timeout |
| `motor_fault` và driver watchdog/service late | Tìm đoạn chặn MotorTask/critical-section/log ngoài chỗ cho phép; đo timing |
| Xe tăng tốc sai phía | Dấu gyro/góc và ánh xạ DIR từng bánh phải cùng quy ước |
| Dao động dù không lỗi số học | Đo trễ/bão hòa, kiểm tra hệ số, độ rơ/rung và lực motor |

## 8. Khi thêm chức năng mới

**Điều khiển tiến/lùi:** tạo nguồn lệnh vận tốc có hạn hiệu lực riêng, giới hạn gia tốc
đặt và đưa vào vòng PI. Tránh dùng Serial làm vòng định thời PID.

**Quay đầu:** hiện đã có `tronUuTienCanBang()` và lệnh j/l; phần quay bị chặn theo
dư địa sau thành phần cân bằng. Nếu thêm giữ hướng, cần phản hồi yaw và test riêng;
không coi tốc độ quay đặt là phép đo hướng thực.

**Giữ vị trí:** thêm vòng ngoài chậm hơn vòng vận tốc, sai lệch vị trí tạo vận tốc
đặt. Nhớ rằng vị trí hiện tại là suy từ xung, có thể lệch khi bánh trượt.

**Tách task ra file riêng:** chuyển thân `imuLoop/controlLoop/motorLoop` vào các
file tương ứng nhưng giữ nguyên chủ sở hữu đối tượng, queue và phiên. Việc tạo file
rỗng không tự tạo FreeRTOS task; chỉ `xTaskCreate` mới tạo task đang chạy.
