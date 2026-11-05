# Checklist tính năng và mức kiểm chứng

## Bổ sung 05/11/2026 — sơ đồ, bù gá lắp và UI calib

- [x] Phép quay sensor→xe cho cả accel và gyro, cấu hình Euler hoặc ma trận.
- [x] Kiểm tra ma trận, loại NaN/Inf, shear và determinant âm trước khi cho chạy.
- [x] Estimator vector phương đứng3D, pitch có dấu và pitch rate có bù roll/yaw.
- [x] Công cụ dựng R từ gravity chuẩn và gyro nghiêng tiến; log RAW_IMU5Hz khi motor tắt.
- [x] Test mounting3114 assertion và5 test Python đã đạt; các test nền vẫn đạt.
- [x] Đối chiếu GPIO theo sơ đồ: SDA10/SCL11, CS9/SA08, EN2, nút3, INT7.
- [x] Ánh xạ theo người dùng: motor 1 phải DIR12/STEP13, motor 2 trái DIR5/STEP6.
- [x] Driver đặt CS HIGH và SA0 theo địa chỉ I²C trước khi mở bus; static_assert phát hiện trùng GPIO.
- [x] UI nút/RGB trong setup: bias tĩnh → mốc trái → lật trái–phải → mốc phải → kiểm tư thế; lỗi cho thử lại với EN tắt.
- [x] Giữ X từ chuyển động khung khi dựng ma trận; không ép pitch người giữ thành0.
- [x] Native calib roll đạt, gồm pitch ban đầu ±6° và roll tham chiếu -4°/0°/+4°.
- [x] LED trắng so tư thế với `gocCanBang_rad`; xanh lá chỉ báo READY, Serial `a` mới yêu cầu bật.
- [ ] Xác minh RGB GPIO48/thứ tự GRB và nút kéo xuống GND trên board thật.
- [ ] Thực hiện UI calib trên xe, kiểm góc/dấu sau bù; nếu tắt UI, nhập ma trận đo vào cấu hình identity.
- [ ] Xác định góc cân bằng cơ học riêng; `gocCanBang_rad=0` chưa phải kết quả đo trọng tâm.
- [ ] Nghiệm thu sai số góc/timing/ngã ngang thật sau khi bù gá.

Xem [báo cáo góc lắp](BAO_CAO_BU_LECH_LAP_IMU.md),
[sơ đồ và flow calib](DOI_CHIEU_SO_DO_VA_FLOW_CALIB.md). Các dấu đã làm bên dưới vẫn chỉ
phản ánh mã/test phần mềm nếu chưa có bằng chứng đo xe.

> Cập nhật lọc và di chuyển 03/11/2026: [ICM20602](BAO_CAO_ICM20602_LOC_NHIEU.md), [MKS42C/17HS4401S](BAO_CAO_MKS42C_17HS4401S.md). Runtime không DMP; PT1 chỉ nhánh D; có lệnh di chuyển có hạn; R01 đã sửa.

Ngày đồng bộ: **05/11/2026 — Asia/Saigon**. Áp dụng cho working tree hiện tại,
ESP32-S3 + ICM20602 + hai MKS Servo42C nhận STEP/DIR.

**Dấu `[x]` ở phần A nghĩa là đã có trong mã, không có nghĩa đã chạy đạt trên xe.**
Mỗi dòng có mức kiểm chứng riêng:

- **Đọc mã + build**: đã đọc luồng và compiler/linker chấp nhận; chưa test hành vi I/O.
- **PC**: đã chạy kiểm thử số học/logic trên máy.
- **Giả lập I/O**: chạy mã giám sát thật với GPIO/GPTimer giả; không đo điện hay timing chip.
- **Xe thật**: phải có log/phép đo thực tế; đợt này chưa có bằng chứng để đánh dấu đạt.

Kết quả chi tiết và việc còn mở: [báo cáo rà soát](KET_QUA_RA_SOAT_2026-11-03.md).

## A. Tính năng đã có trong mã

| Đã có | Tính năng | Nơi thực hiện | Kiểm chứng hiện có / giới hạn |
| --- | --- | --- | --- |
| [x] | Khởi tạo ICM20602, kiểm tra ID `0x12` | `Icm20602Driver::begin()` | Đọc mã + build; chưa test Wire/ICM thật |
| [x] | Reset/wake, cấu hình DLPF, range và đọc lại thanh ghi | Driver IMU | Đọc mã + build; dải đang cố định ±4 g/±500 dps |
| [x] | Đọc burst 14 byte, đổi m/s², rad/s, nhiệt độ | `Icm20602Driver::read()` | Đọc mã + build; chưa có native test driver I²C |
| [x] | Thu bias gyro 500 mẫu, kiểm tra phương sai và timeout | `hieuChinhConQuay()` | Đọc mã + build; người dùng phải giữ xe yên |
| [x] | Data-ready ISR ghi timestamp và báo task | `RobotApp::imuInterrupt()` | Đọc mã + build; cần đo nhịp ngắt thật |
| [x] | Loại thông báo trùng/lùi, xử lý số thứ tự quay vòng | `BoLocThuTuMau` | PC |
| [x] | Đếm mẫu nhảy số thứ tự | `BoLocThuTuMau` | PC |
| [x] | Bỏ kết quả đọc nếu có ngắt mới trong lúc I²C | `RobotApp::imuLoop()` | Đọc mã + build; chưa thử lịch ngắt thật |
| [x] | Bộ lọc bổ sung ước lượng góc dọc | `ComplementaryEstimator` | PC với dữ liệu tổng hợp |
| [x] | Chặn NaN/Inf, norm ngoài miền, mẫu toàn 0, timestamp lỗi | Estimator + SafetyMonitor | PC, gồm mẫu toàn 0 sau init |
| [x] | Kiểm tra góc/tốc độ nghiêng ngang khi bật; bảo vệ ngã ngang | Estimator + SafetyMonitor | PC, có trường hợp nghiêng ±85° |
| [x] | Phân biệt điều kiện đang chạy và điều kiện bật | `check()` / `canArm()` | PC |
| [x] | PCNT đếm STEP có dấu theo DIR của từng bánh | `WheelPulseCounter` | Đọc mã + build; chưa đo cạnh/dấu/tràn counter trên chip |
| [x] | Quy đổi 6400 xung/vòng theo cấu hình | `DifferentialDrive` | PC kiểm tra đơn vị và đảo chiều |
| [x] | Tính vị trí dọc và lọc vận tốc từ xung | `DifferentialDrive::update()` | PC; vị trí là ước lượng, chưa có giữ vị trí |
| [x] | Cấu trúc PID góc, D từ gyro, giới hạn đầu ra | `AnglePidController` | PC với Ki góc mặc định bằng 0 |
| [x] | Suy luận mờ 5×5, ba bảng hệ số nhân Kp/Ki/Kd | `scheduledGains()` | PC quét miền hệ số; bảng luật chưa tối ưu trên xe |
| [x] | PI vận tốc ngoài, góc đặt có giới hạn | `CascadeController` | PC và mô hình chấp hành lý tưởng |
| [x] | Tích phân gia tốc → vận tốc → STEP/s | `CascadeController::update()` | PC kiểm tra đơn vị, dấu, trần và reset |
| [x] | Chặn tích phân/bão hòa | PID/PI | Có mã; regression xả tích phân Ki20 hai chiều đã đạt; chưa thử xe |
| [x] | MCPWM phát STEP liên tục và single-shot | `McpwmStepOutput` | Đọc mã + build; chưa đo dạng sóng |
| [x] | Tích lũy xung thưa, không ép tốc độ tối thiểu | `MksServo42CPair::advance()` | Đọc mã + build; test odometry xung thưa không thay test bộ phát xung |
| [x] | Pause/hold/setup khi đổi DIR hoặc sparse↔continuous | Driver motor | Đọc mã + build; cần logic analyzer |
| [x] | Giới hạn tần số, hạn lệnh, lỗi phục vụ chậm và backlog | Driver motor | Đọc mã + build; chưa native test toàn bộ MKS pair |
| [x] | GPTimer giám sát tách khỏi MotorTask, chỉ tắt EN trong ISR | `GiamSatDongCo` | Giả lập I/O: 19 kiểm tra; chưa đo thời gian cắt EN |
| [x] | Giữ lỗi sau watchdog, không tự hồi sinh khi feed muộn | `GiamSatDongCo` | Giả lập I/O |
| [x] | Bật lại theo mã phiên, xác nhận hai lệnh tốc độ bằng 0 | `PhienCanBang` + RobotApp | Helper có PC; phối hợp task thật mới đọc mã + build |
| [x] | Reset controller khi đổi phiên/tắt/lỗi/đổi chế độ mờ | `controlLoop()` | Có test reset controller và phiên riêng; chưa test toàn app FreeRTOS |
| [x] | Queue dài 1 lưu trạng thái/lệnh mới nhất | `RobotApp` | Đọc mã + build |
| [x] | Serial `a/d/x/k/f/h`, ưu tiên dừng trước bật | `RobotApp::loop/motorLoop()` | Đọc mã + build; thử từng ký tự trên board còn thiếu |
| [x] | Telemetry góc, vận tốc, xung, hệ số, lỗi, phase/epoch | `RobotApp::loop()` | Đọc mã + build; chưa ghi log phiên chạy thực |
| [x] | Cờ xác nhận phần cứng chặn enable mặc định | `MotorConfig.h` + driver | Đọc mã + build; hiện là `false` |
| [x] | Native test và script chạy năm nhóm test | `test/run_native.ps1` | Đã chạy PASS ngày 05/11/2026; gồm calib roll |
| [x] | Tài liệu sử dụng, lập trình, logic, danh mục hàm và sửa đổi | `docs/` | Đối chiếu source, đường dẫn và công thức |

## A2. Đã bổ sung sau rà soát

- [x] DLPF ICM20602 gyro41Hz/accel44,8Hz, dữ liệu raw và fusion trên ESP32.
- [x] PT1 nhánh D, test đáp ứng50Hz và NaN/reset/bypass.
- [x] Trọng số norm/innovation, test gia tốc ngang3m/s² và mất tin cậy kéo dài.
- [x] Lệnh w/s/j/l/z có epoch/deadline500ms, ramp mục tiêu.
- [x] Mixer giới hạn yaw mà giữ thành phần cân bằng; test hai chiều/saturation.
- [x] Motor17HS4401S và h0,025m được ghi vào tính toán; chưa xác nhận nguồn.
- [x] Script tái sinh JSON/CSV/Markdown từ config.

## B. Việc phần mềm còn mở

- [x] **R01:** cho xả tích phân theo hướng thoát trần; regression Ki20 hai chiều đạt. Ki production vẫn0.
- [ ] **R02:** lưu/in lại lỗi khởi động để monitor mở muộn vẫn đọc được nguyên nhân.
- [ ] Viết test giao thức I²C cho driver ICM: ID sai, thiếu byte, reset/readback lỗi,
  timeout và hiệu chuẩn khi dữ liệu thay đổi. Hiện driver chưa có test native trực tiếp.
- [ ] Test tích hợp RobotApp/FreeRTOS, các queue và thứ tự ngắt; helper test chưa bao phủ lịch task thật.
- [ ] Test trực tiếp MKS pair, MCPWM và PCNT (mô phỏng peripheral hoặc bench phần cứng).
- [ ] Chốt phiên bản nền tảng thay cho URL `stable` để lần cài sau tái lập cùng bộ công cụ.
- [ ] Đối chiếu profile `lolin_s3_mini` với đúng biến thể ESP32-S3 Super Mini của bạn.

R01 đã có sửa và test; cấu hình hiện tại vẫn `kiGoc=0`. R02 có cách làm
tạm thời: mở monitor, giữ xe yên rồi reset board để xem lại log setup.

## C. Tính năng chưa triển khai / chưa thuộc phiên bản này

- [ ] Giữ hướng/yaw vòng kín. Đã có trộn lệnh quay Serial và giới hạn hai bánh ưu tiên cân bằng.
- [ ] Vòng giữ vị trí tuyệt đối; code hiện mới tính vị trí từ xung.
- [ ] Điều khiển vận tốc từ xa qua Bluetooth/Wi-Fi/gamepad.
- [ ] Thay đổi/lưu hệ số PID hoặc cấu hình qua Serial/NVS; hiện sửa config rồi build lại.
- [ ] Lưu ma trận UI calib qua NVS; hiện ma trận chỉ tồn tại trong RAM và đo lại mỗi boot.
- [ ] Nút GPIO3 bật/tắt cân bằng sau READY; hiện chỉ dùng trong calib, vận hành qua Serial.
- [ ] Hiệu chuẩn gia tốc đầy đủ, bù nhiệt gyro hoặc tư thế 3D có yaw xác định; hiện đã có vector phương đứng3D.
- [ ] Bù góc thân vào góc bánh tuyệt đối, phát hiện trượt bánh.
- [ ] Mô hình mô phỏng có trễ, ma sát, lực motor, giới hạn nguồn và rung cảm biến thực.
- [ ] Đọc encoder/trạng thái MKS qua UART — không phải yêu cầu bắt buộc của kiến trúc đã thống nhất.
- [ ] Tách các task sang `src/tasks/`, hoàn thiện `RobotCore`, `Logger`, interfaces,
  KalmanEstimator nếu thực sự cần. Hiện các file khung này vẫn rỗng.

## D. Checklist nghiệm thu trên xe thật

Chỉ đánh dấu sau khi có bằng chứng đo, ghi firmware/config dùng khi thử.

| Đạt | Phép thử | Tiêu chí / bằng chứng cần lưu |
| --- | --- | --- |
| [ ] | Board và sơ đồ chân | Đúng số GPIO, flash/PSRAM, nguồn và mức logic của từng module |
| [ ] | ICM khởi tạo | ID 0x12, range/readback đúng, đọc không thiếu byte |
| [ ] | IMU đứng yên | Norm gần trọng lực, gyro sau hiệu chuẩn gần 0; ghi log và điều kiện đo |
| [ ] | Chiều dọc/ngang | Dấu theta/rate phù hợp cách lắp; roll phản ánh nghiêng ngang |
| [ ] | Nhịp IMU | Đo data-ready và thời gian đọc; kiểm tra miss/dup/overlap khi các task cùng chạy |
| [ ] | Không tự bật | Reset/mất kết nối console/lỗi setup vẫn giữ EN tắt |
| [ ] | STEP/DIR | Đo HIGH/LOW, hold/setup, cạnh đầu, đảo chiều, qua 1000 STEP/s |
| [ ] | PCNT | Đếm khớp analyzer cả hai dấu, single-shot, xung thưa, continuous, qua ±30000 |
| [ ] | Cơ khí/motor | Số vòng tương ứng xung dưới tải; thử phản ứng gần 0 và đảo chiều |
| [ ] | EN và watchdog | EN thực của MKS tắt khi task chậm; đo độ trễ cắt, thử cả hạn lệnh/hạn phục vụ |
| [ ] | Bật lại nhanh | Lệnh cũ bị bỏ; phiên mới bắt đầu 0; quan sát bằng log/trace thích hợp |
| [ ] | Tư thế/lỗi mẫu | Từ chối bật khi nghiêng ngang; dừng khi ngã/mẫu lỗi/quá hạn |
| [ ] | Lệnh điều hành | Thử riêng a/d/x/k/f/h; k không tự bật, f chỉ đổi khi tắt |
| [ ] | PD cố định | Có số đo dao động, góc cực đại, thời gian hồi phục trước khi ghép vòng khác |
| [ ] | PI vận tốc | Hạn chế trôi trong điều kiện thử cụ thể, có số liệu bão hòa |
| [ ] | Bộ mờ | So sánh cùng điều kiện với PD cố định, không chỉ đánh giá bằng cảm giác |
| [ ] | Ki góc >0 nếu dùng | Đo thoát bão hòa trên xe; regression phần mềm R01 đã đạt |
| [ ] | Chạy dài | Ghi thời lượng, nhiệt, nguồn, lỗi, điều kiện mặt sàn và nhiễu thực |

Mẫu ghi kết quả: `ngày — firmware SHA256 — cấu hình — nguồn/motor/tải — phép thử —
giá trị đo — đạt/chưa đạt — đường dẫn log/ảnh analyzer`.

## E. Kết quả kiểm tra trước đợt nâng cấp lọc/di chuyển ngày 03/11/2026

| Kiểm tra | Kết quả | Phạm vi |
| --- | --- | --- |
| Native điều khiển/ước lượng/an toàn/phiên | PASS, 14507 assertion | Gồm các vòng quét/mô phỏng; không phải 14507 kịch bản độc lập |
| Mã GiamSatDongCo với GPIO/GPTimer giả | PASS, 19 assertion | Kiểm tra logic EN/deadline/latch, không đo ngắt vật lý |
| `pio run -e lolin_s3_mini` | SUCCESS | Build và link firmware; chưa upload |
| RAM/Flash theo build | 56964 / 461478 byte | 17,4% RAM; 35,2% phân vùng ứng dụng |
| Callback/helper giám sát trong ELF | `.iram0.text` | Xác nhận vị trí mã của ba hàm; chưa chứng minh toàn bộ đường ISR luôn chạy khi cache tắt |

Sau khi sửa code hoặc đổi toolchain, cập nhật ngày, kết quả và các ô có liên quan.

Bản05/11 sau cập nhật sơ đồ/UI: năm nhóm C++ đạt (14507 + 19 + 169 + 3114
assertion ở bốn nhóm cũ, thêm nhóm calib roll); build SUCCESS, RAM57044 byte,
Flash469974 byte. Năm test Python đã đạt ở đợt góc gá trước đó. Kiểm thử lần này
chưa có upload, nút/RGB/IMU thật hoặc thử cân bằng, nên các ô nghiệm thu phần cứng
vẫn chưa được đánh dấu.
