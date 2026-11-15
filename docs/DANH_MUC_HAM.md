# Danh mục công dụng các hàm

> Bản 05/11/2026 bổ sung hệ trục và estimator vector3D; xem [báo cáo góc lắp](BAO_CAO_BU_LECH_LAP_IMU.md).

> Cập nhật lọc và di chuyển 03/11/2026: [ICM20602](BAO_CAO_ICM20602_LOC_NHIEU.md), [MKS42C/17HS4401S](BAO_CAO_MKS42C_17HS4401S.md). Runtime không DMP; PT1 chỉ nhánh D; có lệnh di chuyển có hạn; R01 đã sửa.

Đối chiếu lại ngày **08/11/2026**. Xem [checklist](CHECKLIST_TINH_NANG.md) để biết
hàm/tính năng đã có test PC hay còn cần phép đo trên chip.

Tài liệu đi theo module, bao gồm hàm public, private, callback, helper, test và
chương trình bench còn giữ trong repo. Tên hàm giữ đúng source để dùng tìm kiếm
trong IDE. API Arduino/ESP-IDF bên ngoài được giải thích riêng ở cuối tài liệu.
Đọc [logic tổng thể](LOGIC_CHUONG_TRINH.md) trước nếu chưa rõ luồng dữ liệu.

## 1. Điểm vào — `src/main.cpp`

| Hàm | Ai gọi | Công dụng và kết quả |
| --- | --- | --- |
| `setup()` | Arduino, một lần | Gọi `xeCanBang.begin()`. Chưa có cơ chế tự khởi tạo lại nếu thất bại |
| `loop()` | Arduino, lặp lại | Gọi `xeCanBang.loop()` để nhận lệnh và ghi log; không tự tính PID |

`xeCanBang` là đối tượng toàn cục, tồn tại suốt chương trình. Điều này cũng bảo đảm
các con trỏ `this` truyền vào task/ISR không trỏ tới đối tượng đã hết vòng đời.

## 2. Ứng dụng — `src/app/RobotApp.h/.cpp`

| Hàm | Đầu vào/kết quả | Công dụng, người gọi và lưu ý |
| --- | --- | --- |
| `RobotApp()` | Không tham số | Gắn `_camBienGoc` với `Wire`; chưa truy cập phần cứng |
| `begin()` | Trả `bool` | `setup()` gọi. Tắt EN, kiểm tra config, tạo motor/PCNT/IMU, chạy UI hoặc bias đơn lẻ, áp dụng ma trận, tạo queue/task/ngắt/timer, bỏ phím Serial chờ. Thành công đặt `_daSanSang`, LED cam chờ dữ liệu và in FIRMWARE/READY; sau attach đánh thức ImuTask ACK cờ tồn, không coi lần đánh thức không có timestamp nguồn là mẫu hợp lệ |
| `loop()` | Không trả dữ liệu | Console đọc tối đa 16 ký tự/lượt, gom bit yêu cầu, gửi cho MotorTask; in snapshot/chẩn đoán mỗi 200 ms và cập nhật LED runtime cam/xanh/đỏ theo dữ liệu/lỗi. `delay(1)` chỉ ở task console |
| `cleanupStartup()` | Không tham số | Dọn tài nguyên khi tạo queue/task/timer thất bại: dừng timer, tháo ISR, xóa task, disable motor, xóa queue |
| `imuInterrupt(void* nguCanh)` | Con trỏ ứng dụng | ISR data-ready. Chụp `esp_timer_get_time()`, tăng số ngắt dưới khóa rồi đánh thức ImuTask. Không đọc I²C tại đây |
| `motorTimer(void* nguCanh)` | Con trỏ ứng dụng | Callback `esp_timer` trong task timer; gửi bit `suKienDinhKy`. Đây là bộ đánh thức, không phải bộ ngắt EN độc lập |
| `imuEntry(void*)` | Con trỏ `RobotApp` | Hàm static phù hợp chữ ký FreeRTOS, chuyển sang `imuLoop()` của đúng đối tượng |
| `controlEntry(void*)` | Tương tự | Chuyển sang `controlLoop()` |
| `motorEntry(void*)` | Tương tự | Chuyển sang `motorLoop()` |
| `imuLoop()` | Chạy vô hạn | Nhận ngắt, loại thứ tự đã xử lý, đọc status+14 byte để ACK, gọi `chotMauImu`; khi hết chờ 20 ms chỉ ACK phục hồi và xuất frame chưa hợp lệ. Lưu thống kê IRQ/I²C/timeout/đọc trễ trên queue chẩn đoán riêng, kể cả khi frame góc bị bỏ |
| `controlLoop()` | Chạy vô hạn | Nhận frame, lấy snapshot xung, cập nhật ước lượng và an toàn. Reset controller khi đổi phiên/tắt/lỗi/đổi mờ. Gửi lệnh với đúng mã phiên và số thứ tự lệnh |
| `motorLoop()` | Chạy vô hạn | Xử lý ưu tiên dừng, quản lý `PhienCanBang`, kiểm tra lệnh mới/tư thế, enable sau xác nhận 0, apply/service motor, đọc PCNT và gửi snapshot |
| `nowUs()` | Trả `uint64_t` | Helper trong file: đọc thời gian đơn điệu microsecond |
| `faultName(FaultCode)` | Trả `const char*` | Đổi lỗi ứng dụng thành nhãn log như `fallen`, `imu_invalid`, `arm_timeout` |

`imuEntry/controlEntry/motorEntry` cần vì FreeRTOS nhận con trỏ hàm thông thường;
hàm thành viên không static có tham số `this` ẩn. Các wrapper lấy lại `this` qua
`nguCanh` rồi gọi hàm thành viên tương ứng.

## 3. Phiên bật/tắt — `src/core/PhienCanBang.h`

Chỉ MotorTask thay đổi đối tượng này. ControlTask đọc bản sao `maPhien/cheDo` trong
snapshot, không gọi các hàm thay đổi phiên.

| Hàm | Công dụng | Điều kiện/kết quả |
| --- | --- | --- |
| `yeuCauBat(thoiDiem_us)` | Tăng mã phiên, lưu thời điểm yêu cầu, vào `ChoLenhMoi` | App chỉ gọi từ `Tat` sau kiểm tra tư thế. Chưa bật GPIO |
| `tat()` | Tăng mã phiên và chuyển `Tat` | Vô hiệu hóa mọi lệnh của phiên trước. Việc tắt phần cứng do driver thực hiện riêng |
| `xacNhanDaBat()` | Chuyển `ChoLenhMoi` → `DangChay` | Trả false nếu không đang chờ. App gọi sau `enable() == Ok` |
| `quaHanCho(thoiDiem_us)` | Kiểm tra hạn chờ 100 ms hoặc thời gian lùi bất thường | Chỉ true khi đang chờ và vi phạm hạn |
| `lenhKhoiDongHopLe(maPhien, lenh, thoiDiem_us)` | Kiểm tra xác nhận khởi động | Yêu cầu đúng phiên, timestamp sau yêu cầu và không ở tương lai, tuổi <10 ms, chưa hết hạn, cả hai tốc độ bằng 0 |
| `maPhien()` | Đọc mã hiện tại | Không sửa dữ liệu |
| `cheDo()` | Đọc `Tat/ChoLenhMoi/DangChay` | Không phản ánh trực tiếp GPIO; trạng thái driver vẫn phải kiểm tra |

## 4. Số thứ tự IMU — `src/core/BoLocThuTuMau.h`

| Hàm | Công dụng |
| --- | --- |
| `chapNhan(soThuTu)` | Nhận mẫu đầu; sau đó chỉ nhận số thứ tự tiến lên. Phép trừ unsigned hỗ trợ quay vòng 32 bit. Trùng hoặc lùi bị bỏ, tăng bộ đếm trùng; nhảy nhiều bước tăng số bỏ lỡ |
| `soMauBoLo()` | Trả tổng số thứ tự bị nhảy, 64 bit. Không gồm mẫu riêng bị bỏ vì I²C trùng ngắt |
| `soThongBaoTrung()` | Trả số thông báo trùng/lùi đã bỏ |

Hàm này không tự đọc cảm biến hoặc quyết định ngắt motor. Tuổi dữ liệu và lỗi I²C
được xử lý bởi các tầng tiếp theo.

## 5. Driver ICM20602 — `lib/imu/src/Icm20602Driver.h/.cpp`

| Hàm | Đầu vào/đầu ra | Công dụng |
| --- | --- | --- |
| `Icm20602Driver(TwoWire& busI2c)` | Tham chiếu bus | Lưu bus trong `_busI2c`; không tự sao chép/tạo bus |
| `begin()` | `bool` | Kiểm tra config ±4 g/±500 dps, tần số chia1000; đặt CS GPIO9 HIGH và SA0 GPIO8 theo địa chỉ0x68/0x69, mở bus SDA10/SCL11, kiểm tra ID, reset/wake, ghi DLPF/divider/range/INT giữ mức, đọc lại cấu hình. `INT_PIN_CFG=0x20` active HIGH hoặc `0xA0` active LOW; chỉ đọc INT_STATUS mới ACK. Chỉ đặt `_daKhoiTao` nếu đủ bước |
| `docMauMoi(KhungDuLieuImu&)` | Frame; `bool` | Xóa frame cũ, poll `INT_STATUS`; có data-ready mới đọc. false + `DuLieuCu` là đang chờ; false + `LoiIO` là lỗi bus. Dùng trước ISR/task |
| `read(KhungDuLieuImu& khungDuLieu)` | Ghi vào frame; trả thành công | Xóa frame, đọc 14 byte từ `0x3B`, gọi `giaiMaMau()`. Đây là đường đọc raw, không ACK `INT_STATUS`; runtime dùng `docMauTheoNgat()` |
| `docMauTheoNgat(KhungDuLieuImu&)` | Frame; `bool` | Burst 15 byte từ `0x3A`: ACK status và nhận accel/temp/gyro, yêu cầu DATA_RDY. false + `DuLieuCu` khi chưa có mẫu, false + `LoiIO` khi lỗi bus; chưa tự gắn timestamp |
| `giaiMaMau(boDem, KhungDuLieuImu&)` | Private; 14 byte mẫu | Ghép big-endian có dấu, đổi đơn vị, trừ bias và đặt trạng thái; dùng chung cho đường raw và runtime |
| `hieuChinhConQuay(uint16_t soMau)` | Số mẫu; `bool` | Xóa bias cũ, thu cả accel/gyro bằng data-ready, dùng Welford kiểm độ yên; khoảng rung tự thu lại trong 30 s. Lỗi bus/mất mẫu riêng; chỉ chấp nhận bias sau khi đạt |
| `kiemTraSauKhoiDong(BaoCaoKiemTraIcm&)` | Ghi kết quả; `bool` | Kiểm ID, cấu hình, nguồn, ngắt, rồi 32 mẫu data-ready mới hợp lệ trong 1,5 s; không phải self-test MEMS |
| `maLoiI2cCuoi()` / `thanhGhiCuoi()` / `maNhanDang()` | Getter | Cho tầng app/UI in mã bus, thanh ghi cuối và ID đã đọc mà không gây thêm giao dịch |
| `ghiThanhGhi(diaChi, giaTri)` | Một thanh ghi/một byte; `bool` | Gửi địa chỉ thanh ghi rồi dữ liệu qua I²C; kiểm tra `endTransmission()` |
| `docNhieuByte(diachiBatDau, duLieu, soByte)` | Vùng nhớ ghi, độ dài 1–15; `IcmI2cStatus` | Ghi con trỏ thanh ghi, repeated-start, yêu cầu đủ byte, kiểm tra từng lần đọc. Các lỗi phân biệt tham số/giao tiếp/thiếu byte |
| `ghepSo16Bit(byteCao, byteThap)` | Trả `int16_t` | Ghép big-endian và biểu diễn số âm đúng từ dữ liệu hai byte |

`begin()` và hiệu chuẩn có `delay()` nhưng chỉ chạy trước khi điều khiển.
`read()` không tự chờ data-ready. ImuTask dùng `docMauTheoNgat()` và chốt mẫu
bằng ngắt nguồn; UI polling vẫn dùng `docMauMoi()`. Không đọc bus từ hai task đồng thời. Cảm biến
không trực tiếp trả góc nghiêng; góc được tính ở module tiếp theo.

## 6. Bộ lọc góc — `lib/estimation/ComplementaryEstimator.h/.cpp`

| Hàm | Công dụng/đầu ra |
| --- | --- |
| `update(const KhungDuLieuImu&, TrangThaiXe&)` | Kiểm tra frame/bias/số hữu hạn/norm/timestamp; tính roll bảo vệ và pitch từ accel; lần đầu khởi tạo bằng accel tin cậy; sau đó tích phân gyro và hiệu chỉnh bằng accel. Ghi góc, gyro, chất lượng, timestamp vào trạng thái; false giữ cờ góc không hợp lệ |
| `reset()` | Đánh dấu chưa khởi tạo, xóa mốc thời gian. Mẫu hợp lệ tiếp theo khởi tạo lại góc bằng accel; không trực tiếp bật/tắt motor |

Gia tốc ngoài miền cứng khiến update thất bại/reset. Gia tốc trong miền cứng nhưng
không gần trọng lực vẫn cho phép dự đoán bằng gyro sau khởi tạo, còn `canArm()` sẽ
từ chối bật mới. Góc ngang dùng gia tốc tức thời nên cần đánh giá nhiễu/rung thực tế.

## 7. Quy đổi bánh xe — `lib/kinematics/DifferentialDrive.h/.cpp`

| Hàm | Đầu vào/kết quả | Công dụng |
| --- | --- | --- |
| `metersPerStep()` | Trả `double`, m/xung | `2πR / (buocDayDu × viBuocDauVao × tiSoTruyen)`; kiểm tra cấu hình dương khi compile |
| `stepsPerSecond(vanToc_mps)` | Trả `float`, xung/s có dấu | Chia vận tốc cho quãng đường mỗi xung; không tự giới hạn tốc độ |
| `update(const WheelPulseFrame&, TrangThaiXe&)` | Ghi vị trí/vận tốc/cờ/nguồn/timestamp | Vị trí từ tổng xung; vận tốc từ chênh lệch hai mẫu và bộ lọc bậc một. Mẫu đầu thiếu vận tốc; timestamp trùng/lùi không hợp lệ; khoảng mẫu quá dài reset vận tốc lọc |

Tổng trái/phải được tính trung bình nên hai bánh quay ngược bằng nhau không tăng
quãng đường dọc. Module chưa tính góc quay đầu/yaw hoặc bù chuyển động vỏ motor do thân nghiêng.

## 8. PID góc — `lib/controllers/AnglePidController.h/.cpp`

| Hàm | Công dụng |
| --- | --- |
| `reset()` | Xóa tích phân góc và hệ số báo cáo, dùng khi tắt/đổi phiên/lỗi |
| `update(gocNghien_rad, tocDoNghien_radps, gocMucTieu_rad, buocThoiGian_s, dungPidMo, huongBaoHoaVanToc)` | Trả gia tốc m/s². Chặn đầu vào lỗi hoặc dt ngoài (0;0,02] bằng NaN; tính sai lệch, chọn hệ số, tích phân có giới hạn/chống bão hòa, D từ gyro, chặn gia tốc |
| `scheduledGains(saiLechGoc_rad, tocDoNghien_radps, dungPidMo)` | Trả `PidGains`. Tắt mờ trả hệ số cơ sở; bật mờ chuẩn hóa, duyệt 25 luật, giải mờ ba bảng hệ số nhân |
| `heSoPid()` | Trả hệ số lần cập nhật gần nhất để ghi log |
| `membership(giaTriChuanHoa, chiSoTapMo)` | Helper độ thuộc tam giác, tâm -1/-0,5/0/0,5/1. Đầu vào đã clamp nên hai tập biên giữ độ thuộc ở ngoài miền |

`bangHeSoTiLe`, `bangHeSoTichPhan`, `bangHeSoViPhan` là ba bảng 5×5, không phải
hàm. Hàng theo sai lệch, cột theo gyro chuẩn hóa. `PidGains` chứa `heSoTiLe`,
`heSoTichPhan`, `heSoViPhan`; đây là các hệ số hiện hành sau suy luận mờ.

## 9. Điều khiển tầng — `lib/controllers/CascadeController.h/.cpp`

| Hàm | Công dụng |
| --- | --- |
| `reset()` | Reset PID góc, mốc thời gian, tích phân PI vận tốc, bộ đếm chu kỳ PI và vận tốc lệnh; góc đặt về góc cân bằng |
| `update(trangThaiXe, thoiDiemHienTai_us, dungPidMo, lenhBanh)` | Kiểm tra đầu vào, tạo timestamp/deadline lệnh. Lần đầu sau reset trả lệnh tốc độ 0. Các lần sau tính dt, cập nhật PI khi đủ 20 ms, gọi PID góc, tích phân/chặn vận tốc rồi đổi sang STEP/s hai bánh |
| `gocDat_rad()` | Trả góc mục tiêu hiện tại để ghi log |
| `heSoPid()` | Chuyển tiếp hệ số từ `_boPidGoc` |

Đầu ra hai bánh gồm thành phần cân bằng chung và thành phần quay có giới hạn. `update()` không gọi GPIO,
không tự enable và không biết mã phiên; mã phiên do RobotApp gắn bên ngoài.

## 10. Kiểm tra an toàn — `src/safety/SafetyMonitor.h/.cpp`

| Hàm | Công dụng |
| --- | --- |
| `check(trangThaiXe, thoiDiemHienTai_us)` | Trả lỗi đầu tiên: góc/norm không hợp lệ, IMU quá hạn/timestamp tương lai, vị trí/vận tốc lỗi, xung quá hạn, hoặc góc dọc/ngang vượt ngưỡng ngã. `None` nghĩa qua các kiểm tra này |
| `canArm(trangThaiXe, thoiDiemHienTai_us)` | Yêu cầu `check()==None`, gia tốc tin cậy, cả góc dọc/ngang nhỏ và cả tốc độ quay dọc/ngang nhỏ. Trả bool; không trực tiếp cấp EN |

`canArm()` chặt hơn `check()`: đang cân bằng có thể nghiêng vừa phải, nhưng lúc bật
phải gần thẳng. Cờ xác nhận phần cứng còn được kiểm tra riêng trong driver motor.

## 11. Driver cặp bánh — `lib/motor_drv/src/MksServo42C.h/.cpp`

Tất cả hàm thay đổi driver được gọi từ cùng MotorTask sau khởi động. Timer giám sát
có ngoại lệ rõ ràng: nó chỉ được tắt EN qua lớp GiamSatDongCo.

| Hàm | Công dụng/kết quả |
| --- | --- |
| `begin()` | Kiểm tra config/chân trùng, đặt EN tắt và DIR tiến, tạo hai MCPWM, xác nhận dải tần, tạo giám sát GPTimer. Không chạy motor |
| `enable()` | Kiểm tra đã khởi tạo, không lỗi, cờ phần cứng true, STEP đã yên. Arm giám sát trước khi bật EN; đặt hạn lệnh và thời gian ổn định EN. Có thể trả `Pending` |
| `disable()` | Tắt EN qua giám sát trước khi ép STEP LOW, xóa mục tiêu/tích lũy/pha/hạn lệnh. Không xóa lỗi đã giữ |
| `isEnabled()` | Đọc trạng thái EN theo phần mềm driver. Sau ISR ngắt, cờ này được đồng bộ khi `service()` chạy; không phải đo phản hồi từ MKS |
| `apply(const WheelStepCommand&)` | Kiểm tra hai mục tiêu, timestamp, hạn lệnh; phục vụ lệnh cũ trước; giới hạn cả cặp theo cùng tỉ lệ, cập nhật mục tiêu và hạn, phục vụ lệnh mới |
| `stopMotion()` | Đặt mục tiêu hai bánh bằng 0, giữ EN. Không gia hạn deadline: vẫn phải gửi lệnh mới nếu muốn tiếp tục giữ hoạt động |
| `service()` | Nhận lỗi giám sát, kiểm tra hạn lệnh/khoảng phục vụ, gọi `advance()` từng bánh, gia hạn heartbeat khi xử lý thành công. Phải gọi đều, kể cả khi tốc độ bằng 0 |
| `emergencyStop()` | Giữ `EmergencyStop` nếu chưa có lỗi, rồi disable |
| `clearFault()` | Chỉ khi disabled: dừng STEP, xóa lỗi giám sát/driver và mốc lệnh; không enable |
| `snapshot()` | Sao chép mục tiêu/tần số liên tục/EN/lỗi/deadline để ghi log. `daBat` loại trừ trường hợp giám sát đã ngắt |
| `advance(Axis&, dtSeconds, nowUs)` | Máy trạng thái một bánh: đổi mode, dừng hoàn chỉnh xung, đổi DIR, chờ setup, phát liên tục hoặc tích lũy xung thưa |
| `requestPause(Axis&)` | Yêu cầu backend ngăn cạnh lên mới, đặt hạn DIR hold, chuyển `Pausing` |
| `writeDirection(Axis&, dangTien)` | Ánh xạ chiều tiến của bánh sang mức GPIO, đặt DIR và hạn setup, chuyển `DirSetup` |
| `trip(MksFault)` | Giữ nguyên lỗi đầu tiên rồi disable. Nếu chính thao tác disable lỗi, có thể ghi `BackendError` |
| `dangChoXuLy()` | Kiểm tra còn chờ pause/setup/start của một trong hai bánh; dùng để trả `Pending` |
| `nowUs()` | Helper thời gian microsecond trong file |

`Axis` giữ trạng thái mỗi bánh. `Phase` là Idle/Running/Pausing/DirSetup;
`Mode` là None/Continuous/Sparse. Hai khái niệm này tách giai đoạn chuyển tín hiệu
với phương pháp phát xung.

## 12. Backend STEP — `lib/motor_drv/src/McpwmStepOutput.h/.cpp`

| Hàm | Công dụng |
| --- | --- |
| Constructor mặc định | Khởi tạo handle/cờ rỗng; không truy cập peripheral |
| Copy constructor, `operator=` bị `delete` | Cấm sao chép đối tượng đang sở hữu tài nguyên/callback |
| `begin(stepPin, pulseHighUs, minLowUs)` | Tính tick, tạo timer/operator/comparator/generator/software-sync, gắn callback stop, cấu hình cạnh HIGH/LOW; giữ STEP LOW |
| `supportsFrequency(hz)` | Kiểm tra chu kỳ nguyên tick chứa đủ HIGH/LOW và nằm trong độ rộng bộ đếm |
| `setFrequency(hz)` | Với 0: yêu cầu pause. Khi đang chạy: đổi period ở biên chu kỳ. Khi phát lại: chuẩn bị, sync tạo cạnh đầu, bật cạnh HIGH ở EMPTY cho chu kỳ tiếp theo |
| `pauseAfterPulse()` | Ngăn cạnh lên tương lai, để HIGH hiện tại kết thúc; tính thời điểm đủ yên. Không chờ bằng delay |
| `isPauseReady()` | True khi đã yêu cầu pause, qua thời gian bảo vệ và không còn single-shot bận |
| `emitOnePulse()` | Phát một HIGH bằng sync, comparator hạ LOW; EMPTY giữ LOW. Yêu cầu dừng timer ở FULL; kể cả task bị ngắt giữa chừng, EMPTY không tạo xung thêm |
| `stop()` | Ép LOW ngay và ngăn cạnh mới; có thể cắt HIGH, nên tầng trên tắt EN trước |
| `actualFrequencyHz()` | Trả tần số cấu hình thực theo tick khi continuous, còn lại trả 0. Đây không phải vận tốc bánh đo được |
| `onTimerStop(..., context)` | ISR khi timer single-shot dừng; xóa cờ bận dưới khóa |
| `oneShotBusy()` | Đọc cờ single-shot bận dưới khóa |
| `setOneShotBusy(busy)` | Ghi cờ bận dưới khóa ở ngữ cảnh task |
| `setEmptyAction(action)` | Đặt hành động generator tại sự kiện timer EMPTY |
| `prepareStart(periodTicks)` | Yêu cầu đã pause, giữ LOW, đổi period, chạy timer và bỏ force để sẵn sàng sync |
| `cleanupInit()` | Dọn handle khi begin thất bại, giữ EN/STEP không chạy. Luôn trả false để trả về lỗi khởi tạo |
| `usToTicks(us, hz)` | Đổi microsecond sang số tick, làm tròn lên |
| `nowUs()` | Helper thời gian microsecond trong file |

Tránh tự gọi backend song song với MksServo42CPair cho cùng chân STEP.

## 13. Đếm xung — `lib/motor_drv/src/WheelPulseCounter.h/.cpp`

| Hàm | Công dụng |
| --- | --- |
| `begin()` | Tạo hai kênh trái/phải sau khi GPIO output đã được motor khởi tạo; lỗi thì giữ chưa sẵn sàng |
| `beginAxis(trucBanh, chanXung, chanChieu, mucTienCao)` | Tạo PCNT có giới hạn ±30000, bật bộ cộng bù tràn, đếm cạnh lên STEP và đảo dấu theo DIR, thêm watchpoint và bắt đầu đếm |
| `readAxis(Axis&)` | Đọc số đếm 32 bit, tính chênh lệch unsigned rồi chuyển về signed 64 bit; cộng vào tổng xung mà không clear counter mỗi mẫu |
| `read()` | Đọc cả hai bánh, gắn timestamp sau đọc, trả `WheelPulseFrame`; nếu lỗi một bánh thì frame không hợp lệ |

Hai bánh được đọc nối tiếp, không phải chụp phần cứng đồng thời tuyệt đối. Sai lệch
thời điểm nhỏ cần xem xét nếu sau này dùng tốc độ cao/điều khiển yaw chính xác.

## 14. Giám sát EN — `lib/motor_drv/src/GiamSatDongCo.h/.cpp`

| Hàm | Công dụng/điều kiện |
| --- | --- |
| `begin(chanEn, mucBatCao, hanPhucVu_us, chuKyKiemTra_us)` | Kiểm tra chân/nhịp, đặt EN tắt, tạo GPTimer 1 MHz ngắt định kỳ. Trả false nếu cấp phát/cấu hình thất bại |
| `bat(hanLenh_us)` | Trong khóa: yêu cầu đã init, chưa bật/chưa lỗi, hạn lệnh ở tương lai; ghi hai deadline rồi mới bật EN |
| `giaHan(hanLenh_us)` | Trong khóa: kiểm tra hạn cũ trước khi cập nhật. Nếu đã hết hạn, tắt EN và giữ lỗi; nếu chưa hết, gia hạn mốc phục vụ nhưng giữ hạn lệnh được truyền vào |
| `tat()` | Trong khóa: EN về tắt, `_dangBat=false`; giữ nguyên `_daNgat` |
| `xoaLoi()` | Chỉ cho xóa cờ lỗi khi `_dangBat=false`; không bật EN |
| `daNgat()` | Đọc cờ lỗi dưới khóa; MotorTask dùng để đồng bộ lỗi watchdog |
| `daKhoiTao()` | Đọc cờ tài nguyên đã sẵn sàng |
| `khiBaoThuc(..., nguCanh)` | Callback GPTimer ISR: khóa, lấy giờ, kiểm tra deadline, ngắt nếu cần. Không đánh thức task hay in log |
| `quaHanTrongKhoa(thoiDiem_us)` | Helper kiểm tra `_dangBat` và một trong hai deadline đã tới. Người gọi phải giữ khóa |
| `ngatTrongKhoa()` | Helper ghi EN tắt bằng GPIO low-level, bỏ trạng thái bật, giữ cờ lỗi. Người gọi phải giữ khóa |

Các hàm ISR được gắn IRAM ở khai báo. Phần mềm không thể bảo đảm thời gian ngắt
khi CPU không phục vụ ISR; xem giới hạn trong tài liệu logic và quy trình đo EN.

## 15. Driver MPU6050 cũ — `lib/imu/src/mpu6050Driver.h/.cpp`

Không được RobotApp hiện tại sử dụng. Giữ cho các chương trình bench cũ.

| Hàm | Công dụng |
| --- | --- |
| `mpu6050Drv(TwoWire&)` | Lưu tham chiếu bus |
| `begin()` | Khởi tạo MPU6050, ID `0x68`, range ±4 g/±500 dps, divider, DLPF, INT và đọc lại config |
| `read(KhungDuLieuImu&)` | Đọc 14 byte, chuyển đổi; nhiệt độ theo công thức MPU6050, khác ICM20602 |
| `hieuChinhConQuay(soMau)` | Thu gyro theo data-ready, trung bình/phương sai, lưu bias khi ổn định |
| `kiemTraSauKhoiDong(BaoCaoKiemTraIcm&)` | Ghi kết quả; `bool` | Kiểm ID, cấu hình, nguồn, ngắt, rồi 32 mẫu data-ready mới hợp lệ trong 1,5 s; không phải self-test MEMS |
| `maLoiI2cCuoi()` / `thanhGhiCuoi()` / `maNhanDang()` | Getter | Cho tầng app/UI in mã bus, thanh ghi cuối và ID đã đọc mà không gây thêm giao dịch |
| `ghiThanhGhi(diaChi, giaTri)` | Ghi một thanh ghi I²C |
| `docNhieuByte(diachiBatDau, duLieu, soByte)` | Đọc burst I²C có kiểm tra lỗi/độ dài |
| `ghepSo16Bit(byteCao, byteThap)` | Helper ghép số signed 16 bit riêng trong file |

## 16. Các chương trình bench cũ trong `test/`

Các file này là những chương trình `setup()/loop()` độc lập để đối chiếu, không
phải tất cả cùng được biên dịch vào firmware chính. Bench có MPU6050 phải được
chuyển đúng driver/config nếu dùng ICM20602. Không ghép nhiều `setup()/loop()` cùng lúc.

### `test/test_mpu6050Driver.cpp`

| Hàm | Công dụng |
| --- | --- |
| `setup()` | Mở Serial, khởi tạo driver MPU và báo kết quả |
| `loop()` | Đọc mẫu, đo thời gian đọc, gắn timestamp/số mẫu, in gia tốc/gyro/norm hoặc lỗi |

### `test/stepMeansure.cpp`

| Hàm | Công dụng |
| --- | --- |
| `setup()` | Luôn đặt EN disable, cấu hình DIR và hai backend STEP, in menu |
| `loop()` | Nhận lệnh phát xung liên tục/một xung/pause/stop/đổi tần số; tự dừng khi hết cửa sổ thử |
| `stopAllSignals()` | Ép dừng cả hai STEP, xóa cờ thử và báo lỗi API nếu có |

Bench này gọi backend trực tiếp khi EN tắt; không đi qua PID hoặc giám sát EN của MKS pair.

### Các hàm chung của `test/test_MPU_MKS.cpp` và `test/main_test.cpp`

| Hàm | Công dụng |
| --- | --- |
| `layThoiGian_us()` | Lấy giờ microsecond |
| `docThanhGhi(reg, value)` | Đọc một byte thanh ghi IMU bằng I²C |
| `soSanhThanhGhi(name, reg, expected, mask)` | Đọc, áp mask, đối chiếu giá trị dự kiến và in kết quả |
| `kiemTraCauHinhMpu()` | Đối chiếu ID/divider/range/lọc/ngắt với config |
| `inQuyDoiMotor(name, steps, microstep, ratio)` | In quy đổi xung/RPM từ thông số nhập tay; không đọc menu MKS |
| `inCauHinhMotor()` | In chân, mức logic, timing, vi bước và các giới hạn |
| `tenKetQua(MksResult)` | Chuyển kết quả driver thành chữ |
| `tenLoi(MksFault)` | Chuyển lỗi driver thành chữ; nhánh watchdog mới cần cập nhật nếu tái sử dụng bench |
| `onImuDataReady()` | ISR chụp timestamp/số mẫu và đánh thức task MPU |
| `taskMpu(void*)` | Đọc theo ngắt, thống kê mất mẫu/độ trễ/I²C, gửi snapshot; bản bench này dừng task khi lỗi |
| `goiTaskMotor(void*)` | Callback timer đánh thức task motor |
| `taskMotor(void*)` | Nhận lệnh thử tốc độ, kiểm tra IMU, áp lệnh định kỳ, phục vụ motor, thử timeout, gửi thống kê |
| `inHuongDan()` | In các phím thử một bánh/hai bánh, đảo dấu, dừng và thử timeout |
| `setup()` | Khởi tạo motor/MPU, readback/hiệu chuẩn, queue/task/ISR/timer, in READY |
| `loop()` | Đọc phím Serial, gửi lệnh thử có hạn, in thống kê IMU/motor |

### Các hàm bổ sung riêng trong `test/main_test.cpp`

| Hàm | Công dụng |
| --- | --- |
| `quetDiaChiI2c()` | Quét bus lúc khởi tạo, lưu kết quả tìm thiết bị |
| `inTrangThaiI2c()` | In kết quả quét/bus đã lưu để chẩn đoán |
| `ghiNhanGiaiDoanKhoiDong(tenGiaiDoan)` | Lưu và in bước setup hiện tại |
| `ghiNhanLoiKhoiDong(tenLoi)` | Lưu và in nguyên nhân setup thất bại |

## 17. Kiểm thử native trên PC

### `test/native/control_tests.cpp`

| Hàm | Công dụng |
| --- | --- |
| `check(dieuKien, moTa)` | Tăng số kiểm tra, báo và dừng chương trình khi sai |
| `near(giaTriThuNhat, giaTriThuHai, saiSoChoPhep)` | So sánh số thực có dung sai |
| `imu(thoiDiemMau_us, gocNghien_rad, tocDoNghien_radps)` | Tạo mẫu IMU lý tưởng theo góc/gyro để test |
| `validState(thoiDiemMau_us)` | Tạo trạng thái xe đứng thẳng, cờ/norm/timestamp hợp lệ |
| `odometryTests()` | Kiểm tra đơn vị, tiến/lùi, xung thưa, quay tại chỗ, timestamp và khoảng mẫu |
| `estimatorTests()` | Kiểm tra góc/dấu/radian, quay, NaN, mẫu trùng và phục hồi sau khoảng mất mẫu |
| `safetyTests()` | Kiểm tra arm, ngã, deadline, timestamp tương lai và vận tốc vô hạn |
| `kiemTraTuTheVaDuLieuLoi()` | Tái hiện nghiêng ngang ±85°, toàn 0 sau init, gia tốc không tin cậy và quay ngang lúc arm |
| `kiemTraPhienVaThuTuMau()` | Kiểm tra ngắt trùng/quay vòng; từ chối lệnh cũ/nonzero, tắt-bật nhanh, reset controller và hết hạn chờ |
| `controllerTests()` | Kiểm tra dấu đầu ra, miền hệ số mờ, trần gia tốc/vận tốc, dt lỗi và reset |
| `plantTest(dungPidMo)` | Mô phỏng con lắc lý tưởng với vòng góc cố định/mờ |
| `cascadePlantTest(dungPidMo)` | Mô phỏng điều khiển tầng với odometry lượng tử hóa theo xung |
| `main()` | Chạy toàn bộ nhóm trên và in PASS/số kiểm tra |

### `test/native/watchdog_tests.cpp` và `stubs/`

| Hàm | Công dụng |
| --- | --- |
| `kiemTra(dieuKien, moTa)` | Assertion đơn giản cho bộ test watchdog |
| `esp_timer_get_time()` giả | Trả đồng hồ `thoiDiemGia_us` do test điều khiển |
| `gpio_set_level()` giả | Ghi mức vào mảng `GPIO.mucChan` |
| `gpio_set_direction()` giả | Trả thành công, không có GPIO vật lý |
| `gpio_ll_set_level()` giả trong header | Đường ghi EN được mã ISR thật gọi, ghi vào cùng mảng |
| `gptimer_new_timer()` giả | Cấp đối tượng timer giả hoặc chủ động trả lỗi |
| `gptimer_del_timer()` giả | Giải phóng timer giả |
| `gptimer_register_event_callbacks()` giả | Lưu callback và con trỏ đối tượng để test gọi ISR thật |
| `gptimer_set_alarm_action()` giả | Nhận cấu hình và trả thành công; không tạo nhịp tự chạy |
| `gptimer_enable()/disable()/start()` giả | Trả thành công; tác dụng thời gian do test điều khiển |
| `chayNgatTai(thoiDiem_us)` | Đặt đồng hồ rồi gọi callback `GiamSatDongCo::khiBaoThuc()` đã đăng ký |
| `main()` | Thử task ngừng, feed muộn, deadline lệnh, clear khi bật/tắt, hai cực tính EN và lỗi init |

Các macro critical-section trong stub là no-op: bộ test duyệt thứ tự sự kiện tuần
tự, không chứng minh lịch FreeRTOS hoặc độ trễ ISR trên chip. Mã GiamSatDongCo được
biên dịch trực tiếp vào test, không viết lại thuật toán watchdog riêng để test chính nó.

`test/run_native.ps1` không định nghĩa hàm phụ: nó tìm compiler, tạm thêm thư mục
compiler vào PATH, biên dịch/chạy sáu executable, kiểm tra exit code và phục hồi
PATH/thư mục trong `finally`. Kết quả nằm trong `.cache/`, không nạp lên ESP32.

## 18. Hàm bổ sung cho lọc/di chuyển

| Hàm | Công dụng |
| --- | --- |
| `LocThongThap::datLai()` | Xóa trạng thái PT1; mẫu đầu kế tiếp khởi tạo trực tiếp từ giá trị đo |
| `LocThongThap::capNhat(giaTri, buocThoiGian_s, tanSoDanhNghia_hz)` | PT1 với dt thực, kiểm NaN/Inf/tham số, cutoff0 bypass |
| `LocThongThap::giaTri()` | Lấy mẫu lọc hiện tại để báo cáo |
| `AnglePidController(PidGains)` | Cho hệ số cơ sở có thể truyền vào, mặc định từ config; dùng để test Ki khác0 |
| `AnglePidController::tocDoDaLoc_radps()` | Gyro sau PT1 được dùng cho D và luật mờ |
| `CascadeController::vanTocMucTieu_mps()` | Mục tiêu tiến/lùi sau giới hạn và ramp |
| `CascadeController::tocDoQuayMucTieu_radps()` | Mục tiêu quay sau giới hạn và ramp |
| `CascadeController::tocDoDaLoc_radps()` | Chuyển tiếp gyro D để log |
| `LenhDiChuyen::hopLe(maPhienHienTai, hienTai_us)` | Kiểm tra mã phiên, số hữu hạn, timestamp và lease tối đa500ms |
| `tronUuTienCanBang(v, yaw, B, vmax)` | Trộn hai bánh; clamp phần quay vào dư địa, giữ trung bình bằng v cân bằng |

`AnglePidController::update()` hiện nhận `int huongBaoHoaVanToc` (-1/0/+1) thay cho
bool để cho xả tích phân đúng hướng. `scheduledGains()` nhận thêm hệ số cơ sở tùy
chọn. `CascadeController::update()` nhận thêm hai mục tiêu vận tốc/quay tùy chọn;
app truyền mục tiêu từ queue di chuyển còn hạn, hoặc0 khi hết hạn/khác phiên.
`filter_motion_tests.cpp` có helper `kiemTra()`, `taoMau()` và `main()` kiểm PT1,
innovation, anti-windup Ki20, mixer, lease và ramp. Script native chạy thêm nhóm này.

`tools/tinh_toan_loc_va_motor.py`: `gia_tri()` đọc hằng config, `dap_ung()` tính
H(z), `ghi_csv()` ghi bảng, `main()` tính và xuất JSON/CSV/Markdown.

## 19. Hệ trục và hiệu chuẩn góc gá

| Hàm | Công dụng |
| --- | --- |
| `HeTrucImu()` | Tính/lấy R một lần từ ImuMountConfig; kiểm tra ma trận |
| `HeTrucImu(MaTranQuayImu)` | Nhận R tùy chọn và kiểm tra, dùng cho test/ứng dụng |
| `HeTrucImu::tuGocLap_deg()` | Tạo Rz(yaw)Ry(pitch)Rx(roll) sensor→xe từ ba góc độ |
| `HeTrucImu::kiemTraMaTran()` | Kiểm tra hữu hạn, trực giao và det≈+1; loại phản xạ/shear |
| `HeTrucImu::hopLe()` / `maTran()` | Lấy cờ hợp lệ/ma trận đã chuẩn bị |
| `HeTrucImu::chuyenSangXe()` | Nhân R với một vector sensor; trả NaN nếu đầu vào/hệ trục lỗi |
| `HeTrucImu::tuHaiVector()` | Phương pháp offline: dựng R từ gravity ở tư thế chuẩn thẳng đã biết và gyro nghiêng TIẾN; kiểm norm/gyro/độ nhiễm trục đứng. UI hai mốc ĐẦU/ĐUÔI không dùng hàm này |
| `tichVoHuong()` / `tichCoHuong()` | Phép dot/cross dùng dựng cơ sở và truyền vector phương đứng |
| `vectoHuuHan()` / `doLonVecto()` / `chuanHoaVecto()` | Kiểm tra, norm và chuẩn hóa; từ chối vector rỗng/không hữu hạn |
| `ComplementaryEstimator(HeTrucImu)` | Nhận phép quay; mặc định lấy config |
| `ComplementaryEstimator::heTrucHopLe()` | Cho RobotApp kiểm tra ma trận trước khi khởi tạo các phần cứng còn lại |

`ComplementaryEstimator::update()` nay xoay cả accel/gyro, lưu vector `_huongLen`,
truyền vector bằng Rodrigues(-omega*dt), rồi hiệu chỉnh theo góc giữa hai vector.
Pitch Euler và pitch rate dùng đủ thành phần body cần thiết; `innov` là góc vector
không âm. `reset()` vẫn chỉ reset estimator, không tạo lại ma trận lắp hoặc zero khung xe.

`tools/hieu_chuan_lap_imu.py`: `tich_vo_huong/tich_co_huong/do_lon/chuan_hoa` là các
phép vector; `tao_ma_tran` dựng cơ sở; `doc_mau` đọc RAW_IMU/CSV; `tu_log` kiểm tra
chất lượng hai tập mẫu; `mau_config` tạo đoạn config; `main` xử lý CLI và xuất file.
`tools/tinh_toan_lech_lap_imu.py`: `ma_tran_euler`, `sang_sensor`, `sang_xe` tạo ví
dụ lý tưởng; `main` xuất bảng sai số và JSON minh họa, không ghi vào config xe.

## 20. UI hiệu chuẩn — `src/calib/UI_Calib.*`

`UI_Calib` chạy đồng bộ trong setup khi EN tắt, trước ImuTask/ControlTask/MotorTask.
Ma trận kết quả đi vào estimator trong RAM; chưa lưu NVS. Hướng dẫn thao tác và
giới hạn tư thế nằm trong [UI_Calib](HUONG_DAN_UI_CALIB.md) và
[báo cáo flow](DOI_CHIEU_SO_DO_VA_FLOW_CALIB.md).

| Hàm | Công dụng và kết quả |
| --- | --- |
| `UI_Calib(Icm20602Driver&)` | Giữ tham chiếu tới driver đang dùng, không tạo bus mới |
| `khoiTao()` | Cấu hình nút GPIO3 theo cực tính, đồng bộ trạng thái nút và sáng cam trong khi kiểm tra boot |
| `hienMau(doLed, xanhLa, xanhDuong)` | Gửi màu tới RGB với thứ tự GRB/RGB cấu hình; bỏ qua nếu chân<0 |
| `baoSanSang()` / `baoLoi()` / `baoChoDuLieu()` | Sáng xanh lá/đỏ/cam; báo trạng thái, không điều khiển EN hoặc đọc bus. Sau calib app dùng các hàm này để phản ánh dữ liệu runtime |
| `dongBoNut()` | Lấy mức hiện tại, xóa lần nhấn đang chờ; tránh dùng lại lần nhấn ở bước trước |
| `daXacNhan()` | Chống dội35ms, chỉ trả true sau một lượt nhấn rồi nhả ổn định |
| `choXacNhan()` | Đồng bộ và chờ lượt nhấn–nhả; nhường CPU bằng delay5ms; không có hạn chờ |
| `layGiaTocTinh(Vec3Imu&, buoc)` | Chờ 500 ms, tìm khoảng 80 mẫu ổn định trong 30 s; rung chỉ xóa khoảng đo và tự thu lại. In tên mốc, kiểm cả phương sai accel/gyro, trả accel trung bình |
| `kiemTraLoiDoc(mau, lanCoMau_ms, buoc)` | In lỗi I²C hoặc mất mẫu mới 500 ms, trả false để dừng phép đo; chưa có mẫu ngắn hạn vẫn được chờ |
| `choVeThang(HeTrucImu)` | LED trắng, tính pitch/roll đã bù; báo CALIB_RETURN theo rad và độ mỗi500ms; chờ80 mẫu tĩnh liên tiếp với sai số pitch so với `gocCanBang_rad`≤0,10rad và roll≤0,10rad; timeout30s |
| `thuMotLuot(MaTranQuayImu&)` | Đo bias, mốc tham chiếu, mốc ĐẦU nằm, lượt quay qua đứng sang ĐUÔI nằm, mốc sau, dựng/in ma trận và kiểm tư thế khởi động; false nếu một bước không đạt |
| `chay(MaTranQuayImu&)` | Chờ xác nhận tham chiếu ở xanh dương; chạy thử một lượt; khi lỗi sáng đỏ, chờ người dùng đặt lại tư thế rồi nhấn–nhả và làm lại |
| `dangNhanNut()` | Helper đọc mức GPIO theo `nutCalibMucThap` |
| `giaTocCua()` / `gyroCua()` | Helper lấy vector3 thành phần từ frame của driver |
| `mauDungYen()` | Kiểm đã trừ bias, norm accel cách9,80665 không quá0,5m/s² và norm gyro≤0,06rad/s |

Tên `choVeThang()` được giữ trong source; điều kiện thực là gần góc đặt khởi động.
Hàm không xác định cân bằng cơ học. Sau READY, nút không được poll tiếp; dùng
Serial `a` để yêu cầu bật cân bằng qua toàn bộ điều kiện an toàn.

## 21. Dựng trục từ lượt ĐẦU/ĐUÔI — `HieuChuanNghiengDoc.*`

| Hàm / kiểu | Công dụng và kết quả |
| --- | --- |
| `themMau(gyro, khoangThoiGian_s)` | Nhận gyro XYZ đã trừ bias; giữ mẫu tốc độ0,07..2,5rad/s và0<dt≤20ms; cộng moment `gyro·gyroᵀ·dt`, vector góc tích phân và số mẫu |
| `tinhMaTran(giaTocDung, giaTocTruoc, giaTocSau, ketQua, baoCao)` | Kiểm ba mốc; tìm Y chính bằng12 bước power iteration, năng lượng≥95%; chọn Y ngược tổng góc trước→sau. Dựng X từ hiệu gia tốc sau−trước đã chiếu vuông gócY, Z=X×Y. Kiểm ma trận, tư thế, góc gyro và vector hai mốc. Chỉ ghi ma trận khi đạt; tham số báo cáo tùy chọn |
| `BaoCaoCalibDoc` | Chứa `lyDo`, góc mốc trước/sau, góc tích phân gyro, tỷ lệ năng lượng trục và sai lệch vector; dùng để giải thích lý do lỗi/PASS |
| `ketThuc` | Lambda đặt lý do và sao chép báo cáo chẩn đoán ra ngoài trước khi trả true/false |
| `nhanMoment` | Lambda nhân moment3×3 với vector trong power iteration; không phải phép quay sensor→xe |
| `cacChanChucNangKhongTrung()` | Helper constexpr trong `PinConfig.h`, kiểm GPIO của IMU/motor/nút/RGB; bỏ qua chân âm, static_assert khi trùng |

Mục tiêu người dùng là đặt nằm80–90° ở phía ĐẦU và ĐUÔI. Cửa sổ kiểm tra trong
`CalibConfig.h` hiện 80–92° theo **góc sau fit**, dư 2° ở phía 90° cho nhiễu/độ phẳng;
không xác nhận góc thật và không phải tuyên bố độ chính xác. Góc gyro và so khớp vector cần sai lệch≤0,15rad.
Phép quay Rodrigues kiểm hai vector ngay cả khi tổng góc gần180°.

Hai mốc chuẩn giúp giữ pitch ban đầu; gravity lúc dựng chỉ kiểm hướng lên,
không ép pitch=0. Nếu cả hai góc thật thuộc 80–90° nhưng không đối xứng, mốc có thể lệch
khoảng ±5° mà hai góc fit vẫn có vẻ đối xứng. Cần mặt đỡ cơ khí chuẩn và kiểm góc
độc lập sau calib. `test/native/calib_pitch_tests.cpp` đã PASS 2.038 kiểm tra ngày
08/11/2026; nhóm boot/UI mới PASS 44 kiểm tra. Sáu nhóm native và build đều PASS;
chưa upload/thử xe thật. Đây là kết quả riêng cho luồng mới, không dùng PASS của
luồng roll cũ làm bằng chứng.

Xem [hướng dẫn thao tác](HUONG_DAN_UI_CALIB.md) và
[báo cáo hình học](CALIB_DAU_DUOI_VA_KIEM_CHUNG_GOC_2026-11-08.md).

## 22. Thống kê dữ liệu calib — `lib/imu/src/ThongKeMauImu.h`

| Hàm | Công dụng |
| --- | --- |
| `mauImuHuuHan()` | Kiểm trạng thái frame và tính hữu hạn của accel/gyro/nhiệt độ |
| `normGiaTocImu()` | Tính độ lớn vector gia tốc |
| `mauImuGanTinh(mau, gioiHanGyro_radps)` | Kiểm norm gia tốc gần g và norm gyro theo giới hạn trước/sau bias |
| `ThongKeMauImu::xoa()` | Xóa toàn bộ số mẫu, trung bình và M2 của khoảng đang thu |
| `themMau()` | Welford sáu thành phần, không lưu lịch sử mảng mẫu |
| `soMau()` / `trungBinh(truc)` | Số mẫu và trung bình; 0–2 accel, 3–5 gyro rad/s |
| `phuongSaiGiaToc()` | Tổng phương sai mẫu của accel XYZ; vô hạn nếu chưa đủ hai mẫu |
| `doLechChuanGyroLonNhat_dps()` | Độ lệch chuẩn lớn nhất của ba trục, đổi sang độ/s |
| `onDinh()` | Đủ hai mẫu, phương sai accel và độ lệch chuẩn gyro đạt ngưỡng; bên gọi kiểm thêm đủ 80/500 mẫu |

Xem [báo cáo sửa calib và kiểm tra boot](SUA_LOI_CALIB_VA_KIEM_TRA_ICM_KHOI_DONG.md).

## 23. API thư viện thường gặp khi đọc source

| API | Cách hiểu trong chương trình |
| --- | --- |
| `xQueueCreate(1, sizeof(...))` | Tạo hộp chứa một bản dữ liệu có kích thước cố định |
| `xQueueOverwrite()` | Gửi bản mới, thay bản cũ nếu chưa ai lấy |
| `xQueueReceive()` | Lấy và bỏ bản trong queue; có thể chờ |
| `xQueuePeek()` | Sao chép bản hiện có mà giữ nó trong queue |
| `vTaskNotifyGiveFromISR()` / `ulTaskNotifyTake()` | ISR báo có mẫu; task nhận số thông báo tích lũy |
| `xTaskNotify(..., eSetBits)` / `xTaskNotifyWait()` | Gửi/nhận các bit sự kiện motor; cùng lượt ưu tiên dừng |
| `portENTER/EXIT_CRITICAL[_ISR]()` | Khóa ngắn bảo vệ dữ liệu chung; cặp ISR dùng trong ngắt |
| `esp_timer_get_time()` | Thời gian đơn điệu microsecond kể từ boot |
| `Wire.beginTransmission/write/endTransmission/requestFrom/read` | Các bước gửi địa chỉ thanh ghi và đọc/ghi I²C |
| `std::clamp()` | Chặn giá trị vào một khoảng |
| `std::isfinite()` | Loại NaN/vô hạn trước khi tính điều khiển |
| `std::atan2()` | Tính góc có xét dấu/hai thành phần vector |

Các hàm này thuộc Arduino/FreeRTOS/ESP-IDF/C++; giữ tên theo thư viện. Không đổi tên
field như `gptimer_config_t::resolution_hz` vì chúng là giao diện của SDK.

## 24. Chốt mẫu và chẩn đoán runtime — bản 09/11/2026

| Thành phần | Công dụng và giới hạn |
| --- | --- |
| `phienBanFirmware` trong `ProjectConfig.h` | Hằng chuỗi `imuAck-20261109`, được in ở setup, phím `h` và trường `fw` để đối chiếu bản đã nạp |
| `ChanDoanLuongImu` trong `SensorFrame.h` | Snapshot số ngắt/thời điểm ngắt cuối, lần chốt tốt cuối, tổng mẫu tốt, timeout/I²C/status rỗng/ACK phục hồi/đọc trễ/overlap và trạng thái đọc cuối. Có bản trong frame và queue chẩn đoán riêng dùng cho console |
| `chotMauImu(mau, coThongBao, soNgatTruoc, soNgatSau, thoiDiemNgat_us, lucDocXong_us, chuKyMau_us)` | Chốt frame HopLe có thông báo và timestamp nguồn khác 0, sequence không đổi trong lúc đọc, thời gian hoàn thành trước một chu kỳ mẫu. Sequence 0 sau quay vòng vẫn hợp lệ nếu có timestamp nguồn. Xóa frame thành DuLieuCu khi không đạt; không dùng mẫu poll như góc mới |
| `KetQuaChotMauImu::ChapNhan` | Mẫu đã có ngắt nguồn/timestamp nhất quán để chuyển vào estimator |
| `KhongCoMau` | Frame chưa HopLe, giữ nguyên trạng thái để phân biệt DuLieuCu/LoiIO |
| `ThieuNgat` | Thiếu thông báo hoặc timestamp nguồn; ACK có thể thành công nhưng không được dùng như mẫu runtime mới |
| `NgatChong` | Sequence thay đổi khi đọc; ImuTask tăng overlap và bỏ mẫu không chắc timestamp |
| `QuaChuKyMau` | Chu kỳ không hợp lệ, timestamp ngắt ở tương lai hoặc đọc hoàn tất cách ngắt ít nhất một chu kỳ; 5 ms ở 200 Hz. Tăng imuLate và bỏ mẫu |
| `tuoiMs` trong `RobotApp::loop()` | Lambda tính tuổi các mốc microsecond để in log; trả −1 khi không có mốc hợp lệ cho phép tính tuổi |

`imuLoop()` là nơi sở hữu bus sau khi tạo task. Đánh thức ban đầu sau attach IRQ
chỉ giúp ACK cờ đang giữ mức trước lúc gắn ISR; không tự tạo thời điểm mẫu. Khi
hết chờ 20 ms, ImuTask thử ACK để giải phóng INT sau một lần đọc lỗi. Kể cả nhận
được byte mẫu mới bằng poll, `chotMauImu()` vẫn từ chối nếu thiếu thông báo ngắt.
Vì INT giữ mức, thanh ghi cảm biến vẫn có thể cập nhật khi chưa có cạnh ngắt
kế tiếp. Guard đọc trước một chu kỳ (5 ms ở 200 Hz) tránh gán dữ liệu mới hơn
cho timestamp IRQ cũ. Đây là kiểm tra tính nhất quán mẫu, không nới giới hạn
an toàn 20 ms; timeout I²C vẫn 2 ms.

App hiện có năm queue: IMU, motor, điều khiển, di chuyển và chẩn đoán IMU.
`_hangDoiChanDoanImu` được công bố trước khi bỏ frame overlap/đọc trễ. Console
lấy thống kê từ queue này, nên không phải chờ có frame góc được chốt mới thấy
lỗi tăng. `ready` được kiểm tra lại tại thời điểm in; `angleValid` bao gồm tuổi
góc dưới 20 ms, không chỉ chép cờ gocHopLe cũ.

`imuI2c` là số lỗi, `i2cLast`/`regLast` giữ lỗi gần nhất sau khi bus phục hồi.
`biasOk` là cờ trong frame đang công bố, không phải phép đo bias độc lập; frame
lỗi thường có cờ 0 dù bias đã đo trước đó chưa bị xóa. Số góc còn lưu khi
`angleValid=0` không được xem là tư thế hiện tại.

LED sau calib được app cập nhật theo dữ liệu runtime, không theo một cờ “đã calib”
cố định: cam chưa có báo cáo, xanh khi kiểm tra an toàn hiện tại và lỗi đều đạt,
đỏ khi có lỗi/quá hạn. Xanh không đồng nghĩa `canArm()` hay cấu hình motor đã xác
nhận. Xem [chẩn đoán runtime](CHAN_DOAN_IMU_RUNTIME_2026-11-09.md).
