# Hướng dẫn sử dụng xe cân bằng

> Bản có `UI_Calib` dùng GPIO3 cho nút xác nhận, SDA GPIO10 và SCL GPIO11. Motor 1 phải, motor 2 trái theo xác nhận của người dùng. Thực hiện các bước theo LED trước khi gửi `a`. [Hướng dẫn UI_Calib](HUONG_DAN_UI_CALIB.md) · [Sơ đồ và luồng calib](DOI_CHIEU_SO_DO_VA_FLOW_CALIB.md).

> **05/11/2026:** UI đang đo ma trận lắp mỗi lần boot. Khi tắt UI, nhập ma trận vào `ImuMountConfig.h`; xem [phương pháp offline](BAO_CAO_BU_LECH_LAP_IMU.md).
> Gửi `i` khi motor tắt để bật/tắt dòng RAW_IMU5Hz. Đầu ra này ở hệ sensor để hiệu chuẩn.

Cập nhật theo mã nguồn ngày **05/11/2026** (Asia/Saigon).

Đợt cập nhật lọc/di chuyển: xem [báo cáo ICM20602](BAO_CAO_ICM20602_LOC_NHIEU.md)
và [báo cáo 42C/17HS4401S](BAO_CAO_MKS42C_17HS4401S.md). Trọng tâm đã cập nhật
0,025 m từ trục bánh theo người dùng; nguồn motor vẫn cần xác nhận.

Tài liệu này hướng dẫn cấu hình, biên dịch, nạp, vận hành và đọc log của firmware
ESP32-S3 + ICM20602 + hai MKS Servo42C. Những bước đo trên xe dưới đây là quy trình
cần thực hiện; tài liệu không xác nhận phần cứng đã vượt qua các phép đo đó.

## 1. Phạm vi và nguyên lý vận hành

Firmware đọc ICM20602, ước lượng góc thân xe, dùng PID mờ giữ góc và PI vận tốc.
ESP32 phát STEP/DIR; Servo42C ở chế độ `CR_vFOC` tự khép vòng bằng encoder nội bộ.
PCNT của ESP32 đếm cạnh STEP có dấu theo DIR để ước lượng quãng đường/vận tốc bánh.
Ước lượng này giả thiết motor bám xung, tỉ số truyền đúng và bánh không trượt.

Giao diện sử dụng hiện tại là Serial. Mặc định vận tốc đặt bằng 0; chương trình
đã có lệnh thử tiến/lùi/quay qua Serial, chưa có điều khiển từ xa hoặc giữ vị trí/
hướng tuyệt đối. Các phím bench cũ có nghĩa khác các phím di chuyển hiện tại.

Để hiểu cách chương trình hoạt động hoặc tự sửa:

- [Logic toàn bộ chương trình](LOGIC_CHUONG_TRINH.md): luồng mẫu, task, phiên bật và bộ điều khiển.
- [Danh mục các hàm](DANH_MUC_HAM.md): công dụng, đầu vào/đầu ra và nơi gọi.
- [Học lập trình từng bước](HUONG_DAN_LAP_TRINH.md): tên biến, đọc I²C, đơn vị và bộ lọc.
- [Sửa đổi và kiểm thử](HUONG_DAN_SUA_VA_KIEM_THU.md): thay chức năng ở đâu và kiểm tra như thế nào.
- [Checklist tính năng](CHECKLIST_TINH_NANG.md): phân biệt đã có mã, đã test và đã đo trên xe.
- [Kết quả rà soát 03/11/2026](KET_QUA_RA_SOAT_2026-11-03.md): bằng chứng kiểm tra và các điểm còn mở.

## 2. Đối chiếu board và chân kết nối

Các số dưới đây là **số GPIO của ESP32-S3**, không phải thứ tự chân vật lý trên
hai hàng chân của board. Đối chiếu đúng sơ đồ Super Mini đang dùng.

| Tín hiệu | Biến trong `PinConfig.h` | GPIO cấu hình |
| --- | --- | --- |
| ICM20602 SDA | `chanSdaMpu` | GPIO 10 |
| ICM20602 SCL | `chanSclMpu` | GPIO 11 |
| ICM20602 SA0 | `chanSa0Icm` | GPIO 8; LOW cho địa chỉ `0x68` |
| ICM20602 CS | `chanCsIcm` | GPIO 9; HIGH để dùng I²C |
| Nút xác nhận | `chanNutCalib` | GPIO 3 |
| LED RGB địa chỉ | `chanRgbCalib` | GPIO 48/GRB mặc định, chưa xác minh trên board |
| ICM20602 INT | `chanNgatMpu` | GPIO 7 |
| STEP bánh trái, motor 2 / ST2 | `chanStepTrai` | GPIO 6 |
| DIR bánh trái, motor 2 / DIR2 | `chanDirTrai` | GPIO 5 |
| STEP bánh phải, motor 1 / ST1 | `chanStepPhai` | GPIO 13 |
| DIR bánh phải, motor 1 / DIR1 | `chanDirPhai` | GPIO 12 |
| EN dùng chung hai driver | `chanBatMotor` | GPIO 2 |

Header 4 chân trong sơ đồ lần lượt là IO1, IO3, IO4, GND. Code giả định nút
nhấn nối GPIO3 xuống GND và dùng pull-up nội; sơ đồ được gửi chưa thể hiện chính
mạch nút hoặc chân LED RGB. Driver đặt CS/SA0 trước khi khởi tạo bus I²C.
`PinConfig.h` có kiểm tra trùng GPIO ngay khi biên dịch.

Mức điện, cấp nguồn, GND và COM phải đấu theo manual đúng phiên bản module/driver.
Bảng này chỉ mô tả ánh xạ tín hiệu trong code, không thay thế sơ đồ điện của board.
Không suy mức điện nguồn từ số GPIO hoặc từ tên chân COM.

`platformio.ini` hiện dùng môi trường `lolin_s3_mini`, board `lolin_s3_mini`.
Đây là cấu hình build đang có; cần đối chiếu flash/PSRAM/chân của ESP32-S3 Super Mini
thực tế trước khi nạp. Dự án dùng Arduino trên nền tảng pioarduino, có API MCPWM,
PCNT và GPTimer của ESP-IDF.

## 3. Những mặc định cần biết

| Thiết lập | Giá trị hiện tại | Ý nghĩa khi sử dụng |
| --- | --- | --- |
| `daXacNhanCauHinhPhanCungMotor` | `false` | Driver phần mềm từ chối bật motor cho đến khi cấu hình phần cứng được xác nhận |
| `mucBatMotorCao` | `false` | EN active LOW, cần khớp menu `En = L` của MKS |
| DIR tiến trái/phải | HIGH / LOW | Giá trị ban đầu cần kiểm tra với cách lắp hai motor đối xứng |
| Motor / vi bước đầu vào | 200 bước/vòng / 32 | 6400 STEP/vòng motor; tỉ số truyền đang là 1:1 |
| Bán kính bánh | 0,04 m | Dùng quy đổi xung sang quãng đường/vận tốc |
| I²C / IMU | 100 kHz / 200 Hz | Địa chỉ I²C `0x68`; ID ICM20602 phải là `0x12` |
| Hiệu chuẩn gyro | 500 mẫu | Giữ xe yên lúc khởi động; riêng thời gian thu mẫu lý tưởng khoảng 2,5 giây |
| Serial | 115200 baud | Nhận phím và in log |
| `batPidMo` | `true` | Mặc định dùng luật mờ; có thể đổi bằng phím `f` khi tắt motor |
| Kp/Ki/Kd góc | 30 / 0 / 3 | Giá trị khởi đầu; Ki=0 nghĩa đang vận hành PD có điều chỉnh mờ |
| Vận tốc đặt | 0 m/s | Vòng vận tốc cố gắng hạn chế xe trôi |
| Trần STEP | 10000 STEP/s | Khoảng 93,75 RPM, 0,393 m/s theo cơ khí cấu hình; đây là trần lệnh |

Hai biến `daXacNhanCauHinhPhanCungMotor=false` và `mucBatMotorCao=false` có ý nghĩa
khác nhau: biến thứ nhất là điều kiện cho phép trong phần mềm; biến thứ hai là
cực tính điện của EN. Khởi động chương trình luôn đưa EN về trạng thái tắt.

Các thông số HIGH/LOW STEP, DIR hold/setup và thời gian ổn định EN trong
`MotorConfig.h` đang được ghi rõ là giá trị thử. Cần đối chiếu tài liệu/đo thực tế.

## 4. Biên dịch và chạy kiểm thử trên máy

Mở terminal PowerShell ở thư mục dự án:

```powershell
pio run -e lolin_s3_mini
./test/run_native.ps1 -TrinhBienDich g++
```

Nếu `pio` hoặc GCC chưa có trong PATH, có thể dùng các đường dẫn đã cài trên máy
hiện tại:

```powershell
$duongDanPio = 'C:\Users\ADMIN\.platformio\penv\Scripts\pio.exe'
& $duongDanPio run -e lolin_s3_mini
./test/run_native.ps1 -TrinhBienDich C:\msys64\ucrt64\bin\g++.exe
```

`pio run` tạo firmware trong `.pio/build/lolin_s3_mini/`; lệnh này chỉ biên dịch.
Script native tạo/chạy các kiểm thử trên PC trong `.cache/`. Mô hình lý tưởng và
driver giả giúp kiểm tra logic, còn timing và tín hiệu điện phải đo trên ESP32-S3.

## 5. Nạp firmware và mở Serial

1. Giữ cờ xác nhận phần cứng ở `false` trong lần nạp/quan sát IMU đầu tiên.
2. Kết nối board với máy tính, xác định cổng bằng `pio device list`.
3. Thay `COM7` trong các lệnh bên dưới bằng cổng thực tế.
4. Nạp firmware; sau khi nạp, mở monitor và reset board nếu cần xem lại log khởi động.

```powershell
pio device list
pio run -e lolin_s3_mini -t upload --upload-port COM7
pio device monitor --port COM7 --baud 115200
```

Với đường dẫn đầy đủ, dùng `& $duongDanPio` thay cho `pio`. Đóng monitor đang giữ
cổng nếu nạp bị báo cổng bận. `Ctrl+C` thoát monitor; cách đưa board vào chế độ nạp
phụ thuộc board thực tế và cần theo hướng dẫn của board đó.

Trong lúc khởi động, đặt hai bánh cùng độ cao, giữ khung gần thẳng và yên, rồi
nhấn nhả nút ở bước LED xanh dương. Đây là tư thế tham chiếu do người giữ,
không phải bằng chứng xe đang ở góc cân bằng cơ học. Tiếp tục theo
[trình tự LED](HUONG_DAN_UI_CALIB.md). Khi setup thành công,
chương trình in dòng bắt đầu bằng `READY ICM20602`, sau đó có log định kỳ. Nếu
setup thất bại, chương trình giữ trạng thái chưa sẵn sàng; sửa nguyên nhân rồi reset.
Riêng phép đo bias kiểm tra phương sai gyro; quay đều vẫn có thể bị nhận nhầm
thành bias. UI có thêm kiểm tra mốc gia tốc và lượt quay, nhưng người dùng vẫn
phải giữ xe thật yên trong giai đoạn đo bias.

UI đo bias lúc đứng yên, sau đó lấy trục dọc khung từ lượt lật trái–phải. Ma trận
giữ trục dọc đo được nên pitch ban đầu không bị ép về 0; roll 0 vẫn theo tư thế
tham chiếu, vì thế hai bánh phải cùng độ cao. Ma trận chỉ lưu trong RAM và đo lại
sau reset. Góc cân bằng `gocCanBang_rad` trong `ControlConfig.h` cần xác định riêng;
giá trị 0 hiện tại là giá trị khởi đầu. Ở LED trắng, giữ yên gần góc đặt đó:
`|pitch - gocCanBang_rad| ≤ 0,10 rad` và `|roll| ≤ 0,10 rad` trong 80 mẫu liên tiếp.
Xem `CALIB_RETURN` trên Serial nếu bước này chưa qua.

LED xanh lá báo setup sẵn sàng. Nút GPIO3 chỉ phục vụ calib; sau READY, nút chưa
có chức năng bật/tắt cân bằng. Gửi `a` riêng qua Serial để yêu cầu bật, sau khi
các điều kiện phần cứng và tư thế đã được đáp ứng. Mặc định cờ phần cứng vẫn `false`.

## 6. Vận hành bằng các lệnh Serial

Gửi **từng ký tự riêng** và quan sát kết quả trước khi gửi lệnh kế tiếp. Ký tự
xuống dòng không gây hành động. Các lệnh được gom thành bit sự kiện, nên không
coi chuỗi `ka` hoặc `da` gửi một lần là một kịch bản thực hiện tuần tự đầy đủ.

| Phím | Công dụng | Điều kiện/kết quả |
| --- | --- | --- |
| `h` | In hướng dẫn ngắn | Có tác dụng sau khi ứng dụng sẵn sàng |
| `i` | Bật/tắt log RAW_IMU | Chỉ in khi phase0; tự tắt chế độ in raw khi rời phase0 |
| `a` | Yêu cầu bật cân bằng | Cần mẫu mới, tư thế phù hợp, không có lỗi giữ lại và cờ phần cứng đã xác nhận. Chờ lệnh 0 đúng phiên trước khi bật EN |
| `d` | Tắt motor | Hủy phiên hiện tại, tắt EN và dừng STEP; không tự xóa lỗi |
| `x` | Dừng khẩn | Tắt motor và giữ lỗi `emergency` |
| `k` | Xóa lỗi | Chỉ khi motor đã tắt; không tự bật lại |
| `f` | Đổi PID cố định/mờ | Chỉ khi motor đã tắt; hủy yêu cầu bật đang chờ. Giá trị không lưu qua reset |
| `w` / `s` | Yêu cầu tiến/lùi ±0,15 m/s | Chỉ phase2; lặp dưới500ms để giữ yêu cầu |
| `j` / `l` | Yêu cầu quay trái/phải ±0,60 rad/s | Có ramp và giới hạn dư địa cân bằng; chưa có yaw feedback |
| `z` | Giảm mục tiêu tiến/quay về0 | Giữ cân bằng; không phải tắt motor/dừng khẩn |

Thứ tự thường dùng:

```text
h → quan sát dữ liệu
a → thử bật khi đủ điều kiện
d → tắt bình thường
x → dừng khẩn khi cần
k → xóa lỗi sau khi đã tắt và xử lý nguyên nhân
a → yêu cầu bật lại riêng
```

Mỗi lượt nhận sự kiện ưu tiên dừng khẩn, tắt, xóa lỗi, đổi mờ rồi mới xét bật.
Sau `f`, xác nhận trường `fuzzy` trong log trước khi gửi `a`.
Mỗi phím di chuyển đặt một mục tiêu đầy đủ, không cộng dồn. Lệnh hết500ms hoặc sai
phiên đưa mục tiêu về0 qua ramp; `d/x` vẫn có vai trò tắt/dừng khẩn. Hạn này là
lease của yêu cầu di chuyển, độc lập với deadline lệnh STEP10/20ms.

### Điều kiện bật hiện tại

`canArm()` yêu cầu dữ liệu góc/xung hợp lệ và còn mới; gia tốc tin cậy; góc dọc
cách góc cân bằng không quá 0,10 rad, góc ngang không quá 0,10 rad (khoảng 5,7°);
tốc độ quay dọc/ngang không quá 0,35 rad/s. Đây là ngưỡng trong cấu hình, cần đối
chiếu cách lắp và chất lượng đo thực tế.

Nếu gửi `a` khi các điều kiện chưa đủ, motor tiếp tục tắt. `ready=none` chỉ cho biết
mẫu đã qua kiểm tra chạy, còn kiểm tra bật chặt hơn và còn cờ xác nhận phần cứng.

### Hiểu `phase` và `epoch`

| Trường | Giá trị | Ý nghĩa |
| --- | --- | --- |
| `phase` | 0 | Đang tắt |
| `phase` | 1 | Chờ ControlTask reset và gửi lệnh bằng 0 đúng mã phiên |
| `phase` | 2 | Đã bật, đang nhận lệnh điều khiển của phiên hiện tại |
| `epoch` | Số nguyên | Mã phiên; thay đổi khi bật/tắt hoặc các thao tác hủy phiên |

Giai đoạn chờ có thể ngắn hơn chu kỳ log 200 ms, nên monitor có thể chỉ thấy
`phase=0` rồi `phase=2`. Không tự tăng tần suất log trong task điều khiển để bắt giai
đoạn này; dùng kiểm thử hoặc phép đo riêng khi cần kiểm tra trình tự chi tiết.

## 7. Trình tự thử và tiêu chí đạt

| Bước | Thực hiện | Tiêu chí quan sát/đo |
| --- | --- | --- |
| 1. Khởi động, EN tắt | Nạp với cờ phần cứng `false`, làm theo nút và LED | Có READY sau khi LED xanh lá; EN phần mềm tắt |
| 2. Đọc IMU | Nghiêng dọc/ngang từ từ khi motor tắt | `theta` tăng dương khi nghiêng trước theo trục đã chọn; `rate` cùng chiều lúc bắt đầu nghiêng; `roll` thay đổi khi nghiêng ngang; norm gia tốc đứng yên gần 9,81 |
| 3. Tư thế không cho bật | Đặt thân nghiêng ngang lớn và gửi `a` | Hàm kiểm tra báo tư thế không phù hợp và EN giữ tắt; ngưỡng ngã hiện tại là 0,50 rad |
| 4. Đo STEP/DIR/EN | Dùng bench tín hiệu và thiết bị đo, đối chiếu manual MKS | Độ rộng, cực tính, khoảng đổi DIR và phản ứng EN phù hợp driver thực tế |
| 5. Thử bánh | Sau khi đã xác nhận phần cứng, đặt cờ `true`, build/nạp lại; kê bánh và giữ thân | Cả hai bánh chạy theo cùng quy ước tiến; dấu PCNT khớp lệnh; số cạnh phù hợp phép đo |
| 6. Bật/tắt và lỗi | Thử riêng `a`, `d`, `x`, `k`; thử bật lại nhanh | Phiên mới bắt đầu bằng lệnh 0; `x` giữ lỗi; `k` không tự bật |
| 7. Deadline và mất mẫu | Dùng bản thử có chủ đích theo tài liệu kiểm thử | Mất lệnh/mẫu khiến dừng; task motor bị chặn trong khi ngắt còn hoạt động thì GPTimer cắt EN; đo thời gian bằng thiết bị |
| 8. Tinh chỉnh | Thử PD cố định trước, sau đó PI vận tốc và luật mờ | Có log/đo trước-sau về góc cực đại, dao động, thời gian hồi phục và bão hòa |

Ở bước 4, `test/stepMeansure.cpp` là chương trình bench riêng luôn giữ EN tắt.
Nếu dùng nó làm main để đo, cần chỉ biên dịch một bộ `setup()/loop()` và khôi phục
main ứng dụng sau phép thử. Các file `test/main_test.cpp` và `test/test_MPU_MKS.cpp`
vẫn dùng driver MPU6050 cũ; cần điều chỉnh đúng ICM20602 trước khi dùng lại.

Với 200 bước/vòng, vi bước 32, truyền động 1:1: 6400 cạnh STEP tương ứng một vòng
bánh khi motor bám xung. Đối chiếu bằng cạnh thực đã đo; `cmd` là tốc độ mục tiêu,
có thể khác tần số tức thời trong khoảng dừng/đổi DIR.

Chỉ chuyển `daXacNhanCauHinhPhanCungMotor` sang `true` sau khi mức điện, menu EN,
chiều và timing đã được kiểm tra. Giá trị đó ghi nhận việc đã đối chiếu phần cứng;
nó không tự chứng minh kết quả đo đúng.

## 8. Đọc log

| Trường | Đơn vị/ý nghĩa |
| --- | --- |
| `EN` | Trạng thái bật theo driver, có loại trừ khi giám sát đã ngắt; không phải dữ liệu UART từ MKS |
| `fuzzy` | 1: dùng luật mờ; 0: hệ số cố định |
| `fault` | Lỗi ứng dụng giữ lại, chẳng hạn `emergency`, `fallen`, `motor_fault` |
| `ready` | Kiểm tra mẫu gần nhất; `none` nghĩa chưa có lỗi ở bước kiểm tra đó |
| `motorFault` | Mã lỗi lớp motor |
| `last` | Kết quả thao tác motor gần nhất |
| `theta`, `rate`, `target` | Góc dọc, gyro dọc, góc đặt: rad, rad/s, rad |
| `roll`, `accel` | Góc ngang từ gia tốc, độ lớn gia tốc: rad, m/s² |
| `x`, `v` | Vị trí/vận tốc ước lượng: m, m/s |
| `STEP` | Tổng xung trái/phải có dấu, không phải tốc độ |
| `cmd` | Tốc độ STEP/s mục tiêu sau giới hạn của driver |
| `K` | Kp/Ki/Kd hiện hành; sau reset có thể là 0 trước khi cập nhật |
| `serviceMax` | Khoảng cách MotorTask lớn nhất, µs; số thống kê gồm cả thời gian motor tắt |
| `miss`, `dup`, `overlap` | Số mẫu nhảy thứ tự, thông báo trùng/lùi, lần đọc gặp ngắt mới |
| `gyroD` | Gyro sau PT1 nhánh D, rad/s |
| `innov`, `trust` | Sai lệch góc accel–dự đoán, rad; trọng số tin cậy0..1 |
| `vref`, `yawref` | Mục tiêu sau ramp, m/s và rad/s |
| `tilt`, `tiltDir` | Độ nghiêng tổng và hướng trong XY xe, rad; hướng không xác định khi thẳng được in0 |

Đối chiếu `serviceMax` với thời điểm thử và trạng thái motor. Một giá trị cực đại
đã ghi từ trước không giảm khi task hoạt động bình thường trở lại; reset bắt đầu
đợt thống kê mới. Độ trễ cắt EN cần đo riêng, không suy trực tiếp từ trường này.

Mã `last`: `0=Ok`, `1=Saturated`, `2=Pending`, `3=Disabled`, `4=NotInitialized`,
`5=ConfigInvalid`, `6=HardwareConfigUnconfirmed`, `7=FaultLatched`.

Mã `motorFault`: `0=None`, `1=InvalidCommand`, `2=CommandExpired`, `3=ServiceLate`,
`4=BackendError`, `5=SparseBacklog`, `6=EmergencyStop`, `7=WatchdogExpired`.
Với lỗi dừng do ứng dụng, xem đồng thời `fault` để tìm nguyên nhân gốc.

## 9. Xử lý tình huống thường gặp

| Hiện tượng | Cách kiểm tra |
| --- | --- |
| Không có cổng COM | Kiểm tra kết nối/board và `pio device list`; đối chiếu cách vào chế độ nạp của board |
| Không thấy log khởi động | Mở đúng cổng/baud rồi reset board; log setup được in một lần |
| `BOOT ERROR: ICM20602` | Kiểm tra chân, địa chỉ I²C, ID `0x12`, bus và nguồn cảm biến theo module thực tế |
| `BOOT ERROR: ma tran lap IMU` | Kiểm tra R sensor→xe: hữu hạn, trực giao, det+1; xem công cụ hiệu chuẩn |
| Hiệu chuẩn gyro thất bại | Giữ cảm biến yên, kiểm tra rung, dữ liệu và timeout trước khi nới ngưỡng |
| Gửi `a` nhưng EN vẫn 0 | Kiểm tra `fault`, tư thế, gyro, `ready`, cờ phần cứng và `last`; khi driver từ chối vì cờ phần cứng, `last=6` |
| `ready=pulse_invalid` lúc mới chạy | Mẫu đầu chưa đủ tính vận tốc; nếu kéo dài, kiểm tra PCNT và timestamp |
| `fallen` khi thấy xe đứng thẳng | Đối chiếu trục/dấu cảm biến, góc cân bằng và cách lắp |
| `imu_stale` hoặc `pulse_stale` | Kiểm tra ngắt IMU, task, lịch phục vụ và tuổi mẫu; không chỉ tăng timeout để che nguyên nhân |
| `arm_timeout` | Kiểm tra việc tạo lệnh 0 đúng `epoch` và hoạt động của ControlTask |
| `motorFault=7` | GPTimer đã ngắt EN; tìm khoảng MotorTask không được phục vụ hoặc deadline lệnh đã hết |
| `overlap` tăng nhiều | Đo thời gian I²C và độ trễ task; kiểm tra ngắt có đúng nhịp |
| Bánh chạy ngược chiều đỡ thân | Kiểm tra riêng dấu IMU và ánh xạ DIR từng bánh |
| Xe dao động dù không báo lỗi | Đo bão hòa, độ trễ, rung/cơ khí và tinh chỉnh hệ số theo từng vòng |

Sau lỗi, xử lý nguyên nhân, xác nhận motor đã tắt, gửi `k`, rồi gửi `a` riêng khi
tư thế và dữ liệu đã phù hợp. `k` không sửa dây nối, thông số hoặc chất lượng mẫu.

## 10. Giới hạn cần hiểu khi vận hành

- Các hệ số PID/PI, luật mờ và ngưỡng thử chưa chứng minh xe thật cân bằng ổn định.
- Ki góc hiện bằng0. R01 đã sửa theo hướng bão hòa và có regression test Ki20 hai
  chiều; bộ hệ số Ki khác0 vẫn cần được tinh chỉnh/kiểm thử trên xe.
- PCNT biết xung tại ESP32; nó không phát hiện trực tiếp sai lệch bám bên trong MKS,
  bánh trượt hoặc chuyển động do đẩy khi motor tắt.
- Giám sát GPTimer độc lập với MotorTask, nhưng vẫn cần CPU phục vụ ngắt. Khóa ngắt,
  lỗi CPU hoặc cấu hình cache có thể làm phản ứng trễ; phải đo trên bản firmware thực.
- Profile build còn mang tên LOLIN S3 Mini; tên ESP32-S3 chung chưa đủ xác nhận mọi
  thông số flash/PSRAM của board Super Mini.
- Khi thay cấu hình hoặc cập nhật nền tảng PlatformIO, chạy lại bộ test và các phép
  đo liên quan; giá trị `stable` trong URL nền tảng có thể trỏ tới bản khác ở lần cài sau.

Các phép thử gây mất mẫu/chậm task và bảng vị trí cần sửa nằm trong
[Hướng dẫn sửa và kiểm thử](HUONG_DAN_SUA_VA_KIEM_THU.md). Lưu cấu hình cùng log mỗi
đợt thử để việc so sánh đáp ứng trước/sau có ý nghĩa.
