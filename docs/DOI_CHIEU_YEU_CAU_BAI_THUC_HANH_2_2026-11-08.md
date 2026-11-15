# Đối chiếu yêu cầu Bài thực hành 2 với xe hai bánh tự cân bằng

Ngày đánh giá: **08/11/2026 — Asia/Saigon**.
Nguồn yêu cầu: ảnh đề bài do người dùng cung cấp trong chat.
Phạm vi: mã nguồn và tài liệu trong working tree hiện tại, kể cả các thay đổi
calib/kiểm tra ICM chưa commit; nội dung bản đặc tả Word trong thư mục gốc.

**Kết luận: chưa đủ bằng chứng đáp ứng toàn bộ bài thực hành.** Firmware đã có
đo góc, hiệu chuẩn, PID cố định/mờ, PI vận tốc, STEP/DIR và bảo vệ. Phần còn thiếu
chính là số liệu khảo sát xe thực, đồ thị, cơ sở chọn hệ số từ khảo sát và kết quả
thực nghiệm bám giá trị đặt. Build/test phần mềm đạt chưa thay được các phần đó.

Đây là đối chiếu mức đáp ứng theo hồ sơ hiện có, không chấm điểm thay giảng viên.
Những dữ liệu hoặc kết quả người dùng đã có ở ngoài workspace chưa được đánh giá.
Đợt này không sửa firmware và không chạy lại build/native test; kết quả kiểm thử
được đối chiếu với bản ghi đã có ngày 05/11/2026.

## 1. Bảng đối chiếu từng ý trong đề

| Ý của đề bài | Mức đáp ứng hiện tại | Bằng chứng / phần cần bổ sung |
| --- | --- | --- |
| Thay tên mô hình bằng đối tượng đã đăng ký | Có đối tượng cụ thể, chưa xác nhận danh sách đăng ký | Có thể dùng tên “Xe hai bánh tự cân bằng”; chưa có danh sách nhóm đăng ký để đối chiếu |
| YC1: thiết kế đối tượng | Có một phần hồ sơ kỹ thuật | Có sơ đồ chân, cơ khí khai báo, phần cứng ICM20602/ESP32-S3/MKS42C/17HS4401S, kiến trúc điều khiển |
| YC1: chế tạo hoặc trình bày chi tiết đối tượng đã có | Chưa đủ hồ sơ để xác nhận hoàn tất yêu cầu trình bày | Người dùng đã phản ánh vận hành LED trên xe; chưa thấy bộ ảnh toàn xe, bản vẽ lắp, bảng linh kiện đầy đủ, mạch nguồn và mô tả quy trình chế tạo/đo thông số |
| YC2: xác định ngõ vào/ngõ ra của đối tượng | Có nền tảng, cần chốt phạm vi khảo sát trong báo cáo | Có lệnh STEP/DIR và đo góc ICM; cần ghi rõ đối tượng toàn xe hay chỉ cụm motor, cùng đơn vị và điều kiện thử |
| YC2: xác định khoảng giá trị bằng khảo sát | Chưa có bằng chứng thực nghiệm | Các giới hạn STEP, gia tốc, miền mờ đang là hằng số thiết kế khởi đầu |
| YC2: các cặp điểm vào/ra đã đo | Chưa tìm thấy bộ dữ liệu xe thực | Có telemetry nhưng chưa có log khảo sát lưu lại trong hồ sơ được kiểm tra |
| YC2: đồ thị tương quan vào/ra | Chưa có đồ thị khảo sát xe thực | CSV hiện tại là kết quả công thức/mô hình lý tưởng; không chứng minh đặc tính đã đo của xe |
| YC3: có bộ điều khiển PID | Có trong mã | PID có P/I/D, giới hạn và chống tích lũy; có chuyển cố định/mờ |
| YC3: thiết kế PID dựa trên dữ liệu YC2 | Chưa có bằng chứng | Config ghi rõ hệ số khởi đầu, chưa nhận dạng thực nghiệm; chưa có nhật ký khảo sát → chọn hệ số |
| YC3: ngõ ra bám giá trị đặt | Có đường điều khiển, chưa có kết quả xe thực | Có đặt góc cân bằng, vòng vận tốc và lệnh di chuyển; native test chỉ kiểm tra mô hình lý tưởng |
| YC3: trình bày quá trình và kết quả thử nghiệm | Có hướng dẫn, chưa có hồ sơ kết quả | Thiếu bảng từng lần chỉnh hệ số, đồ thị đáp ứng và chỉ tiêu đo được |
| Nộp file theo mẫu báo cáo | Chưa xác nhận | Chưa được cung cấp mẫu báo cáo của giảng viên; file Word hiện có là đặc tả kiến trúc phần mềm |
| Demo trực tiếp | Chưa đủ căn cứ xác nhận sẵn sàng | Cờ xác nhận motor trong source vẫn false; chưa có log/video chứng minh cân bằng và bám đặt trên xe |

## 2. Yêu cầu 1 — thiết kế và chế tạo

Thông tin đã có:

- ESP32-S3 Super Mini; cảm biến **ICM20602**; hai driver **MKS Servo42C** nhận STEP/DIR.
- Motor **17HS4401S**; 200 bước/vòng, vi bước đầu vào 32, tỉ số truyền khai báo 1:1.
- Khối lượng khai báo 1,45 kg; bán kính bánh 0,04 m; khoảng cách hai bánh 0,20 m.
- Trọng tâm cao 0,025 m tính từ trục bánh theo xác nhận người dùng.
- Sơ đồ chân và quy ước motor 1 phải, motor 2 trái; thiết kế luồng cảm biến →
  ước lượng → điều khiển → motor và giám sát lỗi.

Bằng chứng: [RobotGeometry.h](../include/config/RobotGeometry.h),
[PinConfig.h](../include/config/PinConfig.h),
[báo cáo sơ đồ và flow](DOI_CHIEU_SO_DO_VA_FLOW_CALIB.md),
[logic chương trình](LOGIC_CHUONG_TRINH.md).

Hồ sơ cần bổ sung để trình bày YC1: ảnh xe thật và các cụm; kích thước/bản vẽ
lắp; bảng linh kiện; sơ đồ cấp nguồn và điện áp motor thực; cách bố trí IMU;
các bước gia công, gá lắp, đấu dây và kiểm tra. Ghi rõ thông số nào đã đo, thông
số nào chỉ là giá trị đang khai báo. Nếu xe có sẵn, trình bày chi tiết cấu tạo
và nguyên lý theo lựa chọn mà đề cho phép, không cần dựng lại quá trình chế tạo
mà nhóm chưa thực hiện.

## 3. Yêu cầu 2 — khảo sát đặc tính vào và ra

### 3.1 Chọn đúng đối tượng và đại lượng

Với bài toán cân bằng, cách mô tả phù hợp mã hiện tại là:

```text
Đầu vào chấp hành toàn xe: lệnh STEP có dấu của hai bánh, đơn vị STEP/s
Đầu ra chính: góc thân xe theta, rad hoặc độ
Đầu ra phụ: tốc độ nghiêng, vận tốc ước lượng từ xung, vị trí ước lượng
```

Cần tách điều này khỏi ngõ vào/ra của khối mờ:

```text
Khối mờ nhận: sai lệch góc e và tốc độ nghiêng đã lọc
Khối mờ điều chỉnh: Kp, Ki, Kd bằng các bảng hệ số nhân
PID tạo: gia tốc lệnh, rồi tích phân thành vận tốc và quy đổi STEP/s
```

Code dùng `e = theta - theta_dat`. Đầu vào thứ hai của mờ là tốc độ nghiêng,
không phải toàn bộ đạo hàm của e khi góc đặt thay đổi. Quy ước dấu cần ghi đúng
theo source khi trình bày sơ đồ và công thức.

Bằng chứng: [AnglePidController.cpp](../lib/controllers/AnglePidController.cpp)
và [CascadeController.cpp](../lib/controllers/CascadeController.cpp).

### 3.2 Những khoảng giá trị hiện có chỉ là cấu hình

| Đại lượng | Giá trị trong code | Ý nghĩa thực sự |
| --- | --- | --- |
| Miền chuẩn hóa sai lệch cho mờ | ±0,15 rad ≈ ±8,59° | Giá trị khởi đầu; vượt miền thì clamp đầu vào chuẩn hóa, không phải miền góc đã khảo sát |
| Miền chuẩn hóa tốc độ nghiêng cho mờ | ±1,5 rad/s ≈ ±85,94°/s | Giá trị khởi đầu, chưa có thống kê xe chứng minh lựa chọn |
| Gia tốc đầu ra PID | ±3 m/s² | Trần lệnh phần mềm, chưa phải gia tốc thật đã đo |
| Tần số STEP mỗi motor | tối đa 10.000 STEP/s | Trần lệnh thử, chưa phải giới hạn motor đã nghiệm thu |
| Tốc độ bánh quy đổi ở trần STEP | khoảng 0,392699 m/s | Tính từ bán kính, số bước, vi bước và tỉ số truyền; phụ thuộc bám lệnh/bánh không trượt |
| Góc bù do PI vận tốc | ±0,10 rad | Giới hạn góc đặt của vòng ngoài, không phải kết quả khảo sát vào/ra |

Nguồn: [ControlConfig.h](../include/config/ControlConfig.h),
[MotorConfig.h](../include/config/MotorConfig.h),
[bảng tính hiện có](du_lieu_tinh_toan/KET_QUA_TINH_TOAN.md).

### 3.3 Phần thu dữ liệu đang thiếu

Telemetry runtime tại `RobotApp.cpp` có `theta`, `rate`, `target`, `v`, `STEP`,
`cmd`, `K`, trạng thái mờ và lỗi. Chu kỳ hiện tại 200 ms, tức **5 Hz**. Dòng này
chưa có timestamp nguồn của mẫu điều khiển, chưa in trực tiếp gia tốc đầu ra PID
hoặc e. Có thể suy e từ theta và target nhưng vẫn cần thời gian và ghép mẫu
nhất quán để vẽ đáp ứng. Snapshot motor và controller được lấy từ hai queue,
chưa đóng thành bản ghi khảo sát có thời gian đồng bộ.

`RAW_IMU` có timestamp nhưng chỉ in khi motor tắt, cũng ở nhịp console 5 Hz;
chưa thay được dữ liệu đáp ứng cân bằng đang chạy. Bốn CSV hiện có là tốc độ
bánh lý thuyết, so sánh vi bước, đáp ứng bộ lọc tính toán và sai số góc lắp lý
tưởng. Chưa tìm thấy log/đồ thị khảo sát xe thật trong hồ sơ đã kiểm tra.

Đặc biệt, đồ thị STEP/s so với vận tốc cũng suy từ chính PCNT đếm STEP chủ yếu
kiểm chứng phép đổi đơn vị. Nó chưa đo độc lập motor có bám đúng hay bánh có
trượt. Nếu khảo sát tốc độ bánh thật, cần phép đo độc lập phù hợp; không nhất
thiết phải thêm encoder ngoài. Với khảo sát cân bằng, góc đo bằng ICM là đầu ra
đo độc lập so với lệnh STEP.

Bằng chứng: [RobotApp.cpp](../src/app/RobotApp.cpp),
[SafetyConfig.h](../include/config/SafetyConfig.h),
[DifferentialDrive.cpp](../lib/kinematics/DifferentialDrive.cpp),
[WheelPulseCounter.cpp](../lib/motor_drv/src/WheelPulseCounter.cpp).

Để hoàn thành YC2, thu dữ liệu có thời gian ở nhiều mức kích thích nhỏ, ghi điều
kiện nguồn/tải/cách giữ xe và trạng thái vòng điều khiển. Với mô hình xe cân bằng
động, cần trình bày đáp ứng theo thời gian và ảnh hưởng trạng thái trước đó;
không mặc định một lệnh STEP/s luôn tương ứng một góc cân bằng tĩnh. Dữ liệu trên
giá đỡ hoặc vòng kín phải ghi rõ điều kiện vì điều kiện thử ảnh hưởng đặc tính.

## 4. Yêu cầu 3 — PID bám giá trị đặt

### 4.1 Những phần đã có

- Bộ PID góc có P/I/D, D dùng gyro đã lọc, giới hạn và chống tích lũy.
- PID cố định và Sugeno 5×5 điều chỉnh hệ số, chuyển bằng phím `f` khi motor tắt.
- Góc cân bằng cơ học `gocCanBang_rad`; PI vận tốc ngoài tạo góc bù.
- Vận tốc đặt mặc định 0; lệnh thử tiến/lùi/quay có giới hạn, ramp và thời hạn.
- Hiệu chuẩn IMU, kiểm tra boot, giới hạn góc/dữ liệu và dừng khi lỗi.

### 4.2 Ba điểm chưa đủ để kết luận đạt trọn YC3

**Hệ số chưa có nguồn gốc thực nghiệm.** `ControlConfig.h` ghi rõ đây là giá trị
khởi đầu. Chưa có chuỗi dữ liệu khảo sát → chọn tham số → thử đáp ứng → chỉnh
lại → so sánh kết quả để chứng minh yêu cầu “dựa vào dữ liệu khảo sát ở YC2”.

**Cấu hình góc hiện là PD trong cấu trúc PID.** Hệ số cơ sở là Kp=30, Ki=0,
Kd=3; mặc định mờ bật. Vì khối mờ chỉ nhân hệ số Ki cơ sở nên Ki hiệu dụng vẫn
bằng 0. Vòng ngoài là PI vận tốc. Điều này không tự động khẳng định bị loại bởi
đề bài, nhưng cần mô tả đúng: chưa có bằng chứng PID góc với cả ba thành phần
hoạt động đã được tinh chỉnh. Nếu tiêu chí yêu cầu ba thành phần, phải có lý do
chọn Ki và thử nghiệm phù hợp, không chỉ đổi Ki sang số khác 0 để đổi tên.

**Chưa có kết quả bám trên xe.** Các test `plantTest()` và `cascadePlantTest()`
trong native dùng con lắc điểm khối/chấp hành lý tưởng. Chúng hỗ trợ kiểm tra
dấu/đơn vị/logic, chưa mô tả trễ, ma sát, mô-men motor và nguồn thực. Bản ghi
kiểm thử gần nhất ghi chưa upload/chưa thử xe trong lần kiểm thử đó.

Bằng chứng: [ControlConfig.h](../include/config/ControlConfig.h),
[AnglePidController.cpp](../lib/controllers/AnglePidController.cpp),
[control_tests.cpp](../test/native/control_tests.cpp),
[bản ghi kiểm thử 05/11](du_lieu_tinh_toan/PHIEN_KIEM_THU_ICM_BOOT.json).

### 4.3 Kết quả cần đưa vào báo cáo

Chọn góc thân làm đầu ra chính; góc cân bằng cơ học đã xác định là giá trị đặt.
Bám một góc không đổi đã đáp ứng ý “giá trị cho trước”; ảnh không bắt buộc phải
có giao diện nhập mọi góc hoặc chạy quỹ đạo phức tạp. Khi ghép PI vận tốc cần
ghi thêm góc đặt thực tế của vòng góc vì nó có thể thay đổi quanh góc cơ học.

Cần có log/đồ thị theta và theta_dat theo thời gian, sai lệch, lệnh điều khiển,
hệ số, tình trạng giới hạn và lỗi; bảng điều kiện thử và từng lần chọn hệ số.
Đề không ghi chỉ tiêu định lượng bắt buộc, nhưng độ lệch đỉnh, thời gian vào dải
sai số đã công bố, sai lệch còn lại hoặc RMSE là bằng chứng đánh giá hữu ích.
Khi góc đặt bằng 0, tránh tính phần trăm vượt lố bằng cách chia cho giá trị đặt 0.

Có thể dùng chế độ cố định để trình bày nền tảng PID rồi bổ sung so sánh mờ nếu
đưa mờ vào báo cáo. YC3 trong ảnh yêu cầu PID; không ghi bắt buộc phải so sánh hai
chế độ. Phần mờ đã có là lợi thế kỹ thuật nhưng không thay dữ liệu khảo sát YC2.

## 5. Báo cáo và demo

File `DacTaChuongTrinh_XeCanBang_Formatted.docx` hiện là đặc tả kiến trúc/hợp đồng
module; còn nhắc driver MPU6050 ở nhiều phần. Không tìm thấy kết quả khảo sát
hoặc thử xe trong nội dung đó. Chưa có mẫu báo cáo giảng viên để xác nhận bố cục
nộp đúng mẫu. Cần viết báo cáo thực hành theo ba yêu cầu, nêu đúng **ICM20602**
và các đại lượng thực đo của xe hiện hành.

Source hiện giữ `daXacNhanCauHinhPhanCungMotor=false`; `enable()` sẽ trả
`HardwareConfigUnconfirmed`. Vì vậy build từ cấu hình này chưa thể bật motor
để demo cân bằng. Cần xác minh DIR/EN/vi bước/phần cứng trước khi xác nhận cờ.
Đánh giá này chỉ nói về source hiện có; không suy đoán cấu hình đã nạp ở ESP.

Ảnh đề ghi nộp file và demo trực tiếp **14/11/2026 từ 14h00**, tại phòng Kỹ thuật
PLC và IoT Công nghiệp (PLC&IIoT), thư viện trường Bách Khoa. Đây là mốc được chép
từ ảnh, chưa có thông báo cập nhật khác để đối chiếu.

## 6. Thứ tự hoàn thiện

1. Chốt mô tả đối tượng, hồ sơ cơ khí/điện và nguồn motor; hoàn tất xác minh
   phần cứng, calib và góc cân bằng cơ học.
2. Bổ sung cơ chế lưu dữ liệu khảo sát có timestamp/đại lượng đồng bộ ở nhịp phù
   hợp đáp ứng xe, lấy từ snapshot/bộ đệm để không chặn vòng PID.
3. Thu các phép thử nhỏ có kiểm soát, lưu log gốc và điều kiện đo; từ đó xác định
   miền hoạt động, miền mờ và giới hạn chấp hành có căn cứ.
4. Chọn/tinh chỉnh hệ số dựa vào log, thử bám và hồi phục, vẽ đồ thị, tính chỉ
   tiêu đã chọn; lưu cả cấu hình và kết quả từng lần thử.
5. Điền báo cáo đúng mẫu, đính kèm dữ liệu/đồ thị và tập demo trên xe.

Phần thiếu lớn nhất hiện nay là bằng chứng thực nghiệm và trình bày kết quả.
Không thể đánh dấu hoàn tất bài chỉ từ số lượng hàm, tính năng hoặc test native.
