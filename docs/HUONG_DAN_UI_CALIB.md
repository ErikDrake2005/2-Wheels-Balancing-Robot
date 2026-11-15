# Hiệu chuẩn góc bằng thao tác ĐẦU và ĐUÔI xe

Cập nhật **09/11/2026**; thao tác ĐẦU/ĐUÔI giữ theo bản 08/11. Quy trình hiện tại dùng **chúi đầu/ngả sau quanh trục
hai bánh**. Hướng dẫn “nghiêng trái/phải 30–45°” trước đây được thay thế bằng
quy trình dưới đây. Giữ motor tắt trong toàn bộ calib.

`UI_Calib` đo bias gyro, xác định phép quay từ cảm biến sang thân xe, rồi kiểm
tra tư thế khởi động. Hoàn tất calib chưa chứng minh xe đã cân bằng được.
Xem [báo cáo hình học và kiểm chứng góc](CALIB_DAU_DUOI_VA_KIEM_CHUNG_GOC_2026-11-08.md).

## 1. Xác định hướng trước khi bấm nút

Bạn tự chọn phía nào là hướng xe sẽ chạy tiến và dán nhãn **ĐẦU** lên phía đó.
Phía đối diện là **ĐUÔI**. Giữ quy ước này trong cả calib lẫn kiểm tra chiều motor.

- **Chúi đầu/nghiêng tiến:** hạ ĐẦU xuống, ĐUÔI nâng lên. Đây là pitch dương.
- **Ngả sau/nghiêng lùi:** hạ ĐUÔI xuống, ĐẦU nâng lên. Đây là pitch âm.
- Trục quay của thao tác đi dọc **trục nối hai bánh**. Hai bánh không bị lật
  sang một bên như thao tác nghiêng ngang trước đây.

Gyro Y đo tốc độ **quay quanh Y**, không phải “góc đo bằng accel Y”. Nếu IMU lắp
gần đúng hệ thân, khi chúi đầu gyro Y thường là thành phần chính; PCB xoay xéo
có thể trộn X, Y và cả Z. Firmware dùng đủ các thành phần để tìm trục thật,
rồi đổi cả gyro và accel sang hệ thân. Không chọn cứng một trục cảm biến.

## 2. Ba việc hiệu chỉnh khác nhau

| Việc | Cách xác định | Giới hạn |
| --- | --- | --- |
| Bias gyro | Giữ thân hoàn toàn yên, lấy 500 mẫu ổn định | Chuyển động lật xe không dùng để đo bias |
| Hướng lắp IMU so với thân | Hai mốc ĐẦU/ĐUÔI nằm và lượt quay qua tư thế đứng | Cần quay đúng quanh trục bánh và có mặt đỡ hình học phù hợp |
| Góc cân bằng cơ học `gocCanBang_rad` | Vị trí trọng tâm và kiểm chứng trên xe thực | Không suy ra chỉ từ tư thế người đang đỡ |

Tư thế dựng ở bước xanh dương chỉ để lấy bias và mốc kiểm tra hướng lên. Nó
**không bị gán pitch bằng 0**. Khi hai mốc nằm đúng theo hình học thân, góc
nghiêng trước/sau ban đầu vẫn còn trong số đo sau calib.

### Vì sao phải đặt nằm trên mặt đỡ ổn định?

Mục tiêu thao tác là đặt nằm gần **90° ở cả phía ĐẦU và phía ĐUÔI**, thay cho
ước lượng 30–45° bằng tay. Chuẩn bị sàn phẳng hoặc gối đỡ cứng chạm vào khung,
với mặt chuẩn vuông góc hướng tiến. Không lấy linh kiện nhô trên PCB làm mặt chuẩn.

Khoảng 80–90° giúp thao tác dễ hơn, nhưng không tự bảo đảm mốc 0 chính xác.
Giới hạn sai mốc ±5° chỉ áp dụng khi **cả hai góc thật** đều trong khoảng 80–90°.
Nếu phía ĐẦU thật là 80° và phía ĐUÔI thật là 90°, khung suy ra có thể lệch mốc
pitch khoảng **5°**. Hai góc sau khi fit có thể trông đối xứng, nên thuật toán
không thể phát hiện sai lệch này chỉ từ hai mốc đó. Muốn đo lặp lại đúng, cần
hai mặt đỡ hoặc đồ gá có hình học đã biết; không chỉ giữ gần 90° bằng tay.

## 3. Chân và chuẩn bị

| Tín hiệu | GPIO | Ghi chú |
| --- | --- | --- |
| ICM20602 SDA / SCL | 10 / 11 | Theo sơ đồ người dùng |
| ICM20602 SA0 / CS | 8 / 9 | LOW / HIGH để dùng I²C tại `0x68` |
| ICM20602 INT | 7 | Data-ready sau khi UI hoàn tất |
| Motor 1 phải DIR / STEP | 12 / 13 | Net `DIR1` / `ST1` |
| Motor 2 trái DIR / STEP | 5 / 6 | Net `DIR2` / `ST2` |
| EN chung | 2 | Hiện active LOW, phải khớp MKS |
| Nút confirm | 3 | Nhấn kéo xuống GND, dùng pull-up nội theo cấu hình hiện tại |
| LED RGB | 48 | WS2812/GRB mặc định, cần đối chiếu đúng phiên bản board |

Header 4P: chân 1 GPIO1, chân 2 GPIO3, chân 3 GPIO4, chân 4 GND. Với cấu hình nút
hiện tại, nối nút giữa chân 2 và chân 4. Nếu mạch nút thực tế nhấn kéo lên 3,3 V,
đổi `nutCalibMucThap=false`; không đưa 5 V vào GPIO.

RGB có thể đổi `chanRgbCalib` và `thuTuRgbCalibLaGRB`; đặt chân -1 để tắt RGB
và theo dõi Serial. Nhả nút khi reset/cấp nguồn vì GPIO3 có vai trò strapping
chọn nguồn JTAG tùy eFuse.
[Datasheet ESP32-S3](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)

Cố định PCB vào khung, dán nhãn ĐẦU/ĐUÔI, chuẩn bị hai mặt đỡ và mở Serial 115200.
Giữ `daXacNhanCauHinhPhanCungMotor=false` khi kiểm tra cảm biến. Reset sẽ đo lại;
ma trận hiện chỉ lưu RAM.

## 4. Các bước theo màu LED

| Màu / bước | Thao tác | Firmware xử lý |
| --- | --- | --- |
| Cam | Chờ, chưa nhấn nút | Khởi tạo và kiểm tra ID/cấu hình/32 mẫu mới ICM20602 |
| Xanh dương | Hai bánh cùng cao, dựng thân gần thẳng, giữ yên; **nhấn rồi nhả** | Nhận mốc tham chiếu, không xác định góc cân bằng cơ học |
| Vàng | Tiếp tục giữ hoàn toàn yên | Đợi 500 ms, đo 500 mẫu bias và 80 mẫu gia tốc; rung thì tự thu lại trong thời hạn |
| Xanh lơ | **Hạ ĐẦU xuống**, đặt nằm lên mặt đỡ phía trước gần 90°; giữ yên, **nhấn rồi nhả** | Yêu cầu lấy `moc_truoc` |
| Vàng | Giữ nguyên tư thế ĐẦU nằm | Lấy 80 mẫu tĩnh phía trước |
| Tím | Quay chậm từ ĐẦU nằm → **dựng đứng** → **ĐUÔI nằm**; dừng yên trên mặt đỡ sau rồi **nhấn, nhả** | Thu gyro cho lượt chuyển trước → sau, kết thúc trong 30 s |
| Vàng | Giữ ĐUÔI nằm | Lấy 80 mẫu phía sau, tính ma trận và kiểm tra chất lượng |
| Trắng | Dựng trở lại gần góc đặt khởi động, giữ yên; không cần bấm | In góc theo độ/rad và chờ 80 mẫu đạt tư thế |
| Cam sau calib | Chờ bản dữ liệu runtime đầu tiên; xem `FIRMWARE imuAck-20261109` | Setup đã xong nhưng chưa xác nhận dữ liệu runtime |
| Xanh lá sau calib | Xem `ready`, `angleValid`, tuổi mẫu và lỗi | Bản trạng thái hiện có vượt kiểm tra an toàn, không tự arm |
| Đỏ sau calib | Đọc `frame`, `irqSeq`, tuổi mẫu, I²C và lỗi trên Serial | Dữ liệu/trạng thái runtime lỗi hoặc quá hạn; đây không phải yêu cầu làm lại calib bằng nút |
| Đỏ + `CALIB LOI` | Dựng lại tư thế tham chiếu, hai bánh cùng cao, giữ yên rồi nhấn nhả | Bắt đầu lại từ đo bias |
| Đỏ + `BOOT ERROR` | Đọc lỗi, xử lý kết nối/cấu hình rồi reset | Nút không sửa được lỗi khởi tạo |

Hướng chuyển ở tím phải đi **qua tư thế đứng**, không lật qua tư thế úp ngược.
Nên chuyển chậm và liên tục trong khoảng 4–8 giây. Tránh xoay đầu xe sang hướng khác
hoặc làm khung lắc sang hai bên. Motor vẫn tắt; bạn đang xoay thân bằng tay.

Một xác nhận cần nhấn và nhả ổn định qua debounce 35 ms. Giữ nút qua bước sau
không tự xác nhận bước đó. Rung ngắn ở phép đo tĩnh chỉ làm firmware thu lại;
không cần bấm liên tục. `CALIB_WAIT` là đang chờ dữ liệu đủ yên.

## 5. Kiểm tra góc và đọc lỗi sau calib

Sau khi fit, Serial in các trục/ma trận và kết quả hai mốc cùng góc tích phân
gyro để đối chiếu. Ở bước trắng, `CALIB_RETURN` hiển thị cả rad và độ.
Dựng gần góc đặt hiện tại; firmware yêu cầu:

```text
abs(pitch - gocCanBang_rad) <= 0,10 rad ≈ 5,73°
abs(roll)                 <= 0,10 rad ≈ 5,73°
norm(gyro đã trừ bias)     <= 0,06 rad/s
abs(norm(accel) - g)       <= 0,50 m/s²
80 mẫu liên tiếp đạt điều kiện
```

Bước trắng chỉ kiểm tư thế khởi động. Nó không xác nhận trọng tâm nằm trên trục
bánh hoặc xe sẽ đứng được khi thả tay. Với hai mốc chuẩn, nếu ban đầu giữ chúi
6° thì số đo vẫn khoảng 6°; cần dựng về gần góc đặt mới qua bước trắng.

Sau `READY`, khi EN vẫn tắt, kiểm chứng độc lập bằng một góc nhỏ:
**hạ ĐẦU thì theta dương, hạ ĐUÔI thì theta âm, trở lại cùng tư thế thì số đo
phải trở lại gần giá trị trước đó**. Nếu sai dấu hoặc lệch lặp lại lớn, không
arm; kiểm tra hướng thao tác, mặt đỡ và PCB cố định rồi reset/calib lại. Thử
góc khác hai mốc đã dùng để tránh chỉ kiểm lại chính dữ liệu fit.

UI xử lý hai mốc gần ±90° bằng vector. Bộ ước lượng cân bằng/PID chỉ được đưa
vào làm việc khi xe trở lại gần đứng; không dùng bước nằm ngang để yêu cầu arm.
Pitch Euler có suy biến gần ±90° nên không dùng số góc runtime ở tư thế nằm
để khẳng định phép đo động đang chính xác.

Các mốc tĩnh chờ 500 ms, cần 80 mẫu ổn định trong tối đa 30 s; tổng phương sai
accel ≤0,04 (m/s²)², độ lệch chuẩn gyro lớn nhất ≤0,5°/s. Lỗi I²C được báo riêng;
không có mẫu mới 500 ms là lỗi dữ liệu. Xem
[bảng lỗi ICM_BOOT/CALIB_WAIT](SUA_LOI_CALIB_VA_KIEM_TRA_ICM_KHOI_DONG.md).
Cửa sổ góc nằm trong `CalibConfig.h` là80–92° **sau khi fit và đổi hệ**,
dư 2° ở phía 90° cho nhiễu/mặt đỡ. Nó không bảo đảm hai góc thật thuộc80–92° và
không phải sai số được bảo đảm. Trục chính cần ≥95% năng lượng, ít nhất 40 mẫu
chuyển động và tổng góc ≥2,5 rad. So khớp góc/vector dùng ngưỡng 0,15 rad.
Đỏ sau lượt tím có thể do quay sai trục/chiều, hai mốc không đủ nằm, mẫu bị gián
đoạn hoặc góc gyro không khớp hai mốc; đọc dòng lỗi cụ thể trước `CALIB LOI`.

Từ bản `imuAck-20261109`, LED sau calib được cập nhật theo snapshot runtime:
cam khi chưa có báo cáo, xanh khi kiểm tra an toàn dữ liệu hiện tại và các lỗi đều
hợp lệ, đỏ khi có lỗi/quá hạn. `ready` và `angleValid` được kiểm lại theo thời điểm in;
góc lưu quá 20 ms không còn hợp lệ. Chẩn đoán có queue riêng nên `irqSeq`,
`imuLate` và `readStatus` vẫn giúp kiểm tra khi mẫu góc bị bỏ. Xanh không tự chứng minh `canArm`, góc/gyro đã đủ
điều kiện bật, hoặc cấu hình motor đã xác nhận. Nút vẫn chỉ phục vụ calib, chưa
có chức năng arm/disarm/dừng khẩn sau READY. Nếu đỏ sau READY, đọc
[chẩn đoán runtime](CHAN_DOAN_IMU_RUNTIME_2026-11-09.md), không nhấn nút để mong xóa lỗi.

## 6. Kích hoạt cân bằng sau calib

1. Xác minh cấu hình MKS: chế độ STEP/DIR vòng kín, vi bước 32, EN cùng cực tính
   và chiều quay từng bên. Mapping motor trái/phải đúng chưa chứng minh mức DIR
   tiến đã đúng: `mucDirTienBanhTraiCao = true`,
   `mucDirTienBanhPhaiCao = false` vẫn cần kiểm chứng trên xe.
2. Khi đã xác minh phần cứng, đặt `daXacNhanCauHinhPhanCungMotor = true` trong
   `MotorConfig.h`, build/nạp lại. Không sửa cờ chỉ để vượt lỗi khi chưa kiểm tra.
3. Chạy lại calib, xác nhận `FIRMWARE imuAck-20261109` và `READY`; chờ dữ liệu
   runtime hợp lệ (LED xanh, `ready=none`, `angleValid=1`, tuổi mẫu còn mới).
   Giữ xe gần góc đặt, ít rung, trên giá
   thử có thể đỡ thân nếu chiều điều khiển sai.
4. Gửi riêng `a` qua Serial **sau READY**. Byte Serial còn chờ trong giai đoạn
   calib bị xóa trước khi READY để không phát lại lệnh cũ.
5. Kiểm tra `EN=1`, `phase=2`, `fault=none`. `phase=1` là giai đoạn chờ lệnh 0
   của phiên khởi động; `phase=2` mới là chạy điều khiển. `EN` trong telemetry
   là trạng thái cho phép logic, không phải mức điện áp GPIO2.

Arm còn kiểm tra dữ liệu IMU/PCNT mới và hợp lệ, độ tin cậy gia tốc, pitch gần
`gocCanBang_rad`, roll nhỏ, tốc độ nghiêng nhỏ, lỗi motor và đồng bộ phiên.
Nếu từ chối, đọc `last`, `fault`, `phase`; xem bảng trong
[hướng dẫn sử dụng](HUONG_DAN_SU_DUNG.md). Cờ phần cứng vẫn `false` ở bản hiện tại.

`d` tắt motor; `x` dừng khẩn có chốt lỗi; `k` yêu cầu xóa lỗi khi trạng thái đã
hợp lệ. `w/s` tiến/lùi; `j/l` quay trái/phải; `z` đưa mục tiêu về 0 theo giới hạn
gia tốc. Lệnh di chuyển cần lặp trước 500 ms, hết hạn sẽ đưa mục tiêu về 0.
Chỉ thử di chuyển sau khi xe đã giữ cân bằng ổn định.

## 7. Chỉnh góc cân bằng và sửa chương trình

`gocCanBang_rad` nằm trong `include/config/ControlConfig.h`. Giá trị mặc định 0
là giả định khởi đầu. Với trọng tâm có tọa độ `(x, z)` so với trục bánh và quy
ước pitch dương chúi trước, mô hình tĩnh cho `theta = -atan2(x, z)`.
Ví dụ **giả định** trọng tâm lệch trước 2 mm, cao 25 mm: góc khoảng −4,57°.
Con số này không phải góc đã đo cho xe của bạn.

Xác định chiều điều khiển trước khi chỉnh PID. Khi xe đã có thể giữ cân bằng
trên giá thử, ghi log góc, tốc độ, lệnh motor ở mục tiêu tốc độ 0; kiểm tra ảnh
hưởng ma sát, mặt sàn và tải rồi chỉnh góc đặt từng bước nhỏ. Hiện chưa có thuật
toán tự tìm góc cân bằng hay lưu góc vừa tìm bằng nút.

| Muốn sửa | File / hàm |
| --- | --- |
| Chân, cực tính nút, chân LED | `include/config/PinConfig.h` |
| Bật UI, thứ tự màu RGB, cửa sổ góc nằm và dung sai khớp gyro | `include/config/CalibConfig.h` |
| Debounce, thời hạn, số mẫu tĩnh, màu, lời nhắc | `src/calib/UI_Calib.cpp` |
| Điều kiện nhận lượt lật, PCA và dựng ma trận | `lib/estimation/HieuChuanNghiengDoc.cpp` |
| Góc cơ học mong muốn | `include/config/ControlConfig.h` |
| Giới hạn góc arm/ngã | `include/config/SafetyConfig.h` |
| Cực tính DIR/EN, vi bước và xác nhận phần cứng | `include/config/MotorConfig.h` |

Bias và ma trận từ UI chỉ tồn tại trong RAM, mỗi reset chạy lại. Nếu đặt
`batUiCalib = false`, hệ thống vẫn hiệu chuẩn bias lúc khởi động nhưng dùng
ma trận khai báo tại `ImuMountConfig.h`; cần chuẩn bị đúng ma trận trước.
Công cụ offline dùng gravity ở tư thế chuẩn có giả thiết khác, được giải thích
trong [báo cáo lắp IMU](BAO_CAO_BU_LECH_LAP_IMU.md).

Xem [báo cáo sơ đồ, flow và tính toán](DOI_CHIEU_SO_DO_VA_FLOW_CALIB.md) và
[danh mục hàm](DANH_MUC_HAM.md) để đi sâu vào chương trình.


## 8. Kết quả lưu của bản 08/11/2026

Kết quả này thuộc bản 08/11. Bản runtime `imuAck-20261109` đã build và chạy sáu
nhóm native PASS; boot/UI/runtime 265 điều kiện. Chưa upload/thử xe; xem
[kết quả và cách đọc log mới](CHAN_DOAN_IMU_RUNTIME_2026-11-09.md).
Build và sáu nhóm native PASS; calib pitch 2.038 kiểm tra, boot/UI 44 kiểm tra.
Nhóm pitch có 45 góc gá, ba mức mốc 80°/85°/90° và ba góc tham chiếu, cùng các ca
không đạt/không quan sát được sai mốc. RAM 57.044 byte, flash 476.206 byte.
Chưa upload hoặc kiểm chứng độ chính xác trên xe thật; cần làm phép kiểm góc
độc lập và độ lặp theo bước5 sau khi nạp.
