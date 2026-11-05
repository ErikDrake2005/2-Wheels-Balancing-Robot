# Báo cáo Servo42C + 17HS4401S — 03/11/2026

## 1. Cấu hình và dữ liệu còn thiếu

Người dùng xác nhận **17HS4401S** và h=**0,025 m từ trục bánh**. Repo khai báo khối
lượng 1,45 kg, R=0,04 m, khoảng cách tâm bánh B=0,20 m, truyền động1:1, motor200
bước/vòng, MStep32. Điện áp nguồn và hãng/nhãn định mức motor chưa được xác nhận.

[Datasheet Handson cho cùng mã](https://www.handsontec.com/dataspecs/17HS4401S.pdf)
ghi 1,7A/pha, 1,8°, Rpha1,5Ω, Lpha2,8mH, holding torque43Ncm và quán tính rotor
54gcm². Đây là tài liệu tham khảo theo hãng đó, không tự coi mọi motor cùng mã có
cùng thông số. Holding torque không phải mô-men có sẵn ở mọi tốc độ; chưa dùng
các số tham khảo này để tự thay dòng/MaxT trên MKS.

## 2. Những gì khai thác từ 42C

Theo [Makerbase V1.1.2 manual](https://darxton.ru/files/_misc/8f4/8f48cf73-13a7-11f0-aa2b-000c29f5eb60.pdf),
CR_vFOC nhận EN/STP/DIR và tự điều chỉnh dòng theo tải; `Ma` dành cho CR_OPEN.
MPlyer nội suy nội bộ256 không thay số xung đầu vào MStep dùng để tính vòng bánh.
Các lệnh ACC/PID/MaxT có trong [wiki V1.1](https://raw.githubusercontent.com/wiki/makerbase-mks/MKS-SERVO42C/Serial-communication-description.md),
nhưng không có đủ ánh xạ đơn vị/đáp ứng để đổi trực tiếp thành giới hạn m/s² của xe.

Do đó firmware dùng tần số STEP có dấu, phần cứng MCPWM và PCNT cùng timestamp.
Giữ MStep32 cho tới khi đo so sánh; không tự tăng subdivision vì nghĩ càng cao
càng chính xác. Độ rộng STEP10µs, DIRhold/setup10µs, ENsettle100µs và trần10kHz
vẫn là thông số thử, chưa phải giới hạn timing đã xác nhận của MKS.

Không áp nguyên tham số của Servo42D hoặc công thức tốc độ serial sang chế độ
STEP/DIR của 42C. Không sửa firmware/cấu hình điện của MKS từ ESP32 trong thay đổi này.

## 3. Quy đổi xung, tốc độ và gia tốc

```text
N = 200 * 32 * 1 = 6400 xung/vòng bánh
d = 2*pi*0.04 / 6400 = 0.000039269908 m/xung
vmax = 10000 * d = 0.392699 m/s
rpm_max = 10000*60/6400 = 93.75 RPM
df/dt tại a=3 = a/d = 76394.37 STEP/s²
delta_f mỗi5ms tại a=3 = 381.97 STEP/s
```

Một xung trong cửa sổ5ms tương ứng lượng tử vận tốc0,007854m/s. Vì vậy vận tốc
từ đếm xung cần lọc; tau0,03s hiện dùng có alpha1/7 ở200Hz. Đây là vòng ngoài,
không thêm lọc chậm lên lệnh cân bằng cuối cùng.

| MStep | Xung/vòng | µm/xung | vmax với10kHz |
| --- | --- | --- | --- |
| 16 | 3200 | 78,540 | 0,785398 m/s |
| 32, đang dùng | 6400 | 39,270 | 0,392699 m/s |
| 64 | 12800 | 19,635 | 0,196350 m/s |

Tăng MStep làm giảm lượng tử nhưng tiêu tốn ngân sách tần số. Bảng không chứng
minh motor đạt vận tốc/lực đó. Đổi MStep trên driver phải đổi config tương ứng.

## 4. Tính lực sơ bộ

Chỉ xét phần tịnh tiến, chia đều hai bánh:

```text
F = m*a = 1.45*3 = 4.35 N
T_moi_banh = F*R/2 = 0.087 N·m
omega_banh = v/R
```

Còn thiếu lực để điều khiển thân nghiêng, quán tính thân/bánh/rotor, ma sát, độ rơ,
trượt và suy giảm mô-men theo tốc độ/điện áp. Vì vậy không lấy tỷ số0,43/0,087 làm
hệ số dự phòng đã chứng minh. Thời gian đặc trưng con lắc điểm khối lượng50,49ms
cũng không thay cho phép đo động học cả xe.

Để xác nhận khả năng 17HS4401S, cần điện áp nguồn motor dưới tải, dòng/nhãn motor,
đường cong hoặc phép đo mô-men–tốc độ, thời gian đảo chiều và sai lệch bám thực.
Nguồn thiếu dòng có thể làm driver chậm/bảo vệ dù STEP ESP32 vẫn chính xác.

## 5. Điều khiển di chuyển đã bổ sung

Lệnh người dùng đi vào trước PI vận tốc, có giới hạn riêng:

- Vận tốc đặt: ±0,20m/s; tốc độ thay đổi đặt tối đa0,50m/s².
- Tốc độ quay đặt: ±0,80rad/s; tốc độ thay đổi đặt tối đa2rad/s².
- Lệnh Serial thử: `w/s` yêu cầu ±0,15m/s; `j/l` yêu cầu ±0,60rad/s;
  mỗi phím tạo một mục tiêu đầy đủ, không cộng dồn với phím trước.
- `z` đưa mục tiêu tiến/quay về0 và vẫn giữ cân bằng.
- Lệnh chỉ nhận khi `phase=2`, gắn đúng `epoch`, hết hạn sau500ms.
  Cần lặp phím dưới500ms để tiếp tục; hết hạn thì mục tiêu giảm về0 theo ramp.
- `d` và `x` vẫn là tắt/dừng khẩn. `z`/hết hạn không đồng nghĩa dừng motor tức thì.

Mỗi phiên mới reset mục tiêu/ramp và vẫn chờ zero-ack trước khi enable. Queue di
chuyển cũ không được áp sang phiên mới. Chưa có Bluetooth/Wi-Fi hoặc vòng giữ hướng/yaw.

## 6. Bộ trộn ưu tiên cân bằng

```text
duDia = max(0, vmax - abs(v_can_bang))
delta = clamp(yaw_dat * B/2, -duDia, +duDia)
v_trai = v_can_bang - delta
v_phai = v_can_bang + delta
```

Trung bình hai bánh vẫn là `v_can_bang` khi lệnh quay bị giới hạn. Không giảm thành
phần cân bằng để giữ một lệnh quay quá lớn. Ví dụ vchung0,15m/s và yaw0,6rad/s:
delta0,06m/s → hai bánh0,09/0,21m/s, còn trong trần0,392699. Khi vchung gần trần,
khả năng quay tự giảm về0. Đây là trộn động học theo xung, chưa phải điều khiển
yaw vòng kín từ cảm biến hướng thực.

Chỉ ramp mục tiêu của người dùng/yaw. Không thêm ramp motor độc lập quá chậm sau
PID góc: vòng cân bằng phải được phép đổi gia tốc nhanh trong giới hạn thiết kế.

## 7. Cải thiện chống bão hòa

R01 của báo cáo rà soát trước đã được xử lý: PID nhận **hướng** bão hòa -1/0/+1.
Khi ở trần dương vẫn cho tích phân với sai lệch âm, và ngược lại. Chặn phần tích
phân làm tăng bão hòa; tiếp tục giữ giới hạn gia tốc/tích phân. Test Ki=20 với cả
hai hướng xác nhận phần I giảm theo hướng thoát trần. Ki production vẫn0 cho tới
khi tinh chỉnh; test không tự chứng minh bộ hệ số Ki20 phù hợp xe.

## 8. Kiểm chứng và kế hoạch đo

Ba nhóm test trên PC đạt: nền14507 assertion, watchdog19, lọc/di chuyển/anti-windup169.
Test mới kiểm tra hết hạn/khác phiên, ramp, reset, giới hạn và bảo toàn thành phần
cân bằng của mixer. Build ESP32-S3 đạt. Chưa chạy phép thử di chuyển vật lý.

Thứ tự đo: cấu hình MKS/nguồn → STEP/PCNT hai chiều → đảo chiều gần0 → EN/timeout →
PD đứng với giá đỡ → PI vận tốc → w/s ngắn → j/l ngắn → so sánh mờ. Ghi cùng
firmware/config/tải/mặt sàn. Không tăng cùng lúc trần STEP, dòng driver và hệ số PID.

Kết quả số tái lập: [bảng](du_lieu_tinh_toan/KET_QUA_TINH_TOAN.md),
[vận tốc CSV](du_lieu_tinh_toan/thong_so_banh.csv),
[so sánh MStep](du_lieu_tinh_toan/so_sanh_vi_buoc.csv). Điện áp nguồn vẫn là trường
`null` trong JSON để tránh dùng một giả thiết nguồn chưa được người dùng xác nhận.
