# Calib ĐẦU ĐUÔI và kiểm chứng góc ICM20602

Cập nhật **08/11/2026**. UI mới thay thao tác lật trái/phải 30–45° bằng chúi
ĐẦU/ngả ĐUÔI, đặt thân nằm gần 90° ở hai phía. Motor tắt trong toàn bộ quy trình.

## 1. Vì sao quy trình cũ gây nhầm?

Thuật toán cũ dùng chuyển động lật ngang quanh trục dọc thân để tìm X xe. Đó là
thao tác đo hướng lắp, khác chuyển động cân bằng tiến/lùi. Hướng dẫn “nghiêng
trái” không đủ rõ. Nếu người dùng chúi đầu/ngả sau trong bước đó, trục quay qua
hai bánh có thể bị hiểu thành trục dọc, khiến góc sai hoặc calib bị từ chối.

Bản mới dùng chuyển động **hạ ĐẦU → qua đứng → hạ ĐUÔI**, quanh trục nối hai bánh.
Chưa có log xe thật để kết luận mọi lần sai trước đây đều do cùng một nguyên nhân.

## 2. Hiểu đúng hướng và trục

Đánh dấu mũi tên **ĐẦU / TIẾN** trên khung theo hướng xe sẽ chạy tới. Phía đối
diện là **ĐUÔI / LÙI**. Đây là hướng/mặt trước và sau của khung, không phải
đỉnh/đáy xe hoặc vị trí người quan sát.

- **Hạ ĐẦU:** nghiêng thân về phía mũi tên tiến, mặt trước hướng xuống mặt đỡ,
  mặt sau hướng lên; thân cao gần nằm ngang.
- **Hạ ĐUÔI:** ngả về phía lùi, mặt sau xuống, mặt trước lên.
- **Qua đứng:** nâng đầu từ mốc nằm trước trở về gần thẳng rồi tiếp tục ngả sau;
  không lăn qua tư thế úp ngược và không lật sang phía một bánh.

Hệ thân dùng X tiến, Y qua hai bánh theo hệ tay phải, Z lên. Chúi đầu/ngả sau
là quay quanh **Y thân**. Khi gá thẳng, gyroY chiếm ưu thế, còn gia tốc X/Z thay
đổi để cho biết góc. Khi gá xéo, gyroX/gyroY và có thể gyroZ đều có thành phần.

```text
a_than     = R * a_cam_bien
omega_than = R * omega_cam_bien
```

Ví dụ toán học hàng Y của R là [0,5; 0,8660254; 0] thì tốc độ quay thân là
`0,5*gyroX + 0,8660254*gyroY`. Không lấy riêng sensor Y rồi bỏ thành phần X.
Đây chỉ là ví dụ hệ số, chưa phải kết quả đo trên xe.

Khi gần đứng, góc tĩnh tính từ `atan2(-a_X_than, sqrt(a_Y_than²+a_Z_than²))`.
Runtime truyền vector phương đứng bằng gyro XYZ rồi hiệu chỉnh bằng gia tốc
tin cậy. **Chúi ĐẦU là góc dương; ngả ĐUÔI là góc âm.** `theta`/`target` là rad,
`thetaDeg`/`targetDeg` mới thêm là độ; 0,10 rad xấp xỉ 5,73°.

## 3. Quy trình mới theo màu LED

| Màu | Thao tác |
| --- | --- |
| Cam | Chờ kiểm tra ID/cấu hình/32 mẫu mới ICM; chưa bấm nút |
| Xanh dương | Dựng gần thẳng, hai bánh cùng cao, giữ yên rồi nhấn–nhả; không tự gán tư thế này thành pitch 0 |
| Vàng lần 1 | Giữ yên để đo bias gyro và mốc tham chiếu |
| Xanh lơ | Hạ **ĐẦU**, đặt nằm mặt trước khoảng 80–90° trên mặt đỡ cứng, giữ yên, nhấn–nhả |
| Vàng lần 2 | Giữ yên ở mặt trước để lấy mốc |
| Tím | Nâng ĐẦU qua **đứng**, tiếp tục hạ **ĐUÔI** nằm phía sau khoảng 80–90°. Chuyển chậm 4–8 s, giữ yên rồi nhấn–nhả |
| Vàng lần 3 | Giữ yên ở mặt sau, lấy mốc và tính ma trận |
| Trắng | Dựng lại, hai bánh cùng cao, xem góc in theo độ và giữ gần góc đặt; không cần bấm |
| Xanh lá | READY; nút không arm, muốn chạy còn cần xác minh phần cứng và Serial `a` |
| Đỏ | Đọc lý do Serial, dựng lại và giữ yên trước khi nhấn–nhả để làm lại từ bias |

Các bước vàng đợi rung lắng 500 ms; bias cần 500 mẫu, mốc tĩnh cần 80 mẫu. Rung
làm tự thu lại trong 30 s mỗi phép đo. Không tỳ lên PCB/nút/dây làm thân đổi tư
thế. Dùng gối đỡ đúng hình học nếu bánh hoặc chi tiết nhô ra cản mặt khung nằm phẳng.

Cửa kiểm tra phần mềm **80–92°** là góc sau fitting, dư 2° phía 90° cho nhiễu/mặt
đỡ. Nó không chứng minh từng tư thế vật lý thật sự nằm trong khoảng đó, và
không phải độ chính xác calib được bảo đảm.

## 4. Không thể suy zero chính xác chỉ từ việc ước lượng 80–90°

Đặt nằm trên hai mặt/gối đỡ theo hình học khung giúp lặp lại được tư thế mà không
phải giữ 30°/45° bằng mắt. Nếu chỉ ước lượng bằng tay, sai lệch hai phía tạo sai
mốc góc. Với góc thật trước +alpha và sau −beta:

```text
delta = (alpha - beta)/2
theta_do = theta_that - delta
```

Ví dụ trước +80°, sau −90° gây lệch mốc +5°; sau fitting hai đầu vẫn hiện khoảng
+85° và −85°. Do đó, khi cả hai tư thế thật nằm trong 80–90°, sai mốc có thể tới
±5°. Đây là giới hạn có điều kiện, không phải mức sai số được code bảo đảm.
Chẳng hạn tư thế thật +75°/−105° cũng có thể fit thành ±90° và sai zero 15°.

Một trục quay và gravity không đủ tách hoàn toàn sai góc đặt khung với sai góc
gá quanh Y. Cần mặt/gối đỡ chuẩn theo khung hoặc thước vuông/thước đo góc để
kiểm độc lập nếu muốn zero chính xác. Đường cong nhiều mẫu giúp tìm trục quay,
nhưng không tự tạo ra chuẩn hình học còn thiếu.

## 5. Thuật toán trong code

`HieuChuanNghiengDoc` thay helper cũ. Dùng gyro đã trừ bias:

```text
M = sum(omega * transpose(omega) * dt)
q = sum(omega * dt)
Y = vector riêng chính của M, chọn dấu để dot(Y,q) < 0
d = a_sau - a_truoc
X = normalize(d - Y*dot(Y,d))
Z = normalize(X × Y)
R có các hàng transpose(X), transpose(Y), transpose(Z)
```

Lượt ĐẦU → ĐUÔI có pitch giảm, nên q ngược chiều Y dương. Hiệu hai vector gia
tốc giúp triệt bias cố định khi tìm hướng X; không tự hiệu chuẩn bias/scale
accelerometer. Gravity lúc người giữ chỉ kiểm Z lên và tư thế gần đứng, không
dùng làm Z để ép pitch ban đầu thành 0. Hai mốc chuẩn ±90° cho phép giữ pitch
ban đầu khác 0; nếu mốc lệch, vẫn chịu sai zero như mục 4.

Các kiểm tra gồm: norm mốc gần g, ít nhất 40 mẫu chuyển động, norm tổng góc ít
nhất 2,5 rad, năng lượng trục chính ít nhất 95%, hiệu hai mốc gần vuông góc Y,
mốc dựng gần thẳng, gia tốc ngang nhỏ, mốc nằm đủ tách, góc tích phân khớp hai
mốc trong 0,15 rad. Phép quay Rodrigues so vector mốc sau quay ngược về mốc
trước, hoạt động cả ở đúng tổng góc 180°.

Ở các mốc nằm, UI dùng vector và góc mặt phẳng XZ. Estimator cân bằng chỉ bắt
đầu sau khi xe dựng lại gần góc đặt; không dùng pitch/rate Euler suy biến tại
±90° để điều khiển xe đang nằm.

## 6. Kiểm chứng sau calib khi EN vẫn tắt

1. Xem `CALIB_DOC PASS` và `CALIB_AXIS X/Y/Z_sensor`. Hàng Y cho biết trọng số
   kết hợp gyro XYZ thành trục quay cân bằng, không cần luôn bằng [0,1,0].
2. Xem `CALIB_RETURN` rồi `thetaDeg`, so với chuẩn góc độc lập nếu muốn đánh giá
   độ chính xác. Tư thế tay đang giữ không tự là chuẩn đúng.
3. Chúi nhẹ ĐẦU: `thetaDeg` phải tăng; ngả nhẹ ĐUÔI: phải giảm. Làm chậm và dừng
   yên để kiểm tra góc tĩnh; gia tốc dịch chuyển làm nhiễu phép đo đang chuyển động.
4. Đặt lại cùng vị trí trên đồ gá, kiểm độ lặp góc. Reset và calib lại trên
   cùng hai mặt đỡ, so cả góc lẫn ma trận để phát hiện thao tác không lặp lại.
5. Kiểm chiều tiến motor theo cùng dấu ĐẦU trước khi cho phép chạy. Calib IMU
   không tự xác minh chiều DIR/EN và không tự tìm góc cân bằng cơ học.

Nếu đã có mặt chuẩn mà góc vẫn lệch, kiểm bias/scale accel, gá lỏng, nguồn,
nhiệt độ và rung. Góc cân bằng cơ học `gocCanBang_rad` vẫn xác định riêng theo
trọng tâm; đo một góc khác 0 không tự chứng minh IMU sai.

## 7. Log cần giữ khi lỗi

| Dòng | Ý nghĩa |
| --- | --- |
| `ICM_BOOT` | ID, dữ liệu mới và lỗi giao tiếp sau reset |
| `CALIB_GYRO` | Bias và trạng thái đo tĩnh |
| `CALIB_DOC` | Lý do, góc hai mốc, sweep gyro, năng lượng trục và sai lệch vector |
| `CALIB_MOC` | Ba vector gia tốc trung bình trong hệ sensor khi dựng ma trận thất bại |
| `CALIB_AXIS` | Ba hàng R sau khi thành công |
| `CALIB_RETURN` / `thetaDeg` | Góc theo độ để kiểm trực tiếp sau đổi hệ |

Nếu FAIL, đọc lý do: `luot_quay_qua_ngan`, `quay_lan_nhieu_truc`,
`nguoc_chieu_hoac_moc_dung_qua_lech`, `hai_moc_chua_dat_nam_ngang`,
`goc_gyro_khong_khop_hai_moc` hoặc `vector_gyro_khong_khop`. Lỗi I²C và mất mẫu
được in riêng. LED đỏ một mình không đủ xác định nguyên nhân.

## 8. Kiểm thử ngày 08/11/2026

Sáu nhóm native PASS. Nhóm pitch **2.038 điều kiện**: 45 góc lắp, mốc 80°/85°/90°,
pitch người giữ −6°/0°/+6°, nối estimator để kiểm góc/rate, đúng 180°, đảo chiều,
thiếu góc, nhiều trục, NaN và trường hợp 80°/90° gây sai zero 5°.
Nhóm boot/UI **44 điều kiện** dùng Wire/nút/RGB giả, gồm đầy đủ ĐẦU → ĐUÔI và
rung ở mỗi mốc được tự thu lại.

Build `lolin_s3_mini`: PASS; RAM **57.044 byte**, flash **476.206 byte**. Chưa nạp
hay đo góc trên xe trong đợt sửa này. Kết quả tổng hợp không thay phép kiểm mặt
gá, dấu và độ lặp trên xe thật.

Bản ghi kiểm thử và SHA-256 firmware:
`docs/du_lieu_tinh_toan/PHIEN_KIEM_THU_CALIB_DOC_2026-11-08.json`.

Nguồn code: [UI_Calib.cpp](../src/calib/UI_Calib.cpp),
[HieuChuanNghiengDoc.cpp](../lib/estimation/HieuChuanNghiengDoc.cpp),
[ComplementaryEstimator.cpp](../lib/estimation/ComplementaryEstimator.cpp),
[CalibConfig.h](../include/config/CalibConfig.h).
