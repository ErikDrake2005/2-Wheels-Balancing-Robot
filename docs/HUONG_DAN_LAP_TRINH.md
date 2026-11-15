# Hướng dẫn lập trình xe cân bằng

> Cập nhật lọc và di chuyển 03/11/2026: [ICM20602](BAO_CAO_ICM20602_LOC_NHIEU.md), [MKS42C/17HS4401S](BAO_CAO_MKS42C_17HS4401S.md). Runtime không DMP; PT1 chỉ nhánh D; có lệnh di chuyển có hạn; R01 đã sửa.

Cập nhật theo rà soát **08/11/2026**. Cách nạp/vận hành nằm trong
[hướng dẫn sử dụng](HUONG_DAN_SU_DUNG.md); trạng thái hoàn thành nằm trong
[checklist](CHECKLIST_TINH_NANG.md). Phần giải thích toàn bộ hàm được tra cứu ở
[danh mục hàm](DANH_MUC_HAM.md).

Mỗi phần được đọc theo thứ tự: mục đích → biến sử dụng → đoạn mã → giải thích →
kết quả cần kiểm tra. Các ví dụ dưới đây bám theo chương trình hiện có.

## 1. Quy ước tên biến

- Dùng tiếng Việt không dấu, viết theo kiểu `gocNghien`, `soXung`, `trangThaiXe`.
- Đại lượng vật lý có hậu tố đơn vị: `_rad`, `_radps`, `_m`, `_mps`, `_mps2`, `_s`, `_ms`, `_us`.
- Biến đúng/sai có tên thể hiện điều kiện: `daKhoiTao`, `gocHopLe`, `dungPidMo`.
- Biến thành viên riêng của lớp có `_` ở đầu: `_camBienGoc`, `_huongLen`.
- Tên API thư viện như `Wire.begin`, `Serial.printf`, `xQueueReceive` theo thư viện.
  Các tên kiểu dữ liệu và tên file hiện có dùng để tìm module tương ứng.
- Chú thích và phần giải thích dùng tiếng Việt có dấu để dễ đọc.

| Biến | Ý nghĩa |
| --- | --- |
| `maNhanDang` | Byte nhận dạng cảm biến |
| `ketQuaDoc` | Trạng thái thành công hoặc lỗi khi đọc I²C |
| `khungDuLieu` | Một mẫu gia tốc, tốc độ góc, nhiệt độ và trạng thái |
| `trangThaiXe` | Góc, tốc độ nghiêng, vị trí và vận tốc đã ước lượng |
| `gocNghien_rad` | Góc nghiêng hiện tại, đơn vị radian |
| `tocDoNghien_radps` | Tốc độ thay đổi góc nghiêng, radian/giây |
| `buocThoiGian_s` | Khoảng thời gian giữa hai lần tính, đơn vị giây |
| `saiLechGoc_rad` | Góc nghiêng trừ góc mục tiêu theo quy ước của bộ điều khiển |
| `lenhBanh` | Lệnh tốc độ STEP cho hai bánh và thời hạn hiệu lực |

## 2. Đọc ID của ICM20602

Mở `lib/imu/src/Icm20602Driver.cpp`, tìm `Icm20602Driver::begin()`.
Hàm này khởi tạo cảm biến và trả về `true` khi các bước khởi tạo thành công.
Theo sơ đồ hiện tại, trước khi mở I²C SDA10/SCL11, hàm đặt CS GPIO9 lên HIGH,
SA0 GPIO8 xuống LOW cho `0x68` (hoặc HIGH nếu cấu hình `0x69`). Các GPIO này
được khai báo ở `PinConfig.h`; hàm `cacChanChucNangKhongTrung()` và `static_assert`
phát hiện trùng chân lúc biên dịch. Motor 1 là phải (DIR12/STEP13), motor 2 là
trái (DIR5/STEP6); đổi chân và đổi cực tính chiều là hai việc riêng.
Đoạn sau kiểm tra nhận dạng, nằm sau bước khởi tạo I²C:

```cpp
uint8_t maNhanDang = 0;
const IcmI2cStatus ketQuaDoc = docNhieuByte(0x75, &maNhanDang, 1);

if (ketQuaDoc != IcmI2cStatus::OK) {
    return false;
}
if (maNhanDang != 0x12) {
    return false;
}
```

Giải thích từng phần:

1. `uint8_t` chứa số nguyên không dấu 8 bit, có giá trị từ 0 đến 255. ID có một byte
   nên kiểu này phù hợp. `maNhanDang = 0` đặt giá trị ban đầu.
2. `0x75` là địa chỉ thanh ghi `WHO_AM_I` bên trong cảm biến. Địa chỉ thiết bị trên
   bus I²C được lấy riêng từ `diaChiI2cMpu` trong `include/config/ImuConfig.h`.
3. `&maNhanDang` lấy địa chỉ vùng nhớ của biến. Hàm `docNhieuByte()` dùng địa chỉ này
   để ghi byte nhận được vào chính biến `maNhanDang`.
4. Tham số `1` yêu cầu đọc đúng một byte.
5. `ketQuaDoc` chứa trạng thái giao tiếp. `const` cho biết biến này không bị gán lại.
6. `return false` kết thúc hàm `begin()` khi giao tiếp lỗi hoặc ID khác `0x12`.
   Nếu cả hai kiểm tra đều qua, chương trình tiếp tục reset và cấu hình cảm biến.

Để quan sát khi học, có thể tạm thêm dòng này ngay sau lệnh đọc, trong `begin()`:

```cpp
Serial.printf("ICM20602: maNhanDang=0x%02X, ketQuaDoc=%u\n",
    static_cast<unsigned>(maNhanDang),
    static_cast<unsigned>(ketQuaDoc));
```

Kết quả mong đợi: `maNhanDang=0x12, ketQuaDoc=0`. Giá trị `0` của `ketQuaDoc`
là `IcmI2cStatus::OK`. Chỉ chuyển sang xem dữ liệu đo khi bước nhận dạng thành công.
`Serial` đã được khởi tạo trong `RobotApp::begin()` trước khi gọi driver cảm biến.

## 3. Đọc dữ liệu thô và đổi đơn vị

Trong cùng file, tìm `Icm20602Driver::read()`.
Mỗi lần đọc tạo một `khungDuLieu` mới. Driver đọc 14 byte bắt đầu từ thanh ghi `0x3B`:
gia tốc X/Y/Z, nhiệt độ, tốc độ góc X/Y/Z. Mỗi đại lượng dùng hai byte.

```cpp
const int16_t giaTocThoX = ghepSo16Bit(boDem[0], boDem[1]);
const int16_t tocDoGocThoY = ghepSo16Bit(boDem[10], boDem[11]);
```

`boDem` là mảng byte nhận được. `ghepSo16Bit()` ghép byte cao và byte thấp thành
số nguyên có dấu, vì cảm biến đo được chuyển động theo cả hai chiều.

Với dải ±4 g và ±500 độ/giây đang cấu hình:

```cpp
constexpr float heSoDoiGiaToc = 9.80665f / 8192.0f;
constexpr float soDonViThoMoiDoTrenGiay = 65.5f;
constexpr float heSoDoiDoSangRad = 0.01745329252f;

khungDuLieu.giaTocX_mps2 = giaTocThoX * heSoDoiGiaToc;
khungDuLieu.tocDoGocY_radps =
    (tocDoGocThoY / soDonViThoMoiDoTrenGiay - _doLechConQuay_dps[1]) *
    heSoDoiDoSangRad;
```

`constexpr` biểu diễn hằng số tính được khi biên dịch. Hậu tố `f` tạo hằng kiểu
`float`. Với gyro, thứ tự xử lý là: đổi dữ liệu thô sang độ/giây → trừ độ lệch
đã hiệu chuẩn → đổi sang radian/giây. Các hệ số này đi cùng dải đo; khi đổi dải đo,
cần cập nhật cả cấu hình thanh ghi và hệ số chuyển đổi.

Khi cảm biến đứng yên, trục Z hướng lên, gia tốc Z gần `9,81 m/s²`, X/Y gần 0;
gyro sau hiệu chuẩn gần 0. Gia tốc vẫn bao gồm trọng lực. Sai số thực tế phải đo
trên cảm biến, nên không kiểm tra bằng phép so sánh bằng tuyệt đối với số 0.

## 4. Từ dữ liệu IMU đến góc nghiêng

Mở `lib/estimation/ComplementaryEstimator.cpp`, tìm `update()`.
Hai tham số là `khungDuLieu` và `trangThaiXe`:

```cpp
bool update(const KhungDuLieuImu& khungDuLieu, TrangThaiXe& trangThaiXe);
```

`const ...&` truyền tham chiếu để đọc mẫu mà không sao chép hoặc sửa mẫu.
`TrangThaiXe&` cho phép hàm cập nhật trạng thái của xe ở nơi gọi.
Kiểu hiện hành là `KhungDuLieuImu`; alias `KhungDuLieuMPU` chỉ dành cho bench cũ.

Bản 05/11/2026 dùng `HeTrucImu` để đổi cả accel và gyro sang hệ xe trước khi tính.
Ví dụ phép đổi vector:

```cpp
const Vec3Imu giaTocXe = heTruc.chuyenSangXe(
    {khungDuLieu.giaTocX_mps2, khungDuLieu.giaTocY_mps2, khungDuLieu.giaTocZ_mps2});
const float gocTuGiaToc_rad = std::atan2(-giaTocXe.x, std::hypot(giaTocXe.y, giaTocXe.z));
```

`heTruc` là một đối tượng `HeTrucImu` đã được cấu hình. Khi `batUiCalib=true`,
app dùng ma trận đo ở UI trong lần khởi động hiện tại; khi tắt UI, góc lắp lấy
từ `ImuMountConfig.h`. Không chọn riêng một trục X/Y khi PCB đặt chéo. Estimator
nhớ vector `_huongLen`, xoay nó bằng ba gyro và hiệu chỉnh theo accel. Khi gần
thẳng và chỉ pitch, phép truyền này tương đương tích phân gyroY; khi có roll/yaw
nó xử lý thêm liên kết giữa các trục. Xem [công thức đầy đủ và cách hiệu chuẩn](BAO_CAO_BU_LECH_LAP_IMU.md).

Quan sát `theta` trong log: khi nghiêng thân về trước theo hệ trục mặc định, góc
phải tăng theo chiều dương; trở về thẳng thì gần góc ban đầu. Chỉ ghép điều khiển
sau khi đã xác nhận dấu góc và gyro phù hợp với cách lắp cảm biến.

### Phân biệt ba việc hiệu chỉnh

1. **Bias gyro:** `hieuChinhConQuay(500)` lấy trung bình một khoảng đứng yên;
   `ThongKeMauImu` kiểm phương sai gyro/gia tốc. Khoảng rung bị bỏ và thu lại trong
   30 s. Trước calib, `kiemTraSauKhoiDong()` xác nhận ID/cấu hình và 32 mẫu mới.
   Lật xe không phải cách đo bias; lượt lật chỉ bắt đầu sau khi đã trừ bias.
2. **Góc gá sensor so với khung:** `HieuChuanNghiengDoc::themMau()` tích lũy
   moment gyro XYZ và tổng góc của lượt ĐẦU nằm → qua đứng → ĐUÔI nằm.
   `tinhMaTran()` tìm Y bằng power iteration; tổng góc lượt trước → sau ngược +Y.
   Lấy `d=a_sau-a_truoc`, `X=normalize(d-Y*dot(Y,d))`, `Z=X×Y`.
   Ma trận ba hàng X/Y/Z đổi cả accel lẫn gyro sang hệ xe.
3. **Góc cân bằng cơ học:** `gocCanBang_rad` là mục tiêu nền của vòng điều khiển.
   Người đang đỡ xe tạo thêm lực nên tư thế họ gọi là thẳng chưa cho biết mục tiêu
   cân bằng thực. UI không tự ghi biến này; mặc định 0 cần kiểm tra/chỉnh riêng.

ĐẦU là hướng tiến do người dùng chọn và đánh dấu; ĐUÔI đối diện. Hạ ĐẦU là pitch
dương, hạ ĐUÔI là pitch âm. Thao tác quay quanh trục nối hai bánh. Gyro Y là tốc
độ quay quanh Y; khi PCB đặt xéo chuyển động có thể xuất hiện trên cả gyro XYZ,
nên không chọn cứng sensor Y và không lấy accel Y làm góc pitch.

Hai mốc nằm đúng hình học cho phép giữ pitch ban đầu khác0 vì gravity người giữ
không bị dùng để đặt Z. Nếu cả hai góc thật đều trong 80–90°, sai mốc có thể tới
khoảng ±5° khi một bên 80° bên kia 90°. Cửa sổ 80–92° sau fit không xác minh góc thật. Sai lệch này không tự lộ
ra qua các góc fit đối xứng. Cần mặt đỡ/đồ gá chuẩn theo khung để đo lặp lại đúng.

Mốc gần±90° được xử lý bằng vector; không đưa estimator/PID cân bằng vào hoạt
động khi xe còn nằm. LED trắng đợi80 mẫu đứng yên gần góc đặt
(`|pitch-target|≤0,10 rad`, `|roll|≤0,10 rad`), có log góc theo độ/rad. Sau READY
và khi EN vẫn tắt, chúi đầu góc nhỏ phải báo theta dương, ngả sau phải âm; trở về
cùng tư thế phải gần số đo cũ. Nó không đo trọng tâm hoặc chứng minh xe tự cân bằng.

`HeTrucImu::tuHaiVector()` là phương pháp offline khác, dùng gravity ở **tư thế
chuẩn đã biết** và gyro nghiêng tiến. UI hai mốc hiện không dùng hàm đó vì nó đặt
Z theo gravity của tư thế người giữ. Xem [UI_Calib](HUONG_DAN_UI_CALIB.md) và
[báo cáo ĐẦU/ĐUÔI](CALIB_DAU_DUOI_VA_KIEM_CHUNG_GOC_2026-11-08.md).

Kiểm chứng 08/11/2026: sáu nhóm native và build PASS, gồm 2.038 kiểm tra calib pitch
và 44 kiểm tra boot/UI. Chưa upload hoặc đo trên xe; mặt chuẩn và dấu motor vẫn
phải kiểm chứng thực tế.

## 5. Các phần tiếp theo cần đọc

| Thứ tự | File/hàm | Nội dung cần hiểu |
| --- | --- | --- |
| 1 | `WheelPulseCounter.cpp`, `readAxis()` | Lấy số đếm PCNT, tính chênh lệch và cộng vào `tongSoXung` |
| 2 | `DifferentialDrive.cpp`, `update()` | Đổi số xung thành `viTriDoc_m`, lọc `vanTocChuaLoc_mps` |
| 3 | `AnglePidController.cpp`, `update()` | Tính `saiLechGoc_rad`, tích phân, gia tốc và chống bão hòa |
| 4 | `AnglePidController.cpp`, `scheduledGains()` | Chuẩn hóa đầu vào, tính `trongSoLuat`, điều chỉnh `heSoPid` |
| 5 | `CascadeController.cpp`, `update()` | PI vận tốc tạo góc đặt; PID góc tạo gia tốc; tích phân thành lệnh bánh |
| 6 | `RobotApp.cpp` | Ghép ngắt, task, queue, lệnh bật/tắt và báo cáo |

Trong PID góc, quy ước `saiLechGoc_rad = gocNghien_rad - gocMucTieu_rad` được chọn
để nghiêng trước tạo gia tốc bánh tiến. Cần hiểu dấu của cả góc và lệnh motor khi
so sánh với công thức PID trong tài liệu khác.

## 6. Từ số xung đến vận tốc bánh

Mở `DifferentialDrive.cpp`, đọc `metersPerStep()` trước. Hàm này dùng các hằng số
cơ khí để tính quãng đường tương ứng một cạnh STEP:

```text
soXungMoiVong = 200 × 32 × 1 = 6400
quangDuongMoiXung_m = 2 × pi × 0,04 / 6400 ≈ 0,00003927
```

Trong `update()`, hiệu giữa hai tổng xung tạo số xung của khoảng lấy mẫu. Dấu âm
nghĩa là quay ngược quy ước tiến. Hai bánh được lấy trung bình để tính chuyển động dọc.

```cpp
const double soXungTrungBinh =
    ((khungDuLieu.trai - _khungXungTruoc.trai) +
     (khungDuLieu.phai - _khungXungTruoc.phai)) * 0.5;

const float vanTocChuaLoc_mps = static_cast<float>(
    soXungTrungBinh * quangDuongMoiXung_m / buocThoiGian_s);
```

Ví dụ mỗi bánh tăng 10 xung trong 0,005 s: vận tốc chưa lọc khoảng 0,07854 m/s.
Code còn lọc vận tốc nên `vanTocDoc_mps` không lập tức bằng giá trị chưa lọc.
Lần đầu chỉ biết tổng xung, chưa có khoảng thời gian để tính vận tốc, vì vậy cờ
`vanTocHopLe` chưa bật. Đây là lý do cần hai mẫu xung trước khi xét arm.

PCNT nhận cạnh STEP tại GPIO; `cmd` là mục tiêu STEP/s. Hãy theo dõi hai dữ liệu
này đúng ý nghĩa: đổi DIR/chế độ phát có khoảng chờ nên số xung tức thời có thể
khác phép nhân đơn giản `cmd × dt`. Việc motor bám xung vẫn cần đo trên xe.

## 7. Hiểu đầu vào và đầu ra PID góc bằng một phép tính

Mở `AnglePidController.cpp`, tìm `update()`.

| Tham số | Công dụng |
| --- | --- |
| `gocNghien_rad` | Góc đo/ước lượng hiện tại |
| `tocDoNghien_radps` | Gyro của trục nghiêng dùng cho D |
| `gocMucTieu_rad` | Góc đặt, có thể do PI vận tốc tạo |
| `buocThoiGian_s` | Thời gian thực giữa hai lần tính |
| `dungPidMo` | Chọn luật mờ hoặc hệ số cơ sở |
| `huongBaoHoaVanToc` | -1/0/+1: trần âm, chưa bão hòa, trần dương; cho phép xả I đúng hướng |

Ví dụ với hệ số cố định hiện tại, Ki=0, thân nghiêng trước 0,01 rad, gyro=0,
góc đặt=0 và dt=0,005 s:

```cpp
AnglePidController boPidGoc;
const float giaTocLenh_mps2 = boPidGoc.update(
    0.01f, 0.0f, 0.0f, 0.005f, false);
// Kp=30: giaTocLenh_mps2 xấp xỉ 0,3 m/s².
```

Ở bước tích phân vận tốc của CascadeController, nếu vận tốc lệnh trước là 0:

```text
vanTocLenh_mps = 0 + 0,3 × 0,005 = 0,0015 m/s
tanSoStep = 0,0015 / 0,00003927 ≈ 38,2 STEP/s
```

Đây là minh họa một lượt tính đã hoạt động. Riêng lần đầu `CascadeController::update()`
sau reset chủ động trả lệnh 0 để đồng bộ khởi động. Lệnh 38,2 STEP/s thuộc vùng xung
thưa: driver tích lũy phần bước rồi phát xung hoàn chỉnh khi đủ một bước.

**R01 đã sửa:** cho xả I theo hướng thoát trần và có regression test Ki20 hai chiều.
Ki production vẫn0; cần tinh chỉnh và đo trên xe trước khi dùng Ki khác0.

## 8. Đọc phần mờ theo từng vòng lặp

`scheduledGains()` thực hiện bốn bước:

1. Chia sai lệch góc/tốc độ nghiêng cho `mienSaiLechMo_rad` và `mienTocDoMo_radps`, rồi chặn [-1,1].
2. `membership()` tính độ thuộc vào 5 tập NB/NS/Z/PS/PB.
3. Với mỗi hàng/cột, nhân hai độ thuộc thành `trongSoLuat`. Cộng trọng số nhân
   từng phần tử của ba bảng vào `tongHeSoTiLe/TichPhan/ViPhan`.
4. Chia cho `tongTrongSo`, nhân với Kp/Ki/Kd cơ sở rồi trả `heSoPid`.

Hàng là sai lệch chuẩn hóa; cột là gyro chuẩn hóa. Đầu vào thứ hai là tốc độ nghiêng
đo được, không thêm phép lấy đạo hàm số của góc. Khi Ki cơ sở bằng 0, nhân bất kỳ
hệ số mờ nào thì Ki cuối vẫn bằng 0. Muốn học cơ chế, theo dõi `K` trong log khi
đang chạy; lúc tắt/reset, hệ số báo cáo có thể bằng 0 trước lượt cập nhật tiếp theo.

## 9. Theo một mẫu qua ba task

Đọc theo chuỗi hàm này trong `RobotApp.cpp`:

```text
imuInterrupt() → imuLoop() → queue IMU
                                  ↓
                          controlLoop()
                          góc + xung + safety
                          PI → PID → STEP/s
                                  ↓
                          queue điều khiển
                                  ↓
                           motorLoop()
                          phiên + hạn lệnh
                          apply()/service()
```

`xQueueOverwrite()` thay bản cũ bằng bản mới. `xQueueReceive()` nhận rồi lấy bản ra;
`xQueuePeek()` lấy bản sao nhưng giữ dữ liệu trong queue. Mỗi task xử lý bản dữ liệu
của mình, nhờ vậy controller không cần trực tiếp chạm vào bộ phát STEP.

Phân biệt vòng lặp Arduino `loop()` với các vòng `imuLoop/controlLoop/motorLoop`:
Arduino loop phụ trách console; ba task FreeRTOS được tạo bằng `xTaskCreate()`.
Thêm `delay()` vào console và thêm nó vào motor task có tác động rất khác nhau.

### Vì sao bật motor cần mã phiên?

Giả sử phiên 10 có lệnh 2000 STEP/s còn trong queue. Gửi `d` làm phiên đổi; gửi
`a` tạo phiên mới nữa. ControlTask phải reset và tạo lệnh 0 mang mã mới. Lệnh 2000
STEP/s mang mã 10 bị từ chối dù chưa hết hạn thời gian. Biến `maPhien` quản lý lần
bật, còn `soThuTuLenh` quản lý từng lệnh trong một phiên.

Học bằng cách theo dõi: `PhienCanBang::yeuCauBat()` → snapshot `maPhien/cheDo` →
reset trong ControlTask → `lenhKhoiDongHopLe()` → `enable()` → `DangChay`.

## 10. Chọn đúng nơi lưu một biến mới

| Trường hợp | Nơi phù hợp |
| --- | --- |
| Hệ số không đổi trong lúc chạy | `include/config/`, thường dùng `constexpr` |
| Giá trị trung gian của một phép tính | Biến local trong hàm, ví dụ `saiLechGoc_rad` |
| Giá trị phải nhớ qua nhiều mẫu | Thành viên đối tượng, ví dụ `_tichPhanGoc_rad_s` |
| Kết quả cần truyền giữa task | Thêm vào struct snapshot/frame, gửi bằng queue |
| Dữ liệu chung ISR và task | Khóa ngắn phù hợp như các timestamp IMU; không chỉ thêm `volatile` |

Ví dụ muốn log thêm norm gia tốc: dữ liệu đã có ở `TrangThaiXe.doLonGiaToc_mps2`,
được gán trong estimator và in ở console. Hãy đọc đường đi đó trước khi thêm một
biến global thứ hai chứa cùng giá trị. Không gọi Serial từ PID hoặc ISR để lấy log.

## 11. Quy trình tự sửa và tự kiểm tra

1. Đọc hợp đồng hàm trong [danh mục hàm](DANH_MUC_HAM.md): đơn vị, đầu vào, đầu ra, nơi gọi.
2. Viết một ví dụ số hoặc kịch bản cụ thể, gồm trường hợp bình thường và lỗi.
3. Sửa một ý chính, giữ cách đặt tên biến tiếng Việt không dấu.
4. Với logic tính toán/phiên, thêm test vào nhóm native thích hợp. Với I/O, cần
   mock giao thức hoặc phép đo chip; test công thức không thay được test peripheral.
5. Chạy `test/run_native.ps1` và `pio run -e lolin_s3_mini`.
6. Cập nhật [checklist](CHECKLIST_TINH_NANG.md), ghi rõ mới đạt trên PC hay đã đo trên xe.

Các file task/Logger/RobotCore/Kalman đang rỗng là khung dự kiến; triển khai thực
đã nằm trong những module được nêu ở trên. Không cần điền mọi file rỗng để bắt đầu
hiểu và thử firmware này.
