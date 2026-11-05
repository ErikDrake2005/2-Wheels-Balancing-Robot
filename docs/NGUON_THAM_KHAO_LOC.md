# Nguồn tham khảo và cách áp dụng — 03/11/2026

**Bổ sung 05/11/2026:** tham khảo [NXP AN3461](https://www.nxp.com.cn/docs/en/application-note/AN3461.pdf)
cho hệ trục và tilt; triển khai phép quay proper3D và vector phương đứng, xem
[báo cáo bù gá](BAO_CAO_BU_LECH_LAP_IMU.md). Công thức sử dụng quy ước X tiến/Y trái/Z lên của xe.

Firmware dùng **ICM20602 raw accel/gyro**, DLPF trên chip và fusion trên ESP32-S3.
Không nạp firmware DMP, không dùng quaternion/yaw/pitch/roll trả từ DMP MPU6050.
`KhungDuLieuImu` là kiểu frame hiện hành; các tên MPU cũ chỉ là alias cho bench cũ.

## Nguồn chính

| Nguồn gốc | Vai trò trong thiết kế | Phạm vi áp dụng |
| --- | --- | --- |
| [TDK DS-000176 rev1.1](https://product.tdk.cn/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000176-icm-20602-v1.1.pdf) | Thanh ghi ICM20602, dải đo, DLPF và bus | Dùng bảng CONFIG/ACCEL_CONFIG2 của ICM20602, không dùng bảng MPU6050 |
| [PX4 AlphaFilter.hpp](https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/src/lib/mathlib/math/filter/AlphaFilter.hpp) | PT1 cập nhật theo dt/time constant và khởi tạo trạng thái | Đối chiếu công thức PT1; class `LocThongThap` được viết riêng, không nhập toàn bộ PX4 |
| [x-io Fusion: acceleration rejection](https://github.com/xioTechnologies/Fusion#acceleration-rejection) | Ý tưởng giảm ảnh hưởng accel khi sai lệch góc so với dự đoán lớn | Bản mới dùng sai lệch giữa hai vector phương đứng với trọng số liên tục; không port nguyên AHRS/quaternion hoặc cơ chế recovery của Fusion |
| [Betaflight filter.c](https://github.com/betaflight/betaflight/blob/master/src/main/common/filter.c) | Tham khảo cách tách nhánh lọc, PT1/biquad/notch trong điều khiển nhanh | Không sao chép mã GPL vào dự án; chưa bật notch khi chưa có phổ rung đo được |

PX4 dùng BSD-3-Clause, x-io Fusion dùng MIT, Betaflight dùng GPL. Bản triển khai hiện
tại viết độc lập từ công thức toán học tiêu chuẩn và kiến trúc đã đối chiếu; không
chép thân hàm của các dự án. Nếu sau này nhập trực tiếp mã nguồn, giữ đầy đủ license/
copyright tương ứng và kiểm tra nghĩa vụ phân phối của từng license.

Các URL nhánh `main/master` có thể thay đổi. Báo cáo ghi ngày đối chiếu; khi quyết
định nhập một thư viện trực tiếp nên pin commit cụ thể. Không tuyên bố đây là bộ
lọc “tối ưu nhất” cho mọi xe: cutoff/gating cuối cùng cần được chọn từ nhiễu và độ
trễ đo trên xe này. [Báo cáo tính toán](BAO_CAO_ICM20602_LOC_NHIEU.md) giải thích
vì sao dùng cấu trúc nhẹ, thay vì mặc định đưa một bộ Kalman/EKF lớn vào firmware.
