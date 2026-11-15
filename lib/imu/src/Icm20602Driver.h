#ifndef ICM20602DRIVER_H
#define ICM20602DRIVER_H
#include <Arduino.h>
#include <Wire.h>
#include "model/SensorFrame.h"
#include "config/ImuConfig.h"
#include "config/PinConfig.h"

enum class IcmI2cStatus : uint8_t {
    OK = 0,
    BUFFER_OVERFLOW = 1,
    ADDR_NACK = 2,
    DATA_NACK = 3,
    OTHER_ERROR = 4,
    TIMEOUT = 5,
    READ_SIZE_MISMATCH = 6,
    INVALID_PARAM = 7
};
struct BaoCaoKiemTraIcm {
    const char* ketQua = "chua_kiem_tra";
    uint16_t soMauTot = 0, soMauLoi = 0;
    uint32_t thoiGian_ms = 0;
    KhungDuLieuImu mauCuoi{};
};
class Icm20602Driver {
public:
    explicit Icm20602Driver(TwoWire& busI2c);
    bool begin();
    // Kiểm tra sau reset, trước calib/task: ID, cấu hình và chuỗi mẫu data-ready mới.
    // Không phải MEMS self-test, không yêu cầu giữ xe đúng góc hoặc đã trừ bias.
    bool kiemTraSauKhoiDong(BaoCaoKiemTraIcm& baoCao);
    IcmI2cStatus maLoiI2cCuoi() const { return _maLoiI2cCuoi; }
    uint8_t thanhGhiCuoi() const { return _thanhGhiCuoi; }
    uint8_t maNhanDang() const { return _maNhanDang; }
    // Chỉ task sở hữu driver lấy snapshot rồi gửi qua queue, console không đọc trực tiếp.
    ChanDoanGiaoDichI2c giaoDichDocCuoi() const { return _giaoDichDocCuoi; }
    ChanDoanGiaoDichI2c giaoDichDocLoiCuoi() const { return _giaoDichDocLoiCuoi; }
    // Đọc một mẫu accel, gyro và nhiệt độ.
    //
    // Điều kiện gọi:
    // - Chỉ một task được sử dụng driver tại một thời điểm.
    // - Không gọi từ ISR.
    //
    // Kết quả:
    // - true: đọc đủ dữ liệu; khungDuLieu.trangThai = HopLe.
    // - false: mẫu không hợp lệ; bên gọi phải bỏ mẫu.
    //
    // Quy ước dữ liệu:
    // - Accel: m/s², vẫn bao gồm ảnh hưởng của trọng lực.
    // - Gyro: rad/s, theo trục ICM20602.
    // - Chưa ánh xạ dữ liệu sang hệ trục của xe.
    //
    // Trách nhiệm của tầng gọi:
    // - Điền timestamp và số thứ tự mẫu sau khi đọc thành công.
    // - Kiểm tra tuổi dữ liệu trước khi đưa vào bộ điều khiển.
    bool read(KhungDuLieuImu& khungDuLieu);
    // Runtime ISR: một burst 15 byte từ INT_STATUS qua accel/temp/gyro.
    // ACK ngắt giữ mức và kiểm data-ready; timestamp vẫn do ISR/tầng app cung cấp.
    bool docMauTheoNgat(KhungDuLieuImu& khungDuLieu);
    // Poll data-ready trước ISR. false + LoiIO: lỗi bus; false + DuLieuCu: chưa có mẫu.
    // Luôn xóa frame đầu ra để không dùng lại dữ liệu cũ.
    bool docMauMoi(KhungDuLieuImu& khungDuLieu);
    // Chỉ gọi trước khi cân bằng, khi xe đứng yên và motor đã disable.
    // Hàm thu nhiều mẫu nên có thể mất vài giây.
    // Không gọi từ ISR hoặc trong vòng PID.
    bool hieuChinhConQuay(uint16_t soMau);
    // Dùng nội bộ trong task sở hữu ICM20602 (ImuTask/boot fallback) để nạp hoặc đọc offset gyro (dps).
    void datDoLechConQuay_dps(float bx, float by, float bz);
    void layDoLechConQuay_dps(float& bx, float& by, float& bz, bool& daHieuChinh) const;
private:

    TwoWire& _busI2c;
    float _doLechConQuay_dps[3]={0.0f, 0.0f, 0.0f};
    bool _daKhoiTao = false;
    bool _daHieuChinhConQuay = false;
    IcmI2cStatus _maLoiI2cCuoi = IcmI2cStatus::OK;
    uint8_t _thanhGhiCuoi = 0, _maNhanDang = 0;
    ChanDoanGiaoDichI2c _giaoDichDocCuoi{}, _giaoDichDocLoiCuoi{};
    bool ghiThanhGhi(uint8_t diaChi, uint8_t giaTri);
    IcmI2cStatus docNhieuByte(uint8_t diachiBatDau, uint8_t* duLieu, size_t soByte);
    bool giaiMaMau(const uint8_t* boDem, KhungDuLieuImu& khungDuLieu) const;
};
#endif
