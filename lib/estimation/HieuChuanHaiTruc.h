#pragma once
#include "HeTrucImu.h"

struct BaoCaoCalibHaiTruc {
    const char* lyDo = "chua_tinh";
    float gocMocTruoc_rad = 0, gocMocSau_rad = 0, gocQuayPitchGyro_rad = 0;
    float gocMocTrai_rad = 0, gocMocPhai_rad = 0, gocQuayRollGyro_rad = 0;
    float tyLeNangLuongPitch = 0, tyLeNangLuongRoll = 0;
    float lechVuongGoc_rad = 0;       // Độ lệch góc giữa trục X_acc và Y_acc so với 90 độ.
    float lechTrucXGyro_rad = 0;      // Độ lệch giữa X_acc và X_gyro.
    float lechTrucYGyro_rad = 0;      // Độ lệch giữa Y_acc và Y_gyro.
};

// Hiệu chuẩn ma trận lắp 3D bằng CẢ 2 TRỤC (Pitch: ĐẦU->ĐUÔI + Roll: TRÁI->PHẢI):
// - Chốt độc lập cả hai trục X (tiến) và Y (trái) từ hiệu trọng lực tĩnh, kiểm tra chéo
//   với trục quay gyro của từng lượt, sau đó trực giao hóa đối xứng (Wahba / Polar Decomposition)
//   để chia đều sai số góc cho cả hai trục và triệt nhiễu xuyên trục Pitch <-> Roll.
class HieuChuanHaiTruc {
public:
    void themMauPitch(const Vec3Imu& gyro, float khoangThoiGian_s);
    void themMauRoll(const Vec3Imu& gyro, float khoangThoiGian_s);

    bool tinhMaTran(const Vec3Imu& giaTocDung,
                    const Vec3Imu& giaTocTruoc, const Vec3Imu& giaTocSau,
                    const Vec3Imu& giaTocTrai, const Vec3Imu& giaTocPhai,
                    MaTranQuayImu& ketQua,
                    BaoCaoCalibHaiTruc* baoCao = nullptr) const;
private:
    struct BoTichLuyGyro {
        float moment[3][3]{};
        Vec3Imu tongGoc{};
        unsigned soMau = 0;
        void them(const Vec3Imu& gyro, float dt_s);
        bool timTrucChinh(Vec3Imu& truc, float& tyLeNangLuong) const;
    };
    BoTichLuyGyro _pitch{};
    BoTichLuyGyro _roll{};
};
