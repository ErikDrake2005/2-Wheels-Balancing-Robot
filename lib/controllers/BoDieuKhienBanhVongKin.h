#pragma once
#include "model/RobotState.h"
#include "config/MotorConfig.h"

struct TrangThaiVongKinBanhXe {
    float saiLechViTriTrai_m = 0.0f;
    float saiLechViTriPhai_m = 0.0f;
    float saiLechGocTrai_rad = 0.0f;
    float saiLechGocPhai_rad = 0.0f;
    float vanTocBuTrai_mps = 0.0f;
    float vanTocBuPhai_mps = 0.0f;
    float vanTocLenhTrai_mps = 0.0f;
    float vanTocLenhPhai_mps = 0.0f;
    bool biKetBanh = false;
};

// Bộ xử lý logic bánh xe vòng kín trên ESP32 dành cho cấu hình TMC2209 + MT6701.
// Kết hợp lệnh vận tốc feedforward từ bộ trộn cân bằng với phản hồi vị trí/vận tốc
// thực từ encoder MT6701 để:
// 1. Bù trượt bước / trễ pha cơ-điện của từng bánh (P vị trí + PI vận tốc).
// 2. Đồng bộ điểm tham chiếu với vị trí encoder thực ngay khi arm/reset (không giật).
// 3. Phát hiện kẹt rotor / mất bước kéo dài để ngắt an toàn.
class BoDieuKhienBanhVongKin {
public:
    void reset();
    bool capNhat(float vanTocDatTrai_mps, float vanTocDatPhai_mps,
                 const TrangThaiXe& trangThaiXe, float buocThoiGian_s,
                 float vanTocToiDa_mps, TrangThaiVongKinBanhXe& ketQua);
    const TrangThaiVongKinBanhXe& trangThai() const { return _trangThai; }
    bool daDongBo() const { return _daDongBo; }
private:
    struct TrucBanh {
        float viTriThamChieu_m = 0.0f;
        float tichPhanVanToc_m = 0.0f;
    };
    float capNhatMotBanh(TrucBanh& truc, float vanTocDat_mps, float viTriDo_m, float vanTocDo_mps,
                         float buocThoiGian_s, float vanTocToiDa_mps,
                         float& saiLechViTri_m, float& saiLechGoc_rad, float& vanTocBu_mps);
    TrucBanh _banhTrai{};
    TrucBanh _banhPhai{};
    TrangThaiVongKinBanhXe _trangThai{};
    float _thoiGianLoiBam_s = 0.0f;
    bool _daDongBo = false;
};
