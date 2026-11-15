#pragma once
#include "model/SensorFrame.h"

enum class KetQuaChotMauImu { ChapNhan, KhongCoMau, ThieuNgat, NgatChong, QuaChuKyMau };

// Đọc status để ACK/phục hồi không đồng nghĩa có timestamp nguồn hợp lệ.
// Mẫu poll khi mất IRQ tuyệt đối không được biến thành mẫu góc mới để arm.
inline KetQuaChotMauImu chotMauImu(KhungDuLieuImu& mau, bool coThongBao,
    uint32_t soNgatTruoc, uint32_t soNgatSau, uint64_t thoiDiemNgat_us,
    uint64_t lucDocXong_us, uint32_t chuKyMau_us) {
    if (mau.trangThai != TrangThaiKhungDuLieuImu::HopLe) return KetQuaChotMauImu::KhongCoMau;
    if (!coThongBao || thoiDiemNgat_us == 0 || soNgatTruoc != soNgatSau) {
        mau = KhungDuLieuImu{};
        mau.trangThai = TrangThaiKhungDuLieuImu::DuLieuCu;
        return coThongBao && soNgatTruoc != soNgatSau ? KetQuaChotMauImu::NgatChong : KetQuaChotMauImu::ThieuNgat;
    }
    // Khi INT latch, cảm biến vẫn cập nhật thanh ghi dù chưa có cạnh IRQ kế tiếp.
    // Không gán dữ liệu mới hơn cho IRQ cũ nếu đọc hết một chu kỳ mẫu.
    if (!chuKyMau_us || lucDocXong_us < thoiDiemNgat_us || lucDocXong_us-thoiDiemNgat_us >= chuKyMau_us) {
        mau = KhungDuLieuImu{};
        mau.trangThai = TrangThaiKhungDuLieuImu::DuLieuCu;
        return KetQuaChotMauImu::QuaChuKyMau;
    }
    mau.thoiDiemLayMau_us = thoiDiemNgat_us;
    mau.soMauLayDuoc = soNgatTruoc;
    return KetQuaChotMauImu::ChapNhan;
}
