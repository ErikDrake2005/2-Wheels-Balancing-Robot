#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

// Chỉ Arduino loop ghi. TinyUSB có thể chỉ nhận 64 byte/lần.
// Nhận trọn dòng vào RAM rồi chuyển từng phần, giữ thứ tự cả khi CDC ghi thiếu.
class NhatKyNghienCuu {
public:
    bool conDuLieu() const { return dau < cuoi; }

    bool them(const char* noiDung, size_t soByte, bool themXuongDong = false) {
        const size_t can = soByte + (themXuongDong ? 1u : 0u);
        if (can > sizeof(duLieu) - (cuoi - dau)) return false;
        if (can > sizeof(duLieu) - cuoi) {
            std::memmove(duLieu, duLieu + dau, cuoi - dau);
            cuoi -= dau;
            dau = 0;
        }
        std::memcpy(duLieu + cuoi, noiDung, soByte);
        cuoi += soByte;
        if (themXuongDong) duLieu[cuoi++] = '\n';
        return true;
    }

    template<class CongSerial>
    void phat(CongSerial& cong) {
        if (!conDuLieu()) return;
        const int trong = cong.availableForWrite();
        if (trong <= 0) return;
        const size_t gui = std::min(cuoi - dau, static_cast<size_t>(trong));
        dau += cong.write(reinterpret_cast<const uint8_t*>(duLieu + dau), gui);
        if (dau == cuoi) dau = cuoi = 0;
    }

private:
    char duLieu[2048]{};
    size_t dau = 0, cuoi = 0;
};
