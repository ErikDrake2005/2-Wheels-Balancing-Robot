#pragma once
#include "model/SensorFrame.h"
// Shared bus diagnostics. Legacy names retained for CSV/protocol compatibility.
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
