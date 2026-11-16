#include "config/MotorConfig.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
void check(bool ok, const char* label) {
    if (!ok) { std::cerr << "FAIL: " << label << '\n'; std::exit(1); }
}
uint64_t ticks(uint32_t us, uint32_t hz) {
    return (static_cast<uint64_t>(us) * hz + 999999u) / 1000000u;
}
}

int main() {
    const auto clock = doPhanGiaiBoDemMcpwm_hz;
    check(80000000u % clock == 0, "requested clock divides the SDK's MCPWM group clock exactly");
    check(ticks(chuKyKhoiTaoMcpwm_us, clock) < 65536,
          "boot period fits the 16-bit timer");
    const auto high = ticks(thoiGianMucCaoStep_us, clock) + 1u;
    const auto low = ticks(thoiGianMucThapToiThieuStep_us, clock);
    for (float requested : {tanSoChayLienTucToiThieu_hz, 1000.0f, 5000.0f, tanSoStepToiDa_hz}) {
        const double period = std::ceil(static_cast<double>(clock) / requested);
        check(period < 65536 && period >= high + low, "operating range fits period and STEP pulse widths");
        check(clock / period <= requested + 1e-6, "rounding period up cannot over-speed the requested pulse rate");
    }
    if (!dungPhanCungTmcMt) {
        const double topPeriod = static_cast<double>(clock) / tanSoStepToiDa_hz;
        check(clock == 5000000u && topPeriod == 125,
              "MKS 40 kHz uses exactly 125 ticks at 5 MHz");
        const double neighboringRate = clock / (topPeriod + 1);
        check((tanSoStepToiDa_hz - neighboringRate) / tanSoStepToiDa_hz < 0.008,
              "adjacent timer rate below 40 kHz is within 0.8 percent");
        check(ticks(chuKyKhoiTaoMcpwm_us, 10000000u) >= 65536,
              "regression: 10 MHz is incompatible with the current 10 ms boot period");
    }
    std::cout << "PASS: motor timer exact divider, 16-bit range, pulse widths and frequency granularity\n";
}
