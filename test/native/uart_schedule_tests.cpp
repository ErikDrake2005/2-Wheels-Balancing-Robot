#include "LichHoiUartMks.h"
#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>

using Luot = LichHoiUartMks::Luot;

namespace {
void check(bool ok, const char* message) {
    if (!ok) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

unsigned nominalCadence(uint32_t pollPeriod_us) {
    LichHoiUartMks lich;
    unsigned trai = 0, phai = 0;
    for (uint64_t t = 0; t < 1'000'000; t += pollPeriod_us) {
        const Luot luot = lich.chonLuot(t);
        if (luot == Luot::KhongHoi) continue;
        if (luot == Luot::Trai) ++trai; else ++phai;
        const uint64_t writeDuration_us = 5 + (trai + phai) % 16;
        lich.ghiNhanKetQua(t + writeDuration_us, true);
    }
    check(trai == phai, "uniform polling is fair");
    return trai;
}

void delayedCompletion() {
    for (bool success : {true, false}) {
        LichHoiUartMks lich;
        check(lich.chonLuot(0) == Luot::Trai, "cold start at t=0");
        check(lich.chonLuot(1500) == Luot::KhongHoi, "cannot reserve again before TX completion");
        lich.ghiNhanKetQua(1800, success);
        check(lich.chonLuot(2500) == Luot::KhongHoi && lich.chonLuot(3799) == Luot::KhongHoi,
              "guard uses write completion, even after a short/failed TX");
        check(lich.chonLuot(3800) == (success ? Luot::Phai : Luot::Trai),
              "first safe slot is 2 ms after completion; only complete requests alternate");
    }
}

void jitterAndTransmissionFailures() {
    LichHoiUartMks lich;
    const std::array<uint32_t, 5> intervals = {925, 1150, 970, 1050, 880};
    uint64_t now = 80'000, last = 0;
    unsigned acceptedL = 0, acceptedR = 0, attempts = 0;
    for (unsigned i = 0; i < 6000; ++i) {
        const auto turn = lich.chonLuot(now);
        if (turn != Luot::KhongHoi) {
            check(!attempts || now-last >= 2000, "jitter or failed write must not shorten spacing");
            check(turn == (acceptedL == acceptedR ? Luot::Trai : Luot::Phai),
                  "skip and TX error do not advance the wheel");
            ++attempts;
            const bool success = attempts % 7 != 0;
            if (success) {
                if (turn == Luot::Trai) ++acceptedL; else ++acceptedR;
            }
            last = now + 5 + attempts % 16;
            lich.ghiNhanKetQua(last, success);
            check(lich.chonLuot(now) == Luot::KhongHoi, "no immediate retry, even after failed TX");
        }
        now += intervals[i % intervals.size()];
    }
    check(acceptedL > 900 && acceptedR > 900 && acceptedL-acceptedR <= 1, "fair polling under jitter");
}

void stallAndReset() {
    LichHoiUartMks lich;
    constexpr uint64_t t0 = 8'000'000'000ULL;
    check(lich.chonLuot(t0) == Luot::Trai, "first slot is immediate, left first");
    lich.ghiNhanKetQua(t0+10, true);
    check(lich.chonLuot(t0-1) == Luot::KhongHoi, "clock moved back cannot trigger a burst");
    constexpr uint64_t stall = 1'000'000ULL * 2500 + 2490;
    check(lich.chonLuot(t0+stall) == Luot::Phai, "only next wheel after a long stall");
    lich.ghiNhanKetQua(t0+stall+200, true);
    check(lich.soKheBoLo() == 999'999, "obsolete slots counted without replay");
    check(lich.chonLuot(t0+stall+10) == Luot::KhongHoi, "following grid edge too close to the last TX");
    check(lich.chonLuot(t0+stall+2199) == Luot::KhongHoi, "strict 2 ms guard after a long stall and preemption");
    check(lich.chonLuot(t0+stall+2200) == Luot::Trai, "resume latest due slot at first safe call");
    lich.ghiNhanKetQua(t0+stall+2250, false);
    check(lich.chonLuot(t0+stall+4249) == Luot::KhongHoi, "partial TX also reserves full guard");
    check(lich.chonLuot(t0+stall+4250) == Luot::Trai, "failed request stays on this wheel");
    lich.ghiNhanKetQua(t0+stall+4260, true);
    lich.reset();
    check(lich.soKheBoLo() == 0 && lich.chonLuot(t0+stall+5000) == Luot::KhongHoi,
          "odometry reset clears grid/counters but preserves the preceding TX guard");
    check(lich.chonLuot(t0+stall+6260) == Luot::Trai, "reset establishes a new grid with the left wheel");
    lich.ghiNhanKetQua(t0+stall+6270, true);
    check(lich.chonLuot(t0+stall+8760) == Luot::Phai, "first complete new slot is right");
}
}

int main() {
    const auto fast = nominalCadence(500);
    const auto slow = nominalCadence(1000);
    check(fast == 200 && slow == 167, "2 ms post-write guard and 5-20 us call duration distinguish poll rates");
    delayedCompletion();
    jitterAndTransmissionFailures();
    stallAndReset();
    std::cout << "PASS: UART MKS post-write guard; simulated queries/wheel in 1 s: 500 us poll="
              << fast << ", 1000 us poll=" << slow << "; jitter, partial write, preemption, stall and reset\n";
}
