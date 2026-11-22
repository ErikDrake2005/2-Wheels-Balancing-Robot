#pragma once
#include "Arduino.h"
#include <array>
#include <cassert>

constexpr int MSBFIRST = 1, SPI_MODE3 = 3;
constexpr int OUTPUT = 1, INPUT = 0;
inline void digitalWrite(int, int) {}
inline void delayMicroseconds(unsigned us) {
    dongHoGia_subUs += us;
    dongHoGia_ms += dongHoGia_subUs / 1000u;
    dongHoGia_subUs %= 1000u;
}
struct SPISettings {
    uint32_t hz;
    int order, mode;
    SPISettings(uint32_t h, int o, int m) : hz(h), order(o), mode(m) {}
};

// Register-level model. DRDY stays high until output registers are consumed.
// A new frame arrives every periodUs (or periodMs*1000) in simulated time, independently of reads.
struct SPIClass {
    std::array<uint8_t, 128> reg{};
    bool busOk = true, connected = true, sampling = true, corruptConfig = false;
    uint32_t periodMs = 5;
    uint32_t periodUs = 0; // Nếu >0 sẽ ưu tiên chu kỳ theo micro-giây (vd: 4167 us cho 240 Hz).
    uint8_t ready = 3, forceAfterBurst = 0;
    unsigned readsUntilAccel = 0;
    unsigned burstCount = 0;
    uint32_t payloadDelayUs = 0;
    uint64_t lastFrameUs = 0;
    bool firstByte = true, reading = false;
    uint8_t addr = 0;
    SPIClass() { reg[0x0F] = 0x70; setWord(0x2C, 8197); }
    void setWord(uint8_t r, int16_t x) {
        reg[r] = static_cast<uint8_t>(x);
        reg[r+1] = static_cast<uint16_t>(x) >> 8;
    }
    bool begin(int sck, int miso, int mosi, int cs) {
        assert(sck == 11 && miso == 9 && mosi == 10 && cs == 8);
        return busOk;
    }
    void beginTransaction(SPISettings s) {
        assert(s.mode == SPI_MODE3 && s.order == MSBFIRST && s.hz <= 10000000);
        firstByte = true;
    }
    uint8_t transfer(uint8_t b) {
        if (firstByte) {
            firstByte = false;
            reading = (b & 0x80) != 0;
            addr = b & 0x7f;
            if (reading && addr == 0x20) {
                ++burstCount;
                delayMicroseconds(payloadDelayUs);
            }
            return 0;
        }
        const uint8_t pos = addr++;
        if (!connected) return 0xFF;
        if (!reading) {
            if (pos == 0x12 && (b & 1)) reg[pos] = 0x04;
            else reg[pos] = corruptConfig && pos == 0x62 ? 0 : b;
            return 0;
        }
        if (pos == 0x1E) {
            if (readsUntilAccel && --readsUntilAccel == 0) ready |= 1u;
            const uint64_t stepUs = periodUs ? periodUs : static_cast<uint64_t>(periodMs) * 1000ULL;
            const uint64_t nowUs = microsExact64();
            if (nowUs < lastFrameUs) lastFrameUs = nowUs;
            if (sampling && reg[0x10] && reg[0x11] && nowUs - lastFrameUs >= stepUs) {
                lastFrameUs = nowUs - (nowUs - lastFrameUs) % stepUs;
                ready = 3;
            }
            return ready;
        }
        if (pos == 0x27) ready &= ~2u;
        if (pos == 0x2D) ready = (ready & ~1u) | forceAfterBurst;
        return reg[pos];
    }
    void endTransaction() {}
    void transferBytes(const uint8_t* src, uint8_t* dest, uint32_t n) {
        for (uint32_t i = 0; i < n; ++i) dest[i] = transfer(src ? src[i] : 0);
    }
};
inline SPIClass SPI;
