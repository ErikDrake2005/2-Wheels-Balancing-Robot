#pragma once
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

inline uint64_t dongHoGia_ms = 0;
inline uint32_t millis() { return static_cast<uint32_t>(dongHoGia_ms); }
inline uint32_t micros() { return static_cast<uint32_t>(dongHoGia_ms * 1000); }
inline void delay(uint32_t ms) { dongHoGia_ms += ms; }
constexpr int LOW = 0, HIGH = 1, INPUT_PULLUP = 2, INPUT_PULLDOWN = 3;
inline std::function<int(int)> docNutGia;
inline int digitalRead(int chan) { return docNutGia ? docNutGia(chan) : HIGH; }
inline void pinMode(int, int) {}
struct SerialGia {
    std::string noiDung;
    template<class... T> void printf(const char* mau, T... giaTri) {
        char dong[512];
        std::snprintf(dong, sizeof(dong), mau, giaTri...);
        noiDung += dong;
    }
    void println(const char* dong) { noiDung += std::string(dong) + "\n"; }
    int available() const { return 0; }
    int read() const { return -1; }
};
inline SerialGia Serial;
