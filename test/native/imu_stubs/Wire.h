#pragma once
#include "Arduino.h"
#include <array>

// Mô phỏng thanh ghi ICM và data-ready read-to-clear; chạy chính driver production.
class TwoWire {
public:
    std::array<uint8_t, 128> thanhGhi{};
    std::function<std::array<int16_t, 7>(uint32_t)> taoMau;
    int thanhGhiNack = -1, thanhGhiDocThieu = -1;
    bool coDataReady = true, choPhepBus = true;
    uint32_t chuKyMau_ms = 5, lanDaXoaReady = 0;
    unsigned soLanReset = 0, soLanDocMau = 0;
    unsigned soLanBurstStatusMau = 0;
    uint16_t timeoutDaDat_ms = 0;
    uint32_t treDoc_ms = 0;
    bool traRong = false;
    int hutKhiDocByte = -1;
    int chanSda = -1, chanScl = -1;
    TwoWire() { thanhGhi[0x75] = 0x12; }
    bool begin(int sda, int scl) { chanSda = sda; chanScl = scl; return choPhepBus; }
    void setClock(uint32_t) {}
    void setTimeOut(uint16_t timeout_ms) { timeoutDaDat_ms = timeout_ms; }
    bool intDangGiuMuc() const { return coDataReady && millis()/chuKyMau_ms != lanDaXoaReady; }
    void beginTransmission(uint8_t) { _gui.clear(); }
    void write(uint8_t byte) { _gui.push_back(byte); }
    uint8_t endTransmission(bool = true) {
        _diaChi = _gui.at(0);
        if (_diaChi == thanhGhiNack) return 2;
        if (_gui.size() == 2) {
            if (_diaChi == 0x6B && _gui[1] == 0x80) {
                const auto who = thanhGhi[0x75];
                thanhGhi.fill(0);
                thanhGhi[0x75] = who;
                ++soLanReset;
            } else thanhGhi[_diaChi] = _gui[1];
        }
        return 0;
    }
    size_t requestFrom(uint8_t, size_t soByte, bool) {
        _nhan.clear();
        _viTri = 0;
        // Chỉ mô phỏng thời gian API; không thay thế timing I2C/FreeRTOS trên board.
        if (timeoutDaDat_ms && treDoc_ms > timeoutDaDat_ms) {
            delay(timeoutDaDat_ms);
            return 0;
        }
        delay(treDoc_ms);
        if (traRong) return 0;
        if (_diaChi == 0x3A) {
            const uint32_t mocMau = millis() / chuKyMau_ms;
            _nhan.push_back(coDataReady && mocMau != lanDaXoaReady ? 1 : 0);
            lanDaXoaReady = mocMau;
            if (soByte == 15) { ++soLanBurstStatusMau; themMauVaoBoDem(); }
        } else if (_diaChi == 0x3B) {
            themMauVaoBoDem();
        } else {
            for (size_t i = 0; i < soByte; ++i) _nhan.push_back(thanhGhi.at(_diaChi + i));
        }
        if (_diaChi == thanhGhiDocThieu && !_nhan.empty()) _nhan.pop_back();
        return _nhan.size();
    }
    int read() {
        if (static_cast<int>(_viTri) == hutKhiDocByte) return -1;
        return _viTri < _nhan.size() ? _nhan[_viTri++] : -1;
    }
private:
    void themMauVaoBoDem() {
        ++soLanDocMau;
        const auto mau = taoMau ? taoMau(millis()) : std::array<int16_t,7>{0,0,8192,0,0,0,0};
        for (const int16_t v : mau) {
            _nhan.push_back(static_cast<uint16_t>(v) >> 8);
            _nhan.push_back(static_cast<uint16_t>(v) & 0xFF);
        }
    }
    std::vector<uint8_t> _gui, _nhan;
    uint8_t _diaChi = 0;
    size_t _viTri = 0;
};
