#include "model/GoiLenhNhiPhan.h"
#include <cassert>
#include <cmath>
#include <iostream>

static GoiLenhWebKhaoSat makeWeb(int16_t forward, int16_t turn, uint8_t scale = 75) {
    GoiLenhWebKhaoSat p{};
    p.dauKhung = dauKhungNhiPhan;
    p.maLenh = maLenhWebKhaoSatNhiPhan;
    p.mucTienLui = 250;
    p.mucReNgang = -500;
    p.forward_pct = forward;
    p.turn_pct = turn;
    p.speed_pct = scale;
    p.maKiemTra = tinhMaKiemTraNhiPhan(reinterpret_cast<const uint8_t*>(&p), sizeof(p)-1);
    return p;
}

int main() {
    float v = 0, yaw = 0;
    char key = 0;
    bool motion = false;
    MucLenhWeb meta{};
    const auto decode = [&](const GoiLenhWebKhaoSat& p) {
        return giaiMaGoiLenhNhiPhan(reinterpret_cast<const uint8_t*>(&p), sizeof(p), .5f, .8f,
                                    v, yaw, key, motion, &meta);
    };
    for (int16_t forward : {-100, -37, 0, 42, 100}) {
        for (int16_t turn : {-100, 0, 100}) {
            const auto p = makeWeb(forward, turn);
            assert(decode(p) && motion && key == 0 && meta.valid);
            assert(meta.forward_pct == forward && meta.turn_pct == turn && meta.speed_pct == 75);
            assert(std::fabs(v - .125f) < 1e-6f && std::fabs(yaw + .4f) < 1e-6f);
        }
    }
    auto p = makeWeb(37, -40);
    for (size_t byte = 0; byte < sizeof(p); ++byte) {
        auto bad = p;
        reinterpret_cast<uint8_t*>(&bad)[byte] ^= 1;
        assert(!decode(bad) && !meta.valid);
    }
    for (unsigned invalid = 0; invalid < 8; ++invalid) {
        auto bad = p;
        switch (invalid) {
            case 0: bad.forward_pct = 101; break;
            case 1: bad.turn_pct = -101; break;
            case 2: bad.speed_pct = 101; break;
            case 3: bad.reserved = 1; break;
            case 4: bad.phimHanhDong = 'a'; break;
            case 5: bad.mucTienLui = 1001; break;
            case 6: bad.mucReNgang = -1001; break;
            case 7: bad.maLenh = maLenhDiChuyenNhiPhan; break;
        }
        bad.maKiemTra = tinhMaKiemTraNhiPhan(reinterpret_cast<const uint8_t*>(&bad), sizeof(bad)-1);
        assert(!decode(bad) && !meta.valid);
    }
    assert(!giaiMaGoiLenhNhiPhan(reinterpret_cast<const uint8_t*>(&p), sizeof(p)-1,
                                .5f, .8f, v, yaw, key, motion, &meta));
    assert(!giaiMaGoiLenhNhiPhan(reinterpret_cast<const uint8_t*>(&p), sizeof(p),
                                NAN, .8f, v, yaw, key, motion, &meta));
    for (uint8_t type : {maLenhDiChuyenNhiPhan, maLenhPhimNhiPhan}) {
        const auto legacy = dongGoiLenhNhiPhan(.125f, -.4f, .5f, .8f, 'p', type);
        meta.valid = true;
        assert(giaiMaGoiLenhNhiPhan(reinterpret_cast<const uint8_t*>(&legacy), sizeof(legacy),
                                    .5f, .8f, v, yaw, key, motion, &meta));
        assert(!meta.valid && key == 'p' && motion == (type == maLenhDiChuyenNhiPhan));
        assert(std::fabs(v - .125f) < 1e-6f && std::fabs(yaw + .4f) < 1e-6f);
    }
    std::cout << "PASS: Web 14-byte raw joystick levels, validation and legacy 8-byte commands\n";
}
