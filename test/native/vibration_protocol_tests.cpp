#include "model/GiaoThucKhaoSatRung.h"
#include "core/GiamSatLenhTuXa.h"
#include <cassert>
#include <iostream>

int main() {
    KhaoSatRung::Subscribe subscription{}, decoded{};
    KhaoSatRung::pack(subscription, 123, 1);
    assert(KhaoSatRung::decode(reinterpret_cast<const uint8_t*>(&subscription), sizeof(subscription), decoded));
    ChanDoanEspNow::GoiLenh command{};
    ChanDoanEspNow::GoiHello hello{};
    assert(!ChanDoanEspNow::giaiMa(reinterpret_cast<const uint8_t*>(&subscription), sizeof(subscription), command));
    assert(!ChanDoanEspNow::giaiMa(reinterpret_cast<const uint8_t*>(&subscription), sizeof(subscription), hello));
    GiamSatLenhTuXa control(42);
    ChanDoanEspNow::GoiTelemetry status{}; control.telemetry(status);
    assert(status.remote_state == 0); // Passive frame carries no ARM/control authority.

    KhaoSatRung::Samples data{}, parsed{};
    data.boot_id = 42; data.log_id = 1; data.count = 2; data.sample_hz = 200;
    data.sample[0].t_us = 1000; data.sample[1].t_us = 6000;
    data.sample[0].forward_pct = -100; data.sample[1].turn_pct = 100;
    data.sample[1].flags = KhaoSatRung::MotorOn | KhaoSatRung::UiValid;
    KhaoSatRung::pack(data, 123, 2);
    assert(sizeof(data) == 248 && KhaoSatRung::valid(data));
    assert(KhaoSatRung::decode(reinterpret_cast<const uint8_t*>(&data), sizeof(data), parsed));
    assert(parsed.sample[1].t_us-parsed.sample[0].t_us == 5000);
    data.sample[0].t_us = 999;
    assert(!KhaoSatRung::decode(reinterpret_cast<const uint8_t*>(&data), sizeof(data), parsed));
    data.sample[0].forward_pct = 101;
    assert(!KhaoSatRung::valid(data));
    data.sample[0].forward_pct = -100; data.sample[1].t_us = 998;
    assert(!KhaoSatRung::valid(data));
    data.sample[1].t_us = 6000; data.sample[0].gx = INFINITY;
    assert(!KhaoSatRung::valid(data));
    data.sample[0].gx = 0; data.count = 3;
    assert(!KhaoSatRung::valid(data));
    KhaoSatRung::Info info{}, decodedInfo{};
    info.boot_id=42; info.t_us=9000; info.sample_hz=200;
    info.firmware[0]='x';
    KhaoSatRung::pack(info, 123, 3);
    assert(sizeof(info)<=250 && KhaoSatRung::valid(info));
    assert(KhaoSatRung::decode(reinterpret_cast<const uint8_t*>(&info),sizeof(info),decodedInfo));
    info.h.version++;
    assert(!KhaoSatRung::decode(reinterpret_cast<const uint8_t*>(&info),sizeof(info),decodedInfo));
    std::cout << "PASS: passive vibration protocol, timestamps, bounds, CRC and control namespace isolation\n";
}
