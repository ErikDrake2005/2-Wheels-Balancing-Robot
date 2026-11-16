#include "core/GiamSatLenhTuXa.h"
#include <cstdlib>
#include <iostream>
using namespace ChanDoanEspNow;
void check(bool ok, const char* msg) { if (!ok) { std::cerr << "FAIL: " << msg << '\n'; std::exit(1); } }
GoiHello hello(uint32_t session, uint32_t seq) { GoiHello p; dongGoi(p, session, seq); return p; }
GoiLenh cmd(uint32_t seq, MaLenh op, uint16_t lease = 0, float v = 0) {
    GoiLenh p; p.boot_id = 77; p.opcode = op; p.lease_ms = lease; p.v_mps = v; dongGoi(p, 4, seq); return p;
}
int main() {
    GiamSatLenhTuXa s(77); float v, yaw;
    check(s.hello(hello(4,1),true), "bind while motor off");
    check(s.receive(cmd(2,MaLenh::Arm),1000), "fresh explicit arm accepted");
    check(s.service(2000,false).arm, "arm emitted only once");
    s.service(3000,true);
    GoiTelemetry t; s.telemetry(t); check(t.ack_seq==2, "arm applied ACK only after EN true");
    check(s.receive(cmd(3,MaLenh::Move,100,.05f),4000), "bounded motion");
    s.service(5000,true); s.desired(6000,v,yaw); check(v==.05f,"fresh move active");
    check(!s.receive(cmd(3,MaLenh::Move,100,.05f),7000),"duplicate cannot extend lease");
    check(s.receive(cmd(4,MaLenh::Heartbeat,150),80000),"heartbeat keeps authority");
    s.service(105000,true); s.desired(105000,v,yaw); check(v==0,"heartbeat never extends old nonzero move");
    s.service(231000,true); check(!s.receive(cmd(5,MaLenh::Move,100,.05f),232000),"expired authority cannot revive by MOVE");
    check(s.receive(cmd(6,MaLenh::Arm),233000),"operator can explicitly rearm hold");
    check(!s.service(234000,true).arm,"rearm on enabled motor does not send another enable edge");
    check(s.receive(cmd(7,MaLenh::Stop),235000),"stop accepted");
    check(!s.receive(cmd(8,MaLenh::Arm),235100),"pending stop wins over arm");
    s.service(236000,true);s.desired(236000,v,yaw);check(v==0,"stop holds neutral");
    check(!s.hello(hello(9,1),false),"new session cannot bind while motor running");
    check(s.receive(cmd(9,MaLenh::Disable),237000),"explicit disarm accepted");
    check(s.service(238000,true).disable,"disable edge emitted");
    s.service(239000,false);s.telemetry(t);check(t.ack_seq==9,"disable ACK after EN false");
    check(s.hello(hello(9,1),true),"new session permitted after disarm");
    check(!s.receive(cmd(10,MaLenh::Arm),240000),"old session cannot replay");
    GiamSatLenhTuXa stale(77);stale.hello(hello(4,1),true);stale.receive(cmd(2,MaLenh::Arm),1000);
    check(!stale.service(151000,false).arm,"queue delay cannot renew stale ARM");
    GiamSatLenhTuXa pending(77);pending.hello(hello(4,1),true);pending.receive(cmd(2,MaLenh::Arm),1000);
    pending.receive(cmd(3,MaLenh::Disable),1100);
    auto d=pending.service(1200,false);check(d.disable&&!d.arm,"disable overrides queued arm");
    std::cout << "PASS: remote receiver lease, replay, stop priority, late ARM and confirmed motor ACK\n";
}
