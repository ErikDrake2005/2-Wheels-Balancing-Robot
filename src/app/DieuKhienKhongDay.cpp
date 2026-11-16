#include "DieuKhienKhongDay.h"
#include "config/ControlConfig.h"
#include "config/WifiConfig.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <mbedtls/base64.h>
#include <mbedtls/sha1.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace {
WebServer mayChuWeb(congMayChuWeb);
WiFiServer mayChuWsNhiPhan(congWebSocketNhiPhan);
WiFiClient khachWsNhiPhan;
bool daBatTayWsNhiPhan = false;
char boDemBatTayWs[640]{};
size_t doDaiBatTayWs = 0u;
uint8_t boDemKhungWs[160]{};
size_t doDaiKhungWs = 0u;
WiFiUDP congUdpLenh;

void noiChuoiJsonAnToan(String& dich, const char* nguon) {
    if (!nguon) return;
    for (const char* conTro = nguon; *conTro != '\0'; ++conTro) {
        const unsigned char kyTu = static_cast<unsigned char>(*conTro);
        if (kyTu == '"' || kyTu == '\\') {
            dich += '\\';
            dich += static_cast<char>(kyTu);
        } else if (kyTu == '\n') {
            dich += "\\n";
        } else if (kyTu == '\r') {
            dich += "\\r";
        } else if (kyTu == '\t') {
            dich += "\\t";
        } else if (kyTu >= 0x20u) {
            dich += static_cast<char>(kyTu);
        }
    }
}

int chuyenKyTuHex(char kyTu) {
    if (kyTu >= '0' && kyTu <= '9') return kyTu - '0';
    if (kyTu >= 'a' && kyTu <= 'f') return 10 + (kyTu - 'a');
    if (kyTu >= 'A' && kyTu <= 'F') return 10 + (kyTu - 'A');
    return -1;
}

bool giaiMaChuoiHex8Byte(const String& chuoiHex, uint8_t duLieuRa[sizeof(GoiLenhNhiPhan)]) {
    if (chuoiHex.length() != static_cast<unsigned int>(sizeof(GoiLenhNhiPhan) * 2u)) return false;
    for (size_t chiSo = 0u; chiSo < sizeof(GoiLenhNhiPhan); ++chiSo) {
        const int cao = chuyenKyTuHex(chuoiHex.charAt(chiSo * 2u));
        const int thap = chuyenKyTuHex(chuoiHex.charAt(chiSo * 2u + 1u));
        if (cao < 0 || thap < 0) return false;
        duLieuRa[chiSo] = static_cast<uint8_t>((cao << 4) | thap);
    }
    return true;
}

bool timKhoaWebSocket(const char* yeuCauHttp, char* khoaRa, size_t kichThuocRa) {
    if (!yeuCauHttp || !khoaRa || kichThuocRa == 0u) return false;
    static constexpr char tenHeader[] = "sec-websocket-key:";
    const size_t doDaiTen = sizeof(tenHeader) - 1u;
    for (const char* conTro = yeuCauHttp; *conTro != '\0'; ++conTro) {
        bool khop = true;
        for (size_t i = 0u; i < doDaiTen; ++i) {
            if (conTro[i] == '\0' ||
                std::tolower(static_cast<unsigned char>(conTro[i])) != tenHeader[i]) {
                khop = false;
                break;
            }
        }
        if (!khop) continue;
        const char* batDau = conTro + doDaiTen;
        while (*batDau == ' ' || *batDau == '\t') ++batDau;
        size_t doDai = 0u;
        while (batDau[doDai] != '\0' && batDau[doDai] != '\r' &&
               batDau[doDai] != '\n' && doDai + 1u < kichThuocRa) {
            khoaRa[doDai] = batDau[doDai];
            ++doDai;
        }
        while (doDai > 0u && (khoaRa[doDai - 1u] == ' ' || khoaRa[doDai - 1u] == '\t')) {
            --doDai;
        }
        khoaRa[doDai] = '\0';
        return doDai > 0u;
    }
    return false;
}

const char trangDieuKhienHtml[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Xe Can Bang &amp; Terminal Serial</title>
<style>
:root{--nen:#0d1117;--the:#161b22;--vien:#2d3643;--chu:#e6edf3;--mo:#8b949e;--xanh:#2ea043;--xanhDuong:#2f81f7;--do:#f85149;--vang:#d29922}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;padding:10px;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:var(--nen);color:var(--chu);overscroll-behavior:contain}
.khungChinh{max-width:980px;margin:0 auto;display:flex;flex-direction:column;gap:10px}
.the{background:var(--the);border:1px solid var(--vien);border-radius:14px;padding:11px;box-shadow:0 4px 16px rgba(0,0,0,.32)}
.tieuDe{display:flex;justify-content:space-between;align-items:center;gap:8px;margin-bottom:8px;font-weight:700;font-size:14px}
.thanhTrangThaiDinh{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin-bottom:8px}
.mucTrangThai{display:flex;align-items:center;gap:5px;padding:3px 8px;border-radius:8px;background:#0d1117;border:1px solid var(--vien);font-size:11px;font-weight:700;white-space:nowrap}
.chamKetNoi{width:8px;height:8px;border-radius:50%;display:inline-block;background:var(--mo)}
.ketNoiTot{background:#00ff88;box-shadow:0 0 8px #00ff88}
.ketNoiCham{background:#ffaa00;box-shadow:0 0 8px #ffaa00}
.matKetNoi{background:#ff4444;box-shadow:0 0 8px #ff4444}
.nhan{display:inline-block;padding:3px 9px;border-radius:999px;font-size:11px;font-weight:700;background:#21262d;color:var(--chu)}
.nhanBat{background:rgba(46,160,67,.22);color:#56d364;border:1px solid rgba(46,160,67,.5)}
.nhanTat{background:rgba(139,148,158,.15);color:var(--mo);border:1px solid var(--vien)}
.nhanLoi{background:rgba(248,81,73,.2);color:#ff7b72;border:1px solid rgba(248,81,73,.5)}
.luoiChiSo{display:grid;grid-template-columns:repeat(3,1fr);gap:7px}
.oSo{background:#0d1117;border:1px solid var(--vien);border-radius:10px;padding:7px;text-align:center}
.tenSo{font-size:11px;color:var(--mo);margin-bottom:2px}
.giaTriSo{font-size:14px;font-weight:700;font-variant-numeric:tabular-nums}
.noiDungDieuKhien{display:flex;flex-direction:column;gap:10px}
.hangNut{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}
.hangBaNut{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.hangBonNut{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}
.hangBonNut .nut{padding:9px 4px;font-size:11.5px;line-height:1.25}
.nut{border:1px solid var(--vien);background:#21262d;color:var(--chu);border-radius:11px;padding:10px 7px;font-size:13px;font-weight:700;cursor:pointer;touch-action:manipulation;user-select:none;-webkit-user-select:none}
.nut:active{transform:scale(.96);filter:brightness(1.15)}
.nutXanh{background:var(--xanh);border-color:#3fb950;color:#fff}
.nutDo{background:#b62324;border-color:var(--do);color:#fff}
.nutVang{background:#9e6a03;border-color:var(--vang);color:#fff}
.nutXanhDuong{background:#1f6feb;border-color:var(--xanhDuong);color:#fff}
.nutNho{border:1px solid var(--vien);background:#21262d;color:var(--chu);border-radius:8px;padding:5px 9px;font-size:11px;font-weight:700;cursor:pointer;touch-action:manipulation}
.nutNho:active{transform:scale(.96)}
.nutNho.nutXanhDuong{background:#1f6feb;border-color:var(--xanhDuong);color:#fff}
.khuHaiCanGat{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;align-items:stretch}
.hopCanGat{background:#0d1117;border:1px solid var(--vien);border-radius:12px;padding:10px 8px;display:flex;flex-direction:column;align-items:center;justify-content:space-between;gap:8px}
.tieuDeCanGat{width:100%;display:flex;justify-content:space-between;align-items:center;font-size:12px;font-weight:700;color:var(--chu)}
.giaTriTruc{font-size:12px;font-weight:700;color:#58a6ff;font-variant-numeric:tabular-nums}
.vungCanGatDoc{position:relative;width:110px;height:200px;border-radius:26px;background:linear-gradient(180deg,#182434 0%,#0d1117 50%,#182434 100%);border:2px solid #384454;touch-action:none;user-select:none;-webkit-user-select:none}
.vachTamNgang{position:absolute;left:10px;right:10px;top:50%;height:1px;background:#304054;pointer-events:none}
.vachTamDoc{position:absolute;top:10px;bottom:10px;left:50%;width:1px;background:#304054;pointer-events:none}
.khungRanhNgang{display:flex;align-items:center;justify-content:center;width:100%;height:200px}
.vungCanGatNgang{position:relative;width:100%;max-width:200px;height:110px;border-radius:26px;background:linear-gradient(90deg,#182434 0%,#0d1117 50%,#182434 100%);border:2px solid #384454;touch-action:none;user-select:none;-webkit-user-select:none}
.nhanTrucDinh{position:absolute;top:8px;left:0;right:0;text-align:center;font-size:10px;font-weight:700;color:var(--mo);pointer-events:none}
.nhanTrucDay{position:absolute;bottom:8px;left:0;right:0;text-align:center;font-size:10px;font-weight:700;color:var(--mo);pointer-events:none}
.nhanTrucTrai{position:absolute;left:8px;top:50%;transform:translateY(-50%);font-size:10px;font-weight:700;color:var(--mo);pointer-events:none}
.nhanTrucPhai{position:absolute;right:8px;top:50%;transform:translateY(-50%);font-size:10px;font-weight:700;color:var(--mo);pointer-events:none}
.numCanGatTienLui{position:absolute;left:50%;top:50%;width:62px;height:62px;margin:-31px 0 0 -31px;border-radius:50%;background:radial-gradient(circle at 32% 32%,#58a6ff,#1f6feb);border:2px solid #9ecbff;box-shadow:0 6px 18px rgba(0,0,0,.55);pointer-events:none}
.numCanGatReNgang{position:absolute;left:50%;top:50%;width:62px;height:62px;margin:-31px 0 0 -31px;border-radius:50%;background:radial-gradient(circle at 32% 32%,#56d364,#238636);border:2px solid #9be9a8;box-shadow:0 6px 18px rgba(0,0,0,.55);pointer-events:none}
.thanhTruot{width:100%;accent-color:#2f81f7;height:26px}
.khungTerminalSerial{background:#080c10;border:1px solid #303a48;border-radius:12px;padding:10px;display:flex;flex-direction:column;gap:8px;touch-action:auto}
.hangNutTerminal{display:flex;flex-wrap:wrap;gap:5px;touch-action:auto}
.khungXuatSerial{height:210px;overflow-y:auto;background:#04070a;border:1px solid #222c38;border-radius:8px;padding:8px;font-family:'Courier New',monospace;font-size:11.5px;line-height:1.4;color:#00ff88;word-break:break-word;touch-action:pan-y;user-select:text;-webkit-user-select:text;-webkit-overflow-scrolling:touch}
.khungXuatSerial div{padding:2px 0;border-bottom:1px solid rgba(255,255,255,.04)}
.hangNhapSerial{display:flex;gap:6px;touch-action:auto}
.oNhapLenhSerial{flex:1;min-width:0;background:#0d1117;border:1px solid #384454;border-radius:8px;color:#00ff88;padding:8px 10px;font-size:13px;font-family:'Courier New',monospace;outline:none;touch-action:auto;user-select:text;-webkit-user-select:text}
.oNhapLenhSerial:focus{border-color:#2f81f7}
.luoiPid{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}
.nhomNhap{display:flex;gap:4px;align-items:center;background:#0d1117;border:1px solid var(--vien);border-radius:10px;padding:6px 8px}
.nhomNhap label{font-size:12px;color:var(--mo);min-width:34px}
.nhomNhap input{width:100%;min-width:0;background:transparent;border:none;color:var(--chu);font-size:14px;font-weight:700;outline:none}
.nhomNhap button{background:#1f6feb;border:none;color:#fff;border-radius:7px;padding:6px 10px;font-weight:700;font-size:12px}
.dongPhu{font-size:12px;color:var(--mo);line-height:1.45;word-break:break-word}
.thongBaoNoi{position:fixed;top:16px;left:50%;transform:translateX(-50%);background:rgba(13,17,23,.94);color:#fff;padding:10px 16px;border-radius:10px;font-size:13px;font-weight:700;z-index:9999;pointer-events:none;border:1px solid #384454;box-shadow:0 8px 24px rgba(0,0,0,.6);opacity:0;transition:opacity .2s;max-width:88vw;text-align:center}
.lopPhuCuaSoCon{display:none;position:fixed;inset:0;background:rgba(0,0,0,.78);z-index:9000;align-items:center;justify-content:center;padding:10px}
.lopPhuCuaSoCon.dangHienCuaSo{display:flex}
.hopCuaSoCon{width:100%;max-width:540px;max-height:90vh;overflow-y:auto;background:var(--the);border:1px solid #384454;border-radius:16px;padding:12px;display:flex;flex-direction:column;gap:10px;box-shadow:0 16px 40px rgba(0,0,0,.75);touch-action:pan-y}
.dauCuaSoCon{display:flex;justify-content:space-between;align-items:center;gap:8px}
.nhomMucPhu{background:#0d1117;border:1px solid var(--vien);border-radius:12px;padding:9px;display:flex;flex-direction:column;gap:8px}
.nhanNhomPhu{font-size:11px;font-weight:700;color:var(--mo);letter-spacing:.3px}
@media (orientation:landscape) and (min-width:680px){
  .luoiChiSo{grid-template-columns:repeat(6,1fr)}
}
</style>
</head>
<body>
<div id="thongBaoNoi" class="thongBaoNoi"></div>
<div class="khungChinh">
  <div class="the">
    <div class="tieuDe">
      <span>XE CAN BANG ESP32-S3</span>
      <span id="nhanTrangThai" class="nhan nhanTat">TAT MOTOR</span>
    </div>
    <div class="thanhTrangThaiDinh">
      <div class="mucTrangThai"><span id="chamKetNoi" class="chamKetNoi"></span><span id="nhanKetNoiMang">Dang ket noi...</span></div>
      <div class="mucTrangThai">Tre: <span id="oDoTreMang">-</span></div>
      <div class="mucTrangThai">Che do: <span id="oCheDo">PID_THUAN (Banh)</span></div>
      <span id="nhanKenhNhiPhan" class="nhan nhanTat">HTTP 8B</span>
    </div>
    <div class="luoiChiSo">
      <div class="oSo"><div class="tenSo">Goc nghieng</div><div id="oGocNghien" class="giaTriSo">0.00°</div></div>
      <div class="oSo"><div class="tenSo">Goc can bang</div><div id="oGocCanBang" class="giaTriSo">0.00°</div></div>
      <div class="oSo"><div class="tenSo">Vi tri giu</div><div id="oViTri" class="giaTriSo">0.000m</div></div>
      <div class="oSo"><div class="tenSo">Van toc</div><div id="oVanToc" class="giaTriSo">0.000</div></div>
      <div class="oSo"><div class="tenSo">Loi</div><div id="oLoi" class="giaTriSo">none</div></div>
      <div class="oSo"><div class="tenSo">Goi 8B</div><div id="oTongGoiNhiPhan" class="giaTriSo">0</div></div>
    </div>
    <div class="dongPhu" id="thongTinPhu" style="margin-top:8px">Dang ket noi toi xe...</div>
  </div>

  <div class="the">
    <div class="hangBonNut">
      <button class="nut nutXanh" onclick="batCanBangBanhXe()">BAT CAN BANG (a)</button>
      <button class="nut" onclick="guiPhim('d','Tat motor')">TAT MOTOR (d)</button>
      <button class="nut nutDo" onclick="guiPhim('x','Dung khan cap')">DUNG KHAN (x)</button>
      <button class="nut nutVang" onclick="guiPhim('k','Da gui xoa loi')">XOA LOI (k)</button>
    </div>
    <div class="tieuDe" style="margin-bottom:4px">
      <span>2 JOYSTICK &amp; LENH NHI PHAN 8B</span>
      <span id="oPhanTramTocDo" style="font-size:12px;color:#58a6ff">100%</span>
    </div>
    <input id="thanhTocDo" class="thanhTruot" type="range" min="25" max="100" step="5" value="100" style="margin-bottom:6px">
    <div class="khuHaiCanGat">
      <div class="hopCanGat">
        <div class="tieuDeCanGat">
          <span>1. TIEN / LUI</span>
          <span id="nhanTocDoTienLui" class="giaTriTruc">0.00 m/s</span>
        </div>
        <div id="vungCanGatTienLui" class="vungCanGatDoc">
          <div class="vachTamNgang"></div>
          <div class="nhanTrucDinh">TIEN +</div>
          <div class="nhanTrucDay">LUI -</div>
          <div id="numCanGatTienLui" class="numCanGatTienLui"></div>
        </div>
      </div>
      <div class="hopCanGat">
        <div class="tieuDeCanGat">
          <span>2. RE TRAI / PHAI</span>
          <span id="nhanTocDoReNgang" class="giaTriTruc" style="color:#56d364">0.00 rad/s</span>
        </div>
        <div class="khungRanhNgang">
          <div id="vungCanGatReNgang" class="vungCanGatNgang">
            <div class="vachTamDoc"></div>
            <div class="nhanTrucTrai">&lt; TRAI</div>
            <div class="nhanTrucPhai">PHAI &gt;</div>
            <div id="numCanGatReNgang" class="numCanGatReNgang"></div>
          </div>
        </div>
      </div>
    </div>
    <button class="nut nutVang" style="width:100%;margin-top:8px" onclick="phanhGiuViTri()">PHANH &amp; GIU VI TRI DUNG</button>
    <div class="dongPhu" id="khungHexNhiPhan" style="margin-top:6px;margin-bottom:8px">Khung 8B: A5 01 00 00 00 00 00 A4 | Da gui: 0</div>
    <div class="hangBaNut">
      <button class="nut nutXanhDuong" onclick="moCuaSoCon('cuaSoCalib')">CALIB &amp; CHE DO</button>
      <button class="nut nutXanhDuong" onclick="moCuaSoCon('cuaSoPid')">TINH CHINH PID</button>
      <button class="nut nutXanhDuong" onclick="moCuaSoCon('cuaSoTerminal')">TERMINAL SERIAL</button>
    </div>
  </div>
</div>

<div id="lopPhuCuaSoCon" class="lopPhuCuaSoCon" onclick="if(event.target===this)dongCuaSoCon()">
  <div id="hopCuaSoCon" class="hopCuaSoCon">
    <div class="dauCuaSoCon">
      <span id="tieuDeCuaSoCon" style="font-weight:700;font-size:14px">CALIB &amp; CHE DO</span>
      <button class="nutNho nutDo" onclick="dongCuaSoCon()">DONG (X)</button>
    </div>
    <div class="hangBaNut">
      <button id="nutTabCalib" class="nutNho nutXanhDuong" onclick="moCuaSoCon('cuaSoCalib')">CALIB &amp; CHE DO</button>
      <button id="nutTabPid" class="nutNho" onclick="moCuaSoCon('cuaSoPid')">TINH CHINH PID</button>
      <button id="nutTabTerminal" class="nutNho" onclick="moCuaSoCon('cuaSoTerminal')">TERMINAL SERIAL</button>
    </div>

    <div id="cuaSoCalib" style="display:none">
      <div class="nhomMucPhu" style="margin-bottom:8px">
        <div class="nhanNhomPhu">CHON CHE DO CAN BANG BANH XE (Bench=OFF, VongViTri=ON)</div>
        <div class="hangBaNut">
          <button id="nutChonPidThuan" class="nut nutXanhDuong" onclick="guiPhim('p','Da chon PID THUAN (Banh xe)')">PID THUAN (p)</button>
          <button id="nutChonPidMo" class="nut" onclick="guiPhim('m','Da chon PID MO (Banh xe)')">PID MO (m)</button>
          <button class="nut" onclick="guiPhim('f','Dao che do PID/MO')">DOI PID/MO (f)</button>
        </div>
      </div>
      <div class="nhomMucPhu" style="margin-bottom:8px">
        <div class="nhanNhomPhu">CALIB &amp; GIU VI TRI TREN BANH XE</div>
        <div class="hangBaNut">
          <button class="nut" onclick="guiPhim('c','Giu xe yen 2s de calib')">CALIB 2S (c)</button>
          <button class="nut" onclick="guiPhim('o','Zero nhanh pitch+odo')">ZERO NHANH (o)</button>
          <button class="nut nutXanhDuong" onclick="guiPhim('g','Da chot vi tri dung')">CHOT VI TRI (g)</button>
          <button class="nut" onclick="guiPhim('[','Trim -0.1 deg')">TRIM -0.1° ([)</button>
          <button class="nut" onclick="guiPhim(']','Trim +0.1 deg')">TRIM +0.1° (])</button>
          <button class="nut nutXanh" onclick="guiDong('save','Da luu Flash NVS')">LUU NVS (save)</button>
        </div>
      </div>
      <div class="nhomMucPhu">
        <div class="nhanNhomPhu">TUY CHON PHU (TEST TREN GIA DO / VONG NGOAI / CHIEU DONG CO)</div>
        <div class="hangBaNut">
          <button id="nutVongViTri" class="nut" onclick="guiPhim('v','Dao vong vi tri')">VONG VI TRI (v)</button>
          <button id="nutBenchMode" class="nut" onclick="guiPhim('b','Dao Bench Mode')">BENCH MODE (b)</button>
          <button id="nutDaoChieuDongCo" class="nut" onclick="guiDong('daochieu','Da dao chieu dong co')">DAO CHIEU DONG CO</button>
        </div>
      </div>
    </div>

    <div id="cuaSoPid" style="display:none">
      <div class="tieuDe" style="margin-bottom:6px">
        <span id="tieuDePid">TINH CHINH: PID_THUAN</span>
        <div style="display:flex;gap:6px">
          <button id="nutPidThuanNhanh" class="nutNho nutXanhDuong" onclick="guiPhim('p','Da chon PID THUAN (Banh xe)')">PID THUAN (p)</button>
          <button id="nutPidMoNhanh" class="nutNho" onclick="guiPhim('m','Da chon PID MO (Banh xe)')">PID MO (m)</button>
        </div>
      </div>
      <div class="luoiPid" style="margin-bottom:8px">
        <div class="nhomNhap"><label>Kp</label><input id="nhapKp" type="number" step="0.5"><button onclick="guiThongSo('kp','nhapKp')">Dat</button></div>
        <div class="nhomNhap"><label>Ki</label><input id="nhapKi" type="number" step="0.1"><button onclick="guiThongSo('ki','nhapKi')">Dat</button></div>
        <div class="nhomNhap"><label>Kd</label><input id="nhapKd" type="number" step="0.1"><button onclick="guiThongSo('kd','nhapKd')">Dat</button></div>
        <div class="nhomNhap"><label>Kpv</label><input id="nhapKpv" type="number" step="0.01"><button onclick="guiThongSo('kpv','nhapKpv')">Dat</button></div>
        <div class="nhomNhap"><label>Kiv</label><input id="nhapKiv" type="number" step="0.01"><button onclick="guiThongSo('kiv','nhapKiv')">Dat</button></div>
        <div class="nhomNhap"><label>Ref°</label><input id="nhapRefDo" type="number" step="0.05"><button onclick="guiThongSo('refdeg','nhapRefDo')">Dat</button></div>
      </div>
      <div class="hangNut" style="margin-bottom:8px">
        <button class="nut nutXanh" onclick="guiDong('save','Da luu Flash NVS')">LUU NVS (save)</button>
        <button class="nut nutDo" onclick="guiDong('resetcfg','Da khoi phuc thong so goc')">KHOI PHUC GOC (resetcfg)</button>
      </div>
      <div class="dongPhu" id="thongTinMang">ESP-NOW song song &amp; WiFi AP</div>
    </div>

    <div id="cuaSoTerminal" style="display:none">
      <div class="khungTerminalSerial">
        <div class="tieuDe" style="margin-bottom:2px">
          <span>TERMINAL SERIAL TRUC TIEP</span>
          <div style="display:flex;gap:6px">
            <button id="nutLuongTelemetryWeb" class="nutNho" onclick="guiLenhChuoiTerminal('tw')">LUONG TELEMETRY (tw)</button>
            <button class="nutNho" onclick="xoaManHinhTerminal()">XOA MAN HINH</button>
          </div>
        </div>
        <div class="hangNutTerminal">
          <button class="nutNho" onclick="guiLenhChuoiTerminal('status')">status</button>
          <button class="nutNho" onclick="guiLenhChuoiTerminal('h')">h</button>
          <button class="nutNho" onclick="guiLenhChuoiTerminal('c')">calib (c)</button>
          <button class="nutNho" onclick="guiLenhChuoiTerminal('o')">zero (o)</button>
          <button class="nutNho" onclick="guiLenhChuoiTerminal('p')">pid (p)</button>
          <button class="nutNho" onclick="guiLenhChuoiTerminal('m')">fuzzy (m)</button>
          <button class="nutNho" onclick="guiLenhChuoiTerminal('save')">save</button>
        </div>
        <div id="khungXuatSerial" class="khungXuatSerial"></div>
        <div class="hangNhapSerial">
          <input id="oNhapLenhSerial" class="oNhapLenhSerial" type="text" autocomplete="off" autocapitalize="off" spellcheck="false"
            placeholder="Nhap lenh Serial (status, h, c, o, a, d, kp 30, refdeg 0.5, save...)">
          <button class="nut nutXanhDuong" style="padding:8px 14px" onclick="guiLenhTerminal()">GUI</button>
        </div>
      </div>
    </div>
  </div>
</div>

<script>
const vanTocToiDaXe = 0.40;
const tocDoQuayToiDaXe = 0.80;
const congWebSocket = 81;
const chuKyGuiNhiPhan = 30;
const chuKyDocNhatKyChamMs = 450;
const chuKyDocNhatKyNhanhMs = 80;
const soDongNhatKyMoiTrang = 20;
const soDongDomToiDa = 200;

let vanTocHienTai = 0;
let tocDoQuayHienTai = 0;
let maConTroTienLui = null;
let maConTroReNgang = null;
let boDemGuiLienTuc = null;
let mocGuiWsGanNhat = 0;
let mocGuiHttpGanNhat = 0;
let heSoTocDoDiaHinh = 1.0;
let daDienThongSoBanDau = false;
let cheDoTruoc = '';
let ketNoiWs = null;
let boDemKetNoiLaiWs = null;
let demKhungDaGui = 0;
let lichSuDoTre = new Array(8).fill(0);
let viTriDoTre = 0;
let soLanLoiLienTiep = 0;
let boHenThongBao = null;
let dangBatBench = false;
let cuaSoDangMo = '';
let terminalDangMo = false;
let chiSoNhatKyCuoi = 0;
let dangDocNhatKy = false;
let boHenDocNhatKy = null;
let lichSuLenhSerial = [];
let viTriLichSuSerial = -1;

const vungCanGatTienLui = document.getElementById('vungCanGatTienLui');
const numCanGatTienLui = document.getElementById('numCanGatTienLui');
const vungCanGatReNgang = document.getElementById('vungCanGatReNgang');
const numCanGatReNgang = document.getElementById('numCanGatReNgang');
const thanhTocDo = document.getElementById('thanhTocDo');
const oPhanTramTocDo = document.getElementById('oPhanTramTocDo');
const nhanTocDoTienLui = document.getElementById('nhanTocDoTienLui');
const nhanTocDoReNgang = document.getElementById('nhanTocDoReNgang');
const nhanKenhNhiPhan = document.getElementById('nhanKenhNhiPhan');
const khungHexNhiPhan = document.getElementById('khungHexNhiPhan');
const khungXuatSerial = document.getElementById('khungXuatSerial');
const oNhapLenhSerial = document.getElementById('oNhapLenhSerial');

thanhTocDo.addEventListener('input', () => {
  heSoTocDoDiaHinh = Number(thanhTocDo.value) / 100;
  oPhanTramTocDo.textContent = thanhTocDo.value + '%';
});

function hienThongBao(thongDiep) {
  const hopThongBao = document.getElementById('thongBaoNoi');
  hopThongBao.textContent = thongDiep;
  hopThongBao.style.opacity = '1';
  clearTimeout(boHenThongBao);
  boHenThongBao = setTimeout(() => { hopThongBao.style.opacity = '0'; }, 1800);
}

function capNhatTrangThaiKetNoi(daKetNoi, doTreMs) {
  const cham = document.getElementById('chamKetNoi');
  const nhan = document.getElementById('nhanKetNoiMang');
  const oTre = document.getElementById('oDoTreMang');
  if (!daKetNoi) {
    cham.className = 'chamKetNoi matKetNoi';
    nhan.textContent = 'Mat ket noi';
    nhan.style.color = '#ff7b72';
    return;
  }
  if (typeof doTreMs === 'number') {
    lichSuDoTre[viTriDoTre] = doTreMs;
    viTriDoTre = (viTriDoTre + 1) % lichSuDoTre.length;
    const trungBinh = Math.round(lichSuDoTre.reduce((a, b) => a + b, 0) / lichSuDoTre.length);
    oTre.textContent = trungBinh + 'ms';
    cham.className = trungBinh > 150 ? 'chamKetNoi ketNoiCham' : 'chamKetNoi ketNoiTot';
  } else {
    cham.className = 'chamKetNoi ketNoiTot';
  }
  nhan.textContent = 'Da ket noi';
  nhan.style.color = '#56d364';
}

function noiSuyTangTocKeo(giaTriKeo) {
  const giaTriKep = Math.max(-1, Math.min(1, giaTriKeo));
  const doLonTho = Math.abs(giaTriKep);
  const vungChet = 0.04;
  const heSoCong = 0.15;
  if (doLonTho <= vungChet) return 0;
  const tyLe = (doLonTho - vungChet) / (1 - vungChet);
  const doLon = (1 - heSoCong) * tyLe + heSoCong * tyLe * tyLe;
  return giaTriKep >= 0 ? doLon : -doLon;
}

function datViTriNumTienLui(lechY) {
  numCanGatTienLui.style.transform = `translate(0px, ${lechY.toFixed(1)}px)`;
}

function datViTriNumReNgang(lechX) {
  numCanGatReNgang.style.transform = `translate(${lechX.toFixed(1)}px, 0px)`;
}

function dongGoiKhungNhiPhan(maLenh, vanToc, tocDoQuay, phimKemTheo) {
  const boDem = new ArrayBuffer(8);
  const gocNhin = new DataView(boDem);
  const mucTien = Math.round(Math.max(-1, Math.min(1, vanToc / vanTocToiDaXe)) * 1000);
  const mucRe = Math.round(Math.max(-1, Math.min(1, tocDoQuay / tocDoQuayToiDaXe)) * 1000);
  gocNhin.setUint8(0, 0xA5);
  gocNhin.setUint8(1, maLenh & 0xFF);
  gocNhin.setInt16(2, mucTien, true);
  gocNhin.setInt16(4, mucRe, true);
  gocNhin.setUint8(6, phimKemTheo & 0xFF);
  let maXor = 0;
  for (let i = 0; i < 7; i++) maXor ^= gocNhin.getUint8(i);
  gocNhin.setUint8(7, maXor);
  return boDem;
}

function dinhDangHexKhung(boDem, coKhoangTrang) {
  const mangByte = new Uint8Array(boDem);
  let chuoi = '';
  for (let i = 0; i < mangByte.length; i++) {
    if (coKhoangTrang && i > 0) chuoi += ' ';
    chuoi += mangByte[i].toString(16).toUpperCase().padStart(2, '0');
  }
  return chuoi;
}

function capNhatTrangThaiKenh() {
  if (ketNoiWs && ketNoiWs.readyState === 1) {
    nhanKenhNhiPhan.className = 'nhan nhanBat';
    nhanKenhNhiPhan.textContent = 'WS NHI PHAN 8B';
  } else {
    nhanKenhNhiPhan.className = 'nhan nhanTat';
    nhanKenhNhiPhan.textContent = 'HTTP NHI PHAN 8B';
  }
}

function ketNoiWebSocketNhiPhan() {
  if (ketNoiWs && (ketNoiWs.readyState === 0 || ketNoiWs.readyState === 1)) return;
  try {
    const mayChu = window.location.hostname || '192.168.4.1';
    ketNoiWs = new WebSocket(`ws://${mayChu}:${congWebSocket}/`);
    ketNoiWs.binaryType = 'arraybuffer';
    ketNoiWs.onopen = () => capNhatTrangThaiKenh();
    ketNoiWs.onclose = () => {
      capNhatTrangThaiKenh();
      if (!boDemKetNoiLaiWs) {
        boDemKetNoiLaiWs = setTimeout(() => {
          boDemKetNoiLaiWs = null;
          ketNoiWebSocketNhiPhan();
        }, 1000);
      }
    };
    ketNoiWs.onerror = () => capNhatTrangThaiKenh();
  } catch (e) {
    capNhatTrangThaiKenh();
  }
}

function guiKhungNhiPhan(batBuoc, maLenh, phimKemTheo) {
  const boDem = dongGoiKhungNhiPhan(maLenh, vanTocHienTai, tocDoQuayHienTai, phimKemTheo);
  nhanTocDoTienLui.textContent = vanTocHienTai.toFixed(2) + ' m/s';
  nhanTocDoReNgang.textContent = tocDoQuayHienTai.toFixed(2) + ' rad/s';
  const chuoiHienThi = dinhDangHexKhung(boDem, true);
  const thoiGian = Date.now();
  if (ketNoiWs && ketNoiWs.readyState === 1) {
    if (!batBuoc && thoiGian - mocGuiWsGanNhat < 40) return false;
    mocGuiWsGanNhat = thoiGian;
    ketNoiWs.send(boDem);
    demKhungDaGui++;
    khungHexNhiPhan.textContent = `Khung 8B (WS): ${chuoiHienThi} | Da gui: ${demKhungDaGui}`;
    return true;
  }
  if (!batBuoc && thoiGian - mocGuiHttpGanNhat < 60) return false;
  mocGuiHttpGanNhat = thoiGian;
  demKhungDaGui++;
  khungHexNhiPhan.textContent = `Khung 8B (HTTP): ${chuoiHienThi} | Da gui: ${demKhungDaGui}`;
  fetch('/nhiphan?g=' + dinhDangHexKhung(boDem, false)).catch(() => {});
  return true;
}

function batDauGuiLienTuc() {
  if (boDemGuiLienTuc) return;
  boDemGuiLienTuc = setInterval(() => {
    if (maConTroTienLui !== null || maConTroReNgang !== null ||
        Math.abs(vanTocHienTai) > 0.001 || Math.abs(tocDoQuayHienTai) > 0.001) {
      guiKhungNhiPhan(false, 0x01, 0);
    } else {
      dungGuiLienTuc();
    }
  }, chuKyGuiNhiPhan);
}

function dungGuiLienTuc() {
  if (boDemGuiLienTuc) {
    clearInterval(boDemGuiLienTuc);
    boDemGuiLienTuc = null;
  }
}

function phanhGiuViTri() {
  maConTroTienLui = null;
  maConTroReNgang = null;
  vanTocHienTai = 0;
  tocDoQuayHienTai = 0;
  datViTriNumTienLui(0);
  datViTriNumReNgang(0);
  dungGuiLienTuc();
  guiKhungNhiPhan(true, 0x01, 0);
  hienThongBao('Phanh & giu vi tri dung');
}

function capNhatJoystickTienLui(toaDoY) {
  const khung = vungCanGatTienLui.getBoundingClientRect();
  const tamY = khung.top + khung.height / 2;
  const hanhTrinhToiDa = Math.max(10, khung.height / 2 - 33);
  const lechY = Math.max(-hanhTrinhToiDa, Math.min(hanhTrinhToiDa, toaDoY - tamY));
  datViTriNumTienLui(lechY);
  const tyLeKeo = -lechY / hanhTrinhToiDa;
  vanTocHienTai = noiSuyTangTocKeo(tyLeKeo) * vanTocToiDaXe * heSoTocDoDiaHinh;
  batDauGuiLienTuc();
  guiKhungNhiPhan(false, 0x01, 0);
}

function capNhatJoystickReNgang(toaDoX) {
  const khung = vungCanGatReNgang.getBoundingClientRect();
  const tamX = khung.left + khung.width / 2;
  const hanhTrinhToiDa = Math.max(10, khung.width / 2 - 33);
  const lechX = Math.max(-hanhTrinhToiDa, Math.min(hanhTrinhToiDa, toaDoX - tamX));
  datViTriNumReNgang(lechX);
  const tyLeKeo = -lechX / hanhTrinhToiDa;
  tocDoQuayHienTai = noiSuyTangTocKeo(tyLeKeo) * tocDoQuayToiDaXe * heSoTocDoDiaHinh;
  batDauGuiLienTuc();
  guiKhungNhiPhan(false, 0x01, 0);
}

function nhaJoystickTienLui(suKien) {
  if (maConTroTienLui === null || suKien.pointerId !== maConTroTienLui) return;
  maConTroTienLui = null;
  vanTocHienTai = 0;
  datViTriNumTienLui(0);
  if (maConTroReNgang === null) dungGuiLienTuc();
  guiKhungNhiPhan(true, 0x01, 0);
}

function nhaJoystickReNgang(suKien) {
  if (maConTroReNgang === null || suKien.pointerId !== maConTroReNgang) return;
  maConTroReNgang = null;
  tocDoQuayHienTai = 0;
  datViTriNumReNgang(0);
  if (maConTroTienLui === null) dungGuiLienTuc();
  guiKhungNhiPhan(true, 0x01, 0);
}

vungCanGatTienLui.addEventListener('pointerdown', (suKien) => {
  suKien.preventDefault();
  if (maConTroTienLui !== null) return;
  maConTroTienLui = suKien.pointerId;
  vungCanGatTienLui.setPointerCapture(suKien.pointerId);
  capNhatJoystickTienLui(suKien.clientY);
});
vungCanGatTienLui.addEventListener('pointermove', (suKien) => {
  if (suKien.pointerId !== maConTroTienLui) return;
  suKien.preventDefault();
  capNhatJoystickTienLui(suKien.clientY);
});
vungCanGatTienLui.addEventListener('pointerup', nhaJoystickTienLui);
vungCanGatTienLui.addEventListener('pointercancel', nhaJoystickTienLui);
vungCanGatTienLui.addEventListener('lostpointercapture', nhaJoystickTienLui);

vungCanGatReNgang.addEventListener('pointerdown', (suKien) => {
  suKien.preventDefault();
  if (maConTroReNgang !== null) return;
  maConTroReNgang = suKien.pointerId;
  vungCanGatReNgang.setPointerCapture(suKien.pointerId);
  capNhatJoystickReNgang(suKien.clientX);
});
vungCanGatReNgang.addEventListener('pointermove', (suKien) => {
  if (suKien.pointerId !== maConTroReNgang) return;
  suKien.preventDefault();
  capNhatJoystickReNgang(suKien.clientX);
});
vungCanGatReNgang.addEventListener('pointerup', nhaJoystickReNgang);
vungCanGatReNgang.addEventListener('pointercancel', nhaJoystickReNgang);
vungCanGatReNgang.addEventListener('lostpointercapture', nhaJoystickReNgang);

function guiPhim(kyTu, thongBao) {
  if (thongBao) hienThongBao(thongBao);
  if (ketNoiWs && ketNoiWs.readyState === 1) {
    guiKhungNhiPhan(true, 0x02, kyTu.charCodeAt(0));
    setTimeout(() => { capNhatTrangThaiXe(); docNhatKySerial(); }, 40);
    return;
  }
  fetch(`/lenh?phim=${encodeURIComponent(kyTu)}`)
    .then(() => { capNhatTrangThaiXe(); docNhatKySerial(); })
    .catch(() => {});
}

function guiDong(dongLenh, thongBao) {
  if (thongBao) hienThongBao(thongBao);
  fetch(`/lenh?dong=${encodeURIComponent(dongLenh)}`)
    .then(() => { capNhatTrangThaiXe(); docNhatKySerial(); })
    .catch(() => {});
}

function guiThongSo(tenLenh, idO) {
  const giaTri = document.getElementById(idO).value;
  if (giaTri === '') return;
  guiDong(`${tenLenh} ${giaTri}`, `Da dat ${tenLenh} = ${giaTri}`);
}

function batCanBangBanhXe() {
  if (dangBatBench) {
    if (ketNoiWs && ketNoiWs.readyState === 1) {
      ketNoiWs.send('CMD:bench 0');
      hienThongBao('Tat Bench, bat can bang banh xe');
      setTimeout(() => guiPhim('a', 'Bat can bang banh xe'), 60);
    } else {
      fetch('/lenh?dong=' + encodeURIComponent('bench 0'))
        .then(() => guiPhim('a', 'Bat can bang banh xe'))
        .catch(() => guiPhim('a', 'Bat can bang banh xe'));
    }
    return;
  }
  guiPhim('a', 'Bat can bang banh xe');
}

function moCuaSoCon(idCuaSo) {
  cuaSoDangMo = idCuaSo;
  document.getElementById('lopPhuCuaSoCon').classList.add('dangHienCuaSo');
  const danhSach = ['cuaSoCalib', 'cuaSoPid', 'cuaSoTerminal'];
  const tenTab = {cuaSoCalib: 'CALIB & CHE DO', cuaSoPid: 'TINH CHINH PID', cuaSoTerminal: 'TERMINAL SERIAL'};
  const idTab = {cuaSoCalib: 'nutTabCalib', cuaSoPid: 'nutTabPid', cuaSoTerminal: 'nutTabTerminal'};
  for (let i = 0; i < danhSach.length; i++) {
    const ma = danhSach[i];
    document.getElementById(ma).style.display = (ma === idCuaSo) ? 'block' : 'none';
    document.getElementById(idTab[ma]).className = (ma === idCuaSo) ? 'nutNho nutXanhDuong' : 'nutNho';
  }
  document.getElementById('tieuDeCuaSoCon').textContent = tenTab[idCuaSo] || 'MENU PHU';
  terminalDangMo = (idCuaSo === 'cuaSoTerminal');
  clearTimeout(boHenDocNhatKy);
  if (terminalDangMo) {
    docNhatKySerial();
  }
}

function dongCuaSoCon() {
  cuaSoDangMo = '';
  terminalDangMo = false;
  clearTimeout(boHenDocNhatKy);
  document.getElementById('lopPhuCuaSoCon').classList.remove('dangHienCuaSo');
}

function xoaManHinhTerminal() {
  khungXuatSerial.innerHTML = '';
}

function henDocNhatKy(treMs) {
  if (!terminalDangMo) return;
  clearTimeout(boHenDocNhatKy);
  boHenDocNhatKy = setTimeout(docNhatKySerial, treMs);
}

function themCacDongVaoTerminal(danhSachDong) {
  if (!danhSachDong || danhSachDong.length === 0) return;
  const manh = document.createDocumentFragment();
  for (let i = 0; i < danhSachDong.length; i++) {
    const oDong = document.createElement('div');
    oDong.textContent = danhSachDong[i];
    manh.appendChild(oDong);
  }
  khungXuatSerial.appendChild(manh);
  while (khungXuatSerial.children.length > soDongDomToiDa) {
    khungXuatSerial.removeChild(khungXuatSerial.firstChild);
  }
  khungXuatSerial.scrollTop = khungXuatSerial.scrollHeight;
}

function docNhatKySerial() {
  if (!terminalDangMo) return;
  if (dangDocNhatKy) {
    henDocNhatKy(chuKyDocNhatKyNhanhMs);
    return;
  }
  dangDocNhatKy = true;
  fetch(`/nhatky?tu=${chiSoNhatKyCuoi}&gioihan=${soDongNhatKyMoiTrang}`)
    .then((phanHoi) => phanHoi.json())
    .then((duLieu) => {
      if (duLieu.dong && duLieu.dong.length > 0) {
        themCacDongVaoTerminal(duLieu.dong);
      }
      if (typeof duLieu.tiep === 'number') chiSoNhatKyCuoi = duLieu.tiep;
      else if (typeof duLieu.tong === 'number') chiSoNhatKyCuoi = duLieu.tong;
      henDocNhatKy(duLieu.conNua ? chuKyDocNhatKyNhanhMs : chuKyDocNhatKyChamMs);
    })
    .catch(() => henDocNhatKy(chuKyDocNhatKyChamMs))
    .finally(() => { dangDocNhatKy = false; });
}

function guiLenhChuoiTerminal(lenhTho) {
  const lenh = (lenhTho || '').trim();
  if (!lenh) return;
  if (lichSuLenhSerial[0] !== lenh) {
    lichSuLenhSerial.unshift(lenh);
    if (lichSuLenhSerial.length > 20) lichSuLenhSerial.pop();
  }
  viTriLichSuSerial = -1;
  hienThongBao(`Lenh: ${lenh}`);
  if (ketNoiWs && ketNoiWs.readyState === 1 && lenh.length < 110) {
    ketNoiWs.send('CMD:' + lenh);
    setTimeout(() => { capNhatTrangThaiXe(); docNhatKySerial(); }, 40);
    return;
  }
  fetch('/nhatky/lenh', {method:'POST', headers:{'Content-Type':'text/plain'}, body:lenh})
    .then((phanHoi) => phanHoi.json())
    .then(() => { capNhatTrangThaiXe(); docNhatKySerial(); })
    .catch(() => {});
}

function guiLenhTerminal() {
  const lenh = oNhapLenhSerial.value.trim();
  if (!lenh) return;
  oNhapLenhSerial.value = '';
  guiLenhChuoiTerminal(lenh);
}

oNhapLenhSerial.addEventListener('keydown', (suKien) => {
  if (suKien.key === 'Enter') {
    suKien.preventDefault();
    guiLenhTerminal();
  } else if (suKien.key === 'ArrowUp') {
    suKien.preventDefault();
    if (viTriLichSuSerial < lichSuLenhSerial.length - 1) {
      viTriLichSuSerial++;
      oNhapLenhSerial.value = lichSuLenhSerial[viTriLichSuSerial];
    }
  } else if (suKien.key === 'ArrowDown') {
    suKien.preventDefault();
    if (viTriLichSuSerial > 0) {
      viTriLichSuSerial--;
      oNhapLenhSerial.value = lichSuLenhSerial[viTriLichSuSerial];
    } else {
      viTriLichSuSerial = -1;
      oNhapLenhSerial.value = '';
    }
  }
});

function capNhatTrangThaiXe() {
  const mocBatDau = performance.now();
  fetch('/trangthai')
    .then((phanHoi) => {
      if (!phanHoi.ok) throw new Error();
      return phanHoi.json();
    })
    .then((duLieu) => {
      soLanLoiLienTiep = 0;
      capNhatTrangThaiKetNoi(true, performance.now() - mocBatDau);
      const nhanTT = document.getElementById('nhanTrangThai');
      if (duLieu.tenLoi && duLieu.tenLoi !== 'none') {
        nhanTT.className = 'nhan nhanLoi';
        nhanTT.textContent = 'LOI: ' + duLieu.tenLoi;
      } else if (duLieu.daBat) {
        nhanTT.className = 'nhan nhanBat';
        nhanTT.textContent = 'DANG CAN BANG';
      } else {
        nhanTT.className = 'nhan nhanTat';
        nhanTT.textContent = 'TAT MOTOR';
      }
      dangBatBench = Boolean(duLieu.cheDoNangBanh);
      document.getElementById('oGocNghien').textContent = duLieu.gocNghienDo.toFixed(2) + '°';
      document.getElementById('oGocCanBang').textContent = duLieu.gocCanBangDo.toFixed(2) + '°';
      document.getElementById('oViTri').textContent = duLieu.viTriMet.toFixed(3) + 'm';
      document.getElementById('oVanToc').textContent = duLieu.vanTocMetGiay.toFixed(3);
      document.getElementById('oCheDo').textContent = `${duLieu.cheDoDieuKhien} (${dangBatBench ? 'Bench' : 'Banh'})`;
      document.getElementById('oLoi').textContent = duLieu.tenLoi + '/' + duLieu.tenSanSang;
      document.getElementById('oTongGoiNhiPhan').textContent = duLieu.tongGoiNhiPhan;
      const nutThuan = document.getElementById('nutChonPidThuan');
      const nutMo = document.getElementById('nutChonPidMo');
      const nutThuanNhanh = document.getElementById('nutPidThuanNhanh');
      const nutMoNhanh = document.getElementById('nutPidMoNhanh');
      if (duLieu.dangChayPidMo) {
        nutThuan.className = 'nut';
        nutMo.className = 'nut nutXanhDuong';
        nutThuanNhanh.className = 'nutNho';
        nutMoNhanh.className = 'nutNho nutXanhDuong';
      } else {
        nutThuan.className = 'nut nutXanhDuong';
        nutMo.className = 'nut';
        nutThuanNhanh.className = 'nutNho nutXanhDuong';
        nutMoNhanh.className = 'nutNho';
      }
      const nutVong = document.getElementById('nutVongViTri');
      const nutBench = document.getElementById('nutBenchMode');
      const nutDaoChieu = document.getElementById('nutDaoChieuDongCo');
      nutVong.textContent = 'VONG VI TRI (v): ' + (duLieu.batVongViTri ? 'ON' : 'OFF');
      nutVong.className = duLieu.batVongViTri ? 'nut nutXanh' : 'nut';
      nutBench.textContent = 'BENCH MODE (b): ' + (dangBatBench ? 'ON' : 'OFF');
      nutBench.className = dangBatBench ? 'nut nutVang' : 'nut';
      nutDaoChieu.textContent = 'DAO CHIEU DONG CO: ' + (duLieu.daoChieuDongCo ? 'DAO (ON)' : 'MAC DINH (OFF)');
      nutDaoChieu.className = duLieu.daoChieuDongCo ? 'nut nutVang' : 'nut';
      document.getElementById('tieuDePid').textContent = 'TINH CHINH: ' + duLieu.cheDoDieuKhien;
      document.getElementById('thongTinPhu').textContent =
        `K=(${duLieu.kpHieuDung.toFixed(2)}, ${duLieu.kiHieuDung.toFixed(2)}, ${duLieu.kdHieuDung.toFixed(2)}) | ` +
        `STEP(${duLieu.xungBanhTrai}, ${duLieu.xungBanhPhai}) | MKS(${duLieu.encoderTrai}, ${duLieu.encoderPhai}) | ` +
        `VongViTri=${duLieu.batVongViTri ? 'ON' : 'OFF'} | Bench=${duLieu.cheDoNangBanh ? 'ON' : 'OFF'} | ` +
        `DaoChieu=${duLieu.daoChieuDongCo ? 'ON' : 'OFF'} | motorFault=${duLieu.maLoiDongCo}`;
      document.getElementById('thongTinMang').textContent =
        `WiFi AP: ${duLieu.diaChiIp} | WS :81 | UDP :4210 | NhiPhan: ${duLieu.tongGoiNhiPhan} | ESP-NOW(${duLieu.diaChiMac}): ${duLieu.tongGoiEspNow}`;

      if (!daDienThongSoBanDau || duLieu.cheDoDieuKhien !== cheDoTruoc) {
        document.getElementById('nhapKp').value = duLieu.kpGoc.toFixed(2);
        document.getElementById('nhapKi').value = duLieu.kiGoc.toFixed(2);
        document.getElementById('nhapKd').value = duLieu.kdGoc.toFixed(2);
        document.getElementById('nhapKpv').value = duLieu.kpVanToc.toFixed(3);
        document.getElementById('nhapKiv').value = duLieu.kiVanToc.toFixed(3);
        if (!daDienThongSoBanDau) {
          document.getElementById('nhapRefDo').value = duLieu.gocCanBangDo.toFixed(2);
        }
        cheDoTruoc = duLieu.cheDoDieuKhien;
        daDienThongSoBanDau = true;
      }
    })
    .catch(() => {
      if (++soLanLoiLienTiep >= 3) capNhatTrangThaiKetNoi(false);
    });
}

ketNoiWebSocketNhiPhan();
setInterval(capNhatTrangThaiXe, 250);
capNhatTrangThaiXe();
</script>
</body>
</html>)HTML";
} // namespace

DieuKhienKhongDay* DieuKhienKhongDay::doiTuongHienTai = nullptr;

bool DieuKhienKhongDay::khoiTao(HamNhanDiChuyen hamDiChuyenMoi,
                                HamNhanPhimLenh hamPhimMoi,
                                HamNhanDongLenh hamDongMoi,
                                HamLayTrangThai hamTrangThaiMoi, HamNhanChanDoan hamChanDoanMoi) {
    hamDiChuyen = hamDiChuyenMoi;
    hamPhimLenh = hamPhimMoi;
    hamDongLenh = hamDongMoi;
    hamLayTrangThai = hamTrangThaiMoi;
    hamNhanChanDoan = hamChanDoanMoi;
    doiTuongHienTai = this;

    if (!batDieuKhienWifi && !batThuEspNow) return true;

    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);

    if (batDieuKhienWifi) {
        daBatWifi = WiFi.softAP(tenMangWifiAp, matKhauWifiAp, kenhWifiAp, 0, 4);
        if (daBatWifi) {
            const IPAddress ipAp = WiFi.softAPIP();
            std::snprintf(chuoiDiaChiIp, sizeof(chuoiDiaChiIp), "%u.%u.%u.%u",
                          ipAp[0], ipAp[1], ipAp[2], ipAp[3]);
            mayChuWeb.on("/", HTTP_GET, [this]() { xuLyTrangChu(); });
            mayChuWeb.on("/dichuyen", HTTP_GET, [this]() { xuLyApiDiChuyen(); });
            mayChuWeb.on("/nhiphan", HTTP_GET, [this]() { xuLyApiNhiPhan(); });
            mayChuWeb.on("/nhiphan", HTTP_POST, [this]() { xuLyApiNhiPhan(); });
            mayChuWeb.on("/lenh", HTTP_GET, [this]() { xuLyApiLenh(); });
            mayChuWeb.on("/trangthai", HTTP_GET, [this]() { xuLyApiTrangThai(); });
            mayChuWeb.on("/nhatky", HTTP_GET, [this]() { xuLyApiNhatKy(); });
            mayChuWeb.on("/nhatky/lenh", HTTP_GET, [this]() { xuLyApiLenhNhatKy(); });
            mayChuWeb.on("/nhatky/lenh", HTTP_POST, [this]() { xuLyApiLenhNhatKy(); });
            mayChuWeb.begin();
            mayChuWsNhiPhan.begin();
            mayChuWsNhiPhan.setNoDelay(true);
            congUdpLenh.begin(congUdpNhiPhan);
        }
    }

    const String macChuoi = WiFi.macAddress();
    std::snprintf(chuoiDiaChiMac, sizeof(chuoiDiaChiMac), "%s", macChuoi.c_str());

    if (batThuEspNow) {
        esp_wifi_set_channel(kenhWifiAp, WIFI_SECOND_CHAN_NONE);
        if (esp_now_init() == ESP_OK &&
            esp_now_register_recv_cb(
                khiNhanDuLieuEspNow) == ESP_OK) {
            daBatEspNow = true;
        }
    }
    return (!batDieuKhienWifi || daBatWifi) && (!batThuEspNow || daBatEspNow);
}

void DieuKhienKhongDay::khiNhanDuLieuEspNow(const esp_now_recv_info_t* thongTinNhan, const uint8_t* duLieu, int doDaiDuLieu) {
    if (!doiTuongHienTai || !duLieu || doDaiDuLieu <= 0) return;
    const auto* info = static_cast<const esp_now_recv_info_t*>(thongTinNhan);
    if (info && doiTuongHienTai->hamNhanChanDoan &&
        doiTuongHienTai->hamNhanChanDoan(info->src_addr, duLieu, static_cast<size_t>(doDaiDuLieu))) return;
    doiTuongHienTai->dayDuLieuEspNow(duLieu, doDaiDuLieu);
}

void DieuKhienKhongDay::dayDuLieuEspNow(const uint8_t* duLieu, int doDaiDuLieu) {
    if (doDaiDuLieu == static_cast<int>(sizeof(GoiLenhNhiPhan)) && duLieu[0] == dauKhungNhiPhan) {
        float vt = 0.0f;
        float qy = 0.0f;
        char phim = '\0';
        bool coDiChuyen = false;
        if (giaiMaGoiLenhNhiPhan(duLieu, static_cast<size_t>(doDaiDuLieu),
                                  vanTocDiChuyenToiDa_mps, tocDoQuayToiDa_radps,
                                  vt, qy, phim, coDiChuyen)) {
            portENTER_CRITICAL(&khoaDuLieuEspNow);
            if (coDiChuyen) {
                vanTocEspNowCho = vt;
                tocDoQuayEspNowCho = qy;
                coLenhDiChuyenEspNow = true;
            }
            if (phim != '\0') {
                phimHanhDongEspNowCho = phim;
            }
            ++demGoiEspNow;
            ++demGoiNhiPhan;
            portEXIT_CRITICAL(&khoaDuLieuEspNow);
            return;
        }
    }

    if (doDaiDuLieu == static_cast<int>(sizeof(GoiLenhEspNow))) {
        GoiLenhEspNow goiNhan{};
        std::memcpy(&goiNhan, duLieu, sizeof(GoiLenhEspNow));
        if (goiNhan.maNhanDang == maGoiEspNowHopLe &&
            std::isfinite(goiNhan.vanTocTienLui) &&
            std::isfinite(goiNhan.tocDoQuayNgang)) {
            const float vt = std::clamp(goiNhan.vanTocTienLui, -vanTocDiChuyenToiDa_mps, vanTocDiChuyenToiDa_mps);
            const float qy = std::clamp(goiNhan.tocDoQuayNgang, -tocDoQuayToiDa_radps, tocDoQuayToiDa_radps);
            portENTER_CRITICAL(&khoaDuLieuEspNow);
            vanTocEspNowCho = vt;
            tocDoQuayEspNowCho = qy;
            coLenhDiChuyenEspNow = true;
            if (goiNhan.maHanhDong != 0u) {
                phimHanhDongEspNowCho = static_cast<char>(goiNhan.maHanhDong);
            }
            ++demGoiEspNow;
            portEXIT_CRITICAL(&khoaDuLieuEspNow);
            return;
        }
    }

    if (doDaiDuLieu > 0 && doDaiDuLieu < static_cast<int>(sizeof(boDemChuoiEspNow))) {
        char boDemTam[sizeof(boDemChuoiEspNow)]{};
        std::memcpy(boDemTam, duLieu, static_cast<size_t>(doDaiDuLieu));
        boDemTam[doDaiDuLieu] = '\0';
        portENTER_CRITICAL(&khoaDuLieuEspNow);
        std::strncpy(boDemChuoiEspNow, boDemTam, sizeof(boDemChuoiEspNow) - 1u);
        boDemChuoiEspNow[sizeof(boDemChuoiEspNow) - 1u] = '\0';
        coChuoiLenhEspNow = true;
        ++demGoiEspNow;
        portEXIT_CRITICAL(&khoaDuLieuEspNow);
    }
}

void DieuKhienKhongDay::xuLy() {
    if (daBatWifi) {
        mayChuWeb.handleClient();
        xuLyWebSocketNhiPhan();
        xuLyUdpNhiPhan();
    }

    bool coDiChuyen = false;
    float vt = 0.0f;
    float qy = 0.0f;
    char phimNhan = '\0';
    bool coChuoi = false;
    char chuoiNhan[sizeof(boDemChuoiEspNow)]{};

    portENTER_CRITICAL(&khoaDuLieuEspNow);
    if (coLenhDiChuyenEspNow) {
        coDiChuyen = true;
        vt = vanTocEspNowCho;
        qy = tocDoQuayEspNowCho;
        coLenhDiChuyenEspNow = false;
    }
    if (phimHanhDongEspNowCho != '\0') {
        phimNhan = phimHanhDongEspNowCho;
        phimHanhDongEspNowCho = '\0';
    }
    if (coChuoiLenhEspNow) {
        coChuoi = true;
        std::strncpy(chuoiNhan, boDemChuoiEspNow, sizeof(chuoiNhan) - 1u);
        chuoiNhan[sizeof(chuoiNhan) - 1u] = '\0';
        coChuoiLenhEspNow = false;
    }
    portEXIT_CRITICAL(&khoaDuLieuEspNow);

    if (phimNhan != '\0' && hamPhimLenh) hamPhimLenh(phimNhan);
    if (coDiChuyen && hamDiChuyen) hamDiChuyen(vt, qy);
    if (coChuoi && hamDongLenh) hamDongLenh(chuoiNhan);
}

void DieuKhienKhongDay::xuLyTrangChu() {
    mayChuWeb.sendHeader("Cache-Control", "no-store");
    mayChuWeb.send_P(200, "text/html; charset=utf-8", trangDieuKhienHtml);
}

void DieuKhienKhongDay::xuLyApiDiChuyen() {
    float vt = 0.0f;
    float qy = 0.0f;
    if (mayChuWeb.hasArg("vantoc")) vt = mayChuWeb.arg("vantoc").toFloat();
    if (mayChuWeb.hasArg("quay")) qy = mayChuWeb.arg("quay").toFloat();
    if (!std::isfinite(vt)) vt = 0.0f;
    if (!std::isfinite(qy)) qy = 0.0f;
    vt = std::clamp(vt, -vanTocDiChuyenToiDa_mps, vanTocDiChuyenToiDa_mps);
    qy = std::clamp(qy, -tocDoQuayToiDa_radps, tocDoQuayToiDa_radps);
    if (hamDiChuyen) hamDiChuyen(vt, qy);
    mayChuWeb.send(200, "text/plain", "ok");
}

bool DieuKhienKhongDay::apDungKhungNhiPhan(const uint8_t* duLieu, size_t doDai) {
    float vt = 0.0f;
    float qy = 0.0f;
    char phim = '\0';
    bool coDiChuyen = false;
    if (!giaiMaGoiLenhNhiPhan(duLieu, doDai, vanTocDiChuyenToiDa_mps, tocDoQuayToiDa_radps,
                              vt, qy, phim, coDiChuyen)) {
        return false;
    }
    portENTER_CRITICAL(&khoaDuLieuEspNow);
    ++demGoiNhiPhan;
    portEXIT_CRITICAL(&khoaDuLieuEspNow);
    if (phim != '\0' && hamPhimLenh) hamPhimLenh(phim);
    if (coDiChuyen && hamDiChuyen) hamDiChuyen(vt, qy);
    return true;
}

void DieuKhienKhongDay::xuLyApiNhiPhan() {
    uint8_t duLieuNhiPhan[sizeof(GoiLenhNhiPhan)]{};
    bool hopLe = false;
    if (mayChuWeb.hasArg("g") && giaiMaChuoiHex8Byte(mayChuWeb.arg("g"), duLieuNhiPhan)) {
        hopLe = apDungKhungNhiPhan(duLieuNhiPhan, sizeof(duLieuNhiPhan));
    }
    mayChuWeb.sendHeader("Cache-Control", "no-store");
    mayChuWeb.send(hopLe ? 204 : 400, "text/plain", hopLe ? "" : "bad");
}

void DieuKhienKhongDay::xuLyUdpNhiPhan() {
    for (int luot = 0; luot < 8; ++luot) {
        const int kichThuoc = congUdpLenh.parsePacket();
        if (kichThuoc <= 0) break;
        if (kichThuoc == static_cast<int>(sizeof(GoiLenhNhiPhan))) {
            uint8_t boDemUdp[sizeof(GoiLenhNhiPhan)]{};
            const int daDoc = congUdpLenh.read(boDemUdp, sizeof(boDemUdp));
            if (daDoc == static_cast<int>(sizeof(GoiLenhNhiPhan))) {
                apDungKhungNhiPhan(boDemUdp, sizeof(boDemUdp));
            }
        } else {
            congUdpLenh.flush();
        }
    }
}

void DieuKhienKhongDay::xuLyWebSocketNhiPhan() {
    if (mayChuWsNhiPhan.hasClient()) {
        WiFiClient khachMoi = mayChuWsNhiPhan.accept();
        if (khachMoi) {
            khachMoi.setNoDelay(true);
            if (khachWsNhiPhan) khachWsNhiPhan.stop();
            khachWsNhiPhan = khachMoi;
            daBatTayWsNhiPhan = false;
            doDaiBatTayWs = 0u;
            doDaiKhungWs = 0u;
        }
    }

    if (!khachWsNhiPhan) return;
    if (!khachWsNhiPhan.connected()) {
        khachWsNhiPhan.stop();
        daBatTayWsNhiPhan = false;
        doDaiBatTayWs = 0u;
        doDaiKhungWs = 0u;
        return;
    }

    if (!daBatTayWsNhiPhan) {
        while (khachWsNhiPhan.available() > 0 && doDaiBatTayWs + 1u < sizeof(boDemBatTayWs)) {
            const int byteDoc = khachWsNhiPhan.read();
            if (byteDoc < 0) break;
            boDemBatTayWs[doDaiBatTayWs++] = static_cast<char>(byteDoc);
        }
        boDemBatTayWs[doDaiBatTayWs] = '\0';
        if (doDaiBatTayWs + 1u >= sizeof(boDemBatTayWs) &&
            std::strstr(boDemBatTayWs, "\r\n\r\n") == nullptr) {
            khachWsNhiPhan.stop();
            doDaiBatTayWs = 0u;
            return;
        }
        if (std::strstr(boDemBatTayWs, "\r\n\r\n") != nullptr) {
            char khoaWs[48]{};
            if (!timKhoaWebSocket(boDemBatTayWs, khoaWs, sizeof(khoaWs))) {
                khachWsNhiPhan.stop();
                doDaiBatTayWs = 0u;
                return;
            }
            char chuoiNoi[96]{};
            const int doDaiNoi = std::snprintf(
                chuoiNoi, sizeof(chuoiNoi), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", khoaWs);
            if (doDaiNoi <= 0 || static_cast<size_t>(doDaiNoi) >= sizeof(chuoiNoi)) {
                khachWsNhiPhan.stop();
                doDaiBatTayWs = 0u;
                return;
            }
            unsigned char bamSha1[20]{};
            if (mbedtls_sha1(reinterpret_cast<const unsigned char*>(chuoiNoi),
                             static_cast<size_t>(doDaiNoi), bamSha1) != 0) {
                khachWsNhiPhan.stop();
                doDaiBatTayWs = 0u;
                return;
            }
            char maChapNhanWs[48]{};
            size_t doDaiBase64 = 0u;
            if (mbedtls_base64_encode(reinterpret_cast<unsigned char*>(maChapNhanWs),
                                      sizeof(maChapNhanWs) - 1u, &doDaiBase64,
                                      bamSha1, sizeof(bamSha1)) != 0) {
                khachWsNhiPhan.stop();
                doDaiBatTayWs = 0u;
                return;
            }
            maChapNhanWs[doDaiBase64] = '\0';
            char phanHoi[224]{};
            const int doDaiPhanHoi = std::snprintf(
                phanHoi, sizeof(phanHoi),
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                "Sec-WebSocket-Accept: %s\r\n\r\n",
                maChapNhanWs);
            if (doDaiPhanHoi > 0) {
                khachWsNhiPhan.write(reinterpret_cast<const uint8_t*>(phanHoi),
                                     static_cast<size_t>(doDaiPhanHoi));
            }
            daBatTayWsNhiPhan = true;
            doDaiBatTayWs = 0u;
            doDaiKhungWs = 0u;
        }
        return;
    }

    while (khachWsNhiPhan.available() > 0 && doDaiKhungWs < sizeof(boDemKhungWs)) {
        const int byteDoc = khachWsNhiPhan.read();
        if (byteDoc < 0) break;
        boDemKhungWs[doDaiKhungWs++] = static_cast<uint8_t>(byteDoc);
    }

    for (int luot = 0; luot < 8 && doDaiKhungWs >= 2u; ++luot) {
        const uint8_t byte0 = boDemKhungWs[0];
        const uint8_t byte1 = boDemKhungWs[1];
        const uint8_t maTacVu = byte0 & 0x0Fu;
        const bool coMatNa = (byte1 & 0x80u) != 0u;
        uint64_t doDaiTai = byte1 & 0x7Fu;
        size_t viTriHeader = 2u;
        if (doDaiTai == 126u) {
            if (doDaiKhungWs < 4u) break;
            doDaiTai = (static_cast<uint64_t>(boDemKhungWs[2]) << 8) | boDemKhungWs[3];
            viTriHeader = 4u;
        } else if (doDaiTai == 127u || doDaiTai > 128u) {
            khachWsNhiPhan.stop();
            daBatTayWsNhiPhan = false;
            doDaiKhungWs = 0u;
            break;
        }
        if (doDaiTai > 128u) {
            khachWsNhiPhan.stop();
            daBatTayWsNhiPhan = false;
            doDaiKhungWs = 0u;
            break;
        }
        const size_t viTriMask = viTriHeader;
        const size_t viTriPayload = coMatNa ? (viTriHeader + 4u) : viTriHeader;
        const size_t tongDoDaiKhung = viTriPayload + static_cast<size_t>(doDaiTai);
        if (doDaiKhungWs < tongDoDaiKhung) break;

        uint8_t taiGiaiMa[128]{};
        for (size_t i = 0u; i < static_cast<size_t>(doDaiTai); ++i) {
            taiGiaiMa[i] = coMatNa
                ? static_cast<uint8_t>(boDemKhungWs[viTriPayload + i] ^ boDemKhungWs[viTriMask + (i & 3u)])
                : boDemKhungWs[viTriPayload + i];
        }

        const size_t conLai = doDaiKhungWs - tongDoDaiKhung;
        if (conLai > 0u) {
            std::memmove(boDemKhungWs, boDemKhungWs + tongDoDaiKhung, conLai);
        }
        doDaiKhungWs = conLai;

        if (maTacVu == 0x08u) {
            const uint8_t khungDong[2] = {0x88u, 0x00u};
            khachWsNhiPhan.write(khungDong, sizeof(khungDong));
            khachWsNhiPhan.stop();
            daBatTayWsNhiPhan = false;
            doDaiKhungWs = 0u;
            break;
        }
        if (maTacVu == 0x09u) {
            uint8_t khungPong[130]{};
            khungPong[0] = 0x8Au;
            khungPong[1] = static_cast<uint8_t>(doDaiTai);
            if (doDaiTai > 0u) {
                std::memcpy(khungPong + 2u, taiGiaiMa, static_cast<size_t>(doDaiTai));
            }
            khachWsNhiPhan.write(khungPong, 2u + static_cast<size_t>(doDaiTai));
            continue;
        }
        if (maTacVu == 0x01u && doDaiTai >= 4u && std::memcmp(taiGiaiMa, "CMD:", 4u) == 0) {
            char chuoiLenhWs[129]{};
            const size_t doDaiLenh = static_cast<size_t>(doDaiTai) - 4u;
            std::memcpy(chuoiLenhWs, taiGiaiMa + 4u, doDaiLenh);
            chuoiLenhWs[doDaiLenh] = '\0';
            thucThiChuoiLenhWeb(chuoiLenhWs);
            continue;
        }
        if ((maTacVu == 0x02u || maTacVu == 0x01u) && doDaiTai == sizeof(GoiLenhNhiPhan)) {
            apDungKhungNhiPhan(taiGiaiMa, sizeof(GoiLenhNhiPhan));
        }
    }
}

void DieuKhienKhongDay::ghiDongNhatKy(const char* noiDung) {
    if (!noiDung || *noiDung == '\0') return;
    const char* conTro = noiDung;
    while (*conTro != '\0') {
        while (*conTro == '\r' || *conTro == '\n') ++conTro;
        if (*conTro == '\0') break;
        char dongTam[doDaiDongNhatKyToiDa]{};
        size_t viTri = 0u;
        while (*conTro != '\0' && *conTro != '\r' && *conTro != '\n') {
            if (viTri + 1u < doDaiDongNhatKyToiDa) {
                dongTam[viTri++] = *conTro;
            }
            ++conTro;
        }
        dongTam[viTri] = '\0';
        if (viTri > 0u) {
            portENTER_CRITICAL(&khoaDuLieuEspNow);
            const size_t viTriVong = static_cast<size_t>(tongSoDongNhatKy % soDongNhatKyToiDa);
            std::strncpy(boDemDongNhatKy[viTriVong], dongTam, doDaiDongNhatKyToiDa - 1u);
            boDemDongNhatKy[viTriVong][doDaiDongNhatKyToiDa - 1u] = '\0';
            ++tongSoDongNhatKy;
            portEXIT_CRITICAL(&khoaDuLieuEspNow);
        }
    }
}

void DieuKhienKhongDay::ghiNhatKyDinhDang(const char* dinhDang, ...) {
    if (!dinhDang) return;
    char boDemTam[320]{};
    va_list danhSachThamSo;
    va_start(danhSachThamSo, dinhDang);
    std::vsnprintf(boDemTam, sizeof(boDemTam), dinhDang, danhSachThamSo);
    va_end(danhSachThamSo);
    ghiDongNhatKy(boDemTam);
}

void DieuKhienKhongDay::thucThiChuoiLenhWeb(const char* chuoiLenh) {
    if (!chuoiLenh) return;
    while (*chuoiLenh == ' ' || *chuoiLenh == '\t') ++chuoiLenh;
    if (*chuoiLenh == '\0') return;

    char banSao[96]{};
    std::strncpy(banSao, chuoiLenh, sizeof(banSao) - 1u);
    size_t doDai = std::strlen(banSao);
    while (doDai > 0u && (banSao[doDai - 1u] == ' ' || banSao[doDai - 1u] == '\t' ||
                          banSao[doDai - 1u] == '\r' || banSao[doDai - 1u] == '\n')) {
        banSao[--doDai] = '\0';
    }
    if (doDai == 0u) return;

    ghiNhatKyDinhDang("> %s", banSao);
    if (doDai == 1u) {
        if (hamPhimLenh) hamPhimLenh(banSao[0]);
    } else if (hamDongLenh) {
        hamDongLenh(banSao);
    }
}

void DieuKhienKhongDay::xuLyApiNhatKy() {
    uint32_t tuChiSo = 0u;
    if (mayChuWeb.hasArg("tu")) {
        const long giaTri = mayChuWeb.arg("tu").toInt();
        if (giaTri > 0) tuChiSo = static_cast<uint32_t>(giaTri);
    }
    size_t gioiHan = 20u;
    if (mayChuWeb.hasArg("gioihan")) {
        const long giaTri = mayChuWeb.arg("gioihan").toInt();
        if (giaTri >= 1 && giaTri <= 24) gioiHan = static_cast<size_t>(giaTri);
    }

    portENTER_CRITICAL(&khoaDuLieuEspNow);
    const uint32_t tongHienTai = tongSoDongNhatKy;
    const uint32_t cuNhatConLuu =
        (tongHienTai > soDongNhatKyToiDa) ? (tongHienTai - static_cast<uint32_t>(soDongNhatKyToiDa)) : 0u;
    uint32_t batDau = (tuChiSo > cuNhatConLuu) ? tuChiSo : cuNhatConLuu;
    if (batDau > tongHienTai) batDau = tongHienTai;
    const uint32_t conLaiTuBatDau = tongHienTai - batDau;
    const uint32_t soDongLay = (conLaiTuBatDau > static_cast<uint32_t>(gioiHan))
        ? static_cast<uint32_t>(gioiHan) : conLaiTuBatDau;
    const uint32_t ketThuc = batDau + soDongLay;
    portEXIT_CRITICAL(&khoaDuLieuEspNow);

    String phanHoi;
    phanHoi.reserve(1024);
    phanHoi += "{\"tong\":";
    phanHoi += String(tongHienTai);
    phanHoi += ",\"tiep\":";
    phanHoi += String(ketThuc);
    phanHoi += ",\"conNua\":";
    phanHoi += (ketThuc < tongHienTai) ? "true" : "false";
    phanHoi += ",\"dong\":[";

    bool daCoPhanTu = false;
    for (uint32_t chiSo = batDau; chiSo < ketThuc; ++chiSo) {
        char dongTam[doDaiDongNhatKyToiDa]{};
        bool layDuoc = false;
        portENTER_CRITICAL(&khoaDuLieuEspNow);
        const uint32_t moiNhat = tongSoDongNhatKy;
        if (chiSo < moiNhat && (moiNhat - chiSo) <= soDongNhatKyToiDa) {
            const size_t viTriVong = static_cast<size_t>(chiSo % soDongNhatKyToiDa);
            std::strncpy(dongTam, boDemDongNhatKy[viTriVong], doDaiDongNhatKyToiDa - 1u);
            dongTam[doDaiDongNhatKyToiDa - 1u] = '\0';
            layDuoc = true;
        }
        portEXIT_CRITICAL(&khoaDuLieuEspNow);
        if (!layDuoc) continue;
        if (daCoPhanTu) phanHoi += ',';
        phanHoi += '"';
        noiChuoiJsonAnToan(phanHoi, dongTam);
        phanHoi += '"';
        daCoPhanTu = true;
    }
    phanHoi += "]}";

    mayChuWeb.sendHeader("Cache-Control", "no-store");
    mayChuWeb.send(200, "application/json", phanHoi);
}

void DieuKhienKhongDay::xuLyApiLenhNhatKy() {
    String chuoiLenh;
    if (mayChuWeb.hasArg("dong")) chuoiLenh = mayChuWeb.arg("dong");
    else if (mayChuWeb.hasArg("plain")) chuoiLenh = mayChuWeb.arg("plain");
    chuoiLenh.trim();
    if (chuoiLenh.length() == 0) {
        mayChuWeb.sendHeader("Cache-Control", "no-store");
        mayChuWeb.send(400, "application/json", "{\"ok\":false}");
        return;
    }
    thucThiChuoiLenhWeb(chuoiLenh.c_str());

    portENTER_CRITICAL(&khoaDuLieuEspNow);
    const uint32_t tongHienTai = tongSoDongNhatKy;
    portEXIT_CRITICAL(&khoaDuLieuEspNow);

    char phanHoi[64]{};
    std::snprintf(phanHoi, sizeof(phanHoi), "{\"ok\":true,\"tong\":%lu}",
                  static_cast<unsigned long>(tongHienTai));
    mayChuWeb.sendHeader("Cache-Control", "no-store");
    mayChuWeb.send(200, "application/json", phanHoi);
}

void DieuKhienKhongDay::xuLyApiLenh() {
    if (mayChuWeb.hasArg("phim")) {
        const String chuoiPhim = mayChuWeb.arg("phim");
        if (chuoiPhim.length() >= 1 && hamPhimLenh) {
            hamPhimLenh(chuoiPhim.charAt(0));
        }
    }
    if (mayChuWeb.hasArg("dong")) {
        const String dongLenh = mayChuWeb.arg("dong");
        if (dongLenh.length() > 0 && hamDongLenh) {
            hamDongLenh(dongLenh.c_str());
        }
    }
    mayChuWeb.send(200, "text/plain", "ok");
}

void DieuKhienKhongDay::xuLyApiTrangThai() {
    ThongTinTrangThaiWeb trangThai{};
    if (hamLayTrangThai) hamLayTrangThai(trangThai);
    char boDemJson[960]{};
    std::snprintf(
        boDemJson, sizeof(boDemJson),
        "{\"daBat\":%s,\"dangChayPidMo\":%s,\"cheDoNangBanh\":%s,\"batVongViTri\":%s,\"daoChieuDongCo\":%s,"
        "\"gocHopLe\":%s,\"heThongSanSang\":%s,"
        "\"gocNghienDo\":%.2f,\"gocCanBangDo\":%.2f,\"gocDatDo\":%.2f,"
        "\"viTriMet\":%.4f,\"vanTocMetGiay\":%.4f,\"tocDoGocRadGiay\":%.4f,"
        "\"xungBanhTrai\":%lld,\"xungBanhPhai\":%lld,\"encoderTrai\":%lld,\"encoderPhai\":%lld,"
        "\"kpGoc\":%.3f,\"kiGoc\":%.3f,\"kdGoc\":%.3f,\"kpVanToc\":%.4f,\"kiVanToc\":%.4f,"
        "\"kpHieuDung\":%.2f,\"kiHieuDung\":%.2f,\"kdHieuDung\":%.2f,"
        "\"maLoiDongCo\":%u,\"tenLoi\":\"%s\",\"tenSanSang\":\"%s\",\"cheDoDieuKhien\":\"%s\","
        "\"tongGoiEspNow\":%lu,\"tongGoiNhiPhan\":%lu,\"diaChiIp\":\"%s\",\"diaChiMac\":\"%s\"}",
        trangThai.daBat ? "true" : "false",
        trangThai.dangChayPidMo ? "true" : "false",
        trangThai.cheDoNangBanh ? "true" : "false",
        trangThai.batVongViTri ? "true" : "false",
        trangThai.daoChieuDongCo ? "true" : "false",
        trangThai.gocHopLe ? "true" : "false",
        trangThai.heThongSanSang ? "true" : "false",
        trangThai.gocNghienDo, trangThai.gocCanBangDo, trangThai.gocDatDo,
        trangThai.viTriMet, trangThai.vanTocMetGiay, trangThai.tocDoGocRadGiay,
        trangThai.xungBanhTrai, trangThai.xungBanhPhai,
        trangThai.encoderTrai, trangThai.encoderPhai,
        trangThai.kpGoc, trangThai.kiGoc, trangThai.kdGoc,
        trangThai.kpVanToc, trangThai.kiVanToc,
        trangThai.kpHieuDung, trangThai.kiHieuDung, trangThai.kdHieuDung,
        trangThai.maLoiDongCo, trangThai.tenLoi, trangThai.tenSanSang, trangThai.cheDoDieuKhien,
        static_cast<unsigned long>(demGoiEspNow), static_cast<unsigned long>(demGoiNhiPhan),
        chuoiDiaChiIp, chuoiDiaChiMac);
    mayChuWeb.sendHeader("Cache-Control", "no-store");
    mayChuWeb.send(200, "application/json", boDemJson);
}
