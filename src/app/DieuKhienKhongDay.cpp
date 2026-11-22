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
uint32_t mocMoWsMs = 0u;
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

bool giaiMaChuoiHexLenh(const String& chuoiHex, uint8_t* duLieuRa, size_t& doDaiRa) {
    doDaiRa = chuoiHex.length() / 2u;
    if ((chuoiHex.length() & 1u) ||
        (doDaiRa != sizeof(GoiLenhNhiPhan) && doDaiRa != sizeof(GoiLenhWebKhaoSat))) return false;
    for (size_t chiSo = 0u; chiSo < doDaiRa; ++chiSo) {
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
<title>Xe Cân Bằng ESP32-S3</title>
<style>
:root{--nen:#0b0f14;--the:#141a23;--o:#0d1219;--vien:#273242;--chu:#eef3f8;--mo:#92a0b3;--xanh:#238636;--xanhSang:#3fb950;--xanhDuong:#1f6feb;--xanhDuongSang:#58a6ff;--do:#da3633;--doNhat:#ff7b72;--vang:#9e6a03;--vangSang:#e3b341}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;padding:10px;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:var(--nen);color:var(--chu);overscroll-behavior:contain;line-height:1.35}
button,input{font-family:inherit}
.khungChinh{max-width:1120px;margin:0 auto;display:flex;flex-direction:column;gap:10px}
.luoiNoiDung{display:flex;flex-direction:column;gap:10px}
.cotTrangThai,.cotDieuKhien{display:flex;flex-direction:column;gap:10px;min-width:0}
.the{background:var(--the);border:1px solid var(--vien);border-radius:16px;padding:12px;box-shadow:0 6px 20px rgba(0,0,0,.35)}
.tieuDe{display:flex;justify-content:space-between;align-items:center;gap:8px;margin-bottom:8px;font-weight:700;font-size:14px}
.tenXe{display:flex;align-items:center;gap:8px;font-size:15px;letter-spacing:.2px}
.bieuTuongXe{width:24px;height:24px;border-radius:7px;background:rgba(31,111,235,.18);border:1px solid rgba(88,166,255,.4);display:inline-flex;align-items:center;justify-content:center;color:var(--xanhDuongSang);flex-shrink:0}
.thanhTrangThaiDinh{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin-bottom:8px}
.mucTrangThai{display:inline-flex;align-items:center;gap:5px;padding:4px 9px;border-radius:8px;background:var(--o);border:1px solid var(--vien);font-size:11.5px;font-weight:600;white-space:nowrap}
.chamKetNoi{width:8px;height:8px;border-radius:50%;display:inline-block;background:var(--mo);flex-shrink:0}
.ketNoiTot{background:#00e676;box-shadow:0 0 8px rgba(0,230,118,.75)}
.ketNoiCham{background:#ffab00;box-shadow:0 0 8px rgba(255,171,0,.75)}
.matKetNoi{background:#ff5252;box-shadow:0 0 8px rgba(255,82,82,.75)}
.nhan{display:inline-flex;align-items:center;padding:4px 10px;border-radius:999px;font-size:11.5px;font-weight:700;background:#21262d;color:var(--chu);border:1px solid var(--vien);white-space:nowrap}
.nhanBat{background:rgba(46,160,67,.2);color:#56d364;border-color:rgba(63,185,80,.55)}
.nhanTat{background:rgba(146,160,179,.12);color:var(--mo);border-color:var(--vien)}
.nhanLoi{background:rgba(248,81,73,.22);color:#ff7b72;border-color:rgba(248,81,73,.6)}
.nhanCanhBao{background:rgba(210,153,34,.2);color:var(--vangSang);border-color:rgba(227,179,65,.55)}
.khungHuongDanAnToan{display:flex;align-items:center;justify-content:space-between;gap:8px;padding:8px 10px;border-radius:11px;font-size:12.5px;font-weight:600;margin-bottom:9px;border:1px solid var(--vien);background:var(--o);color:var(--chu)}
.huongDanXanh{background:rgba(46,160,67,.13);border-color:rgba(63,185,80,.45);color:#7ee787}
.huongDanVang{background:rgba(210,153,34,.14);border-color:rgba(227,179,65,.5);color:#f2cc60}
.huongDanDo{background:rgba(248,81,73,.16);border-color:rgba(248,81,73,.55);color:#ffa198}
.chuHuongDan{display:flex;align-items:center;gap:7px;min-width:0;flex:1}
.chamHuongDan{width:8px;height:8px;border-radius:50%;background:currentColor;flex-shrink:0}
.khungThuocGoc{background:var(--o);border:1px solid var(--vien);border-radius:12px;padding:9px 10px;margin-bottom:8px}
.hangChiSoGoc{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-bottom:8px}
.oGocNho{text-align:center}
.tenSo{font-size:11px;color:var(--mo);margin-bottom:2px}
.giaTriSo{font-size:clamp(12px,3.2vw,14.5px);font-weight:700;font-variant-numeric:tabular-nums;word-break:break-word}
.giaTriGocChinh{font-size:17px;font-weight:800;font-variant-numeric:tabular-nums;color:var(--xanhDuongSang)}
.thuocGocNghieng{position:relative;height:20px;border-radius:999px;background:linear-gradient(90deg,rgba(248,81,73,.25) 0%,rgba(210,153,34,.18) 20%,rgba(39,50,66,.5) 36%,rgba(39,50,66,.5) 64%,rgba(210,153,34,.18) 80%,rgba(248,81,73,.25) 100%);border:1px solid #324054;overflow:hidden}
.vungAnToanKhoiDong{position:absolute;top:0;bottom:0;left:40.5%;width:19%;background:rgba(63,185,80,.24);border-left:1px dashed rgba(63,185,80,.65);border-right:1px dashed rgba(63,185,80,.65);pointer-events:none}
.vachTamThuoc{position:absolute;top:0;bottom:0;left:50%;width:1px;background:#5b6b80;pointer-events:none}
.vachGocDat{position:absolute;top:2px;bottom:2px;left:50%;width:3px;margin-left:-1.5px;border-radius:2px;background:var(--xanhDuongSang);opacity:.9;transition:left .12s linear;pointer-events:none}
.kimGocNghien{position:absolute;top:50%;left:50%;width:14px;height:14px;margin:-7px 0 0 -7px;border-radius:50%;background:var(--xanhSang);border:2px solid #fff;box-shadow:0 0 8px rgba(0,0,0,.65);transition:left .12s linear;pointer-events:none}
.kimGocNghien.kimVang{background:var(--vangSang)}
.kimGocNghien.kimDo{background:var(--doNhat)}
.chuThichThuoc{display:flex;justify-content:space-between;font-size:10px;color:var(--mo);margin-top:4px;font-variant-numeric:tabular-nums}
.luoiChiSo{display:grid;grid-template-columns:repeat(2,1fr);gap:7px}
.oSo{background:var(--o);border:1px solid var(--vien);border-radius:10px;padding:7px 8px;text-align:center}
.hopChanDoan{margin-top:8px;background:var(--o);border:1px solid var(--vien);border-radius:10px;padding:7px 9px}
.hopChanDoan summary{cursor:pointer;font-size:12px;font-weight:700;color:var(--mo);user-select:none;-webkit-user-select:none;list-style:none}
.hopChanDoan summary::-webkit-details-marker{display:none}
.hopChanDoan[open] summary{color:var(--chu);margin-bottom:6px}
.hangNut{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}
.hangBaNut{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.hangNutDongCo{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;margin-bottom:10px}
.nut{border:1px solid var(--vien);background:#212833;color:var(--chu);border-radius:12px;padding:11px 8px;min-height:46px;font-size:13px;font-weight:700;cursor:pointer;touch-action:manipulation;user-select:none;-webkit-user-select:none;display:inline-flex;flex-direction:column;align-items:center;justify-content:center;gap:2px;text-align:center;transition:transform .08s,filter .12s,border-color .15s}
.nut:active{transform:scale(.97);filter:brightness(1.12)}
.nutPhuNho{font-size:10.5px;font-weight:500;opacity:.88}
.nutXanh{background:var(--xanh);border-color:var(--xanhSang);color:#fff}
.nutDo{background:#b62324;border-color:var(--do);color:#fff}
.nutVang{background:var(--vang);border-color:var(--vangSang);color:#fff}
.nutXanhDuong{background:var(--xanhDuong);border-color:#4493f8;color:#fff}
.nutGoiYBat{box-shadow:0 0 0 2px rgba(63,185,80,.45)}
.nutBiKhoa{opacity:.48;filter:saturate(.4)}
.nutNho{border:1px solid var(--vien);background:#212833;color:var(--chu);border-radius:9px;padding:6px 10px;min-height:34px;font-size:11.5px;font-weight:700;cursor:pointer;touch-action:manipulation;display:inline-flex;align-items:center;justify-content:center}
.nutNho:active{transform:scale(.96)}
.nutNho.nutXanhDuong{background:var(--xanhDuong);border-color:#4493f8;color:#fff}
.nutNho.nutVang{background:var(--vang);border-color:var(--vangSang);color:#fff}
.nutNho.nutDo{background:#b62324;border-color:var(--do);color:#fff}
.khungTocDo{background:var(--o);border:1px solid var(--vien);border-radius:12px;padding:8px 10px;margin-bottom:9px}
.hangPresetTocDo{display:flex;align-items:center;gap:6px;flex-wrap:wrap;margin-top:4px}
.hangPresetTocDo .thanhTruot{flex:1;min-width:130px;margin:0}
.thanhTruot{width:100%;accent-color:#2f81f7;height:26px;cursor:pointer}
.khuHaiCanGat{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;align-items:stretch}
.hopCanGat{background:var(--o);border:1px solid var(--vien);border-radius:14px;padding:9px 8px;display:flex;flex-direction:column;align-items:center;justify-content:space-between;gap:7px}
.tieuDeCanGat{width:100%;display:flex;justify-content:space-between;align-items:center;gap:4px;font-size:11.5px;font-weight:700;color:var(--chu)}
.giaTriTruc{font-size:11.5px;font-weight:700;color:var(--xanhDuongSang);font-variant-numeric:tabular-nums;white-space:nowrap}
.vungCanGatDoc{position:relative;width:116px;height:192px;border-radius:28px;background:linear-gradient(180deg,#182434 0%,#0b1017 50%,#182434 100%);border:2px solid #354457;touch-action:none;user-select:none;-webkit-user-select:none;transition:border-color .12s,box-shadow .12s}
.vungCanGatDoc.dangKeo,.vungCanGatNgang.dangKeo{border-color:var(--xanhDuongSang);box-shadow:0 0 0 3px rgba(88,166,255,.2)}
.vungCanGatNgang.dangKeo{border-color:var(--xanhSang);box-shadow:0 0 0 3px rgba(63,185,80,.2)}
.vachTamNgang{position:absolute;left:12px;right:12px;top:50%;height:1px;background:#36485f;pointer-events:none}
.vachTamDoc{position:absolute;top:12px;bottom:12px;left:50%;width:1px;background:#36485f;pointer-events:none}
.khungRanhNgang{display:flex;align-items:center;justify-content:center;width:100%;height:192px}
.vungCanGatNgang{position:relative;width:100%;max-width:200px;height:116px;border-radius:28px;background:linear-gradient(90deg,#182434 0%,#0b1017 50%,#182434 100%);border:2px solid #354457;touch-action:none;user-select:none;-webkit-user-select:none;transition:border-color .12s,box-shadow .12s}
.nhanTrucDinh{position:absolute;top:8px;left:0;right:0;text-align:center;font-size:10.5px;font-weight:700;color:var(--mo);pointer-events:none}
.nhanTrucDay{position:absolute;bottom:8px;left:0;right:0;text-align:center;font-size:10.5px;font-weight:700;color:var(--mo);pointer-events:none}
.nhanTrucTrai{position:absolute;left:8px;top:50%;transform:translateY(-50%);font-size:10.5px;font-weight:700;color:var(--mo);pointer-events:none}
.nhanTrucPhai{position:absolute;right:8px;top:50%;transform:translateY(-50%);font-size:10.5px;font-weight:700;color:var(--mo);pointer-events:none}
.numCanGatTienLui{position:absolute;left:50%;top:50%;width:60px;height:60px;margin:-30px 0 0 -30px;border-radius:50%;background:radial-gradient(circle at 32% 32%,#79c0ff,#1f6feb);border:2px solid #b5dcff;box-shadow:0 6px 18px rgba(0,0,0,.6);pointer-events:none}
.numCanGatReNgang{position:absolute;left:50%;top:50%;width:60px;height:60px;margin:-30px 0 0 -30px;border-radius:50%;background:radial-gradient(circle at 32% 32%,#7ee787,#238636);border:2px solid #b8f5bf;box-shadow:0 6px 18px rgba(0,0,0,.6);pointer-events:none}
.goiYBanPhim{display:none;font-size:11.5px;color:var(--mo);text-align:center;margin-top:6px}
.khungTerminalSerial{background:#080c10;border:1px solid #303a48;border-radius:12px;padding:10px;display:flex;flex-direction:column;gap:8px;touch-action:auto}
.hangNutTerminal{display:flex;flex-wrap:wrap;gap:5px;touch-action:auto}
.khungXuatSerial{height:240px;overflow-y:auto;background:#04070a;border:1px solid #222c38;border-radius:8px;padding:8px;font-family:ui-monospace,SFMono-Regular,Consolas,'Courier New',monospace;font-size:11.5px;line-height:1.45;color:#c9d1d9;word-break:break-word;touch-action:pan-y;user-select:text;-webkit-user-select:text;-webkit-overflow-scrolling:touch}
.khungXuatSerial div{padding:2px 0;border-bottom:1px solid rgba(255,255,255,.04)}
.dongLenh{color:#79c0ff}
.dongTot{color:#56d364}
.dongCanhBao{color:#e3b341}
.dongLoi{color:#ff7b72}
.hangNhapSerial{display:flex;gap:6px;touch-action:auto}
.oNhapLenhSerial{flex:1;min-width:0;background:var(--o);border:1px solid #384454;border-radius:9px;color:#7ee787;padding:9px 10px;font-size:13px;font-family:ui-monospace,SFMono-Regular,Consolas,'Courier New',monospace;outline:none;touch-action:auto;user-select:text;-webkit-user-select:text}
.oNhapLenhSerial:focus{border-color:#4493f8}
.luoiPid{display:grid;grid-template-columns:1fr;gap:8px}
.nhomNhap{display:flex;gap:5px;align-items:center;background:var(--the);border:1px solid var(--vien);border-radius:10px;padding:6px 8px}
.nhomNhap label{font-size:12px;font-weight:700;color:var(--mo);min-width:48px}
.nhomNhap input{flex:1;width:100%;min-width:0;background:var(--o);border:1px solid var(--vien);border-radius:7px;padding:6px 8px;color:var(--chu);font-size:14px;font-weight:700;font-variant-numeric:tabular-nums;outline:none}
.nhomNhap input:focus{border-color:#4493f8}
.nutTinh{background:#212833;border:1px solid var(--vien);color:var(--chu);border-radius:7px;width:28px;height:32px;font-weight:700;font-size:15px;cursor:pointer;flex-shrink:0}
.nutDatPid{background:var(--xanhDuong);border:none;color:#fff;border-radius:7px;padding:0 11px;height:32px;font-weight:700;font-size:12px;cursor:pointer;flex-shrink:0}
.dongPhu{font-size:12px;color:var(--mo);line-height:1.45;word-break:break-word}
.thongBaoNoi{position:fixed;top:14px;left:50%;transform:translateX(-50%);background:rgba(13,17,23,.96);color:#fff;padding:10px 16px;border-radius:11px;font-size:13px;font-weight:700;z-index:9999;pointer-events:none;border:1px solid #445368;box-shadow:0 8px 24px rgba(0,0,0,.65);opacity:0;transition:opacity .2s;max-width:90vw;text-align:center}
.lopPhuCuaSoCon{display:none;position:fixed;inset:0;background:rgba(0,0,0,.78);z-index:9000;align-items:flex-end;justify-content:center;padding:0}
.lopPhuCuaSoCon.dangHienCuaSo{display:flex}
.hopCuaSoCon{width:100%;max-width:580px;max-height:92vh;overflow-y:auto;background:var(--the);border:1px solid #384454;border-radius:18px 18px 0 0;padding:12px;display:flex;flex-direction:column;gap:10px;box-shadow:0 16px 40px rgba(0,0,0,.75);touch-action:pan-y}
.hopCuaSoCon.cuaSoRong{max-width:740px}
.hangHudLai{display:grid;grid-template-columns:auto 1fr 1fr auto;gap:6px;align-items:center;background:var(--o);border:1px solid var(--vien);border-radius:11px;padding:6px 8px;margin-bottom:8px}
.dauCuaSoCon{display:flex;justify-content:space-between;align-items:center;gap:8px}
.thanhTabCuaSo{display:grid;grid-template-columns:repeat(4,1fr);gap:5px;background:var(--o);padding:4px;border-radius:11px;border:1px solid var(--vien)}
.thanhTabCuaSo .nutNho{min-height:36px;border-color:transparent;background:transparent}
.thanhTabCuaSo .nutNho.nutXanhDuong{background:var(--xanhDuong);border-color:#4493f8}
.nhomMucPhu{background:var(--o);border:1px solid var(--vien);border-radius:12px;padding:10px;display:flex;flex-direction:column;gap:8px}
.nhanNhomPhu{font-size:11.5px;font-weight:700;color:var(--mo);letter-spacing:.2px}
.hangTrimGoc{display:grid;grid-template-columns:1fr auto 1fr;gap:8px;align-items:center}
.hienThiTrim{background:var(--the);border:1px solid var(--vien);border-radius:10px;padding:8px 12px;text-align:center}
@media (min-width:640px){
  .lopPhuCuaSoCon{align-items:center;padding:14px}
  .hopCuaSoCon{border-radius:18px}
  .luoiPid{grid-template-columns:repeat(2,1fr)}
  .luoiChiSo{grid-template-columns:repeat(4,1fr)}
  .hangNutDongCo{grid-template-columns:repeat(4,1fr)}
  .goiYBanPhim{display:block}
}
@media (min-width:860px){
  body{padding:14px}
  .luoiNoiDung{display:grid;grid-template-columns:1.02fr 1.18fr;align-items:start}
  .hangNutDongCo{grid-template-columns:repeat(2,1fr)}
}
@media (orientation:landscape) and (max-height:540px){
  body{padding:6px}
  .khungChinh,.luoiNoiDung{gap:7px}
  .luoiNoiDung{display:grid;grid-template-columns:0.95fr 1.25fr;align-items:start}
  .the{padding:9px}
  .vungCanGatDoc{height:152px;width:104px}
  .khungRanhNgang{height:152px}
  .vungCanGatNgang{height:104px}
  .numCanGatTienLui,.numCanGatReNgang{width:54px;height:54px;margin:-27px 0 0 -27px}
  .hangChiSoGoc{grid-template-columns:repeat(2,1fr);gap:4px}
}
</style>
</head>
<body>
<div id="thongBaoNoi" class="thongBaoNoi"></div>
<div class="khungChinh">
  <div class="luoiNoiDung">
    <div class="cotTrangThai">
      <div class="the">
        <div class="tieuDe">
          <span class="tenXe">
            <span class="bieuTuongXe" aria-hidden="true">
              <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><circle cx="7" cy="18" r="3"/><circle cx="17" cy="18" r="3"/><path d="M12 18V5m-4 4 4-4 4 4"/></svg>
            </span>
            <span>XE CÂN BẰNG ESP32-S3</span>
          </span>
          <span id="nhanTrangThai" class="nhan nhanTat">TẮT MOTOR</span>
        </div>
        <div class="thanhTrangThaiDinh">
          <div class="mucTrangThai"><span id="chamKetNoi" class="chamKetNoi"></span><span id="nhanKetNoiMang">Đang kết nối...</span></div>
          <div class="mucTrangThai">Trễ: <span id="oDoTreMang">-</span></div>
          <div class="mucTrangThai">Chế độ: <span id="oCheDo">PID Thuần (Bánh)</span></div>
          <span id="nhanKenhNhiPhan" class="nhan nhanTat">HTTP 8B</span>
        </div>
        <div id="khungHuongDanAnToan" class="khungHuongDanAnToan">
          <span class="chuHuongDan"><span class="chamHuongDan"></span><span id="chuHuongDanAnToan">Đang đọc trạng thái cảm biến và động cơ...</span></span>
          <button id="nutXoaLoiNhanh" class="nutNho nutVang" style="display:none" onclick="guiPhim('k','Đã gửi xóa lỗi')">Xóa lỗi</button>
        </div>
        <div class="khungThuocGoc">
          <div class="hangChiSoGoc">
            <div class="oGocNho"><div class="tenSo">Góc nghiêng</div><div id="oGocNghien" class="giaTriGocChinh">0.00°</div></div>
            <div class="oGocNho"><div class="tenSo">Góc đặt</div><div id="oGocDat" class="giaTriSo">0.00°</div></div>
            <div class="oGocNho"><div class="tenSo">Cân bằng</div><div id="oGocCanBang" class="giaTriSo">0.00°</div></div>
            <div class="oGocNho"><div class="tenSo">Tốc độ</div><div id="oTocDoGoc" class="giaTriSo">0.00</div></div>
          </div>
          <div id="thuocGocNghieng" class="thuocGocNghieng">
            <div id="vungAnToanKhoiDong" class="vungAnToanKhoiDong"></div>
            <div class="vachTamThuoc"></div>
            <div id="vachGocDat" class="vachGocDat"></div>
            <div id="kimGocNghien" class="kimGocNghien"></div>
          </div>
          <div class="chuThichThuoc"><span>-30° Ngả sau</span><span>Vùng bật ±5.7°</span><span>Ngả trước +30°</span></div>
        </div>
        <div class="luoiChiSo">
          <div class="oSo"><div class="tenSo">Vị trí giữ</div><div id="oViTri" class="giaTriSo">0.000 m</div></div>
          <div class="oSo"><div class="tenSo">Vận tốc</div><div id="oVanToc" class="giaTriSo">0.000 m/s</div></div>
          <div class="oSo"><div class="tenSo">An toàn</div><div id="oLoi" class="giaTriSo">Bình thường</div></div>
          <div class="oSo"><div class="tenSo">Gói lệnh</div><div id="oTongGoiNhiPhan" class="giaTriSo">0</div></div>
        </div>
        <details class="hopChanDoan">
          <summary>Chi tiết kỹ thuật &amp; Chẩn đoán giao tiếp ▾</summary>
          <div class="dongPhu" id="thongTinPhu">Đang kết nối tới xe...</div>
          <div class="dongPhu" id="khungHexNhiPhan" style="margin-top:4px">Khung 8B: A5 01 00 00 00 00 00 A4 | Đã gửi: 0</div>
        </details>
      </div>
    </div>

    <div class="cotDieuKhien">
      <div class="the">
        <div class="hangNutDongCo">
          <button id="nutBatCanBang" class="nut nutXanh" onclick="batCanBangBanhXe()"><span>BẬT CÂN BẰNG</span><span class="nutPhuNho">Phím (a)</span></button>
          <button id="nutTatMotor" class="nut" onclick="guiPhimThaoTac('d','Đã tắt motor')"><span>TẮT MOTOR</span><span class="nutPhuNho">Đỡ tay trước (d)</span></button>
          <button id="nutDungKhan" class="nut nutDo" onclick="guiPhimThaoTac('x','ĐÃ DỪNG KHẨN CẤP')"><span>DỪNG KHẨN</span><span class="nutPhuNho">Ngắt tức thì (x)</span></button>
          <button id="nutXoaLoi" class="nut" onclick="guiPhimThaoTac('k','Đã gửi xóa lỗi')"><span>XÓA LỖI</span><span class="nutPhuNho">Sau khi tắt (k)</span></button>
        </div>

        <button id="nutMoDieuKhien" class="nut nutXanhDuong" style="width:100%;min-height:54px;margin-bottom:9px;font-size:14.5px" onclick="moCuaSoCon('cuaSoDieuKhien')"><span>MỞ CẦN ĐIỀU KHIỂN LÁI XE</span><span class="nutPhuNho">2 cần gạt Tiến/Lùi · Rẽ · Tốc độ · Phanh</span></button>

        <div id="khungLogRung" style="display:none;margin-top:9px">
          <button id="nutLogRung" class="nut" style="width:100%" onclick="doiLogRung()">BẬT LOG RUNG ESP-NOW</button>
          <div id="trangThaiLogRung" class="dongPhu" style="margin-top:5px">Đang đọc trạng thái log...</div>
        </div>

        <div class="hangBaNut" style="margin-top:9px">
          <button class="nut nutXanhDuong" onclick="moCuaSoCon('cuaSoCalib')"><span>Chế độ &amp; Calib</span><span class="nutPhuNho">Cài đặt xe</span></button>
          <button class="nut nutXanhDuong" onclick="moCuaSoCon('cuaSoPid')"><span>Tinh chỉnh PID</span><span class="nutPhuNho">Kp / Ki / Kd</span></button>
          <button class="nut nutXanhDuong" onclick="moCuaSoCon('cuaSoTerminal')"><span>Terminal Serial</span><span class="nutPhuNho">Nhật ký &amp; Lệnh</span></button>
        </div>
      </div>
    </div>
  </div>
</div>

<div id="lopPhuCuaSoCon" class="lopPhuCuaSoCon" onclick="if(event.target===this)dongCuaSoCon()">
  <div id="hopCuaSoCon" class="hopCuaSoCon">
    <div class="dauCuaSoCon">
      <span id="tieuDeCuaSoCon" style="font-weight:700;font-size:15px">CẦN ĐIỀU KHIỂN LÁI XE</span>
      <button class="nutNho nutDo" onclick="dongCuaSoCon()">Đóng (Esc)</button>
    </div>
    <div class="thanhTabCuaSo">
      <button id="nutTabDieuKhien" class="nutNho nutXanhDuong" onclick="moCuaSoCon('cuaSoDieuKhien')">Lái xe</button>
      <button id="nutTabCalib" class="nutNho" onclick="moCuaSoCon('cuaSoCalib')">Chế độ &amp; Calib</button>
      <button id="nutTabPid" class="nutNho" onclick="moCuaSoCon('cuaSoPid')">Tinh chỉnh PID</button>
      <button id="nutTabTerminal" class="nutNho" onclick="moCuaSoCon('cuaSoTerminal')">Terminal</button>
    </div>

    <div id="cuaSoDieuKhien" style="display:none">
      <div class="hangHudLai">
        <span id="nhanTrangThaiLai" class="nhan nhanTat">TẮT MOTOR</span>
        <div class="oGocNho"><div class="tenSo">Nghiêng</div><div id="oGocNghienLai" class="giaTriSo">0.00°</div></div>
        <div class="oGocNho"><div class="tenSo">Vận tốc</div><div id="oVanTocLai" class="giaTriSo">0.00 m/s</div></div>
        <button class="nutNho nutDo" onclick="guiPhimThaoTac('x','ĐÃ DỪNG KHẨN CẤP')">DỪNG KHẨN</button>
      </div>
      <div class="khungTocDo">
        <div class="tieuDe" style="margin-bottom:2px">
          <span>GIỚI HẠN TỐC ĐỘ DI CHUYỂN</span>
          <span id="oPhanTramTocDo" style="font-size:13px;color:var(--xanhDuongSang)">100%</span>
        </div>
        <div class="hangPresetTocDo">
          <button id="nutTocDo35" class="nutNho" onclick="datMucTocDoNhanh(35)">Chậm 35%</button>
          <button id="nutTocDo65" class="nutNho" onclick="datMucTocDoNhanh(65)">Vừa 65%</button>
          <button id="nutTocDo100" class="nutNho nutXanhDuong" onclick="datMucTocDoNhanh(100)">Nhanh 100%</button>
          <input id="thanhTocDo" class="thanhTruot" type="range" min="25" max="100" step="5" value="100">
        </div>
      </div>
      <div class="khuHaiCanGat">
        <div class="hopCanGat">
          <div class="tieuDeCanGat">
            <span>1. TIẾN / LÙI</span>
            <span id="nhanTocDoTienLui" class="giaTriTruc">0% | 0.00 m/s</span>
          </div>
          <div id="vungCanGatTienLui" class="vungCanGatDoc">
            <div class="vachTamNgang"></div>
            <div class="nhanTrucDinh">▲ TIẾN</div>
            <div class="nhanTrucDay">▼ LÙI</div>
            <div id="numCanGatTienLui" class="numCanGatTienLui"></div>
          </div>
        </div>
        <div class="hopCanGat">
          <div class="tieuDeCanGat">
            <span>2. RẼ TRÁI / PHẢI</span>
            <span id="nhanTocDoReNgang" class="giaTriTruc" style="color:var(--xanhSang)">0% | 0.00 rad/s</span>
          </div>
          <div class="khungRanhNgang">
            <div id="vungCanGatReNgang" class="vungCanGatNgang">
              <div class="vachTamDoc"></div>
              <div class="nhanTrucTrai">◄ TRÁI</div>
              <div class="nhanTrucPhai">PHẢI ►</div>
              <div id="numCanGatReNgang" class="numCanGatReNgang"></div>
            </div>
          </div>
        </div>
      </div>
      <div class="goiYBanPhim">Phím tắt PC: W/S hoặc ↑/↓ (Tiến/Lùi) · A/D hoặc ←/→ (Rẽ) · Space (Phanh) · Esc (Đóng)</div>
      <button id="nutPhanhLai" class="nut nutVang" style="width:100%;margin-top:8px" onclick="phanhGiuViTri()">PHANH &amp; GIỮ VỊ TRÍ ĐỨNG YÊN</button>
    </div>

    <div id="cuaSoCalib" style="display:none">
      <div id="khungCalib2Truc" class="nhomMucPhu" style="display:none;margin-bottom:9px">
        <div class="tieuDe" style="margin-bottom:2px">
          <span class="nhanNhomPhu" style="margin:0">HIỆU CHUẨN 3 TƯ THẾ LẮP IMU (ĐỨNG / TRƯỚC 90° / SAU 90°)</span>
          <span id="nhanPhaCalib2" class="nhan nhanTat">Chưa chạy</span>
        </div>
        <div id="huongDanCalib2" class="dongPhu">Nhấn Bắt đầu hoặc dùng nút S3/BOOT để hiệu chuẩn 3 tư thế cảm biến.</div>
        <div class="hangBaNut">
          <button id="nutBatDauCalib2" class="nut nutXanhDuong" onclick="thaoTacKhiTatMotor('dong','cal3 start','Bắt đầu calib 3 tư thế')"><span>Bắt đầu</span><span class="nutPhuNho">3 bước nhanh</span></button>
          <button id="nutXacNhanCalib2" class="nut nutXanh" onclick="guiDong('cal3 next','Đã xác nhận bước calib')"><span>Xác nhận</span><span class="nutPhuNho">Web / S3 / BOOT</span></button>
          <button id="nutHuyCalib2" class="nut nutDo" onclick="guiDong('cal3 cancel','Đã hủy calib 3 tư thế')"><span>Hủy</span><span class="nutPhuNho">Thoát</span></button>
        </div>
      </div>
      <div class="nhomMucPhu" style="margin-bottom:9px">
        <div class="nhanNhomPhu">BỘ ĐIỀU KHIỂN CÂN BẰNG (CHUYỂN ĐỔI ÊM)</div>
        <div class="hangBaNut">
          <button id="nutChonPidThuan" class="nut nutXanhDuong" onclick="guiPhim('p','Đã chọn PID Thuần')"><span>PID Thuần</span><span class="nutPhuNho">Hệ số cố định</span></button>
          <button id="nutChonPidMo" class="nut" onclick="guiPhim('m','Đã chọn PID Mờ Fuzzy')"><span>PID Mờ</span><span class="nutPhuNho">25 luật thích nghi</span></button>
          <button class="nut" onclick="guiPhim('f','Đã đảo chế độ PID/Mờ')"><span>Đảo nhanh</span><span class="nutPhuNho">Phím (f)</span></button>
        </div>
      </div>
      <div class="nhomMucPhu" style="margin-bottom:9px">
        <div class="nhanNhomPhu">TINH CHỈNH GÓC CÂN BẰNG &amp; HIỆU CHUẨN</div>
        <div id="nhacKhoaCalib" class="dongPhu" style="display:none;color:var(--vangSang)">Motor đang bật: hãy Tắt motor trước khi Calib 2s, Zero nhanh hoặc Đảo chiều.</div>
        <div class="hangTrimGoc">
          <button class="nut" onclick="guiPhim('[','Giảm góc cân bằng -0.1°')"><span>-0.1°</span><span class="nutPhuNho">Ngả sau ([)</span></button>
          <div class="hienThiTrim"><div class="tenSo">Góc cân bằng</div><div id="oGocCanBangCalib" class="giaTriSo">0.00°</div></div>
          <button class="nut" onclick="guiPhim(']','Tăng góc cân bằng +0.1°')"><span>+0.1°</span><span class="nutPhuNho">Ngả trước (])</span></button>
        </div>
        <div class="hangBaNut">
          <button id="nutCalib2s" class="nut" onclick="thaoTacKhiTatMotor('phim','c','Giữ xe đứng yên 2 giây để calib...')"><span>Calib 2s (c)</span><span class="nutPhuNho">Đo bias + góc</span></button>
          <button id="nutZeroNhanh" class="nut" onclick="thaoTacKhiTatMotor('phim','o','Đã đặt mốc 0 góc &amp; vị trí')"><span>Zero nhanh (o)</span><span class="nutPhuNho">Đặt gốc tại chỗ</span></button>
          <button class="nut nutXanhDuong" onclick="guiPhim('g','Đã chốt vị trí đứng tại chỗ')"><span>Chốt vị trí (g)</span><span class="nutPhuNho">Giữ yên tại đây</span></button>
        </div>
        <button class="nut nutXanh" style="width:100%" onclick="guiDong('save','Đã lưu cấu hình vào bộ nhớ Flash')">LƯU CẤU HÌNH VÀO FLASH (save)</button>
      </div>
      <div class="nhomMucPhu">
        <div class="nhanNhomPhu">TÙY CHỌN THỬ NGHIỆM &amp; PHẦN CỨNG</div>
        <div class="hangBaNut">
          <button id="nutVongViTri" class="nut" onclick="guiPhim('v','Đã bật/tắt vòng vị trí')">Vòng vị trí (v)</button>
          <button id="nutBenchMode" class="nut" onclick="guiPhim('b','Đã bật/tắt chế độ giá đỡ')">Giá đỡ (b)</button>
          <button id="nutDaoChieuDongCo" class="nut" onclick="thaoTacKhiTatMotor('dong','daochieu','Đã đảo chiều động cơ')">Đảo chiều</button>
        </div>
      </div>
    </div>

    <div id="cuaSoPid" style="display:none">
      <div class="tieuDe" style="margin-bottom:6px">
        <span id="tieuDePid">TINH CHỈNH: PID_THUAN</span>
        <div style="display:flex;gap:6px">
          <button id="nutPidThuanNhanh" class="nutNho nutXanhDuong" onclick="guiPhim('p','Đã chọn PID Thuần')">PID Thuần</button>
          <button id="nutPidMoNhanh" class="nutNho" onclick="guiPhim('m','Đã chọn PID Mờ')">PID Mờ</button>
        </div>
      </div>
      <div id="pidHieuDungTomTat" class="dongPhu" style="margin-bottom:8px;color:var(--xanhDuongSang);font-weight:600">Hệ số hiệu dụng: Kp=0.00 | Ki=0.00 | Kd=0.00</div>
      <div class="nhomMucPhu" style="margin-bottom:8px">
        <div class="nhanNhomPhu">VÒNG TRONG — GIỮ GÓC NGHIÊNG (200 Hz)</div>
        <div class="luoiPid">
          <div class="nhomNhap"><label>Kp góc</label><button class="nutTinh" onclick="buocThongSo('kp','nhapKp',-0.5,2)">-</button><input id="nhapKp" type="number" step="0.5" onkeydown="nhanEnterThongSo(event,'kp','nhapKp')"><button class="nutTinh" onclick="buocThongSo('kp','nhapKp',0.5,2)">+</button><button class="nutDatPid" onclick="guiThongSo('kp','nhapKp')">Đặt</button></div>
          <div class="nhomNhap"><label>Ki góc</label><button class="nutTinh" onclick="buocThongSo('ki','nhapKi',-0.1,2)">-</button><input id="nhapKi" type="number" step="0.1" onkeydown="nhanEnterThongSo(event,'ki','nhapKi')"><button class="nutTinh" onclick="buocThongSo('ki','nhapKi',0.1,2)">+</button><button class="nutDatPid" onclick="guiThongSo('ki','nhapKi')">Đặt</button></div>
          <div class="nhomNhap"><label>Kd góc</label><button class="nutTinh" onclick="buocThongSo('kd','nhapKd',-0.1,2)">-</button><input id="nhapKd" type="number" step="0.1" onkeydown="nhanEnterThongSo(event,'kd','nhapKd')"><button class="nutTinh" onclick="buocThongSo('kd','nhapKd',0.1,2)">+</button><button class="nutDatPid" onclick="guiThongSo('kd','nhapKd')">Đặt</button></div>
          <div class="nhomNhap"><label>Ref (°)</label><button class="nutTinh" onclick="buocThongSo('refdeg','nhapRefDo',-0.1,2)">-</button><input id="nhapRefDo" type="number" step="0.05" onkeydown="nhanEnterThongSo(event,'refdeg','nhapRefDo')"><button class="nutTinh" onclick="buocThongSo('refdeg','nhapRefDo',0.1,2)">+</button><button class="nutDatPid" onclick="guiThongSo('refdeg','nhapRefDo')">Đặt</button></div>
        </div>
      </div>
      <div class="nhomMucPhu" style="margin-bottom:8px">
        <div class="nhanNhomPhu">VÒNG NGOÀI — VẬN TỐC &amp; GIỮ VỊ TRÍ (50 Hz)</div>
        <div class="luoiPid">
          <div class="nhomNhap"><label>Kpv</label><button class="nutTinh" onclick="buocThongSo('kpv','nhapKpv',-0.01,3)">-</button><input id="nhapKpv" type="number" step="0.01" onkeydown="nhanEnterThongSo(event,'kpv','nhapKpv')"><button class="nutTinh" onclick="buocThongSo('kpv','nhapKpv',0.01,3)">+</button><button class="nutDatPid" onclick="guiThongSo('kpv','nhapKpv')">Đặt</button></div>
          <div class="nhomNhap"><label>Kiv</label><button class="nutTinh" onclick="buocThongSo('kiv','nhapKiv',-0.01,3)">-</button><input id="nhapKiv" type="number" step="0.01" onkeydown="nhanEnterThongSo(event,'kiv','nhapKiv')"><button class="nutTinh" onclick="buocThongSo('kiv','nhapKiv',0.01,3)">+</button><button class="nutDatPid" onclick="guiThongSo('kiv','nhapKiv')">Đặt</button></div>
        </div>
      </div>
      <div class="hangBaNut" style="margin-bottom:8px">
        <button class="nut nutXanh" onclick="guiDong('save','Đã lưu Flash NVS')">Lưu Flash</button>
        <button class="nut" onclick="napLaiThongSoTuXe()">Đọc lại xe</button>
        <button id="nutKhoiPhucGoc" class="nut nutDo" onclick="khoiPhucMacDinhAnToan()">Mặc định</button>
      </div>
      <div class="dongPhu" id="thongTinMang">ESP-NOW song song &amp; WiFi AP</div>
    </div>

    <div id="cuaSoTerminal" style="display:none">
      <div class="khungTerminalSerial">
        <div class="tieuDe" style="margin-bottom:2px">
          <span>TERMINAL SERIAL TRỰC TIẾP</span>
          <div style="display:flex;gap:5px;flex-wrap:wrap">
            <button id="nutLuongTelemetryWeb" class="nutNho" onclick="doiTelemetryWeb()">Telemetry (tw)</button>
            <button id="nutTuCuonTerminal" class="nutNho nutXanhDuong" onclick="doiTuCuonTerminal()">Tự cuộn: BẬT</button>
            <button class="nutNho" onclick="xoaManHinhTerminal()">Xóa</button>
          </div>
        </div>
        <div class="hangNutTerminal">
          <button class="nutNho" onclick="guiLenhChuoiTerminal('status')">status</button>
          <button class="nutNho" onclick="guiLenhChuoiTerminal('h')">help</button>
          <button class="nutNho" onclick="guiLenhChuoiTerminal('c')">calib (c)</button>
          <button class="nutNho" onclick="guiLenhChuoiTerminal('o')">zero (o)</button>
          <button class="nutNho" onclick="guiLenhChuoiTerminal('p')">pid (p)</button>
          <button class="nutNho" onclick="guiLenhChuoiTerminal('m')">fuzzy (m)</button>
          <button class="nutNho" onclick="guiLenhChuoiTerminal('save')">save</button>
        </div>
        <div id="khungXuatSerial" class="khungXuatSerial"></div>
        <div class="hangNhapSerial">
          <input id="oNhapLenhSerial" class="oNhapLenhSerial" type="text" autocomplete="off" autocapitalize="off" spellcheck="false"
            placeholder="Nhập lệnh Serial (status, h, c, o, a, d, kp 33, refdeg 0.5, save...)">
          <button class="nut nutXanhDuong" style="padding:8px 14px;min-height:40px" onclick="guiLenhTerminal()">GỬI</button>
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
let tyLeJoystickTienLui = 0;
let tyLeJoystickReNgang = 0;
let logRungSupported = false;
let logRungEnabled = false;
let doiLogRungDangCho = false;
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
let dangDocTrangThai = false;
let boHenThongBao = null;
let dangBatBench = false;
let motorDangChay = false;
let cuaSoDangMo = '';
let terminalDangMo = false;
let tuCuonTerminal = true;
let telemetryWebDangBat = false;
let chiSoNhatKyCuoi = 0;
let dangDocNhatKy = false;
let boHenDocNhatKy = null;
let lichSuLenhSerial = [];
let viTriLichSuSerial = -1;
const phimDangGiu = {};

const bangMoTaLoi = {
  none: 'Bình thường',
  fallen: 'Nghiêng quá giới hạn ngã',
  imu_invalid: 'Cảm biến IMU chưa hợp lệ',
  imu_stale: 'Dữ liệu IMU bị trễ',
  pulse_invalid: 'Lỗi đọc xung bánh xe',
  pulse_stale: 'Phản hồi bánh xe quá hạn',
  control_invalid: 'Lệnh điều khiển không hợp lệ',
  motor_fault: 'Lỗi driver động cơ',
  emergency: 'Đã dừng khẩn cấp',
  arm_timeout: 'Quá hạn chờ bật motor',
  need_mount_calib: 'Cần hiệu chuẩn cảm biến'
};
const huongDanPhaCalib2 = [
  'Chưa chạy — Nhấn Bắt đầu để hiệu chuẩn 3 tư thế (Đứng → Trước 90° → Sau 90°).',
  'Bước 1/3 (Đứng thẳng): Dựng xe thẳng đứng, giữ yên rồi bấm Xác nhận (hoặc nút S3 / BOOT).',
  'Đang đo bias gyro & mốc đứng... Giữ xe thật yên.',
  'Bước 2/3 (Nghiêng 90° về trước): Hạ ĐẦU xe nằm về trước (~80-90°), giữ yên rồi bấm Xác nhận.',
  'Đang đo mốc nghiêng trước 90°... Giữ thật yên.',
  'Bước 3/3 (Nghiêng 90° về sau): Nâng qua đứng thẳng sang hạ ĐUÔI nằm về sau (~80-90°, chậm 4-8s), giữ yên rồi bấm Xác nhận.',
  'Đang đo mốc nghiêng sau 90° & tính ma trận lắp 3D...',
  'THÀNH CÔNG: Đã tính và lưu ma trận lắp vào Flash NVS! Motor đã mở khóa.',
  'THẤT BẠI — Nhấn Bắt đầu để thử lại.'
];

function moTaMaLoi(ma) {
  if (!ma) return 'Không xác định';
  return bangMoTaLoi[ma] || ('Mã lỗi: ' + ma);
}

function rungPhanHoi(ms) {
  if (typeof navigator !== 'undefined' && typeof navigator.vibrate === 'function') {
    try { navigator.vibrate(ms || 12); } catch (e) {}
  }
}

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

function capNhatNutPresetTocDo(phanTram) {
  const cacMuc = [[35, 'nutTocDo35'], [65, 'nutTocDo65'], [100, 'nutTocDo100']];
  for (let i = 0; i < cacMuc.length; i++) {
    const nut = document.getElementById(cacMuc[i][1]);
    if (nut) nut.className = (Math.round(phanTram) === cacMuc[i][0]) ? 'nutNho nutXanhDuong' : 'nutNho';
  }
}

function datMucTocDoNhanh(phanTram) {
  const giaTri = Math.max(25, Math.min(100, Math.round(phanTram)));
  thanhTocDo.value = String(giaTri);
  heSoTocDoDiaHinh = giaTri / 100;
  oPhanTramTocDo.textContent = giaTri + '%';
  capNhatNutPresetTocDo(giaTri);
  tinhLaiVanTocJoystick();
  if (maConTroTienLui !== null || maConTroReNgang !== null || Math.abs(tyLeJoystickTienLui) > 0 || Math.abs(tyLeJoystickReNgang) > 0) {
    guiKhungNhiPhan(true, 0x01, 0);
  }
  rungPhanHoi(8);
}

thanhTocDo.addEventListener('input', () => {
  const giaTri = Number(thanhTocDo.value);
  heSoTocDoDiaHinh = giaTri / 100;
  oPhanTramTocDo.textContent = thanhTocDo.value + '%';
  capNhatNutPresetTocDo(giaTri);
  tinhLaiVanTocJoystick();
  if (maConTroTienLui !== null || maConTroReNgang !== null) guiKhungNhiPhan(true, 0x01, 0);
});

function hienThongBao(thongDiep) {
  const hopThongBao = document.getElementById('thongBaoNoi');
  hopThongBao.textContent = thongDiep;
  hopThongBao.style.opacity = '1';
  clearTimeout(boHenThongBao);
  boHenThongBao = setTimeout(() => { hopThongBao.style.opacity = '0'; }, 2000);
}

function capNhatTrangThaiKetNoi(daKetNoi, doTreMs) {
  const cham = document.getElementById('chamKetNoi');
  const nhan = document.getElementById('nhanKetNoiMang');
  const oTre = document.getElementById('oDoTreMang');
  if (!daKetNoi) {
    cham.className = 'chamKetNoi matKetNoi';
    nhan.textContent = 'Mất kết nối';
    nhan.style.color = '#ff7b72';
    const khungHd = document.getElementById('khungHuongDanAnToan');
    const chuHd = document.getElementById('chuHuongDanAnToan');
    khungHd.className = 'khungHuongDanAnToan huongDanDo';
    chuHd.textContent = 'Mất kết nối tới xe — Kiểm tra Wi-Fi AP của xe (192.168.4.1).';
    document.getElementById('nutXoaLoiNhanh').style.display = 'none';
    return;
  }
  if (typeof doTreMs === 'number') {
    lichSuDoTre[viTriDoTre] = doTreMs;
    viTriDoTre = (viTriDoTre + 1) % lichSuDoTre.length;
    const trungBinh = Math.round(lichSuDoTre.reduce((a, b) => a + b, 0) / lichSuDoTre.length);
    oTre.textContent = trungBinh + ' ms';
    cham.className = trungBinh > 150 ? 'chamKetNoi ketNoiCham' : 'chamKetNoi ketNoiTot';
  } else {
    cham.className = 'chamKetNoi ketNoiTot';
  }
  nhan.textContent = 'Đã kết nối';
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

function tinhLaiVanTocJoystick() {
  vanTocHienTai = noiSuyTangTocKeo(tyLeJoystickTienLui) * vanTocToiDaXe * heSoTocDoDiaHinh;
  tocDoQuayHienTai = noiSuyTangTocKeo(tyLeJoystickReNgang) * tocDoQuayToiDaXe * heSoTocDoDiaHinh;
}

function dongGoiKhungNhiPhan(maLenh, vanToc, tocDoQuay, phimKemTheo) {
  const coMucWeb = logRungSupported && maLenh === 0x01;
  const boDem = new ArrayBuffer(coMucWeb ? 14 : 8);
  const gocNhin = new DataView(boDem);
  const mucTien = Math.round(Math.max(-1, Math.min(1, vanToc / vanTocToiDaXe)) * 1000);
  const mucRe = Math.round(Math.max(-1, Math.min(1, tocDoQuay / tocDoQuayToiDaXe)) * 1000);
  gocNhin.setUint8(0, 0xA5);
  gocNhin.setUint8(1, coMucWeb ? 0x03 : maLenh & 0xFF);
  gocNhin.setInt16(2, mucTien, true);
  gocNhin.setInt16(4, mucRe, true);
  gocNhin.setUint8(6, phimKemTheo & 0xFF);
  if (coMucWeb) {
    gocNhin.setUint8(7, 0);
    gocNhin.setInt16(8, Math.round(tyLeJoystickTienLui * 100), true);
    gocNhin.setInt16(10, Math.round(tyLeJoystickReNgang * 100), true);
    gocNhin.setUint8(12, Math.round(heSoTocDoDiaHinh * 100));
  }
  let maXor = 0;
  for (let i = 0; i < boDem.byteLength - 1; i++) maXor ^= gocNhin.getUint8(i);
  gocNhin.setUint8(boDem.byteLength - 1, maXor);
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
    nhanKenhNhiPhan.textContent = logRungSupported ? 'WS 14B' : 'WS 8B';
  } else {
    nhanKenhNhiPhan.className = 'nhan nhanTat';
    nhanKenhNhiPhan.textContent = logRungSupported ? 'HTTP 14B' : 'HTTP 8B';
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
    ketNoiWs.onerror = () => {
      capNhatTrangThaiKenh();
      try { ketNoiWs.close(); } catch (e) {}
    };
  } catch (e) {
    capNhatTrangThaiKenh();
  }
}

function guiKhungNhiPhan(batBuoc, maLenh, phimKemTheo) {
  const boDem = dongGoiKhungNhiPhan(maLenh, vanTocHienTai, tocDoQuayHienTai, phimKemTheo);
  nhanTocDoTienLui.textContent = Math.round(tyLeJoystickTienLui * 100) + '% | ' + vanTocHienTai.toFixed(2) + ' m/s';
  nhanTocDoReNgang.textContent = Math.round(tyLeJoystickReNgang * 100) + '% | ' + tocDoQuayHienTai.toFixed(2) + ' rad/s';
  const chuoiHienThi = dinhDangHexKhung(boDem, true);
  const thoiGian = Date.now();
  if (ketNoiWs && ketNoiWs.readyState === 1) {
    if (!batBuoc && thoiGian - mocGuiWsGanNhat < 40) return false;
    mocGuiWsGanNhat = thoiGian;
    ketNoiWs.send(boDem);
    demKhungDaGui++;
    khungHexNhiPhan.textContent = `Khung ${boDem.byteLength}B (WS): ${chuoiHienThi} | Đã gửi: ${demKhungDaGui}`;
    return true;
  }
  if (!batBuoc && thoiGian - mocGuiHttpGanNhat < 60) return false;
  mocGuiHttpGanNhat = thoiGian;
  demKhungDaGui++;
  khungHexNhiPhan.textContent = `Khung ${boDem.byteLength}B (HTTP): ${chuoiHienThi} | Đã gửi: ${demKhungDaGui}`;
  fetch('/nhiphan?g=' + dinhDangHexKhung(boDem, false)).catch(() => {});
  return true;
}

function batDauGuiLienTuc() {
  if (boDemGuiLienTuc) return;
  boDemGuiLienTuc = setInterval(() => {
    if ((logRungEnabled && !document.hidden) || maConTroTienLui !== null || maConTroReNgang !== null ||
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

function xoaPhimDangGiu() {
  for (const k in phimDangGiu) delete phimDangGiu[k];
}

function phanhGiuViTri(imLang) {
  xoaPhimDangGiu();
  maConTroTienLui = null;
  maConTroReNgang = null;
  vungCanGatTienLui.classList.remove('dangKeo');
  vungCanGatReNgang.classList.remove('dangKeo');
  tyLeJoystickTienLui = tyLeJoystickReNgang = 0;
  vanTocHienTai = 0;
  tocDoQuayHienTai = 0;
  datViTriNumTienLui(0);
  datViTriNumReNgang(0);
  if (!logRungEnabled || document.hidden) dungGuiLienTuc();
  guiKhungNhiPhan(true, 0x01, 0);
  if (!imLang) {
    rungPhanHoi(16);
    hienThongBao('Đã phanh & giữ vị trí đứng yên');
  }
}

function capNhatJoystickTienLui(toaDoY) {
  const khung = vungCanGatTienLui.getBoundingClientRect();
  const tamY = khung.top + khung.height / 2;
  const hanhTrinhToiDa = Math.max(10, khung.height / 2 - 32);
  const lechY = Math.max(-hanhTrinhToiDa, Math.min(hanhTrinhToiDa, toaDoY - tamY));
  datViTriNumTienLui(lechY);
  const tyLeKeo = -lechY / hanhTrinhToiDa;
  tyLeJoystickTienLui = tyLeKeo;
  tinhLaiVanTocJoystick();
  batDauGuiLienTuc();
  guiKhungNhiPhan(false, 0x01, 0);
}

function capNhatJoystickReNgang(toaDoX) {
  const khung = vungCanGatReNgang.getBoundingClientRect();
  const tamX = khung.left + khung.width / 2;
  const hanhTrinhToiDa = Math.max(10, khung.width / 2 - 32);
  const lechX = Math.max(-hanhTrinhToiDa, Math.min(hanhTrinhToiDa, toaDoX - tamX));
  datViTriNumReNgang(lechX);
  const tyLeKeo = -lechX / hanhTrinhToiDa;
  tyLeJoystickReNgang = tyLeKeo;
  tinhLaiVanTocJoystick();
  batDauGuiLienTuc();
  guiKhungNhiPhan(false, 0x01, 0);
}

function nhaJoystickTienLui(suKien) {
  if (maConTroTienLui === null || suKien.pointerId !== maConTroTienLui) return;
  maConTroTienLui = null;
  vungCanGatTienLui.classList.remove('dangKeo');
  tyLeJoystickTienLui = 0;
  vanTocHienTai = 0;
  datViTriNumTienLui(0);
  if (maConTroReNgang === null) dungGuiLienTuc();
  guiKhungNhiPhan(true, 0x01, 0);
}

function nhaJoystickReNgang(suKien) {
  if (maConTroReNgang === null || suKien.pointerId !== maConTroReNgang) return;
  maConTroReNgang = null;
  vungCanGatReNgang.classList.remove('dangKeo');
  tyLeJoystickReNgang = 0;
  tocDoQuayHienTai = 0;
  datViTriNumReNgang(0);
  if (maConTroTienLui === null) dungGuiLienTuc();
  guiKhungNhiPhan(true, 0x01, 0);
}

function capNhatBanPhim() {
  const tien = (phimDangGiu.w || phimDangGiu.arrowup ? 1 : 0) - (phimDangGiu.s || phimDangGiu.arrowdown ? 1 : 0);
  const re = (phimDangGiu.a || phimDangGiu.arrowleft ? 1 : 0) - (phimDangGiu.d || phimDangGiu.arrowright ? 1 : 0);
  if (maConTroTienLui === null) {
    tyLeJoystickTienLui = tien;
    const hTien = Math.max(10, vungCanGatTienLui.getBoundingClientRect().height / 2 - 32);
    datViTriNumTienLui(-tien * hTien);
    if (tien !== 0) vungCanGatTienLui.classList.add('dangKeo');
    else vungCanGatTienLui.classList.remove('dangKeo');
  }
  if (maConTroReNgang === null) {
    tyLeJoystickReNgang = re;
    const hRe = Math.max(10, vungCanGatReNgang.getBoundingClientRect().width / 2 - 32);
    datViTriNumReNgang(-re * hRe);
    if (re !== 0) vungCanGatReNgang.classList.add('dangKeo');
    else vungCanGatReNgang.classList.remove('dangKeo');
  }
  tinhLaiVanTocJoystick();
  if (Math.abs(tyLeJoystickTienLui) > 0 || Math.abs(tyLeJoystickReNgang) > 0) batDauGuiLienTuc();
  else if (maConTroTienLui === null && maConTroReNgang === null) dungGuiLienTuc();
  guiKhungNhiPhan(true, 0x01, 0);
}

vungCanGatTienLui.addEventListener('pointerdown', (suKien) => {
  suKien.preventDefault();
  if (maConTroTienLui !== null) return;
  maConTroTienLui = suKien.pointerId;
  vungCanGatTienLui.classList.add('dangKeo');
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
  vungCanGatReNgang.classList.add('dangKeo');
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

window.addEventListener('blur', () => phanhGiuViTri());
document.addEventListener('visibilitychange', () => {
  if (document.hidden) phanhGiuViTri();
});
window.addEventListener('keydown', (suKien) => {
  const k = (suKien.key || '').toLowerCase();
  if (k === 'escape' && cuaSoDangMo) { dongCuaSoCon(); return; }
  const dich = suKien.target && suKien.target.tagName ? suKien.target.tagName : '';
  if (dich === 'INPUT' || dich === 'TEXTAREA' || dich === 'SELECT' ||
      (cuaSoDangMo !== '' && cuaSoDangMo !== 'cuaSoDieuKhien') ||
      suKien.ctrlKey || suKien.altKey || suKien.metaKey) return;
  if (k === ' ') {
    suKien.preventDefault();
    phanhGiuViTri();
    return;
  }
  if (k === 'w' || k === 's' || k === 'a' || k === 'd' ||
      k === 'arrowup' || k === 'arrowdown' || k === 'arrowleft' || k === 'arrowright') {
    suKien.preventDefault();
    if (phimDangGiu[k]) return;
    phimDangGiu[k] = true;
    capNhatBanPhim();
  }
});
window.addEventListener('keyup', (suKien) => {
  const k = (suKien.key || '').toLowerCase();
  if (phimDangGiu[k]) {
    delete phimDangGiu[k];
    capNhatBanPhim();
  }
});

function doiLogRung() {
  if (!logRungSupported || doiLogRungDangCho) return;
  doiLogRungDangCho = true;
  document.getElementById('nutLogRung').disabled = true;
  guiLenhChuoiTerminal(logRungEnabled ? 'viblog 0' : 'viblog 1');
  setTimeout(() => { doiLogRungDangCho = false; capNhatTrangThaiXe(); }, 1000);
}

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

function guiPhimThaoTac(kyTu, thongBao) {
  rungPhanHoi(kyTu === 'x' ? 25 : 12);
  guiPhim(kyTu, thongBao);
}

function guiDong(dongLenh, thongBao) {
  if (thongBao) hienThongBao(thongBao);
  fetch(`/lenh?dong=${encodeURIComponent(dongLenh)}`)
    .then(() => { capNhatTrangThaiXe(); docNhatKySerial(); })
    .catch(() => {});
}

function thaoTacKhiTatMotor(loai, thamSo, thongBao) {
  if (motorDangChay) {
    hienThongBao('Vui lòng tắt motor trước khi hiệu chuẩn / đảo chiều!');
    rungPhanHoi(20);
    return;
  }
  if (loai === 'phim') guiPhim(thamSo, thongBao);
  else guiDong(thamSo, thongBao);
}

function guiThongSo(tenLenh, idO) {
  const giaTri = document.getElementById(idO).value;
  if (giaTri === '') return;
  guiDong(`${tenLenh} ${giaTri}`, `Đã đặt ${tenLenh} = ${giaTri}`);
}

function buocThongSo(tenLenh, idO, delta, soLe) {
  const o = document.getElementById(idO);
  const cu = parseFloat(o.value);
  const coSo = isFinite(cu) ? cu : 0;
  let moi = coSo + delta;
  if (tenLenh !== 'refdeg') moi = Math.max(0, moi);
  else moi = Math.max(-20, Math.min(20, moi));
  o.value = moi.toFixed(soLe);
  guiThongSo(tenLenh, idO);
}

function nhanEnterThongSo(suKien, tenLenh, idO) {
  if (suKien.key === 'Enter') {
    suKien.preventDefault();
    guiThongSo(tenLenh, idO);
  }
}

function napLaiThongSoTuXe() {
  daDienThongSoBanDau = false;
  capNhatTrangThaiXe();
  hienThongBao('Đang đọc lại thông số từ xe...');
}

function khoiPhucMacDinhAnToan() {
  if (motorDangChay) {
    hienThongBao('Hãy tắt motor trước khi khôi phục mặc định!');
    return;
  }
  if (typeof confirm === 'function' && !confirm('Xóa toàn bộ cấu hình PID/Calib trong Flash và khôi phục mặc định?')) {
    return;
  }
  guiDong('resetcfg', 'Đã khôi phục thông số mặc định');
  daDienThongSoBanDau = false;
}

function batCanBangBanhXe() {
  rungPhanHoi(14);
  if (dangBatBench) {
    if (ketNoiWs && ketNoiWs.readyState === 1) {
      ketNoiWs.send('CMD:bench 0');
      hienThongBao('Tắt chế độ giá đỡ, bật cân bằng bánh xe');
      setTimeout(() => guiPhim('a', 'Đang bật cân bằng...'), 60);
    } else {
      fetch('/lenh?dong=' + encodeURIComponent('bench 0'))
        .then(() => guiPhim('a', 'Đang bật cân bằng...'))
        .catch(() => guiPhim('a', 'Đang bật cân bằng...'));
    }
    return;
  }
  guiPhim('a', 'Đang bật cân bằng bánh xe...');
}

function moCuaSoCon(idCuaSo) {
  if (cuaSoDangMo === 'cuaSoDieuKhien' && idCuaSo !== 'cuaSoDieuKhien') {
    phanhGiuViTri(true);
  }
  cuaSoDangMo = idCuaSo;
  document.getElementById('lopPhuCuaSoCon').classList.add('dangHienCuaSo');
  const hop = document.getElementById('hopCuaSoCon');
  if (idCuaSo === 'cuaSoDieuKhien') hop.classList.add('cuaSoRong');
  else hop.classList.remove('cuaSoRong');
  const danhSach = ['cuaSoDieuKhien', 'cuaSoCalib', 'cuaSoPid', 'cuaSoTerminal'];
  const tenTab = {
    cuaSoDieuKhien: 'CẦN ĐIỀU KHIỂN LÁI XE',
    cuaSoCalib: 'CHẾ ĐỘ & HIỆU CHUẨN',
    cuaSoPid: 'TINH CHỈNH PID',
    cuaSoTerminal: 'TERMINAL SERIAL'
  };
  const idTab = {
    cuaSoDieuKhien: 'nutTabDieuKhien',
    cuaSoCalib: 'nutTabCalib',
    cuaSoPid: 'nutTabPid',
    cuaSoTerminal: 'nutTabTerminal'
  };
  for (let i = 0; i < danhSach.length; i++) {
    const ma = danhSach[i];
    document.getElementById(ma).style.display = (ma === idCuaSo) ? 'block' : 'none';
    document.getElementById(idTab[ma]).className = (ma === idCuaSo) ? 'nutNho nutXanhDuong' : 'nutNho';
  }
  document.getElementById('tieuDeCuaSoCon').textContent = tenTab[idCuaSo] || 'CÀI ĐẶT';
  terminalDangMo = (idCuaSo === 'cuaSoTerminal');
  clearTimeout(boHenDocNhatKy);
  if (terminalDangMo) {
    docNhatKySerial();
  }
}

function dongCuaSoCon() {
  if (cuaSoDangMo === 'cuaSoDieuKhien') {
    phanhGiuViTri(true);
  }
  cuaSoDangMo = '';
  terminalDangMo = false;
  clearTimeout(boHenDocNhatKy);
  document.getElementById('hopCuaSoCon').classList.remove('cuaSoRong');
  document.getElementById('lopPhuCuaSoCon').classList.remove('dangHienCuaSo');
}

function doiTuCuonTerminal() {
  tuCuonTerminal = !tuCuonTerminal;
  const nut = document.getElementById('nutTuCuonTerminal');
  nut.textContent = tuCuonTerminal ? 'Tự cuộn: BẬT' : 'Tự cuộn: TẮT';
  nut.className = tuCuonTerminal ? 'nutNho nutXanhDuong' : 'nutNho';
}

function doiTelemetryWeb() {
  telemetryWebDangBat = !telemetryWebDangBat;
  const nut = document.getElementById('nutLuongTelemetryWeb');
  nut.className = telemetryWebDangBat ? 'nutNho nutXanhDuong' : 'nutNho';
  guiLenhChuoiTerminal('tw');
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
    const dong = String(danhSachDong[i]);
    oDong.textContent = dong;
    if (dong.indexOf('> ') === 0) oDong.className = 'dongLenh';
    else if (dong.indexOf('ERROR') >= 0 || dong.indexOf('FAIL') >= 0 || dong.indexOf('LOI') >= 0) oDong.className = 'dongLoi';
    else if (dong.indexOf('WARN') >= 0) oDong.className = 'dongCanhBao';
    else if (dong.indexOf('PASS') >= 0 || dong.indexOf('READY') >= 0 || dong.indexOf('SAVED') >= 0) oDong.className = 'dongTot';
    manh.appendChild(oDong);
  }
  khungXuatSerial.appendChild(manh);
  while (khungXuatSerial.children.length > soDongDomToiDa) {
    khungXuatSerial.removeChild(khungXuatSerial.firstChild);
  }
  if (tuCuonTerminal) khungXuatSerial.scrollTop = khungXuatSerial.scrollHeight;
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
  hienThongBao(`Lệnh: ${lenh}`);
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

function apDungTrangThaiLenUi(duLieu) {
  const coLoi = Boolean(duLieu.tenLoi && duLieu.tenLoi !== 'none');
  const nhanTT = document.getElementById('nhanTrangThai');
  motorDangChay = Boolean(duLieu.daBat);
  dangBatBench = Boolean(duLieu.cheDoNangBanh);

  if (coLoi) {
    nhanTT.className = 'nhan nhanLoi';
    nhanTT.textContent = 'LỖI: ' + moTaMaLoi(duLieu.tenLoi);
  } else if (motorDangChay) {
    nhanTT.className = 'nhan nhanBat';
    nhanTT.textContent = dangBatBench ? 'CHẾ ĐỘ GIÁ ĐỠ' : 'ĐANG CÂN BẰNG';
  } else {
    nhanTT.className = 'nhan nhanTat';
    nhanTT.textContent = 'TẮT MOTOR';
  }

  const lechGoc = duLieu.gocNghienDo - duLieu.gocCanBangDo;
  const duDieuKienBat = Boolean(duLieu.gocHopLe && duLieu.heThongSanSang && Math.abs(lechGoc) <= 5.7);
  const khungHd = document.getElementById('khungHuongDanAnToan');
  const chuHd = document.getElementById('chuHuongDanAnToan');
  const nutXoaNhanh = document.getElementById('nutXoaLoiNhanh');
  nutXoaNhanh.style.display = coLoi ? 'inline-flex' : 'none';

  if (coLoi) {
    khungHd.className = 'khungHuongDanAnToan huongDanDo';
    chuHd.textContent = 'Cảnh báo: ' + moTaMaLoi(duLieu.tenLoi) + (motorDangChay ? '' : ' — Nhấn Xóa lỗi để tiếp tục.');
  } else if (motorDangChay) {
    if (dangBatBench) {
      khungHd.className = 'khungHuongDanAnToan huongDanVang';
      chuHd.textContent = 'Đang chạy Chế độ Giá đỡ (nhấc bánh) — Tắt chế độ này trước khi đặt xe xuống sàn.';
    } else {
      khungHd.className = 'khungHuongDanAnToan huongDanXanh';
      chuHd.textContent = 'Xe đang tự cân bằng ổn định — Nhấn Mở cần điều khiển để lái hoặc Dừng khẩn khi cần.';
    }
  } else if (duLieu.canCalibLap) {
    khungHd.className = 'khungHuongDanAnToan huongDanVang';
    chuHd.textContent = 'Mở Chế độ & Calib. Với LSM đã khai báo chiều tiến, dựng xe đứng yên rồi chọn Calib 2s (c). Khi cần đo hướng lắp bằng chuyển động, chọn hiệu chuẩn 3 tư thế.';
  } else if (!duLieu.gocHopLe) {
    khungHd.className = 'khungHuongDanAnToan huongDanVang';
    chuHd.textContent = 'Đang chờ dữ liệu góc ổn định từ cảm biến IMU...';
  } else if (Math.abs(lechGoc) > 5.7) {
    khungHd.className = 'khungHuongDanAnToan huongDanVang';
    const huongLech = lechGoc > 0 ? 'ngả trước' : 'ngả sau';
    chuHd.textContent = `Dựng thẳng xe về gần ${duLieu.gocCanBangDo.toFixed(1)}° để bật cân bằng (đang ${huongLech} ${Math.abs(lechGoc).toFixed(1)}°).`;
  } else if (!duLieu.heThongSanSang) {
    khungHd.className = 'khungHuongDanAnToan huongDanVang';
    const lyDo = (duLieu.tenSanSang && duLieu.tenSanSang !== 'none') ? moTaMaLoi(duLieu.tenSanSang) : 'Hệ thống chưa sẵn sàng';
    chuHd.textContent = `Chưa sẵn sàng khởi động: ${lyDo}.`;
  } else {
    khungHd.className = 'khungHuongDanAnToan huongDanXanh';
    chuHd.textContent = 'Sẵn sàng khởi động — Giữ xe thẳng đứng yên rồi nhấn BẬT CÂN BẰNG.';
  }

  const nutBat = document.getElementById('nutBatCanBang');
  const nutXoa = document.getElementById('nutXoaLoi');
  nutBat.className = (!motorDangChay && !coLoi && duDieuKienBat) ? 'nut nutXanh nutGoiYBat' : 'nut nutXanh';
  nutXoa.className = coLoi ? 'nut nutVang' : 'nut';

  const vungAnToan = document.getElementById('vungAnToanKhoiDong');
  const tamAnToan = Math.max(9.5, Math.min(90.5, 50 + (duLieu.gocCanBangDo / 60) * 100));
  vungAnToan.style.left = (tamAnToan - 9.5).toFixed(2) + '%';
  vungAnToan.style.width = '19%';

  const gocDat = typeof duLieu.gocDatDo === 'number' ? duLieu.gocDatDo : duLieu.gocCanBangDo;
  const pctDat = Math.max(0, Math.min(100, 50 + (gocDat / 60) * 100));
  const pctKim = Math.max(0, Math.min(100, 50 + (duLieu.gocNghienDo / 60) * 100));
  document.getElementById('vachGocDat').style.left = pctDat.toFixed(2) + '%';
  const kim = document.getElementById('kimGocNghien');
  kim.style.left = pctKim.toFixed(2) + '%';
  const absLech = Math.abs(lechGoc);
  kim.className = absLech > 20 ? 'kimGocNghien kimDo' : (absLech > 8 ? 'kimGocNghien kimVang' : 'kimGocNghien');

  logRungSupported = Boolean(duLieu.logRungSupported);
  logRungEnabled = Boolean(duLieu.logRungEnabled);
  if (logRungEnabled && !document.hidden) batDauGuiLienTuc();
  document.getElementById('khungLogRung').style.display = logRungSupported ? 'block' : 'none';
  const nutLog = document.getElementById('nutLogRung');
  nutLog.disabled = doiLogRungDangCho;
  nutLog.textContent = logRungEnabled ? 'TẮT LOG RUNG ESP-NOW' : 'BẬT LOG RUNG ESP-NOW';
  nutLog.className = logRungEnabled ? 'nut nutXanh' : 'nut';
  document.getElementById('trangThaiLogRung').textContent =
    (logRungEnabled ? 'Đang ghi' : 'Đã tắt') + ' | ESP thu ' + (duLieu.logRungReceiver ? 'đã kết nối' : 'chưa kết nối') +
    ' | ' + duLieu.logRungHz + ' Hz | Phiên ' + duLieu.logRungId +
    ' | Mẫu ' + duLieu.logRungProduced + ' | Bỏ hàng đợi/gửi ' + duLieu.logRungQueueDrop + '/' + duLieu.logRungTxDrop;
  capNhatTrangThaiKenh();

  document.getElementById('oGocNghien').textContent = duLieu.gocNghienDo.toFixed(2) + '°';
  document.getElementById('oGocDat').textContent = gocDat.toFixed(2) + '°';
  document.getElementById('oGocCanBang').textContent = duLieu.gocCanBangDo.toFixed(2) + '°';
  document.getElementById('oGocCanBangCalib').textContent = duLieu.gocCanBangDo.toFixed(2) + '°';
  document.getElementById('oTocDoGoc').textContent = (duLieu.tocDoGocRadGiay || 0).toFixed(2);
  document.getElementById('oViTri').textContent = duLieu.viTriMet.toFixed(3) + ' m';
  document.getElementById('oVanToc').textContent = duLieu.vanTocMetGiay.toFixed(3) + ' m/s';
  const tenCheDoDep = duLieu.dangChayPidMo ? 'PID Mờ' : 'PID Thuần';
  document.getElementById('oCheDo').textContent = `${tenCheDoDep} (${dangBatBench ? 'Giá đỡ' : 'Bánh'})`;
  document.getElementById('oLoi').textContent = coLoi ? moTaMaLoi(duLieu.tenLoi) :
    (duLieu.tenSanSang && duLieu.tenSanSang !== 'none' ? moTaMaLoi(duLieu.tenSanSang) : 'Bình thường');
  document.getElementById('oTongGoiNhiPhan').textContent = duLieu.tongGoiNhiPhan;
  const nhanLai = document.getElementById('nhanTrangThaiLai');
  nhanLai.className = nhanTT.className;
  nhanLai.textContent = nhanTT.textContent;
  document.getElementById('oGocNghienLai').textContent = duLieu.gocNghienDo.toFixed(2) + '°';
  document.getElementById('oVanTocLai').textContent = duLieu.vanTocMetGiay.toFixed(2) + ' m/s';

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
  const nutCalib = document.getElementById('nutCalib2s');
  const nutZero = document.getElementById('nutZeroNhanh');
  const nhacKhoa = document.getElementById('nhacKhoaCalib');
  nutVong.textContent = 'Vòng vị trí (v): ' + (duLieu.batVongViTri ? 'BẬT' : 'TẮT');
  nutVong.className = duLieu.batVongViTri ? 'nut nutXanh' : 'nut';
  nutBench.textContent = 'Giá đỡ (b): ' + (dangBatBench ? 'BẬT' : 'TẮT');
  nutBench.className = dangBatBench ? 'nut nutVang' : 'nut';
  nutDaoChieu.textContent = 'Đảo chiều: ' + (duLieu.daoChieuDongCo ? 'ĐẢO' : 'MẶC ĐỊNH');
  nutDaoChieu.className = (duLieu.daoChieuDongCo ? 'nut nutVang' : 'nut') + (motorDangChay ? ' nutBiKhoa' : '');
  nutCalib.className = motorDangChay ? 'nut nutBiKhoa' : 'nut';
  nutZero.className = motorDangChay ? 'nut nutBiKhoa' : 'nut';
  nhacKhoa.style.display = motorDangChay ? 'block' : 'none';

  const khungCalib2 = document.getElementById('khungCalib2Truc');
  const hoTroCalib2 = Boolean(duLieu.hoTroCalib2Truc);
  khungCalib2.style.display = hoTroCalib2 ? 'flex' : 'none';
  if (hoTroCalib2) {
    const pha2 = Math.max(0, Math.min(8, Number(duLieu.phaCalib2Truc || 0)));
    const mau2 = Number(duLieu.mauCalib2Truc || 0);
    const tong2 = Number(duLieu.tongMauCalib2Truc || 0);
    const choXacNhan2 = (pha2 === 1 || pha2 === 3 || pha2 === 5);
    const dangDo2 = (pha2 === 2 || pha2 === 4 || pha2 === 6);
    const dangChay2 = pha2 >= 1 && pha2 <= 6;
    const nhanPha2 = document.getElementById('nhanPhaCalib2');
    nhanPha2.textContent = pha2 === 7 ? 'Đã xong' : (pha2 === 8 ? ('Lỗi: ' + (duLieu.loiCalib2Truc || 'fail')) : (dangChay2 ? (dangDo2 && tong2 > 0 ? `${mau2}/${tong2}` : 'Đang chạy') : (duLieu.canCalibLap ? 'Cần calib' : 'Sẵn sàng')));
    nhanPha2.className = pha2 === 7 ? 'nhan nhanBat' : (pha2 === 8 ? 'nhan nhanLoi' : (dangChay2 || duLieu.canCalibLap ? 'nhan nhanCanhBao' : 'nhan nhanTat'));
    let txtHd2 = huongDanPhaCalib2[pha2] || huongDanPhaCalib2[0];
    if (dangDo2 && tong2 > 0) txtHd2 += ` (${mau2}/${tong2} mẫu)`;
    if (pha2 === 8 && duLieu.loiCalib2Truc && duLieu.loiCalib2Truc !== 'none') txtHd2 = `THẤT BẠI (${duLieu.loiCalib2Truc}) — Nhấn Bắt đầu để thử lại.`;
    document.getElementById('huongDanCalib2').textContent = txtHd2;
    const nBatDau2 = document.getElementById('nutBatDauCalib2');
    const nXacNhan2 = document.getElementById('nutXacNhanCalib2');
    const nHuy2 = document.getElementById('nutHuyCalib2');
    nBatDau2.disabled = motorDangChay || dangChay2;
    nBatDau2.className = nBatDau2.disabled ? 'nut nutXanhDuong nutBiKhoa' : 'nut nutXanhDuong';
    nXacNhan2.disabled = !choXacNhan2;
    nXacNhan2.className = choXacNhan2 ? 'nut nutXanh nutGoiYBat' : 'nut nutBiKhoa';
    nHuy2.disabled = !dangChay2;
    nHuy2.className = dangChay2 ? 'nut nutDo' : 'nut nutDo nutBiKhoa';
  }

  document.getElementById('tieuDePid').textContent = 'TINH CHỈNH: ' + (duLieu.dangChayPidMo ? 'PID MỜ (FUZZY)' : 'PID THUẦN');
  document.getElementById('pidHieuDungTomTat').textContent =
    `Hệ số hiệu dụng đang chạy: Kp=${duLieu.kpHieuDung.toFixed(2)} | Ki=${duLieu.kiHieuDung.toFixed(2)} | Kd=${duLieu.kdHieuDung.toFixed(2)}`;
  document.getElementById('thongTinPhu').textContent =
    `K hiệu dụng=(${duLieu.kpHieuDung.toFixed(2)}, ${duLieu.kiHieuDung.toFixed(2)}, ${duLieu.kdHieuDung.toFixed(2)}) | ` +
    `STEP=(${duLieu.xungBanhTrai}, ${duLieu.xungBanhPhai}) | MKS=(${duLieu.encoderTrai}, ${duLieu.encoderPhai}) | ` +
    `VòngVịTrí=${duLieu.batVongViTri ? 'BẬT' : 'TẮT'} | GiáĐỡ=${dangBatBench ? 'BẬT' : 'TẮT'} | ` +
    `ĐảoChiều=${duLieu.daoChieuDongCo ? 'BẬT' : 'TẮT'} | motorFault=${duLieu.maLoiDongCo} | raw=${duLieu.tenLoi}/${duLieu.tenSanSang}`;
  document.getElementById('thongTinMang').textContent =
    `WiFi AP: ${duLieu.diaChiIp} | WS :81 | UDP :4210 | Nhị phân: ${duLieu.tongGoiNhiPhan} | ESP-NOW(${duLieu.diaChiMac}): ${duLieu.tongGoiEspNow}`;

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
}

function capNhatTrangThaiXe() {
  if (dangDocTrangThai) return;
  dangDocTrangThai = true;
  const mocBatDau = performance.now();
  const boHuy = (typeof AbortController === 'function') ? new AbortController() : null;
  const henHuy = boHuy ? setTimeout(() => boHuy.abort(), 1500) : null;
  fetch('/trangthai', boHuy ? { signal: boHuy.signal } : undefined)
    .then((phanHoi) => {
      if (!phanHoi.ok) throw new Error();
      return phanHoi.json();
    })
    .then((duLieu) => {
      if (henHuy) clearTimeout(henHuy);
      dangDocTrangThai = false;
      soLanLoiLienTiep = 0;
      capNhatTrangThaiKetNoi(true, performance.now() - mocBatDau);
      apDungTrangThaiLenUi(duLieu);
    })
    .catch(() => {
      if (henHuy) clearTimeout(henHuy);
      dangDocTrangThai = false;
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
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (batDieuKhienWifi) {
        daBatWifi = WiFi.softAP(tenMangWifiAp, matKhauWifiAp, kenhWifiAp, 0, 4);
        if (daBatWifi) {
            datCongSuatWifi(11);
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
            mayChuWeb.enableDelay(false);
            mayChuWeb.begin();
            mayChuWsNhiPhan.begin();
            mayChuWsNhiPhan.setNoDelay(true);
            congUdpLenh.begin(congUdpNhiPhan);
        }
    } else {
        datCongSuatWifi(11);
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
    if (coDiChuyen && hamDiChuyen) hamDiChuyen(vt, qy, {});
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
    if (hamDiChuyen) hamDiChuyen(vt, qy, {});
    mayChuWeb.send(200, "text/plain", "ok");
}

bool DieuKhienKhongDay::apDungKhungNhiPhan(const uint8_t* duLieu, size_t doDai) {
    float vt = 0.0f;
    float qy = 0.0f;
    char phim = '\0';
    bool coDiChuyen = false;
    MucLenhWeb mucWeb{};
    if (!giaiMaGoiLenhNhiPhan(duLieu, doDai, vanTocDiChuyenToiDa_mps, tocDoQuayToiDa_radps,
                              vt, qy, phim, coDiChuyen, &mucWeb)) {
        return false;
    }
    portENTER_CRITICAL(&khoaDuLieuEspNow);
    ++demGoiNhiPhan;
    portEXIT_CRITICAL(&khoaDuLieuEspNow);
    if (phim != '\0' && hamPhimLenh) hamPhimLenh(phim);
    if (coDiChuyen && hamDiChuyen) hamDiChuyen(vt, qy, mucWeb);
    return true;
}

void DieuKhienKhongDay::xuLyApiNhiPhan() {
    uint8_t duLieuNhiPhan[sizeof(GoiLenhWebKhaoSat)]{};
    size_t doDai = 0;
    bool hopLe = false;
    if (mayChuWeb.hasArg("g") && giaiMaChuoiHexLenh(mayChuWeb.arg("g"), duLieuNhiPhan, doDai)) {
        hopLe = apDungKhungNhiPhan(duLieuNhiPhan, doDai);
    }
    mayChuWeb.sendHeader("Cache-Control", "no-store");
    mayChuWeb.send(hopLe ? 204 : 400, "text/plain", hopLe ? "" : "bad");
}

void DieuKhienKhongDay::xuLyUdpNhiPhan() {
    for (int luot = 0; luot < 8; ++luot) {
        const int kichThuoc = congUdpLenh.parsePacket();
        if (kichThuoc <= 0) break;
        if (kichThuoc == static_cast<int>(sizeof(GoiLenhNhiPhan)) ||
            kichThuoc == static_cast<int>(sizeof(GoiLenhWebKhaoSat))) {
            uint8_t boDemUdp[sizeof(GoiLenhWebKhaoSat)]{};
            const int daDoc = congUdpLenh.read(boDemUdp, sizeof(boDemUdp));
            if (daDoc == kichThuoc) {
                apDungKhungNhiPhan(boDemUdp, static_cast<size_t>(daDoc));
            }
        } else {
            congUdpLenh.flush();
        }
    }
}

uint8_t DieuKhienKhongDay::soKhachWifiAp() const {
    return daBatWifi ? WiFi.softAPgetStationNum() : 0u;
}

bool DieuKhienKhongDay::datCongSuatWifi(int dbm) {
    wifi_power_t muc = WIFI_POWER_11dBm;
    const char* nhan = "11dBm";
    if (dbm == 8) { muc = WIFI_POWER_8_5dBm; nhan = "8.5dBm"; }
    else if (dbm == 11) { muc = WIFI_POWER_11dBm; nhan = "11dBm"; }
    else if (dbm == 13) { muc = WIFI_POWER_13dBm; nhan = "13dBm"; }
    else if (dbm == 15) { muc = WIFI_POWER_15dBm; nhan = "15dBm"; }
    else if (dbm == 19) { muc = WIFI_POWER_19_5dBm; nhan = "19.5dBm"; }
    else return false;
    if (!WiFi.setTxPower(muc)) return false;
    std::snprintf(chuoiCongSuatWifi, sizeof(chuoiCongSuatWifi), "%s", nhan);
    return true;
}

void DieuKhienKhongDay::xuLyWebSocketNhiPhan() {
    if (mayChuWsNhiPhan.hasClient()) {
        WiFiClient khachMoi = mayChuWsNhiPhan.accept();
        if (khachMoi) {
            khachMoi.setNoDelay(true);
            khachMoi.setTimeout(50);
            if (khachWsNhiPhan) khachWsNhiPhan.stop();
            khachWsNhiPhan = khachMoi;
            daBatTayWsNhiPhan = false;
            doDaiBatTayWs = 0u;
            doDaiKhungWs = 0u;
            mocMoWsMs = millis();
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
        if (millis() - mocMoWsMs > 1500u) {
            khachWsNhiPhan.stop();
            doDaiBatTayWs = 0u;
            return;
        }
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
        if ((maTacVu == 0x02u || maTacVu == 0x01u) &&
            (doDaiTai == sizeof(GoiLenhNhiPhan) || doDaiTai == sizeof(GoiLenhWebKhaoSat))) {
            apDungKhungNhiPhan(taiGiaiMa, static_cast<size_t>(doDaiTai));
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
    char boDemJson[1792]{};
    const int doDaiJson = std::snprintf(
        boDemJson, sizeof(boDemJson),
        "{\"daBat\":%s,\"dangChayPidMo\":%s,\"cheDoNangBanh\":%s,\"batVongViTri\":%s,\"daoChieuDongCo\":%s,"
        "\"gocHopLe\":%s,\"heThongSanSang\":%s,"
        "\"gocNghienDo\":%.2f,\"gocCanBangDo\":%.2f,\"gocDatDo\":%.2f,"
        "\"viTriMet\":%.4f,\"vanTocMetGiay\":%.4f,\"tocDoGocRadGiay\":%.4f,"
        "\"xungBanhTrai\":%lld,\"xungBanhPhai\":%lld,\"encoderTrai\":%lld,\"encoderPhai\":%lld,"
        "\"kpGoc\":%.3f,\"kiGoc\":%.3f,\"kdGoc\":%.3f,\"kpVanToc\":%.4f,\"kiVanToc\":%.4f,"
        "\"kpHieuDung\":%.2f,\"kiHieuDung\":%.2f,\"kdHieuDung\":%.2f,"
        "\"maLoiDongCo\":%u,\"tenLoi\":\"%s\",\"tenSanSang\":\"%s\",\"cheDoDieuKhien\":\"%s\","
        "\"hoTroCalib2Truc\":%s,\"canCalibLap\":%s,\"phaCalib2Truc\":%u,\"mauCalib2Truc\":%u,\"tongMauCalib2Truc\":%u,\"loiCalib2Truc\":\"%s\","
        "\"logRungSupported\":%s,\"logRungEnabled\":%s,\"logRungReceiver\":%s,"
        "\"logRungId\":%lu,\"logRungQueueDrop\":%lu,\"logRungTxDrop\":%lu,\"logRungHz\":%u,\"logRungProduced\":%lu,"
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
        trangThai.hoTroCalib2Truc ? "true" : "false", trangThai.canCalibLap ? "true" : "false",
        static_cast<unsigned>(trangThai.phaCalib2Truc), static_cast<unsigned>(trangThai.mauCalib2Truc),
        static_cast<unsigned>(trangThai.tongMauCalib2Truc), trangThai.loiCalib2Truc,
        trangThai.logRungSupported ? "true" : "false", trangThai.logRungEnabled ? "true" : "false",
        trangThai.logRungReceiver ? "true" : "false", static_cast<unsigned long>(trangThai.logRungId),
        static_cast<unsigned long>(trangThai.logRungQueueDrop), static_cast<unsigned long>(trangThai.logRungTxDrop),
        static_cast<unsigned>(trangThai.logRungHz), static_cast<unsigned long>(trangThai.logRungProduced),
        static_cast<unsigned long>(demGoiEspNow), static_cast<unsigned long>(demGoiNhiPhan),
        chuoiDiaChiIp, chuoiDiaChiMac);
    if (doDaiJson < 0 || static_cast<size_t>(doDaiJson) >= sizeof(boDemJson)) {
        mayChuWeb.send(500, "application/json", "{\"error\":\"status_overflow\"}");
        return;
    }
    mayChuWeb.sendHeader("Cache-Control", "no-store");
    mayChuWeb.send(200, "application/json", boDemJson);
}
