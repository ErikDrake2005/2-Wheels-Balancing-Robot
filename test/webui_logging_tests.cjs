
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const page=fs.readFileSync('src/app/DieuKhienKhongDay.cpp','utf8');
const src=page.split('<script>')[1].split('</script>')[0];
new vm.Script(src);
const elements=new Map();
function elt(id){if(!elements.has(id))elements.set(id,{style:{},value:'100',listeners:{},classList:{add(){},remove(){}},addEventListener(k,f){this.listeners[k]=f},getBoundingClientRect(){return {top:0,left:0,width:200,height:200}},setPointerCapture(){}});return elements.get(id)}
const windowEvents={},documentEvents={};
let tick=null;let timerCleared=false;
const context={console,DataView,ArrayBuffer,Uint8Array,Math,Date,performance:{now:()=>0},
setInterval:(f)=>{tick=f;timerCleared=false;return 1;},clearInterval(){timerCleared=true;},setTimeout:()=>1,clearTimeout(){},
window:{location:{hostname:'192.168.4.1'},addEventListener(k,f){windowEvents[k]=f}},
document:{hidden:false,getElementById:elt,addEventListener(k,f){documentEvents[k]=f}},
fetch:()=>new Promise(()=>{}),WebSocket:function(){this.readyState=0},assert};
vm.createContext(context);
vm.runInContext(src+`
logRungSupported=true;
tyLeJoystickTienLui=.37;tyLeJoystickReNgang=-.42;heSoTocDoDiaHinh=.75;
tinhLaiVanTocJoystick();
let f=dongGoiKhungNhiPhan(1,vanTocHienTai,tocDoQuayHienTai,0),d=new DataView(f);
assert.strictEqual(f.byteLength,14);assert.strictEqual(d.getUint8(1),3);
assert.strictEqual(d.getInt16(8,true),37);assert.strictEqual(d.getInt16(10,true),-42);assert.strictEqual(d.getUint8(12),75);
let sum=0;for(let i=0;i<13;i++)sum^=d.getUint8(i);assert.strictEqual(sum,d.getUint8(13));
assert.strictEqual(dongGoiKhungNhiPhan(2,0,0,97).byteLength,8);
maConTroTienLui=5;nhaJoystickTienLui({pointerId:5});
assert.strictEqual(tyLeJoystickTienLui,0);assert.strictEqual(vanTocHienTai,0);
assert.strictEqual(tyLeJoystickReNgang,-.42);
tyLeJoystickTienLui=.5;maConTroTienLui=1;
const before=noiSuyTangTocKeo(.5)*vanTocToiDaXe;
thanhTocDo.value='50';thanhTocDo.listeners.input();
assert(Math.abs(vanTocHienTai-before*.5)<1e-12);
phanhGiuViTri();
assert.strictEqual(tyLeJoystickTienLui,0);assert.strictEqual(tyLeJoystickReNgang,0);
logRungSupported=false;assert.strictEqual(dongGoiKhungNhiPhan(1,0,0,0).byteLength,8);
logRungSupported=true;logRungEnabled=true;
maConTroTienLui=null;maConTroReNgang=null;
batDauGuiLienTuc();
`,context);
assert(tick);tick();assert(!timerCleared, 'logging must keep neutral UI updates');
vm.runInContext('logRungEnabled=false;',context);
tick();assert(timerCleared, 'idle UI stops heartbeat when logging is off');

// Check single <script> tag and every getElementById(...) exists in HTML
assert.strictEqual(page.split('<script>').length, 2, 'must have exactly one <script> tag');
assert.strictEqual(page.split('</script>').length, 2, 'must have exactly one </script> tag');
const htmlOnly = page.split('const char trangDieuKhienHtml[] PROGMEM = R"HTML(')[1].split(')HTML";')[0];
const domIds = new Set([...htmlOnly.matchAll(/id="([^"]+)"/g)].map(m => m[1]));
for (const m of src.matchAll(/getElementById\('([^']+)'\)/g)) {
  assert(domIds.has(m[1]), 'missing DOM id in HTML: ' + m[1]);
}

// Verify Vietnamese fault descriptions, status UI updates, calib lock, speed presets, and keyboard shortcuts
let fetchCalls = [];
context.fetch = (url) => { fetchCalls.push(url); return new Promise(() => {}); };
vm.runInContext(`
assert.strictEqual(moTaMaLoi('none'), 'Bình thường');
assert.strictEqual(moTaMaLoi('fallen'), 'Nghiêng quá giới hạn ngã');
assert.strictEqual(moTaMaLoi('motor_fault'), 'Lỗi driver động cơ');
assert.strictEqual(moTaMaLoi('custom_code'), 'Mã lỗi: custom_code');

const baseStatus = {
  daBat: false, dangChayPidMo: false, cheDoNangBanh: false, batVongViTri: true, daoChieuDongCo: false,
  gocHopLe: true, heThongSanSang: true,
  gocNghienDo: 0.4, gocCanBangDo: 0.2, gocDatDo: 0.2,
  viTriMet: 0.0, vanTocMetGiay: 0.0, tocDoGocRadGiay: 0.01,
  xungBanhTrai: 10, xungBanhPhai: 12, encoderTrai: 10, encoderPhai: 12,
  kpGoc: 33, kiGoc: 0, kdGoc: 3.3, kpVanToc: 0.15, kiVanToc: 0.06,
  kpHieuDung: 33, kiHieuDung: 0, kdHieuDung: 3.3,
  maLoiDongCo: 0, tenLoi: 'none', tenSanSang: 'none', cheDoDieuKhien: 'PID_THUAN',
  logRungSupported: false, logRungEnabled: false, logRungReceiver: false,
  logRungId: 0, logRungQueueDrop: 0, logRungTxDrop: 0, logRungHz: 200, logRungProduced: 0,
  tongGoiEspNow: 0, tongGoiNhiPhan: 5, diaChiIp: '192.168.4.1', diaChiMac: 'AA:BB:CC:DD:EE:FF'
};

// Case A: Idle and ready to arm
apDungTrangThaiLenUi(baseStatus);
assert.strictEqual(document.getElementById('nhanTrangThai').textContent, 'TẮT MOTOR');
assert(document.getElementById('khungHuongDanAnToan').className.includes('huongDanXanh'));
assert(document.getElementById('nutBatCanBang').className.includes('nutGoiYBat'));
assert.strictEqual(document.getElementById('nutXoaLoiNhanh').style.display, 'none');
assert.strictEqual(document.getElementById('nutCalib2s').className, 'nut');
assert.strictEqual(document.getElementById('nhacKhoaCalib').style.display, 'none');

// Case B: Tilted beyond arm limit
apDungTrangThaiLenUi(Object.assign({}, baseStatus, { gocNghienDo: 9.0, tenSanSang: 'fallen', heThongSanSang: false }));
assert(document.getElementById('khungHuongDanAnToan').className.includes('huongDanVang'));
assert(document.getElementById('chuHuongDanAnToan').textContent.includes('Dựng thẳng xe'));

// Case C: Latched fault
apDungTrangThaiLenUi(Object.assign({}, baseStatus, { tenLoi: 'fallen', heThongSanSang: false }));
assert.strictEqual(document.getElementById('nhanTrangThai').textContent, 'LỖI: Nghiêng quá giới hạn ngã');
assert(document.getElementById('khungHuongDanAnToan').className.includes('huongDanDo'));
assert.strictEqual(document.getElementById('nutXoaLoiNhanh').style.display, 'inline-flex');
assert.strictEqual(document.getElementById('nutXoaLoi').className, 'nut nutVang');

// Case D: Active balancing in PID_MO + calib lock enforcement
apDungTrangThaiLenUi(Object.assign({}, baseStatus, {
  daBat: true, dangChayPidMo: true, cheDoDieuKhien: 'PID_MO', kpHieuDung: 35.4, kdHieuDung: 3.9
}));
assert.strictEqual(document.getElementById('nhanTrangThai').textContent, 'ĐANG CÂN BẰNG');
assert.strictEqual(document.getElementById('oCheDo').textContent, 'PID Mờ (Bánh)');
assert(document.getElementById('nutCalib2s').className.includes('nutBiKhoa'));
assert(document.getElementById('nutZeroNhanh').className.includes('nutBiKhoa'));
assert(document.getElementById('nutDaoChieuDongCo').className.includes('nutBiKhoa'));
assert.strictEqual(document.getElementById('nhacKhoaCalib').style.display, 'block');
assert.strictEqual(document.getElementById('pidHieuDungTomTat').textContent.includes('Kp=35.40'), true);

// Case D2: Active balancing in FUZZY_SISO
apDungTrangThaiLenUi(Object.assign({}, baseStatus, {
  daBat: true, dangChayPidMo: false, cheDoDieuKhien: 'FUZZY_SISO', kpHieuDung: 0, kiHieuDung: 0, kdHieuDung: 0
}));
assert.strictEqual(document.getElementById('oCheDo').textContent, 'Mờ SISO (Bánh)');
assert(document.getElementById('nutChonMoSiso').className.includes('nutXanhDuong'));
assert(document.getElementById('nutMoSisoNhanh').className.includes('nutXanhDuong'));
assert(!document.getElementById('nutChonPidThuan').className.includes('nutXanhDuong'));
assert(!document.getElementById('nutChonPidMo').className.includes('nutXanhDuong'));
assert(document.getElementById('tieuDePid').textContent.includes('MỜ SISO TRỰC TIẾP'));
assert(document.getElementById('pidHieuDungTomTat').textContent.includes('Mờ SISO trực tiếp'));
assert(document.getElementById('pidHieuDungTomTat').textContent.includes('7 luật e -> a_cmd'));

// Case E: Active in Bench mode
apDungTrangThaiLenUi(Object.assign({}, baseStatus, { daBat: true, cheDoNangBanh: true }));
assert.strictEqual(document.getElementById('nhanTrangThai').textContent, 'CHẾ ĐỘ GIÁ ĐỠ');
assert(document.getElementById('khungHuongDanAnToan').className.includes('huongDanVang'));
assert.strictEqual(document.getElementById('khungCalib2Truc').style.display, 'none');
`, context);

const callsBeforeLock = fetchCalls.length;
vm.runInContext(`thaoTacKhiTatMotor('phim', 'c', 'calib');`, context);
assert.strictEqual(fetchCalls.length, callsBeforeLock, 'calib must be blocked while motor is running');
assert(context.document.getElementById('thongBaoNoi').textContent.includes('tắt motor'));

vm.runInContext(`
// Case F: 3-step Web calibration wizard supported & waiting for confirm
apDungTrangThaiLenUi(Object.assign({}, baseStatus, {
  heThongSanSang: false, tenSanSang: 'need_mount_calib',
  hoTroCalib2Truc: true, canCalibLap: true, phaCalib2Truc: 1
}));
assert.strictEqual(document.getElementById('khungCalib2Truc').style.display, 'flex');
assert(document.getElementById('chuHuongDanAnToan').textContent.includes('3 tư thế'));
assert(document.getElementById('chuHuongDanAnToan').textContent.includes('Calib 2s (c)'));
assert.strictEqual(document.getElementById('nutXacNhanCalib2').disabled, false);
assert.strictEqual(document.getElementById('nutBatDauCalib2').disabled, true);
assert(document.getElementById('huongDanCalib2').textContent.includes('Bước 1/3'));

// Case G: Measuring phase (progress counter & confirm button locked) + Done phase (7)
apDungTrangThaiLenUi(Object.assign({}, baseStatus, {
  heThongSanSang: false, tenSanSang: 'need_mount_calib',
  hoTroCalib2Truc: true, canCalibLap: true, phaCalib2Truc: 4, mauCalib2Truc: 45, tongMauCalib2Truc: 80
}));
assert.strictEqual(document.getElementById('nhanPhaCalib2').textContent, '45/80');
assert.strictEqual(document.getElementById('nutXacNhanCalib2').disabled, true);
assert(document.getElementById('huongDanCalib2').textContent.includes('(45/80 mẫu)'));
apDungTrangThaiLenUi(Object.assign({}, baseStatus, {
  hoTroCalib2Truc: true, canCalibLap: false, phaCalib2Truc: 7
}));
assert.strictEqual(document.getElementById('nhanPhaCalib2').textContent, 'Đã xong');
assert.strictEqual(document.getElementById('nutBatDauCalib2').disabled, false);
`, context);

vm.runInContext(`
// Speed presets
datMucTocDoNhanh(35);
assert.strictEqual(thanhTocDo.value, '35');
assert.strictEqual(heSoTocDoDiaHinh, 0.35);
assert.strictEqual(document.getElementById('oPhanTramTocDo').textContent, '35%');
assert(document.getElementById('nutTocDo35').className.includes('nutXanhDuong'));
datMucTocDoNhanh(65);
assert(document.getElementById('nutTocDo65').className.includes('nutXanhDuong'));
datMucTocDoNhanh(100);
assert(document.getElementById('nutTocDo100').className.includes('nutXanhDuong'));
`, context);

// Keyboard control & modal Esc
windowEvents.keydown({ key: 'w', target: { tagName: 'BODY' }, preventDefault(){} });
assert.strictEqual(vm.runInContext('tyLeJoystickTienLui', context), 1);
windowEvents.keyup({ key: 'w' });
assert.strictEqual(vm.runInContext('tyLeJoystickTienLui', context), 0);
windowEvents.keydown({ key: 'w', target: { tagName: 'INPUT' }, preventDefault(){} });
assert.strictEqual(vm.runInContext('tyLeJoystickTienLui', context), 0, 'keyboard driving ignored inside INPUT');

// Drive controls live inside modal cuaSoDieuKhien, opened by nutMoDieuKhien
assert(htmlOnly.indexOf('id="nutMoDieuKhien"') >= 0, 'home button nutMoDieuKhien must exist');
assert(htmlOnly.indexOf('id="cuaSoDieuKhien"') >= 0, 'drive modal cuaSoDieuKhien must exist');
assert(htmlOnly.indexOf('id="vungCanGatTienLui"') > htmlOnly.indexOf('id="cuaSoDieuKhien"'),
  'joysticks must reside inside cuaSoDieuKhien modal, not home screen');
assert(htmlOnly.indexOf('id="vungCanGatReNgang"') > htmlOnly.indexOf('id="cuaSoDieuKhien"'),
  'steering joystick must reside inside cuaSoDieuKhien modal');

vm.runInContext(`moCuaSoCon('cuaSoDieuKhien');`, context);
assert.strictEqual(vm.runInContext('cuaSoDangMo', context), 'cuaSoDieuKhien');
assert.strictEqual(context.document.getElementById('cuaSoDieuKhien').style.display, 'block');
windowEvents.keydown({ key: 'w', target: { tagName: 'BODY' }, preventDefault(){} });
assert.strictEqual(vm.runInContext('tyLeJoystickTienLui', context), 1, 'keyboard driving works inside cuaSoDieuKhien');
vm.runInContext(`tyLeJoystickTienLui = 0.6; tyLeJoystickReNgang = -0.4; tinhLaiVanTocJoystick(); dongCuaSoCon();`, context);
assert.strictEqual(vm.runInContext('cuaSoDangMo', context), '');
assert.strictEqual(vm.runInContext('tyLeJoystickTienLui', context), 0, 'closing drive modal auto-zeros throttle');
assert.strictEqual(vm.runInContext('tyLeJoystickReNgang', context), 0, 'closing drive modal auto-zeros steering');
assert.strictEqual(vm.runInContext('vanTocHienTai', context), 0);
assert.strictEqual(vm.runInContext('tocDoQuayHienTai', context), 0);

// Switching away from cuaSoDieuKhien to another tab also auto-zeros joysticks
vm.runInContext(`moCuaSoCon('cuaSoDieuKhien'); tyLeJoystickTienLui = 0.5; tinhLaiVanTocJoystick(); moCuaSoCon('cuaSoPid');`, context);
assert.strictEqual(vm.runInContext('cuaSoDangMo', context), 'cuaSoPid');
assert.strictEqual(vm.runInContext('tyLeJoystickTienLui', context), 0, 'tab switch away from drive modal auto-zeros');
windowEvents.keydown({ key: 'Escape', target: { tagName: 'BODY' }, preventDefault(){} });
assert.strictEqual(vm.runInContext('cuaSoDangMo', context), '');

// In-flight guard on capNhatTrangThaiXe prevents overlapping /trangthai fetches
fetchCalls = [];
vm.runInContext(`dangDocTrangThai = false; capNhatTrangThaiXe(); capNhatTrangThaiXe();`, context);
assert.strictEqual(fetchCalls.filter(u => u === '/trangthai').length, 1, 'in-flight guard blocks duplicate /trangthai');

console.log('PASS: WebUI JS syntax, 14/8-byte framing, raw-percent signs, slider scaling, release/brake reset, UX status/gauge, safety lock, speed presets, drive modal auto-zero & keyboard controls');
