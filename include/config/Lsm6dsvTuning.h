#pragma once
// Tham số tối ưu từ nhận dạng hệ thực 240 Hz khi chạy (f_n ~12.2 Hz, Q ~2.2, trễ ~18.5 ms tại 10 Hz, IMU cao +10 mm):
// save/resetcfg chỉ tác động namespace xe_lsm6dsv khi nạp profile này.
namespace lsm6dsvTune {
// Theo cách gá trên xe: chiều -X in trên module chỉ về đầu xe.
constexpr float huongTienSensor[3] = {-1.0f, 0.0f, 0.0f};
constexpr float kp = 28.0f, ki = 0.0f, kd = 2.20f;
constexpr float kpv = 0.14f, kiv = 0.06f;
constexpr float kpMo = 22.0f, kiMo = 0.0f, kdMo = 1.85f;
constexpr float kpvMo = 0.13f, kivMo = 0.06f;
constexpr float gocCanBang_rad = 0.0f;
constexpr float gyroD_hz = 65.0f;
constexpr float locGocDatTinh_s = 0.004f; // Gần như không trễ pha tại đứng yên (1 mẫu @ 240 Hz).
constexpr float locGocDat_s = 0.030f;     // Làm mượt PT1 góc đặt khi đang chạy/chuyển hướng.
constexpr float vanTocBatDauGiamGain_mps = 0.03f;
constexpr float vanTocGiamGainToiDa_mps = 0.12f;
constexpr float heSoGiamChayKp = 0.72f;   // 28.0 -> 20.16 khi chạy (GM >= +3.7 dB @ 10 Hz).
constexpr float heSoGiamChayKd = 0.78f;   // 2.20 -> 1.72 khi chạy; PID Mờ 1.85*0.78*1.5 = 2.16.
constexpr float hangSoLocTocDoSchedule_s = 0.06f;
constexpr float locGoc_s = 0.5f;
constexpr float saiLechGiaToc_mps2 = 2.5f;
constexpr float giamTinGoc_rad = 0.08f, matTinGoc_rad = 0.25f, matTin_s = 0.5f;
constexpr bool buGiaTocImu = false;
constexpr bool tangGiaToc = false, tangBamVanToc = true;
// Khi chưa lưu ma trận: dùng c (một tư thế) hoặc cal3. Chỉ bật cờ nếu đã nhập ma trận đo sẵn.
constexpr bool daXacNhanMaTranLap = false;
constexpr float maTranLap[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
}
