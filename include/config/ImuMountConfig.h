#pragma once

// Hệ xe: X tiến, Y trái, Z lên (tay phải).
// v_xe = R_camBienSangXe * v_camBien, áp cho cả accel và gyro sau trừ bias.
// Ma trận dưới đây đã được trích xuất và chuẩn hóa trực giao (det = +1) từ dữ liệu thực tế
// CALIB_MOC trên xe của bạn:
//   a_dung  = [-0.0105, -0.1388,  9.7529] m/s^2
//   a_truoc = [-0.1806, +9.7183,  0.3280] m/s^2 (Hạ đầu xe ~+90 deg)
//   a_sau   = [+0.0719, -9.8278,  0.7497] m/s^2 (Hạ đuôi xe ~-90 deg)
constexpr bool dungMaTranLapImu = true;
constexpr float gocLapRoll_deg = 0.0f;
constexpr float gocLapPitch_deg = 0.0f;
constexpr float gocLapYaw_deg = 90.0f;
constexpr float maTranCamBienSangXe[3][3] = {
    { 0.0129582f, -0.9998150f, -0.0142151f},
    { 0.9999155f,  0.0129416f,  0.0012556f},
    {-0.0010714f, -0.0142302f,  0.9998982f}
};
