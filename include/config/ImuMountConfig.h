#pragma once

// Hệ xe: X tiến, Y trái, Z lên (tay phải).
// v_xe = R_camBienSangXe * v_camBien, áp cho cả accel và gyro sau trừ bias.
// R = Rz(yaw) * Ry(pitch) * Rx(roll), góc của hệ cảm biến so với hệ xe.
// Mặc định chưa đo góc gá nên là identity; KHÔNG tự lấy tư thế lúc boot làm chuẩn.
constexpr bool dungMaTranLapImu = false;
constexpr float gocLapRoll_deg = 0.0f;
constexpr float gocLapPitch_deg = 0.0f;
constexpr float gocLapYaw_deg = 0.0f;
// Khi dùng ma trận hiệu chuẩn, bật dungMaTranLapImu và thay đủ chín phần tử.
constexpr float maTranCamBienSangXe[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};
