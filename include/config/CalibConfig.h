#pragma once
#include "config/PinConfig.h"

// Mặc định tắt UI_Calib 3 tư thế mỗi lần bật nguồn vì đã lưu ma trận thực tế vào ImuMountConfig.h.
// Khi bật nguồn: nếu giữ nút GPIO3 lúc khởi động thì vẫn vào UI_Calib 3 tư thế; nếu thả nút thì
// chỉ đo nhanh bias gyro (2.5 giây) rồi vào thẳng chế độ cân bằng READY.
constexpr bool batUiCalib = false;
constexpr bool thuTuRgbCalibLaGRB = true;
// Mục tiêu đặt nằm 75–92°; nới nhẹ để thao tác tay thuận tiện hơn.
constexpr float gocNamCalibToiThieu_rad = 1.3089969f; // 75°.
constexpr float gocNamCalibToiDa_rad = 1.6231562f; // 93°.
constexpr float saiSoGocQuayCalib_rad = 0.25f; // ~14.3°: dung sai thực tế khi lật xe bằng tay.
static_assert(!batUiCalib || (chanNutCalib != chanSdaMpu && chanNutCalib != chanSclMpu &&
    chanNutCalib != chanNgatMpu && chanNutCalib != chanDirTrai && chanNutCalib != chanDirPhai &&
    chanNutCalib != chanStepTrai && chanNutCalib != chanStepPhai && chanNutCalib != chanBatMotor),
    "Chan nut calib trung chan IMU/motor");
static_assert(!batUiCalib || chanRgbCalib < 0 ||
    (chanRgbCalib != chanNutCalib && chanRgbCalib != chanSdaMpu && chanRgbCalib != chanSclMpu &&
     chanRgbCalib != chanNgatMpu && chanRgbCalib != chanDirTrai && chanRgbCalib != chanDirPhai &&
     chanRgbCalib != chanStepTrai && chanRgbCalib != chanStepPhai && chanRgbCalib != chanBatMotor),
    "Chan RGB calib trung chan nut/IMU/motor");
