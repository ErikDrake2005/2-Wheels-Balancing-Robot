#pragma once
#include "config/PinConfig.h"

constexpr bool batUiCalib = true;
constexpr bool thuTuRgbCalibLaGRB = true;
static_assert(!batUiCalib || (chanNutCalib != chanSdaMpu && chanNutCalib != chanSclMpu &&
    chanNutCalib != chanNgatMpu && chanNutCalib != chanDirTrai && chanNutCalib != chanDirPhai &&
    chanNutCalib != chanStepTrai && chanNutCalib != chanStepPhai && chanNutCalib != chanBatMotor),
    "Chan nut calib trung chan IMU/motor");
static_assert(!batUiCalib || chanRgbCalib < 0 ||
    (chanRgbCalib != chanNutCalib && chanRgbCalib != chanSdaMpu && chanRgbCalib != chanSclMpu &&
     chanRgbCalib != chanNgatMpu && chanRgbCalib != chanDirTrai && chanRgbCalib != chanDirPhai &&
     chanRgbCalib != chanStepTrai && chanRgbCalib != chanStepPhai && chanRgbCalib != chanBatMotor),
    "Chan RGB calib trung chan nut/IMU/motor");
