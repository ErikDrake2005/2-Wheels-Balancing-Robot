param([Alias('Compiler')][string]$TrinhBienDich = 'g++')
$ErrorActionPreference = 'Stop'
$thuMucDuAn = Split-Path -Parent $PSScriptRoot
$duongDanCu = $env:PATH
$thuMucTrinhBienDich = Split-Path -Parent (Get-Command $TrinhBienDich -ErrorAction Stop).Source
$env:PATH = "$thuMucTrinhBienDich;$env:PATH"
Push-Location $thuMucDuAn
try {
    New-Item -ItemType Directory -Force -Path .cache | Out-Null
    & $TrinhBienDich -std=c++17 -O2 -Wall -Wextra -Werror -static -Iinclude -Isrc `
        -Ilib/controllers -Ilib/estimation -Ilib/kinematics `
        test/native/control_tests.cpp lib/controllers/AnglePidController.cpp `
        lib/controllers/BoDieuKhienBanhVongKin.cpp `
        lib/controllers/CascadeController.cpp lib/estimation/ComplementaryEstimator.cpp `
        lib/kinematics/DifferentialDrive.cpp lib/estimation/HeTrucImu.cpp src/safety/SafetyMonitor.cpp `
        -o .cache/control_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Native test compilation failed' }
    & ./.cache/control_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Native tests failed' }
    & $TrinhBienDich -std=c++17 -O2 -Wall -Wextra -Werror -static `
        -Itest/native/stubs -Ilib/motor_drv/src test/native/watchdog_tests.cpp `
        lib/motor_drv/src/GiamSatDongCo.cpp -o .cache/watchdog_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Watchdog test compilation failed' }
    & ./.cache/watchdog_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Watchdog tests failed' }
    & $TrinhBienDich -std=c++17 -O2 -Wall -Wextra -Werror -static -Isrc `
        test/native/research_logger_tests.cpp -o .cache/research_logger_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Research logger test compilation failed' }
    & ./.cache/research_logger_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Research logger tests failed' }
    & $TrinhBienDich -std=c++17 -O2 -Wall -Wextra -Werror -static -Iinclude -Ilib/motor_drv/src `
        test/native/uart_schedule_tests.cpp -o .cache/uart_schedule_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'UART schedule test compilation failed' }
    & ./.cache/uart_schedule_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'UART schedule tests failed' }
    & $TrinhBienDich -std=c++17 -O2 -Wall -Wextra -Werror -static -Iinclude `
        test/native/motor_timing_tests.cpp -o .cache/motor_timing_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor timing test compilation failed' }
    & ./.cache/motor_timing_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor timing tests failed' }
    foreach ($remoteTest in @('espnow_protocol_tests','remote_lease_tests')) {
        & $TrinhBienDich -std=c++17 -O2 -Wall -Wextra -Werror -static -Iinclude `
            "test/native/$remoteTest.cpp" -o ".cache/$remoteTest.exe"
        if ($LASTEXITCODE -ne 0) { throw "$remoteTest compilation failed" }
        & ".cache/$remoteTest.exe"
        if ($LASTEXITCODE -ne 0) { throw "$remoteTest failed" }
    }
    & $TrinhBienDich -std=c++17 -O2 -Wall -Wextra -Werror -static -Iinclude -Isrc `
        -Ilib/controllers -Ilib/estimation -Ilib/kinematics `
        test/native/filter_motion_tests.cpp lib/controllers/AnglePidController.cpp `
        lib/controllers/BoDieuKhienBanhVongKin.cpp `
        lib/controllers/CascadeController.cpp lib/estimation/ComplementaryEstimator.cpp `
        lib/kinematics/DifferentialDrive.cpp lib/estimation/HeTrucImu.cpp -o .cache/filter_motion_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Filter/motion test compilation failed' }
    & ./.cache/filter_motion_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Filter/motion tests failed' }
    & $TrinhBienDich -std=c++17 -O2 -Wall -Wextra -Werror -static `
        "-DCAU_HINH_PHAN_CUNG_DONG_CO=LoaiPhanCungDongCo::TMC_MT" -Iinclude -Isrc `
        -Ilib/controllers -Ilib/estimation -Ilib/kinematics `
        test/native/filter_motion_tests.cpp lib/controllers/AnglePidController.cpp `
        lib/controllers/BoDieuKhienBanhVongKin.cpp `
        lib/controllers/CascadeController.cpp lib/estimation/ComplementaryEstimator.cpp `
        lib/kinematics/DifferentialDrive.cpp lib/estimation/HeTrucImu.cpp -o .cache/filter_motion_tmc_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'TMC_MT filter/motion test compilation failed' }
    & ./.cache/filter_motion_tmc_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'TMC_MT filter/motion tests failed' }
    & $TrinhBienDich -std=c++17 -O2 -Wall -Wextra -Werror -static -Iinclude -Isrc -Ilib/estimation `
        test/native/imu_alignment_tests.cpp lib/estimation/HeTrucImu.cpp `
        lib/estimation/ComplementaryEstimator.cpp src/safety/SafetyMonitor.cpp `
        -o .cache/imu_alignment_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'IMU alignment test compilation failed' }
    & ./.cache/imu_alignment_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'IMU alignment tests failed' }
    & $TrinhBienDich -std=c++17 -O2 -Wall -Wextra -Werror -static -Iinclude -Ilib/estimation `
        test/native/calib_pitch_tests.cpp lib/estimation/HieuChuanNghiengDoc.cpp `
        lib/estimation/HeTrucImu.cpp lib/estimation/ComplementaryEstimator.cpp -o .cache/calib_pitch_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Calibration test compilation failed' }
    & ./.cache/calib_pitch_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Calibration tests failed' }
    & $TrinhBienDich -std=c++17 -O2 -Wall -Wextra -Werror -static `
        -Itest/native/imu_stubs -Iinclude -Isrc -Ilib/imu/src -Ilib/estimation `
        test/native/imu_boot_calib_tests.cpp lib/imu/src/Icm20602Driver.cpp `
        src/calib/UI_Calib.cpp lib/estimation/HieuChuanNghiengDoc.cpp lib/estimation/HeTrucImu.cpp `
        -o .cache/imu_boot_calib_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'IMU boot/calibration test compilation failed' }
    & ./.cache/imu_boot_calib_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'IMU boot/calibration tests failed' }
} finally { $env:PATH = $duongDanCu; Pop-Location }
