# Kết quả tính toán tái lập từ config

Sinh bởi `tools/tinh_toan_loc_va_motor.py`; không phải số đo phần cứng.

- Trọng tâm từ trục: 0.025 m; thời gian đặc trưng mô hình điểm khối lượng: 50.490 ms.
- PT1 danh nghĩa 40 Hz @ 200 Hz: k=0.556862724, -3dB thực=27.47216143672588 Hz (None: bypass/không có điểm cắt dưới Nyquist).
- DC group delay phần mềm=3.979 ms; RMS nhiễu trắng ra/vào=0.6212.
- Fusion tau=0.4 s, beta tối đa=0.012345679; không có nghĩa cả góc bị trễ tau.
- 6400 xung/vòng, 39.2699 µm/xung; vmax=0.392699 m/s, 93.750 RPM.
- a=3 m/s² cần df/dt=76394.37 STEP/s²; torque phần tịnh tiến=0.087 N·m/bánh.
- Chưa biết điện áp nguồn, đường cong mô-men/tốc độ và trễ MKS; torque trên chưa gồm quán tính/ma sát.

| f (Hz) | Biên độ (dB) | Pha (độ) | Trễ pha (ms) |
| --- | --- | --- | --- |
| 1 | -0.006 | -1.431 | 3.976 |
| 2 | -0.024 | -2.856 | 3.967 |
| 5 | -0.150 | -7.028 | 3.904 |
| 10 | -0.569 | -13.316 | 3.699 |
| 20 | -1.892 | -22.099 | 3.069 |
| 40 | -4.735 | -26.027 | 1.807 |
| 50 | -5.864 | -23.900 | 1.328 |
| 80 | -7.903 | -10.854 | 0.377 |

Các cột này chỉ mô tả PT1; chưa gồm DLPF chip, lấy mẫu, I²C, lịch task và motor.
