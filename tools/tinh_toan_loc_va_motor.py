"""Sinh số liệu thiết kế từ config; đây là mô hình, không phải dữ liệu đo xe."""
from pathlib import Path
import cmath
import csv
import json
import math
import re

GOC = Path(__file__).resolve().parents[1]
THU_MUC_RA = GOC / "docs" / "du_lieu_tinh_toan"


def gia_tri(tep, ten):
    noi_dung = (GOC / "include/config" / tep).read_text(encoding="utf-8")
    mau = rf"(?:#define\s+{ten}\s+|constexpr\s+\w+\s+{ten}\s*=\s*)(-?\d+(?:\.\d+)?)"
    khop = re.search(mau, noi_dung)
    if not khop:
        raise ValueError(f"Không thấy hằng số: {ten}")
    return float(khop[1])


def dap_ung(he_so, tan_so_hz, tan_so_mau_hz):
    goc = 2 * math.pi * tan_so_hz / tan_so_mau_hz
    ham_truyen = he_so / (1 - (1 - he_so) * cmath.exp(-1j * goc))
    pha = cmath.phase(ham_truyen)
    return {"tan_so_hz": tan_so_hz, "bien_do": abs(ham_truyen),
            "bien_do_db": 20 * math.log10(abs(ham_truyen)),
            "pha_do": math.degrees(pha), "tre_pha_ms": -pha / (2*math.pi*tan_so_hz)*1000}


def ghi_csv(ten, cac_dong):
    with (THU_MUC_RA / ten).open("w", newline="", encoding="utf-8") as tep:
        ghi = csv.DictWriter(tep, fieldnames=list(cac_dong[0]))
        ghi.writeheader()
        ghi.writerows(cac_dong)


def main():
    THU_MUC_RA.mkdir(parents=True, exist_ok=True)
    tan_so_mau = gia_tri("ImuConfig.h", "tanSoLayMauMpu_hz")
    tan_so_loc = gia_tri("ControlConfig.h", "tanSoLocGyroD_hz")
    if tan_so_mau <= 0 or tan_so_loc < 0:
        raise ValueError("Tần số mẫu/lọc không hợp lệ")
    buoc_s = 1 / tan_so_mau
    tau = 1 / (2 * math.pi * tan_so_loc) if tan_so_loc > 0 else 0
    he_so = buoc_s / (tau + buoc_s)
    cuc = 1 - he_so
    cos_cat = (1 + cuc*cuc - 2*he_so*he_so) / (2*cuc) if cuc else -2
    tan_so_cat = math.acos(cos_cat)*tan_so_mau/(2*math.pi) if -1 <= cos_cat <= 1 else None
    tau_goc = gia_tri("ImuConfig.h", "hangSoThoiGianLocGoc_s")
    tau_van_toc = gia_tri("ControlConfig.h", "hangSoLocVanToc_s")
    ban_kinh = gia_tri("RobotGeometry.h", "banKinhBanh_m")
    chieu_cao = gia_tri("RobotGeometry.h", "toaDoTrongTamZ_m")
    khoi_luong = gia_tri("RobotGeometry.h", "khoiLuongXeKhongTai_kg")
    khoang_banh = gia_tri("RobotGeometry.h", "khoangCachTamHaiBanh_m")
    ti_so = gia_tri("RobotGeometry.h", "soVongDongCoTrenMotVongBanh")
    so_buoc = gia_tri("MotorConfig.h", "soBuocDayDuMoiVongDongCo")
    vi_buoc = gia_tri("MotorConfig.h", "soViBuocDauVaoMks")
    tan_so_toi_da = gia_tri("MotorConfig.h", "tanSoStepToiDa_hz")
    gia_toc = gia_tri("ControlConfig.h", "giaTocCanBangToiDa_mps2")
    so_xung_vong = so_buoc * vi_buoc * ti_so
    quang_duong_xung = 2*math.pi*ban_kinh/so_xung_vong
    van_toc_max = tan_so_toi_da*quang_duong_xung
    loc = {"fs_hz": tan_so_mau, "fc_danh_nghia_hz": tan_so_loc, "dt_s": buoc_s,
           "tau_s": tau, "he_so_pt1": he_so, "fc_3db_roi_rac_hz": tan_so_cat,
           "tre_nhom_dc_ms": tau*1000, "rms_white_noise_ratio": math.sqrt(he_so/(2-he_so)),
           "fusion_tau_s": tau_goc, "fusion_fc_hz": 1/(2*math.pi*tau_goc),
           "fusion_beta_toi_da": buoc_s/(tau_goc+buoc_s),
           "velocity_tau_s": tau_van_toc, "velocity_alpha": buoc_s/(tau_van_toc+buoc_s),
           "gyro_dlpf": gia_tri("ImuConfig.h", "cauHinhLocThongThapMpu"),
           "accel_dlpf": gia_tri("ImuConfig.h", "locGiaTocIcm"),
           "chip_group_delay_ms": None}
    motor = {"model_nguoi_dung": "17HS4401S", "dien_ap_nguon_V": None,
             "h_tu_truc_m": chieu_cao, "khoi_luong_kg": khoi_luong,
             "R_m": ban_kinh, "B_m": khoang_banh, "xung_vong_banh": so_xung_vong,
             "m_moi_xung": quang_duong_xung, "vmax_mps": van_toc_max,
             "rpm_banh_max": tan_so_toi_da*60/so_xung_vong,
             "luong_tu_v_moi_mau_mps": quang_duong_xung/buoc_s,
             "df_dt_step_per_s2": gia_toc/quang_duong_xung,
             "torque_tinh_tien_moi_banh_Nm": khoi_luong*gia_toc*ban_kinh/2,
             "omega_con_lac_ly_tuong_radps": math.sqrt(9.80665/chieu_cao),
             "thoi_gian_dac_trung_s": math.sqrt(chieu_cao/9.80665),
             "goc_gia_accel_3mps2_do": math.degrees(math.atan(3/9.80665)),
             "tang_norm_accel_3mps2": math.hypot(3,9.80665)-9.80665,
             "i2c_14byte_toi_thieu_100k_ms": 153/100000*1000,
             "i2c_14byte_toi_thieu_400k_ms": 153/400000*1000}
    phan_ung = [dap_ung(he_so, tan_so, tan_so_mau) for tan_so in [1,2,5,10,20,40,50,80] if tan_so < tan_so_mau/2]
    bang_banh = [{"van_toc_mps": v,"step_hz": v/quang_duong_xung,
                 "rpm": v/(2*math.pi*ban_kinh)*60} for v in [0.01,0.05,0.1,0.15,0.2,0.3,van_toc_max]]
    bang_vi_buoc = [{"vi_buoc": m,"xung_vong": so_buoc*m*ti_so,
                    "m_xung": 2*math.pi*ban_kinh/(so_buoc*m*ti_so),
                    "vmax_10k_mps": tan_so_toi_da*2*math.pi*ban_kinh/(so_buoc*m*ti_so)} for m in [16,32,64]]
    ghi_csv("dap_ung_loc.csv", phan_ung)
    ghi_csv("thong_so_banh.csv", bang_banh)
    ghi_csv("so_sanh_vi_buoc.csv", bang_vi_buoc)
    (THU_MUC_RA / "bao_cao_so.json").write_text(json.dumps({"loc":loc,"motor":motor},indent=2,ensure_ascii=False),encoding="utf-8")
    dong = ["# Kết quả tính toán tái lập từ config", "", "Sinh bởi `tools/tinh_toan_loc_va_motor.py`; không phải số đo phần cứng.",
            "", f"- Trọng tâm từ trục: {chieu_cao:.3f} m; thời gian đặc trưng mô hình điểm khối lượng: {motor['thoi_gian_dac_trung_s']*1000:.3f} ms.",
            f"- PT1 danh nghĩa {tan_so_loc:g} Hz @ {tan_so_mau:g} Hz: k={he_so:.9f}, -3dB thực={tan_so_cat} Hz (None: bypass/không có điểm cắt dưới Nyquist).",
            f"- DC group delay phần mềm={tau*1000:.3f} ms; RMS nhiễu trắng ra/vào={loc['rms_white_noise_ratio']:.4f}.",
            f"- Fusion tau={tau_goc:g} s, beta tối đa={loc['fusion_beta_toi_da']:.9f}; không có nghĩa cả góc bị trễ tau.",
            f"- {so_xung_vong:g} xung/vòng, {quang_duong_xung*1e6:.4f} µm/xung; vmax={van_toc_max:.6f} m/s, {motor['rpm_banh_max']:.3f} RPM.",
            f"- a={gia_toc:g} m/s² cần df/dt={motor['df_dt_step_per_s2']:.2f} STEP/s²; torque phần tịnh tiến={motor['torque_tinh_tien_moi_banh_Nm']:.3f} N·m/bánh.",
            "- Chưa biết điện áp nguồn, đường cong mô-men/tốc độ và trễ MKS; torque trên chưa gồm quán tính/ma sát.",
            "", "| f (Hz) | Biên độ (dB) | Pha (độ) | Trễ pha (ms) |", "| --- | --- | --- | --- |"]
    dong += [f"| {r['tan_so_hz']} | {r['bien_do_db']:.3f} | {r['pha_do']:.3f} | {r['tre_pha_ms']:.3f} |" for r in phan_ung]
    dong += ["", "Các cột này chỉ mô tả PT1; chưa gồm DLPF chip, lấy mẫu, I²C, lịch task và motor."]
    (THU_MUC_RA / "KET_QUA_TINH_TOAN.md").write_text("\n".join(dong)+"\n",encoding="utf-8")
    print(json.dumps({"loc":loc,"motor":motor},ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
