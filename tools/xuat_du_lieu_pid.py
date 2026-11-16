"""Xuất u/y từ một lượt io_dataset PID thuần đã thu trên sàn, không giữ tay."""

import argparse
import csv
import hashlib
import io
import json
import math
from datetime import datetime, timezone
from pathlib import Path

try:
    from .thu_nghiem_can_bang import CFG_FIELDS, COUNTERS, FIELDS, CaptureError, NcProtocol, active, fields_to_numbers
except ImportError:  # Chạy trực tiếp python tools/xuat_du_lieu_pid.py.
    from thu_nghiem_can_bang import CFG_FIELDS, COUNTERS, FIELDS, CaptureError, NcProtocol, active, fields_to_numbers


class ExportError(ValueError):
    pass


SIMULATION_COLUMNS = (
    "t_s", "u_left_steps_per_s", "u_right_steps_per_s", "u_common_steps_per_s",
    "a_cmd_mps2", "y_pitch_rad", "y_pitch_deg", "target_pitch_rad", "e_rad", "pitch_rate_rad_per_s",
)


def require(condition, message):
    if not condition:
        raise ExportError(message)


def sample_key(sample):
    return tuple(sample[k] for k in ("t_us", "cseq", "cfg", "csess", "m_us"))


def read_capture(folder):
    paths = {name: folder/name for name in ("metadata.json", "samples.csv", "serial.raw.log", "events.csv")}
    blobs = {name: path.read_bytes() for name, path in paths.items()}
    meta = json.loads(blobs["metadata.json"].decode("utf-8-sig"))
    require(meta.get("purpose") == "io_dataset", "Nguồn phải được thu với --purpose io_dataset")
    require(meta.get("condition") == "floor-unheld", "Cần điều kiện floor-unheld do người vận hành xác nhận")
    require(all(meta.get(k) is True for k in ("completed", "io_dataset_valid", "measurement_window_complete")),
            "Lượt thu chưa xác nhận hoàn thành cửa sổ đo hợp lệ")
    require(meta.get("errors") == [], "Lượt thu có lỗi")
    require(meta.get("requested_mode") == "pid", "Lượt thu phải yêu cầu PID thuần")
    require(meta.get("trial_type") in ("record", "stationary_arm"), "Thiếu loại lượt thu")
    if meta["trial_type"] == "stationary_arm":
        require(meta.get("motor_off_confirmed") is True, "Lượt thu chủ động chưa xác nhận motor tắt")
    reader = csv.DictReader(io.StringIO(blobs["samples.csv"].decode("utf-8-sig")), strict=True)
    require(reader.fieldnames is not None and len(reader.fieldnames) == len(set(reader.fieldnames)),
            "CSV thiếu header hoặc có cột trùng")
    require(set(FIELDS + ["phase", "host_utc", "host_elapsed_s"]) <= set(reader.fieldnames),
            "CSV thiếu trường nguồn")
    source_rows = list(reader)
    require(source_rows and meta.get("nc_samples") == len(source_rows), "Số dòng nguồn khác metadata")
    rows = []
    in_measure = False
    measurement_ended = False
    for number, row in enumerate(source_rows, 2):
        require(None not in row and all(v is not None for v in row.values()), f"CSV sai số cột ở dòng {number}")
        phase = row["phase"]
        require(phase in ("preflight", "await_arm", "inactive", "settle", "measure", "after"),
                f"CSV có pha bị loại ở dòng {number}")
        values = fields_to_numbers(FIELDS, [row[k] for k in FIELDS])
        if phase != "measure":
            measurement_ended |= in_measure
            continue
        require(not measurement_ended, "Cửa sổ measure bị ngắt thành nhiều đoạn")
        in_measure = True
        require(active(values) and values["mode"] == 0, f"Dòng measure {number} không phải PID đang chạy hợp lệ")
        rows.append((values, row["host_utc"], row["host_elapsed_s"]))
    require(len(rows) >= 2, "Không đủ dòng measure")
    first = rows[0][0]
    duration = (rows[-1][0]["t_us"] - first["t_us"]) / 1e6
    require(duration >= 15, "Cửa sổ measure phải dài ít nhất 15 giây theo timestamp nguồn")
    for i, (s, _, host_s) in enumerate(rows):
        require(all(s[k] == first[k] for k in ("mode", "cfg", "csess", "msess", "feedback", *COUNTERS)),
                "Cửa sổ measure đổi mode, cấu hình, phiên, nguồn phản hồi hoặc bộ đếm lỗi")
        require(s["feedback"] in (1, 2), "Thiếu nguồn phản hồi chuyển động")
        require(math.isfinite(float(host_s)), "Timestamp host không hữu hạn")
        require(abs(s["e"] - (s["theta"]-s["target"])) <= 1e-6, "Sai lệch e khác theta-target")
        require(abs(s["t_us"]+s["age_us"]-s["m_us"]) < 20000, "Snapshot motor hết hạn")
        if i:
            previous = rows[i-1][0]
            require(0 < s["t_us"]-previous["t_us"] <= 500000
                    and s["cseq"] > previous["cseq"] and s["seq"] > previous["seq"]
                    and s["m_us"] > previous["m_us"], "Timestamp/sequence nguồn lặp, quay lùi hoặc có khoảng trống quá 0,5 giây")
    configurations = []
    for cfg in meta.get("firmware_configurations", []):
        parsed = fields_to_numbers(CFG_FIELDS, [str(cfg[k]) for k in CFG_FIELDS])
        if parsed["cfg"] == first["cfg"] and parsed["mode"] == 0:
            configurations.append(parsed)
    require(configurations, "Thiếu NC_CFG tương ứng trong metadata")
    cfg = configurations[-1]
    require(all(c == cfg for c in configurations) and cfg["bench"] == 0 and cfg["vloop"] == 1,
            "NC_CFG không cố định hoặc chế độ cân bằng trên sàn không phù hợp")
    require(all(cfg[k] >= 0 for k in ("kp0", "ki0", "kd0", "kpv0", "kiv0")), "Hệ số PID âm")
    for s, _, _ in rows:
        require(all(math.isclose(s[k], cfg[k+"0"], rel_tol=2e-5, abs_tol=1e-5) for k in ("kp", "ki", "kd")),
                "Hệ số PID đang chạy khác hệ số cơ sở đã chốt")

    # Xác thực từng dòng measure với byte Serial lưu gốc, gồm đúng NC_CFG đi trước mẫu.
    expected = {sample_key(s): s for s, _, _ in rows}
    require(len(expected) == len(rows), "Lặp mẫu trong cửa sổ measure")
    parser = NcProtocol()
    matched = set()
    for raw in blobs["serial.raw.log"].splitlines():
        is_nc = raw.startswith((b"NC,", b"NC_HEADER,", b"NC_CFG,"))
        line = raw.decode("utf-8", errors="strict" if is_nc else "replace")
        kind, value = parser.parse(line)
        if kind == "sample" and sample_key(value) in expected:
            key = sample_key(value)
            require(key not in matched and value == expected[key] and parser.cfg == cfg,
                    "Dữ liệu CSV hoặc NC_CFG khác Serial gốc")
            matched.add(key)
    require(len(matched) == len(rows), "Có dòng measure không tìm thấy trong Serial gốc")
    trace = {name: {"path": str(paths[name].resolve()), "sha256": hashlib.sha256(blob).hexdigest(),
                    "bytes": len(blob)} for name, blob in blobs.items()}
    return rows, cfg, meta, trace


def export_row(s, host_utc, host_s, first_us, cfg):
    return dict(
        t_s=(s["t_us"]-first_us)/1e6, imu_source_us=s["t_us"], motor_snapshot_us=s["m_us"],
        host_utc=host_utc, host_elapsed_s=host_s,
        u_left_steps_per_s=s["cmdL"], u_right_steps_per_s=s["cmdR"],
        u_common_steps_per_s=(s["cmdL"]+s["cmdR"])/2, u_diff_steps_per_s=(s["cmdR"]-s["cmdL"])/2,
        a_cmd_mps2=s["a_cmd"], y_pitch_rad=s["theta"], y_pitch_deg=math.degrees(s["theta"]),
        target_pitch_rad=s["target"], equilibrium_pitch_rad=s["eq"], e_rad=s["e"], e_deg=math.degrees(s["e"]),
        pitch_rate_rad_per_s=s["rate"], estimated_v_mps=s["v"],
        v_reference_mps=s["vref"], yaw_reference_rad_per_s=s["yawref"],
        kp_active=s["kp"], ki_active=s["ki"], kd_active=s["kd"],
        kp_base=cfg["kp0"], ki_base=cfg["ki0"], kd_base=cfg["kd0"],
        kpv_base=cfg["kpv0"], kiv_base=cfg["kiv0"], mode_id=s["mode"], config_id=s["cfg"],
        control_session_id=s["csess"], motor_session_id=s["msess"], imu_sequence=s["seq"],
        control_sequence=s["cseq"], feedback_id=s["feedback"], angle_age_us=s["age_us"],
        missed_imu_samples=s["miss"], dropped_telemetry_samples=s["drop"], i2c_errors=s["i2c"],
        late_imu_reads=s["late"])


def export(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    require(source.is_dir(), "Thư mục nguồn không tồn tại")
    require(not destination.exists(), "Thư mục xuất phải mới để tránh ghi đè")
    rows, cfg, source_meta, trace = read_capture(source)
    output = [export_row(s, utc, host_s, rows[0][0]["t_us"], cfg) for s, utc, host_s in rows]
    csv_buffer = io.StringIO(newline="")
    writer = csv.DictWriter(csv_buffer, fieldnames=list(output[0]))
    writer.writeheader()
    writer.writerows(output)
    payload = csv_buffer.getvalue().encode("utf-8-sig")
    simulation_buffer = io.StringIO(newline="")
    simulation_writer = csv.DictWriter(simulation_buffer, fieldnames=SIMULATION_COLUMNS, extrasaction="ignore")
    simulation_writer.writeheader()
    simulation_writer.writerows(output)
    simulation_payload = simulation_buffer.getvalue().encode("utf-8-sig")
    duration = output[-1]["t_s"]
    meta = dict(format_version=1, created_utc=datetime.now(timezone.utc).isoformat(),
                source_folder=str(source), source_files=trace, purpose="io_dataset", controller="PID_THUAN",
                condition=source_meta["condition"], condition_source="operator-declared",
                selection="all rows with phase=measure, in original order", rows=len(output),
                duration_s=duration, effective_sample_rate_hz=(len(output)-1)/duration,
                nominal_sample_rate_hz=50, base_configuration=cfg,
                csv_file="data_pid_io.csv", csv_sha256=hashlib.sha256(payload).hexdigest(),
                simulation_csv_file="pid_u_y.csv",
                simulation_csv_sha256=hashlib.sha256(simulation_payload).hexdigest(),
                column_definitions={
                    "t_s": "Source IMU time rebased to zero at the first exported measure row [s].",
                    "imu_source_us": "Firmware IMU source timestamp [us].",
                    "motor_snapshot_us": "Timestamp of the separately sampled motor state [us].",
                    "u_left_steps_per_s": "Left wheel STEP frequency command, signed [STEP/s].",
                    "u_right_steps_per_s": "Right wheel STEP frequency command, signed [STEP/s].",
                    "u_common_steps_per_s": "(u_left + u_right) / 2 [STEP/s].",
                    "u_diff_steps_per_s": "(u_right - u_left) / 2 [STEP/s].",
                    "a_cmd_mps2": "Longitudinal acceleration command after speed limit, before individual wheel correction [m/s^2].",
                    "y_pitch_rad": "IMU estimator pitch output [rad].",
                    "e_rad": "y_pitch_rad - target_pitch_rad [rad].",
                    "feedback_id": "1=STEP pulse estimate, 2=wheel encoder.",
                    "mode_id": "0=PID_THUAN.",
                },
                remarks=[
                    "Closed-loop balance measurements with the fixed PID configuration recorded in NC_CFG.",
                    "u_* and a_cmd are commanded quantities. Actual wheel acceleration is not measured by these columns.",
                    "Pitch is an IMU estimate. estimated_v_mps uses the source identified by feedback_id.",
                    "Controller data and motor-state data have separate timestamps.",
                    "Original timestamps and row order are preserved. No resampling or interpolation is applied.",
                ])
    destination.mkdir(parents=True, exist_ok=False)
    (destination/"data_pid_io.csv").write_bytes(payload)
    (destination/"pid_u_y.csv").write_bytes(simulation_payload)
    (destination/"metadata.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return meta


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        result = export(args.source, args.out)
    except (OSError, ValueError, KeyError, TypeError, CaptureError, csv.Error) as error:
        print(json.dumps({"ok": False, "error": str(error)}))
        return 1
    print(json.dumps({"ok": True, "rows": result["rows"], "duration_s": result["duration_s"],
                      "csv": str(args.out.resolve()/"data_pid_io.csv")}))
    return 0


if __name__ == "__main__":
    import sys
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
    raise SystemExit(main())
