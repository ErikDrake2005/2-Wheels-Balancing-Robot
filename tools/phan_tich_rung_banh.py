"""Đối chiếu rung và tốc độ bánh từ một thư mục do thu_rung_lsm6dsv.py thu."""
import argparse
import csv
import itertools
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path


PERIOD_US = 1_000_000
U32 = 1 << 32
KEY = ("boot_id", "bridge_session", "log_id", "cfg")
FLOATS = {"rate", "vref", "yawref", "v_request", "yaw_request"}
IMU = {"t_us", "flags", "sample_seq", "mode", "diag_flags"} | FLOATS
WHEELS = {"t_us", "motor_us", "en", "angle_valid", "motor_session", "diag_session", "packet_seq",
          "steps_l", "steps_r", "enc_l", "enc_r", "uart_rx_l", "uart_rx_r", "uart_age_l_us", "uart_age_r_us"}


def load_windows(path, fields):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = fields | set(KEY)
        if not fields.issubset(reader.fieldnames or []):
            raise ValueError(f"{path.name} thiếu cột {sorted(fields - set(reader.fieldnames or []))}")

        def records():
            for data in reader:
                try:
                    row = {k: float(data[k]) if k in FLOATS else int(data[k]) for k in fields}
                    if not all(math.isfinite(row[k]) for k in fields & FLOATS):
                        raise ValueError("NaN/Inf")
                except (ValueError, TypeError) as exc:
                    raise ValueError(f"{path.name} dòng {reader.line_num}: {exc}") from exc
                yield row

        def key(r):
            return tuple(r[k] for k in KEY) + (r["t_us"] // PERIOD_US,)

        for group, rows in itertools.groupby(records(), key):
            yield group, list(rows)


def continuous(rows, sequence, clock, max_gap_us):
    return len(rows) >= 2 and rows[-1][clock] - rows[0][clock] >= 850_000 and all(
        (b[sequence] - a[sequence]) % U32 == 1 and 0 < b[clock] - a[clock] <= max_gap_us
        for a, b in zip(rows, rows[1:]))


def wheel_window(rows, meters_per_step):
    if not all(r["en"] == 1 and r["angle_valid"] == 1 for r in rows):
        return "wheel_inactive", {}
    if len({(r["diag_session"], r["motor_session"]) for r in rows}) != 1:
        return "wheel_session_change", {}
    if any(abs(r["motor_us"] - r["t_us"]) > 30_000 for r in rows):
        return "wheel_imu_time_skew", {}
    if not continuous(rows, "packet_seq", "motor_us", 60_000):
        return "wheel_gap_or_short", {}
    first, last = rows[0], rows[-1]
    result = {}
    for side in ("l", "r"):
        age, rx = "uart_age_" + side + "_us", "uart_rx_" + side
        if any(not 0 <= r[age] <= 15_000 or r["motor_us"] < r[age] for r in rows):
            return "encoder_stale", {}
        if any(not 0 < (b[rx] - a[rx]) % U32 < U32//2 for a, b in zip(rows, rows[1:])):
            return "encoder_counter_stale_or_reset", {}
        def speeds(a, b):
            dt_pulse = (b["motor_us"] - a["motor_us"]) * 1e-6
            # UART ages are measured at telemetry creation, shortly after the motor snapshot.
            dt_encoder = dt_pulse + (a[age] - b[age]) * 1e-6
            pulse = (b["steps_" + side] - a["steps_" + side]) * meters_per_step / dt_pulse
            rotor = (b["enc_" + side] - a["enc_" + side]) * meters_per_step / dt_encoder
            return pulse, rotor
        pulse, rotor = speeds(first, last)
        squared_error, duration = 0., 0
        start = first
        for end in rows[1:]:
            dt = end["motor_us"] - start["motor_us"]
            if dt < 50_000:
                continue
            target, actual = speeds(start, end)
            squared_error += (actual - target)**2 * dt
            duration += dt
            start = end
        result.update({f"step_{side}_mps": pulse, f"encoder_{side}_mps": rotor,
                       f"error_{side}_mps": rotor - pulse,
                       f"block_error_rms_{side}_mps": math.sqrt(squared_error / duration)})
    return "ok", result


def summarize(run, radius_m=.04, steps_per_rev=12800, windows_csv=None):
    if not math.isfinite(radius_m) or radius_m <= 0 or steps_per_rev <= 0:
        raise ValueError("Bán kính và số xung mỗi vòng bánh phải dương")
    meters_per_step = 2 * math.pi * radius_m / steps_per_rev
    wheels = {}
    wheel_path = run / "wheels.csv"
    if wheel_path.exists():
        for key, rows in load_windows(wheel_path, WHEELS):
            wheels[key] = ("wheel_reordered", {}) if key in wheels else wheel_window(rows, meters_per_step)

    groups = defaultdict(list)
    rejected, wheel_quality = Counter(), Counter()
    columns = [*KEY, "start_us", "mode", "diag_flags", "phase", "direction", "speed_bin_low_mps",
               "vref_mean_mps", "rate_rms_ac_radps", "wheel_quality",
               "step_l_mps", "encoder_l_mps", "error_l_mps", "block_error_rms_l_mps",
               "step_r_mps", "encoder_r_mps", "error_r_mps", "block_error_rms_r_mps"]
    writer = csv.DictWriter(windows_csv, columns) if windows_csv else None
    if writer: writer.writeheader()
    for key, rows in load_windows(run / "samples.csv", IMU):
        if not continuous(rows, "sample_seq", "t_us", 10_000):
            rejected["imu_gap_or_short"] += 1
            continue
        if not all(r["flags"] & 3 == 3 for r in rows) or len({(r["mode"], r["diag_flags"]) for r in rows}) != 1:
            rejected["imu_inactive_or_config_change"] += 1
            continue
        refs = [r["vref"] for r in rows]
        requests = [r["v_request"] for r in rows]
        speed = statistics.mean(refs)
        if any(max(abs(r["yawref"]), abs(r["yaw_request"])) > .03 for r in rows):
            phase = "turn"
        elif max(refs) - min(refs) > .025 or max(requests) - min(requests) > .025:
            phase = "accel_decel"
        else:
            phase = "stand" if max(map(abs, refs)) < .02 else "straight_cruise"
        direction = "near_zero" if abs(speed) < .02 else ("forward" if speed > 0 else "reverse")
        speed_bin = math.floor((abs(speed) + 1e-9) / .05) / 20
        row = dict(zip(KEY, key[:4]))
        quality, motion = wheels.get(key, ("wheel_missing", {}))
        wheel_quality[quality] += 1
        row.update(start_us=key[-1] * PERIOD_US, mode=rows[0]["mode"], diag_flags=rows[0]["diag_flags"],
                   phase=phase, direction=direction, speed_bin_low_mps=speed_bin, vref_mean_mps=speed,
                   rate_rms_ac_radps=statistics.pstdev(r["rate"] for r in rows), wheel_quality=quality, **motion)
        if writer: writer.writerow(row)
        group = tuple(row[k] for k in (*KEY, "mode", "diag_flags", "phase", "direction", "speed_bin_low_mps"))
        groups[group].append(row)
    bins = []
    for key, items in sorted(groups.items()):
        r = dict(zip((*KEY, "mode", "diag_flags", "phase", "direction", "speed_bin_low_mps"), key))
        r["windows"] = len(items)
        r["pitch_rate_rms_ac_median_radps"] = statistics.median(x["rate_rms_ac_radps"] for x in items)
        matched = [x for x in items if x["wheel_quality"] == "ok"]
        r["matched_wheel_windows"] = len(matched)
        if matched:
            for side in ("l", "r"):
                r[f"median_abs_speed_error_{side}_mps"] = statistics.median(abs(x[f"error_{side}_mps"]) for x in matched)
                r[f"median_block_error_rms_{side}_mps"] = statistics.median(x[f"block_error_rms_{side}_mps"] for x in matched)
        bins.append(r)
    return {"window_s": 1, "radius_m": radius_m, "steps_per_wheel_rev": steps_per_rev,
            "max_uart_age_us": 15000, "rejected_imu_windows": dict(rejected),
            "wheel_window_quality": dict(wheel_quality), "bins": bins}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="Thư mục chứa samples.csv và wheels.csv")
    parser.add_argument("--windows-csv", type=Path, help="Ghi chi tiết mỗi giây vào tệp CSV mới")
    parser.add_argument("--radius-m", type=float, default=.04)
    parser.add_argument("--steps-per-wheel-rev", type=int, default=12800)
    args = parser.parse_args()
    if args.windows_csv:
        with args.windows_csv.open("x", encoding="utf-8-sig", newline="") as table:
            result = summarize(args.run, args.radius_m, args.steps_per_wheel_rev, table)
    else:
        result = summarize(args.run, args.radius_m, args.steps_per_wheel_rev)
    print(json.dumps(result, ensure_ascii=False, indent=2))
