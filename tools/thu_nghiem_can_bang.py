"""Thu một lần đo Serial. Mặc định chỉ ghi. Thêm --arm --mode pid|fuzzy để thử cân bằng tại chỗ."""

import argparse
import csv
import json
import math
import re
import statistics
import sys
import time
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path


FIELDS = "t_us seq cseq cfg csess cphase mode cfault valid m_us msess mphase en mfault theta rate target eq e v vref yawref cmdL cmdR kp ki kd miss age_us drop i2c late a_cmd feedback".split()
CFG_FIELDS = "cfg mode kp0 ki0 kd0 kpv0 kiv0 bench vloop".split()
INT_FIELDS = set("t_us seq cseq cfg csess cphase mode cfault valid m_us msess mphase en mfault miss age_us drop i2c late bench vloop feedback".split())
GAIN_LIMITS = {"kp": 300, "ki": 100, "kd": 50, "kpv": 5, "kiv": 5}
COUNTERS = ("miss", "drop", "i2c", "late")
PERIOD_US = 20000  # research 1, 50 Hz
MAX_AGE_S = 0.5


class CaptureError(Exception):
    pass


def fields_to_numbers(names, cells):
    if len(names) != len(cells):
        raise CaptureError("NC sai số cột")
    result = {}
    for name, cell in zip(names, cells):
        if name in INT_FIELDS:
            if not re.fullmatch(r"-?[0-9]+", cell):
                raise CaptureError(f"NC {name} không phải số nguyên")
            value = int(cell)
            if value < (-1 if name == "age_us" else 0):
                raise CaptureError(f"NC {name} ngoài miền")
            if name in ("valid", "en", "bench", "vloop") and value not in (0, 1):
                raise CaptureError(f"NC {name} ngoài miền")
            if name in ("mode", "cphase", "mphase", "feedback") and value not in (0, 1, 2):
                raise CaptureError(f"NC {name} ngoài miền")
        else:
            try:
                value = float(cell)
            except ValueError as error:
                raise CaptureError(f"NC {name} không phải số") from error
            if not math.isfinite(value):
                raise CaptureError(f"NC {name} không hữu hạn")
        result[name] = value
    return result


class NcProtocol:
    def __init__(self):
        self.header = None
        self.cfg = None

    def parse(self, line):
        if not line.startswith(("NC,", "NC_HEADER,", "NC_CFG,")):
            return "console", line
        try:
            tag, *cells = next(csv.reader([line], strict=True))
        except csv.Error as error:
            raise CaptureError("NC CSV sai định dạng") from error
        if tag == "NC_HEADER":
            if len(cells) != len(FIELDS) or set(cells) != set(FIELDS):
                raise CaptureError("NC_HEADER khác phiên bản công cụ")
            if self.header and self.header != cells:
                raise CaptureError("NC_HEADER đổi thứ tự trong cùng lượt đo")
            self.header, self.cfg = cells, None
            return "header", cells
        if self.header is None:
            return "preheader", line  # bộ đệm có thể chứa đuôi của stream trước
        if tag == "NC_CFG":
            self.cfg = fields_to_numbers(CFG_FIELDS, cells)
            return "config", self.cfg
        sample = fields_to_numbers(self.header, cells)
        if self.cfg is None or any(self.cfg[k] != sample[k] for k in ("cfg", "mode")):
            raise CaptureError("NC thiếu NC_CFG tương ứng")
        return "sample", sample


def active(s):
    return (s["en"] == 1 and s["cphase"] == s["mphase"] == 2 and s["csess"] == s["msess"]
            and s["cfault"] == s["mfault"] == 0 and s["valid"] == 1 and 0 <= s["age_us"] < 20000
            and s["t_us"] > 0 and s["m_us"] > 0)


def guard(s, previous):
    if s["cfault"] or s["mfault"]:
        raise CaptureError(f"fault điều khiển/motor {s['cfault']}/{s['mfault']}")
    if not s["valid"] or not 0 <= s["age_us"] < 20000 or min(s["t_us"], s["m_us"]) <= 0:
        raise CaptureError("dữ liệu góc/motor hết hạn hoặc không hợp lệ")
    if abs(s["t_us"]+s["age_us"]-s["m_us"]) >= 20000:
        raise CaptureError("snapshot motor quá xa thời điểm xuất góc")
    if previous:
        dt = s["t_us"] - previous["t_us"]
        if not 0 < dt <= MAX_AGE_S * 1e6 or s["cseq"] <= previous["cseq"] or s["m_us"] < previous["m_us"]:
            raise CaptureError("thời gian/chu kỳ nguồn lặp, quay lùi hoặc mất quá 0,5 s")
        for key in COUNTERS:
            if s[key] != previous[key]:
                raise CaptureError(f"bộ đếm {key} thay đổi {previous[key]} -> {s[key]}")
    if max(abs(s["e"]), abs(s["theta"] - s["eq"])) > math.radians(8):
        raise CaptureError("sai lệch pitch vượt 8 độ")
    if abs(s["rate"]) > 1.5 or abs(s["v"]) > 0.25:
        raise CaptureError("vượt giới hạn tốc độ góc 1,5 rad/s hoặc vận tốc 0,25 m/s")
    if abs(s["vref"]) > 1e-4 or abs(s["yawref"]) > 1e-4:
        raise CaptureError("có lệnh di chuyển trong phép thử đứng yên")


def summary(runs, step_limit_hz=None):
    """runs chứa các đoạn measure liên tục của từng phiên, chỉ các mẫu active hợp lệ."""
    samples = [s for run in runs for s in run]
    n = len(samples)
    intervals = [b["t_us"] - a["t_us"] for run in runs for a, b in zip(run, run[1:])]
    duration = sum(intervals) / 1e6
    missing = sum(max(0, round(dt / PERIOD_US) - 1) for dt in intervals)
    result = dict(valid_active_measure_samples=n, observed_measure_span_s=duration,
                  longest_measure_run_s=max(((r[-1]["t_us"]-r[0]["t_us"])/1e6 for r in runs if r), default=0),
                  source_rate_hz=len(intervals) / duration if duration else None,
                  inferred_missing_slots=missing, missing_fraction=missing / (missing + n) if n else None,
                  mean_error_deg=None, rmse_error_deg=None, p95_abs_error_deg=None, max_abs_error_deg=None,
                  rms_rate_rad_s=None, wheel_command_saturation_fraction=None)
    if n:
        errors = [math.degrees(s["e"]) for s in samples]
        abs_errors = sorted(map(abs, errors))
        result.update(mean_error_deg=statistics.mean(errors), rmse_error_deg=math.sqrt(statistics.mean(e*e for e in errors)),
                      p95_abs_error_deg=abs_errors[math.ceil(0.95*n)-1], max_abs_error_deg=abs_errors[-1],
                      rms_rate_rad_s=math.sqrt(statistics.mean(s["rate"]**2 for s in samples)))
        if step_limit_hz is not None:
            result["wheel_command_saturation_fraction"] = sum(
                max(abs(s["cmdL"]), abs(s["cmdR"])) >= 0.999 * step_limit_hz for s in samples) / n
    return result


def options(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--out", required=True, type=Path, help="thư mục mới riêng cho lượt thu")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--duration", type=float, default=20, help="giây gồm 3 giây đầu ổn định")
    p.add_argument("--arm", action="store_true", help="bật motor sau khi kiểm tra telemetry và cấu hình")
    p.add_argument("--purpose", choices=("tuning", "io_dataset"), default="tuning",
                   help="io_dataset khóa PID thuần và cấu hình hiện tại khi ghi u/y")
    p.add_argument("--condition", choices=("floor-unheld", "floor-held", "wheels-raised", "unknown"),
                   default="unknown", help="điều kiện thực nghiệm do người vận hành cung cấp")
    p.add_argument("--mode", choices=("pid", "fuzzy", "siso"))
    for key in GAIN_LIMITS:
        p.add_argument("--" + key, type=float, help="đặt trong RAM trước khi arm")
    p.add_argument("--step-limit-hz", type=float, help="trần STEP/s thực của bản nạp để tính tỷ lệ bão hòa")
    args = p.parse_args(argv)
    args.gains = {k: getattr(args, k) for k in GAIN_LIMITS if getattr(args, k) is not None}
    if args.purpose == "io_dataset":
        if args.gains or args.mode in ("fuzzy", "siso"):
            p.error("io_dataset cần PID thuần đã chốt, không được đổi mode hoặc gain")
        args.mode = "pid"
    if not math.isfinite(args.duration) or args.duration <= 0 or ((args.arm or args.purpose == "io_dataset") and args.duration < 18):
        p.error("live trial cần ít nhất 18 giây (3 giây ổn định + 15 giây đo)")
    if args.arm and args.mode is None:
        p.error("--arm cần --mode pid, --mode fuzzy hoặc --mode siso")
    if not args.arm and args.purpose != "io_dataset" and (args.mode or args.gains):
        p.error("--mode và hệ số chỉ dùng khi có --arm")
    if any(not 0 <= v <= GAIN_LIMITS[k] for k, v in args.gains.items()):
        p.error("hệ số ngoài miền firmware")
    if args.baud <= 0 or (args.step_limit_hz is not None and
            (not math.isfinite(args.step_limit_hz) or args.step_limit_hz <= 0)):
        p.error("baud và trần STEP/s phải dương")
    return args


class Files:
    def __init__(self, folder, stack):
        folder.mkdir(parents=True, exist_ok=False)
        self.raw = stack.enter_context((folder/"serial.raw.log").open("wb"))
        def csv_file(name):
            return csv.writer(stack.enter_context((folder/name).open("w", encoding="utf-8-sig", newline="")))
        self.samples = csv_file("samples.csv")
        self.events = csv_file("events.csv")
        self.events.writerow(("host_utc", "host_elapsed_s", "type", "detail"))
        self.folder = folder


class Trial:
    def __init__(self, port, args, files, clock):
        self.port, self.args, self.files, self.clock = port, args, files, clock
        self.start = self.clock()
        self.protocol = NcProtocol()
        self.pending = bytearray()
        self.last = None
        self.last_host = None
        self.runs = []
        self.run = None
        self.active_start = None
        self.last_active = None
        self.cfg_history = []
        self.count = 0
        self.disarmed = None
        self.written_header = False
        self.refused = None
        self.nominal_limit = args.step_limit_hz
        self.status_lines = []
        self.first_source_us = None

    def stamp(self):
        return (datetime.now(timezone.utc).isoformat(timespec="milliseconds"), f"{self.clock()-self.start:.6f}")

    def event(self, kind, detail):
        self.files.events.writerow((*self.stamp(), kind, detail))

    def send(self, command):
        data = (command + "\n").encode("ascii")
        if self.port.write(data) != len(data):
            raise CaptureError(f"không gửi đủ lệnh {command}")
        self.event("tx", command)

    def read(self):
        chunk = self.port.readline()
        if not chunk:
            return None
        self.files.raw.write(chunk)
        self.pending.extend(chunk)
        if len(self.pending) > 8192:
            raise CaptureError("dòng Serial vượt 8192 byte")
        if not self.pending.endswith(b"\n"):
            return None
        data = bytes(self.pending).rstrip(b"\r\n")
        self.pending.clear()
        is_nc = data.startswith((b"NC,", b"NC_HEADER,", b"NC_CFG,"))
        try:
            line = data.decode("utf-8", errors="strict" if is_nc else "replace")
        except UnicodeDecodeError as error:
            raise CaptureError("dữ liệu NC chứa byte không hợp lệ") from error
        kind, value = self.protocol.parse(line)
        if kind == "header":
            self.event("header", line)
            if not self.written_header:
                self.files.samples.writerow(("host_utc", "host_elapsed_s", "phase", "t_s", "e_deg",
                                             "u_cmd_mps2", "y_pitch_rad", *value))
                self.written_header = True
        elif kind == "config":
            self.event("config", line)
            self.cfg_history.append(value.copy())
        elif kind == "console":
            self.event("console", line)
            if line.startswith(("Mode=", "PID_THUAN:", "PID_MO", "FUZZY_SISO:", "Equilibrium:", "MKS_UART", "FIRMWARE", "[STEP]")):
                self.status_lines.append(line)
            if line.startswith("[PID]") and "ngoai mien" in line:
                self.refused = line
        return value if kind == "sample" else None

    def store_sample(self, s, phase):
        if self.first_source_us is None and s["t_us"] > 0:
            self.first_source_us = s["t_us"]
        t = (s["t_us"] - self.first_source_us) / 1e6 if self.first_source_us and s["t_us"] else ""
        self.files.samples.writerow((*self.stamp(), phase, t, math.degrees(s["e"]), s["a_cmd"], s["theta"],
                                    *(s[k] for k in self.protocol.header)))
        self.count += 1
        self.last, self.last_host = s, self.clock()

    def wait_off(self, expected=None, generation=None, timeout=3):
        previous = None
        consecutive = 0
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            s = self.read()
            if self.refused:
                raise CaptureError(self.refused)
            if s is None:
                continue
            self.store_sample(s, "preflight")
            guard(s, previous)
            if s["en"] or s["mphase"] or s["cphase"] or s["csess"] != s["msess"]:
                raise CaptureError("motor chưa tắt trước khi thiết lập phép thử")
            cfg = self.protocol.cfg
            matches = expected is None or (cfg["mode"] == expected and not cfg["bench"] and cfg["vloop"]
                and (generation is None or cfg["cfg"] == generation)
                and all(abs(cfg[k+"0"]-v) <= 0.00051 for k, v in self.args.gains.items()))
            consecutive = consecutive + 1 if matches else 0
            if consecutive >= 3:
                return s
            previous = s
        raise CaptureError("hết thời gian chờ ba mẫu mới xác nhận EN=0 và NC_CFG")

    def configure(self):
        mode = {"pid": 0, "fuzzy": 1, "siso": 2}.get(self.args.mode, 0)
        if self.args.purpose == "io_dataset":
            self.wait_off(expected=0, timeout=8)
            return self.protocol.cfg.copy()
        before = self.wait_off(timeout=8)
        self.send(self.args.mode)
        suffix = "m" if mode == 1 else "p"
        for key, value in self.args.gains.items():
            self.send(f"{key}_{suffix} {value:.6f}")
        self.wait_off(expected=mode, generation=(before["cfg"] + 1 + len(self.args.gains)) % (1 << 32))
        return self.protocol.cfg.copy()

    def capture(self):
        self.send("status")
        self.send("research 1")
        armed = self.args.arm
        io_dataset = self.args.purpose == "io_dataset"
        configured = None
        previous = None
        if armed:
            configured = self.configure()
            previous = self.last
            self.send("arm")
        start = self.clock()
        first_active_host = None
        last_fresh_host = start
        active_session = None
        while True:
            now = self.clock()
            if not armed and self.count and now - start >= self.args.duration:
                if not io_dataset:
                    return
                if self.run and self.run[-1]["t_us"] - self.run[0]["t_us"] >= 15_000_000:
                    return
                if now - start >= self.args.duration + MAX_AGE_S:
                    raise CaptureError("thiếu 15 giây measure liên tục trong một phiên PID thuần")
            if now - last_fresh_host > (MAX_AGE_S if armed or self.count else 8):
                raise CaptureError("không nhận được mẫu NC mới đúng hạn")
            if armed and first_active_host is None and now - start > 1.5:
                raise CaptureError("arm chưa được firmware xác nhận EN=1 và hai phase chạy")
            if armed and first_active_host is not None and now - first_active_host > self.args.duration + MAX_AGE_S:
                raise CaptureError("đồng hồ nguồn không đủ thời gian đo")
            s = self.read()
            if s is None:
                continue
            fresh = previous is None or (s["t_us"] > previous["t_us"] and s["cseq"] > previous["cseq"])
            if fresh:
                last_fresh_host = self.clock()
            if not armed and not self.count:
                start = last_fresh_host
            if io_dataset:
                if configured is None:
                    configured = self.protocol.cfg.copy()
                if s["mode"] != 0 or self.protocol.cfg != configured:
                    self.store_sample(s, "rejected_config")
                    raise CaptureError("io_dataset yêu cầu cùng cấu hình PID thuần trong toàn bộ phép đo")
                if active_session is not None and (not fresh or not active(s) or s["msess"] != active_session
                    or s["t_us"]-previous["t_us"] > MAX_AGE_S*1e6
                    or any(s[k] != previous[k] for k in COUNTERS)):
                    self.store_sample(s, "rejected_session")
                    raise CaptureError("io_dataset mất tính liên tục của phiên PID, mẫu hoặc bộ đếm")
            if armed:
                try:
                    guard(s, previous)
                    if self.protocol.cfg != configured:
                        raise CaptureError("cấu hình đổi sau khi arm")
                    if active_session is not None and (not active(s) or s["msess"] != active_session):
                        raise CaptureError("EN, pha hoặc phiên chạy thay đổi")
                except CaptureError:
                    self.store_sample(s, "guard_trip")
                    raise
            if active(s) and fresh:
                if self.active_start is None or self.last_active != (s["msess"], s["cfg"], s["mode"]):
                    self.active_start = s["t_us"]
                    self.last_active = (s["msess"], s["cfg"], s["mode"])
                    self.run = []
                    self.runs.append(self.run)
                    self.event("active", f"session={s['msess']} t_us={s['t_us']}")
                if first_active_host is None:
                    first_active_host = self.clock()
                    active_session = s["msess"]
                elapsed = (s["t_us"] - self.active_start) / 1e6
                phase = "settle" if elapsed < 3 else "measure"
                if phase == "measure":
                    self.run.append(s)
            else:
                phase = "await_arm" if armed else "inactive"
                self.active_start = None
            self.store_sample(s, phase)
            previous = s
            if armed and phase == "measure" and elapsed >= self.args.duration and (
                    s["t_us"]-self.run[0]["t_us"] >= 15_000_000):
                return

    def finish(self):
        if self.args.arm:
            # Gửi trước mọi ghi log/đọc trạng thái khi thoát, kể cả lỗi đọc và Ctrl+C.
            self.disarmed = False
            try:
                self.send("disarm")
            except (KeyboardInterrupt, Exception):
                self.send("\ndisarm")
            deadline = self.clock() + 1
            last_motor = self.last["m_us"] if self.last else 0
            while self.clock() < deadline:
                s = self.read()
                if s is None:
                    continue
                self.store_sample(s, "after")
                if s["m_us"] > last_motor and not s["en"] and s["mphase"] == 0:
                    self.disarmed = True
                    break
        self.send("research 0")
        self.send("status")
        deadline = self.clock() + 0.35
        while self.clock() < deadline:
            s = self.read()
            if s is not None:
                self.store_sample(s, "after")
        if self.disarmed is False:
            raise CaptureError("đã gửi disarm nhưng chưa xác nhận EN=0")


def capture(port, args, files, clock=time.monotonic):
    trial = Trial(port, args, files, clock)
    errors = []
    try:
        trial.capture()
    except (KeyboardInterrupt, Exception) as error:
        errors.append(str(error) or type(error).__name__)
    finally:
        try:
            trial.finish()
        except (KeyboardInterrupt, Exception) as error:
            errors.append("kết thúc: " + (str(error) or type(error).__name__))
    for message in errors:
        trial.event("error", message)
    metrics = summary(trial.runs, trial.nominal_limit)
    measurement_valid = not errors and metrics["longest_measure_run_s"] >= 15
    metadata = dict(port=args.port, baud=args.baud, trial_type="stationary_arm" if args.arm else "record",
                    purpose=args.purpose, condition=args.condition,
                    requested_duration_s=args.duration, settle_s=3, requested_mode=args.mode,
                    requested_gains=args.gains, step_limit_hz=trial.nominal_limit,
                    motor_off_confirmed=trial.disarmed, completed=not errors, errors=errors,
                    measurement_window_complete=measurement_valid,
                    io_dataset_valid=(measurement_valid if args.purpose == "io_dataset" else None),
                    firmware_configurations=trial.cfg_history, status_lines=trial.status_lines,
                    nc_samples=trial.count, summary=metrics,
                    columns={"t_s": "seconds from first positive IMU t_us in this file", "e_deg": "degrees(theta-target)",
                             "u_cmd_mps2": "a_cmd, longitudinal acceleration command [m/s^2]",
                             "y_pitch_rad": "theta, measured pitch estimate [rad]", "target": "pitch reference [rad]",
                             "eq": "pitch at equilibrium [rad]", "rate": "pitch rate [rad/s]", "v": "estimated speed [m/s]",
                             "cmdL": "left wheel STEP/s", "cmdR": "right wheel STEP/s",
                             "feedback": "0 unavailable, 1 STEP, 2 encoder"},
                    notes="phase/mode/EN lấy từ firmware. Không xác định xe được giữ, đặt trên sàn hay nhấc bánh từ Serial. "
                          "Missing slots ước tính từ khoảng cách timestamp nguồn trong từng đoạn measure active. "
                          "Bão hòa là |cmdL| hoặc |cmdR| >=99,9% trần tần số chỉ định.")
    (files.folder/"metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return metadata


def main(argv=None):
    args = options(argv)
    try:
        import serial
        with ExitStack() as stack:
            files = Files(args.out, stack)
            # Đặt đường điều khiển trước open để tránh xung reset do RTS/DTR trên cổng USB-UART.
            port = serial.Serial(port=None, baudrate=args.baud, timeout=0.05, write_timeout=0.25)
            port.dtr = True  # USB CDC của ESP32-S3 trên xe cần DTR để gửi dữ liệu tới host.
            port.rts = False
            port.port = args.port
            port.open()
            stack.callback(port.close)
            result = capture(port, args, files)
        print(json.dumps({"completed": result["completed"], "errors": result["errors"], "summary": result["summary"]}))
        return 0 if result["completed"] else 1
    except (ImportError, OSError, ValueError) as error:
        print(f"Thu Serial: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
    raise SystemExit(main())
