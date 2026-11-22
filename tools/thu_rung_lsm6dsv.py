"""Thu log rung LSM6DSV liên tục qua ESP32 thu USB. Dừng bằng Ctrl+C."""
import argparse
import csv
import json
import math
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


SAMPLE_FIELDS = """bridge_rx_us boot_id bridge_session packet_seq log_id sample_index sample_hz imu
queue_drop tx_drop t_us command_age_us sample_seq cfg forward_pct turn_pct flags mode diag_flags
source speed_pct ax ay az gx gy gz temp_c theta rate target gyro_d vref yawref cmd_l cmd_r a_cmd v
v_request yaw_request rssi_dbm receiver_queue_drop receiver_reject receiver_packet_gap serial_drop
serial_crc16""".split()
WHEEL_FIELDS = """bridge_rx_us boot_id bridge_session diag_session packet_seq log_id t_us motor_us sample_seq
cfg motor_session en angle_valid cmd_l cmd_r steps_l steps_r enc_l enc_r uart_rx_l uart_rx_r
uart_age_l_us uart_age_r_us diag_packet_gap rssi_dbm serial_drop serial_crc16""".split()
INFO_FIELDS = """bridge_rx_us boot_id bridge_session packet_seq log_id t_us produced queue_drop tx_drop
tx_fail spi_hz imu_bus_errors imu_missed imu_late sample_hz enabled imu sck mosi miso cs interrupt
gyro_lpf accel_lpf filter_flags accel_range_g gyro_range_dps eq kp ki kd kpv kiv fusion_tau d_lpf_hz
cfg gyro_bias_x_dps gyro_bias_y_dps gyro_bias_z_dps trust_norm trust_start trust_end trust_timeout
mount00 mount01 mount02 mount10 mount11 mount12 mount20 mount21 mount22 firmware rssi_dbm
receiver_queue_drop receiver_reject receiver_packet_gap serial_drop serial_crc16""".split()
SAMPLE_FLOATS = set("ax ay az gx gy gz temp_c theta rate target gyro_d vref yawref cmd_l cmd_r a_cmd v v_request yaw_request".split())
INFO_FLOATS = set("eq kp ki kd kpv kiv fusion_tau d_lpf_hz gyro_bias_x_dps gyro_bias_y_dps gyro_bias_z_dps trust_norm trust_start trust_end trust_timeout mount00 mount01 mount02 mount10 mount11 mount12 mount20 mount21 mount22".split())
SCHEMAS = {"VR": SAMPLE_FIELDS, "VI": INFO_FIELDS, "VW": WHEEL_FIELDS}
FLOATS = {"VR": SAMPLE_FLOATS, "VI": INFO_FLOATS, "VW": {"cmd_l", "cmd_r"}}
SIGNED_FIELDS = {"forward_pct", "turn_pct", "rssi_dbm", "sck", "mosi", "miso", "cs", "interrupt"}
U64_FIELDS = {"bridge_rx_us", "t_us", "motor_us"}
I64_FIELDS = {"steps_l", "steps_r", "enc_l", "enc_r"}
HOST_FIELDS = ["host_utc", "host_elapsed_s"]
INTEGER = re.compile(r"-?[0-9]+\Z")
FIRMWARE = re.compile(r"[A-Za-z0-9_.+-]{1,31}\Z")
SOURCE_NAMES = {0: "none", 1: "web", 2: "other", 3: "remote"}
MAX_LINE_BYTES = 4096


class FrameError(ValueError):
    pass


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def parse_header(line):
    for kind in SCHEMAS:
        marker = kind + "_HEADER,"
        at = line.find(marker)
        if at < 0:
            continue
        fields = line[at + len(marker):].split(",")
        if len(fields) != len(set(fields)) or set(fields) != set(SCHEMAS[kind]) or fields[-1] != "serial_crc16":
            raise FrameError("Header không đúng schema " + kind)
        return kind, fields
    return None


def parse_frame(line, headers):
    kind = line.split(",", 1)[0]
    if kind not in SCHEMAS:
        return None
    if kind not in headers:
        return kind, None
    body, separator, checksum = line.rpartition(",")
    try:
        expected = int(checksum)
        if not separator or not 0 <= expected <= 65535 or crc16(body.encode("ascii")) != expected:
            raise FrameError("CRC dòng USB không khớp")
    except (ValueError, UnicodeEncodeError) as error:
        raise FrameError("CRC/byte dòng USB không hợp lệ") from error
    values = line.split(",")[1:]
    fields = headers[kind]
    if len(values) != len(fields):
        raise FrameError("Số cột không khớp header")
    parsed = {}
    for key, value in zip(fields, values):
        if key == "firmware":
            if not FIRMWARE.fullmatch(value):
                raise FrameError("Mã firmware không hợp lệ")
            parsed[key] = value
        elif key in FLOATS[kind]:
            try:
                number = float(value)
            except ValueError as error:
                raise FrameError("Giá trị không phải số " + key) from error
            if not math.isfinite(number):
                raise FrameError("NaN/Inf tại " + key)
            parsed[key] = number
        else:
            if not INTEGER.fullmatch(value):
                raise FrameError("Số nguyên không hợp lệ " + key)
            number = int(value)
            if key in I64_FIELDS:
                if not -(1 << 63) <= number < (1 << 63):
                    raise FrameError("Số xung ngoài miền " + key)
            else:
                limit = (1 << 64) - 1 if key in U64_FIELDS else (1 << 32) - 1
                if number > limit or (number < 0 and key not in SIGNED_FIELDS):
                    raise FrameError("Số nguyên ngoài miền " + key)
            parsed[key] = number
    if not parsed["boot_id"] or not parsed["bridge_session"] or not parsed["packet_seq"] or not parsed["t_us"]:
        raise FrameError("Thiếu boot/session/sequence/time nguồn")
    if kind != "VW" and (parsed["imu"] != 1 or not 1 <= parsed["sample_hz"] <= 1000):
        raise FrameError("Loại IMU hoặc nhịp mẫu không phù hợp")
    if not -128 <= parsed["rssi_dbm"] <= 0:
        raise FrameError("RSSI ngoài miền")
    if kind == "VR":
        if not parsed["log_id"] or parsed["sample_index"] not in (0, 1):
            raise FrameError("Phiên log hoặc sample_index không hợp lệ")
        if not (-100 <= parsed["forward_pct"] <= 100 and -100 <= parsed["turn_pct"] <= 100 and 0 <= parsed["speed_pct"] <= 100):
            raise FrameError("Mức điều khiển ngoài miền")
        if parsed["flags"] & ~15 or parsed["diag_flags"] & ~7 or parsed["mode"] not in (0, 1) or parsed["source"] not in SOURCE_NAMES:
            raise FrameError("Cờ hoặc nguồn điều khiển ngoài miền")
    elif kind == "VW":
        if not parsed["diag_session"] or not parsed["motor_us"] or not parsed["log_id"] or \
                parsed["en"] not in (0, 1) or parsed["angle_valid"] not in (0, 1):
            raise FrameError("Phiên, thời gian hoặc trạng thái bánh không hợp lệ")
    elif parsed["enabled"] not in (0, 1):
        raise FrameError("Cờ bật log không hợp lệ")
    return kind, parsed


class Lines:
    """Bounded newline framing. Discard a whole overlong line until its newline."""
    def __init__(self):
        self.pending = bytearray()
        self.discarding = False
        self.too_long = 0

    def add(self, chunk):
        lines = []
        for part in chunk.splitlines(keepends=True):
            end = part.endswith(b"\n")
            if not self.discarding:
                if len(self.pending) + len(part) > MAX_LINE_BYTES:
                    self.pending.clear()
                    self.discarding = True
                    self.too_long += 1
                else:
                    self.pending.extend(part)
            if end:
                if not self.discarding:
                    lines.append(bytes(self.pending).rstrip(b"\r\n").decode("utf-8", errors="replace"))
                self.pending.clear()
                self.discarding = False
        return lines


class Tracker:
    def __init__(self):
        self.groups = {}
        self.duplicates = 0
        self.out_of_order = 0

    def accept(self, sample):
        key = (sample["boot_id"], sample["bridge_session"], sample["log_id"])
        group = self.groups.get(key)
        if group is None:
            self.groups[key] = {
                "boot_id": key[0], "bridge_session": key[1], "log_id": key[2],
                "rows": 1, "first_t_us": sample["t_us"], "last_t_us": sample["t_us"],
                "first_sample_seq": sample["sample_seq"], "last_sample_seq": sample["sample_seq"],
                "source_sequence_gaps": 0, "largest_gap_samples": 0,
                "observed_sample_hz": sample["sample_hz"],
            }
            return True
        delta = (sample["sample_seq"] - group["last_sample_seq"]) & 0xFFFFFFFF
        if delta == 0:
            self.duplicates += 1
            return False
        if delta >= 0x80000000 or sample["t_us"] <= group["last_t_us"]:
            self.out_of_order += 1
            return False
        gap = delta - 1
        group["source_sequence_gaps"] += gap
        group["largest_gap_samples"] = max(gap, group["largest_gap_samples"])
        group["last_t_us"] = sample["t_us"]
        group["last_sample_seq"] = sample["sample_seq"]
        group["rows"] += 1
        return True

    def summary(self):
        result = []
        for value in self.groups.values():
            group = dict(value)
            duration = (group["last_t_us"] - group["first_t_us"]) / 1e6
            group["source_duration_s"] = duration
            group["received_rows_per_second"] = (group["rows"] - 1) / duration if duration > 0 else None
            result.append(group)
        return result


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_metadata(path, metadata):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def options(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="cổng USB ESP32 thu, ví dụ COM5")
    parser.add_argument("--out", type=Path, required=True, help="thư mục phiên mới, chưa tồn tại")
    parser.add_argument("--seconds", type=float, default=0, help="0: liên tục đến Ctrl+C")
    parser.add_argument("--native-usb", action="store_true", help="chỉ bật cho ESP32-S3 USB native")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("seconds phải là số hữu hạn >=0")
    return args


def main(argv=None):
    args = options(argv)
    import serial

    args.out.mkdir(parents=True, exist_ok=False)
    start = time.monotonic()
    headers = {}
    lines = Lines()
    tracker = Tracker()
    row_counts = {"VR": "sample_rows", "VI": "config_rows", "VW": "wheel_rows"}
    counters = {"sample_rows": 0, "config_rows": 0, "wheel_rows": 0, "invalid_frames": 0, "invalid_headers": 0,
                "preheader_frames": 0, "other_lines": 0, "reconnect_attempts": 0, "serial_errors": 0}
    serial_errors = []
    end_reason = "duration"
    started_utc = utc_now()
    metadata_path = args.out / "metadata.json"
    port = None
    last_flush = last_metadata = start
    next_headers = next_open = start
    last_info_state = None

    def record_serial_error(error):
        counters["serial_errors"] += 1
        serial_errors.append({"host_utc": utc_now(), "error": str(error)})
        del serial_errors[:-100]

    def metadata(final=False):
        return {
            "format": "LSM6DSV_ESP_NOW_XVR1", "started_utc": started_utc,
            "updated_utc": utc_now(), "port": args.port, "baud": 921600,
            "elapsed_s": time.monotonic() - start, "finalized": final,
            "end_reason": end_reason if final else "running", "counters": dict(counters),
            "overlong_lines": lines.too_long, "duplicate_samples": tracker.duplicates,
            "out_of_order_samples": tracker.out_of_order, "serial_errors": serial_errors[-100:],
            "groups": tracker.summary(), "headers": dict(headers),
            "units": {"t_us": "robot monotonic microseconds since boot", "bridge_rx_us": "receiver monotonic microseconds",
                      "host_utc": "PC UTC arrival time", "ax_ay_az": "sensor-frame m/s^2, gravity included",
                      "gx_gy_gz": "sensor-frame rad/s, gyro bias removed", "theta_rate_target": "rad, rad/s, rad",
                      "cmd_l_cmd_r": "STEP/s", "a_cmd": "m/s^2", "v_vref_v_request": "m/s",
                      "yawref_yaw_request": "rad/s", "temp_c": "degree C",
                      "motor_us": "robot time of motor snapshot, microseconds",
                      "steps_l_steps_r": "signed cumulative STEP pulses", "enc_l_enc_r": "MKS UART position in equivalent STEP pulses"},
            "ui": {"forward_pct": "+100 forward, -100 backward; UI before shaping",
                   "turn_pct": "+100 left, -100 right; UI before shaping",
                   "speed_pct": "UI speed scale 0..100", "flags": {"1": "motor on", "2": "angle valid", "4": "command valid", "8": "UI levels valid"},
                   "source": SOURCE_NAMES},
            "notes": [
                "Use robot t_us for spectra and delay analysis, not USB/PC arrival intervals.",
                "Each boot_id + bridge_session + log_id is a separate continuous-log group.",
                "Source gaps within a group include lost IMU/control/log/RF/USB samples; they are not automatically RF loss.",
                "Intentional WebUI off/on creates a new log_id; no interpolation is written.",
                "High-rate measurements are already filtered by the configured IMU; config.csv records the configuration.",
                "When UiValid is clear, percent fields do not establish a valid WebUI slider position.",
                "wheels.csv uses independent 50 Hz diagnostic telemetry. Join on boot_id, bridge_session, log_id, t_us and sample_seq; use motor_us and per-wheel UART age/RX increments to assess freshness. UINT32_MAX means age unavailable.",
            ],
        }

    try:
        with (args.out / "serial.raw.log").open("wb") as raw, \
             (args.out / "samples.csv").open("w", encoding="utf-8-sig", newline="") as samples, \
             (args.out / "config.csv").open("w", encoding="utf-8-sig", newline="") as configs, \
             (args.out / "wheels.csv").open("w", encoding="utf-8-sig", newline="") as wheels:
            writers = {"VR": csv.DictWriter(samples, fieldnames=HOST_FIELDS + SAMPLE_FIELDS),
                       "VI": csv.DictWriter(configs, fieldnames=HOST_FIELDS + INFO_FIELDS),
                       "VW": csv.DictWriter(wheels, fieldnames=HOST_FIELDS + WHEEL_FIELDS)}
            for writer in writers.values():
                writer.writeheader()
            write_metadata(metadata_path, metadata())
            print("Đang chờ ESP32 thu. Bật Log rung ESP-NOW trên WebUI để nhận mẫu. Ctrl+C để kết thúc.", flush=True)
            try:
                while not args.seconds or time.monotonic() - start < args.seconds:
                    now = time.monotonic()
                    if port is None:
                        if now < next_open:
                            time.sleep(.1)
                            continue
                        counters["reconnect_attempts"] += 1
                        port = serial.Serial(port=None, baudrate=921600, timeout=.05, write_timeout=.2)
                        port.dtr = args.native_usb
                        port.rts = False
                        port.port = args.port
                        try:
                            port.open()
                            if hasattr(port, "set_buffer_size"):
                                try:
                                    port.set_buffer_size(rx_size=262144, tx_size=4096)
                                except (OSError, ValueError):
                                    pass
                            headers.clear()
                            lines.pending.clear()
                            lines.discarding = False
                            next_headers = now
                        except (OSError, serial.SerialException) as error:
                            record_serial_error(error)
                            port.close()
                            port = None
                            next_open = now + 1
                            if now - last_metadata >= 5:
                                write_metadata(metadata_path, metadata())
                                last_metadata = now
                            continue
                    try:
                        if now >= next_headers:
                            port.write(b"headers\n")
                            next_headers = now + (10 if "VR" in headers and "VI" in headers else 2)
                        chunk = port.read(min(max(getattr(port, "in_waiting", 0), 1), 32768))
                    except (OSError, serial.SerialException) as error:
                        record_serial_error(error)
                        port.close()
                        port = None
                        next_open = now + 1
                        continue
                    if chunk:
                        raw.write(chunk)
                        for line in lines.add(chunk):
                            try:
                                header = parse_header(line)
                            except FrameError:
                                counters["invalid_headers"] += 1
                                continue
                            if header:
                                headers[header[0]] = header[1]
                                continue
                            try:
                                result = parse_frame(line, headers)
                            except FrameError:
                                counters["invalid_frames"] += 1
                                continue
                            if result is None:
                                counters["other_lines"] += 1
                                continue
                            kind, record = result
                            if record is None:
                                counters["preheader_frames"] += 1
                                continue
                            if kind == "VR" and not tracker.accept(record):
                                continue
                            if kind == "VI":
                                state = (record["boot_id"], record["log_id"], record["enabled"])
                                if state != last_info_state:
                                    print(json.dumps({"log_enabled": bool(record["enabled"]), "boot_id": record["boot_id"],
                                                      "log_id": record["log_id"], "sample_hz": record["sample_hz"]}), flush=True)
                                    last_info_state = state
                            record["host_utc"] = utc_now()
                            record["host_elapsed_s"] = f"{time.monotonic() - start:.6f}"
                            writers[kind].writerow(record)
                            counters[row_counts[kind]] += 1
                    if now - last_flush >= .5:
                        raw.flush()
                        samples.flush()
                        configs.flush()
                        wheels.flush()
                        last_flush = now
                    if now - last_metadata >= 5:
                        write_metadata(metadata_path, metadata())
                        print(json.dumps({"sample_rows": counters["sample_rows"], "config_rows": counters["config_rows"],
                                          "wheel_rows": counters["wheel_rows"],
                                          "invalid_frames": counters["invalid_frames"],
                                          "elapsed_s": round(now-start, 1)}), flush=True)
                        last_metadata = now
            except KeyboardInterrupt:
                end_reason = "operator_ctrl_c"
            finally:
                raw.flush()
                samples.flush()
                configs.flush()
                wheels.flush()
    except Exception:
        end_reason = "error"
        raise
    finally:
        if port is not None:
            port.close()
        write_metadata(metadata_path, metadata(final=True))
    print(json.dumps({"out": str(args.out), "end_reason": end_reason, **counters}, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
