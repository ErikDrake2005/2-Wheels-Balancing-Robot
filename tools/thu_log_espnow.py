"""Thu CSV từ ESP32 cầu nối; mặc định chỉ quan sát. Thử chuyển động cần --arm."""
import argparse
import csv
import json
import math
import time
import sys
from pathlib import Path
from datetime import datetime, timezone


class RadioError(RuntimeError):
    pass


def crc16(data):
    crc=0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc=((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def send_usb(port, command):
    payload=command.encode('ascii')
    frame=b'@'+payload+f'#{crc16(payload):04X}\n'.encode('ascii')
    if port.write(frame) != len(frame):
        raise RadioError('Không ghi đủ khung lệnh USB')


def header_from_line(line):
    # USB-UART có thể còn vài byte ở baud cũ trước dòng tiêu đề đầu tiên.
    at = line.find('BN_HEADER,')
    if at < 0:
        return None
    columns = next(csv.reader([line[at:]]))[1:]
    required = {'boot_id','bridge_session','radio_seq','t_us','motor_us','cfg','en','mode',
                'control_session','motor_session','control_phase','motor_phase','ack_seq',
                'theta','eq','target','rate','v','vref','yawref','angle_valid','remote_state',
                'control_fault','motor_fault','steps_l','steps_r','imu_miss','imu_i2c','imu_late'}
    if len(columns) != len(set(columns)) or not required.issubset(columns):
        raise RadioError('BN_HEADER thiếu cột bắt buộc hoặc có tên trùng')
    return columns


def parse_row(line, header):
    if not line.startswith("BN,"):
        return None
    if header is None:
        return None  # Đuôi stream trước có thể còn trong TX USB khi vừa connect.
    if header[-1] == 'serial_crc16':
        body,_,checksum=line.rpartition(',')
        try:
            if crc16(body.encode('ascii')) != int(checksum):
                raise RadioError('CRC dòng telemetry USB không khớp')
        except (ValueError,UnicodeEncodeError) as e:
            raise RadioError('CRC/byte telemetry USB không hợp lệ') from e
    cells = next(csv.reader([line]))[1:]
    if not header or len(cells) != len(header):
        raise RadioError("Số cột telemetry không khớp BN_HEADER")
    try:
        values = [float(v) for v in cells]
    except ValueError as e:
        raise RadioError("Telemetry không phải số") from e
    if not all(math.isfinite(v) for v in values):
        raise RadioError("Telemetry chứa NaN/Inf")
    return dict(zip(header, cells))


def check_live(row, initial, require_authority=True):
    for k in ("boot_id", "bridge_session", "cfg"):
        if row[k] != initial[k]:
            raise RadioError(f"{k} thay đổi trong lần thử")
    if row.get('diag_flags') != initial.get('diag_flags'):
        raise RadioError('Cờ A/B thay đổi trong phiên đo')
    if any(int(row[k]) for k in ("control_fault", "motor_fault", "mode")) or not int(row["angle_valid"]):
        raise RadioError("Góc/PID/trạng thái lỗi không phù hợp")
    if require_authority and int(row.get('en','0')) and not (int(row.get('remote_state','0')) & 2):
        raise RadioError("Quyền điều khiển từ xa đã hết hạn; không tiếp tục gửi MOVE")
    if abs(float(row["theta"]) - float(row["eq"])) > math.radians(5):
        raise RadioError("Nghiêng quá 5 độ trong thử tại chỗ")
    if abs(float(row["rate"])) > 1 or abs(float(row["v"])) > .15:
        raise RadioError("Tốc độ góc/vận tốc vượt giới hạn thử")
    estimated_travel = .5 * (int(row["steps_l"])-int(initial["steps_l"]) +
                             int(row["steps_r"])-int(initial["steps_r"])) * (2*math.pi*.04/12800)
    if abs(estimated_travel) > .10:
        raise RadioError("Quãng đường STEP ước lượng vượt 0,10 m; cần người kiểm tra vị trí")
    # ACK MAC có thể mất dù gói ứng dụng đã tới. Những bộ đếm radio là số liệu
    # chất lượng; timeout và thời gian nguồn mới quyết định ngắt liên lạc.
    for k in ("imu_miss", "imu_i2c", "imu_late"):
        if int(row[k]) != int(initial[k]):
            raise RadioError(f"Bộ đếm {k} tăng; dừng phép thử để kiểm tra")


def ready_to_arm(row):
    """Tư thế chờ nghiêm hơn ngưỡng bảo vệ sau arm; chưa phải phép đo tự cân bằng."""
    if any(int(row[k]) for k in ("control_fault", "motor_fault", "mode", "en", "motor_phase")):
        raise RadioError("Preflight yêu cầu PID thuần, motor tắt và không có lỗi")
    if not int(row["angle_valid"]):
        return False
    if not (int(row.get('remote_state','0')) & 1) or int(row.get('control_phase','0')) != 0 or \
            row.get('control_session') != row.get('motor_session'):
        return False
    return (abs(float(row["theta"])-float(row["eq"])) <= math.radians(3) and
            abs(float(row["rate"])) <= .20 and abs(float(row["v"])) <= .04 and
            abs(float(row["vref"])) < .0001 and abs(float(row["yawref"])) < .0001)


def options(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--robot-mac", default="AC:27:6E:D0:3A:24")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--seconds", type=float, default=20)
    p.add_argument("--arm", action="store_true")
    p.add_argument("--expect-diag-flags",type=int,help="yêu cầu đúng cờ A/B: bit0 imucomp, bit1 accelboost, bit2 vgainboost")
    p.add_argument("--arm-wait", type=float, default=20, help="giây chờ người vận hành dựng thẳng; cần ổn định 0,5 giây")
    p.add_argument("--native-usb", action="store_true", help="bật DTR cho cầu nối ESP32-S3 USB native; DevKit/WROOM không cần")
    p.add_argument("--pulse-v", type=float, default=0, help="m/s, tối đa 0,10; tích vận tốc × thời gian không quá 0,05 m")
    p.add_argument("--pulse-seconds", type=float, default=.3)
    p.add_argument("--release-gate", type=Path, help="chỉ đo/phát xung sau khi người vận hành xác nhận thả tay bằng file tín hiệu mới")
    p.add_argument("--disarm-at-end", action="store_true", help="chỉ dùng khi người vận hành sẵn sàng đỡ xe ở cuối")
    a = p.parse_args(argv)
    if not math.isfinite(a.seconds) or not 5 <= a.seconds <= 600:
        p.error("thời gian phải trong [5,600] giây")
    if not math.isfinite(a.pulse_v) or abs(a.pulse_v) > .10 or (a.pulse_v and not a.arm):
        p.error("pulse-v cần --arm và nằm trong [-0.10,0.10]")
    if not math.isfinite(a.pulse_seconds) or not 0 < a.pulse_seconds <= 1.0:
        p.error("pulse-seconds phải trong (0,1.0]; chỉ dùng 1 giây sau phép thử xung ngắn có người quan sát")
    if abs(a.pulse_v)*a.pulse_seconds > .0500001:
        p.error("Quãng đường đặt của một xung không được vượt 0,05 m")
    if a.disarm_at_end and not a.arm:
        p.error("disarm-at-end chỉ dùng với --arm")
    if a.release_gate and (not a.arm or a.release_gate.exists()):
        p.error("release-gate cần --arm và đường dẫn file chưa tồn tại")
    if a.expect_diag_flags is not None and not 0 <= a.expect_diag_flags <= 7:
        p.error('expect-diag-flags phải trong [0,7]')
    if not math.isfinite(a.arm_wait) or not 1 <= a.arm_wait <= 30:
        p.error("arm-wait phải trong [1,30] giây")
    return a


def main(argv=None):
    a = options(argv)
    import serial
    a.out.mkdir(parents=True, exist_ok=False)
    errors = []
    initial = last = None
    header = None
    count = 0
    usb_bad_frames = 0
    host_sequence_gaps = 0
    uart_buffers_resized = False
    armed_confirmed = False
    motor_off_confirmed = None
    candidate = None
    preflight_count = 0
    upright_start_us = None
    upright_start_host = None
    previous_preflight_host = None
    active_epoch = None
    last_wait_notice = -1
    with (a.out/"serial.raw.log").open("wb") as raw, \
         (a.out/"samples.csv").open("w", encoding="utf-8-sig", newline="") as table:
        writer = csv.writer(table)
        s = serial.Serial(port=None, baudrate=921600, timeout=.02, write_timeout=.2)
        s.dtr = a.native_usb; s.rts = False; s.port = a.port
        start = time.monotonic(); last_rx = start; next_tx = start
        arm_at = active_at = None
        release_at = None
        last_header_request = start
        pulse_finished = False
        last_flush = start
        pending = bytearray()
        try:
            s.open()
            if hasattr(s,'set_buffer_size'):
                try:
                    s.set_buffer_size(rx_size=65536,tx_size=4096);uart_buffers_resized=True
                except OSError:
                    pass
            send_usb(s,f"connect {a.robot_mac}")
            while True:
                now = time.monotonic()
                if now - last_rx > (0.2 if initial else 10):
                    raise RadioError("Không có telemetry mới; STOP và khóa lần thử")
                if a.arm and arm_at is None and now-start > a.arm_wait:
                    raise RadioError("Hết thời gian chờ xe gần thẳng và ổn định 0,5 giây; chưa ARM")
                if a.arm and active_at is not None and a.release_gate and release_at is None:
                    if a.release_gate.exists():
                        release_at = now
                        print('RELEASE_CONFIRMED: chờ 3 giây ổn định trước xung',flush=True)
                    elif now-active_at > 60:
                        raise RadioError("Hết 60 giây chờ xác nhận thả tay; STOP/giữ cân bằng, không phát xung")
                measurement_start = release_at if a.release_gate else active_at
                if now >= next_tx:
                    if a.arm and armed_confirmed and measurement_start is not None and a.pulse_v and not pulse_finished and now-measurement_start >= 3:
                        if now-measurement_start < 3+a.pulse_seconds:
                            send_usb(s,f"move {a.pulse_v:.5f} 0 100")
                        else:
                            send_usb(s,"stop"); pulse_finished = True
                    else:
                        send_usb(s,"hb")
                    next_tx = now + .04
                if header is None and now-last_header_request >= 1:
                    send_usb(s,'monitor')
                    last_header_request = now
                chunk = s.readline()
                if chunk:
                    raw.write(chunk); pending.extend(chunk)
                    if len(pending) > 4096:
                        raise RadioError("Serial line overflow")
                    if not pending.endswith(b"\n"):
                        continue
                    line = pending.decode("utf-8", errors="replace").strip(); pending.clear()
                    new_header = header_from_line(line)
                    if new_header is not None:
                        if header and new_header != header:
                            raise RadioError("Schema thay đổi")
                        if not header:
                            header = new_header; writer.writerow(["host_utc", "host_elapsed_s", "phase", *header])
                        continue
                    try:
                        row = parse_row(line, header)
                    except RadioError:
                        usb_bad_frames += 1
                        continue
                    if row:
                        if last and row['bridge_session'] == last['bridge_session']:
                            sequence_delta=(int(row['radio_seq'])-int(last['radio_seq'])) & 0xffffffff
                            if 0 < sequence_delta < 0x80000000:
                                host_sequence_gaps+=sequence_delta-1
                        if last and (int(row["radio_seq"]) <= int(last["radio_seq"]) or int(row["t_us"]) <= int(last["t_us"])):
                            raise RadioError("Telemetry nguồn không tiến hoặc bị lặp")
                        if a.arm and last and int(row["t_us"])-int(last["t_us"]) > 200000:
                            raise RadioError("Khoảng cách mẫu nguồn vượt 200 ms")
                        last_rx = now
                        phase='monitor'
                        if a.arm:
                            if arm_at is None:phase='preflight'
                            elif active_at is None:phase='await_arm'
                            elif measurement_start is None:phase='wait_operator_release'
                            elif now-measurement_start < 3:phase='settle_after_release'
                            elif a.pulse_v and now-measurement_start < 3+a.pulse_seconds:phase='pulse_scheduled'
                            else:phase='recovery' if a.pulse_v else 'measure'
                        writer.writerow([datetime.now(timezone.utc).isoformat(), now-start, phase, *(row[k] for k in header)])
                        if now-last_flush >= .5:
                            raw.flush();table.flush();last_flush=now
                        count += 1; last = row
                        if initial is None:
                            if a.arm:
                                same = candidate is not None and all(row[k] == candidate[k] for k in ("boot_id","bridge_session","cfg"))
                                same = same and all(row[k] == candidate[k] for k in ('imu_miss','imu_i2c','imu_late')) and \
                                    previous_preflight_host is not None and now-previous_preflight_host <= .15
                                pose_ready = ready_to_arm(row) and (a.expect_diag_flags is None or
                                    int(row.get('diag_flags','-1')) == a.expect_diag_flags)
                                if not same or not pose_ready or upright_start_us is None:
                                    upright_start_us = int(row['t_us']) if pose_ready else None
                                    upright_start_host = now if pose_ready else None
                                    preflight_count = 0
                                preflight_count += int(pose_ready)
                                candidate = row
                                previous_preflight_host = now
                                if int(now-start) != last_wait_notice:
                                    print(json.dumps({'state':'waiting_upright','pitch_deg':round(math.degrees(float(row['theta'])),2),
                                          'error_deg':round(math.degrees(float(row['theta'])-float(row['eq'])),2),
                                          'diag_flags':row.get('diag_flags')},ensure_ascii=False),flush=True)
                                    last_wait_notice = int(now-start)
                                if upright_start_us is None or preflight_count < 20 or int(row['t_us'])-upright_start_us < 500000 or \
                                        now-upright_start_host < .5:
                                    continue
                            initial = row
                            if a.arm:
                                send_usb(s,"arm"); arm_at = now
                                print('ARM_SENT: chờ xác nhận EN=1',flush=True)
                        if a.arm:
                            check_live(row, initial, require_authority=not pulse_finished)
                            ack_delta = (int(row['ack_seq'])-int(initial['ack_seq'])) & 0xffffffff
                            active_sample = (int(row["en"]) and int(row["control_phase"]) == int(row["motor_phase"]) == 2 and
                                             row['control_session'] == row['motor_session'])
                            confirmed = (active_sample and
                                         0 < ack_delta < 0x80000000 and int(row['remote_state']) & 2)
                            if confirmed:
                                if active_at is None:
                                    active_at = now; armed_confirmed = True
                                    active_epoch = row['motor_session']
                                    print('ARM_CONFIRMED: bắt đầu cửa sổ đo',flush=True)
                            elif armed_confirmed and not active_sample:
                                raise RadioError("Motor/phiên cân bằng đã dừng")
                            if armed_confirmed and (row['control_session'] != active_epoch or row['motor_session'] != active_epoch):
                                raise RadioError("Phiên motor thay đổi trong lúc đo")
                            if active_at is None and now-arm_at > 1.5:
                                raise RadioError("Không xác nhận được ARM")
                done = (measurement_start is not None and now-measurement_start >= a.seconds) if a.arm else (initial and now-start >= a.seconds)
                if done:
                    if a.arm and not armed_confirmed:
                        raise RadioError("Không có cân bằng active")
                    break
        except (KeyboardInterrupt, Exception) as e:
            errors.append(str(e) or type(e).__name__)
        finally:
            if a.arm and s.is_open:
                try:
                    send_usb(s,"stop")
                    # Ngắt bất thường: STOP/giữ cân bằng, không làm xe đổ bởi
                    # DISARM sớm hơn thời điểm người vận hành đang chờ.
                    if a.disarm_at_end and not errors:
                        send_usb(s,"disarm")
                        motor_off_confirmed = False
                    # Không tuyên bố EN=0 chỉ từ việc gửi lệnh. Giữ log phản hồi cuối.
                    end = time.monotonic()+.6
                    tail = bytearray()
                    before_stop_motor_us = int(last["motor_us"]) if last else 0
                    while time.monotonic() < end:
                        b = s.readline()
                        if b:
                            raw.write(b); tail.extend(b)
                            if tail.endswith(b"\n"):
                                text = tail.decode("utf-8", errors="replace").strip(); tail.clear()
                                try:
                                    sample = parse_row(text, header)
                                except RadioError:
                                    usb_bad_frames+=1;continue
                                if sample and a.disarm_at_end and not errors and not int(sample["en"]) and int(sample["motor_phase"]) == 0 and \
                                    int(sample["motor_us"]) > before_stop_motor_us and \
                                    (initial is None or sample["boot_id"] == initial["boot_id"]):
                                    motor_off_confirmed = True
                                    last = sample
                    if a.disarm_at_end and not errors and not motor_off_confirmed:
                        errors.append("Đã gửi DISARM nhưng chưa xác nhận EN=0 qua telemetry")
                except Exception as e:
                    errors.append("Kết thúc: "+str(e))
            s.close()
    meta = {"completed": not errors, "errors": errors, "rows": count, "arm_requested": a.arm,
            "usb_bad_frames": usb_bad_frames, "uart_rx_buffer_65536_requested": uart_buffers_resized,
            "host_observed_radio_sequence_gaps":host_sequence_gaps,
            "expected_diag_flags":a.expect_diag_flags,
            "arm_sent": arm_at is not None, "preflight_samples": preflight_count,
            "release_gate": str(a.release_gate) if a.release_gate else None,
            "operator_release_confirmed": release_at is not None if a.release_gate else None,
            "active_host_elapsed_s": active_at-start if active_at is not None else None,
            "operator_release_host_elapsed_s": release_at-start if release_at is not None else None,
            "pulse_v_mps": a.pulse_v, "pulse_seconds": a.pulse_seconds,
            "disarm_at_normal_end_requested": a.disarm_at_end, "last_sample": last,
            "motor_off_confirmed": motor_off_confirmed,
            "note": "STOP giữ cân bằng với lệnh v/yaw=0; DISARM chỉ ở cuối phiên hoàn tất bình thường khi đã chọn. "
                    "Các lỗi ACK radio được ghi nhưng không tự DISARM. Không có xác nhận vị trí tuyệt đối trên sàn."}
    (a.out/"metadata.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"completed": not errors, "rows": count, "errors": errors}, ensure_ascii=False))
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
    raise SystemExit(main())
