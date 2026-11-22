import csv
import io
import json
import re
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from tools import thu_rung_lsm6dsv as capture


def record(kind="VR", **overrides):
    result = {field: "0" for field in capture.SCHEMAS[kind]}
    result.update(boot_id="8", bridge_session="7", packet_seq="1", log_id="4",
                  t_us="9007199254741001", bridge_rx_us="5000000", sample_hz="200", imu="1",
                  rssi_dbm="-54")
    if kind == "VR":
        result.update(sample_seq="1", ax="0.1234567", flags="15", source="1")
    else:
        result.update(firmware="lsm6dsv-spi-20261117", enabled="1")
    result.update({key: str(value) for key, value in overrides.items()})
    return result


def header(kind, fields=None):
    return kind + "_HEADER," + ",".join(fields or capture.SCHEMAS[kind]) + "\n"


def frame(kind, row, fields=None):
    fields = fields or capture.SCHEMAS[kind]
    body = kind + "," + ",".join(row[field] for field in fields[:-1])
    return body + "," + str(capture.crc16(body.encode("ascii"))) + "\n"


class Clock:
    def __init__(self):
        self.value = 100.0

    def now(self):
        return self.value

    def sleep(self, interval):
        self.value += interval


class Port:
    def __init__(self, chunks, clock, interrupt_at_end=False):
        self.chunks = list(chunks)
        self.clock = clock
        self.interrupt_at_end = interrupt_at_end
        self.commands = []
        self.is_open = False
        self.dtr = self.rts = None

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def write(self, content):
        self.commands.append(content)
        return len(content)

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, _):
        self.clock.value += .02
        if self.chunks:
            return self.chunks.pop(0)
        if self.interrupt_at_end:
            raise KeyboardInterrupt()
        return b""


class VibrationCaptureTests(unittest.TestCase):
    def test_crc_and_exact_integer_timestamp(self):
        self.assertEqual(capture.crc16(b"123456789"), 0x29B1)
        row = record()
        _, data = capture.parse_frame(frame("VR", row).strip(), {"VR": capture.SAMPLE_FIELDS})
        self.assertEqual(data["t_us"], 9007199254741001)
        self.assertIsInstance(data["t_us"], int)
        self.assertEqual(data["ax"], .1234567)
        with self.assertRaises(capture.FrameError):
            capture.parse_frame(frame("VR", row).replace("0.1234567", "0.2").strip(), {"VR": capture.SAMPLE_FIELDS})
        with self.assertRaises(capture.FrameError):
            capture.parse_frame(frame("VR", record(ax="nan")).strip(), {"VR": capture.SAMPLE_FIELDS})
        with self.assertRaises(capture.FrameError):
            capture.parse_frame(frame("VR", record(t_us="9007199254741001.0")).strip(), {"VR": capture.SAMPLE_FIELDS})

    def test_header_reorder_and_incompatible_schema(self):
        swapped = list(capture.SAMPLE_FIELDS)
        swapped[0], swapped[1] = swapped[1], swapped[0]
        kind, fields = capture.parse_header("\ufffdjunk" + header("VR", swapped).strip())
        _, data = capture.parse_frame(frame("VR", record(), swapped).strip(), {kind: fields})
        self.assertEqual(data["boot_id"], 8)
        with self.assertRaises(capture.FrameError):
            capture.parse_header(header("VR", swapped[:-2] + ["unknown", "serial_crc16"]).strip())

    def test_receiver_cpp_headers_match_collector(self):
        path = Path("tools/espnow_rung_receiver/espnow_rung_receiver/ReceiverImpl.h")
        source = path.read_text(encoding="utf-8")
        for kind, name in (("VR", "VrHeader"), ("VI", "ViHeader"), ("VW", "VwHeader")):
            section = source.split("const char " + name + "[] =", 1)[1].split(";", 1)[0]
            literal = "".join(json.loads('"' + piece + '"') for piece in re.findall(r'"([^"\\]*(?:\\.[^"\\]*)*)"', section))
            self.assertEqual(capture.parse_header(literal.strip()), (kind, capture.SCHEMAS[kind]))

    def test_fragmentation_bounded_line_resync(self):
        lines = capture.Lines()
        text = (header("VR") + frame("VR", record())).encode()
        result = []
        for i in range(0, len(text), 7):
            result += lines.add(text[i:i+7])
        self.assertEqual(len(result), 2)
        self.assertEqual(lines.add(b"x" * (capture.MAX_LINE_BYTES + 100)), [])
        self.assertTrue(lines.discarding)
        self.assertEqual(lines.add(b"tail\nOK\n"), ["OK"])
        self.assertEqual(lines.too_long, 1)

    def test_groups_gaps_duplicate_reorder_and_wrap(self):
        tracker = capture.Tracker()
        schema = {"VR": capture.SAMPLE_FIELDS}

        def sample(**kw):
            return capture.parse_frame(frame("VR", record(**kw)).strip(), schema)[1]

        self.assertTrue(tracker.accept(sample(sample_seq=0xFFFFFFFE, t_us=100)))
        self.assertTrue(tracker.accept(sample(sample_seq=1, t_us=15100)))
        self.assertFalse(tracker.accept(sample(sample_seq=1, t_us=15100)))
        self.assertFalse(tracker.accept(sample(sample_seq=0, t_us=10100)))
        self.assertTrue(tracker.accept(sample(boot_id=9, sample_seq=1, t_us=10)))
        self.assertTrue(tracker.accept(sample(boot_id=9, log_id=5, sample_seq=50, t_us=250000)))
        groups = tracker.summary()
        self.assertEqual(len(groups), 3)
        self.assertEqual(groups[0]["source_sequence_gaps"], 2)
        self.assertEqual(tracker.duplicates, 1)
        self.assertEqual(tracker.out_of_order, 1)

    def run_capture(self, chunks, interrupt=False, seconds=".6", include_wheels=False):
        clock = Clock()
        port = Port(chunks, clock, interrupt)
        serial_module = SimpleNamespace(Serial=lambda **_: port, SerialException=OSError)
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp) / "log"
            with patch.dict("sys.modules", {"serial": serial_module}), \
                 patch.object(capture.time, "monotonic", clock.now), \
                 patch.object(capture.time, "sleep", clock.sleep), redirect_stdout(io.StringIO()):
                self.assertEqual(capture.main(["--port", "COM5", "--out", str(out), "--seconds", seconds]), 0)
            metadata = json.loads((out / "metadata.json").read_text(encoding="utf-8"))
            with (out / "samples.csv").open(encoding="utf-8-sig", newline="") as f:
                samples = list(csv.DictReader(f))
            with (out / "config.csv").open(encoding="utf-8-sig", newline="") as f:
                config = list(csv.DictReader(f))
            with (out / "wheels.csv").open(encoding="utf-8-sig", newline="") as f:
                wheels = list(csv.DictReader(f))
            raw = (out / "serial.raw.log").read_bytes()
        self.assertFalse(port.dtr)
        self.assertFalse(port.rts)
        self.assertFalse(port.is_open)
        self.assertTrue(all(command == b"headers\n" for command in port.commands))
        if include_wheels:
            return wheels, metadata
        return metadata, samples, config, raw

    def test_wheel_stream_exact_counts_and_freshness(self):
        fields = capture.WHEEL_FIELDS
        r = record("VW", diag_session=123, motor_us=9007199254741003, enc_l=-(1<<63),
                   steps_r=(1<<63)-1, uart_rx_l=123, uart_age_r_us=(1<<32)-1, en=1)
        content = (header("VW") + frame("VW", r) +
                   frame("VW", dict(r, steps_l=str(1<<63))) +
                   frame("VW", dict(r, en="2"))).encode()
        wheels, meta = self.run_capture([content], include_wheels=True)
        self.assertEqual(meta["counters"]["invalid_frames"], 2)
        self.assertEqual(meta["counters"]["wheel_rows"], 1)
        self.assertEqual(wheels[0]["motor_us"], "9007199254741003")
        self.assertEqual(wheels[0]["enc_l"], str(-(1<<63)))
        self.assertEqual(wheels[0]["uart_age_r_us"], str((1<<32)-1))
        self.assertEqual(capture.parse_header(header("VW").strip())[1], fields)

    def test_capture_corrupt_then_reboot_preserves_data(self):
        valid = frame("VR", record()).encode()
        corrupt = valid.replace(b"0.1234567", b"0.7654321")
        body = header("VR").encode() + header("VI").encode() + valid + corrupt
        chunks = [body[:13], body[13:]]
        chunks += [frame("VI", record("VI")).encode(), valid,
                   frame("VR", record(boot_id=9, log_id=1, t_us=100, sample_seq=1)).encode()]
        meta, samples, config, raw = self.run_capture(chunks)
        self.assertEqual(len(samples), 2)
        self.assertEqual(samples[0]["t_us"], "9007199254741001")
        self.assertEqual(len(config), 1)
        self.assertIn(corrupt, raw)
        self.assertEqual(meta["counters"]["invalid_frames"], 1)
        self.assertEqual(meta["duplicate_samples"], 1)
        self.assertEqual(len(meta["groups"]), 2)
        self.assertTrue(meta["finalized"])

    def test_idle_and_header_change(self):
        invalid = header("VR", capture.SAMPLE_FIELDS[:-2] + ["bogus", "serial_crc16"])
        meta, samples, config, _ = self.run_capture([invalid.encode(), b"receiver ready\n"])
        self.assertFalse(samples)
        self.assertFalse(config)
        self.assertEqual(meta["counters"]["invalid_headers"], 1)
        self.assertEqual(meta["end_reason"], "duration")

    def test_interrupt_finalizes_all_files(self):
        content = (header("VR") + frame("VR", record())).encode()
        meta, samples, _, _ = self.run_capture([content], interrupt=True, seconds="0")
        self.assertEqual(len(samples), 1)
        self.assertEqual(meta["end_reason"], "operator_ctrl_c")
        self.assertTrue(meta["finalized"])

    def test_usb_disconnect_and_reopen(self):
        clock = Clock()

        class DisconnectingPort(Port):
            def read(self, size):
                if not self.chunks:
                    raise OSError("USB unplugged")
                return super().read(size)

        first = DisconnectingPort([(header("VR") + frame("VR", record())).encode()], clock)
        second = Port([(header("VR") + frame("VR", record(log_id=5, sample_seq=50, t_us=50000000000000000))).encode()], clock, True)
        ports = iter([first, second])
        serial_module = SimpleNamespace(Serial=lambda **_: next(ports), SerialException=OSError)
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp) / "log"
            with patch.dict("sys.modules", {"serial": serial_module}), \
                 patch.object(capture.time, "monotonic", clock.now), \
                 patch.object(capture.time, "sleep", clock.sleep), redirect_stdout(io.StringIO()):
                capture.main(["--port", "COM5", "--out", str(out)])
            metadata = json.loads((out / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["counters"]["sample_rows"], 2)
            self.assertEqual(metadata["counters"]["reconnect_attempts"], 2)
            self.assertEqual(len(metadata["groups"]), 2)
            self.assertEqual(len(metadata["serial_errors"]), 1)
            self.assertTrue(metadata["finalized"])
        self.assertFalse(first.is_open)
        self.assertFalse(second.is_open)


if __name__ == "__main__":
    unittest.main()
