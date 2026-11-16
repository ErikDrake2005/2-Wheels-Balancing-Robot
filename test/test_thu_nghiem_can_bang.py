"""Kiểm tra thu dữ liệu với đồng hồ và cổng Serial giả; chạy bằng python -m unittest."""

import csv
import io
import math
import tempfile
import unittest
from contextlib import ExitStack, redirect_stderr
from pathlib import Path

from tools import thu_nghiem_can_bang as capture


def sample(t, enabled=True, **values):
    s = dict.fromkeys(capture.FIELDS, 0)
    s.update(t_us=t, m_us=t+100, seq=t//5000, cseq=t//5000, cfg=3, mode=0,
             csess=2, msess=2, cphase=2 if enabled else 0, mphase=2 if enabled else 0,
             en=int(enabled), valid=1, age_us=1000, kp=30, kd=3, feedback=1)
    s.update(values)
    return s


def row(s):
    return "NC," + ",".join(str(s[k]) for k in capture.FIELDS)


class FakeSerial:
    def __init__(self, fail=None, mode=0, initially_enabled=False):
        self.time = 0
        self.writes = []
        self.queue = []
        self.enabled = initially_enabled
        self.streaming = False
        self.fail = fail
        self.active_reads = 0
        self.mode = mode
        self.cfg = 3
        self.gains = dict(kp=30, ki=0, kd=3, kpv=0.15, kiv=0.08)

    def config(self):
        return "NC_CFG," + ",".join(map(str, [self.cfg, self.mode, *self.gains.values(), 0, 1]))

    def write(self, data):
        cmd = data.decode().strip()
        self.writes.append(cmd)
        if cmd == "research 1":
            self.streaming = True
            self.queue += ["NC_HEADER,"+",".join(capture.FIELDS), self.config()]
        elif cmd == "status":
            self.queue.append("Mode=PID_THUAN EN="+str(int(self.enabled)))
        elif cmd == "research 0":
            self.streaming = False
        elif cmd == "arm":
            self.enabled = True
        elif cmd == "disarm":
            self.enabled = False
        elif cmd in ("pid", "fuzzy"):
            self.mode = int(cmd == "fuzzy")
            self.cfg += 1
            self.queue.append(self.config())
        elif "_" in cmd:
            name, value = cmd.split()
            self.gains[name.split("_")[0]] = float(value)
            self.cfg += 1
            self.queue.append(self.config())
        return len(data)

    def readline(self):
        self.time += 0.02
        if self.queue:
            return (self.queue.pop(0)+"\n").encode()
        if not self.streaming:
            return b""
        t = round(self.time*1e6)
        if self.fail == "short_active_window" and self.time >= 14:
            self.enabled = True
        s = sample(t, self.enabled, mode=self.mode, cfg=self.cfg)
        if self.enabled:
            self.active_reads += 1
            if self.active_reads == 3:
                if self.fail == "rate":
                    s["rate"] = 2.
                elif self.fail == "interrupt":
                    raise KeyboardInterrupt
                elif self.fail == "read":
                    raise OSError("USB disconnected briefly")
                elif self.fail == "change_cfg":
                    self.cfg += 1
                    return (self.config()+"\n").encode()
                elif self.fail == "changed_session":
                    s["msess"] = s["csess"] = 9
            if self.active_reads >= 3 and self.fail == "stale":
                return b""
        return (row(s)+"\n").encode()


class CaptureTests(unittest.TestCase):
    def test_strict_protocol(self):
        parser = capture.NcProtocol()
        header = "NC_HEADER," + ",".join(capture.FIELDS)
        with self.assertRaises(capture.CaptureError):
            parser.parse(header.replace("vref,", "v,"))
        parser.parse(header)
        with self.assertRaises(capture.CaptureError):
            parser.parse(row(sample(100000)))
        parser.parse("NC_CFG,3,0,30,0,3,.15,.08,0,1")
        for corrupted in (row(sample(100000, mode=1)), row(sample(100000, kp="nan")),
                          row(sample(100000, seq="1.2")), row(sample(100000, en=2)),
                          row(sample(100000))+",0"):
            with self.subTest(corrupted=corrupted), self.assertRaises(capture.CaptureError):
                parser.parse(corrupted)
        self.assertEqual(parser.parse(row(sample(100000)))[1]["t_us"], 100000)

    def test_summary_and_guards(self):
        a = sample(100000, e=math.radians(1), rate=.5, cmdL=19990)
        b = sample(140000, e=math.radians(-2), rate=-.5)
        report = capture.summary([[a, b]], 20000)
        self.assertAlmostEqual(report["rmse_error_deg"], math.sqrt(2.5))
        self.assertAlmostEqual(report["mean_error_deg"], -.5)
        self.assertAlmostEqual(report["p95_abs_error_deg"], 2)
        self.assertAlmostEqual(report["rms_rate_rad_s"], .5)
        self.assertEqual(report["inferred_missing_slots"], 1)
        self.assertEqual(report["source_rate_hz"], 25)
        self.assertEqual(report["wheel_command_saturation_fraction"], .5)
        for change in ({"miss":1}, {"i2c":1}, {"drop":1}, {"late":1}, {"e":.15}, {"theta":.2},
                       {"t_us":650000}, {"t_us":100000}, {"v":.26}, {"yawref":.1},
                       {"valid":0}, {"mfault":1}, {"age_us":20000}, {"m_us":61000}):
            with self.subTest(change=change), self.assertRaises(capture.CaptureError):
                capture.guard(dict(b, **change), a)

    def run_fake(self, fake, *args):
        with tempfile.TemporaryDirectory() as tmp:
            location = Path(tmp)/"run"
            opts = capture.options(["--port", "FAKE", "--out", str(location), *args])
            with ExitStack() as stack:
                files = capture.Files(location, stack)
                report = capture.capture(fake, opts, files, lambda: fake.time)
            with (location/"samples.csv").open(encoding="utf-8-sig", newline="") as f:
                samples = list(csv.DictReader(f))
            self.assertTrue((location/"metadata.json").is_file())
            self.assertTrue((location/"serial.raw.log").read_bytes())
            self.assertTrue((location/"events.csv").read_bytes().startswith(b"\xef\xbb\xbf"))
            return report, samples

    def test_record_observes_disabled_robot(self):
        fake = FakeSerial()
        report, samples = self.run_fake(fake, "--duration", "0.2")
        self.assertTrue(report["completed"], report["errors"])
        self.assertEqual(report["summary"]["valid_active_measure_samples"], 0)
        self.assertFalse(report["measurement_window_complete"])
        self.assertTrue(samples)
        self.assertEqual(fake.writes, ["status", "research 1", "research 0", "status"])

    def test_live_trial_acknowledges_config_before_arm(self):
        fake = FakeSerial()
        report, samples = self.run_fake(fake, "--arm", "--mode", "fuzzy", "--kd", "2.5",
                                        "--duration", "18", "--condition", "floor-unheld")
        self.assertTrue(report["completed"], report["errors"])
        self.assertTrue(report["motor_off_confirmed"])
        self.assertEqual(report["condition"], "floor-unheld")
        self.assertLess(fake.writes.index("kd_m 2.500000"), fake.writes.index("arm"))
        self.assertGreaterEqual(report["summary"]["observed_measure_span_s"], 15)
        self.assertTrue(all(s["mode"] == "1" for s in samples if s["phase"] == "measure"))

    def test_fails_closed_on_read_failures(self):
        for failure in ("rate", "interrupt", "read", "stale"):
            fake = FakeSerial(fail=failure)
            with self.subTest(failure=failure):
                report, _ = self.run_fake(fake, "--arm", "--mode", "pid")
                self.assertFalse(report["completed"])
                self.assertTrue(report["motor_off_confirmed"])
                self.assertIn("disarm", fake.writes[fake.writes.index("arm")+1:])
        fake = FakeSerial(initially_enabled=True)
        report, _ = self.run_fake(fake, "--arm", "--mode", "pid")
        self.assertFalse(report["completed"])
        self.assertNotIn("arm", fake.writes)
        self.assertNotIn("pid", fake.writes)

    def test_dataset_requires_frozen_pid(self):
        with redirect_stderr(io.StringIO()):
            for bad in (["--mode","fuzzy"], ["--kp","20"]):
                with self.assertRaises(SystemExit):
                    capture.options(["--port", "X", "--out", "Y", "--purpose", "io_dataset", *bad])
        for fake in (FakeSerial(mode=1, initially_enabled=True), FakeSerial(fail="change_cfg", initially_enabled=True)):
            report, samples = self.run_fake(fake, "--purpose", "io_dataset")
            self.assertFalse(report["completed"])
            self.assertTrue(any(s["phase"] == "rejected_config" for s in samples))
            self.assertNotIn("disarm", fake.writes)
        fake = FakeSerial()
        report, _ = self.run_fake(fake, "--purpose", "io_dataset", "--arm")
        self.assertTrue(report["completed"], report["errors"])
        self.assertEqual(report["requested_mode"], "pid")
        self.assertTrue(report["io_dataset_valid"])
        self.assertNotIn("pid", fake.writes)

    def test_passive_dataset_rejects_short_or_discontinuous_capture(self):
        for fake in (FakeSerial(fail="short_active_window"), FakeSerial(fail="changed_session", initially_enabled=True)):
            report, _ = self.run_fake(fake, "--purpose", "io_dataset", "--duration", "18")
            self.assertFalse(report["io_dataset_valid"])
            self.assertFalse(report["completed"])
            self.assertNotIn("disarm", fake.writes)
        fake = FakeSerial(initially_enabled=True)
        report, _ = self.run_fake(fake, "--purpose", "io_dataset", "--duration", "18")
        self.assertTrue(report["io_dataset_valid"], report["errors"])


if __name__ == "__main__":
    unittest.main()
