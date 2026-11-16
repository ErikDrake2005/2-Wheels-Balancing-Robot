"""Dữ liệu giả chỉ dùng kiểm thử xác thực/xuất CSV, không mở cổng Serial."""

import csv
import hashlib
import io
import json
import math
import tempfile
import unittest
from pathlib import Path

from tools import thu_nghiem_can_bang as capture
from tools import xuat_du_lieu_pid as exporter


class ExportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root/"capture"
        self.source.mkdir()
        self.destination = self.root/"export"
        self.cfg = dict(cfg=7, mode=0, kp0=30, ki0=0, kd0=3, kpv0=.15, kiv0=.08, bench=0, vloop=1)
        self.rows = []
        for i in range(752):
            s = dict.fromkeys(capture.FIELDS, 0)
            timestamp = 1_000_000 if i == 0 else 4_000_000 + (i-1)*20000
            s.update(t_us=timestamp, m_us=timestamp+900, seq=timestamp//5000, cseq=timestamp//5000,
                     cfg=7, csess=9, msess=9, cphase=2, mphase=2, valid=1, en=1, age_us=1000,
                     theta=.01, target=.005, e=.005, eq=.002, rate=.02, v=.001, cmdL=120, cmdR=160,
                     kp=30, kd=3, a_cmd=.3, feedback=1)
            self.rows.append(dict(host_utc="2026-11-15T16:00:00+00:00", host_elapsed_s=i*.02,
                                  phase="settle" if i == 0 else "measure", **s))
        self.meta = dict(purpose="io_dataset", condition="floor-unheld", requested_mode="pid",
                         trial_type="stationary_arm", motor_off_confirmed=True,
                         completed=True, io_dataset_valid=True, measurement_window_complete=True, errors=[],
                         firmware_configurations=[self.cfg])

    def write_source(self):
        self.meta["nc_samples"] = len(self.rows)
        (self.source/"metadata.json").write_text(json.dumps(self.meta), encoding="utf-8")
        with (self.source/"samples.csv").open("w", encoding="utf-8-sig", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)
        raw = ["boot test fixture", "NC_HEADER,"+",".join(capture.FIELDS),
               "NC_CFG,"+",".join(str(self.cfg[k]) for k in capture.CFG_FIELDS)]
        raw += ["NC,"+",".join(str(row[k]) for k in capture.FIELDS) for row in self.rows]
        (self.source/"serial.raw.log").write_text("\n".join(raw)+"\n", encoding="utf-8")
        (self.source/"events.csv").write_text("host_utc,host_elapsed_s,type,detail\n", encoding="utf-8-sig")

    def assert_rejected(self):
        with self.assertRaises((exporter.ExportError, capture.CaptureError)):
            exporter.export(self.source, self.destination)
        self.assertFalse(self.destination.exists())

    def test_export_units_values_provenance_and_no_source_changes(self):
        self.write_source()
        originals = {p.name: p.read_bytes() for p in self.source.iterdir()}
        result = exporter.export(self.source, self.destination)
        payload = (self.destination/"data_pid_io.csv").read_bytes()
        self.assertTrue(payload.startswith(b"\xef\xbb\xbf"))
        output = list(csv.DictReader(io.StringIO(payload.decode("utf-8-sig"))))
        self.assertEqual(len(output), 751)
        first, last = output[0], output[-1]
        self.assertEqual(float(first["t_s"]), 0)
        self.assertEqual(float(last["t_s"]), 15)
        self.assertEqual(int(first["imu_source_us"]), 4_000_000)
        self.assertEqual(int(first["motor_snapshot_us"]), 4_000_900)
        self.assertEqual(float(first["u_common_steps_per_s"]), 140)
        self.assertEqual(float(first["u_diff_steps_per_s"]), 20)
        self.assertAlmostEqual(float(first["y_pitch_deg"]), math.degrees(.01))
        self.assertEqual(float(first["a_cmd_mps2"]), .3)
        self.assertEqual(float(first["kp_base"]), 30)
        self.assertEqual(result["csv_sha256"], hashlib.sha256(payload).hexdigest())
        simulation_payload = (self.destination/"pid_u_y.csv").read_bytes()
        simulation_reader = csv.DictReader(io.StringIO(simulation_payload.decode("utf-8-sig")))
        simulation_rows = list(simulation_reader)
        self.assertEqual(simulation_reader.fieldnames, list(exporter.SIMULATION_COLUMNS))
        self.assertEqual(len(simulation_rows), len(output))
        for simulation, full in zip(simulation_rows, output):
            self.assertEqual(simulation, {key: full[key] for key in exporter.SIMULATION_COLUMNS})
            self.assertTrue(all(math.isfinite(float(value)) for value in simulation.values()))
        self.assertEqual(result["simulation_csv_file"], "pid_u_y.csv")
        self.assertEqual(result["simulation_csv_sha256"], hashlib.sha256(simulation_payload).hexdigest())
        for name, blob in originals.items():
            self.assertEqual((self.source/name).read_bytes(), blob)
            self.assertEqual(result["source_files"][name]["sha256"], hashlib.sha256(blob).hexdigest())
        with self.assertRaises(exporter.ExportError):
            exporter.export(self.source, self.destination)
        self.assertEqual((self.destination/"data_pid_io.csv").read_bytes(), payload)

    def test_reject_metadata_claims_and_wrong_condition(self):
        for key, value in (("purpose", "tuning"), ("condition", "floor-held"), ("requested_mode", "fuzzy"),
                           ("completed", False), ("io_dataset_valid", 1), ("measurement_window_complete", False),
                           ("motor_off_confirmed", False), ("errors", ["sensor fault"])):
            old = self.meta[key]
            self.meta[key] = value
            self.write_source()
            with self.subTest(key=key):
                self.assert_rejected()
            self.meta[key] = old

    def test_reject_corruption_discontinuities_and_faults(self):
        for field, value in (("t_us", 4_000_000), ("seq", 800), ("cseq", 800), ("mode", 1),
                             ("cfg", 8), ("csess", 10), ("en", 0), ("valid", 0), ("mfault", 1),
                             ("drop", 1), ("i2c", 1), ("feedback", 0), ("kp", 31), ("rate", "nan"),
                             ("m_us", 1), ("e", .05), ("phase", "rejected_session"), ("phase", "after")):
            old = self.rows[3][field]
            self.rows[3][field] = value
            self.write_source()
            with self.subTest(field=field, value=value):
                self.assert_rejected()
            self.rows[3][field] = old

    def test_reject_short_measure_and_missing_raw_match(self):
        self.rows.pop()
        self.write_source()
        self.assert_rejected()
        self.rows[-1]["t_us"] += 20000
        self.rows[-1]["m_us"] += 20000
        self.rows[-1]["seq"] += 4
        self.rows[-1]["cseq"] += 4
        self.write_source()
        raw = (self.source/"serial.raw.log").read_text(encoding="utf-8")
        (self.source/"serial.raw.log").write_text(raw.replace(",120,160,", ",121,160,", 1+len(self.rows)), encoding="utf-8")
        self.assert_rejected()

    def test_reject_missing_configuration_and_allow_new_output_subfolder(self):
        self.meta["firmware_configurations"] = []
        self.write_source()
        self.assert_rejected()
        self.meta["firmware_configurations"] = [self.cfg]
        self.write_source()
        exporter.export(self.source, self.source/"export")
        self.assertTrue((self.source/"export"/"data_pid_io.csv").exists())


if __name__ == "__main__":
    unittest.main()
