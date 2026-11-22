import csv
import io
import math
import tempfile
import unittest
from pathlib import Path
from tools import phan_tich_rung_banh as analyze


def rows(second=0, base_seq=0):
    group = dict(boot_id=100, bridge_session=200, log_id=1, cfg=18)
    signal = [dict(group, t_us=1_000_000*(second+1)+i*5000, flags=15, mode=0, diag_flags=0,
                   sample_seq=(base_seq+i) % analyze.U32, rate=.7 + .1*(-1)**i,
                   yawref=0, yaw_request=0, v_request=-.16, vref=-.16) for i in range(200)]
    wheel = [dict(group, t_us=r["t_us"], motor_us=r["t_us"]+123, en=1, angle_valid=1,
                  diag_session=21, motor_session=13, packet_seq=(base_seq+i) % analyze.U32,
                  uart_rx_l=(base_seq+i*4) % analyze.U32, uart_rx_r=(base_seq+i*4) % analyze.U32,
                  steps_l=-100*i, steps_r=-100*i, enc_l=-50*i, enc_r=-100*i,
                  uart_age_l_us=2000, uart_age_r_us=4300) for i,r in enumerate(signal[::4])]
    return signal, wheel


class MotionAnalysisTest(unittest.TestCase):
    def summarize(self, signal, wheels=None):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            for kind, data in (("samples", signal), ("wheels", wheels)):
                if data is None: continue
                with (path/(kind+".csv")).open("w", newline="") as f:
                    writer = csv.DictWriter(f, list(data[0]))
                    writer.writeheader()
                    writer.writerows(data)
            table = io.StringIO()
            report = analyze.summarize(path, windows_csv=table)
            table.seek(0)
            return report, list(csv.DictReader(table))

    def test_signed_tracking_and_rate_ac_with_wrap(self):
        signal, wheels = rows(base_seq=analyze.U32-30)
        report, table = self.summarize(signal, wheels)
        self.assertEqual(len(table), 1)
        self.assertEqual(table[0]["direction"], "reverse")
        self.assertEqual(table[0]["phase"], "straight_cruise")
        self.assertAlmostEqual(float(table[0]["rate_rms_ac_radps"]), .1)
        self.assertAlmostEqual(float(table[0]["error_l_mps"]), 2500*2*math.pi*.04/12800)
        self.assertAlmostEqual(float(table[0]["error_r_mps"]), 0)
        self.assertEqual(report["bins"][0]["matched_wheel_windows"], 1)
        self.assertEqual(report["bins"][0]["speed_bin_low_mps"], .15)

    def test_turn_ramp_and_missing_wheels(self):
        first, _ = rows()
        second, _ = rows(second=1)
        for r in first: r["yaw_request"] = .3
        for i,r in enumerate(second): r["vref"] = i*.001
        report, table = self.summarize(first+second)
        self.assertEqual([r["phase"] for r in table], ["turn","accel_decel"])
        self.assertEqual(report["wheel_window_quality"], {"wheel_missing": 2})

    def test_reversing_error_cancels_in_net_but_not_short_blocks(self):
        signal, wheel = rows()
        for i, r in enumerate(wheel):
            r["enc_r"] += round(200 * math.sin(2*math.pi*i/7))
        report, table = self.summarize(signal, wheel)
        self.assertAlmostEqual(float(table[0]["error_r_mps"]), 0)
        self.assertGreater(report["bins"][0]["median_block_error_rms_r_mps"], .03)

    def test_drop_stale_reset_and_epoch(self):
        signal, wheel = rows()
        report, table = self.summarize(signal[:20]+signal[21:], wheel)
        self.assertFalse(table)
        self.assertEqual(report["rejected_imu_windows"], {"imu_gap_or_short": 1})
        for idx,key,value,reason in [
            (24,"uart_age_l_us",4294967295,"encoder_stale"), (4,"uart_rx_r",12,"encoder_counter_stale_or_reset"),
            (26,"motor_session",45,"wheel_session_change"), (33,"packet_seq",545,"wheel_gap_or_short"),
            (18,"motor_us",9_000_000,"wheel_imu_time_skew")]:
            changed = [dict(r) for r in wheel]
            changed[idx][key] = value
            report, table = self.summarize(signal, changed)
            self.assertEqual(table[0]["wheel_quality"], reason)
            self.assertEqual(table[0]["encoder_l_mps"], "")
            self.assertEqual(report["bins"][0]["matched_wheel_windows"], 0)
        for r in wheel: r["boot_id"] += 1
        self.assertEqual(self.summarize(signal, wheel)[1][0]["wheel_quality"], "wheel_missing")


if __name__ == "__main__":
    unittest.main()
