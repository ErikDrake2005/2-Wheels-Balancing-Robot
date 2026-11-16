import math
import unittest
import tempfile
import json
import io
import sys
from pathlib import Path
from types import SimpleNamespace
from contextlib import redirect_stdout
from unittest.mock import patch
from tools import thu_log_espnow as capture
from tools.thu_log_espnow import RadioError, parse_row, check_live, options, ready_to_arm, header_from_line, crc16

def row():
    return dict(boot_id="4",bridge_session="8",cfg="3",control_fault="0",motor_fault="0",mode="0",
                angle_valid="1",theta="0",eq="0",rate="0",v="0",steps_l="0",steps_r="0",
                imu_miss="0",imu_i2c="0",imu_late="0",bridge_missed_radio="0",robot_tx_fail="0",usb_dropped="0")

class RadioCaptureTests(unittest.TestCase):
    def test_csv_exact_finite(self):
        self.assertEqual(parse_row("BN,1,0.02",["seq","theta"]), {"seq":"1","theta":"0.02"})
        self.assertIsNone(parse_row("BN,1,0.02",None))
        for text in ["BN,1", "BN,1,nan", "BN,1,inf", "BN,abc,1"]:
            with self.assertRaises(RadioError): parse_row(text,["seq","theta"])
        self.assertEqual(crc16(b'123456789'),0x29b1)
        body='BN,1,0.02';frame=body+','+str(crc16(body.encode()))
        self.assertEqual(parse_row(frame,['seq','theta','serial_crc16'])['theta'],'0.02')
        with self.assertRaises(RadioError):parse_row(frame.replace('0.02','0.03'),['seq','theta','serial_crc16'])
    def test_real_usb_prefix_header_can_resynchronize(self):
        path=Path('outputs/RADIO_TEST_20261116/forward_pulse_01/serial.raw.log')
        if path.exists():
            first=path.read_bytes().splitlines()[0].decode('utf-8',errors='replace')
            columns=header_from_line(first)
            self.assertIn('theta',columns);self.assertIn('rssi_dbm',columns)
        self.assertIsNone(header_from_line('garbage'))
        with self.assertRaises(RadioError):header_from_line('BN_HEADER,theta,theta')
    def test_guards(self):
        base=row(); check_live(base,base)
        for k,v in [("boot_id","5"),("cfg","4"),("mode","1"),("theta",str(math.radians(6))),
                    ("imu_miss","1"),("steps_l","15000"),("v","0.2")]:
            changed=base.copy();changed[k]=v
            with self.assertRaises(RadioError,msg=k):check_live(changed,base)
    def test_default_no_motion(self):
        a=options(["--port","COM1","--out","test-output"])
        self.assertFalse(a.arm);self.assertEqual(a.pulse_v,0)
        b=options(['--port','COM1','--out','test-output','--arm','--pulse-v','.05','--pulse-seconds','1'])
        self.assertEqual(b.pulse_seconds,1)
        c=options(['--port','COM1','--out','test-output','--arm','--pulse-v','.1','--pulse-seconds','.5'])
        self.assertEqual(c.pulse_v*c.pulse_seconds,.05)
    def test_mac_ack_is_not_a_control_fault(self):
        base=row();changed=base.copy()
        changed.update(robot_tx_fail="3",bridge_missed_radio="1",usb_dropped="1")
        check_live(changed,base)
    def test_waiting_pose_is_not_an_active_fault(self):
        base=row();base.update(en='0',motor_phase='0',vref='0',yawref='0',remote_state='1')
        self.assertTrue(ready_to_arm(base))
        for k,v in [('theta',str(math.radians(5.46))),('rate','.25'),('v','.05'),('angle_valid','0')]:
            changed=base.copy();changed[k]=v
            self.assertFalse(ready_to_arm(changed))
        changed=base.copy();changed['en']='1'
        with self.assertRaises(RadioError):ready_to_arm(changed)
    def test_stop_holds_on_error_disarm_only_on_normal_end(self):
        class Clock:
            value=100.0
            def now(self): return self.value
        class Port:
            def __init__(self,clock,fail,wait_reads,gate=None):
                self.clock=clock;self.fail=fail;self.commands=[];self.en=False
                self.wait_reads=wait_reads
                self.gate=gate;self.release_time=None;self.moves=0
                self.seq=0;self.active_reads=0;self.header=True;self.is_open=False
                self.stale=True;self.before_arm_rows=0
            def open(self): self.is_open=True
            def close(self): self.is_open=False
            def write(self,data):
                cmd=data.decode().strip();self.commands.append(cmd)
                if cmd.startswith('@'):
                    body,check=cmd[1:].rsplit('#',1)
                    if int(check,16)!=crc16(body.encode()):raise AssertionError('Invalid host USB command CRC')
                    cmd=body;self.commands[-1]=cmd
                if cmd=='arm':
                    if self.before_arm_rows < 4: raise AssertionError('ARM before stale tail plus three fresh preflight rows')
                    self.en=True
                if cmd=='disarm': self.en=False
                if cmd.startswith('move ') and self.gate:
                    if self.release_time is None or self.clock.value-self.release_time < 3:
                        raise AssertionError('MOVE before operator release plus 3-second settling')
                    self.moves+=1
                return len(data)
            def readline(self):
                self.clock.value+=.02;self.seq+=1
                data=row()
                data.update(radio_seq=str(self.seq),t_us=str(round(self.clock.value*1e6)),
                            motor_us=str(round(self.clock.value*1e6)+100),en=str(int(self.en)),
                            motor_phase='2' if self.en else '0',control_phase='2' if self.en else '0',remote_state='3' if self.en else '1',
                            target='0',vref='0',yawref='0',control_session='2' if self.en else '1',motor_session='2' if self.en else '1',
                            ack_seq='3' if self.en else '0')
                if not self.en and self.before_arm_rows < self.wait_reads:data['theta']=str(math.radians(5.46))
                if self.en:
                    self.active_reads+=1
                    if self.gate and self.active_reads==70:
                        self.gate.write_text('operator released body',encoding='utf-8')
                        self.release_time=self.clock.value
                    if self.fail is True and self.active_reads>=3: data['imu_miss']='1'
                if self.stale:
                    self.stale=False;self.before_arm_rows+=1
                    return ('BN,'+','.join(data.values())+'\n').encode()
                if self.header:
                    self.header=False
                    return b'\x13b\xff'+('BN_HEADER,'+','.join(data)+'\n').encode()
                if not self.en: self.before_arm_rows+=1
                if self.fail=='usb' and self.en and self.active_reads==3:
                    return ('BN,'+','.join(list(data.values())[:-1])+'\n').encode()
                return ('BN,'+','.join(data.values())+'\n').encode()
        for fail,wait_reads,gate_on in [(True,6,False),(False,6,False),(False,350,False),(False,6,True),('usb',6,False)]:
            with self.subTest(fail=fail,wait_reads=wait_reads,gate=gate_on), tempfile.TemporaryDirectory() as folder:
                gate=Path(folder)/'released.flag' if gate_on else None
                clock=Clock();port=Port(clock,fail,wait_reads,gate);out=Path(folder)/'capture'
                with patch.dict(sys.modules,{'serial':SimpleNamespace(Serial=lambda **kw:port)}), \
                     patch('tools.thu_log_espnow.time.monotonic',clock.now),redirect_stdout(io.StringIO()):
                    args=['--port','FAKE','--out',str(out),'--seconds','5','--arm','--disarm-at-end']
                    if gate_on:args+=['--release-gate',str(gate),'--pulse-v','0.05']
                    code=capture.main(args)
                result=json.loads((out/'metadata.json').read_text(encoding='utf-8'))
                self.assertGreaterEqual(result['preflight_samples'],20)
                self.assertTrue(result['arm_sent'],result['errors'])
                self.assertIn('stop',port.commands)
                if fail is True:
                    self.assertEqual(code,1);self.assertNotIn('disarm',port.commands)
                    self.assertIsNone(result['motor_off_confirmed'])
                else:
                    self.assertEqual(code,0);self.assertIn('disarm',port.commands)
                    self.assertTrue(result['motor_off_confirmed'])
                    self.assertGreaterEqual(port.active_reads,250)
                    if fail=='usb':
                        self.assertEqual(result['usb_bad_frames'],1)
                        self.assertEqual(result['host_observed_radio_sequence_gaps'],1)
                    if gate_on:
                        self.assertGreater(port.moves,0)
                        self.assertTrue(result['operator_release_confirmed'])

if __name__ == "__main__": unittest.main()
