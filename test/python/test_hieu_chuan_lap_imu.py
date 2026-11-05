"""Test dữ liệu đầu vào thật định dạng RAW_IMU và phép dựng hệ trục, không đo xe."""
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

GOC=Path(__file__).resolve().parents[2]
DAC_TA=importlib.util.spec_from_file_location('hieu_chuan',GOC/'tools/hieu_chuan_lap_imu.py')
HIEU_CHUAN=importlib.util.module_from_spec(DAC_TA)
DAC_TA.loader.exec_module(HIEU_CHUAN)


class KiemTraHieuChuan(unittest.TestCase):
    def test_yaw30_tinh_tay(self):
        ma_tran=HIEU_CHUAN.tao_ma_tran([0,0,9.80665],[0.1,0.1*math.sqrt(3),0])['R_camBienSangXe']
        self.assertAlmostEqual(ma_tran[0][0],math.sqrt(3)/2)
        self.assertAlmostEqual(ma_tran[0][1],-0.5)
        self.assertAlmostEqual(ma_tran[1][0],0.5)

    def test_vector_loi(self):
        for gia_toc,gyro in [([0,0,0],[0,0.3,0]),([0,0,9.80665],[0,0,0]),
                             ([0,0,9.80665],[0,0,0.3]),([float('nan'),0,9.8],[0,0.3,0])]:
            with self.subTest(gia_toc=gia_toc,gyro=gyro):
                with self.assertRaises(ValueError): HIEU_CHUAN.tao_ma_tran(gia_toc,gyro)

    def test_log_va_huong_nguoc(self):
        with tempfile.TemporaryDirectory() as thu_muc:
            dung=Path(thu_muc)/'dung.txt';nghieng=Path(thu_muc)/'nghieng.txt'
            dung.write_text('EN=0\n'+''.join(f'RAW_IMU,{1000+i*200000},0,0,9.80665,0,0,0\n' for i in range(8)))
            nghieng.write_text(''.join(f'RAW_IMU,{2000000+i*200000},0,0,9.80665,0.1,0.1732050808,0\n' for i in range(8)))
            ket_qua=HIEU_CHUAN.tu_log(dung,nghieng)
            self.assertEqual(ket_qua['so_mau_nghieng'],8)
            self.assertAlmostEqual(ket_qua['R_camBienSangXe'][1][0],0.5)
            # Một lượt nghiêng về: không được lấy trung bình cùng lượt nghiêng tiến.
            with nghieng.open('a') as tep: tep.write('RAW_IMU,4000000,0,0,9.80665,-0.1,-0.1732050808,0\n')
            with self.assertRaises(ValueError): HIEU_CHUAN.tu_log(dung,nghieng)

    def test_log_thieu_mau_va_thoi_gian_lui(self):
        with tempfile.TemporaryDirectory() as thu_muc:
            tep=Path(thu_muc)/'loi.txt'
            tep.write_text('RAW_IMU,1000,0,0,9.8,0,0.3,0\n')
            with self.assertRaises(ValueError): HIEU_CHUAN.doc_mau(tep)
            tep.write_text('RAW_IMU,1000,0,0,9.8,0,0.3,0\nRAW_IMU,500,0,0,9.8,0,0.3,0\n')
            with self.assertRaises(ValueError): HIEU_CHUAN.doc_mau(tep)

    def test_tru_gyro_du_tu_doan_dung(self):
        with tempfile.TemporaryDirectory() as thu_muc:
            dung=Path(thu_muc)/'dung.txt';nghieng=Path(thu_muc)/'nghieng.txt'
            dung.write_text(''.join(f'RAW_IMU,{1000+i*200000},0,0,9.80665,0.005,-0.004,0.003\n' for i in range(8)))
            nghieng.write_text(''.join(f'RAW_IMU,{2000000+i*200000},0,0,9.80665,0.105,0.1692050808,0.003\n' for i in range(8)))
            ma_tran=HIEU_CHUAN.tu_log(dung,nghieng)['R_camBienSangXe']
            self.assertAlmostEqual(ma_tran[1][0],0.5)


if __name__=='__main__': unittest.main()
