"""Tạo bảng sai số minh họa gá IMU, không phải phép đo hay hiệu chuẩn xe thực."""
from pathlib import Path
import math
import json
import csv
import sys
from hieu_chuan_lap_imu import tao_ma_tran


def ma_tran_euler(roll,pitch,yaw):
    roll,pitch,yaw=map(math.radians,(roll,pitch,yaw))
    cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
    return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
            [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]


def sang_sensor(ma_tran,vecto_xe):
    return [sum(ma_tran[hang][cot]*vecto_xe[hang] for hang in range(3)) for cot in range(3)]


def sang_xe(ma_tran,vecto_sensor):
    return [sum(ma_tran[hang][cot]*vecto_sensor[cot] for cot in range(3)) for hang in range(3)]


def main():
    if hasattr(sys.stdout,'reconfigure'): sys.stdout.reconfigure(encoding='utf-8')
    thu_muc=Path(__file__).resolve().parents[1]/'docs/du_lieu_tinh_toan'
    thu_muc.mkdir(parents=True,exist_ok=True)
    cac_dong=[]
    for roll,pitch,yaw in [(0,0,0),(0,0,15),(0,0,30),(0,0,45),(-4,5,30),(7,-8,-45)]:
        ma_tran=ma_tran_euler(roll,pitch,yaw)
        for goc_xe in [-10,0,10]:
            goc=math.radians(goc_xe)
            gia_toc_sensor=sang_sensor(ma_tran,[-9.80665*math.sin(goc),0,9.80665*math.cos(goc)])
            gyro_sensor=sang_sensor(ma_tran,[0,0.3,0])
            gia_toc_xe=sang_xe(ma_tran,gia_toc_sensor)
            gyro_xe=sang_xe(ma_tran,gyro_sensor)
            cac_dong.append({'roll_lap_deg':roll,'pitch_lap_deg':pitch,'yaw_lap_deg':yaw,
                'pitch_that_deg':goc_xe,'pitch_mot_truc_deg':math.degrees(math.atan2(-gia_toc_sensor[0],gia_toc_sensor[2])),
                'roll_gia_deg':math.degrees(math.atan2(gia_toc_sensor[1],gia_toc_sensor[2])),
                'gyroY_mot_truc_radps':gyro_sensor[1],
                'pitch_sau_bu_deg':math.degrees(math.atan2(-gia_toc_xe[0],math.hypot(*gia_toc_xe[1:]))),
                'gyroY_sau_bu_radps':gyro_xe[1]})
    with (thu_muc/'sai_so_goc_lap.csv').open('w',newline='',encoding='utf-8') as tep:
        ghi=csv.DictWriter(tep,fieldnames=list(cac_dong[0]));ghi.writeheader();ghi.writerows(cac_dong)
    vi_du=ma_tran_euler(-4,5,30)
    ket_qua=tao_ma_tran(sang_sensor(vi_du,[0,0,9.80665]),sang_sensor(vi_du,[0,0.3,0]))
    ket_qua['nguon']='MINH_HOA_SO_HOC_KHONG_PHAI_HIEU_CHUAN_XE'
    ket_qua['sai_so_ma_tran_max']=max(abs(vi_du[i][j]-ket_qua['R_camBienSangXe'][i][j]) for i in range(3) for j in range(3))
    (thu_muc/'vi_du_hieu_chuan_lap.json').write_text(json.dumps(ket_qua,ensure_ascii=False,indent=2),encoding='utf-8')
    bang=['# Sai số góc lắp: ví dụ số học','',
          'Bảng tạo từ ma trận quay lý tưởng; không phải log đo xe. Gyro pitch thật=0,3 rad/s.', '',
          '| Gá roll/pitch/yaw (°) | Pitch xe (°) | Pitch cũ (°) | Roll giả (°) | GyroY cũ | Pitch mới (°) |',
          '| --- | --- | --- | --- | --- | --- |']
    for dong in cac_dong:
        bang.append(f"| {dong['roll_lap_deg']}/{dong['pitch_lap_deg']}/{dong['yaw_lap_deg']} | {dong['pitch_that_deg']} | {dong['pitch_mot_truc_deg']:.4f} | {dong['roll_gia_deg']:.4f} | {dong['gyroY_mot_truc_radps']:.6f} | {dong['pitch_sau_bu_deg']:.4f} |")
    (thu_muc/'TINH_TOAN_GOC_LAP.md').write_text('\n'.join(bang)+'\n',encoding='utf-8')
    print('Đã sinh sai_so_goc_lap.csv, TINH_TOAN_GOC_LAP.md và JSON minh họa.')


if __name__=='__main__':
    main()
