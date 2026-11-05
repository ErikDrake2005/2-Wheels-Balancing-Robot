"""Hiệu chuẩn phép quay sensor->xe từ hai vector; không tự sửa firmware/config."""
import argparse
import json
import math
from pathlib import Path
import statistics
import sys


def tich_vo_huong(a, b):
    return sum(x*y for x,y in zip(a,b))


def tich_co_huong(a, b):
    return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]


def do_lon(vecto):
    return math.sqrt(tich_vo_huong(vecto,vecto))


def chuan_hoa(vecto):
    if len(vecto)!=3 or not all(math.isfinite(x) for x in vecto):
        raise ValueError("Vector phải có 3 số hữu hạn")
    norm=do_lon(vecto)
    if norm<1e-9:
        raise ValueError("Vector quá nhỏ")
    return [x/norm for x in vecto]


def tao_ma_tran(gia_toc_dung, gyro_nghieng_tien):
    if abs(do_lon(gia_toc_dung)-9.80665)>0.5:
        raise ValueError("Norm gia tốc tư thế chuẩn không gần g trong ±0,5 m/s²")
    if do_lon(gyro_nghieng_tien)<0.05:
        raise ValueError("Gyro quá nhỏ; cần đoạn nghiêng tiến, không lấy cả lượt tiến và lùi")
    truc_len=chuan_hoa(gia_toc_dung)
    chuan_hoa(gyro_nghieng_tien) # kiểm tra NaN/Inf trước phép chiếu
    phan_dung=tich_vo_huong(gyro_nghieng_tien,truc_len)
    ty_le_dung=abs(phan_dung)/do_lon(gyro_nghieng_tien)
    if ty_le_dung>0.25:
        raise ValueError("Gyro có quá nhiều thành phần quay quanh trục đứng")
    truc_trai=chuan_hoa([gyro_nghieng_tien[i]-phan_dung*truc_len[i] for i in range(3)])
    truc_tien=chuan_hoa(tich_co_huong(truc_trai,truc_len))
    truc_trai=tich_co_huong(truc_len,truc_tien)
    ma_tran=[truc_tien,truc_trai,truc_len]
    return {"R_camBienSangXe":ma_tran,
            "det":tich_vo_huong(truc_tien,tich_co_huong(truc_trai,truc_len)),
            "ty_le_gyro_truc_dung":ty_le_dung,
            "gia_toc_dung_mps2":gia_toc_dung,"gyro_nghieng_tien_radps":gyro_nghieng_tien,
            "luu_y":"Tư thế chuẩn và chiều nghiêng TIẾN do người đo xác nhận; chưa tự ghi vào config."}


def doc_mau(duong_dan):
    cac_mau=[]
    thoi_diem_truoc=-1
    for dong in Path(duong_dan).read_text(encoding="utf-8-sig").splitlines():
        cot=dong.strip().split(',')
        if cot[0]=="RAW_IMU": cot=cot[1:]
        elif not cot[0].isdigit(): continue # bỏ log/status/header
        if len(cot)!=7: raise ValueError("Cần timestamp,ax,ay,az,gx,gy,gz")
        so=[float(x) for x in cot]
        if not all(math.isfinite(x) for x in so): raise ValueError("Log chứa NaN/Inf")
        if so[0]<thoi_diem_truoc: raise ValueError("Timestamp lùi; không ghép nhiều lần boot")
        if so[0]==thoi_diem_truoc: continue
        thoi_diem_truoc=so[0]
        cac_mau.append(so)
    if len(cac_mau)<5: raise ValueError("Cần ít nhất 5 mẫu khác timestamp mỗi giai đoạn")
    return cac_mau


def tu_log(tep_dung,tep_nghieng):
    dung=doc_mau(tep_dung)
    if max(do_lon(mau[4:7]) for mau in dung)>0.03:
        raise ValueError("Gyro tư thế đứng còn lớn; giữ xe yên và hiệu chuẩn bias trước")
    if max(statistics.pstdev(mau[truc] for mau in dung) for truc in [1,2,3])>0.15:
        raise ValueError("Gia tốc tư thế đứng biến thiên quá lớn")
    gia_toc=[statistics.mean(mau[truc] for mau in dung) for truc in [1,2,3]]
    gyro_du=[statistics.mean(mau[truc] for mau in dung) for truc in [4,5,6]]
    nghieng=[]
    for mau in doc_mau(tep_nghieng):
        da_tru_du=mau[:4]+[mau[4+truc]-gyro_du[truc] for truc in range(3)]
        if do_lon(da_tru_du[4:7])>=0.05: nghieng.append(da_tru_du)
    if len(nghieng)<5: raise ValueError("Thiếu mẫu đang nghiêng tiến; đoạn đứng yên được loại")
    gyro=[statistics.mean(mau[truc] for mau in nghieng) for truc in [4,5,6]]
    huong=chuan_hoa(gyro)
    sai_lech_huong=max(math.acos(max(-1,min(1,tich_vo_huong(chuan_hoa(mau[4:7]),huong)))) for mau in nghieng)
    if sai_lech_huong>math.radians(8):
        raise ValueError("Hướng quay không nhất quán; không dùng đoạn quay về hoặc rung lắc nhiều trục")
    ket_qua=tao_ma_tran(gia_toc,gyro)
    ket_qua.update(so_mau_dung=len(dung),so_mau_nghieng=len(nghieng),sai_lech_huong_max_deg=math.degrees(sai_lech_huong))
    ket_qua['gyro_du_trung_binh_dung_radps']=gyro_du
    return ket_qua


def mau_config(ma_tran):
    cac_dong=["// Thay nội dung tương ứng trong ImuMountConfig.h sau khi kiểm tra chiều.",
              "constexpr bool dungMaTranLapImu = true;",
              "constexpr float maTranCamBienSangXe[3][3] = {"]
    cac_dong += ["    {"+", ".join(f"{x:.9f}f" for x in hang)+"}," for hang in ma_tran]
    return "\n".join(cac_dong+["};",""])


def main():
    for luong in (sys.stdout,sys.stderr):
        if hasattr(luong,'reconfigure'): luong.reconfigure(encoding='utf-8')
    tham_so=argparse.ArgumentParser(description=__doc__)
    tham_so.add_argument('--dung',type=Path,help='Log khi khung xe ở tư thế thẳng đã biết')
    tham_so.add_argument('--nghieng',type=Path,help='Log đoạn chỉ nghiêng TIẾN quanh trục bánh')
    tham_so.add_argument('--gia-toc',nargs=3,type=float,help='Accel trung bình tư thế chuẩn, m/s² hệ sensor')
    tham_so.add_argument('--gyro',nargs=3,type=float,help='Gyro nghiêng tiến đã trừ bias, rad/s hệ sensor')
    tham_so.add_argument('--out',type=Path,default=Path('.cache/ma_tran_lap_imu.json'))
    lua_chon=tham_so.parse_args()
    try:
        if lua_chon.dung and lua_chon.nghieng and lua_chon.gia_toc is None and lua_chon.gyro is None:
            ket_qua=tu_log(lua_chon.dung,lua_chon.nghieng)
        elif lua_chon.gia_toc is not None and lua_chon.gyro is not None and not lua_chon.dung and not lua_chon.nghieng:
            ket_qua=tao_ma_tran(lua_chon.gia_toc,lua_chon.gyro)
        else: raise ValueError("Chọn cặp --dung/--nghieng hoặc cặp --gia-toc/--gyro")
    except ValueError as loi:
        tham_so.error(str(loi))
    lua_chon.out.parent.mkdir(parents=True,exist_ok=True)
    lua_chon.out.write_text(json.dumps(ket_qua,ensure_ascii=False,indent=2),encoding='utf-8')
    lua_chon.out.with_suffix('.h.txt').write_text(mau_config(ket_qua['R_camBienSangXe']),encoding='utf-8')
    print(mau_config(ket_qua['R_camBienSangXe']))
    print('Đã ghi',lua_chon.out,'; chưa thay config trên xe.')


if __name__=='__main__':
    main()
