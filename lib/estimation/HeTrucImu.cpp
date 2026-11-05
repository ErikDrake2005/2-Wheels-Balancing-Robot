#include "HeTrucImu.h"
#include "config/ImuMountConfig.h"
#include <limits>

HeTrucImu::HeTrucImu() {
    if (dungMaTranLapImu) {
        for (unsigned hang=0;hang<3;++hang) for(unsigned cot=0;cot<3;++cot)
            _maTran.phanTu[hang][cot]=maTranCamBienSangXe[hang][cot];
    } else _maTran=tuGocLap_deg(gocLapRoll_deg,gocLapPitch_deg,gocLapYaw_deg);
    _hopLe=kiemTraMaTran(_maTran);
}
HeTrucImu::HeTrucImu(const MaTranQuayImu& maTran) : _maTran(maTran), _hopLe(kiemTraMaTran(maTran)) {}

MaTranQuayImu HeTrucImu::tuGocLap_deg(float roll_deg,float pitch_deg,float yaw_deg) {
    constexpr float doiRad=0.017453292519943295f;
    const float cr=std::cos(roll_deg*doiRad),sr=std::sin(roll_deg*doiRad);
    const float cp=std::cos(pitch_deg*doiRad),sp=std::sin(pitch_deg*doiRad);
    const float cy=std::cos(yaw_deg*doiRad),sy=std::sin(yaw_deg*doiRad);
    return {{{cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr},
             {sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr},
             {-sp,cp*sr,cp*cr}}};
}
bool HeTrucImu::kiemTraMaTran(const MaTranQuayImu& maTran) {
    Vec3Imu cacHang[3];
    for(unsigned hang=0;hang<3;++hang) {
        cacHang[hang]={maTran.phanTu[hang][0],maTran.phanTu[hang][1],maTran.phanTu[hang][2]};
        if (!vectoHuuHan(cacHang[hang])) return false;
    }
    for(unsigned hang=0;hang<3;++hang) for(unsigned cot=0;cot<3;++cot) {
        if (std::fabs(tichVoHuong(cacHang[hang],cacHang[cot])-(hang==cot ? 1.0f : 0.0f))>1e-3f)
            return false;
    }
    return std::fabs(tichVoHuong(cacHang[0],tichCoHuong(cacHang[1],cacHang[2]))-1.0f)<2e-3f;
}
Vec3Imu HeTrucImu::chuyenSangXe(const Vec3Imu& vectoCamBien) const {
    if (!_hopLe || !vectoHuuHan(vectoCamBien)) {
        const float loi=std::numeric_limits<float>::quiet_NaN();
        return {loi,loi,loi};
    }
    Vec3Imu ketQua;
    ketQua.x=_maTran.phanTu[0][0]*vectoCamBien.x+_maTran.phanTu[0][1]*vectoCamBien.y+_maTran.phanTu[0][2]*vectoCamBien.z;
    ketQua.y=_maTran.phanTu[1][0]*vectoCamBien.x+_maTran.phanTu[1][1]*vectoCamBien.y+_maTran.phanTu[1][2]*vectoCamBien.z;
    ketQua.z=_maTran.phanTu[2][0]*vectoCamBien.x+_maTran.phanTu[2][1]*vectoCamBien.y+_maTran.phanTu[2][2]*vectoCamBien.z;
    return ketQua;
}
bool HeTrucImu::tuHaiVector(const Vec3Imu& giaTocDung,const Vec3Imu& gyroNghiengTien,
                          MaTranQuayImu& ketQua) {
    const float normGiaToc=doLonVecto(giaTocDung),normGyro=doLonVecto(gyroNghiengTien);
    if (!vectoHuuHan(giaTocDung) || !vectoHuuHan(gyroNghiengTien) ||
        !std::isfinite(normGiaToc) || !std::isfinite(normGyro) ||
        std::fabs(normGiaToc-9.80665f)>0.5f || normGyro<0.05f) return false;
    Vec3Imu trucLen=giaTocDung;
    if (!chuanHoaVecto(trucLen)) return false;
    const float thanhPhanQuayDung=tichVoHuong(gyroNghiengTien,trucLen);
    if (std::fabs(thanhPhanQuayDung)/normGyro>0.25f) return false;
    Vec3Imu trucTrai={gyroNghiengTien.x-thanhPhanQuayDung*trucLen.x,
                     gyroNghiengTien.y-thanhPhanQuayDung*trucLen.y,
                     gyroNghiengTien.z-thanhPhanQuayDung*trucLen.z};
    if (!chuanHoaVecto(trucTrai)) return false;
    Vec3Imu trucTien=tichCoHuong(trucTrai,trucLen);
    if (!chuanHoaVecto(trucTien)) return false;
    trucTrai=tichCoHuong(trucLen,trucTien);
    const MaTranQuayImu maTran={{{trucTien.x,trucTien.y,trucTien.z},
                                {trucTrai.x,trucTrai.y,trucTrai.z},
                                {trucLen.x,trucLen.y,trucLen.z}}};
    if (!kiemTraMaTran(maTran)) return false;
    ketQua=maTran;
    return true;
}
