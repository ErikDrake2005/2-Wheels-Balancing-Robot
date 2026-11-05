#include "GiamSatDongCo.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include <cstdio>
#include <cstdlib>

struct BoHenGia { gptimer_alarm_cb_t hamNgat=nullptr; void* nguCanh=nullptr; };
gpio_dev_t GPIO;
static int64_t thoiDiemGia_us=0;
static BoHenGia* boHenMoiNhat=nullptr;
static bool lamLoiKhoiTao=false;
static unsigned soKiemTra=0;
void kiemTra(bool dieuKien,const char* moTa) {
    ++soKiemTra;
    if (!dieuKien) { std::printf("FAIL: %s\n",moTa); std::exit(1); }
}
int64_t esp_timer_get_time() { return thoiDiemGia_us; }
esp_err_t gpio_set_level(gpio_num_t chan,uint32_t muc) { GPIO.mucChan[chan]=muc; return ESP_OK; }
esp_err_t gpio_set_direction(gpio_num_t,int) { return ESP_OK; }
esp_err_t gptimer_new_timer(const gptimer_config_t*,gptimer_handle_t* boHen) {
    if (lamLoiKhoiTao) return -1;
    *boHen=boHenMoiNhat=new BoHenGia; return ESP_OK;
}
esp_err_t gptimer_del_timer(gptimer_handle_t boHen) { delete boHen; return ESP_OK; }
esp_err_t gptimer_register_event_callbacks(gptimer_handle_t boHen,const gptimer_event_callbacks_t* suKien,void* nguCanh) {
    boHen->hamNgat=suKien->on_alarm; boHen->nguCanh=nguCanh; return ESP_OK;
}
esp_err_t gptimer_set_alarm_action(gptimer_handle_t,const gptimer_alarm_config_t*) { return ESP_OK; }
esp_err_t gptimer_enable(gptimer_handle_t) { return ESP_OK; }
esp_err_t gptimer_disable(gptimer_handle_t) { return ESP_OK; }
esp_err_t gptimer_start(gptimer_handle_t) { return ESP_OK; }
void chayNgatTai(int64_t thoiDiem_us) {
    thoiDiemGia_us=thoiDiem_us;
    boHenMoiNhat->hamNgat(boHenMoiNhat,nullptr,boHenMoiNhat->nguCanh);
}
int main() {
    GiamSatDongCo giamSat;
    kiemTra(giamSat.begin(2,false,2000,250),"khoi tao giam sat");
    kiemTra(GPIO.mucChan[2]==1,"mac dinh EN tat");
    kiemTra(giamSat.bat(20000) && GPIO.mucChan[2]==0,"bat EN khi co deadline");
    chayNgatTai(1999);
    kiemTra(!giamSat.daNgat(),"chua qua han");
    chayNgatTai(2000); // Không gọi bất kỳ hàm nào của MotorTask.
    kiemTra(giamSat.daNgat() && GPIO.mucChan[2]==1,"ISR tu tat EN khi MotorTask ngung");
    kiemTra(!giamSat.giaHan(30000) && !giamSat.bat(30000),"task quay lai khong tu xoa loi");
    giamSat.tat();
    kiemTra(giamSat.daNgat(),"disable khong xoa loi");
    kiemTra(giamSat.xoaLoi() && giamSat.bat(5000),"clear roi bat lai");
    thoiDiemGia_us=3000;
    kiemTra(giamSat.giaHan(5000),"gia han nhip phuc vu");
    thoiDiemGia_us=4000;
    kiemTra(giamSat.giaHan(5000),"gia han nhip giu deadline lenh goc");
    chayNgatTai(5000);
    kiemTra(giamSat.daNgat() && GPIO.mucChan[2]==1,"lenh het han du task van phuc vu");
    giamSat.xoaLoi();
    kiemTra(giamSat.bat(20000),"bat de thu gia han den tre");
    thoiDiemGia_us=7001; // ISR chưa có cơ hội chạy: task cũng không được hồi sinh deadline.
    kiemTra(!giamSat.giaHan(30000) && giamSat.daNgat() && GPIO.mucChan[2]==1,
            "feed muon tu ngat truoc khi gia han");
    giamSat.xoaLoi(); giamSat.bat(30000);
    kiemTra(!giamSat.xoaLoi(),"khong clear khi EN dang bat");
    giamSat.tat();
    chayNgatTai(50000);
    kiemTra(!giamSat.daNgat() && GPIO.mucChan[2]==1,"disabled khong bao timeout");
    GiamSatDongCo giamSatMucCao;
    kiemTra(giamSatMucCao.begin(3,true,2000,250),"cau hinh EN active high");
    kiemTra(giamSatMucCao.bat(60000) && GPIO.mucChan[3]==1,"EN high khi bat");
    chayNgatTai(52000);
    kiemTra(giamSatMucCao.daNgat() && GPIO.mucChan[3]==0,"EN low khi ngat");
    lamLoiKhoiTao=true;
    GiamSatDongCo giamSatLoi;
    kiemTra(!giamSatLoi.begin(4,false,2000,250) && !giamSatLoi.bat(60000) && GPIO.mucChan[4]==1,
            "loi tao timer giu EN tat");
    std::printf("PASS: GiamSatDongCo production code + fake GPTimer/GPIO, %u checks\n",soKiemTra);
}
