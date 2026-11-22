#pragma once
#include <atomic>
#include "LienKetChanDoanEspNow.h"
#include "freertos/queue.h"

// ControlTask copies samples to a bounded queue. Only the lower-priority task sends RF.
class BoGhiRungEspNow {
public:
    bool begin(LienKetChanDoanEspNow& link, uint16_t sampleHz);
    bool setEnabled(bool enabled);
    bool enabled() const { return epoch.load() != 0; }
    uint32_t logId() const { return lastId; } // main task only
    void record(const KhaoSatRung::Sample& sample);
    void updateInfo(const KhaoSatRung::Info& info);
    uint32_t produced() const { return count.load(); }
    uint32_t queueDrop() const { return dropped.load(); }
    uint32_t txDrop() const { return txDropped.load(); }
private:
    struct Item { uint32_t log_id; KhaoSatRung::Sample sample; };
    std::atomic<uint32_t> epoch{0}, count{0}, dropped{0}, txDropped{0};
    uint32_t lastId = 0;
    portMUX_TYPE epochMux = portMUX_INITIALIZER_UNLOCKED;
    uint64_t startedUs = 0;
    QueueHandle_t samples = nullptr, information = nullptr;
    TaskHandle_t task = nullptr;
    LienKetChanDoanEspNow* radio = nullptr;
    uint16_t sensorHz = 0;
    static void entry(void* arg);
    void run();
};
