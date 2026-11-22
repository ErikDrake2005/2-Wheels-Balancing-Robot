#include "BoGhiRungEspNow.h"
#include "esp_timer.h"

bool BoGhiRungEspNow::begin(LienKetChanDoanEspNow& link, uint16_t sampleHz) {
    if (!sampleHz) return false;
    sensorHz = sampleHz;
    samples = xQueueCreate(64, sizeof(Item));
    information = xQueueCreate(1, sizeof(KhaoSatRung::Info));
    if (!samples || !information) {
        if (samples) vQueueDelete(samples);
        if (information) vQueueDelete(information);
        samples = information = nullptr;
        return false;
    }
    radio = &link;
    if (xTaskCreatePinnedToCore(entry, "VibrationLog", 4096, this, 1, &task, 0) != pdPASS) {
        vQueueDelete(samples); vQueueDelete(information);
        samples = information = nullptr;
        return false;
    }
    return true;
}

bool BoGhiRungEspNow::setEnabled(bool on) {
    if (!task) return false;
    if (on == enabled()) return true;
    if (on) {
        if (!++lastId) ++lastId;
        const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
        portENTER_CRITICAL(&epochMux);
        startedUs = now;
        epoch.store(lastId);
        portEXIT_CRITICAL(&epochMux);
    } else epoch.store(0);
    return true;
}

void BoGhiRungEspNow::record(const KhaoSatRung::Sample& sample) {
    portENTER_CRITICAL(&epochMux);
    const uint32_t id = epoch.load();
    const uint64_t start = startedUs;
    portEXIT_CRITICAL(&epochMux);
    if (!id) return;
    if (sample.t_us < start) return;
    ++count;
    if (!KhaoSatRung::valid(sample)) { ++dropped; return; }
    const Item item{id, sample};
    if (xQueueSend(samples, &item, 0) != pdTRUE) ++dropped;
}

void BoGhiRungEspNow::updateInfo(const KhaoSatRung::Info& info) {
    if (information) xQueueOverwrite(information, &info);
}

void BoGhiRungEspNow::entry(void* arg) { static_cast<BoGhiRungEspNow*>(arg)->run(); }

void BoGhiRungEspNow::run() {
    uint64_t nextInfo = 0;
    uint32_t previousId = UINT32_MAX;
    uint32_t previousCfg = 0;
    uint8_t previousEnabled = 255;
    for (;;) {
        KhaoSatRung::Info info{};
        const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
        if (xQueuePeek(information, &info, 0) == pdTRUE &&
            (now >= nextInfo || info.log_id != previousId || info.enabled != previousEnabled || info.cfg != previousCfg)) {
            info.t_us = now;
            info.produced = count.load(); info.queue_drop = dropped.load(); info.tx_drop = txDropped.load();
            bool infoSent = false;
            for (unsigned retry = 0; retry < 4 && !infoSent && radio->logReceiverReady(now); ++retry) {
                infoSent = radio->sendLog(info);
                if (!infoSent) vTaskDelay(pdMS_TO_TICKS(1));
            }
            if (infoSent) {
                nextInfo = now + 1000000u;
                previousId = info.log_id; previousEnabled = info.enabled;
                previousCfg = info.cfg;
            }
        }
        Item item{};
        if (xQueueReceive(samples, &item, pdMS_TO_TICKS(5)) != pdTRUE) continue;
        KhaoSatRung::Samples packet{};
        packet.log_id = item.log_id;
        packet.sample_hz = sensorHz;
        packet.sample[0] = item.sample;
        packet.count = 1;
        // Wait at most one sample period for the second item; preserve short final batches.
        Item second{};
        if (xQueuePeek(samples, &second, pdMS_TO_TICKS(5)) == pdTRUE && second.log_id == item.log_id &&
            xQueueReceive(samples, &second, 0) == pdTRUE) {
            packet.sample[1] = second.sample; packet.count = 2;
        }
        packet.queue_drop = dropped.load(); packet.tx_drop = txDropped.load();
        bool sent = false;
        for (unsigned retry = 0; retry < 4 && !sent && radio->logReceiverReady(now); ++retry) {
            sent = radio->sendLog(packet);
            if (!sent) vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (!sent) txDropped.fetch_add(packet.count);
    }
}
