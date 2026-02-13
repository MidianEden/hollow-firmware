#include "ble_file.h"

#include <Arduino.h>
#include <string>

#include "../hardware_config.h"
#include "../audio/audio_i2s.h"

void sendRecordedFileOverBle(BLECharacteristic *pChar) {
    if (!pChar) return;
    if (g_recorded_adpcm.empty()) {
        return;
    }

    uint32_t total_len = static_cast<uint32_t>(g_recorded_adpcm.size());
    uint8_t header[4];
    header[0] = (uint8_t)(total_len & 0xFF);
    header[1] = (uint8_t)((total_len >> 8) & 0xFF);
    header[2] = (uint8_t)((total_len >> 16) & 0xFF);
    header[3] = (uint8_t)((total_len >> 24) & 0xFF);

    pChar->setValue(header, 4);
    pChar->notify();

    const size_t CHUNK_SIZE = 128;
    size_t offset = 0;
    while (offset < g_recorded_adpcm.size()) {
        size_t remaining = g_recorded_adpcm.size() - offset;
        size_t toSend = remaining < CHUNK_SIZE ? remaining : CHUNK_SIZE;
        pChar->setValue(&g_recorded_adpcm[offset], toSend);
        pChar->notify();
        offset += toSend;
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

class FileCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        std::string value = c->getValue();
        if (value.size() == 1 && value[0] == 0x01) {
            sendRecordedFileOverBle(c);
        }
    }
};

BLECharacteristicCallbacks *createFileCallbacks() {
    return new FileCharCallbacks();
}
