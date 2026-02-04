#include "ble_audio.h"

#include <cstring>

#include "ble_core.h"

void bleSendAudioChunk(const uint8_t *data, size_t len) {
    BLECharacteristic *c = bleGetAudioChar();
    if (!c) return;
    // BLECharacteristic::setValue expects non-const buffer
    c->setValue(const_cast<uint8_t*>(data), len);
    c->notify();
}

void bleSendControlMessage(const char *msg) {
    BLECharacteristic *c = bleGetAudioChar();
    if (!c || !msg) return;
    c->setValue((uint8_t*)msg, strlen(msg));
    c->notify();
}
