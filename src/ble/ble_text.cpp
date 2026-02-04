#include "ble_text.h"

#include <Arduino.h>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include "../system/time_sync.h"
#include "../system/state.h"
#include "../ui/ui_answer.h"
#include "../system/sleep.h"

constexpr uint32_t TEXT_CHUNK_TIMEOUT_MS = 120;

static portMUX_TYPE g_textMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_textPending = false;
static std::string g_pendingValue;
static uint32_t g_pendingReadyAtMs = 0;

static bool popPendingText(std::string &out) {
    bool has = false;
    uint32_t now = millis();
    portENTER_CRITICAL(&g_textMux);
    if (g_textPending && now >= g_pendingReadyAtMs) {
        out.swap(g_pendingValue);
        g_textPending = false;
        has = true;
    }
    portEXIT_CRITICAL(&g_textMux);
    return has;
}

class TextCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        std::string value = c->getValue();
        if (value.empty()) return;

        portENTER_CRITICAL(&g_textMux);
        g_pendingValue.append(value);
        g_textPending = true;
        g_pendingReadyAtMs = millis() + TEXT_CHUNK_TIMEOUT_MS;
        portEXIT_CRITICAL(&g_textMux);
    }
};

BLECharacteristicCallbacks *createTextCallbacks() {
    return new TextCharCallbacks();
}

void processPendingText() {
    std::string value;
    if (!popPendingText(value)) return;

    if (value.rfind("TIME:", 0) == 0) {
        handleTimeMessage(value);
        return;
    }

    // Got a text response - clear waiting state
    g_waitingStartMs = 0;
    g_lastText = value.c_str();
    currentState = ANSWER;
    resetAnswerScrollState();
    markActivity();
    Serial.printf("[BLE] Received text response (%d chars) -> ANSWER\n", value.length());
}
