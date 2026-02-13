#include "ble_ota.h"

#include <Arduino.h>
#include <Update.h>
#include <esp_system.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <string>

#include "../hardware_config.h"
#include "../system/state.h"
#include "../system/sleep.h"

constexpr uint32_t OTA_CHUNK_TIMEOUT_MS     = 10000;
constexpr uint32_t OTA_RESTART_DELAY_MS     = 800;
constexpr uint32_t OTA_PROGRESS_INTERVAL_MS = 750;

static BLECharacteristic *g_otaChar          = nullptr;
static bool g_otaActive                      = false;
static uint32_t g_expectedSize               = 0;
static uint32_t g_receivedSize               = 0;
static uint32_t g_lastChunkMs                = 0;
static uint32_t g_lastProgressNotifyMs       = 0;
static bool g_restartPending                 = false;
static uint32_t g_restartAtMs                = 0;

void initOtaCharacteristic(BLECharacteristic *c) {
    g_otaChar = c;
}

bool otaInProgress() {
    return g_otaActive;
}

static void sendStatus(const char *msg) {
    if (!g_otaChar || !msg) return;
    g_otaChar->setValue((uint8_t*)msg, strlen(msg));
    g_otaChar->notify();
}

static void resetOtaState(const char *reason) {
    if (reason) {
        sendStatus(reason);
    }
    Update.abort();
    g_otaActive = false;
    g_expectedSize = 0;
    g_receivedSize = 0;
    g_lastChunkMs = 0;
    g_lastProgressNotifyMs = 0;
    g_restartPending = false;
    g_restartAtMs = 0;
}

static void maybeSendProgress() {
    if (!g_otaActive || !g_expectedSize) return;
    uint32_t now = millis();
    if (now - g_lastProgressNotifyMs < OTA_PROGRESS_INTERVAL_MS) return;
    uint32_t pct = (g_receivedSize * 100UL) / g_expectedSize;
    char buf[16];
    snprintf(buf, sizeof(buf), "PROG:%u", pct);
    sendStatus(buf);
    g_lastProgressNotifyMs = now;
}

static void finalizeOta() {
    bool ok = Update.end(true);
    if (!ok) {
        resetOtaState("ERR:END");
        return;
    }
    g_otaActive = false;
    sendStatus("OTA_OK");
    g_restartPending = true;
    g_restartAtMs = millis() + OTA_RESTART_DELAY_MS;
}

static bool startOta(uint32_t size, const std::string &md5) {
    if (g_otaActive) {
        sendStatus("ERR:BUSY");
        return false;
    }
    if (size == 0) {
        sendStatus("ERR:SIZE");
        return false;
    }

    if (g_recordingInProgress) {
        stopRecording();
    }
    markActivity();

    Update.abort();
    if (!Update.begin(size)) {
        sendStatus("ERR:BEGIN");
        return false;
    }
    if (md5.length() == 32) {
        Update.setMD5(md5.c_str());
    }

    g_otaActive = true;
    g_expectedSize = size;
    g_receivedSize = 0;
    g_lastChunkMs = millis();
    g_lastProgressNotifyMs = 0;
    sendStatus("ACK:BEGIN");
    return true;
}

static bool handleBeginMessage(const std::string &value) {
    const size_t metaStart = 6; // after "BEGIN:"
    if (value.length() <= metaStart) {
        sendStatus("ERR:SIZE");
        return false;
    }

    size_t nextColon = value.find(':', metaStart);
    std::string sizeStr;
    std::string md5;
    if (nextColon == std::string::npos) {
        sizeStr = value.substr(metaStart);
    } else {
        sizeStr = value.substr(metaStart, nextColon - metaStart);
        md5 = value.substr(nextColon + 1);
    }

    md5.erase(std::remove_if(md5.begin(), md5.end(), [](unsigned char c){ return std::isspace(c); }), md5.end());

    char *end = nullptr;
    unsigned long parsed = strtoul(sizeStr.c_str(), &end, 10);
    if (parsed == 0 || (end && *end != '\0')) {
        sendStatus("ERR:SIZE");
        return false;
    }

    return startOta(static_cast<uint32_t>(parsed), md5);
}

static void handleDataChunk(const std::string &value) {
    if (!g_otaActive || value.empty()) return;

    size_t len = value.size();
    size_t written = Update.write(reinterpret_cast<uint8_t*>(const_cast<char*>(value.data())), len);
    if (written != len) {
        resetOtaState("ERR:WRITE");
        return;
    }

    g_receivedSize += written;
    g_lastChunkMs = millis();
    markActivity();
    maybeSendProgress();

    if (g_receivedSize > g_expectedSize) {
        resetOtaState("ERR:SIZE_MISMATCH");
        return;
    }

    if (g_receivedSize == g_expectedSize) {
        finalizeOta();
    }
}

class OtaCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        std::string value = c->getValue();
        if (value.empty()) return;

        if (value.rfind("BEGIN:", 0) == 0) {
            if (g_otaActive) {
                sendStatus("ERR:BUSY");
            } else {
                handleBeginMessage(value);
            }
            return;
        }

        if (value == "ABORT") {
            resetOtaState("ERR:ABORT");
            return;
        }

        if (!g_otaActive) return;

        handleDataChunk(value);
    }
};

BLECharacteristicCallbacks *createOtaCallbacks() {
    return new OtaCharCallbacks();
}

void otaHandleDisconnected() {
    if (g_otaActive) {
        resetOtaState(nullptr);
    }
    g_restartPending = false;
    g_restartAtMs = 0;
}

void otaLoop() {
    uint32_t now = millis();
    if (g_restartPending && now >= g_restartAtMs) {
        ESP.restart();
    }

    if (g_otaActive && g_lastChunkMs > 0 && (now - g_lastChunkMs) > OTA_CHUNK_TIMEOUT_MS) {
        resetOtaState("ERR:TIMEOUT");
    }
}
