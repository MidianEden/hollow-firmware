#include "time_sync.h"

#include <time.h>
#include <cstdlib>
#include <cstdio>
#include <Preferences.h>

#include "../ble/ble_core.h"
#include "../ble/ble_audio.h"
#include "../system/state.h"
#include "../system/sleep.h"
#include "../ui/ui_common.h"
#include "../ui/ui_wait.h"

constexpr uint32_t TIME_REQ_RETRY_MS     = 7000;
constexpr uint8_t TIME_REQ_MAX_ATTEMPTS  = 5;
constexpr uint32_t TIME_RESYNC_PERIOD_MS = 60000;
constexpr const char *TIME_PREF_NAMESPACE = "time";
constexpr const char *TIME_PREF_EPOCH_KEY = "epoch";
constexpr const char *TIME_PREF_MS_KEY    = "ms";

time_t g_buildEpoch = 0;
bool g_haveHostTime = false;
bool g_waitingForTime = false;
uint8_t g_timeRequestAttempts = 0;
uint32_t g_lastTimeRequestMs = 0;
uint32_t g_lastTimeSyncMs = 0;
static Preferences g_timePrefs;
static bool g_timePrefsReady = false;

static void persistTimeState() {
    if (!g_timePrefsReady || g_buildEpoch <= 0) return;
    g_timePrefs.putLong64(TIME_PREF_EPOCH_KEY, static_cast<int64_t>(g_buildEpoch));
    g_timePrefs.putUInt(TIME_PREF_MS_KEY, g_lastTimeSyncMs);
}

static void loadStoredTime() {
    if (!g_timePrefsReady) return;
    int64_t savedEpoch = g_timePrefs.getLong64(TIME_PREF_EPOCH_KEY, 0);
    uint32_t savedMs = g_timePrefs.getUInt(TIME_PREF_MS_KEY, 0);
    if (savedEpoch <= 0) return;

    uint32_t nowMs = millis();
    g_buildEpoch = static_cast<time_t>(savedEpoch);
    g_haveHostTime = true;
    g_lastTimeSyncMs = nowMs;

    if (savedMs > 0 && nowMs > savedMs) {
        g_buildEpoch += static_cast<time_t>((nowMs - savedMs) / 1000);
    }

    uiInvalidateClock();
}

void timeSyncInit() {
    g_buildEpoch = 0;
    g_haveHostTime = false;
    g_waitingForTime = false;
    g_timeRequestAttempts = 0;
    g_lastTimeRequestMs = 0;
    g_lastTimeSyncMs = 0;

    g_timePrefsReady = g_timePrefs.begin(TIME_PREF_NAMESPACE, false);
    if (!g_timePrefsReady) {
        Serial.println("WARN: Time prefs init failed; clock persistence disabled");
    } else {
        loadStoredTime();
    }
}

void setCurrentEpoch(time_t epoch) {
    if (epoch <= 0) return;
    g_buildEpoch = epoch;
    g_haveHostTime = true;
    g_lastTimeSyncMs = millis();
    uiInvalidateClock();
    persistTimeState();
}

time_t getCurrentEpoch() {
    if (!g_haveHostTime) return 0;
    // Advance the stored epoch by the elapsed millis since the last sync
    const uint32_t elapsedMs = millis() - g_lastTimeSyncMs;
    return g_buildEpoch + static_cast<time_t>(elapsedMs / 1000);
}

void requestTimeFromHub(bool showWaitingScreen) {
    g_waitingForTime = true;
    if (showWaitingScreen && !g_sleeping) {
        resetWaitingAnimation();
    }
    g_lastTimeRequestMs = millis();
    g_lastWaitAnimMs = millis();
    if (!canSendControlMessages()) {
        g_lastTimeRequestMs = millis() - TIME_REQ_RETRY_MS;
        return;
    }
    bleSendControlMessage("REQ_TIME");
    g_timeRequestAttempts++;
}

void updateTimeRequest() {
    if (!bleIsConnected() || !bleNotifyEnabled()) return;

    const uint32_t now = millis();

    if (g_haveHostTime && !g_waitingForTime &&
        (now - g_lastTimeSyncMs) >= TIME_RESYNC_PERIOD_MS) {
        requestTimeFromHub(false);
        return;
    }

    if (!g_haveHostTime && !g_waitingForTime && g_timeRequestAttempts == 0) {
        requestTimeFromHub(false);
        return;
    }

    if (g_waitingForTime &&
        g_timeRequestAttempts < TIME_REQ_MAX_ATTEMPTS &&
        (now - g_lastTimeRequestMs) > TIME_REQ_RETRY_MS) {
        requestTimeFromHub(false);
    }

    if (g_waitingForTime &&
        g_timeRequestAttempts >= TIME_REQ_MAX_ATTEMPTS &&
        (now - g_lastTimeRequestMs) > (TIME_REQ_RETRY_MS * 4)) {
        g_timeRequestAttempts = 0;
    }
}

void timeSyncHandleConnected() {
    g_timeRequestAttempts = 0;
    g_waitingForTime = false;
    requestTimeFromHub(false);
}

void timeSyncHandleDisconnected() {
    g_timeRequestAttempts = 0;
    g_waitingForTime = false;
}

void handleTimeMessage(const std::string &value) {
    const char *payload = value.c_str() + 5;
    char *end = nullptr;
    long long epoch = strtoll(payload, &end, 10);
    long offset = 0;
    if (end && *end == ':') {
        offset = strtol(end + 1, nullptr, 10);
    }
    const bool isBackgroundSync = g_haveHostTime && currentState != WAITING_TIME;

    if (epoch > 0) {
        setCurrentEpoch((time_t)(epoch + offset));
        g_waitingForTime = false;
        g_timeRequestAttempts = 0;
        if (currentState == WAITING_TIME) {
            currentState = IDLE;
        }
        uiInvalidateClock();
        // Only mark activity (which prevents sleep) if this is NOT a background sync
        // Background time syncs should not wake the device or reset sleep timer
        if (!isBackgroundSync) {
            markActivity();
        }
    }
}

String formatClock(time_t now) {
    if (!g_haveHostTime || now <= 0) return String("00:00");
    tm *lt = localtime(&now);
    if (!lt) return String("00:00");
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", lt->tm_hour, lt->tm_min);
    return String(buf);
}
