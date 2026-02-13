#include "state.h"

#include "../hardware_config.h"
#include "../audio/audio_i2s.h"
#include "../audio/audio_adpcm.h"
#include "../ble/ble_audio.h"
#include "../ble/ble_core.h"
#include "../system/sleep.h"
#include "../ui/ui_wait.h"

UIState currentState   = IDLE;
UIState lastDrawnState = (UIState)999;

bool g_bleConnected        = false;
bool g_recordingInProgress = false;

bool g_sleeping   = false;
bool g_dimmed     = false;
bool g_isCharging = false;
bool g_ignoreTap  = false;

uint32_t g_lastActivityMs      = 0;
uint32_t g_lastAdvertisingKickMs = 0;

String g_lastText;

uint32_t g_recordingStartMs = 0;
uint32_t g_waitingStartMs = 0;  // Track when we started waiting

void initState() {
    g_lastActivityMs = millis();
    g_waitingStartMs = 0;
}

void finalizeRecordingTimer() {
    if (g_recordingStartMs == 0) return;
    g_recordingStartMs = 0;
}

void startRecording() {
    if (!canSendControlMessages()) return;
    markActivity();
    startMic();
    clearRecordingBuffer();
    setRecordingActive(true);
    g_recordingInProgress = true;
    g_recordingStartMs = millis();
    g_waitingStartMs = 0;  // Clear waiting timestamp
    currentState = RECORDING;
    ima_reset_state();
    bleSendControlMessage("START_V");
}

void stopRecording() {
    markActivity();
    stopMic();
    g_recordingInProgress = false;
    setRecordingActive(false);
    finalizeRecordingTimer();
    resetWaitingAnimation();
    bool bleReady = canSendControlMessages();
    if (bleReady) {
        currentState = WAITING_ANSWER;
        g_waitingStartMs = millis();  // Start timeout clock
        bleSendControlMessage("END");
    } else {
        currentState = IDLE;
        g_waitingStartMs = 0;
    }
}

// Call from main loop to timeout waiting states
void checkWaitingTimeout() {
    if (currentState != WAITING_ANSWER && currentState != WAITING_TIME) {
        return;
    }

    if (g_waitingStartMs == 0) {
        g_waitingStartMs = millis();  // Safety: start timing if not set
        return;
    }

    uint32_t elapsed = millis() - g_waitingStartMs;
    if (elapsed >= WAITING_ANSWER_TIMEOUT_MS) {
        currentState = IDLE;
        g_waitingStartMs = 0;
        markActivity();
    }
}
