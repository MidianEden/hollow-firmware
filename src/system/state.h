#pragma once

#include <Arduino.h>
#include <WString.h>

// Global UI/application state shared across modules
enum UIState { IDLE, RECORDING, ANSWER, WAITING_TIME, WAITING_ANSWER, CHARGING };

extern UIState currentState;
extern UIState lastDrawnState;

extern bool g_bleConnected;
extern bool g_recordingInProgress;

extern bool g_sleeping;
extern bool g_dimmed;
extern bool g_isCharging;
extern bool g_ignoreTap;

extern uint32_t g_lastActivityMs;
extern uint32_t g_lastAdvertisingKickMs;

extern String g_lastText;

extern uint32_t g_recordingStartMs;
extern uint32_t g_waitingStartMs;  // When we entered WAITING_ANSWER state

// Timeout for waiting states (30 seconds)
constexpr uint32_t WAITING_ANSWER_TIMEOUT_MS = 30000;

void initState();
void startRecording();
void stopRecording();
void finalizeRecordingTimer();
void checkWaitingTimeout();  // Call from main loop to timeout waiting states
