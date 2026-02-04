#pragma once

// =============================================================================
// AUDIO I2S - Persistent PDM Microphone Driver
// =============================================================================
// The I2S driver is installed once at boot and kept running for instant
// recording start. This eliminates the ~50ms delay of driver initialization.
// =============================================================================

#include <Arduino.h>
#include <vector>

constexpr size_t MAX_RECORDING_BYTES = 120000;

extern bool g_is_recording;
extern std::vector<uint8_t> g_recorded_adpcm;

// Lifecycle management
void initMic();       // Install I2S driver (but don't start capture)
void startMic();      // Start active capture (will auto-init if needed)
void stopMic();       // Stop capture (driver remains installed)
void deinitMic();     // Fully uninstall driver and release GPIO (deep sleep only)

// State queries
bool isMicRunning();    // True if actively capturing
bool isMicInstalled();  // True if driver is installed (even if stopped)

// Idle check - NO-OP (driver is persistent)
void micIdleCheck();

// Recording loop - call while recording to stream audio over BLE
void updateRecording();

// Buffer management
void clearRecordingBuffer();
void setRecordingActive(bool active);
bool isRecordingActive();

// Recording diagnostics
uint32_t getRecordingDurationMs();
uint32_t getRecordingChunksSent();
uint32_t getRecordingErrors();
