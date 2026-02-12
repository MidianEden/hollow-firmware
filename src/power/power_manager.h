#pragma once

// =============================================================================
// POWER MANAGER - Central power control for T-Watch S3
// =============================================================================
// This module owns ALL power decisions. No other module should directly
// call esp_light_sleep_start() or modify CPU frequency.
//
// OPTIMIZED FOR:
// - Instant wake response (<100ms from tap to screen)
// - Maximum battery life while maintaining BLE connection
// - Smooth state transitions with no glitches
// =============================================================================

#include <Arduino.h>
#include <esp_pm.h>
#include <esp_sleep.h>

// -----------------------------------------------------------------------------
// Power States
// -----------------------------------------------------------------------------
enum PowerState {
    POWER_ACTIVE,       // Display on, full CPU, BLE active
    POWER_DIMMED,       // Display dimmed, full CPU, BLE active
    POWER_LIGHT_SLEEP,  // Display off, CPU sleeps between events, BLE active
    POWER_DEEP_SLEEP    // Everything off except RTC, requires full reboot
};

extern PowerState g_powerState;

// -----------------------------------------------------------------------------
// Configuration Constants - TUNED FOR RESPONSIVENESS + BATTERY
// -----------------------------------------------------------------------------

// CPU Frequency limits (MHz)
// Higher max freq improves wake response and BLE throughput
constexpr int CPU_FREQ_MAX = 160;     // Increased from 80 for faster wake + BLE
constexpr int CPU_FREQ_MIN = 10;      // Min freq during light sleep idle

// Timeouts (milliseconds) - OPTIMIZED FOR BATTERY LIFE
// Total time to deep sleep = TIMEOUT_LIGHT_SLEEP_MS + TIMEOUT_DEEP_SLEEP_MS = 300s (5 min)
constexpr uint32_t TIMEOUT_DIM_MS         = 10000;   // Dim after 10s
constexpr uint32_t TIMEOUT_LIGHT_SLEEP_MS = 20000;   // Light sleep after 20s
constexpr uint32_t TIMEOUT_DEEP_SLEEP_MS  = 280000;  // 280s in light sleep + 20s = 5 min total idle

// Wake timing targets
constexpr uint32_t WAKE_TARGET_MS = 50;    // Target wake time in ms
constexpr uint32_t WAKE_MAX_MS = 100;      // Maximum acceptable wake time

// Brownout thresholds - slightly more conservative for stability
constexpr int BROWNOUT_THRESHOLD_MV = 3000;  // Warn user below this (was 2800)
constexpr int SHUTDOWN_THRESHOLD_MV = 2700;  // Force shutdown below this (was 2650)

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

// Call once in setup() BEFORE any other init
// Returns true if power management configured successfully
bool powerManagerInit();

// -----------------------------------------------------------------------------
// Activity Tracking
// -----------------------------------------------------------------------------

// Call when user interacts (touch, BLE message received)
// Resets sleep timers and wakes from light sleep if needed
void powerMarkActivity();

// Call when BLE connects/disconnects
void powerHandleBLEConnect();
void powerHandleBLEDisconnect();

// -----------------------------------------------------------------------------
// State Transitions
// -----------------------------------------------------------------------------

// Called from main loop - handles automatic state transitions
// Returns true if CPU can proceed with normal work
// Returns false if we just woke from light sleep (minimal work needed)
bool powerUpdate();

// Force immediate state change (use sparingly)
void powerForceActive();
void powerForceLightSleep();
void powerForceDeepSleep();  // WARNING: This does not return!

// -----------------------------------------------------------------------------
// Query Functions
// -----------------------------------------------------------------------------

bool powerIsActive();
bool powerIsDimmed();
bool powerIsLightSleep();
bool powerCanDoWork();  // True if not in deep sleep transition

// Get time since last activity (for UI timeout decisions)
uint32_t powerGetIdleTimeMs();

// -----------------------------------------------------------------------------
// Wake Handling
// -----------------------------------------------------------------------------

// Flag set when device wakes from light sleep via touch
// Main loop MUST check this and call handleWakeFromLightSleep() BEFORE processing input
extern volatile bool g_wokeFromSleep;

// Called immediately after wake - resets UI to HOME, reinits display
// Returns only after display is fully on and showing HOME screen
void handleWakeFromLightSleep();

// Call early in setup() after powerManagerInit() and Serial init.
// Validates deep sleep wake cause. If spurious, goes back to deep sleep (does NOT return).
void powerValidateWake();

// -----------------------------------------------------------------------------
// Diagnostics
// -----------------------------------------------------------------------------

// Print current power state and measurements to Serial
void powerPrintDiagnostics();

// Get estimated current draw in mA (rough approximation)
float powerEstimateCurrentMa();
