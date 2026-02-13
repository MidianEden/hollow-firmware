// =============================================================================
// HOLLOW WATCH - MAIN FIRMWARE
// =============================================================================
// Optimized for T-Watch S3 (ESP32-S3)
// Features:
//   - Fast tap-to-wake (<100ms response)
//   - Smooth 60fps UI when active
//   - Stable BLE connection
//   - Low power standby with instant wake
//   - Battery percentage always visible
// =============================================================================

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Hardware config
#include "hardware_config.h"

// UI
#include "ui/ui_common.h"
#include "ui/ui_idle.h"
#include "ui/ui_record.h"
#include "ui/ui_answer.h"
#include "ui/ui_wait.h"

// Subsystems
#include "ble/ble_core.h"
#include "ble/ble_text.h"
#include "ble/ble_ota.h"
#include "audio/audio_i2s.h"
#include "power/pmu.h"
#include "power/battery.h"
#include "power/power_manager.h"
#include "input/touch.h"
#include "system/time_sync.h"
#include "system/state.h"

// =============================================================================
// FIRMWARE VERSION
// =============================================================================
#define FIRMWARE_VERSION "1.2.0"
#define BUILD_DATE __DATE__ " " __TIME__

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    // -------------------------------------------------------------------------
    // 0. SERIAL - for diagnostics (enable ARDUINO_USB_CDC_ON_BOOT=1 to see)
    // -------------------------------------------------------------------------
    Serial.begin(115200);

    // -------------------------------------------------------------------------
    // 1. POWER MANAGER FIRST
    // -------------------------------------------------------------------------
    // This disables WiFi, sets CPU frequency, and configures power management
    // MUST happen before any other initialization
    powerManagerInit();

    // -------------------------------------------------------------------------
    // 2.5. VALIDATE DEEP SLEEP WAKE - may not return if spurious
    // -------------------------------------------------------------------------
    // If woken from deep sleep by an uncleared touch INT (no real finger),
    // this goes back to deep sleep immediately without full init.
    powerValidateWake();

    esp_reset_reason_t resetReason = esp_reset_reason();

    // Check wake reason and handle deep sleep wake
    esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
    bool wokeFromDeepSleep = (resetReason == ESP_RST_DEEPSLEEP) ||
                              (wakeReason == ESP_SLEEP_WAKEUP_EXT0 ||
                               wakeReason == ESP_SLEEP_WAKEUP_EXT1);

    // Log reset/wake reason
    const char *rstStr = "POWERON";
    switch (resetReason) {
        case ESP_RST_DEEPSLEEP: rstStr = "DEEPSLEEP"; break;
        case ESP_RST_PANIC:     rstStr = "PANIC"; break;
        case ESP_RST_TASK_WDT:  rstStr = "WDT"; break;
        case ESP_RST_INT_WDT:   rstStr = "INT_WDT"; break;
        case ESP_RST_BROWNOUT:  rstStr = "BROWNOUT"; break;
        case ESP_RST_SW:        rstStr = "SW_RESET"; break;
        default: break;
    }
    const char *wakeStr = "NONE";
    switch (wakeReason) {
        case ESP_SLEEP_WAKEUP_EXT0:      wakeStr = "TOUCH(EXT0)"; break;
        case ESP_SLEEP_WAKEUP_EXT1:      wakeStr = "BUTTON(EXT1)"; break;
        case ESP_SLEEP_WAKEUP_TIMER:     wakeStr = "TIMER"; break;
        case ESP_SLEEP_WAKEUP_GPIO:      wakeStr = "GPIO"; break;
        case ESP_SLEEP_WAKEUP_UNDEFINED: wakeStr = "UNDEF"; break;
        default: break;
    }
    Serial.printf("[BOOT] reset=%s wake=%s deep_sleep_wake=%d\n",
                  rstStr, wakeStr, wokeFromDeepSleep);

    // -------------------------------------------------------------------------
    // 3. Watchdog (30 second timeout)
    // -------------------------------------------------------------------------
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);

    // -------------------------------------------------------------------------
    // 4. PMU (controls power rails)
    // -------------------------------------------------------------------------
    g_pmuPresent = initPMU();

    // -------------------------------------------------------------------------
    // 5. Display
    // -------------------------------------------------------------------------
    uiInitDisplay();

    // Skip boot animation if waking from deep sleep (faster wake)
    if (!wokeFromDeepSleep) {
        playBootAnimation();
    }

    // -------------------------------------------------------------------------
    // 6. State and timekeeping
    // -------------------------------------------------------------------------
    initState();
    timeSyncInit();
    initBatterySimulator();

    g_lastWaitAnimMs = millis();
    g_waitingDots = 0;

    // -------------------------------------------------------------------------
    // 7. BLE (after PMU and display are ready)
    // -------------------------------------------------------------------------
    initBLE();

    // -------------------------------------------------------------------------
    // 8. Mic - Initialize I2S driver at boot for instant recording
    // -------------------------------------------------------------------------
    initMic();  // Install I2S driver once, never uninstall

    // -------------------------------------------------------------------------
    // 9. Final setup
    // -------------------------------------------------------------------------
    delay(50);  // Reduced from 100ms
    testBatteryDisplay();

    drawIdleScreen();
    lastDrawnState = IDLE;

    updateChargingState();

}

// =============================================================================
// MAIN LOOP - OPTIMIZED FOR RESPONSIVENESS + POWER
// =============================================================================
// Key design principles:
// 1. Fast wake handling - check g_wokeFromSleep FIRST
// 2. Minimal work during light sleep state
// 3. Adaptive frame pacing for smooth UI
// 4. Event-driven touch and BLE (via callbacks/interrupts)
// =============================================================================

void loop() {
    // -------------------------------------------------------------------------
    // Watchdog reset
    // -------------------------------------------------------------------------
    esp_task_wdt_reset();

    // -------------------------------------------------------------------------
    // WAKE HANDLER - MUST RUN FIRST
    // -------------------------------------------------------------------------
    // Critical for fast wake response. If we woke from light sleep:
    // 1. Turn on display immediately
    // 2. Show HOME screen
    // 3. Consume the wake tap (don't forward to UI)
    if (g_wokeFromSleep) {
        handleWakeFromLightSleep();
        return;  // Skip rest of loop this iteration
    }

    // -------------------------------------------------------------------------
    // Power manager state update
    // -------------------------------------------------------------------------
    powerUpdate();

    // -------------------------------------------------------------------------
    // Handle touch input
    // -------------------------------------------------------------------------
    handleTouch();

    // -------------------------------------------------------------------------
    // BLE maintenance (event callbacks handle most work)
    // -------------------------------------------------------------------------
    processPendingText();
    otaLoop();
    ensureAdvertisingAlive();

    // -------------------------------------------------------------------------
    // Check for waiting state timeout (prevents infinite wait)
    // -------------------------------------------------------------------------
    checkWaitingTimeout();

    // -------------------------------------------------------------------------
    // Recording (only when active)
    // -------------------------------------------------------------------------
    if (g_recordingInProgress) {
        updateRecording();
    }

    // -------------------------------------------------------------------------
    // UI updates (skip during light sleep to save power)
    // -------------------------------------------------------------------------
    if (!powerIsLightSleep()) {
        // Charging state check
        updateChargingState();

        // Time sync
        updateTimeRequest();

        // Screen state machine
        if (currentState != lastDrawnState) {
            switch (currentState) {
                case IDLE:           drawIdleScreen(); break;
                case RECORDING:      drawRecordingScreen(); break;
                case ANSWER:         drawFullAnswerScreen(); break;
                case WAITING_TIME:   drawWaitingForTimeScreen(); break;
                case WAITING_ANSWER: drawWaitingForAnswerScreen(); break;
                default: break;
            }
            lastDrawnState = currentState;
        }

        // Animations (dots, etc.)
        updateWaitingForTimeAnimation();

        // Clock update (throttled internally, once per minute)
        refreshClockIfNeeded();

        // Battery percentage (throttled internally)
        updateBatteryPercent();

        // Redraw battery overlay only when percentage changes
        static int s_lastDisplayedBatteryPct = -1;
        if (g_batteryPercent != s_lastDisplayedBatteryPct) {
            s_lastDisplayedBatteryPct = g_batteryPercent;
            drawBatteryOverlay(false);
        }
    }

    // -------------------------------------------------------------------------
    // Frame pacing - Adaptive for smooth UI
    // -------------------------------------------------------------------------
    // POWER CRITICAL: Use vTaskDelay() not delay() to allow FreeRTOS tickless idle!
    // delay() busy-waits and prevents light sleep. vTaskDelay() yields to scheduler.
    // -------------------------------------------------------------------------
    static uint32_t s_frameStartMs = 0;
    uint32_t frameTime = millis() - s_frameStartMs;

    if (g_recordingInProgress) {
        // Recording: Minimal delay for audio streaming (CPU lock prevents sleep)
        vTaskDelay(1);
    }
    else if (powerIsLightSleep()) {
        // POWER: Light sleep mode - longer delay for power savings
        // Touch GPIO check happens at start of handleTouch() - no I2C needed
        // Using 200ms reduces polling frequency significantly
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    else if (powerIsDimmed()) {
        // Dimmed: ~5fps to save power
        const uint32_t targetFrameMs = 200;
        if (frameTime < targetFrameMs) {
            vTaskDelay(pdMS_TO_TICKS(targetFrameMs - frameTime));
        }
    }
    else {
        // Active: ~20fps for decent UI (saves power vs 30fps)
        const uint32_t targetFrameMs = 50;
        if (frameTime < targetFrameMs) {
            vTaskDelay(pdMS_TO_TICKS(targetFrameMs - frameTime));
        }
    }

    s_frameStartMs = millis();
}
