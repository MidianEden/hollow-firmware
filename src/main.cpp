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

// Hardware config (includes HOLLOW_DEBUG logging macros)
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
    // 1. POWER MANAGER FIRST
    // -------------------------------------------------------------------------
    // This disables WiFi, sets CPU frequency, and configures power management
    // MUST happen before any other initialization
    powerManagerInit();

    // -------------------------------------------------------------------------
    // 2. Serial for debugging (POWER: disabled in production via HOLLOW_DEBUG=0)
    // -------------------------------------------------------------------------
#if HOLLOW_DEBUG
    Serial.begin(115200);
    delay(50);  // Reduced from 100ms for faster boot

    Serial.println("\n\n========================================");
    Serial.println("  HOLLOW WATCH FIRMWARE v" FIRMWARE_VERSION);
    Serial.println("  Build: " BUILD_DATE);
    Serial.println("========================================\n");
#endif

    // Log reset reason
    esp_reset_reason_t resetReason = esp_reset_reason();
    Serial.printf("Reset reason: %d ", resetReason);
    switch (resetReason) {
        case ESP_RST_POWERON:   Serial.println("(Power on)"); break;
        case ESP_RST_SW:        Serial.println("(Software reset)"); break;
        case ESP_RST_PANIC:     Serial.println("(Panic/crash!)"); break;
        case ESP_RST_INT_WDT:   Serial.println("(Interrupt watchdog!)"); break;
        case ESP_RST_TASK_WDT:  Serial.println("(Task watchdog!)"); break;
        case ESP_RST_WDT:       Serial.println("(Other watchdog)"); break;
        case ESP_RST_DEEPSLEEP: Serial.println("(Deep sleep wake)"); break;
        case ESP_RST_BROWNOUT:  Serial.println("(BROWNOUT - battery critical!)"); break;
        default:                Serial.println("(Unknown)"); break;
    }

    // Check wake reason
    esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
    bool wokeFromDeepSleep = (wakeReason == ESP_SLEEP_WAKEUP_EXT0 ||
                              wakeReason == ESP_SLEEP_WAKEUP_EXT1);
    if (wokeFromDeepSleep) {
        Serial.println("Woke from deep sleep via touch/button");
    }

    // -------------------------------------------------------------------------
    // 3. Watchdog (30 second timeout)
    // -------------------------------------------------------------------------
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);

    // -------------------------------------------------------------------------
    // 4. PMU (controls power rails)
    // -------------------------------------------------------------------------
    Serial.println("\n[INIT] PMU...");
    g_pmuPresent = initPMU();

    // -------------------------------------------------------------------------
    // 5. Display
    // -------------------------------------------------------------------------
    Serial.println("[INIT] Display...");
    uiInitDisplay();

    // Skip boot animation if waking from deep sleep (faster wake)
    if (!wokeFromDeepSleep) {
        playBootAnimation();
    }

    // -------------------------------------------------------------------------
    // 6. State and timekeeping
    // -------------------------------------------------------------------------
    Serial.println("[INIT] State...");
    initState();
    timeSyncInit();
    initBatterySimulator();

    g_lastWaitAnimMs = millis();
    g_waitingDots = 0;

    // -------------------------------------------------------------------------
    // 7. BLE (after PMU and display are ready)
    // -------------------------------------------------------------------------
    Serial.println("[INIT] BLE...");
    initBLE();

    // -------------------------------------------------------------------------
    // 8. Mic - Initialize I2S driver at boot for instant recording
    // -------------------------------------------------------------------------
    Serial.println("[INIT] Mic (persistent I2S driver)...");
    initMic();  // Install I2S driver once, never uninstall

    // -------------------------------------------------------------------------
    // 9. Final setup
    // -------------------------------------------------------------------------
    delay(50);  // Reduced from 100ms
    testBatteryDisplay();

    drawIdleScreen();
    lastDrawnState = IDLE;
    updateChargingState();

    powerPrintDiagnostics();

    Serial.println("\n[INIT] Setup complete - entering main loop");
    Serial.printf("[INIT] Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.println("========================================\n");
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
