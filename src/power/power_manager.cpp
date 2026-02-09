// =============================================================================
// POWER MANAGER - OPTIMIZED FOR T-WATCH S3
// =============================================================================
// Key optimizations:
// 1. Fast wake path (<100ms from touch to screen)
// 2. Proper CPU lock during recording/BLE transfers
// 3. Smooth state transitions with no display glitches
// 4. ESP-IDF automatic power management enabled
// =============================================================================

#include "power_manager.h"
#include "pmu.h"
#include "battery.h"
#include "../hardware_config.h"
#include "../ui/ui_common.h"
#include "../ui/ui_idle.h"
#include "../system/state.h"
#include "../audio/audio_i2s.h"
#include "../ble/ble_core.h"  // POWER: For BLE sleep mode control

#include <esp_pm.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <driver/rtc_io.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include <soc/rtc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// =============================================================================
// State Variables
// =============================================================================

PowerState g_powerState = POWER_ACTIVE;
volatile bool g_wokeFromSleep = false;
static uint32_t s_lastActivityMs = 0;
static uint32_t s_lightSleepEnteredMs = 0;
static bool s_pmConfigured = false;
static bool s_bleConnected = false;

// Wake timing metrics (for diagnostics)
static uint32_t s_lastWakeTimeUs = 0;
static uint32_t s_wakeCount = 0;
static uint32_t s_maxWakeTimeUs = 0;

// Light sleep lock - prevent sleep during critical operations
static esp_pm_lock_handle_t s_cpuLock = nullptr;
static bool s_cpuLockHeld = false;

// =============================================================================
// Internal: CPU Lock Management
// =============================================================================

static void acquireCpuLock() {
    if (s_cpuLock && !s_cpuLockHeld) {
        esp_pm_lock_acquire(s_cpuLock);
        s_cpuLockHeld = true;
    }
}

static void releaseCpuLock() {
    if (s_cpuLock && s_cpuLockHeld) {
        esp_pm_lock_release(s_cpuLock);
        s_cpuLockHeld = false;
    }
}

// =============================================================================
// Internal: Display Power Control
// =============================================================================

static void displaySetActive() {
    // POWER: Restore full CPU frequency for responsive UI
    setCpuFrequencyMhz(CPU_FREQ_MAX);
    pmuEnableDisplay();
    gfx.wakeup();
    gfx.setBrightness(g_isCharging ? BRIGHTNESS_CHARGING : BRIGHTNESS_ACTIVE);
}

static void displaySetDimmed() {
    // POWER: Reduce CPU frequency when dimmed (80MHz is enough for basic UI)
    setCpuFrequencyMhz(80);
    pmuEnableDisplay();
    gfx.wakeup();
    gfx.setBrightness(BRIGHTNESS_DIM);
}

static void displaySetOff() {
    // POWER: Optimized display shutdown sequence
    // 1. Turn off backlight FIRST (instant visual off)
    gfx.setBrightness(0);
    // 2. Put display controller into sleep mode (reduces display controller power)
    //    Skip fillScreen - it wastes power and display is already off visually
    gfx.sleep();
    // 3. Cut power to backlight circuit via PMU (ALDO2 off)
    pmuDisableDisplay();
}

// =============================================================================
// Internal: Configure ESP-IDF Power Management
// =============================================================================

static bool configurePowerManagement() {
    // ESP32-S3 power management configuration
    // This enables automatic light sleep when all tasks are idle

    esp_pm_config_esp32s3_t pm_config = {};
    pm_config.max_freq_mhz = CPU_FREQ_MAX;
    pm_config.min_freq_mhz = CPU_FREQ_MIN;
    pm_config.light_sleep_enable = true;

    esp_err_t err = esp_pm_configure(&pm_config);
    if (err != ESP_OK) {
        LOG("[POWER] PM configure failed: %s\n", esp_err_to_name(err));
        return false;
    }

    // Create a CPU frequency lock for when we need guaranteed performance
    // (e.g., during audio recording or BLE transfers)
    err = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpu_work", &s_cpuLock);
    if (err != ESP_OK) {
        LOG("[POWER] Lock create failed: %s\n", esp_err_to_name(err));
        return false;
    }

    LOG("[POWER] PM configured: %d-%dMHz, light_sleep=ON\n",
                  CPU_FREQ_MIN, CPU_FREQ_MAX);
    return true;
}

// =============================================================================
// Internal: Configure Wake Sources
// =============================================================================

static void configureWakeSources() {
    // Touch interrupt - GPIO 16, active LOW
    // Level-triggered wake for light sleep - fires when touch INT goes low
    gpio_wakeup_enable((gpio_num_t)TOUCH_INT_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    // PMU interrupt - GPIO 21, active LOW (power button, charger events)
    gpio_wakeup_enable((gpio_num_t)PMU_INT_PIN, GPIO_INTR_LOW_LEVEL);

    LOGLN("[POWER] Wake sources: GPIO16(touch), GPIO21(PMU)");
}

// =============================================================================
// Internal: Brownout Detection
// =============================================================================

static void checkBatteryHealth() {
    if (!g_pmuPresent) return;

    int voltage = g_pmu.getBattVoltage();

    if (voltage < SHUTDOWN_THRESHOLD_MV && !g_isCharging) {
        LOG("[POWER] CRITICAL: Battery %dmV - forcing shutdown!\n", voltage);
        LOG_FLUSH();

        // Give user visual feedback if possible
        gfx.fillScreen(TFT_RED);
        gfx.setTextColor(TFT_WHITE);
        gfx.setTextDatum(textdatum_t::middle_center);
        gfx.drawString("LOW BATTERY", SCREEN_W/2, SCREEN_H/2);
        delay(2000);

        // Shutdown via PMU (cleaner than brownout reset)
        g_pmu.shutdown();
    }
    else if (voltage < BROWNOUT_THRESHOLD_MV && !g_isCharging) {
        // Log warning but don't shutdown yet
        static uint32_t lastWarnMs = 0;
        if (millis() - lastWarnMs > 30000) {
            LOG("[POWER] WARNING: Battery low %dmV\n", voltage);
            lastWarnMs = millis();
        }
    }
}

// =============================================================================
// Public: Initialization
// =============================================================================

bool powerManagerInit() {
    LOGLN("\n[POWER] Initializing power manager...");

    // 1. Ensure WiFi is completely disabled
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_OK) {
        esp_wifi_deinit();
        LOGLN("[POWER] WiFi disabled");
    }

    // NOTE: USB CDC is disabled via platformio.ini (ARDUINO_USB_CDC_ON_BOOT=0)
    // This saves 5-10mA without requiring code changes

    // 2. Explicitly disable unused peripherals
    // LoRa module - hold CS high and reset low to keep it in lowest power state
    pinMode(RADIO_CS_PIN, OUTPUT);
    digitalWrite(RADIO_CS_PIN, HIGH);      // Deselect SPI
    pinMode(RADIO_RST_PIN, OUTPUT);
    digitalWrite(RADIO_RST_PIN, LOW);      // Hold in reset
    // Other LoRa pins as inputs with no pull (saves power)
    pinMode(RADIO_MOSI_PIN, INPUT);
    pinMode(RADIO_MISO_PIN, INPUT);
    pinMode(RADIO_SCLK_PIN, INPUT);
    pinMode(RADIO_DIO1_PIN, INPUT);
    pinMode(RADIO_BUSY_PIN, INPUT);
    LOGLN("[POWER] LoRa module disabled (held in reset)");

    // Accelerometer - not used, set interrupt pin as input
    pinMode(ACCEL_INT_PIN, INPUT);

    // IR transmitter - not used
    pinMode(IR_TX_PIN, INPUT);

    // 3. Set initial CPU frequency
    setCpuFrequencyMhz(CPU_FREQ_MAX);
    LOG("[POWER] CPU set to %dMHz\n", getCpuFrequencyMhz());

    // 4. Configure ESP-IDF power management for automatic light sleep
    s_pmConfigured = configurePowerManagement();
    if (!s_pmConfigured) {
        LOGLN("[POWER] WARNING: PM not configured - no auto light sleep!");
    }

    // 5. Configure wake sources
    configureWakeSources();

    // 6. Initialize timing
    s_lastActivityMs = millis();
    g_powerState = POWER_ACTIVE;

    LOGLN("[POWER] Power manager initialized\n");
    return s_pmConfigured;
}

// =============================================================================
// Public: Activity Tracking
// =============================================================================

void powerMarkActivity() {
    s_lastActivityMs = millis();

    // If we were in light sleep mode, set wake flag for main loop
    if (g_powerState == POWER_LIGHT_SLEEP) {
        g_wokeFromSleep = true;  // Main loop MUST call handleWakeFromLightSleep()
        // Don't transition here - handleWakeFromLightSleep() does full wake sequence
        return;
    }

    // From dimmed, just transition to active (no special handling needed)
    if (g_powerState == POWER_DIMMED) {
        g_powerState = POWER_ACTIVE;
        g_sleeping = false;
        g_dimmed = false;
        displaySetActive();
        s_lightSleepEnteredMs = 0;
        LOGLN("[POWER] -> ACTIVE (user activity)");
    }
}

void powerHandleBLEConnect() {
    s_bleConnected = true;
    powerMarkActivity();
    LOGLN("[POWER] BLE connected");
}

void powerHandleBLEDisconnect() {
    s_bleConnected = false;
    // Don't change power state - let normal timeout handle it
    LOGLN("[POWER] BLE disconnected");
}

// =============================================================================
// Public: State Machine Update
// =============================================================================

bool powerUpdate() {
    const uint32_t now = millis();
    const uint32_t idleMs = now - s_lastActivityMs;

    // Check battery health periodically
    static uint32_t lastBatteryCheck = 0;
    if (now - lastBatteryCheck > 10000) {
        lastBatteryCheck = now;
        checkBatteryHealth();
    }

    // Don't transition during recording
    if (g_recordingInProgress) {
        if (g_powerState != POWER_ACTIVE) {
            g_powerState = POWER_ACTIVE;
            g_sleeping = false;
            g_dimmed = false;
            displaySetActive();
            acquireCpuLock();  // Prevent auto light sleep during recording
        }
        return true;
    } else {
        releaseCpuLock();  // Allow auto light sleep
    }

    // State machine
    switch (g_powerState) {
        case POWER_ACTIVE:
            if (idleMs >= TIMEOUT_DIM_MS) {
                g_powerState = POWER_DIMMED;
                g_dimmed = true;  // Sync legacy global
                g_sleeping = false;
                displaySetDimmed();
                LOGLN("[POWER] -> DIMMED");
            }
            break;

        case POWER_DIMMED:
            if (idleMs >= TIMEOUT_LIGHT_SLEEP_MS && !g_isCharging) {
                g_powerState = POWER_LIGHT_SLEEP;
                g_sleeping = true;  // Sync legacy global
                g_dimmed = false;
                displaySetOff();
                s_lightSleepEnteredMs = now;
                // POWER: Enter BLE sleep mode (slower polling, not disabled)
                bleEnterSleepMode();
                LOGLN("[POWER] -> LIGHT_SLEEP");
            }
            break;

        case POWER_LIGHT_SLEEP:
            if (g_isCharging) {
                g_powerState = POWER_DIMMED;
                g_dimmed = true;
                g_sleeping = false;
                displaySetDimmed();
                bleExitSleepMode();
                break;
            }
            // Check if we should go to deep sleep (maximum power saving)
            // Deep sleep triggers after 5 minutes of inactivity
            if (TIMEOUT_DEEP_SLEEP_MS > 0 && !g_isCharging) {
                uint32_t lightSleepDuration = now - s_lightSleepEnteredMs;
                if (lightSleepDuration >= TIMEOUT_DEEP_SLEEP_MS) {
                    LOGLN("[POWER] -> DEEP_SLEEP (5min timeout)");
                    powerForceDeepSleep();  // Does not return
                }
            }
            // In light sleep, ESP-IDF handles actual sleep automatically
            // We just need short delays in main loop
            break;

        case POWER_DEEP_SLEEP:
            // Should never be here - deep sleep triggers reset on wake
            break;
    }

    return true;
}

// =============================================================================
// Public: Force State Changes
// =============================================================================

void powerForceActive() {
    g_powerState = POWER_ACTIVE;
    g_sleeping = false;
    g_dimmed = false;
    s_lastActivityMs = millis();
    displaySetActive();
}

void powerForceLightSleep() {
    if (g_recordingInProgress) return;  // Safety check

    g_powerState = POWER_LIGHT_SLEEP;
    g_sleeping = true;
    g_dimmed = false;
    displaySetOff();
    s_lightSleepEnteredMs = millis();
}

void powerForceDeepSleep() {
    LOGLN("[POWER] Entering MAXIMUM power saving deep sleep...");
    LOG_FLUSH();

    // 1. Stop all audio
    if (isMicRunning()) stopMic();
    deinitMic();

    // 2. Turn off display completely
    displaySetOff();

    // 3. Disable BLE completely for maximum power saving
    // BLE will reinitialize on wake (device does full reset from deep sleep)
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    // 4. Prepare PMU - disable all non-essential rails for maximum power saving
    pmuPrepareDeepSleep();

    // 5. Clear pending touch interrupt before entering deep sleep
    // During light sleep, we only check GPIO state but never do an I2C read
    // to clear the touch controller's interrupt. The INT pin stays stuck LOW.
    // We must read touch data via I2C to clear it, then wait for pin to go HIGH.
    {
        // Wake display controller briefly to allow I2C touch read
        gfx.wakeup();
        delay(10);

        // Read touch data to clear pending interrupt on the touch controller
        lgfx::touch_point_t tp;
        gfx.getTouch(&tp);
        delay(10);
        gfx.getTouch(&tp);  // Read twice to be safe
        delay(10);

        // Put display back to sleep
        gfx.sleep();

        // Now wait for touch INT pin to actually go HIGH
        int waitMs = 0;
        while (digitalRead(TOUCH_INT_PIN) == LOW && waitMs < 2000) {
            delay(10);
            waitMs += 10;
        }
        if (waitMs >= 2000) {
            // Touch INT is stuck LOW - skip touch as wake source, use only PMU button
            LOG("[POWER] Touch INT stuck LOW - using button-only wake\n");
            esp_sleep_enable_ext1_wakeup((1ULL << PMU_INT_PIN), ESP_EXT1_WAKEUP_ALL_LOW);
        } else {
            LOG("[POWER] Touch INT clear (waited %dms) - configuring wake sources\n", waitMs);
            delay(50);  // Extra settle time

            // 6. Configure wake sources for deep sleep
            esp_sleep_enable_ext0_wakeup((gpio_num_t)TOUCH_INT_PIN, 0);  // Touch: wake on LOW
            esp_sleep_enable_ext1_wakeup((1ULL << PMU_INT_PIN), ESP_EXT1_WAKEUP_ALL_LOW);  // Button
        }
    }

    // 7. Isolate ALL unused GPIO to prevent leakage current (maximum power saving)
    // Audio pins (mic + speaker)
    rtc_gpio_isolate((gpio_num_t)MIC_DATA_PIN);      // GPIO 47
    rtc_gpio_isolate((gpio_num_t)MIC_CLK_PIN);       // GPIO 44
    rtc_gpio_isolate((gpio_num_t)I2S_BCK_PIN);       // GPIO 48
    rtc_gpio_isolate((gpio_num_t)I2S_WS_PIN);        // GPIO 15
    rtc_gpio_isolate((gpio_num_t)I2S_DOUT_PIN);      // GPIO 46

    // LoRa SPI pins (module is powered off but pins can leak)
    rtc_gpio_isolate((gpio_num_t)RADIO_MOSI_PIN);    // GPIO 1
    rtc_gpio_isolate((gpio_num_t)RADIO_SCLK_PIN);    // GPIO 3
    rtc_gpio_isolate((gpio_num_t)RADIO_MISO_PIN);    // GPIO 4
    rtc_gpio_isolate((gpio_num_t)RADIO_CS_PIN);      // GPIO 5
    rtc_gpio_isolate((gpio_num_t)RADIO_DIO3_PIN);    // GPIO 6
    rtc_gpio_isolate((gpio_num_t)RADIO_BUSY_PIN);    // GPIO 7
    rtc_gpio_isolate((gpio_num_t)RADIO_RST_PIN);     // GPIO 8
    rtc_gpio_isolate((gpio_num_t)RADIO_DIO1_PIN);    // GPIO 9

    // Accelerometer interrupt (not used)
    rtc_gpio_isolate((gpio_num_t)ACCEL_INT_PIN);     // GPIO 14

    // IR transmitter (not used)
    rtc_gpio_isolate((gpio_num_t)IR_TX_PIN);         // GPIO 2

    LOGLN("[POWER] Entering deep sleep NOW - maximum power saving mode");
    LOGLN("[POWER] Wake on touch/button -> will auto-start BLE scanning");
    LOG_FLUSH();

    delay(100);  // Ensure log output completes

    esp_deep_sleep_start();
    // Never returns - device resets on wake
    // On wake, setup() will detect deep sleep wake and auto-start BLE scanning
}

// =============================================================================
// Public: Wake Handler - MUST be called from main loop when g_wokeFromSleep is true
// =============================================================================
// CRITICAL: This function is optimized for SPEED. Target: <100ms wake time
// Every millisecond counts for user experience.
// =============================================================================

void handleWakeFromLightSleep() {
    uint32_t wakeStartUs = micros();

    // 1. Update power state FIRST (before any hardware access)
    //    This ensures other code sees the correct state immediately
    g_powerState = POWER_ACTIVE;
    g_sleeping = false;
    g_dimmed = false;
    s_lastActivityMs = millis();
    s_lightSleepEnteredMs = 0;

    // 2. Clear wake flag EARLY to prevent re-entry race conditions
    g_wokeFromSleep = false;

    // 3. POWER: Exit BLE sleep mode (restore faster polling)
    bleExitSleepMode();

    // 4. Acquire CPU lock to prevent auto-sleep during wake sequence
    acquireCpuLock();

    // 5. Enable display power rail FIRST (PMU is fast, ~1ms)
    pmuEnableDisplay();

    // 6. Wake display controller - CRITICAL PATH
    //    wakeup() exits sleep mode, much faster than full init()
    //    Typical: ~5-10ms vs 50-100ms for init()
    gfx.wakeup();

    // 7. Set brightness BEFORE drawing to avoid flash
    gfx.setBrightness(g_isCharging ? BRIGHTNESS_CHARGING : BRIGHTNESS_ACTIVE);

    // 8. FORCE UI STATE TO HOME - Always return to home on wake
    //    This is what users expect - instant, predictable behavior
    currentState = IDLE;
    lastDrawnState = IDLE;

    // 9. Draw HOME screen - Use optimized drawing path
    //    drawIdleScreen() clears screen and draws logo + battery
    drawIdleScreen();

    // 10. Invalidate clock cache so it redraws on next refresh
    uiInvalidateClock();

    // 11. Reset battery voltage tracking to prevent "catch up" jumps
    //     Battery voltage can fluctuate when load changes after wake
    batteryResetAfterWake();

    // 12. Mark wake tap consumed - do NOT forward to UI input
    //     This prevents the wake tap from triggering recording
    g_ignoreTap = true;

    // 13. Release CPU lock - normal power management resumes
    releaseCpuLock();

    // Track wake timing for diagnostics
    uint32_t wakeTimeUs = micros() - wakeStartUs;
    s_lastWakeTimeUs = wakeTimeUs;
    s_wakeCount++;
    if (wakeTimeUs > s_maxWakeTimeUs) {
        s_maxWakeTimeUs = wakeTimeUs;
    }

    LOG("[POWER] WAKE #%lu: %lu us (%.1f ms) -> HOME%s\n",
                  s_wakeCount, wakeTimeUs, wakeTimeUs / 1000.0f,
                  (wakeTimeUs > WAKE_MAX_MS * 1000) ? " [SLOW!]" : "");
}

// =============================================================================
// Public: Query Functions
// =============================================================================

bool powerIsActive() {
    return g_powerState == POWER_ACTIVE;
}

bool powerIsDimmed() {
    return g_powerState == POWER_DIMMED;
}

bool powerIsLightSleep() {
    return g_powerState == POWER_LIGHT_SLEEP;
}

bool powerCanDoWork() {
    return g_powerState != POWER_DEEP_SLEEP;
}

uint32_t powerGetIdleTimeMs() {
    return millis() - s_lastActivityMs;
}

// =============================================================================
// Public: Diagnostics
// =============================================================================

void powerPrintDiagnostics() {
    LOGLN("\n========== POWER DIAGNOSTICS ==========");

    // CPU info
    LOG("CPU Frequency: %d MHz (range: %d-%d)\n",
                  getCpuFrequencyMhz(), CPU_FREQ_MIN, CPU_FREQ_MAX);
    LOG("Power State: %d ", g_powerState);
    switch(g_powerState) {
        case POWER_ACTIVE: LOGLN("(ACTIVE)"); break;
        case POWER_DIMMED: LOGLN("(DIMMED)"); break;
        case POWER_LIGHT_SLEEP: LOGLN("(LIGHT_SLEEP)"); break;
        case POWER_DEEP_SLEEP: LOGLN("(DEEP_SLEEP)"); break;
    }

    LOG("Idle Time: %lu ms\n", powerGetIdleTimeMs());
    LOG("BLE Connected: %s\n", s_bleConnected ? "YES" : "NO");
    LOG("Recording: %s\n", g_recordingInProgress ? "YES" : "NO");
    LOG("Charging: %s\n", g_isCharging ? "YES" : "NO");
    LOG("PM Configured: %s\n", s_pmConfigured ? "YES" : "NO");
    LOG("CPU Lock Held: %s\n", s_cpuLockHeld ? "YES" : "NO");

    // Wake timing stats
    LOGLN("\nWake Performance:");
    LOG("  Wake Count: %lu\n", s_wakeCount);
    LOG("  Last Wake: %.1f ms\n", s_lastWakeTimeUs / 1000.0f);
    LOG("  Max Wake: %.1f ms\n", s_maxWakeTimeUs / 1000.0f);
    LOG("  Target: <%lu ms\n", WAKE_TARGET_MS);

    // PMU rails
    if (g_pmuPresent) {
        LOGLN("\nPMU Power Rails:");
        LOG("  ALDO1: %s\n", g_pmu.isEnableALDO1() ? "ON" : "off");
        LOG("  ALDO2: %s (backlight)\n", g_pmu.isEnableALDO2() ? "ON" : "off");
        LOG("  ALDO3: %s (display+touch)\n", g_pmu.isEnableALDO3() ? "ON" : "off");
        LOG("  ALDO4: %s\n", g_pmu.isEnableALDO4() ? "ON" : "off");
        LOG("  BLDO1: %s\n", g_pmu.isEnableBLDO1() ? "ON" : "off");
        LOG("  BLDO2: %s (haptics)\n", g_pmu.isEnableBLDO2() ? "ON" : "off");
        LOG("  DLDO1: %s (speaker)\n", g_pmu.isEnableDLDO1() ? "ON" : "off");
        LOG("  DLDO2: %s\n", g_pmu.isEnableDLDO2() ? "ON" : "off");
        LOG("  DC2: %s\n", g_pmu.isEnableDC2() ? "ON" : "off");
        LOG("  DC3: %s (GPS)\n", g_pmu.isEnableDC3() ? "ON" : "off");
        LOG("  DC4: %s\n", g_pmu.isEnableDC4() ? "ON" : "off");
        LOG("  DC5: %s\n", g_pmu.isEnableDC5() ? "ON" : "off");

        LOGLN("\nBattery:");
        LOG("  Voltage: %d mV\n", g_pmu.getBattVoltage());
        LOG("  Percent: %d%%\n", g_pmu.getBatteryPercent());
        LOG("  Charging: %s\n", g_pmu.isCharging() ? "YES" : "NO");
    }

    LOGLN("========================================\n");
}

float powerEstimateCurrentMa() {
    float current = 0.0f;

    // Base ESP32-S3 current at current frequency
    int freq = getCpuFrequencyMhz();
    if (freq >= 240) current += 50.0f;
    else if (freq >= 160) current += 35.0f;
    else if (freq >= 80) current += 25.0f;
    else current += 10.0f;

    // BLE adds significant current
    if (s_bleConnected) current += 15.0f;
    else current += 5.0f;  // Advertising

    // Display
    if (g_powerState == POWER_ACTIVE) current += 20.0f;
    else if (g_powerState == POWER_DIMMED) current += 5.0f;

    // Recording
    if (g_recordingInProgress) current += 10.0f;

    return current;
}
