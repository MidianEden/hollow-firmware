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
#include <Wire.h>
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
    esp_pm_config_esp32s3_t pm_config = {};
    pm_config.max_freq_mhz = CPU_FREQ_MAX;
    pm_config.min_freq_mhz = CPU_FREQ_MIN;
    pm_config.light_sleep_enable = true;

    esp_err_t err = esp_pm_configure(&pm_config);
    if (err != ESP_OK) return false;

    err = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpu_work", &s_cpuLock);
    if (err != ESP_OK) return false;

    return true;
}

// =============================================================================
// Internal: Configure Wake Sources
// =============================================================================

static void configureWakeSources() {
    // Light sleep wake via GPIO (digital domain). Keep both lines pulled high.
    pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
    pinMode(PMU_INT_PIN, INPUT_PULLUP);

    gpio_wakeup_enable((gpio_num_t)TOUCH_INT_PIN, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)PMU_INT_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
}

// =============================================================================
// Internal: Brownout Detection
// =============================================================================

static void checkBatteryHealth() {
    if (!g_pmuPresent) return;

    int voltage = g_pmu.getBattVoltage();

    if (voltage < SHUTDOWN_THRESHOLD_MV && !g_isCharging) {
        // Give user visual feedback if possible
        gfx.fillScreen(TFT_RED);
        gfx.setTextColor(TFT_WHITE);
        gfx.setTextDatum(textdatum_t::middle_center);
        gfx.drawString("LOW BATTERY", SCREEN_W/2, SCREEN_H/2);
        delay(2000);

        // Shutdown via PMU (cleaner than brownout reset)
        g_pmu.shutdown();
    }
}

// =============================================================================
// Public: Initialization
// =============================================================================

bool powerManagerInit() {
    // Ensure WiFi is completely disabled
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_OK) esp_wifi_deinit();

    // Disable unused peripherals
    pinMode(RADIO_CS_PIN, OUTPUT);
    digitalWrite(RADIO_CS_PIN, HIGH);
    pinMode(RADIO_RST_PIN, OUTPUT);
    digitalWrite(RADIO_RST_PIN, LOW);
    pinMode(RADIO_MOSI_PIN, INPUT);
    pinMode(RADIO_MISO_PIN, INPUT);
    pinMode(RADIO_SCLK_PIN, INPUT);
    pinMode(RADIO_DIO1_PIN, INPUT);
    pinMode(RADIO_BUSY_PIN, INPUT);
    pinMode(ACCEL_INT_PIN, INPUT);
    pinMode(IR_TX_PIN, INPUT);

    setCpuFrequencyMhz(CPU_FREQ_MAX);
    s_pmConfigured = configurePowerManagement();
    configureWakeSources();

    s_lastActivityMs = millis();
    g_powerState = POWER_ACTIVE;

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

    // From dimmed, just transition to active
    if (g_powerState == POWER_DIMMED) {
        g_powerState = POWER_ACTIVE;
        g_sleeping = false;
        g_dimmed = false;
        displaySetActive();
        s_lightSleepEnteredMs = 0;
    }
}

void powerHandleBLEConnect() {
    s_bleConnected = true;
    powerMarkActivity();
}

void powerHandleBLEDisconnect() {
    s_bleConnected = false;
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
                g_dimmed = true;
                g_sleeping = false;
                displaySetDimmed();
            }
            break;

        case POWER_DIMMED:
            if (idleMs >= TIMEOUT_LIGHT_SLEEP_MS) {
                g_powerState = POWER_LIGHT_SLEEP;
                g_sleeping = true;
                g_dimmed = false;
                displaySetOff();
                s_lightSleepEnteredMs = now;
                bleEnterSleepMode();
            }
            break;

        case POWER_LIGHT_SLEEP:
            if (TIMEOUT_DEEP_SLEEP_MS > 0) {
                uint32_t lightSleepDuration = now - s_lightSleepEnteredMs;
                if (lightSleepDuration >= TIMEOUT_DEEP_SLEEP_MS) {
                    powerForceDeepSleep();  // Does not return
                }
            }
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

// =============================================================================
// Internal: Clear FT6336 touch controller interrupt via I2C
// =============================================================================
// The FT6336 keeps INT (GPIO 16) asserted LOW until the host reads touch data.
// If not cleared before deep sleep, EXT0 wakeup triggers immediately.
// Returns true if INT was successfully cleared (HIGH), false if still stuck LOW.
// =============================================================================
static bool clearTouchInterrupt() {
    Wire1.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN, 100000);

    // Set FT6336 to interrupt trigger mode (INT stays LOW until data read)
    Wire1.beginTransmission(TOUCH_I2C_ADDR);
    Wire1.write(0xA4);  // G_MODE register
    Wire1.write(0x00);  // 0x00 = Interrupt Trigger mode
    Wire1.endTransmission();

    bool cleared = false;
    for (int attempt = 0; attempt < 5; attempt++) {
        // Read touch data registers (0x00-0x06) to deassert INT
        Wire1.beginTransmission(TOUCH_I2C_ADDR);
        Wire1.write(0x00);
        Wire1.endTransmission(false);

        Wire1.requestFrom(TOUCH_I2C_ADDR, 7);
        while (Wire1.available()) Wire1.read();

        delay(15);  // Allow INT line to settle

        if (digitalRead(TOUCH_INT_PIN) == HIGH) {
            cleared = true;
            break;
        }
    }

    Wire1.end();
    return cleared;
}

// =============================================================================
// Internal: Put FT6336 into Monitor mode for deep sleep
// =============================================================================
// Monitor mode (0xA5=0x01) periodically scans for touches at a low rate.
// When a touch is detected, INT is asserted LOW — perfect for EXT0 wake.
// Hibernate mode (0x03) does NOT scan and requires a hardware reset to exit,
// so it cannot be used for touch wake.
// =============================================================================
static void touchEnterMonitor() {
    Wire1.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN, 100000);

    // First: clear any pending interrupt by reading touch data
    Wire1.beginTransmission(TOUCH_I2C_ADDR);
    Wire1.write(0x00);
    Wire1.endTransmission(false);
    Wire1.requestFrom(TOUCH_I2C_ADDR, 7);
    while (Wire1.available()) Wire1.read();
    delay(10);

    // Set interrupt trigger mode: INT stays LOW until touch data is read
    Wire1.beginTransmission(TOUCH_I2C_ADDR);
    Wire1.write(0xA4);  // G_MODE register
    Wire1.write(0x00);  // 0x00 = Interrupt Trigger mode
    Wire1.endTransmission();

    // Set monitor scan period to maximum (~2.5s between scans) to save power
    Wire1.beginTransmission(TOUCH_I2C_ADDR);
    Wire1.write(0x87);  // PERIOD_MONITOR register
    Wire1.write(0xFF);  // 255 * ~10ms = ~2.5s between scans
    Wire1.endTransmission();

    // Enter Monitor mode (periodic low-power scanning)
    Wire1.beginTransmission(TOUCH_I2C_ADDR);
    Wire1.write(0xA5);  // Power mode register
    Wire1.write(0x01);  // 0x01 = Monitor mode
    Wire1.endTransmission();

    Wire1.end();

    // Wait for FT6336 to enter Monitor mode
    delay(50);

}

static void configureRtcWakeInput(gpio_num_t pin) {
    rtc_gpio_init(pin);
    rtc_gpio_set_direction(pin, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en(pin);
    rtc_gpio_pulldown_dis(pin);
}

static bool configureDeepSleepWakeSources() {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    // EXT0 needs RTC IO and RTC_PERIPH power domain alive.
    configureRtcWakeInput((gpio_num_t)TOUCH_INT_PIN);
    configureRtcWakeInput((gpio_num_t)PMU_INT_PIN);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);

    esp_err_t err = esp_sleep_enable_ext0_wakeup((gpio_num_t)TOUCH_INT_PIN, 0);
    if (err != ESP_OK) {
        return false;
    }

    // Keep PMU interrupt/button wake as a secondary wake source.
    err = esp_sleep_enable_ext1_wakeup((1ULL << PMU_INT_PIN), ESP_EXT1_WAKEUP_ALL_LOW);
    if (err != ESP_OK) {
        return false;
    }

    return true;
}

void powerForceDeepSleep() {
    Serial.println("[PWR] entering deep sleep");
    Serial.flush();

    // Shutdown peripherals
    if (isMicRunning()) stopMic();
    deinitMic();
    gfx.setBrightness(0);
    pmuDisableDisplay();
    bleFullShutdown();
    pmuPrepareDeepSleep();
    touchEnterMonitor();

    if (!configureDeepSleepWakeSources()) {
        delay(100);
        ESP.restart();
    }

    delay(100);

    esp_deep_sleep_start();
}

// =============================================================================
// Public: Wake Handler - MUST be called from main loop when g_wokeFromSleep is true
// =============================================================================
// CRITICAL: This function is optimized for SPEED. Target: <100ms wake time
// Every millisecond counts for user experience.
// =============================================================================

void handleWakeFromLightSleep() {
    g_powerState = POWER_ACTIVE;
    g_sleeping = false;
    g_dimmed = false;
    s_lastActivityMs = millis();
    s_lightSleepEnteredMs = 0;
    g_wokeFromSleep = false;

    bleExitSleepMode();
    acquireCpuLock();
    pmuEnableDisplay();
    gfx.wakeup();
    gfx.setBrightness(g_isCharging ? BRIGHTNESS_CHARGING : BRIGHTNESS_ACTIVE);

    currentState = IDLE;
    lastDrawnState = IDLE;
    drawIdleScreen();
    uiInvalidateClock();
    batteryResetAfterWake();
    g_ignoreTap = true;

    releaseCpuLock();
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
    // Diagnostics removed - no serial output
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

// =============================================================================
// Public: Wake Validation - call early in setup()
// =============================================================================
// After deep sleep wake, validates the wake source to prevent spurious wake loops.
// If the wake was spurious (e.g., uncleared touch INT), goes back to deep sleep
// immediately WITHOUT returning - avoids a full init cycle that wastes ~315 seconds.
// =============================================================================

void powerValidateWake() {
    esp_reset_reason_t resetReason = esp_reset_reason();
    if (resetReason != ESP_RST_DEEPSLEEP) return;

    esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
    Serial.printf("[PWR] deep sleep wake, cause=%d\n", (int)wakeReason);

    // Wake pins become RTC IOs in deep sleep; restore to digital GPIO for runtime.
    rtc_gpio_deinit((gpio_num_t)TOUCH_INT_PIN);
    rtc_gpio_deinit((gpio_num_t)PMU_INT_PIN);
    pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
    pinMode(PMU_INT_PIN, INPUT_PULLUP);

    // Unexpected wake sources - go back to sleep
    if (wakeReason != ESP_SLEEP_WAKEUP_EXT0 &&
        wakeReason != ESP_SLEEP_WAKEUP_EXT1) {
        Serial.println("[PWR] unexpected wake source, back to sleep");
        Serial.flush();
        if (!configureDeepSleepWakeSources()) return;
        esp_deep_sleep_start();
    }

    // Touch wake - validate finger is actually present
    if (wakeReason == ESP_SLEEP_WAKEUP_EXT0) {
        // Step 1: Clear pending INT via I2C
        bool cleared = clearTouchInterrupt();

        // Step 2: Wait for FT6336 scan cycle (~250ms in Monitor mode)
        delay(300);

        // Step 3: Check if finger is still there
        pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
        int pinState = digitalRead(TOUCH_INT_PIN);

        if (pinState == HIGH) {
            Serial.println("[PWR] spurious touch wake, back to sleep");
            Serial.flush();
            if (!configureDeepSleepWakeSources()) return;
            esp_deep_sleep_start();
        }

        Serial.println("[PWR] touch wake validated, finger present");
    }
}
