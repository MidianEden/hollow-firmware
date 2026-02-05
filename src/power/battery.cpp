#include "battery.h"

#include <Arduino.h>

#include "../hardware_config.h"
#include "pmu.h"
#include "power_manager.h"
#include "../ui/ui_common.h"
#include "../system/state.h"

// =============================================================================
// BATTERY MEASUREMENT - LOAD-COMPENSATED
// =============================================================================
//
// Problem: LiPo voltage sags significantly under load
// - At 100mA load, voltage can drop 50-150mV from open-circuit
// - At 200mA load, voltage can drop 100-300mV
// - This causes "high reading then sudden shutdown" because:
//   1. High load -> voltage sags to 3.0V -> brownout
//   2. Load removed -> voltage recovers to 3.4V
//   3. Last logged reading was the recovered voltage
//
// Solution: Estimate current draw and compensate voltage reading
// =============================================================================

// Battery parameters for T-Watch S3 (400-470mAh LiPo)
constexpr int BATTERY_CAPACITY_MAH = 450;
constexpr int BATTERY_INTERNAL_RESISTANCE_MOHM = 150;  // Typical for small LiPo

// Voltage thresholds (open-circuit voltage, not under load)
constexpr int VOLTAGE_FULL_MV = 4150;      // 100% (4.2V nominal, but rarely reaches 4.2)
constexpr int VOLTAGE_NOMINAL_MV = 3700;   // ~50%
constexpr int VOLTAGE_LOW_MV = 3400;       // ~15% - start warning user
constexpr int VOLTAGE_CRITICAL_MV = 3200;  // ~5% - prepare for shutdown
constexpr int VOLTAGE_EMPTY_MV = 3000;     // 0% - cutoff

// POWER: Update intervals - reduced polling to save power
// Each ADC read costs ~50-100µA for a few ms, plus I2C bus activity
constexpr uint32_t BATTERY_UPDATE_MS = 15000;         // Update every 15 seconds (was 5s)
constexpr uint32_t BATTERY_UPDATE_SLEEP_MS = 60000;   // Update every 60s when sleeping
constexpr uint32_t CHARGE_POLL_MS = 5000;             // Check charging state every 5s (was 2s)
constexpr uint32_t CHARGE_REDRAW_MS = 8000;           // Redraw charging animation (was 4s)

// State
uint32_t g_lastBatteryUpdateMs = 0;
uint32_t g_lastChargeCheckMs = 0;
uint32_t g_lastChargeRedrawMs = 0;
int g_batteryPercent = 100;
int g_batteryVoltageMv = 4000;  // Last raw voltage reading

static int s_drawnBatteryLevel = -1;
static bool s_drawnCharging = false;

// POWER: Smoothing filter for battery percentage (prevents jitter and jumps)
// Lower alpha = more smoothing = more stable readings
static int s_smoothedPercent = -1;
constexpr float SMOOTHING_ALPHA = 0.1f;  // Reduced from 0.3 for more stability

// Multi-sample averaging for voltage stability
static int s_voltageSamples[4] = {0, 0, 0, 0};
static int s_sampleIndex = 0;
static bool s_samplesInitialized = false;

// Track last drawn percentage for smart redraw (used by drawBatteryOverlay)
static int s_lastDrawnPercent = -1;

// =============================================================================
// Load Compensation
// =============================================================================

static int estimateLoadCurrentMa() {
    // Estimate current draw based on device state
    // These are approximate values - measure with a current meter for accuracy
    int current = 0;

    // Base ESP32-S3 current (varies with CPU activity)
    if (powerIsActive()) {
        current += 35;  // 80MHz, display on
    } else if (powerIsDimmed()) {
        current += 30;  // 80MHz, display dimmed
    } else if (powerIsLightSleep()) {
        current += 15;  // Between sleep periods
    } else {
        current += 25;  // Default
    }

    // BLE adds current
    if (g_bleConnected) {
        current += 10;  // Connected, periodic events
    } else {
        current += 3;   // Advertising
    }

    // Display backlight (major consumer)
    if (powerIsActive()) {
        current += 15;  // Full brightness
    } else if (powerIsDimmed()) {
        current += 3;   // Dimmed
    }

    // Recording adds significant current
    if (g_recordingInProgress) {
        current += 20;  // Mic + I2S + processing
    }

    return current;
}

static int compensateVoltageForLoad(int rawVoltageMv, int loadCurrentMa) {
    // V_opencircuit = V_measured + (I * R_internal)
    // This estimates what the voltage would be with no load
    int voltageDrop = (loadCurrentMa * BATTERY_INTERNAL_RESISTANCE_MOHM) / 1000;
    return rawVoltageMv + voltageDrop;
}

// =============================================================================
// Voltage to Percentage Conversion
// =============================================================================
// LiPo discharge curve is non-linear:
// - 4.2V to 3.9V: Rapid drop (100% -> 70%)
// - 3.9V to 3.7V: Plateau (70% -> 40%)
// - 3.7V to 3.5V: Gradual decline (40% -> 20%)
// - 3.5V to 3.0V: Steep drop (20% -> 0%)

static int voltageToPercent(int voltageMv) {
    if (voltageMv >= VOLTAGE_FULL_MV) {
        return 100;
    }

    if (voltageMv <= VOLTAGE_EMPTY_MV) {
        return 0;
    }

    // Piecewise linear approximation of LiPo discharge curve
    int percent;

    if (voltageMv >= 4000) {
        // 4.0V - 4.15V: 85% - 100%
        percent = 85 + ((voltageMv - 4000) * 15) / 150;
    }
    else if (voltageMv >= 3850) {
        // 3.85V - 4.0V: 70% - 85%
        percent = 70 + ((voltageMv - 3850) * 15) / 150;
    }
    else if (voltageMv >= 3750) {
        // 3.75V - 3.85V: 50% - 70% (plateau region)
        percent = 50 + ((voltageMv - 3750) * 20) / 100;
    }
    else if (voltageMv >= 3650) {
        // 3.65V - 3.75V: 35% - 50%
        percent = 35 + ((voltageMv - 3650) * 15) / 100;
    }
    else if (voltageMv >= 3500) {
        // 3.5V - 3.65V: 20% - 35%
        percent = 20 + ((voltageMv - 3500) * 15) / 150;
    }
    else if (voltageMv >= 3300) {
        // 3.3V - 3.5V: 5% - 20%
        percent = 5 + ((voltageMv - 3300) * 15) / 200;
    }
    else {
        // 3.0V - 3.3V: 0% - 5%
        percent = ((voltageMv - 3000) * 5) / 300;
    }

    // Clamp
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    return percent;
}

// =============================================================================
// Battery Reading
// =============================================================================

static int readCompensatedBatteryPercent() {
    if (!g_pmuPresent) {
        return 100;  // Assume full if no PMU
    }

    // Read raw voltage from AXP2101
    int rawVoltage = g_pmu.getBattVoltage();
    if (rawVoltage <= 0) {
        LOGLN("[BATT] WARNING: Failed to read voltage");
        return g_batteryPercent;  // Return last known value
    }

    // POWER FIX: Multi-sample averaging to prevent voltage spike jumps
    // This fixes the "30% -> 38% jump" issue caused by load changes
    s_voltageSamples[s_sampleIndex] = rawVoltage;
    s_sampleIndex = (s_sampleIndex + 1) % 4;

    if (!s_samplesInitialized) {
        // Initialize all samples with first reading
        for (int i = 0; i < 4; i++) {
            s_voltageSamples[i] = rawVoltage;
        }
        s_samplesInitialized = true;
    }

    // Use average of last 4 samples
    int avgVoltage = 0;
    for (int i = 0; i < 4; i++) {
        avgVoltage += s_voltageSamples[i];
    }
    avgVoltage /= 4;

    g_batteryVoltageMv = avgVoltage;

    // Estimate current load and compensate
    int loadCurrent = estimateLoadCurrentMa();
    int compensatedVoltage = compensateVoltageForLoad(avgVoltage, loadCurrent);

    // Convert to percentage
    int rawPercent = voltageToPercent(compensatedVoltage);

    // Apply smoothing to prevent jitter (now with lower alpha = 0.1)
    if (s_smoothedPercent < 0) {
        s_smoothedPercent = rawPercent;
    } else {
        s_smoothedPercent = (int)(SMOOTHING_ALPHA * rawPercent +
                                   (1.0f - SMOOTHING_ALPHA) * s_smoothedPercent);
    }

    // Anti-jump logic: Battery shouldn't increase much unless charging
    // But allow tiny increases (1-2%) for measurement noise / load compensation
    static int s_lastReportedPercent = -1;

    // First reading - just use it
    if (s_lastReportedPercent < 0) {
        s_lastReportedPercent = s_smoothedPercent;
    }
    else if (!g_isCharging) {
        int diff = s_smoothedPercent - s_lastReportedPercent;
        if (diff <= 1) {
            // Allow decreases and tiny increases (noise)
            s_lastReportedPercent = s_smoothedPercent;
        }
        // Ignore larger increases (load recovery artifacts)
    } else {
        // Charging - allow increases
        s_lastReportedPercent = s_smoothedPercent;
    }

    return s_lastReportedPercent;
}

// =============================================================================
// Public API
// =============================================================================

void initBatterySimulator() {
    uint32_t now = millis();
    g_lastBatteryUpdateMs = now;
    g_lastChargeCheckMs = now;
    g_lastChargeRedrawMs = now;
    s_drawnBatteryLevel = -1;
    s_drawnCharging = false;
    s_smoothedPercent = -1;
    s_lastDrawnPercent = -1;
    s_samplesInitialized = false;  // Reset multi-sample averaging
    s_sampleIndex = 0;

    // Read initial battery level
    g_batteryPercent = readCompensatedBatteryPercent();

    LOG("[BATT] Init: %d%% (%dmV raw)\n",
                  g_batteryPercent, g_batteryVoltageMv);

    if (g_pmuPresent) {
        LOG("[BATT] Charging: %s, VBUS: %s\n",
                      g_pmu.isCharging() ? "YES" : "NO",
                      g_pmu.isVbusIn() ? "YES" : "NO");
    }
}

void updateBatteryPercent() {
    uint32_t now = millis();

    // POWER: Use longer interval when sleeping to reduce ADC wakeups
    uint32_t updateInterval = powerIsLightSleep() ? BATTERY_UPDATE_SLEEP_MS : BATTERY_UPDATE_MS;
    if (now - g_lastBatteryUpdateMs < updateInterval) {
        return;
    }
    g_lastBatteryUpdateMs = now;

    int newPercent = readCompensatedBatteryPercent();

    // Log changes
    if (newPercent != g_batteryPercent) {
        LOG("[BATT] %d%% -> %d%% (%dmV, ~%dmA load)\n",
                      g_batteryPercent, newPercent,
                      g_batteryVoltageMv, estimateLoadCurrentMa());
    }

    g_batteryPercent = newPercent;
}

void updateChargingState() {
    uint32_t now = millis();

    if (g_lastChargeCheckMs == 0) {
        g_lastChargeCheckMs = now - CHARGE_POLL_MS;
    }

    // Handle charging animation redraw
    if (g_isCharging && (now - g_lastChargeRedrawMs) > CHARGE_REDRAW_MS) {
        g_lastChargeRedrawMs = now;
        s_drawnBatteryLevel = -1;
        drawBatteryOverlay(true);
    }

    if (now - g_lastChargeCheckMs < CHARGE_POLL_MS) {
        return;
    }
    g_lastChargeCheckMs = now;

    bool wasCharging = g_isCharging;
    g_isCharging = g_pmuPresent ? g_pmu.isVbusIn() : false;

    if (g_isCharging != wasCharging) {
        LOG("[BATT] Charging state: %s\n",
                      g_isCharging ? "STARTED" : "STOPPED");

        // Reset ALL battery tracking when charging state changes
        // This fixes "stuck" readings by allowing fresh calibration
        s_smoothedPercent = -1;
        s_samplesInitialized = false;
        s_sampleIndex = 0;

        // Force immediate battery update
        g_lastBatteryUpdateMs = 0;
        updateBatteryPercent();

        if (g_isCharging) {
            powerMarkActivity();  // Wake display
            gfx.setBrightness(BRIGHTNESS_CHARGING);
        } else {
            gfx.setBrightness(BRIGHTNESS_ACTIVE);
        }

        s_drawnBatteryLevel = -1;
        drawBatteryOverlay(true);
        g_lastChargeRedrawMs = now;
    }
}

// =============================================================================
// Battery UI
// =============================================================================

static int batteryLevelBucket(int pct) {
    if (pct <= 15) return 0;   // Low (red)
    if (pct <= 50) return 1;   // Medium (orange)
    return 2;                  // Full (green)
}

static uint16_t levelColor(int level) {
    if (g_isCharging) return TFT_GREEN;
    if (level == 0) return TFT_RED;
    if (level == 1) return TFT_ORANGE;
    return TFT_GREEN;
}

void drawBatteryOverlay(bool force) {
    if (force) {
        g_lastBatteryUpdateMs = 0;
        updateBatteryPercent();
    }

    int pct = g_batteryPercent;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    int level = batteryLevelBucket(pct);

    // Skip redraw if nothing changed
    if (!force && level == s_drawnBatteryLevel &&
        s_drawnCharging == g_isCharging &&
        pct == s_lastDrawnPercent) {
        return;
    }

    uint16_t bg = g_isCharging ? gfx.color565(8, 12, 16) : TFT_BLACK;
    const int w = 24;
    const int h = 14;
    const int x = SCREEN_W - w - 10;
    const int y = 4;

    // Clear area for percentage text (left of battery icon)
    gfx.fillRect(x - 40, y - 1, 38, h + 2, TFT_BLACK);

    // Draw percentage text
    char pctStr[8];
    snprintf(pctStr, sizeof(pctStr), "%d%%", pct);
    uint16_t textColor = levelColor(level);
    gfx.setTextSize(1);
    gfx.setTextColor(textColor, TFT_BLACK);
    gfx.setTextDatum(textdatum_t::middle_right);
    gfx.drawString(pctStr, x - 4, y + h / 2);

    // Battery outline
    gfx.fillRect(x - 1, y - 1, w + 4, h + 2, TFT_DARKGREY);
    gfx.fillRect(x, y, w, h, bg);
    gfx.fillRect(x + w, y + 4, 3, h - 8, TFT_DARKGREY);

    // Fill based on percentage
    uint16_t color = levelColor(level);
    float fillRatio = pct / 100.0f;
    int fillW = (int)((w - 4) * fillRatio);
    if (fillW < 2 && pct > 0) fillW = 2;
    if (fillW > w - 4) fillW = w - 4;
    gfx.fillRect(x + 2, y + 2, fillW, h - 4, color);

    // Border
    gfx.drawRect(x, y, w, h, TFT_WHITE);

    // Charging indicator (lightning bolt)
    if (g_isCharging) {
        int cx = x + w / 2;
        int cy = y + h / 2;
        gfx.fillTriangle(cx - 3, cy - 5, cx + 1, cy - 5, cx - 1, cy + 5, TFT_YELLOW);
        gfx.fillTriangle(cx + 3, cy + 5, cx - 1, cy + 5, cx + 1, cy - 5, TFT_YELLOW);
    }

    s_drawnBatteryLevel = level;
    s_drawnCharging = g_isCharging;
    s_lastDrawnPercent = pct;
}

void testBatteryDisplay() {
    LOGLN("\n========== BATTERY TEST ==========");

    if (g_pmuPresent) {
        int rawVoltage = g_pmu.getBattVoltage();
        int loadCurrent = estimateLoadCurrentMa();
        int compensatedVoltage = compensateVoltageForLoad(rawVoltage, loadCurrent);

        LOG("Raw Voltage: %d mV\n", rawVoltage);
        LOG("Est. Load Current: %d mA\n", loadCurrent);
        LOG("Compensated Voltage: %d mV\n", compensatedVoltage);
        LOG("Battery Percent: %d%%\n", g_batteryPercent);
        LOG("Fuel Gauge Reading: %d%%\n", g_pmu.getBatteryPercent());
        LOG("Charging: %s\n", g_pmu.isCharging() ? "YES" : "NO");
        LOG("VBUS Present: %s\n", g_pmu.isVbusIn() ? "YES" : "NO");
        LOG("Battery Connected: %s\n", g_pmu.isBatteryConnect() ? "YES" : "NO");

        // Voltage reference table
        LOGLN("\nVoltage Reference (open-circuit):");
        LOGLN("  4.15V+ = 100%");
        LOGLN("  4.00V  = ~85%");
        LOGLN("  3.85V  = ~70%");
        LOGLN("  3.75V  = ~50%");
        LOGLN("  3.65V  = ~35%");
        LOGLN("  3.50V  = ~20%");
        LOGLN("  3.30V  = ~5%");
        LOGLN("  3.00V  = 0%");
    } else {
        LOGLN("PMU not present");
    }

    LOG("\nDisplayed: %d%%\n", g_batteryPercent);
    s_drawnBatteryLevel = -1;
    drawBatteryOverlay(true);

    LOGLN("===================================\n");
}

// Get raw voltage for external use
int getBatteryVoltageMv() {
    return g_batteryVoltageMv;
}
