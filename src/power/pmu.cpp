#include "pmu.h"

#include <Arduino.h>
#include <Wire.h>
#include "../hardware_config.h"

XPowersAXP2101 g_pmu;
bool g_pmuPresent = false;

static uint16_t detectBatteryCapacityMah() {
#if defined(ARDUINO_T_WATCH_S3_PLUS) || defined(T_WATCH_S3_PLUS) || defined(TWATCH_S3_PLUS)
    return 915;  // T-Watch S3 Plus battery
#else
    return 470;  // Standard T-Watch S3 battery
#endif
}

static void calibrationPMU(uint16_t batteryCapacityMah) {
    // Enable fuel gauge for battery percentage
    g_pmu.enableGauge();
    g_pmu.fuelGaugeControl(true, true);
    LOG("Fuel gauge configured for %d mAh battery\n", batteryCapacityMah);
}

bool initPMU() {
    Wire.begin(PMU_SDA_PIN, PMU_SCL_PIN);
    pinMode(PMU_INT_PIN, INPUT_PULLUP);

    if (!g_pmu.init(Wire)) {
        LOGLN("ERROR: PMU init failed!");
        return false;
    }
    LOGLN("PMU initialized successfully");

    // =========================================================================
    // USB/VBUS settings
    // =========================================================================
    g_pmu.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);
    g_pmu.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_900MA);
    g_pmu.setSysPowerDownVoltage(2600);  // Emergency shutdown at 2.6V

    // =========================================================================
    // POWER: Set power rail voltages
    // =========================================================================
    g_pmu.setALDO2Voltage(3300);  // Display backlight
    g_pmu.setALDO3Voltage(3300);  // Display + Touch
    g_pmu.setBLDO2Voltage(3300);  // Haptics (DRV2605)
    g_pmu.setButtonBatteryChargeVoltage(3300); // RTC backup

    // =========================================================================
    // POWER CRITICAL: Disable ALL unused power rails!
    // Each active rail draws quiescent current even with no load
    // =========================================================================
    g_pmu.disableDC2();       // POWER: Not used - saves ~0.5mA
    g_pmu.disableDC3();       // POWER: GPS power - not used, saves ~1-2mA
    g_pmu.disableDC4();       // POWER: Not used
    g_pmu.disableDC5();       // POWER: Not used

    g_pmu.disableALDO1();     // POWER: Not used
    g_pmu.disableALDO4();     // POWER: LoRa/Radio - NOT USED! Saves ~2-5mA!

    g_pmu.disableBLDO1();     // POWER: GPS (some revisions) - not used
    // BLDO2 = Haptics - enable only when needed

    g_pmu.disableDLDO1();     // POWER: Speaker amp - enable only when needed
    g_pmu.disableDLDO2();     // POWER: Not used

    g_pmu.disableCPUSLDO();   // POWER: Not used

    // =========================================================================
    // Enable only essential power rails
    // =========================================================================
    g_pmu.enableALDO2();      // Display backlight
    g_pmu.enableALDO3();      // Display + Touch (needed for touch wake)
    g_pmu.disableBLDO2();     // POWER: Haptics OFF by default - enable only when needed
    g_pmu.enableButtonBatteryCharge();  // RTC backup

    // =========================================================================
    // Power button settings
    // =========================================================================
    g_pmu.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    g_pmu.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);

    // =========================================================================
    // POWER: Battery monitoring - disable unnecessary ADC channels!
    // Each enabled ADC channel draws ~50-100µA
    // =========================================================================
    g_pmu.enableBattDetection();
    g_pmu.enableBattVoltageMeasure();     // KEEP: Needed for battery %

    // POWER: Disable these ADC channels - not critical, saves ~100-200µA
    g_pmu.disableTSPinMeasure();          // POWER: Temp sensor not needed
    g_pmu.disableVbusVoltageMeasure();    // POWER: USB voltage not critical
    g_pmu.disableSystemVoltageMeasure();  // POWER: System voltage not needed

    // =========================================================================
    // POWER: Charging LED off - saves a few mA when charging
    // =========================================================================
    g_pmu.setChargingLedMode(XPOWERS_CHG_LED_OFF);

    // =========================================================================
    // Configure interrupts - only enable what we need for wake
    // =========================================================================
    g_pmu.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    g_pmu.enableIRQ(
        XPOWERS_AXP2101_PKEY_SHORT_IRQ |    // Power button wake
        XPOWERS_AXP2101_VBUS_INSERT_IRQ |   // USB plugged in
        XPOWERS_AXP2101_VBUS_REMOVE_IRQ     // USB unplugged
    );
    g_pmu.clearIrqStatus();

    // =========================================================================
    // Charging parameters (conservative for battery longevity)
    // =========================================================================
    g_pmu.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
    g_pmu.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);
    g_pmu.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
    g_pmu.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

    // Initialize fuel gauge
    calibrationPMU(detectBatteryCapacityMah());

    // Print diagnostics
    LOGLN("\n=== Battery Diagnostics ===");
    LOG("Battery Voltage: %d mV\n", g_pmu.getBattVoltage());
    LOG("Battery Percent: %d%%\n", g_pmu.getBatteryPercent());
    LOG("Charging: %s\n", g_pmu.isCharging() ? "YES" : "NO");
    LOG("VBUS In: %s\n", g_pmu.isVbusIn() ? "YES" : "NO");
    LOGLN("===========================\n");

    g_pmuPresent = true;
    LOGLN("PMU configured for ULTRA-LOW POWER");
    return true;
}

// =============================================================================
// POWER CONTROL FUNCTIONS
// =============================================================================

void pmuEnableDisplay() {
    if (!g_pmuPresent) return;
    g_pmu.enableALDO2();  // Backlight power
    g_pmu.enableALDO3();  // Display + Touch power
}

void pmuDisableDisplay() {
    if (!g_pmuPresent) return;
    // POWER: Turning off display backlight saves ~15-30mA!
    g_pmu.disableALDO2();  // Backlight power OFF
    // Note: Keep ALDO3 on for touch wake capability
}

void pmuEnableHaptics() {
    if (!g_pmuPresent) return;
    g_pmu.enableBLDO2();
}

void pmuDisableHaptics() {
    if (!g_pmuPresent) return;
    // POWER: Disable haptics when not in use - saves ~0.5-1mA
    g_pmu.disableBLDO2();
}

void pmuPrepareDeepSleep() {
    if (!g_pmuPresent) return;

    // =========================================================================
    // MAXIMUM POWER SAVING MODE - Deep Sleep
    // Target: <100µA total system current in deep sleep
    // This is the most aggressive power saving mode
    // =========================================================================

    LOG("[PMU] Preparing for maximum power saving deep sleep...\n");

    // Disable display power rails (display is already off via sleep command)
    g_pmu.disableALDO2();    // POWER: Display backlight OFF
    // Keep ALDO3 enabled for touch wake interrupt capability (GPIO needs power)

    // Disable all other non-essential rails
    g_pmu.disableBLDO2();    // POWER: Haptics OFF
    g_pmu.disableDLDO1();    // POWER: Speaker amplifier OFF

    // Disable battery monitoring ADCs to save power (not needed during sleep)
    g_pmu.disableBattVoltageMeasure();    // POWER: Save ~50µA
    g_pmu.disableTSPinMeasure();          // POWER: Already off but ensure
    g_pmu.disableVbusVoltageMeasure();    // POWER: Already off but ensure
    g_pmu.disableSystemVoltageMeasure();  // POWER: Already off but ensure

    // Ensure wake interrupts are enabled
    g_pmu.clearIrqStatus();
    g_pmu.enableIRQ(XPOWERS_AXP2101_PKEY_SHORT_IRQ);  // Button wake

    LOG("[PMU] Deep sleep mode enabled - current should be <100µA\n");
}

void pmuRestoreFromSleep() {
    if (!g_pmuPresent) return;

    LOG("[PMU] Restoring from deep sleep...\n");

    // Re-enable display power rails
    g_pmu.enableALDO2();    // Backlight
    g_pmu.enableALDO3();    // Display + Touch

    // Haptics - keep disabled, enable only when needed
    // g_pmu.enableBLDO2();

    // Re-enable battery voltage measurement for battery monitoring
    g_pmu.enableBattVoltageMeasure();

    // Clear any pending IRQs
    g_pmu.clearIrqStatus();

    LOG("[PMU] PMU restored - normal operation mode\n");
}
