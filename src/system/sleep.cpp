// =============================================================================
// SLEEP.CPP - REPLACED BY POWER_MANAGER
// =============================================================================
// REMOVED:
// - delay() based fake sleep
// - g_sleeping polling variable
// - enterSleepIfInactive() polling loop
// - 0xFFFFFFFF deep sleep disable hack
//
// REPLACED WITH:
// - ESP-IDF esp_pm_configure() for automatic light sleep
// - GPIO wake interrupts
// - Proper power state machine in power_manager.cpp
// =============================================================================

#include "sleep.h"
#include "../power/power_manager.h"

// Legacy variable for compatibility - now managed by power_manager
uint32_t g_dimmedAtMs = 0;

void wakeFromSleep() {
    powerForceActive();
}

void markActivity() {
    powerMarkActivity();
}

void enterSleepIfInactive() {
    // REMOVED: This was a polling loop checking timeouts
    // NOW: powerUpdate() handles state machine in main loop
    // Actual light sleep happens automatically via ESP-IDF PM
}
