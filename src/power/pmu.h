#pragma once

#include <XPowersLib.h>

extern XPowersAXP2101 g_pmu;
extern bool g_pmuPresent;

bool initPMU();

// =============================================================================
// POWER MANAGEMENT FUNCTIONS
// Call these to enable/disable peripheral power rails for maximum savings
// =============================================================================

// Display power control (ALDO2 = backlight, ALDO3 = display + touch)
void pmuEnableDisplay();
void pmuDisableDisplay();

// Haptics power control (BLDO2)
void pmuEnableHaptics();
void pmuDisableHaptics();

// Prepare PMU for deep sleep (disable all non-essential rails)
void pmuPrepareDeepSleep();

// Restore PMU after deep sleep wake
void pmuRestoreFromSleep();
