#pragma once

#include <Arduino.h>

// Global battery state
extern int g_batteryPercent;      // 0-100%
extern int g_batteryVoltageMv;    // Raw voltage in mV

// Initialization
void initBatterySimulator();

// Periodic updates (call from main loop)
void updateBatteryPercent();
void updateChargingState();

// Power management integration
void batteryResetAfterWake();

// UI
void drawBatteryOverlay(bool force = false);

// Diagnostics
void testBatteryDisplay();
int getBatteryVoltageMv();
