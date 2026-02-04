#pragma once

#include <Arduino.h>

void wakeFromSleep();
void markActivity();
void enterSleepIfInactive();

extern uint32_t g_dimmedAtMs;
