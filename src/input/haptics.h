#pragma once

#include <Arduino.h>

constexpr uint8_t HAPTIC_EFFECT = 47;

bool initHaptics();
void pulseHaptic(uint8_t effect = HAPTIC_EFFECT);
