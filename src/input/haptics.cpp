#include "haptics.h"

#include <Wire.h>
#include <Adafruit_DRV2605.h>

static Adafruit_DRV2605 g_haptics;
bool g_hapticsReady = false;

bool initHaptics() {
    if (!g_haptics.begin(&Wire)) {
        return false;
    }
    g_haptics.selectLibrary(1);
    g_haptics.setMode(DRV2605_MODE_INTTRIG);
    g_hapticsReady = true;
    return true;
}

void pulseHaptic(uint8_t effect) {
    if (!g_hapticsReady) return;
    g_haptics.setWaveform(0, effect);
    g_haptics.setWaveform(1, 0);
    g_haptics.go();
}
