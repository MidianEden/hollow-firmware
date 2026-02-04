#pragma once

#include <LovyanGFX.hpp>
#include <Arduino.h>

// Display driver and shared UI utilities
class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ST7789 _panel;
    lgfx::Bus_SPI      _bus;
    lgfx::Light_PWM    _light;
    lgfx::Touch_FT5x06 _touch;
public:
    LGFX();
};

extern LGFX gfx;

extern const int SCREEN_W;
extern const int SCREEN_H;

extern const uint8_t BRIGHTNESS_ACTIVE;
extern const uint8_t BRIGHTNESS_DIM;
extern const uint8_t BRIGHTNESS_CHARGING;
extern const uint8_t TEXT_SIZE_PRIMARY;
extern const uint8_t TEXT_SIZE_SECONDARY;

void uiInitDisplay();
void playBootAnimation();
void drawClock(const String &timeStr);
void refreshClockIfNeeded();
void uiInvalidateClock();
