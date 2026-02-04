// =============================================================================
// UI COMMON - OPTIMIZED DISPLAY DRIVER FOR T-WATCH S3
// =============================================================================
// Key optimizations:
// 1. Higher SPI frequency (80MHz) for faster screen updates
// 2. DMA enabled for non-blocking transfers
// 3. Optimized brightness levels for battery life
// 4. Always-visible battery percentage on all screens
// =============================================================================

#include "ui_common.h"

#include "../hardware_config.h"
#include "../power/battery.h"
#include "../system/state.h"
#include "../system/time_sync.h"

const int SCREEN_W = SCREEN_WIDTH;
const int SCREEN_H = SCREEN_HEIGHT;

// =============================================================================
// BRIGHTNESS LEVELS - Reduced for battery life
// =============================================================================
const uint8_t BRIGHTNESS_ACTIVE   = 70;   // Reduced from 100 for power savings
const uint8_t BRIGHTNESS_DIM      = 12;   // Minimum usable visibility
const uint8_t BRIGHTNESS_CHARGING = 50;   // Higher when plugged in
const uint8_t TEXT_SIZE_PRIMARY   = 3;
const uint8_t TEXT_SIZE_SECONDARY = 2;

static int32_t g_lastClockMinute = -1;

// =============================================================================
// DISPLAY DRIVER CONFIGURATION
// =============================================================================

LGFX::LGFX() {
    {   // SPI bus - ST7789 Display
        auto b = _bus.config();
        b.spi_host   = SPI2_HOST;
        b.spi_mode   = 0;
        b.freq_write = 80000000;   // 80MHz for faster updates
        b.freq_read  = 16000000;
        b.spi_3wire  = false;
        b.use_lock   = true;
        b.dma_channel = SPI_DMA_CH_AUTO;
        b.pin_sclk = TFT_SCLK_PIN;
        b.pin_mosi = TFT_MOSI_PIN;
        b.pin_miso = TFT_MISO_PIN;
        b.pin_dc   = TFT_DC_PIN;
        _bus.config(b);
        _panel.setBus(&_bus);
    }
    {   // Panel - ST7789
        auto p = _panel.config();
        p.pin_cs   = TFT_CS_PIN;
        p.pin_rst  = TFT_RST_PIN;
        p.panel_width  = SCREEN_WIDTH;
        p.panel_height = SCREEN_HEIGHT;
        p.invert   = true;
        _panel.config(p);
    }
    {   // Backlight - PWM controlled
        auto l = _light.config();
        l.pin_bl = TFT_BL_PIN;
        l.freq   = 44100;  // Higher PWM frequency for flicker-free
        _light.config(l);
        _panel.setLight(&_light);
    }
    {   // Touch - FT6336
        auto t = _touch.config();
        t.i2c_port = 1;
        t.pin_sda = TOUCH_SDA_PIN;
        t.pin_scl = TOUCH_SCL_PIN;
        t.pin_int = TOUCH_INT_PIN;
        t.freq = 400000;
        t.i2c_addr = TOUCH_I2C_ADDR;
        _touch.config(t);
        _panel.setTouch(&_touch);
    }
    setPanel(&_panel);
}

LGFX gfx;

void uiInitDisplay() {
    gfx.init();
    gfx.setBrightness(BRIGHTNESS_ACTIVE);
    gfx.setRotation(0);
    gfx.fillScreen(TFT_BLACK);
}

void uiInvalidateClock() {
    g_lastClockMinute = -1;
}

void drawClock(const String &timeStr) {
    const int y = 12;
    gfx.setTextSize(TEXT_SIZE_PRIMARY);
    gfx.setTextColor(TFT_WHITE, TFT_BLACK);
    gfx.setTextDatum(textdatum_t::top_center);
    gfx.drawString(timeStr, SCREEN_W / 2, y);

    // Always redraw battery after clock to ensure it's visible
    drawBatteryOverlay(true);
}

void refreshClockIfNeeded() {
    if (g_sleeping || currentState != IDLE) {
        g_lastClockMinute = -1;
        return;
    }
    time_t now = getCurrentEpoch();
    int32_t minuteStamp = now / 60;
    if (minuteStamp != g_lastClockMinute) {
        g_lastClockMinute = minuteStamp;
        drawClock(formatClock(now));
    }
}

void playBootAnimation() {
    // Faster boot animation
    gfx.fillScreen(TFT_BLACK);
    int cx = SCREEN_W / 2;
    int cy = SCREEN_H / 2;

    // Faster expanding circles
    for (int r = 10; r <= 90; r += 16) {
        uint16_t color = gfx.color565(20 + r, 20 + r, 40 + r / 2);
        gfx.drawCircle(cx, cy, r, color);
        delay(12);
    }

    // Logo
    gfx.fillCircle(cx, cy, 34, TFT_WHITE);
    gfx.setTextDatum(textdatum_t::middle_center);
    gfx.setTextColor(TFT_BLACK, TFT_WHITE);
    gfx.setTextSize(2);
    gfx.drawString("Hollow", cx, cy);
    delay(300);

    gfx.fillScreen(TFT_BLACK);
}
