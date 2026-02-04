#include "ui_idle.h"

#include <cstring>

#include "../ui/ui_common.h"
#include "../power/battery.h"
#include "../system/time_sync.h"
#include "../system/state.h"
#include "../hollowlogo.h"

void drawIdleScreen() {
    gfx.fillScreen(TFT_BLACK);

    int w = LOGO_W;
    int h = LOGO_H;

    int x0 = (SCREEN_W - w) / 2;
    int y0 = (SCREEN_H - h) / 2;

    static uint16_t buf[LOGO_W];
    gfx.startWrite();
    for (int y = 0; y < h; y++) {
        const uint16_t *src = &hollowlogo[y * LOGO_W];
        memcpy(buf, src, LOGO_W * sizeof(uint16_t));
        gfx.setAddrWindow(x0, y0 + y, w, 1);
        gfx.pushPixels(buf, w);
    }
    gfx.endWrite();

    // Draw clock last so it sits above the logo
    uiInvalidateClock(); // force refresh
    drawBatteryOverlay(true);
}
