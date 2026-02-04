#include "ui_record.h"

#include "../ui/ui_common.h"
#include "../power/battery.h"

void drawRecordingScreen() {
    gfx.fillScreen(TFT_BLACK);
    gfx.setTextColor(TFT_CYAN, TFT_BLACK);
    gfx.setTextDatum(textdatum_t::middle_center);
    gfx.setTextSize(TEXT_SIZE_PRIMARY);
    gfx.drawString("Listening...", SCREEN_W / 2, SCREEN_H / 2);
    drawBatteryOverlay(true);
}
