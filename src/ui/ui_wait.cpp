#include "ui_wait.h"

#include "../ui/ui_common.h"
#include "../power/battery.h"
#include "../system/state.h"

uint32_t g_lastWaitAnimMs = 0;
int g_waitingDots = 0;

void resetWaitingAnimation() {
    g_waitingDots = 0;
    g_lastWaitAnimMs = millis();
}

void drawWaitingForTimeScreen() {
    gfx.fillScreen(TFT_BLACK);
    gfx.setTextDatum(textdatum_t::middle_center);
    gfx.setTextSize(TEXT_SIZE_PRIMARY);
    gfx.setTextColor(TFT_WHITE, TFT_BLACK);
    String dots = "";
    for (int i = 0; i < g_waitingDots; i++) dots += ".";
    gfx.drawString("Waiting" + dots, SCREEN_W / 2, SCREEN_H / 2 - 8);
    gfx.setTextSize(TEXT_SIZE_SECONDARY);
    gfx.drawString("for time sync", SCREEN_W / 2, SCREEN_H / 2 + 16);
    drawBatteryOverlay(true);
}

void drawWaitingForAnswerScreen() {
    gfx.fillScreen(TFT_BLACK);
    gfx.setTextDatum(textdatum_t::middle_center);
    gfx.setTextSize(TEXT_SIZE_PRIMARY);
    gfx.setTextColor(TFT_WHITE, TFT_BLACK);
    String dots = "";
    for (int i = 0; i < g_waitingDots; i++) dots += ".";
    gfx.drawString("Waiting" + dots, SCREEN_W / 2, SCREEN_H / 2 - 8);
    gfx.setTextSize(TEXT_SIZE_SECONDARY);
    gfx.drawString("for reply", SCREEN_W / 2, SCREEN_H / 2 + 16);
    drawBatteryOverlay(true);
}

void updateWaitingForTimeAnimation() {
    if (currentState != WAITING_TIME && currentState != WAITING_ANSWER) return;
    uint32_t now = millis();
    if (now - g_lastWaitAnimMs < 500) return;
    g_lastWaitAnimMs = now;
    g_waitingDots = (g_waitingDots + 1) % 4;
    if (currentState == WAITING_TIME) {
        drawWaitingForTimeScreen();
    } else {
        drawWaitingForAnswerScreen();
    }
}
