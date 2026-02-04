#include "ui_answer.h"

#include "../ui/ui_common.h"
#include "../power/battery.h"
#include "../system/state.h"

int g_scrollY = 0;
int g_lastTouchY = -1;
int g_maxScroll = 0;
int g_touchStartX = 0;
int g_touchStartY = 0;
bool g_touchMoved = false;

void resetAnswerScrollState() {
    g_scrollY = 0;
    g_lastTouchY = -1;
    g_touchMoved = false;
}

void drawFullAnswerScreen() {
    gfx.fillScreen(TFT_BLACK);

    String text = g_lastText;
    if (text == "") text = "(No reply)";

    const int textSize = TEXT_SIZE_PRIMARY;

    gfx.setTextSize(textSize);
    gfx.setTextColor(TFT_WHITE, TFT_BLACK);
    gfx.setTextDatum(textdatum_t::top_left);

    const int margin   = 10;
    const int maxWidth = SCREEN_W - margin * 2;

    const int lineHeight = 14 * textSize + 4;

    auto render = [&](bool draw) {
        int y = margin - (draw ? g_scrollY : 0);
        int height = 0;
        String line = "";

        auto flushLine = [&](bool drawLine) {
            if (line.length() == 0) {
                height += lineHeight;
                if (drawLine) y += lineHeight;
                return;
            }
            // Only draw lines that are visible on screen (optimization)
            if (drawLine) {
                if (y + lineHeight > 0 && y < SCREEN_H) {
                    gfx.drawString(line, margin, y);
                }
                y += lineHeight;
            }
            height += lineHeight;
            line = "";
        };

        for (int i = 0; i < text.length(); i++) {
            char c = text[i];

            if (c == '\n') {
                flushLine(draw);
                continue;
            }

            if (c == ' ') {
                String candidate = line + ' ';
                if (gfx.textWidth(candidate) > maxWidth && line.length() > 0) {
                    flushLine(draw);
                }
                line += ' ';
                continue;
            }

            line += c;
            if (gfx.textWidth(line) > maxWidth) {
                char lastChar = line.charAt(line.length() - 1);
                line.remove(line.length() - 1);
                if (line.length() > 0) {
                    flushLine(draw);
                }
                line = String(lastChar);
            }
        }

        if (line.length()) {
            flushLine(draw);
        }

        return height;
    };

    int totalHeight = render(false);
    // Max scroll = total text height - visible area + margin for battery overlay
    // margin*2 accounts for top margin and bottom padding
    g_maxScroll = max(0, totalHeight - SCREEN_H + margin * 2);

    // DEBUG: Print scroll state to help diagnose scrolling issues
    Serial.printf("[SCROLL] textLen=%d totalHeight=%d maxScroll=%d scrollY=%d\n",
                  text.length(), totalHeight, g_maxScroll, g_scrollY);

    render(true);
    drawBatteryOverlay(true);
}
