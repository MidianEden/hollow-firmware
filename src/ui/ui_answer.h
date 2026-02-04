#pragma once

#include <Arduino.h>

extern int g_scrollY;
extern int g_lastTouchY;
extern int g_maxScroll;
extern int g_touchStartX;
extern int g_touchStartY;
extern bool g_touchMoved;

void resetAnswerScrollState();
void drawFullAnswerScreen();
