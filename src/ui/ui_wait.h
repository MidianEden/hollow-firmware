#pragma once

#include <Arduino.h>

extern uint32_t g_lastWaitAnimMs;
extern int g_waitingDots;

void resetWaitingAnimation();
void drawWaitingForTimeScreen();
void drawWaitingForAnswerScreen();
void updateWaitingForTimeAnimation();
