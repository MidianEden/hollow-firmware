#pragma once

#include <Arduino.h>
#include <string>

extern time_t g_buildEpoch;
extern bool g_waitingForTime;
extern bool g_haveHostTime;

void timeSyncInit();
void setCurrentEpoch(time_t epoch);
time_t getCurrentEpoch();
String formatClock(time_t now);
void handleTimeMessage(const std::string &value);
void requestTimeFromHub(bool showWaitingScreen);
void updateTimeRequest();
void timeSyncHandleConnected();
void timeSyncHandleDisconnected();
