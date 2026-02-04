#pragma once

#include <BLECharacteristic.h>

void initOtaCharacteristic(BLECharacteristic *c);
BLECharacteristicCallbacks *createOtaCallbacks();
void otaLoop();
void otaHandleDisconnected();
bool otaInProgress();
