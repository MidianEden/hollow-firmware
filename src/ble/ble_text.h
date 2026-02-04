#pragma once

#include <BLECharacteristic.h>

BLECharacteristicCallbacks *createTextCallbacks();
void processPendingText();
