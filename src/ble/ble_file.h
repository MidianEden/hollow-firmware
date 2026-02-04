#pragma once

#include <BLECharacteristic.h>

BLECharacteristicCallbacks *createFileCallbacks();
void sendRecordedFileOverBle(BLECharacteristic *pChar);
