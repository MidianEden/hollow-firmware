#pragma once

// =============================================================================
// BLE CORE - OPTIMIZED FOR STABILITY AND POWER EFFICIENCY
// =============================================================================

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Initialization
void initBLE();

// Connection state
bool bleIsConnected();
bool bleNotifyEnabled();
bool canSendControlMessages();

// Characteristic accessors
BLECharacteristic *bleGetAudioChar();
BLECharacteristic *bleGetTextChar();
BLECharacteristic *bleGetFileChar();
BLECharacteristic *bleGetOtaChar();

// Power management integration
// Call when starting/ending high-throughput transfers (audio streaming)
// This adjusts BLE connection parameters for optimal power/throughput tradeoff
void bleEnterActiveTransfer();
void bleExitActiveTransfer();

// POWER: Sleep mode BLE optimization
// Call when device enters/exits light sleep to reduce BLE power while maintaining connection
// This increases polling intervals to save power without disabling BLE
void bleEnterSleepMode();
void bleExitSleepMode();
bool bleIsInSleepMode();

// Periodic maintenance (call from loop, handles advertising restart)
void ensureAdvertisingAlive();

// Connection quality and error handling
bool bleSendNotifyWithRetry(BLECharacteristic* characteristic, const uint8_t* data, size_t len);
uint32_t bleGetConnectionErrors();
void bleResetConnectionErrors();
bool bleIsConnectionHealthy();
