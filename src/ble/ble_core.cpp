// =============================================================================
// BLE CORE - OPTIMIZED FOR STABILITY AND LOW POWER
// =============================================================================
// Key optimizations:
// 1. Improved connection parameter negotiation
// 2. Error tracking and recovery
// 3. Notify with retry for reliability
// 4. Faster advertising restart for better reconnection
// =============================================================================

#include "ble_core.h"

#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>

#include "../hardware_config.h"
#include "../system/state.h"
#include "../system/time_sync.h"
#include "../audio/audio_i2s.h"
#include "../power/power_manager.h"
#include "ble_text.h"
#include "ble_file.h"
#include "ble_ota.h"

// =============================================================================
// BLE CONFIGURATION - OPTIMIZED FOR STABILITY + POWER
// =============================================================================

static const char *DEVICE_NAME              = "Hollow 1W";
static const char *HOLLOW_SERVICE_UUID      = "4FAFC201-1FB5-459E-8FCC-C5C9C331914B";
static const char *AUDIO_CHAR_UUID          = "BEB5483E-36E1-4688-B7F5-EA07361B26A8";
static const char *TEXT_CHAR_UUID           = "0A3D547E-6967-4660-A744-8ACE08191266";
static const char *HOLLOW_FILE_SERVICE_UUID = "12345678-1234-5678-1234-56789ABCDEF0";
static const char *HOLLOW_FILE_CHAR_UUID    = "12345678-1234-5678-1234-56789ABCDEF1";
static const char *HOLLOW_OTA_SERVICE_UUID  = "B3F2D342-6A44-4B85-9F3A-4AEDA89753A2";
static const char *HOLLOW_OTA_CHAR_UUID     = "B3F2D342-6A44-4B85-9F3A-4AEDA89753A3";

// -----------------------------------------------------------------------------
// BLE CONNECTION PARAMETERS - TUNED FOR RELIABILITY + POWER
// -----------------------------------------------------------------------------
// Connection interval: Time between connection events (units: 1.25ms)
// Slave latency: Number of connection events peripheral can skip
// Supervision timeout: Time before connection considered lost (units: 10ms)
// -----------------------------------------------------------------------------

// Normal operation: Balance responsiveness and power
// Slightly tighter parameters for better reliability
constexpr uint16_t BLE_CONN_INT_MIN_NORMAL = 60;    // 75ms (was 100ms)
constexpr uint16_t BLE_CONN_INT_MAX_NORMAL = 120;   // 150ms (was 200ms)
constexpr uint16_t BLE_LATENCY_NORMAL = 1;          // Skip 1 event (was 2)
constexpr uint16_t BLE_TIMEOUT_NORMAL = 500;        // 5 seconds (was 4)

// Active transfer (recording): Fast updates needed for smooth audio
constexpr uint16_t BLE_CONN_INT_MIN_ACTIVE = 8;     // 10ms (was 15ms)
constexpr uint16_t BLE_CONN_INT_MAX_ACTIVE = 16;    // 20ms (was 30ms)
constexpr uint16_t BLE_LATENCY_ACTIVE = 0;          // No skipping during transfer
constexpr uint16_t BLE_TIMEOUT_ACTIVE = 300;        // 3 seconds

// POWER: Advertising intervals - balance between reconnection speed and power
// Units: 0.625ms per unit
// Normal advertising (device active): 500-1000ms
constexpr uint16_t BLE_ADV_INT_MIN_NORMAL = 0x0320;   // 500ms
constexpr uint16_t BLE_ADV_INT_MAX_NORMAL = 0x0640;   // 1000ms
// Sleep mode advertising: slower to save power
constexpr uint16_t BLE_ADV_INT_MIN_SLEEP = 0x0640;    // 1000ms
constexpr uint16_t BLE_ADV_INT_MAX_SLEEP = 0x0C80;    // 2000ms

// POWER: Sleep mode connection parameters - much slower to reduce BLE wakeups
constexpr uint16_t BLE_CONN_INT_MIN_SLEEP = 200;   // 250ms
constexpr uint16_t BLE_CONN_INT_MAX_SLEEP = 400;   // 500ms
constexpr uint16_t BLE_LATENCY_SLEEP = 4;          // Skip 4 events (aggressive power save)
constexpr uint16_t BLE_TIMEOUT_SLEEP = 600;        // 6 seconds

// Track current BLE power mode
static bool s_bleSleepMode = false;

// MTU: 247 bytes is optimal for ESP32 BLE
constexpr uint16_t BLE_MTU_SIZE = 247;

// -----------------------------------------------------------------------------
// State Variables
// -----------------------------------------------------------------------------

static BLECharacteristic *g_audioChar = nullptr;
static BLECharacteristic *g_textChar  = nullptr;
static BLECharacteristic *g_fileChar  = nullptr;
static BLECharacteristic *g_otaChar   = nullptr;
static BLEServer *g_server = nullptr;

static uint16_t g_connId = 0xFFFF;
static bool g_activeTransfer = false;
static esp_bd_addr_t g_peerBda = {0};  // Peer Bluetooth Device Address for connection params

// Connection parameter update throttling
static uint32_t s_lastParamUpdateMs = 0;
constexpr uint32_t BLE_PARAM_UPDATE_MIN_INTERVAL_MS = 1500;  // Reduced from 2s

// Error tracking for connection health monitoring
static uint32_t s_notifyErrors = 0;
static uint32_t s_connectionErrors = 0;
static uint32_t s_lastSuccessfulNotifyMs = 0;
constexpr uint32_t CONNECTION_UNHEALTHY_THRESHOLD_MS = 5000;  // 5s without success
constexpr uint32_t MAX_NOTIFY_RETRIES = 3;

// -----------------------------------------------------------------------------
// Connection Parameter Update
// -----------------------------------------------------------------------------

static void requestConnectionParams(bool activeTransfer) {
    if (g_connId == 0xFFFF || !g_bleConnected) return;

    // Skip if already in requested state
    if (g_activeTransfer == activeTransfer) {
        return;
    }

    // Throttle updates - minimum interval between changes
    uint32_t now = millis();
    if (s_lastParamUpdateMs > 0 && (now - s_lastParamUpdateMs) < BLE_PARAM_UPDATE_MIN_INTERVAL_MS) {
        return;  // Silently skip (don't spam logs)
    }

    esp_ble_conn_update_params_t params = {};
    memcpy(params.bda, g_peerBda, sizeof(esp_bd_addr_t));  // Required: peer device address

    if (activeTransfer) {
        params.min_int = BLE_CONN_INT_MIN_ACTIVE;
        params.max_int = BLE_CONN_INT_MAX_ACTIVE;
        params.latency = BLE_LATENCY_ACTIVE;
        params.timeout = BLE_TIMEOUT_ACTIVE;
        LOGLN("[BLE] -> ACTIVE params (fast transfer)");
    } else {
        params.min_int = BLE_CONN_INT_MIN_NORMAL;
        params.max_int = BLE_CONN_INT_MAX_NORMAL;
        params.latency = BLE_LATENCY_NORMAL;
        params.timeout = BLE_TIMEOUT_NORMAL;
        LOGLN("[BLE] -> NORMAL params (low power)");
    }

    esp_err_t err = esp_ble_gap_update_conn_params(&params);
    if (err != ESP_OK) {
        LOG("[BLE] Param update failed: %s\n", esp_err_to_name(err));
        s_connectionErrors++;
    }

    g_activeTransfer = activeTransfer;
    s_lastParamUpdateMs = now;
}

// -----------------------------------------------------------------------------
// Server Callbacks
// -----------------------------------------------------------------------------

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override {
        g_bleConnected = true;
        g_connId = param->connect.conn_id;
        memcpy(g_peerBda, param->connect.remote_bda, sizeof(esp_bd_addr_t));  // Save peer address
        s_lastSuccessfulNotifyMs = millis();

        // Reset error counters on new connection
        s_notifyErrors = 0;
        s_connectionErrors = 0;

        LOG("[BLE] Connected, conn_id=%d\n", g_connId);

        // Notify power manager
        powerHandleBLEConnect();
        powerMarkActivity();

        // Short delay for connection to stabilize
        delay(50);  // Reduced from 100ms

        // Request connection parameters for normal operation
        requestConnectionParams(false);

        timeSyncHandleConnected();
    }

    void onDisconnect(BLEServer* pServer) override {
        g_bleConnected = false;
        g_connId = 0xFFFF;
        g_activeTransfer = false;
        s_lastParamUpdateMs = 0;
        memset(g_peerBda, 0, sizeof(g_peerBda));  // Clear peer address

        LOG("[BLE] Disconnected (errors: notify=%lu, conn=%lu)\n",
                      s_notifyErrors, s_connectionErrors);

        // Stop any ongoing recording
        if (g_recordingInProgress) {
            g_recordingInProgress = false;
            finalizeRecordingTimer();
            setRecordingActive(false);
            stopMic();
        }
        currentState = IDLE;

        timeSyncHandleDisconnected();
        otaHandleDisconnected();

        powerHandleBLEDisconnect();
        // POWER: Don't wake device on disconnect - let it stay in current power state
        // This prevents the watch from turning on when BLE disconnects while sleeping
        // powerMarkActivity();

        // Restart advertising immediately
        BLEDevice::startAdvertising();
        LOGLN("[BLE] Advertising restarted");
    }
};

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

void initBLE() {
    LOGLN("[BLE] Initializing...");

    BLEDevice::init(DEVICE_NAME);

    // Set MTU
    BLEDevice::setMTU(BLE_MTU_SIZE);

    // =========================================================================
    // TX POWER - Optimized for watch scenario
    // =========================================================================
    // Using slightly higher power for better reliability
    // ESP_PWR_LVL_N0 = 0 dBm (good balance of range and power)
    // ESP_PWR_LVL_P3 = +3 dBm (better range, slightly more power)

    // Advertising power: 0 dBm for reliable discovery
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N0);

    // Connection power: +3 dBm for reliability during transfers
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL2, ESP_PWR_LVL_P3);

    // Default/scan power
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N0);

    LOGLN("[BLE] TX power: ADV=0dBm, CONN=+3dBm");

    // Create server
    g_server = BLEDevice::createServer();
    g_server->setCallbacks(new ServerCallbacks());

    // Create services
    BLEService *service = g_server->createService(HOLLOW_SERVICE_UUID);
    BLEService *fileService = g_server->createService(HOLLOW_FILE_SERVICE_UUID);
    BLEService *otaService = g_server->createService(HOLLOW_OTA_SERVICE_UUID);

    // Audio characteristic (notify)
    g_audioChar = service->createCharacteristic(
        AUDIO_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    g_audioChar->addDescriptor(new BLE2902());

    // Text characteristic (write)
    g_textChar = service->createCharacteristic(
        TEXT_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    g_textChar->setCallbacks(createTextCallbacks());

    // File characteristic (notify + write)
    g_fileChar = fileService->createCharacteristic(
        HOLLOW_FILE_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
    );
    g_fileChar->addDescriptor(new BLE2902());
    g_fileChar->setCallbacks(createFileCallbacks());

    // OTA characteristic (notify + write)
    g_otaChar = otaService->createCharacteristic(
        HOLLOW_OTA_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
    );
    g_otaChar->addDescriptor(new BLE2902());
    initOtaCharacteristic(g_otaChar);
    g_otaChar->setCallbacks(createOtaCallbacks());

    // Start services
    service->start();
    fileService->start();
    otaService->start();

    // Configure advertising
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(HOLLOW_SERVICE_UUID);
    adv->addServiceUUID(HOLLOW_FILE_SERVICE_UUID);
    adv->addServiceUUID(HOLLOW_OTA_SERVICE_UUID);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);  // Preferred connection interval hint
    adv->setMaxPreferred(0x12);

    // POWER: Set advertising intervals - start with normal intervals
    adv->setMinInterval(BLE_ADV_INT_MIN_NORMAL);
    adv->setMaxInterval(BLE_ADV_INT_MAX_NORMAL);

    // Start advertising
    BLEDevice::startAdvertising();

    LOG("[BLE] Initialized: MTU=%d, ADV=%d-%dms (normal mode)\n",
                  BLE_MTU_SIZE,
                  (BLE_ADV_INT_MIN_NORMAL * 625) / 1000,
                  (BLE_ADV_INT_MAX_NORMAL * 625) / 1000);
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

bool bleIsConnected() {
    return g_bleConnected;
}

BLECharacteristic *bleGetAudioChar() {
    return g_audioChar;
}

BLECharacteristic *bleGetTextChar() {
    return g_textChar;
}

BLECharacteristic *bleGetFileChar() {
    return g_fileChar;
}

BLECharacteristic *bleGetOtaChar() {
    return g_otaChar;
}

bool bleNotifyEnabled() {
    if (!g_audioChar) return false;
    BLEDescriptor *d = g_audioChar->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    if (!d) return false;
    uint8_t *val = d->getValue();
    return val && (val[0] & 0x01);
}

bool canSendControlMessages() {
    return g_bleConnected && bleNotifyEnabled();
}

// Called when starting audio transfer - switch to fast connection params
void bleEnterActiveTransfer() {
    requestConnectionParams(true);
}

// Called when audio transfer complete - switch back to low power params
void bleExitActiveTransfer() {
    requestConnectionParams(false);
}

// Periodic advertising restart
void ensureAdvertisingAlive() {
    if (g_bleConnected) return;

    static uint32_t lastKickMs = 0;
    uint32_t now = millis();

    // POWER: Restart advertising less frequently - every 30 seconds
    // (was 15s, but advertising rarely fails and this saves power)
    if (now - lastKickMs > 30000) {
        lastKickMs = now;
        BLEDevice::startAdvertising();
    }
}

// =============================================================================
// POWER: Sleep Mode BLE Optimization
// =============================================================================
// Call these when device enters/exits sleep to reduce BLE power consumption
// while maintaining connectivity (per user requirement: don't disable BLE)

void bleEnterSleepMode() {
    if (s_bleSleepMode) return;  // Already in sleep mode
    s_bleSleepMode = true;

    // If connected, request slower connection parameters
    if (g_bleConnected && g_connId != 0xFFFF) {
        esp_ble_conn_update_params_t params = {};
        memcpy(params.bda, g_peerBda, sizeof(esp_bd_addr_t));  // Required: peer device address
        params.min_int = BLE_CONN_INT_MIN_SLEEP;
        params.max_int = BLE_CONN_INT_MAX_SLEEP;
        params.latency = BLE_LATENCY_SLEEP;
        params.timeout = BLE_TIMEOUT_SLEEP;

        esp_err_t err = esp_ble_gap_update_conn_params(&params);
        if (err == ESP_OK) {
            LOGLN("[BLE] -> SLEEP params (250-500ms interval)");
        }
    }

    // If advertising, switch to slower intervals
    if (!g_bleConnected) {
        BLEAdvertising *adv = BLEDevice::getAdvertising();
        adv->setMinInterval(BLE_ADV_INT_MIN_SLEEP);
        adv->setMaxInterval(BLE_ADV_INT_MAX_SLEEP);
        BLEDevice::startAdvertising();
        LOGLN("[BLE] -> SLEEP advertising (1-2s interval)");
    }
}

void bleExitSleepMode() {
    if (!s_bleSleepMode) return;  // Not in sleep mode
    s_bleSleepMode = false;

    // If connected, restore normal connection parameters
    if (g_bleConnected && g_connId != 0xFFFF) {
        esp_ble_conn_update_params_t params = {};
        memcpy(params.bda, g_peerBda, sizeof(esp_bd_addr_t));  // Required: peer device address
        params.min_int = BLE_CONN_INT_MIN_NORMAL;
        params.max_int = BLE_CONN_INT_MAX_NORMAL;
        params.latency = BLE_LATENCY_NORMAL;
        params.timeout = BLE_TIMEOUT_NORMAL;

        esp_err_t err = esp_ble_gap_update_conn_params(&params);
        if (err == ESP_OK) {
            LOGLN("[BLE] -> NORMAL params (75-150ms interval)");
        }
    }

    // If advertising, restore normal intervals
    if (!g_bleConnected) {
        BLEAdvertising *adv = BLEDevice::getAdvertising();
        adv->setMinInterval(BLE_ADV_INT_MIN_NORMAL);
        adv->setMaxInterval(BLE_ADV_INT_MAX_NORMAL);
        BLEDevice::startAdvertising();
        LOGLN("[BLE] -> NORMAL advertising (500-1000ms interval)");
    }
}

bool bleIsInSleepMode() {
    return s_bleSleepMode;
}

// -----------------------------------------------------------------------------
// Error Handling and Connection Health
// -----------------------------------------------------------------------------

bool bleSendNotifyWithRetry(BLECharacteristic* characteristic, const uint8_t* data, size_t len) {
    if (!characteristic || !g_bleConnected) {
        return false;
    }

    // Try to send with retries on failure
    for (uint32_t retry = 0; retry < MAX_NOTIFY_RETRIES; retry++) {
        characteristic->setValue(const_cast<uint8_t*>(data), len);
        characteristic->notify();

        // Check if still connected after notify (connection may drop on error)
        if (g_bleConnected) {
            // Success tracking
            s_lastSuccessfulNotifyMs = millis();
            return true;
        }

        // Connection lost during notify - retry after brief delay
        if (retry < MAX_NOTIFY_RETRIES - 1) {
            delay(10);  // Brief delay before retry
        }
    }

    // All retries failed
    s_notifyErrors++;
    return false;
}

uint32_t bleGetConnectionErrors() {
    return s_connectionErrors + s_notifyErrors;
}

void bleResetConnectionErrors() {
    s_connectionErrors = 0;
    s_notifyErrors = 0;
}

bool bleIsConnectionHealthy() {
    if (!g_bleConnected) return false;

    uint32_t now = millis();
    uint32_t timeSinceSuccess = now - s_lastSuccessfulNotifyMs;

    // Connection unhealthy if no successful notify in threshold time
    return timeSinceSuccess < CONNECTION_UNHEALTHY_THRESHOLD_MS;
}

// =============================================================================
// Full BLE shutdown for deep sleep
// =============================================================================
void bleFullShutdown() {
    LOGLN("[BLE] Full shutdown for deep sleep...");

    // 1. Stop advertising
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    if (adv) {
        adv->stop();
    }

    // 2. Disconnect any active connection
    if (g_bleConnected && g_server) {
        g_server->disconnect(g_connId);
        g_bleConnected = false;
        g_connId = 0xFFFF;
    }

    // 3. Wait for BLE stack to process pending operations
    delay(150);

    // 4. Deinit entire BLE stack (bluedroid + controller + memory release)
    BLEDevice::deinit(true);

    LOGLN("[BLE] Shutdown complete - stack fully deinitialized");
}
