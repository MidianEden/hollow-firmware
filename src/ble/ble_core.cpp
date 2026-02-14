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
#include <BLESecurity.h>

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
// Fast advertising on boot/wake: 50ms for quick discovery
constexpr uint16_t BLE_ADV_INT_MIN_FAST   = 0x0050;   // 50ms
constexpr uint16_t BLE_ADV_INT_MAX_FAST   = 0x0050;   // 50ms
constexpr uint32_t BLE_FAST_ADV_DURATION_MS = 25000;   // 25 seconds
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

// Fast advertising state (boot/deep-sleep wake burst)
static uint32_t s_fastAdvStartMs = 0;
static bool s_fastAdvActive = false;

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
    } else {
        params.min_int = BLE_CONN_INT_MIN_NORMAL;
        params.max_int = BLE_CONN_INT_MAX_NORMAL;
        params.latency = BLE_LATENCY_NORMAL;
        params.timeout = BLE_TIMEOUT_NORMAL;
    }

    esp_err_t err = esp_ble_gap_update_conn_params(&params);
    if (err != ESP_OK) {
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

        Serial.printf("[BLE] connected peer=%02X:%02X:%02X:%02X:%02X:%02X\n",
                      g_peerBda[0], g_peerBda[1], g_peerBda[2],
                      g_peerBda[3], g_peerBda[4], g_peerBda[5]);

        // Notify power manager
        powerHandleBLEConnect();
        powerMarkActivity();

        // Short delay for connection to stabilize
        delay(50);  // Reduced from 100ms

        // Request link encryption - restores bonding keys for bonded peers,
        // or triggers pairing/bonding for new peers
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);

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

        Serial.println("[BLE] disconnected, restarting advertising");

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

        // Restart advertising - use fast intervals if still in boot burst window
        if (s_fastAdvActive) {
            BLEAdvertising *adv = BLEDevice::getAdvertising();
            adv->setMinInterval(BLE_ADV_INT_MIN_FAST);
            adv->setMaxInterval(BLE_ADV_INT_MAX_FAST);
        }
        BLEDevice::startAdvertising();
        Serial.println("[BLE] advertising restarted after disconnect");
    }
};

// -----------------------------------------------------------------------------
// Security Callbacks - BLE bonding event handler
// -----------------------------------------------------------------------------

class SecurityCallbacks : public BLESecurityCallbacks {
    uint32_t onPassKeyRequest() override {
        Serial.println("[BLE-SEC] passkey request (Just Works)");
        return 0;
    }
    void onPassKeyNotify(uint32_t pass_key) override {
        Serial.printf("[BLE-SEC] passkey display: %06lu\n", (unsigned long)pass_key);
    }
    bool onConfirmPIN(uint32_t pin) override {
        Serial.printf("[BLE-SEC] numeric comparison %06lu -> auto-accept\n", (unsigned long)pin);
        return true;
    }
    bool onSecurityRequest() override {
        Serial.println("[BLE-SEC] security request -> accept");
        return true;
    }
    void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override {
        if (cmpl.success) {
            Serial.printf("[BLE-SEC] bonding OK peer=%02X:%02X:%02X:%02X:%02X:%02X mode=0x%02x\n",
                cmpl.bd_addr[0], cmpl.bd_addr[1], cmpl.bd_addr[2],
                cmpl.bd_addr[3], cmpl.bd_addr[4], cmpl.bd_addr[5],
                cmpl.auth_mode);
        } else {
            Serial.printf("[BLE-SEC] bonding FAILED reason=0x%x\n", cmpl.fail_reason);
        }
    }
};

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

void initBLE() {

    BLEDevice::init(DEVICE_NAME);

    // Log BLE address - this is the public address from eFuse, stable across deep sleep
    esp_bd_addr_t bleAddr;
    memcpy(bleAddr, BLEDevice::getAddress().getNative(), sizeof(bleAddr));
    Serial.printf("[BLE] address=%02X:%02X:%02X:%02X:%02X:%02X (public, eFuse-stable)\n",
                  bleAddr[0], bleAddr[1], bleAddr[2], bleAddr[3], bleAddr[4], bleAddr[5]);

    // =========================================================================
    // SECURITY - BLE bonding for iOS auto-reconnect after deep sleep
    // =========================================================================
    // Bond data is persisted in NVS automatically by the ESP-IDF Bluedroid stack.
    // The ESP32's public BLE address (eFuse) is stable across deep sleep.
    // After initial pairing, iOS stores the bond and can auto-reconnect to the
    // same address + service UUID without user interaction, even from background.
    BLEDevice::setSecurityCallbacks(new SecurityCallbacks());

    BLESecurity security;
    security.setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);  // Secure Connections + Bonding
    security.setCapability(ESP_IO_CAP_NONE);                  // Just Works (no I/O on watch)
    security.setKeySize(16);
    security.setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);  // Distribute LTK + IRK
    security.setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);  // Accept LTK + IRK
    Serial.println("[BLE-SEC] security: SC+Bond, JustWorks, LTK+IRK");

    // Log bonded peers persisted in NVS from previous sessions
    int bondCount = esp_ble_get_bond_device_num();
    Serial.printf("[BLE-SEC] bonded peers in NVS: %d\n", bondCount);
    if (bondCount > 0) {
        esp_ble_bond_dev_t *bondList = (esp_ble_bond_dev_t *)malloc(
            bondCount * sizeof(esp_ble_bond_dev_t));
        if (bondList) {
            int actual = bondCount;
            esp_ble_get_bond_device_list(&actual, bondList);
            for (int i = 0; i < actual; i++) {
                Serial.printf("[BLE-SEC]   peer[%d]=%02X:%02X:%02X:%02X:%02X:%02X\n", i,
                    bondList[i].bd_addr[0], bondList[i].bd_addr[1], bondList[i].bd_addr[2],
                    bondList[i].bd_addr[3], bondList[i].bd_addr[4], bondList[i].bd_addr[5]);
            }
            free(bondList);
        }
    }

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

    // =========================================================================
    // ADVERTISING - iOS auto-reconnect compatible
    // =========================================================================
    // Primary ADV payload: Hollow service UUID ONLY (no file/OTA UUIDs)
    // iOS matches on this UUID for background reconnection.
    // File and OTA services are discovered via GATT after connection.
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(HOLLOW_SERVICE_UUID);
    adv->setScanResponse(true);    // Name goes in scan response
    adv->setMinPreferred(0x06);    // Preferred connection interval hint
    adv->setMaxPreferred(0x12);

    // Start with fast advertising (50ms) for quick discovery on boot/wake
    adv->setMinInterval(BLE_ADV_INT_MIN_FAST);
    adv->setMaxInterval(BLE_ADV_INT_MAX_FAST);
    s_fastAdvStartMs = millis();
    s_fastAdvActive = true;

    // Start advertising
    BLEDevice::startAdvertising();
    Serial.printf("[BLE] advertising started (fast 50ms, svc=%s)\n", HOLLOW_SERVICE_UUID);
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

// Periodic advertising restart + fast→normal transition
void ensureAdvertisingAlive() {
    if (g_bleConnected) return;

    uint32_t now = millis();

    // Transition from fast to normal advertising after 25 seconds
    if (s_fastAdvActive && (now - s_fastAdvStartMs >= BLE_FAST_ADV_DURATION_MS)) {
        s_fastAdvActive = false;
        BLEAdvertising *adv = BLEDevice::getAdvertising();
        adv->setMinInterval(BLE_ADV_INT_MIN_NORMAL);
        adv->setMaxInterval(BLE_ADV_INT_MAX_NORMAL);
        BLEDevice::startAdvertising();
        return;
    }

    static uint32_t lastKickMs = 0;

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
    s_fastAdvActive = false;  // Cancel fast advertising burst

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
        }
    }

    // If advertising, switch to slower intervals
    if (!g_bleConnected) {
        BLEAdvertising *adv = BLEDevice::getAdvertising();
        adv->setMinInterval(BLE_ADV_INT_MIN_SLEEP);
        adv->setMaxInterval(BLE_ADV_INT_MAX_SLEEP);
        BLEDevice::startAdvertising();
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
        }
    }

    // If advertising, restore normal intervals
    if (!g_bleConnected) {
        BLEAdvertising *adv = BLEDevice::getAdvertising();
        adv->setMinInterval(BLE_ADV_INT_MIN_NORMAL);
        adv->setMaxInterval(BLE_ADV_INT_MAX_NORMAL);
        BLEDevice::startAdvertising();
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

}
