// =============================================================================
// AUDIO I2S - OPTIMIZED PERSISTENT PDM MICROPHONE DRIVER
// =============================================================================
// Key optimizations:
// 1. Persistent driver - installed once, never uninstalled for instant start
// 2. Larger DMA buffers for better reliability
// 3. Timeout protection during recording
// 4. BLE disconnection detection with auto-stop
// 5. Error counting and recovery
// =============================================================================

#include "audio_i2s.h"

#include <driver/i2s.h>
#include <driver/gpio.h>
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../hardware_config.h"
#include "audio_adpcm.h"
#include "../ble/ble_core.h"
#include "../ble/ble_audio.h"
#include "../system/state.h"
#include "../power/power_manager.h"

// =============================================================================
// CONFIGURATION
// =============================================================================

// DMA buffer configuration - larger buffers for more stability
constexpr int DMA_BUF_COUNT = 6;         // Number of DMA buffers
constexpr int DMA_BUF_LEN = 512;         // Samples per buffer (increased from 256)

// Recording timeout protection
constexpr uint32_t RECORDING_MAX_DURATION_MS = 60000;  // 60 second max recording
constexpr uint32_t RECORDING_IDLE_TIMEOUT_MS = 5000;   // Auto-stop if no BLE send for 5s
constexpr uint32_t I2S_READ_TIMEOUT_MS = 100;          // Timeout for i2s_read()

// Error thresholds
constexpr uint32_t MAX_CONSECUTIVE_ERRORS = 10;        // Auto-stop after this many errors

// =============================================================================
// State Tracking
// =============================================================================

static bool s_i2sInstalled = false;
static bool s_i2sRunning = false;
static bool s_micPinsConfigured = false;

// Recording state
bool g_is_recording = false;
std::vector<uint8_t> g_recorded_adpcm;

// Timing and error tracking
static uint32_t s_recordingStartMs = 0;
static uint32_t s_lastSuccessfulSendMs = 0;
static uint32_t s_consecutiveErrors = 0;
static uint32_t s_totalChunksSent = 0;

// =============================================================================
// I2S Configuration for PDM Microphone
// =============================================================================

static const i2s_config_t I2S_CONFIG = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,        // APLL uses more power, not needed for 16kHz
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
};

static const i2s_pin_config_t I2S_PINS = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = I2S_PIN_NO_CHANGE,
    .ws_io_num = MIC_CLK_PIN,          // PDM CLK (GPIO 44)
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = MIC_DATA_PIN        // PDM DATA (GPIO 47)
};

// =============================================================================
// GPIO Power Control
// =============================================================================

static void micGpioConfigure() {
    // Configure GPIO for I2S PDM operation
    gpio_set_direction((gpio_num_t)MIC_CLK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)MIC_DATA_PIN, GPIO_MODE_INPUT);
    s_micPinsConfigured = true;
}

static void micGpioRelease() {
    // Put GPIO in high-impedance state to prevent current leakage
    gpio_reset_pin((gpio_num_t)MIC_CLK_PIN);
    gpio_reset_pin((gpio_num_t)MIC_DATA_PIN);

    // Explicitly set to input with no pull to minimize leakage
    gpio_set_direction((gpio_num_t)MIC_CLK_PIN, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)MIC_DATA_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)MIC_CLK_PIN, GPIO_FLOATING);
    gpio_set_pull_mode((gpio_num_t)MIC_DATA_PIN, GPIO_FLOATING);
    s_micPinsConfigured = false;
}

// =============================================================================
// Public API: Initialization
// =============================================================================

void initMic() {
    if (s_i2sInstalled) {
        LOGLN("[MIC] Already installed");
        return;
    }

    LOGLN("[MIC] Installing I2S driver...");

    // Install I2S driver
    esp_err_t err = i2s_driver_install(I2S_NUM_0, &I2S_CONFIG, 0, nullptr);
    if (err != ESP_OK) {
        LOG("[MIC] ERROR: i2s_driver_install failed: %s\n", esp_err_to_name(err));
        return;
    }

    // Configure GPIO + pins
    micGpioConfigure();
    err = i2s_set_pin(I2S_NUM_0, &I2S_PINS);
    if (err != ESP_OK) {
        LOG("[MIC] ERROR: i2s_set_pin failed: %s\n", esp_err_to_name(err));
        i2s_driver_uninstall(I2S_NUM_0);
        micGpioRelease();
        return;
    }

    // Stop immediately - we only start when actually recording
    i2s_stop(I2S_NUM_0);
    // POWER: Release mic GPIOs while idle to reduce leakage
    micGpioRelease();

    s_i2sInstalled = true;
    s_i2sRunning = false;

    LOG("[MIC] I2S driver installed (DMA: %d x %d samples)\n",
                  DMA_BUF_COUNT, DMA_BUF_LEN);
}

void startMic() {
    // Driver should already be installed at boot
    if (!s_i2sInstalled) {
        LOGLN("[MIC] WARNING: Driver not installed, initializing now");
        initMic();
        if (!s_i2sInstalled) {
            LOGLN("[MIC] ERROR: Failed to init");
            return;
        }
    }

    if (s_i2sRunning) {
        LOGLN("[MIC] Already running");
        return;
    }

    // Ensure GPIOs/pins are configured before starting
    if (!s_micPinsConfigured) {
        micGpioConfigure();
        esp_err_t pinErr = i2s_set_pin(I2S_NUM_0, &I2S_PINS);
        if (pinErr != ESP_OK) {
            LOG("[MIC] ERROR: i2s_set_pin failed: %s\n", esp_err_to_name(pinErr));
            micGpioRelease();
            return;
        }
    }

    // Reset state tracking
    s_recordingStartMs = millis();
    s_lastSuccessfulSendMs = millis();
    s_consecutiveErrors = 0;
    s_totalChunksSent = 0;

    // Clear DMA buffers for clean start
    i2s_zero_dma_buffer(I2S_NUM_0);

    // Start I2S - instant since driver is already installed
    esp_err_t err = i2s_start(I2S_NUM_0);
    if (err != ESP_OK) {
        LOG("[MIC] ERROR: i2s_start failed: %s\n", esp_err_to_name(err));
        return;
    }

    s_i2sRunning = true;

    // Notify BLE to use fast connection parameters
    bleEnterActiveTransfer();

    LOGLN("[MIC] Started - recording active");
}

void stopMic() {
    if (!s_i2sRunning) {
        return;
    }

    i2s_stop(I2S_NUM_0);
    s_i2sRunning = false;
    // POWER: Release mic GPIOs when idle to reduce leakage
    micGpioRelease();

    // Calculate recording stats
    uint32_t durationMs = millis() - s_recordingStartMs;
    LOG("[MIC] Stopped after %lu ms, %lu chunks sent\n",
                  durationMs, s_totalChunksSent);

    // Notify BLE to return to low-power connection parameters
    bleExitActiveTransfer();
}

void deinitMic() {
    // NOTE: This should ONLY be called during deep sleep/shutdown
    if (s_i2sRunning) {
        stopMic();
    }

    if (!s_i2sInstalled) {
        return;
    }

    // Uninstall I2S driver
    i2s_driver_uninstall(I2S_NUM_0);
    s_i2sInstalled = false;

    // Release GPIO to minimize leakage current
    micGpioRelease();

    LOGLN("[MIC] I2S driver uninstalled (shutdown only)");
}

bool isMicRunning() {
    return s_i2sRunning;
}

bool isMicInstalled() {
    return s_i2sInstalled;
}

// =============================================================================
// Idle Check - NO-OP (driver is persistent)
// =============================================================================

void micIdleCheck() {
    // NO-OP: Driver is persistent for instant recording start
}

// =============================================================================
// Recording Buffer Management
// =============================================================================

void clearRecordingBuffer() {
    g_recorded_adpcm.clear();
    g_recorded_adpcm.shrink_to_fit();
}

void setRecordingActive(bool active) {
    g_is_recording = active;
}

bool isRecordingActive() {
    return g_is_recording;
}

// =============================================================================
// Recording Loop - Called from main loop during recording
// =============================================================================

void updateRecording() {
    // -------------------------------------------------------------------------
    // Safety checks
    // -------------------------------------------------------------------------
    if (!s_i2sRunning || !g_is_recording) {
        return;
    }

    // Reset watchdog
    esp_task_wdt_reset();

    uint32_t now = millis();

    // -------------------------------------------------------------------------
    // Timeout protection: Max recording duration (60 seconds)
    // -------------------------------------------------------------------------
    if (now - s_recordingStartMs > RECORDING_MAX_DURATION_MS) {
        LOGLN("[MIC] Max duration reached - auto-stopping");
        stopRecording();
        return;
    }

    // -------------------------------------------------------------------------
    // BLE connection check - auto-stop if disconnected
    // -------------------------------------------------------------------------
    if (!bleIsConnected()) {
        LOGLN("[MIC] BLE disconnected - stopping recording");
        stopRecording();
        return;
    }

    // NOTE: Removed aggressive idle timeout and error threshold checks
    // These were causing premature recording stops. The max duration
    // and BLE disconnect checks provide sufficient protection.

    // -------------------------------------------------------------------------
    // Read PCM samples from I2S DMA buffer
    // -------------------------------------------------------------------------
    const int PCM_COUNT = DMA_BUF_LEN;
    int16_t pcm[PCM_COUNT];
    size_t bytesRead = 0;

    esp_err_t err = i2s_read(I2S_NUM_0, pcm, sizeof(pcm), &bytesRead,
                             I2S_READ_TIMEOUT_MS / portTICK_PERIOD_MS);

    if (err != ESP_OK) {
        s_consecutiveErrors++;
        if (s_consecutiveErrors % 5 == 0) {  // Log every 5th error
            LOG("[MIC] i2s_read error: %s\n", esp_err_to_name(err));
        }
        return;
    }

    if (bytesRead == 0) {
        return;  // No data yet, normal
    }

    size_t sampleCount = bytesRead / sizeof(int16_t);
    if (sampleCount == 0) {
        return;
    }

    // -------------------------------------------------------------------------
    // Encode PCM to ADPCM (4:1 compression)
    // -------------------------------------------------------------------------
    uint8_t adpcmBuf[PCM_COUNT / 2 + 4];
    size_t outBytes = ima_encode_block(pcm, sampleCount, adpcmBuf);

    if (outBytes == 0) {
        s_consecutiveErrors++;
        return;
    }

    // -------------------------------------------------------------------------
    // Send over BLE
    // -------------------------------------------------------------------------
    bleSendAudioChunk(adpcmBuf, outBytes);

    // Success - reset error counter and update timing
    s_consecutiveErrors = 0;
    s_lastSuccessfulSendMs = now;
    s_totalChunksSent++;

    // -------------------------------------------------------------------------
    // Yield to BLE stack
    // -------------------------------------------------------------------------
    // Brief delay to let BLE notification queue process
    // This is critical for preventing buffer overflows
    vTaskDelay(1);
}

// =============================================================================
// Recording Statistics (for diagnostics)
// =============================================================================

uint32_t getRecordingDurationMs() {
    if (!s_i2sRunning) return 0;
    return millis() - s_recordingStartMs;
}

uint32_t getRecordingChunksSent() {
    return s_totalChunksSent;
}

uint32_t getRecordingErrors() {
    return s_consecutiveErrors;
}
