#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// =============================================================================
// POWER: Debug Logging Control
// =============================================================================
// Set HOLLOW_DEBUG=0 in platformio.ini for production to disable all logging
// This saves ~2-5mA by not keeping USB/UART peripherals active
// =============================================================================
#ifndef HOLLOW_DEBUG
#define HOLLOW_DEBUG 0  // Default to disabled for power savings
#endif

#if HOLLOW_DEBUG
    #define LOG_INIT()     Serial.begin(115200)
    #define LOG(fmt, ...)  Serial.printf(fmt, ##__VA_ARGS__)
    #define LOGLN(msg)     Serial.println(msg)
#else
    #define LOG_INIT()     do {} while(0)
    #define LOG(fmt, ...)  do {} while(0)
    #define LOGLN(msg)     do {} while(0)
#endif

// T-Watch S3 Hardware Pin Definitions
// Based on official LilyGo T-Watch S3 specifications

// I2C Bus (Shared by PMU, RTC, Accelerometer)
#define I2C_SDA_PIN             10
#define I2C_SCL_PIN             11

// Display SPI (ST7789)
#define TFT_MOSI_PIN            13
#define TFT_MISO_PIN            -1   // Not used
#define TFT_SCLK_PIN            18
#define TFT_CS_PIN              12
#define TFT_DC_PIN              38
#define TFT_RST_PIN             -1   // Not used (controlled via PMU)
#define TFT_BL_PIN              45   // Backlight PWM

// Touch Controller (FT6336)
#define TOUCH_SDA_PIN           39
#define TOUCH_SCL_PIN           40
#define TOUCH_INT_PIN           16
#define TOUCH_I2C_ADDR          0x38

// Power Management Unit (AXP2101)
#define PMU_SDA_PIN             10   // Shared I2C bus
#define PMU_SCL_PIN             11   // Shared I2C bus
#define PMU_INT_PIN             21

// Audio Codec (MAX98357A) - I2S Output
#define I2S_BCK_PIN             48   // Bit clock
#define I2S_WS_PIN              15   // Word select (LRCLK)
#define I2S_DOUT_PIN            46   // Data out

// Microphone (PDM)
#define MIC_DATA_PIN            47   // PDM data
#define MIC_CLK_PIN             44   // PDM clock

// Accelerometer (BMA423)
#define ACCEL_INT_PIN           14

// RTC (PCF8563)
#define RTC_INT_PIN             17

// IR Transmitter
#define IR_TX_PIN               2

// Radio Module (SX1262) - Optional
#define RADIO_MOSI_PIN          1
#define RADIO_MISO_PIN          4
#define RADIO_SCLK_PIN          3
#define RADIO_CS_PIN            5
#define RADIO_DIO1_PIN          9
#define RADIO_RST_PIN           8
#define RADIO_BUSY_PIN          7
#define RADIO_DIO3_PIN          6

// GPS Module - Optional
#define GPS_TX_PIN              42
#define GPS_RX_PIN              41

// Display Configuration
#define SCREEN_WIDTH            240
#define SCREEN_HEIGHT           240
#define SCREEN_ROTATION         0

// Hardware Features
#define HAS_DISPLAY             1
#define HAS_TOUCH               1
#define HAS_PMU                 1
#define HAS_ACCELEROMETER       1
#define HAS_RTC                 1
#define HAS_HAPTIC              1
#define HAS_MICROPHONE          1
#define HAS_SPEAKER             1

#endif // HARDWARE_CONFIG_H
