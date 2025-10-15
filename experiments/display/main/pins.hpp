#pragma once

// Hardware pin definitions and constants for Adafruit ESP32-S3 Reverse TFT

// ============================================================================
// Display Pins
// ============================================================================
#define TFT_I2C_POWER 7     // Display power enable

// SPI Pins
#define TFT_PIN_SCLK  36    // SPI SCLK
#define TFT_PIN_MOSI  35    // SPI MOSI
#define TFT_PIN_MISO  37    // SPI MISO (not used for write-only)
#define TFT_PIN_DC    40    // Data/Command pin
#define TFT_PIN_CS    42    // Chip select
#define TFT_PIN_RST   41    // Reset pin
#define TFT_PIN_BL    45    // Backlight PWM pin

// Display Panel Configuration
#define TFT_PANEL_WIDTH     135
#define TFT_PANEL_HEIGHT    240
#define TFT_OFFSET_X        52
#define TFT_OFFSET_Y        40
#define TFT_OFFSET_ROTATION 0
#define TFT_INVERT          true
#define TFT_RGB_ORDER       false  // false = RGB, true = BGR

// SPI Configuration
#define TFT_SPI_HOST        SPI2_HOST
#define TFT_SPI_MODE        0
#define TFT_SPI_FREQ_WRITE  40000000  // 40MHz
#define TFT_SPI_FREQ_READ   16000000  // 16MHz

// Backlight PWM Configuration
#define TFT_BL_FREQ         44100
#define TFT_BL_PWM_CHANNEL  1

// ============================================================================
// Physical Button GPIOs
// ============================================================================
#define GPIO_BUTTON_UP   2  // D2 - Top button (normally low, active high)
#define GPIO_BUTTON_MODE 1  // D1 - Middle button (normally low, active high)
#define GPIO_BUTTON_DOWN 0  // D0 - Bottom button (normally high, active low)

// Button Active Levels
#define BUTTON_UP_ACTIVE_LOW   false
#define BUTTON_MODE_ACTIVE_LOW false
#define BUTTON_DOWN_ACTIVE_LOW true

// Display Configuration
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 135
#define SCREEN_ROTATION 3  // Landscape mode

// UI Layout Constants
#define STATUS_BAR_HEIGHT 16
#define MODE_PANEL_WIDTH  70
#define BUTTON_PANEL_WIDTH 50
#define BUTTON_SPACING 0

// Calculate derived dimensions
#define CONTENT_AREA_WIDTH (SCREEN_WIDTH - BUTTON_PANEL_WIDTH)
#define STATUS_BAR_WIDTH (CONTENT_AREA_WIDTH)
#define MAIN_CONTENT_HEIGHT (SCREEN_HEIGHT - STATUS_BAR_HEIGHT)
#define LEVEL_DISPLAY_WIDTH (CONTENT_AREA_WIDTH - MODE_PANEL_WIDTH)

// ============================================================================
// Timing Configuration
// ============================================================================
#define DEBOUNCE_DELAY_MS 50
#define STARTUP_ANIMATION_FRAME_DELAY_MS 2
