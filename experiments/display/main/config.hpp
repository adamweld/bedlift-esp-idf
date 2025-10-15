#pragma once

#include <cstdint>

// ============================================================================
// Color Definitions
// ============================================================================
// All colors defined as RGB565 hex values for LovyanGFX

// Basic Colors
#define COLOR_BLACK         0x0000  // 0, 0, 0
#define COLOR_WHITE         0xFFFF  // 255, 255, 255

// Grayscale
#define COLOR_DARKGREY      0x7BEF  // 128, 128, 128
#define COLOR_GREY          0xAD55  // 169, 169, 169
#define COLOR_LIGHTGREY     0xD69A  // 211, 211, 211

// Primary Colors
#define COLOR_RED           0xF800  // 255, 0, 0
#define COLOR_GREEN         0x07E0  // 0, 255, 0
#define COLOR_BLUE          0x001F  // 0, 0, 255

// Secondary Colors
#define COLOR_CYAN          0x07FF  // 0, 255, 255
#define COLOR_MAGENTA       0xF81F  // 255, 0, 255
#define COLOR_YELLOW        0xFFE0  // 255, 255, 0

// Dark Variants
#define COLOR_DARKRED       0x7800  // 128, 0, 0
#define COLOR_DARKGREEN     0x03E0  // 0, 128, 0
#define COLOR_DARKBLUE      0x000F  // 0, 0, 128
#define COLOR_DARKCYAN      0x03EF  // 0, 128, 128

// Accent Colors
#define COLOR_ORANGE        0xFDA0  // 255, 180, 0
#define COLOR_PURPLE        0x780F  // 128, 0, 128
#define COLOR_BROWN         0x9A60  // 150, 75, 0
#define COLOR_PINK          0xFE19  // 255, 192, 203

// Special Purpose
#define COLOR_TRANSPARENT   0x0120  // Transparent color marker


// ============================================================================
// Status Bar Monitor Configuration
// ============================================================================

enum class MonitorType {
    DEV_MODE = 0,
    MOTORS,
    SENSORS,
    LOCK,
    BATTERY,
    MONITOR_COUNT
};

struct MonitorConfig {
    const char* name;              // Monitor name for debugging
    const char* icon_true_file;    // Icon filename when true (null = hide)
    const char* icon_false_file;   // Icon filename when false (null = hide)
};

// Monitor configurations indexed by MonitorType enum
static const MonitorConfig MONITOR_CONFIGS[] = {
    // DEV_MODE
    {
        .name = "Dev Mode",
        .icon_true_file = "hand-middle-finger.png",
        .icon_false_file = nullptr
    },
    // MOTORS
    {
        .name = "Motors",
        .icon_true_file = "settings.png",
        .icon_false_file = nullptr
    },
    // SENSORS
    {
        .name = "Sensors",
        .icon_true_file = "ruler-measure.png",
        .icon_false_file = nullptr
    },
    // LOCK
    {
        .name = "Lock",
        .icon_true_file = "lock.png",
        .icon_false_file = "lock-open.png"
    },
    // BATTERY
    {
        .name = "Battery",
        .icon_true_file = "battery.png",
        .icon_false_file = "battery-off.png"
    }
};

// Monitor state storage (shared between UI and background threads)
struct MonitorStates {
    bool dev_mode = false;
    bool motors = false;
    bool sensors = false;
    bool lock = false;
    bool battery = false;
};

// ============================================================================
// Mode Configuration
// ============================================================================

struct ModeConfig {
    const char* name;            // Display name for the mode
    const char* icon_file;       // Icon filename (without path, from ../icons/)
    int rotation;                // Icon rotation in degrees: 0, 90, 180, 270
    bool dev_only;               // True if mode requires dev flag to be enabled
    const char* button_up_file;  // Up button icon (48x48, no rotation)
    const char* button_mode_file; // Mode button icon (48x48, no rotation)
    const char* button_down_file; // Down button icon (48x48, no rotation)
    uint16_t bg_color;           // Background color for mode panel (RGB565)
};

// Mode configurations indexed by OperationMode enum
static const ModeConfig MODE_CONFIGS[] = {
    // Index 0: UP_DOWN
    {
        .name = "Up/Down",
        .icon_file = "arrows-up-down.png",
        .rotation = 0,
        .dev_only = false,
        .button_up_file = "caret-up.png",
        .button_mode_file = "stack.png",
        .button_down_file = "caret-down.png",
        .bg_color = COLOR_BLACK
    },

    // Index 1: ROLL
    {
        .name = "Roll",
        .icon_file = "rotate-360.png",
        .rotation = 1,
        .dev_only = false,
        .button_up_file = "caret-right.png",
        .button_mode_file = "stack.png",
        .button_down_file = "caret-left.png",
        .bg_color = COLOR_BLACK
    },

    // Index 2: PITCH
    {
        .name = "Pitch",
        .icon_file = "view-360-arrow.png",
        .rotation = 1,
        .dev_only = false,
        .button_up_file = "caret-up.png",
        .button_mode_file = "stack.png",
        .button_down_file = "caret-down.png",
        .bg_color = COLOR_BLACK
    },

    // Index 3: TORSION
    {
        .name = "Torsion",
        .icon_file = "stretching.png",
        .rotation = 0,
        .dev_only = false,
        .button_up_file = "caret-right.png",
        .button_mode_file = "stack.png",
        .button_down_file = "caret-left.png",
        .bg_color = COLOR_BLACK
    },

    // Index 4: LEVEL
    {
        .name = "Level",
        // .icon_file = "atom-2.png",
        .icon_file = "wand.png",
        .rotation = 0,
        .dev_only = false,
        .button_up_file = "sparkles.png",
        .button_mode_file = "stack.png",
        .button_down_file = "hand-middle-finger.png",
        .bg_color = COLOR_BLACK
    },

    // Index 5: MOTOR_1 (dev only)
    {
        .name = "Motor 1",
        .icon_file = "box-align-bottom-right.png",
        .rotation = 0,
        .dev_only = true,
        .button_up_file = "caret-up.png",
        .button_mode_file = "stack.png",
        .button_down_file = "caret-down.png",
        .bg_color = COLOR_DARKBLUE
    },

    // Index 6: MOTOR_2 (dev only)
    {
        .name = "Motor 2",
        .icon_file = "box-align-bottom-right.png",
        .rotation = 1,
        .dev_only = true,
        .button_up_file = "caret-up.png",
        .button_mode_file = "stack.png",
        .button_down_file = "caret-down.png",
        .bg_color = COLOR_DARKGREEN
    },

    // Index 7: MOTOR_3 (dev only)
    {
        .name = "Motor 3",
        .icon_file = "box-align-bottom-right.png",
        .rotation = 3,
        .dev_only = true,
        .button_up_file = "caret-up.png",
        .button_mode_file = "stack.png",
        .button_down_file = "caret-down.png",
        .bg_color = COLOR_DARKRED
    },

    // Index 8: MOTOR_4 (dev only)
    {
        .name = "Motor 4",
        .icon_file = "box-align-bottom-right.png",
        .rotation = 2,
        .dev_only = true,
        .button_up_file = "caret-up.png",
        .button_mode_file = "stack.png",
        .button_down_file = "caret-down.png",
        .bg_color = COLOR_DARKCYAN
    }
};

// ============================================================================
// UI Color Scheme
// ============================================================================
// Application-specific color assignments

// General UI
#define COLOR_BACKGROUND        COLOR_BLACK
#define COLOR_FOREGROUND        COLOR_WHITE
#define COLOR_ACCENT            COLOR_DARKGREY
#define COLOR_BORDER            COLOR_LIGHTGREY

// Status Bar
#define COLOR_STATUS_BAR_BG     COLOR_DARKGREY
#define COLOR_STATUS_BAR_TEXT   COLOR_WHITE
#define COLOR_STATUS_BAR_ACCENT COLOR_GREEN

// Mode Panel
#define COLOR_MODE_PANEL_BG     COLOR_BLACK
#define COLOR_MODE_PANEL_BORDER COLOR_DARKGREY
#define COLOR_MODE_PANEL_TEXT   COLOR_WHITE
#define COLOR_MODE_ICON_FG      COLOR_WHITE
#define COLOR_MODE_ICON_BG      COLOR_DARKGREY

// Level Display
#define COLOR_LEVEL_BG          COLOR_BLACK
#define COLOR_LEVEL_BORDER      COLOR_DARKGREY
#define COLOR_LEVEL_CROSSHAIR   COLOR_WHITE
#define COLOR_LEVEL_BUBBLE_FG   COLOR_WHITE
#define COLOR_LEVEL_BUBBLE_BG   COLOR_DARKGREY
#define COLOR_LEVEL_TEXT        COLOR_WHITE

// Button Panel
#define COLOR_BUTTON_NORMAL     COLOR_DARKGREY
#define COLOR_BUTTON_PRESSED    COLOR_WHITE
#define COLOR_BUTTON_BORDER     COLOR_WHITE
#define COLOR_BUTTON_TEXT       COLOR_WHITE
#define COLOR_BUTTON_TEXT_INV   COLOR_BLACK

// Status Indicators
#define COLOR_STATUS_OK         COLOR_GREEN
#define COLOR_STATUS_WARNING    COLOR_YELLOW
#define COLOR_STATUS_ERROR      COLOR_RED
#define COLOR_STATUS_INFO       COLOR_CYAN


// ============================================================================
// Power Management and Display
// ============================================================================
// Auto-dim timeout in seconds (reduce backlight)
#define AUTO_DIM_TIMEOUT_SEC 45

// Auto-sleep timeout in seconds (enter deep sleep)
#define AUTO_SLEEP_TIMEOUT_SEC 60

// Backlight levels
#define BACKLIGHT_FULL 128
#define BACKLIGHT_DIMMED 32

// Display Configuration
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 135
#define SCREEN_ROTATION 3  // Landscape mode

// UI Layout Constants
#define STATUS_BAR_HEIGHT 32
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
#define STARTUP_ANIMATION_FRAME_DELAY_MS 4


// ============================================================================
// Helper Macros
// ============================================================================

// Convert RGB888 to RGB565
#define RGB565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

// Extract RGB components from RGB565
#define RGB565_R(color) (((color) >> 11) & 0x1F)
#define RGB565_G(color) (((color) >> 5) & 0x3F)
#define RGB565_B(color) ((color) & 0x1F)
