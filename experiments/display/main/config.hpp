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
// Helper Macros
// ============================================================================

// Convert RGB888 to RGB565
#define RGB565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

// Extract RGB components from RGB565
#define RGB565_R(color) (((color) >> 11) & 0x1F)
#define RGB565_G(color) (((color) >> 5) & 0x3F)
#define RGB565_B(color) ((color) & 0x1F)
