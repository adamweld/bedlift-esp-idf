#include "ui.hpp"
#include "config.hpp"
#include "assets/icons.hpp"
#include <cstring>
#include <cstring>  // for strcmp in icon lookup

// ============================================================================
// StatusBar Implementation
// ============================================================================
StatusBar::StatusBar()
    : gfx(nullptr), x_(0), y_(0), w_(0), h_(0), monitors_(nullptr) {
}

void StatusBar::init(LGFX* display, int x, int y, int w, int h, MonitorStates* monitors) {
    gfx = display;
    x_ = x;
    y_ = y;
    w_ = w;
    h_ = h;
    monitors_ = monitors;
}

void StatusBar::draw() {
    if (!gfx || !monitors_) return;

    // Background (blank/black)
    gfx->fillRect(x_, y_, w_, h_, COLOR_BLACK);

    // Dim border
    gfx->drawRect(x_, y_, w_, h_, COLOR_DARKGREY);

    // Create array of monitor states for iteration
    bool monitor_states[5] = {
        monitors_->dev_mode,
        monitors_->motors,
        monitors_->sensors,
        monitors_->lock,
        monitors_->battery
    };

    // Draw monitors from left to right with spacing
    int icon_spacing = 2;
    int current_x = x_ + 4;  // Start with 4px padding from left edge

    for (int i = 0; i < 5; i++) {
        const uint8_t* icon = nullptr;

        // Get the appropriate icon based on monitor state
        if (monitor_states[i]) {
            icon = getMonitorIconTrue(i);
        } else {
            icon = getMonitorIconFalse(i);
        }

        // Only draw if icon exists (null = hide)
        if (icon) {
            int icon_y = y_ + (h_ - MONITOR_ICON_SIZE) / 2;  // Center vertically
            gfx->drawBitmap(current_x, icon_y, icon, MONITOR_ICON_SIZE, MONITOR_ICON_SIZE, COLOR_WHITE);
            current_x += MONITOR_ICON_SIZE + icon_spacing;
        }
    }
}

// ============================================================================
// ModePanel Implementation
// ============================================================================
ModePanel::ModePanel()
    : gfx(nullptr), x_(0), y_(0), w_(0), h_(0),
      current_mode(OperationMode::UP_DOWN) {}

void ModePanel::init(LGFX* display, int x, int y, int w, int h) {
    gfx = display;
    x_ = x;
    y_ = y;
    w_ = w;
    h_ = h;
}

void ModePanel::draw() {
    if (!gfx) return;

    // Background
    gfx->fillRect(x_, y_, w_, h_, COLOR_MODE_PANEL_BG);
    gfx->drawRect(x_, y_, w_, h_, COLOR_MODE_PANEL_BORDER);

    // Icon area (top 2/3)
    int icon_area_h = (h_ * 2) / 3;
    drawIcon();

    // Mode name (bottom 1/3)
    int text_y = y_ + icon_area_h + (h_ - icon_area_h) / 2;
    gfx->setTextColor(COLOR_MODE_PANEL_TEXT);
    gfx->setTextSize(1);
    gfx->setTextDatum(middle_center);
    gfx->drawString(getModeName(), x_ + w_ / 2, text_y);
}

void ModePanel::setMode(OperationMode mode) {
    current_mode = mode;
}

const char* ModePanel::getModeName() const {
    int mode_index = (int)current_mode;
    if (mode_index >= 0 && mode_index < (int)OperationMode::MODE_COUNT) {
        return MODE_CONFIGS[mode_index].name;
    }
    return "UNKNOWN";
}

void ModePanel::drawIcon() {
    // Get icon data for current mode
    const uint8_t* icon_data = getModeIcon((int)current_mode);

    if (!icon_data) {
        // Fallback if icon not found
        return;
    }

    // Center the 64x64 icon in the available space
    int icon_area_h = (h_ * 2) / 3;
    int icon_x = x_ + (w_ - MODE_ICON_SIZE) / 2;
    int icon_y = y_ + (icon_area_h - MODE_ICON_SIZE) / 2;

    // Draw monochrome bitmap
    // LovyanGFX drawBitmap: drawBitmap(x, y, bitmap_data, width, height, color)
    gfx->drawBitmap(icon_x, icon_y, icon_data, MODE_ICON_SIZE, MODE_ICON_SIZE, COLOR_MODE_ICON_FG);
}

// ============================================================================
// LevelDisplay Implementation
// ============================================================================
LevelDisplay::LevelDisplay()
    : gfx(nullptr), x_(0), y_(0), w_(0), h_(0),
      pitch_angle(0.0f), roll_angle(0.0f) {}

void LevelDisplay::init(LGFX* display, int x, int y, int w, int h) {
    gfx = display;
    x_ = x;
    y_ = y;
    w_ = w;
    h_ = h;
}

void LevelDisplay::draw() {
    if (!gfx) return;
    drawPlaceholder();
}

void LevelDisplay::setAngle(float pitch, float roll) {
    pitch_angle = pitch;
    roll_angle = roll;
}

void LevelDisplay::clear() {
    if (!gfx) return;
    gfx->fillRect(x_, y_, w_, h_, COLOR_LEVEL_BG);
}

void LevelDisplay::drawPlaceholder() {
    // Background
    gfx->fillRect(x_, y_, w_, h_, COLOR_LEVEL_BG);
    gfx->drawRect(x_, y_, w_, h_, COLOR_LEVEL_BORDER);

    // Draw crosshair center
    int cx = x_ + w_ / 2;
    int cy = y_ + h_ / 2;
    int crosshair_len = 10;

    gfx->drawLine(cx - crosshair_len, cy, cx + crosshair_len, cy, COLOR_LEVEL_CROSSHAIR);
    gfx->drawLine(cx, cy - crosshair_len, cx, cy + crosshair_len, COLOR_LEVEL_CROSSHAIR);

    // Draw bubble (placeholder - will use sensor data)
    int bubble_radius = 8;
    int bubble_x = cx + (int)(roll_angle * 20);   // Scale for visualization
    int bubble_y = cy + (int)(pitch_angle * 20);  // Scale for visualization
    gfx->fillCircle(bubble_x, bubble_y, bubble_radius, COLOR_LEVEL_BUBBLE_BG);
    gfx->drawCircle(bubble_x, bubble_y, bubble_radius, COLOR_LEVEL_BUBBLE_FG);

    // Text
    gfx->setTextColor(COLOR_LEVEL_TEXT);
    gfx->setTextSize(1);
    gfx->setTextDatum(bottom_center);
    gfx->drawString("LEVEL", cx, y_ + h_ - 4);
}

// ============================================================================
// ButtonPanel Implementation
// ============================================================================
ButtonPanel::ButtonPanel()
    : gfx(nullptr), x_(0), y_(0), w_(0), h_(0), button_height(0),
      current_mode(OperationMode::UP_DOWN) {
    for (int i = 0; i < 3; i++) {
        buttons[i].label = "";
        buttons[i].is_pressed = false;
    }
}

void ButtonPanel::init(LGFX* display, int x, int y, int w, int h) {
    gfx = display;
    x_ = x;
    y_ = y;
    w_ = w;
    h_ = h;
    button_height = h / 3;
}

void ButtonPanel::draw() {
    if (!gfx) return;
    for (int i = 0; i < 3; i++) {
        drawButton(i);
    }
}

void ButtonPanel::setButtonLabels(const char* up, const char* mode, const char* down) {
    buttons[0].label = up;
    buttons[1].label = mode;
    buttons[2].label = down;
}

void ButtonPanel::setButtonState(int button_index, bool pressed) {
    if (button_index >= 0 && button_index < 3) {
        buttons[button_index].is_pressed = pressed;
    }
}

void ButtonPanel::updateForMode(OperationMode mode) {
    current_mode = mode;
    // Labels kept for MODE button (middle button still uses text)
    buttons[1].label = "MODE";
}

void ButtonPanel::drawButton(int index) {
    if (index < 0 || index >= 3) return;

    int button_y = y_ + (index * button_height);

    // Button 0 (top) = UP button with icon
    // Button 1 (middle) = MODE button with text
    // Button 2 (bottom) = DOWN button with icon

    if (index == 0) {
        // Up button - use icon
        const uint8_t* icon = getButtonUpIcon((int)current_mode);
        if (icon) {
            drawIconButton(x_, button_y, w_, button_height,
                          buttons[index].is_pressed, icon, BUTTON_ICON_SIZE);
        } else {
            // Fallback to text if icon not available
            drawButtonRect(x_, button_y, w_, button_height,
                          buttons[index].is_pressed, "UP");
        }
    }
    else if (index == 1) {
        // Mode button - use icon
        const uint8_t* icon = getButtonModeIcon((int)current_mode);
        if (icon) {
            drawIconButton(x_, button_y, w_, button_height,
                          buttons[index].is_pressed, icon, BUTTON_ICON_SIZE);
        } else {
            // Fallback to text if icon not available
            drawButtonRect(x_, button_y, w_, button_height,
                          buttons[index].is_pressed, "MODE");
        }
    }
    else if (index == 2) {
        // Down button - use icon
        const uint8_t* icon = getButtonDownIcon((int)current_mode);
        if (icon) {
            drawIconButton(x_, button_y, w_, button_height,
                          buttons[index].is_pressed, icon, BUTTON_ICON_SIZE);
        } else {
            // Fallback to text if icon not available
            drawButtonRect(x_, button_y, w_, button_height,
                          buttons[index].is_pressed, "DN");
        }
    }
}

void ButtonPanel::drawButtonRect(int x, int y, int w, int h, bool inverted, const char* label) {
    uint16_t fill_color = inverted ? COLOR_BUTTON_PRESSED : COLOR_BUTTON_NORMAL;
    uint16_t outline_color = COLOR_BUTTON_BORDER;
    uint16_t text_color = inverted ? COLOR_BUTTON_TEXT_INV : COLOR_BUTTON_TEXT;

    // Draw button
    gfx->fillRect(x, y, w, h, fill_color);
    gfx->drawRect(x, y, w, h, outline_color);

    // Draw label
    gfx->setTextColor(text_color);
    gfx->setTextDatum(middle_center);
    gfx->setTextSize(1);
    gfx->drawString(label, x + w / 2, y + h / 2);
}

void ButtonPanel::drawIconButton(int x, int y, int w, int h, bool pressed, const uint8_t* icon_data, int icon_size) {
    uint16_t fill_color = pressed ? COLOR_BUTTON_PRESSED : COLOR_BUTTON_NORMAL;
    uint16_t outline_color = COLOR_BUTTON_BORDER;
    uint16_t icon_color = pressed ? COLOR_BUTTON_TEXT_INV : COLOR_BUTTON_TEXT;

    // Draw button background
    gfx->fillRect(x, y, w, h, fill_color);
    gfx->drawRect(x, y, w, h, outline_color);

    // Center the icon in the button
    int icon_x = x + (w - icon_size) / 2;
    int icon_y = y + (h - icon_size) / 2;

    // Draw the monochrome bitmap icon
    gfx->drawBitmap(icon_x, icon_y, icon_data, icon_size, icon_size, icon_color);
}
