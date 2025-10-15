#include "ui.hpp"
#include "config.hpp"
#include "assets/icons.hpp"
#include <cstring>

// ============================================================================
// StatusBar Implementation
// ============================================================================
StatusBar::StatusBar()
    : gfx(nullptr), x_(0), y_(0), w_(0), h_(0),
      battery_level(100), is_connected(false) {
    message[0] = '\0';
}

void StatusBar::init(LGFX* display, int x, int y, int w, int h) {
    gfx = display;
    x_ = x;
    y_ = y;
    w_ = w;
    h_ = h;
}

void StatusBar::draw() {
    if (!gfx) return;

    // Background
    gfx->fillRect(x_, y_, w_, h_, COLOR_STATUS_BAR_BG);

    // Battery indicator (right side)
    int batt_w = 30;
    int batt_h = h_ - 4;
    int batt_x = x_ + w_ - batt_w - 4;
    int batt_y = y_ + 2;

    gfx->drawRect(batt_x, batt_y, batt_w, batt_h, COLOR_STATUS_BAR_TEXT);
    int fill_w = (batt_w - 4) * battery_level / 100;
    gfx->fillRect(batt_x + 2, batt_y + 2, fill_w, batt_h - 4, COLOR_STATUS_BAR_ACCENT);

    // Connection status (left side)
    gfx->setTextColor(COLOR_STATUS_BAR_TEXT);
    gfx->setTextSize(1);
    gfx->setTextDatum(middle_left);
    gfx->drawString(is_connected ? "CONN" : "----", x_ + 4, y_ + h_ / 2);

    // Message (center)
    if (message[0] != '\0') {
        gfx->setTextDatum(middle_center);
        gfx->drawString(message, x_ + w_ / 2, y_ + h_ / 2);
    }
}

void StatusBar::setBatteryLevel(int percent) {
    battery_level = percent > 100 ? 100 : (percent < 0 ? 0 : percent);
}

void StatusBar::setConnectionStatus(bool connected) {
    is_connected = connected;
}

void StatusBar::setMessage(const char* msg) {
    if (msg) {
        strncpy(message, msg, sizeof(message) - 1);
        message[sizeof(message) - 1] = '\0';
    } else {
        message[0] = '\0';
    }
}

// ============================================================================
// ModePanel Implementation
// ============================================================================
ModePanel::ModePanel()
    : gfx(nullptr), x_(0), y_(0), w_(0), h_(0),
      current_mode(OperationMode::MANUAL) {}

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
    switch (current_mode) {
        case OperationMode::MANUAL:   return "MANUAL";
        case OperationMode::AUTO:     return "AUTO";
        case OperationMode::PRESET_1: return "PSET 1";
        case OperationMode::PRESET_2: return "PSET 2";
        case OperationMode::LEVEL:    return "LEVEL";
        default: return "UNKNOWN";
    }
}

void ModePanel::drawIcon() {
    // Draw placeholder icon - will be replaced with actual icons from assets
    int icon_size = w_ / 2;
    int icon_x = x_ + (w_ - icon_size) / 2;
    int icon_y = y_ + icon_size / 2;

    // Simple placeholder: draw a filled circle with mode indicator
    gfx->fillCircle(icon_x + icon_size / 2, icon_y + icon_size / 2, icon_size / 2, COLOR_MODE_ICON_BG);
    gfx->drawCircle(icon_x + icon_size / 2, icon_y + icon_size / 2, icon_size / 2, COLOR_MODE_ICON_FG);

    // Draw mode number in circle
    gfx->setTextColor(COLOR_MODE_ICON_FG);
    gfx->setTextSize(2);
    gfx->setTextDatum(middle_center);
    char mode_num[2] = {(char)('0' + (int)current_mode), '\0'};
    gfx->drawString(mode_num, icon_x + icon_size / 2, icon_y + icon_size / 2);
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
    : gfx(nullptr), x_(0), y_(0), w_(0), h_(0), button_height(0) {
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
    switch (mode) {
        case OperationMode::MANUAL:
            setButtonLabels("Up", "MODE", "Down");
            break;
        case OperationMode::AUTO:
            setButtonLabels("+", "MODE", "-");
            break;
        case OperationMode::PRESET_1:
            setButtonLabels("Save", "MODE", "Load");
            break;
        case OperationMode::PRESET_2:
            setButtonLabels("Save", "MODE", "Load");
            break;
        case OperationMode::LEVEL:
            setButtonLabels("Cal+", "MODE", "Cal-");
            break;
        default:
            setButtonLabels("?", "MODE", "?");
            break;
    }
}

void ButtonPanel::drawButton(int index) {
    if (index < 0 || index >= 3) return;

    int button_y = y_ + (index * button_height);
    drawButtonRect(x_, button_y, w_, button_height,
                   buttons[index].is_pressed, buttons[index].label);
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
