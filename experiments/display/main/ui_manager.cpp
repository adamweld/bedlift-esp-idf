#include "ui_manager.hpp"
#include "config.hpp"
#include "esp_log.h"

static const char* TAG = "UIManager";

UIManager::UIManager() : gfx(nullptr), dev_flag(false) {}

void UIManager::init(LGFX* display, bool dev_flag_param) {
    gfx = display;
    dev_flag = dev_flag_param;
    calculateLayout();

    // Initialize monitor state with dev_flag
    monitors.dev_mode = dev_flag_param;

    // Initialize all panels with their layout positions
    status_bar.init(gfx, layout.status_bar_x, layout.status_bar_y,
                    layout.status_bar_w, layout.status_bar_h, &monitors);

    mode_panel.init(gfx, layout.mode_panel_x, layout.mode_panel_y,
                    layout.mode_panel_w, layout.mode_panel_h);

    level_display.init(gfx, layout.level_display_x, layout.level_display_y,
                       layout.level_display_w, layout.level_display_h);

    button_panel.init(gfx, layout.button_panel_x, layout.button_panel_y,
                      layout.button_panel_w, layout.button_panel_h);

    // Set initial mode
    setMode(OperationMode::UP_DOWN);

    ESP_LOGI(TAG, "UI Manager initialized with layout:");
    ESP_LOGI(TAG, "  Status bar: (%d,%d) %dx%d", layout.status_bar_x, layout.status_bar_y,
             layout.status_bar_w, layout.status_bar_h);
    ESP_LOGI(TAG, "  Mode panel: (%d,%d) %dx%d", layout.mode_panel_x, layout.mode_panel_y,
             layout.mode_panel_w, layout.mode_panel_h);
    ESP_LOGI(TAG, "  Level display: (%d,%d) %dx%d", layout.level_display_x, layout.level_display_y,
             layout.level_display_w, layout.level_display_h);
    ESP_LOGI(TAG, "  Button panel: (%d,%d) %dx%d", layout.button_panel_x, layout.button_panel_y,
             layout.button_panel_w, layout.button_panel_h);
}

void UIManager::refresh() {
    if (!gfx) return;

    // Clear screen
    gfx->fillScreen(COLOR_BLACK);

    // Draw all panels
    status_bar.draw();
    mode_panel.draw();
    level_display.draw();
    button_panel.draw();
}

void UIManager::refreshStatusBar() {
    if (!gfx) return;
    status_bar.draw();
}

void UIManager::refreshModePanel() {
    if (!gfx) return;
    mode_panel.draw();
}

void UIManager::refreshLevelDisplay() {
    if (!gfx) return;
    level_display.draw();
}

void UIManager::refreshButtonPanel() {
    if (!gfx) return;
    button_panel.draw();
}

void UIManager::setMode(OperationMode mode) {
    mode_panel.setMode(mode);
    button_panel.updateForMode(mode);
    ESP_LOGI(TAG, "Mode changed to: %s", mode_panel.getModeName());
}

OperationMode UIManager::getMode() const {
    return mode_panel.getMode();
}

void UIManager::cycleMode() {
    int current = (int)getMode();
    int next_mode = current;

    // Find next available mode (skip dev_only modes if dev_flag is false)
    do {
        next_mode = (next_mode + 1) % (int)OperationMode::MODE_COUNT;

        // If we've cycled back to current mode, we're stuck (shouldn't happen)
        if (next_mode == current) {
            break;
        }

        // Check if this mode is available
        if (!MODE_CONFIGS[next_mode].dev_only || dev_flag) {
            break;  // Mode is available
        }
    } while (true);

    setMode((OperationMode)next_mode);
}

void UIManager::setButtonState(int button_index, bool pressed) {
    button_panel.setButtonState(button_index, pressed);
}

void UIManager::setLevelAngle(float pitch, float roll) {
    level_display.setAngle(pitch, roll);
}

void UIManager::calculateLayout() {
    // Button panel on right - full height
    layout.button_panel_x = SCREEN_WIDTH - BUTTON_PANEL_WIDTH;
    layout.button_panel_y = 0;
    layout.button_panel_w = BUTTON_PANEL_WIDTH;
    layout.button_panel_h = SCREEN_HEIGHT;

    // Status bar at top left of buttons
    layout.status_bar_x = 0;
    layout.status_bar_y = 0;
    layout.status_bar_w = STATUS_BAR_WIDTH;
    layout.status_bar_h = STATUS_BAR_HEIGHT;

    // Mode panel on left below status bar
    layout.mode_panel_x = 0;
    layout.mode_panel_y = STATUS_BAR_HEIGHT;
    layout.mode_panel_w = MODE_PANEL_WIDTH;
    layout.mode_panel_h = MAIN_CONTENT_HEIGHT;

    // Level display in center below status bar
    layout.level_display_x = MODE_PANEL_WIDTH;
    layout.level_display_y = STATUS_BAR_HEIGHT;
    layout.level_display_w = LEVEL_DISPLAY_WIDTH;
    layout.level_display_h = MAIN_CONTENT_HEIGHT;
}
