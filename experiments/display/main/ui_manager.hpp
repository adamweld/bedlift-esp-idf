#pragma once

#include "lgfx_config.hpp"
#include "ui.hpp"
#include "pins.hpp"

// ============================================================================
// UIManager - Coordinates all UI panels and manages screen layout
// ============================================================================
class UIManager {
public:
    UIManager();

    // Initialize display and all UI panels
    void init(LGFX* display);

    // Refresh entire UI
    void refresh();

    // Refresh individual panels
    void refreshStatusBar();
    void refreshModePanel();
    void refreshLevelDisplay();
    void refreshButtonPanel();

    // Mode management
    void setMode(OperationMode mode);
    OperationMode getMode() const;
    void cycleMode();  // Move to next mode

    // Button state updates
    void setButtonState(int button_index, bool pressed);

    // Status updates
    void setBatteryLevel(int percent);
    void setConnectionStatus(bool connected);
    void setStatusMessage(const char* msg);

    // Level display updates
    void setLevelAngle(float pitch, float roll);

    // Access to panels (for advanced use)
    StatusBar& getStatusBar() { return status_bar; }
    ModePanel& getModePanel() { return mode_panel; }
    LevelDisplay& getLevelDisplay() { return level_display; }
    ButtonPanel& getButtonPanel() { return button_panel; }

private:
    LGFX* gfx;

    // UI Panel instances
    StatusBar status_bar;
    ModePanel mode_panel;
    LevelDisplay level_display;
    ButtonPanel button_panel;

    // Layout calculations
    struct Layout {
        int status_bar_x, status_bar_y, status_bar_w, status_bar_h;
        int mode_panel_x, mode_panel_y, mode_panel_w, mode_panel_h;
        int level_display_x, level_display_y, level_display_w, level_display_h;
        int button_panel_x, button_panel_y, button_panel_w, button_panel_h;
    };
    Layout layout;

    void calculateLayout();
};
