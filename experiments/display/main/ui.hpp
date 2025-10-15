#pragma once

#include "lgfx_config.hpp"
#include "pins.hpp"

// Forward declarations
class LGFX;

// ============================================================================
// StatusBar - Top status panel
// ============================================================================
class StatusBar {
public:
    StatusBar();
    void init(LGFX* display, int x, int y, int w, int h);
    void draw();
    void setBatteryLevel(int percent);
    void setConnectionStatus(bool connected);
    void setMessage(const char* msg);

private:
    LGFX* gfx;
    int x_, y_, w_, h_;
    int battery_level;
    bool is_connected;
    char message[32];
};

// ============================================================================
// ModePanel - Left panel showing current mode with icon
// ============================================================================
enum class OperationMode {
    MANUAL = 0,
    AUTO,
    PRESET_1,
    PRESET_2,
    LEVEL,
    MODE_COUNT  // For cycling
};

class ModePanel {
public:
    ModePanel();
    void init(LGFX* display, int x, int y, int w, int h);
    void draw();
    void setMode(OperationMode mode);
    OperationMode getMode() const { return current_mode; }
    const char* getModeName() const;

private:
    LGFX* gfx;
    int x_, y_, w_, h_;
    OperationMode current_mode;
    void drawIcon();
};

// ============================================================================
// LevelDisplay - Center panel with bubble level visualization
// ============================================================================
class LevelDisplay {
public:
    LevelDisplay();
    void init(LGFX* display, int x, int y, int w, int h);
    void draw();
    void setAngle(float pitch, float roll);
    void clear();

private:
    LGFX* gfx;
    int x_, y_, w_, h_;
    float pitch_angle;
    float roll_angle;
    void drawPlaceholder();  // Placeholder until sensor integration
};

// ============================================================================
// ButtonPanel - Right panel with 3 context-sensitive buttons
// ============================================================================
struct ButtonInfo {
    const char* label;
    bool is_pressed;
};

class ButtonPanel {
public:
    ButtonPanel();
    void init(LGFX* display, int x, int y, int w, int h);
    void draw();
    void setButtonLabels(const char* up, const char* mode, const char* down);
    void setButtonState(int button_index, bool pressed);  // 0=up, 1=mode, 2=down
    void updateForMode(OperationMode mode);

private:
    LGFX* gfx;
    int x_, y_, w_, h_;
    int button_height;
    ButtonInfo buttons[3];  // [0]=up, [1]=mode, [2]=down

    void drawButton(int index);
    void drawButtonRect(int x, int y, int w, int h, bool inverted, const char* label);
};
