#include <cstdio>
#include <cmath>
#include "ui_panels.hpp"
#include "icons_legacy.hpp"   // 1bpp bitmaps from the display experiment

struct ModeMeta {
    const char *name;
    const uint8_t *icon;      // 64x64 1bpp
};

// Order matches app_mode_e. Motor icons: corner-box rotations = FL/FR/RL/RR.
static const ModeMeta k_modes[APP_MODE_COUNT] = {
    { "LIFT",  icon_mode_arrows_up_down },
    { "PITCH", icon_mode_view_360_arrow_r90 },
    { "ROLL",  icon_mode_rotate_360_r90 },
    { "TWIST", icon_mode_stretching },
    { "M1 FL", icon_mode_box_align_bottom_right },
    { "M2 FR", icon_mode_box_align_bottom_right_r90 },
    { "M3 RL", icon_mode_box_align_bottom_right_r270 },
    { "M4 RR", icon_mode_box_align_bottom_right_r180 },
};

// palette
static inline uint16_t C(LGFX_Sprite &fb, uint8_t r, uint8_t g, uint8_t b)
{
    return fb.color565(r, g, b);
}

#define COL_BG(fb)      C(fb, 12, 14, 13)
#define COL_PANEL(fb)   C(fb, 24, 28, 26)
#define COL_LINE(fb)    C(fb, 60, 68, 64)
#define COL_TEXT(fb)    C(fb, 235, 238, 235)
#define COL_DIM(fb)     C(fb, 140, 148, 144)
#define COL_OK(fb)      C(fb, 110, 215, 140)
#define COL_WARN(fb)    C(fb, 235, 180, 80)
#define COL_ERR(fb)     C(fb, 240, 95, 75)
#define COL_ACCENT(fb)  C(fb, 100, 170, 235)

static const char *motion_name(motion_state_e m)
{
    switch (m) {
        case MOTION_IDLE: return "idle";
        case MOTION_POWER_UP: return "power up";
        case MOTION_READY: return "ready";
        case MOTION_PAWL_UNLOAD: return "unload";
        case MOTION_UNLOCK: return "unlock";
        case MOTION_MOVING_UP: return "raising";
        case MOTION_MOVING_DOWN: return "lowering";
        case MOTION_LEVELING: return "leveling";
        case MOTION_RAMP_DOWN: return "stopping";
        case MOTION_SETTLE: return "settling";
        case MOTION_POWER_DOWN: return "power dn";
        case MOTION_FAULT: return "FAULT";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Status bar: 6 slots across the top (motors, locks, tilt, LiPo, VBUS, safety)
// ---------------------------------------------------------------------------
static void draw_status(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    fb.fillRect(0, 0, UI_W, UI_STATUS_H, COL_PANEL(fb));
    fb.drawFastHLine(0, UI_STATUS_H - 1, UI_W, COL_LINE(fb));

    int x = 4;
    const int cy = UI_STATUS_H / 2 - 1;

    // motors: four dots
    for (int i = 0; i < SYS_NUM_MOTORS; i++) {
        const motor_snap_t &m = s.motor[i];
        uint16_t c = !s.motor_ssr_on ? COL_DIM(fb)
                     : (!m.online ? COL_DIM(fb)
                        : (m.faults ? COL_ERR(fb) : COL_OK(fb)));
        fb.fillCircle(x + 4 + (i % 2) * 10, cy - 5 + (i / 2) * 10, 3, c);
    }
    x += 28;

    // locks: padlock body + shackle; open (green outline) when energized
    {
        uint16_t c = s.lock_energized ? COL_WARN(fb) : COL_OK(fb);
        fb.drawRoundRect(x + 3, cy - 7, 10, 7, 3, c);       // shackle
        if (s.lock_energized) fb.fillRect(x + 3, cy - 4, 6, 4, COL_PANEL(fb));
        fb.fillRoundRect(x + 1, cy - 1, 14, 9, 2, c);       // body
    }
    x += 24;

    // tilt sensors: triangle, colored by validity
    {
        bool ok = s.tilt_front.valid && s.tilt_rear.valid;
        uint16_t c = ok ? COL_OK(fb) : COL_ERR(fb);
        fb.drawTriangle(x + 7, cy - 7, x, cy + 7, x + 14, cy + 7, c);
        fb.fillCircle(x + 7, cy + 2, 1, c);
    }
    x += 22;

    // LiPo battery glyph + %
    {
        uint16_t c = s.lipo_soc > 0.3f ? COL_OK(fb)
                     : (s.lipo_soc > 0.15f ? COL_WARN(fb) : COL_ERR(fb));
        fb.drawRect(x, cy - 5, 18, 10, c);
        fb.fillRect(x + 18, cy - 2, 2, 4, c);
        int w = (int)(14.0f * (s.lipo_soc < 0 ? 0 : s.lipo_soc > 1 ? 1 : s.lipo_soc));
        fb.fillRect(x + 2, cy - 3, w, 6, c);
    }
    x += 26;

    // VBUS: last-read value persists; dimmed when the SSR is off (stale)
    {
        float v = 0;
        for (int i = 0; i < SYS_NUM_MOTORS; i++)
            if (s.motor[i].vbus_v > v) v = s.motor[i].vbus_v;
        fb.setTextSize(1);
        fb.setTextDatum(lgfx::middle_left);
        if (v > 1.0f) {
            uint16_t c = v < 21.0f ? (v < 19.5f ? COL_ERR(fb) : COL_WARN(fb))
                                   : COL_TEXT(fb);
            if (!s.motor_ssr_on) c = COL_DIM(fb);   // stale but still shown
            fb.setTextColor(c);
            char buf[8];
            snprintf(buf, sizeof(buf), "%4.1fV", v);
            fb.drawString(buf, x, cy);
        } else {
            fb.setTextColor(COL_DIM(fb));
            fb.drawString("--.-V", x, cy);
        }
    }
    x += 36;

    // safety: warning triangle, only when flags latched (blinks via now_us)
    if (s.safety_flags) {
        bool on = ((s.now_us / 400000) & 1) != 0;
        uint16_t c = on ? COL_ERR(fb) : COL_WARN(fb);
        fb.fillTriangle(x + 8, cy - 8, x, cy + 7, x + 16, cy + 7, c);
        fb.setTextColor(COL_PANEL(fb));
        fb.setTextDatum(lgfx::middle_center);
        fb.drawString("!", x + 8, cy + 1);
    }
}

// ---------------------------------------------------------------------------
// Mode panel (left column): mode glyph + name + motion state
// ---------------------------------------------------------------------------
static void draw_mode(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    const int px = 0, py = UI_STATUS_H;
    const int pw = UI_MODE_W, ph = UI_H - UI_STATUS_H;
    fb.drawFastVLine(pw - 1, py, ph, COL_LINE(fb));

    const int cx = px + pw / 2;
    const ModeMeta &meta = k_modes[s.mode < APP_MODE_COUNT ? s.mode : 0];

    fb.drawBitmap(cx - 32, py + 2, meta.icon, 64, 64, COL_ACCENT(fb));

    fb.setTextSize(1);
    fb.setTextDatum(lgfx::middle_center);
    fb.setTextColor(COL_TEXT(fb));
    fb.drawString(meta.name, cx, py + 74);
    fb.setTextColor(s.motion == MOTION_FAULT ? COL_ERR(fb) : COL_DIM(fb));
    fb.drawString(motion_name(s.motion), cx, py + 86);

    // solenoid budget bar along the bottom of the panel
    int bw = pw - 12;
    fb.drawRect(px + 6, UI_H - 12, bw, 6, COL_LINE(fb));
    float f = s.sol_budget_frac < 0 ? 0 : s.sol_budget_frac > 1 ? 1 : s.sol_budget_frac;
    uint16_t bc = f > 0.4f ? COL_OK(fb) : (f > 0.15f ? COL_WARN(fb) : COL_ERR(fb));
    fb.fillRect(px + 7, UI_H - 11, (int)((bw - 2) * f), 4, bc);
    if (s.sol_cooldown_s > 0) {
        fb.setTextColor(COL_WARN(fb));
        char buf[12];
        snprintf(buf, sizeof(buf), "%lus", (unsigned long)s.sol_cooldown_s);
        fb.drawString(buf, cx, UI_H - 20);
    }
}

// ---------------------------------------------------------------------------
// Level display (center): dual bubble, front + rear independent
// ---------------------------------------------------------------------------
static void draw_level(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    const int px = UI_MODE_W, py = UI_STATUS_H;
    const int pw = UI_W - UI_MODE_W - UI_BTN_W, ph = UI_H - UI_STATUS_H;
    const int cx = px + pw / 2, cy = py + ph / 2;

    float rack = fabsf(s.tilt_front.roll_deg - s.tilt_rear.roll_deg);
    if (rack > RACK_WARN_DEG) {
        uint16_t tint = rack > RACK_TRIP_DEG ? C(fb, 60, 22, 18) : C(fb, 55, 44, 16);
        fb.fillRect(px, py, pw, ph, tint);
    }

    // crosshair + rings: LEVEL_RING_DEG reference, LEVEL_RANGE_DEG full scale
    const float k = (ph / 2 - 6) / LEVEL_RANGE_DEG;   // px per degree
    fb.drawFastHLine(px + 6, cy, pw - 12, COL_LINE(fb));
    fb.drawFastVLine(cx, py + 6, ph - 12, COL_LINE(fb));
    fb.drawCircle(cx, cy, (int)(LEVEL_RING_DEG * k), COL_LINE(fb));
    fb.drawCircle(cx, cy, (int)(LEVEL_RANGE_DEG * k), COL_LINE(fb));

    auto bubble = [&](const tilt_snap_t &t, uint16_t col, bool fill) {
        if (!t.valid) return;
        float bx = t.roll_deg * k, by = t.pitch_deg * k;
        float lim = (LEVEL_RANGE_DEG + 2.0f) * k;
        if (bx > lim) bx = lim;
        if (bx < -lim) bx = -lim;
        if (by > lim) by = lim;
        if (by < -lim) by = -lim;
        if (fill) fb.fillCircle(cx + (int)bx, cy + (int)by, 5, col);
        else fb.drawCircle(cx + (int)bx, cy + (int)by, 5, col);
        fb.drawCircle(cx + (int)bx, cy + (int)by, 5, col);
    };
    bubble(s.tilt_front, COL_ACCENT(fb), true);          // front: filled
    bubble(s.tilt_rear, C(fb, 235, 160, 90), false);     // rear: outline

    fb.setTextSize(1);
    fb.setTextDatum(lgfx::top_left);
    fb.setTextColor(COL_DIM(fb));
    fb.setCursor(px + 4, py + 3);
    fb.printf("F%+4.1f", s.tilt_front.roll_deg);
    fb.setCursor(px + 4, py + 13);
    fb.printf("R%+4.1f", s.tilt_rear.roll_deg);
    fb.setTextDatum(lgfx::top_right);
    char buf[10];
    snprintf(buf, sizeof(buf), "P%+4.1f",
             0.5f * (s.tilt_front.pitch_deg + s.tilt_rear.pitch_deg));
    fb.drawString(buf, px + pw - 4, py + 3);
}

// Fault detail replaces the level display when latched
static void draw_fault_detail(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    const int px = UI_MODE_W, py = UI_STATUS_H;
    const int pw = UI_W - UI_MODE_W - UI_BTN_W, ph = UI_H - UI_STATUS_H;
    fb.fillRect(px, py, pw, ph, C(fb, 45, 18, 15));
    fb.setTextSize(1);
    fb.setTextDatum(lgfx::top_left);
    fb.setTextColor(COL_ERR(fb));
    fb.setCursor(px + 6, py + 4);
    fb.print("SAFETY FAULT");

    static const struct { uint32_t bit; const char *name; } names[] = {
        { SAFE_F_UNDERVOLT, "24V undervolt" }, { SAFE_F_MOTOR_FAULT, "motor fault" },
        { SAFE_F_OVERSPEED, "overspeed" }, { SAFE_F_TELEM_LOSS, "telemetry lost" },
        { SAFE_F_THERMAL, "motor thermal" }, { SAFE_F_RACKING, "frame racking" },
        { SAFE_F_DESYNC, "corner desync" }, { SAFE_F_SOL_DUTY, "solenoid duty" },
        { SAFE_F_TILT_FAIL, "tilt sensor" }, { SAFE_F_OVERTRAVEL, "overtravel" },
    };
    int y = py + 18;
    fb.setTextColor(COL_TEXT(fb));
    for (auto &n : names) {
        if ((s.safety_flags & n.bit) && y < py + ph - 20) {
            fb.setCursor(px + 10, y);
            fb.print(n.name);
            y += 10;
        }
    }
    fb.setTextColor(COL_WARN(fb));
    fb.setCursor(px + 6, py + ph - 12);
    fb.print("hold \x07 to reset");
}

// ---------------------------------------------------------------------------
// Button panel (right column): three cells with mode-dependent hints
// ---------------------------------------------------------------------------
static void draw_buttons(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    const int px = UI_W - UI_BTN_W, py = 0;
    const int ph = UI_H, cellh = ph / 3;
    fb.drawFastVLine(px, 0, UI_H, COL_LINE(fb));

    const char *hints[3];
    if (s.motion == MOTION_FAULT) {
        hints[0] = ""; hints[1] = "RESET"; hints[2] = "";
    } else if (s.mode == APP_MODE_LIFT) {
        hints[0] = "UP"; hints[1] = "LEVEL"; hints[2] = "DOWN";
    } else {
        static const char *axis_up[APP_MODE_COUNT] =
            { "", "NOSE+", "LEFT+", "TW+", "M1+", "M2+", "M3+", "M4+" };
        static const char *axis_dn[APP_MODE_COUNT] =
            { "", "NOSE-", "LEFT-", "TW-", "M1-", "M2-", "M3-", "M4-" };
        hints[0] = axis_up[s.mode]; hints[1] = "NEXT"; hints[2] = axis_dn[s.mode];
    }

    for (int i = 0; i < 3; i++) {
        int cy0 = py + i * cellh;
        bool pressed = (s.btn_pressed_mask >> i) & 1;
        uint16_t bg = pressed ? COL_ACCENT(fb) : COL_PANEL(fb);
        uint16_t fg = pressed ? COL_PANEL(fb) : COL_TEXT(fb);
        fb.fillRoundRect(px + 3, cy0 + 3, UI_BTN_W - 7, cellh - 6, 4, bg);
        fb.drawRoundRect(px + 3, cy0 + 3, UI_BTN_W - 7, cellh - 6, 4, COL_LINE(fb));

        int cx = px + UI_BTN_W / 2 - 1, cc = cy0 + cellh / 2;
        // glyphs: up arrow / dot / down arrow, plus hint text
        if (i == 0 && s.motion != MOTION_FAULT)
            fb.fillTriangle(cx, cc - 9, cx - 8, cc + 1, cx + 8, cc + 1, fg);
        else if (i == 2 && s.motion != MOTION_FAULT)
            fb.fillTriangle(cx, cc + 9, cx - 8, cc - 1, cx + 8, cc - 1, fg);
        else if (i == 1)
            fb.fillCircle(cx, cc - 5, 3, fg);

        fb.setTextSize(1);
        fb.setTextDatum(lgfx::middle_center);
        fb.setTextColor(fg);
        fb.drawString(hints[i], cx, cc + (i == 1 ? 7 : 12));
    }
}

// Debug display (triple-click center): all motors pos / velocity / force
static void draw_debug_table(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    const int px = UI_MODE_W, py = UI_STATUS_H;
    const int pw = UI_W - UI_MODE_W - UI_BTN_W;

    fb.setTextSize(1);
    fb.setTextDatum(lgfx::top_left);
    fb.setTextColor(COL_DIM(fb));
    fb.setCursor(px + 6, py + 4);
    fb.printf("    pos     vel    F");

    for (int i = 0; i < SYS_NUM_MOTORS; i++) {
        const motor_snap_t &m = s.motor[i];
        int y = py + 18 + i * 13;
        bool stale = !m.online;
        fb.setTextColor(stale ? COL_DIM(fb)
                              : (m.faults ? COL_ERR(fb) : COL_TEXT(fb)));
        fb.setCursor(px + 6, y);
        fb.printf("M%d %+7.2f %+6.2f %+5.1f", i + 1,
                  m.theta_rad, m.vel_rad_s, m.torque_nm);
    }

    fb.setTextColor(COL_DIM(fb));
    fb.setCursor(px + 6, py + 18 + SYS_NUM_MOTORS * 13 + 4);
    fb.printf("rad     rad/s   Nm");
}

void ui_render(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    fb.fillScreen(COL_BG(fb));
    draw_status(fb, s);
    draw_mode(fb, s);
    // full fault screen only for a latched FAULT; warnings keep the level
    // display (racking tint + blinking status triangle carry the message)
    if (s.motion == MOTION_FAULT)
        draw_fault_detail(fb, s);
    else if (s.debug_screen)
        draw_debug_table(fb, s);
    else
        draw_level(fb, s);
    draw_buttons(fb, s);
}
