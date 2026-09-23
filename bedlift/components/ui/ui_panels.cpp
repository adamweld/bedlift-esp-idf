#include <cstdio>
#include <cmath>
#include "ui_panels.hpp"
#include "ui_icons.hpp"

struct ModeMeta {
    const char *name;
    const char *icon;         // name in assets/icons/icons.txt
    int rot;                  // clockwise quarter turns
};

// Order matches app_mode_e. Motor icons: corner-box rotations = FL/FR/RL/RR.
static const ModeMeta k_modes[APP_MODE_COUNT] = {
    { "LIFT",    "arrows-up-down",         0 },
    { "UP/DOWN", "arrows-up-down",         0 },   // raw up/down, no feedback
    { "PITCH",   "view-360-arrow",         3 },
    { "ROLL",    "rotate-360",             3 },
    { "TWIST",   "stretching",             0 },
    { "M1 FL",   "box-align-bottom-right", 0 },
    { "M2 FR",   "box-align-bottom-right", 3 },
    { "M3 RL",   "box-align-bottom-right", 1 },
    { "M4 RR",   "box-align-bottom-right", 2 },
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

// At rest (IDLE, power off) the button glyphs step back so the screen is
// calm to look at.
static inline bool calm(const sys_snapshot_t &s) { return s.motion == MOTION_IDLE; }

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
// Status icons: a fixed row along the bottom of the mode panel, one slot
// each so nothing shifts when an icon appears.
// ---------------------------------------------------------------------------
static void draw_status_icons(LGFX_Sprite &fb, const sys_snapshot_t &s, int cy)
{
    const int slot[3] = { 12, 33, 54 };

    // motor power: bolt. dim = SSR off, green = on and all motors online,
    // amber = on but some offline, red = any motor fault or 24V undervolt
    // (there is no pack voltage sense; UV is fault-bit only)
    {
        int online = 0; bool fault = false;
        for (int i = 0; i < SYS_NUM_MOTORS; i++) {
            online += s.motor[i].online;
            fault |= s.motor[i].faults != 0;
        }
        fault |= (s.safety_flags & SAFE_F_UNDERVOLT) != 0;
        uint16_t c = fault ? COL_ERR(fb)
                     : !s.motor_ssr_on ? COL_DIM(fb)
                     : online == SYS_NUM_MOTORS ? COL_OK(fb) : COL_WARN(fb);
        ui_icon_draw_centered(fb, "bolt", slot[0], cy, c);
    }

    // locks: open (amber) while the solenoids are energized, green locked
    // while powered, grey once the SSR drops (nothing to report at rest)
    ui_icon_draw_centered(fb, s.lock_energized ? "lock_open" : "lock", slot[1], cy,
                          s.lock_energized ? COL_WARN(fb)
                          : s.motor_ssr_on ? COL_OK(fb) : COL_DIM(fb));

    // safety: warning sign while any flag is set, blinking red/amber
    if (s.safety_flags) {
        bool on = ((s.now_us / 400000) & 1) != 0;
        ui_icon_draw_centered(fb, "warning", slot[2], cy,
                              on ? COL_ERR(fb) : COL_WARN(fb));
    }
}

// ---------------------------------------------------------------------------
// Mode panel (left column): mode glyph, name, motion state, status icons
// ---------------------------------------------------------------------------
static void draw_mode(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    const int pw = UI_MODE_W;
    fb.drawFastVLine(pw - 1, 0, UI_H, COL_LINE(fb));

    const int cx = pw / 2;
    const ModeMeta &meta = k_modes[s.mode < APP_MODE_COUNT ? s.mode : 0];

    // top 14px stay clear for the warning banner
    ui_icon_draw_centered(fb, meta.icon, cx, 14 + 32, COL_ACCENT(fb), meta.rot);

    fb.setTextSize(1);
    fb.setTextDatum(lgfx::middle_center);
    fb.setTextColor(COL_TEXT(fb));
    fb.drawString(meta.name, cx, 86);
    fb.setTextColor(s.motion == MOTION_FAULT ? COL_ERR(fb) : COL_DIM(fb));
    fb.drawString(motion_name(s.motion), cx, 98);

    draw_status_icons(fb, s, UI_H - 14);
}

// ---------------------------------------------------------------------------
// Level display (center): dual bubble, front + rear independent
// ---------------------------------------------------------------------------
static void draw_level(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    const int px = UI_MODE_W, py = 0;
    const int pw = UI_W - UI_MODE_W - UI_BTN_W, ph = UI_H;
    const int cx = px + pw / 2, cy = py + ph / 2;

    // Background: GREEN when the bed is level AND untwisted (both bubbles and
    // the twist inside the target ring) during a motion command only — at rest
    // sensor noise near the threshold made it flicker. Amber/red when racked.
    bool tv = s.tilt_front.valid && s.tilt_rear.valid;
    float pitch = 0.5f * (s.tilt_front.pitch_deg + s.tilt_rear.pitch_deg);
    float rack = fabsf(s.tilt_front.roll_deg - s.tilt_rear.roll_deg);
    bool moving = s.motion == MOTION_MOVING_UP || s.motion == MOTION_MOVING_DOWN ||
                  s.motion == MOTION_LEVELING;
    bool good = moving && tv &&
                fabsf(s.tilt_front.roll_deg) < LEVEL_TARGET_DEG &&
                fabsf(s.tilt_rear.roll_deg)  < LEVEL_TARGET_DEG &&
                fabsf(pitch) < LEVEL_TARGET_DEG &&
                rack < LEVEL_TARGET_DEG;
    if (good) {
        fb.fillRect(px, py, pw, ph, C(fb, 18, 66, 30));            // level: green
    } else if (rack > RACK_WARN_DEG) {
        uint16_t tint = rack > RACK_TRIP_DEG ? C(fb, 60, 22, 18) : C(fb, 55, 44, 16);
        fb.fillRect(px, py, pw, ph, tint);
    }

    // crosshair + rings: inner = target level, outer = full scale.
    // Round on the glass, not in pixels: x radii are UI_PX_ASPECT wider.
    const float A = UI_PX_ASPECT;
    const int half = (pw < ph ? pw : ph) / 2;
    const int Rx = half - 8;                          // outer ring, px
    const int Ry = (int)lroundf(Rx / A);
    const float kx = Rx / LEVEL_RANGE_DEG, ky = Ry / LEVEL_RANGE_DEG;  // px/deg
    auto ring = [&](int x, int y, int r, uint16_t col, bool fill) {
        int rx = (int)lroundf(r * A);
        if (fill) fb.fillEllipse(x, y, rx, r, col);
        fb.drawEllipse(x, y, rx, r, col);
    };
    // crosshair overshoots the outer ring by the same stub on all four sides
    const int stub = 5;
    fb.drawFastHLine(cx - Rx - stub, cy, 2 * (Rx + stub) + 1, COL_LINE(fb));
    fb.drawFastVLine(cx, cy - Ry - stub, 2 * (Ry + stub) + 1, COL_LINE(fb));
    // center ring is a sighting mark, sized just outside a centered bubble
    // (r=5) so it stays visible; the level test itself uses LEVEL_TARGET_DEG
    ring(cx, cy, 8, COL_LINE(fb), false);
    fb.drawEllipse(cx, cy, Rx, Ry, COL_LINE(fb));

    // On-screen a bubble is a disc; beyond the outer ring it becomes an arrow
    // at the rim pointing toward where the bubble actually is.
    auto bubble = [&](const tilt_snap_t &t, uint16_t col, bool fill) {
        if (!t.valid) return;
        float d = sqrtf(t.roll_deg * t.roll_deg + t.pitch_deg * t.pitch_deg);
        if (d <= LEVEL_RANGE_DEG || d < 0.01f) {
            ring(cx + (int)lroundf(t.roll_deg * kx), cy + (int)lroundf(t.pitch_deg * ky),
                 5, col, fill);
            return;
        }
        // direction in degree space == direction on the glass
        float ux = t.roll_deg / d, uy = t.pitch_deg / d;
        float wx = -uy * 6 * A, wy = ux * 6;         // half-width of the base
        int tx = cx + (int)(ux * Rx),            ty = cy + (int)(uy * Ry);
        int mx = cx + (int)(ux * (Rx - 10 * A)), my = cy + (int)(uy * (Ry - 10));
        if (fill)
            fb.fillTriangle(tx, ty, mx + (int)wx, my + (int)wy,
                            mx - (int)wx, my - (int)wy, col);
        else
            fb.drawTriangle(tx, ty, mx + (int)wx, my + (int)wy,
                            mx - (int)wx, my - (int)wy, col);
    };
    // legend in the free bottom-left corner outside the ring
    fb.setTextSize(1);
    fb.setTextDatum(lgfx::middle_left);
    fb.setTextColor(COL_DIM(fb));
    fb.fillCircle(px + 7, UI_H - 18, 3, COL_ACCENT(fb));
    fb.drawString("F", px + 13, UI_H - 18);
    fb.drawCircle(px + 7, UI_H - 7, 3, C(fb, 235, 160, 90));
    fb.drawString("R", px + 13, UI_H - 7);

    bubble(s.tilt_front, COL_ACCENT(fb), true);          // front: filled
    bubble(s.tilt_rear, C(fb, 235, 160, 90), false);     // rear: outline
    // numeric R/P/F values live on the debug screen now; bubbles stay clean
}

// Fault detail replaces the level display when latched: header, the latched
// causes, and how to clear it (the middle button)
static void draw_fault_detail(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    const int px = UI_MODE_W, py = 0;
    const int pw = UI_W - UI_MODE_W - UI_BTN_W, ph = UI_H;
    fb.fillRect(px, py, pw, ph, C(fb, 45, 18, 15));

    fb.setTextSize(1);
    fb.setTextDatum(lgfx::middle_left);
    ui_icon_draw_centered(fb, "warning", px + 14, py + 13, COL_ERR(fb));
    fb.setTextColor(COL_ERR(fb));
    fb.drawString("SAFETY FAULT", px + 28, py + 13);
    fb.drawFastHLine(px + 6, py + 26, pw - 12, C(fb, 90, 40, 34));

    static const struct { uint32_t bit; const char *name; } names[] = {
        { SAFE_F_UNDERVOLT, "24V undervolt" }, { SAFE_F_MOTOR_FAULT, "motor fault" },
        { SAFE_F_OVERSPEED, "overspeed" }, { SAFE_F_TELEM_LOSS, "telemetry lost" },
        { SAFE_F_THERMAL, "motor thermal" }, { SAFE_F_RACKING, "frame racking" },
        { SAFE_F_DESYNC, "corner desync" }, { SAFE_F_SOL_DUTY, "solenoid duty" },
        { SAFE_F_TILT_FAIL, "tilt sensor" }, { SAFE_F_OVERTRAVEL, "overtravel" },
    };
    fb.setTextColor(COL_TEXT(fb));
    int y = py + 38;
    for (auto &n : names) {
        if (!(s.safety_flags & n.bit)) continue;
        if (y > py + ph - 30) break;                 // keep the reset hint clear
        fb.fillCircle(px + 9, y, 2, COL_ERR(fb));
        fb.drawString(n.name, px + 16, y);
        y += 13;
    }

    // reset hint
    fb.drawFastHLine(px + 6, py + ph - 20, pw - 12, C(fb, 90, 40, 34));
    fb.setTextColor(COL_WARN(fb));
    fb.drawString("hold RESET to clear", px + 6, py + ph - 10);
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
        // ROLL up-button raises the RIGHT rail (idx0,idx1 = +lr); label matches
        // the motion, not the old "LEFT+" which read backwards on the bench.
        static const char *axis_up[APP_MODE_COUNT] =
            { "", "UP", "NOSE+", "RIGHT+", "TW+", "M1+", "M2+", "M3+", "M4+" };
        static const char *axis_dn[APP_MODE_COUNT] =
            { "", "DOWN", "NOSE-", "RIGHT-", "TW-", "M1-", "M2-", "M3-", "M4-" };
        hints[0] = axis_up[s.mode]; hints[1] = "NEXT"; hints[2] = axis_dn[s.mode];
    }

    // LIFT: the glyphs say it all, so bigger icons and no words
    const bool icon_only = s.mode == APP_MODE_LIFT && s.motion != MOTION_FAULT;
    static const char *const lift_icons[3] = { "caret-up-lg", "level", "caret-down-lg" };

    for (int i = 0; i < 3; i++) {
        int cy0 = py + i * cellh;
        // a button with nothing to do draws as a faint outline only
        if (!hints[i][0]) {
            fb.drawRoundRect(px + 3, cy0 + 3, UI_BTN_W - 7, cellh - 6, 4, C(fb, 36, 42, 39));
            continue;
        }
        bool pressed = (s.btn_pressed_mask >> i) & 1;
        uint16_t bg = pressed ? COL_ACCENT(fb) : COL_PANEL(fb);
        uint16_t fg = pressed ? COL_PANEL(fb)
                      : calm(s) ? C(fb, 110, 118, 114) : COL_TEXT(fb);
        fb.fillRoundRect(px + 3, cy0 + 3, UI_BTN_W - 7, cellh - 6, 4, bg);
        fb.drawRoundRect(px + 3, cy0 + 3, UI_BTN_W - 7, cellh - 6, 4, COL_LINE(fb));

        int cx = px + UI_BTN_W / 2 - 1, cc = cy0 + cellh / 2;
        if (icon_only) {
            ui_icon_draw_centered(fb, lift_icons[i], cx, cc, fg);
            continue;
        }
        // glyphs: up arrow / dot / down arrow, plus hint text
        // same layout in every cell: glyph centered at cc-6, label at cc+9
        if (i == 0 && s.motion != MOTION_FAULT)
            ui_icon_draw_centered(fb, "caret-up", cx, cc - 6, fg);
        else if (i == 2 && s.motion != MOTION_FAULT)
            ui_icon_draw_centered(fb, "caret-down", cx, cc - 6, fg);
        else if (i == 1)
            fb.fillCircle(cx, cc - 6, 3, fg);

        fb.setTextSize(1);
        fb.setTextDatum(lgfx::middle_center);
        fb.setTextColor(fg);
        fb.drawString(hints[i], cx, cc + 9);
    }
}

// Debug display (triple-click center): all motors pos / velocity / force
static void draw_debug_table(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    const int px = UI_MODE_W, py = 14;          // below the warning banner

    fb.setTextSize(1);
    fb.setTextDatum(lgfx::top_left);
    fb.setTextColor(COL_DIM(fb));
    fb.setCursor(px + 2, py + 3);
    fb.printf("    pos   vel     F");   // rad / rad/s / Nm

    for (int i = 0; i < SYS_NUM_MOTORS; i++) {
        const motor_snap_t &m = s.motor[i];
        int y = py + 15 + i * 11;
        bool stale = !m.online;
        fb.setTextColor(stale ? COL_DIM(fb)
                              : (m.faults ? COL_ERR(fb) : COL_TEXT(fb)));
        fb.setCursor(px + 2, y);
        // tightened: no gaps after M%d; %+6.1f carries its own leading space
        fb.printf("M%d%+6.1f%+6.1f%+6.1f", i + 1,
                  m.theta_rad, m.vel_rad_s, m.torque_nm);
    }

    // tilt values (moved off the bubble screen) along the bottom
    float pitch = 0.5f * (s.tilt_front.pitch_deg + s.tilt_rear.pitch_deg);
    float twist = s.tilt_front.roll_deg - s.tilt_rear.roll_deg;
    fb.setTextColor(COL_DIM(fb));
    fb.setCursor(px + 2, py + 62);
    fb.printf("roll F%+.1f R%+.1f", s.tilt_front.roll_deg, s.tilt_rear.roll_deg);
    fb.setCursor(px + 2, py + 73);
    fb.printf("pitch%+.1f tw%+.1f", pitch, twist);
}

// Top banner: warnings and non-emergency errors, spelled out. Cycles when
// several are active. Emergencies (MOTION_FAULT) use the full fault screen.
static void draw_warning_banner(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    char msgs[6][30];
    int n = 0;

    if (s.safety_flags & SAFE_F_UNDERVOLT)
        snprintf(msgs[n++], 30, "MOTOR BATTERY LOW");   // no voltage sense; fault-bit only
    if (s.safety_flags & SAFE_F_RACKING) {
        float rack = s.tilt_front.roll_deg - s.tilt_rear.roll_deg;
        snprintf(msgs[n++], 30, "FRAME RACKED %+.1f deg", rack);
    }
    if (s.safety_flags & SAFE_F_MOTOR_FAULT) {
        int m = 0;
        for (int i = 0; i < SYS_NUM_MOTORS; i++)
            if (s.motor[i].faults) { m = i + 1; break; }
        snprintf(msgs[n++], 30, "MOTOR M%d FAULT", m);
    }
    if (s.safety_flags & SAFE_F_THERMAL) {
        float tmax = 0;
        for (int i = 0; i < SYS_NUM_MOTORS; i++)
            if (s.motor[i].temp_c > tmax) tmax = s.motor[i].temp_c;
        snprintf(msgs[n++], 30, "MOTOR HOT %.0fC", tmax);
    }
    if (s.safety_flags & SAFE_F_DESYNC)
        snprintf(msgs[n++], 30, "CORNER OUT OF SYNC");
    if (s.safety_flags & SAFE_F_TILT_FAIL)
        snprintf(msgs[n++], 30, "TILT SENSOR FAULT");
    if (s.safety_flags & SAFE_F_OVERTRAVEL)
        snprintf(msgs[n++], 30, "AT TRAVEL LIMIT");
    if ((s.safety_flags & SAFE_F_SOL_DUTY) || s.sol_cooldown_s > 0)
        snprintf(msgs[n++], 30, "LOCKS COOLING %lus",
                 (unsigned long)s.sol_cooldown_s);

    if (n == 0) return;

    const int bh = 14;
    const int bw = UI_W - UI_BTN_W;
    fb.fillRect(0, 0, bw, bh, C(fb, 82, 60, 14));
    fb.drawFastHLine(0, bh - 1, bw, COL_WARN(fb));

    int idx = (int)((s.now_us / 1500000) % n);
    fb.setTextSize(1);
    fb.setTextDatum(lgfx::middle_left);
    fb.setTextColor(C(fb, 250, 214, 140));
    fb.drawString(msgs[idx], 5, bh / 2);
    if (n > 1) {
        char cnt[16];
        // single digits by construction (max 6 messages); % keeps GCC's
        // format-truncation analysis happy on the target build
        snprintf(cnt, sizeof(cnt), "%u/%u",
                 (unsigned)(idx + 1) % 10, (unsigned)n % 10);
        fb.setTextDatum(lgfx::middle_right);
        fb.setTextColor(C(fb, 190, 160, 100));
        fb.drawString(cnt, bw - 4, bh / 2);
    }
}

void ui_render(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    fb.fillScreen(COL_BG(fb));
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
    if (s.motion != MOTION_FAULT)
        draw_warning_banner(fb, s);
}
