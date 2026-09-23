#include <cstdio>
#include <cmath>
#include "ui_panels.hpp"
#include "ui_icons.hpp"

struct ModeMeta {
    const char *name;
    const char *icon;         // name in icons/icons.txt
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
// Status bar: motors, locks, tilt, battery-warning (UV only), safety flash
// ---------------------------------------------------------------------------
static void draw_status(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    // stops at the button-column divider; the buttons run full height
    const int sw = UI_W - UI_BTN_W;
    fb.fillRect(0, 0, sw, UI_STATUS_H, COL_PANEL(fb));
    fb.drawFastHLine(0, UI_STATUS_H - 1, sw, COL_LINE(fb));

    int x = 4;
    const int cy = UI_STATUS_H / 2 - 1;

    // motors: one cycle glyph. dim = powered off, green = all online,
    // amber = some offline, red = any motor fault
    {
        int online = 0; bool fault = false;
        for (int i = 0; i < SYS_NUM_MOTORS; i++) {
            online += s.motor[i].online;
            fault |= s.motor[i].faults != 0;
        }
        uint16_t c = fault ? COL_ERR(fb)
                     : !s.motor_ssr_on ? COL_DIM(fb)
                     : online == SYS_NUM_MOTORS ? COL_OK(fb) : COL_WARN(fb);
        ui_icon_draw_centered(fb, "cycle", x + 10, cy + 1, c);
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

    // Battery: no motor-pack voltage sense exists, so this is purely an
    // undervoltage warning — an empty red battery, shown only on a UV fault.
    if (s.safety_flags & SAFE_F_UNDERVOLT) {
        uint16_t c = COL_ERR(fb);
        fb.drawRect(x, cy - 5, 18, 10, c);
        fb.fillRect(x + 18, cy - 2, 2, 4, c);
        fb.drawLine(x + 3, cy + 3, x + 15, cy - 3, c);   // slash = empty/fault
        x += 26;
    }

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

    ui_icon_draw_centered(fb, meta.icon, cx, py + 34, COL_ACCENT(fb), meta.rot);

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
    // cooldown countdown lives in the bottom warning banner
}

// ---------------------------------------------------------------------------
// Level display (center): dual bubble, front + rear independent
// ---------------------------------------------------------------------------
static void draw_level(LGFX_Sprite &fb, const sys_snapshot_t &s)
{
    const int px = UI_MODE_W, py = UI_STATUS_H;
    const int pw = UI_W - UI_MODE_W - UI_BTN_W, ph = UI_H - UI_STATUS_H;
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

    // crosshair + rings: inner = target level, outer = full scale
    const float k = (ph / 2 - 6) / LEVEL_RANGE_DEG;   // px per degree
    const int R = (int)(LEVEL_RANGE_DEG * k);         // outer ring radius (px)
    // crosshair overshoots the outer ring by the same stub on all four sides
    const int X = R + 5;
    fb.drawFastHLine(cx - X, cy, 2 * X + 1, COL_LINE(fb));
    fb.drawFastVLine(cx, cy - X, 2 * X + 1, COL_LINE(fb));
    // center ring is a sighting mark, sized just outside a centered bubble
    // (r=5) so it stays visible; the level test itself uses LEVEL_TARGET_DEG
    fb.drawCircle(cx, cy, 8, COL_LINE(fb));
    fb.drawCircle(cx, cy, R, COL_LINE(fb));

    // On-screen a bubble is a disc; beyond the outer ring it becomes an arrow
    // at the rim pointing toward where the bubble actually is.
    auto bubble = [&](const tilt_snap_t &t, uint16_t col, bool fill) {
        if (!t.valid) return;
        float bx = t.roll_deg * k, by = t.pitch_deg * k;
        float r = sqrtf(bx * bx + by * by);
        if (r <= R || r < 1.0f) {
            if (fill) fb.fillCircle(cx + (int)bx, cy + (int)by, 5, col);
            else      fb.drawCircle(cx + (int)bx, cy + (int)by, 5, col);
            fb.drawCircle(cx + (int)bx, cy + (int)by, 5, col);
            return;
        }
        float ux = bx / r, uy = by / r;              // outward unit vector
        float wx = -uy, wy = ux;                     // perpendicular
        int tx = cx + (int)(ux * R),        ty = cy + (int)(uy * R);
        int mx = cx + (int)(ux * (R - 10)), my = cy + (int)(uy * (R - 10));
        if (fill)
            fb.fillTriangle(tx, ty, mx + (int)(wx * 6), my + (int)(wy * 6),
                            mx - (int)(wx * 6), my - (int)(wy * 6), col);
        else
            fb.drawTriangle(tx, ty, mx + (int)(wx * 6), my + (int)(wy * 6),
                            mx - (int)(wx * 6), my - (int)(wy * 6), col);
    };
    bubble(s.tilt_front, COL_ACCENT(fb), true);          // front: filled
    bubble(s.tilt_rear, C(fb, 235, 160, 90), false);     // rear: outline
    // numeric R/P/F values live on the debug screen now; bubbles stay clean
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
    fb.print("hold CENTER to reset");
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

    for (int i = 0; i < 3; i++) {
        int cy0 = py + i * cellh;
        bool pressed = (s.btn_pressed_mask >> i) & 1;
        uint16_t bg = pressed ? COL_ACCENT(fb) : COL_PANEL(fb);
        uint16_t fg = pressed ? COL_PANEL(fb) : COL_TEXT(fb);
        fb.fillRoundRect(px + 3, cy0 + 3, UI_BTN_W - 7, cellh - 6, 4, bg);
        fb.drawRoundRect(px + 3, cy0 + 3, UI_BTN_W - 7, cellh - 6, 4, COL_LINE(fb));

        int cx = px + UI_BTN_W / 2 - 1, cc = cy0 + cellh / 2;
        // glyphs: up arrow / dot / down arrow, plus hint text
        // same layout in every cell: glyph centered at cc-6, label at cc+9
        if (i == 0 && s.motion != MOTION_FAULT)
            fb.fillTriangle(cx, cc - 11, cx - 8, cc - 1, cx + 8, cc - 1, fg);
        else if (i == 2 && s.motion != MOTION_FAULT)
            fb.fillTriangle(cx, cc - 1, cx - 8, cc - 11, cx + 8, cc - 11, fg);
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
    const int px = UI_MODE_W, py = UI_STATUS_H;
    const int pw = UI_W - UI_MODE_W - UI_BTN_W;

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

// Bottom banner: warnings and non-emergency errors, spelled out. Cycles when
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
    fb.fillRect(0, UI_H - bh, bw, bh, C(fb, 82, 60, 14));
    fb.drawFastHLine(0, UI_H - bh, bw, COL_WARN(fb));

    int idx = (int)((s.now_us / 1500000) % n);
    fb.setTextSize(1);
    fb.setTextDatum(lgfx::middle_left);
    fb.setTextColor(C(fb, 250, 214, 140));
    fb.drawString(msgs[idx], 5, UI_H - bh / 2);
    if (n > 1) {
        char cnt[16];
        // single digits by construction (max 6 messages); % keeps GCC's
        // format-truncation analysis happy on the target build
        snprintf(cnt, sizeof(cnt), "%u/%u",
                 (unsigned)(idx + 1) % 10, (unsigned)n % 10);
        fb.setTextDatum(lgfx::middle_right);
        fb.setTextColor(C(fb, 190, 160, 100));
        fb.drawString(cnt, bw - 4, UI_H - bh / 2);
    }
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
    if (s.motion != MOTION_FAULT)
        draw_warning_banner(fb, s);
}
