// Bedlift host simulator: the real cybergear driver + button state machine
// talking to the virtual plant, rendered through LovyanGFX's SDL backend.
//
// Buttons (feed the production btn_sm):
//   Up / Down arrows = bed buttons, M = center/mode button
//   (hold both arrows for the chord)
// Bench controls:
//   P ssr toggle   K lock rail toggle   E init motors   SPACE stop all
//   7/8/9/0 select M1..M4, then:
//   u=undervolt o=overcurrent d=driver c=comm-loss s=snag w=runaway x=clear
// NOTE: avoid L/R (Panel_sdl rotation) and 1-6 (Panel_sdl scaling).

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#include <SDL2/SDL.h>
#include <chrono>
#include <cstdio>
#include <cstring>

#include "cybergear.h"
#include "sim.h"
#include "buttons_sm.h"
#include "sys_state.h"
#include "ui_panels.hpp"
#include "motion_fsm.h"
#include "hazard_checks.h"
#include "control_law.h"

static LGFX lcd(240, 135);
static LGFX_Sprite frame(&lcd);       // full-frame back buffer (flicker fix)

static cybergear_motor_t motors[SIM_NUM_MOTORS];
static const uint8_t k_ids[SIM_NUM_MOTORS] = { 0x01, 0x04, 0x03, 0x02 };
static bool ssr_on = false, lock_on = false;
static int sel = 0;
static sim_fault_e injected[SIM_NUM_MOTORS] = {};
static btn_sm_t btn_sm;

// last few button events for the on-screen log (M3 verification)
static char ev_log[6][28];
static int ev_count = 0;

// motion intent derived from button events (placeholder until the motion FSM)
static float cmd_v = 0;
static bool ui_view = true;              // TAB toggles production UI / bench view
static app_mode_e app_mode = APP_MODE_LIFT;
static bool debug_screen = false;        // triple-click center toggles

// per-mode motion vectors for the up button (down = negated)
static const float k_vec_up[APP_MODE_COUNT][SIM_NUM_MOTORS] = {
    MVEC_LIFT_UP,                       // LIFT
    MVEC_PITCH_POS,                     // PITCH: nose up
    MVEC_ROLL_POS,                      // ROLL: left side up
    MVEC_TWIST_POS,                     // TWIST
    { 1, 0, 0, 0 }, { 0, 1, 0, 0 },    // M1, M2
    { 0, 0, 1, 0 }, { 0, 0, 0, 1 },    // M3, M4
};

// APP mode: the motion FSM owns power + choreography, exactly like the target
static motion_fsm_t fsm;
static motion_out_t fsm_out;
static bool up_held = false, down_held = false, level_held = false;
static safety_ctx_t safety;
static level_law_t level_law;
static uint32_t latched_flags = 0;       // shown until fault ack
static uint32_t warn_flags = 0;          // live warnings (racking etc.)

static const char *motion_state_name(motion_state_e m)
{
    static const char *n[] = { "idle", "pwrup", "ready", "unload", "unlock",
                               "up", "down", "level", "ramp", "settle",
                               "pwrdn", "FAULT" };
    return (m >= 0 && m <= MOTION_FAULT) ? n[m] : "?";
}

static int64_t now_us()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

static void log_event(const btn_event_t *e)
{
    static const char *names[] = { "DOWN", "UP", "SHORT", "HOLD", "HOLD_END",
                                   "REPEAT", "CHORD", "CHORD_END" };
    static const char *btns[] = { "UP", "MODE", "DOWN" };
    memmove(ev_log[1], ev_log[0], sizeof(ev_log) - sizeof(ev_log[0]));
    if (e->type == BEV_CHORD_UPDOWN || e->type == BEV_CHORD_UPDOWN_END)
        snprintf(ev_log[0], sizeof(ev_log[0]), "%s", names[e->type]);
    else
        snprintf(ev_log[0], sizeof(ev_log[0]), "%s %s", btns[e->id], names[e->type]);
    if (ev_count < 6) ev_count++;
}

static void pump_rx()
{
    twai_message_t m;
    while (sim_can_poll_rx(&m)) {
        for (int i = 0; i < SIM_NUM_MOTORS; i++)
            if (cybergear_process_message(&motors[i], &m, now_us()) != ESP_ERR_NOT_FOUND)
                break;
    }
}

static void motors_init_all()
{
    for (int i = 0; i < SIM_NUM_MOTORS; i++) {
        cybergear_stop(&motors[i]);
        cybergear_set_mode(&motors[i], CYBERGEAR_MODE_SPEED);
        cybergear_set_limit_current(&motors[i], 7.0f);
        cybergear_enable(&motors[i]);
    }
}

// Apply FSM outputs to the plant + real driver — the exact surface the
// target's motion_task will apply to GPIO + TWAI.
static void app_apply_outputs(int64_t t)
{
    static motion_out_t prev = {};
    sim_power_set(fsm_out.ssr_on, fsm_out.lock_on);
    ssr_on = fsm_out.ssr_on;                 // reflect into the status header
    lock_on = fsm_out.lock_on;

    if (fsm_out.req_init && !prev.req_init) {
        for (auto &m : motors) {
            cybergear_stop(&m);
            cybergear_set_mode(&m, CYBERGEAR_MODE_SPEED);
            cybergear_set_limit_current(&m, 7.0f);
        }
    }
    if (fsm_out.req_enable && !prev.req_enable)
        for (auto &m : motors) cybergear_enable(&m);
    if (fsm_out.req_disable && !prev.req_disable)
        for (auto &m : motors) cybergear_stop(&m);
    prev = fsm_out;

    // 100 Hz per-motor speed stream (write-echo telemetry)
    static int64_t last_cmd = 0;
    if (fsm_out.ssr_on && t - last_cmd > 10000) {
        last_cmd = t;
        for (int i = 0; i < SIM_NUM_MOTORS; i++)
            cybergear_set_speed(&motors[i], fsm_out.v_cmd[i]);
    }
}

static void app_step(int64_t t, float dt)
{
    motion_intent_e intent = MI_NONE;
    float vec[SIM_NUM_MOTORS] = {};
    if (level_held && app_mode == APP_MODE_LIFT) {
        intent = MI_LEVEL;
    } else if (up_held != down_held) {             // exactly one direction held
        intent = MI_MOVE;
        float sign = up_held ? 1.0f : -1.0f;
        for (int i = 0; i < SIM_NUM_MOTORS; i++)
            vec[i] = sign * k_vec_up[app_mode][i];
    }

    motion_motor_in_t in[SIM_NUM_MOTORS];
    for (int i = 0; i < SIM_NUM_MOTORS; i++) {
        in[i].theta_rad = sim_motor_angle(i);   // target: unwrapped telemetry
        in[i].vel_rad_s = motors[i].status.speed;
        in[i].torque_nm = motors[i].status.torque;
        in[i].temp_c = motors[i].status.temperature;
        in[i].online = t - motors[i].status.last_rx_us < 150000;
        in[i].faults = motors[i].faults;
    }

    // safety pass first, exactly like the target's motion_task cycle
    static float v_cmd_prev[SIM_NUM_MOTORS] = {};
    tilt_snap_t raw_f, raw_r;
    { float ax, ay, az;
      sim_accel_read(0, &ax, &ay, &az);
      raw_f = { atan2f(ay, az) * 57.2958f, atan2f(ax, az) * 57.2958f, true, t };
      sim_accel_read(1, &ax, &ay, &az);
      raw_r = { atan2f(ay, az) * 57.2958f, atan2f(ax, az) * 57.2958f, true, t }; }
    static tilt_snap_t tf = {}, tr = {};
    tilt_filter(&tf, &raw_f, dt, 0.25f);
    tilt_filter(&tr, &raw_r, dt, 0.25f);
    float vbus = 0;
    for (auto &m : motors) if (m.params.vbus > vbus) vbus = m.params.vbus;

    float level_v[SIM_NUM_MOTORS] = {};
    if (intent == MI_LEVEL || fsm.state == MOTION_LEVELING)
        level_control(&level_law, &tf, &tr, level_v);

    // leveling auto-complete: within 1 deg (level + twist) for 500 ms
    static int64_t within_since = 0;
    bool within = level_within(&tf, &tr, level_law.done_deg);
    if (!within) within_since = 0;
    else if (within_since == 0) within_since = t;
    bool level_done = within && (t - within_since > 500000);

    // travel trim: only for uniform LIFT moves (differential modes command
    // tilt changes on purpose — trim would fight the user)
    float trim_v[SIM_NUM_MOTORS] = {};
    if (app_mode == APP_MODE_LIFT &&
        (fsm.state == MOTION_MOVING_UP || fsm.state == MOTION_MOVING_DOWN))
        travel_trim(&level_law, &tf, &tr, fabsf(fsm.group_v), trim_v);

    // a fresh motion attempt from READY re-evaluates; stop-latched flags clear
    if (fsm.state == MOTION_READY && intent != MI_NONE) latched_flags = 0;

    uint32_t warn_only = (app_mode_group(app_mode) != GROUP_DEFAULT)
                         ? SAFE_F_RACKING : 0;
    safety_result_t sr;
    safety_check(&safety, t, dt, in, v_cmd_prev, &tf, &tr, fsm.state, vbus,
                 warn_only, &sr);
    warn_flags = (sr.verdict == SAFE_WARN) ? sr.flags : 0;   // live, not latched
    if (sr.verdict == SAFE_TRIP && fsm.state != MOTION_FAULT) {
        // trip order: locks first (pawls = unpowered brake), brake window
        // would go here on target, then SSR off, then latch
        latched_flags |= sr.flags;
        sim_power_set(false, false);
        ssr_on = false; lock_on = false;
        for (auto &m : motors) cybergear_stop(&m);
        motion_fsm_fault(&fsm);
    } else if (sr.verdict == SAFE_STOP) {
        latched_flags |= sr.flags;               // shown until ack
        intent = MI_NONE;                        // force controlled stop
    }

    motion_fsm_step(&fsm, t, intent, vec, level_v, trim_v, level_done, in, dt, &fsm_out);
    if (fsm.state != MOTION_FAULT) app_apply_outputs(t);
    memcpy(v_cmd_prev, fsm_out.v_cmd, sizeof(v_cmd_prev));

    // console telemetry: sim truth + driver view side by side (2 Hz while
    // anything is happening, so the UI window and numbers coexist)
    static int64_t last_spew = 0;
    bool active = fsm.state != MOTION_IDLE;
    for (int i = 0; i < SIM_NUM_MOTORS && !active; i++)
        if (fabsf(sim_motor_vel(i)) > 0.02f) active = true;
    if (active && t - last_spew > 500000) {
        last_spew = t;
        printf("[%s] cmd[%+.2f %+.2f %+.2f %+.2f] simv[%+.2f %+.2f %+.2f %+.2f] "
               "th[%.2f %.2f %.2f %.2f] h[%.3f %.3f %.3f %.3f] "
               "tilt F%+.1f/R%+.1f P%+.1f vbus %.1f%s%s\n",
               motion_state_name(fsm.state),
               fsm_out.v_cmd[0], fsm_out.v_cmd[1], fsm_out.v_cmd[2], fsm_out.v_cmd[3],
               sim_motor_vel(0), sim_motor_vel(1), sim_motor_vel(2), sim_motor_vel(3),
               sim_motor_angle(0), sim_motor_angle(1), sim_motor_angle(2), sim_motor_angle(3),
               sim_corner_height_m(0), sim_corner_height_m(1),
               sim_corner_height_m(2), sim_corner_height_m(3),
               tf.roll_deg, tr.roll_deg,
               0.5f * (tf.pitch_deg + tr.pitch_deg), vbus,
               latched_flags ? " LATCHED" : "", warn_flags ? " WARN" : "");
    }
}

void setup()
{
    ((lgfx::Panel_sdl *)lcd.getPanel())->setScaling(3, 3);
    lcd.init();
    frame.setColorDepth(16);
    frame.createSprite(240, 135);

    int fails = cybergear_selftest_run();
    printf("selftest failures: %d\n", fails);

    sim_config_t cfg;
    sim_default_config(&cfg);
    sim_init(&cfg);
    cybergear_set_transport(sim_can_tx, nullptr);
    for (int i = 0; i < SIM_NUM_MOTORS; i++)
        cybergear_init(&motors[i], 0x00, k_ids[i], 0);

    btn_sm_init(&btn_sm);
    motion_fsm_init(&fsm);
    fsm.t_boot_us = 300 * 1000;    // sim motors boot instantly; keep it snappy
    safety_init(&safety);
    level_law_init(&level_law);
}

static void handle_bench_keys(const Uint8 *k)
{
    static bool prev[SDL_NUM_SCANCODES] = {};
    auto edge = [&](int sc) { return k[sc] && !prev[sc]; };

    if (edge(SDL_SCANCODE_TAB)) ui_view = !ui_view;
    if (edge(SDL_SCANCODE_E)) motors_init_all();
    if (edge(SDL_SCANCODE_SPACE))
        for (auto &m : motors) cybergear_stop(&m);
    if (edge(SDL_SCANCODE_K)) { lock_on = !lock_on; sim_power_set(ssr_on, lock_on); }
    if (edge(SDL_SCANCODE_P)) { ssr_on = !ssr_on; sim_power_set(ssr_on, lock_on); }

    const int selkeys[4] = { SDL_SCANCODE_7, SDL_SCANCODE_8, SDL_SCANCODE_9,
                             SDL_SCANCODE_0 };
    for (int i = 0; i < 4; i++)
        if (edge(selkeys[i])) sel = i;

    struct { int sc; sim_fault_e f; } faults[] = {
        { SDL_SCANCODE_U, SIM_FAULT_UNDERVOLTAGE }, { SDL_SCANCODE_O, SIM_FAULT_OVERCURRENT },
        { SDL_SCANCODE_D, SIM_FAULT_DRIVER }, { SDL_SCANCODE_C, SIM_FAULT_COMM_LOSS },
        { SDL_SCANCODE_S, SIM_FAULT_SNAG }, { SDL_SCANCODE_W, SIM_FAULT_RUNAWAY },
        { SDL_SCANCODE_X, SIM_FAULT_NONE },
    };
    for (auto &f : faults)
        if (edge(f.sc)) { injected[sel] = f.f; sim_inject(sel, f.f); }

    memcpy(prev, k, SDL_NUM_SCANCODES);
}

static void apply_button_events()
{
    btn_event_t e;
    while (btn_sm_poll(&btn_sm, &e)) {
        log_event(&e);
        switch (e.type) {
            case BEV_DOWN:
                if (e.id == BTN_UP) { up_held = true; cmd_v = 2.0f; }
                if (e.id == BTN_DOWN) { down_held = true; cmd_v = -2.0f; }
                break;
            case BEV_UP:
                if (e.id == BTN_UP) { up_held = false; cmd_v = 0; }
                if (e.id == BTN_DOWN) { down_held = false; cmd_v = 0; }
                if (e.id == BTN_MODE) level_held = false;
                break;
            case BEV_SHORT:
                if (e.id == BTN_MODE) {
                    // triple-click detection: burst of 3 shorts < 900 ms
                    static int64_t t1 = 0, t2 = 0;
                    static app_mode_e pre_burst = APP_MODE_LIFT;
                    if (e.t_us - t2 > 900000) pre_burst = app_mode;
                    if (t1 != 0 && e.t_us - t1 < 900000) {
                        // third click of a burst: toggle display, undo cycling
                        debug_screen = !debug_screen;
                        app_mode = pre_burst;
                        t1 = t2 = 0;
                        break;
                    }
                    t1 = t2; t2 = e.t_us;
                    // single short: cycle within the current group
                    switch (app_mode) {
                        case APP_MODE_PITCH: app_mode = APP_MODE_ROLL; break;
                        case APP_MODE_ROLL:  app_mode = APP_MODE_TWIST; break;
                        case APP_MODE_TWIST: app_mode = APP_MODE_PITCH; break;
                        case APP_MODE_M1: app_mode = APP_MODE_M2; break;
                        case APP_MODE_M2: app_mode = APP_MODE_M3; break;
                        case APP_MODE_M3: app_mode = APP_MODE_M4; break;
                        case APP_MODE_M4: app_mode = APP_MODE_M1; break;
                        default: break;            // LIFT: shorts do nothing
                    }
                }
                break;
            case BEV_HOLD:
                if (e.id == BTN_MODE) {
                    if (fsm.state == MOTION_FAULT) {
                        motion_fsm_ack(&fsm);
                        latched_flags = 0;         // diagnostics rerun on next move
                    }
                    else if (app_mode != APP_MODE_LIFT) app_mode = APP_MODE_LIFT;
                    else level_held = true;        // center hold = self-level
                }
                break;
            case BEV_CHORD_UPDOWN:
                cmd_v = 0;                         // chord never moves the bed
                up_held = down_held = false;
                switch (app_mode_group(app_mode)) {
                    case GROUP_DEFAULT: app_mode = APP_MODE_PITCH; break;
                    case GROUP_MANUAL:  app_mode = APP_MODE_M1; break;
                    case GROUP_DEBUG:   app_mode = APP_MODE_PITCH; break;
                }
                break;
            default:
                break;
        }
    }
}

static const char *fault_name(sim_fault_e f)
{
    switch (f) {
        case SIM_FAULT_UNDERVOLTAGE: return "UV";
        case SIM_FAULT_OVERCURRENT: return "OC";
        case SIM_FAULT_DRIVER: return "DRV";
        case SIM_FAULT_COMM_LOSS: return "COMM";
        case SIM_FAULT_SNAG: return "SNAG";
        case SIM_FAULT_RUNAWAY: return "RUN!";
        default: return "-";
    }
}

static void build_snapshot(sys_snapshot_t *s)
{
    memset(s, 0, sizeof(*s));
    int64_t t = now_us();
    s->now_us = t;
    for (int i = 0; i < SIM_NUM_MOTORS; i++) {
        const cybergear_status_t &st = motors[i].status;
        s->motor[i].online = ssr_on && (t - st.last_rx_us < 300000);
        s->motor[i].vel_rad_s = st.speed;
        s->motor[i].torque_nm = st.torque;
        s->motor[i].temp_c = st.temperature;
        s->motor[i].theta_rad = sim_motor_angle(i);   // TODO: unwrap from telemetry
        s->motor[i].vbus_v = motors[i].params.vbus;   // via real type-17 reads
        s->motor[i].faults = motors[i].faults;
        s->motor[i].last_rx_us = st.last_rx_us;
    }
    // tilt via the same math the sensor task will use
    float ax, ay, az;
    sim_accel_read(0, &ax, &ay, &az);
    s->tilt_front = { atan2f(ax, az) * 57.2958f, atan2f(ay, az) * 57.2958f, true, t };
    // reuse roll/pitch naming: roll from x, pitch from y
    { float bx, by, bz; sim_accel_read(1, &bx, &by, &bz);
      s->tilt_rear = { atan2f(bx, bz) * 57.2958f, atan2f(by, bz) * 57.2958f, true, t }; }
    // swap: tilt struct is {pitch, roll} — fix field order
    { float r = s->tilt_front.pitch_deg; s->tilt_front.pitch_deg = s->tilt_front.roll_deg; s->tilt_front.roll_deg = r; }
    { float r = s->tilt_rear.pitch_deg; s->tilt_rear.pitch_deg = s->tilt_rear.roll_deg; s->tilt_rear.roll_deg = r; }

    s->hall_top = sim_hall_top();
    s->hall_bottom = sim_hall_bottom();
    s->lipo_soc = 0.82f;
    s->lipo_v = 3.9f;
    s->motor_ssr_on = ssr_on;
    s->lock_energized = lock_on;
    (void)0;
    s->sol_budget_frac = 1.0f;            // duty model lands in M7
    s->btn_pressed_mask = (uint8_t)((btn_sm.b[BTN_UP].stable ? 1 : 0) |
                                    (btn_sm.b[BTN_MODE].stable ? 2 : 0) |
                                    (btn_sm.b[BTN_DOWN].stable ? 4 : 0));
    s->mode = app_mode;
    s->debug_screen = debug_screen;
    s->motion = ui_view ? fsm.state
                : (!ssr_on ? MOTION_IDLE
                   : (cmd_v > 0.1f ? MOTION_MOVING_UP
                      : (cmd_v < -0.1f ? MOTION_MOVING_DOWN : MOTION_READY)));
    s->safety_flags = ui_view ? (latched_flags | warn_flags) : 0;
}

static void draw_ui()
{
    sys_snapshot_t s;
    build_snapshot(&s);
    ui_render(frame, s);
    frame.pushSprite(0, 0);
}

static void draw()
{
    frame.fillScreen(TFT_BLACK);
    frame.setTextSize(1);
    frame.setTextDatum(lgfx::top_left);

    frame.setTextColor(TFT_WHITE);
    frame.setCursor(2, 2);
    frame.printf("SSR:%s LOCK:%s VBUS:%4.1fV sel:M%d v:%+.1f",
                 ssr_on ? "ON " : "off", lock_on ? "ON " : "off",
                 sim_vbus(), sel + 1, cmd_v);

    for (int i = 0; i < SIM_NUM_MOTORS; i++) {
        int y = 13 + i * 10;
        const cybergear_status_t &st = motors[i].status;
        bool stale = now_us() - st.last_rx_us > 300000;
        frame.setTextColor(stale ? frame.color565(120, 120, 120)
                                 : (motors[i].faults ? frame.color565(240, 90, 70)
                                                     : frame.color565(120, 220, 140)));
        frame.setCursor(2, y);
        frame.printf("M%d %s v%+5.2f %4.1fC %s %s", i + 1,
                     st.state == CYBERGEAR_STATE_RUNNING ? "RUN" : "rst",
                     st.speed, st.temperature,
                     sim_pawl_seated(i) ? "SEAT" : "free",
                     fault_name(injected[i]));
    }

    int bx = 2, by = 56;
    for (int i = 0; i < SIM_NUM_MOTORS; i++) {
        float h = sim_corner_height_m(i);
        int barh = (int)(h * 38.0f);
        if (barh > 38) barh = 38;
        if (barh < 0) barh = 0;
        frame.fillRect(bx + i * 16, by + 38 - barh, 11, barh,
                       frame.color565(90, 150, 220));
        frame.drawRect(bx + i * 16, by, 11, 38, frame.color565(70, 80, 75));
    }
    frame.setTextColor(TFT_WHITE);
    frame.setCursor(72, by);
    frame.printf("pitch%+5.2f", sim_bed_pitch_deg());
    frame.setCursor(72, by + 9);
    frame.printf("rollF%+5.2f", sim_bed_roll_front_deg());
    frame.setCursor(72, by + 18);
    frame.printf("rollR%+5.2f", sim_bed_roll_rear_deg());
    frame.setCursor(72, by + 27);
    frame.printf("hall T%d B%d", sim_hall_top(), sim_hall_bottom());

    // button event log (right column)
    frame.setTextColor(frame.color565(200, 190, 120));
    for (int i = 0; i < ev_count; i++) {
        frame.setCursor(148, 56 + i * 9);
        frame.print(ev_log[i]);
    }

    frame.setTextColor(frame.color565(150, 150, 150));
    frame.setCursor(2, 98);
    frame.print("arrows/M:buttons P:ssr K:lock E:init");
    frame.setCursor(2, 107);
    frame.print("SPC:stop 7890:sel u/o/d/c/s/w/x:fault");
    frame.setCursor(2, 116);
    frame.print("(L/R rotate, 1-6 scale = SDL keys)");

    frame.pushSprite(0, 0);
}

void loop();

static int user_func(bool *running)
{
    setup();
    do {
        loop();
    } while (*running);
    return 0;
}

// ---- headless screenshot mode: ./hostsim --snap <outdir> -------------------
static void write_bmp(const char *path, LGFX_Sprite &sp)
{
    int w = sp.width(), h = sp.height();
    int row = (w * 3 + 3) & ~3;
    int datasz = row * h, filesz = 54 + datasz;
    uint8_t hdr[54] = { 'B', 'M' };
    auto put32 = [&](int off, uint32_t v) {
        hdr[off] = v; hdr[off + 1] = v >> 8; hdr[off + 2] = v >> 16; hdr[off + 3] = v >> 24;
    };
    put32(2, filesz); put32(10, 54); put32(14, 40);
    put32(18, w); put32(22, h);
    hdr[26] = 1; hdr[28] = 24;
    put32(34, datasz);
    FILE *f = fopen(path, "wb");
    if (!f) return;
    fwrite(hdr, 1, 54, f);
    uint8_t *line = (uint8_t *)calloc(1, row);
    for (int y = h - 1; y >= 0; y--) {
        for (int x = 0; x < w; x++) {
            auto c = sp.readPixelRGB(x, y);
            line[x * 3] = c.b; line[x * 3 + 1] = c.g; line[x * 3 + 2] = c.r;
        }
        fwrite(line, 1, row, f);
    }
    free(line);
    fclose(f);
}

static void snap_all(const char *dir)
{
    LGFX_Sprite sp;
    sp.setColorDepth(16);
    sp.createSprite(UI_W, UI_H);

    sys_snapshot_t s;
    memset(&s, 0, sizeof(s));
    s.now_us = 1000000;
    for (int i = 0; i < SYS_NUM_MOTORS; i++) {
        s.motor[i].online = true;
        s.motor[i].vel_rad_s = 1.9f - 0.1f * i;
        s.motor[i].torque_nm = 2.2f + 0.3f * i;
        s.motor[i].temp_c = 24.5f + i;
        s.motor[i].theta_rad = 10.5f + 0.4f * i;
        s.motor[i].vbus_v = 24.3f;
        s.motor[i].last_rx_us = 900000;
    }
    s.tilt_front = { 3.5f, -6.2f, true, 900000 };
    s.tilt_rear = { 3.1f, -2.0f, true, 900000 };
    s.lipo_soc = 0.82f; s.lipo_v = 3.9f;
    s.motor_ssr_on = true; s.lock_energized = true;
    s.sol_budget_frac = 0.72f;
    s.mode = APP_MODE_LIFT;
    s.motion = MOTION_MOVING_UP;
    s.btn_pressed_mask = 1;

    char p[256];
    auto shoot = [&](const char *name) {
        ui_render(sp, s);
        snprintf(p, sizeof(p), "%s/%s.bmp", dir, name);
        write_bmp(p, sp);
        printf("wrote %s\n", p);
    };

    shoot("lift_moving");

    s.motion = MOTION_IDLE; s.motor_ssr_on = false; s.lock_energized = false;
    s.btn_pressed_mask = 0;
    for (auto &m : s.motor) m.online = false;
    shoot("lift_idle");

    s.mode = APP_MODE_PITCH; shoot("pitch");
    s.mode = APP_MODE_TWIST; shoot("twist");
    s.mode = APP_MODE_M3; shoot("m3");

    s.mode = APP_MODE_LIFT; s.debug_screen = true;
    s.motor_ssr_on = true;
    for (auto &m : s.motor) m.online = true;
    s.motion = MOTION_MOVING_DOWN;
    shoot("debug_table");

    s.debug_screen = false;
    s.motion = MOTION_FAULT;
    s.safety_flags = SAFE_F_OVERSPEED | SAFE_F_TELEM_LOSS;
    s.motor_ssr_on = false;
    shoot("fault");

    s.motion = MOTION_MOVING_UP;
    s.safety_flags = SAFE_F_RACKING;
    s.tilt_front = { 2.0f, 8.0f, true, 900000 };
    s.tilt_rear = { 2.0f, -4.0f, true, 900000 };
    shoot("racking_warn");
}

// ---- headless leveling experiment: ./hostsim --test-level ------------------
// Reproduces the manual test: disturb the frame in pitch, roll and twist,
// then self-level from LIFT. Synthetic 1 kHz clock, faster than real time.
static int run_level_test()
{
    sim_config_t cfg;
    sim_default_config(&cfg);
    sim_init(&cfg);
    cybergear_set_transport(sim_can_tx, nullptr);
    for (int i = 0; i < SIM_NUM_MOTORS; i++)
        cybergear_init(&motors[i], 0x00, k_ids[i], 0);
    motion_fsm_init(&fsm);
    fsm.t_boot_us = 300 * 1000;
    safety_init(&safety);
    level_law_init(&level_law);

    const float vec_up[4] = MVEC_LIFT_UP;
    const float vec_pitch[4] = MVEC_PITCH_POS;
    const float vec_roll[4] = MVEC_ROLL_POS;
    const float vec_twist[4] = MVEC_TWIST_POS;

    int64_t t = 0;
    const float dt = 0.001f;
    float v_prev[4] = {};
    motion_out_t prev_out = {};
    int64_t last_cmd = 0, last_print = 0;

    for (int step = 0; step < 90000; step++) {
        t += 1000;
        float ts = t / 1e6f;

        motion_intent_e intent = MI_NONE;
        const float *vec = nullptr;
        // schedule: raise, then disturb pitch/roll/twist, pause, then level
        if (ts < 4.0f)      { intent = MI_MOVE; vec = vec_up; }
        else if (ts < 6.0f) { intent = MI_MOVE; vec = vec_pitch; }
        else if (ts < 7.5f) { intent = MI_MOVE; vec = vec_roll; }
        else if (ts < 8.3f) { intent = MI_MOVE; vec = vec_twist; }
        else if (ts < 10.0f) intent = MI_NONE;
        else intent = MI_LEVEL;   // held throughout: FSM must auto-complete

        motion_motor_in_t in[4];
        for (int i = 0; i < 4; i++) {
            in[i].theta_rad = sim_motor_angle(i);
            in[i].vel_rad_s = motors[i].status.speed;
            in[i].torque_nm = motors[i].status.torque;
            in[i].temp_c = motors[i].status.temperature;
            in[i].online = t - motors[i].status.last_rx_us < 150000;
            in[i].faults = motors[i].faults;
        }
        tilt_snap_t raw_f, raw_r;
        { float ax, ay, az;
          sim_accel_read(0, &ax, &ay, &az);
          raw_f = { atan2f(ay, az) * 57.2958f, atan2f(ax, az) * 57.2958f, true, t };
          sim_accel_read(1, &ax, &ay, &az);
          raw_r = { atan2f(ay, az) * 57.2958f, atan2f(ax, az) * 57.2958f, true, t }; }
        static tilt_snap_t tf = {}, tr = {};
        tilt_filter(&tf, &raw_f, dt, 0.25f);
        tilt_filter(&tr, &raw_r, dt, 0.25f);

        float level_v[4] = {};
        if (intent == MI_LEVEL || fsm.state == MOTION_LEVELING)
            level_control(&level_law, &tf, &tr, level_v);
        static int64_t within_since = 0;
        bool within = level_within(&tf, &tr, level_law.done_deg);
        if (!within) within_since = 0;
        else if (within_since == 0) within_since = t;
        bool level_done = within && (t - within_since > 500000);

        uint32_t warn_only = SAFE_F_RACKING;   // deliberate twisting below
        safety_result_t sr;
        safety_check(&safety, t, dt, in, v_prev, &tf, &tr, fsm.state, 24.5f,
                     warn_only, &sr);
        if (sr.verdict == SAFE_TRIP) {
            printf("TRIP flags=0x%x at t=%.1fs — test FAIL\n", sr.flags, ts);
            return 1;
        }

        motion_fsm_step(&fsm, t, intent, vec, level_v, nullptr, level_done, in, dt, &fsm_out);

        sim_power_set(fsm_out.ssr_on, fsm_out.lock_on);
        if (fsm_out.req_init && !prev_out.req_init)
            for (auto &m : motors) {
                cybergear_stop(&m);
                cybergear_set_mode(&m, CYBERGEAR_MODE_SPEED);
                cybergear_set_limit_current(&m, 7.0f);
            }
        if (fsm_out.req_enable && !prev_out.req_enable)
            for (auto &m : motors) cybergear_enable(&m);
        if (fsm_out.req_disable && !prev_out.req_disable)
            for (auto &m : motors) cybergear_stop(&m);
        prev_out = fsm_out;
        if (fsm_out.ssr_on && t - last_cmd >= 10000) {
            last_cmd = t;
            for (int i = 0; i < 4; i++)
                cybergear_set_speed(&motors[i], fsm_out.v_cmd[i]);
        }
        memcpy(v_prev, fsm_out.v_cmd, sizeof(v_prev));

        sim_step(t, dt);
        { twai_message_t m;
          while (sim_can_poll_rx(&m))
              for (int i = 0; i < 4; i++)
                  if (cybergear_process_message(&motors[i], &m, t) != ESP_ERR_NOT_FOUND)
                      break; }

        if (t - last_print >= 1000000) {
            last_print = t;
            printf("t=%4.1fs %-7s rollF %+6.2f rollR %+6.2f pitch %+6.2f "
                   "th[%5.2f %5.2f %5.2f %5.2f]\n",
                   ts, motion_state_name(fsm.state),
                   tf.roll_deg, tr.roll_deg,
                   0.5f * (tf.pitch_deg + tr.pitch_deg),
                   sim_motor_angle(0), sim_motor_angle(1),
                   sim_motor_angle(2), sim_motor_angle(3));
        }

        if (fsm.level_complete && fsm.state == MOTION_READY) {
            float p = 0.5f * (tf.pitch_deg + tr.pitch_deg);
            // gate = auto-lock target (1 deg) + settle margin
            float g = level_law.done_deg + 0.2f;
            bool pass = fabsf(tf.roll_deg) < g && fabsf(tr.roll_deg) < g &&
                        fabsf(p) < g && fabsf(tf.roll_deg - tr.roll_deg) < g;
            printf("auto-completed+settled at t=%.1fs: rollF %+.2f rollR %+.2f "
                   "pitch %+.2f -> %s\n",
                   ts, tf.roll_deg, tr.roll_deg, p, pass ? "PASS" : "FAIL");
            return pass ? 0 : 1;
        }
    }
    printf("timeout without settling — FAIL\n");
    return 1;
}

// ---- travel feedback test: ./hostsim --test-travel -------------------------
// Disturb the frame (pitch + twist ~2 deg), then run a long LIFT raise with
// travel trim active. Pass: tilt errors shrink below 1 deg DURING travel.
static int run_travel_test()
{
    sim_config_t cfg;
    sim_default_config(&cfg);
    sim_init(&cfg);
    cybergear_set_transport(sim_can_tx, nullptr);
    for (int i = 0; i < SIM_NUM_MOTORS; i++)
        cybergear_init(&motors[i], 0x00, k_ids[i], 0);
    motion_fsm_init(&fsm);
    fsm.t_boot_us = 300 * 1000;
    safety_init(&safety);
    level_law_init(&level_law);

    const float vec_up[4] = MVEC_LIFT_UP;
    const float vec_pitch[4] = MVEC_PITCH_POS;
    const float vec_twist[4] = MVEC_TWIST_POS;

    int64_t t = 0;
    const float dt = 0.001f;
    float v_prev[4] = {};
    motion_out_t prev_out = {};
    int64_t last_cmd = 0, last_print = 0;

    for (int step = 0; step < 60000; step++) {
        t += 1000;
        float ts = t / 1e6f;

        motion_intent_e intent = MI_NONE;
        const float *vec = nullptr;
        bool lift_phase = false;
        if (ts < 2.0f)      { intent = MI_MOVE; vec = vec_up; }
        else if (ts < 3.6f) { intent = MI_MOVE; vec = vec_pitch; }
        else if (ts < 4.6f) { intent = MI_MOVE; vec = vec_twist; }
        else if (ts < 6.0f) intent = MI_NONE;
        else if (ts < 20.0f) { intent = MI_MOVE; vec = vec_up; lift_phase = true; }
        else intent = MI_NONE;

        motion_motor_in_t in[4];
        for (int i = 0; i < 4; i++) {
            in[i].theta_rad = sim_motor_angle(i);
            in[i].vel_rad_s = motors[i].status.speed;
            in[i].torque_nm = motors[i].status.torque;
            in[i].temp_c = motors[i].status.temperature;
            in[i].online = t - motors[i].status.last_rx_us < 150000;
            in[i].faults = motors[i].faults;
        }
        tilt_snap_t raw_f, raw_r;
        { float ax, ay, az;
          sim_accel_read(0, &ax, &ay, &az);
          raw_f = { atan2f(ay, az) * 57.2958f, atan2f(ax, az) * 57.2958f, true, t };
          sim_accel_read(1, &ax, &ay, &az);
          raw_r = { atan2f(ay, az) * 57.2958f, atan2f(ax, az) * 57.2958f, true, t }; }
        static tilt_snap_t tf = {}, tr = {};
        tilt_filter(&tf, &raw_f, dt, 0.25f);
        tilt_filter(&tr, &raw_r, dt, 0.25f);

        float trim_v[4] = {};
        if (lift_phase &&
            (fsm.state == MOTION_MOVING_UP || fsm.state == MOTION_MOVING_DOWN))
            travel_trim(&level_law, &tf, &tr, fabsf(fsm.group_v), trim_v);

        safety_result_t sr;
        safety_check(&safety, t, dt, in, v_prev, &tf, &tr, fsm.state, 24.5f,
                     SAFE_F_RACKING, &sr);
        if (sr.verdict == SAFE_TRIP) {
            printf("TRIP flags=0x%x at t=%.1fs — test FAIL\n", sr.flags, ts);
            return 1;
        }

        motion_fsm_step(&fsm, t, intent, vec, nullptr, trim_v, false, in, dt, &fsm_out);

        sim_power_set(fsm_out.ssr_on, fsm_out.lock_on);
        if (fsm_out.req_init && !prev_out.req_init)
            for (auto &m : motors) {
                cybergear_stop(&m);
                cybergear_set_mode(&m, CYBERGEAR_MODE_SPEED);
                cybergear_set_limit_current(&m, 7.0f);
            }
        if (fsm_out.req_enable && !prev_out.req_enable)
            for (auto &m : motors) cybergear_enable(&m);
        if (fsm_out.req_disable && !prev_out.req_disable)
            for (auto &m : motors) cybergear_stop(&m);
        prev_out = fsm_out;
        if (fsm_out.ssr_on && t - last_cmd >= 10000) {
            last_cmd = t;
            for (int i = 0; i < 4; i++)
                cybergear_set_speed(&motors[i], fsm_out.v_cmd[i]);
        }
        memcpy(v_prev, fsm_out.v_cmd, sizeof(v_prev));

        sim_step(t, dt);
        { twai_message_t m;
          while (sim_can_poll_rx(&m))
              for (int i = 0; i < 4; i++)
                  if (cybergear_process_message(&motors[i], &m, t) != ESP_ERR_NOT_FOUND)
                      break; }

        if (t - last_print >= 1000000) {
            last_print = t;
            printf("t=%4.1fs %-7s rollF %+6.2f rollR %+6.2f pitch %+6.2f "
                   "twist %+6.2f th[%5.2f %5.2f %5.2f %5.2f]\n",
                   ts, motion_state_name(fsm.state),
                   tf.roll_deg, tr.roll_deg,
                   0.5f * (tf.pitch_deg + tr.pitch_deg),
                   tf.roll_deg - tr.roll_deg,
                   sim_motor_angle(0), sim_motor_angle(1),
                   sim_motor_angle(2), sim_motor_angle(3));
        }

        if (ts > 19.5f && ts < 19.6f) {
            float p = 0.5f * (tf.pitch_deg + tr.pitch_deg);
            bool pass = fabsf(tf.roll_deg) < 1.0f && fabsf(tr.roll_deg) < 1.0f &&
                        fabsf(p) < 1.0f && fabsf(tf.roll_deg - tr.roll_deg) < 1.0f;
            printf("end of travel: rollF %+.2f rollR %+.2f pitch %+.2f "
                   "twist %+.2f -> %s\n",
                   tf.roll_deg, tr.roll_deg, p, tf.roll_deg - tr.roll_deg,
                   pass ? "PASS" : "FAIL");
            return pass ? 0 : 1;
        }
    }
    printf("timeout — FAIL\n");
    return 1;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "--test-travel") == 0)
        return run_travel_test();
    if (argc >= 2 && strcmp(argv[1], "--test-level") == 0)
        return run_level_test();
    if (argc >= 3 && strcmp(argv[1], "--snap") == 0) {
        snap_all(argv[2]);
        return 0;
    }
    return lgfx::Panel_sdl::main(user_func);
}

void loop()
{
    static int64_t last = now_us();
    static int64_t last_cmd = 0;
    static int frame_n = 0;

    int64_t t = now_us();
    float dt = (t - last) / 1e6f;
    last = t;
    if (dt > 0.1f) dt = 0.1f;

    const Uint8 *k = SDL_GetKeyboardState(nullptr);
    handle_bench_keys(k);

    // production button path: raw levels -> pure SM -> events
    btn_sm_step(&btn_sm, k[SDL_SCANCODE_UP], k[SDL_SCANCODE_M],
                k[SDL_SCANCODE_DOWN], t);
    apply_button_events();

    if (ui_view) {
        // APP mode: the controller owns power + choreography (product loop)
        app_step(t, dt);
    } else {
        // bench mode: manual P/K/E keys + direct speed command
        if (t - last_cmd > 10000) {
            last_cmd = t;
            if (ssr_on)
                for (auto &m : motors) cybergear_set_speed(&m, cmd_v);
        }
    }

    // 1 Hz VBUS poll while powered (real type-17 read path)
    static int64_t last_vbus = 0;
    if (ssr_on && t - last_vbus > 1000000) {
        last_vbus = t;
        for (auto &m : motors) cybergear_get_param(&m, CG_ADDR_VBUS);
    }

    sim_step(t, dt);
    pump_rx();

    if (++frame_n % 2 == 0) {
        if (ui_view) draw_ui();
        else draw();
    }
    lgfx::delay(16);
}
