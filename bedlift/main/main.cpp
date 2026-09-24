// Bedlift production firmware — target integration.
//
// Boots safety-first, then wires the sim-proven logic modules to real
// hardware: dual ADXL tilt, TWAI CyberGear fleet, three buttons, ST7789 UI.
//
// SAFETY: motion is gated behind MOTION_ARMED. With it 0 (default), the full
// stack runs — real UI, real bubble level, buttons, mode tree — but the
// motion FSM never powers the SSR or commands velocity, so the bed cannot
// move. Set to 1 only when the bed is clear to move, then reflash.

#include <cstdio>
#include <cstring>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "app_config.hpp"
#include "lgfx_config.hpp"
#include "cybergear.h"
#include "canbus.h"
#include "sensors.h"
#include "state_store.h"
#include "buttons_sm.h"
#include "motion_fsm.h"
#include "bed_geometry.h"
#include "control_law.h"
#include "hazard_checks.h"
#include "ui_panels.hpp"

#define MOTION_ARMED 1   // 1 = allow the bed to move. Reflash to change.

static const char *TAG = "bedlift";

// Per-module debug spew, each under its own tag so they can be silenced or
// enabled independently via esp_log_level_set():
//   motion — per-cycle motor telemetry [state] cmd/fbv/tq/th (always on)
//   acc    — raw accelerometer readout (hidden for now; re-enable by setting
//            its level back to INFO)
// Safety warnings (TAG, ESP_LOGW) are never gated.

static LGFX display;
static LGFX_Sprite frame(&display);

static const uint8_t k_motor_ids[SYS_NUM_MOTORS] = MOTOR_CAN_IDS;
static const float k_motor_dir[SYS_NUM_MOTORS] = MOTOR_DIR_SIGN;

// shared control-side state (owned by the motion/app tasks)
static motion_fsm_t s_fsm;
static safety_ctx_t s_safety;
static level_law_t s_level;
static btn_sm_t s_btn;

static volatile app_mode_e s_mode = APP_MODE_LIFT;
static volatile bool s_debug_screen = false;
static volatile bool s_up = false, s_down = false, s_level_held = false;
static volatile uint32_t s_latched = 0;

// Per-mode "up" vector, filled at boot from bed_geometry.h (the single source
// of truth). LIFT/SIMPLE = group up; PITCH/ROLL/TWIST = the derived axes;
// M1..M4 = single-winch jog. build_mode_vectors() runs once in app_main.
static float k_vec_up[APP_MODE_COUNT][SYS_NUM_MOTORS];

static void build_mode_vectors(void)
{
    for (int i = 0; i < SYS_NUM_MOTORS; i++) {
        k_vec_up[APP_MODE_LIFT][i]   = bed_axis_vec(BED_AXIS_LIFT,  i);
        k_vec_up[APP_MODE_SIMPLE][i] = bed_axis_vec(BED_AXIS_LIFT,  i);
        k_vec_up[APP_MODE_PITCH][i]  = bed_axis_vec(BED_AXIS_PITCH, i);
        k_vec_up[APP_MODE_ROLL][i]   = bed_axis_vec(BED_AXIS_ROLL,  i);
        k_vec_up[APP_MODE_TWIST][i]  = bed_axis_vec(BED_AXIS_TWIST, i);
        k_vec_up[APP_MODE_M1][i]     = (i == 0) ? 1.0f : 0.0f;
        k_vec_up[APP_MODE_M2][i]     = (i == 1) ? 1.0f : 0.0f;
        k_vec_up[APP_MODE_M3][i]     = (i == 2) ? 1.0f : 0.0f;
        k_vec_up[APP_MODE_M4][i]     = (i == 3) ? 1.0f : 0.0f;
    }
}

// ---------------------------------------------------------------------------
// boot: force power gates low before anything else (H9 fail-safe posture)
// ---------------------------------------------------------------------------
static void outputs_safe(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_SSR_EN) | (1ULL << PIN_LOCK_EN),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);
    gpio_set_level((gpio_num_t)PIN_MOTOR_SSR_EN, 0);
    gpio_set_level((gpio_num_t)PIN_LOCK_EN, 0);
}

static void set_ssr(bool on)  { gpio_set_level((gpio_num_t)PIN_MOTOR_SSR_EN, on); }
static void set_lock(bool on) { gpio_set_level((gpio_num_t)PIN_LOCK_EN, on); }

static const char *motion_state_str(motion_state_e m)
{
    static const char *n[] = { "idle", "pwrup", "ready", "unload", "unlock",
                               "up", "down", "level", "ramp", "settle",
                               "pwrdn", "FAULT" };
    return (m >= 0 && m <= MOTION_FAULT) ? n[m] : "?";
}

// ---------------------------------------------------------------------------
// buttons: poll raw levels, drive the pure SM, apply events to mode/intent
// ---------------------------------------------------------------------------
static void apply_btn_event(const btn_event_t *e)
{
    switch (e->type) {
        case BEV_DOWN:
            if (e->id == BTN_UP) s_up = true;
            if (e->id == BTN_DOWN) s_down = true;
            break;
        case BEV_UP:
            if (e->id == BTN_UP) s_up = false;
            if (e->id == BTN_DOWN) s_down = false;
            if (e->id == BTN_MODE) s_level_held = false;
            break;
        case BEV_SHORT:
            if (e->id == BTN_MODE) {
                static int64_t t1 = 0, t2 = 0;
                static app_mode_e pre = APP_MODE_LIFT;
                if (e->t_us - t2 > 900000) pre = s_mode;
                if (t1 && e->t_us - t1 < 900000) {
                    s_debug_screen = !s_debug_screen; s_mode = pre; t1 = t2 = 0; break;
                }
                t1 = t2; t2 = e->t_us;
                switch (s_mode) {
                    case APP_MODE_SIMPLE: s_mode = APP_MODE_ROLL; break;
                    case APP_MODE_ROLL:  s_mode = APP_MODE_PITCH; break;
                    case APP_MODE_PITCH: s_mode = APP_MODE_TWIST; break;
                    case APP_MODE_TWIST: s_mode = APP_MODE_SIMPLE; break;
                    case APP_MODE_M1: s_mode = APP_MODE_M2; break;
                    case APP_MODE_M2: s_mode = APP_MODE_M3; break;
                    case APP_MODE_M3: s_mode = APP_MODE_M4; break;
                    case APP_MODE_M4: s_mode = APP_MODE_M1; break;
                    default: break;
                }
            }
            break;
        case BEV_HOLD:
            if (e->id == BTN_MODE) {
                if (s_fsm.state == MOTION_FAULT) { motion_fsm_ack(&s_fsm); s_latched = 0; }
                else if (s_mode != APP_MODE_LIFT) s_mode = APP_MODE_LIFT;
                else s_level_held = true;
            }
            break;
        case BEV_CHORD_UPDOWN:
            s_up = s_down = false;
            switch (app_mode_group(s_mode)) {
                case GROUP_DEFAULT: s_mode = APP_MODE_ROLL; break;
                case GROUP_MANUAL:  s_mode = APP_MODE_M1; break;
                case GROUP_DEBUG:   s_mode = APP_MODE_ROLL; break;
            }
            break;
        default: break;
    }
}

static void button_task(void *arg)
{
    // D0 active-low, D1/D2 active-high
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PIN_BTN_DOWN) | (1ULL << PIN_BTN_MODE) | (1ULL << PIN_BTN_UP),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&in);
    gpio_set_pull_mode((gpio_num_t)PIN_BTN_DOWN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)PIN_BTN_MODE, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode((gpio_num_t)PIN_BTN_UP, GPIO_PULLDOWN_ONLY);
    btn_sm_init(&s_btn);

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        bool up = gpio_get_level((gpio_num_t)PIN_BTN_UP);       // active high
        bool mode = gpio_get_level((gpio_num_t)PIN_BTN_MODE);   // active high
        bool down = !gpio_get_level((gpio_num_t)PIN_BTN_DOWN);  // active low
        btn_sm_step(&s_btn, up, mode, down, esp_timer_get_time());
        btn_event_t e;
        while (btn_sm_poll(&s_btn, &e)) apply_btn_event(&e);

        uint8_t mask = (s_btn.b[BTN_UP].stable ? 1 : 0) |
                       (s_btn.b[BTN_MODE].stable ? 2 : 0) |
                       (s_btn.b[BTN_DOWN].stable ? 4 : 0);
        state_set_buttons(mask);
        state_set_mode(s_mode, s_debug_screen);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
    }
}

// ---------------------------------------------------------------------------
// sensors: 50 Hz tilt + halls -> state
// ---------------------------------------------------------------------------
static void sensor_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    int64_t last_log = 0;
    for (;;) {
        sensors_update(0.02f);
        sensor_tilt_t f, r;
        sensors_get_tilt(&f, &r);

        // raw accel readout at 2 Hz — for the orientation tilt test
        int64_t now = esp_timer_get_time();
        if (now - last_log > 500000) {
            last_log = now;
            float fx, fy, fz, rx, ry, rz;
            sensors_get_raw(0, &fx, &fy, &fz);
            sensors_get_raw(1, &rx, &ry, &rz);
            ESP_LOGI("acc", "FRONT x%+.2f y%+.2f z%+.2f | REAR x%+.2f y%+.2f z%+.2f",
                     fx, fy, fz, rx, ry, rz);
        }
        tilt_snap_t tf = { f.pitch_deg, f.roll_deg, f.valid, esp_timer_get_time() };
        tilt_snap_t tr = { r.pitch_deg, r.roll_deg, r.valid, esp_timer_get_time() };
        state_set_tilt(true, &tf);
        state_set_tilt(false, &tr);
        state_set_halls(sensors_hall1(), sensors_hall2());
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
    }
}

// ---------------------------------------------------------------------------
// motion: 100 Hz safety -> FSM -> apply. Sole SSR/lock/velocity commander.
// ---------------------------------------------------------------------------

// Boot profiling: timestamps relative to SSR-close, logged once per power cycle.
static struct {
    int64_t ssr_close;
    int64_t motor_first_rx[SYS_NUM_MOTORS];
    int64_t all_online;
    int64_t init_done;
    bool logged;
} s_boot_prof;

static void boot_prof_reset(int64_t now)
{
    s_boot_prof.ssr_close = now;
    for (int i = 0; i < SYS_NUM_MOTORS; i++) s_boot_prof.motor_first_rx[i] = 0;
    s_boot_prof.all_online = 0;
    s_boot_prof.init_done = 0;
    s_boot_prof.logged = false;
}

static void boot_prof_motor_online(int idx, int64_t now)
{
    if (!s_boot_prof.motor_first_rx[idx])
        s_boot_prof.motor_first_rx[idx] = now;
}

static void boot_prof_log(void)
{
    if (s_boot_prof.logged) return;
    s_boot_prof.logged = true;
    int64_t t0 = s_boot_prof.ssr_close;
    for (int i = 0; i < SYS_NUM_MOTORS; i++)
        ESP_LOGI(TAG, "boot: M%d online at +%lld ms", i + 1,
                 (long long)((s_boot_prof.motor_first_rx[i] - t0) / 1000));
    ESP_LOGI(TAG, "boot: all online +%lld ms, init done +%lld ms (total %lld ms)",
             (long long)((s_boot_prof.all_online - t0) / 1000),
             (long long)((s_boot_prof.init_done - t0) / 1000),
             (long long)((s_boot_prof.init_done - t0) / 1000));
}

static void apply_motion_out(const motion_out_t *o, const motion_out_t *prev,
                             int jog_idx)
{
    if (o->ssr_on && !prev->ssr_on) boot_prof_reset(esp_timer_get_time());
    set_ssr(o->ssr_on);
    set_lock(o->lock_on);
    if (o->req_ping) {
        static int64_t last_ping = 0;
        int64_t now = esp_timer_get_time();
        if (now - last_ping > 10000) {
            last_ping = now;
            if (jog_idx >= 0)
                cybergear_ping(canbus_motor(jog_idx));
            else
                for (int i = 0; i < SYS_NUM_MOTORS; i++) cybergear_ping(canbus_motor(i));
        }
    }
    if (o->req_init && !prev->req_init) {
        s_boot_prof.all_online = esp_timer_get_time();
        int64_t t_init_start = esp_timer_get_time();
        int lo = (jog_idx >= 0) ? jog_idx : 0;
        int hi = (jog_idx >= 0) ? jog_idx + 1 : SYS_NUM_MOTORS;
        for (int i = lo; i < hi; i++) {
            cybergear_motor_t *m = canbus_motor(i);
            cybergear_stop(m);
            cybergear_set_mode(m, CYBERGEAR_MODE_SPEED);
            cybergear_set_limit_speed(m, MOTOR_LIMIT_SPEED_RADS);
            cybergear_set_limit_current(m, MOTOR_LIMIT_CURRENT_A);
            cybergear_set_limit_torque(m, MOTOR_LIMIT_TORQUE_NM);
        }
        s_boot_prof.init_done = esp_timer_get_time();
        ESP_LOGI(TAG, "init: %d SDO writes in %lld ms",
                 (hi - lo) * 5,
                 (long long)((s_boot_prof.init_done - t_init_start) / 1000));
        boot_prof_log();
    }
    if (o->req_enable && !prev->req_enable) {
        if (jog_idx >= 0)
            cybergear_enable(canbus_motor(jog_idx));
        else
            for (int i = 0; i < SYS_NUM_MOTORS; i++) cybergear_enable(canbus_motor(i));
    }
    if (o->req_disable && !prev->req_disable) {
        if (jog_idx >= 0)
            cybergear_stop(canbus_motor(jog_idx));
        else
            for (int i = 0; i < SYS_NUM_MOTORS; i++) cybergear_stop(canbus_motor(i));
    }
}

static void motion_task(void *arg)
{
    motion_out_t out = {}, prev = {};
    float v_prev[SYS_NUM_MOTORS] = {};
    int64_t last_cmd = 0;
    TickType_t last = xTaskGetTickCount();
    const float dt = 0.01f;

    for (;;) {
        int64_t now = esp_timer_get_time();

        // gather motor telemetry from the fork objects
        motion_motor_in_t in[SYS_NUM_MOTORS];
        for (int i = 0; i < SYS_NUM_MOTORS; i++) {
            cybergear_status_t st; uint32_t f;
            canbus_motor_snapshot(i, &st, &f);
            // motor-frame -> bed-frame (+ = up); unwrapped multi-turn angle
            in[i].theta_rad = k_motor_dir[i] * st.position_unwrapped;
            in[i].vel_rad_s = k_motor_dir[i] * st.speed;
            in[i].torque_nm = k_motor_dir[i] * st.torque;
            in[i].temp_c = st.temperature;
            in[i].online = out.ssr_on && (now - st.last_rx_us < TELEM_LOSS_MS * 1000);
            in[i].faults = f;
            if (in[i].online) boot_prof_motor_online(i, now);
        }

        // tilt from state
        sys_snapshot_t snap;
        state_snapshot(&snap);
        tilt_snap_t tf = snap.tilt_front, tr = snap.tilt_rear;

        // intent (gated by arming)
        motion_intent_e intent = MI_NONE;
        float vec[SYS_NUM_MOTORS] = {};
        float level_v[SYS_NUM_MOTORS] = {};
        float trim_v[SYS_NUM_MOTORS] = {};
        bool jog_mode = (app_mode_group(s_mode) == GROUP_DEBUG);
        if (jog_mode) {
            int ji = s_mode - APP_MODE_M1;
            for (int i = 0; i < SYS_NUM_MOTORS; i++) {
                if (i == ji) continue;
                in[i].online = in[ji].online;
                in[i].vel_rad_s = 0;
                in[i].torque_nm = 0;
                in[i].faults = 0;
            }
        }
#if MOTION_ARMED
        if (jog_mode) {
            if (s_up != s_down) {
                intent = MI_MOVE;
                int ji = s_mode - APP_MODE_M1;
                vec[ji] = s_up ? 1.0f : -1.0f;
            }
        } else if (s_level_held && s_mode == APP_MODE_LIFT) {
            intent = MI_LEVEL;
        } else if (s_up != s_down) {
            intent = MI_MOVE;
            float sign = s_up ? 1.0f : -1.0f;
            for (int i = 0; i < SYS_NUM_MOTORS; i++) vec[i] = sign * k_vec_up[s_mode][i];
        }
        if (intent == MI_LEVEL || s_fsm.state == MOTION_LEVELING)
            level_control(&s_level, &tf, &tr, level_v);
        if (s_mode == APP_MODE_LIFT &&
            (s_fsm.state == MOTION_MOVING_UP || s_fsm.state == MOTION_MOVING_DOWN))
            travel_trim(&s_level, &tf, &tr, fabsf(s_fsm.group_v), trim_v);
#endif
        bool level_done = level_within(&tf, &tr, s_level.done_deg);

        // safety first
        uint32_t warn_only = (app_mode_group(s_mode) != GROUP_DEFAULT) ? SAFE_F_RACKING : 0;
        safety_result_t sr;
        safety_check(&s_safety, now, dt, in, v_prev, &tf, &tr, s_fsm.state,
                     0.0f /* vbus unused: fw<1.2.1.5, use fault bits */, warn_only, &sr);
        if (sr.verdict == SAFE_TRIP && s_fsm.state != MOTION_FAULT) {
            set_lock(false); set_ssr(false);
            for (int i = 0; i < SYS_NUM_MOTORS; i++) cybergear_stop(canbus_motor(i));
            motion_fsm_fault(&s_fsm);
            s_latched |= sr.flags;
        } else if (sr.verdict == SAFE_STOP) {
            s_latched |= sr.flags;
            intent = MI_NONE;
        }

        // Diagnostic: dump hazard detail when flags change (rising edge) AND
        // pre-trip breadcrumbs at 2 Hz when counters are elevated.
        static uint32_t last_flags = 0;
        if (sr.flags != last_flags) {
            uint32_t added = sr.flags & ~last_flags;
            if (added) {
                ESP_LOGW(TAG, "SAFETY v=%d flags=0x%03lx [%s]", sr.verdict,
                         (unsigned long)sr.flags, motion_state_str(s_fsm.state));
                for (int i = 0; i < SYS_NUM_MOTORS; i++)
                    ESP_LOGW(TAG, "  M%d cmd=%+.2f env=%+.2f fbv=%+.2f verr=%+.2f osc=%d/%d",
                             i + 1, v_prev[i], s_safety.cmd_env[i], in[i].vel_rad_s,
                             s_safety.vel_err[i], s_safety.overspeed_count[i],
                             s_safety.overspeed_samples);
                float ve_max = -1e9f, ve_min = 1e9f;
                int worst = 0;
                for (int i = 0; i < SYS_NUM_MOTORS; i++) {
                    if (s_safety.vel_err[i] > ve_max) { ve_max = s_safety.vel_err[i]; worst = i; }
                    if (s_safety.vel_err[i] < ve_min) ve_min = s_safety.vel_err[i];
                }
                ESP_LOGW(TAG, "  desync_cnt=%d/%d spread=%.2f (lim %.2f) worst=M%d "
                         "th[%+.1f %+.1f %+.1f %+.1f]",
                         s_safety.desync_count, s_safety.desync_samples,
                         ve_max - ve_min, s_safety.vel_desync_max, worst + 1,
                         in[0].theta_rad, in[1].theta_rad,
                         in[2].theta_rad, in[3].theta_rad);
                ESP_LOGW(TAG, "  tilt F=%+.1f R=%+.1f twist=%+.1f",
                         tf.roll_deg, tr.roll_deg, tf.roll_deg - tr.roll_deg);
            }
            last_flags = sr.flags;
        }

        // Pre-trip breadcrumbs (2 Hz): log when overspeed or desync counters
        // are non-zero but haven't tripped yet — shows the buildup in the log.
        {
            static int64_t last_bc = 0;
            bool elevated = s_safety.desync_count > 0;
            for (int i = 0; i < SYS_NUM_MOTORS && !elevated; i++)
                if (s_safety.overspeed_count[i] > 0) elevated = true;
            if (elevated && now - last_bc > 500000) {
                last_bc = now;
                ESP_LOGW("safety", "PRE [%s] osc[%d %d %d %d] desync=%d "
                         "verr[%+.2f %+.2f %+.2f %+.2f] "
                         "cmd[%+.2f %+.2f %+.2f %+.2f] env[%+.2f %+.2f %+.2f %+.2f] "
                         "fbv[%+.2f %+.2f %+.2f %+.2f]",
                         motion_state_str(s_fsm.state),
                         s_safety.overspeed_count[0], s_safety.overspeed_count[1],
                         s_safety.overspeed_count[2], s_safety.overspeed_count[3],
                         s_safety.desync_count,
                         s_safety.vel_err[0], s_safety.vel_err[1],
                         s_safety.vel_err[2], s_safety.vel_err[3],
                         v_prev[0], v_prev[1], v_prev[2], v_prev[3],
                         s_safety.cmd_env[0], s_safety.cmd_env[1],
                         s_safety.cmd_env[2], s_safety.cmd_env[3],
                         in[0].vel_rad_s, in[1].vel_rad_s,
                         in[2].vel_rad_s, in[3].vel_rad_s);
            }
        }

        mode_group_e grp = app_mode_group(s_mode);
        s_fsm.v_cruise = (grp == GROUP_MANUAL || grp == GROUP_DEBUG)
                         ? V_MANUAL_RAD_S : V_CRUISE_RAD_S;
        static motion_state_e prev_state = MOTION_IDLE;
        motion_fsm_step(&s_fsm, now, intent, vec, level_v, trim_v, level_done, in, dt, &out);
        // Single-motor jog: mask all motors except the selected one so
        // the FSM's unload/settle/move cycle only acts on that corner.
        if (jog_mode) {
            int ji = s_mode - APP_MODE_M1;
            for (int i = 0; i < SYS_NUM_MOTORS; i++)
                if (i != ji) out.v_cmd[i] = 0.0f;
        }
        if (s_fsm.state != prev_state) {
            ESP_LOGI(TAG, "FSM %s -> %s", motion_state_str(prev_state),
                     motion_state_str(s_fsm.state));
            prev_state = s_fsm.state;
        }
        int jog_idx = jog_mode ? (s_mode - APP_MODE_M1) : -1;
        if (s_fsm.state != MOTION_FAULT) apply_motion_out(&out, &prev, jog_idx);
        prev = out;

        // Always-on motor telemetry while active (4 Hz): cmd / envelope /
        // feedback vel / torque / angle per corner, plus vel-err and tilt.
        static int64_t last_tlm = 0;
        bool tlm_active = s_fsm.state != MOTION_IDLE &&
                          s_fsm.state != MOTION_READY;
        if (tlm_active && now - last_tlm > 250000) {
            last_tlm = now;
            ESP_LOGI("motion", "[%s] cmd[%+.2f %+.2f %+.2f %+.2f] "
                     "fbv[%+.2f %+.2f %+.2f %+.2f] "
                     "tq[%+.1f %+.1f %+.1f %+.1f] th[%+.1f %+.1f %+.1f %+.1f] "
                     "verr[%+.2f %+.2f %+.2f %+.2f] tiltF%+.1f R%+.1f",
                     motion_state_str(s_fsm.state),
                     out.v_cmd[0], out.v_cmd[1], out.v_cmd[2], out.v_cmd[3],
                     in[0].vel_rad_s, in[1].vel_rad_s, in[2].vel_rad_s, in[3].vel_rad_s,
                     in[0].torque_nm, in[1].torque_nm, in[2].torque_nm, in[3].torque_nm,
                     in[0].theta_rad, in[1].theta_rad, in[2].theta_rad, in[3].theta_rad,
                     s_safety.vel_err[0], s_safety.vel_err[1],
                     s_safety.vel_err[2], s_safety.vel_err[3],
                     tf.roll_deg, tr.roll_deg);
        }

        // 100 Hz per-motor velocity stream while powered
        if (out.ssr_on && now - last_cmd > 10000) {
            last_cmd = now;
            for (int i = 0; i < SYS_NUM_MOTORS; i++)   // bed-frame -> motor-frame
                cybergear_set_speed(canbus_motor(i), k_motor_dir[i] * out.v_cmd[i]);
        }
        memcpy(v_prev, out.v_cmd, sizeof(v_prev));

        // publish
        for (int i = 0; i < SYS_NUM_MOTORS; i++) {
            motor_snap_t ms = {};
            ms.online = in[i].online;
            ms.vel_rad_s = in[i].vel_rad_s;
            ms.torque_nm = in[i].torque_nm;
            ms.temp_c = in[i].temp_c;
            ms.theta_rad = in[i].theta_rad;
            ms.faults = in[i].faults;
            ms.last_rx_us = 0;
            state_set_motor(i, &ms);
        }
        state_set_power(out.ssr_on, out.lock_on);
        state_set_motion(s_fsm.state, s_latched | (sr.verdict == SAFE_WARN ? sr.flags : 0));

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}

// ---------------------------------------------------------------------------
// ui: snapshot -> render -> push
// ---------------------------------------------------------------------------
static void ui_task(void *arg)
{
    frame.setColorDepth(16);
    frame.createSprite(UI_W, UI_H);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        sys_snapshot_t s;
        state_snapshot(&s);
        ui_render(frame, s);
        frame.pushSprite(0, 0);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));   // 20 fps
    }
}

// ---------------------------------------------------------------------------
static void display_init(void)
{
    gpio_config_t pwr = { .pin_bit_mask = (1ULL << PIN_I2C_POWER),
                          .mode = GPIO_MODE_INPUT_OUTPUT };
    gpio_config(&pwr);
    gpio_set_level((gpio_num_t)PIN_I2C_POWER, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    display.init();
    display.setRotation(TFT_SCREEN_ROTATION);
    display.setBrightness(BACKLIGHT_FULL);
}

extern "C" void app_main(void)
{
    outputs_safe();
    build_mode_vectors();
    ESP_LOGI(TAG, "boot: outputs safe; wake %d; MOTION_ARMED=%d",
             (int)esp_sleep_get_wakeup_cause(), MOTION_ARMED);

    int fails = cybergear_selftest_run();
    ESP_LOGI(TAG, "cybergear selftest failures: %d", fails);

    display_init();
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_WHITE);
    display.setTextDatum(lgfx::middle_center);
    display.setTextSize(2);
    display.drawString("BEDLIFT", display.width() / 2, display.height() / 2 - 10);
    display.setTextSize(1);
    display.drawString(MOTION_ARMED ? "ARMED" : "monitor (motion off)",
                       display.width() / 2, display.height() / 2 + 12);
    vTaskDelay(pdMS_TO_TICKS(800));

    esp_log_level_set("acc", ESP_LOG_INFO);   // accel debug ON for offset calibration

    state_init();
    motion_fsm_init(&s_fsm);
    safety_init(&s_safety);
    level_law_init(&s_level);

    // app_config.hpp is the single tuning surface: apply its speeds/limits over
    // the module defaults so travel speed, ramps, and the leveling clamp change
    // in one place. (Leveling PI gains live in level_law_init.)
    s_fsm.v_cruise     = V_CRUISE_RAD_S;
    s_fsm.a_max        = A_MAX_RAD_S2;
    s_fsm.a_stop       = A_STOP_RAD_S2;
    s_fsm.v_level_max  = V_LEVEL_MAX_RAD_S;
    s_fsm.v_settle     = V_SETTLE_RAD_S;
    s_fsm.seat_torque  = SEAT_TORQUE_NM;      // seat force against the pawl
    s_fsm.v_unload     = V_UNLOAD_RAD_S;
    s_fsm.theta_unload = THETA_UNLOAD_RAD;    // rotation above the latch to unseat
    s_fsm.t_unload_max_us = (int64_t)T_UNLOAD_TIMEOUT_MS * 1000;
    s_fsm.t_settle_max_us = (int64_t)T_SETTLE_TIMEOUT_MS * 1000;
    s_fsm.t_keepwarm_us   = (int64_t)T_READY_KEEPWARM_MS * 1000;
    s_level.v_max      = V_LEVEL_MAX_RAD_S;
    s_safety.vel_abs_max      = VEL_ABS_MAX_RAD_S;
    s_safety.overspeed_factor = OVERSPEED_FACTOR;

    sensors_cfg_t sc = {
        .sda = PIN_I2C_SDA, .scl = PIN_I2C_SCL, .i2c_power_pin = PIN_I2C_POWER,
        .acc_sdo_pin = PIN_ACC_SDO, .hall1_pin = PIN_HALL_1, .hall2_pin = PIN_HALL_2,
        .addr_front = I2C_ADDR_ACC_FRONT, .addr_rear = I2C_ADDR_ACC_REAR,
        .lp_tau_s = 0.25f,
    };
    sensors_init(&sc);
    canbus_init(PIN_CAN_TX, PIN_CAN_RX, CG_MASTER_ID, k_motor_ids);

    xTaskCreatePinnedToCore(motion_task, "motion", 6144, NULL, 20, NULL, 1);
    xTaskCreatePinnedToCore(sensor_task, "sensor", 4096, NULL, 15, NULL, 0);
    xTaskCreatePinnedToCore(button_task, "button", 4096, NULL, 18, NULL, 0);
    xTaskCreatePinnedToCore(ui_task, "ui", 8192, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "tasks up; %s", MOTION_ARMED ? "MOTION ARMED" : "monitor mode");
}
