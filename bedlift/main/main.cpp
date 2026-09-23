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
#include "control_law.h"
#include "hazard_checks.h"
#include "ui_panels.hpp"

#define MOTION_ARMED 1   // 1 = allow the bed to move. Reflash to change.

static const char *TAG = "bedlift";

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

static const float k_vec_up[APP_MODE_COUNT][SYS_NUM_MOTORS] = {
    MVEC_LIFT_UP, MVEC_LIFT_UP,                 // LIFT, SIMPLE (both all-up)
    MVEC_PITCH_POS, MVEC_ROLL_POS, MVEC_TWIST_POS,
    { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 },
};

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
                    case APP_MODE_SIMPLE: s_mode = APP_MODE_PITCH; break;
                    case APP_MODE_PITCH: s_mode = APP_MODE_ROLL; break;
                    case APP_MODE_ROLL:  s_mode = APP_MODE_TWIST; break;
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
                case GROUP_DEFAULT: s_mode = APP_MODE_SIMPLE; break;
                case GROUP_MANUAL:  s_mode = APP_MODE_M1; break;
                case GROUP_DEBUG:   s_mode = APP_MODE_SIMPLE; break;
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
    for (;;) {
        sensors_update(0.02f);
        sensor_tilt_t f, r;
        sensors_get_tilt(&f, &r);
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
static void apply_motion_out(const motion_out_t *o, const motion_out_t *prev)
{
    static int64_t ssr_on_us = 0;
    if (o->ssr_on && !prev->ssr_on) ssr_on_us = esp_timer_get_time();
    set_ssr(o->ssr_on);
    set_lock(o->lock_on);
    // presence probe: ping all motors, throttled to ~20 Hz (motion runs 100 Hz)
    if (o->req_ping) {
        static int64_t last_ping = 0;
        int64_t now = esp_timer_get_time();
        if (now - last_ping > 50000) {
            last_ping = now;
            for (int i = 0; i < SYS_NUM_MOTORS; i++) cybergear_ping(canbus_motor(i));
        }
    }
    if (o->req_init && !prev->req_init) {
        // measured, not assumed: how long from SSR-close to all motors online
        ESP_LOGI(TAG, "motors online %lld ms after SSR close",
                 (long long)((esp_timer_get_time() - ssr_on_us) / 1000));
        for (int i = 0; i < SYS_NUM_MOTORS; i++) {
            // proven order (old motor.cpp): stop -> mode -> speed -> current -> torque
            cybergear_motor_t *m = canbus_motor(i);
            cybergear_stop(m); vTaskDelay(pdMS_TO_TICKS(2));
            cybergear_set_mode(m, CYBERGEAR_MODE_SPEED); vTaskDelay(pdMS_TO_TICKS(2));
            cybergear_set_limit_speed(m, MOTOR_LIMIT_SPEED_RADS); vTaskDelay(pdMS_TO_TICKS(2));
            cybergear_set_limit_current(m, MOTOR_LIMIT_CURRENT_A); vTaskDelay(pdMS_TO_TICKS(2));
            cybergear_set_limit_torque(m, MOTOR_LIMIT_TORQUE_NM);
        }
    }
    if (o->req_enable && !prev->req_enable)
        for (int i = 0; i < SYS_NUM_MOTORS; i++) cybergear_enable(canbus_motor(i));
    if (o->req_disable && !prev->req_disable)
        for (int i = 0; i < SYS_NUM_MOTORS; i++) cybergear_stop(canbus_motor(i));
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
#if MOTION_ARMED
        if (s_level_held && s_mode == APP_MODE_LIFT) intent = MI_LEVEL;
        else if (s_up != s_down) {
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

        // Diagnostic: dump hazard detail once, only when the hazard SET changes
        // (rising edge), so a latched fault doesn't spam. Quiet otherwise.
        static uint32_t last_flags = 0;
        if (sr.flags != last_flags) {
            uint32_t added = sr.flags & ~last_flags;
            if (added) {
                ESP_LOGW(TAG, "SAFETY v=%d flags=0x%03lx [%s]", sr.verdict,
                         (unsigned long)sr.flags, motion_state_str(s_fsm.state));
                for (int i = 0; i < SYS_NUM_MOTORS; i++)
                    ESP_LOGW(TAG, "  M%d cmd=%+.2f fbv=%+.2f dev=%+.2f osc=%d",
                             i + 1, v_prev[i], in[i].vel_rad_s,
                             in[i].theta_rad - s_safety.theta_ref[i],
                             s_safety.overspeed_count[i]);
                ESP_LOGW(TAG, "  tilt F=%+.1f R=%+.1f twist=%+.1f",
                         tf.roll_deg, tr.roll_deg, tf.roll_deg - tr.roll_deg);
            }
            last_flags = sr.flags;
        }

        motion_fsm_step(&s_fsm, now, intent, vec, level_v, trim_v, level_done, in, dt, &out);
        if (s_fsm.state != MOTION_FAULT) apply_motion_out(&out, &prev);
        prev = out;

        // periodic telemetry while active (1 Hz) — normal-operation visibility
        static int64_t last_tlm = 0;
        if (s_fsm.state != MOTION_IDLE && s_fsm.state != MOTION_READY &&
            now - last_tlm > 1000000) {
            last_tlm = now;
            ESP_LOGI(TAG, "[%s] cmd[%+.2f %+.2f %+.2f %+.2f] fbv[%+.2f %+.2f %+.2f %+.2f] "
                     "th[%+.1f %+.1f %+.1f %+.1f] tiltF%+.1f R%+.1f",
                     motion_state_str(s_fsm.state),
                     out.v_cmd[0], out.v_cmd[1], out.v_cmd[2], out.v_cmd[3],
                     in[0].vel_rad_s, in[1].vel_rad_s, in[2].vel_rad_s, in[3].vel_rad_s,
                     in[0].theta_rad, in[1].theta_rad, in[2].theta_rad, in[3].theta_rad,
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

    state_init();
    motion_fsm_init(&s_fsm);
    safety_init(&s_safety);
    level_law_init(&s_level);

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
