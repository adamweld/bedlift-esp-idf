// Thread-safe SystemSnapshot store. Producers publish through field-scoped
// setters (short mutex holds); consumers take a coherent copy. One global
// instance — the whole app shares it.

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "state_store.h"

static sys_snapshot_t s_state;
static SemaphoreHandle_t s_mtx;

void state_init(void)
{
    s_mtx = xSemaphoreCreateMutex();
    memset(&s_state, 0, sizeof(s_state));
    s_state.halls_enabled = false;   // endstops not mechanically mounted yet
}

static inline void lock(void)   { xSemaphoreTake(s_mtx, portMAX_DELAY); }
static inline void unlock(void) { xSemaphoreGive(s_mtx); }

void state_set_motor(int i, const motor_snap_t *m)
{
    if (i < 0 || i >= SYS_NUM_MOTORS) return;
    lock(); s_state.motor[i] = *m; unlock();
}

void state_set_tilt(bool front, const tilt_snap_t *t)
{
    lock(); if (front) s_state.tilt_front = *t; else s_state.tilt_rear = *t; unlock();
}

void state_set_halls(bool top, bool bottom)
{
    lock(); s_state.hall_top = top; s_state.hall_bottom = bottom; unlock();
}

void state_set_lipo(float soc, float v)
{
    lock(); s_state.lipo_soc = soc; s_state.lipo_v = v; unlock();
}

void state_set_power(bool ssr_on, bool lock_energized)
{
    lock(); s_state.motor_ssr_on = ssr_on; s_state.lock_energized = lock_energized; unlock();
}

void state_set_motion(motion_state_e motion, uint32_t safety_flags)
{
    lock(); s_state.motion = motion; s_state.safety_flags = safety_flags; unlock();
}

void state_set_mode(app_mode_e mode, bool debug_screen)
{
    lock(); s_state.mode = mode; s_state.debug_screen = debug_screen; unlock();
}

void state_set_buttons(uint8_t pressed_mask)
{
    lock(); s_state.btn_pressed_mask = pressed_mask; unlock();
}

void state_set_solenoid(float budget_frac, uint32_t cooldown_s)
{
    lock(); s_state.sol_budget_frac = budget_frac; s_state.sol_cooldown_s = cooldown_s; unlock();
}

void state_snapshot(sys_snapshot_t *out)
{
    lock();
    *out = s_state;
    out->now_us = esp_timer_get_time();
    unlock();
}
