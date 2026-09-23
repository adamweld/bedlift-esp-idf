#ifndef BEDLIFT_STATE_STORE_H
#define BEDLIFT_STATE_STORE_H

// Thread-safe accessor layer over sys_snapshot_t. See state_store.c.

#include "sys_state.h"

#ifdef __cplusplus
extern "C" {
#endif

void state_init(void);

void state_set_motor(int i, const motor_snap_t *m);
void state_set_tilt(bool front, const tilt_snap_t *t);
void state_set_halls(bool top, bool bottom);
void state_set_lipo(float soc, float v);
void state_set_power(bool ssr_on, bool lock_energized);
void state_set_motion(motion_state_e motion, uint32_t safety_flags);
void state_set_mode(app_mode_e mode, bool debug_screen);
void state_set_buttons(uint8_t pressed_mask);
void state_set_solenoid(float budget_frac, uint32_t cooldown_s);

// Coherent copy; stamps now_us.
void state_snapshot(sys_snapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif
