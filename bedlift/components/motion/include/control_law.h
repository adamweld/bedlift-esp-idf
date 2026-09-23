#ifndef BEDLIFT_CONTROL_LAW_H
#define BEDLIFT_CONTROL_LAW_H

// Leveling control law (pure). Front and rear tilt sensors act on their own
// motor pairs (the frame twists — that is why the rear sensor exists):
//   front roll  -> front pair left/right differential
//   rear roll   -> rear pair left/right differential
//   mean pitch  -> front pair vs rear pair
// Corner polarity (which index is which corner) comes entirely from
// bed_geometry.h — this file assumes nothing about the layout.
// P-control with a deadband; conservative gains, tuned in Phase C.

#include "sys_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp_roll;      // rad/s of corner motion per degree of roll error
    float kp_pitch;
    float deadband_deg; // errors below this contribute nothing (anti-hunt)
    float v_max;        // per-motor clamp, rad/s
    float done_deg;     // leveling auto-completes inside this (level + twist)
    float trim_frac;    // travel trim authority as a fraction of |v_group|
} level_law_t;

void level_law_init(level_law_t *l);

// Returns true when all three errors are inside the deadband (converged).
// v_out: per-motor velocity indexed by logical motor (see bed_geometry.h), + = up.
bool level_control(const level_law_t *l, const tilt_snap_t *front,
                   const tilt_snap_t *rear, float v_out[SYS_NUM_MOTORS]);

// First-order low-pass for tilt readings (the control input must never be a
// raw accel sample — winch vibration bounces it in and out of the deadband).
// `state` holds the filtered value; seed it from the first valid raw sample.
void tilt_filter(tilt_snap_t *state, const tilt_snap_t *raw,
                 float dt_s, float tau_s);

// True when both rolls, the mean pitch AND the front/rear twist are all
// inside target_deg — the leveling auto-complete condition.
bool level_within(const tilt_snap_t *front, const tilt_snap_t *rear,
                  float target_deg);

// Tilt trim for uniform (LIFT) travel: same front/rear-independent mapping
// as leveling, scaled and clamped to trim_frac x |v_group| so the correction
// rides on top of the group motion and cancels level/twist drift en route.
// Returns zeros when tilt is invalid.
void travel_trim(const level_law_t *l, const tilt_snap_t *front,
                 const tilt_snap_t *rear, float v_group_abs,
                 float trim_out[SYS_NUM_MOTORS]);

#ifdef __cplusplus
}
#endif

#endif
