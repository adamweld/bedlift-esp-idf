#ifndef BEDLIFT_CONTROL_LAW_H
#define BEDLIFT_CONTROL_LAW_H

// Leveling control law (pure). Front and rear tilt sensors act on their own
// motor pairs (the frame twists — that is why the rear sensor exists):
//   front roll  -> FL vs FR differential
//   rear roll   -> RL vs RR differential
//   mean pitch  -> front pair vs rear pair
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
} level_law_t;

void level_law_init(level_law_t *l);

// Returns true when all three errors are inside the deadband (converged).
// v_out: per-motor velocity, order FL FR RL RR, + = up.
bool level_control(const level_law_t *l, const tilt_snap_t *front,
                   const tilt_snap_t *rear, float v_out[SYS_NUM_MOTORS]);

// First-order low-pass for tilt readings (the control input must never be a
// raw accel sample — winch vibration bounces it in and out of the deadband).
// `state` holds the filtered value; seed it from the first valid raw sample.
void tilt_filter(tilt_snap_t *state, const tilt_snap_t *raw,
                 float dt_s, float tau_s);

#ifdef __cplusplus
}
#endif

#endif
