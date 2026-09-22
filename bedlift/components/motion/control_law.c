#include <math.h>
#include <string.h>
#include "control_law.h"

void level_law_init(level_law_t *l)
{
    l->kp_roll = 0.15f;     // 1 deg error -> 0.15 rad/s corner correction
    l->kp_pitch = 0.15f;
    l->deadband_deg = 0.25f;
    l->v_max = 1.0f;
}

// Minimum correction speed outside the deadband: without it the P output
// asymptotes to zero at the deadband edge and final approach takes forever.
#define LEVEL_V_FLOOR 0.08f

static float with_floor(float v)
{
    if (v > 0 && v < LEVEL_V_FLOOR) return LEVEL_V_FLOOR;
    if (v < 0 && v > -LEVEL_V_FLOOR) return -LEVEL_V_FLOOR;
    return v;
}

void tilt_filter(tilt_snap_t *state, const tilt_snap_t *raw,
                 float dt_s, float tau_s)
{
    if (!raw->valid) { state->valid = false; return; }
    if (!state->valid) {
        *state = *raw;                       // seed from first valid sample
        return;
    }
    float a = dt_s / (tau_s + dt_s);
    state->pitch_deg += a * (raw->pitch_deg - state->pitch_deg);
    state->roll_deg += a * (raw->roll_deg - state->roll_deg);
    state->t_us = raw->t_us;
}

static float band(float e, float db)
{
    if (e > db) return e - db;
    if (e < -db) return e + db;
    return 0;
}

static float clampf(float v, float lim)
{
    if (v > lim) return lim;
    if (v < -lim) return -lim;
    return v;
}

bool level_control(const level_law_t *l, const tilt_snap_t *front,
                   const tilt_snap_t *rear, float v_out[SYS_NUM_MOTORS])
{
    memset(v_out, 0, sizeof(float) * SYS_NUM_MOTORS);
    if (!front->valid || !rear->valid) return false;

    // roll: + = right side high -> raise left / lower right of that pair
    float rf = band(front->roll_deg, l->deadband_deg);
    float rr = band(rear->roll_deg, l->deadband_deg);
    // pitch: + = front high -> lower front pair / raise rear pair
    float p = band(0.5f * (front->pitch_deg + rear->pitch_deg), l->deadband_deg);

    v_out[0] = clampf(with_floor(+l->kp_roll * rf - l->kp_pitch * p), l->v_max);
    v_out[1] = clampf(with_floor(-l->kp_roll * rf - l->kp_pitch * p), l->v_max);
    v_out[2] = clampf(with_floor(+l->kp_roll * rr + l->kp_pitch * p), l->v_max);
    v_out[3] = clampf(with_floor(-l->kp_roll * rr + l->kp_pitch * p), l->v_max);

    return rf == 0 && rr == 0 && p == 0;
}
