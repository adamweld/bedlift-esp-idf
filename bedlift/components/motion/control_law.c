#include <math.h>
#include <string.h>
#include "control_law.h"
#include "bed_geometry.h"   // single source of truth for corner polarity

void level_law_init(level_law_t *l)
{
    l->kp_roll = 0.15f;     // 1 deg error -> 0.15 rad/s corner correction
    l->kp_pitch = 0.15f;
    l->deadband_deg = 0.25f;
    l->v_max = 1.0f;
    l->done_deg = LEVEL_TARGET_DEG;  // auto-lock target = the inner ring on the UI
    l->trim_frac = 0.15f;
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

    // Negative feedback along the bed axes (corner polarity from bed_geometry.h):
    //   +pitch (front high) -> lower front / raise rear:  -kp_pitch * p  * fb[i]
    //   +roll  (right high) -> lower right / raise left :  -kp_roll  * r  * lr[i]
    // Front accel drives the FRONT pair's roll, rear accel the REAR pair's —
    // independent, so this de-twists. Pitch uses the mean of the two sensors.
    float rf = band(front->roll_deg, l->deadband_deg);
    float rr = band(rear->roll_deg, l->deadband_deg);
    float p = band(0.5f * (front->pitch_deg + rear->pitch_deg), l->deadband_deg);

    for (int i = 0; i < SYS_NUM_MOTORS; i++) {
        float r = bed_is_front(i) ? rf : rr;
        float v = -l->kp_roll * r * bed_lr(i) - l->kp_pitch * p * bed_fb(i);
        v_out[i] = clampf(with_floor(v), l->v_max);
    }

    return rf == 0 && rr == 0 && p == 0;
}

bool level_within(const tilt_snap_t *front, const tilt_snap_t *rear,
                  float target_deg)
{
    if (!front->valid || !rear->valid) return false;
    float p = 0.5f * (front->pitch_deg + rear->pitch_deg);
    float twist = front->roll_deg - rear->roll_deg;
    return fabsf(front->roll_deg) < target_deg &&
           fabsf(rear->roll_deg) < target_deg &&
           fabsf(p) < target_deg &&
           fabsf(twist) < target_deg;
}

void travel_trim(const level_law_t *l, const tilt_snap_t *front,
                 const tilt_snap_t *rear, float v_group_abs,
                 float trim_out[SYS_NUM_MOTORS])
{
    memset(trim_out, 0, sizeof(float) * SYS_NUM_MOTORS);
    if (!front->valid || !rear->valid) return;

    float rf = band(front->roll_deg, l->deadband_deg);
    float rr = band(rear->roll_deg, l->deadband_deg);
    float p = band(0.5f * (front->pitch_deg + rear->pitch_deg), l->deadband_deg);
    float lim = l->trim_frac * v_group_abs;

    // Identical mapping to level_control (bed_geometry.h), clamped to a
    // fraction of the group speed instead of v_max, and no minimum-speed floor.
    for (int i = 0; i < SYS_NUM_MOTORS; i++) {
        float r = bed_is_front(i) ? rf : rr;
        float v = -l->kp_roll * r * bed_lr(i) - l->kp_pitch * p * bed_fb(i);
        trim_out[i] = clampf(v, lim);
    }
}
