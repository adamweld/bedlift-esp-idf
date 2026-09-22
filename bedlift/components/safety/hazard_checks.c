#include <math.h>
#include <string.h>
#include "hazard_checks.h"
#include "cybergear_defs.h"

void safety_init(safety_ctx_t *c)
{
    memset(c, 0, sizeof(*c));
    c->vbus_warn_v = 21.0f;         // 6S Li-ion
    c->vbus_lockout_v = 19.5f;
    c->vel_abs_max = 8.0f;
    c->overspeed_factor = 1.5f;
    c->overspeed_samples = 3;
    c->telem_loss_us = 150 * 1000;
    c->temp_stop_c = 70.0f;
    c->rack_warn_deg = RACK_WARN_DEG;
    c->rack_trip_deg = RACK_TRIP_DEG;
    c->pos_desync_max_rad = 2.0f;
}

static bool moving_state(motion_state_e m)
{
    return m == MOTION_PAWL_UNLOAD || m == MOTION_UNLOCK ||
           m == MOTION_MOVING_UP || m == MOTION_MOVING_DOWN ||
           m == MOTION_LEVELING || m == MOTION_RAMP_DOWN || m == MOTION_SETTLE;
}

static void raise(safety_result_t *r, safety_verdict_e v, uint32_t flag)
{
    r->flags |= flag;
    if (v > r->verdict) r->verdict = v;
}

void safety_check(safety_ctx_t *c, int64_t now,
                  const motion_motor_in_t motor[SYS_NUM_MOTORS],
                  const float v_cmd_prev[SYS_NUM_MOTORS],
                  const tilt_snap_t *tf, const tilt_snap_t *tr,
                  motion_state_e motion, float vbus_v,
                  safety_result_t *out)
{
    memset(out, 0, sizeof(*out));
    bool moving = moving_state(motion);

    // latch per-motor references at the start of a move (H7 baseline)
    if (moving && !c->theta_ref_valid) {
        for (int i = 0; i < SYS_NUM_MOTORS; i++)
            c->theta_ref[i] = motor[i].theta_rad;
        c->theta_ref_valid = true;
    }
    if (!moving) c->theta_ref_valid = false;

    // ---- H1: bus undervoltage (gate; only meaningful when vbus fresh) ------
    if (vbus_v > 1.0f) {
        if (vbus_v < c->vbus_lockout_v)
            raise(out, moving ? SAFE_STOP : SAFE_WARN, SAFE_F_UNDERVOLT);
        else if (vbus_v < c->vbus_warn_v)
            raise(out, SAFE_WARN, SAFE_F_UNDERVOLT);
    }

    float mean_dtheta = 0;
    int online_n = 0;

    for (int i = 0; i < SYS_NUM_MOTORS; i++) {
        const motion_motor_in_t *m = &motor[i];

        // ---- H2: motor-reported faults ------------------------------------
        if (m->faults) {
            uint32_t severe = CG_FAULT_OVERCURRENT | CG_FAULT_OC_PHASE_A |
                              CG_FAULT_OC_PHASE_B | CG_FAULT_OC_PHASE_C |
                              CG_FAULT_DRIVER_CHIP;
            if (m->faults & severe) {
                raise(out, moving ? SAFE_TRIP : SAFE_WARN, SAFE_F_MOTOR_FAULT);
                out->skip_brake_window = true;   // suspect driver: cut, don't brake
            } else {
                // UV / thermal / encoder: controlled stop is enough
                raise(out, moving ? SAFE_STOP : SAFE_WARN, SAFE_F_MOTOR_FAULT);
            }
        }

        // ---- H5 (partial): temperature ------------------------------------
        if (m->temp_c > c->temp_stop_c)
            raise(out, moving ? SAFE_STOP : SAFE_WARN, SAFE_F_THERMAL);

        if (!moving) continue;   // the rest arm only while moving

        // ---- H4: telemetry loss -> treated as runaway ----------------------
        int64_t last_seen = 0;
        // caller encodes freshness in 'online' (checked against its own clock);
        // a motor that has gone silent mid-move is indistinguishable from a
        // runaway — worst case assumed.
        (void)last_seen;
        if (!m->online) {
            raise(out, SAFE_TRIP, SAFE_F_TELEM_LOSS);
            continue;
        }

        // ---- H3: overspeed / runaway ---------------------------------------
        // Trips when a motor exceeds the absolute ceiling, or runs well past
        // its own command (1.5x + a small floor so tiny commands don't
        // false-trip on noise). N consecutive samples required.
        float cmd = fabsf(v_cmd_prev[i]);
        float vel = fabsf(m->vel_rad_s);
        bool over = vel > c->vel_abs_max ||
                    (vel > cmd * c->overspeed_factor + 0.3f);
        if (over) {
            if (++c->overspeed_count[i] >= c->overspeed_samples)
                raise(out, SAFE_TRIP, SAFE_F_OVERSPEED);
        } else {
            c->overspeed_count[i] = 0;
        }

        mean_dtheta += m->theta_rad - c->theta_ref[i];
        online_n++;
    }

    // ---- H7: corner desync (relative progress since motion start) ----------
    if (moving && c->theta_ref_valid && online_n == SYS_NUM_MOTORS) {
        mean_dtheta /= online_n;
        for (int i = 0; i < SYS_NUM_MOTORS; i++) {
            float dev = fabsf((motor[i].theta_rad - c->theta_ref[i]) - mean_dtheta);
            if (dev > c->pos_desync_max_rad)
                raise(out, SAFE_STOP, SAFE_F_DESYNC);
        }
    }

    // ---- H6: frame racking --------------------------------------------------
    if (tf->valid && tr->valid) {
        float rack = fabsf(tf->roll_deg - tr->roll_deg);
        if (rack > c->rack_trip_deg)
            raise(out, moving ? SAFE_STOP : SAFE_WARN, SAFE_F_RACKING);
        else if (rack > c->rack_warn_deg)
            raise(out, SAFE_WARN, SAFE_F_RACKING);
    } else if (moving && motion == MOTION_LEVELING) {
        // ---- H10: leveling without valid tilt is not allowed ---------------
        raise(out, SAFE_STOP, SAFE_F_TILT_FAIL);
    }
}
