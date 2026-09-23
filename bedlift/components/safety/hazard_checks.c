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

static uint32_t s_warn_only;   // set per call; flags herein cap at WARN

static void raise(safety_result_t *r, safety_verdict_e v, uint32_t flag)
{
    r->flags |= flag;
    if ((flag & s_warn_only) && v > SAFE_WARN) v = SAFE_WARN;
    if (v > r->verdict) r->verdict = v;
}

void safety_check(safety_ctx_t *c, int64_t now, float dt_s,
                  const motion_motor_in_t motor[SYS_NUM_MOTORS],
                  const float v_cmd_prev[SYS_NUM_MOTORS],
                  const tilt_snap_t *tf, const tilt_snap_t *tr,
                  motion_state_e motion, float vbus_v,
                  uint32_t warn_only_mask, safety_result_t *out)
{
    memset(out, 0, sizeof(*out));
    s_warn_only = warn_only_mask;
    bool moving = moving_state(motion);

    // H7 baseline: expected angle per motor, integrated from the COMMANDED
    // velocities — correct for differential (pitch/roll/twist/jog) moves,
    // where deviation-from-group-mean would false-trip by design.
    if (moving && !c->theta_ref_valid) {
        for (int i = 0; i < SYS_NUM_MOTORS; i++)
            c->theta_ref[i] = motor[i].theta_rad;
        c->theta_ref_valid = true;
    } else if (moving) {
        for (int i = 0; i < SYS_NUM_MOTORS; i++)
            c->theta_ref[i] += v_cmd_prev[i] * dt_s;
    }
    if (!moving) c->theta_ref_valid = false;

    // ---- H1: bus undervoltage ----------------------------------------------
    // This firmware serves no VBUS read (SDO reads + scope both non-functional,
    // bench 2026-09-23), so the ONLY undervoltage signal is the motor's own UV
    // fault bit in the feedback frame. vbus_v is unused (kept for a future
    // rev-B ADC divider). Any motor asserting UV -> stop while moving, warn
    // otherwise; drives the battery-warning glyph.
    (void)vbus_v;
    for (int i = 0; i < SYS_NUM_MOTORS; i++)
        if (motor[i].faults & CG_FAULT_UNDERVOLTAGE)
            raise(out, moving ? SAFE_STOP : SAFE_WARN, SAFE_F_UNDERVOLT);

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
        // Compared against a command ENVELOPE that decays at a plausible
        // decel rate, so a motor coasting down after its command dropped to
        // zero (end of pawl-unload, entry to leveling) is not a "runaway" —
        // but sustained/growing speed with no command still trips.
        float cmd = fabsf(v_cmd_prev[i]);
        float env = c->cmd_env[i] - 3.0f * dt_s;    // slow decay tolerates coast-down
        if (cmd > env) env = cmd;
        c->cmd_env[i] = env;
        float vel = fabsf(m->vel_rad_s);
        bool over = vel > c->vel_abs_max ||
                    (vel > env * c->overspeed_factor + 0.5f);
        if (over) {
            if (++c->overspeed_count[i] >= c->overspeed_samples)
                raise(out, SAFE_TRIP, SAFE_F_OVERSPEED);
        } else {
            c->overspeed_count[i] = 0;
        }

        online_n++;
    }

    // ---- H7: corner desync — SPREAD of tracking errors, not absolute lag ----
    // The motors lag the commanded velocity roughly equally (uniform tracking
    // error is normal); desync is one CORNER diverging from the others. So we
    // flag the spread (max-min) of per-motor tracking error, which is ~0 when
    // all lag together and only grows when a single corner sticks/runs ahead.
    if (moving && c->theta_ref_valid && online_n == SYS_NUM_MOTORS) {
        float emax = -1e9f, emin = 1e9f;
        for (int i = 0; i < SYS_NUM_MOTORS; i++) {
            float e = motor[i].theta_rad - c->theta_ref[i];
            if (e > emax) emax = e;
            if (e < emin) emin = e;
        }
        if (emax - emin > c->pos_desync_max_rad)
            raise(out, SAFE_STOP, SAFE_F_DESYNC);
    }

    // ---- H6: frame racking (with a physical-plausibility gate) --------------
    // Tilt beyond what the frame can physically reach is a sensor problem
    // (loose/unmounted accel), not real racking — treat as a tilt fault and
    // don't stop on it. Real racking within the plausible range still trips.
    bool tilt_plausible = tf->valid && tr->valid &&
        fabsf(tf->roll_deg) < TILT_MAX_VALID_DEG &&
        fabsf(tr->roll_deg) < TILT_MAX_VALID_DEG &&
        fabsf(tf->pitch_deg) < TILT_MAX_VALID_DEG &&
        fabsf(tr->pitch_deg) < TILT_MAX_VALID_DEG;

    if (tilt_plausible) {
        float rack = fabsf(tf->roll_deg - tr->roll_deg);
        if (rack > c->rack_trip_deg)
            raise(out, moving ? SAFE_STOP : SAFE_WARN, SAFE_F_RACKING);
        else if (rack > c->rack_warn_deg)
            raise(out, SAFE_WARN, SAFE_F_RACKING);
    } else {
        // no trustworthy tilt: warn, and forbid leveling (needs real tilt)
        raise(out, (moving && motion == MOTION_LEVELING) ? SAFE_STOP : SAFE_WARN,
              SAFE_F_TILT_FAIL);
    }
}
