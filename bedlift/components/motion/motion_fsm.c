#include <math.h>
#include <string.h>
#include "motion_fsm.h"

void motion_fsm_init(motion_fsm_t *f)
{
    memset(f, 0, sizeof(*f));
    f->v_cruise = 2.0f;
    f->a_max = 2.0f;
    f->a_stop = 4.0f;
    f->v_unload = 3.0f;
    f->theta_unload = 0.12f;
    f->v_settle = 1.0f;
    f->v_level_max = 1.0f;
    f->seat_torque = 2.0f;   // seat when |torque| reaches ~2 N*m against the pawl
    f->t_boot_timeout_us = 4000 * 1000;   // failsafe upper bound, not a wait
    f->t_unload_max_us = 350 * 1000;
    f->t_unlock_us = 150 * 1000;
    f->t_settle_max_us = 1000 * 1000;
    f->t_keepwarm_us = 2 * 1000 * 1000;   // depower SSRs after 2 s of inaction
    f->state = MOTION_IDLE;
}

void motion_fsm_fault(motion_fsm_t *f) { f->state = MOTION_FAULT; }

void motion_fsm_ack(motion_fsm_t *f)
{
    if (f->state == MOTION_FAULT) f->state = MOTION_IDLE;
}

static void enter(motion_fsm_t *f, motion_state_e s, int64_t now)
{
    f->state = s;
    f->t_entry = now;
}

static float slew(float v, float target, float rate, float dt)
{
    float dv = rate * dt;
    if (v < target - dv) return v + dv;
    if (v > target + dv) return v - dv;
    return target;
}

void motion_fsm_step(motion_fsm_t *f, int64_t now, motion_intent_e intent,
                     const float vec[SYS_NUM_MOTORS],
                     const float level_v[SYS_NUM_MOTORS],
                     const float trim_v[SYS_NUM_MOTORS],
                     bool level_done,
                     const motion_motor_in_t in[SYS_NUM_MOTORS],
                     float dt_s, motion_out_t *out)
{
    memset(out, 0, sizeof(*out));
    int64_t age = now - f->t_entry;

    if (intent != MI_LEVEL) f->level_complete = false;

    switch (f->state) {

    case MOTION_FAULT:
        // outputs stay all-off/safe; caller handles ack
        f->group_v = 0;
        break;

    case MOTION_IDLE:
        f->group_v = 0;
        f->init_done = false;
        if (intent != MI_NONE) enter(f, MOTION_POWER_UP, now);
        break;

    case MOTION_POWER_UP: {
        // Poll, don't guess: hold SSR on and probe until every motor answers,
        // then init once and go. No fixed boot wait — proceed the instant the
        // motors are up. FAULT only if they don't answer within the failsafe.
        out->ssr_on = true;
        bool all_online = true;
        for (int i = 0; i < SYS_NUM_MOTORS; i++)
            if (!in[i].online) all_online = false;

        if (!all_online) {
            out->req_ping = true;                     // caller pings (throttled)
            if (age > f->t_boot_timeout_us) f->state = MOTION_FAULT;
            break;
        }
        if (!f->init_done) {
            out->req_init = true;                     // stop -> mode -> limits
            f->init_done = true;
            f->t_entry = now;                         // time the init settle
            break;
        }
        if (age > 60000) enter(f, MOTION_READY, now); // brief settle after init
    }
        break;

    case MOTION_READY: {
        out->ssr_on = true;
        f->group_v = 0;
        if (intent == MI_MOVE && vec) {
            bool desc = false;
            for (int i = 0; i < SYS_NUM_MOTORS; i++) {
                f->vec[i] = vec[i];
                if (vec[i] < -0.01f) desc = true;
            }
            f->descending = desc;
            out->req_enable = true;
            if (!desc) {
                enter(f, MOTION_UNLOCK, now);   // pure raise: unlock + go together
            } else {
                for (int i = 0; i < SYS_NUM_MOTORS; i++) {
                    f->theta_start[i] = in[i].theta_rad;
                    f->unload_adv[i] = 0.0f;
                    f->unloaded[i] = false;
                }
                enter(f, MOTION_PAWL_UNLOAD, now);
            }
        } else if (intent == MI_LEVEL && !f->level_complete) {
            f->descending = true;               // leveling may lower corners
            out->req_enable = true;
            for (int i = 0; i < SYS_NUM_MOTORS; i++) {
                f->theta_start[i] = in[i].theta_rad;
                f->unload_adv[i] = 0.0f;
                f->unloaded[i] = false;
            }
            enter(f, MOTION_PAWL_UNLOAD, now);
        } else if (age >= f->t_keepwarm_us) {
            enter(f, MOTION_POWER_DOWN, now);
        }
        break;
    }

    case MOTION_PAWL_UNLOAD: {
        out->ssr_on = true;
        // Drive up until each motor has advanced theta_unload of rotation above
        // the latch. Prefer ACTUAL position (theta - theta_start) — the motors
        // were usually just powered, so position is fresh and unsaturated. Only
        // when the position field is saturated near its +/-12.5 rad range (top
        // of travel) fall back to integrating measured velocity over this short
        // move, where the tiny drift is negligible.
        bool all = true;
        for (int i = 0; i < SYS_NUM_MOTORS; i++) {
            if (!f->unloaded[i]) {
                float adv;
                if (fabsf(in[i].theta_rad) < 12.0f) {
                    adv = in[i].theta_rad - f->theta_start[i];
                } else {
                    f->unload_adv[i] += fabsf(in[i].vel_rad_s) * dt_s;
                    adv = f->unload_adv[i];
                }
                if (adv >= f->theta_unload) f->unloaded[i] = true;
            }
            out->v_cmd[i] = f->unloaded[i] ? 0 : f->v_unload;
            if (!f->unloaded[i]) all = false;
        }
        if (all || age >= f->t_unload_max_us)
            enter(f, MOTION_UNLOCK, now);
        if (intent == MI_NONE) {                 // released during unload
            enter(f, MOTION_RAMP_DOWN, now);
        }
        break;
    }

    case MOTION_UNLOCK:
        out->ssr_on = true;
        out->lock_on = true;                     // energize: retract pawls
        if (intent == MI_LEVEL) {
            if (age >= f->t_unlock_us) enter(f, MOTION_LEVELING, now);
        } else if (!f->descending) {
            // pure raise drives away from the pawl: go immediately
            enter(f, MOTION_MOVING_UP, now);
        } else if (age >= f->t_unlock_us) {
            enter(f, MOTION_MOVING_DOWN, now);
        }
        if (intent == MI_NONE) enter(f, MOTION_RAMP_DOWN, now);
        break;

    case MOTION_MOVING_UP:
    case MOTION_MOVING_DOWN: {
        out->ssr_on = true;
        out->lock_on = true;
        // continue while intent holds AND the requested vector is unchanged
        bool want = (intent == MI_MOVE) && vec;
        if (want)
            for (int i = 0; i < SYS_NUM_MOTORS; i++)
                if (fabsf(vec[i] - f->vec[i]) > 0.01f) want = false;
        float target = want ? f->v_cruise : 0.0f;
        f->group_v = slew(f->group_v, target, f->a_max, dt_s);
        for (int i = 0; i < SYS_NUM_MOTORS; i++) {
            float trim = trim_v ? trim_v[i] : 0.0f;
            out->v_cmd[i] = f->group_v * f->vec[i] + trim;
        }
        if (!want) enter(f, MOTION_RAMP_DOWN, now);
        break;
    }

    case MOTION_LEVELING:
        out->ssr_on = true;
        out->lock_on = true;
        for (int i = 0; i < SYS_NUM_MOTORS; i++) {
            float v = level_v ? level_v[i] : 0.0f;
            if (v > f->v_level_max) v = f->v_level_max;
            if (v < -f->v_level_max) v = -f->v_level_max;
            out->v_cmd[i] = v;
            f->vec[i] = 0;               // settle ramp uses zero vector
        }
        if (level_done) {
            // within target and untwisted: finish the sequence on our own —
            // ramp down, settle onto pawls, depower. The user just holds.
            f->level_complete = true;
            enter(f, MOTION_RAMP_DOWN, now);
        }
        if (intent != MI_LEVEL) enter(f, MOTION_RAMP_DOWN, now);
        break;

    case MOTION_RAMP_DOWN: {
        out->ssr_on = true;
        out->lock_on = true;                     // stay unlocked while slowing
        f->group_v = slew(f->group_v, 0.0f, f->a_stop, dt_s);
        for (int i = 0; i < SYS_NUM_MOTORS; i++)
            out->v_cmd[i] = f->group_v * f->vec[i];
        if (fabsf(f->group_v) < 0.05f) {
            for (int i = 0; i < SYS_NUM_MOTORS; i++) {
                f->seated[i] = false;
                f->seat_moved[i] = false;
                f->seat_since[i] = 0;
            }
            enter(f, MOTION_SETTLE, now);
        }
        break;
    }

    case MOTION_SETTLE: {
        out->ssr_on = true;
        out->lock_on = false;                    // drop pawls
        bool all = true;
        for (int i = 0; i < SYS_NUM_MOTORS; i++) {
            if (!f->seated[i]) {
                // Lowering onto the pawl transfers the load to the pawl, which
                // UNLOADS the motor (torque falls, bench-confirmed) — so the
                // seat signature is a velocity STALL, not a torque rise. Accept
                // the stall only after the corner has actually descended, so we
                // don't false-seat on the zero-crossing at settle entry.
                if (fabsf(in[i].vel_rad_s) > 0.3f) f->seat_moved[i] = true;
                bool stalled = f->seat_moved[i] && fabsf(in[i].vel_rad_s) < 0.12f;
                if (stalled) {
                    if (f->seat_since[i] == 0) f->seat_since[i] = now;
                    if (now - f->seat_since[i] > 30000) f->seated[i] = true;
                } else {
                    f->seat_since[i] = 0;
                }
            }
            out->v_cmd[i] = f->seated[i] ? 0 : -f->v_settle;
            if (!f->seated[i]) all = false;
        }
        if (all || age >= f->t_settle_max_us) {
            out->req_disable = true;
            enter(f, MOTION_READY, now);
        }
        break;
    }

    case MOTION_POWER_DOWN:
        out->ssr_on = false;
        out->req_disable = true;
        enter(f, MOTION_IDLE, now);
        break;
    }

    out->state = f->state;
}
