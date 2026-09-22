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
    f->seat_torque = 1.5f;
    f->t_boot_us = 1500 * 1000;
    f->t_unload_max_us = 350 * 1000;
    f->t_unlock_us = 100 * 1000;
    f->t_settle_max_us = 1000 * 1000;
    f->t_keepwarm_us = 5 * 1000 * 1000;   // depower SSRs after 5 s of inaction
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
                     const motion_motor_in_t in[SYS_NUM_MOTORS],
                     float dt_s, motion_out_t *out)
{
    memset(out, 0, sizeof(*out));
    int64_t age = now - f->t_entry;

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

    case MOTION_POWER_UP:
        out->ssr_on = true;
        if (age >= f->t_boot_us) {
            if (!f->init_done) {
                out->req_init = true;      // stop -> speed mode -> limits
                f->init_done = true;
                break;
            }
            // require every motor to be answering before arming motion
            bool all_online = true;
            for (int i = 0; i < SYS_NUM_MOTORS; i++)
                if (!in[i].online) all_online = false;
            if (all_online) enter(f, MOTION_READY, now);
            else if (age > f->t_boot_us + 2000000) {
                // presence probe failed — treat as fault (caller shows which)
                f->state = MOTION_FAULT;
            }
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
                    f->unloaded[i] = false;
                }
                enter(f, MOTION_PAWL_UNLOAD, now);
            }
        } else if (intent == MI_LEVEL) {
            f->descending = true;               // leveling may lower corners
            out->req_enable = true;
            for (int i = 0; i < SYS_NUM_MOTORS; i++) {
                f->theta_start[i] = in[i].theta_rad;
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
        // drive up until each motor's angle has advanced past its pawl seat
        bool all = true;
        for (int i = 0; i < SYS_NUM_MOTORS; i++) {
            if (!f->unloaded[i] &&
                in[i].theta_rad - f->theta_start[i] >= f->theta_unload)
                f->unloaded[i] = true;
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
        for (int i = 0; i < SYS_NUM_MOTORS; i++)
            out->v_cmd[i] = f->group_v * f->vec[i];  // control-law trims: M10
        if (!want) enter(f, MOTION_RAMP_DOWN, now);
        break;
    }

    case MOTION_LEVELING:
        out->ssr_on = true;
        out->lock_on = true;
        // leveling control law lands in M10; hold at zero for now
        for (int i = 0; i < SYS_NUM_MOTORS; i++) out->v_cmd[i] = 0;
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
                // stall signature: commanded down, not moving, torque rising
                bool contact = fabsf(in[i].torque_nm) > f->seat_torque &&
                               fabsf(in[i].vel_rad_s) < 0.15f;
                if (contact) {
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
