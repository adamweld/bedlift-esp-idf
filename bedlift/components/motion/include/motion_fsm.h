#ifndef BEDLIFT_MOTION_FSM_H
#define BEDLIFT_MOTION_FSM_H

// Motion state machine (pure, platform-free). Owns the power-gating and
// pawl choreography:
//   IDLE -> POWER_UP -> READY -> (UNLOCK_RAISE | PAWL_UNLOAD -> UNLOCK)
//        -> MOVING/LEVELING -> RAMP_DOWN -> SETTLE -> READY -> POWER_DOWN
// The caller (motion_task on target, the host loop in sim) applies outputs:
// SSR/lock states, motor init/enable/disable requests, per-motor velocity.

#include <stdbool.h>
#include <stdint.h>
#include "sys_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MI_NONE = 0,   // no motion requested (releases stop the bed)
    MI_MOVE,       // move along the caller-supplied per-motor vector
    MI_LEVEL,
} motion_intent_e;

// Motion direction vectors are NOT defined here — they derive from the single
// source of truth in bed_geometry.h (bed_axis_vec(BED_AXIS_*, i)). The FSM
// multiplies the slewed group velocity by the caller-supplied vector; any
// negative component routes through the pawl-unload path.

typedef struct {
    float theta_rad;    // cumulative angle (+ = up)
    float vel_rad_s;
    float torque_nm;
    float temp_c;
    bool online;
    uint32_t faults;
} motion_motor_in_t;

typedef struct {
    bool ssr_on;
    bool lock_on;
    bool req_ping;      // level: probe motors for presence (caller throttles)
    bool req_init;      // rising edge: run stop -> set_mode -> limits
    bool req_enable;    // rising edge: enable all motors
    bool req_disable;   // rising edge: stop/disable all motors
    float v_cmd[SYS_NUM_MOTORS];
    motion_state_e state;
} motion_out_t;

typedef struct {
    // tunables (defaults from motion_fsm_init; override before first step)
    float v_cruise;         // rad/s
    float a_max;            // rad/s^2 slew
    float a_stop;           // rad/s^2 stop slew
    float v_unload;         // pawl-unload drive (positive = up)
    float theta_unload;     // required advance per motor
    float v_settle;         // settle-onto-pawl drive (positive magnitude)
    float v_level_max;      // clamp on leveling velocities
    float seat_torque;      // |torque| indicating pawl contact
    int64_t t_boot_timeout_us; // give up waiting for motors -> FAULT (failsafe bound)
    int64_t t_unload_min_us;
    int64_t t_unload_max_us;
    int64_t t_unlock_us;    // solenoid retract dwell
    int64_t t_settle_max_us;
    int64_t t_keepwarm_us;  // READY -> POWER_DOWN

    // internal
    motion_state_e state;
    int64_t t_entry;
    float group_v;
    float vec[SYS_NUM_MOTORS];  // active direction vector
    bool descending;            // any negative component (unload path)
    float theta_start[SYS_NUM_MOTORS];  // pawl-unload angle reference (future: position path)
    float unload_adv[SYS_NUM_MOTORS];   // rotation advanced this unseat (velocity-integrated)
    bool unloaded[SYS_NUM_MOTORS];
    bool seated[SYS_NUM_MOTORS];
    bool seat_moved[SYS_NUM_MOTORS];    // corner descended before we accept a stall
    int64_t seat_since[SYS_NUM_MOTORS];
    bool init_done;
    bool level_complete;    // auto-completed; don't re-level until released
} motion_fsm_t;

void motion_fsm_init(motion_fsm_t *f);

// Latch a fault from the safety pass: outputs go safe, state -> MOTION_FAULT.
void motion_fsm_fault(motion_fsm_t *f);
// Acknowledge a fault: back to IDLE (caller re-runs diagnostics on next move).
void motion_fsm_ack(motion_fsm_t *f);

// vec: per-motor direction (-1..1), only read when intent == MI_MOVE.
// level_v: leveling-law velocities, read during MOTION_LEVELING (clamped).
// trim_v: travel-trim velocities added on top of group motion while MOVING
// (caller passes zeros/NULL for differential modes, where trim would fight
// the user's commanded tilt change).
// level_done: leveling auto-complete condition (level + untwisted, dwelled);
// when it goes true during LEVELING the FSM ramps down, settles onto the
// pawls and latches level_complete so a still-held LEVEL doesn't restart.
// Any pointer may be NULL when unused for the current intent.
void motion_fsm_step(motion_fsm_t *f, int64_t now_us, motion_intent_e intent,
                     const float vec[SYS_NUM_MOTORS],
                     const float level_v[SYS_NUM_MOTORS],
                     const float trim_v[SYS_NUM_MOTORS],
                     bool level_done,
                     const motion_motor_in_t in[SYS_NUM_MOTORS],
                     float dt_s, motion_out_t *out);

#ifdef __cplusplus
}
#endif

#endif
