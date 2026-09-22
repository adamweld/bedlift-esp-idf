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

// Direction vectors (motor order 0=FL 1=FR 2=RL 3=RR; + = up). The FSM
// multiplies the slewed group velocity by the vector; any negative component
// routes through the pawl-unload path. Signs are sim conventions — bench
// verification before M9 decides real-world polarity per corner.
#define MVEC_LIFT_UP    { +1, +1, +1, +1 }
#define MVEC_LIFT_DOWN  { -1, -1, -1, -1 }
#define MVEC_PITCH_POS  { +1, +1, -1, -1 }   // nose up
#define MVEC_ROLL_POS   { +1, -1, +1, -1 }   // left side up
#define MVEC_TWIST_POS  { +1, -1, -1, +1 }   // diagonal torsion

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
    float seat_torque;      // |torque| indicating pawl contact
    int64_t t_boot_us;      // SSR-close -> motors ready
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
    float theta_start[SYS_NUM_MOTORS];
    bool unloaded[SYS_NUM_MOTORS];
    bool seated[SYS_NUM_MOTORS];
    int64_t seat_since[SYS_NUM_MOTORS];
    bool init_done;
} motion_fsm_t;

void motion_fsm_init(motion_fsm_t *f);

// Latch a fault from the safety pass: outputs go safe, state -> MOTION_FAULT.
void motion_fsm_fault(motion_fsm_t *f);
// Acknowledge a fault: back to IDLE (caller re-runs diagnostics on next move).
void motion_fsm_ack(motion_fsm_t *f);

// vec: per-motor direction (-1..1), only read when intent == MI_MOVE; may be
// NULL for MI_NONE / MI_LEVEL.
void motion_fsm_step(motion_fsm_t *f, int64_t now_us, motion_intent_e intent,
                     const float vec[SYS_NUM_MOTORS],
                     const motion_motor_in_t in[SYS_NUM_MOTORS],
                     float dt_s, motion_out_t *out);

#ifdef __cplusplus
}
#endif

#endif
