#ifndef BEDLIFT_HAZARD_CHECKS_H
#define BEDLIFT_HAZARD_CHECKS_H

// Hazard rules (pure, platform-free). Runs at the top of every motion cycle
// — before the FSM and control law — per the plan's hazard table. The caller
// owns actuation: on a TRIP verdict it must de-energize LOCK_EN first (pawls
// are the unpowered brake), hold a short zero-speed braking window if CAN is
// alive, then drop MOTOR_EN, then latch the FSM into FAULT.

#include <stdbool.h>
#include <stdint.h>
#include "sys_state.h"
#include "motion_fsm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SAFE_OK = 0,
    SAFE_WARN,          // flag for UI, motion continues
    SAFE_STOP,          // request controlled stop (RAMP_DOWN -> SETTLE)
    SAFE_TRIP,          // immediate: locks -> brake window -> SSR off -> FAULT
} safety_verdict_e;

typedef struct {
    // tunables (defaults from safety_init)
    float vbus_warn_v, vbus_lockout_v;
    float vel_abs_max, overspeed_factor;
    int overspeed_samples;
    int64_t telem_loss_us;
    float temp_stop_c;
    float rack_warn_deg, rack_trip_deg;
    float pos_desync_max_rad;

    // internal
    int overspeed_count[SYS_NUM_MOTORS];
    float cmd_env[SYS_NUM_MOTORS];     // command envelope (tolerates decel)
    float theta_ref[SYS_NUM_MOTORS];   // command-integrated expected angle
    bool theta_ref_valid;
    uint32_t warn_only_mask;           // flags demoted to WARN (mode-dependent)
} safety_ctx_t;

typedef struct {
    safety_verdict_e verdict;
    uint32_t flags;         // SAFE_F_* bits describing what fired
    bool skip_brake_window; // true when the driver itself is suspect (H2 OC/driver)
} safety_result_t;

void safety_init(safety_ctx_t *c);

// Inputs: the motor telemetry the FSM also sees, the commanded velocities
// from the previous cycle, tilt, and the current motion state (arming).
// warn_only_mask: SAFE_F_* bits that never exceed WARN this cycle — manual
// and debug modes demote racking so the frame can be driven OUT of a bad
// mechanical state instead of being blocked by it.
void safety_check(safety_ctx_t *c, int64_t now_us, float dt_s,
                  const motion_motor_in_t motor[SYS_NUM_MOTORS],
                  const float v_cmd_prev[SYS_NUM_MOTORS],
                  const tilt_snap_t *tilt_front, const tilt_snap_t *tilt_rear,
                  motion_state_e motion, float vbus_v,
                  uint32_t warn_only_mask, safety_result_t *out);

#ifdef __cplusplus
}
#endif

#endif
