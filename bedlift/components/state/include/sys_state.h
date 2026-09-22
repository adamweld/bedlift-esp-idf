#ifndef BEDLIFT_SYS_STATE_H
#define BEDLIFT_SYS_STATE_H

// SystemSnapshot: the one shared-state contract. Producers publish through
// field-scoped setters (target: mutex store in state.c, added with the task
// layer); consumers — UI above all — read a coherent copy. Pure header, no
// platform includes, so host tools share it.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SYS_NUM_MOTORS 4

// Shared leveling/racking constants (used by safety rules AND the UI).
// The system levels the bed with the vehicle parked up to ~20 deg off-level
// (bench-tested); front/rear accel disagreement to 10 deg is safe.
#define RACK_WARN_DEG 10.0f
#define RACK_TRIP_DEG 15.0f
#define LEVEL_RING_DEG 10.0f    // reference ring on the level display
#define LEVEL_RANGE_DEG 30.0f   // full-scale of the level display

typedef enum {
    MOTION_IDLE = 0,
    MOTION_POWER_UP,
    MOTION_READY,
    MOTION_PAWL_UNLOAD,
    MOTION_UNLOCK,
    MOTION_MOVING_UP,
    MOTION_MOVING_DOWN,
    MOTION_LEVELING,
    MOTION_RAMP_DOWN,
    MOTION_SETTLE,
    MOTION_POWER_DOWN,
    MOTION_FAULT,
} motion_state_e;

// Mode tree (chord = hold up+down; center short = cycle within group;
// center hold = back to LIFT (or level, when already in LIFT); triple-click
// center = toggle bubble/debug display):
//   LIFT --chord--> manual [PITCH ROLL TWIST] --chord--> debug [M1..M4]
//                                 ^-------------chord--------------'
typedef enum {
    APP_MODE_LIFT = 0,      // hold up/down to move, hold center to self-level
    APP_MODE_PITCH,         // front pair vs rear pair
    APP_MODE_ROLL,          // left pair vs right pair
    APP_MODE_TWIST,         // diagonal pairs (frame torsion)
    APP_MODE_M1,            // individual winch jog (debug group)
    APP_MODE_M2,
    APP_MODE_M3,
    APP_MODE_M4,
    APP_MODE_COUNT,
} app_mode_e;

typedef enum { GROUP_DEFAULT = 0, GROUP_MANUAL, GROUP_DEBUG } mode_group_e;

static inline mode_group_e app_mode_group(app_mode_e m)
{
    if (m == APP_MODE_LIFT) return GROUP_DEFAULT;
    if (m <= APP_MODE_TWIST) return GROUP_MANUAL;
    return GROUP_DEBUG;
}

// safety_flags bits (latched hazards; mirrors the plan's hazard table)
#define SAFE_F_UNDERVOLT   (1u << 0)
#define SAFE_F_MOTOR_FAULT (1u << 1)
#define SAFE_F_OVERSPEED   (1u << 2)
#define SAFE_F_TELEM_LOSS  (1u << 3)
#define SAFE_F_THERMAL     (1u << 4)
#define SAFE_F_RACKING     (1u << 5)
#define SAFE_F_DESYNC      (1u << 6)
#define SAFE_F_SOL_DUTY    (1u << 7)
#define SAFE_F_TILT_FAIL   (1u << 8)
#define SAFE_F_OVERTRAVEL  (1u << 9)

typedef struct {
    bool online;            // answered during this power session
    float vel_rad_s;
    float torque_nm;
    float temp_c;
    float theta_rad;        // unwrapped cumulative angle
    float vbus_v;           // last standby read; 0 = never read
    uint32_t faults;        // CG_FAULT_* bitmask
    int64_t last_rx_us;
} motor_snap_t;

typedef struct {
    float pitch_deg, roll_deg;
    bool valid;
    int64_t t_us;
} tilt_snap_t;

typedef struct {
    motor_snap_t motor[SYS_NUM_MOTORS];
    tilt_snap_t tilt_front, tilt_rear;
    bool hall_top, hall_bottom;
    bool halls_enabled;         // false until mechanically mounted
    float lipo_soc, lipo_v;
    bool motor_ssr_on;
    bool lock_energized;
    float sol_budget_frac;      // 0..1 remaining solenoid duty budget
    uint32_t sol_cooldown_s;    // 0 = usable now
    uint8_t btn_pressed_mask;   // bit0 up, bit1 mode, bit2 down (UI feedback)
    motion_state_e motion;
    app_mode_e mode;
    bool debug_screen;          // triple-click center: motor table vs bubble
    uint32_t safety_flags;
    int64_t now_us;             // timestamp the snapshot was taken
} sys_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif // BEDLIFT_SYS_STATE_H
