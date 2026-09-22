#ifndef BEDLIFT_SIM_H
#define BEDLIFT_SIM_H

// Virtual bedlift plant: four CyberGear winches, pawl locks, bed frame, 6S
// battery, ADXL tilt outputs. Target-agnostic (no FreeRTOS/ESP includes) —
// runs under the host SDL sim, host unit tests, and the on-device sim mode.
//
// CAN side: consumes the same frame layout the real motors do (built by the
// cybergear component) and produces protocol-faithful replies: type-2
// write-echo feedback, type-17 param replies, type-0 ping replies, type-21
// fault frames.
//
// Sign convention: positive motor angle/velocity = bed UP. Pawl semantics
// (ratchet): with the solenoid de-energized the pawl allows raising but
// blocks lowering; energizing retracts it. A pawl carrying load must be
// unloaded (short up-drive) before it can retract.

#include <stdbool.h>
#include <stdint.h>
#include "cg_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_NUM_MOTORS 4

typedef enum {
    SIM_FAULT_NONE = 0,
    SIM_FAULT_UNDERVOLTAGE,   // motor reports UV in echo + type-21
    SIM_FAULT_OVERCURRENT,    // phase OC fault, motor drops to reset state
    SIM_FAULT_DRIVER,         // driver-chip fault
    SIM_FAULT_COMM_LOSS,      // motor stops replying entirely
    SIM_FAULT_SNAG,           // strap snag: motor stalls (vel->0, iq climbs)
    SIM_FAULT_RUNAWAY,        // speed loop ignores command, accelerates down
} sim_fault_e;

typedef struct {
    // physics
    float drum_radius_m;      // base drum radius
    float wrap_radius_gain;   // radius grows by this fraction per motor rad
    float tau_s;              // speed-loop time constant
    float gravity_backdrive;  // rad/s^2 applied when unpowered + unlocked
    float travel_rad;         // motor rad from bottom to top endstop
    // battery
    float batt_v0;            // open-circuit voltage (6S: ~24.6 nominal)
    float batt_r;             // effective sag ohms (V drop per total amp)
    // sensors
    float accel_noise_g;      // vibration noise while any motor moves
} sim_config_t;

void sim_default_config(sim_config_t *cfg);
void sim_init(const sim_config_t *cfg);

// Advance physics + telemetry. Call at ~1 kHz (host loop or device timer).
void sim_step(int64_t now_us, float dt_s);

// ---- HAL surface the firmware talks to ------------------------------------
// CAN out from the firmware into the plant (transport callback target).
esp_err_t sim_can_tx(const twai_message_t *msg, TickType_t ticks, void *ctx);
// CAN in: drain queued motor replies. Returns true while frames remain.
bool sim_can_poll_rx(twai_message_t *out);
// Power gates (mirrors MOTOR_EN / LOCK_EN writes).
void sim_power_set(bool motor_ssr_on, bool lock_energized);
// Virtual ADXL sample in g (idx 0=front, 1=rear).
void sim_accel_read(int idx, float *x_g, float *y_g, float *z_g);
// Hall endstops (true = magnet present).
bool sim_hall_top(void);
bool sim_hall_bottom(void);

// ---- inspection / injection (test + hostsim hotkeys) -----------------------
void sim_inject(int motor_idx, sim_fault_e fault);   // SIM_FAULT_NONE clears
float sim_motor_angle(int motor_idx);                // cumulative rad
float sim_motor_vel(int motor_idx);
float sim_corner_height_m(int motor_idx);
float sim_bed_pitch_deg(void);
float sim_bed_roll_front_deg(void);
float sim_bed_roll_rear_deg(void);
float sim_vbus(void);
bool sim_pawl_seated(int motor_idx);

#ifdef __cplusplus
}
#endif

#endif // BEDLIFT_SIM_H
