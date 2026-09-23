#ifndef CYBERGEAR_H
#define CYBERGEAR_H

// Forked from bedlift (PlatformIO) lib/cybergear, itself derived from
// cybergear-robotics/cybergear. Protocol fixes applied per the tiered list in
// the bedlift control-system plan; see cybergear_defs.h for per-fix notes.

#include <stdbool.h>
#include <stdint.h>

#include "cg_compat.h"
#include "cybergear_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float position;          // rad, packed-field (wraps at ±CG_POS_RANGE)
    float position_unwrapped; // rad, cumulative multi-turn (wrap-tracked in RX)
    int   pos_wraps;         // net wrap count (± CG_POS_RANGE crossings)
    bool  pos_init;          // false until the first feedback seeds the unwrap
    float speed;             // rad/s, ±30
    float torque;            // Nm, ±12
    float temperature;       // °C
    cybergear_state_e state;
    int64_t last_rx_us;      // esp_timer time of last processed feedback frame
} cybergear_status_t;

typedef struct {
    float position, speed, torque, kp, kd;
} cybergear_motion_cmd_t;

typedef struct {
    uint16_t run_mode;
    float iq_ref, spd_ref, limit_torque;
    float cur_kp, cur_ki, cur_filt_gain;
    float loc_ref, limit_spd, limit_cur;
    float mech_pos, iqf, mech_vel, vbus;
    int16_t rotation;
    float loc_kp, spd_kp, spd_ki;
    uint16_t last_index;  // index of the most recent param reply
    bool updated;
    int64_t last_rx_us;
} cybergear_params_t;

typedef struct {
    uint8_t master_can_id;
    uint8_t can_id;
    TickType_t transmit_ticks_to_wait;
    cybergear_params_t params;
    cybergear_status_t status;
    uint32_t faults;        // CG_FAULT_* bitmask (live echo bits + last type-21)
    uint32_t fault_word;    // raw type-21 fault word
    uint32_t warn_word;     // raw type-21 warning word
    uint64_t mcu_uid;       // from ping reply
    int64_t ping_rx_us;
} cybergear_motor_t;

// Optional frame tap for protocol characterization ("CAN snoop" instrument):
// called for every TX (dir=0) and every processed RX (dir=1).
typedef void (*cybergear_frame_tap_t)(int dir, const twai_message_t *msg);
void cybergear_set_frame_tap(cybergear_frame_tap_t tap);

// Injectable transport: the library never touches TWAI directly. On target,
// the canbus component registers a twai_transmit-based sender; in sim/host
// builds, the virtual plant registers its own. Unset transport -> commands
// return ESP_ERR_INVALID_STATE.
typedef esp_err_t (*cybergear_send_fn_t)(const twai_message_t *msg,
                                         TickType_t ticks_to_wait, void *ctx);
void cybergear_set_transport(cybergear_send_fn_t send, void *ctx);

// ---- lifecycle -------------------------------------------------------------
esp_err_t cybergear_init(cybergear_motor_t *motor, uint8_t master_can_id,
                         uint8_t can_id, TickType_t transmit_ticks_to_wait);

// ---- commands --------------------------------------------------------------
esp_err_t cybergear_ping(cybergear_motor_t *motor);          // type 0 (bench-verified probe)
esp_err_t cybergear_enable(cybergear_motor_t *motor);
esp_err_t cybergear_stop(cybergear_motor_t *motor);          // type 4, data0=0
esp_err_t cybergear_clear_fault(cybergear_motor_t *motor);   // type 4, data0=1 (new; test A4)
esp_err_t cybergear_set_mode(cybergear_motor_t *motor, cybergear_mode_e mode);
esp_err_t cybergear_set_motor_can_id(cybergear_motor_t *motor, uint8_t can_id);
esp_err_t cybergear_set_mech_position_to_zero(cybergear_motor_t *motor);
esp_err_t cybergear_get_param(cybergear_motor_t *motor, uint16_t index); // type 17 read

esp_err_t cybergear_set_limit_speed(cybergear_motor_t *motor, float speed);
esp_err_t cybergear_set_limit_current(cybergear_motor_t *motor, float current);
esp_err_t cybergear_set_limit_torque(cybergear_motor_t *motor, float torque);

esp_err_t cybergear_set_motion_cmd(cybergear_motor_t *motor, const cybergear_motion_cmd_t *cmd);
esp_err_t cybergear_set_current_kp(cybergear_motor_t *motor, float kp);
esp_err_t cybergear_set_current_ki(cybergear_motor_t *motor, float ki);
esp_err_t cybergear_set_current_filter_gain(cybergear_motor_t *motor, float gain);
esp_err_t cybergear_set_current(cybergear_motor_t *motor, float current);
esp_err_t cybergear_set_position_kp(cybergear_motor_t *motor, float kp);
esp_err_t cybergear_set_position(cybergear_motor_t *motor, float position); // raw float loc_ref
esp_err_t cybergear_set_speed_kp(cybergear_motor_t *motor, float kp);
esp_err_t cybergear_set_speed_ki(cybergear_motor_t *motor, float ki);
esp_err_t cybergear_set_speed(cybergear_motor_t *motor, float speed);

// ---- RX --------------------------------------------------------------------
// Offer a received frame; ESP_ERR_NOT_FOUND if it belongs to another motor.
// now_us: caller-supplied monotonic timestamp (esp_timer on target, chrono on
// host) — keeps this module free of platform time dependencies.
esp_err_t cybergear_process_message(cybergear_motor_t *motor,
                                    const twai_message_t *message, int64_t now_us);

// ---- accessors -------------------------------------------------------------
void cybergear_get_status(const cybergear_motor_t *motor, cybergear_status_t *status);
uint32_t cybergear_get_faults(const cybergear_motor_t *motor);
bool cybergear_has_faults(const cybergear_motor_t *motor);

// ---- pure frame builders (unit-testable without TWAI; Phase-0 tests) -------
void cybergear_frame_cmd(twai_message_t *out, uint8_t type, uint16_t id_opt,
                         uint8_t target_id, const uint8_t data[8]);
void cybergear_frame_param_write_f32(twai_message_t *out, uint8_t master_id,
                                     uint8_t target_id, uint16_t addr,
                                     float value, float min, float max);
void cybergear_frame_param_write_u8(twai_message_t *out, uint8_t master_id,
                                    uint8_t target_id, uint16_t addr, uint8_t value);
void cybergear_frame_param_read(twai_message_t *out, uint8_t master_id,
                                uint8_t target_id, uint16_t addr);
void cybergear_frame_motion(twai_message_t *out, uint8_t target_id,
                            const cybergear_motion_cmd_t *cmd);
uint16_t cybergear_float_to_uint(float x, float x_min, float x_max, int bits);
float cybergear_uint_to_float(uint16_t x, float x_min, float x_max);

// Phase-0 self-test: byte-exact frame checks of every fork fix. Returns the
// number of failures (0 = pass). No hardware touched.
int cybergear_selftest_run(void);

#ifdef __cplusplus
}
#endif

#endif // CYBERGEAR_H
