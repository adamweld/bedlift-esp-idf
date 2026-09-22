#pragma once

// ============================================================================
// Bedlift system configuration — single home for every pin and constant.
// Hardware facts below are bench-verified (see bring-up, 2026-09-20..22).
// ============================================================================

// --- Pins: power switching -------------------------------------------------
#define PIN_MOTOR_SSR_EN 5    // gates 24V to all four motor SSRs (K2..K5)
#define PIN_LOCK_EN      6    // gates 12V to all four lock solenoids (K1)

// --- Pins: I2C / sensors ---------------------------------------------------
#define PIN_I2C_SDA   3
#define PIN_I2C_SCL   4
#define PIN_I2C_POWER 7       // Feather TFT_I2C_POWER: must be high or the
                              // I2C rail + pull-ups are unpowered
#define PIN_ACC_SDO   8       // ADXL345 addr strap: high -> front @0x1D
#define I2C_ADDR_ACC_FRONT 0x1D
#define I2C_ADDR_ACC_REAR  0x53
#define I2C_ADDR_FUEL      0x36  // MAX17048 (Feather LiPo only)
#define I2C_FREQ_HZ 400000

// --- Pins: hall endstops (soft-disabled until mechanically mounted) --------
#define PIN_HALL_1 11         // physical hall-1 (PCB net/silk crossed; fw follows physical)
#define PIN_HALL_2 9

// --- Pins: CAN (bench-verified 2026-09-22: PCB net names were correct) -----
#define PIN_CAN_TX 39
#define PIN_CAN_RX 38

// --- Pins: buttons (Feather on-module) -------------------------------------
#define PIN_BTN_DOWN 0        // D0, active LOW  (RTC ext0 wake)
#define PIN_BTN_MODE 1        // D1, active HIGH (RTC ext1 wake)
#define PIN_BTN_UP   2        // D2, active HIGH (RTC ext1 wake)

// --- CAN / motors -----------------------------------------------------------
#define CG_MASTER_ID 0x00
// Logical motor index 0..3 -> CAN id (front-left, front-right, rear-left,
// rear-right assignment TBD at bench; ids per old firmware convention)
#define MOTOR_CAN_IDS { 0x01, 0x04, 0x03, 0x02 }
#define NUM_MOTORS 4

// --- Motion / control (initial conservative values; tuned in Phase B/C) ----
#define V_CRUISE_RAD_S      2.0f   // start ~half target; raise in Phase C
#define A_MAX_RAD_S2        2.0f
#define A_STOP_RAD_S2       4.0f
#define V_SETTLE_RAD_S      1.0f
#define V_UNLOAD_RAD_S      3.0f   // pawl-unload magnitude (sign TBD bench)
#define THETA_UNLOAD_RAD    0.15f
#define T_UNLOAD_TIMEOUT_MS 350
#define T_SETTLE_TIMEOUT_MS 1000
#define T_BOOT_SETTLE_MS    1500   // SSR-close -> motors ready; measure in A1
#define T_READY_KEEPWARM_MS 5000   // SSR depowers after 5 s of inaction
#define MOTOR_LIMIT_CURRENT_A 7.0f
#define TRIM_POS_CLAMP_FRAC  0.10f
#define TRIM_TILT_CLAMP_FRAC 0.05f
#define V_LEVEL_MAX_RAD_S    1.0f
#define LEVEL_DONE_DEG       0.3f
#define LEVEL_DONE_HOLD_MS   500

// --- Safety thresholds (6S Li-ion 24V pack) ---------------------------------
#define VBUS_WARN_V     21.0f
#define VBUS_LOCKOUT_V  19.5f
#define VEL_ABS_MAX_RAD_S 8.0f
#define OVERSPEED_FACTOR  1.5f
#define OVERSPEED_SAMPLES 3
#define TELEM_LOSS_MS     150
#define MOTOR_TEMP_STOP_C 70.0f
// RACK_WARN_DEG / RACK_TRIP_DEG live in sys_state.h (shared with the UI):
// 10 / 15 deg — the vehicle itself may be parked ~20 deg off-level.
#define POS_DESYNC_MAX_RAD 2.0f
#define SAFETY_BRAKE_WINDOW_MS 200

// --- Solenoid duty (leaky bucket; tune with thermocouple in M7) -------------
#define SOL_BUCKET_MAX_S  30.0f
#define SOL_COOL_RATE     0.25f   // 4:1 rest:work
#define SOL_RESERVE_S     8.0f

// --- Buttons timing ----------------------------------------------------------
#define BTN_DEBOUNCE_MS   20
#define BTN_CHORD_MS      80
#define BTN_HOLD_MS       600
#define BTN_REPEAT_MS     150

// --- Power / sleep -----------------------------------------------------------
#define AUTO_DIM_S    45
#define AUTO_SLEEP_S  60
#define BACKLIGHT_FULL   128
#define BACKLIGHT_DIMMED 32

// --- Accel / tilt filtering --------------------------------------------------
#define ACC_SAMPLE_HZ 50
#define TILT_LP_FC_HZ 1.5f
