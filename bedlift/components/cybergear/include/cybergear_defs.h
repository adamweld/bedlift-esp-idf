#ifndef CYBERGEAR_DEFS_H
#define CYBERGEAR_DEFS_H

// Communication types (29-bit ID bits 28..24), per the translated manual.
// Verified on bench 2026-09-22: PING (0), ENABLE (3), RESET (4), RAM_WRITE (18).
#define CG_TYPE_PING       0x00  // get device id; reply low byte = 0xFE
#define CG_TYPE_MOTION     0x01  // MIT-style; torque in ID bits 23..8
#define CG_TYPE_FEEDBACK   0x02  // motor feedback (reply to most commands)
#define CG_TYPE_ENABLE     0x03
#define CG_TYPE_RESET      0x04  // data[0]=0 stop; data[0]=1 clear fault
#define CG_TYPE_SET_ZERO   0x06  // data[0]=1; lost on power-down
#define CG_TYPE_SET_CAN_ID 0x07  // new id in ID bits 23..16
#define CG_TYPE_PARAM_READ  0x11 // 17 dec — single param read (fork fix: was 0x17)
#define CG_TYPE_PARAM_WRITE 0x12 // 18 dec — single param write; echoes type-2
#define CG_TYPE_FAULT      0x15  // 21 dec — fault feedback frame

// RAM parameter table (type 17/18), addresses per manual §4.1.12
#define CG_ADDR_RUN_MODE      0x7005  // u8: 0 motion / 1 position / 2 speed / 3 current
#define CG_ADDR_IQ_REF        0x7006  // float A
#define CG_ADDR_SPD_REF       0x700A  // float rad/s
#define CG_ADDR_LIMIT_TORQUE  0x700B  // float Nm
#define CG_ADDR_CUR_KP        0x7010
#define CG_ADDR_CUR_KI        0x7011
#define CG_ADDR_CUR_FILT_GAIN 0x7014
#define CG_ADDR_LOC_REF       0x7016  // float rad (raw float — position cmds not ±4π-packed)
#define CG_ADDR_LIMIT_SPD     0x7017
#define CG_ADDR_LIMIT_CUR     0x7018
#define CG_ADDR_MECH_POS      0x7019  // R, float rad (fw >= 1.2.1.5)
#define CG_ADDR_IQF           0x701A  // R
#define CG_ADDR_MECH_VEL      0x701B  // R
#define CG_ADDR_VBUS          0x701C  // R, float V
#define CG_ADDR_ROTATION      0x701D  // int16 turn count
#define CG_ADDR_LOC_KP        0x701E  // fork fix: gains are RAM 0x701E..0x7020,
#define CG_ADDR_SPD_KP        0x701F  // not the 0x2xxx config region the old lib wrote
#define CG_ADDR_SPD_KI        0x7020

// Packed-field ranges for type-1 motion frames and type-2 feedback.
// NOTE: manual says position ±4π (±12.566); its own sample code (and the old
// lib) use ±12.5. ~0.5% difference — resolve empirically in bench test B2,
// then flip CG_POS_RANGE. Position *commands* (loc_ref) are raw floats and
// unaffected (they work beyond 4π, observed with the upstream position app).
#ifndef CG_POS_RANGE
#define CG_POS_RANGE 12.5f
#endif
#define CG_VEL_RANGE 30.0f
#define CG_TORQUE_RANGE 12.0f
#define CG_KP_MAX 500.0f
#define CG_KI_MAX 10.0f
#define CG_KD_MAX 5.0f
#define CG_CUR_MAX 23.0f  // rated 6.5A, peak 23A

typedef enum {
    CYBERGEAR_MODE_MOTION   = 0x00,
    CYBERGEAR_MODE_POSITION = 0x01,
    CYBERGEAR_MODE_SPEED    = 0x02,
    CYBERGEAR_MODE_CURRENT  = 0x03,
} cybergear_mode_e;

typedef enum {
    CYBERGEAR_STATE_RESET       = 0x00,
    CYBERGEAR_STATE_CALIBRATION = 0x01,
    CYBERGEAR_STATE_RUNNING     = 0x02,
} cybergear_state_e;

// Unified fault bitmask (fork fix: replaces the 11-bool/uint16 union that
// made has_faults() miss 9 of 11 flags). Sources noted per bit.
#define CG_FAULT_UNDERVOLTAGE   (1u << 0)   // echo bit16 / fault word bit2
#define CG_FAULT_OVERCURRENT    (1u << 1)   // echo bit17 (manual: overcurrent, not overload)
#define CG_FAULT_OVERTEMP       (1u << 2)   // echo bit18 / fault word bit0
#define CG_FAULT_MAG_ENCODER    (1u << 3)   // echo bit19
#define CG_FAULT_HALL           (1u << 4)   // echo bit20
#define CG_FAULT_UNCALIBRATED   (1u << 5)   // echo bit21 / fault word bit7
#define CG_FAULT_OC_PHASE_A     (1u << 6)   // fault word bit16
#define CG_FAULT_OC_PHASE_B     (1u << 7)   // fault word bit4
#define CG_FAULT_OC_PHASE_C     (1u << 8)   // fault word bit5
#define CG_FAULT_OVERVOLTAGE    (1u << 9)   // fault word bit3
#define CG_FAULT_DRIVER_CHIP    (1u << 10)  // fault word bit1
#define CG_FAULT_OVERLOAD_MASK  (0xFFu << 16) // fault word bits 15..8, raw (fork fix: was never decoded)
#define CG_WARN_OVERTEMP        (1u << 0)   // warning word bit0 (75 °C default)

#endif // CYBERGEAR_DEFS_H
