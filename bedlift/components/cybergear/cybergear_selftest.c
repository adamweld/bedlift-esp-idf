// Phase-0 frame-builder self-tests: byte-exact checks of every fork fix
// against the translated protocol manual. No hardware touched — pure builders.

#include <string.h>
#include <stdio.h>
#include "cybergear.h"

static int s_fail;

#define CHECK(cond, name) do { \
    if (!(cond)) { printf("  FAIL: %s\n", name); s_fail++; } \
} while (0)

int cybergear_selftest_run(void)
{
    s_fail = 0;
    twai_message_t m;

    // Enable: type 3, master 0, motor 1 -> id 0x03000001, 8 zero bytes
    cybergear_frame_cmd(&m, CG_TYPE_ENABLE, 0x00, 0x01, NULL);
    CHECK(m.identifier == 0x03000001 && m.extd == 1 && m.data_length_code == 8,
          "enable frame id/dlc");
    CHECK(m.data[0] == 0, "enable payload zeroed");

    // Clear fault: type 4, data[0]=1 (new capability)
    uint8_t cf[8] = { 0x01 };
    cybergear_frame_cmd(&m, CG_TYPE_RESET, 0x00, 0x01, cf);
    CHECK(m.identifier == 0x04000001 && m.data[0] == 0x01, "clear-fault frame");

    // Param read VBUS: type 17 (0x11), index LE in data[0..1]
    cybergear_frame_param_read(&m, 0x00, 0x02, CG_ADDR_VBUS);
    CHECK(m.identifier == 0x11000002, "param-read type is 0x11");
    CHECK(m.data[0] == 0x1C && m.data[1] == 0x70, "param-read index LE");

    // Param write u8 run_mode=speed: index 0x7005, value in data[4]
    cybergear_frame_param_write_u8(&m, 0x00, 0x01, CG_ADDR_RUN_MODE, 2);
    CHECK(m.identifier == 0x12000001 && m.data[0] == 0x05 && m.data[1] == 0x70 &&
          m.data[4] == 0x02, "run-mode write frame");

    // Clamp fix: spd_ref 50 clamped to 30.0f -> bytes 00 00 F0 41
    cybergear_frame_param_write_f32(&m, 0x00, 0x01, CG_ADDR_SPD_REF, 50.0f,
                                    -CG_VEL_RANGE, CG_VEL_RANGE);
    CHECK(m.data[4] == 0x00 && m.data[5] == 0x00 && m.data[6] == 0xF0 &&
          m.data[7] == 0x41, "f32 write clamps to max (was dead code)");
    cybergear_frame_param_write_f32(&m, 0x00, 0x01, CG_ADDR_SPD_REF, -50.0f,
                                    -CG_VEL_RANGE, CG_VEL_RANGE);
    float v; memcpy(&v, &m.data[4], 4);
    CHECK(v == -30.0f, "f32 write clamps to min");

    // Motion frame: 16-bit torque in ID bits 23..8 (was truncated to 8 bits)
    cybergear_motion_cmd_t mc = { .position = 0, .speed = 0, .torque = 0,
                                  .kp = 0, .kd = 0 };
    cybergear_frame_motion(&m, 0x01, &mc);
    uint16_t tq = (m.identifier >> 8) & 0xFFFF;
    CHECK(tq == 0x7FFF, "motion torque midpoint = 0x7FFF in ID 23..8");
    CHECK(((m.identifier >> 24) & 0x1F) == CG_TYPE_MOTION, "motion type");

    // Set CAN id: new id in ID bits 23..16 (was truncated away entirely)
    cybergear_frame_cmd(&m, CG_TYPE_SET_CAN_ID, (uint16_t)(0x05 << 8) | 0x00, 0x01, NULL);
    CHECK(((m.identifier >> 16) & 0xFF) == 0x05, "set-can-id carries new id");

    // Converters: symmetric midpoint
    CHECK(cybergear_float_to_uint(0.0f, -30.0f, 30.0f, 16) == 0x7FFF,
          "float_to_uint midpoint");
    float back = cybergear_uint_to_float(0x7FFF, -30.0f, 30.0f);
    CHECK(back > -0.01f && back < 0.01f, "uint_to_float midpoint");

    // RX: synthetic type-2 feedback for motor 3 (id bits 15..8), run mode 2,
    // fault bit 17 -> OVERCURRENT (manual), not overload
    cybergear_motor_t motor;
    cybergear_init(&motor, 0x00, 0x03, 0);
    twai_message_t rx = { .extd = 1, .data_length_code = 8 };
    rx.identifier = ((uint32_t)CG_TYPE_FEEDBACK << 24) | (2u << 22) |
                    (1u << 17) | (0x03 << 8) | 0x00;
    uint16_t mid = 0x7FFF;
    rx.data[0] = mid >> 8; rx.data[1] = mid & 0xFF;   // pos ~0
    rx.data[2] = mid >> 8; rx.data[3] = mid & 0xFF;   // vel ~0
    rx.data[4] = mid >> 8; rx.data[5] = mid & 0xFF;   // torque ~0
    rx.data[6] = 0x00; rx.data[7] = 0xC7;             // 19.9 C
    CHECK(cybergear_process_message(&motor, &rx) == ESP_OK, "feedback accepted");
    CHECK(motor.status.state == CYBERGEAR_STATE_RUNNING, "run-mode bits 23..22");
    CHECK(motor.status.temperature > 19.8f && motor.status.temperature < 20.0f,
          "temp scale /10");
    CHECK(motor.faults & CG_FAULT_OVERCURRENT, "echo bit17 = overcurrent");
    CHECK(cybergear_has_faults(&motor), "has_faults sees bit17 (union fix)");

    // RX routing: frame for motor 3 rejected by motor 1's handler
    cybergear_motor_t other;
    cybergear_init(&other, 0x00, 0x01, 0);
    CHECK(cybergear_process_message(&other, &rx) == ESP_ERR_NOT_FOUND,
          "reply routed by ID bits 15..8");

    // RX: type-21 fault frame — full word decode incl. overload byte
    twai_message_t fx = { .extd = 1, .data_length_code = 8 };
    fx.identifier = ((uint32_t)CG_TYPE_FAULT << 24) | (0x03 << 8) | 0x00;
    uint32_t fword = (1u << 16) | (0xA5u << 8) | (1u << 3) | (1u << 1);
    fx.data[0] = fword >> 24; fx.data[1] = fword >> 16;
    fx.data[2] = fword >> 8;  fx.data[3] = fword & 0xFF;
    fx.data[4] = fx.data[5] = fx.data[6] = 0; fx.data[7] = 0;
    CHECK(cybergear_process_message(&motor, &fx) == ESP_OK, "fault frame accepted");
    CHECK((motor.faults & CG_FAULT_OC_PHASE_A) && (motor.faults & CG_FAULT_OVERVOLTAGE)
          && (motor.faults & CG_FAULT_DRIVER_CHIP), "fault word bits decoded");
    CHECK(((motor.faults >> 16) & 0xFF) == 0xA5, "overload byte decoded (was TODO)");

    // RX: type-17 param reply (VBUS)
    twai_message_t px = { .extd = 1, .data_length_code = 8 };
    px.identifier = ((uint32_t)CG_TYPE_PARAM_READ << 24) | (0x03 << 8) | 0x00;
    px.data[0] = 0x1C; px.data[1] = 0x70;
    float vbus = 24.6f; memcpy(&px.data[4], &vbus, 4);
    CHECK(cybergear_process_message(&motor, &px) == ESP_OK, "param reply accepted");
    CHECK(motor.params.vbus > 24.5f && motor.params.vbus < 24.7f &&
          motor.params.updated, "vbus float landed");

    printf("cybergear selftest: %s (%d failures)\n",
           s_fail == 0 ? "PASS" : "FAIL", s_fail);
    return s_fail;
}
