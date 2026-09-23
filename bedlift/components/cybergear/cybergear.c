#include <string.h>
#include "cybergear.h"

static cybergear_frame_tap_t s_tap = NULL;
static cybergear_send_fn_t s_send = NULL;
static void *s_send_ctx = NULL;

void cybergear_set_frame_tap(cybergear_frame_tap_t tap) { s_tap = tap; }

void cybergear_set_transport(cybergear_send_fn_t send, void *ctx)
{
    s_send = send;
    s_send_ctx = ctx;
}

// ---------------------------------------------------------------------------
// Pure frame builders
// ---------------------------------------------------------------------------

// 29-bit ID layout: [28..24]=type, [23..8]=id_opt (16 bits — fork fix: the old
// uint8_t option truncated motion torque and set-can-id), [7..0]=target.
void cybergear_frame_cmd(twai_message_t *out, uint8_t type, uint16_t id_opt,
                         uint8_t target_id, const uint8_t data[8])
{
    memset(out, 0, sizeof(*out));
    out->extd = 1;
    out->identifier = ((uint32_t)(type & 0x1F) << 24) |
                      ((uint32_t)id_opt << 8) | target_id;
    out->data_length_code = 8;
    if (data) memcpy(out->data, data, 8);
}

void cybergear_frame_param_write_f32(twai_message_t *out, uint8_t master_id,
                                     uint8_t target_id, uint16_t addr,
                                     float value, float min, float max)
{
    // Fork fix: the old lib computed a clamped value and then sent the raw one.
    float v = value;
    if (v < min) v = min;
    if (v > max) v = max;
    uint8_t data[8] = { (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8), 0, 0 };
    memcpy(&data[4], &v, 4);
    cybergear_frame_cmd(out, CG_TYPE_PARAM_WRITE, master_id, target_id, data);
}

void cybergear_frame_param_write_u8(twai_message_t *out, uint8_t master_id,
                                    uint8_t target_id, uint16_t addr, uint8_t value)
{
    uint8_t data[8] = { (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8), 0, 0, value };
    cybergear_frame_cmd(out, CG_TYPE_PARAM_WRITE, master_id, target_id, data);
}

void cybergear_frame_param_read(twai_message_t *out, uint8_t master_id,
                                uint8_t target_id, uint16_t addr)
{
    // Fork fix: single-param read is type 17 dec (0x11); the old 0x17 path was
    // an undefined type the motor ignored (why VBUS polling never returned).
    uint8_t data[8] = { (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8) };
    cybergear_frame_cmd(out, CG_TYPE_PARAM_READ, master_id, target_id, data);
}

void cybergear_frame_motion(twai_message_t *out, uint8_t target_id,
                            const cybergear_motion_cmd_t *cmd)
{
    uint8_t data[8];
    uint16_t pos = cybergear_float_to_uint(cmd->position, -CG_POS_RANGE, CG_POS_RANGE, 16);
    uint16_t vel = cybergear_float_to_uint(cmd->speed, -CG_VEL_RANGE, CG_VEL_RANGE, 16);
    uint16_t kp  = cybergear_float_to_uint(cmd->kp, 0.0f, CG_KP_MAX, 16);
    uint16_t kd  = cybergear_float_to_uint(cmd->kd, 0.0f, CG_KD_MAX, 16);
    uint16_t tq  = cybergear_float_to_uint(cmd->torque, -CG_TORQUE_RANGE, CG_TORQUE_RANGE, 16);
    data[0] = pos >> 8; data[1] = pos & 0xFF;
    data[2] = vel >> 8; data[3] = vel & 0xFF;
    data[4] = kp >> 8;  data[5] = kp & 0xFF;
    data[6] = kd >> 8;  data[7] = kd & 0xFF;
    // torque rides in ID bits 23..8 — full 16 bits now (fork fix)
    cybergear_frame_cmd(out, CG_TYPE_MOTION, tq, target_id, data);
}

uint16_t cybergear_float_to_uint(float x, float x_min, float x_max, int bits)
{
    if (bits > 16) bits = 16;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    float span = x_max - x_min;
    return (uint16_t)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

float cybergear_uint_to_float(uint16_t x, float x_min, float x_max)
{
    float span = x_max - x_min;
    return (float)x / 0xFFFF * span + x_min;
}

// ---------------------------------------------------------------------------
// TX
// ---------------------------------------------------------------------------

static esp_err_t cg_transmit(cybergear_motor_t *motor, twai_message_t *msg)
{
    if (s_tap) s_tap(0, msg);
    if (!s_send) return ESP_ERR_INVALID_STATE;
    return s_send(msg, motor->transmit_ticks_to_wait, s_send_ctx);
}

static esp_err_t cg_send_simple(cybergear_motor_t *motor, uint8_t type,
                                const uint8_t data[8])
{
    twai_message_t msg;
    cybergear_frame_cmd(&msg, type, motor->master_can_id, motor->can_id, data);
    return cg_transmit(motor, &msg);
}

static esp_err_t cg_send_f32(cybergear_motor_t *motor, uint16_t addr,
                             float value, float min, float max)
{
    twai_message_t msg;
    cybergear_frame_param_write_f32(&msg, motor->master_can_id, motor->can_id,
                                    addr, value, min, max);
    return cg_transmit(motor, &msg);
}

esp_err_t cybergear_init(cybergear_motor_t *motor, uint8_t master_can_id,
                         uint8_t can_id, TickType_t transmit_ticks_to_wait)
{
    memset(motor, 0, sizeof(*motor));
    motor->master_can_id = master_can_id;
    motor->can_id = can_id;
    motor->transmit_ticks_to_wait = transmit_ticks_to_wait;
    return ESP_OK;
}

esp_err_t cybergear_ping(cybergear_motor_t *motor)
{
    return cg_send_simple(motor, CG_TYPE_PING, NULL);
}

esp_err_t cybergear_enable(cybergear_motor_t *motor)
{
    return cg_send_simple(motor, CG_TYPE_ENABLE, NULL);
}

esp_err_t cybergear_stop(cybergear_motor_t *motor)
{
    return cg_send_simple(motor, CG_TYPE_RESET, NULL);
}

esp_err_t cybergear_clear_fault(cybergear_motor_t *motor)
{
    uint8_t data[8] = { 0x01 };
    return cg_send_simple(motor, CG_TYPE_RESET, data);
}

esp_err_t cybergear_set_mode(cybergear_motor_t *motor, cybergear_mode_e mode)
{
    twai_message_t msg;
    cybergear_frame_param_write_u8(&msg, motor->master_can_id, motor->can_id,
                                   CG_ADDR_RUN_MODE, (uint8_t)mode);
    return cg_transmit(motor, &msg);
}

esp_err_t cybergear_get_param(cybergear_motor_t *motor, uint16_t index)
{
    motor->params.updated = false;
    twai_message_t msg;
    cybergear_frame_param_read(&msg, motor->master_can_id, motor->can_id, index);
    return cg_transmit(motor, &msg);
}

esp_err_t cybergear_set_motor_can_id(cybergear_motor_t *motor, uint8_t can_id)
{
    // new id in ID bits 23..16, master in 15..8 (needs the 16-bit id_opt)
    twai_message_t msg;
    uint16_t opt = ((uint16_t)can_id << 8) | motor->master_can_id;
    cybergear_frame_cmd(&msg, CG_TYPE_SET_CAN_ID, opt, motor->can_id, NULL);
    esp_err_t err = cg_transmit(motor, &msg);
    if (err == ESP_OK) motor->can_id = can_id;
    return err;
}

esp_err_t cybergear_set_mech_position_to_zero(cybergear_motor_t *motor)
{
    uint8_t data[8] = { 0x01 };
    return cg_send_simple(motor, CG_TYPE_SET_ZERO, data);
}

esp_err_t cybergear_set_limit_speed(cybergear_motor_t *motor, float speed)
{
    return cg_send_f32(motor, CG_ADDR_LIMIT_SPD, speed, 0.0f, CG_VEL_RANGE);
}

esp_err_t cybergear_set_limit_current(cybergear_motor_t *motor, float current)
{
    return cg_send_f32(motor, CG_ADDR_LIMIT_CUR, current, 0.0f, CG_CUR_MAX);
}

esp_err_t cybergear_set_limit_torque(cybergear_motor_t *motor, float torque)
{
    return cg_send_f32(motor, CG_ADDR_LIMIT_TORQUE, torque, 0.0f, CG_TORQUE_RANGE);
}

esp_err_t cybergear_set_motion_cmd(cybergear_motor_t *motor, const cybergear_motion_cmd_t *cmd)
{
    twai_message_t msg;
    cybergear_frame_motion(&msg, motor->can_id, cmd);
    return cg_transmit(motor, &msg);
}

esp_err_t cybergear_set_current_kp(cybergear_motor_t *motor, float kp)
{
    return cg_send_f32(motor, CG_ADDR_CUR_KP, kp, 0.0f, CG_KP_MAX);
}

esp_err_t cybergear_set_current_ki(cybergear_motor_t *motor, float ki)
{
    return cg_send_f32(motor, CG_ADDR_CUR_KI, ki, 0.0f, CG_KI_MAX);
}

esp_err_t cybergear_set_current_filter_gain(cybergear_motor_t *motor, float gain)
{
    return cg_send_f32(motor, CG_ADDR_CUR_FILT_GAIN, gain, 0.0f, 1.0f);
}

esp_err_t cybergear_set_current(cybergear_motor_t *motor, float current)
{
    return cg_send_f32(motor, CG_ADDR_IQ_REF, current, -CG_CUR_MAX, CG_CUR_MAX);
}

esp_err_t cybergear_set_position_kp(cybergear_motor_t *motor, float kp)
{
    // Fork fix: RAM address 0x701E (old lib wrote 0x2016, the debugger-only
    // config region — a silent no-op over CAN)
    return cg_send_f32(motor, CG_ADDR_LOC_KP, kp, 0.0f, CG_KP_MAX);
}

esp_err_t cybergear_set_position(cybergear_motor_t *motor, float position)
{
    // loc_ref is a raw float in radians — multi-turn targets are valid
    // (observed working beyond ±4π with the upstream app); clamp generously.
    return cg_send_f32(motor, CG_ADDR_LOC_REF, position, -1000.0f, 1000.0f);
}

esp_err_t cybergear_set_speed_kp(cybergear_motor_t *motor, float kp)
{
    return cg_send_f32(motor, CG_ADDR_SPD_KP, kp, 0.0f, CG_KP_MAX);
}

esp_err_t cybergear_set_speed_ki(cybergear_motor_t *motor, float ki)
{
    return cg_send_f32(motor, CG_ADDR_SPD_KI, ki, 0.0f, CG_KI_MAX);
}

esp_err_t cybergear_set_speed(cybergear_motor_t *motor, float speed)
{
    return cg_send_f32(motor, CG_ADDR_SPD_REF, speed, -CG_VEL_RANGE, CG_VEL_RANGE);
}

// ---------------------------------------------------------------------------
// RX
// ---------------------------------------------------------------------------

static esp_err_t cg_rx_feedback(cybergear_motor_t *motor, const twai_message_t *m, int64_t now_us)
{
    esp_err_t err = ESP_OK;
    uint16_t raw_pos  = (uint16_t)(m->data[0] << 8 | m->data[1]);
    uint16_t raw_vel  = (uint16_t)(m->data[2] << 8 | m->data[3]);
    uint16_t raw_tq   = (uint16_t)(m->data[4] << 8 | m->data[5]);
    uint16_t raw_temp = (uint16_t)(m->data[6] << 8 | m->data[7]);

    uint32_t mode = (m->identifier >> 22) & 0x3;
    switch (mode) {
        case 0: motor->status.state = CYBERGEAR_STATE_RESET; break;
        case 1: motor->status.state = CYBERGEAR_STATE_CALIBRATION; break;
        case 2: motor->status.state = CYBERGEAR_STATE_RUNNING; break;
        default: err = ESP_ERR_INVALID_RESPONSE; break;
    }
    float pos = cybergear_uint_to_float(raw_pos, -CG_POS_RANGE, CG_POS_RANGE);
    // Multi-turn unwrap: the packed position wraps at ±CG_POS_RANGE. Detect a
    // wrap as a jump larger than half the span and accumulate. This is the
    // ONLY way to get cumulative angle — the firmware serves no mechPos read.
    if (motor->status.pos_init) {
        float d = pos - motor->status.position;
        if (d > CG_POS_RANGE) motor->status.pos_wraps--;
        else if (d < -CG_POS_RANGE) motor->status.pos_wraps++;
    } else {
        motor->status.pos_init = true;
    }
    motor->status.position = pos;
    motor->status.position_unwrapped =
        pos + (float)motor->status.pos_wraps * (2.0f * CG_POS_RANGE);
    motor->status.speed       = cybergear_uint_to_float(raw_vel, -CG_VEL_RANGE, CG_VEL_RANGE);
    motor->status.torque      = cybergear_uint_to_float(raw_tq, -CG_TORQUE_RANGE, CG_TORQUE_RANGE);
    motor->status.temperature = (float)raw_temp / 10.0f;
    motor->status.last_rx_us  = now_us;

    // Live fault bits in ID 21..16 (fork fix: bit17 is overcurrent per manual)
    uint32_t f = motor->faults &
                 ~(CG_FAULT_UNDERVOLTAGE | CG_FAULT_OVERCURRENT | CG_FAULT_OVERTEMP |
                   CG_FAULT_MAG_ENCODER | CG_FAULT_HALL | CG_FAULT_UNCALIBRATED);
    if (m->identifier & (1u << 16)) f |= CG_FAULT_UNDERVOLTAGE;
    if (m->identifier & (1u << 17)) f |= CG_FAULT_OVERCURRENT;
    if (m->identifier & (1u << 18)) f |= CG_FAULT_OVERTEMP;
    if (m->identifier & (1u << 19)) f |= CG_FAULT_MAG_ENCODER;
    if (m->identifier & (1u << 20)) f |= CG_FAULT_HALL;
    if (m->identifier & (1u << 21)) f |= CG_FAULT_UNCALIBRATED;
    motor->faults = f;
    return err;
}

static esp_err_t cg_rx_fault(cybergear_motor_t *motor, const twai_message_t *m)
{
    uint32_t fault = (uint32_t)m->data[0] << 24 | (uint32_t)m->data[1] << 16 |
                     (uint32_t)m->data[2] << 8 | m->data[3];
    uint32_t warn  = (uint32_t)m->data[4] << 24 | (uint32_t)m->data[5] << 16 |
                     (uint32_t)m->data[6] << 8 | m->data[7];
    motor->fault_word = fault;
    motor->warn_word = warn;

    uint32_t f = motor->faults &
                 ~(CG_FAULT_OC_PHASE_A | CG_FAULT_OC_PHASE_B | CG_FAULT_OC_PHASE_C |
                   CG_FAULT_OVERVOLTAGE | CG_FAULT_DRIVER_CHIP | CG_FAULT_OVERLOAD_MASK);
    if (fault & (1u << 16)) f |= CG_FAULT_OC_PHASE_A;
    if (fault & (1u << 4))  f |= CG_FAULT_OC_PHASE_B;
    if (fault & (1u << 5))  f |= CG_FAULT_OC_PHASE_C;
    if (fault & (1u << 3))  f |= CG_FAULT_OVERVOLTAGE;
    if (fault & (1u << 2))  f |= CG_FAULT_UNDERVOLTAGE;
    if (fault & (1u << 1))  f |= CG_FAULT_DRIVER_CHIP;
    if (fault & (1u << 7))  f |= CG_FAULT_UNCALIBRATED;
    if (warn  & (1u << 0))  f |= CG_FAULT_OVERTEMP;
    // Overload byte fault[15..8], previously never decoded — stored raw
    f |= ((fault >> 8) & 0xFF) << 16;
    motor->faults = f;
    return ESP_OK;
}

static esp_err_t cg_rx_param(cybergear_motor_t *motor, const twai_message_t *m, int64_t now_us)
{
    uint16_t index = (uint16_t)(m->data[1] << 8 | m->data[0]);
    uint8_t u8; int16_t i16; float f32;
    memcpy(&u8, &m->data[4], 1);
    memcpy(&i16, &m->data[4], 2);
    memcpy(&f32, &m->data[4], 4);

    switch (index) {
        case CG_ADDR_RUN_MODE:      motor->params.run_mode = u8; break;
        case CG_ADDR_IQ_REF:        motor->params.iq_ref = f32; break;
        case CG_ADDR_SPD_REF:       motor->params.spd_ref = f32; break;
        case CG_ADDR_LIMIT_TORQUE:  motor->params.limit_torque = f32; break;
        case CG_ADDR_CUR_KP:        motor->params.cur_kp = f32; break;
        case CG_ADDR_CUR_KI:        motor->params.cur_ki = f32; break;
        case CG_ADDR_CUR_FILT_GAIN: motor->params.cur_filt_gain = f32; break;
        case CG_ADDR_LOC_REF:       motor->params.loc_ref = f32; break;
        case CG_ADDR_LIMIT_SPD:     motor->params.limit_spd = f32; break;
        case CG_ADDR_LIMIT_CUR:     motor->params.limit_cur = f32; break;
        case CG_ADDR_MECH_POS:      motor->params.mech_pos = f32; break;
        case CG_ADDR_IQF:           motor->params.iqf = f32; break;
        case CG_ADDR_MECH_VEL:      motor->params.mech_vel = f32; break;
        case CG_ADDR_VBUS:          motor->params.vbus = f32; break;
        case CG_ADDR_ROTATION:      motor->params.rotation = i16; break;
        case CG_ADDR_LOC_KP:        motor->params.loc_kp = f32; break;
        case CG_ADDR_SPD_KP:        motor->params.spd_kp = f32; break;
        case CG_ADDR_SPD_KI:        motor->params.spd_ki = f32; break;
        default: return ESP_ERR_INVALID_RESPONSE;
    }
    motor->params.last_index = index;
    motor->params.updated = true;
    motor->params.last_rx_us = now_us;
    return ESP_OK;
}

static esp_err_t cg_rx_ping(cybergear_motor_t *motor, const twai_message_t *m, int64_t now_us)
{
    memcpy(&motor->mcu_uid, m->data, 8);
    motor->ping_rx_us = now_us;
    return ESP_OK;
}

esp_err_t cybergear_process_message(cybergear_motor_t *motor, const twai_message_t *m, int64_t now_us)
{
    // 5-bit type field (fork fix: old mask took 6 bits)
    uint8_t type = (m->identifier >> 24) & 0x1F;

    // Reply frames carry motor id in ID bits 15..8 (fork fix: the old code
    // matched param replies on bits 7..0 — always the master id 0x00 — so
    // every param reply was dropped; bits 15..8 is the manual's layout and
    // matches our bench-captured type-0 reply 0x000001FE).
    uint8_t src_id = (m->identifier >> 8) & 0xFF;
    if (src_id != motor->can_id) return ESP_ERR_NOT_FOUND;

    if (s_tap) s_tap(1, m);

    switch (type) {
        case CG_TYPE_PING:       return cg_rx_ping(motor, m, now_us);
        case CG_TYPE_FEEDBACK:   return cg_rx_feedback(motor, m, now_us);
        case CG_TYPE_PARAM_READ: return cg_rx_param(motor, m, now_us);
        case CG_TYPE_FAULT:      return cg_rx_fault(motor, m);
        default:                 return ESP_ERR_INVALID_RESPONSE;
    }
}

void cybergear_get_status(const cybergear_motor_t *motor, cybergear_status_t *status)
{
    memcpy(status, &motor->status, sizeof(*status));
}

uint32_t cybergear_get_faults(const cybergear_motor_t *motor)
{
    return motor->faults;
}

bool cybergear_has_faults(const cybergear_motor_t *motor)
{
    return motor->faults != 0;
}
