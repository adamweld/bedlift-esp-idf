#include <math.h>
#include <string.h>
#include "sim.h"
#include "cybergear.h"
#include "bed_geometry.h"   // corner layout: idx0=FR idx1=BR idx2=FL idx3=BL

// Motor CAN ids indexed by logical motor (M1=idx0..M4=idx3): M1=1,M2=4,M3=3,M4=2.
// This is the CAN bus address map only — corner geometry lives in bed_geometry.h.
static const uint8_t k_can_ids[SIM_NUM_MOTORS] = { 0x01, 0x04, 0x03, 0x02 };

#define BED_TRACK_M 1.2f   // left-right corner spacing
#define BED_LENGTH_M 2.0f  // front-rear corner spacing
#define PAWL_UNLOAD_RAD 0.12f
#define RXQ_LEN 64

typedef struct {
    bool enabled;
    uint8_t run_mode;
    float spd_ref;
    float limit_cur;
    float theta;        // cumulative rad, + = up
    float vel;          // rad/s
    float iq;           // A
    float temp_c;
    sim_fault_e fault;
    bool pawl_seated;   // pawl carrying load (needs unload before retract)
    float seat_theta;   // angle where the pawl took the load
    float runaway_v;
} sim_motor_t;

static struct {
    sim_config_t cfg;
    sim_motor_t m[SIM_NUM_MOTORS];
    bool ssr_on;
    bool lock_energized;
    float vbus;
    uint32_t noise_seed;
    twai_message_t rxq[RXQ_LEN];
    int rx_head, rx_tail;
} S;

void sim_default_config(sim_config_t *cfg)
{
    cfg->drum_radius_m = 0.025f;
    cfg->wrap_radius_gain = 0.0008f;   // ~+8% radius per 100 rad wound
    cfg->tau_s = 0.05f;
    cfg->gravity_backdrive = 6.0f;     // rad/s^2 falling when free
    cfg->travel_rad = 40.0f;           // ~1 m at 25 mm drum
    cfg->batt_v0 = 24.6f;
    cfg->batt_r = 0.08f;
    cfg->accel_noise_g = 0.02f;
}

void sim_init(const sim_config_t *cfg)
{
    memset(&S, 0, sizeof(S));
    S.cfg = *cfg;
    S.vbus = cfg->batt_v0;
    S.noise_seed = 0x1234567;
    for (int i = 0; i < SIM_NUM_MOTORS; i++) {
        S.m[i].temp_c = 20.0f;
        S.m[i].pawl_seated = true;      // depowered system rests on pawls
        S.m[i].theta = 2.0f;            // parked slightly above bottom stop
        S.m[i].seat_theta = 2.0f;
        S.m[i].limit_cur = 23.0f;
    }
}

// --- reply queue ------------------------------------------------------------
static void rxq_push(const twai_message_t *msg)
{
    int next = (S.rx_head + 1) % RXQ_LEN;
    if (next == S.rx_tail) return;      // overflow: drop (mimics rx_missed)
    S.rxq[S.rx_head] = *msg;
    S.rx_head = next;
}

bool sim_can_poll_rx(twai_message_t *out)
{
    if (S.rx_tail == S.rx_head) return false;
    *out = S.rxq[S.rx_tail];
    S.rx_tail = (S.rx_tail + 1) % RXQ_LEN;
    return true;
}

// --- reply builders ---------------------------------------------------------
static uint32_t motor_fault_id_bits(const sim_motor_t *m)
{
    uint32_t bits = 0;
    if (m->fault == SIM_FAULT_UNDERVOLTAGE) bits |= 1u << 16;
    if (m->fault == SIM_FAULT_OVERCURRENT)  bits |= 1u << 17;
    return bits;
}

static void push_feedback(int idx)
{
    const sim_motor_t *m = &S.m[idx];
    twai_message_t f;
    memset(&f, 0, sizeof(f));
    f.extd = 1;
    f.data_length_code = 8;

    uint32_t mode = 0; // reset
    if (m->enabled && m->fault != SIM_FAULT_OVERCURRENT &&
        m->fault != SIM_FAULT_DRIVER) mode = 2; // running

    f.identifier = ((uint32_t)CG_TYPE_FEEDBACK << 24) | (mode << 22) |
                   motor_fault_id_bits(m) | ((uint32_t)k_can_ids[idx] << 8);

    // wrap position into the packed field range like the real motor
    float wrapped = fmodf(m->theta + CG_POS_RANGE, 2.0f * CG_POS_RANGE);
    if (wrapped < 0) wrapped += 2.0f * CG_POS_RANGE;
    wrapped -= CG_POS_RANGE;
    uint16_t pos = cybergear_float_to_uint(wrapped, -CG_POS_RANGE, CG_POS_RANGE, 16);
    uint16_t vel = cybergear_float_to_uint(m->vel, -CG_VEL_RANGE, CG_VEL_RANGE, 16);
    uint16_t tq  = cybergear_float_to_uint(m->iq * 0.5f, -CG_TORQUE_RANGE, CG_TORQUE_RANGE, 16);
    uint16_t temp = (uint16_t)(m->temp_c * 10.0f);
    f.data[0] = pos >> 8; f.data[1] = pos & 0xFF;
    f.data[2] = vel >> 8; f.data[3] = vel & 0xFF;
    f.data[4] = tq >> 8;  f.data[5] = tq & 0xFF;
    f.data[6] = temp >> 8; f.data[7] = temp & 0xFF;
    rxq_push(&f);
}

static void push_param_reply(int idx, uint16_t addr)
{
    const sim_motor_t *m = &S.m[idx];
    twai_message_t f;
    memset(&f, 0, sizeof(f));
    f.extd = 1;
    f.data_length_code = 8;
    f.identifier = ((uint32_t)CG_TYPE_PARAM_READ << 24) |
                   ((uint32_t)k_can_ids[idx] << 8);
    f.data[0] = addr & 0xFF;
    f.data[1] = addr >> 8;
    float val = 0;
    switch (addr) {
        case CG_ADDR_VBUS:
            val = (m->fault == SIM_FAULT_UNDERVOLTAGE) ? 18.5f : S.vbus; break;
        case CG_ADDR_MECH_POS: {
            float wrapped = fmodf(m->theta, 2.0f * (float)M_PI); val = wrapped; break;
        }
        case CG_ADDR_MECH_VEL: val = m->vel; break;
        case CG_ADDR_IQF:      val = m->iq; break;
        case CG_ADDR_SPD_REF:  val = m->spd_ref; break;
        case CG_ADDR_LIMIT_CUR: val = m->limit_cur; break;
        case CG_ADDR_ROTATION: {
            int16_t rot = (int16_t)(m->theta / (2.0f * (float)M_PI));
            memcpy(&f.data[4], &rot, 2);
            rxq_push(&f);
            return;
        }
        case CG_ADDR_RUN_MODE:
            f.data[4] = m->run_mode;
            rxq_push(&f);
            return;
        default: break;
    }
    memcpy(&f.data[4], &val, 4);
    rxq_push(&f);
}

static void push_fault_frame(int idx)
{
    const sim_motor_t *m = &S.m[idx];
    twai_message_t f;
    memset(&f, 0, sizeof(f));
    f.extd = 1;
    f.data_length_code = 8;
    f.identifier = ((uint32_t)CG_TYPE_FAULT << 24) | ((uint32_t)k_can_ids[idx] << 8);
    uint32_t fault = 0;
    if (m->fault == SIM_FAULT_UNDERVOLTAGE) fault |= 1u << 2;
    if (m->fault == SIM_FAULT_OVERCURRENT)  fault |= 1u << 16;
    if (m->fault == SIM_FAULT_DRIVER)       fault |= 1u << 1;
    f.data[0] = fault >> 24; f.data[1] = fault >> 16;
    f.data[2] = fault >> 8;  f.data[3] = fault & 0xFF;
    rxq_push(&f);
}

// --- CAN in -----------------------------------------------------------------
esp_err_t sim_can_tx(const twai_message_t *msg, TickType_t ticks, void *ctx)
{
    (void)ticks; (void)ctx;
    uint8_t type = (msg->identifier >> 24) & 0x1F;
    uint8_t target = msg->identifier & 0xFF;

    int idx = -1;
    for (int i = 0; i < SIM_NUM_MOTORS; i++)
        if (k_can_ids[i] == target) { idx = i; break; }
    if (idx < 0) return ESP_OK;              // no ACK modeled; frame vanishes

    sim_motor_t *m = &S.m[idx];
    if (!S.ssr_on) return ESP_OK;            // unpowered motors are silent
    if (m->fault == SIM_FAULT_COMM_LOSS) return ESP_OK;

    switch (type) {
        case CG_TYPE_PING: {
            twai_message_t f;
            memset(&f, 0, sizeof(f));
            f.extd = 1;
            f.data_length_code = 8;
            f.identifier = ((uint32_t)k_can_ids[idx] << 8) | 0xFE;
            memcpy(f.data, "SIMUID0", 7);
            f.data[7] = k_can_ids[idx];
            rxq_push(&f);
            break;
        }
        case CG_TYPE_ENABLE:
            if (m->fault != SIM_FAULT_OVERCURRENT && m->fault != SIM_FAULT_DRIVER)
                m->enabled = true;
            push_feedback(idx);
            break;
        case CG_TYPE_RESET:
            if (msg->data[0] == 0x01) {       // clear fault
                if (m->fault == SIM_FAULT_OVERCURRENT ||
                    m->fault == SIM_FAULT_DRIVER ||
                    m->fault == SIM_FAULT_UNDERVOLTAGE)
                    m->fault = SIM_FAULT_NONE;
            }
            m->enabled = false;
            m->spd_ref = 0;
            push_feedback(idx);
            break;
        case CG_TYPE_PARAM_WRITE: {
            uint16_t addr = (uint16_t)(msg->data[1] << 8 | msg->data[0]);
            float fval; memcpy(&fval, &msg->data[4], 4);
            switch (addr) {
                case CG_ADDR_RUN_MODE:
                    // manual rule: rejected while running (characterize in B1)
                    if (!m->enabled) m->run_mode = msg->data[4];
                    break;
                case CG_ADDR_SPD_REF:   m->spd_ref = fval; break;
                case CG_ADDR_LIMIT_CUR: m->limit_cur = fval; break;
                default: break;
            }
            push_feedback(idx);               // the write-echo property
            break;
        }
        case CG_TYPE_PARAM_READ: {
            uint16_t addr = (uint16_t)(msg->data[1] << 8 | msg->data[0]);
            push_param_reply(idx, addr);
            break;
        }
        case CG_TYPE_SET_ZERO:
            m->theta = 0;
            m->seat_theta = 0;
            push_feedback(idx);
            break;
        default:
            break;
    }
    return ESP_OK;
}

void sim_power_set(bool motor_ssr_on, bool lock_energized)
{
    if (S.ssr_on && !motor_ssr_on) {
        // power cut: motors drop out
        for (int i = 0; i < SIM_NUM_MOTORS; i++) {
            S.m[i].enabled = false;
            S.m[i].spd_ref = 0;
        }
    }
    S.ssr_on = motor_ssr_on;
    S.lock_energized = lock_energized;
}

// --- physics ----------------------------------------------------------------
void sim_step(int64_t now_us, float dt_s)
{
    (void)now_us;
    float total_amps = 0;

    for (int i = 0; i < SIM_NUM_MOTORS; i++) {
        sim_motor_t *m = &S.m[i];

        // pawl retracts only when energized AND not carrying load
        bool pawl_retracted = S.lock_energized && !m->pawl_seated;

        bool driven = S.ssr_on && m->enabled && m->run_mode == 2 &&
                      m->fault != SIM_FAULT_OVERCURRENT &&
                      m->fault != SIM_FAULT_DRIVER;

        float target = m->spd_ref;
        if (m->fault == SIM_FAULT_RUNAWAY && driven) {
            m->runaway_v -= 4.0f * dt_s;      // accelerates downward
            target = m->runaway_v;
        } else {
            m->runaway_v = 0;
        }
        if (m->fault == SIM_FAULT_SNAG) target = 0;

        if (driven) {
            m->vel += (target - m->vel) * dt_s / S.cfg.tau_s;
            // load current: holding torque + acceleration
            m->iq = 1.5f + fabsf(target) * 0.4f + (target > 0.05f ? 1.5f : 0.0f);
            if (m->fault == SIM_FAULT_SNAG && fabsf(m->spd_ref) > 0.05f)
                m->iq = m->limit_cur;         // stalled against snag
        } else {
            // unpowered/disabled
            if (pawl_retracted) {
                m->vel -= S.cfg.gravity_backdrive * dt_s;   // free fall
            } else {
                m->vel = 0;                    // held (pawl or friction)
            }
            m->iq = 0;
        }

        // ratchet: pawl engaged blocks downward motion
        if (!pawl_retracted && m->vel < 0) {
            m->vel = 0;
            if (driven && m->spd_ref < -0.05f) {
                m->iq = m->limit_cur * 0.6f;   // fighting the pawl: seat signature
                m->pawl_seated = true;
                m->seat_theta = m->theta;
            }
        }

        m->theta += m->vel * dt_s;

        // travel limits
        if (m->theta < 0) { m->theta = 0; if (m->vel < 0) m->vel = 0; }
        if (m->theta > S.cfg.travel_rad) {
            m->theta = S.cfg.travel_rad;
            if (m->vel > 0) m->vel = 0;
        }

        // pawl load state: seated when it last took load; unloaded by up-drive
        if (m->pawl_seated && m->theta > m->seat_theta + PAWL_UNLOAD_RAD)
            m->pawl_seated = false;
        if (!S.lock_energized && !driven && m->vel == 0 && !m->pawl_seated) {
            // resting back onto the pawl
            m->pawl_seated = true;
            m->seat_theta = m->theta;
        }

        total_amps += fabsf(m->iq);
    }

    S.vbus = S.cfg.batt_v0 - S.cfg.batt_r * total_amps;
}

// --- geometry / sensors -----------------------------------------------------
static float corner_height(int i)
{
    const sim_motor_t *m = &S.m[i];
    // effective radius grows with wrap
    float r = S.cfg.drum_radius_m * (1.0f + S.cfg.wrap_radius_gain * m->theta);
    return m->theta * r;
}

float sim_corner_height_m(int i) { return corner_height(i); }

// Tilt from corner heights, mapping each motor to its corner via the single
// source of truth (bed_geometry.h). + roll = right high, + pitch = front high,
// matching the IMU calibration — so the sim exercises the SAME polarity the
// control law assumes. `front_pair`: true = front axle only, false = rear.
static float roll_from_corners(bool front_pair)
{
    float right = 0, left = 0;
    for (int i = 0; i < SIM_NUM_MOTORS; i++) {
        if (bed_is_front(i) != front_pair) continue;
        if (bed_lr(i) > 0) right = corner_height(i);
        else               left  = corner_height(i);
    }
    return atan2f(right - left, BED_TRACK_M) * 57.2958f;
}

float sim_bed_roll_front_deg(void) { return roll_from_corners(true); }
float sim_bed_roll_rear_deg(void)  { return roll_from_corners(false); }

float sim_bed_pitch_deg(void)
{
    float front = 0, rear = 0;
    int nf = 0, nr = 0;
    for (int i = 0; i < SIM_NUM_MOTORS; i++) {
        if (bed_fb(i) > 0) { front += corner_height(i); nf++; }
        else               { rear  += corner_height(i); nr++; }
    }
    return atan2f(front / nf - rear / nr, BED_LENGTH_M) * 57.2958f;
}

static float noise(void)
{
    S.noise_seed = S.noise_seed * 1664525u + 1013904223u;
    return ((float)(S.noise_seed >> 8) / 8388608.0f - 1.0f);
}

void sim_accel_read(int idx, float *x_g, float *y_g, float *z_g)
{
    float roll = (idx == 0 ? sim_bed_roll_front_deg() : sim_bed_roll_rear_deg())
                 / 57.2958f;
    float pitch = sim_bed_pitch_deg() / 57.2958f;
    bool moving = false;
    for (int i = 0; i < SIM_NUM_MOTORS; i++)
        if (fabsf(S.m[i].vel) > 0.05f) moving = true;
    float n = moving ? S.cfg.accel_noise_g : 0.002f;
    *x_g = sinf(roll) + n * noise();
    *y_g = sinf(pitch) + n * noise();
    *z_g = cosf(roll) * cosf(pitch) + n * noise();
}

bool sim_hall_top(void)
{
    for (int i = 0; i < SIM_NUM_MOTORS; i++)
        if (S.m[i].theta > S.cfg.travel_rad - 0.2f) return true;
    return false;
}

bool sim_hall_bottom(void)
{
    for (int i = 0; i < SIM_NUM_MOTORS; i++)
        if (S.m[i].theta < 0.2f) return true;
    return false;
}

// --- injection / inspection -------------------------------------------------
void sim_inject(int motor_idx, sim_fault_e fault)
{
    if (motor_idx < 0 || motor_idx >= SIM_NUM_MOTORS) return;
    S.m[motor_idx].fault = fault;
    if (fault == SIM_FAULT_OVERCURRENT || fault == SIM_FAULT_DRIVER ||
        fault == SIM_FAULT_UNDERVOLTAGE) {
        if (S.ssr_on) push_fault_frame(motor_idx);
    }
}

float sim_motor_angle(int i) { return S.m[i].theta; }
float sim_motor_vel(int i) { return S.m[i].vel; }
float sim_vbus(void) { return S.vbus; }
bool sim_pawl_seated(int i) { return S.m[i].pawl_seated; }
