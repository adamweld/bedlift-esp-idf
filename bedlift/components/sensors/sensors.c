#include <math.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sensors.h"

static const char *TAG = "sensors";

#define ADXL_REG_DEVID     0x00
#define ADXL_REG_POWER_CTL 0x2D
#define ADXL_REG_DATA_FMT  0x31
#define ADXL_REG_DATA      0x32
#define ADXL_DEVID         0xE5
#define ADXL_LSB_PER_G     256.0f   // full-res, any range

static sensors_cfg_t s_cfg;
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_front, s_rear;
static bool s_front_ok, s_rear_ok;

static sensor_tilt_t s_tf, s_tr;
static bool s_tf_seed, s_tr_seed;

static esp_err_t adxl_add(uint8_t addr, i2c_master_dev_handle_t *dev)
{
    i2c_device_config_t c = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    return i2c_master_bus_add_device(s_bus, &c, dev);
}

static esp_err_t adxl_wr(i2c_master_dev_handle_t d, uint8_t reg, uint8_t v)
{
    uint8_t b[2] = { reg, v };
    return i2c_master_transmit(d, b, 2, 100);
}

static esp_err_t adxl_rd(i2c_master_dev_handle_t d, uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(d, &reg, 1, buf, n, 100);
}

static bool adxl_setup(i2c_master_dev_handle_t d)
{
    uint8_t id = 0;
    if (adxl_rd(d, ADXL_REG_DEVID, &id, 1) != ESP_OK || id != ADXL_DEVID)
        return false;
    adxl_wr(d, ADXL_REG_DATA_FMT, 0x08);   // full resolution, +/-2g
    adxl_wr(d, ADXL_REG_POWER_CTL, 0x08);  // measure mode
    return true;
}

esp_err_t sensors_init(const sensors_cfg_t *cfg)
{
    s_cfg = *cfg;

    // I2C rail power (Feather TFT_I2C_POWER) must be high
    gpio_config_t pwr = { .pin_bit_mask = 1ULL << cfg->i2c_power_pin,
                          .mode = GPIO_MODE_INPUT_OUTPUT };
    gpio_config(&pwr);
    gpio_set_level((gpio_num_t)cfg->i2c_power_pin, 1);

    // ADXL address strap: high -> front @ 0x1D
    gpio_config_t sdo = { .pin_bit_mask = 1ULL << cfg->acc_sdo_pin,
                          .mode = GPIO_MODE_INPUT_OUTPUT };
    gpio_config(&sdo);
    gpio_set_level((gpio_num_t)cfg->acc_sdo_pin, 1);

    // halls: inputs with pull-ups (open-collector sensors)
    gpio_config_t h = { .pin_bit_mask = (1ULL << cfg->hall1_pin) | (1ULL << cfg->hall2_pin),
                        .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&h);

    vTaskDelay(pdMS_TO_TICKS(50));

    i2c_master_bus_config_t bc = {
        .i2c_port = -1, .sda_io_num = cfg->sda, .scl_io_num = cfg->scl,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bc, &s_bus);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2c bus: %s", esp_err_to_name(err)); return err; }

    if (adxl_add(cfg->addr_front, &s_front) == ESP_OK) s_front_ok = adxl_setup(s_front);
    if (adxl_add(cfg->addr_rear, &s_rear) == ESP_OK)   s_rear_ok = adxl_setup(s_rear);
    ESP_LOGI(TAG, "ADXL front %s, rear %s",
             s_front_ok ? "ok" : "MISSING", s_rear_ok ? "ok" : "MISSING");
    return ESP_OK;
}

static float s_raw[2][3];   // [front/rear][x/y/z], filtered g

static void read_one(int idx, i2c_master_dev_handle_t d, bool ok, sensor_tilt_t *t,
                     bool *seed, float dt)
{
    if (!ok) { t->valid = false; return; }
    uint8_t b[6];
    if (adxl_rd(d, ADXL_REG_DATA, b, 6) != ESP_OK) { t->valid = false; return; }
    float x = (int16_t)(b[1] << 8 | b[0]) / ADXL_LSB_PER_G;
    float y = (int16_t)(b[3] << 8 | b[2]) / ADXL_LSB_PER_G;
    float z = (int16_t)(b[5] << 8 | b[4]) / ADXL_LSB_PER_G;
    // filtered raw axes (for orientation calibration)
    float a = *seed ? dt / (s_cfg.lp_tau_s + dt) : 1.0f;
    s_raw[idx][0] += a * (x - s_raw[idx][0]);
    s_raw[idx][1] += a * (y - s_raw[idx][1]);
    s_raw[idx][2] += a * (z - s_raw[idx][2]);
    // Bench-calibrated mapping (2026-09-23): vertical is Y; the rear sensor is
    // rotated 180 deg about X relative to the front (Y and Z inverted, X same).
    // Pitch lives in Z (nose-up = +pitch); roll lives in X. Roll sign set so
    // RIGHT rail high = +roll, matching the motor geometry (idx0,idx1 = right,
    // confirmed at the bench when self-level drove roll the wrong way with the
    // earlier -x sign). Level-zero offsets captured 2026-09-23 with the frame
    // externally leveled, subtracted so a level frame reads 0/0.
    const float PITCH_OFF_FRONT = 4.8f, ROLL_OFF_FRONT = 3.7f;
    const float PITCH_OFF_REAR  = 3.2f, ROLL_OFF_REAR  = 3.7f;
    float pitch, roll;
    if (idx == 0) {                 // front: gravity down = -Y
        pitch = atan2f(-z, -y) * 57.2958f - PITCH_OFF_FRONT;
        roll  = atan2f(x, -y) * 57.2958f - ROLL_OFF_FRONT;
    } else {                        // rear: down = +Y, Z inverted
        pitch = atan2f(z, y) * 57.2958f - PITCH_OFF_REAR;
        roll  = atan2f(x, y) * 57.2958f - ROLL_OFF_REAR;
    }
    if (!*seed) { t->pitch_deg = pitch; t->roll_deg = roll; *seed = true; }
    else {
        t->pitch_deg += a * (pitch - t->pitch_deg);
        t->roll_deg  += a * (roll - t->roll_deg);
    }
    t->valid = true;
}

void sensors_update(float dt_s)
{
    read_one(0, s_front, s_front_ok, &s_tf, &s_tf_seed, dt_s);
    read_one(1, s_rear, s_rear_ok, &s_tr, &s_tr_seed, dt_s);
}

void sensors_get_tilt(sensor_tilt_t *front, sensor_tilt_t *rear)
{
    if (front) *front = s_tf;
    if (rear) *rear = s_tr;
}

void sensors_get_raw(int idx, float *x, float *y, float *z)
{
    if (idx < 0 || idx > 1) return;
    if (x) *x = s_raw[idx][0];
    if (y) *y = s_raw[idx][1];
    if (z) *z = s_raw[idx][2];
}

bool sensors_hall1(void) { return gpio_get_level((gpio_num_t)s_cfg.hall1_pin); }
bool sensors_hall2(void) { return gpio_get_level((gpio_num_t)s_cfg.hall2_pin); }
