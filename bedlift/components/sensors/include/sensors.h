#ifndef BEDLIFT_SENSORS_H
#define BEDLIFT_SENSORS_H

// Dual ADXL345 tilt + hall endstops, raw IDF I2C (no espp dependency).
// Reads are filtered (EMA) and served as pitch/roll degrees.

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int sda, scl, i2c_power_pin, acc_sdo_pin, hall1_pin, hall2_pin;
    uint8_t addr_front, addr_rear;
    float lp_tau_s;
} sensors_cfg_t;

esp_err_t sensors_init(const sensors_cfg_t *cfg);

// Sample both accels, update filters. Call at the sensor-task rate (~50 Hz).
void sensors_update(float dt_s);

typedef struct { float pitch_deg, roll_deg; bool valid; } sensor_tilt_t;

void sensors_get_tilt(sensor_tilt_t *front, sensor_tilt_t *rear);
// Raw filtered acceleration (g), for orientation calibration. idx 0=front, 1=rear.
void sensors_get_raw(int idx, float *x, float *y, float *z);
bool sensors_hall1(void);
bool sensors_hall2(void);

#ifdef __cplusplus
}
#endif

#endif
