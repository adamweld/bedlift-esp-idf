#ifndef BEDLIFT_CANBUS_H
#define BEDLIFT_CANBUS_H

// TWAI owner + CyberGear motor fleet. Installs the bus, registers itself as
// the fork's transport, runs the sole RX task (parses every frame into the
// four motor objects), and exposes coherent telemetry snapshots.

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "cybergear.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CANBUS_NUM_MOTORS 4

// tx/rx: TWAI GPIOs. master_id: host CAN id. motor_ids: 4 logical->CAN ids.
esp_err_t canbus_init(int tx, int rx, uint8_t master_id, const uint8_t *motor_ids);
cybergear_motor_t *canbus_motor(int idx);   // idx 0..3 -> logical FL/FR/RL/RR

// Snapshot a motor's current telemetry (thread-safe copy under the RX lock).
void canbus_motor_snapshot(int idx, cybergear_status_t *st, uint32_t *faults);

// TWAI health for the safety/telemetry-loss checks.
void canbus_bus_status(uint32_t *tx_err, uint32_t *rx_err, int *state);

#ifdef __cplusplus
}
#endif

#endif
