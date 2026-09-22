#pragma once

// Carrier-board pin map (see bedlift-pcb layouts/default/default.kicad_pcb)
// Matches experiments/display/main/pins.hpp

#define GPIO_MOTOR_POWER 5   // MOTOR_EN -> K2..K5 (24V motor SSRs, paralleled)
#define GPIO_LOCK_POWER  6   // LOCK_EN  -> K1 (12V lock SSR)

#define GPIO_I2C_SDA 3
#define GPIO_I2C_SCL 4
#define I2C_FREQ_HZ  400000
#define GPIO_I2C_POWER 7     // Feather TFT_I2C_POWER: gates I2C rail + pull-ups, must be high

#define GPIO_ACC_SDO 8       // ADXL345 SDO/ALT-ADDRESS on J7 (high -> 0x1D)
#define ADXL_ADDR_FRONT 0x1D // SDO high (J7, driven by GPIO_ACC_SDO)
#define ADXL_ADDR_REAR  0x53 // SDO low/floating (e.g. Grove-attached unit)

// Bench-verified 2026-09-22: the physical hall-1 connector toggles IO11 and
// hall-2 toggles IO9 — opposite of the PCB net names HALL_1/HALL_2. Firmware
// follows the physical labeling; fix the net/silk names in the next PCB rev.
#define GPIO_HALL_1 11
#define GPIO_HALL_2 9

// CAN: bench-verified 2026-09-22 with the M5Stack CAN unit (CA-IS3050G) on a
// straight Grove cable — TX=39/RX=38 talks to the CyberGears cleanly; the
// PCB net names (IO38=CAN_RX, IO39=CAN_TX) were correct all along.
#define GPIO_CAN_TX 39
#define GPIO_CAN_RX 38

// Feather on-module buttons (not on carrier headers)
#define GPIO_BTN_D0 0        // active low, needs pull-up
#define GPIO_BTN_D1 1        // active high
#define GPIO_BTN_D2 2        // active high
