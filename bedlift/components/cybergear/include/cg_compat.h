#pragma once

// Platform compatibility: on ESP-IDF targets we use the real TWAI message
// struct and FreeRTOS tick type; on host builds (SDL sim, unit tests) we
// provide layout-compatible stand-ins so the frame builders/parsers — the
// actual protocol logic — compile unchanged.

#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "driver/twai.h"
#else
#include <stdint.h>

typedef uint32_t TickType_t;

typedef struct {
    union {
        struct {
            uint32_t extd : 1;
            uint32_t rtr : 1;
            uint32_t ss : 1;
            uint32_t self : 1;
            uint32_t dlc_non_comp : 1;
            uint32_t reserved : 27;
        };
        uint32_t flags;
    };
    uint32_t identifier;
    uint8_t data_length_code;
    uint8_t data[8];
} twai_message_t;

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERR_INVALID_RESPONSE 0x108
#endif
