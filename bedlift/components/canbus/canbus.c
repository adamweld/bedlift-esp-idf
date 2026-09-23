#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "canbus.h"

static const char *TAG = "canbus";

static uint8_t k_ids[CANBUS_NUM_MOTORS];
static cybergear_motor_t s_motors[CANBUS_NUM_MOTORS];
static SemaphoreHandle_t s_lock;   // guards motor objects (RX writer vs readers)

// Fork transport: called for every command frame. TWAI TX is thread-safe.
static esp_err_t cb_send(const twai_message_t *msg, TickType_t ticks, void *ctx)
{
    (void)ctx;
    twai_message_t m = *msg;
    return twai_transmit(&m, ticks);
}

static void rx_task(void *arg)
{
    (void)arg;
    twai_message_t m;
    for (;;) {
        if (twai_receive(&m, pdMS_TO_TICKS(50)) != ESP_OK) continue;
        int64_t now = esp_timer_get_time();
        xSemaphoreTake(s_lock, portMAX_DELAY);
        for (int i = 0; i < CANBUS_NUM_MOTORS; i++)
            if (cybergear_process_message(&s_motors[i], &m, now) != ESP_ERR_NOT_FOUND)
                break;
        xSemaphoreGive(s_lock);
    }
}

esp_err_t canbus_init(int tx, int rx, uint8_t master_id, const uint8_t *motor_ids)
{
    for (int i = 0; i < CANBUS_NUM_MOTORS; i++) k_ids[i] = motor_ids[i];

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)tx, (gpio_num_t)rx, TWAI_MODE_NORMAL);
    g.rx_queue_len = 32;
    g.tx_queue_len = 16;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g, &t, &f);
    if (err != ESP_OK) { ESP_LOGE(TAG, "install: %s", esp_err_to_name(err)); return err; }
    err = twai_start();
    if (err != ESP_OK) { ESP_LOGE(TAG, "start: %s", esp_err_to_name(err)); return err; }

    s_lock = xSemaphoreCreateMutex();
    for (int i = 0; i < CANBUS_NUM_MOTORS; i++)
        cybergear_init(&s_motors[i], master_id, k_ids[i], pdMS_TO_TICKS(20));
    cybergear_set_transport(cb_send, NULL);

    // core 1, high priority: sole RX consumer
    xTaskCreatePinnedToCore(rx_task, "can_rx", 4096, NULL, 21, NULL, 1);
    ESP_LOGI(TAG, "up: TX=IO%d RX=IO%d 1Mbps", tx, rx);
    return ESP_OK;
}

cybergear_motor_t *canbus_motor(int idx)
{
    return (idx >= 0 && idx < CANBUS_NUM_MOTORS) ? &s_motors[idx] : NULL;
}

void canbus_motor_snapshot(int idx, cybergear_status_t *st, uint32_t *faults)
{
    if (idx < 0 || idx >= CANBUS_NUM_MOTORS) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (st) *st = s_motors[idx].status;
    if (faults) *faults = s_motors[idx].faults;
    xSemaphoreGive(s_lock);
}

void canbus_bus_status(uint32_t *tx_err, uint32_t *rx_err, int *state)
{
    twai_status_info_t s;
    if (twai_get_status_info(&s) != ESP_OK) return;
    if (tx_err) *tx_err = s.tx_error_counter;
    if (rx_err) *rx_err = s.rx_error_counter;
    if (state) *state = s.state;
}
