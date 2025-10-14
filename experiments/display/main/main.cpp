#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lgfx_config.hpp"

static const char *TAG = "DISPLAY_TEST";

// Global display instance
LGFX display;

// TFT power pin - MUST be enabled for display to work
#define TFT_I2C_POWER 7

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting basic display test...");

    // Enable TFT power (GPIO7 must be HIGH)
    ESP_LOGI(TAG, "Enabling TFT power on GPIO%d...", TFT_I2C_POWER);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << TFT_I2C_POWER);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)TFT_I2C_POWER, 1);  // Set HIGH to enable power

    // Wait a moment for power to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize the display
    display.init();

    ESP_LOGI(TAG, "Display initialized");

    // Set backlight brightness to maximum
    display.setBrightness(255);

    ESP_LOGI(TAG, "Backlight set to maximum");

    // Set rotation (0 = portrait, 1 = landscape, 2 = portrait inverted, 3 = landscape inverted)
    display.setRotation(1);

    ESP_LOGI(TAG, "Rotation set to 1 (landscape)");

    // Get display dimensions
    int width = display.width();
    int height = display.height();
    ESP_LOGI(TAG, "Display size: %dx%d", width, height);

    // Set backlight to mid level
    display.setBrightness(128);
    ESP_LOGI(TAG, "Backlight set to mid level (128)");

    // Cycle through colors
    ESP_LOGI(TAG, "Testing color fills...");

    while (1) {
        ESP_LOGI(TAG, "Filling screen with WHITE...");
        display.fillScreen(TFT_WHITE);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Filling screen with RED...");
        display.fillScreen(TFT_RED);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Filling screen with GREEN...");
        display.fillScreen(TFT_GREEN);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Filling screen with BLUE...");
        display.fillScreen(TFT_BLUE);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Filling screen with YELLOW...");
        display.fillScreen(TFT_YELLOW);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Filling screen with CYAN...");
        display.fillScreen(TFT_CYAN);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Filling screen with MAGENTA...");
        display.fillScreen(TFT_MAGENTA);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Filling screen with BLACK...");
        display.fillScreen(TFT_BLACK);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
