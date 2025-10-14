#pragma once

#include "lgfx_config.hpp"
#include "lvgl.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Global display instance
extern LGFX display;

// TFT power pin - MUST be enabled for display to work
#define TFT_I2C_POWER 7

// LVGL flush callback - draws the buffer to the screen
void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    // Use LovyanGFX to draw the buffer
    display.startWrite();
    display.setAddrWindow(area->x1, area->y1, w, h);
    display.writePixels((lgfx::rgb565_t *)px_map, w * h);
    display.endWrite();

    // Inform LVGL that flushing is complete
    lv_display_flush_ready(disp);
}

// Initialize LVGL with LovyanGFX backend
void lvgl_init(void)
{
    // Enable TFT power (GPIO7 must be HIGH)
    ESP_LOGI("LVGL_DRIVER", "Enabling TFT power on GPIO%d...", TFT_I2C_POWER);
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

    // Initialize LovyanGFX
    display.init();
    display.setRotation(3); // Landscape inverted mode
    display.setBrightness(100); // Set backlight brightness (0-255)

    // Initialize LVGL
    lv_init();

    // Get display dimensions
    uint32_t screen_width = display.width();
    uint32_t screen_height = display.height();

    // Create LVGL display
    lv_display_t *lv_disp = lv_display_create(screen_width, screen_height);

    // Allocate draw buffers (1/10th of screen size for each buffer)
    uint32_t buf_size = screen_width * screen_height / 10;

    static lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    static lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    // Set the buffers
    lv_display_set_buffers(lv_disp, buf1, buf2, buf_size * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Set the flush callback
    lv_display_set_flush_cb(lv_disp, lvgl_flush_cb);
}
