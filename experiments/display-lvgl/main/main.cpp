#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lvgl_driver.hpp"

static const char *TAG = "LVGL_DEMO";

// Global display instance (required by lvgl_driver.hpp)
LGFX display;

// GPIO pin definitions for buttons
// Note: Adjust these GPIO numbers based on your actual D0, D1, D2 pinout
#define GPIO_D0 0  // D0 - active low (high when idle, low when pressed)
#define GPIO_D1 1  // D1 - active high (low when idle, high when pressed)
#define GPIO_D2 2  // D2 - active high (low when idle, high when pressed)

// Button configuration
#define BUTTON_WIDTH 60  // Configurable button width
#define BUTTON_SPACING 5 // Spacing between buttons

// LVGL button objects
static lv_obj_t *btn_d0 = NULL;
static lv_obj_t *btn_d1 = NULL;
static lv_obj_t *btn_d2 = NULL;

// Initialize GPIO pins for button inputs (polling mode)
void init_gpio_buttons(void)
{
    // Configure D0 (active low with pull-up)
    gpio_config_t io_conf_d0 = {};
    io_conf_d0.intr_type = GPIO_INTR_DISABLE;  // No interrupts, use polling
    io_conf_d0.mode = GPIO_MODE_INPUT;
    io_conf_d0.pin_bit_mask = (1ULL << GPIO_D0);
    io_conf_d0.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf_d0.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf_d0);

    // Configure D1 (active high with pull-down)
    gpio_config_t io_conf_d1 = {};
    io_conf_d1.intr_type = GPIO_INTR_DISABLE;  // No interrupts, use polling
    io_conf_d1.mode = GPIO_MODE_INPUT;
    io_conf_d1.pin_bit_mask = (1ULL << GPIO_D1);
    io_conf_d1.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf_d1.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf_d1);

    // Configure D2 (active high with pull-down)
    gpio_config_t io_conf_d2 = {};
    io_conf_d2.intr_type = GPIO_INTR_DISABLE;  // No interrupts, use polling
    io_conf_d2.mode = GPIO_MODE_INPUT;
    io_conf_d2.pin_bit_mask = (1ULL << GPIO_D2);
    io_conf_d2.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf_d2.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf_d2);

    ESP_LOGI(TAG, "GPIO buttons initialized (polling mode): D0=%d, D1=%d, D2=%d", GPIO_D0, GPIO_D1, GPIO_D2);
}

// LVGL timer task with GPIO polling
void lvgl_timer_task(void *pvParameter)
{
    static bool last_d0_state = false;
    static bool last_d1_state = false;
    static bool last_d2_state = false;
    static int poll_count = 0;

    ESP_LOGI(TAG, "LVGL timer task started");

    while (1) {
        // Poll GPIO states
        int d0_level = gpio_get_level((gpio_num_t)GPIO_D0);
        int d1_level = gpio_get_level((gpio_num_t)GPIO_D1);
        int d2_level = gpio_get_level((gpio_num_t)GPIO_D2);

        // Debug: Log GPIO levels periodically
        if (++poll_count % 200 == 0) {
            ESP_LOGI(TAG, "GPIO levels: D0=%d D1=%d D2=%d", d0_level, d1_level, d2_level);
        }

        // Determine current button states
        bool d0_pressed = (d0_level == 0);  // Active low
        bool d1_pressed = (d1_level == 1);  // Active high
        bool d2_pressed = (d2_level == 1);  // Active high

        // Update button visual states if they changed
        if (d0_pressed != last_d0_state) {
            last_d0_state = d0_pressed;
            if (d0_pressed) {
                lv_obj_add_state(btn_d0, LV_STATE_PRESSED);
                ESP_LOGI(TAG, "D0 PRESSED");
            } else {
                lv_obj_remove_state(btn_d0, LV_STATE_PRESSED);
                ESP_LOGI(TAG, "D0 RELEASED");
            }
            lv_obj_invalidate(btn_d0);  // Force redraw
        }

        if (d1_pressed != last_d1_state) {
            last_d1_state = d1_pressed;
            if (d1_pressed) {
                lv_obj_add_state(btn_d1, LV_STATE_PRESSED);
                ESP_LOGI(TAG, "D1 PRESSED");
            } else {
                lv_obj_remove_state(btn_d1, LV_STATE_PRESSED);
                ESP_LOGI(TAG, "D1 RELEASED");
            }
            lv_obj_invalidate(btn_d1);  // Force redraw
        }

        if (d2_pressed != last_d2_state) {
            last_d2_state = d2_pressed;
            if (d2_pressed) {
                lv_obj_add_state(btn_d2, LV_STATE_PRESSED);
                ESP_LOGI(TAG, "D2 PRESSED");
            } else {
                lv_obj_remove_state(btn_d2, LV_STATE_PRESSED);
                ESP_LOGI(TAG, "D2 RELEASED");
            }
            lv_obj_invalidate(btn_d2);  // Force redraw
        }

        // Update LVGL timers and tasks
        uint32_t delay = lv_timer_handler();

        // Sleep for the time recommended by LVGL (or minimum 5ms for responsive button polling)
        vTaskDelay(pdMS_TO_TICKS(delay < 5 ? 5 : delay));
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Sleeping...");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Starting LVGL button display...");

    // Initialize GPIO buttons
    init_gpio_buttons();

    // Initialize LVGL with LovyanGFX backend
    lvgl_init();

    ESP_LOGI(TAG, "LVGL initialized, creating UI...");

    // Set screen background to black
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_black(), LV_PART_MAIN);

    // Get screen dimensions (should be 240x135 in landscape inverted)
    int screen_width = lv_obj_get_width(lv_screen_active());
    int screen_height = lv_obj_get_height(lv_screen_active());

    // Calculate button dimensions - three buttons taking equal vertical space
    int button_height = (screen_height - (4 * BUTTON_SPACING)) / 3;
    int button_x_pos = screen_width - BUTTON_WIDTH - BUTTON_SPACING;

    ESP_LOGI(TAG, "Screen: %dx%d, Button size: %dx%d", screen_width, screen_height, BUTTON_WIDTH, button_height);

    // Create style for pressed buttons
    static lv_style_t style_pressed;
    lv_style_init(&style_pressed);
    lv_style_set_bg_color(&style_pressed, lv_color_hex(0xFF0000));  // Red when pressed
    lv_style_set_shadow_width(&style_pressed, 0);
    lv_style_set_transform_width(&style_pressed, -3);
    lv_style_set_transform_height(&style_pressed, -3);

    // Create button D2 (top)
    btn_d2 = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn_d2, BUTTON_WIDTH, button_height);
    lv_obj_set_pos(btn_d2, button_x_pos, BUTTON_SPACING);
    lv_obj_add_style(btn_d2, &style_pressed, LV_STATE_PRESSED);

    lv_obj_t *label_d2 = lv_label_create(btn_d2);
    lv_label_set_text(label_d2, "D2");
    lv_obj_center(label_d2);

    // Create button D1 (middle)
    btn_d1 = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn_d1, BUTTON_WIDTH, button_height);
    lv_obj_set_pos(btn_d1, button_x_pos, BUTTON_SPACING * 2 + button_height);
    lv_obj_add_style(btn_d1, &style_pressed, LV_STATE_PRESSED);

    lv_obj_t *label_d1 = lv_label_create(btn_d1);
    lv_label_set_text(label_d1, "D1");
    lv_obj_center(label_d1);

    // Create button D0 (bottom)
    btn_d0 = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn_d0, BUTTON_WIDTH, button_height);
    lv_obj_set_pos(btn_d0, button_x_pos, BUTTON_SPACING * 3 + button_height * 2);
    lv_obj_add_style(btn_d0, &style_pressed, LV_STATE_PRESSED);

    lv_obj_t *label_d0 = lv_label_create(btn_d0);
    lv_label_set_text(label_d0, "D0");
    lv_obj_center(label_d0);

    ESP_LOGI(TAG, "UI created successfully!");

    // Create LVGL timer task
    xTaskCreate(lvgl_timer_task, "lvgl_timer", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Button display running...");
}
