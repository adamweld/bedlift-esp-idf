#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lgfx_config.hpp"
#include "pins.hpp"
#include "config.hpp"
#include "ui_manager.hpp"

static const char *TAG = "BedLift";

// Global instances
LGFX display;
UIManager ui;

// GPIO event queue
static QueueHandle_t gpio_event_queue = NULL;

// ============================================================================
// Startup Animation
// ============================================================================
void show_startup_animation(void) {
    int32_t i, i2;
    int32_t cx = SCREEN_WIDTH / 2;
    int32_t cy = SCREEN_HEIGHT / 2;
    int32_t w = SCREEN_WIDTH;
    int32_t h = SCREEN_HEIGHT;
    int32_t n = (w < h) ? w : h;

    ESP_LOGI(TAG, "Startup animation");

    // Filled round rectangles
    display.fillScreen(COLOR_BLACK);
    for (i = n; i > 10; i -= 3) {
        i2 = i / 2;
        uint16_t color = display.color565(0, i % 256, 0);
        display.fillRoundRect(cx - i2, cy - i2, i, i, i / 8, color);
        vTaskDelay(pdMS_TO_TICKS(STARTUP_ANIMATION_FRAME_DELAY_MS));
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Filled circles
    display.fillScreen(COLOR_BLACK);
    int32_t max_radius = (h < w) ? (h / 2 - 2) : (w / 2 - 2);
    for (i = max_radius; i > 5; i -= 2) {
        uint16_t color = display.color565(0, i * 3, i * 3);
        display.fillCircle(cx, cy, i, color);
        vTaskDelay(pdMS_TO_TICKS(STARTUP_ANIMATION_FRAME_DELAY_MS));
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Filled triangles
    display.fillScreen(COLOR_BLACK);
    int32_t max_tri_size = (cx < cy) ? cx : cy;
    for (i = max_tri_size - 5; i > 10; i -= 3) {
        int32_t x1 = cx;
        int32_t y1 = cy - (i * 2 / 1.732);
        int32_t x2 = cx - i;
        int32_t y2 = cy + (i / 1.732);
        int32_t x3 = cx + i;
        int32_t y3 = cy + (i / 1.732);

        uint16_t color = display.color565(i % 256, i % 256, 0);
        display.fillTriangle(x1, y1, x2, y2, x3, y3, color);
        vTaskDelay(pdMS_TO_TICKS(STARTUP_ANIMATION_FRAME_DELAY_MS));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ============================================================================
// GPIO Interrupt Handling
// ============================================================================
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    gpio_num_t gpio_num = (gpio_num_t)(uint32_t)arg;
    xQueueSendFromISR(gpio_event_queue, &gpio_num, NULL);
}

void init_gpio_buttons(void) {
    // Create event queue
    gpio_event_queue = xQueueCreate(10, sizeof(gpio_num_t));

    // Configure GPIO_BUTTON_UP (D2)
    gpio_config_t io_conf_up = {};
    io_conf_up.intr_type = GPIO_INTR_ANYEDGE;
    io_conf_up.mode = GPIO_MODE_INPUT;
    io_conf_up.pin_bit_mask = (1ULL << GPIO_BUTTON_UP);
    io_conf_up.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf_up.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf_up);

    // Configure GPIO_BUTTON_MODE (D1)
    gpio_config_t io_conf_mode = {};
    io_conf_mode.intr_type = GPIO_INTR_ANYEDGE;
    io_conf_mode.mode = GPIO_MODE_INPUT;
    io_conf_mode.pin_bit_mask = (1ULL << GPIO_BUTTON_MODE);
    io_conf_mode.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf_mode.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf_mode);

    // Configure GPIO_BUTTON_DOWN (D0)
    gpio_config_t io_conf_down = {};
    io_conf_down.intr_type = GPIO_INTR_ANYEDGE;
    io_conf_down.mode = GPIO_MODE_INPUT;
    io_conf_down.pin_bit_mask = (1ULL << GPIO_BUTTON_DOWN);
    io_conf_down.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf_down.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf_down);

    // Install ISR service
    gpio_install_isr_service(0);

    // Attach handlers
    gpio_isr_handler_add((gpio_num_t)GPIO_BUTTON_UP, gpio_isr_handler, (void*)GPIO_BUTTON_UP);
    gpio_isr_handler_add((gpio_num_t)GPIO_BUTTON_MODE, gpio_isr_handler, (void*)GPIO_BUTTON_MODE);
    gpio_isr_handler_add((gpio_num_t)GPIO_BUTTON_DOWN, gpio_isr_handler, (void*)GPIO_BUTTON_DOWN);

    ESP_LOGI(TAG, "GPIO buttons initialized");
}

// ============================================================================
// Button Event Handlers
// ============================================================================
void handle_button_up_press() {
    OperationMode mode = ui.getMode();
    ESP_LOGI(TAG, "Button UP pressed in mode %d", (int)mode);

    switch (mode) {
        case OperationMode::MANUAL:
            ESP_LOGI(TAG, "Manual: Move up");
            // TODO: Motor control - move up
            break;
        case OperationMode::AUTO:
            ESP_LOGI(TAG, "Auto: Increase target");
            // TODO: Adjust target position
            break;
        case OperationMode::PRESET_1:
        case OperationMode::PRESET_2:
            ESP_LOGI(TAG, "Preset: Save");
            // TODO: Save current position to preset
            break;
        case OperationMode::LEVEL:
            ESP_LOGI(TAG, "Level: Calibrate +");
            // TODO: Calibration adjustment
            break;
        default:
            break;
    }
}

void handle_button_mode_press() {
    ESP_LOGI(TAG, "Button MODE pressed - cycling mode");
    ui.cycleMode();
    ui.refreshModePanel();
    ui.refreshButtonPanel();
}

void handle_button_down_press() {
    OperationMode mode = ui.getMode();
    ESP_LOGI(TAG, "Button DOWN pressed in mode %d", (int)mode);

    switch (mode) {
        case OperationMode::MANUAL:
            ESP_LOGI(TAG, "Manual: Move down");
            // TODO: Motor control - move down
            break;
        case OperationMode::AUTO:
            ESP_LOGI(TAG, "Auto: Decrease target");
            // TODO: Adjust target position
            break;
        case OperationMode::PRESET_1:
        case OperationMode::PRESET_2:
            ESP_LOGI(TAG, "Preset: Load");
            // TODO: Load position from preset
            break;
        case OperationMode::LEVEL:
            ESP_LOGI(TAG, "Level: Calibrate -");
            // TODO: Calibration adjustment
            break;
        default:
            break;
    }
}

// ============================================================================
// GPIO Event Processing Task
// ============================================================================
void gpio_event_task(void *pvParameter) {
    gpio_num_t gpio_num;
    static bool last_up_state = false;
    static bool last_mode_state = false;
    static bool last_down_state = false;

    ESP_LOGI(TAG, "GPIO event task started");

    // Initialize button states
    last_up_state = (gpio_get_level((gpio_num_t)GPIO_BUTTON_UP) == !BUTTON_UP_ACTIVE_LOW);
    last_mode_state = (gpio_get_level((gpio_num_t)GPIO_BUTTON_MODE) == !BUTTON_MODE_ACTIVE_LOW);
    last_down_state = (gpio_get_level((gpio_num_t)GPIO_BUTTON_DOWN) == !BUTTON_DOWN_ACTIVE_LOW);

    ui.setButtonState(0, last_up_state);
    ui.setButtonState(1, last_mode_state);
    ui.setButtonState(2, last_down_state);

    while (1) {
        if (xQueueReceive(gpio_event_queue, &gpio_num, portMAX_DELAY)) {
            // Debounce delay
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));

            // Read current state
            int level = gpio_get_level((gpio_num_t)gpio_num);

            // Process based on which button
            if (gpio_num == GPIO_BUTTON_UP) {
                bool pressed = (level == !BUTTON_UP_ACTIVE_LOW);
                if (pressed != last_up_state) {
                    last_up_state = pressed;
                    ui.setButtonState(0, pressed);
                    ui.refreshButtonPanel();
                    if (pressed) {
                        handle_button_up_press();
                    }
                }
            } else if (gpio_num == GPIO_BUTTON_MODE) {
                bool pressed = (level == !BUTTON_MODE_ACTIVE_LOW);
                if (pressed != last_mode_state) {
                    last_mode_state = pressed;
                    ui.setButtonState(1, pressed);
                    ui.refreshButtonPanel();
                    if (pressed) {
                        handle_button_mode_press();
                    }
                }
            } else if (gpio_num == GPIO_BUTTON_DOWN) {
                bool pressed = (level == !BUTTON_DOWN_ACTIVE_LOW);
                if (pressed != last_down_state) {
                    last_down_state = pressed;
                    ui.setButtonState(2, pressed);
                    ui.refreshButtonPanel();
                    if (pressed) {
                        handle_button_down_press();
                    }
                }
            }
        }
    }
}

// ============================================================================
// Display Initialization
// ============================================================================
void init_display(void) {
    // Enable TFT power
    ESP_LOGI(TAG, "Enabling TFT power on GPIO%d", TFT_I2C_POWER);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << TFT_I2C_POWER);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)TFT_I2C_POWER, 1);

    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize display
    display.init();
    display.setRotation(SCREEN_ROTATION);
    display.setBrightness(128);

    ESP_LOGI(TAG, "Display initialized: %dx%d", display.width(), display.height());
}

// ============================================================================
// Main Application Entry Point
// ============================================================================
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "BedLift Controller Starting...");

    // Initialize display
    init_display();

    // Show startup animation
    show_startup_animation();

    // Initialize UI
    ui.init(&display);

    // Initial refresh
    ui.refresh();

    // Initialize GPIO buttons
    init_gpio_buttons();

    // Create GPIO event task
    xTaskCreate(gpio_event_task, "gpio_event", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "BedLift Controller Running");
}
