#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lgfx_config.hpp"

static const char *TAG = "BUTTON_UI";

// Global display instance
LGFX display;

// TFT power pin - MUST be enabled for display to work
#define TFT_I2C_POWER 7

// GPIO pin definitions - mapping to hardware buttons
#define GPIO_D0 0  // D0 - normally high (active low)
#define GPIO_D1 1  // D1 - normally low (active high)
#define GPIO_D2 2  // D2 - normally low (active high)

// Button UI configuration
#define BUTTON_WIDTH 50
#define BUTTON_SPACING 0

// Startup Animation
#define STARTUP_FRAME_DELAY pdMS_TO_TICKS(2)

// Unified button state structure
typedef struct {
    bool pressed;           // Current pressed state (unified for physical and virtual)
    bool last_pressed;      // Previous pressed state for edge detection
    LGFX_Button* ui_button; // Pointer to UI button object
    gpio_num_t gpio_pin;    // Associated GPIO pin
    bool active_low;        // True if button is active low
} button_state_t;

// Button state instances
button_state_t button_d0;
button_state_t button_d1;
button_state_t button_d2;

// LovyanGFX Button instances
LGFX_Button btn_d0;
LGFX_Button btn_d1;
LGFX_Button btn_d2;

// Queue for GPIO events
static QueueHandle_t gpio_event_queue = NULL;

// GPIO interrupt handler
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    gpio_num_t gpio_num = (gpio_num_t)(uint32_t)arg;
    xQueueSendFromISR(gpio_event_queue, &gpio_num, NULL);
}

// Custom draw callback for rectangular buttons (no rounded corners)
void draw_rectangular_button(LovyanGFX *gfx, int32_t x, int32_t y, int32_t w, int32_t h, bool invert, const char* long_name)
{
    uint16_t fill_color = invert ? TFT_WHITE : TFT_DARKGREY;
    uint16_t outline_color = invert ? TFT_DARKGREY : TFT_WHITE;
    uint16_t text_color = invert ? TFT_BLACK : TFT_WHITE;

    // Draw rectangular button
    gfx->fillRect(x, y, w, h, fill_color);
    gfx->drawRect(x, y, w, h, outline_color);

    // Draw label text
    gfx->setTextColor(text_color);
    gfx->setTextDatum(middle_center);
    gfx->setTextSize(1.5);
    gfx->drawString(long_name, x + w / 2, y + h / 2);
}

// Update unified button state and UI
void update_button_state(button_state_t* btn_state)
{
    // Read current GPIO level
    int gpio_level = gpio_get_level(btn_state->gpio_pin);

    // Determine pressed state based on active level
    bool new_pressed = btn_state->active_low ? (gpio_level == 0) : (gpio_level == 1);

    // Update state if changed
    if (new_pressed != btn_state->pressed) {
        btn_state->last_pressed = btn_state->pressed;
        btn_state->pressed = new_pressed;

        // Get button label
        const char* label = (btn_state->gpio_pin == GPIO_D0) ? "D0" :
                           (btn_state->gpio_pin == GPIO_D1) ? "D1" : "D2";

        // Update UI button state and redraw with label
        btn_state->ui_button->press(btn_state->pressed);
        btn_state->ui_button->drawButton(btn_state->pressed, label);

        // Log state change
        ESP_LOGI(TAG, "%s %s (GPIO%d=%d)", label,
                 btn_state->pressed ? "PRESSED" : "RELEASED",
                 btn_state->gpio_pin, gpio_level);
    }
}

// Initialize GPIO pins for button inputs with interrupts
void init_gpio_buttons(void)
{
    // Create queue for GPIO events
    gpio_event_queue = xQueueCreate(10, sizeof(gpio_num_t));

    // Configure D0 (active low with pull-up)
    gpio_config_t io_conf_d0 = {};
    io_conf_d0.intr_type = GPIO_INTR_ANYEDGE;  // Interrupt on both edges
    io_conf_d0.mode = GPIO_MODE_INPUT;
    io_conf_d0.pin_bit_mask = (1ULL << GPIO_D0);
    io_conf_d0.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf_d0.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf_d0);

    // Configure D1 (active high with pull-down)
    gpio_config_t io_conf_d1 = {};
    io_conf_d1.intr_type = GPIO_INTR_ANYEDGE;  // Interrupt on both edges
    io_conf_d1.mode = GPIO_MODE_INPUT;
    io_conf_d1.pin_bit_mask = (1ULL << GPIO_D1);
    io_conf_d1.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf_d1.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf_d1);

    // Configure D2 (active high with pull-down)
    gpio_config_t io_conf_d2 = {};
    io_conf_d2.intr_type = GPIO_INTR_ANYEDGE;  // Interrupt on both edges
    io_conf_d2.mode = GPIO_MODE_INPUT;
    io_conf_d2.pin_bit_mask = (1ULL << GPIO_D2);
    io_conf_d2.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf_d2.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf_d2);

    // Install GPIO ISR service
    gpio_install_isr_service(0);

    // Attach interrupt handlers
    gpio_isr_handler_add((gpio_num_t)GPIO_D0, gpio_isr_handler, (void*)GPIO_D0);
    gpio_isr_handler_add((gpio_num_t)GPIO_D1, gpio_isr_handler, (void*)GPIO_D1);
    gpio_isr_handler_add((gpio_num_t)GPIO_D2, gpio_isr_handler, (void*)GPIO_D2);

    ESP_LOGI(TAG, "GPIO buttons initialized with interrupts: D0=%d (active low), D1=%d (active high), D2=%d (active high)",
             GPIO_D0, GPIO_D1, GPIO_D2);
}

// Startup animation with filled shapes
void show_startup_screen(void)
{
    int32_t i, i2;
    int32_t cx = display.width() / 2;
    int32_t cy = display.height() / 2;
    int32_t w = display.width();
    int32_t h = display.height();
    int32_t n = (w < h) ? w : h;

    ESP_LOGI(TAG, "Drawing startup animation...");

    // Filled round rectangles (largest to smallest)
    display.fillScreen(TFT_BLACK);
    for (i = n; i > 10; i -= 3)
    {
        i2 = i / 2;
        uint16_t color = display.color565(0, i % 256, 0);
        display.fillRoundRect(cx - i2, cy - i2, i, i, i / 8, color);
        vTaskDelay(STARTUP_FRAME_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Filled circles (largest to smallest, constrained by height)
    display.fillScreen(TFT_BLACK);
    int32_t max_radius = (h < w) ? (h / 2 - 2) : (w / 2 - 2);
    for (i = max_radius; i > 5; i -= 2)
    {
        uint16_t color = display.color565(0, i * 3, i * 3);
        display.fillCircle(cx, cy, i, color);
        vTaskDelay(STARTUP_FRAME_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Filled triangles (largest to smallest, equilateral, centered)
    display.fillScreen(TFT_BLACK);
    int32_t max_tri_size = (cx < cy) ? cx : cy;
    for (i = max_tri_size - 5; i > 10; i -= 3)
    {
        // Equilateral triangle centered at (cx, cy)
        // Height from center to top vertex: (2/3) * altitude = (2/3) * (sqrt(3)/2) * side
        // For radius i (distance from center to vertex): side = i * 2 / sqrt(3)
        // Simplified: vertices at 120° intervals from center
        int32_t x1 = cx;
        int32_t y1 = cy - (i * 2 / 1.732);  // Top vertex (sqrt(3) ≈ 1.732)
        int32_t x2 = cx - i;
        int32_t y2 = cy + (i / 1.732);      // Bottom left vertex
        int32_t x3 = cx + i;
        int32_t y3 = cy + (i / 1.732);      // Bottom right vertex

        uint16_t color = display.color565(i % 256, i % 256, 0);
        display.fillTriangle(x1, y1, x2, y2, x3, y3, color);
        vTaskDelay(STARTUP_FRAME_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// GPIO event handling task
void gpio_event_task(void *pvParameter)
{
    gpio_num_t gpio_num;

    ESP_LOGI(TAG, "GPIO event task started");

    // Initialize button states by reading current GPIO levels
    update_button_state(&button_d0);
    update_button_state(&button_d1);
    update_button_state(&button_d2);

    while (1) {
        // Wait for GPIO event from queue
        if (xQueueReceive(gpio_event_queue, &gpio_num, portMAX_DELAY)) {
            // Small debounce delay
            vTaskDelay(pdMS_TO_TICKS(50));

            // Update appropriate button state
            switch (gpio_num) {
                case GPIO_D0:
                    update_button_state(&button_d0);
                    break;
                case GPIO_D1:
                    update_button_state(&button_d1);
                    break;
                case GPIO_D2:
                    update_button_state(&button_d2);
                    break;
                default:
                    break;
            }
        }
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting button UI test...");

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
    display.setRotation(3);  // Landscape mode
    display.setBrightness(128);

    // Get display dimensions
    int width = display.width();
    int height = display.height();
    ESP_LOGI(TAG, "Display initialized: %dx%d", width, height);

    // Show startup animation
    show_startup_screen();

    // Clear screen to black for button UI
    display.fillScreen(TFT_BLACK);

    // Calculate button positions (right side of screen, bottom to top: D0, D1, D2)
    // Buttons stretch from top to bottom with equal spacing
    int button_x = width - BUTTON_WIDTH - BUTTON_SPACING;

    // Calculate total available height and divide equally among 3 buttons
    int total_height = height - (2 * BUTTON_SPACING);  // Spacing at top and bottom
    int button_height = (total_height - (2 * BUTTON_SPACING)) / 3;  // 2 gaps between 3 buttons

    int button_y_d2 = BUTTON_SPACING;  // Top
    int button_y_d1 = button_y_d2 + button_height + BUTTON_SPACING;  // Middle
    int button_y_d0 = button_y_d1 + button_height + BUTTON_SPACING;  // Bottom

    // Initialize button D0 (bottom)
    btn_d0.initButtonUL(&display, button_x, button_y_d0, BUTTON_WIDTH, button_height,
                        TFT_WHITE, TFT_DARKGREY, TFT_WHITE, "D0", 1.5f);
    btn_d0.setDrawCb(draw_rectangular_button);
    btn_d0.drawButton(false, "D0");

    // Initialize button D1 (middle)
    btn_d1.initButtonUL(&display, button_x, button_y_d1, BUTTON_WIDTH, button_height,
                        TFT_WHITE, TFT_DARKGREY, TFT_WHITE, "D1", 1.5f);
    btn_d1.setDrawCb(draw_rectangular_button);
    btn_d1.drawButton(false, "D1");

    // Initialize button D2 (top)
    btn_d2.initButtonUL(&display, button_x, button_y_d2, BUTTON_WIDTH, button_height,
                        TFT_WHITE, TFT_DARKGREY, TFT_WHITE, "D2", 1.5f);
    btn_d2.setDrawCb(draw_rectangular_button);
    btn_d2.drawButton(false, "D2");

    ESP_LOGI(TAG, "Button layout: height=%d, y_positions=[%d,%d,%d]",
             button_height, button_y_d2, button_y_d1, button_y_d0);

    // Initialize unified button states
    button_d0.pressed = false;
    button_d0.last_pressed = false;
    button_d0.ui_button = &btn_d0;
    button_d0.gpio_pin = (gpio_num_t)GPIO_D0;
    button_d0.active_low = true;

    button_d1.pressed = false;
    button_d1.last_pressed = false;
    button_d1.ui_button = &btn_d1;
    button_d1.gpio_pin = (gpio_num_t)GPIO_D1;
    button_d1.active_low = false;

    button_d2.pressed = false;
    button_d2.last_pressed = false;
    button_d2.ui_button = &btn_d2;
    button_d2.gpio_pin = (gpio_num_t)GPIO_D2;
    button_d2.active_low = false;

    ESP_LOGI(TAG, "UI initialized");

    // Initialize GPIO buttons with interrupts
    init_gpio_buttons();

    // Create GPIO event handling task
    xTaskCreate(gpio_event_task, "gpio_event", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Button UI running with interrupt-driven state management");
}
