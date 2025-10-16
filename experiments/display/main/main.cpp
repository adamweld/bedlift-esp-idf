#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
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

// Activity tracking
static volatile int64_t last_activity_time = 0;
static volatile bool is_dimmed = false;

// Dev mode flag
static bool dev_flag = false;

// ============================================================================
// Activity Timer
// ============================================================================
void reset_activity_timer(void) {
    last_activity_time = esp_timer_get_time();

    // Restore full brightness if dimmed
    if (is_dimmed) {
        display.setBrightness(BACKLIGHT_FULL);
        is_dimmed = false;
        ESP_LOGI(TAG, "Display brightness restored to full");
    }
}

int64_t get_idle_time_ms(void) {
    return (esp_timer_get_time() - last_activity_time) / 1000;
}

// ============================================================================
// Power Management
// ============================================================================
void perform_shutdown(void) {
    ESP_LOGI(TAG, "Performing shutdown sequence...");

    // TODO: Stop any active motor movements
    // TODO: Save current state/settings to NVS
    // TODO: Disable power outputs
    // TODO: Turn off display backlight

    ESP_LOGI(TAG, "Shutdown complete");
}

void enter_deep_sleep(void) {
    ESP_LOGI(TAG, "Entering deep sleep mode");

    // Perform shutdown tasks
    perform_shutdown();

    // Fade to black from current brightness
    int current_brightness = is_dimmed ? BACKLIGHT_DIMMED : BACKLIGHT_FULL;
    for (int brightness = current_brightness; brightness >= 0; brightness -= 8) {
        display.setBrightness(brightness);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    display.fillScreen(COLOR_BLACK);
    display.setBrightness(0);


    // Wait for any button releases to settle
    vTaskDelay(pdMS_TO_TICKS(100));

    // Generate io_mask for GPIO 1 (MODE) and GPIO 2 (UP)
    uint64_t io_mask = (1ULL << GPIO_BUTTON_MODE) | (1ULL << GPIO_BUTTON_UP);

    // Keep RTC peripherals powered during sleep for GPIO wakeup
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    // Configure RTC GPIO pull-downs for wake pins (keep them LOW when not pressed)
    // This ensures pins stay LOW during deep sleep and only go HIGH when button pressed
    rtc_gpio_pullup_dis((gpio_num_t)GPIO_BUTTON_MODE);
    rtc_gpio_pulldown_en((gpio_num_t)GPIO_BUTTON_MODE);

    rtc_gpio_pullup_dis((gpio_num_t)GPIO_BUTTON_UP);
    rtc_gpio_pulldown_en((gpio_num_t)GPIO_BUTTON_UP);

    ESP_LOGI(TAG, "RTC GPIO pull-downs configured for wake pins");

    // Configure EXT1 wakeup (wake when any pin goes HIGH)
    esp_sleep_enable_ext1_wakeup_io(io_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    ESP_LOGI(TAG, "Wake sources configured: GPIO %d (MODE), GPIO %d (UP)",
             GPIO_BUTTON_MODE, GPIO_BUTTON_UP);

    // Small delay to ensure log is flushed
    vTaskDelay(pdMS_TO_TICKS(100));

    // Enter deep sleep
    esp_deep_sleep_start();
}

// ============================================================================
// Startup Animation
// ============================================================================
void show_startup_animation(void) {
    int32_t cx = SCREEN_WIDTH / 2;
    int32_t cy = SCREEN_HEIGHT / 2;
    // int32_t w = SCREEN_WIDTH;
    // int32_t h = SCREEN_HEIGHT;

    ESP_LOGI(TAG, "Startup animation");

    // Start with pink background
    display.fillScreen(COLOR_PINK);

    // Calculate maximum triangle size to cover entire display
    // For equilateral triangle centered on screen, we need to reach corners
    float max_radius = sqrt(cx * cx + cy * cy) * 1.6f;  // Distance to corner + margin
    int32_t min_size = 15;  // Minimum triangle size

    // Shrinking triangle animation
    for (int32_t size = (int32_t)max_radius; size > min_size; size -= 3) {
        // Calculate equilateral triangle vertices
        float angle1 = -90.0f * 3.14159f / 180.0f;  // Top vertex (pointing up)
        float angle2 = 30.0f * 3.14159f / 180.0f;   // Bottom right
        float angle3 = 150.0f * 3.14159f / 180.0f;  // Bottom left

        int32_t x1 = cx + (int32_t)(size * cos(angle1));
        int32_t y1 = cy + (int32_t)(size * sin(angle1));
        int32_t x2 = cx + (int32_t)(size * cos(angle2));
        int32_t y2 = cy + (int32_t)(size * sin(angle2));
        int32_t x3 = cx + (int32_t)(size * cos(angle3));
        int32_t y3 = cy + (int32_t)(size * sin(angle3));

        // Color pattern: cycle through hues while dimming
        // Start with bright colors, end with dim colors
        float progress = (float)(max_radius - size) / (max_radius - min_size);

        // Hue rotation: start at 330° (pink) and cycle through spectrum
        int hue = (330 + (int)(progress * 360.0f)) % 360;

        // Brightness: start bright, fade to dim
        float brightness = 1.0f - (progress * 0.7f);  // Keep at least 30% brightness

        // Convert HSV to RGB
        float h = hue / 60.0f;
        float c = brightness;
        float x = c * (1.0f - fabs(fmod(h, 2.0f) - 1.0f));

        float r, g, b;
        if (h < 1.0f) { r = c; g = x; b = 0; }
        else if (h < 2.0f) { r = x; g = c; b = 0; }
        else if (h < 3.0f) { r = 0; g = c; b = x; }
        else if (h < 4.0f) { r = 0; g = x; b = c; }
        else if (h < 5.0f) { r = x; g = 0; b = c; }
        else { r = c; g = 0; b = x; }

        uint16_t color = display.color565(
            (uint8_t)(r * 255.0f),
            (uint8_t)(g * 255.0f),
            (uint8_t)(b * 255.0f)
        );

        display.fillTriangle(x1, y1, x2, y2, x3, y3, color);
        vTaskDelay(pdMS_TO_TICKS(STARTUP_ANIMATION_FRAME_DELAY_MS));
    }

    vTaskDelay(pdMS_TO_TICKS(200));
}

// ============================================================================
// Power Control Functions
// ============================================================================
void lock_solenoids(void) {
    ESP_LOGI(TAG, "Locking solenoids - enabling lock power");
    gpio_set_level((gpio_num_t)GPIO_LOCK_POWER, 1);
    ui.getMonitors().lock = true;
    ui.refreshStatusBar();
}

void unlock_solenoids(void) {
    ESP_LOGI(TAG, "Unlocking solenoids - disabling lock power");
    gpio_set_level((gpio_num_t)GPIO_LOCK_POWER, 0);
    ui.getMonitors().lock = false;
    ui.refreshStatusBar();
}

// ============================================================================
// Motor Control Functions
// ============================================================================
void spin_motors(int direction) {
    // Stub function for motor control
    // direction: 1 = up/forward, -1 = down/reverse, 0 = stop
    // TODO: Implement CAN bus motor commands
    static int call_count = 0;
    if (++call_count % 10 == 0) {  // Log every 10th call to avoid spam
        ESP_LOGI(TAG, "spin_motors: direction=%d", direction);
    }
}

// ============================================================================
// GPIO Pin Initialization
// ============================================================================
void init_gpio_pins(void) {
    ESP_LOGI(TAG, "Initializing GPIO pins");

    // Configure motor power load switch (GPIO 5) - output, active high
    gpio_config_t motor_power_conf = {};
    motor_power_conf.intr_type = GPIO_INTR_DISABLE;
    motor_power_conf.mode = GPIO_MODE_OUTPUT;
    motor_power_conf.pin_bit_mask = (1ULL << GPIO_MOTOR_POWER);
    motor_power_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    motor_power_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&motor_power_conf);

    // Configure lock power load switch (GPIO 6) - output, active high
    gpio_config_t lock_power_conf = {};
    lock_power_conf.intr_type = GPIO_INTR_DISABLE;
    lock_power_conf.mode = GPIO_MODE_OUTPUT;
    lock_power_conf.pin_bit_mask = (1ULL << GPIO_LOCK_POWER);
    lock_power_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    lock_power_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&lock_power_conf);

    // Enable motor power on startup
    gpio_set_level((gpio_num_t)GPIO_MOTOR_POWER, 1);
    ESP_LOGI(TAG, "Motor power enabled (GPIO %d)", GPIO_MOTOR_POWER);

    // Keep lock power off initially
    gpio_set_level((gpio_num_t)GPIO_LOCK_POWER, 0);
    ESP_LOGI(TAG, "Lock power disabled (GPIO %d)", GPIO_LOCK_POWER);

    // Update monitor states to reflect hardware
    ui.getMonitors().motors = true;   // Motor power is on
    ui.getMonitors().lock = false;    // Lock power is off
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
    reset_activity_timer();
    OperationMode mode = ui.getMode();
    ESP_LOGI(TAG, "Button UP pressed in mode %d", (int)mode);

    switch (mode) {
        case OperationMode::UP_DOWN:
            ESP_LOGI(TAG, "Up/Down: Move up - unlocking and starting motors");
            unlock_solenoids();
            vTaskDelay(pdMS_TO_TICKS(100));  // Brief delay for locks to release
            break;
        case OperationMode::ROLL:
        case OperationMode::PITCH:
        case OperationMode::TORSION:
            ESP_LOGI(TAG, "%s: Increase", MODE_CONFIGS[(int)mode].name);
            // TODO: Adjust orientation
            break;
        case OperationMode::LEVEL:
            ESP_LOGI(TAG, "Level: Calibrate +");
            // TODO: Calibration adjustment
            break;
        case OperationMode::MOTOR_1:
        case OperationMode::MOTOR_2:
        case OperationMode::MOTOR_3:
        case OperationMode::MOTOR_4:
            ESP_LOGI(TAG, "%s: Forward", MODE_CONFIGS[(int)mode].name);
            // TODO: Individual motor control
            break;
        default:
            break;
    }
}

void handle_button_up_release() {
    reset_activity_timer();
    OperationMode mode = ui.getMode();
    ESP_LOGI(TAG, "Button UP released in mode %d", (int)mode);

    switch (mode) {
        case OperationMode::UP_DOWN:
            ESP_LOGI(TAG, "Up/Down: Stop motors and lock");
            spin_motors(0);  // Stop motors
            vTaskDelay(pdMS_TO_TICKS(50));  // Brief delay for motors to stop
            lock_solenoids();
            break;
        default:
            break;
    }
}

void handle_button_mode_press() {
    reset_activity_timer();
    ESP_LOGI(TAG, "Button MODE pressed - cycling mode");
    ui.cycleMode();
    ui.refreshModePanel();
    ui.refreshButtonPanel();
}

void handle_button_down_press() {
    reset_activity_timer();
    OperationMode mode = ui.getMode();
    ESP_LOGI(TAG, "Button DOWN pressed in mode %d", (int)mode);

    switch (mode) {
        case OperationMode::UP_DOWN:
            ESP_LOGI(TAG, "Up/Down: Move down - unlocking and starting motors");
            unlock_solenoids();
            vTaskDelay(pdMS_TO_TICKS(100));  // Brief delay for locks to release
            break;
        case OperationMode::ROLL:
        case OperationMode::PITCH:
        case OperationMode::TORSION:
            ESP_LOGI(TAG, "%s: Decrease", MODE_CONFIGS[(int)mode].name);
            // TODO: Adjust orientation
            break;
        case OperationMode::LEVEL:
            ESP_LOGI(TAG, "Level: Calibrate -");
            // TODO: Calibration adjustment
            break;
        case OperationMode::MOTOR_1:
        case OperationMode::MOTOR_2:
        case OperationMode::MOTOR_3:
        case OperationMode::MOTOR_4:
            ESP_LOGI(TAG, "%s: Reverse", MODE_CONFIGS[(int)mode].name);
            // TODO: Individual motor control
            break;
        default:
            break;
    }
}

void handle_button_down_release() {
    reset_activity_timer();
    OperationMode mode = ui.getMode();
    ESP_LOGI(TAG, "Button DOWN released in mode %d", (int)mode);

    switch (mode) {
        case OperationMode::UP_DOWN:
            ESP_LOGI(TAG, "Up/Down: Stop motors and lock");
            spin_motors(0);  // Stop motors
            vTaskDelay(pdMS_TO_TICKS(50));  // Brief delay for motors to stop
            lock_solenoids();
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

    const TickType_t motor_spin_period = pdMS_TO_TICKS(50);  // Spin motors every 50ms while held

    while (1) {
        // Wait for button event with timeout to allow continuous motor spinning
        if (xQueueReceive(gpio_event_queue, &gpio_num, motor_spin_period)) {
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
                    } else {
                        handle_button_up_release();
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
                    } else {
                        handle_button_down_release();
                    }
                }
            }
        }

        // Continuously spin motors while buttons are held in UP_DOWN mode
        OperationMode mode = ui.getMode();
        if (mode == OperationMode::UP_DOWN) {
            if (last_up_state) {
                spin_motors(1);  // Spin up
            } else if (last_down_state) {
                spin_motors(-1);  // Spin down
            }
        }
    }
}

// ============================================================================
// Inactivity Monitoring Task
// ============================================================================
void inactivity_monitor_task(void *pvParameter) {
    ESP_LOGI(TAG, "Inactivity monitor task started");

    // Log initial stack high water mark
    UBaseType_t initial_stack = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Inactivity task initial stack HWM: %u bytes", initial_stack);

    while (1) {
        // Check every second
        vTaskDelay(pdMS_TO_TICKS(1000));

        int64_t idle_time_ms = get_idle_time_ms();
        int64_t idle_time_sec = idle_time_ms / 1000;
        int64_t dim_timeout_ms = AUTO_DIM_TIMEOUT_SEC * 1000;
        int64_t sleep_timeout_ms = AUTO_SLEEP_TIMEOUT_SEC * 1000;

        // Log stack usage periodically (every 5 seconds)
        static int counter = 0;
        if (++counter % 5 == 0) {
            UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI(TAG, "Idle: %lld s, Dimmed: %s, Stack HWM: %u bytes",
                     idle_time_sec, is_dimmed ? "YES" : "NO", stack_hwm);
        }

        // Auto-dim after dim timeout (with fade)
        if (!is_dimmed && idle_time_ms >= dim_timeout_ms) {
            ESP_LOGI(TAG, "Dimming display after %d seconds of inactivity", AUTO_DIM_TIMEOUT_SEC);

            // Fade from full to dimmed
            for (int brightness = BACKLIGHT_FULL; brightness >= BACKLIGHT_DIMMED; brightness -= 4) {
                display.setBrightness(brightness);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            display.setBrightness(BACKLIGHT_DIMMED);
            is_dimmed = true;
        }

        // Auto-sleep after sleep timeout
        if (idle_time_ms >= sleep_timeout_ms) {
            ESP_LOGI(TAG, "Inactivity timeout reached (%lld ms), entering sleep mode",
                     idle_time_ms);
            UBaseType_t pre_sleep_stack = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI(TAG, "Pre-sleep stack HWM: %u bytes", pre_sleep_stack);

            enter_deep_sleep();
            // Will not return from deep sleep
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
    display.setBrightness(BACKLIGHT_FULL);

    ESP_LOGI(TAG, "Display initialized: %dx%d", display.width(), display.height());
}

// ============================================================================
// Main Application Entry Point
// ============================================================================
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "BedLift Controller Starting...");

    // Check wake-up reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
        ESP_LOGI(TAG, "Woke up from deep sleep via GPIO");
    } else {
        ESP_LOGI(TAG, "Cold boot or reset");
    }

    // Check for dev mode activation (D1 and D2 held on boot)
    // Configure pins temporarily to read state
    gpio_set_direction((gpio_num_t)GPIO_BUTTON_MODE, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)GPIO_BUTTON_MODE, GPIO_PULLDOWN_ONLY);
    gpio_set_direction((gpio_num_t)GPIO_BUTTON_UP, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)GPIO_BUTTON_UP, GPIO_PULLDOWN_ONLY);

    vTaskDelay(pdMS_TO_TICKS(10));  // Short delay for pins to settle

    bool d1_pressed = (gpio_get_level((gpio_num_t)GPIO_BUTTON_MODE) == 1);
    bool d2_pressed = (gpio_get_level((gpio_num_t)GPIO_BUTTON_UP) == 1);

    if (d1_pressed && d2_pressed) {
        dev_flag = true;
        ESP_LOGI(TAG, "*** DEV MODE ENABLED ***");
    } else {
        dev_flag = false;
        ESP_LOGI(TAG, "Normal mode (dev modes hidden)");
    }

    // Initialize activity timer
    reset_activity_timer();

    // Initialize display
    init_display();

    // Show startup animation
    show_startup_animation();

    // Initialize UI with dev flag
    ui.init(&display, dev_flag);

    // Initialize GPIO pins (power switches)
    init_gpio_pins();

    // Initial refresh (will show motor/lock monitor states)
    ui.refresh();

    // Initialize GPIO buttons
    init_gpio_buttons();

    // Create GPIO event task
    xTaskCreate(gpio_event_task, "gpio_event", 4096, NULL, 5, NULL);

    // Create inactivity monitor task with larger stack for enter_deep_sleep()
    xTaskCreate(inactivity_monitor_task, "inactivity", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "BedLift Controller Running (auto-sleep in %d seconds)",
             AUTO_SLEEP_TIMEOUT_SEC);
}
