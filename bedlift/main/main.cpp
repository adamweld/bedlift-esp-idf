// Bedlift production firmware — M0 scaffold.
// Boot order is safety-first: force both power-gate outputs low before
// anything else can run, then bring up the display and report status.

#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"

#include "app_config.hpp"
#include "lgfx_config.hpp"
#include "cybergear.h"

static const char *TAG = "bedlift";

static LGFX display;

// First thing after boot: SSR and lock-rail gates low (H9 fail-safe posture).
static void outputs_safe(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_SSR_EN) | (1ULL << PIN_LOCK_EN),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);
    gpio_set_level((gpio_num_t)PIN_MOTOR_SSR_EN, 0);
    gpio_set_level((gpio_num_t)PIN_LOCK_EN, 0);
    // released again on the way into deep sleep after re-driving low
    gpio_hold_dis((gpio_num_t)PIN_MOTOR_SSR_EN);
    gpio_hold_dis((gpio_num_t)PIN_LOCK_EN);
}

static void display_init(void)
{
    gpio_config_t pwr = {
        .pin_bit_mask = (1ULL << PIN_I2C_POWER),
        .mode = GPIO_MODE_INPUT_OUTPUT,
    };
    gpio_config(&pwr);
    gpio_set_level((gpio_num_t)PIN_I2C_POWER, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    display.init();
    display.setRotation(TFT_SCREEN_ROTATION);
    display.setBrightness(BACKLIGHT_FULL);
}

static void splash(int selftest_failures)
{
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_WHITE);
    display.setTextSize(2);
    display.setTextDatum(lgfx::middle_center);
    display.drawString("BEDLIFT", display.width() / 2, display.height() / 2 - 20);
    display.setTextSize(1);
    display.drawString("M0 scaffold", display.width() / 2, display.height() / 2 + 8);
    if (selftest_failures == 0) {
        display.setTextColor(display.color565(80, 220, 120));
        display.drawString("cybergear selftest PASS", display.width() / 2,
                           display.height() / 2 + 28);
    } else {
        display.setTextColor(display.color565(240, 90, 70));
        char buf[40];
        snprintf(buf, sizeof(buf), "selftest FAIL (%d)", selftest_failures);
        display.drawString(buf, display.width() / 2, display.height() / 2 + 28);
    }
}

extern "C" void app_main(void)
{
    outputs_safe();
    ESP_LOGI(TAG, "boot: outputs safe (SSR/lock gates low); wake cause %d",
             (int)esp_sleep_get_wakeup_cause());

    int failures = cybergear_selftest_run();

    display_init();
    splash(failures);

    ESP_LOGI(TAG, "M0 scaffold up; selftest failures: %d", failures);
}
