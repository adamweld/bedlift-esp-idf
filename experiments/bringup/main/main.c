/*
 * Bedlift carrier-board bring-up console.
 *
 * Serial REPL (over the Feather's native USB-C) with commands to exercise
 * each board subsystem in isolation: SSR enables, I2C scan, ADXL345,
 * hall inputs, TWAI/CAN, raw GPIO. Type `help` at the prompt.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/twai.h"
#include "pins.h"

static const char *TAG = "bringup";

// ---------------------------------------------------------------------------
// I2C bus (lazy init so the console comes up even with SDA/SCL shorted)
// ---------------------------------------------------------------------------
static i2c_master_bus_handle_t s_i2c_bus;
static uint32_t s_i2c_hz = 100000;   // bring-up default: slow and forgiving

static esp_err_t i2c_bus_get(i2c_master_bus_handle_t *out)
{
    if (!s_i2c_bus) {
        i2c_master_bus_config_t cfg = {
            .i2c_port = -1,
            .sda_io_num = GPIO_I2C_SDA,
            .scl_io_num = GPIO_I2C_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        esp_err_t err = i2c_new_master_bus(&cfg, &s_i2c_bus);
        if (err != ESP_OK) {
            printf("i2c bus init failed: %s\n", esp_err_to_name(err));
            return err;
        }
    }
    *out = s_i2c_bus;
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// en — SSR enable outputs
// ---------------------------------------------------------------------------
static int cmd_en(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "status") != 0) {
        if (argc < 3 || (strcmp(argv[2], "0") && strcmp(argv[2], "1"))) {
            printf("usage: en <motor|lock|status> [0|1]\n");
            return 1;
        }
        int level = argv[2][0] - '0';
        int pin;
        if (!strcmp(argv[1], "motor")) pin = GPIO_MOTOR_POWER;
        else if (!strcmp(argv[1], "lock")) pin = GPIO_LOCK_POWER;
        else { printf("usage: en <motor|lock|status> [0|1]\n"); return 1; }
        gpio_set_level(pin, level);
        printf("%s_EN (IO%d) -> %d\n", pin == GPIO_MOTOR_POWER ? "MOTOR" : "LOCK",
               pin, level);
        return 0;
    }
    printf("MOTOR_EN (IO%d): %d\nLOCK_EN  (IO%d): %d\n",
           GPIO_MOTOR_POWER, gpio_get_level(GPIO_MOTOR_POWER),
           GPIO_LOCK_POWER, gpio_get_level(GPIO_LOCK_POWER));
    return 0;
}

// ---------------------------------------------------------------------------
// i2cscan
// ---------------------------------------------------------------------------
static int cmd_i2cscan(int argc, char **argv)
{
    i2c_master_bus_handle_t bus;
    if (i2c_bus_get(&bus) != ESP_OK) return 1;
    int found = 0;
    printf("scanning 0x03..0x77 @ %lu Hz ...\n", (unsigned long)s_i2c_hz);
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
            const char *hint = "";
            if (addr == ADXL_ADDR_FRONT) hint = "  (ADXL345, SDO high — front)";
            if (addr == ADXL_ADDR_REAR)  hint = "  (ADXL345, SDO low — rear)";
            printf("  0x%02X ACK%s\n", addr, hint);
            found++;
        }
    }
    printf("%d device(s)\n", found);
    return 0;
}

// ---------------------------------------------------------------------------
// adxl — ADXL345 over I2C, raw register access (no external components)
// ---------------------------------------------------------------------------
#define ADXL_REG_DEVID     0x00
#define ADXL_REG_POWER_CTL 0x2D
#define ADXL_REG_DATA      0x32

static esp_err_t adxl_dev(uint8_t addr, i2c_master_dev_handle_t *dev)
{
    i2c_master_bus_handle_t bus;
    esp_err_t err = i2c_bus_get(&bus);
    if (err != ESP_OK) return err;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = s_i2c_hz,
    };
    return i2c_master_bus_add_device(bus, &cfg, dev);
}

static esp_err_t adxl_rd(i2c_master_dev_handle_t dev, uint8_t reg,
                         uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, 100);
}

static esp_err_t adxl_wr(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(dev, b, 2, 100);
}

static int cmd_adxl(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: adxl id|read|sdo ...\n"
               "  adxl id   [front|rear]      check DEVID (expect 0xE5)\n"
               "  adxl read [front|rear] [n]  stream n samples (default 10)\n"
               "  adxl sdo  <0|1>             drive IO%d (addr select)\n",
               GPIO_ACC_SDO);
        return 1;
    }

    if (!strcmp(argv[1], "sdo")) {
        if (argc < 3) { printf("usage: adxl sdo <0|1>\n"); return 1; }
        int level = argv[2][0] - '0';
        gpio_set_level(GPIO_ACC_SDO, level);
        printf("IMU_SDO (IO%d) -> %d  (J7 ADXL now at 0x%02X)\n",
               GPIO_ACC_SDO, level, level ? ADXL_ADDR_FRONT : ADXL_ADDR_REAR);
        return 0;
    }

    uint8_t addr = ADXL_ADDR_FRONT;
    int argi = 2;
    if (argc > argi && !strcmp(argv[argi], "front")) { addr = ADXL_ADDR_FRONT; argi++; }
    else if (argc > argi && !strcmp(argv[argi], "rear")) { addr = ADXL_ADDR_REAR; argi++; }

    i2c_master_dev_handle_t dev;
    if (adxl_dev(addr, &dev) != ESP_OK) { printf("add device failed\n"); return 1; }
    int rc = 1;
    uint8_t id = 0;

    if (adxl_rd(dev, ADXL_REG_DEVID, &id, 1) != ESP_OK) {
        printf("0x%02X: no response (device absent, or SDO strap wrong?)\n", addr);
        goto out;
    }
    if (!strcmp(argv[1], "id")) {
        printf("0x%02X DEVID = 0x%02X %s\n", addr, id,
               id == 0xE5 ? "(ADXL345 OK)" : "(unexpected!)");
        rc = (id == 0xE5) ? 0 : 1;
        goto out;
    }
    if (!strcmp(argv[1], "read")) {
        if (id != 0xE5) { printf("0x%02X DEVID 0x%02X != 0xE5\n", addr, id); goto out; }
        int n = (argc > argi) ? atoi(argv[argi]) : 10;
        if (n < 1) n = 1;
        adxl_wr(dev, ADXL_REG_POWER_CTL, 0x08);           // measure mode
        vTaskDelay(pdMS_TO_TICKS(20));
        for (int i = 0; i < n; i++) {
            uint8_t d[6];
            if (adxl_rd(dev, ADXL_REG_DATA, d, 6) != ESP_OK) {
                printf("read failed\n"); goto out;
            }
            int16_t x = (int16_t)(d[1] << 8 | d[0]);
            int16_t y = (int16_t)(d[3] << 8 | d[2]);
            int16_t z = (int16_t)(d[5] << 8 | d[4]);
            // ±2g full-res: 256 LSB/g
            printf("  x=%+6.3fg y=%+6.3fg z=%+6.3fg\n",
                   x / 256.0, y / 256.0, z / 256.0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        rc = 0;
        goto out;
    }
    printf("unknown subcommand '%s'\n", argv[1]);
out:
    i2c_master_bus_rm_device(dev);
    return rc;
}

// ---------------------------------------------------------------------------
// acc — both accels side by side (orientation calibration)
// ---------------------------------------------------------------------------
static bool acc_read_g(uint8_t addr, float *x, float *y, float *z)
{
    i2c_master_dev_handle_t dev;
    if (adxl_dev(addr, &dev) != ESP_OK) return false;
    adxl_wr(dev, 0x31, 0x08);            // DATA_FORMAT: full res, +/-2g
    adxl_wr(dev, 0x2D, 0x08);            // POWER_CTL: measure
    uint8_t d[6];
    bool ok = adxl_rd(dev, ADXL_REG_DATA, d, 6) == ESP_OK;
    if (ok) {
        *x = (int16_t)(d[1] << 8 | d[0]) / 256.0f;
        *y = (int16_t)(d[3] << 8 | d[2]) / 256.0f;
        *z = (int16_t)(d[5] << 8 | d[4]) / 256.0f;
    }
    i2c_master_bus_rm_device(dev);
    return ok;
}

static int cmd_acc(int argc, char **argv)
{
    int secs = (argc > 1) ? atoi(argv[1]) : 0;
    int64_t end = esp_timer_get_time() + (int64_t)secs * 1000000;
    printf("      FRONT(0x1D)                        REAR(0x53)\n");
    do {
        float fx, fy, fz, rx, ry, rz;
        bool fo = acc_read_g(ADXL_ADDR_FRONT, &fx, &fy, &fz);
        bool ro = acc_read_g(ADXL_ADDR_REAR, &rx, &ry, &rz);
        if (fo && ro) {
            // plumb = angle of the gravity vector from its dominant axis
            // (whichever of x/y/z is vertical for this mounting). Both should
            // be within ~10 deg right now.
            float fmag = sqrtf(fx*fx + fy*fy + fz*fz);
            float rmag = sqrtf(rx*rx + ry*ry + rz*rz);
            float fdom = fmaxf(fabsf(fx), fmaxf(fabsf(fy), fabsf(fz)));
            float rdom = fmaxf(fabsf(rx), fmaxf(fabsf(ry), fabsf(rz)));
            float fpl = acosf(fdom / (fmag > 0.01f ? fmag : 1.0f)) * 57.2958f;
            float rpl = acosf(rdom / (rmag > 0.01f ? rmag : 1.0f)) * 57.2958f;
            float fp = atan2f(fy, fz) * 57.2958f, frl = atan2f(fx, fz) * 57.2958f;
            float rp = atan2f(ry, rz) * 57.2958f, rrl = atan2f(rx, rz) * 57.2958f;
            printf("x%+.2f y%+.2f z%+.2f p%+5.1f r%+5.1f plumb%5.1f%s | "
                   "x%+.2f y%+.2f z%+.2f p%+5.1f r%+5.1f plumb%5.1f%s\n",
                   fx, fy, fz, fp, frl, fpl, fpl < 10.0f ? " OK" : " !!",
                   rx, ry, rz, rp, rrl, rpl, rpl < 10.0f ? " OK" : " !!");
        } else {
            printf("read failed (front=%d rear=%d)\n", fo, ro);
        }
        if (secs > 0) vTaskDelay(pdMS_TO_TICKS(250));
    } while (secs > 0 && esp_timer_get_time() < end);
    return 0;
}

// ---------------------------------------------------------------------------
// i2cfreq / i2creg — bus speed + generic register read
// ---------------------------------------------------------------------------
static int cmd_i2cfreq(int argc, char **argv)
{
    if (argc > 1) s_i2c_hz = (uint32_t)atoi(argv[1]) * 1000;
    printf("i2c speed: %lu Hz\n", (unsigned long)s_i2c_hz);
    return 0;
}

static int cmd_i2creg(int argc, char **argv)
{
    if (argc < 3) { printf("usage: i2creg <addr-hex> <reg-hex> [count]\n"); return 1; }
    uint8_t addr = strtoul(argv[1], NULL, 16);
    uint8_t reg = strtoul(argv[2], NULL, 16);
    int n = (argc > 3) ? atoi(argv[3]) : 1;
    if (n < 1) n = 1;
    if (n > 32) n = 32;
    i2c_master_dev_handle_t dev;
    if (adxl_dev(addr, &dev) != ESP_OK) { printf("add device failed\n"); return 1; }
    uint8_t buf[32];
    esp_err_t err = adxl_rd(dev, reg, buf, n);
    if (err == ESP_OK) {
        printf("0x%02X reg 0x%02X:", addr, reg);
        for (int i = 0; i < n; i++) printf(" %02X", buf[i]);
        printf("\n");
    } else {
        printf("read failed: %s\n", esp_err_to_name(err));
    }
    i2c_master_bus_rm_device(dev);
    return err == ESP_OK ? 0 : 1;
}

// ---------------------------------------------------------------------------
// i2cbb — bit-banged I2C register read (bypasses the hardware controller)
// ---------------------------------------------------------------------------
#define BB_DELAY() esp_rom_delay_us(50)   // ~10 kHz

static void bb_sda(int v) { gpio_set_level(GPIO_I2C_SDA, v); }
static void bb_scl(int v) { gpio_set_level(GPIO_I2C_SCL, v); BB_DELAY(); }
static int  bb_rd_sda(void) { return gpio_get_level(GPIO_I2C_SDA); }

static void bb_start(void) { bb_sda(1); bb_scl(1); bb_sda(0); BB_DELAY(); bb_scl(0); }
static void bb_stop(void)  { bb_sda(0); bb_scl(1); bb_sda(1); BB_DELAY(); }

static int bb_write_byte(uint8_t b)   // returns 1 if ACKed
{
    for (int i = 7; i >= 0; i--) {
        bb_sda((b >> i) & 1);
        bb_scl(1); bb_scl(0);
    }
    bb_sda(1);                        // release for ACK
    bb_scl(1);
    int ack = !bb_rd_sda();
    bb_scl(0);
    return ack;
}

static uint8_t bb_read_byte(int ack)
{
    uint8_t b = 0;
    bb_sda(1);                        // release
    for (int i = 7; i >= 0; i--) {
        bb_scl(1);
        b |= bb_rd_sda() << i;
        bb_scl(0);
    }
    bb_sda(ack ? 0 : 1);
    bb_scl(1); bb_scl(0);
    bb_sda(1);
    return b;
}

static int cmd_i2cbb(int argc, char **argv)
{
    if (argc < 3) { printf("usage: i2cbb <addr-hex> <reg-hex> [count]\n"); return 1; }
    uint8_t addr = strtoul(argv[1], NULL, 16);
    uint8_t reg = strtoul(argv[2], NULL, 16);
    int n = (argc > 3) ? atoi(argv[3]) : 1;
    if (n < 1) n = 1;
    if (n > 16) n = 16;

    // steal the pins from the hw controller
    if (s_i2c_bus) { i2c_del_master_bus(s_i2c_bus); s_i2c_bus = NULL; }
    gpio_config_t od = {
        .pin_bit_mask = (1ULL << GPIO_I2C_SDA) | (1ULL << GPIO_I2C_SCL),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&od);
    bb_sda(1); bb_scl(1);

    printf("idle: SDA=%d SCL=%d\n", gpio_get_level(GPIO_I2C_SDA),
           gpio_get_level(GPIO_I2C_SCL));

    bb_start();
    int a1 = bb_write_byte(addr << 1);          // write, for reg pointer
    int a2 = a1 ? bb_write_byte(reg) : 0;
    bb_start();                                  // repeated start
    int a3 = bb_write_byte(addr << 1 | 1);      // read
    printf("ack: addr-w=%d reg=%d addr-r=%d\n", a1, a2, a3);
    if (a3) {
        printf("0x%02X reg 0x%02X:", addr, reg);
        for (int i = 0; i < n; i++)
            printf(" %02X", bb_read_byte(i < n - 1));
        printf("\n");
    }
    bb_stop();
    return 0;
}

// buswire — pairwise short test between SDA (IO3), SCL (IO4), SDO (IO8).
// Each pin in turn is driven low (open-drain); the others are inputs with
// pull-ups. A follower reading 0 means the two nets are bridged.
static int cmd_buswire(int argc, char **argv)
{
    const int pins[] = { GPIO_I2C_SDA, GPIO_I2C_SCL, GPIO_ACC_SDO };
    const char *names[] = { "SDA/IO3", "SCL/IO4", "SDO/IO8" };
    const int N = 3;

    if (s_i2c_bus) { i2c_del_master_bus(s_i2c_bus); s_i2c_bus = NULL; }
    for (int i = 0; i < N; i++) {
        gpio_config_t od = {
            .pin_bit_mask = (1ULL << pins[i]),
            .mode = GPIO_MODE_INPUT_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        gpio_config(&od);
        gpio_set_level(pins[i], 1);   // released
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    printf("idle:");
    for (int i = 0; i < N; i++)
        printf("  %s=%d", names[i], gpio_get_level(pins[i]));
    printf("\n");

    bool bridged = false;
    for (int i = 0; i < N; i++) {
        gpio_set_level(pins[i], 0);
        vTaskDelay(pdMS_TO_TICKS(2));
        for (int j = 0; j < N; j++) {
            if (j == i) continue;
            int lvl = gpio_get_level(pins[j]);
            if (lvl == 0) {
                printf("  BRIDGE? driving %s low pulls %s low\n", names[i], names[j]);
                bridged = true;
            }
        }
        gpio_set_level(pins[i], 1);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (!bridged) printf("no pairwise shorts among SDA/SCL/SDO\n");

    // restore SDO strap high (push-pull)
    gpio_config_t sdo = {
        .pin_bit_mask = (1ULL << GPIO_ACC_SDO),
        .mode = GPIO_MODE_INPUT_OUTPUT,
    };
    gpio_config(&sdo);
    gpio_set_level(GPIO_ACC_SDO, 1);
    return 0;
}

static int cmd_i2creset(int argc, char **argv)
{
    if (s_i2c_bus) { i2c_del_master_bus(s_i2c_bus); s_i2c_bus = NULL; }
    printf("i2c bus handle dropped; next command re-creates it\n");
    return 0;
}

// ---------------------------------------------------------------------------
// hall — read / watch the two hall inputs
// ---------------------------------------------------------------------------
static int cmd_hall(int argc, char **argv)
{
    int secs = (argc > 1) ? atoi(argv[1]) : 0;
    printf("HALL_1 (IO%d): %d   HALL_2 (IO%d): %d\n",
           GPIO_HALL_1, gpio_get_level(GPIO_HALL_1),
           GPIO_HALL_2, gpio_get_level(GPIO_HALL_2));
    if (secs <= 0) return 0;

    printf("watching %ds (move a magnet past each sensor)...\n", secs);
    int l1 = gpio_get_level(GPIO_HALL_1), l2 = gpio_get_level(GPIO_HALL_2);
    int e1 = 0, e2 = 0;
    int64_t end = esp_timer_get_time() + (int64_t)secs * 1000000;
    while (esp_timer_get_time() < end) {
        int n1 = gpio_get_level(GPIO_HALL_1), n2 = gpio_get_level(GPIO_HALL_2);
        if (n1 != l1) { e1++; l1 = n1; printf("  HALL_1 -> %d\n", n1); }
        if (n2 != l2) { e2++; l2 = n2; printf("  HALL_2 -> %d\n", n2); }
        vTaskDelay(1);   // >= 1 tick, or this busy-spins and trips the task WDT
    }
    printf("edges: HALL_1=%d HALL_2=%d\n", e1, e2);
    return 0;
}

// ---------------------------------------------------------------------------
// can — TWAI bring-up
// ---------------------------------------------------------------------------
static bool s_can_up = false;

static int cmd_can(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: can up [kbps] [--swap] | down | status | send <id> [b0..b7] | dump [ms]\n");
        return 1;
    }

    if (!strcmp(argv[1], "up")) {
        if (s_can_up) { printf("already up — `can down` first\n"); return 1; }
        int kbps = 1000;
        int tx = GPIO_CAN_TX, rx = GPIO_CAN_RX;
        for (int i = 2; i < argc; i++) {
            if (!strcmp(argv[i], "--swap")) { tx = GPIO_CAN_RX; rx = GPIO_CAN_TX; }
            else kbps = atoi(argv[i]);
        }
        twai_timing_config_t t;
        switch (kbps) {
            case 1000: t = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS(); break;
            case 500:  t = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS(); break;
            case 250:  t = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS(); break;
            case 125:  t = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS(); break;
            default: printf("supported: 125/250/500/1000 kbps\n"); return 1;
        }
        twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
        g.alerts_enabled = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR |
                           TWAI_ALERT_BUS_OFF | TWAI_ALERT_TX_FAILED;
        twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        esp_err_t err = twai_driver_install(&g, &t, &f);
        if (err == ESP_OK) err = twai_start();
        if (err != ESP_OK) {
            printf("twai up failed: %s\n", esp_err_to_name(err));
            twai_driver_uninstall();
            return 1;
        }
        s_can_up = true;
        printf("twai up: %d kbps, TX=IO%d RX=IO%d%s\n", kbps, tx, rx,
               tx != GPIO_CAN_TX ? "  (SWAPPED)" : "");
        return 0;
    }

    if (!strcmp(argv[1], "down")) {
        if (s_can_up) { twai_stop(); twai_driver_uninstall(); s_can_up = false; }
        printf("twai down\n");
        return 0;
    }

    if (!s_can_up) { printf("`can up` first\n"); return 1; }

    if (!strcmp(argv[1], "status")) {
        twai_status_info_t st;
        twai_get_status_info(&st);
        printf("state=%d tx_q=%lu rx_q=%lu tx_err=%lu rx_err=%lu "
               "tx_failed=%lu rx_missed=%lu bus_err=%lu arb_lost=%lu\n",
               st.state, (unsigned long)st.msgs_to_tx, (unsigned long)st.msgs_to_rx,
               (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter,
               (unsigned long)st.tx_failed_count, (unsigned long)st.rx_missed_count,
               (unsigned long)st.bus_error_count, (unsigned long)st.arb_lost_count);
        uint32_t alerts;
        if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts) {
            printf("alerts: 0x%08lX%s%s%s%s\n", (unsigned long)alerts,
                   alerts & TWAI_ALERT_BUS_OFF ? " BUS_OFF" : "",
                   alerts & TWAI_ALERT_ERR_PASS ? " ERR_PASS" : "",
                   alerts & TWAI_ALERT_BUS_ERROR ? " BUS_ERROR" : "",
                   alerts & TWAI_ALERT_TX_FAILED ? " TX_FAILED" : "");
        }
        return 0;
    }

    if (!strcmp(argv[1], "send")) {
        if (argc < 3) { printf("usage: can send <id-hex> [b0..b7 hex]\n"); return 1; }
        twai_message_t m = { 0 };
        m.identifier = strtoul(argv[2], NULL, 16);
        m.extd = (m.identifier > 0x7FF);
        m.data_length_code = argc - 3 > 8 ? 8 : argc - 3;
        for (int i = 0; i < m.data_length_code; i++)
            m.data[i] = strtoul(argv[3 + i], NULL, 16);
        esp_err_t err = twai_transmit(&m, pdMS_TO_TICKS(500));
        printf("tx id=0x%lX dlc=%d: %s\n", (unsigned long)m.identifier,
               m.data_length_code, esp_err_to_name(err));
        return err == ESP_OK ? 0 : 1;
    }

    if (!strcmp(argv[1], "dump")) {
        int ms = (argc > 2) ? atoi(argv[2]) : 2000;
        int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
        int n = 0;
        printf("listening %d ms...\n", ms);
        while (esp_timer_get_time() < end) {
            twai_message_t m;
            if (twai_receive(&m, pdMS_TO_TICKS(50)) == ESP_OK) {
                printf("  rx %s id=0x%08lX dlc=%d [", m.extd ? "ext" : "std",
                       (unsigned long)m.identifier, m.data_length_code);
                for (int i = 0; i < m.data_length_code; i++)
                    printf("%s%02X", i ? " " : "", m.data[i]);
                printf("]\n");
                n++;
            }
        }
        printf("%d frame(s)\n", n);
        return 0;
    }

    printf("unknown subcommand '%s'\n", argv[1]);
    return 1;
}

// ---------------------------------------------------------------------------
// cg — minimal CyberGear liveness (frame layout per bedlift lib/cybergear)
// ---------------------------------------------------------------------------
#include "cybergear.h"

#define CG_MASTER_ID 0x00
#define CG_DEFAULT_LIMIT_A 4.0f   // gentle default; motor max is 27A

// Motor objects from the PRODUCTION fork, addressed by raw CAN id (1..4).
static cybergear_motor_t s_cg[8];
static bool s_cg_init[8];

static cybergear_motor_t *cg_motor(uint8_t id)
{
    if (id < 1 || id > 7) return NULL;
    if (!s_cg_init[id]) {
        cybergear_init(&s_cg[id], CG_MASTER_ID, id, pdMS_TO_TICKS(50));
        s_cg_init[id] = true;
    }
    return &s_cg[id];
}

static esp_err_t cg_tw_send(const twai_message_t *msg, TickType_t ticks, void *ctx)
{
    (void)ctx;
    twai_message_t m = *msg;
    return twai_transmit(&m, ticks);
}

// Drain RX for up to `ms`, offering every frame to every known motor object.
static int cg_drain(int ms)
{
    int n = 0;
    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    twai_message_t m;
    while (esp_timer_get_time() < end) {
        if (twai_receive(&m, pdMS_TO_TICKS(10)) != ESP_OK) continue;
        n++;
        for (int id = 1; id < 8; id++)
            if (s_cg_init[id] &&
                cybergear_process_message(&s_cg[id], &m,
                                          esp_timer_get_time()) != ESP_ERR_NOT_FOUND)
                break;
    }
    return n;
}

static bool s_tap_on = false;
static void cg_frame_tap(int dir, const twai_message_t *m);

// Raw SDO read (type 0x11) of ANY index — bypasses the driver's known-index
// parser so we can probe arbitrary parameters. Returns true on a matching
// reply, filling val[4] (the value bytes) and the echoed index.
static bool cg_sdo_read_raw(uint8_t id, uint16_t index, uint8_t val[4],
                            uint16_t *echo_index)
{
    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.extd = 1;
    m.data_length_code = 8;
    m.identifier = (0x11u << 24) | ((uint32_t)CG_MASTER_ID << 8) | id;
    m.data[0] = index & 0xFF;
    m.data[1] = index >> 8;
    if (s_tap_on) cg_frame_tap(0, &m);
    twai_transmit(&m, pdMS_TO_TICKS(50));
    int64_t end = esp_timer_get_time() + 300000;
    while (esp_timer_get_time() < end) {
        if (twai_receive(&m, pdMS_TO_TICKS(50)) != ESP_OK) continue;
        uint8_t type = (m.identifier >> 24) & 0x1F;
        uint8_t src = (m.identifier >> 8) & 0xFF;
        if (type == 0x11 && src == id) {
            if (s_tap_on) cg_frame_tap(1, &m);
            if (echo_index) *echo_index = m.data[1] << 8 | m.data[0];
            memcpy(val, &m.data[4], 4);
            return true;
        }
    }
    return false;
}

// Raw SDO write (type 0x12) of a 16-bit value — for echoPara/echoFreHz config.
static void cg_sdo_write_u16(uint8_t id, uint16_t index, uint16_t val)
{
    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.extd = 1;
    m.data_length_code = 8;
    m.identifier = (0x12u << 24) | ((uint32_t)CG_MASTER_ID << 8) | id;
    m.data[0] = index & 0xFF;
    m.data[1] = index >> 8;
    m.data[4] = val & 0xFF;
    m.data[5] = val >> 8;
    if (s_tap_on) cg_frame_tap(0, &m);
    twai_transmit(&m, pdMS_TO_TICKS(50));
}

static char cg_printable(uint8_t c) { return (c >= 0x20 && c < 0x7F) ? c : '.'; }

// Print one raw SDO read as bytes + all plausible interpretations.
static void cg_print_read(uint16_t idx, const uint8_t v[4], uint16_t echo)
{
    uint32_t u; memcpy(&u, v, 4);
    int32_t s; memcpy(&s, v, 4);
    float f; memcpy(&f, v, 4);
    printf("  0x%04X (echo 0x%04X): %02X %02X %02X %02X  u32=%lu i32=%ld "
           "f32=%g  u16lo=%u  ascii='%c%c%c%c'\n",
           idx, echo, v[0], v[1], v[2], v[3],
           (unsigned long)u, (long)s, f, (unsigned)(v[0] | v[1] << 8),
           cg_printable(v[0]), cg_printable(v[1]), cg_printable(v[2]), cg_printable(v[3]));
}

// Frame tap: log every driver TX/RX frame the driver actually handles.
static void cg_frame_tap(int dir, const twai_message_t *m)
{
    printf("  %s id=0x%08lX [", dir ? "RX" : "TX", (unsigned long)m->identifier);
    for (int i = 0; i < m->data_length_code; i++)
        printf("%s%02X", i ? " " : "", m->data[i]);
    printf("]\n");
}

static int cmd_cg(int argc, char **argv)
{
    if (argc >= 2 && !strcmp(argv[1], "tap")) {
        s_tap_on = !(argc >= 3 && !strcmp(argv[2], "off"));
        cybergear_set_frame_tap(s_tap_on ? cg_frame_tap : NULL);
        printf("frame tap %s\n", s_tap_on ? "ON" : "off");
        return 0;
    }
    if (argc < 3) {
        printf("usage: cg <sub> <id> ... (production fork driver)\n"
               "  cg tap [off]           log every driver TX/RX frame\n"
               "  cg ping <id>            type-0 liveness + MCU uid\n"
               "  cg init <id> [limitA]   speed mode + current limit + enable\n"
               "  cg vel  <id> <rad/s>    set speed (init first)\n"
               "  cg stop <id>            reset/disable (zeroes speed)\n"
               "  cg clear <id>           clear-fault (type 4, byte0=1)  [A4]\n"
               "  cg vbus <id> [n]        param-read VBUS n times (0x701C) [A2]\n"
               "  cg pos  <id>            mech_pos/rotation/mech_vel/iqf  [B2]\n"
               "  cg stat <id>            last echo status + fault bits\n"
               "  cg rd <id> <hexidx> [n] raw SDO read of ANY index (probe)\n"
               "  cg ver <id>             read candidate firmware-version strings\n"
               "  cg scan <id> <a> <b>    raw SDO read every index a..b (hex)\n"
               "  cg scope <id> [hz]      stream mechPos/rotation/mechVel/VBUS (echoPara)\n"
               "  cg discover <id> [q]    PARA_STR_INFO type-19 probe (dumps frames)\n");
        return 1;
    }
    if (!s_can_up) { printf("`can up` first\n"); return 1; }
    uint8_t id = strtoul(argv[2], NULL, 0);
    cybergear_motor_t *m = cg_motor(id);
    if (!m) { printf("bad id\n"); return 1; }

    if (!strcmp(argv[1], "rd")) {
        if (argc < 4) { printf("usage: cg rd <id> <hexidx> [n]\n"); return 1; }
        uint16_t idx = strtoul(argv[3], NULL, 16);
        int n = (argc > 4) ? atoi(argv[4]) : 1;
        for (int i = 0; i < n; i++, idx++) {
            uint8_t v[4]; uint16_t echo;
            if (cg_sdo_read_raw(id, idx, v, &echo)) cg_print_read(idx, v, echo);
            else printf("  0x%04X: no reply\n", idx);
        }
        return 0;
    }
    if (!strcmp(argv[1], "scan")) {
        if (argc < 5) { printf("usage: cg scan <id> <a-hex> <b-hex>\n"); return 1; }
        uint16_t a = strtoul(argv[3], NULL, 16), b = strtoul(argv[4], NULL, 16);
        for (uint16_t idx = a; idx <= b; idx++) {
            uint8_t v[4]; uint16_t echo;
            if (cg_sdo_read_raw(id, idx, v, &echo)) cg_print_read(idx, v, echo);
        }
        printf("scan 0x%04X..0x%04X done\n", a, b);
        return 0;
    }
    if (!strcmp(argv[1], "ver")) {
        // v1.2.1.5 table puts version strings at 0x704A..0x7053; SDK/older
        // schemes differ. Read a spread and print ASCII — readable chars
        // reveal which scheme's version index is live on THIS motor.
        uint16_t cand[] = { 0x704A, 0x704B, 0x704C, 0x704D, 0x704E, 0x704F,
                            0x7050, 0x7051, 0x7052, 0x7053, 0x7000, 0x2000 };
        for (unsigned i = 0; i < sizeof(cand) / sizeof(cand[0]); i++) {
            uint8_t v[4]; uint16_t echo;
            if (cg_sdo_read_raw(id, cand[i], v, &echo)) cg_print_read(cand[i], v, echo);
        }
        return 0;
    }
    if (!strcmp(argv[1], "scope")) {
        // Oscilloscope streaming: configure echoPara1-4 to table indices for
        // mechPos/rotation/mechVel/VBUS, set echoFreHz, decode the repurposed
        // type-0x02 frames. Table indices per RE doc (v1.2.1.5) — if they
        // stream plausible, changing values, this is our mechPos/rotation path.
        int freq = (argc > 3) ? atoi(argv[3]) : 100;
        // table index = (SDO addr - 0x7000): mechPos 0x7030=48, rotation
        // 0x702E=46, mechVel 0x7031=49, VBUS 0x7026=38
        const uint16_t chans[4] = { 48, 46, 49, 38 };
        const char *names[4] = { "mechPos", "rotation", "mechVel", "VBUS" };
        for (int i = 0; i < 4; i++) {
            cg_sdo_write_u16(id, 0x7000 + i, chans[i]);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        cg_sdo_write_u16(id, 0x7004, freq);   // echoFreHz -> start streaming
        printf("scope on: ch=[mechPos(48) rotation(46) mechVel(49) VBUS(38)] @%dHz\n",
               freq);
        printf("        %10s %10s %10s %10s\n", names[0], names[1], names[2], names[3]);

        int64_t end = esp_timer_get_time() + 2500000;
        int n = 0;
        twai_message_t rx;
        while (esp_timer_get_time() < end) {
            if (twai_receive(&rx, pdMS_TO_TICKS(50)) != ESP_OK) continue;
            uint8_t type = (rx.identifier >> 24) & 0x1F;
            uint8_t src = (rx.identifier >> 8) & 0xFF;
            if (type != CG_TYPE_FEEDBACK || src != id) continue;
            uint16_t c0 = rx.data[0] | rx.data[1] << 8;
            uint16_t c1 = rx.data[2] | rx.data[3] << 8;
            uint16_t c2 = rx.data[4] | rx.data[5] << 8;
            uint16_t c3 = rx.data[6] | rx.data[7] << 8;
            if ((n++ % 10) == 0)   // print every 10th frame
                printf("  raw   %10u %10u %10u %10u  [%02X%02X %02X%02X %02X%02X %02X%02X]\n",
                       c0, c1, c2, c3, rx.data[0], rx.data[1], rx.data[2], rx.data[3],
                       rx.data[4], rx.data[5], rx.data[6], rx.data[7]);
        }
        cg_sdo_write_u16(id, 0x7004, 0);      // echoFreHz=0 -> stop
        printf("scope off (%d frames). Turn the shaft during the next run to\n"
               "see if mechPos/rotation change.\n", n);
        return 0;
    }
    if (!strcmp(argv[1], "discover")) {
        // PARA_STR_INFO (type 19). Exact request format is undocumented, so
        // send a best-effort request and dump every frame that comes back.
        uint8_t q = (argc > 3) ? strtoul(argv[3], NULL, 0) : 0;
        twai_message_t tx;
        memset(&tx, 0, sizeof(tx));
        tx.extd = 1; tx.data_length_code = 8;
        tx.identifier = (0x13u << 24) | ((uint32_t)q << 16) |
                        ((uint32_t)CG_MASTER_ID << 8) | id;
        printf("TX type-19 id=0x%08lX q=%u; listening 800ms...\n",
               (unsigned long)tx.identifier, q);
        twai_transmit(&tx, pdMS_TO_TICKS(50));
        int64_t end = esp_timer_get_time() + 800000;
        int nframes = 0;
        twai_message_t rx;
        while (esp_timer_get_time() < end) {
            if (twai_receive(&rx, pdMS_TO_TICKS(50)) != ESP_OK) continue;
            nframes++;
            printf("  RX id=0x%08lX [", (unsigned long)rx.identifier);
            for (int i = 0; i < rx.data_length_code; i++)
                printf("%s%02X", i ? " " : "", rx.data[i]);
            printf("]  '");
            for (int i = 0; i < rx.data_length_code; i++)
                printf("%c", cg_printable(rx.data[i]));
            printf("'\n");
        }
        printf("%d frame(s)\n", nframes);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        cybergear_set_speed(m, 0.0f);
        if (cybergear_stop(m) != ESP_OK) return 1;
        cg_drain(50);
        printf("reset/stop sent to motor %u\n", id);
        return 0;
    }
    if (!strcmp(argv[1], "clear")) {
        if (cybergear_clear_fault(m) != ESP_OK) return 1;
        cg_drain(100);
        printf("clear-fault sent to motor %u; faults now 0x%08lX\n",
               id, (unsigned long)cybergear_get_faults(m));
        return 0;
    }
    if (!strcmp(argv[1], "init")) {
        float lim = (argc > 3) ? atof(argv[3]) : CG_DEFAULT_LIMIT_A;
        if (lim > 10.0f) { printf("limit capped at 10A for bring-up\n"); lim = 10.0f; }
        cybergear_stop(m);
        vTaskDelay(pdMS_TO_TICKS(10));
        cybergear_set_mode(m, CYBERGEAR_MODE_SPEED);
        vTaskDelay(pdMS_TO_TICKS(10));
        cybergear_set_limit_current(m, lim);
        vTaskDelay(pdMS_TO_TICKS(10));
        cybergear_set_speed(m, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(10));
        if (cybergear_enable(m) != ESP_OK) return 1;
        cg_drain(100);
        printf("motor %u: speed mode, %.1fA limit, enabled; state=%d\n",
               id, lim, m->status.state);
        return 0;
    }
    if (!strcmp(argv[1], "vel")) {
        if (argc < 4) { printf("usage: cg vel <id> <rad/s>\n"); return 1; }
        float v = atof(argv[3]);
        if (v > 10.0f) v = 10.0f;
        if (v < -10.0f) v = -10.0f;
        if (cybergear_set_speed(m, v) != ESP_OK) return 1;
        cg_drain(50);
        printf("motor %u speed ref -> %.2f (echo: v=%+.2f T=%.1fC)\n",
               id, v, m->status.speed, m->status.temperature);
        return 0;
    }
    if (!strcmp(argv[1], "ping")) {
        int64_t before = m->ping_rx_us;
        if (cybergear_ping(m) != ESP_OK) return 1;
        cg_drain(300);
        if (m->ping_rx_us != before) {
            printf("motor %u alive, mcu uid %016llX\n", id,
                   (unsigned long long)m->mcu_uid);
            return 0;
        }
        printf("no reply from motor %u (check 24V, MOTOR_EN, termination, id)\n", id);
        return 1;
    }
    if (!strcmp(argv[1], "vbus")) {
        int n = (argc > 3) ? atoi(argv[3]) : 1;
        if (n < 1) n = 1;
        if (n > 1000) n = 1000;
        int ok = 0;
        float vmin = 1e9f, vmax = -1e9f;
        for (int i = 0; i < n; i++) {
            if (cybergear_get_param(m, CG_ADDR_VBUS) != ESP_OK) continue;
            cg_drain(60);
            if (m->params.updated && m->params.last_index == CG_ADDR_VBUS) {
                ok++;
                if (m->params.vbus < vmin) vmin = m->params.vbus;
                if (m->params.vbus > vmax) vmax = m->params.vbus;
            }
        }
        if (ok)
            printf("VBUS motor %u: %.2fV (%d/%d replies, min %.2f max %.2f)\n",
                   id, m->params.vbus, ok, n, vmin, vmax);
        else
            printf("VBUS motor %u: NO REPLY (0/%d) — param-read path failed\n",
                   id, n);
        return ok ? 0 : 1;
    }
    if (!strcmp(argv[1], "pos")) {
        static const struct { uint16_t addr; const char *name; } regs[] = {
            { CG_ADDR_MECH_POS, "mech_pos" }, { CG_ADDR_ROTATION, "rotation" },
            { CG_ADDR_MECH_VEL, "mech_vel" }, { CG_ADDR_IQF, "iqf" },
        };
        for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
            if (cybergear_get_param(m, regs[i].addr) != ESP_OK) return 1;
            cg_drain(60);
            if (!(m->params.updated && m->params.last_index == regs[i].addr)) {
                printf("  %s: no reply\n", regs[i].name);
                continue;
            }
            if (regs[i].addr == CG_ADDR_ROTATION)
                printf("  %-8s = %d turns\n", regs[i].name, m->params.rotation);
            else
                printf("  %-8s = %+.4f\n", regs[i].name,
                       regs[i].addr == CG_ADDR_MECH_POS ? m->params.mech_pos :
                       regs[i].addr == CG_ADDR_MECH_VEL ? m->params.mech_vel :
                                                          m->params.iqf);
        }
        return 0;
    }
    if (!strcmp(argv[1], "stat")) {
        cg_drain(50);
        printf("motor %u: state=%d pos=%+.3f v=%+.3f T=%+.2fNm temp=%.1fC "
               "faults=0x%08lX (age %lld ms)\n",
               id, m->status.state, m->status.position, m->status.speed,
               m->status.torque, m->status.temperature,
               (unsigned long)cybergear_get_faults(m),
               (long long)((esp_timer_get_time() - m->status.last_rx_us) / 1000));
        return 0;
    }
    printf("unknown subcommand '%s'\n", argv[1]);
    return 1;
}

// ---------------------------------------------------------------------------
// gpio / btn
// ---------------------------------------------------------------------------
static int cmd_gpio(int argc, char **argv)
{
    if (argc < 2) { printf("usage: gpio <pin> [0|1]\n"); return 1; }
    int pin = atoi(argv[1]);
    if (pin < 0 || pin > 48) { printf("bad pin\n"); return 1; }
    if (argc >= 3) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, argv[2][0] - '0');
        printf("IO%d -> %c\n", pin, argv[2][0]);
    } else {
        printf("IO%d = %d\n", pin, gpio_get_level(pin));
    }
    return 0;
}

static int cmd_btn(int argc, char **argv)
{
    int d0 = gpio_get_level(GPIO_BTN_D0);
    int d1 = gpio_get_level(GPIO_BTN_D1);
    int d2 = gpio_get_level(GPIO_BTN_D2);
    printf("D0=%d (%s)  D1=%d (%s)  D2=%d (%s)\n",
           d0, d0 == 0 ? "PRESSED" : "idle",
           d1, d1 == 1 ? "PRESSED" : "idle",
           d2, d2 == 1 ? "PRESSED" : "idle");
    return 0;
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
static void gpio_init_safe(void)
{
    // Enables LOW before anything else: no load switches on at boot.
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << GPIO_MOTOR_POWER) | (1ULL << GPIO_LOCK_POWER),
        .mode = GPIO_MODE_INPUT_OUTPUT,   // input too, so gpio_get_level works
    };
    gpio_config(&out);
    gpio_set_level(GPIO_MOTOR_POWER, 0);
    gpio_set_level(GPIO_LOCK_POWER, 0);

    // I2C rail/pull-up enable (Feather gates STEMMA/I2C power behind IO7)
    gpio_config_t i2cpwr = {
        .pin_bit_mask = (1ULL << GPIO_I2C_POWER),
        .mode = GPIO_MODE_INPUT_OUTPUT,
    };
    gpio_config(&i2cpwr);
    gpio_set_level(GPIO_I2C_POWER, 1);

    // ADXL address strap high -> J7 device at 0x1D (matches display firmware)
    gpio_config_t sdo = {
        .pin_bit_mask = (1ULL << GPIO_ACC_SDO),
        .mode = GPIO_MODE_INPUT_OUTPUT,
    };
    gpio_config(&sdo);
    gpio_set_level(GPIO_ACC_SDO, 1);

    // Hall inputs with weak pull-ups (open-collector sensors)
    gpio_config_t hall = {
        .pin_bit_mask = (1ULL << GPIO_HALL_1) | (1ULL << GPIO_HALL_2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&hall);

    // On-module buttons
    gpio_config_t b0 = {
        .pin_bit_mask = (1ULL << GPIO_BTN_D0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&b0);
    gpio_config_t b12 = {
        .pin_bit_mask = (1ULL << GPIO_BTN_D1) | (1ULL << GPIO_BTN_D2),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&b12);
}

static void register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        { .command = "en",      .help = "SSR enables: en <motor|lock|status> [0|1]", .func = &cmd_en },
        { .command = "i2cscan", .help = "Scan the I2C bus (SDA=IO3 SCL=IO4)",        .func = &cmd_i2cscan },
        { .command = "adxl",    .help = "ADXL345: adxl id|read|sdo ...",             .func = &cmd_adxl },
        { .command = "acc",     .help = "both accels side by side: acc [watch-secs]", .func = &cmd_acc },
        { .command = "i2cfreq", .help = "Get/set bus speed: i2cfreq [khz]",          .func = &cmd_i2cfreq },
        { .command = "i2creg",  .help = "Read regs: i2creg <addr> <reg> [n]",        .func = &cmd_i2creg },
        { .command = "i2cbb",   .help = "Bit-banged read: i2cbb <addr> <reg> [n]",   .func = &cmd_i2cbb },
        { .command = "i2creset",.help = "Drop and re-create the I2C bus handle",     .func = &cmd_i2creset },
        { .command = "buswire", .help = "Pairwise short test: SDA/SCL/SDO",          .func = &cmd_buswire },
        { .command = "hall",    .help = "Hall inputs: hall [watch-seconds]",         .func = &cmd_hall },
        { .command = "can",     .help = "TWAI: can up|down|status|send|dump",        .func = &cmd_can },
        { .command = "cg",      .help = "CyberGear: cg ping|stop <id>",              .func = &cmd_cg },
        { .command = "gpio",    .help = "Raw pin: gpio <pin> [0|1]",                 .func = &cmd_gpio },
        { .command = "btn",     .help = "Read on-module buttons D0/D1/D2",           .func = &cmd_btn },
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++)
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
}

void app_main(void)
{
    gpio_init_safe();
    cybergear_set_transport(cg_tw_send, NULL);
    esp_log_level_set("i2c.master", ESP_LOG_NONE);  // probe misses are expected during scans
    ESP_LOGI(TAG, "bedlift bring-up console — MOTOR_EN/LOCK_EN forced low");

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "bedlift>";
    repl_cfg.max_cmdline_length = 128;

    esp_console_register_help_command();
    register_commands();

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t hw =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw, &repl_cfg, &repl));
#else
    esp_console_dev_uart_config_t hw = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw, &repl_cfg, &repl));
#endif
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
