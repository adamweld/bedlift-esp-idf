/*
 * Bedlift carrier-board bring-up console.
 *
 * Serial REPL (over the Feather's native USB-C) with commands to exercise
 * each board subsystem in isolation: SSR enables, I2C scan, ADXL345,
 * hall inputs, TWAI/CAN, raw GPIO. Type `help` at the prompt.
 */

#include <stdio.h>
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
#define CG_MASTER_ID 0x00
#define CG_CMD_GET_ID  0x00
#define CG_CMD_ENABLE  0x03
#define CG_CMD_RESET   0x04
#define CG_CMD_RAM_WR  0x12
#define CG_ADDR_RUN_MODE      0x7005
#define CG_ADDR_SPEED_REF     0x700A
#define CG_ADDR_LIMIT_CURRENT 0x7018
#define CG_RUN_MODE_SPEED 2
#define CG_DEFAULT_LIMIT_A 4.0f   // gentle default; motor max is 27A

static int cg_send_data(uint8_t cmd, uint8_t motor_id, const uint8_t *data)
{
    twai_message_t m = { 0 };
    m.extd = 1;
    m.identifier = (uint32_t)cmd << 24 | (uint32_t)CG_MASTER_ID << 8 | motor_id;
    m.data_length_code = 8;
    if (data) memcpy(m.data, data, 8);
    esp_err_t err = twai_transmit(&m, pdMS_TO_TICKS(500));
    if (err != ESP_OK) { printf("tx failed: %s\n", esp_err_to_name(err)); return -1; }
    return 0;
}

static int cg_send(uint8_t cmd, uint8_t motor_id)
{
    return cg_send_data(cmd, motor_id, NULL);
}

static int cg_ram_write_f32(uint8_t motor_id, uint16_t addr, float value)
{
    uint8_t d[8] = { addr & 0xFF, addr >> 8, 0, 0 };
    memcpy(&d[4], &value, 4);
    return cg_send_data(CG_CMD_RAM_WR, motor_id, d);
}

static int cg_ram_write_u8(uint8_t motor_id, uint16_t addr, uint8_t value)
{
    uint8_t d[8] = { addr & 0xFF, addr >> 8, 0, 0, value };
    return cg_send_data(CG_CMD_RAM_WR, motor_id, d);
}

static int cmd_cg(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: cg ping|init|vel|stop ...\n"
               "  cg ping <id>            liveness check\n"
               "  cg init <id> [limitA]   speed mode + current limit + enable\n"
               "  cg vel  <id> <rad/s>    set speed (init first)\n"
               "  cg stop <id>            reset/disable (also zeroes speed)\n");
        return 1;
    }
    if (!s_can_up) { printf("`can up` first\n"); return 1; }
    uint8_t id = strtoul(argv[2], NULL, 0);

    if (!strcmp(argv[1], "stop")) {
        cg_ram_write_f32(id, CG_ADDR_SPEED_REF, 0.0f);
        if (cg_send(CG_CMD_RESET, id)) return 1;
        printf("reset/stop sent to motor %u\n", id);
        return 0;
    }
    if (!strcmp(argv[1], "init")) {
        float lim = (argc > 3) ? atof(argv[3]) : CG_DEFAULT_LIMIT_A;
        if (lim > 10.0f) { printf("limit capped at 10A for bring-up\n"); lim = 10.0f; }
        if (cg_ram_write_u8(id, CG_ADDR_RUN_MODE, CG_RUN_MODE_SPEED)) return 1;
        vTaskDelay(pdMS_TO_TICKS(10));
        if (cg_ram_write_f32(id, CG_ADDR_LIMIT_CURRENT, lim)) return 1;
        vTaskDelay(pdMS_TO_TICKS(10));
        if (cg_ram_write_f32(id, CG_ADDR_SPEED_REF, 0.0f)) return 1;
        vTaskDelay(pdMS_TO_TICKS(10));
        if (cg_send(CG_CMD_ENABLE, id)) return 1;
        printf("motor %u: speed mode, %.1fA limit, enabled at 0 rad/s\n", id, lim);
        return 0;
    }
    if (!strcmp(argv[1], "vel")) {
        if (argc < 4) { printf("usage: cg vel <id> <rad/s>\n"); return 1; }
        float v = atof(argv[3]);
        if (v > 10.0f) v = 10.0f;
        if (v < -10.0f) v = -10.0f;   // old fw used 10 rad/s max
        if (cg_ram_write_f32(id, CG_ADDR_SPEED_REF, v)) return 1;
        printf("motor %u speed ref -> %.2f rad/s\n", id, v);
        return 0;
    }
    if (!strcmp(argv[1], "ping")) {
        if (cg_send(CG_CMD_GET_ID, id)) return 1;
        twai_message_t m;
        int64_t end = esp_timer_get_time() + 500000;
        while (esp_timer_get_time() < end) {
            if (twai_receive(&m, pdMS_TO_TICKS(50)) != ESP_OK) continue;
            printf("motor %u alive: reply id=0x%08lX [", id,
                   (unsigned long)m.identifier);
            for (int i = 0; i < m.data_length_code; i++)
                printf("%s%02X", i ? " " : "", m.data[i]);
            printf("]\n");
            return 0;
        }
        printf("no reply from motor %u (check 24V, MOTOR_EN, termination, id)\n", id);
        return 1;
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
