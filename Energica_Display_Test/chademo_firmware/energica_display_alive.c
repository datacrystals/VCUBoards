/**
 * @file  energica_display_alive.c
 * @brief Keep the Energica display awake with periodic "vehicle" frames
 * @details
 *   Sends 0x100 / 0x101 / 0x102 periodically (100 ms cycle).
 *   Change values on-the-fly over USB serial — no reflashing needed!
 *
 *   SERIAL COMMANDS (type then hit Enter):
 *     v=350        set target voltage (0x102)
 *     a=50         set target current  (0x102)
 *     soc=75       set SOC percent     (0x102)
 *     status=0x09  set status byte     (0x102)
 *     fault=0x00   set fault byte      (0x102)
 *     maxv=420     set max voltage     (0x100)
 *     cap=100      set capacity %%      (0x100)
 *     time10=255   set time10s         (0x101)
 *     time60=255   set time60s         (0x101)
 *     est60=255    set est time60s     (0x101)
 *     totwh=1200   set total cap (0.1kWh) (0x101)
 *     proto=2      set protocol ver    (0x102)
 *     help         show this list
 *
 *   BUILD:  cd build && cmake .. && make energica_display_alive -j4
 */

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"

#include "chademo_config.h"
#include "chademo_hal.h"
#include "chademo_can.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define LED_PIN     25
#define CH          HAL_CAN_CH1
#define BITRATE     500

/* -------------------------------------------------------------------------- */
/*  LIVE-TWEAKABLE GLOBALS (updated via serial)                               */
/* -------------------------------------------------------------------------- */
static uint16_t bat_max_v      = 420;
static uint8_t  bat_cap_pct    = 100;
static uint8_t  charge_time_10 = 0xFF;
static uint8_t  charge_time_60 = 0xFF;
static uint8_t  est_time_60    = 0xFF;
static uint16_t total_cap_wh   = 1200;
static uint8_t  protocol       = 2;
static uint16_t target_v       = 350;
static uint8_t  target_a       = 0;
static uint8_t  fault_flags    = 0;
static uint8_t  status_flags   = 0x08;
static uint8_t  soc            = 75;

/* -------------------------------------------------------------------------- */

static void print_frame(const char *dir, uint32_t t, const chademo_can_frame_t *f)
{
    printf("[%s] %7lu ms  0x%03lX  %u  ",
           dir, (unsigned long)t, (unsigned long)f->id, (unsigned)f->len);
    for (int i = 0; i < f->len; i++) printf("%02X ", f->data[i]);
    for (int i = f->len; i < 8; i++) printf("   ");
    printf("\r\n");
}

static void drain_rx(void)
{
    chademo_can_frame_t f;
    while (hal_can_recv(CH, &f))
        print_frame("RX", hal_millis(), &f);
}

static void print_help(void)
{
    printf("\r\n----- Serial commands -----\r\n");
    printf("v=N        target voltage V      (0x102)\r\n");
    printf("a=N        target current A      (0x102)\r\n");
    printf("soc=N      SOC %%                 (0x102)\r\n");
    printf("status=0xN status flags          (0x102)\r\n");
    printf("fault=0xN  fault flags           (0x102)\r\n");
    printf("maxv=N     max battery V         (0x100)\r\n");
    printf("cap=N      capacity %%            (0x100)\r\n");
    printf("time10=N   max charge time 10s   (0x101)\r\n");
    printf("time60=N   max charge time 60s   (0x101)\r\n");
    printf("est60=N    est time 60s          (0x101)\r\n");
    printf("totwh=N    total cap 0.1kWh      (0x101)\r\n");
    printf("proto=N    protocol version      (0x102)\r\n");
    printf("help       show this list\r\n");
    printf("Current: v=%u a=%u soc=%u status=0x%02X fault=0x%02X\r\n",
           (unsigned)target_v, (unsigned)target_a, (unsigned)soc,
           (unsigned)status_flags, (unsigned)fault_flags);
    printf("---------------------------\r\n\r\n");
}

static void process_serial_cmd(const char *cmd)
{
    char key[16];
    char val_str[32];
    unsigned int val;

    if (sscanf(cmd, "%15[^=]=%31s", key, val_str) != 2) {
        if (strncmp(cmd, "help", 4) == 0) {
            print_help();
        }
        return;
    }

    val = (unsigned int)strtoul(val_str, NULL, 0);

    if      (strcmp(key, "v") == 0)      { target_v = (uint16_t)val; }
    else if (strcmp(key, "a") == 0)      { target_a = (uint8_t)val; }
    else if (strcmp(key, "soc") == 0)    { soc = (uint8_t)val; }
    else if (strcmp(key, "status") == 0) { status_flags = (uint8_t)val; }
    else if (strcmp(key, "fault") == 0)  { fault_flags = (uint8_t)val; }
    else if (strcmp(key, "maxv") == 0)   { bat_max_v = (uint16_t)val; }
    else if (strcmp(key, "cap") == 0)    { bat_cap_pct = (uint8_t)val; }
    else if (strcmp(key, "time10") == 0) { charge_time_10 = (uint8_t)val; }
    else if (strcmp(key, "time60") == 0) { charge_time_60 = (uint8_t)val; }
    else if (strcmp(key, "est60") == 0)  { est_time_60 = (uint8_t)val; }
    else if (strcmp(key, "totwh") == 0)  { total_cap_wh = (uint16_t)val; }
    else if (strcmp(key, "proto") == 0)  { protocol = (uint8_t)val; }
    else {
        printf("[WARN] Unknown cmd: %s\r\n", key);
        return;
    }

    printf("[SET] %s = %u (0x%X)\r\n", key, val, val);
}

static void poll_serial(void)
{
    static char buf[64];
    static int idx = 0;

    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\r' || c == '\n') {
            if (idx > 0) {
                buf[idx] = '\0';
                process_serial_cmd(buf);
                idx = 0;
            }
        } else if (idx < (int)sizeof(buf) - 1 && c >= 32 && c < 127) {
            buf[idx++] = (char)c;
        }
    }
}

/* -------------------------------------------------------------------------- */

int main(void)
{
    stdio_init_all();
    sleep_ms(2500);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    hal_gpio_init();

    printf("\r\n╔══════════════════════════════════════════════════════════════╗\r\n");
    printf("║     Energica Display — Keep-Alive / Vehicle Simulator        ║\r\n");
    printf("║     Type 'help' in serial to see live-tweak commands         ║\r\n");
    printf("╚══════════════════════════════════════════════════════════════╝\r\n");
    printf("[INIT] CAN1 @ %u kbps\r\n", (unsigned)BITRATE);

    if (!hal_can_init_with_bitrate(CH, BITRATE)) {
        printf("[FATAL] CAN init failed\r\n");
        while (1) sleep_ms(100);
    }
    printf("[INIT] CAN1 OK — sending 0x100/0x101/0x102 every 100 ms\r\n\r\n");

    chademo_tx_vehicle_t tx = {0};
    chademo_can_frame_t frames[3];

    absolute_time_t next_tx = get_absolute_time();
    bool led = false;

    while (1) {
        uint32_t now = hal_millis();

        poll_serial();

        tx.h100.max_battery_voltage_V   = bat_max_v;
        tx.h100.charged_rate_ref_const  = bat_cap_pct;
        tx.h101.max_charge_time_10s     = charge_time_10;
        tx.h101.max_charge_time_60s     = charge_time_60;
        tx.h101.est_charge_time_60s     = est_time_60;
        tx.h101.total_capacity_100wh    = total_cap_wh;
        tx.h102.protocol_number         = protocol;
        tx.h102.target_battery_voltage_V= target_v;
        tx.h102.charging_current_request_A = target_a;
        tx.h102.fault                   = fault_flags;
        tx.h102.status                  = status_flags;
        tx.h102.charged_rate_percent    = soc;

        chademo_can_pack_vehicle_frames(&tx, frames);

        if (absolute_time_diff_us(get_absolute_time(), next_tx) <= 0) {
            next_tx = delayed_by_ms(next_tx, 100);
            led = !led;
            gpio_put(LED_PIN, led);

            for (int i = 0; i < 3; i++) {
                hal_can_send(CH, &frames[i]);
                print_frame("TX", now, &frames[i]);
                drain_rx();
                sleep_ms(5);
            }
        }

        drain_rx();
        sleep_ms(5);
    }
}

bi_decl(bi_program_name("Energica Display Alive"));
bi_decl(bi_program_version_string("1.1.0"));
