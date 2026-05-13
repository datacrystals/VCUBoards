/**
 * @file  energica_display_manual.c
 * @brief Manual CAN playground + keep-alive for Energica display
 * @details
 *   Keeps 0x100/0x101/0x102 running in background so display stays awake,
 *   but ALSO lets you fire arbitrary CAN frames from the serial prompt.
 *
 *   SERIAL COMMANDS:
 *     v=350              set 0x102 target voltage
 *     a=50               set 0x102 target current
 *     soc=75             set 0x102 SOC
 *     status=0x09        set 0x102 status flags
 *     s 0x200 01 02 ...  send one-shot frame (up to 8 data bytes)
 *     r 0x200 100 01 ... start repeating frame every N ms
 *     stop               stop all repeating frames
 *     help               show commands
 *
 *   BUILD:  cd build && cmake .. && make energica_display_manual -j4
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

/* ---- live vehicle state ---- */
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

/* ---- repeating frame slot ---- */
static struct {
    bool     active;
    uint32_t period_ms;
    uint32_t last_tx;
    chademo_can_frame_t frame;
} g_repeat = {0};

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

static bool send_frame(const chademo_can_frame_t *f)
{
    bool ok = hal_can_send(CH, f);
    if (ok) print_frame("TX", hal_millis(), f);
    return ok;
}

static void build_vehicle_frames(chademo_can_frame_t out[3])
{
    chademo_tx_vehicle_t tx = {0};
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
    chademo_can_pack_vehicle_frames(&tx, out);
}

static void print_help(void)
{
    printf("\r\n----- Commands -----\r\n");
    printf("v=350        target voltage V       (0x102)\r\n");
    printf("a=50         target current A       (0x102)\r\n");
    printf("soc=75       SOC %%                  (0x102)\r\n");
    printf("status=0x09  status flags           (0x102)\r\n");
    printf("fault=0x00   fault flags            (0x102)\r\n");
    printf("maxv=420     max battery V          (0x100)\r\n");
    printf("cap=100      capacity %%             (0x100)\r\n");
    printf("s 0x200 d0 d1 ... d7   send one frame\r\n");
    printf("r 0x200 100 d0 d1 ...  repeat every 100 ms\r\n");
    printf("stop         stop repeating\r\n");
    printf("help         show this\r\n");
    printf("--------------------\r\n\r\n");
}

static void handle_cmd(char *cmd)
{
    while (*cmd == ' ') cmd++;
    if (cmd[0] == '\0') return;

    /* ---- simple key=val ---- */
    char key[16], val_str[32];
    if (sscanf(cmd, "%15[^=]=%31s", key, val_str) == 2) {
        unsigned int val = (unsigned int)strtoul(val_str, NULL, 0);
        if      (!strcmp(key, "v"))       { target_v = (uint16_t)val; }
        else if (!strcmp(key, "a"))       { target_a = (uint8_t)val; }
        else if (!strcmp(key, "soc"))     { soc = (uint8_t)val; }
        else if (!strcmp(key, "status"))  { status_flags = (uint8_t)val; }
        else if (!strcmp(key, "fault"))   { fault_flags = (uint8_t)val; }
        else if (!strcmp(key, "maxv"))    { bat_max_v = (uint16_t)val; }
        else if (!strcmp(key, "cap"))     { bat_cap_pct = (uint8_t)val; }
        else if (!strcmp(key, "time10"))  { charge_time_10 = (uint8_t)val; }
        else if (!strcmp(key, "time60"))  { charge_time_60 = (uint8_t)val; }
        else if (!strcmp(key, "est60"))   { est_time_60 = (uint8_t)val; }
        else if (!strcmp(key, "totwh"))   { total_cap_wh = (uint16_t)val; }
        else if (!strcmp(key, "proto"))   { protocol = (uint8_t)val; }
        else { printf("[WARN] unknown: %s\r\n", key); return; }
        printf("[SET] %s = %u\r\n", key, val);
        return;
    }

    /* ---- send one ---- */
    if (cmd[0] == 's' && cmd[1] == ' ') {
        char *p = cmd + 2;
        uint32_t id = (uint32_t)strtoul(p, &p, 0);
        chademo_can_frame_t f = {.id = id, .len = 0};
        while (*p == ' ') p++;
        for (int i = 0; i < 8; i++) {
            if (*p == '\0') break;
            f.data[i] = (uint8_t)strtoul(p, &p, 0);
            f.len++;
            while (*p == ' ') p++;
        }
        send_frame(&f);
        return;
    }

    /* ---- repeat ---- */
    if (cmd[0] == 'r' && cmd[1] == ' ') {
        char *p = cmd + 2;
        uint32_t id = (uint32_t)strtoul(p, &p, 0);
        uint32_t period = (uint32_t)strtoul(p, &p, 0);
        chademo_can_frame_t f = {.id = id, .len = 0};
        while (*p == ' ') p++;
        for (int i = 0; i < 8; i++) {
            if (*p == '\0') break;
            f.data[i] = (uint8_t)strtoul(p, &p, 0);
            f.len++;
            while (*p == ' ') p++;
        }
        g_repeat.active = true;
        g_repeat.period_ms = period ? period : 100;
        g_repeat.frame = f;
        g_repeat.last_tx = 0;
        printf("[REP] 0x%03lX every %lu ms, %u bytes\r\n",
               (unsigned long)id, (unsigned long)g_repeat.period_ms, (unsigned)f.len);
        return;
    }

    /* ---- stop ---- */
    if (!strncmp(cmd, "stop", 4)) {
        g_repeat.active = false;
        printf("[REP] stopped\r\n");
        return;
    }

    if (!strncmp(cmd, "help", 4)) {
        print_help();
        return;
    }

    printf("[WARN] unknown cmd: %s\r\n", cmd);
}

static void poll_serial(void)
{
    static char buf[128];
    static int idx = 0;

    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\r' || c == '\n') {
            if (idx > 0) {
                buf[idx] = '\0';
                handle_cmd(buf);
                idx = 0;
            }
        } else if (idx < (int)sizeof(buf) - 2 && c >= 32 && c < 127) {
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
    printf("║     Energica Display — Manual CAN Playground                 ║\r\n");
    printf("║     Type 'help' for commands                                 ║\r\n");
    printf("╚══════════════════════════════════════════════════════════════╝\r\n");

    if (!hal_can_init_with_bitrate(CH, BITRATE)) {
        printf("[FATAL] CAN init failed\r\n");
        while (1) sleep_ms(100);
    }
    printf("[INIT] CAN1 OK @ %u kbps\r\n\r\n", (unsigned)BITRATE);

    chademo_can_frame_t vframes[3];
    absolute_time_t next_v = get_absolute_time();
    bool led = false;

    while (1) {
        uint32_t now = hal_millis();
        poll_serial();

        /* ---- background vehicle keep-alive (100 ms) ---- */
        if (absolute_time_diff_us(get_absolute_time(), next_v) <= 0) {
            next_v = delayed_by_ms(next_v, 100);
            led = !led;
            gpio_put(LED_PIN, led);

            build_vehicle_frames(vframes);
            for (int i = 0; i < 3; i++) {
                hal_can_send(CH, &vframes[i]);
                drain_rx();
                sleep_ms(2);
            }
        }

        /* ---- repeating user frame ---- */
        if (g_repeat.active && (now - g_repeat.last_tx) >= g_repeat.period_ms) {
            g_repeat.last_tx = now;
            send_frame(&g_repeat.frame);
        }

        drain_rx();
        sleep_ms(5);
    }
}

bi_decl(bi_program_name("Energica Display Manual"));
bi_decl(bi_program_version_string("1.0.0"));
