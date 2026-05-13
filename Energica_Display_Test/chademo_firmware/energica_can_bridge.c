/**
 * @file  energica_can_bridge.c
 * @brief USB-serial ↔ CAN bridge (simple, no auto-flood)
 * @details
 *   Receives lines from USB serial in candump format:
 *       102#025E010000084B00
 *   Sends them out on CAN1. Every CAN frame received is printed back
 *   in the same format:
 *       400#0F6F0BCE00820080
 *
 *   BUILD:  cd build && cmake .. && make energica_can_bridge -j4
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

static void led_toggle(void)
{
    static bool s = false;
    s = !s;
    gpio_put(LED_PIN, s ? 1 : 0);
}

static void print_can_frame(const chademo_can_frame_t *f)
{
    printf("%03lX#", (unsigned long)f->id);
    for (int i = 0; i < f->len; i++)
        printf("%02X", f->data[i]);
    printf("\r\n");
}

static bool parse_line(const char *line, chademo_can_frame_t *f)
{
    char *hash = strchr(line, '#');
    if (!hash) return false;

    char id_str[16];
    int id_len = (int)(hash - line);
    if (id_len >= (int)sizeof(id_str)) id_len = (int)sizeof(id_str) - 1;
    memcpy(id_str, line, id_len);
    id_str[id_len] = '\0';
    f->id = (uint32_t)strtoul(id_str, NULL, 16);

    const char *p = hash + 1;
    f->len = 0;
    while (*p && f->len < 8) {
        char byte_str[3] = {p[0], p[1], '\0'};
        if (!byte_str[0] || !byte_str[1]) break;
        f->data[f->len++] = (uint8_t)strtoul(byte_str, NULL, 16);
        p += 2;
    }
    return true;
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2500);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    hal_gpio_init();

    printf("\r\n");
    printf("# Energica CAN Bridge ready\r\n");
    printf("# Format:  102#025E010000084B00\r\n");
    printf("# Bitrate: %u kbps\r\n", (unsigned)BITRATE);

    if (!hal_can_init_with_bitrate(CH, BITRATE)) {
        printf("# FATAL: CAN init failed\r\n");
        while (1) sleep_ms(100);
    }
    printf("# CAN1 OK\r\n");

    static char buf[128];
    int idx = 0;

    while (1) {
        chademo_can_frame_t rx;
        while (hal_can_recv(CH, &rx)) {
            print_can_frame(&rx);
            led_toggle();
        }

        int c;
        while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            if (c == '\r' || c == '\n') {
                if (idx > 0) {
                    buf[idx] = '\0';
                    chademo_can_frame_t tx;
                    if (parse_line(buf, &tx)) {
                        hal_can_send(CH, &tx);
                    } else if (buf[0] != '\0' && buf[0] != '#') {
                        printf("# ERR bad frame: %s\r\n", buf);
                    }
                    idx = 0;
                }
            } else if (idx < (int)sizeof(buf) - 2 && c >= 32 && c < 127) {
                buf[idx++] = (char)c;
            }
        }

        sleep_us(100);
    }
}

bi_decl(bi_program_name("Energica CAN Bridge"));
bi_decl(bi_program_version_string("1.0.0"));
