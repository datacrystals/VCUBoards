/**
 * @file  energica_display_focused.c
 * @brief Focused periodic CAN probe for Energica display wake-up
 * @details
 *   Unlike the rapid sweep, this sends ONE ID at a time periodically
 *   (10 Hz) for a few seconds so the display has time to react.
 *   This lets you map exactly which ID causes which behaviour.
 *
 *   It also echoes back any 0x400 frames it sees, in case the display
 *   expects a handshake / ack.
 *
 *   BUILD:
 *   cd build && cmake .. && make energica_display_focused -j4
 */

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"

#include "chademo_config.h"
#include "chademo_hal.h"
#include "chademo_can.h"

#include <stdio.h>
#include <string.h>

#define LED_PIN             25
#define PROBE_CH            HAL_CAN_CH1

/* How long to hammer each ID (ms) */
#define DWELL_MS            3000

/* Periodic TX rate while dwelling (ms) */
#define PERIOD_MS           100

/* Default bitrate — change if your MCP2515 crystal needs it */
#define PROBE_BITRATE       500

/* ============================================================================
 * STATE
 * ============================================================================ */

static uint32_t g_rx_count = 0;
static uint32_t g_tx_count = 0;
static uint8_t  g_last_400_payload[8];
static bool     g_got_400 = false;

/* ============================================================================
 * HELPERS
 * ============================================================================ */

static void print_frame(const char *dir, uint32_t t, const chademo_can_frame_t *f)
{
    printf("[%s] %6lu ms  0x%03lX  %u  ",
           dir, (unsigned long)t, (unsigned long)f->id, (unsigned)f->len);
    for (int i = 0; i < f->len; i++) printf("%02X ", f->data[i]);
    for (int i = f->len; i < 8; i++) printf("   ");
    printf("\r\n");
}

static void drain_rx(void)
{
    chademo_can_frame_t f;
    while (hal_can_recv(PROBE_CH, &f)) {
        g_rx_count++;
        print_frame("RX", hal_millis(), &f);

        if (f.id == 0x400) {
            memcpy(g_last_400_payload, f.data, 8);
            g_got_400 = true;
        }
    }
}

static bool send_id(uint16_t id, const uint8_t *data, uint8_t len)
{
    chademo_can_frame_t f = {.id = id, .len = len};
    memcpy(f.data, data, len);
    bool ok = hal_can_send(PROBE_CH, &f);
    if (ok) g_tx_count++;
    return ok;
}

/* ============================================================================
 * MAIN
 * ============================================================================ */

int main(void)
{
    stdio_init_all();
    sleep_ms(2500);  /* USB CDC enumerate */

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    hal_gpio_init();

    printf("\r\n");
    printf("╔══════════════════════════════════════════════════════════════╗\r\n");
    printf("║     Energica Display — Focused Periodic Probe                ║\r\n");
    printf("╚══════════════════════════════════════════════════════════════╝\r\n");
    printf("[INIT] CAN1 @ %u kbps, %u MHz crystal config\r\n",
           (unsigned)PROBE_BITRATE, (unsigned)MCP2515_OSC_MHZ);

    if (!hal_can_init_with_bitrate(PROBE_CH, PROBE_BITRATE)) {
        printf("[FATAL] CAN1 init failed\r\n");
        while (1) { sleep_ms(100); }
    }
    printf("[INIT] CAN1 OK\r\n");
    printf("[INFO] Each ID is sent every %u ms for %u seconds\r\n",
           (unsigned)PERIOD_MS, (unsigned)(DWELL_MS / 1000));
    printf("[INFO] WATCH THE DISPLAY! Note which ID triggers lights/screen.\r\n");
    printf("\r\n");

    /* Payloads to try per ID */
    static const uint8_t payloads[][8] = {
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    };
    const int num_payloads = 3;

    for (uint16_t id = 0x000; id <= 0x7FF; id++) {
        printf("\r\n[DWELL] === ID 0x%03X ===\r\n", (unsigned)id);

        uint32_t dwell_start = hal_millis();
        uint32_t last_tx = 0;
        bool led = false;

        while ((hal_millis() - dwell_start) < DWELL_MS) {
            /* Periodic TX */
            if ((hal_millis() - last_tx) >= PERIOD_MS) {
                last_tx = hal_millis();
                led = !led;
                gpio_put(LED_PIN, led ? 1 : 0);

                for (int p = 0; p < num_payloads; p++) {
                    send_id(id, payloads[p], 8);
                    drain_rx();
                    sleep_ms(5);
                }
            }

            /* Also echo back the most recent 0x400 frame */
            if (g_got_400) {
                send_id(0x400, g_last_400_payload, 8);
                g_got_400 = false;  /* only echo once per received frame */
            }

            drain_rx();
            sleep_ms(5);
        }
    }

    printf("\r\n[DONE] Full sweep complete. TX=%lu RX=%lu\r\n",
           (unsigned long)g_tx_count, (unsigned long)g_rx_count);
    printf("[DONE] Entering idle listen mode...\r\n");

    while (1) {
        drain_rx();
        sleep_ms(10);
    }
}

bi_decl(bi_program_name("Energica Display Focused Probe"));
bi_decl(bi_program_version_string("1.0.0"));
