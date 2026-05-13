/**
 * @file  energica_display_probe.c
 * @brief Energica Display CAN Reverse-Engineering Probe
 * @details
 *   Connects to the Energica motorcycle display via CAN1 (SPI0 / non-CHAdeMO
 *   bus) and systematically probes for wake-up and response patterns.
 *
 *   HARDWARE SETUP:
 *   ===============
 *   - CAN1_H  →  Display CAN_H
 *   - CAN1_L  →  Display CAN_L
 *   - Common GND between Pico and display
 *   - 120 Ω termination resistor on the bus (display may have one built-in)
 *
 *   WHAT IT DOES:
 *   =============
 *   1. Tries bitrates: 125, 250, 500, 1000 kbps (most likely 500 or 250)
 *   2. For each bitrate, sweeps all standard CAN IDs (0x000 – 0x7FF)
 *   3. Sends multiple payload patterns per ID:
 *        - All 0x00           (safe / default)
 *        - All 0xFF           (wake / broadcast)
 *        - 0x01 0x00...       (single-bit wake)
 *        - Incremental 0x01..0x08
 *   4. Listens for responses after every single transmission
 *   5. Logs EVERYTHING with timestamps over USB serial
 *
 *   OUTPUT FORMAT:
 *   ==============
 *   [TX] 12345ms  0x123  8  00 00 00 00 00 00 00 00
 *   [RX] 12346ms  0x456  8  DE AD BE EF CA FE BA BE
 *
 *   BUILD & FLASH:
 *   ==============
 *   cd build && cmake .. && make energica_display_probe -j4
 *   cp energica_display_probe.uf2 /media/$USER/RPI-RP2/
 *
 *   MONITOR:
 *   ========
 *   ./monitor_serial.sh   (captures USB serial to a timestamped log file)
 */

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"

#include "chademo_config.h"
#include "chademo_hal.h"
#include "chademo_can.h"

#include <stdio.h>
#include <string.h>

/* ============================================================================
 * CONFIGURATION
 * ============================================================================ */

#define LED_PIN                 25
#define PROBE_CAN_CHANNEL       HAL_CAN_CH1   /* Non-CHAdeMO bus */

/* Delay between frames in the sweep (ms). 10ms = ~100 frames/sec.
 * Increase if you see TX blocked warnings. */
#define INTER_FRAME_DELAY_MS    10

/* How long to listen after completing an ID sweep (ms) */
#define POST_SWEEP_LISTEN_MS    500

/* How many payload patterns to send per ID */
#define NUM_PAYLOAD_PATTERNS    4

/* If 1, do a full 0x000–0x7FF sweep. If 0, only probe "interesting" IDs. */
#define FULL_SWEEP              1

/* ============================================================================
 * PAYLOAD PATTERNS
 * ============================================================================ */

static const uint8_t payload_patterns[NUM_PAYLOAD_PATTERNS][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* All zeros */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* All 0xFF */
    {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* Single-bit wake */
    {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}, /* Incremental */
};

#if !FULL_SWEEP
static const char *pattern_names[NUM_PAYLOAD_PATTERNS] = {
    "ZERO", "0xFF", "WAKE", "INCR"
};
#endif

/* ============================================================================
 * BITRATES TO TEST
 * ============================================================================ */

static const uint16_t test_bitrates[] = {500, 250, 125, 1000};
#define NUM_BITRATES (sizeof(test_bitrates) / sizeof(test_bitrates[0]))

/* ============================================================================
 * "INTERESTING" ID LIST (used when FULL_SWEEP == 0)
 * ============================================================================ */

#if !FULL_SWEEP
static const uint16_t interesting_ids[] = {
    /* Common wake-up / broadcast */
    0x000, 0x001, 0x002, 0x003,
    /* CHAdeMO-adjacent */
    0x100, 0x101, 0x102, 0x103, 0x104, 0x105, 0x106,
    0x108, 0x109, 0x10A,
    /* Motorcycle / EV common */
    0x130, 0x1F0, 0x1F1, 0x1F3,
    0x200, 0x201, 0x230, 0x280, 0x290,
    0x320, 0x360, 0x370, 0x380, 0x3B0,
    /* Instrument cluster / display */
    0x500, 0x520, 0x530, 0x540, 0x550, 0x560, 0x570,
    /* Manufacturer / diagnostic */
    0x600, 0x610, 0x620, 0x630,
    0x700, 0x701,
    /* UDS / OBD-II */
    0x7DF, 0x7E0, 0x7E1, 0x7E2, 0x7E3,
    0x7E8, 0x7E9, 0x7EA, 0x7EB, 0x7EC, 0x7ED, 0x7EE, 0x7EF,
    /* Some other guesses */
    0x150, 0x160, 0x170, 0x180, 0x190,
    0x250, 0x260, 0x270, 0x2A0, 0x2B0, 0x2C0, 0x2D0, 0x2E0, 0x2F0,
    0x310, 0x330, 0x340, 0x350, 0x390, 0x3A0, 0x3C0, 0x3D0, 0x3E0, 0x3F0,
    0x510, 0x580, 0x590, 0x5A0, 0x5B0, 0x5C0, 0x5D0, 0x5E0, 0x5F0,
    0x640, 0x650, 0x660, 0x670, 0x680, 0x690, 0x6A0, 0x6B0, 0x6C0, 0x6D0, 0x6E0, 0x6F0,
};
#define NUM_INTERESTING_IDS (sizeof(interesting_ids) / sizeof(interesting_ids[0]))
#endif

/* ============================================================================
 * STATS
 * ============================================================================ */

typedef struct {
    uint32_t tx_attempts;
    uint32_t tx_ok;
    uint32_t tx_blocked;
    uint32_t rx_count;
    uint32_t rx_unique_ids;   /* Number of distinct IDs we heard back */
    uint16_t rx_ids[32];      /* List of IDs we got responses from */
    uint8_t  rx_id_count;
} probe_stats_t;

static probe_stats_t g_stats;

/* ============================================================================
 * HELPERS
 * ============================================================================ */

static void led_set(bool on)
{
    gpio_put(LED_PIN, on ? 1 : 0);
}

static void print_frame(const char *dir, uint32_t timestamp_ms,
                        const chademo_can_frame_t *f)
{
    printf("[%s] %5lu ms  0x%03lX  %u  ",
           dir, (unsigned long)timestamp_ms,
           (unsigned long)f->id, (unsigned)f->len);
    for (int i = 0; i < f->len; i++) {
        printf("%02X ", f->data[i]);
    }
    /* Pad for alignment if short DLC */
    for (int i = f->len; i < 8; i++) {
        printf("   ");
    }
    /* ASCII hint */
    printf(" | ");
    for (int i = 0; i < f->len; i++) {
        uint8_t c = f->data[i];
        printf("%c", (c >= 32 && c < 127) ? c : '.');
    }
    printf("\r\n");
}

static void drain_rx_and_log(uint32_t listen_ms)
{
    chademo_can_frame_t frame;
    uint32_t start = hal_millis();
    while ((hal_millis() - start) < listen_ms) {
        while (hal_can_recv(PROBE_CAN_CHANNEL, &frame)) {
            g_stats.rx_count++;
            print_frame("RX", hal_millis(), &frame);

            /* Track unique response IDs */
            bool known = false;
            for (int i = 0; i < g_stats.rx_id_count; i++) {
                if (g_stats.rx_ids[i] == frame.id) {
                    known = true;
                    break;
                }
            }
            if (!known && g_stats.rx_id_count < 32) {
                g_stats.rx_ids[g_stats.rx_id_count++] = (uint16_t)frame.id;
            }
        }
        sleep_ms(1);
    }
}

static bool send_probe_frame(uint16_t id, const uint8_t *data, uint8_t len)
{
    chademo_can_frame_t frame = {0};
    frame.id = id;
    frame.len = len;
    if (len > 8) len = 8;
    memcpy(frame.data, data, len);

    g_stats.tx_attempts++;
    bool ok = hal_can_send(PROBE_CAN_CHANNEL, &frame);
    if (ok) {
        g_stats.tx_ok++;
    } else {
        g_stats.tx_blocked++;
    }
    return ok;
}

static void print_header(void)
{
    printf("\r\n");
    printf("╔══════════════════════════════════════════════════════════════╗\r\n");
    printf("║     Energica Display CAN Probe                               ║\r\n");
    printf("║     Reverse-engineering tool for Energica instrument cluster ║\r\n");
    printf("╚══════════════════════════════════════════════════════════════╝\r\n");
    printf("\r\n");
    printf("[INFO] Using CAN1 (SPI0) — NOT the CHAdeMO bus\r\n");
    printf("[INFO] Make sure display CAN_H/CAN_L are wired to CAN1_H/CAN1_L\r\n");
    printf("[INFO] Ensure common GND and 120-ohm termination\r\n");
    printf("\r\n");
}

static void print_banner_bitrate(uint16_t kbps)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("  PROBING AT %u kbps\r\n", (unsigned)kbps);
    printf("========================================\r\n");
}

static void print_summary(void)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("  PROBE COMPLETE\r\n");
    printf("========================================\r\n");
    printf("Total TX attempts:  %lu\r\n", (unsigned long)g_stats.tx_attempts);
    printf("Total TX success:   %lu\r\n", (unsigned long)g_stats.tx_ok);
    printf("Total TX blocked:   %lu\r\n", (unsigned long)g_stats.tx_blocked);
    printf("Total RX frames:    %lu\r\n", (unsigned long)g_stats.rx_count);
    printf("Unique response IDs: %u\r\n", (unsigned)g_stats.rx_id_count);
    if (g_stats.rx_id_count > 0) {
        printf("Response ID list:   ");
        for (int i = 0; i < g_stats.rx_id_count; i++) {
            printf("0x%03X ", g_stats.rx_ids[i]);
        }
        printf("\r\n");
    }
    printf("\r\n");
}

static void reset_stats(void)
{
    memset(&g_stats, 0, sizeof(g_stats));
}

/* ============================================================================
 * MAIN SWEEP LOGIC
 * ============================================================================ */

static void sweep_id(uint16_t id)
{
    for (int p = 0; p < NUM_PAYLOAD_PATTERNS; p++) {
        bool ok = send_probe_frame(id, payload_patterns[p], 8);

        /* Log every TX so the user sees the exact sequence */
        if (ok) {
            chademo_can_frame_t f = {
                .id = id,
                .len = 8,
            };
            memcpy(f.data, payload_patterns[p], 8);
            print_frame("TX", hal_millis(), &f);
        } else {
            printf("[TX] %5lu ms  0x%03X  BLOCKED (TXB0 pending)\r\n",
                   (unsigned long)hal_millis(), (unsigned)id);
        }

        /* Immediately listen for any response */
        drain_rx_and_log(5);  /* 5 ms window after each TX */

        /* Small inter-frame delay */
        sleep_ms(INTER_FRAME_DELAY_MS);
    }
}

static void run_probe_for_bitrate(uint16_t kbps)
{
    reset_stats();
    print_banner_bitrate(kbps);

    /* Re-initialize CAN1 with target bitrate */
    printf("[INIT] Setting CAN1 to %u kbps...\r\n", (unsigned)kbps);
    if (!hal_can_init_with_bitrate(PROBE_CAN_CHANNEL, kbps)) {
        printf("[FATAL] Failed to init CAN1 at %u kbps\r\n", (unsigned)kbps);
        return;
    }
    printf("[INIT] CAN1 OK at %u kbps\r\n", (unsigned)kbps);

    /* Give the bus a moment to settle */
    sleep_ms(100);

    /* Drain any stale frames from previous bitrate */
    drain_rx_and_log(50);

#if FULL_SWEEP
    printf("[SWEEP] Full ID sweep 0x000 → 0x7FF (%u patterns per ID)\r\n",
           (unsigned)NUM_PAYLOAD_PATTERNS);
    printf("[SWEEP] Estimated time: ~%lu seconds\r\n",
           (unsigned long)((2048UL * NUM_PAYLOAD_PATTERNS * (INTER_FRAME_DELAY_MS + 5)) / 1000));

    for (uint16_t id = 0x000; id <= 0x7FF; id++) {
        /* Visual heartbeat every 64 IDs */
        if ((id & 0x3F) == 0) {
            led_set(true);
        } else if ((id & 0x3F) == 0x20) {
            led_set(false);
        }

        sweep_id(id);
    }
#else
    printf("[SWEEP] Probing %u interesting IDs (%u patterns each)\r\n",
           (unsigned)NUM_INTERESTING_IDS, (unsigned)NUM_PAYLOAD_PATTERNS);
    for (size_t i = 0; i < NUM_INTERESTING_IDS; i++) {
        if ((i & 0x07) == 0) led_set(true);
        else if ((i & 0x07) == 0x04) led_set(false);

        sweep_id(interesting_ids[i]);
    }
#endif

    /* Final listen window */
    printf("[SWEEP] Done. Listening for %u ms...\r\n",
           (unsigned)POST_SWEEP_LISTEN_MS);
    drain_rx_and_log(POST_SWEEP_LISTEN_MS);

    printf("[SWEEP] Bitrate %u kbps complete. TX=%lu OK=%lu RX=%lu\r\n",
           (unsigned)kbps,
           (unsigned long)g_stats.tx_attempts,
           (unsigned long)g_stats.tx_ok,
           (unsigned long)g_stats.rx_count);

    if (g_stats.rx_id_count > 0) {
        printf("[HIT!] Responses seen at this bitrate from IDs: ");
        for (int i = 0; i < g_stats.rx_id_count; i++) {
            printf("0x%03X ", g_stats.rx_ids[i]);
        }
        printf("\r\n");
    }
}

/* ============================================================================
 * OPTIONAL: FOCUSED RE-PROBE ON A SINGLE ID
 * ============================================================================
 * If the user sees a response from a particular ID, they can rebuild with
 * FOCUS_ID defined to hammer just that ID with many payloads.
 */

#ifdef FOCUS_ID
static void run_focused_probe(uint16_t focus_id)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("  FOCUSED PROBE ON ID 0x%03X\r\n", (unsigned)focus_id);
    printf("========================================\r\n");

    for (int i = 0; i < 256; i++) {
        uint8_t payload[8] = {(uint8_t)i, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_probe_frame(focus_id, payload, 8);
        drain_rx_and_log(10);
        sleep_ms(20);
    }
}
#endif

/* ============================================================================
 * ENTRY POINT
 * ============================================================================ */

int main(void)
{
    stdio_init_all();

    /* LED init */
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    /* Wait for USB CDC to enumerate so we don't lose early output */
    sleep_ms(2500);

    print_header();

    /* Set GPIO safe state (contactor open, etc.) */
    hal_gpio_init();

    /* ---- Run probe across all bitrates ---- */
    for (size_t b = 0; b < NUM_BITRATES; b++) {
        run_probe_for_bitrate(test_bitrates[b]);
    }

    /* ---- Final summary ---- */
    print_summary();

    printf("[DONE] All probes complete. Entering idle loop.\r\n");
    printf("[DONE] The Pico will continue to log any spontaneous CAN traffic.\r\n");
    printf("[DONE] Press RESET or power-cycle to re-run the probe.\r\n");

    /* Idle: just blink LED and listen for any traffic */
    while (1) {
        led_set(true);
        drain_rx_and_log(250);
        led_set(false);
        drain_rx_and_log(250);
    }

    return 0;
}

/* ============================================================================
 * SDK BINARY INFO
 * ============================================================================ */
bi_decl(bi_program_name("Energica Display CAN Probe"));
bi_decl(bi_program_version_string("1.0.0"));
bi_decl(bi_program_description("CAN reverse-engineering probe for Energica display"));
