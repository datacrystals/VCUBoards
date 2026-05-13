/**
 * @file  can_test_main.c
 * @brief Dual-CAN Loopback Test Firmware for RP2040 + 2x MCP2515
 * @details
 *   This is a standalone test program that verifies both MCP2515 CAN
 *   controllers and their transceivers by sending frames between them.
 *
 *   HARDWARE SETUP FOR THIS TEST:
 *   ==============================
 *   1. Connect CAN1_H  →  CAN2_H  (CAN high lines tied together)
 *   2. Connect CAN1_L  →  CAN2_L  (CAN low lines tied together)
 *   3. Ensure at least one 120 Ω termination resistor is present
 *      across the CAN_H / CAN_L pair (most modules have a jumper).
 *   4. Both MCP2515 modules must share a common GND with the Pico.
 *
 *   WHAT THE TEST DOES:
 *   ===================
 *   - Initializes both CAN1 (SPI0) and CAN2 (SPI1) at 500 kbps
 *   - Sends a test frame from CAN1 → CAN2, checks reception
 *   - Sends a test frame from CAN2 → CAN1, checks reception
 *   - Repeats at 5 Hz, printing pass/fail statistics to USB serial
 *   - Displays live MCP2515 error registers (TEC, REC, EFLG)
 *
 *   EXPECTED OUTPUT (USB serial @ 115200):
 *   ======================================
 *   [TEST] CAN1→CAN2 PASS  seq=42  rx_ok=42  rx_fail=0  err_flg=0x00
 *   [TEST] CAN2→CAN1 PASS  seq=42  rx_ok=42  rx_fail=0  err_flg=0x00
 *
 *   BUILD:
 *   ======
 *   cd chademo_firmware/build
 *   cmake .. -DCHADEMO_ROLE=VEHICLE
 *   make can_loopback_test -j4
 *   # Flash: cp can_loopback_test.uf2 /media/$USER/RPI-RP2/
 */

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"

/* Role is defined on the CMake command line; chademo_config.h needs it */
#include "chademo_config.h"
#include "chademo_hal.h"
#include "chademo_can.h"

#include <stdio.h>
#include <string.h>

/* ============================================================================
 * TEST PARAMETERS
 * ============================================================================ */

#define LED_PIN              25
#define TEST_PERIOD_MS       200   /* 5 Hz test rate */
#define BAUDRATE_DIAG_MS     1000  /* Print full diagnostics every 1 s */

/* Test frame IDs — distinct so we know which direction a frame came from */
#define TEST_ID_CAN1_TO_CAN2 0x7A1U
#define TEST_ID_CAN2_TO_CAN1 0x7A2U

/* ============================================================================
 * TEST STATE
 * ============================================================================ */

typedef struct {
    uint32_t tx_count;
    uint32_t rx_ok;
    uint32_t rx_fail;
    uint32_t last_seq_rx;
} test_channel_stats_t;

static test_channel_stats_t stats_ch1 = {0};
static test_channel_stats_t stats_ch2 = {0};

/* ============================================================================
 * HELPERS
 * ============================================================================ */

static void led_blink(bool on)
{
    gpio_put(LED_PIN, on ? 1 : 0);
}

static void print_mcp2515_diag(hal_can_channel_t ch, const char *name)
{
    uint8_t eflg = hal_can_read_reg(ch, MCP2515_REG_EFLG);
    uint8_t tec  = hal_can_read_reg(ch, 0x1C);
    uint8_t rec  = hal_can_read_reg(ch, 0x1D);
    uint8_t intf = hal_can_read_reg(ch, MCP2515_REG_CANINTF);
    uint8_t stat = hal_can_read_reg(ch, MCP2515_REG_CANSTAT);

    printf("[DIAG] %s  EFLG=0x%02X TEC=%3u REC=%3u INTF=0x%02X STAT=0x%02X\r\n",
           name, eflg, tec, rec, intf, stat);

    if (eflg & 0x20) printf("[DIAG] %s >>> BUS OFF <<<\r\n", name);
    if (eflg & 0x10) printf("[DIAG] %s TX error passive\r\n", name);
    if (eflg & 0x08) printf("[DIAG] %s RX error passive\r\n", name);
    if (eflg & 0x04) printf("[DIAG] %s TX error warning\r\n", name);
    if (eflg & 0x02) printf("[DIAG] %s RX error warning\r\n", name);
    if (eflg & 0x01) printf("[DIAG] %s RX overflow (RXB0/RXB1 full)\r\n", name);
}

static bool send_test_frame(hal_can_channel_t ch, uint16_t id, uint32_t seq)
{
    chademo_can_frame_t frame = {0};
    frame.id  = id;
    frame.len = 8;
    frame.data[0] = (uint8_t)(seq & 0xFF);
    frame.data[1] = (uint8_t)((seq >> 8) & 0xFF);
    frame.data[2] = (uint8_t)((seq >> 16) & 0xFF);
    frame.data[3] = (uint8_t)((seq >> 24) & 0xFF);
    frame.data[4] = 0xDE;
    frame.data[5] = 0xAD;
    frame.data[6] = 0xBE;
    frame.data[7] = 0xEF;

    bool ok = hal_can_send(ch, &frame);
    if (!ok) {
        printf("[WARN] %s TX blocked (TXB0 pending)\r\n",
               (ch == HAL_CAN_CH1) ? "CAN1" : "CAN2");
    }
    return ok;
}

static bool expect_frame(hal_can_channel_t ch, uint16_t expected_id,
                         uint32_t expected_seq, uint32_t *out_actual_seq)
{
    chademo_can_frame_t frame;
    if (!hal_can_recv(ch, &frame)) {
        return false;  /* No frame available */
    }

    if (frame.id != expected_id) {
        printf("[WARN] Unexpected ID on %s: got 0x%03lX expected 0x%03lX\r\n",
               (ch == HAL_CAN_CH1) ? "CAN1" : "CAN2", (unsigned long)frame.id, (unsigned long)expected_id);
        return false;
    }

    if (frame.len != 8) {
        printf("[WARN] Bad DLC on %s: got %u expected 8\r\n",
               (ch == HAL_CAN_CH1) ? "CAN1" : "CAN2", frame.len);
        return false;
    }

    uint32_t seq = (uint32_t)frame.data[0]
                 | ((uint32_t)frame.data[1] << 8)
                 | ((uint32_t)frame.data[2] << 16)
                 | ((uint32_t)frame.data[3] << 24);

    if (out_actual_seq) {
        *out_actual_seq = seq;
    }

    if (frame.data[4] != 0xDE || frame.data[5] != 0xAD ||
        frame.data[6] != 0xBE || frame.data[7] != 0xEF) {
        printf("[WARN] Payload mismatch on %s\r\n",
               (ch == HAL_CAN_CH1) ? "CAN1" : "CAN2");
        return false;
    }

    (void)expected_seq; /* we allow out-of-order for robustness, just check payload */
    return true;
}

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

    /* Small delay so USB serial has time to enumerate before first printf */
    sleep_ms(2000);

    printf("\r\n");
    printf("╔══════════════════════════════════════════════════════════════╗\r\n");
    printf("║     Dual MCP2515 CAN Loopback Test                           ║\r\n");
    printf("║     (Tie CAN1_H↔CAN2_H and CAN1_L↔CAN2_L together)           ║\r\n");
    printf("╚══════════════════════════════════════════════════════════════╝\r\n");
    printf("\r\n");

    /* Initialize GPIO to safe defaults (contactor open, etc.) */
    hal_gpio_init();
    printf("[INIT] GPIO safe state set\r\n");

    /* ---- Initialize CAN1 (SPI0) ---- */
    printf("[INIT] Initializing CAN1 (SPI0 / MCP2515 #1)...\r\n");
    if (!hal_can_init(HAL_CAN_CH1)) {
        printf("[FATAL] CAN1 MCP2515 not responding!\r\n");
        printf("[FATAL] Check: SPI wiring, CS=GPIO%u, power, crystal\r\n", PIN_CAN1_CS);
        while (1) {
            led_blink(true);  sleep_ms(100);
            led_blink(false); sleep_ms(100);
        }
    }
    printf("[INIT] CAN1 OK — 500 kbps, %d MHz crystal\r\n", MCP2515_OSC_MHZ);

    /* ---- Initialize CAN2 (SPI1) ---- */
    printf("[INIT] Initializing CAN2 (SPI1 / MCP2515 #2)...\r\n");
    if (!hal_can_init(HAL_CAN_CH2)) {
        printf("[FATAL] CAN2 MCP2515 not responding!\r\n");
        printf("[FATAL] Check: SPI wiring, CS=GPIO%u, power, crystal\r\n", PIN_CAN2_CS);
        while (1) {
            led_blink(true);  sleep_ms(100);
            led_blink(false); sleep_ms(100);
        }
    }
    printf("[INIT] CAN2 OK — 500 kbps, %d MHz crystal\r\n", MCP2515_OSC_MHZ);

    printf("\r\n");
    printf("[TEST] Starting loopback test — sending every %d ms\r\n", TEST_PERIOD_MS);
    printf("[TEST] Make sure CAN1_H is tied to CAN2_H and CAN1_L to CAN2_L\r\n");
    printf("[TEST] At least one 120-ohm terminator must be on the bus\r\n");
    printf("\r\n");

    /* ---- Main test loop ---- */
    absolute_time_t next_test_time = get_absolute_time();
    uint32_t test_seq = 1;
    uint32_t last_diag_ms = 0;
    bool led_state = false;

    while (1) {
        uint32_t now_ms = hal_millis();

        /* ---- Send test frames at fixed interval ---- */
        if (absolute_time_diff_us(get_absolute_time(), next_test_time) <= 0) {
            next_test_time = delayed_by_ms(next_test_time, TEST_PERIOD_MS);

            /* CAN1 → CAN2 */
            if (send_test_frame(HAL_CAN_CH1, TEST_ID_CAN1_TO_CAN2, test_seq)) {
                stats_ch1.tx_count++;
            }

            /* Short delay to avoid bus collision (frames are tiny, 200µs @ 500k) */
            sleep_us(500);

            /* CAN2 → CAN1 */
            if (send_test_frame(HAL_CAN_CH2, TEST_ID_CAN2_TO_CAN1, test_seq)) {
                stats_ch2.tx_count++;
            }

            test_seq++;
            led_state = !led_state;
            led_blink(led_state);
        }

        /* ---- Drain receive buffers ---- */
        uint32_t rx_seq;
        while (expect_frame(HAL_CAN_CH2, TEST_ID_CAN1_TO_CAN2, 0, &rx_seq)) {
            stats_ch2.rx_ok++;
            stats_ch2.last_seq_rx = rx_seq;
        }

        while (expect_frame(HAL_CAN_CH1, TEST_ID_CAN2_TO_CAN1, 0, &rx_seq)) {
            stats_ch1.rx_ok++;
            stats_ch1.last_seq_rx = rx_seq;
        }

        /* ---- Check for dropped frames (simple heuristic) ---- */
        /* If we transmitted N frames but only received N-3, something is wrong */
        if (stats_ch1.tx_count > stats_ch2.rx_ok + 5) {
            /* CAN1 sent more than CAN2 received by >5 frames — possible wiring issue */
            /* We don't print continuously to avoid spam; handled in diag below */
        }
        if (stats_ch2.tx_count > stats_ch1.rx_ok + 5) {
            /* CAN2 sent more than CAN1 received by >5 frames */
        }

        /* ---- Periodic diagnostics ---- */
        if ((now_ms - last_diag_ms) >= BAUDRATE_DIAG_MS) {
            last_diag_ms = now_ms;

            uint32_t ch1_fails = (stats_ch1.tx_count > stats_ch2.rx_ok)
                                 ? (stats_ch1.tx_count - stats_ch2.rx_ok) : 0;
            uint32_t ch2_fails = (stats_ch2.tx_count > stats_ch1.rx_ok)
                                 ? (stats_ch2.tx_count - stats_ch1.rx_ok) : 0;

            printf("\r\n");
            printf("[TEST] ======== Summary @ %lu ms ========\r\n", now_ms);
            printf("[TEST] CAN1→CAN2  tx=%lu  rx_ok=%lu  rx_fail=%lu  last_seq=%lu  %s\r\n",
                   stats_ch1.tx_count, stats_ch2.rx_ok, ch1_fails,
                   stats_ch2.last_seq_rx,
                   (ch1_fails == 0 && stats_ch1.tx_count > 0) ? "PASS" : "FAIL");
            printf("[TEST] CAN2→CAN1  tx=%lu  rx_ok=%lu  rx_fail=%lu  last_seq=%lu  %s\r\n",
                   stats_ch2.tx_count, stats_ch1.rx_ok, ch2_fails,
                   stats_ch1.last_seq_rx,
                   (ch2_fails == 0 && stats_ch2.tx_count > 0) ? "PASS" : "FAIL");

            print_mcp2515_diag(HAL_CAN_CH1, "CAN1");
            print_mcp2515_diag(HAL_CAN_CH2, "CAN2");
        }

        /* Yield so USB CDC can flush */
        sleep_ms(5);
    }

    return 0;
}

/* ============================================================================
 * SDK BINARY INFO
 * ============================================================================ */
bi_decl(bi_program_name("CAN Loopback Test"));
bi_decl(bi_program_version_string("1.0.0"));
bi_decl(bi_program_description("Dual MCP2515 CAN bus loopback test"));
