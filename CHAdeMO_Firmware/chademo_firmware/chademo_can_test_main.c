/**
 * @file  chademo_can_test_main.c
 * @brief CHAdeMO Protocol CAN Validation Test — Dual MCP2515
 * @details
 *   This standalone test validates the actual CHAdeMO CAN message pack/unpack
 *   logic by making the two onboard MCP2515 controllers talk to each other
 *   using real CHAdeMO frame IDs and payloads.
 *
 *   HARDWARE SETUP:
 *   ===============
 *   - Tie CAN1_H  →  CAN2_H  (CAN high)
 *   - Tie CAN1_L  →  CAN2_L  (CAN low)
 *   - At least one 120 Ω termination resistor across H/L
 *   - Both MCP2515 modules share GND with the Pico
 *
 *   ROLE ASSIGNMENT:
 *   ================
 *   - CAN1 (SPI0) → Charger (EVSE) side — transmits 0x108, 0x109
 *   - CAN2 (SPI1) → Vehicle (EV) side   — transmits 0x100, 0x101, 0x102
 *
 *   The test sends periodic frames from both sides, receives them on the
 *   opposite interface, unpacks the CHAdeMO structures, and prints decoded
 *   values to USB serial every second.
 *
 *   BUILD:
 *   ======
 *   cd chademo_firmware/build
 *   cmake ..
 *   make chademo_can_test -j4
 *   # Flash: cp chademo_can_test.uf2 /media/$USER/RPI-RP2/
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
 * TEST PARAMETERS
 * ============================================================================ */

#define LED_PIN              25
#define TEST_PERIOD_MS       100   /* CHAdeMO spec: 100 ms periodic */
#define DIAG_PERIOD_MS       1000  /* Print decoded values every 1 s */

/* ============================================================================
 * TEST STATE
 * ============================================================================ */

static chademo_tx_vehicle_t g_tx_vehicle = {0};
static chademo_tx_charger_t g_tx_charger = {0};
static chademo_rx_vehicle_t g_rx_vehicle = {0};
static chademo_rx_charger_t g_rx_charger = {0};

static uint32_t g_ch1_tx_ok = 0;   /* CAN1 = charger tx */
static uint32_t g_ch2_tx_ok = 0;   /* CAN2 = vehicle tx */
static uint32_t g_ch1_rx_ok = 0;   /* CAN1 rx (vehicle frames) */
static uint32_t g_ch2_rx_ok = 0;   /* CAN2 rx (charger frames) */

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
    if (eflg & 0x01) printf("[DIAG] %s RX overflow\r\n", name);
}

static void print_rx_vehicle(const char *label, const chademo_rx_vehicle_t *rx)
{
    printf("[RX-V] %s  h100_valid=%d  h101_valid=%d  h102_valid=%d\r\n",
           label, rx->h100_valid, rx->h101_valid, rx->h102_valid);

    if (rx->h100_valid) {
        printf("[RX-V] %s  0x100  MaxBattV=%dV  CapRef=%d%%\r\n",
               label, rx->h100.max_battery_voltage_V,
               rx->h100.charged_rate_ref_const);
    }
    if (rx->h101_valid) {
        printf("[RX-V] %s  0x101  MaxTime10=%d  MaxTime60=%d  TotCap=%d\r\n",
               label, rx->h101.max_charge_time_10s,
               rx->h101.max_charge_time_60s,
               rx->h101.total_capacity_100wh);
    }
    if (rx->h102_valid) {
        printf("[RX-V] %s  0x102  Protocol=%d  TargetV=%dV  ReqA=%dA  "
               "Fault=0x%02X  Status=0x%02X  SOC=%d%%\r\n",
               label, rx->h102.protocol_number,
               rx->h102.target_battery_voltage_V,
               rx->h102.charging_current_request_A,
               rx->h102.fault, rx->h102.status,
               rx->h102.charged_rate_percent);
    }
}

static void print_rx_charger(const char *label, const chademo_rx_charger_t *rx)
{
    printf("[RX-C] %s  h108_valid=%d  h109_valid=%d\r\n",
           label, rx->h108_valid, rx->h109_valid);

    if (rx->h108_valid) {
        printf("[RX-C] %s  0x108  WeldDetect=%d  AvailV=%dV  AvailA=%dA  ThreshV=%dV\r\n",
               label, rx->h108.welding_detection_support ? 1 : 0,
               rx->h108.avail_output_voltage_V,
               rx->h108.avail_output_current_A,
               rx->h108.threshold_voltage_V);
    }
    if (rx->h109_valid) {
        printf("[RX-C] %s  0x109  Protocol=%d  PresV=%dV  PresA=%dA  "
               "Status=0x%02X  Rem10=%d  Rem60=%d\r\n",
               label, rx->h109.protocol_number,
               rx->h109.present_output_voltage_V,
               rx->h109.present_output_current_A,
               rx->h109.status,
               rx->h109.remaining_charge_time_10s,
               rx->h109.remaining_charge_time_60s);
    }
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

    /* Small delay so USB serial enumerates before first printf */
    sleep_ms(2000);

    printf("\r\n");
    printf("╔══════════════════════════════════════════════════════════════╗\r\n");
    printf("║     CHAdeMO CAN Protocol Validation Test                     ║\r\n");
    printf("║     CAN1=Charger(EVSE)  CAN2=Vehicle(EV)                     ║\r\n");
    printf("╚══════════════════════════════════════════════════════════════╝\r\n");
    printf("\r\n");

    /* GPIO to safe defaults (contactor open, etc.) */
    hal_gpio_init();
    printf("[INIT] GPIO safe state set\r\n");

    /* ---- Initialize CAN1 (SPI0) — Charger side ---- */
    printf("[INIT] Initializing CAN1 (SPI0 / MCP2515 #1) as CHARGER...\r\n");
    if (!hal_can_init(HAL_CAN_CH1)) {
        printf("[FATAL] CAN1 MCP2515 not responding!\r\n");
        printf("[FATAL] Check: SPI wiring, CS=GPIO%u, power, crystal\r\n", PIN_CAN1_CS);
        while (1) {
            led_blink(true);  sleep_ms(100);
            led_blink(false); sleep_ms(100);
        }
    }
    printf("[INIT] CAN1 OK — 500 kbps, %d MHz crystal\r\n", MCP2515_OSC_MHZ);

    /* ---- Initialize CAN2 (SPI1) — Vehicle side ---- */
    printf("[INIT] Initializing CAN2 (SPI1 / MCP2515 #2) as VEHICLE...\r\n");
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
    printf("[TEST] Make sure CAN1_H↔CAN2_H and CAN1_L↔CAN2_L are tied together\r\n");
    printf("[TEST] At least one 120-ohm terminator must be on the bus\r\n");
    printf("\r\n");

    /* ---- Initialize RX buffers ---- */
    chademo_can_init_rx_vehicle(&g_rx_vehicle);
    chademo_can_init_rx_charger(&g_rx_charger);

    /* ---- Seed TX structures with example values ---- */
    g_tx_vehicle.h100.minimum_charge_current_A  = 1;     /* 1 A min */
    g_tx_vehicle.h100.minimum_battery_voltage_V = 300;   /* 300 V min */
    g_tx_vehicle.h100.max_battery_voltage_V     = 402;   /* 402 V max */
    g_tx_vehicle.h100.charged_rate_ref_const    = 100;   /* 100 % */

    g_tx_vehicle.h101.max_charge_time_10s       = 30;    /* 300 s */
    g_tx_vehicle.h101.max_charge_time_60s       = 10;    /* 600 s */
    g_tx_vehicle.h101.est_charge_time_60s       = 5;     /* 300 s */
    g_tx_vehicle.h101.total_capacity_100wh      = 400;   /* 40.0 kWh */

    g_tx_vehicle.h102.protocol_number           = CHADEMO_PROTOCOL_V1_0;
    g_tx_vehicle.h102.target_battery_voltage_V  = 355;   /* 355 V target */
    g_tx_vehicle.h102.charging_current_request_A = 100;  /* 100 A */
    g_tx_vehicle.h102.fault                     = 0x00;  /* No faults */
    g_tx_vehicle.h102.status                    = CHADEMO_EV_STATUS_CHARGING_ENABLED;
    g_tx_vehicle.h102.charged_rate_percent      = 45;    /* 45 % SOC */

    g_tx_charger.h108.welding_detection_support = true;
    g_tx_charger.h108.avail_output_voltage_V    = 500;   /* 500 V max */
    g_tx_charger.h108.avail_output_current_A    = 125;   /* 125 A max */
    g_tx_charger.h108.threshold_voltage_V       = 450;   /* 450 V threshold */

    g_tx_charger.h109.protocol_number           = CHADEMO_PROTOCOL_V1_0;
    g_tx_charger.h109.present_output_voltage_V  = 360;   /* 360 V present */
    g_tx_charger.h109.present_output_current_A  = 95;    /* 95 A present */
    g_tx_charger.h109.status                    = CHADEMO_SE_STATUS_CHARGING
                                                   | CHADEMO_SE_STATUS_CONNECTOR_LOCKED;
    g_tx_charger.h109.remaining_charge_time_10s = 0xFF;  /* Invalid */
    g_tx_charger.h109.remaining_charge_time_60s = 12;    /* 12 min */

    /* ---- Main test loop ---- */
    uint32_t last_tx_ms   = 0;
    uint32_t last_diag_ms = 0;
    bool     led_state    = false;

    while (1) {
        uint32_t now_ms = hal_millis();

        /* ---- Periodic TX: both sides send their CHAdeMO frames ---- */
        if ((now_ms - last_tx_ms) >= TEST_PERIOD_MS) {
            last_tx_ms = now_ms;
            led_state  = !led_state;
            led_blink(led_state);

            chademo_can_frame_t frame;

            /* --- CAN2 (vehicle) transmits 0x100, 0x101, 0x102 --- */
            chademo_can_frame_t vframes[3];
            chademo_can_pack_vehicle_frames(&g_tx_vehicle, vframes);
            for (int i = 0; i < 3; i++) {
                if (hal_can_send(HAL_CAN_CH2, &vframes[i])) {
                    g_ch2_tx_ok++;
                }
                sleep_us(500);
                /* Drain RX between frames to avoid MCP2515 buffer overflow */
                while (hal_can_recv(HAL_CAN_CH1, &frame)) {
                    if (chademo_can_unpack_vehicle_frame(&g_rx_vehicle, &frame)) {
                        g_rx_vehicle.last_rx_ms = now_ms;
                        g_ch1_rx_ok++;
                    }
                }
                while (hal_can_recv(HAL_CAN_CH2, &frame)) {
                    if (chademo_can_unpack_charger_frame(&g_rx_charger, &frame)) {
                        g_rx_charger.last_rx_ms = now_ms;
                        g_ch2_rx_ok++;
                    }
                }
            }

            /* --- CAN1 (charger) transmits 0x108, 0x109 --- */
            chademo_can_frame_t cframes[2];
            chademo_can_pack_charger_frames(&g_tx_charger, cframes);
            for (int i = 0; i < 2; i++) {
                if (hal_can_send(HAL_CAN_CH1, &cframes[i])) {
                    g_ch1_tx_ok++;
                }
                sleep_us(500);
                while (hal_can_recv(HAL_CAN_CH1, &frame)) {
                    if (chademo_can_unpack_vehicle_frame(&g_rx_vehicle, &frame)) {
                        g_rx_vehicle.last_rx_ms = now_ms;
                        g_ch1_rx_ok++;
                    }
                }
                while (hal_can_recv(HAL_CAN_CH2, &frame)) {
                    if (chademo_can_unpack_charger_frame(&g_rx_charger, &frame)) {
                        g_rx_charger.last_rx_ms = now_ms;
                        g_ch2_rx_ok++;
                    }
                }
            }

            /* --- Slowly vary some values so the output is obviously live --- */
            g_tx_vehicle.h102.charged_rate_percent++;
            if (g_tx_vehicle.h102.charged_rate_percent > 100) {
                g_tx_vehicle.h102.charged_rate_percent = 0;
            }

            g_tx_charger.h109.present_output_current_A++;
            if (g_tx_charger.h109.present_output_current_A > 125) {
                g_tx_charger.h109.present_output_current_A = 0;
            }
        }

        /* ---- Final RX drain for any late-arriving frames ---- */
        chademo_can_frame_t frame;
        while (hal_can_recv(HAL_CAN_CH1, &frame)) {
            if (chademo_can_unpack_vehicle_frame(&g_rx_vehicle, &frame)) {
                g_rx_vehicle.last_rx_ms = now_ms;
                g_ch1_rx_ok++;
            }
        }
        while (hal_can_recv(HAL_CAN_CH2, &frame)) {
            if (chademo_can_unpack_charger_frame(&g_rx_charger, &frame)) {
                g_rx_charger.last_rx_ms = now_ms;
                g_ch2_rx_ok++;
            }
        }

        /* ---- Periodic diagnostics ---- */
        if ((now_ms - last_diag_ms) >= DIAG_PERIOD_MS) {
            last_diag_ms = now_ms;

            printf("\r\n");
            printf("[TEST] ======== CHAdeMO CAN Test @ %lu ms ========\r\n", now_ms);
            printf("[TEST] CAN1(Chgr) TX=%lu  CAN2(Veh) RX=%lu  link=%s\r\n",
                   g_ch1_tx_ok, g_ch2_rx_ok,
                   chademo_can_charger_link_ok(&g_rx_charger) ? "OK" : "WAITING");
            printf("[TEST] CAN2(Veh)  TX=%lu  CAN1(Chgr) RX=%lu  link=%s\r\n",
                   g_ch2_tx_ok, g_ch1_rx_ok,
                   chademo_can_vehicle_link_ok(&g_rx_vehicle) ? "OK" : "WAITING");

            print_rx_vehicle("CAN1←CAN2", &g_rx_vehicle);
            print_rx_charger("CAN2←CAN1", &g_rx_charger);
            print_mcp2515_diag(HAL_CAN_CH1, "CAN1");
            print_mcp2515_diag(HAL_CAN_CH2, "CAN2");
        }

        /* Yield for USB CDC flush */
        sleep_ms(5);
    }

    return 0;
}

/* ============================================================================
 * SDK BINARY INFO
 * ============================================================================ */
bi_decl(bi_program_name("CHAdeMO CAN Protocol Test"));
bi_decl(bi_program_version_string("1.0.0"));
bi_decl(bi_program_description("Dual MCP2515 CHAdeMO CAN validation test"));
