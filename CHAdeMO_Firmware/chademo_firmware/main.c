/**
 * @file  main.c
 * @brief CHAdeMO Controller — Main Application Loop
 * @details
 *   Entry point for the RP2040 CHAdeMO firmware. Initializes all
 *   subsystems, then runs the main control loop at 100Hz (10ms tick).
 *
 *   Architecture:
 *   =============
 *   +-------------------+     +-------------------+     +------------------+
 *   |   HAL Layer       |<--->|   CAN Driver      |<--->|  MCP2515 (x2)    |
 *   | (chademo_hal.c)   |     | (chademo_can.c)   |     |  SPI0 + SPI1     |
 *   +-------------------+     +-------------------+     +------------------+
 *            ^                         ^
 *            |                         |
 *            v                         v
 *   +-------------------+     +-------------------+
 *   |   FSM Core        |<--->|   GPIO Control    |
 *   | (chademo_fsm.c)   |     | (SS1/SS2/DCP/PP)  |
 *   +-------------------+     +-------------------+
 *            ^
 *            |
 *   +-------------------+
 *   |   main.c (this)   |
 *   |  100Hz main loop  |
 *   +-------------------+
 *
 *   Compile-time role selection:
 *     #define CHADEMO_ROLE_VEHICLE   → Build for EV
 *     #define CHADEMO_ROLE_STATION   → Build for EVSE
 *
 *   Based on open-source references:
 *   - furdog/chademo (generic CHAdeMO library)
 *   - jamiejones85/ESP32-Chademo (EV conversion firmware)
 *   - IEEE Std 2030.1.1-2015
 */

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "hardware/timer.h"

#include "chademo_config.h"
#include "chademo_hal.h"
#include "chademo_can.h"
#include "chademo_fsm.h"
#if IS_STATION
#include "chademo_infypower.h"
#endif

#include <stdio.h>
#include <string.h>

/* ============================================================================
 * BUILD CONFIGURATION VALIDATION
 * ============================================================================ */

#ifndef CHADEMO_ROLE_VEHICLE
#ifndef CHADEMO_ROLE_STATION
#error "Must define CHADEMO_ROLE_VEHICLE or CHADEMO_ROLE_STATION"
#endif
#endif

/* ============================================================================
 * DEBUG UART
 * ============================================================================
 * Use UART0 (GPIO 0/1) for debug output at 115200 baud.
 * This is separate from the SPI buses used for CAN.
 */
#define DEBUG_UART        uart0
#define DEBUG_UART_TX_PIN 0
#define DEBUG_UART_RX_PIN 1
#define DEBUG_BAUDRATE    115200

/* ============================================================================
 * MAIN LOOP TIMING
 * ============================================================================
 * The CHAdeMO spec requires 100ms periodic CAN transmission.
 * We run the main loop at 100Hz (10ms) to ensure timely processing
 * while allowing 10 iterations per CAN transmission period.
 */
#define MAIN_LOOP_HZ      100
#define MAIN_LOOP_PERIOD_MS (1000 / MAIN_LOOP_HZ)

/* CAN transmission period: send one frame every 33ms to achieve
 * ~100ms cycle for all 3 vehicle frames (or 50ms for 2 charger frames) */
#define CAN_TX_PERIOD_MS  33

/* CHAdeMO uses CAN2 (SPI1, GPIO10-14). CAN1 is for internal/BMS. */
#define CHADEMO_CAN_CHANNEL HAL_CAN_CH2
#define INTERNAL_CAN_CHANNEL HAL_CAN_CH1

/* LED for heartbeat (Pico onboard LED is GPIO25) */
#define LED_PIN           25

/* ============================================================================
 * GLOBAL STATE
 * ============================================================================ */

static chademo_context_t g_fsm_ctx;
static uint32_t g_loop_count = 0;
static uint32_t g_can_tx_count = 0;
static uint32_t g_can_rx_count = 0;
static uint32_t g_last_debug_print_ms = 0;

/* ============================================================================
 * FORWARD DECLARATIONS
 * ============================================================================ */

static void debug_init(void);
static void debug_print_startup(void);
static void debug_print_status(void);
static void led_heartbeat(void);
static void process_can_rx(void);
static void process_can_tx(void);
static void process_application_logic(void);
static void log_pin_changes(void);

/* ============================================================================
 * ENTRY POINT
 * ============================================================================ */

int main(void)
{
    /* ---- Initialize onboard LED FIRST for early diagnostics ---- */
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    /* Obvious blink pattern: program is alive (10 blinks = 2 seconds) */
    for (int i = 0; i < 10; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }

    /* ---- Initialize Pico SDK ---- */
    stdio_init_all();
    sleep_ms(500);  /* Let USB CDC enumerate before spamming output */

    /* ---- Initialize debug UART ---- */
    debug_init();

    printf("=== CHADeMO %s BOOT ===\r\n", FSM_ROLE_STR);
    fflush(stdout);

    /* ---- Startup banner ---- */
    debug_print_startup();
    fflush(stdout);

    /* ---- CRITICAL: Initialize GPIO to safe states BEFORE anything else ----
     * This ensures contactor is OPEN and all control lines are de-asserted
     * even if CAN initialization fails.
     */
    hal_gpio_init();
    printf("[%s-HAL] GPIO initialized — contactor OPEN, controls SAFE\r\n", FSM_ROLE_STR);
    fflush(stdout);

    /* ---- Initialize CAN buses ---- */
    printf("[%s-CAN] Initializing CHAdeMO bus (CAN2 / SPI1)...\r\n", FSM_ROLE_STR);
    fflush(stdout);
    if (!hal_can_init(CHADEMO_CAN_CHANNEL)) {
        printf("[%s-CAN] FATAL: MCP2515 #2 (CHAdeMO bus, CAN2/SPI1, CS=GPIO%d) NOT RESPONDING!\r\n",
               FSM_ROLE_STR, PIN_CAN2_CS);
        printf("[%s-CAN] Check: 5V/GND to MCP2515 module, SPI1 wiring (SCK=GPIO10, MOSI=GPIO11, MISO=GPIO12), CS=GPIO14\r\n", FSM_ROLE_STR);
        printf("[%s-CAN] Also verify MCP2515 oscillator is %d MHz (config in chademo_config.h)\r\n",
               FSM_ROLE_STR, MCP2515_OSC_MHZ);
        fflush(stdout);
        /* Fast blink invisible LED and print so we don't go silent */
        while (1) {
            gpio_put(LED_PIN, 1);
            sleep_ms(100);
            gpio_put(LED_PIN, 0);
            sleep_ms(100);
            printf("[%s-CAN] ERROR: MCP2515 #2 (CAN2/SPI1, CS=GPIO%d) not responding — check wiring/power\r\n",
                   FSM_ROLE_STR, PIN_CAN2_CS);
            fflush(stdout);
            hal_watchdog_feed();
        }
    }
    printf("[%s-CAN] CHAdeMO CAN initialized OK (500 kbps, %d MHz crystal)\r\n", FSM_ROLE_STR, MCP2515_OSC_MHZ);
    fflush(stdout);

    /* CAN1: InfyPower DC module bus (station only) or internal BMS (vehicle) */
#if IS_STATION
    printf("[%s-INFY] Initializing InfyPower DC module bus (CAN1 / SPI0 / 125kbps)...\r\n", FSM_ROLE_STR);
    fflush(stdout);
    if (!infypower_init()) {
        printf("[%s-INFY] WARN: InfyPower MCP2515 (CAN1/SPI0, CS=GPIO%d) NOT RESPONDING!\r\n",
               FSM_ROLE_STR, PIN_CAN1_CS);
        printf("[%s-INFY] Check: 5V/GND to MCP2515, SPI0 wiring, oscillator crystal\r\n", FSM_ROLE_STR);
    } else {
        printf("[%s-INFY] InfyPower CAN initialized OK (125 kbps)\r\n", FSM_ROLE_STR);
    }
    fflush(stdout);
#else
    printf("[%s-CAN] Initializing internal bus (CAN1 / SPI0)...\r\n", FSM_ROLE_STR);
    fflush(stdout);
    if (!hal_can_init(INTERNAL_CAN_CHANNEL)) {
        printf("[%s-CAN] WARN: MCP2515 #1 not responding — continuing without it\r\n", FSM_ROLE_STR);
    } else {
        printf("[%s-CAN] MCP2515 #1 initialized OK\r\n", FSM_ROLE_STR);
    }
    fflush(stdout);
#endif

    /* ---- Initialize CHAdeMO state machine ---- */
    chademo_fsm_init(&g_fsm_ctx);
    printf("[%s-FSM] State machine initialized\r\n", FSM_ROLE_STR);
    fflush(stdout);

    /* ---- Set initial battery parameters (VEHICLE role) ----
     * In a real application, these come from the BMS via CAN.
     * These are example values for a 96S Li-ion pack (~355V nominal).
     */
#if IS_VEHICLE
    /* Bench-test values for 450V-class pack to match charger threshold (402V min) */
    chademo_fsm_set_min_voltage(&g_fsm_ctx, 40);     /* 40V min for 50V target */
    chademo_fsm_set_max_voltage(&g_fsm_ctx, 500);    /* 500V max */
    chademo_fsm_set_target_voltage(&g_fsm_ctx, 50);  /* 50V target test */
    chademo_fsm_set_target_current(&g_fsm_ctx, 2);   /* 2A initial request (charger offers 2A) */
    chademo_fsm_set_capacity_kwh(&g_fsm_ctx, 400);   /* 40.0 kWh pack */
    chademo_fsm_set_battery_soc(&g_fsm_ctx, 20);     /* Starting at 20% SOC */
    printf("[%s-BAT] Pack config: 350V min, 450V nom, 500V max, 40kWh (bench test)\r\n", FSM_ROLE_STR);
#else
    /* STATION role: Set available output capability — lie big for bench test */
    chademo_fsm_set_target_voltage(&g_fsm_ctx, CHADEMO_MAX_VOLTAGE_V);
    chademo_fsm_set_target_current(&g_fsm_ctx, CHADEMO_MAX_CURRENT_A);
    printf("[%s-EVSE] Output capability: %dV, %dA (bench test — claims max)\r\n",
           FSM_ROLE_STR, CHADEMO_MAX_VOLTAGE_V, CHADEMO_MAX_CURRENT_A);
#endif

    printf("\r\n");
    printf("============================================\r\n");
    printf("  CHAdeMO Controller Ready\r\n");
    printf("  Role: %s\r\n", FSM_ROLE_STR);
    printf("  Version: %d.%d.%d\r\n",
           FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH);
    printf("============================================\r\n\r\n");
    fflush(stdout);

    /* ---- Initialize watchdog ----
     * We do this AFTER all init is complete so a slow/hung init doesn't
     * trigger a spurious reset.  1000 ms gives plenty of headroom.
     */
    hal_watchdog_init(1000);
    printf("[%s-WDT] Watchdog armed (1000 ms timeout)\r\n", FSM_ROLE_STR);

    /* ---- Main control loop (100 Hz) ---- */
    absolute_time_t next_loop_time = get_absolute_time();

    while (1) {
        /* Calculate delta time for FSM */
        uint32_t dt_ms = MAIN_LOOP_PERIOD_MS;

        /* ---- 1. Process incoming CAN frames ---- */
        process_can_rx();

        /* ---- 2. Run the CHAdeMO state machine ---- */
        chademo_event_t event = chademo_fsm_step(&g_fsm_ctx, dt_ms);

        /* ---- 3. Handle significant FSM events ---- */
        switch (event) {
        case CHADEMO_EVENT_PLUG_INSERTED:
            printf("[%s-EVT] Plug inserted — starting CHAdeMO sequence\r\n", FSM_ROLE_STR);
#if IS_STATION
            /* Reset CAN controller to clear any accumulated bus errors
             * from idle-state noise or previous failed handshakes */
            hal_can_reset(CHADEMO_CAN_CHANNEL);
            if (!hal_can_init(CHADEMO_CAN_CHANNEL)) {
                printf("[%s-CAN] FATAL: MCP2515 #2 reset failed after plug-in\r\n", FSM_ROLE_STR);
            } else {
                printf("[%s-CAN] MCP2515 #2 reset OK\r\n", FSM_ROLE_STR);
            }
#endif
            break;

        case CHADEMO_EVENT_HANDSHAKE_COMPLETE:
            printf("[%s-EVT] CAN handshake complete\r\n", FSM_ROLE_STR);
            break;

        case CHADEMO_EVENT_PARAMS_COMPATIBLE:
            printf("[%s-EVT] Parameters compatible — proceeding\r\n", FSM_ROLE_STR);
            break;

        case CHADEMO_EVENT_INSULATION_OK:
            printf("[%s-EVT] Insulation test passed\r\n", FSM_ROLE_STR);
            break;

        case CHADEMO_EVENT_PRECHARGE_OK:
            printf("[%s-EVT] Pre-charge complete — contactors closing\r\n", FSM_ROLE_STR);
            break;

        case CHADEMO_EVENT_CHARGING_STARTED:
            printf("[%s-EVT] CHARGING ACTIVE\r\n", FSM_ROLE_STR);
            break;

        case CHADEMO_EVENT_CHARGE_COMPLETE:
            printf("[%s-EVT] Charge complete — safe to unplug\r\n", FSM_ROLE_STR);
            break;

        case CHADEMO_EVENT_FAULT: {
            const char *fault_name = chademo_fsm_fault_name(g_fsm_ctx.fault_reason);
            printf("[%s-EVT] FAULT: %s\r\n", FSM_ROLE_STR, fault_name);
            break;
        }

        case CHADEMO_EVENT_UNPLUGGED:
            printf("[%s-EVT] Plug removed — returning to idle\r\n", FSM_ROLE_STR);
            break;

        default:
            break;
        }

        /* ---- 4. Process outgoing CAN frames ---- */
        process_can_tx();

        /* ---- 5. Application-specific logic ----
         * Here you would read ADCs for voltage/current, poll BMS,
         * update display, log data, etc.
         */
        process_application_logic();

        /* ---- 5b. Log control-pin changes for bench debugging ---- */
        log_pin_changes();

        /* ---- 6. Feed watchdog ---- */
        hal_watchdog_feed();

        /* ---- 7. LED heartbeat ---- */
        led_heartbeat();

        /* ---- 8. Periodic debug output (every 1 second) ---- */
        if (g_fsm_ctx.state_entry_ms - g_last_debug_print_ms >= 1000) {
            g_last_debug_print_ms = g_fsm_ctx.state_entry_ms;
            debug_print_status();
        }

        /* ---- 9. Maintain fixed loop rate ---- */
        g_loop_count++;
        next_loop_time = delayed_by_ms(next_loop_time, MAIN_LOOP_PERIOD_MS);
        sleep_until(next_loop_time);
    }

    return 0;  /* Never reached */
}

/* ============================================================================
 * CAN RX PROCESSING
 * ============================================================================
 * Poll both CAN channels for received frames and feed them to the FSM.
 * In a higher-performance implementation, this would be interrupt-driven
 * using the MCP2515 INT pin. For clarity, we use polling here.
 */

static void process_can_rx(void)
{
    chademo_can_frame_t frame;

    /* Poll CHAdeMO bus (CAN2) */
    while (hal_can_recv(CHADEMO_CAN_CHANNEL, &frame)) {
        g_can_rx_count++;

        /* Diagnostic: print every received frame so we can see if the bus
         * is alive but with unexpected IDs */
        printf("[%s-CAN] RX id=0x%03lX len=%u data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               FSM_ROLE_STR, (unsigned long)frame.id, frame.len,
               frame.data[0], frame.data[1], frame.data[2], frame.data[3],
               frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

        /* Push into FSM for protocol processing */
        chademo_fsm_push_can_rx(&g_fsm_ctx, &frame);

        /* Optional: bridge to internal CAN1 for logging */
        /* hal_can_send(INTERNAL_CAN_CHANNEL, &frame); */
    }

    /* Poll internal bus (CAN1) — BMS, display, or logging data */
    while (hal_can_recv(INTERNAL_CAN_CHANNEL, &frame)) {
        /* Internal bus frames */
    }
}

/* ============================================================================
 * CAN TX PROCESSING
 * ============================================================================
 * Pull frames from the FSM and transmit them at the required 100ms rate.
 * Vehicle sends 3 frames (0x100, 0x101, 0x102) — one per 33ms = ~100ms cycle.
 * Charger sends 2 frames (0x108, 0x109) — one per 50ms = ~100ms cycle.
 */

static void process_can_tx(void)
{
    static uint32_t last_tx_time = 0;
    static uint32_t last_tx_err_print = 0;

    uint32_t now = hal_millis();
    if ((now - last_tx_time) < CAN_TX_PERIOD_MS) {
        return;  /* Not time to send yet */
    }
    last_tx_time = now;

    /* Station only sends when a vehicle is detected (PLUG_DETECTED onwards).
     * Vehicle only sends when leaving IDLE. */
#if IS_VEHICLE
    if (g_fsm_ctx.state == CHADEMO_STATE_IDLE ||
        g_fsm_ctx.state == CHADEMO_STATE_STOPPED) {
        return;
    }
#else
    if (g_fsm_ctx.state == CHADEMO_STATE_IDLE ||
        g_fsm_ctx.state == CHADEMO_STATE_STOPPED) {
        return;
    }
#endif

    chademo_can_frame_t frame;
    if (chademo_fsm_pull_can_tx(&g_fsm_ctx, &frame)) {
        if (hal_can_send(CHADEMO_CAN_CHANNEL, &frame)) {
            g_can_tx_count++;
        } else {
            /* Rate-limit the TX blocked messages to avoid console spam */
            if ((now - last_tx_err_print) >= 500) {
                last_tx_err_print = now;
                uint8_t txb0 = hal_can_read_reg(CHADEMO_CAN_CHANNEL, MCP2515_REG_TXB0CTRL);
                printf("[%s-CAN] TX blocked: TXB0CTRL=0x%02X (TXREQ=%d)\r\n",
                       FSM_ROLE_STR, txb0, (txb0 >> 3) & 1);
            }
        }
    }
}

/* ============================================================================
 * APPLICATION LOGIC
 * ============================================================================
 * This is where you'd integrate:
 *   - ADC readings for voltage/current measurement
 *   - BMS CAN communication
 *   - Temperature sensor polling
 *   - Display updates
 *   - Data logging to SD card
 *
 * The example below shows placeholder logic that simulates a charging
 * profile. Replace with your actual hardware interfaces.
 */

static void process_application_logic(void)
{
    /* ---- Update measured values (placeholder -- replace with ADC reads) ----
     * In a real implementation:
     *   ctx->measured_voltage_V = adc_read_voltage();
     *   ctx->measured_current_A = adc_read_current();  // negative = charging
     *   ctx->battery_temp_C = bms_get_max_cell_temp();
     *   ctx->battery_soc = bms_get_soc();
     */

    /* Re-apply battery/capability params every loop.
     * chademo_fsm_init() zeros these on unplug/timeout, so we must
     * refresh them to survive FSM resets without a full reboot. */
#if IS_VEHICLE
    chademo_fsm_set_min_voltage(&g_fsm_ctx, 40);
    chademo_fsm_set_max_voltage(&g_fsm_ctx, 500);
    chademo_fsm_set_target_voltage(&g_fsm_ctx, 50);
    chademo_fsm_set_target_current(&g_fsm_ctx, 2);
    chademo_fsm_set_capacity_kwh(&g_fsm_ctx, 400);
    chademo_fsm_set_battery_soc(&g_fsm_ctx, 20);
    /* Report measured battery voltage back to charger via 0x100 bytes 2-3 */
    if (g_fsm_ctx.state >= CHADEMO_STATE_PRECHARGE &&
        g_fsm_ctx.state <= CHADEMO_STATE_CHARGING) {
        g_fsm_ctx.tx.h100.minimum_battery_voltage_V = g_fsm_ctx.measured_voltage_V;
    }
#else
    chademo_fsm_set_target_voltage(&g_fsm_ctx, CHADEMO_MAX_VOLTAGE_V);
    chademo_fsm_set_target_current(&g_fsm_ctx, CHADEMO_MAX_CURRENT_A);
#endif

    /* Example: simulated charge profile for testing without hardware */
    static uint16_t sim_voltage = 50;   /* Vehicle starts at batt V; station resets below */
    static int16_t  sim_current = 0;    /* Starting at 0A */

#if IS_STATION
    /* Station: non-blocking InfyPower DC module control */
    static uint8_t  infy_phase = 0;   /* 0=off, 1=set_sent, 2=on_sent, 3=running */
    static uint32_t infy_phase_ms = 0;
    static uint16_t infy_target_v = 0;
    static uint8_t  infy_target_a = 0;

    /* Determine target voltage/current based on state */
    uint16_t target_v = 0;
    uint8_t  target_a = 0;

    if (g_fsm_ctx.state == CHADEMO_STATE_INSULATION_TEST) {
        /* Apply 150V during insulation test */
        target_v = 150;
        target_a = 0;
    } else if (g_fsm_ctx.state >= CHADEMO_STATE_PRECHARGE &&
               g_fsm_ctx.state <= CHADEMO_STATE_CHARGING) {
        target_v = g_fsm_ctx.rx.h102_valid
            ? g_fsm_ctx.rx.h102.target_battery_voltage_V : 50;
        target_a = g_fsm_ctx.rx.h102_valid
            ? g_fsm_ctx.rx.h102.charging_current_request_A : 0;
    }

    /* Clamp to InfyPower limits */
    if (target_v > INFYPOWER_VOLTAGE_MAX_V) target_v = INFYPOWER_VOLTAGE_MAX_V;
    if (target_a > INFYPOWER_CURRENT_MAX_A) target_a = INFYPOWER_CURRENT_MAX_A;

    /* Non-blocking startup / shutdown state machine */
    if (target_v > 0) {
        if (infy_phase == 0) {
            infypower_cmd_set_output(target_v, target_a);
            infy_phase = 1;
            infy_phase_ms = hal_millis();
            printf("[STATION-INFY] SET OUTPUT: %uV/%uA\r\n", target_v, target_a);
        } else if (infy_phase == 1) {
            if ((hal_millis() - infy_phase_ms) >= INFYPOWER_STARTUP_DELAY_MS) {
                infypower_cmd_onoff(true);
                infy_phase = 2;
                infy_phase_ms = hal_millis();
                printf("[STATION-INFY] MODULE ON\r\n");
            }
        } else if (infy_phase == 2) {
            if ((hal_millis() - infy_phase_ms) >= INFYPOWER_STARTUP_DELAY_MS) {
                infy_phase = 3;
                infy_target_v = target_v;
                infy_target_a = target_a;
                printf("[STATION-INFY] RUNNING\r\n");
            }
        } else if (infy_phase == 3) {
            if (target_v != infy_target_v || target_a != infy_target_a) {
                infypower_cmd_set_output(target_v, target_a);
                infy_target_v = target_v;
                infy_target_a = target_a;
            }
            infypower_heartbeat(target_v, target_a);
        }
    } else {
        if (infy_phase > 0) {
            infypower_cmd_onoff(false);
            infy_phase = 0;
            sim_voltage = 0;
            sim_current = 0;
            printf("[STATION-INFY] MODULE OFF\r\n");
        }
    }

    /* Poll actual output and update measured values */
    uint16_t actual_v = 0;
    uint8_t  actual_a = 0;
    if (infypower_poll_rx(&actual_v, &actual_a)) {
        sim_voltage = actual_v;
        sim_current = (int16_t)actual_a;
    } else if (infy_phase >= 2) {
        if (sim_voltage < target_v) { sim_voltage += 5; if (sim_voltage > target_v) sim_voltage = target_v; }
        if (sim_voltage > target_v) { sim_voltage -= 5; if (sim_voltage < target_v) sim_voltage = target_v; }
    } else {
        sim_voltage = 0;
        sim_current = 0;
    }
#else
    /* Vehicle: voltage tracks target during precharge/charging */
    if (g_fsm_ctx.state == CHADEMO_STATE_PRECHARGE ||
        g_fsm_ctx.state == CHADEMO_STATE_CHARGING) {
        uint16_t target_v = g_fsm_ctx.tx.h102.target_battery_voltage_V;
        if (sim_voltage < target_v) { sim_voltage += 5; if (sim_voltage > target_v) sim_voltage = target_v; }
        if (sim_voltage > target_v) { sim_voltage -= 5; if (sim_voltage < target_v) sim_voltage = target_v; }
    }
#endif

    if (g_fsm_ctx.state == CHADEMO_STATE_CHARGING) {
        /* Simulate current following the request */
#if IS_VEHICLE
        uint8_t target_a = g_fsm_ctx.asking_amps;
        if (sim_current > -target_a) {
            sim_current -= 2;  /* Ramping into battery (negative) */
        }
        if (sim_current < -target_a) {
            sim_current = -target_a;
        }
#else
        /* EVSE outputs positive current matching EV request */
        uint8_t target_a = g_fsm_ctx.rx.h102_valid
            ? g_fsm_ctx.rx.h102.charging_current_request_A
            : 0;
        if (sim_current < target_a) {
            sim_current++;
        }
        if (sim_current > target_a) {
            sim_current--;
        }
#endif
    } else {
        /* Not charging — current returns to zero */
        if (sim_current < 0) sim_current++;
        if (sim_current > 0) sim_current--;
    }

    if (g_fsm_ctx.state == CHADEMO_STATE_SHUTDOWN ||
        g_fsm_ctx.state == CHADEMO_STATE_WAIT_ZERO_CURRENT) {
        sim_current = 0;
    }

    /* Feed simulated values into FSM */
    chademo_fsm_set_measured_voltage(&g_fsm_ctx, sim_voltage);
    chademo_fsm_set_measured_current(&g_fsm_ctx, sim_current);

#if IS_VEHICLE
    /* ---- Auto-shutdown when SOC reaches 80% (example) ---- */
    if (g_fsm_ctx.state == CHADEMO_STATE_CHARGING && g_fsm_ctx.battery_soc >= 80) {
        printf("[%s-APP] Target SOC reached (80%%) — requesting shutdown\r\n", FSM_ROLE_STR);
        chademo_fsm_request_shutdown(&g_fsm_ctx);
    }
#endif
}

/* ============================================================================
 * PIN CHANGE LOGGING
 * ============================================================================
 * Logs whenever CHAdeMO control pins change state.
 * This is for bench debugging / wiring verification.
 * SS1/SS2 are logged for visibility but NOT used in FSM decisions.
 * ============================================================================ */

static void log_pin_changes(void)
{
#if IS_STATION
    static bool last_dcp       = false;
    static bool last_ss1       = false;
    static bool last_ss2       = false;
    static bool last_contactor = false;

    bool dcp       = hal_gpio_read_dcp();
    bool ss1       = (gpio_get(PIN_OUT_SS1) != 0);
    bool ss2       = (gpio_get(PIN_OUT_SS2) != 0);
    bool contactor = hal_gpio_get_contactor();

    if (dcp != last_dcp) {
        last_dcp = dcp;
        printf("[%s-PIN] DCP (GPIO%u) = %s\r\n",
               FSM_ROLE_STR, PIN_IN_DCP, dcp ? "HIGH (no vehicle)" : "LOW  (vehicle present)");
    }
    if (ss1 != last_ss1) {
        last_ss1 = ss1;
        printf("[%s-PIN] SS1 (GPIO%u) = %s\r\n",
               FSM_ROLE_STR, PIN_OUT_SS1, ss1 ? "HIGH" : "LOW");
    }
    if (ss2 != last_ss2) {
        last_ss2 = ss2;
        printf("[%s-PIN] SS2 (GPIO%u) = %s\r\n",
               FSM_ROLE_STR, PIN_OUT_SS2, ss2 ? "HIGH" : "LOW");
    }
    if (contactor != last_contactor) {
        last_contactor = contactor;
        printf("[%s-PIN] CONTACTOR (GPIO%u) = %s\r\n",
               FSM_ROLE_STR, PIN_CONTACTOR_OUT, contactor ? "CLOSED" : "OPEN");
    }
#else
    static bool last_pp        = false;
    static bool last_ss1       = false;
    static bool last_ss2       = false;
    static bool last_dcp       = false;
    static bool last_contactor = false;

    bool pp        = hal_gpio_read_pp();
    bool ss1       = hal_gpio_read_ss1();
    bool ss2       = hal_gpio_read_ss2();
    bool dcp       = (gpio_get(PIN_OUT_DCP) != 0);
    bool contactor = hal_gpio_get_contactor();

    if (pp != last_pp) {
        last_pp = pp;
        printf("[%s-PIN] PP (GPIO%u) = %s\r\n",
               FSM_ROLE_STR, PIN_IN_PP, pp ? "HIGH (no plug)" : "LOW  (plug inserted)");
    }
    if (ss1 != last_ss1) {
        last_ss1 = ss1;
        printf("[%s-PIN] SS1 (GPIO%u) = %s\r\n",
               FSM_ROLE_STR, PIN_IN_SS1, ss1 ? "HIGH (charger idle)" : "LOW  (charger active)");
    }
    if (ss2 != last_ss2) {
        last_ss2 = ss2;
        printf("[%s-PIN] SS2 (GPIO%u) = %s\r\n",
               FSM_ROLE_STR, PIN_IN_SS2, ss2 ? "HIGH (not charging)" : "LOW  (charge locked)");
    }
    if (dcp != last_dcp) {
        last_dcp = dcp;
        printf("[%s-PIN] DCP (GPIO%u) = %s (%s)\r\n",
               FSM_ROLE_STR, PIN_OUT_DCP,
               dcp ? "HIGH" : "LOW ",
               dcp ? "de-asserted (inverted)" : "asserted (inverted)");
    }
    if (contactor != last_contactor) {
        last_contactor = contactor;
        printf("[%s-PIN] CONTACTOR (GPIO%u) = %s\r\n",
               FSM_ROLE_STR, PIN_CONTACTOR_OUT, contactor ? "CLOSED" : "OPEN");
    }
#endif
}

/* ============================================================================
 * DEBUG / DIAGNOSTICS
 * ============================================================================ */

static void debug_init(void)
{
    uart_init(DEBUG_UART, DEBUG_BAUDRATE);
    gpio_set_function(DEBUG_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DEBUG_UART_RX_PIN, GPIO_FUNC_UART);
}

static void debug_print_startup(void)
{
    printf("\r\n");
    printf("╔══════════════════════════════════════════════════════════════╗\r\n");
    printf("║      CHAdeMO DC Fast Charge Controller — RP2040              ║\r\n");
    printf("║      Role: %-10s                                        ║\r\n", FSM_ROLE_STR);
    printf("║      Version: %d.%d.%d                                         ║\r\n",
           FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH);
    printf("╚══════════════════════════════════════════════════════════════╝\r\n");
    printf("\r\n");
}

static void debug_print_mcp2515_diag(void)
{
    uint8_t eflg   = hal_can_read_reg(CHADEMO_CAN_CHANNEL, MCP2515_REG_EFLG);
    uint8_t tec    = hal_can_read_reg(CHADEMO_CAN_CHANNEL, 0x1C);
    uint8_t rec    = hal_can_read_reg(CHADEMO_CAN_CHANNEL, 0x1D);
    uint8_t intf   = hal_can_read_reg(CHADEMO_CAN_CHANNEL, MCP2515_REG_CANINTF);
    uint8_t txb0   = hal_can_read_reg(CHADEMO_CAN_CHANNEL, MCP2515_REG_TXB0CTRL);
    uint8_t stat   = hal_can_read_reg(CHADEMO_CAN_CHANNEL, MCP2515_REG_CANSTAT);
    uint8_t rxb0c  = hal_can_read_reg(CHADEMO_CAN_CHANNEL, MCP2515_REG_RXB0CTRL);
    uint8_t rxb1c  = hal_can_read_reg(CHADEMO_CAN_CHANNEL, MCP2515_REG_RXB1CTRL);

    printf("[%s-DIAG] EFLG=0x%02X TEC=%u REC=%u INTF=0x%02X TXB0=0x%02X STAT=0x%02X RXB0=0x%02X RXB1=0x%02X\r\n",
           FSM_ROLE_STR, eflg, tec, rec, intf, txb0, stat, rxb0c, rxb1c);
    if (eflg & 0x20) printf("[%s-DIAG] >>> BUS OFF <<<\r\n", FSM_ROLE_STR);
    if (eflg & 0x10) printf("[%s-DIAG] TX error passive\r\n", FSM_ROLE_STR);
    if (eflg & 0x08) printf("[%s-DIAG] RX error passive\r\n", FSM_ROLE_STR);
    if (eflg & 0x04) printf("[%s-DIAG] TX error warning\r\n", FSM_ROLE_STR);
    if (txb0 & 0x08) printf("[%s-DIAG] TXB0 pending (stuck)\r\n", FSM_ROLE_STR);
}

static void debug_print_status(void)
{
    const char *state_name = chademo_fsm_state_name(g_fsm_ctx.state);

    printf("[%s-STAT] State=%s | V=%dV | I=%dA | SOC=%d%% | CAN rx/tx=%lu/%lu | Loop=%lu\r\n",
           FSM_ROLE_STR, state_name,
           g_fsm_ctx.measured_voltage_V,
           g_fsm_ctx.measured_current_A,
           g_fsm_ctx.battery_soc,
           g_can_rx_count,
           g_can_tx_count,
           g_loop_count);

    /* Print MCP2515 diagnostics every ~1 second */
    debug_print_mcp2515_diag();

#if IS_VEHICLE
    /* Show charger-reported values */
    if (g_fsm_ctx.rx.h109_valid) {
        printf("[%s-CHRG] Avail=%dV/%dA | Present=%dV/%dA | Status=0x%02X | Time=%ds\r\n",
               FSM_ROLE_STR,
               g_fsm_ctx.rx.h108.avail_output_voltage_V,
               g_fsm_ctx.rx.h108.avail_output_current_A,
               g_fsm_ctx.rx.h109.present_output_voltage_V,
               g_fsm_ctx.rx.h109.present_output_current_A,
               g_fsm_ctx.rx.h109.status,
               (g_fsm_ctx.rx.h109.remaining_charge_time_10s != 0xFF)
                   ? g_fsm_ctx.rx.h109.remaining_charge_time_10s * 10
                   : g_fsm_ctx.rx.h109.remaining_charge_time_60s * 60);
    }
#else
    /* Show EV-reported values */
    if (g_fsm_ctx.rx.h102_valid) {
        printf("[%s-EV]   MaxV=%dV | Target=%dV/%dA | Fault=0x%02X | Status=0x%02X | SOC=%d%%\r\n",
               FSM_ROLE_STR,
               g_fsm_ctx.rx.h100.max_battery_voltage_V,
               g_fsm_ctx.rx.h102.target_battery_voltage_V,
               g_fsm_ctx.rx.h102.charging_current_request_A,
               g_fsm_ctx.rx.h102.fault,
               g_fsm_ctx.rx.h102.status,
               g_fsm_ctx.rx.h102.charged_rate_percent);
    }
#endif
}

/* ============================================================================
 * LED HEARTBEAT
 * ============================================================================
 * Blink pattern indicates system state:
 *   Slow blink (1Hz):   Idle, waiting for plug
 *   Fast blink (4Hz):   Charging active
 *   Solid ON:           Fault condition
 *   Two quick blinks:   Handshake/pre-charge in progress
 */

static void led_heartbeat(void)
{
    static uint32_t led_timer = 0;
    static bool led_state = false;

    led_timer += MAIN_LOOP_PERIOD_MS;

    uint32_t blink_period;

    switch (g_fsm_ctx.state) {
    case CHADEMO_STATE_CHARGING:
        blink_period = 125;   /* 4 Hz fast blink */
        break;
    case CHADEMO_STATE_FAULT_SHUTDOWN:
        gpio_put(LED_PIN, 1); /* Solid ON */
        return;
    case CHADEMO_STATE_HANDSHAKE:
    case CHADEMO_STATE_PARAM_CHECK:
    case CHADEMO_STATE_PRECHARGE:
        blink_period = 250;   /* Medium blink */
        break;
    default:
        blink_period = 1000;  /* 1 Hz slow blink */
        break;
    }

    if (led_timer >= blink_period) {
        led_timer = 0;
        led_state = !led_state;
        gpio_put(LED_PIN, led_state ? 1 : 0);
    }
}

/* ============================================================================
 * SDK BINARY INFO (visible to picotool)
 * ============================================================================ */

bi_decl(bi_program_name("CHAdeMO Controller for RP2040"));
bi_decl(bi_program_version_string("1.0.0"));
bi_decl(bi_program_description("CHAdeMO DC Fast Charging Protocol Controller"));
bi_decl(bi_program_url("https://github.com/"));
bi_decl(bi_1pin_with_name(PIN_CONTACTOR_OUT, "CONTACTOR"));
bi_decl(bi_1pin_with_name(PIN_CAN1_CS, "CAN1_CS"));
bi_decl(bi_1pin_with_name(PIN_CAN2_CS, "CAN2_CS"));
