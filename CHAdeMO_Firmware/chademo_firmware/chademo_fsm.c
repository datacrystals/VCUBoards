/**
 * @file  chademo_fsm.c
 * @brief CHAdeMO Protocol State Machine Implementation
 * @details
 *   Non-blocking state machine implementing the complete CHAdeMO sequence.
 *   Heavily commented with protocol references.
 *
 *   Based on:
 *   - IEEE Std 2030.1.1-2015 control flow
 *   - furdog/chademo (EVSE state machine reference)
 *   - jamiejones85/ESP32-Chademo (vehicle-side state machine)
 *   - ccs32clara-chademo (timing and sequencing notes)
 */

#include "chademo_fsm.h"
#include "chademo_hal.h"
#include <string.h>

/* ============================================================================
 * INTERNAL HELPER FUNCTIONS
 * ============================================================================ */

static void transition_to(chademo_context_t *ctx, chademo_state_t new_state)
{
    CHADEMO_LOG("STATE: %s -> %s", chademo_fsm_state_name(ctx->state), chademo_fsm_state_name(new_state));
    ctx->prev_state     = ctx->state;
    ctx->state          = new_state;
    ctx->state_entry_ms = 0;  /* Caller increments this; reset on entry */

    /* Reset CAN RX timeout whenever we enter a state that expects charger
     * traffic.  We skip PLUG_DETECTED because the charger may be silent
     * while the user authenticates/pays at the station. */
    if (new_state >= CHADEMO_STATE_HANDSHAKE && new_state <= CHADEMO_STATE_CHARGING) {
        ctx->last_can_rx_ms = 0;
    }
}

static bool timeout_expired(const chademo_context_t *ctx, uint32_t timeout_ms)
{
    return (ctx->state_entry_ms >= timeout_ms);
}

static void reset_tx_defaults(chademo_context_t *ctx)
{
#if IS_VEHICLE
    /* Conservative defaults so the charger sees a valid vehicle.
     * Override these in your app with real battery parameters. */
    ctx->tx.h100.minimum_charge_current_A  = 1;   /* 1 A min (must be >0) */
    ctx->tx.h100.minimum_battery_voltage_V = 350; /* 350 V min (realistic for 450V pack) */
    ctx->tx.h100.max_battery_voltage_V     = 500;  /* 500 V max (bench test) */
    ctx->tx.h100.charged_rate_ref_const    = 100;  /* 100% = 0x64 */
    ctx->tx.h101.max_charge_time_10s       = 0xFF; /* Invalid = no limit */
    ctx->tx.h101.max_charge_time_60s       = 0xFF; /* Invalid = no limit */
    ctx->tx.h101.est_charge_time_60s       = 60;   /* 60 min estimate */
    ctx->tx.h101.total_capacity_100wh      = 400;  /* 40.0 kWh */
    ctx->tx.h102.protocol_number           = CHADEMO_PROTOCOL_V1_0;
    ctx->tx.h102.target_battery_voltage_V  = 450; /* 450 V target (bench test) */
    ctx->tx.h102.charging_current_request_A = 2;  /* 2 A request (conservative) */
    ctx->tx.h102.fault                     = 0;
    ctx->tx.h102.status                    = CHADEMO_EV_STATUS_CONTACTOR_OPEN;
    ctx->tx.h102.charged_rate_percent      = 20;   /* 20% SOC placeholder */
#else
    ctx->tx.h108.welding_detection_support = false;
    ctx->tx.h108.avail_output_voltage_V    = 0;
    ctx->tx.h108.avail_output_current_A    = 0;
    ctx->tx.h108.threshold_voltage_V       = 0;
    ctx->tx.h109.protocol_number           = CHADEMO_PROTOCOL_V1_0;
    ctx->tx.h109.present_output_voltage_V  = 0;
    ctx->tx.h109.present_output_current_A  = 0;
    ctx->tx.h109.status                    = 0;
    ctx->tx.h109.remaining_charge_time_10s = 0xFF;
    ctx->tx.h109.remaining_charge_time_60s = 0xFF;
#endif
}

static void reset_rx_buffers(chademo_context_t *ctx)
{
#if IS_VEHICLE
    chademo_can_init_rx_charger(&ctx->rx);
#else
    chademo_can_init_rx_vehicle(&ctx->rx);
#endif
}

static void enter_fault(chademo_context_t *ctx, chademo_fault_reason_t reason)
{
    if (ctx->state == CHADEMO_STATE_FAULT_SHUTDOWN) {
        return; /* Already in fault shutdown; don't re-enter */
    }
    CHADEMO_LOG("FAULT: %s (in state %s)", chademo_fsm_fault_name(reason), chademo_fsm_state_name(ctx->state));
    ctx->fault_reason = reason;
    transition_to(ctx, CHADEMO_STATE_FAULT_SHUTDOWN);
}

/* Clamp a value between min and max */
static uint8_t clamp_u8(uint8_t val, uint8_t min, uint8_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

#if IS_VEHICLE
/** Update vehicle request to match latest charger limits.
 *  Call from PARAM_CHECK / PRECHARGE / CHARGING so if the charger
 *  updates its available current after handshake, we follow it. */
static void apply_charger_limits(chademo_context_t *ctx)
{
    if (!ctx->rx.h108_valid) return;

    uint16_t avail_v = ctx->rx.h108.avail_output_voltage_V;
    uint8_t  avail_i = ctx->rx.h108.avail_output_current_A;
    uint16_t thr_v   = ctx->rx.h108.threshold_voltage_V;
    uint16_t pres_v  = ctx->rx.h109.present_output_voltage_V;
    uint8_t  pres_i  = ctx->rx.h109.present_output_current_A;
    uint8_t  status  = ctx->rx.h109.status;

    /* Log limits once per second */
    uint32_t now = hal_millis();
    if ((now - ctx->last_log_ms) >= 1000) {
        ctx->last_log_ms = now;
        CHADEMO_LOG("LIMITS avail=%uV/%uA thr=%uV pres=%uV/%uA stat=0x%02X",
                    avail_v, avail_i, thr_v, pres_v, pres_i, status);

        /* Threshold voltage is the STOP voltage (max output), NOT a minimum.
         * It's typically min(vehicle_max_v, charger_avail_v).  A battery
         * voltage below threshold is NORMAL and REQUIRED. */
        if (thr_v > 0 && ctx->tx.h100.max_battery_voltage_V > thr_v) {
            CHADEMO_LOG("WARN maxBattV=%u > chargerThr=%u (charger will stop at threshold)",
                        ctx->tx.h100.max_battery_voltage_V, thr_v);
        }
        /* Warn if target voltage is above charger max */
        if (ctx->tx.h102.target_battery_voltage_V > avail_v) {
            CHADEMO_LOG("WARN targetV=%u > chargerAvailV=%u",
                        ctx->tx.h102.target_battery_voltage_V, avail_v);
        }
        /* Warn if min battery voltage is below a sane level */
        if (ctx->tx.h100.minimum_battery_voltage_V > 0 &&
            ctx->tx.h100.minimum_battery_voltage_V < 200) {
            CHADEMO_LOG("WARN minBattV=%u seems low (<200V)",
                        ctx->tx.h100.minimum_battery_voltage_V);
        }
    }

    /* Cap current request to what charger currently says it can deliver.
     * Don't drop to 0A if charger temporarily reports 0A — keep at least
     * 1A so the charger knows we want to charge. */
    if (avail_i == 0) {
        /* Charger not ready yet; keep our request but don't exceed default */
        if (ctx->tx.h102.charging_current_request_A == 0) {
            ctx->tx.h102.charging_current_request_A = 1; /* signal intent */
        }
    } else if (ctx->tx.h102.charging_current_request_A > avail_i) {
        CHADEMO_LOG("LIMITS capping current %u -> %u",
                    ctx->tx.h102.charging_current_request_A, avail_i);
        ctx->tx.h102.charging_current_request_A = avail_i;
    }
}
#endif

/* ============================================================================
 * PUBLIC: INITIALIZATION
 * ============================================================================ */

void chademo_fsm_init(chademo_context_t *ctx)
{
    /* Preserve application-measured values across resets */
    uint16_t saved_voltage = ctx->measured_voltage_V;
    int16_t  saved_current = ctx->measured_current_A;
    uint8_t  saved_soc     = ctx->battery_soc;
    uint16_t saved_temp    = ctx->battery_temp_C;

    CHADEMO_LOG("FSM init (%s)", FSM_ROLE_STR);
    memset(ctx, 0, sizeof(*ctx));

    ctx->state           = CHADEMO_STATE_IDLE;
    ctx->prev_state      = CHADEMO_STATE_IDLE;
    ctx->fault_reason    = CHADEMO_FAULT_NONE;
    ctx->can_link_ok     = false;
    ctx->contactor_closed = false;
    ctx->charge_enabled  = false;
    ctx->last_log_ms     = 0;

    reset_tx_defaults(ctx);
    reset_rx_buffers(ctx);

    /* Restore measured values so they survive plug-out / handshake-timeout resets */
    ctx->measured_voltage_V = saved_voltage;
    ctx->measured_current_A = saved_current;
    ctx->battery_soc = saved_soc;
    ctx->battery_temp_C = saved_temp;
}

/* ============================================================================
 * PUBLIC: PUSH / PULL CAN FRAMES
 * ============================================================================ */

void chademo_fsm_push_can_rx(chademo_context_t *ctx, const chademo_can_frame_t *frame)
{
#if IS_VEHICLE
    if (chademo_can_unpack_charger_frame(&ctx->rx, frame)) {
        ctx->last_can_rx_ms = 0; /* Reset CAN timeout counter (managed by step) */
    }
#else
    if (chademo_can_unpack_vehicle_frame(&ctx->rx, frame)) {
        ctx->last_can_rx_ms = 0;
    }
#endif
}

bool chademo_fsm_pull_can_tx(chademo_context_t *ctx, chademo_can_frame_t *frame)
{
#if IS_VEHICLE
    chademo_can_frame_t frames[3];
    chademo_can_pack_vehicle_frames(&ctx->tx, frames);

    /* Rotate through the three vehicle frames */
    uint8_t idx = ctx->tx_frame_rotate % 3;
    *frame = frames[idx];
    ctx->tx_frame_rotate++;
    return true;
#else
    chademo_can_frame_t frames[2];
    chademo_can_pack_charger_frames(&ctx->tx, frames);

    uint8_t idx = ctx->tx_frame_rotate % 2;
    *frame = frames[idx];
    ctx->tx_frame_rotate++;
    return true;
#endif
}

/* ============================================================================
 * PUBLIC: SETTERS FOR MEASURED VALUES & TARGETS
 * ============================================================================ */

void chademo_fsm_set_measured_voltage(chademo_context_t *ctx, uint16_t voltage_V)
{
    ctx->measured_voltage_V = voltage_V;
}

void chademo_fsm_set_measured_current(chademo_context_t *ctx, int16_t current_A)
{
    ctx->measured_current_A = current_A;
}

void chademo_fsm_set_battery_soc(chademo_context_t *ctx, uint8_t soc_percent)
{
    ctx->battery_soc = clamp_u8(soc_percent, 0, 100);
}

void chademo_fsm_set_battery_temp(chademo_context_t *ctx, uint16_t temp_C)
{
    ctx->battery_temp_C = temp_C;
}

void chademo_fsm_set_target_voltage(chademo_context_t *ctx, uint16_t voltage_V)
{
#if IS_VEHICLE
    if (voltage_V <= CHADEMO_MAX_VOLTAGE_V) {
        ctx->tx.h102.target_battery_voltage_V = voltage_V;
    }
#else
    /* Charger updates available output voltage in h108 */
    if (voltage_V <= CHADEMO_MAX_VOLTAGE_V) {
        ctx->tx.h108.avail_output_voltage_V = voltage_V;
    }
#endif
}

void chademo_fsm_set_target_current(chademo_context_t *ctx, uint8_t current_A)
{
#if IS_VEHICLE
    if (current_A <= CHADEMO_MAX_CURRENT_A) {
        ctx->tx.h102.charging_current_request_A = current_A;
    }
#else
    if (current_A <= CHADEMO_MAX_CURRENT_A) {
        ctx->tx.h108.avail_output_current_A = current_A;
    }
#endif
}

void chademo_fsm_set_min_voltage(chademo_context_t *ctx, uint16_t min_voltage_V)
{
#if IS_VEHICLE
    ctx->tx.h100.minimum_battery_voltage_V = min_voltage_V;
#endif
}

void chademo_fsm_set_max_voltage(chademo_context_t *ctx, uint16_t max_voltage_V)
{
#if IS_VEHICLE
    ctx->tx.h100.max_battery_voltage_V = max_voltage_V;
#endif
    /* Charger uses this as threshold voltage in h108 */
#if IS_STATION
    ctx->tx.h108.threshold_voltage_V = max_voltage_V + 10; /* 10V margin */
#endif
}

void chademo_fsm_set_capacity_kwh(chademo_context_t *ctx, uint16_t capacity_100wh)
{
#if IS_VEHICLE
    ctx->tx.h101.total_capacity_100wh = capacity_100wh;
#endif
}

void chademo_fsm_request_shutdown(chademo_context_t *ctx)
{
    if (ctx->state == CHADEMO_STATE_CHARGING) {
        transition_to(ctx, CHADEMO_STATE_SHUTDOWN);
    }
}

void chademo_fsm_trigger_estop(chademo_context_t *ctx)
{
    enter_fault(ctx, CHADEMO_FAULT_ESTOP);
}

/* ============================================================================
 * STATE MACHINE STEP -- THE CORE
 * ============================================================================ */

chademo_event_t chademo_fsm_step(chademo_context_t *ctx, uint32_t dt_ms)
{
    /* Update timing accumulators */
    ctx->state_entry_ms  += dt_ms;
    ctx->last_tx_ms      += dt_ms;

    /* Only track CAN RX timeout in active charge-sequence states.
     * Otherwise last_can_rx_ms grows unbounded in IDLE and triggers
     * a spurious timeout immediately upon plug insertion. */
#if CHADEMO_STATION_DETECT_METHOD == 1
    if (ctx->state >= CHADEMO_STATE_PLUG_DETECTED && ctx->state <= CHADEMO_STATE_CHARGING) {
#else
    if (ctx->state >= CHADEMO_STATE_HANDSHAKE && ctx->state <= CHADEMO_STATE_CHARGING) {
#endif
        ctx->last_can_rx_ms += dt_ms;
    }

    chademo_event_t event = CHADEMO_EVENT_NONE;

    /* ================================================================
     * GLOBAL PLUG-REMOVAL MONITOR (VEHICLE)
     * ================================================================
     * If the plug is pulled at any time, immediately abort and return
     * to IDLE.  This avoids waiting for CAN timeout after unplug.
     * ================================================================ */
#if IS_VEHICLE
    if (ctx->state != CHADEMO_STATE_IDLE && ctx->state != CHADEMO_STATE_STOPPED) {
        if (hal_gpio_read_pp()) {
            CHADEMO_LOG("GLOBAL: PP HIGH — plug removed, aborting");
            hal_gpio_set_dcp(false);
            hal_gpio_set_contactor(false);
            chademo_fsm_init(ctx);
            event = CHADEMO_EVENT_UNPLUGGED;
            return event;
        }
    }
#else
    if (ctx->state != CHADEMO_STATE_IDLE && ctx->state != CHADEMO_STATE_STOPPED) {
        /* DCP HIGH = vehicle removed (inverted logic on this hardware) */
        if (hal_gpio_read_dcp()) {
            CHADEMO_LOG("GLOBAL: DCP HIGH — plug removed, aborting");
            hal_gpio_set_ss1(false);
            hal_gpio_set_ss2(false);
            hal_gpio_set_contactor(false);
            chademo_fsm_init(ctx);
            event = CHADEMO_EVENT_UNPLUGGED;
            return event;
        }
    }
#endif

    /* ================================================================
     * GLOBAL FAULT MONITORING (checked only during active charge sequence)
     * ================================================================ */
    if (ctx->state >= CHADEMO_STATE_HANDSHAKE && ctx->state <= CHADEMO_STATE_CHARGING) {
        /* CAN communication timeout (only after handshake starts — charger
         * may be silent during plug-in while user authenticates/pays) */
        if (ctx->last_can_rx_ms > CHADEMO_Timing_CAN_TIMEOUT_MS) {
            CHADEMO_LOG("GLOBAL: CAN timeout, last_rx=%lu ms", ctx->last_can_rx_ms);
            enter_fault(ctx, CHADEMO_FAULT_CAN_TIMEOUT);
        }

        /* Battery overtemperature */
        if (ctx->battery_temp_C > 60) {  /* 60C threshold, adjust as needed */
            CHADEMO_LOG("GLOBAL: Battery overtemp %u C", ctx->battery_temp_C);
            enter_fault(ctx, CHADEMO_FAULT_BATTERY_OVERTEMP);
        }
    }

    /* ================================================================
     * MAIN STATE DISPATCHER
     * ================================================================ */
    switch (ctx->state) {

    /* =================================================================
     * STATE: IDLE -- Waiting for plug insertion
     * =================================================================
     * VEHICLE: Monitor PP (Proximity Pilot, GPIO18). PP goes LOW when
     *          plug is inserted (pull-down via plug resistor).
     * STATION: Not typically in IDLE; station is always ready. Could
     *          monitor for vehicle presence via SS1/SS2.
     *
     * Transition: PLUG_DETECTED when physical connection confirmed.
     * ================================================================= */
    case CHADEMO_STATE_IDLE: {
#if IS_VEHICLE
        {
            bool pp = hal_gpio_read_pp();
            /* PP is active-LOW (pulled down via plug's Rp when inserted) */
            if (!pp) {
                /* Debounce: must be low for >200ms */
                if (ctx->plug_detect_ms == 0) {
                    CHADEMO_LOG("IDLE: PP LOW, starting debounce");
                    ctx->plug_detect_ms = ctx->state_entry_ms;
                } else if ((ctx->state_entry_ms - ctx->plug_detect_ms) >= CHADEMO_Timing_PLUG_DEBOUNCE_MS) {
                    CHADEMO_LOG("IDLE: PP debounced, plug detected");
                    transition_to(ctx, CHADEMO_STATE_PLUG_DETECTED);
                    event = CHADEMO_EVENT_PLUG_INSERTED;
                }
            } else {
                if (ctx->plug_detect_ms != 0) {
                    CHADEMO_LOG("IDLE: PP HIGH, debounce reset");
                }
                ctx->plug_detect_ms = 0;
            }
        }
#else
        {
#if CHADEMO_STATION_DETECT_METHOD == 1
            /* CAN-based detection: DCP input is unreliable on this hardware
             * (external pullup keeps GPIO9 HIGH even with nothing connected).
             * Detect vehicle by receiving any of its periodic CAN frames. */
            bool vehicle_present = (ctx->rx.h100_valid || ctx->rx.h101_valid || ctx->rx.h102_valid);
            const char *detect_src = "CAN";
#else
            bool dcp = hal_gpio_read_dcp();
            /* Hardware is inverted: DCP LOW = vehicle present */
            bool vehicle_present = !dcp;
            const char *detect_src = "DCP";
#endif
            if (vehicle_present) {
                if (ctx->plug_detect_ms == 0) {
                    CHADEMO_LOG("IDLE: %s vehicle detected, starting debounce", detect_src);
                    ctx->plug_detect_ms = ctx->state_entry_ms;
                } else if ((ctx->state_entry_ms - ctx->plug_detect_ms) >= CHADEMO_Timing_PLUG_DEBOUNCE_MS) {
                    CHADEMO_LOG("IDLE: %s debounced, vehicle detected", detect_src);
                    transition_to(ctx, CHADEMO_STATE_PLUG_DETECTED);
                    event = CHADEMO_EVENT_PLUG_INSERTED;
                }
            } else {
                if (ctx->plug_detect_ms != 0) {
                    CHADEMO_LOG("IDLE: %s lost, debounce reset", detect_src);
                }
                ctx->plug_detect_ms = 0;
            }
        }
#endif
        break;
    }

    /* =================================================================
     * STATE: PLUG_DETECTED -- Physical connection made, start CAN
     * =================================================================
     * VEHICLE: Set DCP (GPIO19) HIGH to tell charger "I'm here".
     *          Start sending 0x100/0x101/0x102 periodically.
     *          Wait for charger to respond with 0x108/0x109.
     *
     * STATION: Set SS1 (GPIO7) HIGH -- charge start signal 1.
     *          Start listening for vehicle CAN frames.
     *          When 0x100/0x101/0x102 received, start TX of 0x108/0x109.
     *
     * Transition: HANDSHAKE when first valid CAN frames exchanged.
     * ================================================================= */
    case CHADEMO_STATE_PLUG_DETECTED: {
#if IS_VEHICLE
        /* Assert DCP to tell charger we're connected */
        hal_gpio_set_dcp(true);
        {
            uint32_t now = hal_millis();
            if ((now - ctx->last_log_ms) >= 1000) {
                ctx->last_log_ms = now;
                CHADEMO_LOG("PLUG_DETECTED: DCP=HIGH, CAN h108=%d h109=%d",
                            (int)ctx->rx.h108_valid, (int)ctx->rx.h109_valid);
            }
        }

        /* Start sending our parameters */
        ctx->tx.h102.status = CHADEMO_EV_STATUS_CONTACTOR_OPEN;
        ctx->tx.h102.protocol_number = CHADEMO_PROTOCOL_V1_0;

        /* Wait for charger CAN frames (0x108 or 0x109) */
        if (ctx->rx.h108_valid || ctx->rx.h109_valid) {
            CHADEMO_LOG("PLUG_DETECTED: charger CAN detected -> HANDSHAKE");
            transition_to(ctx, CHADEMO_STATE_HANDSHAKE);
            ctx->can_link_ok = true;
        }

        /* Timeout: if no charger response, return to IDLE */
        if (timeout_expired(ctx, CHADEMO_Timing_HANDSHAKE_TIMEOUT_MS)) {
            CHADEMO_LOG("PLUG_DETECTED: timeout, no charger CAN -> IDLE");
            hal_gpio_set_dcp(false);
            chademo_fsm_init(ctx);  /* Full reset */
        }
#else
        /* EVSE: Assert SS1 (charge sequence signal 1, connector pin 5) */
        hal_gpio_set_ss1(true);
        {
            uint32_t now = hal_millis();
            if ((now - ctx->last_log_ms) >= 1000) {
                ctx->last_log_ms = now;
                CHADEMO_LOG("PLUG_DETECTED: SS1=HIGH, CAN h100=%d h101=%d h102=%d",
                            (int)ctx->rx.h100_valid, (int)ctx->rx.h101_valid, (int)ctx->rx.h102_valid);
            }
        }

        /* Start listening for vehicle CAN */
        if (ctx->rx.h100_valid && ctx->rx.h101_valid && ctx->rx.h102_valid) {
            /* Got all three vehicle frames -- link established */
            CHADEMO_LOG("PLUG_DETECTED: vehicle CAN complete -> HANDSHAKE");
            transition_to(ctx, CHADEMO_STATE_HANDSHAKE);
            ctx->can_link_ok = true;
            event = CHADEMO_EVENT_HANDSHAKE_COMPLETE;
        }

        /* Timeout */
        if (timeout_expired(ctx, CHADEMO_Timing_HANDSHAKE_TIMEOUT_MS)) {
            CHADEMO_LOG("PLUG_DETECTED: timeout, no vehicle CAN -> IDLE");
            hal_gpio_set_ss1(false);
            chademo_fsm_init(ctx);
        }

#if CHADEMO_STATION_DETECT_METHOD == 1
        /* Short CAN-loss timeout: if we detected the vehicle via CAN but then
         * lose communication, return to IDLE quickly (2 s) instead of waiting
         * the full 5-minute handshake timeout. */
        if (ctx->last_can_rx_ms > 2000) {
            CHADEMO_LOG("PLUG_DETECTED: CAN lost (%lu ms), returning to IDLE", ctx->last_can_rx_ms);
            hal_gpio_set_ss1(false);
            chademo_fsm_init(ctx);
        }
#endif
#endif
        break;
    }

    /* =================================================================
     * STATE: HANDSHAKE -- Protocol version and parameter negotiation
     * =================================================================
     * Both sides have established CAN communication. Now verify:
     *   1. Protocol version compatibility (0x102 byte 0 vs 0x109 byte 0)
     *   2. Voltage compatibility (battery max V vs. charger avail V)
     *   3. Current capability
     *
     * VEHICLE: Validate charger can provide our target voltage.
     *          If charger voltage < our max battery voltage -> FAULT.
     *          Copy charger available current as our request limit.
     *
     * STATION: Validate EV's max battery voltage <= our output capability.
     *          If EV max V > charger avail V -> battery incompatible.
     *
     * Transition: PARAM_CHECK if all parameters compatible.
     *             FAULT_SHUTDOWN if incompatible.
     * ================================================================= */
    case CHADEMO_STATE_HANDSHAKE: {
#if IS_VEHICLE
        /* Check charger protocol version */
        if (ctx->rx.h109_valid) {
            CHADEMO_LOG("HANDSHAKE: charger proto=%u availV=%uV availI=%uA",
                        ctx->rx.h109.protocol_number,
                        ctx->rx.h108.avail_output_voltage_V,
                        ctx->rx.h108.avail_output_current_A);

            /* If charger is 1.0, we can use full features */
            if (ctx->rx.h109.protocol_number >= CHADEMO_PROTOCOL_V1_0) {
                ctx->tx.h102.protocol_number = CHADEMO_PROTOCOL_V1_0;
            }

            /* Verify charger can provide our required voltage */
            if (ctx->rx.h108.avail_output_voltage_V < ctx->tx.h100.max_battery_voltage_V) {
                CHADEMO_LOG("HANDSHAKE: INCOMPATIBLE chargerV=%u < maxBattV=%u",
                            ctx->rx.h108.avail_output_voltage_V,
                            ctx->tx.h100.max_battery_voltage_V);
                enter_fault(ctx, CHADEMO_FAULT_INCOMPATIBLE);
                break;
            }

            /* Note: we used to cap current here, but the charger may report
             * 0A initially and then update after auth.  We now apply limits
             * dynamically in PARAM_CHECK / PRECHARGE / CHARGING. */

            CHADEMO_LOG("HANDSHAKE: OK -> PARAM_CHECK");
            transition_to(ctx, CHADEMO_STATE_PARAM_CHECK);
            event = CHADEMO_EVENT_HANDSHAKE_COMPLETE;
        }
#else
        /* EVSE: Check vehicle parameters */
        if (ctx->rx.h102_valid) {
            CHADEMO_LOG("HANDSHAKE: EV proto=%u maxBattV=%uV reqI=%uA",
                        ctx->rx.h102.protocol_number,
                        ctx->rx.h100.max_battery_voltage_V,
                        ctx->rx.h102.charging_current_request_A);

            /* Protocol version check */
            if (ctx->rx.h102.protocol_number >= CHADEMO_PROTOCOL_V1_0) {
                ctx->tx.h109.protocol_number = CHADEMO_PROTOCOL_V1_0;
            }

            /* Battery voltage compatibility check */
            if (ctx->rx.h100.max_battery_voltage_V > ctx->tx.h108.avail_output_voltage_V) {
                /* EV wants more voltage than we can provide */
                CHADEMO_LOG("HANDSHAKE: INCOMPATIBLE EV maxV=%u > availV=%u",
                            ctx->rx.h100.max_battery_voltage_V,
                            ctx->tx.h108.avail_output_voltage_V);
                ctx->tx.h109.status |= CHADEMO_SE_STATUS_BATTERY_INCOMPATIBLE;
                enter_fault(ctx, CHADEMO_FAULT_INCOMPATIBLE);
                break;
            }

            CHADEMO_LOG("HANDSHAKE: OK -> PARAM_CHECK");
            transition_to(ctx, CHADEMO_STATE_PARAM_CHECK);
            event = CHADEMO_EVENT_PARAMS_COMPATIBLE;
        }
#endif
        /* CAN timeout handled by global monitor */
        break;
    }

    /* =================================================================
     * STATE: PARAM_CHECK -- Final compatibility validation
     * =================================================================
     * Last chance to abort before committing to high-voltage operations.
     * Application-level checks (e.g., BMS approval) should happen here.
     *
     * VEHICLE: Ready to proceed; set charging_enabled in status.
     *          Set CHARGE_ENABLE output (DCP stays HIGH).
     *
     * STATION: Lock connector, assert SS2.
     *          Set connector_locked in status.
     *          Wait for vehicle's oc_j (charge permission via DCP).
     *
     * Transition: INSULATION_TEST (EVSE) or PRECHARGE (EV).
     * ================================================================= */
    case CHADEMO_STATE_PARAM_CHECK: {
#if IS_VEHICLE
        /* Signal that we're ready to charge */
        ctx->tx.h102.status |= CHADEMO_EV_STATUS_CHARGING_ENABLED;
        ctx->charge_enabled = true;
        apply_charger_limits(ctx);
        {
            uint32_t now = hal_millis();
            if ((now - ctx->last_log_ms) >= 1000) {
                ctx->last_log_ms = now;
                CHADEMO_LOG("PARAM_CHECK: charge_enabled=1, status=0x%02X", ctx->tx.h102.status);
            }
        }

        /* Wait for charger to signal connector locked and ready */
        if (ctx->rx.h109.status & CHADEMO_SE_STATUS_CONNECTOR_LOCKED) {
            CHADEMO_LOG("PARAM_CHECK: connector locked -> PRECHARGE");
            transition_to(ctx, CHADEMO_STATE_PRECHARGE);
        }

        /* If charger reports incompatibility, abort */
        if (ctx->rx.h109.status & CHADEMO_SE_STATUS_BATTERY_INCOMPATIBLE) {
            CHADEMO_LOG("PARAM_CHECK: charger reports incompatible");
            enter_fault(ctx, CHADEMO_FAULT_INCOMPATIBLE);
        }
#else
        /* EVSE: Lock connector (mechanical lock actuator) */
        /* In hardware, this would trigger a GPIO to the lock motor */
        ctx->tx.h109.status |= CHADEMO_SE_STATUS_CONNECTOR_LOCKED;
        CHADEMO_LOG("PARAM_CHECK: connector locked, status=0x%02X", ctx->tx.h109.status);

        /* Assert SS2 (charge sequence signal 2, connector pin 10) */
        hal_gpio_set_ss2(true);

        /* Wait for vehicle charge permission (DCP should be HIGH) */
        if (hal_gpio_read_dcp()) {
            /* Vehicle is ready -- proceed to insulation test */
            CHADEMO_LOG("PARAM_CHECK: DCP HIGH -> INSULATION_TEST");
            transition_to(ctx, CHADEMO_STATE_INSULATION_TEST);
            event = CHADEMO_EVENT_PARAMS_COMPATIBLE;
        }

        /* Timeout waiting for vehicle */
        if (timeout_expired(ctx, CHADEMO_Timing_HANDSHAKE_TIMEOUT_MS)) {
            CHADEMO_LOG("PARAM_CHECK: timeout waiting for DCP");
            enter_fault(ctx, CHADEMO_FAULT_CAN_TIMEOUT);
        }
#endif
        break;
    }

    /* =================================================================
     * STATE: INSULATION_TEST -- Verify isolation resistance (EVSE only)
     * =================================================================
     * Per IEC 61851-23, the EVSE must verify insulation resistance
     * between DC+ / DC- and protective earth before applying voltage.
     *
     * STATION:
     *   1. Verify EV contactors are OPEN (no voltage on cable).
     *   2. Apply test voltage, measure insulation resistance.
     *   3. If R_iso >= 500 Ohm/V (typically >100kOhm for 200V system), PASS.
     *   4. Report result via CAN and proceed.
     *
     * This is a simplified implementation. A real system would interface
     * with an insulation monitoring device (IMD).
     *
     * VEHICLE: This state is not used (vehicle skips to PRECHARGE).
     *
     * Transition: PRECHARGE on pass, FAULT on fail.
     * ================================================================= */
    case CHADEMO_STATE_INSULATION_TEST: {
#if IS_STATION
        /* Verify no voltage on output terminals (EV contactors open) */
        if (ctx->measured_voltage_V >= 10) {
            /* Voltage present -- EV contactors may be welded! */
            CHADEMO_LOG("INSULATION_TEST: voltage present (%uV), possible weld", ctx->measured_voltage_V);
            if (timeout_expired(ctx, CHADEMO_Timing_PRECHARGE_TIMEOUT_MS / 2)) {
                enter_fault(ctx, CHADEMO_FAULT_CONTACTOR_WELD);
            }
            break;
        }

        /* TODO: Trigger actual insulation test via IMD GPIO */
        /* For now, simulate a passed test after hold time */
        if (timeout_expired(ctx, CHADEMO_Timing_INSULATION_HOLD_MS)) {
            CHADEMO_LOG("INSULATION_TEST: passed -> PRECHARGE");
            transition_to(ctx, CHADEMO_STATE_PRECHARGE);
            event = CHADEMO_EVENT_INSULATION_OK;
        }
#else
        /* VEHICLE: Should never reach here, but if we do, skip to precharge */
        CHADEMO_LOG("INSULATION_TEST: skip (vehicle) -> PRECHARGE");
        transition_to(ctx, CHADEMO_STATE_PRECHARGE);
#endif
        break;
    }

    /* =================================================================
     * STATE: PRECHARGE -- Match EVSE output voltage to battery voltage
     * =================================================================
     * The EVSE ramps its output voltage to match the battery voltage.
     * When the difference is small enough (<50V), the EV closes its
     * contactors, connecting the battery to the DC bus.
     *
     * VEHICLE:
     *   - Monitor charger present_voltage (0x109 bytes 1-2).
     *   - When |V_charger - V_battery| < CHADEMO_PRECHARGE_THRESHOLD_V (50V):
     *     -> Close contactor (GPIO15 HIGH).
     *     -> Set contactor_open = 0 in 0x102 status.
     *   - Wait for charger to detect contactor closure and start current.
     *
     * STATION:
     *   - Ramp output voltage toward EV's target_battery_voltage.
     *   - Report present_voltage in 0x109.
     *   - Wait for EV to signal contactor closed (0x102 bit 3 = 0).
     *   - Once EV contactors closed, transition to CHARGING.
     *
     * CRITICAL TIMING: Vehicle must close contactors within ~1 second
     * of voltage matching, or charger may abort. (Reference: ccs32clara
     * notes that cars will fail if no amps delivered within ~4 seconds
     * of contactor closure).
     *
     * Transition: CHARGING when contactors confirmed closed.
     * ================================================================= */
    case CHADEMO_STATE_PRECHARGE: {
#if IS_VEHICLE
        /* Monitor charger output voltage */
        if (ctx->rx.h109_valid) {
            uint16_t charger_voltage = ctx->rx.h109.present_output_voltage_V;
            uint16_t diff = (charger_voltage > ctx->measured_voltage_V)
                          ? (charger_voltage - ctx->measured_voltage_V)
                          : (ctx->measured_voltage_V - charger_voltage);

            {
                uint32_t now = hal_millis();
                if ((now - ctx->last_log_ms) >= 1000) {
                    ctx->last_log_ms = now;
                    CHADEMO_LOG("PRECHARGE: chargerV=%u battV=%u diff=%u",
                                charger_voltage, ctx->measured_voltage_V, diff);
                }
            }

            apply_charger_limits(ctx);

            /* When voltage difference is small enough, close contactor */
            if (diff < CHADEMO_PRECHARGE_THRESHOLD_V) {
                if (!ctx->contactor_closed) {
                    CHADEMO_LOG("PRECHARGE: diff<%uV, closing contactor", CHADEMO_PRECHARGE_THRESHOLD_V);
                    hal_gpio_set_contactor(true);  /* CLOSE contactor */
                    ctx->contactor_closed = true;
                    ctx->tx.h102.status &= ~CHADEMO_EV_STATUS_CONTACTOR_OPEN;
                }
            }
        }

        /* Check if charger has started delivering current (charging bit set) */
        if (ctx->contactor_closed &&
            (ctx->rx.h109.status & CHADEMO_SE_STATUS_CHARGING)) {
            CHADEMO_LOG("PRECHARGE: charger reports CHARGING -> CHARGING");
            transition_to(ctx, CHADEMO_STATE_CHARGING);
            event = CHADEMO_EVENT_CHARGING_STARTED;
        }

        /* Timeout: if precharge takes too long, abort */
        if (timeout_expired(ctx, CHADEMO_Timing_PRECHARGE_TIMEOUT_MS)) {
            CHADEMO_LOG("PRECHARGE: timeout");
            enter_fault(ctx, CHADEMO_FAULT_VOLTAGE_MISMATCH);
        }
#else
        /* EVSE: Report present voltage approaching target */
        /* In a real system, this would control the PSU setpoint */
        ctx->tx.h109.present_output_voltage_V = ctx->measured_voltage_V;
        CHADEMO_LOG("PRECHARGE: presentV=%u, EV status=0x%02X",
                    ctx->tx.h109.present_output_voltage_V, ctx->rx.h102.status);

        /* Check if EV has closed contactors */
        if (!(ctx->rx.h102.status & CHADEMO_EV_STATUS_CONTACTOR_OPEN)) {
            /* EV contactors are closed! Start charging. */
            CHADEMO_LOG("PRECHARGE: EV contactor closed -> CHARGING");
            ctx->tx.h109.status |= CHADEMO_SE_STATUS_CHARGING;
            transition_to(ctx, CHADEMO_STATE_CHARGING);
            event = CHADEMO_EVENT_CHARGING_STARTED;
        }

        /* Precharge timeout */
        if (timeout_expired(ctx, CHADEMO_Timing_PRECHARGE_TIMEOUT_MS)) {
            CHADEMO_LOG("PRECHARGE: timeout");
            enter_fault(ctx, CHADEMO_FAULT_VOLTAGE_MISMATCH);
        }
#endif
        break;
    }

    /* =================================================================
     * STATE: CHARGING -- Active current delivery with closed-loop control
     * =================================================================
     * The main charging phase. Both sides exchange periodic CAN frames
     * at 100ms intervals for real-time control and monitoring.
     *
     * VEHICLE:
     *   - Update 0x102 with target_voltage, target_current_request.
     *   - Apply 20A/sec slew rate limit to current changes.
     *   - Monitor charger-reported voltage/current for mismatches.
     *   - Check for normal stop conditions (SOC target, timer).
     *   - If fault detected -> enter CEASE_CURRENT sub-state.
     *
     * STATION:
     *   - Update 0x108/0x109 with present output and status.
     *   - Control PSU to deliver requested voltage/current.
     *   - Monitor EV fault flags in 0x102 byte 4.
     *   - If EV requests 0A -> enter SHUTDOWN.
     *
     * SAFETY MONITORS (both sides):
     *   - Voltage mismatch: |V_reported - V_measured| > 12.5% for >5 frames
     *   - Current mismatch: |I_reported - I_measured| > threshold for >5 frames
     *   - EV fault byte (0x102[4]) non-zero
     *   - Charger status bits (0x109[5]) fault indications
     *   - CAN timeout (>1 second without frames)
     *
     * Transition: SHUTDOWN on normal stop or fault.
     * ================================================================= */
    case CHADEMO_STATE_CHARGING: {
#if IS_VEHICLE
        apply_charger_limits(ctx);

        /* ---- Dynamic current control with 20A/sec slew rate ----
         * CHAdeMO spec allows +/-20A per second change in current request.
         * We transmit every 100ms, so max change per step = 20A / 10 = 2A.
         * In practice, we ramp up at 1A per 100ms (10A/sec) for safety,
         * and ramp down at 2A per 100ms (20A/sec) for fast fault response.
         */
        if (ctx->asking_amps < ctx->tx.h102.charging_current_request_A) {
            /* Ramp up slowly: 1A per 100ms call */
            ctx->asking_amps++;
        } else if (ctx->asking_amps > ctx->tx.h102.charging_current_request_A) {
            /* Ramp down fast: up to 2A per 100ms call */
            if (ctx->asking_amps > 0) ctx->asking_amps--;
            if (ctx->asking_amps > ctx->tx.h102.charging_current_request_A &&
                ctx->asking_amps > 0) {
                ctx->asking_amps--;
            }
        }

        /* Clamp asking_amps to what charger says it can deliver */
        if (ctx->rx.h108_valid && ctx->asking_amps > ctx->rx.h108.avail_output_current_A) {
            ctx->asking_amps = ctx->rx.h108.avail_output_current_A;
        }

        /* The actual transmitted current request is the ramped value */
        ctx->tx.h102.charging_current_request_A = ctx->asking_amps;

        /* ---- Voltage mismatch check ----
         * Abort if measured voltage differs from charger-reported voltage
         * by more than 12.5% for 5 consecutive checks.
         */
        if (ctx->rx.h109_valid) {
            uint16_t reported_v = ctx->rx.h109.present_output_voltage_V;
            uint16_t measured_v = ctx->measured_voltage_V;
            uint16_t tolerance = reported_v >> 3;  /* 12.5% */

            if (reported_v > measured_v + tolerance ||
                measured_v > reported_v + tolerance) {
                ctx->v_mismatch_count++;
                if (ctx->v_mismatch_count > 4) {
                    CHADEMO_LOG("CHARGING: voltage mismatch rep=%u meas=%u",
                                reported_v, measured_v);
                    enter_fault(ctx, CHADEMO_FAULT_VOLTAGE_MISMATCH);
                    break;
                }
            } else {
                ctx->v_mismatch_count = 0;
            }

            /* ---- Current mismatch check ---- */
            uint8_t reported_c = ctx->rx.h109.present_output_current_A;
            int16_t measured_c = ctx->measured_current_A < 0
                               ? -ctx->measured_current_A  /* Into battery = positive */
                               : ctx->measured_current_A;
            uint8_t c_tolerance = reported_c >> 3;
            if (c_tolerance < 3) c_tolerance = 3;

            if ((uint16_t)measured_c > reported_c + c_tolerance ||
                (reported_c > (uint16_t)measured_c + c_tolerance)) {
                ctx->c_mismatch_count++;
                if (ctx->c_mismatch_count > 4) {
                    CHADEMO_LOG("CHARGING: current mismatch rep=%u meas=%d",
                                reported_c, measured_c);
                    enter_fault(ctx, CHADEMO_FAULT_CURRENT_MISMATCH);
                    break;
                }
            } else {
                ctx->c_mismatch_count = 0;
            }
        }

        /* ---- Check for EVSE fault status ---- */
        if (ctx->rx.h109.status & (CHADEMO_SE_STATUS_CHARGER_MALFUNCTION |
                                    CHADEMO_SE_STATUS_SYS_MALFUNCTION)) {
            ctx->fault_count++;
            if (ctx->fault_count > 3) {
                CHADEMO_LOG("CHARGING: EVSE fault status=0x%02X", ctx->rx.h109.status);
                enter_fault(ctx, CHADEMO_FAULT_CHARGER_MALFUNCTION);
                break;
            }
        } else {
            ctx->fault_count = 0;
        }

        /* ---- Check for EVSE stop request ---- */
        if (ctx->rx.h109.status & CHADEMO_SE_STATUS_STOP_CONTROL) {
            CHADEMO_LOG("CHARGING: EVSE stop control");
            transition_to(ctx, CHADEMO_STATE_SHUTDOWN);
            break;
        }

        /* ---- Monitor EV fault byte from our own BMS ---- */
        if (ctx->tx.h102.fault != 0) {
            enter_fault(ctx, CHADEMO_FAULT_OTHER);
            break;
        }

#else
        /* ---- EVSE CHARGING LOGIC ---- */

        /* Check for EV fault flags */
        if (ctx->rx.h102.fault != 0) {
            ctx->fault_count++;
            if (ctx->fault_count > 3) {
                ctx->fault_status = ctx->rx.h102.fault;
                CHADEMO_LOG("CHARGING: EV fault=0x%02X", ctx->rx.h102.fault);
                enter_fault(ctx, CHADEMO_FAULT_OTHER);
                break;
            }
        } else {
            ctx->fault_count = 0;
        }

        /* Check if EV requests zero current -> normal shutdown */
        if (ctx->rx.h102_valid && ctx->rx.h102.charging_current_request_A == 0) {
            /* EV wants to stop -- initiate shutdown sequence */
            CHADEMO_LOG("CHARGING: EV requests 0A -> SHUTDOWN");
            transition_to(ctx, CHADEMO_STATE_SHUTDOWN);
            break;
        }

        /* Check EV status for stop request */
        if (ctx->rx.h102.status & CHADEMO_EV_STATUS_NORMAL_STOP_REQUEST) {
            transition_to(ctx, CHADEMO_STATE_SHUTDOWN);
            break;
        }

        /* Check if EV opened contactors unexpectedly */
        if (ctx->rx.h102.status & CHADEMO_EV_STATUS_CONTACTOR_OPEN) {
            enter_fault(ctx, CHADEMO_FAULT_UNPLUG_UNDER_LOAD);
            break;
        }

        /* Update present output in 0x109 */
        ctx->tx.h109.present_output_voltage_V = ctx->measured_voltage_V;
        ctx->tx.h109.present_output_current_A =
            (uint8_t)(ctx->measured_current_A < 0
                      ? -ctx->measured_current_A
                      : ctx->measured_current_A);

        /* Set charging bit in status */
        ctx->tx.h109.status |= CHADEMO_SE_STATUS_CHARGING;
#endif
        break;
    }

    /* =================================================================
     * STATE: SHUTDOWN -- Normal termination sequence
     * =================================================================
     * Graceful shutdown: ramp current to zero, then open contactors.
     *
     * VEHICLE: Set current request to 0A. Wait for charger to report
     *          present_current = 0. Then open contactor.
     *
     * STATION: Ramp output current to 0. When current reaches zero,
     *          clear charging bit, then wait for EV to open contactors.
     *
     * Transition: WAIT_ZERO_CURRENT.
     * ================================================================= */
    case CHADEMO_STATE_SHUTDOWN: {
#if IS_VEHICLE
        /* Request zero current */
        ctx->tx.h102.charging_current_request_A = 0;
        ctx->asking_amps = 0;

        /* Clear charging enabled */
        ctx->tx.h102.status &= ~CHADEMO_EV_STATUS_CHARGING_ENABLED;
        ctx->charge_enabled = false;

        /* Wait for charger to report zero current */
        if (ctx->rx.h109_valid && ctx->rx.h109.present_output_current_A == 0) {
            transition_to(ctx, CHADEMO_STATE_WAIT_ZERO_CURRENT);
        }

        /* Timeout: force contactor open anyway */
        if (timeout_expired(ctx, CHADEMO_Timing_SHUTDOWN_RAMP_MS)) {
            transition_to(ctx, CHADEMO_STATE_WAIT_ZERO_CURRENT);
        }
#else
        /* EVSE: Ramp current to zero */
        ctx->tx.h109.present_output_current_A = 0;

        /* Clear charging status */
        ctx->tx.h109.status &= ~CHADEMO_SE_STATUS_CHARGING;
        ctx->tx.h109.status |= CHADEMO_SE_STATUS_STOP_CONTROL;

        /* Wait for EV to open contactors or request 0A */
        if ((ctx->rx.h102.status & CHADEMO_EV_STATUS_CONTACTOR_OPEN) ||
            ctx->measured_current_A == 0) {
            transition_to(ctx, CHADEMO_STATE_WAIT_ZERO_CURRENT);
        }

        if (timeout_expired(ctx, CHADEMO_Timing_SHUTDOWN_RAMP_MS)) {
            transition_to(ctx, CHADEMO_STATE_WAIT_ZERO_CURRENT);
        }
#endif
        break;
    }

    /* =================================================================
     * STATE: WAIT_ZERO_CURRENT -- Confirm no current before opening
     * =================================================================
     * Safety dwell: wait a short time with zero current confirmed
     * before opening contactors. Prevents arcing.
     *
     * Transition: CONTACTOR_OPEN after 150ms dwell.
     * ================================================================= */
    case CHADEMO_STATE_WAIT_ZERO_CURRENT: {
        if (timeout_expired(ctx, 150)) {
            transition_to(ctx, CHADEMO_STATE_CONTACTOR_OPEN);
        }
        break;
    }

    /* =================================================================
     * STATE: CONTACTOR_OPEN -- Open contactors, clean up
     * =================================================================
     * Open the high-voltage contactor and reset all control signals.
     *
     * VEHICLE: Open contactor (GPIO15 LOW). De-assert DCP.
     * STATION: De-assert SS1 and SS2. Unlock connector.
     *
     * Transition: STOPPED.
     * ================================================================= */
    case CHADEMO_STATE_CONTACTOR_OPEN: {
        /* CRITICAL: Open contactor FIRST */
        hal_gpio_set_contactor(false);
        ctx->contactor_closed = false;

#if IS_VEHICLE
        /* Update CAN status */
        ctx->tx.h102.status |= CHADEMO_EV_STATUS_CONTACTOR_OPEN;
        ctx->tx.h102.status &= ~CHADEMO_EV_STATUS_CHARGING_ENABLED;

        /* De-assert DCP */
        hal_gpio_set_dcp(false);
#else
        /* EVSE: De-assert control signals */
        hal_gpio_set_ss1(false);
        hal_gpio_set_ss2(false);

        /* Clear status bits */
        ctx->tx.h109.status &= ~(CHADEMO_SE_STATUS_CHARGING |
                                  CHADEMO_SE_STATUS_CONNECTOR_LOCKED |
                                  CHADEMO_SE_STATUS_STOP_CONTROL);
#endif

        /* Force one last CAN transmission */
        if (timeout_expired(ctx, 100)) {
            transition_to(ctx, CHADEMO_STATE_STOPPED);
            event = CHADEMO_EVENT_CHARGE_COMPLETE;
        }
        break;
    }

    /* =================================================================
     * STATE: FAULT_SHUTDOWN -- Emergency shutdown
     * =================================================================
     * IMMEDIATE safety shutdown. This path is taken on any fault.
     * Order of operations is critical:
     *   1. Request zero current (via CAN -- EV side)
     *   2. Open contactor (EV side) / disable output (EVSE side)
     *   3. De-assert all control signals
     *   4. Set fault flags in CAN frames
     *
     * Per CHAdeMO spec, the EVSE must cease current within 100ms of
     * detecting a fault. The EV opens contactors when current reaches zero.
     *
     * We use a mini sub-state machine here to ensure sequencing:
     *   Phase 0 (0-50ms):  Set current=0, set fault flags
     *   Phase 1 (50-100ms): Open contactor / stop output
     *   Phase 2 (100-200ms): De-assert control lines
     *   Phase 3 (200ms+):    Go to STOPPED
     * ================================================================= */
    case CHADEMO_STATE_FAULT_SHUTDOWN: {
        if (ctx->state_entry_ms < 50) {
            /* Phase 0: Zero current request, set fault flags */
            CHADEMO_LOG("FAULT_SHUTDOWN: phase 0 (zero current), reason=%s",
                        chademo_fsm_fault_name(ctx->fault_reason));
#if IS_VEHICLE
            ctx->tx.h102.charging_current_request_A = 0;
            ctx->tx.h102.fault |= CHADEMO_EV_FAULT_BATTERY_VOLTAGE_DEV;
            ctx->tx.h102.status |= CHADEMO_EV_STATUS_CHARGING_SYS_FAULT;
#else
            ctx->tx.h109.present_output_current_A = 0;
            ctx->tx.h109.status |= CHADEMO_SE_STATUS_CHARGER_MALFUNCTION;
            ctx->tx.h109.status &= ~CHADEMO_SE_STATUS_CHARGING;
#endif
        } else if (ctx->state_entry_ms < 100) {
            /* Phase 1: Open contactor (EV) or disable output (EVSE) */
            hal_gpio_set_contactor(false);
            ctx->contactor_closed = false;
        } else if (ctx->state_entry_ms < 200) {
            /* Phase 2: De-assert all control signals */
#if IS_VEHICLE
            ctx->tx.h102.status |= CHADEMO_EV_STATUS_CONTACTOR_OPEN;
            hal_gpio_set_dcp(false);
#else
            hal_gpio_set_ss1(false);
            hal_gpio_set_ss2(false);
            ctx->tx.h109.status &= ~CHADEMO_SE_STATUS_CONNECTOR_LOCKED;
#endif
        } else {
            /* Phase 3: Enter STOPPED */
            transition_to(ctx, CHADEMO_STATE_STOPPED);
            event = CHADEMO_EVENT_FAULT;
        }
        break;
    }

    /* =================================================================
     * STATE: STOPPED -- Charge complete or faulted, waiting for unplug
     * =================================================================
     * Safe state. Contactor is open. No high voltage on the connector.
     * We remain here until the plug is removed, then return to IDLE.
     *
     * VEHICLE: Wait for PP to go HIGH (plug removed).
     * STATION: Wait for DCP to go LOW (vehicle disconnected).
     * ================================================================= */
    case CHADEMO_STATE_STOPPED: {
#if IS_VEHICLE
        /* Stop sending periodic CAN frames */
        /* (Application controls this via pull_can_tx calls) */

        /* Wait for plug removal (PP goes HIGH) */
        if (hal_gpio_read_pp()) {
            /* Plug removed -- fully reset and return to IDLE */
            CHADEMO_LOG("STOPPED: PP HIGH, plug removed -> IDLE");
            chademo_fsm_init(ctx);
            event = CHADEMO_EVENT_UNPLUGGED;
        }
#else
        /* EVSE: Wait for vehicle to disconnect (DCP goes LOW) */
        if (!hal_gpio_read_dcp()) {
            CHADEMO_LOG("STOPPED: DCP LOW, vehicle disconnected -> IDLE");
            chademo_fsm_init(ctx);
            event = CHADEMO_EVENT_UNPLUGGED;
        }
#endif
        break;
    }

    default:
        /* Invalid state -- emergency shutdown */
        enter_fault(ctx, CHADEMO_FAULT_OTHER);
        break;
    }

    return event;
}

/* ============================================================================
 * DIAGNOSTIC STRING FUNCTIONS
 * ============================================================================ */

const char *chademo_fsm_state_name(chademo_state_t state)
{
    switch (state) {
    case CHADEMO_STATE_IDLE:               return "IDLE";
    case CHADEMO_STATE_PLUG_DETECTED:      return "PLUG_DETECTED";
    case CHADEMO_STATE_HANDSHAKE:          return "HANDSHAKE";
    case CHADEMO_STATE_PARAM_CHECK:        return "PARAM_CHECK";
    case CHADEMO_STATE_INSULATION_TEST:    return "INSULATION_TEST";
    case CHADEMO_STATE_PRECHARGE:          return "PRECHARGE";
    case CHADEMO_STATE_CHARGING:           return "CHARGING";
    case CHADEMO_STATE_SHUTDOWN:           return "SHUTDOWN";
    case CHADEMO_STATE_WAIT_ZERO_CURRENT:  return "WAIT_ZERO_CURRENT";
    case CHADEMO_STATE_CONTACTOR_OPEN:     return "CONTACTOR_OPEN";
    case CHADEMO_STATE_FAULT_SHUTDOWN:     return "FAULT_SHUTDOWN";
    case CHADEMO_STATE_STOPPED:            return "STOPPED";
    default:                               return "UNKNOWN";
    }
}

const char *chademo_fsm_fault_name(chademo_fault_reason_t fault)
{
    switch (fault) {
    case CHADEMO_FAULT_NONE:                return "NONE";
    case CHADEMO_FAULT_CAN_TIMEOUT:         return "CAN_TIMEOUT";
    case CHADEMO_FAULT_VOLTAGE_MISMATCH:    return "VOLTAGE_MISMATCH";
    case CHADEMO_FAULT_CURRENT_MISMATCH:    return "CURRENT_MISMATCH";
    case CHADEMO_FAULT_BATTERY_OVERVOLTAGE: return "BATTERY_OVERVOLTAGE";
    case CHADEMO_FAULT_BATTERY_UNDERVOLTAGE:return "BATTERY_UNDERVOLTAGE";
    case CHADEMO_FAULT_BATTERY_OVERTEMP:    return "BATTERY_OVERTEMP";
    case CHADEMO_FAULT_INSULATION_FAIL:     return "INSULATION_FAIL";
    case CHADEMO_FAULT_CHARGER_MALFUNCTION: return "CHARGER_MALFUNCTION";
    case CHADEMO_FAULT_INCOMPATIBLE:        return "INCOMPATIBLE";
    case CHADEMO_FAULT_ESTOP:               return "ESTOP";
    case CHADEMO_FAULT_CONTACTOR_WELD:      return "CONTACTOR_WELD";
    case CHADEMO_FAULT_UNPLUG_UNDER_LOAD:   return "UNPLUG_UNDER_LOAD";
    case CHADEMO_FAULT_PP_LOST:             return "PP_LOST";
    case CHADEMO_FAULT_OTHER:               return "OTHER";
    default:                                return "UNKNOWN";
    }
}
