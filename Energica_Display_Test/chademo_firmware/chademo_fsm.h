/**
 * @file  chademo_fsm.h
 * @brief CHAdeMO Protocol State Machine
 * @details
 *   Implements the complete CHAdeMO charging sequence as a non-blocking
 *   finite state machine. The machine adapts at compile time to operate
 *   as either the VEHICLE (EV) or STATION (EVSE) role.
 *
 *   STATE OVERVIEW (both roles):
 *   =============================
 *
 *   [IDLE]  ->  [PLUG_DETECTED]
 *        Physical plug insertion detected via proximity pilot / SS lines.
 *        Debounce applied to prevent false triggers.
 *
 *   [PLUG_DETECTED]  ->  [HANDSHAKE]
 *        Start CAN communication. Vehicle sends 0x100/0x101/0x102.
 *        Charger sends 0x108/0x109. Exchange protocol versions and
 *        capability parameters.
 *
 *   [HANDSHAKE]  ->  [PARAM_CHECK]
 *        Validate compatibility: max battery voltage vs. available output,
 *        current capabilities, protocol version match.
 *        EVSE checks EV parameters; EV checks charger parameters.
 *
 *   [PARAM_CHECK]  ->  [INSULATION_TEST]
 *        EVSE only: Perform insulation resistance test on DC output.
 *        Verify output terminals are at safe voltage (<10V) before test.
 *        Vehicle waits in a sub-state during this phase.
 *
 *   [INSULATION_TEST]  ->  [PRECHARGE]
 *        EVSE ramps output voltage to match battery voltage.
 *        EV monitors voltage difference; when within threshold,
 *        EV closes contactors.
 *
 *   [PRECHARGE]  ->  [CHARGING]
 *        EV requests current via 0x102. EVSE delivers controlled
 *        voltage/current. Both sides monitor for faults.
 *        Dynamic current control: EV can change request by +/-20A/sec.
 *
 *   [CHARGING]  ->  [SHUTDOWN]
 *        Normal termination: EV requests 0A, EVSE ramps down current,
 *        EV opens contactors when current reaches zero.
 *
 *   FAULT PATH (any state):
 *   [ANY]  ->  [FAULT_SHUTDOWN]
 *        Emergency path: Immediate current cessation, contactor open,
 *        safe GPIO states. Entered on: CAN timeout, insulation failure,
 *        voltage/current mismatch, overtemperature, estop.
 *
 *   References:
 *   - IEEE Std 2030.1.1-2015 (CHAdeMO Protocol)
 *   - furdog/chademo: Generic hardware-agnostic CHAdeMO library
 *   - jamiejones85/ESP32-Chademo: Real-world EV-side implementation
 */

#ifndef CHADEMO_FSM_H
#define CHADEMO_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "chademo_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * STATE DEFINITIONS
 * ============================================================================ */

typedef enum {
    /* --- Idle / Waiting --- */
    CHADEMO_STATE_IDLE = 0,

    /* --- Plug Detection --- */
    CHADEMO_STATE_PLUG_DETECTED,

    /* --- Handshake: CAN parameter exchange --- */
    CHADEMO_STATE_HANDSHAKE,

    /* --- Parameter Compatibility Check --- */
    CHADEMO_STATE_PARAM_CHECK,

    /* --- Insulation Test (EVSE) / Wait for EVSE (EV) --- */
    CHADEMO_STATE_INSULATION_TEST,

    /* --- Pre-charge: voltage matching --- */
    CHADEMO_STATE_PRECHARGE,

    /* --- Active Charging --- */
    CHADEMO_STATE_CHARGING,

    /* --- Normal Shutdown Sequence --- */
    CHADEMO_STATE_SHUTDOWN,

    /* --- Wait for Zero Current before contactor open --- */
    CHADEMO_STATE_WAIT_ZERO_CURRENT,

    /* --- Contactor Open / Cleanup --- */
    CHADEMO_STATE_CONTACTOR_OPEN,

    /* --- Fault / Emergency Shutdown --- */
    CHADEMO_STATE_FAULT_SHUTDOWN,

    /* --- Fully stopped, waiting for unplug --- */
    CHADEMO_STATE_STOPPED

} chademo_state_t;

/* ============================================================================
 * EVENT DEFINITIONS (outputs from the FSM to the application)
 * ============================================================================ */

typedef enum {
    CHADEMO_EVENT_NONE = 0,
    CHADEMO_EVENT_PLUG_INSERTED,       /* Physical plug detected */
    CHADEMO_EVENT_HANDSHAKE_COMPLETE,  /* CAN params exchanged */
    CHADEMO_EVENT_PARAMS_COMPATIBLE,   /* Compatibility check passed */
    CHADEMO_EVENT_INSULATION_OK,       /* Insulation test passed */
    CHADEMO_EVENT_PRECHARGE_OK,        /* Voltage matched, ready to charge */
    CHADEMO_EVENT_CHARGING_STARTED,    /* Current flowing */
    CHADEMO_EVENT_CHARGE_COMPLETE,     /* Normal termination */
    CHADEMO_EVENT_FAULT,               /* Any fault condition */
    CHADEMO_EVENT_UNPLUGGED            /* Plug removed */
} chademo_event_t;

/* ============================================================================
 * FAULT REASON CODES
 * ============================================================================ */

typedef enum {
    CHADEMO_FAULT_NONE = 0,
    CHADEMO_FAULT_CAN_TIMEOUT,         /* No CAN frames received */
    CHADEMO_FAULT_VOLTAGE_MISMATCH,    /* |V_charger - V_batt| too large */
    CHADEMO_FAULT_CURRENT_MISMATCH,    /* |I_reported - I_measured| too large */
    CHADEMO_FAULT_BATTERY_OVERVOLTAGE, /* Battery voltage exceeded limit */
    CHADEMO_FAULT_BATTERY_UNDERVOLTAGE,/* Battery voltage below limit */
    CHADEMO_FAULT_BATTERY_OVERTEMP,    /* Battery temperature too high */
    CHADEMO_FAULT_INSULATION_FAIL,     /* Insulation resistance too low */
    CHADEMO_FAULT_CHARGER_MALFUNCTION, /* EVSE reported fault status */
    CHADEMO_FAULT_INCOMPATIBLE,        /* Parameter incompatibility */
    CHADEMO_FAULT_ESTOP,               /* Emergency stop pressed */
    CHADEMO_FAULT_CONTACTOR_WELD,      /* Contactor welded detection */
    CHADEMO_FAULT_UNPLUG_UNDER_LOAD,   /* Plug removed while charging */
    CHADEMO_FAULT_PP_LOST,             /* Proximity pilot lost during charge */
    CHADEMO_FAULT_OTHER
} chademo_fault_reason_t;

/* ============================================================================
 * FSM CONTEXT (holds all state)
 * ============================================================================ */

typedef struct {
    /* ---- Current state ---- */
    chademo_state_t  state;           /**< Current FSM state */
    chademo_state_t  prev_state;      /**< Previous state (for diagnostics) */

    /* ---- Timing ---- */
    uint32_t         state_entry_ms;  /**< millis() when current state was entered */
    uint32_t         last_can_rx_ms;  /**< millis() of last valid CAN frame */
    uint32_t         last_tx_ms;      /**< millis() of last CAN transmission */
    uint32_t         plug_detect_ms;  /**< millis() when plug first detected */
    uint32_t         current_ramp_ms; /**< For 20A/sec current slew limit */

    /* ---- TX buffers (what WE send) ---- */
#if IS_VEHICLE
    chademo_tx_vehicle_t tx;
#else
    chademo_tx_charger_t tx;
#endif

    /* ---- RX buffers (what we RECEIVE from the other side) ---- */
#if IS_VEHICLE
    chademo_rx_charger_t rx;          /**< Unpacked charger (0x108, 0x109) */
#else
    chademo_rx_vehicle_t rx;          /**< Unpacked vehicle (0x100, 0x101, 0x102) */
#endif

    /* ---- Internal tracking ---- */
    uint8_t          tx_frame_rotate; /**< Rotates through 0x100/0x101/0x102 */
    uint8_t          fault_count;     /**< Consecutive fault frames before action */
    uint8_t          v_mismatch_count; /**< Consecutive voltage mismatches */
    uint8_t          c_mismatch_count; /**< Consecutive current mismatches */
    uint8_t          asking_amps;     /**< Ramp-managed current request */

    /* ---- Status flags ---- */
    bool             can_link_ok;     /**< All periodic frames being received */
    bool             contactor_closed;/**< True if contactor is closed */
    bool             charge_enabled;  /**< True if charge permission given */

    /* ---- Fault info ---- */
    chademo_fault_reason_t fault_reason; /**< Last fault reason */
    uint8_t                fault_status; /**< Raw fault byte from CAN */

    /* ---- Measured values (populated by application) ---- */
    uint16_t         measured_voltage_V; /**< Actual DC bus voltage */
    int16_t          measured_current_A; /**< Actual current (negative = into battery) */
    uint8_t          battery_soc;        /**< 0-100% state of charge */
    uint16_t         battery_temp_C;     /**< Battery temperature */

} chademo_context_t;

/* ============================================================================
 * PUBLIC API
 * ============================================================================ */

/** Initialize FSM context to safe defaults. Must be called once at boot. */
void chademo_fsm_init(chademo_context_t *ctx);

/**
 * @brief Execute one FSM step (non-blocking).
 * @param ctx     FSM context
 * @param dt_ms   Milliseconds since last call (for internal timing)
 * @return        Event code -- caller should handle significant events
 *
 * Call this at a regular interval (e.g., every 10ms). The function
 * reads GPIO state, processes CAN RX data, manages state transitions,
 * and prepares CAN TX data.
 */
chademo_event_t chademo_fsm_step(chademo_context_t *ctx, uint32_t dt_ms);

/** Push a received CAN frame into the FSM (call from ISR or main loop) */
void chademo_fsm_push_can_rx(chademo_context_t *ctx, const chademo_can_frame_t *frame);

/** Pull a CAN frame to transmit (call after fsm_step to send frames) */
bool chademo_fsm_pull_can_tx(chademo_context_t *ctx, chademo_can_frame_t *frame);

/** Request normal shutdown (e.g., user pressed "stop" or SOC reached 100%) */
void chademo_fsm_request_shutdown(chademo_context_t *ctx);

/** Trigger emergency stop (immediate fault shutdown) */
void chademo_fsm_trigger_estop(chademo_context_t *ctx);

/** Set measured values from ADC/sensors (call before fsm_step) */
void chademo_fsm_set_measured_voltage(chademo_context_t *ctx, uint16_t voltage_V);
void chademo_fsm_set_measured_current(chademo_context_t *ctx, int16_t current_A);
void chademo_fsm_set_battery_soc(chademo_context_t *ctx, uint8_t soc_percent);
void chademo_fsm_set_battery_temp(chademo_context_t *ctx, uint16_t temp_C);

/** Set target charge parameters (from BMS) */
void chademo_fsm_set_target_voltage(chademo_context_t *ctx, uint16_t voltage_V);
void chademo_fsm_set_target_current(chademo_context_t *ctx, uint8_t current_A);
void chademo_fsm_set_max_voltage(chademo_context_t *ctx, uint16_t max_voltage_V);
void chademo_fsm_set_capacity_kwh(chademo_context_t *ctx, uint16_t capacity_100wh);

/** Get human-readable state name (for debug/logging) */
const char *chademo_fsm_state_name(chademo_state_t state);

/** Get human-readable fault name */
const char *chademo_fsm_fault_name(chademo_fault_reason_t fault);

#ifdef __cplusplus
}
#endif

#endif /* CHADEMO_FSM_H */
