/**
 * @file  chademo_config.h
 * @brief Compile-time configuration and GPIO pin mapping for CHAdeMO controller
 * @details
 *   This file contains all board-specific constants, the compile-time role
 *   selector (Vehicle vs Station), and the GPIO pin mapping for the
 *   Raspberry Pi Pico (RP2040). All other modules include this file.
 *
 * @note Modify ONLY this file to adapt to a different PCB layout.
 */

#ifndef CHADEMO_CONFIG_H
#define CHADEMO_CONFIG_H

/* ============================================================================
 * COMPILE-TIME ROLE SELECTION
 * ============================================================================
 * Define exactly ONE of the following macros before including this header,
 * or uncomment the desired role below.
 */

/* Uncomment the line matching your target role: */
// #define CHADEMO_ROLE_VEHICLE   /* Build for EV (vehicle side) */
// #define CHADEMO_ROLE_STATION   /* Build for EVSE (charging station side) */

/* Role validation — must define exactly one */
#if defined(CHADEMO_ROLE_VEHICLE) && defined(CHADEMO_ROLE_STATION)
#error "Cannot define both CHADEMO_ROLE_VEHICLE and CHADEMO_ROLE_STATION"
#elif !defined(CHADEMO_ROLE_VEHICLE) && !defined(CHADEMO_ROLE_STATION)
#error "Must define CHADEMO_ROLE_VEHICLE or CHADEMO_ROLE_STATION"
#endif

/* Convenience role booleans for use in state machine logic */
#ifdef CHADEMO_ROLE_VEHICLE
#define IS_VEHICLE   1
#define IS_STATION   0
#define FSM_ROLE_STR "VEHICLE"
#else
#define IS_VEHICLE   0
#define IS_STATION   1
#define FSM_ROLE_STR "STATION"
#endif

/* ============================================================================
 * CHAdeMO PROTOCOL CONSTANTS (IEC 61851-23 / CHAdeMO 2.0)
 * ============================================================================ */

#define CHADEMO_CAN_BITRATE_KBPS    500U   /* CHAdeMO fixed CAN bitrate */

/* CAN Arbitration IDs — Vehicle Tx */
#define CAN_ID_VEH_CHARGER_STATUS   0x100U /* Charger status & target values */
#define CAN_ID_VEH_VEHICLE_STATUS   0x101U /* Vehicle status & present values */
#define CAN_ID_VEH_BATTERY_INFO     0x102U /* Battery SOC / capacity info */
#define CAN_ID_VEH_BATTERY_REQ      0x103U /* Battery charge request */
#define CAN_ID_VEH_BATTERY_STATUS   0x104U /* Battery status flags */
#define CAN_ID_VEH_CHARGING_TIME    0x105U /* Remaining charging time */
#define CAN_ID_VEH_MANUF_INFO       0x106U /* Vehicle manufacturer info */
#define CAN_ID_VEH_PROTOCOL_VER     0x700U /* Protocol version (handshake) */

/* CAN Arbitration IDs — Charger (EVSE) Tx */
#define CAN_ID_CHRG_AVAILABLE       0x108U /* Charger available output */
#define CAN_ID_CHRG_SYSTEM_STATUS   0x109U /* Charger system status */
#define CAN_ID_CHRG_MANUF_INFO      0x10AU /* Charger manufacturer info */
#define CAN_ID_CHRG_PROTOCOL_VER    0x701U /* Charger protocol version */

/* CAN Arbitration IDs — Diagnostics */
#define CAN_ID_DIAG_REQUEST         0x7DFU /* UDS/ISO-15765 diagnostic request */
#define CAN_ID_DIAG_RESPONSE        0x7E8U /* Diagnostic response (ECU) */

/* Timing constants (milliseconds) */
#define CHADEMO_Timing_CAN_TIMEOUT_MS      1000U  /* CAN comms watchdog timeout */
#define CHADEMO_Timing_HANDSHAKE_TIMEOUT_MS 300000U /* 5 min: allow time for auth/payment */
#define CHADEMO_Timing_INSULATION_HOLD_MS   5000U /* Insulation test dwell */
#define CHADEMO_Timing_PRECHARGE_TIMEOUT_MS 10000U /* Pre-charge to threshold */
#define CHADEMO_Timing_SHUTDOWN_RAMP_MS    5000U  /* Normal shutdown ramp */
#define CHADEMO_Timing_ESTOP_CONTACTOR_MS  100U   /* Emergency contactor open */
#define CHADEMO_Timing_PLUG_DEBOUNCE_MS    200U   /* Mechanical debounce */
#define CHADEMO_Timing_HEARTBEAT_MS        100U   /* Main loop tick rate */
#define CHADEMO_Timing_WD_FEED_MS          50U    /* External watchdog feed */

/* Voltage / Current safety thresholds */
#define CHADEMO_MAX_VOLTAGE_V       500U   /* Absolute max pack voltage */
#define CHADEMO_MAX_CURRENT_A       200U   /* Absolute max charge current */
#define CHADEMO_PRECHARGE_THRESHOLD_V 50U  /* Min voltage diff for pre-charge OK */
#define CHADEMO_CONTACTOR_CLOSE_MIN_V 100U /* Min system voltage to close contactor */

/* ============================================================================
 * GPIO PIN MAPPING — Raspberry Pi Pico (RP2040)
 * ============================================================================
 * These map directly to the Pico SDK GPIO numbers. All other modules
 * reference these symbolic names — NEVER hardcode a GPIO number elsewhere.
 */

/* --- CAN Bus 1 (SPI0) — Primary vehicle-side or charger-side bus --- */
#define PIN_CAN1_SPI_SCK            2U   /* SPI0 SCK  */
#define PIN_CAN1_SPI_TX             3U   /* SPI0 TX (MOSI) */
#define PIN_CAN1_SPI_RX             4U   /* SPI0 RX (MISO) */
#define PIN_CAN1_CS                 6U   /* Chip select for MCP2515 #1 */
#define PIN_CAN1_INT                5U   /* Interrupt from MCP2515 #1 */

/* --- CAN Bus 2 (SPI1) — Secondary / monitoring bus --- */
#define PIN_CAN2_SPI_SCK            10U  /* SPI1 SCK  */
#define PIN_CAN2_SPI_TX             11U  /* SPI1 TX (MOSI) */
#define PIN_CAN2_SPI_RX             12U  /* SPI1 RX (MISO) */
#define PIN_CAN2_CS                 14U  /* Chip select for MCP2515 #2 */
#define PIN_CAN2_INT                13U  /* Interrupt from MCP2515 #2 */

/* --- Charger Role (EVSE) Control Lines --- */
#define PIN_OUT_SS1                 7U   /* Output: Charger Inlet Lock Signal 1 */
#define PIN_OUT_SS2                 8U   /* Output: Charger Inlet Lock Signal 2 */
#define PIN_IN_DCP                  9U   /* Input:  Charger DC Present detect */

/* --- Vehicle Role Control Lines --- */
#define PIN_IN_SS1                  16U  /* Input:  Vehicle inlet lock #1 */
#define PIN_IN_SS2                  17U  /* Input:  Vehicle inlet lock #2 */
#define PIN_IN_PP                   18U  /* Input:  Proximity Pilot (plug detect) */
#define PIN_OUT_DCP                 19U  /* Output: Vehicle DC Present acknowledge */

/* --- Common / Shared --- */
#define PIN_CONTACTOR_OUT           15U  /* Output: Contactor coil driver (active HIGH) */

/* ============================================================================
 * MCP2515 HARDWARE CONFIGURATION
 * ============================================================================
 * Most MCP2515 modules use an 8MHz crystal. Some use 16MHz.
 * This affects the bit timing registers for 500 kbps.
 * Set this to match your actual hardware.
 */
#define MCP2515_OSC_MHZ             8   /* 8 or 16 — check your module crystal! */

/* ============================================================================
 * MCP2515 SPI INTERFACE CONFIGURATION
 * ============================================================================ */
#define MCP2515_SPI_PORT            spi0   /* CAN1 uses SPI0 */
#define MCP2515_SPI_BAUD_HZ         10000000 /* 10 MHz SPI clock to MCP2515 */

#define MCP2515_SPI_PORT_2          spi1   /* CAN2 uses SPI1 */
#define MCP2515_SPI_BAUD_HZ_2       10000000 /* 10 MHz SPI clock to MCP2515 #2 */

/* MCP2515 instruction set */
#define MCP2515_CMD_RESET           0xC0U
#define MCP2515_CMD_READ            0x03U
#define MCP2515_CMD_WRITE           0x02U
#define MCP2515_CMD_BIT_MODIFY      0x05U
#define MCP2515_CMD_READ_STATUS     0xA0U
#define MCP2515_CMD_RX_STATUS       0xB0U

/* MCP2515 register addresses */
#define MCP2515_REG_CANSTAT         0x0EU
#define MCP2515_REG_CANCTRL         0x0FU
#define MCP2515_REG_CNF1            0x2AU
#define MCP2515_REG_CNF2            0x29U
#define MCP2515_REG_CNF3            0x28U
#define MCP2515_REG_CANINTE         0x2BU
#define MCP2515_REG_CANINTF         0x2CU
#define MCP2515_REG_EFLG            0x2DU
#define MCP2515_REG_TXB0CTRL        0x30U
#define MCP2515_REG_TXB0SIDH        0x31U
#define MCP2515_REG_TXB0SIDL        0x32U
#define MCP2515_REG_TXB0DLC         0x35U
#define MCP2515_REG_TXB0D0          0x36U
#define MCP2515_REG_RXB0CTRL        0x60U
#define MCP2515_REG_RXB0SIDH        0x61U
#define MCP2515_REG_RXB0SIDL        0x62U
#define MCP2515_REG_RXB0DLC         0x65U
#define MCP2515_REG_RXB0D0          0x66U
#define MCP2515_REG_RXB1CTRL        0x70U
#define MCP2515_REG_RXB1SIDH        0x71U

/* MCP2515 configuration mode + CLKOUT */
#define MCP2515_MODE_CONFIG         0x80U
#define MCP2515_MODE_NORMAL         0x00U
#define MCP2515_MODE_LOOPBACK       0x40U
#define MCP2515_CLKOUT_PS1          0x03U  /* CLKOUT = OSC1/1 */

/* Interrupt flags */
#define MCP2515_INT_RX0             0x01U
#define MCP2515_INT_RX1             0x02U
#define MCP2515_INT_TX0             0x04U
#define MCP2515_INT_ERR             0x20U
#define MCP2515_INT_WAK             0x40U
#define MCP2515_INT_MERR            0x80U

/* ============================================================================
 * DEBUG LOGGING
 * ============================================================================
 * Enable printf-style debug output over USB serial.
 * Set to 0 to disable all debug logging for production builds.
 */
#define CHADEMO_DEBUG               1

#if CHADEMO_DEBUG
#include <stdio.h>
#define CHADEMO_LOG(fmt, ...) printf("[CHADEMO] " fmt "\r\n", ##__VA_ARGS__)
#else
#define CHADEMO_LOG(fmt, ...) ((void)0)
#endif

/* ============================================================================
 * BUILD INFO
 * ============================================================================ */
#define FIRMWARE_VERSION_MAJOR      1
#define FIRMWARE_VERSION_MINOR      0
#define FIRMWARE_VERSION_PATCH      0

#endif /* CHADEMO_CONFIG_H */
