/**
 * @file  chademo_infypower.h
 * @brief InfyPower DC Module CAN Driver
 * @details
 *   Controls InfyPower DC/DC or DC/AC modules via CAN bus at 125 kbps.
 *   Protocol: Extended frames, ID = 0x02 | (0x80+Cmd) | Dest | Src
 *   Connected to CAN1 (SPI0 / MCP2515 #1) — separate from CHAdeMO bus.
 */

#ifndef CHADEMO_INFYPOWER_H
#define CHADEMO_INFYPOWER_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * INFYPOWER PROTOCOL CONSTANTS
 * ============================================================================ */

#define INFYPOWER_SRC_ADDRESS      0xF0U   /* Supervisor address */
#define INFYPOWER_DEST_BROADCAST   0x3FU   /* Broadcast to all modules */
#define INFYPOWER_MODULE_ADDR      0x00U   /* Single module address */

/* Command codes (lower 6 bits of ID byte 2) */
#define INFYPOWER_CMD_ONOFF        0x1AU   /* Switch on/off */
#define INFYPOWER_CMD_SET_OUT      0x1BU   /* Set output V/I (broadcast) */
#define INFYPOWER_CMD_SET_SINGLE   0x1CU   /* Set output (single module) */
#define INFYPOWER_CMD_READ_SYS     0x08U   /* Read system V/I */
#define INFYPOWER_CMD_READ_MOD     0x09U   /* Read module V/I */
#define INFYPOWER_CMD_READ_STATUS  0x04U   /* Read module status */

/* Output limits */
#define INFYPOWER_VOLTAGE_MIN_V    50U
#define INFYPOWER_VOLTAGE_MAX_V    500U    /* CHAdeMO limited to 500V */
#define INFYPOWER_CURRENT_MIN_A    0U
#define INFYPOWER_CURRENT_MAX_A    20U     /* Module rated max */

/* Timing */
#define INFYPOWER_HEARTBEAT_MS     200U    /* Must send < 10s to avoid timeout fault */
#define INFYPOWER_STARTUP_DELAY_MS 200U

/* ============================================================================
 * PUBLIC API
 * ============================================================================ */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize InfyPower CAN bus (CAN1 / SPI0 / MCP2515 #1)
 * @return true if MCP2515 responds and enters normal mode
 */
bool infypower_init(void);

/**
 * @brief Send ON or OFF command to InfyPower module(s)
 * @param on true = turn on, false = turn off
 */
void infypower_cmd_onoff(bool on);

/**
 * @brief Set output voltage and current (broadcast to all modules)
 * @param voltage_V Output voltage in volts (clamped 50-500V)
 * @param current_A Output current limit in amps (clamped 0-20A)
 */
void infypower_cmd_set_output(uint16_t voltage_V, uint8_t current_A);

/**
 * @brief Request system voltage/current readback
 */
void infypower_cmd_read_system(void);

/**
 * @brief Send heartbeat / periodic refresh of setpoints.
 *        Call from main loop at ~100Hz; internally rate-limits to 200ms.
 * @param voltage_V Current desired output voltage
 * @param current_A Current desired output current limit
 */
void infypower_heartbeat(uint16_t voltage_V, uint8_t current_A);

/**
 * @brief Poll for InfyPower response frames and parse them.
 *        Updates the provided pointers with latest V/I readings.
 * @param out_voltage_V Pointer to store readback voltage (0 if no new data)
 * @param out_current_A Pointer to store readback current (0 if no new data)
 * @return true if a valid system readback was received
 */
bool infypower_poll_rx(uint16_t *out_voltage_V, uint8_t *out_current_A);

/**
 * @brief Check if InfyPower CAN bus is alive
 */
bool infypower_is_alive(void);

#ifdef __cplusplus
}
#endif

#endif /* CHADEMO_INFYPOWER_H */
