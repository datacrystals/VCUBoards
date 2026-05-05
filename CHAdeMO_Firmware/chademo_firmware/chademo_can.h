/**
 * @file  chademo_can.h
 * @brief CHAdeMO CAN message definitions and protocol structures
 * @details
 *   This module defines the complete CAN message payload structures for
 *   CHAdeMO communication per IEEE Std 2030.1.1-2015 / IEC 61851-24.
 *   It includes packing/unpacking functions for all standard frame IDs.
 *
 *   Based on verified structures from:
 *   - furdog/chademo (generic hardware-agnostic CHAdeMO library)
 *   - jamiejones85/ESP32-Chademo (real-world EV conversion firmware)
 *   - Isaac96/CHAdeMOSoftware (JLD505-based Arduino Due implementation)
 *
 * @ingroup CAN
 */

#ifndef CHADEMO_CAN_H
#define CHADEMO_CAN_H

#include <stdint.h>
#include <stdbool.h>
#include "chademo_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * GENERIC CAN FRAME TYPE
 * ============================================================================ */

/** Simplified CAN 2.0B frame — maps directly to MCP2515 RX/TX buffers. */
typedef struct {
    uint32_t id;       /**< 11-bit standard identifier */
    uint8_t  len;      /**< Data length code (0-8) */
    uint8_t  data[8];  /**< Payload bytes */
} chademo_can_frame_t;

/* ============================================================================
 * VEHICLE-SIDE CAN FRAME DEFINITIONS (EV Tx)
 * ============================================================================
 * The vehicle transmits three periodic messages at 100ms intervals:
 *   0x100 — Battery maximum voltage / pack capacity
 *   0x101 — Charge timing parameters
 *   0x102 — Control request (target V/I, faults, status)
 */

/** --- CAN ID 0x100: Vehicle Charging Parameters ---
 *  Byte:  0      1      2      3      4         5         6       7
 *        [Reserved-------------]  [MaxBattV (LSB)]  [MaxBattV (MSB)]  [Cap]  [Resv]
 *  Scale: max_battery_voltage = 1 V/bit, charged_rate_ref_const = 1%/bit
 */
typedef struct {
    uint16_t max_battery_voltage_V;     /**< Maximum battery voltage (1 V/bit) */
    uint8_t  charged_rate_ref_const;    /**< Pack capacity reference, 100% = 0x64 */
} __attribute__((packed)) chademo_ev_h100_t;

/** --- CAN ID 0x101: Vehicle Charge Timing ---
 *  Byte:  0      1              2              3              4   5   6   7
 *        [Resv] [MaxTime_10s]  [MaxTime_60s]  [EstTime_60s]  [Resv] [TotCap_LSB] [TotCap_MSB] [Resv]
 *  Scale: max_charge_time_10s = 10 s/bit, max_charge_time_60s = 60 s/bit
 *         total_capacity = 0.1 kWh/bit (0-6553.5 kWh range)
 */
typedef struct {
    uint8_t  max_charge_time_10s;       /**< Maximum charge time, 10 sec/bit */
    uint8_t  max_charge_time_60s;       /**< Maximum charge time, 60 sec/bit */
    uint8_t  est_charge_time_60s;       /**< Estimated charge time, 60 sec/bit */
    uint16_t total_capacity_100wh;      /**< Battery capacity, 0.1 kWh/bit */
} __attribute__((packed)) chademo_ev_h101_t;

/** --- CAN ID 0x102: Vehicle Control Request ---
 *  Byte:  0             1                  2                 3               4       5        6       7
 *        [Protocol#]  [TargetV (LSB)]  [TargetV (MSB)]  [CurrentReq]  [Fault]  [Status]  [SOC%]  [Resv]
 *  Scale: target_battery_voltage = 1 V/bit, charging_current_request = 1 A/bit
 */

/* Fault flags (Byte 4 of 0x102) — (0=normal, 1=fault) */
#define CHADEMO_EV_FAULT_BATTERY_OVERVOLTAGE    0x01U
#define CHADEMO_EV_FAULT_BATTERY_UNDERVOLTAGE   0x02U
#define CHADEMO_EV_FAULT_BATTERY_CURRENT_DEV    0x04U
#define CHADEMO_EV_FAULT_BATTERY_HIGH_TEMP      0x08U
#define CHADEMO_EV_FAULT_BATTERY_VOLTAGE_DEV    0x10U

/* Status flags (Byte 5 of 0x102) */
#define CHADEMO_EV_STATUS_CHARGING_ENABLED      0x01U  /* 0=disabled, 1=enabled */
#define CHADEMO_EV_STATUS_SHIFT_NOT_PARKED      0x02U  /* 0=parked, 1=other */
#define CHADEMO_EV_STATUS_CHARGING_SYS_FAULT    0x04U  /* 0=normal, 1=fault */
#define CHADEMO_EV_STATUS_CONTACTOR_OPEN        0x08U  /* 0=closed, 1=open */
#define CHADEMO_EV_STATUS_NORMAL_STOP_REQUEST   0x10U  /* 0=no, 1=stop request */

/* Protocol numbers (Byte 0 of 0x102 / Byte 0 of 0x109) */
#define CHADEMO_PROTOCOL_PRE_0_9    0U
#define CHADEMO_PROTOCOL_V0_9       1U
#define CHADEMO_PROTOCOL_V1_0       2U

typedef struct {
    uint8_t  protocol_number;           /**< CHAdeMO protocol version */
    uint16_t target_battery_voltage_V;  /**< Requested voltage (1 V/bit) */
    uint8_t  charging_current_request_A;/**< Requested current (1 A/bit) */
    uint8_t  fault;                     /**< Bitmap of CHADEMO_EV_FAULT_* */
    uint8_t  status;                    /**< Bitmap of CHADEMO_EV_STATUS_* */
    uint8_t  charged_rate_percent;      /**< SOC in percent (1%/bit) */
} __attribute__((packed)) chademo_ev_h102_t;

/* ============================================================================
 * CHARGER-SIDE (EVSE) CAN FRAME DEFINITIONS (SE Tx)
 * ============================================================================
 * The charger transmits two periodic messages at 100ms intervals:
 *   0x108 — Available output capability
 *   0x109 — Present output and system status
 */

/** --- CAN ID 0x108: Charger Available Output ---
 *  Byte:  0             1                  2                 3            4         5         6    7
 *        [WeldDetect] [AvailV (LSB)]  [AvailV (MSB)]  [AvailCurrent]  [ThreshV_LSB] [ThreshV_MSB] [Resv] [Resv]
 *  Scale: avail_output_voltage = 1 V/bit, avail_output_current = 1 A/bit
 */
typedef struct {
    bool     welding_detection_support; /**< true = supports weld check */
    uint16_t avail_output_voltage_V;    /**< Maximum output voltage (1 V/bit) */
    uint8_t  avail_output_current_A;    /**< Maximum output current (1 A/bit) */
    uint16_t threshold_voltage_V;       /**< Over-voltage protection threshold */
} __attribute__((packed)) chademo_se_h108_t;

/** --- CAN ID 0x109: Charger System Status ---
 *  Byte:  0             1                  2                 3          4      5         6           7
 *        [Protocol#]  [PresV (LSB)]   [PresV (MSB)]   [PresCurrent]  [Resv]  [Status]  [RemTime10s] [RemTime60s]
 *  Scale: present_output_voltage = 1 V/bit, present_output_current = 1 A/bit
 *         remaining_charge_time_10s = 10 s/bit, remaining_charge_time_60s = 60 s/bit
 */

/* Status flags (Byte 5 of 0x109) */
#define CHADEMO_SE_STATUS_CHARGING              0x01U  /* bit 0: 0=standby, 1=charging */
#define CHADEMO_SE_STATUS_CHARGER_MALFUNCTION   0x02U  /* bit 1: 0=normal, 1=fault */
#define CHADEMO_SE_STATUS_CONNECTOR_LOCKED      0x04U  /* bit 2: 0=open, 1=locked */
#define CHADEMO_SE_STATUS_BATTERY_INCOMPATIBLE  0x08U  /* bit 3: 0=compatible, 1=incompatible */
#define CHADEMO_SE_STATUS_SYS_MALFUNCTION       0x10U  /* bit 4: 0=normal, 1=malfunction */
#define CHADEMO_SE_STATUS_STOP_CONTROL          0x20U  /* bit 5: 0=operating, 1=stopped */

typedef struct {
    uint8_t  protocol_number;           /**< CHAdeMO protocol version */
    uint16_t present_output_voltage_V;  /**< Actual output voltage (1 V/bit) */
    uint8_t  present_output_current_A;  /**< Actual output current (1 A/bit) */
    uint8_t  status;                    /**< Bitmap of CHADEMO_SE_STATUS_* */
    uint8_t  remaining_charge_time_10s; /**< Remaining time, 10 sec/bit (0xFF=invalid) */
    uint8_t  remaining_charge_time_60s; /**< Remaining time, 60 sec/bit (0xFF=invalid) */
} __attribute__((packed)) chademo_se_h109_t;

/* ============================================================================
 * COMBINED RX STATE BUFFERS
 * ============================================================================
 * These structures hold the unpacked received data from the opposite side.
 */

typedef struct {
    chademo_ev_h100_t h100;     /**< Unpacked 0x100 */
    chademo_ev_h101_t h101;     /**< Unpacked 0x101 */
    chademo_ev_h102_t h102;     /**< Unpacked 0x102 */
    bool              h100_valid;   /**< True if 0x100 received recently */
    bool              h101_valid;   /**< True if 0x101 received recently */
    bool              h102_valid;   /**< True if 0x102 received recently */
    uint32_t          last_rx_ms;   /**< Timestamp of last valid frame */
} chademo_rx_vehicle_t;

typedef struct {
    chademo_se_h108_t h108;     /**< Unpacked 0x108 */
    chademo_se_h109_t h109;     /**< Unpacked 0x109 */
    bool              h108_valid;   /**< True if 0x108 received recently */
    bool              h109_valid;   /**< True if 0x109 received recently */
    uint32_t          last_rx_ms;   /**< Timestamp of last valid frame */
} chademo_rx_charger_t;

/* ============================================================================
 * COMBINED TX STATE BUFFERS (what WE transmit)
 * ============================================================================
 * Both types are always defined so the CAN pack/unpack API is complete
 * regardless of compile-time role. The FSM uses only the relevant one.
 */

typedef struct {
    chademo_ev_h100_t h100;
    chademo_ev_h101_t h101;
    chademo_ev_h102_t h102;
} chademo_tx_vehicle_t;

typedef struct {
    chademo_se_h108_t h108;
    chademo_se_h109_t h109;
} chademo_tx_charger_t;

/* ============================================================================
 * PUBLIC API
 * ============================================================================ */

/** Initialize RX buffers to safe defaults */
void chademo_can_init_rx_vehicle(chademo_rx_vehicle_t *rx);
void chademo_can_init_rx_charger(chademo_rx_charger_t *rx);

/** Pack local structs into CAN frames for transmission */
void chademo_can_pack_vehicle_frames(const chademo_tx_vehicle_t *tx,
                                      chademo_can_frame_t frames[3]);
void chademo_can_pack_charger_frames(const chademo_tx_charger_t *tx,
                                      chademo_can_frame_t frames[2]);

/** Unpack received CAN frames into local structs. Returns true if consumed. */
bool chademo_can_unpack_vehicle_frame(chademo_rx_vehicle_t *rx,
                                       const chademo_can_frame_t *frame);
bool chademo_can_unpack_charger_frame(chademo_rx_charger_t *rx,
                                       const chademo_can_frame_t *frame);

/** Check if all required periodic frames have been received (link health) */
static inline bool chademo_can_vehicle_link_ok(const chademo_rx_vehicle_t *rx)
{
    return (rx->h100_valid && rx->h101_valid && rx->h102_valid);
}

static inline bool chademo_can_charger_link_ok(const chademo_rx_charger_t *rx)
{
    return (rx->h108_valid && rx->h109_valid);
}

#ifdef __cplusplus
}
#endif

#endif /* CHADEMO_CAN_H */
