/**
 * @file  chademo_can.c
 * @brief CHAdeMO CAN message pack/unpack implementations
 */

#include "chademo_can.h"
#include <string.h>

/* ============================================================================
 * RX INITIALIZATION
 * ============================================================================ */

void chademo_can_init_rx_vehicle(chademo_rx_vehicle_t *rx)
{
    memset(rx, 0, sizeof(*rx));
    rx->h100.max_battery_voltage_V  = 0;
    rx->h100.charged_rate_ref_const = 100; /* 100% default */
    rx->h101.max_charge_time_10s    = 0xFF; /* Invalid */
    rx->h101.max_charge_time_60s    = 0xFF; /* Invalid */
    rx->h101.total_capacity_100wh   = 0;
    rx->h102.protocol_number        = CHADEMO_PROTOCOL_V1_0;
    rx->h102.target_battery_voltage_V = 0;
    rx->h102.charging_current_request_A = 0;
    rx->h102.fault                  = 0;
    rx->h102.status                 = CHADEMO_EV_STATUS_CONTACTOR_OPEN;
    rx->h102.charged_rate_percent   = 0;
    rx->h100_valid = false;
    rx->h101_valid = false;
    rx->h102_valid = false;
    rx->last_rx_ms  = 0;
}

void chademo_can_init_rx_charger(chademo_rx_charger_t *rx)
{
    memset(rx, 0, sizeof(*rx));
    rx->h108.welding_detection_support = false;
    rx->h108.avail_output_voltage_V    = 0;
    rx->h108.avail_output_current_A    = 0;
    rx->h108.threshold_voltage_V       = 0;
    rx->h109.protocol_number           = CHADEMO_PROTOCOL_V1_0;
    rx->h109.present_output_voltage_V  = 0;
    rx->h109.present_output_current_A  = 0;
    rx->h109.status                    = 0;
    rx->h109.remaining_charge_time_10s = 0xFF;
    rx->h109.remaining_charge_time_60s = 0xFF;
    rx->h108_valid = false;
    rx->h109_valid = false;
    rx->last_rx_ms  = 0;
}

/* ============================================================================
 * VEHICLE-SIDE TX PACKING (EV transmits 0x100, 0x101, 0x102)
 * ============================================================================ */

void chademo_can_pack_vehicle_frames(const chademo_tx_vehicle_t *tx,
                                      chademo_can_frame_t frames[3])
{
    chademo_can_frame_t *f;

    /* --- 0x100: Battery Specs --- */
    f       = &frames[0];
    f->id   = CAN_ID_VEH_CHARGER_STATUS;  /* 0x100 */
    f->len  = 8;
    f->data[0] = 0x00; /* Reserved */
    f->data[1] = 0x00; /* Reserved */
    f->data[2] = 0x00; /* Reserved */
    f->data[3] = 0x00; /* Reserved */
    f->data[4] = (uint8_t)(tx->h100.max_battery_voltage_V & 0x00FFU);
    f->data[5] = (uint8_t)((tx->h100.max_battery_voltage_V & 0xFF00U) >> 8);
    f->data[6] = tx->h100.charged_rate_ref_const; /* Pack capacity % */
    f->data[7] = 0x00; /* Reserved */

    /* --- 0x101: Charge Timing --- */
    f       = &frames[1];
    f->id   = CAN_ID_VEH_VEHICLE_STATUS;  /* 0x101 */
    f->len  = 8;
    f->data[0] = 0x00; /* Reserved */
    f->data[1] = tx->h101.max_charge_time_10s;
    f->data[2] = tx->h101.max_charge_time_60s;
    f->data[3] = tx->h101.est_charge_time_60s;
    f->data[4] = 0x00; /* Reserved */
    f->data[5] = (uint8_t)(tx->h101.total_capacity_100wh & 0x00FFU);
    f->data[6] = (uint8_t)((tx->h101.total_capacity_100wh & 0xFF00U) >> 8);
    f->data[7] = 0x00; /* Reserved */

    /* --- 0x102: Control Request --- */
    f       = &frames[2];
    f->id   = CAN_ID_VEH_BATTERY_INFO;  /* 0x102 */
    f->len  = 8;
    f->data[0] = tx->h102.protocol_number;
    f->data[1] = (uint8_t)(tx->h102.target_battery_voltage_V & 0x00FFU);
    f->data[2] = (uint8_t)((tx->h102.target_battery_voltage_V & 0xFF00U) >> 8);
    f->data[3] = tx->h102.charging_current_request_A;
    f->data[4] = tx->h102.fault;
    f->data[5] = tx->h102.status;
    f->data[6] = tx->h102.charged_rate_percent;
    f->data[7] = 0x00; /* Reserved */
}

/* ============================================================================
 * CHARGER-SIDE TX PACKING (EVSE transmits 0x108, 0x109)
 * ============================================================================ */

void chademo_can_pack_charger_frames(const chademo_tx_charger_t *tx,
                                      chademo_can_frame_t frames[2])
{
    chademo_can_frame_t *f;

    /* --- 0x108: Available Output --- */
    f       = &frames[0];
    f->id   = CAN_ID_CHRG_AVAILABLE;  /* 0x108 */
    f->len  = 8;
    f->data[0] = tx->h108.welding_detection_support ? 1 : 0;
    f->data[1] = (uint8_t)(tx->h108.avail_output_voltage_V & 0x00FFU);
    f->data[2] = (uint8_t)((tx->h108.avail_output_voltage_V & 0xFF00U) >> 8);
    f->data[3] = tx->h108.avail_output_current_A;
    f->data[4] = (uint8_t)(tx->h108.threshold_voltage_V & 0x00FFU);
    f->data[5] = (uint8_t)((tx->h108.threshold_voltage_V & 0xFF00U) >> 8);
    f->data[6] = 0x00; /* Reserved */
    f->data[7] = 0x00; /* Reserved */

    /* --- 0x109: System Status --- */
    f       = &frames[1];
    f->id   = CAN_ID_CHRG_SYSTEM_STATUS;  /* 0x109 */
    f->len  = 8;
    f->data[0] = tx->h109.protocol_number;
    f->data[1] = (uint8_t)(tx->h109.present_output_voltage_V & 0x00FFU);
    f->data[2] = (uint8_t)((tx->h109.present_output_voltage_V & 0xFF00U) >> 8);
    f->data[3] = tx->h109.present_output_current_A;
    f->data[4] = 0x00; /* Reserved */
    f->data[5] = tx->h109.status;
    f->data[6] = tx->h109.remaining_charge_time_10s;
    f->data[7] = tx->h109.remaining_charge_time_60s;
}

/* ============================================================================
 * VEHICLE-SIDE RX UNPACKING (we are charger, receiving from EV)
 * ============================================================================ */

bool chademo_can_unpack_vehicle_frame(chademo_rx_vehicle_t *rx,
                                       const chademo_can_frame_t *frame)
{
    bool consumed = true;

    switch (frame->id) {
    case CAN_ID_VEH_CHARGER_STATUS: /* 0x100 */
        rx->h100.max_battery_voltage_V  = (uint16_t)frame->data[4]
                                         | ((uint16_t)frame->data[5] << 8);
        rx->h100.charged_rate_ref_const = frame->data[6];
        rx->h100_valid = true;
        break;

    case CAN_ID_VEH_VEHICLE_STATUS: /* 0x101 */
        rx->h101.max_charge_time_10s    = frame->data[1];
        rx->h101.max_charge_time_60s    = frame->data[2];
        rx->h101.est_charge_time_60s    = frame->data[3];
        rx->h101.total_capacity_100wh   = (uint16_t)frame->data[5]
                                         | ((uint16_t)frame->data[6] << 8);
        rx->h101_valid = true;
        break;

    case CAN_ID_VEH_BATTERY_INFO: /* 0x102 */
        rx->h102.protocol_number              = frame->data[0];
        rx->h102.target_battery_voltage_V     = (uint16_t)frame->data[1]
                                               | ((uint16_t)frame->data[2] << 8);
        rx->h102.charging_current_request_A   = frame->data[3];
        rx->h102.fault                        = frame->data[4];
        rx->h102.status                       = frame->data[5];
        rx->h102.charged_rate_percent         = frame->data[6];
        rx->h102_valid = true;
        break;

    default:
        consumed = false;
        break;
    }

    if (consumed) {
        rx->last_rx_ms = 0; /* Caller updates this with current timestamp */
    }
    return consumed;
}

/* ============================================================================
 * CHARGER-SIDE RX UNPACKING (we are vehicle, receiving from charger)
 * ============================================================================ */

bool chademo_can_unpack_charger_frame(chademo_rx_charger_t *rx,
                                       const chademo_can_frame_t *frame)
{
    bool consumed = true;

    switch (frame->id) {
    case CAN_ID_CHRG_AVAILABLE: /* 0x108 */
        rx->h108.welding_detection_support = (frame->data[0] != 0);
        rx->h108.avail_output_voltage_V    = (uint16_t)frame->data[1]
                                            | ((uint16_t)frame->data[2] << 8);
        rx->h108.avail_output_current_A    = frame->data[3];
        rx->h108.threshold_voltage_V       = (uint16_t)frame->data[4]
                                            | ((uint16_t)frame->data[5] << 8);
        rx->h108_valid = true;
        break;

    case CAN_ID_CHRG_SYSTEM_STATUS: /* 0x109 */
        rx->h109.protocol_number           = frame->data[0];
        rx->h109.present_output_voltage_V  = (uint16_t)frame->data[1]
                                            | ((uint16_t)frame->data[2] << 8);
        rx->h109.present_output_current_A  = frame->data[3];
        rx->h109.status                    = frame->data[5];
        rx->h109.remaining_charge_time_10s = frame->data[6];
        rx->h109.remaining_charge_time_60s = frame->data[7];
        rx->h109_valid = true;
        break;

    default:
        consumed = false;
        break;
    }

    if (consumed) {
        rx->last_rx_ms = 0;
    }
    return consumed;
}
