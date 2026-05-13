/**
 * @file  chademo_infypower.c
 * @brief InfyPower DC Module CAN Driver Implementation
 */

#include "chademo_infypower.h"
#include "chademo_hal.h"
#include "chademo_can.h"
#include "chademo_config.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * INTERNAL STATE
 * ============================================================================ */

static uint32_t g_last_heartbeat_ms = 0;
static bool     g_infypower_alive   = false;
static uint16_t g_last_set_voltage  = 0;
static uint8_t  g_last_set_current  = 0;

/* Forward declaration of HAL hardware config (defined in chademo_hal.c) */
typedef struct { void *spi; uint8_t pin_sck; uint8_t pin_tx; uint8_t pin_rx; uint8_t pin_cs; uint8_t pin_int; uint32_t baud_hz; } can_hw_config_t;
extern const can_hw_config_t can_cfg[2];

/* ============================================================================
 * CAN ID CONSTRUCTION
 * ============================================================================
 * Format: [02] [80|Cmd] [Dest] [Src]
 */

static uint32_t infypower_build_id(uint8_t cmd, uint8_t dest)
{
    return 0x02000000UL
         | (((uint32_t)(0x80U | cmd)) << 16)
         | (((uint32_t)dest) << 8)
         | (uint32_t)INFYPOWER_SRC_ADDRESS;
}

/* ============================================================================
 * LOW-LEVEL TX
 * ============================================================================ */

static void infypower_send(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    chademo_can_frame_t frame;
    frame.id   = id | 0x80000000UL;  /* Extended frame flag */
    frame.len  = dlc;
    memcpy(frame.data, data, dlc);
    /* Zero pad to 8 bytes */
    for (int i = dlc; i < 8; i++) {
        frame.data[i] = 0;
    }
    if (!hal_can_send(HAL_CAN_CH1, &frame)) {
        printf("[STATION-INFY] TX FAILED id=0x%08lX\r\n", (unsigned long)id);
    }
}

/* ============================================================================
 * PUBLIC: INIT
 * ============================================================================ */

bool infypower_init(void)
{
    const can_hw_config_t *cfg = &can_cfg[HAL_CAN_CH1];

    /* ---- Initialize SPI GPIO for CAN1 ---- */
    gpio_init(cfg->pin_cs);
    gpio_set_dir(cfg->pin_cs, GPIO_OUT);
    gpio_put(cfg->pin_cs, 1);

    gpio_set_function(cfg->pin_sck, GPIO_FUNC_SPI);
    gpio_set_function(cfg->pin_tx,  GPIO_FUNC_SPI);
    gpio_set_function(cfg->pin_rx,  GPIO_FUNC_SPI);

    spi_init(cfg->spi, cfg->baud_hz);
    spi_set_format(cfg->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    /* ---- Reset MCP2515 ---- */
    hal_can_reset(HAL_CAN_CH1);

    /* Verify config mode */
    uint8_t canstat = hal_can_read_reg(HAL_CAN_CH1, MCP2515_REG_CANSTAT);
    if ((canstat & 0xE0) != MCP2515_MODE_CONFIG) {
        return false;
    }

    /* ---- Configure for 125 kbps ----
     * Assumes same crystal as CHAdeMO bus (8 MHz).
     * TQ = 2*(BRP+1)/Fosc = 2*2/8MHz = 500ns
     * NBT = 16 TQ  ->  bitrate = 1/(16*500ns) = 125 kbps
     * CNF1=0x01 (BRP=1, SJW=1)
     * CNF2=0xB8 (BTLMODE=1, SAM=0, PHSEG1=7, PRSEG=0)
     * CNF3=0x05 (PHSEG2=5)
     * Total = 1(SYNC) + 1(PROP) + 8(PS1) + 6(PS2) = 16 TQ
     *
     * If your MCP2515 has a 16 MHz crystal, change CNF1 to 0x03.
     */
    hal_can_write_reg(HAL_CAN_CH1, MCP2515_REG_CNF1, 0x01);
    hal_can_write_reg(HAL_CAN_CH1, MCP2515_REG_CNF2, 0xB8);
    hal_can_write_reg(HAL_CAN_CH1, MCP2515_REG_CNF3, 0x05);

    /* Accept all standard + extended frames */
    hal_can_write_reg(HAL_CAN_CH1, MCP2515_REG_RXB0CTRL, 0x64);
    hal_can_write_reg(HAL_CAN_CH1, MCP2515_REG_RXB1CTRL, 0x60);

    /* Enable RX interrupts */
    hal_can_write_reg(HAL_CAN_CH1, MCP2515_REG_CANINTE,
                       MCP2515_INT_RX0 | MCP2515_INT_RX1 |
                       MCP2515_INT_ERR | MCP2515_INT_MERR);
    hal_can_write_reg(HAL_CAN_CH1, MCP2515_REG_CANINTF, 0x00);
    hal_can_write_reg(HAL_CAN_CH1, MCP2515_REG_EFLG, 0x00);

    /* Enter normal mode */
    hal_can_set_mode(HAL_CAN_CH1, MCP2515_MODE_NORMAL);
    for (int i = 0; i < 100; i++) {
        uint8_t stat = hal_can_read_reg(HAL_CAN_CH1, MCP2515_REG_CANSTAT);
        if ((stat & 0xE0) == MCP2515_MODE_NORMAL) {
            g_infypower_alive = true;
            return true;
        }
        sleep_ms(1);
    }
    return false;
}

/* ============================================================================
 * PUBLIC: COMMANDS
 * ============================================================================ */

void infypower_cmd_onoff(bool on)
{
    uint8_t data[8] = {0};
    data[0] = on ? 0x00 : 0x01;  /* 0 = On, 1 = Off */
    uint32_t id = infypower_build_id(INFYPOWER_CMD_ONOFF, INFYPOWER_DEST_BROADCAST);
    infypower_send(id, data, 8);
}

void infypower_cmd_set_output(uint16_t voltage_V, uint8_t current_A)
{
    /* Clamp to safe limits */
    if (voltage_V < INFYPOWER_VOLTAGE_MIN_V) voltage_V = INFYPOWER_VOLTAGE_MIN_V;
    if (voltage_V > INFYPOWER_VOLTAGE_MAX_V) voltage_V = INFYPOWER_VOLTAGE_MAX_V;
    if (current_A > INFYPOWER_CURRENT_MAX_A) current_A = INFYPOWER_CURRENT_MAX_A;

    uint32_t v_mV = (uint32_t)voltage_V * 1000U;
    uint32_t i_mA = (uint32_t)current_A * 1000U;

    uint8_t data[8];
    data[0] = (uint8_t)((v_mV >> 24) & 0xFF);
    data[1] = (uint8_t)((v_mV >> 16) & 0xFF);
    data[2] = (uint8_t)((v_mV >> 8)  & 0xFF);
    data[3] = (uint8_t)(v_mV & 0xFF);
    data[4] = (uint8_t)((i_mA >> 24) & 0xFF);
    data[5] = (uint8_t)((i_mA >> 16) & 0xFF);
    data[6] = (uint8_t)((i_mA >> 8)  & 0xFF);
    data[7] = (uint8_t)(i_mA & 0xFF);

    uint32_t id = infypower_build_id(INFYPOWER_CMD_SET_OUT, INFYPOWER_DEST_BROADCAST);
    infypower_send(id, data, 8);

    g_last_set_voltage = voltage_V;
    g_last_set_current = current_A;
}

void infypower_cmd_read_system(void)
{
    uint8_t data[8] = {0};
    uint32_t id = infypower_build_id(INFYPOWER_CMD_READ_SYS, INFYPOWER_DEST_BROADCAST);
    infypower_send(id, data, 8);
}

/* ============================================================================
 * PUBLIC: HEARTBEAT
 * ============================================================================ */

void infypower_heartbeat(uint16_t voltage_V, uint8_t current_A)
{
    uint32_t now = hal_millis();
    if ((now - g_last_heartbeat_ms) < INFYPOWER_HEARTBEAT_MS) {
        return;
    }
    g_last_heartbeat_ms = now;

    /* Refresh setpoints every heartbeat */
    infypower_cmd_set_output(voltage_V, current_A);
}

/* ============================================================================
 * PUBLIC: RX POLLING
 * ============================================================================ */

bool infypower_poll_rx(uint16_t *out_voltage_V, uint8_t *out_current_A)
{
    chademo_can_frame_t frame;
    bool got_readback = false;

    *out_voltage_V = 0;
    *out_current_A = 0;

    while (hal_can_recv(HAL_CAN_CH1, &frame)) {
        /* Only process extended frames from module addresses (0x00-0x3B) */
        if (!(frame.id & 0x80000000UL)) {
            continue;  /* Skip standard frames */
        }

        uint32_t eff_id = frame.id & 0x1FFFFFFFUL;
        uint8_t  cmd    = (uint8_t)((eff_id >> 16) & 0x3FU);
        uint8_t  src    = (uint8_t)(eff_id & 0xFFU);

        if (src > 0x3BU) {
            continue;  /* Not a module response */
        }

        g_infypower_alive = true;

        if (cmd == INFYPOWER_CMD_READ_SYS || cmd == INFYPOWER_CMD_READ_MOD) {
            if (frame.len >= 8) {
                uint32_t v_raw = ((uint32_t)frame.data[0] << 24)
                               | ((uint32_t)frame.data[1] << 16)
                               | ((uint32_t)frame.data[2] << 8)
                               |  (uint32_t)frame.data[3];
                uint32_t i_raw = ((uint32_t)frame.data[4] << 24)
                               | ((uint32_t)frame.data[5] << 16)
                               | ((uint32_t)frame.data[6] << 8)
                               |  (uint32_t)frame.data[7];
                *out_voltage_V = (uint16_t)(v_raw / 1000U);
                *out_current_A = (uint8_t)(i_raw / 1000U);
                got_readback = true;
            }
        }
    }

    return got_readback;
}

bool infypower_is_alive(void)
{
    return g_infypower_alive;
}
