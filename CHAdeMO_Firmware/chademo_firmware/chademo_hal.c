/**
 * @file  chademo_hal.c
 * @brief Raspberry Pi Pico RP2040 HAL Implementation
 * @details
 *   All Pico SDK-specific code lives here. This is the only file that
 *   includes hardware-specific headers, making porting straightforward.
 */

#include "chademo_hal.h"
#include "chademo_config.h"
#include <string.h>

/* Pico SDK */
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"

/* ============================================================================
 * SPI / PIN MAPPING PER CHANNEL
 * ============================================================================ */

typedef struct {
    spi_inst_t *spi;
    uint8_t     pin_sck;
    uint8_t     pin_tx;   /* MOSI */
    uint8_t     pin_rx;   /* MISO */
    uint8_t     pin_cs;
    uint8_t     pin_int;
    uint32_t    baud_hz;
} can_hw_config_t;

const can_hw_config_t can_cfg[2] = {
    [HAL_CAN_CH1] = {
        .spi      = MCP2515_SPI_PORT,      /* spi0 */
        .pin_sck  = PIN_CAN1_SPI_SCK,      /* GPIO2 */
        .pin_tx   = PIN_CAN1_SPI_TX,       /* GPIO3 (MOSI) */
        .pin_rx   = PIN_CAN1_SPI_RX,       /* GPIO4 (MISO) */
        .pin_cs   = PIN_CAN1_CS,           /* GPIO6 */
        .pin_int  = PIN_CAN1_INT,          /* GPIO5 */
        .baud_hz  = MCP2515_SPI_BAUD_HZ    /* 10 MHz */
    },
    [HAL_CAN_CH2] = {
        .spi      = MCP2515_SPI_PORT_2,    /* spi1 */
        .pin_sck  = PIN_CAN2_SPI_SCK,      /* GPIO10 */
        .pin_tx   = PIN_CAN2_SPI_TX,       /* GPIO11 (MOSI) */
        .pin_rx   = PIN_CAN2_SPI_RX,       /* GPIO12 (MISO) */
        .pin_cs   = PIN_CAN2_CS,           /* GPIO14 */
        .pin_int  = PIN_CAN2_INT,          /* GPIO13 */
        .baud_hz  = MCP2515_SPI_BAUD_HZ_2  /* 10 MHz */
    }
};

/* ============================================================================
 * GPIO HAL IMPLEMENTATION
 * ============================================================================ */

void hal_gpio_init(void)
{
    /* ---- COMMON: Contactor driver (GPIO15) ----
     * CRITICAL SAFETY: Contactor MUST be OPEN on boot.
     * Initialize as output, driven LOW before anything else.
     */
    gpio_init(PIN_CONTACTOR_OUT);
    gpio_set_dir(PIN_CONTACTOR_OUT, GPIO_OUT);
    gpio_put(PIN_CONTACTOR_OUT, 0);  /* Contactor OPEN */

#if IS_STATION
    /* ---- CHARGER (EVSE) ROLE ----
     * Outputs: SS1 (GPIO7), SS2 (GPIO8) — active-HIGH via opto/SolidState
     * Input:  DCP (GPIO9) — active-HIGH from vehicle
     */
    gpio_init(PIN_OUT_SS1);
    gpio_set_dir(PIN_OUT_SS1, GPIO_OUT);
    gpio_put(PIN_OUT_SS1, 0);  /* SS1 de-asserted */

    gpio_init(PIN_OUT_SS2);
    gpio_set_dir(PIN_OUT_SS2, GPIO_OUT);
    gpio_put(PIN_OUT_SS2, 0);  /* SS2 de-asserted */

    gpio_init(PIN_IN_DCP);
    gpio_set_dir(PIN_IN_DCP, GPIO_IN);
    gpio_pull_down(PIN_IN_DCP);  /* Pull-down: default LOW */

#else
    /* ---- VEHICLE ROLE ----
     * Inputs:  SS1 (GPIO16), SS2 (GPIO17), PP (GPIO18)
     * Output: DCP (GPIO19) — active-HIGH to charger
     */
    gpio_init(PIN_IN_SS1);
    gpio_set_dir(PIN_IN_SS1, GPIO_IN);
    gpio_pull_up(PIN_IN_SS1);  /* Pull-up: default HIGH */

    gpio_init(PIN_IN_SS2);
    gpio_set_dir(PIN_IN_SS2, GPIO_IN);
    gpio_pull_up(PIN_IN_SS2);  /* Pull-up: default HIGH */

    gpio_init(PIN_IN_PP);
    gpio_set_dir(PIN_IN_PP, GPIO_IN);
    gpio_pull_up(PIN_IN_PP);   /* Pull-up: default HIGH */

    gpio_init(PIN_OUT_DCP);
    gpio_set_dir(PIN_OUT_DCP, GPIO_OUT);
    gpio_put(PIN_OUT_DCP, 1);  /* DCP de-asserted (inverted: GPIO HIGH = output LOW) */
#endif
}

void hal_gpio_set_contactor(bool close)
{
    gpio_put(PIN_CONTACTOR_OUT, close ? 1 : 0);
}

bool hal_gpio_get_contactor(void)
{
    return (gpio_get(PIN_CONTACTOR_OUT) != 0);
}

#if IS_STATION
void hal_gpio_set_ss1(bool active)
{
    gpio_put(PIN_OUT_SS1, active ? 1 : 0);
}

void hal_gpio_set_ss2(bool active)
{
    gpio_put(PIN_OUT_SS2, active ? 1 : 0);
}

bool hal_gpio_read_dcp(void)
{
    return (gpio_get(PIN_IN_DCP) != 0);
}
#else
void hal_gpio_set_dcp(bool active)
{
    /* Vehicle DCP output: GPIO HIGH asserts DCP at the connector.
     * (Hardware is non-inverting; comment in old code was wrong.) */
    gpio_put(PIN_OUT_DCP, active ? 1 : 0);
}

bool hal_gpio_read_ss1(void)
{
    return (gpio_get(PIN_IN_SS1) != 0);
}

bool hal_gpio_read_ss2(void)
{
    return (gpio_get(PIN_IN_SS2) != 0);
}

bool hal_gpio_read_pp(void)
{
    return (gpio_get(PIN_IN_PP) != 0);
}
#endif

/* ============================================================================
 * SPI HELPERS (private)
 * ============================================================================ */

static inline void cs_low(const can_hw_config_t *cfg)
{
    gpio_put(cfg->pin_cs, 0);
    /* Delay to satisfy MCP2515 t_CSC (CS setup to clock) = 50ns min.
     * At 125 MHz each nop is ~8 ns; 10 nops ≈ 80 ns. */
    asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
}

static inline void cs_high(const can_hw_config_t *cfg)
{
    /* Delay to satisfy MCP2515 t_CSH (CS hold after clock) = 50ns min */
    asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
    gpio_put(cfg->pin_cs, 1);
}

static uint8_t spi_xfer_byte(const can_hw_config_t *cfg, uint8_t b)
{
    uint8_t rx;
    spi_write_read_blocking(cfg->spi, &b, &rx, 1);
    return rx;
}

/* ============================================================================
 * CAN HW CONFIG ACCESSOR
 * ============================================================================ */

const void *hal_can_get_cfg(hal_can_channel_t ch)
{
    return &can_cfg[ch];
}

/* ============================================================================
 * MCP2515 LOW-LEVEL REGISTER ACCESS
 * ============================================================================ */

void hal_can_reset(hal_can_channel_t ch)
{
    const can_hw_config_t *cfg = &can_cfg[ch];
    cs_low(cfg);
    spi_xfer_byte(cfg, MCP2515_CMD_RESET);
    cs_high(cfg);
    /* MCP2515 requires ~2ms to reset */
    sleep_ms(5);
}

uint8_t hal_can_read_reg(hal_can_channel_t ch, uint8_t reg)
{
    const can_hw_config_t *cfg = &can_cfg[ch];
    uint8_t val;
    cs_low(cfg);
    spi_xfer_byte(cfg, MCP2515_CMD_READ);
    spi_xfer_byte(cfg, reg);
    val = spi_xfer_byte(cfg, 0x00);  /* Dummy write to read */
    cs_high(cfg);
    return val;
}

void hal_can_write_reg(hal_can_channel_t ch, uint8_t reg, uint8_t val)
{
    const can_hw_config_t *cfg = &can_cfg[ch];
    cs_low(cfg);
    spi_xfer_byte(cfg, MCP2515_CMD_WRITE);
    spi_xfer_byte(cfg, reg);
    spi_xfer_byte(cfg, val);
    cs_high(cfg);
}

void hal_can_bit_modify(hal_can_channel_t ch, uint8_t reg, uint8_t mask, uint8_t val)
{
    const can_hw_config_t *cfg = &can_cfg[ch];
    cs_low(cfg);
    spi_xfer_byte(cfg, MCP2515_CMD_BIT_MODIFY);
    spi_xfer_byte(cfg, reg);
    spi_xfer_byte(cfg, mask);
    spi_xfer_byte(cfg, val);
    cs_high(cfg);
}

void hal_can_set_mode(hal_can_channel_t ch, uint8_t mode)
{
    /* Mode is in the upper 3 bits of CANCTRL */
    hal_can_bit_modify(ch, MCP2515_REG_CANCTRL, 0xE0, mode);
}

/* ============================================================================
 * MCP2515 INITIALIZATION
 * ============================================================================ */

bool hal_can_init(hal_can_channel_t ch)
{
    const can_hw_config_t *cfg = &can_cfg[ch];

    /* ---- Initialize SPI GPIO ---- */
    gpio_init(cfg->pin_cs);
    gpio_set_dir(cfg->pin_cs, GPIO_OUT);
    gpio_put(cfg->pin_cs, 1);  /* CS idle HIGH */

    gpio_set_function(cfg->pin_sck, GPIO_FUNC_SPI);
    gpio_set_function(cfg->pin_tx,  GPIO_FUNC_SPI);
    gpio_set_function(cfg->pin_rx,  GPIO_FUNC_SPI);

    /* Initialize SPI peripheral */
    spi_init(cfg->spi, cfg->baud_hz);
    spi_set_format(cfg->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    /* ---- Reset and enter config mode ---- */
    hal_can_reset(ch);

    /* Verify we're in config mode */
    uint8_t canstat = hal_can_read_reg(ch, MCP2515_REG_CANSTAT);
    if ((canstat & 0xE0) != MCP2515_MODE_CONFIG) {
        /* MCP2515 not responding — possible hardware fault */
        return false;
    }

    /* ---- Configure bit timing for 500 kbps ----
     * Assumes 8MHz crystal on MCP2515 module.
     * TQ = 2*(BRP+1)/Fosc = 2*1/8MHz = 250ns
     * NBT = 1(SYNC) + 2(PROP) + 4(PS1) + 2(PS2) = 9 TQ
     * Bit rate = 1/(9 * 250ns) = 444 kbps
     *
     * If your MCP2515 has a 16MHz crystal, change CNF1 to 0x01
     * and recalculate:
     * CNF1 = 0x01 (BRP=1, TQ=250ns), CNF2 = 0x91, CNF3 = 0x01
     * NBT = 9 TQ, Bit rate = 1/(9*250ns) = 444kbps
     *
     * For exactly 500kbps with 16MHz: CNF1=0x01, CNF2=0xB8, CNF3=0x05
     * (16 TQ * 250ns = 4us -> 250kbps... no)
     *
     * For 500kbps with 16MHz: use BRP=0 (TQ=125ns), 16 TQ total
     * CNF1=0x00, CNF2=0xB8, CNF3=0x05
     * BTLMODE=1, SAM=0, PHSEG1=7(PS1=8), PRSEG=0(PROP=1), PHSEG2=5(PS2=6)
     * Total = 1+1+8+6 = 16. Bitrate = 1/(16*125ns) = 500kbps. Perfect.
     */
#if MCP2515_OSC_MHZ == 16
    hal_can_write_reg(ch, MCP2515_REG_CNF1, 0x00);  /* BRP=0, SJW=1, TQ=125ns */
    hal_can_write_reg(ch, MCP2515_REG_CNF2, 0xB8);  /* BTLMODE=1, SAM=0, PHSEG1=7, PRSEG=0 */
    hal_can_write_reg(ch, MCP2515_REG_CNF3, 0x05);  /* PHSEG2=5 */
#else
    /* Default: 8MHz crystal (most common on blue PCB modules) */
    hal_can_write_reg(ch, MCP2515_REG_CNF1, 0x00);  /* BRP=0, SJW=1, TQ=250ns */
    hal_can_write_reg(ch, MCP2515_REG_CNF2, 0x91);  /* BTLMODE=1, SAM=0, PHSEG1=3, PRSEG=1 */
    hal_can_write_reg(ch, MCP2515_REG_CNF3, 0x01);  /* PHSEG2=1 */
#endif

    /* ---- Configure RX buffers to accept all standard frames ----
     * RXB0CTRL: RXM[1:0] = 00 (receive all valid msgs using filters)
     * For now, use receive-all mode: RXM = 11
     */
    hal_can_write_reg(ch, MCP2515_REG_RXB0CTRL, 0x64);  /* RXM1=1, RXM0=1 (receive any), BUKT=1 (rollover to RXB1) */
    hal_can_write_reg(ch, MCP2515_REG_RXB1CTRL, 0x60);  /* RXM1=1, RXM0=1 (receive any) */

    /* ---- Configure interrupts ----
     * CANINTE: Enable RX0IE and RX1IE (receive interrupts)
     * Also enable ERRIE (error interrupt) and MERRIE (message error)
     */
    hal_can_write_reg(ch, MCP2515_REG_CANINTE,
                       MCP2515_INT_RX0 | MCP2515_INT_RX1 |
                       MCP2515_INT_ERR | MCP2515_INT_MERR);

    /* Clear all interrupt flags */
    hal_can_write_reg(ch, MCP2515_REG_CANINTF, 0x00);

    /* Clear error flags */
    hal_can_write_reg(ch, MCP2515_REG_EFLG, 0x00);

    /* ---- Enter normal mode ---- */
    hal_can_set_mode(ch, MCP2515_MODE_NORMAL);

    /* Verify mode transition */
    for (int i = 0; i < 100; i++) {
        uint8_t stat = hal_can_read_reg(ch, MCP2515_REG_CANSTAT);
        if ((stat & 0xE0) == MCP2515_MODE_NORMAL) {
            return true;  /* Success */
        }
        sleep_ms(1);
    }
    return false;  /* Failed to enter normal mode */
}

/* ============================================================================
 * CAN FRAME TX / RX
 * ============================================================================ */

bool hal_can_send(hal_can_channel_t ch, const chademo_can_frame_t *frame)
{
    const can_hw_config_t *cfg = &can_cfg[ch];

    /* Round-robin across TXB0, TXB1, TXB2 to avoid getting stuck
     * if one buffer is locked up in an error state. */
    static uint8_t tx_idx = 0;

    const uint8_t ctrl_reg[3] = {
        MCP2515_REG_TXB0CTRL,
        MCP2515_REG_TXB1CTRL,
        MCP2515_REG_TXB2CTRL
    };
    const uint8_t sidh_reg[3] = {
        MCP2515_REG_TXB0SIDH,
        MCP2515_REG_TXB1SIDH,
        MCP2515_REG_TXB2SIDH
    };

    for (int attempt = 0; attempt < 3; attempt++) {
        uint8_t idx = tx_idx % 3;
        uint8_t txbctrl = hal_can_read_reg(ch, ctrl_reg[idx]);

        if (!(txbctrl & 0x08)) {
            /* Buffer is free - use it */
            uint8_t sidh, sidl, eid8, eid0;
            if (frame->id & 0x80000000UL) {
                /* Extended frame (29-bit ID) */
                uint32_t eid = frame->id & 0x1FFFFFFFUL;
                sidh = (uint8_t)((eid >> 21) & 0xFF);
                sidl = (uint8_t)(((eid >> 18) & 0x07) << 5) | 0x08 | ((eid >> 16) & 0x03);
                eid8 = (uint8_t)((eid >> 8) & 0xFF);
                eid0 = (uint8_t)(eid & 0xFF);
            } else {
                /* Standard frame (11-bit ID) */
                sidh = (uint8_t)((frame->id >> 3) & 0xFF);
                sidl = (uint8_t)((frame->id & 0x07) << 5);
                eid8 = 0x00;
                eid0 = 0x00;
            }

            cs_low(cfg);
            spi_xfer_byte(cfg, MCP2515_CMD_WRITE);
            spi_xfer_byte(cfg, sidh_reg[idx]);
            spi_xfer_byte(cfg, sidh);
            spi_xfer_byte(cfg, sidl);
            spi_xfer_byte(cfg, eid8);
            spi_xfer_byte(cfg, eid0);
            spi_xfer_byte(cfg, frame->len); /* DLC */
            for (int i = 0; i < frame->len; i++) {
                spi_xfer_byte(cfg, frame->data[i]);
            }
            for (int i = frame->len; i < 8; i++) {
                spi_xfer_byte(cfg, 0x00);
            }
            cs_high(cfg);

            /* Request transmission */
            cs_low(cfg);
            spi_xfer_byte(cfg, MCP2515_CMD_WRITE);
            spi_xfer_byte(cfg, ctrl_reg[idx]);
            spi_xfer_byte(cfg, 0x08);  /* TXREQ = 1 */
            cs_high(cfg);

            tx_idx = idx + 1;
            return true;
        }

        /* Buffer busy - if TXERR is set, try to abort it */
        if (txbctrl & 0x10) {
            hal_can_bit_modify(ch, ctrl_reg[idx], 0x08, 0x00);
            sleep_us(50);
            txbctrl = hal_can_read_reg(ch, ctrl_reg[idx]);
            if (!(txbctrl & 0x08)) {
                /* Abort succeeded - use this buffer now */
                uint8_t sidh, sidl, eid8, eid0;
                if (frame->id & 0x80000000UL) {
                    uint32_t eid = frame->id & 0x1FFFFFFFUL;
                    sidh = (uint8_t)((eid >> 21) & 0xFF);
                    sidl = (uint8_t)(((eid >> 18) & 0x07) << 5) | 0x08 | ((eid >> 16) & 0x03);
                    eid8 = (uint8_t)((eid >> 8) & 0xFF);
                    eid0 = (uint8_t)(eid & 0xFF);
                } else {
                    sidh = (uint8_t)((frame->id >> 3) & 0xFF);
                    sidl = (uint8_t)((frame->id & 0x07) << 5);
                    eid8 = 0x00;
                    eid0 = 0x00;
                }

                cs_low(cfg);
                spi_xfer_byte(cfg, MCP2515_CMD_WRITE);
                spi_xfer_byte(cfg, sidh_reg[idx]);
                spi_xfer_byte(cfg, sidh);
                spi_xfer_byte(cfg, sidl);
                spi_xfer_byte(cfg, eid8);
                spi_xfer_byte(cfg, eid0);
                spi_xfer_byte(cfg, frame->len);
                for (int i = 0; i < frame->len; i++) {
                    spi_xfer_byte(cfg, frame->data[i]);
                }
                for (int i = frame->len; i < 8; i++) {
                    spi_xfer_byte(cfg, 0x00);
                }
                cs_high(cfg);

                cs_low(cfg);
                spi_xfer_byte(cfg, MCP2515_CMD_WRITE);
                spi_xfer_byte(cfg, ctrl_reg[idx]);
                spi_xfer_byte(cfg, 0x08);
                cs_high(cfg);

                tx_idx = idx + 1;
                return true;
            }
        }

        tx_idx = idx + 1;
    }

    return false;  /* All 3 buffers busy */
}

bool hal_can_recv(hal_can_channel_t ch, chademo_can_frame_t *frame)
{
    /* Check CANINTF for RX interrupt flags instead of using RX_STATUS command */
    uint8_t intf = hal_can_read_reg(ch, MCP2515_REG_CANINTF);
    uint8_t buf_flag = intf & (MCP2515_INT_RX0 | MCP2515_INT_RX1);
    if (buf_flag == 0) {
        return false;  /* No frames pending */
    }

    const can_hw_config_t *cfg = &can_cfg[ch];
    uint8_t reg_base;
    uint8_t intf_bit;

    if (buf_flag & MCP2515_INT_RX0) {
        /* RXB0 has a frame */
        reg_base = MCP2515_REG_RXB0SIDH;
        intf_bit = MCP2515_INT_RX0;
    } else {
        /* RXB1 has a frame */
        reg_base = MCP2515_REG_RXB1SIDH;
        intf_bit = MCP2515_INT_RX1;
    }

    /* Read the frame from the MCP2515 */
    uint8_t buf[13];  /* SIDH, SIDL, EID8, EID0, DLC, D0-D7 */

    cs_low(cfg);
    spi_xfer_byte(cfg, MCP2515_CMD_READ);
    spi_xfer_byte(cfg, reg_base);
    for (int i = 0; i < 13; i++) {
        buf[i] = spi_xfer_byte(cfg, 0x00);
    }
    cs_high(cfg);

    /* Parse ID — detect extended vs standard */
    if (buf[1] & 0x08) {
        /* Extended frame (29-bit ID) */
        uint32_t eid = ((uint32_t)buf[0] << 21)
                     | (((uint32_t)(buf[1] & 0xE0)) << 13)
                     | (((uint32_t)(buf[1] & 0x03)) << 16)
                     | ((uint32_t)buf[2] << 8)
                     | buf[3];
        frame->id = eid | 0x80000000UL;
    } else {
        /* Standard frame (11-bit ID) */
        uint16_t sid = ((uint16_t)buf[0] << 3) | ((uint16_t)(buf[1] & 0xE0) >> 5);
        frame->id = (uint32_t)sid;
    }
    frame->len = buf[4] & 0x0F;
    if (frame->len > 8) frame->len = 8;
    memcpy(frame->data, &buf[5], frame->len);

    /* Clear the RX interrupt flag */
    hal_can_bit_modify(ch, MCP2515_REG_CANINTF, intf_bit, 0);

    return true;
}

bool hal_can_has_rx_pending(hal_can_channel_t ch)
{
    /* Fast check: read INT pin directly (active LOW on MCP2515) */
    const can_hw_config_t *cfg = &can_cfg[ch];
    return (gpio_get(cfg->pin_int) == 0);
}

uint8_t hal_can_read_int_flags(hal_can_channel_t ch)
{
    return hal_can_read_reg(ch, MCP2515_REG_CANINTF);
}

/* ============================================================================
 * WATCHDOG & TIMING
 * ============================================================================ */

uint32_t hal_millis(void)
{
    return to_ms_since_boot(get_absolute_time());
}

void hal_watchdog_feed(void)
{
    watchdog_update();
}

void hal_watchdog_init(uint32_t timeout_ms)
{
    /* RP2040/RP2350 watchdog: watchdog_enable() takes milliseconds */
    if (timeout_ms > 8300) timeout_ms = 8300;
    watchdog_enable(timeout_ms, 1);
}
