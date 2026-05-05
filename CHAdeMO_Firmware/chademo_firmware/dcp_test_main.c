/**
 * @file  dcp_test_main.c
 * @brief CHAdeMO Control-Pin Wiring Test — toggles outputs, reads inputs
 * @details
 *   Continuously exercises all CHAdeMO control pins so you can verify
 *   wiring with a multimeter and confirm inputs read back correctly.
 *
 *   VEHICLE role:
 *     - Toggles DCP (GPIO19) every 5 s
 *     - Prints PP (GPIO18) state continuously
 *
 *   STATION role:
 *     - Toggles SS1 (GPIO7) and SS2 (GPIO8) every 5 s
 *     - Prints DCP (GPIO9) state continuously
 *
 *   BUILD:
 *     cd chademo_firmware/build
 *     cmake .. -DCHADEMO_ROLE=VEHICLE   (or STATION)
 *     make dcp_test -j4
 *     cp dcp_test.uf2 /media/$USER/RPI-RP2/
 */

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"

#include "chademo_config.h"
#include "chademo_hal.h"

#include <stdio.h>

#define LED_PIN 25

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);  /* Wait for USB serial enumeration */

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    hal_gpio_init();

    printf("\r\n");
    printf("╔══════════════════════════════════════════════════════════════╗\r\n");
    printf("║     CHAdeMO Control-Pin Wiring Test                          ║\r\n");
    printf("╚══════════════════════════════════════════════════════════════╝\r\n");
    printf("\r\n");

#if IS_VEHICLE
    printf("[TEST] Role: VEHICLE\r\n");
    printf("[TEST] DCP = GPIO%u (OUTPUT, toggles every 5 s)\r\n", PIN_OUT_DCP);
    printf("[TEST] PP  = GPIO%u (INPUT,  pull-up, active-LOW)\r\n", PIN_IN_PP);
    printf("[TEST] Expected: DCP asserted = ~12V,  DCP de-asserted = 0V\r\n");
    printf("[TEST] Expected: PP HIGH = no plug,  PP LOW = plug inserted\r\n");
#else
    printf("[TEST] Role: STATION\r\n");
    printf("[TEST] SS1 = GPIO%u (OUTPUT, toggles every 5 s)\r\n", PIN_OUT_SS1);
    printf("[TEST] SS2 = GPIO%u (OUTPUT, toggles every 5 s)\r\n", PIN_OUT_SS2);
    printf("[TEST] DCP = GPIO%u (INPUT,  pull-down, active-HIGH)\r\n", PIN_IN_DCP);
    printf("[TEST] Expected: SS1/SS2 asserted = ~12V,  de-asserted = 0V\r\n");
    printf("[TEST] Expected: DCP HIGH = vehicle present,  DCP LOW = no vehicle\r\n");
#endif
    printf("[TEST] Probe with multimeter now. Press BOOTSEL to re-flash.\r\n");
    printf("\r\n");

    bool state = false;
    uint32_t last_toggle_ms = 0;

    while (1) {
        uint32_t now = hal_millis();

        /* ---- Toggle outputs every 5 seconds ---- */
        if ((now - last_toggle_ms) >= 5000) {
            last_toggle_ms = now;
            state = !state;
            gpio_put(LED_PIN, state ? 1 : 0);

#if IS_VEHICLE
            hal_gpio_set_dcp(state);
            printf("\r\n[OUT] DCP GPIO%u = %s  (connector should be %s)\r\n",
                   PIN_OUT_DCP,
                   state ? "ASSERTED" : "DE-ASSERTED",
                   state ? "~11-12V"  : "0V");
#else
            hal_gpio_set_ss1(state);
            hal_gpio_set_ss2(state);
            printf("\r\n[OUT] SS1 GPIO%u = %s   SS2 GPIO%u = %s  (connector ~%s)\r\n",
                   PIN_OUT_SS1, state ? "ASSERTED" : "DE-ASSERTED",
                   PIN_OUT_SS2, state ? "ASSERTED" : "DE-ASSERTED",
                   state ? "11-12V" : "0V");
#endif
        }

        /* ---- Read inputs every 200 ms and report changes ---- */
        static uint8_t last_inputs = 0xFF;  /* 0xFF = unknown, force first print */
        static uint32_t last_input_ms = 0;

        if ((now - last_input_ms) >= 200) {
            last_input_ms = now;

#if IS_VEHICLE
            bool pp  = hal_gpio_read_pp();
            bool ss1 = hal_gpio_read_ss1();
            bool ss2 = hal_gpio_read_ss2();
            uint8_t inputs = (pp ? 0x04 : 0) | (ss1 ? 0x02 : 0) | (ss2 ? 0x01 : 0);
            if (inputs != last_inputs) {
                last_inputs = inputs;
                printf("[IN]  PP=%s  SS1=%s  SS2=%s\r\n",
                       pp  ? "HIGH" : "LOW",
                       ss1 ? "HIGH" : "LOW",
                       ss2 ? "HIGH" : "LOW");
            }
#else
            bool dcp = hal_gpio_read_dcp();
            uint8_t inputs = dcp ? 0x01 : 0x00;
            if (inputs != last_inputs) {
                last_inputs = inputs;
                printf("[IN]  DCP=%s  (%s)\r\n",
                       dcp ? "HIGH" : "LOW",
                       dcp ? "VEHICLE PRESENT" : "no vehicle");
            }
#endif
        }

        sleep_ms(50);
    }

    return 0;
}

bi_decl(bi_program_name("CHAdeMO Pin Wiring Test"));
bi_decl(bi_program_version_string("1.0.0"));
