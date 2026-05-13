#ifndef _HARDWARE_GPIO_H
#define _HARDWARE_GPIO_H
#include <stdint.h>
#include <stdbool.h>
typedef enum { GPIO_FUNC_XIP = 0, GPIO_FUNC_SPI = 1, GPIO_FUNC_UART = 2, GPIO_FUNC_I2C = 3,
    GPIO_FUNC_PWM = 4, GPIO_FUNC_SIO = 5, GPIO_FUNC_PIO0 = 6, GPIO_FUNC_PIO1 = 7,
    GPIO_FUNC_GPCK = 8, GPIO_FUNC_USB = 9, GPIO_FUNC_NULL = 0x1f } gpio_function_t;
typedef enum { GPIO_OUT, GPIO_IN } gpio_dir_t;
#define GPIO_OUT true
#define GPIO_IN  false
static inline void gpio_init(uint gpio) { (void)gpio; }
static inline void gpio_set_dir(uint gpio, bool out) { (void)gpio; (void)out; }
static inline void gpio_put(uint gpio, bool value) { (void)gpio; (void)value; }
static inline bool gpio_get(uint gpio) { (void)gpio; return false; }
static inline void gpio_pull_up(uint gpio) { (void)gpio; }
static inline void gpio_pull_down(uint gpio) { (void)gpio; }
static inline void gpio_disable_pulls(uint gpio) { (void)gpio; }
static inline void gpio_set_function(uint gpio, gpio_function_t fn) { (void)gpio; (void)fn; }
static inline void gpio_set_pulls(uint gpio, bool up, bool down) { (void)gpio; (void)up; (void)down; }
#endif
