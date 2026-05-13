#ifndef _PICO_STDLIB_H
#define _PICO_STDLIB_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "pico/platform.h"
#include "pico/types.h"
#include "pico/time.h"

typedef unsigned int uint;
typedef enum { DORMANT, SLEEP, WAIT_FAST, WAIT_SLOW } sleep_mode_t;
static inline void sleep_ms(uint32_t ms) { (void)ms; }
static inline void sleep_us(uint64_t us) { (void)us; }
static inline void busy_wait_ms(uint32_t ms) { (void)ms; }
static inline void busy_wait_us(uint64_t us) { (void)us; }
static inline void busy_wait_us_32(uint32_t us) { (void)us; }

static inline void stdio_init_all(void) {}

#endif
