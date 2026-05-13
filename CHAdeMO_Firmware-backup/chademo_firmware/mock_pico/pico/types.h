#ifndef _PICO_TYPES_H
#define _PICO_TYPES_H
#include <stdint.h>
#include <stdbool.h>
typedef uint32_t absolute_time_t;
static inline absolute_time_t get_absolute_time(void) { return 0; }
static inline uint32_t to_ms_since_boot(absolute_time_t t) { (void)t; return 0; }
static inline absolute_time_t delayed_by_ms(const absolute_time_t t, uint32_t ms) { (void)t; (void)ms; return 0; }
static inline void sleep_until(absolute_time_t t) { (void)t; }
static inline bool is_nil_time(absolute_time_t t) { (void)t; return true; }
static inline bool time_reached(absolute_time_t t) { (void)t; return true; }
#endif
