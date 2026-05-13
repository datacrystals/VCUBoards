#ifndef _PICO_PLATFORM_H
#define _PICO_PLATFORM_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#define __not_in_flash(group)
#define __no_inline_not_in_flash
#define __time_critical_func(x) x
#define __packed __attribute__((packed))
#define __aligned(x) __attribute__((aligned(x)))
#ifndef __STRING
#define __STRING(x) #x
#endif
static inline void __breakpoint(void) {}
#endif
