#ifndef _HARDWARE_WATCHDOG_H
#define _HARDWARE_WATCHDOG_H
#include <stdint.h>
static inline void watchdog_enable(uint32_t delay_ms, bool pause_on_debug) {
    (void)delay_ms; (void)pause_on_debug;
}
static inline void watchdog_update(void) {}
static inline void watchdog_reboot(uint32_t pc, uint32_t sp, uint32_t delay_ms) {
    (void)pc; (void)sp; (void)delay_ms;
}
static inline bool watchdog_caused_reboot(void) { return false; }
static inline bool watchdog_enable_caused_reboot(void) { return false; }
#endif
