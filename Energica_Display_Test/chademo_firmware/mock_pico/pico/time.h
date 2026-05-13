#ifndef _PICO_TIME_H
#define _PICO_TIME_H
#include "pico/types.h"
#include <stdint.h>
typedef int32_t alarm_id_t;
static inline alarm_id_t add_alarm_in_ms(uint32_t ms, void* callback, void* user_data, bool fire_if_past) {
    (void)ms; (void)callback; (void)user_data; (void)fire_if_past; return 0;
}
static inline bool cancel_alarm(alarm_id_t id) { (void)id; return true; }
#endif
