#ifndef _HARDWARE_UART_H
#define _HARDWARE_UART_H
#include <stdint.h>
#include <stddef.h>
typedef struct uart_inst uart_inst_t;
extern uart_inst_t uart0_inst;
#define uart0 (&uart0_inst)
static inline void uart_init(uart_inst_t* uart, uint baudrate) { (void)uart; (void)baudrate; }
static inline void uart_set_baudrate(uart_inst_t* uart, uint baudrate) { (void)uart; (void)baudrate; }
static inline void uart_set_format(uart_inst_t* uart, uint data_bits, uint stop_bits, uint parity) {
    (void)uart; (void)data_bits; (void)stop_bits; (void)parity;
}
static inline void uart_set_fifo_enabled(uart_inst_t* uart, bool enabled) { (void)uart; (void)enabled; }
static inline void uart_set_translate_crlf(uart_inst_t* uart, bool translate) { (void)uart; (void)translate; }
static inline int uart_is_readable(uart_inst_t* uart) { (void)uart; return 0; }
static inline int uart_getc(uart_inst_t* uart) { (void)uart; return 0; }
static inline void uart_putc(uart_inst_t* uart, char c) { (void)uart; (void)c; }
static inline void uart_puts(uart_inst_t* uart, const char* s) { (void)uart; (void)s; }
static inline size_t uart_read_blocking(uart_inst_t* uart, uint8_t* dst, size_t len) {
    (void)uart; (void)dst; (void)len; return len;
}
static inline size_t uart_write_blocking(uart_inst_t* uart, const uint8_t* src, size_t len) {
    (void)uart; (void)src; (void)len; return len;
}
static inline void uart_set_hw_flow(uart_inst_t* uart, bool cts, bool rts) {
    (void)uart; (void)cts; (void)rts;
}
#endif
