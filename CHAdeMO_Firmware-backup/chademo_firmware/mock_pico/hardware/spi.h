#ifndef _HARDWARE_SPI_H
#define _HARDWARE_SPI_H
#include <stdint.h>
#include <stddef.h>
typedef struct spi_inst spi_inst_t;
extern spi_inst_t spi0_inst;
extern spi_inst_t spi1_inst;
#define spi0 (&spi0_inst)
#define spi1 (&spi1_inst)
enum { SPI_CPOL_0 = 0, SPI_CPHA_0 = 0, SPI_MSB_FIRST = 0 };
static inline uint spi_init(spi_inst_t* spi, uint baudrate) { (void)spi; (void)baudrate; return baudrate; }
static inline void spi_set_format(spi_inst_t* spi, uint data_bits, uint cpol, uint cpha, uint order) {
    (void)spi; (void)data_bits; (void)cpol; (void)cpha; (void)order;
}
static inline int spi_write_read_blocking(spi_inst_t* spi, const uint8_t* src, uint8_t* dst, size_t len) {
    (void)spi; (void)src; (void)dst; (void)len; return (int)len;
}
#endif
