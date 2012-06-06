#ifndef __MACH_LM3S_SPI_H
#define __MACH_LM3S_SPI_H

#include <linux/types.h>

struct spi_lm3s_master {
  uint32_t *chipselect;
  int       num_chipselect;
};

#endif // __MACH_LM3S_SPI_H