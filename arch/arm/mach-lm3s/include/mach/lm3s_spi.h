#ifndef __MACH_LM3S_SPI_H
#define __MACH_LM3S_SPI_H

#include <linux/types.h>

struct spi_lm3s_master {
  uint32_t *chipselect;
  int       num_chipselect;
#ifdef CONFIG_LM3S_DMA
	uint32_t  dma_rx_channel;
	uint32_t  dma_tx_channel;
	void     *dma_rx_buffer;
	void     *dma_tx_buffer;
#endif
};

#endif // __MACH_LM3S_SPI_H