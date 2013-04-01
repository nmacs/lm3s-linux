/****************************************************************************/

/*
 *	mcfuart.h -- Texas Instruments UART support defines.
 *
 *	(C) Copyright 2012 Max Nekludov <macscomp@gmail.com>
 */

/****************************************************************************/
#ifndef MACH_LM3S_UART_H
#define MACH_LM3S_UART_H
/****************************************************************************/

#include <linux/serial_core.h>
#include <linux/platform_device.h>

struct lm3s_platform_uart {
	unsigned long mapbase;    /* Physical address base */
	void __iomem *membase;    /* Virtual address if mapped */
	unsigned int  irq;        /* Interrupt vector */
	uint32_t      rcgc1_mask; /* Mask to enable/disable clock gate */
#ifdef CONFIG_LM3S_DMA
	uint32_t dma_rx_channel;
	uint32_t dma_tx_channel;
	void    *dma_tx_buffer;
	void    *dma_rx_buffer;
	uint32_t dma_buffer_size;
#endif
};

#endif /* MACH_LM3S_UART_H */
