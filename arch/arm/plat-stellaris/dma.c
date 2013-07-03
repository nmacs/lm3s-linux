/*
 *  linux/arch/arm/plat-stellaris/dma.c
 *
 *  Copyright (C) 2012 Max Nekludov <macscomp@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/slab.h>
#include <linux/mman.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <mach/dma.h>
#include <asm/page.h>
#include <asm/fiq.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

//void printascii(const char *str);

#define __dma_channels __sramdata __attribute__((aligned(DMA_CHANNEL_TABLE_ALIGNMENT)))
#define CHANNEL_NUMBER(channel)    (((channel) & DMA_CHANNEL_NUMBER_MASK) >> DMA_CHANNEL_NUMBER_OFFSET)
#define PERIPHERAL_NUMBER(channel) (((channel) & DMA_CHANNEL_PERIPHERAL_MASK) >> DMA_CHANNEL_PERIPHERAL_OFFSET)

struct stellaris_dma_channel {
  uint32_t DMASRCENDP;
  uint32_t DMADSTENDP;
  struct {
		/*
		 * DMA Transfer Mode
		 *
		 * 0x0 Stop
		 * 0x1 Basic
		 * 0x2 Auto-Request
		 * 0x3 Ping-Pong
		 * 0x4 Memory Scatter-Gather
		 * 0x5 Alternate Memory Scatter-Gather
		 * 0x6 Peripheral Scatter-Gather
		 * 0x7 Alternate Peripheral Scatter-Gather
		 */
		uint32_t XFERMODE:3;

		/*
		 * Next Useburst
		 */
		uint32_t NXTUSEBURST:1;

		/*
		 * Transfer Size (minus 1)
		 */
		uint32_t XFERSIZE:10;

		/*
		 * Arbitration Size
		 *
		 * 0x0 1 Transfer
		 * Arbitrates after each μDMA transfer
		 * 0x1 2 Transfers
		 * 0x2 4 Transfers
		 * 0x3 8 Transfers
		 * 0x4 16 Transfers
		 * 0x5 32 Transfers
		 * 0x6 64 Transfers
		 * 0x7 128 Transfers
		 * 0x8 256 Transfers
		 * 0x9 512 Transfers
		 * 0xA-0xF 1024 Transfers
		 * In this configuration, no arbitration occurs during the μDMA
		 * transfer because the maximum transfer size is 1024.
		 */
		uint32_t ARBSIZE:4;

		/*
		 * reserved
		 */
		uint32_t reserved:6;

		/*
		 * Source Data Size
		 *
		 * 0x0 Byte
		 * 8-bit data size.
		 * 0x1 Half-word
		 * 16-bit data size.
		 * 0x2 Word
		 * 32-bit data size.
		 * 0x3 Reserved
		 */
		uint32_t SRCSIZE:2;

		/*
		 * Source Address Increment
		 *
		 * 0x0 Byte
		 * Increment by 8-bit locations
		 * 0x1 Half-word
		 * Increment by 16-bit locations
		 * 0x2 Word
		 * Increment by 32-bit locations
		 * 0x3 No increment
		 * Address remains set to the value of the Source Address End
		 * Pointer (DMASRCENDP) for the chann
		 */
		uint32_t SRCINC:2;

		/*
		 * Destination Data Size
		 *
		 * See description of SRCSIZE
		 */
		uint32_t DSTSIZE:2;

		/*
		 * Destination Address Increment
		 *
		 * See description of SRCINC
		 */
		uint32_t DSTINC:2;
	} DMACHCTL;
  uint32_t unused;
};

static struct stellaris_dma_channel __dma_channels dma_channels[STLR_NUDMA * 2];

void dma_setup_channel(unsigned int channel, unsigned int config)
{
	int ch = CHANNEL_NUMBER(channel);
	int chmask = 1 << ch;

#if defined(CONFIG_ARCH_TM4C)
	int peripheral = PERIPHERAL_NUMBER(channel);
	
	int DMACHMAPn = STLR_DMA_CHMAP0 + (ch >> 3) * 4;
	int DMACHMAPn_shift = (ch & 0x7) * 4;
	
	uint32_t regval = getreg32(DMACHMAPn);
	regval |= peripheral << DMACHMAPn_shift;
	putreg32(regval, DMACHMAPn);
#elif defined(CONFIG_ARCH_LM3S)
	if( channel & DMA_CHANNEL_ALT )
	{
		uint32_t regval = getreg32(STLR_DMA_CHASGN);
		regval |= chmask;
		putreg32(regval, STLR_DMA_CHASGN);
	}
#endif

	if( config & DMA_HIGH_PRIORITY )
		putreg32(chmask, STLR_DMA_PRIOSET);
	else
		putreg32(chmask, STLR_DMA_PRIOCLR);
	putreg32(chmask, STLR_DMA_ALTCLR);
	if( config & DMA_USE_BURST )
		putreg32(chmask, STLR_DMA_USEBURSTSET);
	else
		putreg32(chmask, STLR_DMA_USEBURSTCLR);
}

void __sram dma_setup_xfer(unsigned int channel, void *dst, void *src, size_t size, unsigned int flags)
{
	int ch = CHANNEL_NUMBER(channel);
	int transfer_unit_size_code = 0;
	size_t length;
	struct stellaris_dma_channel *dma_channel = dma_channels + ch;

	if( flags & DMA_XFER_ALT )
		dma_channel += STLR_NUDMA;

	if( flags & DMA_XFER_UNIT_BYTE )
		transfer_unit_size_code = 0;
	else if( flags & DMA_XFER_UNIT_WORD )
		transfer_unit_size_code = 1;
	else if( flags & DMA_XFER_UNIT_DOUBLE_WORD )
		transfer_unit_size_code = 2;

	length = size >> transfer_unit_size_code;

	if( flags & DMA_XFER_DEVICE_TO_MEMORY )
	{
		dma_channel->DMASRCENDP = (uint32_t)src;
		dma_channel->DMADSTENDP = (uint32_t)((char*)dst + length - 1);
		dma_channel->DMACHCTL.DSTINC = transfer_unit_size_code;
		dma_channel->DMACHCTL.SRCINC = 3;
	}
	else
	{
		dma_channel->DMASRCENDP = (uint32_t)((char*)src + length - 1);
		dma_channel->DMADSTENDP = (uint32_t)dst;
		dma_channel->DMACHCTL.DSTINC = 3;
		dma_channel->DMACHCTL.SRCINC = transfer_unit_size_code;
	}

	dma_channel->DMACHCTL.DSTSIZE = transfer_unit_size_code;
	dma_channel->DMACHCTL.SRCSIZE = transfer_unit_size_code;
	dma_channel->DMACHCTL.reserved = 0;
	dma_channel->DMACHCTL.ARBSIZE = 0;
	dma_channel->DMACHCTL.XFERSIZE = length - 1;
	dma_channel->DMACHCTL.NXTUSEBURST = 0;

	if( flags & DMA_XFER_MODE_PINGPONG )
		dma_channel->DMACHCTL.XFERMODE = 3;
	else
		dma_channel->DMACHCTL.XFERMODE = 1;

	//printk("DMA transfer: ch# %i, dst %p, src %p, size %u\n", ch, dst, src, dma_channel->DMACHCTL.XFERSIZE);
}

void __sram dma_start_xfer(unsigned int channel)
{
	int chmask = 1 << CHANNEL_NUMBER(channel);
	putreg32(chmask, STLR_DMA_ALTCLR);
	putreg32(chmask, STLR_DMA_REQMASKCLR);
	putreg32(chmask, STLR_DMA_ENASET);
}

void __sram dma_stop_xfer(unsigned int channel)
{
	int ch = CHANNEL_NUMBER(channel);
	int chmask = 1 << ch;
	putreg32(chmask, STLR_DMA_REQMASKSET);
	putreg32(chmask, STLR_DMA_ENACLR);
}

void dma_wait_xfer_complete(unsigned int channel)
{
	int ch = CHANNEL_NUMBER(channel);
	int chmask = 1 << ch;
	while( getreg32(STLR_DMA_ENASET) & chmask ) {}
}

int __sram dma_ack_interrupt(unsigned int channel)
{
	int ch = CHANNEL_NUMBER(channel);
	int chmask = 1 << ch;
	if( getreg32(STLR_DMA_CHIS) & chmask )
	{
		putreg32(chmask, STLR_DMA_CHIS);
		return 1;
	}
	else
		return 0;
}

int __sram get_units_left(unsigned int channel, int alt)
{
	int ch = CHANNEL_NUMBER(channel);
	int result = 0;
	struct stellaris_dma_channel *dma_channel = dma_channels + ch;
	if( alt )
		dma_channel += STLR_NUDMA;

	result = dma_channel->DMACHCTL.XFERSIZE;
	if( dma_channel->DMACHCTL.XFERMODE )
		result++;

	return result;
}

static int __init dma_init(void)
{
  uint32_t regval;

  /*
   * Enable uDMA clock
   */
  dma_clock_ctrl(SYS_ENABLE_CLOCK);

	/*
   * Enable uDMA
	 */
	regval = getreg32(STLR_DMA_CFG);
	regval |= DMA_CFG_MASTEN_MASK;
	putreg32(regval, STLR_DMA_CFG);

	/*
	 * Program the location of the channel control table
	 */
	putreg32((uint32_t)dma_channels, STLR_DMA_CTLBASE);

	/*
	 * WARNING: Possible TI issue.
	 *          One and only one DMA channel of EPI0 MUST be selected
	 *          to perform MDA transactions from/to SDRAM
	 */
	putreg32((1 << 20) /*|| (1 << 21)*/, STLR_DMA_CHASGN);

  return 0;
}

device_initcall(dma_init);
