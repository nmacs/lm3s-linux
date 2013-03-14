/*
 *  linux/arch/arm/mach-lm3s/dma.c
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

#include <asm/page.h>
#include <asm/fiq.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/sram.h>
#include <asm/uaccess.h>
#include <mach/dma.h>

void printascii(const char *str);

#define __dma_channels __sramdata __attribute__((aligned(DMA_CHANNEL_TABLE_ALIGNMENT)))
#define CHANNEL_NUMBER(channel) ((channel) & 0xff)

struct lm3s_dma_channel {
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

static struct lm3s_dma_channel __dma_channels lm3s_dma_channels[LM3S_NUDMA * 2];

void dma_setup_channel(unsigned int channel, unsigned int config)
{
	int ch = CHANNEL_NUMBER(channel);
	int chmask = 1 << ch;

	if( channel & DMA_CHANNEL_ALT )
	{
		uint32_t regval = lm3s_getreg32(LM3S_DMA_CHASGN);
		regval |= chmask;
		lm3s_putreg32(regval, LM3S_DMA_CHASGN);
	}

	if( config & DMA_HIGH_PRIORITY )
		lm3s_putreg32(chmask, LM3S_DMA_PRIOSET);
	else
		lm3s_putreg32(chmask, LM3S_DMA_PRIOCLR);
	lm3s_putreg32(chmask, LM3S_DMA_ALTCLR);
	if( config & DMA_USE_BURST )
		lm3s_putreg32(chmask, LM3S_DMA_USEBURSTSET);
	else
		lm3s_putreg32(chmask, LM3S_DMA_USEBURSTCLR);
}

void __sram dma_start_xfer(unsigned int channel, void *dst, void *src, size_t size, unsigned int flags)
{
	int ch = CHANNEL_NUMBER(channel);
	int chmask = 1 << ch;
	struct lm3s_dma_channel *dma_channel = lm3s_dma_channels + ch;
	int transfer_unit_size_code = 0;
	size_t length;

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
	dma_channel->DMACHCTL.XFERMODE = 1;

	//printk("DMA transfer: ch# %i, dst %p, src %p, size %u\n", ch, dst, src, dma_channel->DMACHCTL.XFERSIZE);

	lm3s_putreg32(chmask, LM3S_DMA_REQMASKCLR);
	lm3s_putreg32(chmask, LM3S_DMA_ENASET);
}

void dma_stop_xfer(unsigned int channel)
{
	int ch = CHANNEL_NUMBER(channel);
	int chmask = 1 << ch;
	lm3s_putreg32(chmask, LM3S_DMA_REQMASKSET);
	lm3s_putreg32(chmask, LM3S_DMA_ENACLR);
}

void dma_wait_xfer_complete(unsigned int channel)
{
	int ch = CHANNEL_NUMBER(channel);
	int chmask = 1 << ch;
	while( lm3s_getreg32(LM3S_DMA_ENASET) & chmask ) {}
}

int __sram dma_ack_interrupt(unsigned int channel)
{
	int ch = CHANNEL_NUMBER(channel);
	int chmask = 1 << ch;
	if( lm3s_getreg32(LM3S_DMA_CHIS) & chmask )
	{
		lm3s_putreg32(chmask, LM3S_DMA_CHIS);
		return 1;
	}
	else
		return 0;
}

static int __init lm3s_dma_init(void)
{
  uint32_t regval;

  /*
   * Enable uDMA clock
   */
  regval = lm3s_getreg32(LM3S_SYSCON_RCGC2);
  regval |= SYSCON_RCGC2_UDMA;
  lm3s_putreg32(regval, LM3S_SYSCON_RCGC2);

	/*
   * Enable uDMA
	 */
	regval = lm3s_getreg32(LM3S_DMA_CFG);
	regval |= DMA_CFG_MASTEN_MASK;
	lm3s_putreg32(regval, LM3S_DMA_CFG);

	/*
	 * Program the location of the channel control table
	 */
	lm3s_putreg32((uint32_t)lm3s_dma_channels, LM3S_DMA_CTLBASE);

	/*
	 * WARNING: Possible TI issue.
	 *          One and only one DMA channel of EPI0 MUST be selected
	 *          to perform MDA transactions from/to SDRAM
	 */
	lm3s_putreg32((1 << 20) /*|| (1 << 21)*/, LM3S_DMA_CHASGN);

  return 0;
}

device_initcall(lm3s_dma_init);
