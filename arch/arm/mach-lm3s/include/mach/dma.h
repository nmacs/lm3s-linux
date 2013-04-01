/*
 * arch/arm/mach-mps/include/mach/dma.h
 *
 *   Copyright (C) 2012 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <macscomp@gmail.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __ARCH_ARM_MACH_LM3S_DMA_H
#define __ARCH_ARM_MACH_LM3S_DMA_H

#include <mach/hardware.h>
#include <mach/sram.h>

#define DMA_MAX_TRANSFER_SIZE     1024

#define DMA_CHANNEL_ALT           0x80000000

#define DMA_CHANNEL_UART0_RX      8
#define DMA_CHANNEL_UART0_TX      9
#define DMA_CHANNEL_UART1_RX      22
#define DMA_CHANNEL_UART1_TX      23
#define DMA_CHANNEL_UART2_RX      (12 | DMA_CHANNEL_ALT)
#define DMA_CHANNEL_UART2_TX      (13 | DMA_CHANNEL_ALT)

#define DMA_CHANNEL_SSI0_RX       10
#define DMA_CHANNEL_SSI0_TX       11

#define DMA_HIGH_PRIORITY         0x00000001
#define DMA_USE_BURST             0x00000002
#define DMA_DEFAULT_CONFIG        0x00000000

#define DMA_XFER_DEVICE_TO_MEMORY 0x00000001
#define DMA_XFER_MEMORY_TO_DEVICE 0x00000002
#define DMA_XFER_UNIT_BYTE        0x00000004
#define DMA_XFER_UNIT_WORD        0x00000008
#define DMA_XFER_UNIT_DOUBLE_WORD 0x00000010
#define DMA_XFER_ALT              0x00000020
#define DMA_XFER_MODE_PINGPONG    0x00000040

void dma_setup_channel(unsigned int channel, unsigned int config);
void __sram dma_setup_xfer(unsigned int channel, void *dst, void *src, size_t size, unsigned int flags);
void __sram dma_start_xfer(unsigned int channel);
void __sram dma_stop_xfer(unsigned int channel);
void dma_wait_xfer_complete(unsigned int channel);
int __sram dma_ack_interrupt(unsigned int channel);
int __sram get_units_left(unsigned int channel, int alt);

extern void * dma_memcpy(void *, const void *, __kernel_size_t);

#endif /* __ARCH_ARM_MACH_LM3S_DMA_H */