/************************************************************************************
 * arch/arm/plat-stellaris/include/mach/lm3s_dma.h
 *
 *   Copyright (C) 2012 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <macscomp@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __MACH_PLAT_STELLARIS_DMA_H
#define __MACH_PLAT_STELLARIS_DMA_H

#define DMA_CHANNEL_SIZE            16
#define DMA_CHANNEL_TABLE_ALIGNMENT 1024
#define DMA_CHANNEL_TABLE_SIZE      (STLR_NUDMA * DMA_CHANNEL_SIZE * 2)

/* uDMA channel offsets *************************************************************/

#define DMA_CHANNEL_SRC_END_OFFSET       0x000
#define DMA_CHANNEL_DST_END_OFFSET       0x004
#define DMA_CHANNEL_CHCTL_OFFSET         0x008

/* uDMA register offsets ************************************************************/

#define DMA_STAT_OFFSET                  0x000
#define DMA_CFG_OFFSET                   0x004
#define DMA_CTLBASE_OFFSET               0x008
#define DMA_ALTBASE_OFFSET               0x00C
#define DMA_SWREQ_OFFSET                 0x014
#define DMA_USEBURSTSET_OFFSET           0x018
#define DMA_USEBURSTCLR_OFFSET           0x01C
#define DMA_REQMASKSET_OFFSET            0x020
#define DMA_REQMASKCLR_OFFSET            0x024
#define DMA_ENASET_OFFSET                0x028
#define DMA_ENACLR_OFFSET                0x02C
#define DMA_ALTSET_OFFSET                0x030
#define DMA_ALTCLR_OFFSET                0x034
#define DMA_PRIOSET_OFFSET               0x038
#define DMA_PRIOCLR_OFFSET               0x03C
#define DMA_ERRCLR_OFFSET                0x04C
#define DMA_CHASGN_OFFSET                0x500
#define DMA_CHIS_OFFSET                  0x504
#define DMA_CHMAP0_OFFSET                0x510

/* uDMA channel addresses ***********************************************************/

#define STLR_DMA_CHANNEL_BASE(base, n, alt)    ((uint32_t)(base) + DMA_CHANNEL_SIZE*((uint32_t)(n)) + (alt) * DMA_CHANNEL_SIZE * STLR_NUDMA)
#define STLR_DMA_CHANNEL_SRC_END(base, n, alt) (STLR_DMA_CHANNEL_BASE(base, n, alt) + DMA_CHANNEL_SRC_END_OFFSET)
#define STLR_DMA_CHANNEL_DST_END(base, n, alt) (STLR_DMA_CHANNEL_BASE(base, n, alt) + DMA_CHANNEL_DST_END_OFFSET)
#define STLR_DMA_CHANNEL_CHCTL(base, n, alt)   (STLR_DMA_CHANNEL_BASE(base, n, alt) + DMA_CHANNEL_CHCTL_OFFSET)

/* uDMA register addresses **********************************************************/

#define STLR_DMA_STAT                    (STLR_UDMA_BASE + DMA_STAT_OFFSET)
#define STLR_DMA_CFG                     (STLR_UDMA_BASE + DMA_CFG_OFFSET)
#define STLR_DMA_CTLBASE                 (STLR_UDMA_BASE + DMA_CTLBASE_OFFSET)
#define STLR_DMA_ALTBASE                 (STLR_UDMA_BASE + DMA_ALTBASE_OFFSET)
#define STLR_DMA_SWREQ                   (STLR_UDMA_BASE + DMA_SWREQ_OFFSET)
#define STLR_DMA_USEBURSTSET             (STLR_UDMA_BASE + DMA_USEBURSTSET_OFFSET)
#define STLR_DMA_USEBURSTCLR             (STLR_UDMA_BASE + DMA_USEBURSTCLR_OFFSET)
#define STLR_DMA_REQMASKSET              (STLR_UDMA_BASE + DMA_REQMASKSET_OFFSET)
#define STLR_DMA_REQMASKCLR              (STLR_UDMA_BASE + DMA_REQMASKCLR_OFFSET)
#define STLR_DMA_ENASET                  (STLR_UDMA_BASE + DMA_ENASET_OFFSET)
#define STLR_DMA_ENACLR                  (STLR_UDMA_BASE + DMA_ENACLR_OFFSET)
#define STLR_DMA_ALTSET                  (STLR_UDMA_BASE + DMA_ALTSET_OFFSET)
#define STLR_DMA_ALTCLR                  (STLR_UDMA_BASE + DMA_ALTCLR_OFFSET)
#define STLR_DMA_PRIOSET                 (STLR_UDMA_BASE + DMA_PRIOSET_OFFSET)
#define STLR_DMA_PRIOCLR                 (STLR_UDMA_BASE + DMA_PRIOCLR_OFFSET)
#define STLR_DMA_ERRCLR                  (STLR_UDMA_BASE + DMA_ERRCLR_OFFSET)
#define STLR_DMA_CHASGN                  (STLR_UDMA_BASE + DMA_CHASGN_OFFSET)
#define STLR_DMA_CHIS                    (STLR_UDMA_BASE + DMA_CHIS_OFFSET)
#define STLR_DMA_CHMAP0                  (STLR_UDMA_BASE + DMA_CHMAP0_OFFSET)

/* uDMA channel bit defitiions ******************************************************/

/* DMA Channel Control Word (DMACHCTL), offset 0x008 */
#define DMA_CHCTL_XFERMODE_SHIFT        0    /* Bits 2-0: GPTM Timer A Mode */
#define DMA_CHCTL_XFERMODE_MASK         (0x07 << DMA_CHCTL_XFERMODE_SHIFT)
#  define DMA_CHCTL_XFERMODE_STOP       (0x0  << DMA_CHCTL_XFERMODE_SHIFT)
#  define DMA_CHCTL_XFERMODE_BASIC      (0x1  << DMA_CHCTL_XFERMODE_SHIFT)
#  define DMA_CHCTL_XFERMODE_AUTO       (0x2  << DMA_CHCTL_XFERMODE_SHIFT)
#  define DMA_CHCTL_XFERMODE_PING_PONG  (0x3  << DMA_CHCTL_XFERMODE_SHIFT)
#define DMA_CHCTL_XFERSIZE_SHIFT        4    /* Bits 13-4: Transfer Size (minus 1) */
#define DMA_CHCTL_XFERSIZE_MASK         (0x3FF << DMA_CHCTL_XFERSIZE_SHIFT)
#define DMA_CHCTL_SRCSIZE_SHIFT         24   /* Bits 25-24: Source Data Size */
#define DMA_CHCTL_SRCSIZE_MASK          (0x03 << DMA_CHCTL_SRCSIZE_SHIFT)
#  define DMA_CHCTL_SRCSIZE_8           (0x0  << DMA_CHCTL_SRCSIZE_SHIFT)
#  define DMA_CHCTL_SRCSIZE_16          (0x1  << DMA_CHCTL_SRCSIZE_SHIFT)
#  define DMA_CHCTL_SRCSIZE_32          (0x2  << DMA_CHCTL_SRCSIZE_SHIFT)
#define DMA_CHCTL_SRCINC_SHIFT          26   /* Bits 27-26: Source Address Increment */
#define DMA_CHCTL_SRCINC_MASK           (0x03 << DMA_CHCTL_SRCINC_SHIFT)
#  define DMA_CHCTL_SRCINC_8            (0x0  << DMA_CHCTL_SRCINC_SHIFT)
#  define DMA_CHCTL_SRCINC_16           (0x1  << DMA_CHCTL_SRCINC_SHIFT)
#  define DMA_CHCTL_SRCINC_32           (0x2  << DMA_CHCTL_SRCINC_SHIFT)
#  define DMA_CHCTL_SRCINC_NO           (0x3  << DMA_CHCTL_SRCINC_SHIFT)
#define DMA_CHCTL_DSTSIZE_SHIFT         28   /* Bits 29-28: Destination Data Size */
#define DMA_CHCTL_DSTSIZE_MASK          (0x03 << DMA_CHCTL_DSTSIZE_SHIFT)
#  define DMA_CHCTL_DSTSIZE_8           (0x0  << DMA_CHCTL_DSTSIZE_SHIFT)
#  define DMA_CHCTL_DSTSIZE_16          (0x1  << DMA_CHCTL_DSTSIZE_SHIFT)
#  define DMA_CHCTL_DSTSIZE_32          (0x2  << DMA_CHCTL_DSTSIZE_SHIFT)
#define DMA_CHCTL_DSTINC_SHIFT          30   /* Bits 31-30: Destination Address Increment */
#define DMA_CHCTL_DSTINC_MASK           (0x03 << DMA_CHCTL_DSTINC_SHIFT)
#  define DMA_CHCTL_DSTINC_8            (0x0  << DMA_CHCTL_DSTINC_SHIFT)
#  define DMA_CHCTL_DSTINC_16           (0x1  << DMA_CHCTL_DSTINC_SHIFT)
#  define DMA_CHCTL_DSTINC_32           (0x2  << DMA_CHCTL_DSTINC_SHIFT)
#  define DMA_CHCTL_DSTINC_NO           (0x3  << DMA_CHCTL_DSTINC_SHIFT)

/* uDMA register bit defitiions *****************************************************/

/* DMA Status (DMASTAT), offset 0x000 */
#define DMA_STAT_MASTEN_SHIFT           0
#define DMA_STAT_MASTEN_MASK            (0x01 << DMA_STAT_MASTEN_SHIFT)

/* DMA Configuration (DMACFG), offset 0x004 */
#define DMA_CFG_MASTEN_SHIFT           0
#define DMA_CFG_MASTEN_MASK            (0x01 << DMA_CFG_MASTEN_SHIFT)

/* DMA Bus Error Clear (DMAERRCLR), offset 0x04C */
#define DMA_ERRCLR_ERRCLR_SHIFT        0
#define DMA_ERRCLR_ERRCLR_MASK         (0x01 << DMA_ERRCLR_ERRCLR_SHIFT)

#ifndef __ASSEMBLY__

#include <mach/hardware.h>
#include <mach/sram.h>

#define DMA_CHANNEL_NUMBER_OFFSET     0
#define DMA_CHANNEL_NUMBER_MASK       (0x1F << DMA_CHANNEL_NUMBER_OFFSET)

#define DMA_MAX_TRANSFER_SIZE     1024

#if defined(CONFIG_ARCH_LM3S)
#define DMA_CHANNEL_ALT           0x80000000

#define DMA_CHANNEL_UART0_RX      8
#define DMA_CHANNEL_UART0_TX      9
#define DMA_CHANNEL_UART1_RX      22
#define DMA_CHANNEL_UART1_TX      23
#define DMA_CHANNEL_UART2_RX      (12 | DMA_CHANNEL_ALT)
#define DMA_CHANNEL_UART2_TX      (13 | DMA_CHANNEL_ALT)

#define DMA_CHANNEL_SSI0_RX       10
#define DMA_CHANNEL_SSI0_TX       11

#elif defined(CONFIG_ARCH_TM4C)

#define DMA_CHANNEL_PERIPHERAL_OFFSET 9
#define DMA_CHANNEL_PERIPHERAL_MASK   (0x0F << DMA_CHANNEL_PERIPHERAL_OFFSET)

#define DMA_CHANNEL_PERIPHERAL_0      (0 << DMA_CHANNEL_PERIPHERAL_OFFSET)
#define DMA_CHANNEL_PERIPHERAL_1      (1 << DMA_CHANNEL_PERIPHERAL_OFFSET)
#define DMA_CHANNEL_PERIPHERAL_2      (2 << DMA_CHANNEL_PERIPHERAL_OFFSET)
#define DMA_CHANNEL_PERIPHERAL_3      (3 << DMA_CHANNEL_PERIPHERAL_OFFSET)
#define DMA_CHANNEL_PERIPHERAL_4      (4 << DMA_CHANNEL_PERIPHERAL_OFFSET)
#define DMA_CHANNEL_PERIPHERAL_5      (5 << DMA_CHANNEL_PERIPHERAL_OFFSET)
#define DMA_CHANNEL_PERIPHERAL_6      (6 << DMA_CHANNEL_PERIPHERAL_OFFSET)
#define DMA_CHANNEL_PERIPHERAL_7      (7 << DMA_CHANNEL_PERIPHERAL_OFFSET)
#define DMA_CHANNEL_PERIPHERAL_8      (8 << DMA_CHANNEL_PERIPHERAL_OFFSET)

#define DMA_CHANNEL_UART0_RX      (8  | DMA_CHANNEL_PERIPHERAL_0)
#define DMA_CHANNEL_UART0_TX      (9  | DMA_CHANNEL_PERIPHERAL_0)
#define DMA_CHANNEL_UART1_RX      (22 | DMA_CHANNEL_PERIPHERAL_0)
#define DMA_CHANNEL_UART1_TX      (23 | DMA_CHANNEL_PERIPHERAL_0)
#define DMA_CHANNEL_UART2_RX      (12 | DMA_CHANNEL_PERIPHERAL_1)
#define DMA_CHANNEL_UART2_TX      (13 | DMA_CHANNEL_PERIPHERAL_1)

#define DMA_CHANNEL_SSI0_RX       (10 | DMA_CHANNEL_PERIPHERAL_0)
#define DMA_CHANNEL_SSI0_TX       (11 | DMA_CHANNEL_PERIPHERAL_0)

#endif

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

#endif /* __ASSEMBLY__ */

#endif // __MACH_PLAT_STELLARIS_DMA_H