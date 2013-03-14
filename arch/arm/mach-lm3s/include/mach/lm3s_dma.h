/************************************************************************************
 * arch/arm/src/lm3s/lm3s_dma.h
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

#ifndef __ARCH_ARM_SRC_LM3S_DMA_H
#define __ARCH_ARM_SRC_LM3S_DMA_H

#define DMA_CHANNEL_SIZE            16
#define DMA_CHANNEL_TABLE_ALIGNMENT 1024
#define DMA_CHANNEL_TABLE_SIZE      (LM3S_NUDMA * DMA_CHANNEL_SIZE * 2)

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

/* uDMA channel addresses ***********************************************************/

#define LM3S_DMA_CHANNEL_BASE(base, n, alt)    ((uint32_t)(base) + DMA_CHANNEL_SIZE*((uint32_t)(n)) + (alt) * DMA_CHANNEL_SIZE * LM3S_NUDMA)
#define LM3S_DMA_CHANNEL_SRC_END(base, n, alt) (LM3S_DMA_CHANNEL_BASE(base, n, alt) + DMA_CHANNEL_SRC_END_OFFSET)
#define LM3S_DMA_CHANNEL_DST_END(base, n, alt) (LM3S_DMA_CHANNEL_BASE(base, n, alt) + DMA_CHANNEL_DST_END_OFFSET)
#define LM3S_DMA_CHANNEL_CHCTL(base, n, alt)   (LM3S_DMA_CHANNEL_BASE(base, n, alt) + DMA_CHANNEL_CHCTL_OFFSET)

/* uDMA register addresses **********************************************************/

#define LM3S_DMA_STAT                    (LM3S_UDMA_BASE + DMA_STAT_OFFSET)
#define LM3S_DMA_CFG                     (LM3S_UDMA_BASE + DMA_CFG_OFFSET)
#define LM3S_DMA_CTLBASE                 (LM3S_UDMA_BASE + DMA_CTLBASE_OFFSET)
#define LM3S_DMA_ALTBASE                 (LM3S_UDMA_BASE + DMA_ALTBASE_OFFSET)
#define LM3S_DMA_SWREQ                   (LM3S_UDMA_BASE + DMA_SWREQ_OFFSET)
#define LM3S_DMA_USEBURSTSET             (LM3S_UDMA_BASE + DMA_USEBURSTSET_OFFSET)
#define LM3S_DMA_USEBURSTCLR             (LM3S_UDMA_BASE + DMA_USEBURSTCLR_OFFSET)
#define LM3S_DMA_REQMASKSET              (LM3S_UDMA_BASE + DMA_REQMASKSET_OFFSET)
#define LM3S_DMA_REQMASKCLR              (LM3S_UDMA_BASE + DMA_REQMASKCLR_OFFSET)
#define LM3S_DMA_ENASET                  (LM3S_UDMA_BASE + DMA_ENASET_OFFSET)
#define LM3S_DMA_ENACLR                  (LM3S_UDMA_BASE + DMA_ENACLR_OFFSET)
#define LM3S_DMA_ALTSET                  (LM3S_UDMA_BASE + DMA_ALTSET_OFFSET)
#define LM3S_DMA_ALTCLR                  (LM3S_UDMA_BASE + DMA_ALTCLR_OFFSET)
#define LM3S_DMA_PRIOSET                 (LM3S_UDMA_BASE + DMA_PRIOSET_OFFSET)
#define LM3S_DMA_PRIOCLR                 (LM3S_UDMA_BASE + DMA_PRIOCLR_OFFSET)
#define LM3S_DMA_ERRCLR                  (LM3S_UDMA_BASE + DMA_ERRCLR_OFFSET)
#define LM3S_DMA_CHASGN                  (LM3S_UDMA_BASE + DMA_CHASGN_OFFSET)
#define LM3S_DMA_CHIS                    (LM3S_UDMA_BASE + DMA_CHIS_OFFSET)

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

#endif // __ARCH_ARM_SRC_LM3S_DMA_H