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

#ifndef __MACH_PLAT_STELLARIS_ADC_H
#define __MACH_PLAT_STELLARIS_ADC_H

#ifndef __ASSEMBLY__

#define STLR_ADC_TEMPERATURE             (-1)

static inline int adc_to_mV(uint32_t value)
{
	return (int)((3300U * value) / 4096U);
}

static inline int adc_to_mC(uint32_t value)
{
	return 550000 - (int)(value * (1000U / 4U));
}

int adc_convert(uint32_t *value, int channel);
int adc_convert_ex(uint32_t *values, int *channels, int count);

#endif

/* ADC register offsets ************************************************************/

#define STLR_ADC_ACTSS_OFFSET            0x000
#define STLR_ADC_EMUX_OFFSET             0x014
#define STLR_ADC_PSSI_OFFSET             0x028
#define STLR_ADC_SSMUX0_OFFSET           0x040
#define STLR_ADC_SSMUX1_OFFSET           0x060
#define STLR_ADC_SSMUX2_OFFSET           0x080
#define STLR_ADC_SSMUX3_OFFSET           0x0A0
#define STLR_ADC_SSCTL0_OFFSET           0x044
#define STLR_ADC_SSCTL1_OFFSET           0x064
#define STLR_ADC_SSCTL2_OFFSET           0x084
#define STLR_ADC_SSCTL3_OFFSET           0x0A4
#define STLR_ADC_SSFIFO0_OFFSET          0x048
#define STLR_ADC_SSFIFO1_OFFSET          0x068
#define STLR_ADC_SSFIFO2_OFFSET          0x088
#define STLR_ADC_SSFIFO3_OFFSET          0x0A8
#define STLR_ADC_SSFSTAT0_OFFSET         0x04C
#define STLR_ADC_SSFSTAT1_OFFSET         0x06C
#define STLR_ADC_SSFSTAT2_OFFSET         0x08C
#define STLR_ADC_SSFSTAT3_OFFSET         0x0AC

/* uDMA register addresses **********************************************************/

#define STLR_ADC0_ACTSS                  (STLR_ADC0_BASE + STLR_ADC_ACTSS_OFFSET)
#define STLR_ADC0_EMUX                   (STLR_ADC0_BASE + STLR_ADC_EMUX_OFFSET)
#define STLR_ADC0_PSSI                   (STLR_ADC0_BASE + STLR_ADC_PSSI_OFFSET)
#define STLR_ADC0_SSMUX0                 (STLR_ADC0_BASE + STLR_ADC_SSMUX0_OFFSET)
#define STLR_ADC0_SSMUX1                 (STLR_ADC0_BASE + STLR_ADC_SSMUX1_OFFSET)
#define STLR_ADC0_SSMUX2                 (STLR_ADC0_BASE + STLR_ADC_SSMUX2_OFFSET)
#define STLR_ADC0_SSMUX3                 (STLR_ADC0_BASE + STLR_ADC_SSMUX3_OFFSET)
#define STLR_ADC0_SSCTL0                 (STLR_ADC0_BASE + STLR_ADC_SSCTL0_OFFSET)
#define STLR_ADC0_SSCTL1                 (STLR_ADC0_BASE + STLR_ADC_SSCTL1_OFFSET)
#define STLR_ADC0_SSCTL2                 (STLR_ADC0_BASE + STLR_ADC_SSCTL2_OFFSET)
#define STLR_ADC0_SSCTL3                 (STLR_ADC0_BASE + STLR_ADC_SSCTL3_OFFSET)
#define STLR_ADC0_SSFIFO0                (STLR_ADC0_BASE + STLR_ADC_SSFIFO0_OFFSET)
#define STLR_ADC0_SSFIFO1                (STLR_ADC0_BASE + STLR_ADC_SSFIFO1_OFFSET)
#define STLR_ADC0_SSFIFO2                (STLR_ADC0_BASE + STLR_ADC_SSFIFO2_OFFSET)
#define STLR_ADC0_SSFIFO3                (STLR_ADC0_BASE + STLR_ADC_SSFIFO3_OFFSET)
#define STLR_ADC0_SSFSTAT0               (STLR_ADC0_BASE + STLR_ADC_SSFSTAT0_OFFSET)
#define STLR_ADC0_SSFSTAT1               (STLR_ADC0_BASE + STLR_ADC_SSFSTAT1_OFFSET)
#define STLR_ADC0_SSFSTAT2               (STLR_ADC0_BASE + STLR_ADC_SSFSTAT2_OFFSET)
#define STLR_ADC0_SSFSTAT3               (STLR_ADC0_BASE + STLR_ADC_SSFSTAT3_OFFSET)

/* ADC Active Sample Sequencer (ADCACTSS), offset 0x000 */

#define ADC_ACTSS_ASEN0                  (1 << 0)
#define ADC_ACTSS_ASEN1                  (1 << 1)
#define ADC_ACTSS_ASEN2                  (1 << 2)
#define ADC_ACTSS_ASEN3                  (1 << 3)
#define ADC_ACTSS_BUSY                   (1 << 16)

/* ADC Event Multiplexer Select (ADCEMUX), offset 0x014 */

/* ADC Processor Sample Sequence Initiate (ADCPSSI), offset 0x028 */

#define ADC_PSSI_SS0                     (1 << 0)
#define ADC_PSSI_SS1                     (1 << 1)
#define ADC_PSSI_SS2                     (1 << 2)
#define ADC_PSSI_SS3                     (1 << 3)
#define ADC_PSSI_SYNCWAIT                (1 << 27)
#define ADC_PSSI_GSYNC                   (1 << 31)

/* ADC Sample Sequence FIFO Status */

#define ADC_SSFSTAT_EMPTY                (1 << 8)
#define ADC_SSFSTAT_FULL                 (1 << 12)

/* SSCTL */
#define ADC_SSCTL_D                      (1 << 0)
#define ADC_SSCTL_END                    (1 << 1)
#define ADC_SSCTL_IE                     (1 << 2)
#define ADC_SSCTL_TS                     (1 << 3)

#define ADC_SSFIFO_DATA_MASK             0x0FFF

#endif /* __MACH_PLAT_STELLARIS_ADC_H */