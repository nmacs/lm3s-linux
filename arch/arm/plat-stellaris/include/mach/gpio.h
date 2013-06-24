/************************************************************************************
 * arch/arm/plat-stellaris/include/mach/gpio_def.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __MACH_PLAT_STELLARIS_GPIO_H
#define __MACH_PLAT_STELLARIS_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* GPIO Register Offsets ************************************************************/

#define STLR_GPIO_DATA_OFFSET         0x000 /* GPIO Data */
#define STLR_GPIO_DIR_OFFSET          0x400 /* GPIO Direction */
#define STLR_GPIO_IS_OFFSET           0x404 /* GPIO Interrupt Sense */
#define STLR_GPIO_IBE_OFFSET          0x408 /* GPIO Interrupt Both Edges */
#define STLR_GPIO_IEV_OFFSET          0x40c /* GPIO Interrupt Event */
#define STLR_GPIO_IM_OFFSET           0x410 /* GPIO Interrupt Mask */
#define STLR_GPIO_RIS_OFFSET          0x414 /* GPIO Raw Interrupt Status */
#define STLR_GPIO_MIS_OFFSET          0x418 /* GPIO Masked Interrupt Status */
#define STLR_GPIO_ICR_OFFSET          0x41c /* GPIO Interrupt Clear */
#define STLR_GPIO_AFSEL_OFFSET        0x420 /* GPIO Alternate Function */
#define STLR_GPIO_DR2R_OFFSET         0x500 /* Select GPIO 2-mA Drive Select */
#define STLR_GPIO_DR4R_OFFSET         0x504 /* GPIO 4-mA Drive Select */
#define STLR_GPIO_DR8R_OFFSET         0x508 /* GPIO 8-mA Drive Select */
#define STLR_GPIO_ODR_OFFSET          0x50c /* GPIO Open Drain Select */
#define STLR_GPIO_PUR_OFFSET          0x510 /* GPIO Pull-Up Select */
#define STLR_GPIO_PDR_OFFSET          0x514 /* GPIO Pull-Down Select */
#define STLR_GPIO_SLR_OFFSET          0x518 /* GPIO Slew Rate Control Select */
#define STLR_GPIO_DEN_OFFSET          0x51C /* GPIO Digital Enable */
#define STLR_GPIO_LOCK_OFFSET         0x520 /* GPIO Lock */
#define STLR_GPIO_CR_OFFSET           0x524 /* GPIO Commit */
#define STLR_GPIO_AMSEL_OFFSET        0x528 /* GPIO Analog Mode Select */
#define STLR_GPIO_PCTL_OFFSET         0X52C /* GPIO Port Control */
#define STLR_GPIO_PERIPHID4_OFFSET    0xfd0 /* GPIO Peripheral Identification 4 */
#define STLR_GPIO_PERIPHID5_OFFSET    0xfd4 /* GPIO Peripheral Identification 5 */
#define STLR_GPIO_PERIPHID6_OFFSET    0xfd8 /* GPIO Peripheral Identification 6 */
#define STLR_GPIO_PERIPHID7_OFFSET    0xfdc /* GPIO Peripheral Identification 7 */
#define STLR_GPIO_PERIPHID0_OFFSET    0xfe0 /* GPIO Peripheral Identification 0 */
#define STLR_GPIO_PERIPHID1_OFFSET    0xfe4 /* GPIO Peripheral Identification 1 */
#define STLR_GPIO_PERIPHID2_OFFSET    0xfe8 /* GPIO Peripheral Identification 2 */
#define STLR_GPIO_PERIPHID3_OFFSET    0xfec /* GPIO Peripheral Identification 3 */
#define STLR_GPIO_PCELLID0_OFFSET     0xff0 /* GPIO PrimeCell Identification 0 */
#define STLR_GPIO_PCELLID1_OFFSET     0xff4 /* GPIO PrimeCell Identification 1 */
#define STLR_GPIO_PCELLID2_OFFSET     0xff8 /* GPIO PrimeCell Identification 2 */
#define STLR_GPIO_PCELLID3_OFFSET     0xffc /* GPIO PrimeCell Identification 3*/

/* GPIO Register Addresses **********************************************************/

#define STLR_GPIOA_DATA               (STLR_GPIOA_BASE + STLR_GPIO_DATA_OFFSET)
#define STLR_GPIOA_DIR                (STLR_GPIOA_BASE + STLR_GPIO_DIR_OFFSET)
#define STLR_GPIOA_IS                 (STLR_GPIOA_BASE + STLR_GPIO_IS_OFFSET)
#define STLR_GPIOA_IBE                (STLR_GPIOA_BASE + STLR_GPIO_IBE_OFFSET)
#define STLR_GPIOA_IEV                (STLR_GPIOA_BASE + STLR_GPIO_IEV_OFFSET)
#define STLR_GPIOA_IM                 (STLR_GPIOA_BASE + STLR_GPIO_IM_OFFSET)
#define STLR_GPIOA_RIS                (STLR_GPIOA_BASE + STLR_GPIO_RIS_OFFSET)
#define STLR_GPIOA_MIS                (STLR_GPIOA_BASE + STLR_GPIO_MIS_OFFSET)
#define STLR_GPIOA_ICR                (STLR_GPIOA_BASE + STLR_GPIO_ICR_OFFSET)
#define STLR_GPIOA_AFSEL              (STLR_GPIOA_BASE + STLR_GPIO_AFSEL_OFFSET)
#define STLR_GPIOA_DR2R               (STLR_GPIOA_BASE + STLR_GPIO_DR2R_OFFSET)
#define STLR_GPIOA_DR4R               (STLR_GPIOA_BASE + STLR_GPIO_DR4R_OFFSET)
#define STLR_GPIOA_DR8R               (STLR_GPIOA_BASE + STLR_GPIO_DR8R_OFFSET)
#define STLR_GPIOA_ODR                (STLR_GPIOA_BASE + STLR_GPIO_ODR_OFFSET)
#define STLR_GPIOA_PUR                (STLR_GPIOA_BASE + STLR_GPIO_PUR_OFFSET)
#define STLR_GPIOA_PDR                (STLR_GPIOA_BASE + STLR_GPIO_PDR_OFFSET)
#define STLR_GPIOA_SLR                (STLR_GPIOA_BASE + STLR_GPIO_SLR_OFFSET)
#define STLR_GPIOA_DEN                (STLR_GPIOA_BASE + STLR_GPIO_DEN_OFFSET)
#define STLR_GPIOA_LOCK               (STLR_GPIOA_BASE + STLR_GPIO_LOCK_OFFSET)
#define STLR_GPIOA_CR                 (STLR_GPIOA_BASE + STLR_GPIO_CR_OFFSET)
#define STLR_GPIOA_PERIPHID4          (STLR_GPIOA_BASE + STLR_GPIO_PERIPHID4_OFFSET)
#define STLR_GPIOA_PERIPHID5          (STLR_GPIOA_BASE + STLR_GPIO_PERIPHID5_OFFSET)
#define STLR_GPIOA_PERIPHID6          (STLR_GPIOA_BASE + STLR_GPIO_PERIPHID6_OFFSET)
#define STLR_GPIOA_PERIPHID7          (STLR_GPIOA_BASE + STLR_GPIO_PERIPHID7_OFFSET)
#define STLR_GPIOA_PERIPHID0          (STLR_GPIOA_BASE + STLR_GPIO_PERIPHID0_OFFSET)
#define STLR_GPIOA_PERIPHID1          (STLR_GPIOA_BASE + STLR_GPIO_PERIPHID1_OFFSET)
#define STLR_GPIOA_PERIPHID2          (STLR_GPIOA_BASE + STLR_GPIO_PERIPHID2_OFFSET)
#define STLR_GPIOA_PERIPHID3          (STLR_GPIOA_BASE + STLR_GPIO_PERIPHID3_OFFSET)
#define STLR_GPIOA_PCELLID0           (STLR_GPIOA_BASE + STLR_GPIO_PCELLID0_OFFSET)
#define STLR_GPIOA_PCELLID1           (STLR_GPIOA_BASE + STLR_GPIO_PCELLID1_OFFSET)
#define STLR_GPIOA_PCELLID2           (STLR_GPIOA_BASE + STLR_GPIO_PCELLID2_OFFSET)
#define STLR_GPIOA_PCELLID3           (STLR_GPIOA_BASE + STLR_GPIO_PCELLID3_OFFSET)

#define STLR_GPIOB_DATA               (STLR_GPIOB_BASE + STLR_GPIO_DATA_OFFSET)
#define STLR_GPIOB_DIR                (STLR_GPIOB_BASE + STLR_GPIO_DIR_OFFSET)
#define STLR_GPIOB_IS                 (STLR_GPIOB_BASE + STLR_GPIO_IS_OFFSET)
#define STLR_GPIOB_IBE                (STLR_GPIOB_BASE + STLR_GPIO_IBE_OFFSET)
#define STLR_GPIOB_IEV                (STLR_GPIOB_BASE + STLR_GPIO_IEV_OFFSET)
#define STLR_GPIOB_IM                 (STLR_GPIOB_BASE + STLR_GPIO_IM_OFFSET)
#define STLR_GPIOB_RIS                (STLR_GPIOB_BASE + STLR_GPIO_RIS_OFFSET)
#define STLR_GPIOB_MIS                (STLR_GPIOB_BASE + STLR_GPIO_MIS_OFFSET)
#define STLR_GPIOB_ICR                (STLR_GPIOB_BASE + STLR_GPIO_ICR_OFFSET)
#define STLR_GPIOB_AFSEL              (STLR_GPIOB_BASE + STLR_GPIO_AFSEL_OFFSET)
#define STLR_GPIOB_DR2R               (STLR_GPIOB_BASE + STLR_GPIO_DR2R_OFFSET)
#define STLR_GPIOB_DR4R               (STLR_GPIOB_BASE + STLR_GPIO_DR4R_OFFSET)
#define STLR_GPIOB_DR8R               (STLR_GPIOB_BASE + STLR_GPIO_DR8R_OFFSET)
#define STLR_GPIOB_ODR                (STLR_GPIOB_BASE + STLR_GPIO_ODR_OFFSET)
#define STLR_GPIOB_PUR                (STLR_GPIOB_BASE + STLR_GPIO_PUR_OFFSET)
#define STLR_GPIOB_PDR                (STLR_GPIOB_BASE + STLR_GPIO_PDR_OFFSET)
#define STLR_GPIOB_SLR                (STLR_GPIOB_BASE + STLR_GPIO_SLR_OFFSET)
#define STLR_GPIOB_DEN                (STLR_GPIOB_BASE + STLR_GPIO_DEN_OFFSET)
#define STLR_GPIOB_LOCK               (STLR_GPIOB_BASE + STLR_GPIO_LOCK_OFFSET)
#define STLR_GPIOB_CR                 (STLR_GPIOB_BASE + STLR_GPIO_CR_OFFSET)
#define STLR_GPIOB_PERIPHID4          (STLR_GPIOB_BASE + STLR_GPIO_PERIPHID4_OFFSET)
#define STLR_GPIOB_PERIPHID5          (STLR_GPIOB_BASE + STLR_GPIO_PERIPHID5_OFFSET)
#define STLR_GPIOB_PERIPHID6          (STLR_GPIOB_BASE + STLR_GPIO_PERIPHID6_OFFSET)
#define STLR_GPIOB_PERIPHID7          (STLR_GPIOB_BASE + STLR_GPIO_PERIPHID7_OFFSET)
#define STLR_GPIOB_PERIPHID0          (STLR_GPIOB_BASE + STLR_GPIO_PERIPHID0_OFFSET)
#define STLR_GPIOB_PERIPHID1          (STLR_GPIOB_BASE + STLR_GPIO_PERIPHID1_OFFSET)
#define STLR_GPIOB_PERIPHID2          (STLR_GPIOB_BASE + STLR_GPIO_PERIPHID2_OFFSET)
#define STLR_GPIOB_PERIPHID3          (STLR_GPIOB_BASE + STLR_GPIO_PERIPHID3_OFFSET)
#define STLR_GPIOB_PCELLID0           (STLR_GPIOB_BASE + STLR_GPIO_PCELLID0_OFFSET)
#define STLR_GPIOB_PCELLID1           (STLR_GPIOB_BASE + STLR_GPIO_PCELLID1_OFFSET)
#define STLR_GPIOB_PCELLID2           (STLR_GPIOB_BASE + STLR_GPIO_PCELLID2_OFFSET)
#define STLR_GPIOB_PCELLID3           (STLR_GPIOB_BASE + STLR_GPIO_PCELLID3_OFFSET)

#define STLR_GPIOC_DATA               (STLR_GPIOC_BASE + STLR_GPIO_DATA_OFFSET)
#define STLR_GPIOC_DIR                (STLR_GPIOC_BASE + STLR_GPIO_DIR_OFFSET)
#define STLR_GPIOC_IS                 (STLR_GPIOC_BASE + STLR_GPIO_IS_OFFSET)
#define STLR_GPIOC_IBE                (STLR_GPIOC_BASE + STLR_GPIO_IBE_OFFSET)
#define STLR_GPIOC_IEV                (STLR_GPIOC_BASE + STLR_GPIO_IEV_OFFSET)
#define STLR_GPIOC_IM                 (STLR_GPIOC_BASE + STLR_GPIO_IM_OFFSET)
#define STLR_GPIOC_RIS                (STLR_GPIOC_BASE + STLR_GPIO_RIS_OFFSET)
#define STLR_GPIOC_MIS                (STLR_GPIOC_BASE + STLR_GPIO_MIS_OFFSET)
#define STLR_GPIOC_ICR                (STLR_GPIOC_BASE + STLR_GPIO_ICR_OFFSET)
#define STLR_GPIOC_AFSEL              (STLR_GPIOC_BASE + STLR_GPIO_AFSEL_OFFSET)
#define STLR_GPIOC_DR2R               (STLR_GPIOC_BASE + STLR_GPIO_DR2R_OFFSET)
#define STLR_GPIOC_DR4R               (STLR_GPIOC_BASE + STLR_GPIO_DR4R_OFFSET)
#define STLR_GPIOC_DR8R               (STLR_GPIOC_BASE + STLR_GPIO_DR8R_OFFSET)
#define STLR_GPIOC_ODR                (STLR_GPIOC_BASE + STLR_GPIO_ODR_OFFSET)
#define STLR_GPIOC_PUR                (STLR_GPIOC_BASE + STLR_GPIO_PUR_OFFSET)
#define STLR_GPIOC_PDR                (STLR_GPIOC_BASE + STLR_GPIO_PDR_OFFSET)
#define STLR_GPIOC_SLR                (STLR_GPIOC_BASE + STLR_GPIO_SLR_OFFSET)
#define STLR_GPIOC_DEN                (STLR_GPIOC_BASE + STLR_GPIO_DEN_OFFSET)
#define STLR_GPIOC_LOCK               (STLR_GPIOC_BASE + STLR_GPIO_LOCK_OFFSET)
#define STLR_GPIOC_CR                 (STLR_GPIOC_BASE + STLR_GPIO_CR_OFFSET)
#define STLR_GPIOC_PERIPHID4          (STLR_GPIOC_BASE + STLR_GPIO_PERIPHID4_OFFSET)
#define STLR_GPIOC_PERIPHID5          (STLR_GPIOC_BASE + STLR_GPIO_PERIPHID5_OFFSET)
#define STLR_GPIOC_PERIPHID6          (STLR_GPIOC_BASE + STLR_GPIO_PERIPHID6_OFFSET)
#define STLR_GPIOC_PERIPHID7          (STLR_GPIOC_BASE + STLR_GPIO_PERIPHID7_OFFSET)
#define STLR_GPIOC_PERIPHID0          (STLR_GPIOC_BASE + STLR_GPIO_PERIPHID0_OFFSET)
#define STLR_GPIOC_PERIPHID1          (STLR_GPIOC_BASE + STLR_GPIO_PERIPHID1_OFFSET)
#define STLR_GPIOC_PERIPHID2          (STLR_GPIOC_BASE + STLR_GPIO_PERIPHID2_OFFSET)
#define STLR_GPIOC_PERIPHID3          (STLR_GPIOC_BASE + STLR_GPIO_PERIPHID3_OFFSET)
#define STLR_GPIOC_PCELLID0           (STLR_GPIOC_BASE + STLR_GPIO_PCELLID0_OFFSET)
#define STLR_GPIOC_PCELLID1           (STLR_GPIOC_BASE + STLR_GPIO_PCELLID1_OFFSET)
#define STLR_GPIOC_PCELLID2           (STLR_GPIOC_BASE + STLR_GPIO_PCELLID2_OFFSET)
#define STLR_GPIOC_PCELLID3           (STLR_GPIOC_BASE + STLR_GPIO_PCELLID3_OFFSET)

#define STLR_GPIOD_DATA               (STLR_GPIOD_BASE + STLR_GPIO_DATA_OFFSET)
#define STLR_GPIOD_DIR                (STLR_GPIOD_BASE + STLR_GPIO_DIR_OFFSET)
#define STLR_GPIOD_IS                 (STLR_GPIOD_BASE + STLR_GPIO_IS_OFFSET)
#define STLR_GPIOD_IBE                (STLR_GPIOD_BASE + STLR_GPIO_IBE_OFFSET)
#define STLR_GPIOD_IEV                (STLR_GPIOD_BASE + STLR_GPIO_IEV_OFFSET)
#define STLR_GPIOD_IM                 (STLR_GPIOD_BASE + STLR_GPIO_IM_OFFSET)
#define STLR_GPIOD_RIS                (STLR_GPIOD_BASE + STLR_GPIO_RIS_OFFSET)
#define STLR_GPIOD_MIS                (STLR_GPIOD_BASE + STLR_GPIO_MIS_OFFSET)
#define STLR_GPIOD_ICR                (STLR_GPIOD_BASE + STLR_GPIO_ICR_OFFSET)
#define STLR_GPIOD_AFSEL              (STLR_GPIOD_BASE + STLR_GPIO_AFSEL_OFFSET)
#define STLR_GPIOD_DR2R               (STLR_GPIOD_BASE + STLR_GPIO_DR2R_OFFSET)
#define STLR_GPIOD_DR4R               (STLR_GPIOD_BASE + STLR_GPIO_DR4R_OFFSET)
#define STLR_GPIOD_DR8R               (STLR_GPIOD_BASE + STLR_GPIO_DR8R_OFFSET)
#define STLR_GPIOD_ODR                (STLR_GPIOD_BASE + STLR_GPIO_ODR_OFFSET)
#define STLR_GPIOD_PUR                (STLR_GPIOD_BASE + STLR_GPIO_PUR_OFFSET)
#define STLR_GPIOD_PDR                (STLR_GPIOD_BASE + STLR_GPIO_PDR_OFFSET)
#define STLR_GPIOD_SLR                (STLR_GPIOD_BASE + STLR_GPIO_SLR_OFFSET)
#define STLR_GPIOD_DEN                (STLR_GPIOD_BASE + STLR_GPIO_DEN_OFFSET)
#define STLR_GPIOD_LOCK               (STLR_GPIOD_BASE + STLR_GPIO_LOCK_OFFSET)
#define STLR_GPIOD_CR                 (STLR_GPIOD_BASE + STLR_GPIO_CR_OFFSET)
#define STLR_GPIOD_PERIPHID4          (STLR_GPIOD_BASE + STLR_GPIO_PERIPHID4_OFFSET)
#define STLR_GPIOD_PERIPHID5          (STLR_GPIOD_BASE + STLR_GPIO_PERIPHID5_OFFSET)
#define STLR_GPIOD_PERIPHID6          (STLR_GPIOD_BASE + STLR_GPIO_PERIPHID6_OFFSET)
#define STLR_GPIOD_PERIPHID7          (STLR_GPIOD_BASE + STLR_GPIO_PERIPHID7_OFFSET)
#define STLR_GPIOD_PERIPHID0          (STLR_GPIOD_BASE + STLR_GPIO_PERIPHID0_OFFSET)
#define STLR_GPIOD_PERIPHID1          (STLR_GPIOD_BASE + STLR_GPIO_PERIPHID1_OFFSET)
#define STLR_GPIOD_PERIPHID2          (STLR_GPIOD_BASE + STLR_GPIO_PERIPHID2_OFFSET)
#define STLR_GPIOD_PERIPHID3          (STLR_GPIOD_BASE + STLR_GPIO_PERIPHID3_OFFSET)
#define STLR_GPIOD_PCELLID0           (STLR_GPIOD_BASE + STLR_GPIO_PCELLID0_OFFSET)
#define STLR_GPIOD_PCELLID1           (STLR_GPIOD_BASE + STLR_GPIO_PCELLID1_OFFSET)
#define STLR_GPIOD_PCELLID2           (STLR_GPIOD_BASE + STLR_GPIO_PCELLID2_OFFSET)
#define STLR_GPIOD_PCELLID3           (STLR_GPIOD_BASE + STLR_GPIO_PCELLID3_OFFSET)

#define STLR_GPIOE_DATA               (STLR_GPIOE_BASE + STLR_GPIO_DATA_OFFSET)
#define STLR_GPIOE_DIR                (STLR_GPIOE_BASE + STLR_GPIO_DIR_OFFSET)
#define STLR_GPIOE_IS                 (STLR_GPIOE_BASE + STLR_GPIO_IS_OFFSET)
#define STLR_GPIOE_IBE                (STLR_GPIOE_BASE + STLR_GPIO_IBE_OFFSET)
#define STLR_GPIOE_IEV                (STLR_GPIOE_BASE + STLR_GPIO_IEV_OFFSET)
#define STLR_GPIOE_IM                 (STLR_GPIOE_BASE + STLR_GPIO_IM_OFFSET)
#define STLR_GPIOE_RIS                (STLR_GPIOE_BASE + STLR_GPIO_RIS_OFFSET)
#define STLR_GPIOE_MIS                (STLR_GPIOE_BASE + STLR_GPIO_MIS_OFFSET)
#define STLR_GPIOE_ICR                (STLR_GPIOE_BASE + STLR_GPIO_ICR_OFFSET)
#define STLR_GPIOE_AFSEL              (STLR_GPIOE_BASE + STLR_GPIO_AFSEL_OFFSET)
#define STLR_GPIOE_DR2R               (STLR_GPIOE_BASE + STLR_GPIO_DR2R_OFFSET)
#define STLR_GPIOE_DR4R               (STLR_GPIOE_BASE + STLR_GPIO_DR4R_OFFSET)
#define STLR_GPIOE_DR8R               (STLR_GPIOE_BASE + STLR_GPIO_DR8R_OFFSET)
#define STLR_GPIOE_ODR                (STLR_GPIOE_BASE + STLR_GPIO_ODR_OFFSET)
#define STLR_GPIOE_PUR                (STLR_GPIOE_BASE + STLR_GPIO_PUR_OFFSET)
#define STLR_GPIOE_PDR                (STLR_GPIOE_BASE + STLR_GPIO_PDR_OFFSET)
#define STLR_GPIOE_SLR                (STLR_GPIOE_BASE + STLR_GPIO_SLR_OFFSET)
#define STLR_GPIOE_DEN                (STLR_GPIOE_BASE + STLR_GPIO_DEN_OFFSET)
#define STLR_GPIOE_LOCK               (STLR_GPIOE_BASE + STLR_GPIO_LOCK_OFFSET)
#define STLR_GPIOE_CR                 (STLR_GPIOE_BASE + STLR_GPIO_CR_OFFSET)
#define STLR_GPIOE_PERIPHID4          (STLR_GPIOE_BASE + STLR_GPIO_PERIPHID4_OFFSET)
#define STLR_GPIOE_PERIPHID5          (STLR_GPIOE_BASE + STLR_GPIO_PERIPHID5_OFFSET)
#define STLR_GPIOE_PERIPHID6          (STLR_GPIOE_BASE + STLR_GPIO_PERIPHID6_OFFSET)
#define STLR_GPIOE_PERIPHID7          (STLR_GPIOE_BASE + STLR_GPIO_PERIPHID7_OFFSET)
#define STLR_GPIOE_PERIPHID0          (STLR_GPIOE_BASE + STLR_GPIO_PERIPHID0_OFFSET)
#define STLR_GPIOE_PERIPHID1          (STLR_GPIOE_BASE + STLR_GPIO_PERIPHID1_OFFSET)
#define STLR_GPIOE_PERIPHID2          (STLR_GPIOE_BASE + STLR_GPIO_PERIPHID2_OFFSET)
#define STLR_GPIOE_PERIPHID3          (STLR_GPIOE_BASE + STLR_GPIO_PERIPHID3_OFFSET)
#define STLR_GPIOE_PCELLID0           (STLR_GPIOE_BASE + STLR_GPIO_PCELLID0_OFFSET)
#define STLR_GPIOE_PCELLID1           (STLR_GPIOE_BASE + STLR_GPIO_PCELLID1_OFFSET)
#define STLR_GPIOE_PCELLID2           (STLR_GPIOE_BASE + STLR_GPIO_PCELLID2_OFFSET)
#define STLR_GPIOE_PCELLID3           (STLR_GPIOE_BASE + STLR_GPIO_PCELLID3_OFFSET)

#define STLR_GPIOF_DATA               (STLR_GPIOF_BASE + STLR_GPIO_DATA_OFFSET)
#define STLR_GPIOF_DIR                (STLR_GPIOF_BASE + STLR_GPIO_DIR_OFFSET)
#define STLR_GPIOF_IS                 (STLR_GPIOF_BASE + STLR_GPIO_IS_OFFSET)
#define STLR_GPIOF_IBE                (STLR_GPIOF_BASE + STLR_GPIO_IBE_OFFSET)
#define STLR_GPIOF_IEV                (STLR_GPIOF_BASE + STLR_GPIO_IEV_OFFSET)
#define STLR_GPIOF_IM                 (STLR_GPIOF_BASE + STLR_GPIO_IM_OFFSET)
#define STLR_GPIOF_RIS                (STLR_GPIOF_BASE + STLR_GPIO_RIS_OFFSET)
#define STLR_GPIOF_MIS                (STLR_GPIOF_BASE + STLR_GPIO_MIS_OFFSET)
#define STLR_GPIOF_ICR                (STLR_GPIOF_BASE + STLR_GPIO_ICR_OFFSET)
#define STLR_GPIOF_AFSEL              (STLR_GPIOF_BASE + STLR_GPIO_AFSEL_OFFSET)
#define STLR_GPIOF_DR2R               (STLR_GPIOF_BASE + STLR_GPIO_DR2R_OFFSET)
#define STLR_GPIOF_DR4R               (STLR_GPIOF_BASE + STLR_GPIO_DR4R_OFFSET)
#define STLR_GPIOF_DR8R               (STLR_GPIOF_BASE + STLR_GPIO_DR8R_OFFSET)
#define STLR_GPIOF_ODR                (STLR_GPIOF_BASE + STLR_GPIO_ODR_OFFSET)
#define STLR_GPIOF_PUR                (STLR_GPIOF_BASE + STLR_GPIO_PUR_OFFSET)
#define STLR_GPIOF_PDR                (STLR_GPIOF_BASE + STLR_GPIO_PDR_OFFSET)
#define STLR_GPIOF_SLR                (STLR_GPIOF_BASE + STLR_GPIO_SLR_OFFSET)
#define STLR_GPIOF_DEN                (STLR_GPIOF_BASE + STLR_GPIO_DEN_OFFSET)
#define STLR_GPIOF_LOCK               (STLR_GPIOF_BASE + STLR_GPIO_LOCK_OFFSET)
#define STLR_GPIOF_CR                 (STLR_GPIOF_BASE + STLR_GPIO_CR_OFFSET)
#define STLR_GPIOF_PERIPHID4          (STLR_GPIOF_BASE + STLR_GPIO_PERIPHID4_OFFSET)
#define STLR_GPIOF_PERIPHID5          (STLR_GPIOF_BASE + STLR_GPIO_PERIPHID5_OFFSET)
#define STLR_GPIOF_PERIPHID6          (STLR_GPIOF_BASE + STLR_GPIO_PERIPHID6_OFFSET)
#define STLR_GPIOF_PERIPHID7          (STLR_GPIOF_BASE + STLR_GPIO_PERIPHID7_OFFSET)
#define STLR_GPIOF_PERIPHID0          (STLR_GPIOF_BASE + STLR_GPIO_PERIPHID0_OFFSET)
#define STLR_GPIOF_PERIPHID1          (STLR_GPIOF_BASE + STLR_GPIO_PERIPHID1_OFFSET)
#define STLR_GPIOF_PERIPHID2          (STLR_GPIOF_BASE + STLR_GPIO_PERIPHID2_OFFSET)
#define STLR_GPIOF_PERIPHID3          (STLR_GPIOF_BASE + STLR_GPIO_PERIPHID3_OFFSET)
#define STLR_GPIOF_PCELLID0           (STLR_GPIOF_BASE + STLR_GPIO_PCELLID0_OFFSET)
#define STLR_GPIOF_PCELLID1           (STLR_GPIOF_BASE + STLR_GPIO_PCELLID1_OFFSET)
#define STLR_GPIOF_PCELLID2           (STLR_GPIOF_BASE + STLR_GPIO_PCELLID2_OFFSET)
#define STLR_GPIOF_PCELLID3           (STLR_GPIOF_BASE + STLR_GPIO_PCELLID3_OFFSET)

#define STLR_GPIOG_DATA               (STLR_GPIOG_BASE + STLR_GPIO_DATA_OFFSET)
#define STLR_GPIOG_DIR                (STLR_GPIOG_BASE + STLR_GPIO_DIR_OFFSET)
#define STLR_GPIOG_IS                 (STLR_GPIOG_BASE + STLR_GPIO_IS_OFFSET)
#define STLR_GPIOG_IBE                (STLR_GPIOG_BASE + STLR_GPIO_IBE_OFFSET)
#define STLR_GPIOG_IEV                (STLR_GPIOG_BASE + STLR_GPIO_IEV_OFFSET)
#define STLR_GPIOG_IM                 (STLR_GPIOG_BASE + STLR_GPIO_IM_OFFSET)
#define STLR_GPIOG_RIS                (STLR_GPIOG_BASE + STLR_GPIO_RIS_OFFSET)
#define STLR_GPIOG_MIS                (STLR_GPIOG_BASE + STLR_GPIO_MIS_OFFSET)
#define STLR_GPIOG_ICR                (STLR_GPIOG_BASE + STLR_GPIO_ICR_OFFSET)
#define STLR_GPIOG_AFSEL              (STLR_GPIOG_BASE + STLR_GPIO_AFSEL_OFFSET)
#define STLR_GPIOG_DR2R               (STLR_GPIOG_BASE + STLR_GPIO_DR2R_OFFSET)
#define STLR_GPIOG_DR4R               (STLR_GPIOG_BASE + STLR_GPIO_DR4R_OFFSET)
#define STLR_GPIOG_DR8R               (STLR_GPIOG_BASE + STLR_GPIO_DR8R_OFFSET)
#define STLR_GPIOG_ODR                (STLR_GPIOG_BASE + STLR_GPIO_ODR_OFFSET)
#define STLR_GPIOG_PUR                (STLR_GPIOG_BASE + STLR_GPIO_PUR_OFFSET)
#define STLR_GPIOG_PDR                (STLR_GPIOG_BASE + STLR_GPIO_PDR_OFFSET)
#define STLR_GPIOG_SLR                (STLR_GPIOG_BASE + STLR_GPIO_SLR_OFFSET)
#define STLR_GPIOG_DEN                (STLR_GPIOG_BASE + STLR_GPIO_DEN_OFFSET)
#define STLR_GPIOG_LOCK               (STLR_GPIOG_BASE + STLR_GPIO_LOCK_OFFSET)
#define STLR_GPIOG_CR                 (STLR_GPIOG_BASE + STLR_GPIO_CR_OFFSET)
#define STLR_GPIOG_PERIPHID4          (STLR_GPIOG_BASE + STLR_GPIO_PERIPHID4_OFFSET)
#define STLR_GPIOG_PERIPHID5          (STLR_GPIOG_BASE + STLR_GPIO_PERIPHID5_OFFSET)
#define STLR_GPIOG_PERIPHID6          (STLR_GPIOG_BASE + STLR_GPIO_PERIPHID6_OFFSET)
#define STLR_GPIOG_PERIPHID7          (STLR_GPIOG_BASE + STLR_GPIO_PERIPHID7_OFFSET)
#define STLR_GPIOG_PERIPHID0          (STLR_GPIOG_BASE + STLR_GPIO_PERIPHID0_OFFSET)
#define STLR_GPIOG_PERIPHID1          (STLR_GPIOG_BASE + STLR_GPIO_PERIPHID1_OFFSET)
#define STLR_GPIOG_PERIPHID2          (STLR_GPIOG_BASE + STLR_GPIO_PERIPHID2_OFFSET)
#define STLR_GPIOG_PERIPHID3          (STLR_GPIOG_BASE + STLR_GPIO_PERIPHID3_OFFSET)
#define STLR_GPIOG_PCELLID0           (STLR_GPIOG_BASE + STLR_GPIO_PCELLID0_OFFSET)
#define STLR_GPIOG_PCELLID1           (STLR_GPIOG_BASE + STLR_GPIO_PCELLID1_OFFSET)
#define STLR_GPIOG_PCELLID2           (STLR_GPIOG_BASE + STLR_GPIO_PCELLID2_OFFSET)
#define STLR_GPIOG_PCELLID3           (STLR_GPIOG_BASE + STLR_GPIO_PCELLID3_OFFSET)

#define STLR_GPIOH_DATA               (STLR_GPIOH_BASE + STLR_GPIO_DATA_OFFSET)
#define STLR_GPIOH_DIR                (STLR_GPIOH_BASE + STLR_GPIO_DIR_OFFSET)
#define STLR_GPIOH_IS                 (STLR_GPIOH_BASE + STLR_GPIO_IS_OFFSET)
#define STLR_GPIOH_IBE                (STLR_GPIOH_BASE + STLR_GPIO_IBE_OFFSET)
#define STLR_GPIOH_IEV                (STLR_GPIOH_BASE + STLR_GPIO_IEV_OFFSET)
#define STLR_GPIOH_IM                 (STLR_GPIOH_BASE + STLR_GPIO_IM_OFFSET)
#define STLR_GPIOH_RIS                (STLR_GPIOH_BASE + STLR_GPIO_RIS_OFFSET)
#define STLR_GPIOH_MIS                (STLR_GPIOH_BASE + STLR_GPIO_MIS_OFFSET)
#define STLR_GPIOH_ICR                (STLR_GPIOH_BASE + STLR_GPIO_ICR_OFFSET)
#define STLR_GPIOH_AFSEL              (STLR_GPIOH_BASE + STLR_GPIO_AFSEL_OFFSET)
#define STLR_GPIOH_DR2R               (STLR_GPIOH_BASE + STLR_GPIO_DR2R_OFFSET)
#define STLR_GPIOH_DR4R               (STLR_GPIOH_BASE + STLR_GPIO_DR4R_OFFSET)
#define STLR_GPIOH_DR8R               (STLR_GPIOH_BASE + STLR_GPIO_DR8R_OFFSET)
#define STLR_GPIOH_ODR                (STLR_GPIOH_BASE + STLR_GPIO_ODR_OFFSET)
#define STLR_GPIOH_PUR                (STLR_GPIOH_BASE + STLR_GPIO_PUR_OFFSET)
#define STLR_GPIOH_PDR                (STLR_GPIOH_BASE + STLR_GPIO_PDR_OFFSET)
#define STLR_GPIOH_SLR                (STLR_GPIOH_BASE + STLR_GPIO_SLR_OFFSET)
#define STLR_GPIOH_DEN                (STLR_GPIOH_BASE + STLR_GPIO_DEN_OFFSET)
#define STLR_GPIOH_LOCK               (STLR_GPIOH_BASE + STLR_GPIO_LOCK_OFFSET)
#define STLR_GPIOH_CR                 (STLR_GPIOH_BASE + STLR_GPIO_CR_OFFSET)
#define STLR_GPIOH_PERIPHID4          (STLR_GPIOH_BASE + STLR_GPIO_PERIPHID4_OFFSET)
#define STLR_GPIOH_PERIPHID5          (STLR_GPIOH_BASE + STLR_GPIO_PERIPHID5_OFFSET)
#define STLR_GPIOH_PERIPHID6          (STLR_GPIOH_BASE + STLR_GPIO_PERIPHID6_OFFSET)
#define STLR_GPIOH_PERIPHID7          (STLR_GPIOH_BASE + STLR_GPIO_PERIPHID7_OFFSET)
#define STLR_GPIOH_PERIPHID0          (STLR_GPIOH_BASE + STLR_GPIO_PERIPHID0_OFFSET)
#define STLR_GPIOH_PERIPHID1          (STLR_GPIOH_BASE + STLR_GPIO_PERIPHID1_OFFSET)
#define STLR_GPIOH_PERIPHID2          (STLR_GPIOH_BASE + STLR_GPIO_PERIPHID2_OFFSET)
#define STLR_GPIOH_PERIPHID3          (STLR_GPIOH_BASE + STLR_GPIO_PERIPHID3_OFFSET)
#define STLR_GPIOH_PCELLID0           (STLR_GPIOH_BASE + STLR_GPIO_PCELLID0_OFFSET)
#define STLR_GPIOH_PCELLID1           (STLR_GPIOH_BASE + STLR_GPIO_PCELLID1_OFFSET)
#define STLR_GPIOH_PCELLID2           (STLR_GPIOH_BASE + STLR_GPIO_PCELLID2_OFFSET)
#define STLR_GPIOH_PCELLID3           (STLR_GPIOH_BASE + STLR_GPIO_PCELLID3_OFFSET)

#define STLR_GPIOJ_DATA               (STLR_GPIOJ_BASE + STLR_GPIO_DATA_OFFSET)
#define STLR_GPIOJ_DIR                (STLR_GPIOJ_BASE + STLR_GPIO_DIR_OFFSET)
#define STLR_GPIOJ_IS                 (STLR_GPIOJ_BASE + STLR_GPIO_IS_OFFSET)
#define STLR_GPIOJ_IBE                (STLR_GPIOJ_BASE + STLR_GPIO_IBE_OFFSET)
#define STLR_GPIOJ_IEV                (STLR_GPIOJ_BASE + STLR_GPIO_IEV_OFFSET)
#define STLR_GPIOJ_IM                 (STLR_GPIOJ_BASE + STLR_GPIO_IM_OFFSET)
#define STLR_GPIOJ_RIS                (STLR_GPIOJ_BASE + STLR_GPIO_RIS_OFFSET)
#define STLR_GPIOJ_MIS                (STLR_GPIOJ_BASE + STLR_GPIO_MIS_OFFSET)
#define STLR_GPIOJ_ICR                (STLR_GPIOJ_BASE + STLR_GPIO_ICR_OFFSET)
#define STLR_GPIOJ_AFSEL              (STLR_GPIOJ_BASE + STLR_GPIO_AFSEL_OFFSET)
#define STLR_GPIOJ_DR2R               (STLR_GPIOJ_BASE + STLR_GPIO_DR2R_OFFSET)
#define STLR_GPIOJ_DR4R               (STLR_GPIOJ_BASE + STLR_GPIO_DR4R_OFFSET)
#define STLR_GPIOJ_DR8R               (STLR_GPIOJ_BASE + STLR_GPIO_DR8R_OFFSET)
#define STLR_GPIOJ_ODR                (STLR_GPIOJ_BASE + STLR_GPIO_ODR_OFFSET)
#define STLR_GPIOJ_PUR                (STLR_GPIOJ_BASE + STLR_GPIO_PUR_OFFSET)
#define STLR_GPIOJ_PDR                (STLR_GPIOJ_BASE + STLR_GPIO_PDR_OFFSET)
#define STLR_GPIOJ_SLR                (STLR_GPIOJ_BASE + STLR_GPIO_SLR_OFFSET)
#define STLR_GPIOJ_DEN                (STLR_GPIOJ_BASE + STLR_GPIO_DEN_OFFSET)
#define STLR_GPIOJ_LOCK               (STLR_GPIOJ_BASE + STLR_GPIO_LOCK_OFFSET)
#define STLR_GPIOJ_CR                 (STLR_GPIOJ_BASE + STLR_GPIO_CR_OFFSET)
#define STLR_GPIOJ_PERIPHID4          (STLR_GPIOJ_BASE + STLR_GPIO_PERIPHID4_OFFSET)
#define STLR_GPIOJ_PERIPHID5          (STLR_GPIOJ_BASE + STLR_GPIO_PERIPHID5_OFFSET)
#define STLR_GPIOJ_PERIPHID6          (STLR_GPIOJ_BASE + STLR_GPIO_PERIPHID6_OFFSET)
#define STLR_GPIOJ_PERIPHID7          (STLR_GPIOJ_BASE + STLR_GPIO_PERIPHID7_OFFSET)
#define STLR_GPIOJ_PERIPHID0          (STLR_GPIOJ_BASE + STLR_GPIO_PERIPHID0_OFFSET)
#define STLR_GPIOJ_PERIPHID1          (STLR_GPIOJ_BASE + STLR_GPIO_PERIPHID1_OFFSET)
#define STLR_GPIOJ_PERIPHID2          (STLR_GPIOJ_BASE + STLR_GPIO_PERIPHID2_OFFSET)
#define STLR_GPIOJ_PERIPHID3          (STLR_GPIOJ_BASE + STLR_GPIO_PERIPHID3_OFFSET)
#define STLR_GPIOJ_PCELLID0           (STLR_GPIOJ_BASE + STLR_GPIO_PCELLID0_OFFSET)
#define STLR_GPIOJ_PCELLID1           (STLR_GPIOJ_BASE + STLR_GPIO_PCELLID1_OFFSET)
#define STLR_GPIOJ_PCELLID2           (STLR_GPIOJ_BASE + STLR_GPIO_PCELLID2_OFFSET)
#define STLR_GPIOJ_PCELLID3           (STLR_GPIOJ_BASE + STLR_GPIO_PCELLID3_OFFSET)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#include <linux/types.h>

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/****************************************************************************
 * Name: lm3s_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int configgpio(uint32_t cfgset);

/****************************************************************************
 * Name: lm3s_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void gpiowrite(uint32_t pinset, int value);

/****************************************************************************
 * Name: lm3s_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

int gpioread(uint32_t pinset, int value);

void gpioirqenable(uint32_t pinset);

#endif /* __ASSEMBLY__ */

#endif /* __MACH_PLAT_STELLARIS_GPIO_H */