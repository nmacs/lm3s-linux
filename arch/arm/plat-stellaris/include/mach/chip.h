/************************************************************************************
 * arch/plat-stellaris/include/mach/lm3s_chip.h
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

#ifndef __MACH_PLAT_STELLARIS_CHIP_H
#define __MACH_PLAT_STELLARIS_CHIP_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip (only the LM3S6918 and 65 right now) */

#if defined(CONFIG_ARCH_LM3S6918)
#  define STLR_NTIMERS         4  /* Four general purpose timers */
#  define STLR_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  STLR_ETHTS              /* No timestamp register */
#  define STLR_NSSI            2  /* Two SSI modules */
#  define STLR_NUARTS          2  /* Two UART modules */
#  define STLR_NI2C            2  /* Two I2C modules */
#  define STLR_NADC            1  /* One ADC module */
#  define STLR_NPWM            0  /* No PWM generator modules */
#  define STLR_NQEI            0  /* No quadrature encoders */
#  define STLR_NPORTS          8  /* 8 Ports (GPIOA-H) 5-38 GPIOs */
#elif defined(CONFIG_ARCH_LM3S6432)
#  define STLR_NTIMERS         3  /* Three general purpose timers */
#  define STLR_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  STLR_ETHTS              /* No timestamp register */
#  define STLR_NSSI            1  /* One SSI module */
#  define STLR_NUARTS          2  /* Two UART modules */
#  define STLR_NI2C            1  /* Two I2C modules */
#  define STLR_NADC            1  /* One ADC module */
#  define STLR_NPWM            1  /* One PWM generator module */
#  define STLR_NQEI            0  /* No quadrature encoders */
#  define STLR_NPORTS          7  /* 7 Ports (GPIOA-G), 0-42 GPIOs */
#elif defined(CONFIG_ARCH_LM3S6965)
#  define STLR_NTIMERS         4  /* Four general purpose timers */
#  define STLR_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  STLR_ETHTS              /* No timestamp register */
#  define STLR_NSSI            1  /* One SSI module */
#  define STLR_NUARTS          3  /* Three UART modules */
#  define STLR_NI2C            2  /* Two I2C modules */
#  define STLR_NADC            1  /* One ADC module */
#  define STLR_NPWM            3  /* Three PWM generator modules */
#  define STLR_NQEI            2  /* Two quadrature encoders */
#  define STLR_NPORTS          7  /* 7 Ports (GPIOA-G), 0-42 GPIOs */
#elif defined(CONFIG_ARCH_LM3S9B96)
#  define STLR_NTIMERS         4  /* Four general purpose timers */
#  define STLR_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  STLR_ETHTS              /* No timestamp register */
#  define STLR_NSSI            2  /* Two SSI modules */
#  define STLR_NUARTS          3  /* Three UART modules */
#  define STLR_NI2C            2  /* Two I2C modules */
#  define STLR_NADC            2  /* Two ADC module */
#  define STLR_CAN             2  /* Two CAN module */
#  define STLR_NPWM            4  /* Four PWM generator modules */
#  define STLR_NQEI            2  /* Two quadrature encoders */
#  define STLR_NPORTS          9  /* 9 Ports (GPIOA-H,J) 0-65 GPIOs */
#elif defined(CONFIG_ARCH_LM3S8962)
#  define STLR_NTIMERS         4  /* Four general purpose timers */
#  define STLR_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define STLR_NSSI            1  /* One SSI module */
#  define STLR_NUARTS          3  /* Two UART modules */
#  define STLR_NI2C            2  /* One I2C module */
#  define STLR_NADC            1  /* One ADC module */
#  define STLR_NPWM            3  /* Three PWM generator modules */
#  define STLR_NQEI            2  /* Two quadrature encoders */
#  define STLR_NPORTS          7  /* 7 Ports (GPIOA-G), 5-42 GPIOs */
#  define STLR_CANCONTROLLER   1  /* One CAN controller */
#elif defined(CONFIG_ARCH_LM3S1D21)
#  define STLR_NTIMERS         4  /* Four general purpose timers */
#  define STLR_NETHCONTROLLERS 0  /* No Ethernet controller */
#  undef  STLR_ETHTS              /* No timestamp register */
#  define STLR_NSSI            2  /* Two SSI modules */
#  define STLR_NUARTS          3  /* Two UART modules */
#  define STLR_NI2C            2  /* Two I2C modules */
#  define STLR_NADC            1  /* One ADC module */
#  define STLR_NPWM            0  /* No PWM generator modules */
#  define STLR_NQEI            0  /* No quadrature encoders */
#  define STLR_NPORTS          9  /* 8 Ports (GPIOA-H) 5-38 GPIOs */
#  define STLR_NUDMA           32 /* 32 uDMA channels */
#elif defined(CONFIG_ARCH_TM4C12)
#  define STLR_NTIMERS         8  /* Four general purpose timers */
#  define STLR_NETHCONTROLLERS 0  /* No Ethernet controller */
#  undef  STLR_ETHTS              /* No timestamp register */
#  define STLR_NSSI            4  /* Two SSI modules */
#  define STLR_NUARTS          8  /* Two UART modules */
#  define STLR_NI2C            2  /* Two I2C modules */
#  define STLR_NADC            1  /* One ADC module */
#  define STLR_NPWM            0  /* No PWM generator modules */
#  define STLR_NQEI            0  /* No quadrature encoders */
#  define STLR_NPORTS          12 /* 12 Ports (GPIOA-P) */
#  define STLR_NUDMA           32 /* 32 uDMA channels */

#  define STLR_SRAM_SIZE      0x40000
#  define STLR_FLASH_SIZE     0x100000
#else
#  error "Capabilities not specified for this Stellaris chip"
#endif

/* Then get all of the register definitions */

#include "memorymap.h"      /* Memory map */
#include "syscontrol.h"     /* System control module */
#include "epi.h"            /* SDRAM */
#include "gpio.h"           /* GPIO modules */
#include "uart.h"           /* UART modules */
#include "ssi.h"            /* SSI modules */
#include "timer.h"          /* Timer */
#include "internal.h"
#include "scb.h"
#include "watchdog.h"
#include "dma.h"
#include "adc.h"
#include "systick.h"

#ifdef CONFIG_MPU
#  include "mpu.h"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#include <linux/types.h>

# define getreg8(a)           (*(volatile uint8_t *)(a))
# define putreg8(v,a)         (*(volatile uint8_t *)(a) = (v))
# define getreg32(a)          (*(volatile uint32_t *)(a))
# define putreg32(v,a)        (*(volatile uint32_t *)(a) = (v))

/* Some compiler options will convert short loads and stores into byte loads
 * and stores.  We don't want this to happen for IO reads and writes!
 */

/* #define getreg16(a)       (*(volatile uint16_t *)(a)) */
static inline uint16_t getreg16(unsigned int addr)
{
  uint16_t retval;
 __asm__ __volatile__("\tldrh %0, [%1]\n\t" : "=r"(retval) : "r"(addr));
  return retval;
}

/* #define putreg16(v,a)       (*(volatile uint16_t *)(a) = (v)) */
static inline void putreg16(uint16_t val, unsigned int addr)
{
 __asm__ __volatile__("\tstrh %0, [%1]\n\t": : "r"(val), "r"(addr));
}

#endif /* __ASSEMBLY__ */

#endif /* __MACH_PLAT_STELLARIS_CHIP_H */
