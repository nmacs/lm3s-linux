/************************************************************************************
 * arch/arm/src/lm3s/lm3s_memorymap.h
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

#ifndef __MACH_PLAT_STELLARIS_MEMORYMAP_H
#define __MACH_PLAT_STELLARIS_MEMORYMAP_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory map ***********************************************************************/

#if defined(CONFIG_ARCH_LM3S6918) || defined(CONFIG_ARCH_LM3S6432) || \
    defined(CONFIG_ARCH_LM3S6965) || defined(CONFIG_ARCH_LM3S8962)
#  define STLR_FLASH_BASE     0x00000000 /* -0x0003ffff: On-chip FLASH */
                                         /* -0x1fffffff: Reserved */
#  define STLR_SRAM_BASE      0x20000000 /* -0x2000ffff: Bit-banded on-chip SRAM */
                                         /* -0x21ffffff: Reserved */
#  define STLR_ASRAM_BASE     0x22000000 /* -0x221fffff: Bit-band alias of 20000000- */
                                         /* -0x3fffffff: Reserved */
#  define STLR_PERIPH_BASE    0x40000000 /* -0x4001ffff: FiRM Peripherals */
                                         /* -0x41ffffff: Peripherals */
#  define STLR_APERIPH_BASE   0x42000000 /* -0x43ffffff: Bit-band alise of 40000000- */
                                         /* -0xdfffffff: Reserved */
#  define STLR_ITM_BASE       0xe0000000 /* -0xe0000fff: Instrumentation Trace Macrocell */
#  define STLR_DWT_BASE       0xe0001000 /* -0xe0001fff: Data Watchpoint and Trace */
#  define STLR_FPB_BASE       0xe0002000 /* -0xe0002fff: Flash Patch and Breakpoint */
                                         /* -0xe000dfff: Reserved */
#  define STLR_NVIC_BASE      0xe000e000 /* -0xe000efff: Nested Vectored Interrupt Controller */
                                         /* -0xe003ffff: Reserved */
#  define STLR_TPIU_BASE      0xe0040000 /* -0xe0040fff: Trace Port Interface Unit */
                                         /* -0xffffffff: Reserved */
#elif defined(CONFIG_ARCH_LM3S1D21)
#  define STLR_FLASH_BASE     0x00000000 /* -0x0003ffff: On-chip FLASH */
                                         /* -0x1fffffff: Reserved */
#  define STLR_SRAM_BASE      0x20000000 /* -0x2000ffff: Bit-banded on-chip SRAM */
                                         /* -0x21ffffff: Reserved */
#  define STLR_ASRAM_BASE     0x22000000 /* -0x221fffff: Bit-band alias of 20000000- */
                                         /* -0x3fffffff: Reserved */
#  define STLR_PERIPH_BASE    0x40000000 /* -0x4001ffff: FiRM Peripherals */
                                         /* -0x41ffffff: Peripherals */
#  define STLR_APERIPH_BASE   0x42000000 /* -0x43ffffff: Bit-band alise of 40000000- */
                                         /* -0xdfffffff: Reserved */
#  define STLR_EPI0RAM_BASE   0x60000000 /* -0xDfffffff: EPI0 mapped peripheral and RAM */
#  define STLR_ITM_BASE       0xe0000000 /* -0xe0000fff: Instrumentation Trace Macrocell */
#  define STLR_DWT_BASE       0xe0001000 /* -0xe0001fff: Data Watchpoint and Trace */
#  define STLR_FPB_BASE       0xe0002000 /* -0xe0002fff: Flash Patch and Breakpoint */
                                         /* -0xe000dfff: Reserved */
#  define STLR_CM3P_BASE      0xE000E000 /*            : Cortex-M3 Peripherals */
#  define STLR_NVIC_BASE      0xe000e000 /* -0xe000efff: Nested Vectored Interrupt Controller */
                                         /* -0xe003ffff: Reserved */
#  define STLR_TPIU_BASE      0xe0040000 /* -0xe0040fff: Trace Port Interface Unit */
                                         /* -0xffffffff: Reserved */
#elif defined(CONFIG_ARCH_LM3S9B96)
#  define STLR_FLASH_BASE     0x00000000 /* -0x0003ffff: On-chip FLASH */
                                         /* -0x1fffffff: Reserved */
#  define STLR_SRAM_BASE      0x20000000 /* -0x2000ffff: Bit-banded on-chip SRAM */
                                         /* -0x21ffffff: Reserved */
#  define STLR_ASRAM_BASE     0x22000000 /* -0x221fffff: Bit-band alias of 20000000- */
                                         /* -0x3fffffff: Reserved */
#  define STLR_PERIPH_BASE    0x40000000 /* -0x4001ffff: FiRM Peripherals */
                                         /* -0x41ffffff: Peripherals */
#  define STLR_APERIPH_BASE   0x42000000 /* -0x43ffffff: Bit-band alise of 40000000- */
                                         /* -0x5fffffff: Reserved */
#  define STLR_EPI0RAM_BASE   0x60000000 /* -0xDfffffff: EPI0 mapped peripheral and RAM */
#  define STLR_ITM_BASE       0xe0000000 /* -0xe0000fff: Instrumentation Trace Macrocell */
#  define STLR_DWT_BASE       0xe0001000 /* -0xe0001fff: Data Watchpoint and Trace */
#  define STLR_FPB_BASE       0xe0002000 /* -0xe0002fff: Flash Patch and Breakpoint */
                                         /* -0xe000dfff: Reserved */
#  define STLR_NVIC_BASE      0xe000e000 /* -0xe000efff: Nested Vectored Interrupt Controller */
                                         /* -0xe003ffff: Reserved */
#  define STLR_TPIU_BASE      0xe0040000 /* -0xe0040fff: Trace Port Interface Unit */
                                         /* -0xffffffff: Reserved */
#elif defined(CONFIG_ARCH_TM4C12)
#  define STLR_FLASH_BASE     0x00000000 /* -0x0003ffff: On-chip FLASH */
                                         /* -0x1fffffff: Reserved */
#  define STLR_SRAM_BASE      0x20000000 /* -0x2000ffff: Bit-banded on-chip SRAM */
                                         /* -0x21ffffff: Reserved */
#  define STLR_ASRAM_BASE     0x22000000 /* -0x221fffff: Bit-band alias of 20000000- */
                                         /* -0x3fffffff: Reserved */
#  define STLR_PERIPH_BASE    0x40000000 /* -0x4001ffff: FiRM Peripherals */
                                         /* -0x41ffffff: Peripherals */
#  define STLR_APERIPH_BASE   0x42000000 /* -0x43ffffff: Bit-band alise of 40000000- */
                                         /* -0xdfffffff: Reserved */
#  define STLR_EPI0RAM_BASE   0x60000000 /* -0xDfffffff: EPI0 mapped peripheral and RAM */
#  define STLR_ITM_BASE       0xe0000000 /* -0xe0000fff: Instrumentation Trace Macrocell */
#  define STLR_DWT_BASE       0xe0001000 /* -0xe0001fff: Data Watchpoint and Trace */
#  define STLR_FPB_BASE       0xe0002000 /* -0xe0002fff: Flash Patch and Breakpoint */
                                         /* -0xe000dfff: Reserved */
#  define STLR_CM3P_BASE      0xE000E000 /*            : Cortex-M3 Peripherals */
#  define STLR_NVIC_BASE      0xe000e000 /* -0xe000efff: Nested Vectored Interrupt Controller */
                                         /* -0xe003ffff: Reserved */
#  define STLR_SYSTICK_BASE   0xE000E000
#  define STLR_TPIU_BASE      0xe0040000 /* -0xe0040fff: Trace Port Interface Unit */
                                         /* -0xffffffff: Reserved */
#else
#  error "Memory map not specified for this Stellaris chip"
#endif

/* Peripheral base addresses ********************************************************/
/* The LM3S6918 and LM3S6965 differ by only the presence or absence of a few differnt
 * peripheral modules.  They could probably be combined into one peripheral memory
 * map.  However, keeping them separate does also provide so early, compile-time
 * error detection that makes the duplication worthwhile.
 */

#if defined(CONFIG_ARCH_LM3S6918)
/* FiRM Peripheral Base Addresses */

#  define STLR_WDOG_BASE      (STLR_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer */
                                                           /* -0x03fff: Reserved */
#  define STLR_GPIOA_BASE     (STLR_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define STLR_GPIOB_BASE     (STLR_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define STLR_GPIOC_BASE     (STLR_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define STLR_GPIOD_BASE     (STLR_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define STLR_SSI0_BASE      (STLR_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
#  define STLR_SSI1_BASE      (STLR_PERIPH_BASE + 0x09000) /* -0x09fff: SSI1 */
                                                           /* -0x0bfff: Reserved */
#  define STLR_UART0_BASE     (STLR_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define STLR_UART1_BASE     (STLR_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define STLR_I2CM0_BASE     (STLR_PERIPH_BASE + 0x20000)  /* -0x207ff: I2C Master 0 */
#  define STLR_I2CS0_BASE     (STLR_PERIPH_BASE + 0x20800)  /* -0x20fff: I2C Slave 0 */
#  define STLR_I2CM1_BASE     (STLR_PERIPH_BASE + 0x21000)  /* -0x217ff: I2C Master 1 */
#  define STLR_I2CS1_BASE     (STLR_PERIPH_BASE + 0x21800)  /* -0x21fff: I2C Slave 1 */
                                                            /* -0x23fff: Reserved */
#  define STLR_GPIOE_BASE     (STLR_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define STLR_GPIOF_BASE     (STLR_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define STLR_GPIOG_BASE     (STLR_PERIPH_BASE + 0x26000)  /* -0x26fff: GPIO Port G */
#  define STLR_GPIOH_BASE     (STLR_PERIPH_BASE + 0x27000)  /* -0x27fff: GPIO Port H */
                                                            /* -0x2ffff: Reserved */
#  define STLR_TIMER0_BASE    (STLR_PERIPH_BASE + 0x30000)  /* -0x30fff: Timer 0 */
#  define STLR_TIMER1_BASE    (STLR_PERIPH_BASE + 0x31000)  /* -0x31fff: Timer 1 */
#  define STLR_TIMER2_BASE    (STLR_PERIPH_BASE + 0x32000)  /* -0x32fff: Timer 2 */
#  define STLR_TIMER3_BASE    (STLR_PERIPH_BASE + 0x33000)  /* -0x33fff: Timer 3 */
                                                            /* -0x37fff: Reserved */
#  define STLR_ADC_BASE       (STLR_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC */
                                                            /* -0x3bfff: Reserved */
#  define STLR_COMPARE_BASE   (STLR_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
                                                            /* -0x47fff: Reserved */
#  define STLR_ETHCON_BASE    (STLR_PERIPH_BASE + 0x48000)  /* -0x48fff: Ethernet Controller */
                                                            /* -0xfcfff: Reserved */
#  define STLR_HIBERNATE_BASE (STLR_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Ethernet Controller */
#  define STLR_FLASHCON_BASE  (STLR_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define STLR_SYSCON_BASE    (STLR_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
                                                            /* -0x1ffffff: Reserved */
#elif defined(CONFIG_ARCH_LM3S6432)
/* FiRM Peripheral Base Addresses */

#  define STLR_WDOG_BASE      (STLR_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer */
                                                           /* -0x03fff: Reserved */
#  define STLR_GPIOA_BASE     (STLR_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define STLR_GPIOB_BASE     (STLR_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define STLR_GPIOC_BASE     (STLR_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define STLR_GPIOD_BASE     (STLR_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define STLR_SSI0_BASE      (STLR_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
                                                           /* -0x0bfff: Reserved */
#  define STLR_UART0_BASE     (STLR_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define STLR_UART1_BASE     (STLR_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define STLR_I2CM0_BASE     (STLR_PERIPH_BASE + 0x20000)  /* -0x207ff: I2C Master 0 */
#  define STLR_I2CS0_BASE     (STLR_PERIPH_BASE + 0x20800)  /* -0x20fff: I2C Slave 0 */
                                                            /* -0x23fff: Reserved */
#  define STLR_GPIOE_BASE     (STLR_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define STLR_GPIOF_BASE     (STLR_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define STLR_GPIOG_BASE     (STLR_PERIPH_BASE + 0x26000)  /* -0x26fff: GPIO Port G */
                                                            /* -0x27fff: Reserved */
#  define STLR_PWM0_BASE      (STLR_PERIPH_BASE + 0x28000)  /* -0x28fff: PWM */
                                                            /* -0x2ffff: Reserved */
#  define STLR_TIMER0_BASE    (STLR_PERIPH_BASE + 0x30000)  /* -0x30fff: Timer 0 */
#  define STLR_TIMER1_BASE    (STLR_PERIPH_BASE + 0x31000)  /* -0x31fff: Timer 1 */
#  define STLR_TIMER2_BASE    (STLR_PERIPH_BASE + 0x32000)  /* -0x32fff: Timer 2 */
                                                            /* -0x37fff: Reserved */
#  define STLR_ADC_BASE       (STLR_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC */
                                                            /* -0x3bfff: Reserved */
#  define STLR_COMPARE_BASE   (STLR_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
                                                            /* -0x47fff: Reserved */
#  define STLR_ETHCON_BASE    (STLR_PERIPH_BASE + 0x48000)  /* -0x48fff: Ethernet Controller */
                                                            /* -0xfcfff: Reserved */
#  define STLR_HIBERNATE_BASE (STLR_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Ethernet Controller */
#  define STLR_FLASHCON_BASE  (STLR_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define STLR_SYSCON_BASE    (STLR_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
                                                            /* -0x1ffffff: Reserved */

#elif defined(CONFIG_ARCH_LM3S6965)
/* FiRM Peripheral Base Addresses */

#  define STLR_WDOG_BASE      (STLR_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer */
                                                           /* -0x03fff: Reserved */
#  define STLR_GPIOA_BASE     (STLR_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define STLR_GPIOB_BASE     (STLR_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define STLR_GPIOC_BASE     (STLR_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define STLR_GPIOD_BASE     (STLR_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define STLR_SSI0_BASE      (STLR_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
                                                           /* -0x0bfff: Reserved */
#  define STLR_UART0_BASE     (STLR_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define STLR_UART1_BASE     (STLR_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
#  define STLR_UART2_BASE     (STLR_PERIPH_BASE + 0x0e000) /* -0x0dfff: UART2 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define STLR_I2CM0_BASE     (STLR_PERIPH_BASE + 0x20000)  /* -0x207ff: I2C Master 0 */
#  define STLR_I2CS0_BASE     (STLR_PERIPH_BASE + 0x20800)  /* -0x20fff: I2C Slave 0 */
#  define STLR_I2CM1_BASE     (STLR_PERIPH_BASE + 0x21000)  /* -0x217ff: I2C Master 1 */
#  define STLR_I2CS1_BASE     (STLR_PERIPH_BASE + 0x21800)  /* -0x21fff: I2C Slave 1 */
                                                            /* -0x23fff: Reserved */
#  define STLR_GPIOE_BASE     (STLR_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define STLR_GPIOF_BASE     (STLR_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define STLR_GPIOG_BASE     (STLR_PERIPH_BASE + 0x26000)  /* -0x26fff: GPIO Port G */
                                                            /* -0x27fff: Reserved */
#  define STLR_PWM0_BASE      (STLR_PERIPH_BASE + 0x28000)  /* -0x28fff: PWM */
                                                            /* -0x2bfff: Reserved */
#  define STLR_QEI0_BASE      (STLR_PERIPH_BASE + 0x2c000)  /* -0x2cfff: QEI0 */
#  define STLR_QEI1_BASE      (STLR_PERIPH_BASE + 0x2d000)  /* -0x2dfff: QEI1 */
                                                            /* -0x2ffff: Reserved */
#  define STLR_TIMER0_BASE    (STLR_PERIPH_BASE + 0x30000)  /* -0x30fff: Timer 0 */
#  define STLR_TIMER1_BASE    (STLR_PERIPH_BASE + 0x31000)  /* -0x31fff: Timer 1 */
#  define STLR_TIMER2_BASE    (STLR_PERIPH_BASE + 0x32000)  /* -0x32fff: Timer 2 */
#  define STLR_TIMER3_BASE    (STLR_PERIPH_BASE + 0x33000)  /* -0x33fff: Timer 3 */
                                                            /* -0x37fff: Reserved */
#  define STLR_ADC_BASE       (STLR_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC */
                                                            /* -0x3bfff: Reserved */
#  define STLR_COMPARE_BASE   (STLR_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
                                                            /* -0x47fff: Reserved */
#  define STLR_ETHCON_BASE    (STLR_PERIPH_BASE + 0x48000)  /* -0x48fff: Ethernet Controller */
                                                            /* -0xfcfff: Reserved */
#  define STLR_HIBERNATE_BASE (STLR_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Ethernet Controller */
#  define STLR_FLASHCON_BASE  (STLR_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define STLR_SYSCON_BASE    (STLR_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
                                                            /* -0x1ffffff: Reserved */
#elif defined(CONFIG_ARCH_LM3S8962)
/* FiRM Peripheral Base Addresses */

#  define STLR_WDOG_BASE      (STLR_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer */
                                                           /* -0x03fff: Reserved */
#  define STLR_GPIOA_BASE     (STLR_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define STLR_GPIOB_BASE     (STLR_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define STLR_GPIOC_BASE     (STLR_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define STLR_GPIOD_BASE     (STLR_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define STLR_SSI0_BASE      (STLR_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
                                                           /* -0x0bfff: Reserved */
#  define STLR_UART0_BASE     (STLR_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define STLR_UART1_BASE     (STLR_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define STLR_I2CM0_BASE     (STLR_PERIPH_BASE + 0x20000)  /* -0x207ff: I2C Master 0 */
#  define STLR_I2CS0_BASE     (STLR_PERIPH_BASE + 0x20800)  /* -0x20fff: I2C Slave 0 */
                                                            /* -0x23fff: Reserved */
#  define STLR_GPIOE_BASE     (STLR_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define STLR_GPIOF_BASE     (STLR_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define STLR_GPIOG_BASE     (STLR_PERIPH_BASE + 0x26000)  /* -0x26fff: GPIO Port G */
                                                            /* -0x27fff: Reserved */
#  define STLR_PWM0_BASE      (STLR_PERIPH_BASE + 0x28000)  /* -0x28fff: PWM */
                                                            /* -0x2bfff: Reserved */
#  define STLR_QEI0_BASE      (STLR_PERIPH_BASE + 0x2c000)  /* -0x2cfff: QEI0 */
#  define STLR_QEI1_BASE      (STLR_PERIPH_BASE + 0x2d000)  /* -0x2dfff: QEI1 */
                                                            /* -0x2ffff: Reserved */
#  define STLR_TIMER0_BASE    (STLR_PERIPH_BASE + 0x30000)  /* -0x30fff: Timer 0 */
#  define STLR_TIMER1_BASE    (STLR_PERIPH_BASE + 0x31000)  /* -0x31fff: Timer 1 */
#  define STLR_TIMER2_BASE    (STLR_PERIPH_BASE + 0x32000)  /* -0x32fff: Timer 2 */
#  define STLR_TIMER3_BASE    (STLR_PERIPH_BASE + 0x33000)  /* -0x33fff: Timer 3 */
                                                            /* -0x37fff: Reserved */
#  define STLR_ADC_BASE       (STLR_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC */
                                                            /* -0x3bfff: Reserved */
#  define STLR_COMPARE_BASE   (STLR_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
                                                            /* -0x3fffff: Reserved */
#  define STLR_CANCON_BASE    (STLR_PERIPH_BASE + 0x40000)  /* -0x40fff: CAN Controller */
                                                            /* -0x47fff: Reserved */
#  define STLR_ETHCON_BASE    (STLR_PERIPH_BASE + 0x48000)  /* -0x48fff: Ethernet Controller */
                                                            /* -0xfcfff: Reserved */
#  define STLR_HIBERNATE_BASE (STLR_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Hibernation Controller */
#  define STLR_FLASHCON_BASE  (STLR_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define STLR_SYSCON_BASE    (STLR_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
                                                            /* -0x1ffffff: Reserved */
#elif defined(CONFIG_ARCH_LM3S9B96)
/* FiRM Peripheral Base Addresses */

#  define STLR_WDOG_BASE      (STLR_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer */
                                                           /* -0x03fff: Reserved */
#  define STLR_GPIOA_BASE     (STLR_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define STLR_GPIOB_BASE     (STLR_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define STLR_GPIOC_BASE     (STLR_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define STLR_GPIOD_BASE     (STLR_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define STLR_SSI0_BASE      (STLR_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
#  define STLR_SSI1_BASE      (STLR_PERIPH_BASE + 0x09000) /* -0x09fff: SSI0 */
                                                           /* -0x0bfff: Reserved */
#  define STLR_UART0_BASE     (STLR_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define STLR_UART1_BASE     (STLR_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
#  define STLR_UART2_BASE     (STLR_PERIPH_BASE + 0x0e000) /* -0x0dfff: UART2 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define STLR_I2CM0_BASE     (STLR_PERIPH_BASE + 0x20000)  /* -0x207ff: I2C Master 0 */
#  define STLR_I2CS0_BASE     (STLR_PERIPH_BASE + 0x20800)  /* -0x20fff: I2C Slave 0 */
#  define STLR_I2CM1_BASE     (STLR_PERIPH_BASE + 0x21000)  /* -0x217ff: I2C Master 1 */
#  define STLR_I2CS1_BASE     (STLR_PERIPH_BASE + 0x21800)  /* -0x21fff: I2C Slave 1 */
                                                            /* -0x23fff: Reserved */
#  define STLR_GPIOE_BASE     (STLR_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define STLR_GPIOF_BASE     (STLR_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define STLR_GPIOG_BASE     (STLR_PERIPH_BASE + 0x26000)  /* -0x26fff: GPIO Port G */
#  define STLR_GPIOH_BASE     (STLR_PERIPH_BASE + 0x27000)  /* -0x27fff: GPIO Port H */

#  define STLR_PWM0_BASE      (STLR_PERIPH_BASE + 0x28000)  /* -0x28fff: PWM */
                                                            /* -0x2bfff: Reserved */
#  define STLR_QEI0_BASE      (STLR_PERIPH_BASE + 0x2c000)  /* -0x2cfff: QEI0 */
#  define STLR_QEI1_BASE      (STLR_PERIPH_BASE + 0x2d000)  /* -0x2dfff: QEI1 */
                                                            /* -0x2ffff: Reserved */
#  define STLR_TIMER0_BASE    (STLR_PERIPH_BASE + 0x30000)  /* -0x30fff: Timer 0 */
#  define STLR_TIMER1_BASE    (STLR_PERIPH_BASE + 0x31000)  /* -0x31fff: Timer 1 */
#  define STLR_TIMER2_BASE    (STLR_PERIPH_BASE + 0x32000)  /* -0x32fff: Timer 2 */
#  define STLR_TIMER3_BASE    (STLR_PERIPH_BASE + 0x33000)  /* -0x33fff: Timer 3 */
                                                            /* -0x37fff: Reserved */
#  define STLR_ADC0_BASE      (STLR_PERIPH_BASE + 0x38000) /* -0x38fff: ADC 0 */
#  define STLR_ADC1_BASE      (STLR_PERIPH_BASE + 0x39000) /* -0x39fff: ADC 1 */
                                                            /* -0x3bfff: Reserved */
#  define STLR_COMPARE_BASE   (STLR_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
#  define STLR_GPIOJ_BASE     (STLR_PERIPH_BASE + 0x3d000)  /* -0x3dfff: GPIO Port J */
                                                            /* -0x3ffff: Reserved */
#  define STLR_CAN0_BASE      (STLR_PERIPH_BASE + 0x40000)  /* -0x40fff: CAN 0 */
#  define STLR_CAN1_BASE      (STLR_PERIPH_BASE + 0x41000)  /* -0x41fff: CAN 1 */
                                                            /* -0x47fff: Reserved */
#  define STLR_ETHCON_BASE    (STLR_PERIPH_BASE + 0x48000)  /* -0x48fff: Ethernet Controller */
                                                            /* -0x49fff: Reserved */
#  define STLR_USB_BASE       (STLR_PERIPH_BASE + 0x50000)  /* -0x50fff: USB */
                                                            /* -0x53fff: Reserved */
#  define STLR_I2S0_BASE      (STLR_PERIPH_BASE + 0x54000)  /* -0x54fff: I2S 0 */
                                                            /* -0x57fff: Reserved */
#  define STLR_GPIOAAHB_BASE  (STLR_PERIPH_BASE + 0x58000)  /* -0x58fff: GPIO Port A (AHB aperture) */
#  define STLR_GPIOBAHB_BASE  (STLR_PERIPH_BASE + 0x59000)  /* -0x59fff: GPIO Port B (AHB aperture) */
#  define STLR_GPIOCAHB_BASE  (STLR_PERIPH_BASE + 0x5A000)  /* -0x5afff: GPIO Port C (AHB aperture) */
#  define STLR_GPIODAHB_BASE  (STLR_PERIPH_BASE + 0x5B000)  /* -0x5bfff: GPIO Port D (AHB aperture) */
#  define STLR_GPIOEAHB_BASE  (STLR_PERIPH_BASE + 0x5C000)  /* -0x5cfff: GPIO Port E (AHB aperture) */
#  define STLR_GPIOFAHB_BASE  (STLR_PERIPH_BASE + 0x5D000)  /* -0x5dfff: GPIO Port F (AHB aperture) */
#  define STLR_GPIOGAHB_BASE  (STLR_PERIPH_BASE + 0x5E000)  /* -0x5efff: GPIO Port G (AHB aperture) */
#  define STLR_GPIOHAHB_BASE  (STLR_PERIPH_BASE + 0x5F000)  /* -0x5ffff: GPIO Port H (AHB aperture) */
#  define STLR_GPIOJAHB_BASE  (STLR_PERIPH_BASE + 0x60000)  /* -0x60fff: GPIO Port J (AHB aperture) */
                                                            /* -0xcffff: Reserved */
#  define STLR_EPI0_BASE      (STLR_PERIPH_BASE + 0xD0000)  /* -0xd0fff: EPI 0 */
                                                            /* -0xfcfff: Reserved */
#  define STLR_FLASHCON_BASE  (STLR_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define STLR_SYSCON_BASE    (STLR_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
#  define STLR_UDMA_BASE      (STLR_PERIPH_BASE + 0xff000)  /* -0xfffff: System Control */
                                                            /* -0x1ffffff: Reserved */
#elif defined(CONFIG_ARCH_LM3S1D21)
/* FiRM Peripheral Base Addresses */

#  define STLR_WDOG0_BASE     (STLR_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer 0 */
#  define STLR_WDOG1_BASE     (STLR_PERIPH_BASE + 0x01000) /* -0x01fff: Watchdog Timer 1 */
#  define STLR_WDOG_BASE(n)   (STLR_WDOG0_BASE + (n) * 0x1000)
                                                           /* -0x03fff: Reserved */
#  define STLR_GPIOA_BASE     (STLR_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define STLR_GPIOB_BASE     (STLR_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define STLR_GPIOC_BASE     (STLR_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define STLR_GPIOD_BASE     (STLR_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define STLR_SSI0_BASE      (STLR_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
#  define STLR_SSI1_BASE      (STLR_PERIPH_BASE + 0x09000) /* -0x09fff: SSI1 */
                                                           /* -0x0bfff: Reserved */
#  define STLR_UART0_BASE     (STLR_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define STLR_UART1_BASE     (STLR_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
#  define STLR_UART2_BASE     (STLR_PERIPH_BASE + 0x0e000) /* -0x0dfff: UART2 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define STLR_I2CM0_BASE     (STLR_PERIPH_BASE + 0x20000)  /* -0x207ff: I2C Master 0 */
#  define STLR_I2CS0_BASE     (STLR_PERIPH_BASE + 0x20800)  /* -0x20fff: I2C Slave 0 */
#  define STLR_I2CM1_BASE     (STLR_PERIPH_BASE + 0x21000)  /* -0x217ff: I2C Master 1 */
#  define STLR_I2CS1_BASE     (STLR_PERIPH_BASE + 0x21800)  /* -0x21fff: I2C Slave 1 */
                                                            /* -0x23fff: Reserved */
#  define STLR_GPIOE_BASE     (STLR_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define STLR_GPIOF_BASE     (STLR_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define STLR_GPIOG_BASE     (STLR_PERIPH_BASE + 0x26000)  /* -0x26fff: GPIO Port G */
#  define STLR_GPIOH_BASE     (STLR_PERIPH_BASE + 0x27000)  /* -0x27fff: GPIO Port H */
                                                            /* -0x2ffff: Reserved */
#  define STLR_TIMER0_BASE    (STLR_PERIPH_BASE + 0x30000)  /* -0x30fff: Timer 0 */
#  define STLR_TIMER1_BASE    (STLR_PERIPH_BASE + 0x31000)  /* -0x31fff: Timer 1 */
#  define STLR_TIMER2_BASE    (STLR_PERIPH_BASE + 0x32000)  /* -0x32fff: Timer 2 */
#  define STLR_TIMER3_BASE    (STLR_PERIPH_BASE + 0x33000)  /* -0x33fff: Timer 3 */
                                                            /* -0x37fff: Reserved */
#  define STLR_ADC0_BASE       (STLR_PERIPH_BASE + 0x38000) /* -0x38fff: ADC 0 */
                                                            /* -0x3bfff: Reserved */
#  define STLR_COMPARE_BASE   (STLR_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
#  define STLR_GPIOJ_BASE     (STLR_PERIPH_BASE + 0x3d000)  /* -0x3dfff: GPIO Port J */
                                                            /* -0x57fff: Reserved */
#  define STLR_GPIOAAHB_BASE  (STLR_PERIPH_BASE + 0x58000)  /* -0x58fff: GPIO Port A (AHB aperture) */
#  define STLR_GPIOBAHB_BASE  (STLR_PERIPH_BASE + 0x59000)  /* -0x59fff: GPIO Port B (AHB aperture) */
#  define STLR_GPIOCAHB_BASE  (STLR_PERIPH_BASE + 0x5A000)  /* -0x5afff: GPIO Port C (AHB aperture) */
#  define STLR_GPIODAHB_BASE  (STLR_PERIPH_BASE + 0x5B000)  /* -0x5bfff: GPIO Port D (AHB aperture) */
#  define STLR_GPIOEAHB_BASE  (STLR_PERIPH_BASE + 0x5C000)  /* -0x5cfff: GPIO Port E (AHB aperture) */
#  define STLR_GPIOFAHB_BASE  (STLR_PERIPH_BASE + 0x5D000)  /* -0x5dfff: GPIO Port F (AHB aperture) */
#  define STLR_GPIOGAHB_BASE  (STLR_PERIPH_BASE + 0x5E000)  /* -0x5efff: GPIO Port G (AHB aperture) */
#  define STLR_GPIOHAHB_BASE  (STLR_PERIPH_BASE + 0x5F000)  /* -0x5ffff: GPIO Port H (AHB aperture) */
#  define STLR_GPIOJAHB_BASE  (STLR_PERIPH_BASE + 0x60000)  /* -0x60fff: GPIO Port J (AHB aperture) */
                                                            /* -0xcffff: Reserved */
#  define STLR_EPI0_BASE      (STLR_PERIPH_BASE + 0xD0000)  /* -0xd0fff: EPI 0 */
                                                            /* -0xfcfff: Reserved */
#  define STLR_FLASHCON_BASE  (STLR_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define STLR_SYSCON_BASE    (STLR_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
#  define STLR_UDMA_BASE      (STLR_PERIPH_BASE + 0xff000)  /* -0xfffff: System Control */
                                                            /* -0x1ffffff: Reserved */
#elif defined(CONFIG_ARCH_TM4C12)

#  define STLR_GPIOA_BASE     (STLR_PERIPH_BASE + 0x58000) /* GPIO Port A */
#  define STLR_GPIOB_BASE     (STLR_PERIPH_BASE + 0x59000) /* GPIO Port B */
#  define STLR_GPIOC_BASE     (STLR_PERIPH_BASE + 0x5A000) /* GPIO Port C */
#  define STLR_GPIOD_BASE     (STLR_PERIPH_BASE + 0x5B000) /* GPIO Port D */
#  define STLR_GPIOE_BASE     (STLR_PERIPH_BASE + 0x5C000) /* GPIO Port E */
#  define STLR_GPIOF_BASE     (STLR_PERIPH_BASE + 0x5D000) /* GPIO Port F */
#  define STLR_GPIOG_BASE     (STLR_PERIPH_BASE + 0x5E000) /* GPIO Port G */
#  define STLR_GPIOH_BASE     (STLR_PERIPH_BASE + 0x5F000) /* GPIO Port H */
#  define STLR_GPIOK_BASE     (STLR_PERIPH_BASE + 0x61000) /* GPIO Port K */
#  define STLR_GPIOL_BASE     (STLR_PERIPH_BASE + 0x62000) /* GPIO Port L */
#  define STLR_GPIOM_BASE     (STLR_PERIPH_BASE + 0x63000) /* GPIO Port M */
#  define STLR_GPIOP_BASE     (STLR_PERIPH_BASE + 0x65000) /* GPIO Port P */

#  define STLR_WDOG0_BASE     (STLR_PERIPH_BASE + 0x00000) /* Watchdog Timer 0 */
#  define STLR_WDOG1_BASE     (STLR_PERIPH_BASE + 0x01000) /* Watchdog Timer 1 */
#  define STLR_WDOG_BASE(n)   (STLR_WDOG0_BASE + (n) * 0x1000)

#  define STLR_SSI0_BASE      (STLR_PERIPH_BASE + 0x08000) /* SSI0 */
#  define STLR_SSI1_BASE      (STLR_PERIPH_BASE + 0x09000) /* SSI1 */
#  define STLR_SSI2_BASE      (STLR_PERIPH_BASE + 0x0A000) /* SSI2 */
#  define STLR_SSI3_BASE      (STLR_PERIPH_BASE + 0x0B000) /* SSI3 */

#  define STLR_UART0_BASE     (STLR_PERIPH_BASE + 0x0C000) /* UART0 */
#  define STLR_UART1_BASE     (STLR_PERIPH_BASE + 0x0D000) /* UART1 */
#  define STLR_UART2_BASE     (STLR_PERIPH_BASE + 0x0E000) /* UART2 */
#  define STLR_UART3_BASE     (STLR_PERIPH_BASE + 0x0F000) /* UART3 */
#  define STLR_UART4_BASE     (STLR_PERIPH_BASE + 0x10000) /* UART4 */
#  define STLR_UART5_BASE     (STLR_PERIPH_BASE + 0x11000) /* UART5 */
#  define STLR_UART6_BASE     (STLR_PERIPH_BASE + 0x12000) /* UART6 */
#  define STLR_UART7_BASE     (STLR_PERIPH_BASE + 0x13000) /* UART7 */

#  define STLR_TIMER0_BASE    (STLR_PERIPH_BASE + 0x30000)  /* Timer 0 */
#  define STLR_TIMER1_BASE    (STLR_PERIPH_BASE + 0x31000)  /* Timer 1 */
#  define STLR_TIMER2_BASE    (STLR_PERIPH_BASE + 0x32000)  /* Timer 2 */
#  define STLR_TIMER3_BASE    (STLR_PERIPH_BASE + 0x33000)  /* Timer 3 */
#  define STLR_TIMER4_BASE    (STLR_PERIPH_BASE + 0x34000)  /* Timer 4 */
#  define STLR_TIMER5_BASE    (STLR_PERIPH_BASE + 0x35000)  /* Timer 5 */
#  define STLR_TIMER6_BASE    (STLR_PERIPH_BASE + 0xE0000)  /* Timer 6 */
#  define STLR_TIMER7_BASE    (STLR_PERIPH_BASE + 0xE1000)  /* Timer 7 */

#  define STLR_ADC0_BASE      (STLR_PERIPH_BASE + 0x38000)  /* ADC 0 */
#  define STLR_ADC1_BASE      (STLR_PERIPH_BASE + 0x39000)  /* ADC 1 */

#  define STLR_EPI0_BASE      (STLR_PERIPH_BASE + 0xD0000)  /* EPI 0 */

#  define STLR_FLASHCON_BASE  (STLR_PERIPH_BASE + 0xFD000)  /* FLASH Control */
#  define STLR_EEPROMCON_BASE (STLR_PERIPH_BASE + 0xAF000)  /* EEPROM Control */
#  define STLR_ROMCON_BASE    (STLR_PERIPH_BASE + 0xFE000)  /* ROM Control */
#  define STLR_SYSCON_BASE    (STLR_PERIPH_BASE + 0xFE000)  /* System Control */

#  define STLR_UDMA_BASE      (STLR_PERIPH_BASE + 0xFF000)  /* uDMA */
#else
#  error "Peripheral base addresses not specified for this Stellaris chip"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __MACH_PLAT_STELLARIS_MEMORYMAP_H */
