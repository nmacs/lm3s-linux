/*
 * arch/arm/plat-stellaris/include/mach/irqs.h
 *
 * Copyright (C) 2012
 * Max Nekludov <macscomo@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __MACH_PLAT_STELLARIS_IRQS_H
#define __MACH_PLAT_STELLARIS_IRQS_H

/*
 * Interrupt sources
 */

#define STLR_UART0_IRQ    5
#define STLR_UART1_IRQ    6
#define STLR_UART2_IRQ    33

#define STLR_TIMER0_IRQ   19
#define STLR_TIMER1_IRQ   21

#define STLR_SSI0_IRQ     7
#define STLR_SSI1_IRQ     34

#define STLR_GPIOA_IRQ    0
#define STLR_GPIOB_IRQ    1
#define STLR_GPIOC_IRQ    2
#define STLR_GPIOD_IRQ    3
#define STLR_GPIOE_IRQ    4
#define STLR_GPIOF_IRQ    30
#define STLR_GPIOG_IRQ    31
#define STLR_GPIOH_IRQ    32
#define STLR_GPIOK_IRQ    52
#define STLR_GPIOL_IRQ    53
#define STLR_GPIOM_IRQ    72

#if defined(CONFIG_ARCH_LM3S)
# define NR_IRQS			37
#elif defined(CONFIG_ARCH_TM4C)
# define NR_IRQS			114
#endif

#endif /* __MACH_PLAT_STELLARIS_IRQS_H */
