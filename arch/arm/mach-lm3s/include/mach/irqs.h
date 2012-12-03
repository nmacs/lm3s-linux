/*
 * arch/arm/mach-lm3s1d21/include/mach/irqs.h
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

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

/*
 * Interrupt sources
 */

#define NR_IRQS			37

#define LM3S1D21_UART0_IRQ    5
#define LM3S1D21_UART1_IRQ    6
#define LM3S1D21_UART2_IRQ    33

#define LM3S1D21_TIMER0_IRQ   19
#define LM3S1D21_TIMER1_IRQ   21

#define LM3S1D21_SSI0_IRQ     7
#define LM3S1D21_SSI1_IRQ     34

#define LM3S1D21_GPIOA_IRQ    0
#define LM3S1D21_GPIOB_IRQ    1
#define LM3S1D21_GPIOG_IRQ    31

#endif
