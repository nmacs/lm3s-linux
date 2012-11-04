/*
 * arch/arm/mach-lm3s/include/mach/lm3s_clock.h
 *
 * Copyright (C) 2012 Max Nekludov <macscomp@gmail.com>.
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

#ifndef __ARCH_ARM_SRC_LM3S_LM3S_CLOCK_H

#include <linux/types.h>
#include <linux/init.h>

void __init lm3s1d21_timer_init(void);

#endif