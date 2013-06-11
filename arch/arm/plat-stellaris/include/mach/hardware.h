/*
 * arch/arm/mach-mps/include/mach/hardware.h
 *
 * Copyright (C) 2009 ARM Ltd.
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
#ifndef __MACH_PLAT_STELLARIS_HARDWARE_H
#define __MACH_PLAT_STELLARIS_HARDWARE_H

#include <asm/sizes.h>
#include "chip.h"

#define IO_ADDRESS(x)		(x)
#define __io_address(n)		__io(IO_ADDRESS(n))

#endif /* __MACH_PLAT_STELLARIS_HARDWARE_H */
