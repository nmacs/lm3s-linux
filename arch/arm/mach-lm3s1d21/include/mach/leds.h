/* arch/arm/mach-lm3s1d21/include/mach/leds.h
 *
 * Copyright (C) 2012 Max Nekludov <macscomp@gmail.com>
 *
 * Based on:
 * arch/arm/mach-s3c2410/include/mach/leds-gpio.h
 *
 * Copyright (c) 2006 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * LM3S - LEDs GPIO connector
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_LEDS_H
#define __ASM_ARCH_LEDS_H "leds.h"

#define LM3S_LEDF_ACTLOW	(1<<0)		/* LED is on when GPIO low */

struct lm3s_led_platdata {
	unsigned int		 gpio;
	unsigned int		 flags;

	char			*name;
	char			*def_trigger;
};



#endif /* __ASM_ARCH_LEDSGPIO_H */
