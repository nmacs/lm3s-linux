/*
 * arch/arm/plat-stellaris/include/mach/clock.h
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

#ifndef __MACH_PLAT_STELLARIS_CLOCK_H

struct clk {
	const char           *name;
	int                   id;
	unsigned long         rate;
	struct clk           *parent;
	int                   (*set_rate)(struct clk *c, unsigned long rate);
	long                  (*round_rate)(struct clk *clk, unsigned long rate);
};

#endif /* __MACH_PLAT_STELLARIS_CLOCK_H */