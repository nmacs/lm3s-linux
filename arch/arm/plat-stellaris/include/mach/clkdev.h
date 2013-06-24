#ifndef __MACH_PLAT_STELLARIS_CLKDEV_H
#define __MACH_PLAT_STELLARIS_CLKDEV_H

#include <mach/clock.h>

static inline int __clk_get(struct clk *clk)
{
	return 1;
}

static inline void __clk_put(struct clk *clk)
{
}

#endif
