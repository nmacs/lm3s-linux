#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/kernel.h>

#include <mach/hardware.h>
#include <asm/clkdev.h>
#include <mach/clock.h>

static struct clk xtal_clk = {
	.name = "xtal",
	.rate = 12000000,
};

static struct clk_lookup lookups[] = {
	{                       /* XTAL */
		.con_id = "xtal",
		.clk = &xtal_clk,
	},
};

unsigned long clk_get_rate(struct clk *clk)
{
	if (!clk)
		return 0;

	return clk->rate;
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (!clk)
		return -EINVAL;
	return clk->round_rate ? clk->round_rate(clk, rate) : -EINVAL;
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	if (!clk)
		return -EINVAL;
	return clk->set_rate ? clk->set_rate(clk, rate) : -EINVAL;
}
EXPORT_SYMBOL(clk_set_rate);

struct clk *clk_get_parent(struct clk *clk)
{
	if (!clk)
		return NULL;
	return clk->parent;
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	if (!clk)
		return -EINVAL;
	clk->parent = parent;
	return 0;
}

void __init stellaris_clock_init(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(lookups); i++)
		clkdev_add(&lookups[i]);
}