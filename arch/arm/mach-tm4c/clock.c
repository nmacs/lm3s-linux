#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/kernel.h>

#include <mach/hardware.h>
#include <asm/clkdev.h>
#include <mach/clock.h>

static long sys_round_rate(struct clk *c, unsigned long rate)
{
	return (long)rate;
}

extern void update_event_timer(uint32_t new_clock);

static int sys_set_rate(struct clk *c, unsigned long rate)
{
	uint32_t regval;
	
	//printk("------- Set SYSCLK to %u\n", rate);
	
	if (rate > 16000000)
	{
		uint32_t new_div;
		if (c->rate <= 16000000)
		{
			//printk("------- enable PLL\n");
			
			regval = getreg32(STLR_SYSCON_PLLFREQ0);
			regval |= SYSCON_PLLFREQ0_PLLPWR;
			putreg32(regval, STLR_SYSCON_PLLFREQ0);
			
			while( getreg32(STLR_SYSCON_PLLSTAT) == 0 ) {}
			
			//printk("------- enable PLL done\n");
		}
		
		new_div = (480000000 / rate - 1);

		regval = SYSCON_RSCLKCFG_PSYSDIV_SET(new_div) | SYSCON_RSCLKCFG_USEPLL | SYSCON_RSCLKCFG_MEMTIMU |
						SYSCON_RSCLKCFG_PLLSRC_MOSC | SYSCON_RSCLKCFG_NEWFREQ;
		putreg32(regval, STLR_SYSCON_RSCLKCFG);
	}
	else
	{
		//printk("------- disable PLL\n");
		
		uint32_t new_div = (16000000 / rate - 1);
		
		regval = getreg32(STLR_SYSCON_PLLFREQ0);
		regval &= ~SYSCON_PLLFREQ0_PLLPWR;
		putreg32(regval, STLR_SYSCON_PLLFREQ0);
		
		regval = SYSCON_RSCLKCFG_OSYSDIV_SET(new_div) | SYSCON_RSCLKCFG_MEMTIMU | 
		         SYSCON_RSCLKCFG_OSCSRC_PIOSC | SYSCON_RSCLKCFG_NEWFREQ;
		putreg32(regval, STLR_SYSCON_RSCLKCFG);
		
		//printk("------- disable PLL done\n");
	}
	
	c->rate = rate;
	//update_event_timer(rate);
	return 0;
}

static struct clk sys_clk = {
	.name = "sys",
	.rate = 60000000,
	.set_rate = sys_set_rate,
	.round_rate = sys_round_rate,
};

static struct clk_lookup lookups[] = {
	{                       /* SYS */
		.con_id = "sys",
		.clk = &sys_clk,
	},
};

extern void stellaris_clock_init(void);

void __init tm4c_clock_init(void)
{
	int i;

	stellaris_clock_init();

	for (i = 0; i < ARRAY_SIZE(lookups); i++)
		clkdev_add(&lookups[i]);
}