#ifndef __MACH_LM3S_POWER_H
#define __MACH_LM3S_POWER_H

#include <linux/init.h>

extern void __init lm3s_power_init(int switch_pin, int switch_irq, int hold_pin, int switch_off_delay);

#endif // __MACH_LM3S_POWER_H