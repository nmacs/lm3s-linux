#include <linux/init.h>
#include <linux/kernel.h>

#include <mach/hardware.h>

/***************************************************************************/

extern NORET_TYPE void die(const char *str, struct pt_regs *regs, int err);

/***************************************************************************/

NORET_TYPE void lm3s_die(struct pt_regs *regs)
{
	uint32_t regval = lm3s_getreg32(LM3S_SCB_INTCTRL);
	uint32_t exception = (regval & SCB_INTCTRL_VECACT_MASK) >> SCB_INTCTRL_VECACT_SHIFT;

	regval = lm3s_getreg32(LM3S_SCB_FAULTSTAT);
	printk("FAULTSTAT 0x%X\n", regval);

	regval = lm3s_getreg32(LM3S_SCB_HFAULTSTAT);
	printk("HFAULTSTAT 0x%X\n", regval);

	regval = lm3s_getreg32(LM3S_SCB_MMADDR);
	printk("MMADDR 0x%X\n", regval);

	die("LM3S exception", regs, exception);
}