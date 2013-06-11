#include <linux/init.h>
#include <linux/kernel.h>

#include <mach/hardware.h>

/***************************************************************************/

extern NORET_TYPE void die(const char *str, struct pt_regs *regs, int err);

/***************************************************************************/

NORET_TYPE void stellaris_die(struct pt_regs *regs)
{
	uint32_t regval = getreg32(STLR_SCB_INTCTRL);
	uint32_t exception = (regval & SCB_INTCTRL_VECACT_MASK) >> SCB_INTCTRL_VECACT_SHIFT;

	regval = getreg32(STLR_SCB_FAULTSTAT);
	printk("FAULTSTAT 0x%X\n", regval);

	regval = getreg32(STLR_SCB_HFAULTSTAT);
	printk("HFAULTSTAT 0x%X\n", regval);

	regval = getreg32(STLR_SCB_MMADDR);
	printk("MMADDR 0x%X\n", regval);

	die("Stellaris exception", regs, exception);
}