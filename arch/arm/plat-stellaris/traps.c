#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <asm/unwind.h>

#include <mach/hardware.h>

#define ON_ERROR(regval, bit) if ((regval) & bit) printk("\t" bit##_TEXT "\n")

/***************************************************************************/

extern NORET_TYPE void die(const char *str, struct pt_regs *regs, int err);

/***************************************************************************/

NORET_TYPE void stellaris_die(struct pt_regs *regs)
{
	uint32_t regval = getreg32(STLR_SCB_INTCTRL);
	uint32_t exception = (regval & SCB_INTCTRL_VECACT_MASK) >> SCB_INTCTRL_VECACT_SHIFT;
	uint32_t mmaddr = getreg32(STLR_SCB_MMADDR);
	uint32_t faultaddr = getreg32(STLR_SCB_FAULTADDR);

	regval = getreg32(STLR_SCB_FAULTSTAT);
	putreg32(SCB_FAULTSTAT_MASK, STLR_SCB_FAULTSTAT);
	printk("FAULTSTAT [0x%x]:\n", regval);
	ON_ERROR(regval, SCB_FAULTSTAT_IERR);
	ON_ERROR(regval, SCB_FAULTSTAT_DERR);
	ON_ERROR(regval, SCB_FAULTSTAT_MUSTKE);
	ON_ERROR(regval, SCB_FAULTSTAT_MSTKE);
	ON_ERROR(regval, SCB_FAULTSTAT_MLSPERR);
	ON_ERROR(regval, SCB_FAULTSTAT_IBUS);
	ON_ERROR(regval, SCB_FAULTSTAT_PRECISE);
	ON_ERROR(regval, SCB_FAULTSTAT_IMPRE);
	ON_ERROR(regval, SCB_FAULTSTAT_BUSTKE);
	ON_ERROR(regval, SCB_FAULTSTAT_BSTKE);
	ON_ERROR(regval, SCB_FAULTSTAT_BLSPERR);
	ON_ERROR(regval, SCB_FAULTSTAT_UNDEF);
	ON_ERROR(regval, SCB_FAULTSTAT_INVSTAT);
	ON_ERROR(regval, SCB_FAULTSTAT_INVPC);
	ON_ERROR(regval, SCB_FAULTSTAT_NOCP);
	ON_ERROR(regval, SCB_FAULTSTAT_UNALIGN);
	ON_ERROR(regval, SCB_FAULTSTAT_DIV0);

	if (regval & SCB_FAULTSTAT_MMARV)
		printk("\tMemory Management Fault at MMADDR 0x%x\n", mmaddr);
	if (regval & SCB_FAULTSTAT_BFARV)
		printk("\tBus Fault at FAULTADDR 0x%x\n", faultaddr);

	regval = getreg32(STLR_SCB_HFAULTSTAT);
	putreg32(SCB_HFAULTSTAT_MASK, STLR_SCB_HFAULTSTAT);
	printk("HFAULTSTAT [0x%x]:\n", regval);
	ON_ERROR(regval, SCB_HFAULTSTAT_VECT);
	ON_ERROR(regval, SCB_HFAULTSTAT_FORCED);
	ON_ERROR(regval, SCB_HFAULTSTAT_DBG);

	die("Stellaris exception", regs, exception);
}