/*
 * MPU support for LM3S
 *
 * Author: Max Nekludov <macscomp@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <mach/hardware.h>
#include <mach/mpu.h>
#include <linux/mm.h>

#if CONFIG_DRAM_SIZE == 8*1024*1024
#  define DRAM_SIZE_POW2 23
#else
#  error "Unknown DRAM size"
#endif

#define MPU_REGION_SIZE_POW2    (DRAM_SIZE_POW2 - 3)
#define MPU_SUBREGION_SIZE_POW2 (MPU_REGION_SIZE_POW2 - 3)

void protect_page(struct mm_struct *mm, unsigned long addr, unsigned long flags)
{
	unsigned long offset;
	unsigned long page;
	unsigned long bit;
	unsigned long subregion;
	unsigned long page_subregion;
	uint32_t *mpu_attr_regs;
	int *ref_count;

	if (addr >= (CONFIG_DRAM_BASE + CONFIG_DRAM_SIZE) || addr < CONFIG_DRAM_BASE)
		return;

	offset = addr - CONFIG_DRAM_BASE;
	page = offset >> MPU_REGION_SIZE_POW2;
	subregion = offset >> MPU_SUBREGION_SIZE_POW2;
	page_subregion = (offset & ((1 << MPU_REGION_SIZE_POW2) - 1)) >> MPU_SUBREGION_SIZE_POW2;
	bit = 1 << (page_subregion + MPU_ATTR_SRD_SHIFT);

	printk("---------- protect_page 0x%X [flags %i, bit 0x%X]\n",
				 addr, flags, bit);
	printk("offset 0x%X, page %i, page_subregion %i, subregion %i\n",
				 offset, page, page_subregion, subregion);

	mpu_attr_regs = mm->context.mpu_state.mpu_attr_regs;
	ref_count = mm->context.mpu_state.ref_count;

	if (flags != 0)
	{
		if (ref_count[subregion]++ == 0)
			mpu_attr_regs[page] |= bit; // Enable Sub Region
	}
	else
	{
		if (--ref_count[subregion] == 0)
			mpu_attr_regs[page] &= ~bit; // Disable Sub Region
	}
}

void update_protections(struct mm_struct *mm)
{
	uint32_t *mpu_attr_regs = mm->context.mpu_state.mpu_attr_regs;
	asm("isb":::);
	int i;
	for (i = 0; i < MPU_REGIONS_COUNT; i++)
	{
		uint32_t regval;
		lm3s_putreg32(i, LM3S_MPU_NUMBER);
		regval = lm3s_getreg32(LM3S_MPU_ATTR);
		regval &= ~MPU_ATTR_SRD_MASK;
		regval |= (~(mpu_attr_regs[i])) & MPU_ATTR_SRD_MASK;
		lm3s_putreg32(regval, LM3S_MPU_ATTR);
		asm("dsb":::);
	}
}

/* Initialize Memory Protection Unit */
static int __init lm3s_init_mpu(void)
{
	uint32_t base = CONFIG_DRAM_BASE;
	/* Enable MPU and allow access for privilaged code for all address space */
	lm3s_putreg32(MPU_CTRL_ENABLE_MASK | MPU_CTRL_PRIVDEFEN_MASK, LM3S_MPU_CTRL);
	asm("dsb":::);
	int i;
	for (i = 0; i < MPU_REGIONS_COUNT; i++)
	{
		lm3s_putreg32(i, LM3S_MPU_NUMBER);
		lm3s_putreg32(base, LM3S_MPU_BASE);
		lm3s_putreg32((MPU_ATTR_S_MASK | MPU_ATTR_C_MASK | MPU_ATTR_B_MASK) | // Enable S, C, B bits (external SRAM)
		              (0x03 << MPU_ATTR_AP_SHIFT) |                           // Set AP to 0x03 (enable full access)
		              MPU_ATTR_SRD_MASK |                                     // Disable all subregions
		              MPU_ATTR_ENABLE_MASK |                                  // Enable region
		              ((0x13) << MPU_ATTR_SIZE_SHIFT),                        // Set region size 1MiB = 2^(0x13+1)
									LM3S_MPU_ATTR);

		asm("dsb":::);
		base += MPU_REGION_SIZE;
	}

	return 0;
}

device_initcall(lm3s_init_mpu);
