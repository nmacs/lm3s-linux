/*
 * MPU support for Stellaris
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

/* Look up the first VMA which satisfies  addr < vm_end,  NULL if none. */
struct vm_area_struct *mmap_find_vma(struct mm_struct *mm, unsigned long addr)
{
	struct vm_area_struct *vma = NULL;
	if (mm) {
		/* Check the cache first. */
		/* (Cache hit rate is typically around 35%.) */
		vma = mm->mmap_cache;
		if (!(vma && vma->vm_end > addr && vma->vm_start <= addr)) {
			struct rb_node * rb_node;

			rb_node = mm->mm_rb.rb_node;
			vma = NULL;

			while (rb_node) {
				struct vm_area_struct * vma_tmp;

				vma_tmp = rb_entry(rb_node,
						struct vm_area_struct, vm_rb);

				if (vma_tmp->vm_end > addr) {
					vma = vma_tmp;
					if (vma_tmp->vm_start <= addr)
						break;
					rb_node = rb_node->rb_left;
				} else
					rb_node = rb_node->rb_right;
			}
			if (vma)
				mm->mmap_cache = vma;
		}
	}
	return vma;
}

/* Look up the first VMA which intersects the interval start_addr..end_addr-1,
   NULL if none.  Assume start_addr < end_addr. */
static inline struct vm_area_struct * mmap_find_vma_intersection(struct mm_struct * mm, unsigned long start_addr, unsigned long end_addr)
{
	struct vm_area_struct * vma = mmap_find_vma(mm,start_addr);

	if (vma && end_addr <= vma->vm_start)
		vma = NULL;
	return vma;
}

void protect_page(struct mm_struct *mm, unsigned long addr, unsigned long flags)
{
	unsigned long offset;
	unsigned long page;
	unsigned long bit;
	unsigned long subregion;
	unsigned long page_subregion;
	uint32_t *mpu_attr_regs;

	if (addr >= (CONFIG_DRAM_BASE + CONFIG_DRAM_SIZE) || addr < CONFIG_DRAM_BASE)
		return;

	offset = addr - CONFIG_DRAM_BASE;
	page = offset >> MPU_REGION_SIZE_POW2;
	subregion = offset >> MPU_SUBREGION_SIZE_POW2;
	page_subregion = (offset & (MPU_REGION_SIZE - 1)) >> MPU_SUBREGION_SIZE_POW2;
	bit = 1 << (page_subregion + MPU_ATTR_SRD_SHIFT);

	mpu_attr_regs = mm->context.mpu_state.mpu_attr_regs;

	if (flags != 0)
	{
		mpu_attr_regs[page] |= bit; // Enable Sub Region
	}
	else
	{
		unsigned long subregion_start = (subregion << MPU_SUBREGION_SIZE_POW2) + CONFIG_DRAM_BASE;
		unsigned long subregion_end = subregion_start + MPU_SUBREGION_SIZE;
		if (!mmap_find_vma_intersection(mm, subregion_start, subregion_end))
		{
			mpu_attr_regs[page] &= ~bit; // Disable Sub Region
		}
	}
}

void update_protections(struct mm_struct *mm)
{
	int i;
	uint32_t *mpu_attr_regs = mm->context.mpu_state.mpu_attr_regs;
	asm("isb":::);
	for (i = 0; i < MPU_REGIONS_COUNT; i++)
	{
		uint32_t regval;
		putreg32(i, STLR_MPU_NUMBER);
		regval = getreg32(STLR_MPU_ATTR);
		regval &= ~MPU_ATTR_SRD_MASK;
		regval |= (~(mpu_attr_regs[i])) & MPU_ATTR_SRD_MASK;
		putreg32(regval, STLR_MPU_ATTR);
	}
	asm("dsb":::);
}

/* Initialize Memory Protection Unit */
static int __init init_mpu(void)
{
	int i;
	uint32_t base = CONFIG_DRAM_BASE;
	/* Enable MPU and allow access for privilaged code for all address space */
	putreg32(MPU_CTRL_ENABLE_MASK | MPU_CTRL_PRIVDEFEN_MASK, STLR_MPU_CTRL);
	asm("dsb":::);

	for (i = 0; i < MPU_REGIONS_COUNT; i++)
	{
		putreg32(i, STLR_MPU_NUMBER);
		putreg32(base, STLR_MPU_BASE);
		putreg32((MPU_ATTR_S_MASK | MPU_ATTR_C_MASK | MPU_ATTR_B_MASK) | // Enable S, C, B bits (external SRAM)
		              (0x03 << MPU_ATTR_AP_SHIFT) |                           // Set AP to 0x03 (enable full access)
		              MPU_ATTR_SRD_MASK |                                     // Disable all subregions
		              MPU_ATTR_ENABLE_MASK |                                  // Enable region
		              ((0x13) << MPU_ATTR_SIZE_SHIFT),                        // Set region size 1MiB = 2^(0x13+1)
									STLR_MPU_ATTR);

		asm("dsb":::);
		base += MPU_REGION_SIZE;
	}

	return 0;
}

device_initcall(init_mpu);
