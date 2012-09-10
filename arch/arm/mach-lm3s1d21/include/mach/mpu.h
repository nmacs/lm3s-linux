#ifndef _MPU_H
#define _MPU_H

#define MPU_REGIONS_COUNT 8
#define MPU_SUBREGIONS_COUNT 8
#define MPU_REGION_SIZE     ((CONFIG_DRAM_SIZE) / MPU_REGIONS_COUNT)
#define MPU_SUBREGION_SIZE  (MPU_REGION_SIZE / MPU_SUBREGIONS_COUNT)
#define MPU_PAGE_SIZE MPU_SUBREGION_SIZE

struct mpu_state
{
	uint32_t mpu_attr_regs[MPU_REGIONS_COUNT];
};

void update_protections(struct mm_struct *mm);
void protect_page(struct mm_struct *mm, unsigned long addr, unsigned long flags);

#endif // _MPU_H