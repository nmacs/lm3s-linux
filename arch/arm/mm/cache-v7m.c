#include <linux/init.h>
#include <asm/system.h>
#include <asm/cputype.h>
#include <asm/cacheflush.h>
#include <asm/kmap_types.h>
#include <asm/fixmap.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>
#include "mm.h"

#ifdef CONFIG_CPU_V7M

void v7m_flush_kern_cache_all(void) { }
void v7m_flush_user_cache_all(void) { }
void v7m_flush_user_cache_range(unsigned long a, unsigned long b, unsigned int c) { }

void v7m_coherent_kern_range(unsigned long a, unsigned long b) { }
void v7m_coherent_user_range(unsigned long a, unsigned long b) { }
void v7m_flush_kern_dcache_page(void *a) { }
void v7m_flush_kern_dcache_area(void *a, size_t s) { }

void v7m_dma_inv_range(const void *a, const void *b) { }
void v7m_dma_clean_range(const void *a, const void *b) { }
void v7m_dma_flush_range(const void *a, const void *b) { }

void v7m_dma_map_area(const void *s, size_t l, int f) { }
void v7m_dma_unmap_area(const void *s, size_t l, int f) { }

#endif
