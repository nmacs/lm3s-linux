/*
 *  linux/arch/arm/mach-lm3s1d21/dma.c
 *
 *  Copyright (C) 2012 Max Nekludov <macscomp@gmail.com>
 *
 *  Based on:
 *  linux/arch/arm/mach-rpc/dma.c
 *
 *  Copyright (C) 1998 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  DMA functions specific to RiscPC architecture
 */
#include <linux/slab.h>
#include <linux/mman.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>

#include <asm/page.h>
#include <asm/dma.h>
#include <asm/fiq.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>

struct lm3s_dma_channel {
  uint32_t src_end;
  uint32_t dst_end;
  uint32_t ctrl;
  uint32_t unused;
};

struct lm3s_dma {
  struct dma_struct dma;
  struct lm3s_dma_channel *channel;
};

static int lm3s_request_dma(unsigned int chan, dma_t *dma)
{
  struct lm3s_dma *idma = container_of(dma, struct lm3s_dma, dma);
  return 0;
}

static void lm3s_free_dma(unsigned int chan, dma_t *dma)
{
  struct lm3s_dma *idma = container_of(dma, struct lm3s_dma, dma);
}

static void lm3s_enable_dma(unsigned int chan, dma_t *dma)
{
  struct lm3s_dma *idma = container_of(dma, struct lm3s_dma, dma);
}

static void lm3s_disable_dma(unsigned int chan, dma_t *dma)
{
  struct lm3s_dma *idma = container_of(dma, struct lm3s_dma, dma);
  unsigned long flags;

  local_irq_save(flags);

  local_irq_restore(flags);
}

static int lm3s_set_dma_speed(unsigned int chan, dma_t *dma, int cycle)
{
  return 0;
}

static struct dma_ops lm3s_dma_ops = {
  .type     = "lm3s-dma",
  .request  = lm3s_request_dma,
  .free     = lm3s_free_dma,
  .enable   = lm3s_enable_dma,
  .disable  = lm3s_disable_dma,
};

static struct lm3s_dma_channel __attribute__((aligned(1024)) lm3s_dma_channels[LM3S_NUDMA];
static struct lm3s_dma lm3s_dma[LM3S_NUDMA];

static int __init rpc_dma_init(void)
{
  uint32_t regval;
  uint32_t i;

  /*
   * Enable uDMA clock
   */
  regval = lm3s_getreg32(LM3S_SYSCON_RCGC2);
  regval |= SYSCON_RCGC2_UDMA;
  lm3s_putreg32(regval, LM3S_SYSCON_RCGC2);

  // TODO: Enable uDMA

  /*
   * Setup DMA channels
   */
  for (i = 0; i < LM3S_NUDMA; i++)
  {
    lm3s_dma[i].dma.d_ops = &lm3s_dma_ops;
    lm3s_dma[i].channel = &lm3s_dma_channels[i];

    ret = isa_dma_add(i, &lm3s_dma[i].dma);
    if (ret)
      printk(KERN_ERR "DMA%u: unable to register: %d\n", i, ret);
  }

  return 0;
}
core_initcall(lm3s_dma_init);
