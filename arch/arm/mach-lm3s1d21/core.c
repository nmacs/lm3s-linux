#include <linux/init.h>
#include <linux/kernel.h>

#include <mach/hardware.h>
#include <mach/memory.h>
#include <mach/irqs.h>

#include <asm/mach-types.h>

#include <asm/hardware/nvic.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>

/***************************************************************************/

extern void __init lm3s1d21_timer_init();

/***************************************************************************/

int board_translate_cs(unsigned int* translated_cs, unsigned int cs)
{
  switch(cs)
  {
  case 0:
    *translated_cs = GPIO_SSI0_CS_SF;
    return 0;
  case 1:
    *translated_cs = GPIO_SSI0_CS_EE;
    return 0;
  case 2:
    *translated_cs = GPIO_SSI0_CS_ETH;
    return 0;
  default:
    return -1;
  }
}

/***************************************************************************/

static void __init init_machine(void)
{
}

/***************************************************************************/

static void __init init_irq(void)
{
  nvic_init();
}

/***************************************************************************/

static void __init timer_init(void)
{
  lm3s1d21_timer_init();
}

static struct sys_timer timer = {
  .init   = timer_init,
};

/***************************************************************************/

static void __init map_io(void)
{
}

/***************************************************************************/
/***************************************************************************/

MACHINE_START(LM3S1D21, "ARM LM3S1D21")
  .phys_io  = LM3S_PERIPH_BASE,
  .io_pg_offst  = (IO_ADDRESS(0) >> 18) & 0xfffc,
  .boot_params  = PHYS_OFFSET + 0x100,
  .map_io   = map_io,
  .init_irq = init_irq,
  .timer    = &timer,
  .init_machine = init_machine,
MACHINE_END