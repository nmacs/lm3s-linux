/*
 * based on arch/arm/mach-at91/cpuidle.c
 *
 *
 * CPU idle support for LM3S
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * The cpu idle uses Sleep, Deep-Sleep and RAM self refresh in order
 * to implement two idle states -
 * #1 Sleep
 * #2 Deep-Sleep and RAM self refresh
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/cpuidle.h>
#include <asm/proc-fns.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/sram.h>
#include <linux/delay.h>

#define LM3S_MAX_STATES 2
#define DRIVER_NAME "lm3s-idle"

struct lm3s_idle_state
{
  void (*enter)();
};

static void enter_idle0()
{
  cpu_do_idle();
}

static void __sram enter_idle1()
{
  uint32_t sdram_cfg;

  sdram_cfg = lm3s_getreg32(LM3S_EPI0_SDRAMCFG);
  /* Put SDRAM in self-refresh mode */
  lm3s_putreg32(sdram_cfg | EPI_SDRAMCFG_SLEEP_ON, LM3S_EPI0_SDRAMCFG);
  /* Sleep */
  asm("wfi");
  /* Restore SDRAM from self-refresh mode */
  lm3s_putreg32(sdram_cfg, LM3S_EPI0_SDRAMCFG);
}

/* Actual code that puts the SoC in different idle states */
static int lm3s_enter_idle(struct cpuidle_device *dev,
             struct cpuidle_state *state)
{
  struct timeval before, after;
  int idle_time;
  struct lm3s_idle_state *lm3s_state = state->driver_data;

  local_irq_disable();
  do_gettimeofday(&before);
  lm3s_state->enter();
  do_gettimeofday(&after);
  local_irq_enable();
  idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
      (after.tv_usec - before.tv_usec);

  return idle_time;
}

static struct lm3s_idle_state lm3s_idle_states[LM3S_MAX_STATES] =
{
  { .enter = enter_idle0, },
  { .enter = enter_idle1, },
};

struct cpuidle_device lm3s_cpuidle_device =
{
  .state_count = LM3S_MAX_STATES,
  .states =
  {
    {
      .name             = "idle0",
      .desc             = "MCU Sleep",
      .exit_latency     = 1,
      .target_residency = 10000,
      .driver_data      = &lm3s_idle_states[0],
      .enter            = lm3s_enter_idle,
    },
    {
      .name             = "idle1",
      .desc             = "Deep-Sleep & DRAM Self Refresh",
      .exit_latency     = 20,
      .target_residency = 10000,
      .driver_data      = &lm3s_idle_states[1],
      .enter            = lm3s_enter_idle,
    },
  },
};

static struct cpuidle_driver lm3s_idle_driver = {
  .name =         DRIVER_NAME,
  .owner =        THIS_MODULE,
};

/* Initialize CPU idle by registering the idle states */
static int __init lm3s_init_cpuidle(void)
{
  struct cpuidle_device *device;
  uint32_t regval;

  // Enable Deep-Sleep mode
  regval = lm3s_getreg32(0xE000ED10);
  regval |= (1 << 2);
  lm3s_putreg32(regval, 0xE000ED10);

  cpuidle_register_driver(&lm3s_idle_driver);

  if (cpuidle_register_device(&lm3s_cpuidle_device)) {
    printk(KERN_ERR "%s: Failed registering\n", __func__);
    return -EIO;
  }

  return 0;
}

device_initcall(lm3s_init_cpuidle);
