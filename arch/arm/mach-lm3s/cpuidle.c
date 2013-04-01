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
#include <asm/cpu-single.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/sram.h>
#include <linux/delay.h>
#include <linux/leds.h>

#define LM3S_MAX_STATES 2
#define DRIVER_NAME "lm3s-idle"

#ifdef CONFIG_LEDS_TRIGGER_CPUIDLE
DEFINE_LED_TRIGGER(cpuidle_led_trigger);
#endif

struct lm3s_idle_state
{
  int (*enter)(void);
};

/*
static inline void __sram enable_deep_sleep(void)
{
  uint32_t regval;
  regval = lm3s_getreg32(LM3S_SCB_SYSCTRL);
  regval |= SCB_SYSCTRL_SLEEPDEEP_MASK;
  lm3s_putreg32(regval, LM3S_SCB_SYSCTRL);
}

static inline void __sram disable_deep_sleep(void)
{
  uint32_t regval;
  regval = lm3s_getreg32(LM3S_SCB_SYSCTRL);
  regval &= ~SCB_SYSCTRL_SLEEPDEEP_MASK;
  lm3s_putreg32(regval, LM3S_SCB_SYSCTRL);
}
*/

static int __sram cpu_do_sleep(void)
{
	local_irq_disable();
  //enable_deep_sleep();
  asm(
    "ldr r0, =%0\n"     /* r0 = LM3S_EPI0_SDRAMCFG */
    "ldr r1, [r0]\n"    /* r1 = lm3s_getreg32(r0) */
    "ldr r2, =%1\n"     /* r2 = EPI_SDRAMCFG_SLEEP_ON */
    "orr r2, r2, r1\n"  /* r2 |= r1 */
    "str r2, [r0]\n"    /* lm3s_putreg32(r2, r0) - Put SDRAM in self-refresh mode */
    "wfi\n"             /* Sleep */
    "str r1, [r0]\n"    /* lm3s_putreg32(r1, r0) - Restore SDRAM from self-refresh mode */
    "nop\n"             /* Wait SDRAM is ready */
    "nop\n"
    "nop\n"
    "nop\n"
  :
  : "n"(LM3S_EPI0_SDRAMCFG), "n"(EPI_SDRAMCFG_SLEEP_ON)
  : "r0", "r1", "r2"
  );
	local_irq_enable();
  //disable_deep_sleep();
  return 0;
}

/* Actual code that puts the SoC in different idle states */
static int lm3s_enter_idle(struct cpuidle_device *dev,
             struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;
	struct lm3s_idle_state *lm3s_state = state->driver_data;

#ifdef CONFIG_LEDS_TRIGGER_CPUIDLE
	led_trigger_event(cpuidle_led_trigger, LED_OFF);
#endif

	do_gettimeofday(&before);
	lm3s_state->enter();
	do_gettimeofday(&after);
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);

#ifdef CONFIG_LEDS_TRIGGER_CPUIDLE
	led_trigger_event(cpuidle_led_trigger, LED_FULL);
#endif

	return idle_time;
}

static struct lm3s_idle_state lm3s_idle_states[LM3S_MAX_STATES] =
{
	{ .enter = cpu_do_idle },
	{ .enter = cpu_do_sleep },
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
			.target_residency = 50000,
			.driver_data      = &lm3s_idle_states[0],
			.enter            = lm3s_enter_idle,
		},
		{
			.name             = "idle1",
			.desc             = "MCU Sleep & DRAM Self Refresh",
			.exit_latency     = 20,
			.target_residency = 250000,
			.driver_data      = &lm3s_idle_states[1],
			.enter            = lm3s_enter_idle,
		},
	},
};

static struct cpuidle_driver lm3s_idle_driver = {
	.name  = DRIVER_NAME,
	.owner = THIS_MODULE,
};

/* Initialize CPU idle by registering the idle states */
static int __init lm3s_init_cpuidle(void)
{
  cpuidle_register_driver(&lm3s_idle_driver);

  if (cpuidle_register_device(&lm3s_cpuidle_device)) {
    printk(KERN_ERR "%s: Failed registering\n", __func__);
    return -EIO;
  }

#ifdef CONFIG_LEDS_TRIGGER_CPUIDLE
  led_trigger_register_simple("cpuidle", &cpuidle_led_trigger);
#endif

  return 0;
}

device_initcall(lm3s_init_cpuidle);
