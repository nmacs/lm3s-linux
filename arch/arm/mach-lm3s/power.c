#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/cpuidle.h>
#include <asm/proc-fns.h>
#include <asm/cpu-single.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/system.h>

#define DEBUG

static int pwr_hold_pin;
static int pwr_switch_pin;
static struct work_struct pwr_switch_irq_work;
static int pwr_switch_irq;
static int power_off_delay;

extern void lm3s_gpioclearint(uint32_t pinset);

static void set_power_hold_pin(int value)
{
#ifdef DEBUG
	printk("Set power hold pin: %i\n", value);
#endif

	lm3s_gpiowrite(pwr_hold_pin, value);
}

static void lm3s_power_off(void)
{
	set_power_hold_pin(0);
	mdelay(10);
	arch_reset(0, 0);
}

static irqreturn_t power_switch_irq(int irq, void *pw)
{
	if( irq != pwr_switch_irq )
		return IRQ_NONE;

#ifdef DEBUG
	printk("Handle Power Switch interrupt\n");
#endif

	lm3s_gpioclearint(pwr_switch_pin);
	disable_irq_nosync(irq);
	schedule_work(&pwr_switch_irq_work);

	return IRQ_HANDLED;
}

static void switch_irq_work(struct work_struct *work)
{
#ifdef DEBUG
	printk("Send SIGPWR to init\n");
#endif

	if( kill_cad_pid(SIGPWR, 1) < 0 )
	{
#ifdef DEBUG
		printk("Signal FAILED, switch off device\n");
#endif
		kernel_power_off();
	}

#ifdef DEBUG
	printk("Signal OK\n");
	printk("Wait %i milliseconds...\n", power_off_delay);
#endif
	msleep(power_off_delay);

	kernel_power_off();

	enable_irq(pwr_switch_irq);
}

void __init lm3s_power_init(int switch_pin, int switch_irq, int hold_pin, int switch_off_delay)
{
#ifdef DEBUG
	printk("LM3S power module init\n");
#endif

	pwr_hold_pin = hold_pin;
	pm_power_off = lm3s_power_off;

	set_power_hold_pin(1);
}

void __init lm3s_power_switch_init(int switch_pin, int switch_irq, int switch_off_delay)
{
	int ret;

#ifdef DEBUG
	printk("LM3S power switch init\n");
#endif

	pwr_switch_pin = switch_pin;
	pwr_switch_irq = switch_irq;
	power_off_delay = switch_off_delay;

	INIT_WORK(&pwr_switch_irq_work, switch_irq_work);

	ret = request_irq(pwr_switch_irq, power_switch_irq, IRQF_TRIGGER_LOW, "power-switch", NULL);
	if (ret < 0) {
		printk("failed to get irq %i\n", pwr_switch_irq);
	}

	lm3s_gpioirqenable(pwr_switch_pin);
}