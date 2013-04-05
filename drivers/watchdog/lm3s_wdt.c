/*
 * Watchdog driver for LM3S.
 *
 * (C) 2012 Max Nekludov <macscomp@gmail.com>
 * Based on ks8695_wdt.c by (C) 2007 Andrew Victor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <mach/hardware.h>

#define CURRENT_WDT 0

#define WDT_DEFAULT_TIME	30	/* seconds */
#define WDT_MAX_TIME		(0xFFFFFFFF / CLOCK_TICK_RATE)	/* seconds */

static int wdt_time = WDT_DEFAULT_TIME;

module_param(wdt_time, int, 0);
MODULE_PARM_DESC(wdt_time, "Watchdog time in seconds. (default="
					__MODULE_STRING(WDT_DEFAULT_TIME) ")");

static unsigned long lm3s_wdt_busy;
static spinlock_t lm3s_lock;

/* ......................................................................... */

static inline void _wdt_unlock(void)
{
	lm3s_putreg32(WATCHDOG_WDTLOCK_MAGIC, LM3S_WATCHDOG_WDTLOCK(CURRENT_WDT));
}

/*
 * Reload the watchdog timer.  (ie, pat the watchdog)
 */
static inline void lm3s_wdt_reload(void)
{
	spin_lock(&lm3s_lock);
	lm3s_putreg32(1, LM3S_WATCHDOG_WDTICR(CURRENT_WDT));
	spin_unlock(&lm3s_lock);
}

/*
 * Disable the watchdog.
 */
static inline void lm3s_wdt_stop(void)
{
	uint32_t regval;
	spin_lock(&lm3s_lock);
	regval = lm3s_getreg32(LM3S_SYSCON_RCGC0);
	regval &= ~SYSCON_RCGC0_WDT;
	lm3s_putreg32(regval, LM3S_SYSCON_RCGC0);
	spin_unlock(&lm3s_lock);
}

/*
 * Enable and reset the watchdog.
 */
static inline void lm3s_wdt_start(void)
{
	uint32_t regval;
	uint32_t tval = wdt_time * (CLOCK_TICK_RATE / 2);

	spin_lock(&lm3s_lock);
	regval = lm3s_getreg32(LM3S_SYSCON_RCGC0);
	regval |= SYSCON_RCGC0_WDT;
	lm3s_putreg32(regval, LM3S_SYSCON_RCGC0);

	_wdt_unlock();

	lm3s_putreg32(tval, LM3S_WATCHDOG_WDTLOAD(CURRENT_WDT));
	lm3s_wdt_reload();

	regval = lm3s_getreg32(LM3S_WATCHDOG_WDTCTL(CURRENT_WDT));
	regval |= WATCHDOG_WDTCTL_RESEN_MASK;
	lm3s_putreg32(regval, LM3S_WATCHDOG_WDTCTL(CURRENT_WDT));
	regval |= WATCHDOG_WDTCTL_INTEN_MASK;
	lm3s_putreg32(regval, LM3S_WATCHDOG_WDTCTL(CURRENT_WDT));
	spin_unlock(&lm3s_lock);
}

/*
 * Change the watchdog time interval.
 */
static int lm3s_wdt_settimeout(int new_time)
{
	if ((new_time <= 0) || (new_time > WDT_MAX_TIME))
		return -EINVAL;

	/* Set new watchdog time. It will be used when
	   lm3s_wdt_start() is called. */
	wdt_time = new_time;
	return 0;
}

/* ......................................................................... */

/*
 * Watchdog device is opened, and watchdog starts running.
 */
static int lm3s_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &lm3s_wdt_busy))
		return -EBUSY;

	lm3s_wdt_start();
	return nonseekable_open(inode, file);
}

/*
 * Close the watchdog device.
 * If CONFIG_WATCHDOG_NOWAYOUT is NOT defined then the watchdog is also
 *  disabled.
 */
static int lm3s_wdt_close(struct inode *inode, struct file *file)
{
	clear_bit(0, &lm3s_wdt_busy);
	return 0;
}

static struct watchdog_info lm3s_wdt_info = {
	.identity	= "lm3s watchdog",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
};

/*
 * Handle commands from user-space.
 */
static long lm3s_wdt_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_value;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &lm3s_wdt_info,
					sizeof(lm3s_wdt_info)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_SETOPTIONS:
		if (get_user(new_value, p))
			return -EFAULT;
		if (new_value & WDIOS_DISABLECARD)
			lm3s_wdt_stop();
		if (new_value & WDIOS_ENABLECARD)
			lm3s_wdt_start();
		return 0;
	case WDIOC_KEEPALIVE:
		lm3s_wdt_reload();	/* pat the watchdog */
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;
		if (lm3s_wdt_settimeout(new_value))
			return -EINVAL;
		/* Enable new time value */
		if (lm3s_wdt_busy)
			lm3s_wdt_start();
		/* Return current value */
		return put_user(wdt_time, p);
	case WDIOC_GETTIMEOUT:
		return put_user(wdt_time, p);
	default:
		return -ENOTTY;
	}
}

/*
 * Pat the watchdog whenever device is written to.
 */
static ssize_t lm3s_wdt_write(struct file *file, const char *data,
						size_t len, loff_t *ppos)
{
	lm3s_wdt_reload();		/* pat the watchdog */
	return len;
}

/* ......................................................................... */

static const struct file_operations lm3s_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= lm3s_wdt_ioctl,
	.open		= lm3s_wdt_open,
	.release	= lm3s_wdt_close,
	.write		= lm3s_wdt_write,
};

static struct miscdevice lm3s_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &lm3s_wdt_fops,
};

static int __devinit lm3s_wdt_probe(struct platform_device *pdev)
{
	int res;

	if (lm3s_wdt_miscdev.parent)
		return -EBUSY;
	lm3s_wdt_miscdev.parent = &pdev->dev;

	res = misc_register(&lm3s_wdt_miscdev);
	if (res)
		return res;

	printk(KERN_INFO "LM3S Watchdog Timer enabled (%d seconds), nowayout\n", wdt_time);
	return 0;
}

static int __devexit lm3s_wdt_remove(struct platform_device *pdev)
{
	int res;

	res = misc_deregister(&lm3s_wdt_miscdev);
	if (!res)
		lm3s_wdt_miscdev.parent = NULL;

	return res;
}

static void lm3s_wdt_shutdown(struct platform_device *pdev)
{
	lm3s_wdt_stop();
}

#ifdef CONFIG_PM

static int lm3s_wdt_suspend(struct platform_device *pdev, pm_message_t message)
{
	lm3s_wdt_stop();
	return 0;
}

static int lm3s_wdt_resume(struct platform_device *pdev)
{
	if (lm3s_wdt_busy)
		lm3s_wdt_start();
	return 0;
}

#else
#define lm3s_wdt_suspend NULL
#define lm3s_wdt_resume	NULL
#endif

static struct platform_driver lm3s_wdt_driver = {
	.probe		= lm3s_wdt_probe,
	.remove		= __devexit_p(lm3s_wdt_remove),
	.shutdown	= lm3s_wdt_shutdown,
	.suspend	= lm3s_wdt_suspend,
	.resume		= lm3s_wdt_resume,
	.driver		= {
		.name	= "lm3s_wdt",
		.owner	= THIS_MODULE,
	},
};

static int __init lm3s_wdt_init(void)
{
	spin_lock_init(&lm3s_lock);
	/* Check that the heartbeat value is within range;
	   if not reset to the default */
	if (lm3s_wdt_settimeout(wdt_time)) {
		lm3s_wdt_settimeout(WDT_DEFAULT_TIME);
		pr_info("lm3s_wdt: wdt_time value must be 1 <= wdt_time <= %i"
					", using %d\n", wdt_time, WDT_MAX_TIME);
	}
	return platform_driver_register(&lm3s_wdt_driver);
}

static void __exit lm3s_wdt_exit(void)
{
	platform_driver_unregister(&lm3s_wdt_driver);
}

module_init(lm3s_wdt_init);
module_exit(lm3s_wdt_exit);

MODULE_AUTHOR("Max Nekludov");
MODULE_DESCRIPTION("Watchdog driver for LM3S");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:lm3s_wdt");
