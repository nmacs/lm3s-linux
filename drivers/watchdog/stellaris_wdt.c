/*
 * Watchdog driver for Sttellaris.
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

#define CURRENT_WDT 1

#if CURRENT_WDT == 0
# define WATCHDOG_CLOCK_RATE CLOCK_TICK_RATE
#else
# define WATCHDOG_CLOCK_RATE 16000000
#endif

#define WDT_DEFAULT_TIME	60	/* seconds */
#define WDT_MAX_TIME		(0xFFFFFFFF / WATCHDOG_CLOCK_RATE)	/* seconds */

static int wdt_time = WDT_DEFAULT_TIME;

module_param(wdt_time, int, 0);
MODULE_PARM_DESC(wdt_time, "Watchdog time in seconds. (default="
					__MODULE_STRING(WDT_DEFAULT_TIME) ")");

static unsigned long stellaris_wdt_busy;
static spinlock_t stellaris_lock;

/* ......................................................................... */

static inline void putregwdt32(uint32_t value, uint32_t addr)
{
	putreg32(value, addr);
#if CURRENT_WDT == 1
	while( (getreg32(STLR_WATCHDOG_WDTCTL(CURRENT_WDT)) & WATCHDOG_WDTCTL_WRC_MASK) == 0 ) {}
#endif
}

static inline uint32_t getregwdt32(uint32_t addr)
{
	return getreg32(addr);
}

/* ......................................................................... */

static inline void _wdt_unlock(void)
{
	putregwdt32(WATCHDOG_WDTLOCK_MAGIC, STLR_WATCHDOG_WDTLOCK(CURRENT_WDT));
}

/*
 * Reload the watchdog timer.  (ie, pat the watchdog)
 */
static inline void stellaris_wdt_reload(void)
{
	spin_lock(&stellaris_lock);
	putregwdt32(1, STLR_WATCHDOG_WDTICR(CURRENT_WDT));
	spin_unlock(&stellaris_lock);
}

/*
 * Disable the watchdog.
 */
static inline void stellaris_wdt_stop(void)
{
	spin_lock(&stellaris_lock);
	watchdog_clock_ctrl(CURRENT_WDT, SYS_DISABLE_CLOCK);
	spin_unlock(&stellaris_lock);
}

/*
 * Enable and reset the watchdog.
 */
static inline void stellaris_wdt_start(void)
{
	uint32_t regval;
	uint32_t tval = wdt_time * (WATCHDOG_CLOCK_RATE / 2);

	spin_lock(&stellaris_lock);
	watchdog_clock_ctrl(CURRENT_WDT, SYS_ENABLE_CLOCK);
	_wdt_unlock();

	putregwdt32(tval, STLR_WATCHDOG_WDTLOAD(CURRENT_WDT));
	stellaris_wdt_reload();

	regval = getregwdt32(STLR_WATCHDOG_WDTCTL(CURRENT_WDT));
	regval |= WATCHDOG_WDTCTL_RESEN_MASK;
	putregwdt32(regval, STLR_WATCHDOG_WDTCTL(CURRENT_WDT));
	regval |= WATCHDOG_WDTCTL_INTEN_MASK;
	putregwdt32(regval, STLR_WATCHDOG_WDTCTL(CURRENT_WDT));
	spin_unlock(&stellaris_lock);
}

/*
 * Change the watchdog time interval.
 */
static int stellaris_wdt_settimeout(int new_time)
{
	if ((new_time <= 0) || (new_time > WDT_MAX_TIME))
		return -EINVAL;

	/* Set new watchdog time. It will be used when
	   stellaris_wdt_start() is called. */
	wdt_time = new_time;
	return 0;
}

/* ......................................................................... */

/*
 * Watchdog device is opened, and watchdog starts running.
 */
static int stellaris_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &stellaris_wdt_busy))
		return -EBUSY;

	stellaris_wdt_start();
	return nonseekable_open(inode, file);
}

/*
 * Close the watchdog device.
 * If CONFIG_WATCHDOG_NOWAYOUT is NOT defined then the watchdog is also
 *  disabled.
 */
static int stellaris_wdt_close(struct inode *inode, struct file *file)
{
	clear_bit(0, &stellaris_wdt_busy);
	return 0;
}

static struct watchdog_info stellaris_wdt_info = {
	.identity	= "stellaris watchdog",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
};

/*
 * Handle commands from user-space.
 */
static long stellaris_wdt_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_value;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &stellaris_wdt_info,
					sizeof(stellaris_wdt_info)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_SETOPTIONS:
		if (get_user(new_value, p))
			return -EFAULT;
		if (new_value & WDIOS_DISABLECARD)
			stellaris_wdt_stop();
		if (new_value & WDIOS_ENABLECARD)
			stellaris_wdt_start();
		return 0;
	case WDIOC_KEEPALIVE:
		stellaris_wdt_reload();	/* pat the watchdog */
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;
		if (stellaris_wdt_settimeout(new_value))
			return -EINVAL;
		/* Enable new time value */
		if (stellaris_wdt_busy)
			stellaris_wdt_start();
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
static ssize_t stellaris_wdt_write(struct file *file, const char *data,
						size_t len, loff_t *ppos)
{
	stellaris_wdt_reload();		/* pat the watchdog */
	return len;
}

/* ......................................................................... */

static const struct file_operations stellaris_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= stellaris_wdt_ioctl,
	.open		= stellaris_wdt_open,
	.release	= stellaris_wdt_close,
	.write		= stellaris_wdt_write,
};

static struct miscdevice stellaris_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &stellaris_wdt_fops,
};

static int __devinit stellaris_wdt_probe(struct platform_device *pdev)
{
	int res;

	if (stellaris_wdt_miscdev.parent)
		return -EBUSY;
	stellaris_wdt_miscdev.parent = &pdev->dev;

	res = misc_register(&stellaris_wdt_miscdev);
	if (res)
		return res;

	printk(KERN_INFO "Stellaris Watchdog Timer enabled (%d seconds), nowayout\n", wdt_time);
	return 0;
}

static int __devexit stellaris_wdt_remove(struct platform_device *pdev)
{
	int res;

	res = misc_deregister(&stellaris_wdt_miscdev);
	if (!res)
		stellaris_wdt_miscdev.parent = NULL;

	return res;
}

static void stellaris_wdt_shutdown(struct platform_device *pdev)
{
	stellaris_wdt_stop();
}

#ifdef CONFIG_PM

static int stelllaris_wdt_suspend(struct platform_device *pdev, pm_message_t message)
{
	stellaris_wdt_stop();
	return 0;
}

static int stellaris_wdt_resume(struct platform_device *pdev)
{
	if (stellaris_wdt_busy)
		stellaris_wdt_start();
	return 0;
}

#else
#define stellaris_wdt_suspend NULL
#define stellaris_wdt_resume	NULL
#endif

static struct platform_driver stellaris_wdt_driver = {
	.probe		= stellaris_wdt_probe,
	.remove		= __devexit_p(stellaris_wdt_remove),
	.shutdown	= stellaris_wdt_shutdown,
	.suspend	= stellaris_wdt_suspend,
	.resume		= stellaris_wdt_resume,
	.driver		= {
		.name	= "stellaris-wdt",
		.owner	= THIS_MODULE,
	},
};

static int __init stellaris_wdt_init(void)
{
	spin_lock_init(&stellaris_lock);
	/* Check that the heartbeat value is within range;
	   if not reset to the default */
	if (stellaris_wdt_settimeout(wdt_time)) {
		stellaris_wdt_settimeout(WDT_DEFAULT_TIME);
		pr_info("stellaris-wdt: wdt_time value must be 1 <= wdt_time <= %i"
					", using %d\n", wdt_time, WDT_MAX_TIME);
	}
	return platform_driver_register(&stellaris_wdt_driver);
}

static void __exit stellaris_wdt_exit(void)
{
	platform_driver_unregister(&stellaris_wdt_driver);
}

module_init(stellaris_wdt_init);
module_exit(stellaris_wdt_exit);

MODULE_AUTHOR("Max Nekludov");
MODULE_DESCRIPTION("Watchdog driver for Stellaris");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:stellaris_wdt");
