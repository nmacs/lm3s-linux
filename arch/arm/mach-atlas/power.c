#define DEBUG
//#define VERBOSE_DEBUG 1

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
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/pins.h>

#define ATLAS_CAPS_VOLTAGE_CHANNEL   14
#define ATLAS_VUNREG_VOLTAGE_CHANNEL 15

#define CAPS_MAX_VOLTAGE_mV 1900
#define CAPS_MIN_VOLTAGE_mV 1800
#define CAPS_MAX_TEMPERATURE_mC 50000
#define VUNREG_MIN_LEVEL_mV 9000

#define MAX_PROC_SIZE     128
#define OUTAGE_PROC_ENTRY "driver/outage"
#define OUTAGE_MAX_COUNT  10

struct power_sensors {
	int caps_voltage;
	int vunreg_voltage;
	int mcu_temperature;
};

struct power_device {
	struct device *dev;
	struct delayed_work power_check_work;
	struct power_sensors sensors;
	int charging;
	int power_fail_gpio;
	int power_fail_irq;
	struct timer_list timer;
	struct proc_dir_entry *outage_proc_entry;
	uint32_t outage_counter;
	spinlock_t outage_lock;
	unsigned pid_send_outage_to;
	int wait_restoration;
};

static struct power_device device;

static int is_in_outage(void)
{
	return gpioread(device.power_fail_gpio, 0) != 0;
}

#ifdef CONFIG_PROC_FS
static int read_proc_outage(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
	int len = 0;
	uint32_t outage_counter;
	unsigned long flags;

	if (offset)
		return 0;

	spin_lock_irqsave(&device.outage_lock, flags);
	outage_counter = device.outage_counter;
	spin_unlock_irqrestore(&device.outage_lock, flags);
	len = snprintf(buf, PAGE_SIZE, "%i, %u\n", is_in_outage() ? 1 : 0, outage_counter);

	*eof = 1;
	return len;
}

static int write_proc_outage(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	char proc_data[MAX_PROC_SIZE+1];
	unsigned pid = 0;

	if (count > MAX_PROC_SIZE)
		count = MAX_PROC_SIZE;

	if (copy_from_user(proc_data, buf, count))
		return -EFAULT;

	if (count < MAX_PROC_SIZE)
		proc_data[count] = 0;
	else
		proc_data[MAX_PROC_SIZE] = 0;

	if (sscanf(proc_data, "%u", &pid) == 1) {
		unsigned long flags;
		dev_dbg(device.dev, "If outage send SIGPWR to pid:%u\n", pid);
		spin_lock_irqsave(&device.outage_lock, flags);
		device.pid_send_outage_to = pid;
		spin_unlock_irqrestore(&device.outage_lock, flags);
	}

	return count;
}

static int __devinit init_procfs(void)
{
	device.outage_proc_entry = create_proc_entry(OUTAGE_PROC_ENTRY, 0666, NULL);
	if (!device.outage_proc_entry) {
		dev_err(device.dev, "Error creating proc entry: " OUTAGE_PROC_ENTRY);
		return -ENOMEM;
	}

	device.outage_proc_entry->read_proc = read_proc_outage;
	device.outage_proc_entry->write_proc = write_proc_outage;
	device.outage_proc_entry->data = &device;
	
	dev_dbg(device.dev, "%s: done\n", __func__);
	
	return 0;
}
#endif

static int charge_profile(struct power_sensors *sensors, int charging)
{
	if( sensors->mcu_temperature >= CAPS_MAX_TEMPERATURE_mC ||
	    sensors->vunreg_voltage < VUNREG_MIN_LEVEL_mV )
		return 0;

	if (sensors->caps_voltage < CAPS_MIN_VOLTAGE_mV)
		return 1;
	else if (sensors->caps_voltage > CAPS_MAX_VOLTAGE_mV)
		return 0;
	else
		return charging;
}

static int refresh_sensors(struct power_sensors *sensors)
{
	int res;
	int channels[] = {ATLAS_CAPS_VOLTAGE_CHANNEL, ATLAS_VUNREG_VOLTAGE_CHANNEL, STLR_ADC_TEMPERATURE};
	uint32_t values[ARRAY_SIZE(channels)];

	res = adc_convert_ex(values, channels, ARRAY_SIZE(channels));
	if( res != ARRAY_SIZE(channels) )
		return -EIO;
	
	sensors->caps_voltage = adc_to_mV(values[0]);
	sensors->vunreg_voltage = adc_to_mV(values[1]) * 10;
	sensors->mcu_temperature = adc_to_mC(values[2]);
	
	dev_vdbg(device.dev, "%s: caps_voltage = %i mV, vunreg_voltage = %i mV mcu_temperature = %i mC\n", __func__,
	         sensors->caps_voltage, sensors->vunreg_voltage, sensors->mcu_temperature);
	
	return 0;
}

static void power_check_work(struct work_struct *work)
{
	if (refresh_sensors(&device.sensors))
		dev_err(device.dev, "Fail to refresh power sensors.\n");
	else
	{
		int should_charge = charge_profile(&device.sensors, device.charging);
		if (should_charge && !device.charging) {
			//gpiowrite(GPIO_CAP_CHRG, 1);
			device.charging = 1;
			dev_dbg(device.dev, "Enable charging\n");
		}
		else if (!should_charge && device.charging) {
			//gpiowrite(GPIO_CAP_CHRG, 0);
			device.charging = 0;
			dev_dbg(device.dev, "Disable charging\n");
		}
	}

	schedule_delayed_work(&device.power_check_work, msecs_to_jiffies(5000));
}

static irqreturn_t power_fail(int irq, void *pw)
{
	int ret;
	int nr_pid = device.pid_send_outage_to;

	gpioirqdisable(device.power_fail_gpio);
	gpioclearint(device.power_fail_gpio);

	device.outage_counter++;

	dev_emerg(device.dev, "Power outage detected\n");

	if (nr_pid) {
		struct pid *pid = find_get_pid(nr_pid);
		if (pid) {
			dev_dbg(device.dev, "Send signal SIGPWR to pid:%i\n", nr_pid);
			if ((ret = kill_pid(pid, SIGPWR, 0)))
				dev_err(device.dev, "Fail to send SIGPWR to pid:%i code:%i\n", nr_pid, ret);
			put_pid(pid);
		}
		else
			dev_err(device.dev, "Fail to send SIGPWR to pid:%i process not found\n", nr_pid);

		device.pid_send_outage_to = 0;
	}

	mod_timer(&device.timer, jiffies + msecs_to_jiffies(500));

	return IRQ_HANDLED;
}

static void check_outage_timer(unsigned long data)
{
	if (is_in_outage()) {
		mod_timer(&device.timer, jiffies + msecs_to_jiffies(1000));
		return;
	}
	dev_notice(device.dev, "Power restoration detected\n");
	gpioirqenable(device.power_fail_gpio);
}

static void atlas_power_off(void)
{
	arch_reset(0, 0);
}

static int power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	memset(&device, 0, sizeof(device));
	device.dev = dev;
	INIT_DELAYED_WORK(&device.power_check_work, power_check_work);
	spin_lock_init(&device.outage_lock);
	init_timer(&device.timer);
	device.timer.function = check_outage_timer;
	device.timer.data = 0;

	platform_set_drvdata(pdev, &device);

	device.power_fail_gpio = GPIO_POWER_FAIL;
	device.power_fail_irq  = STLR_GPIOK_IRQ;

	ret = request_irq(device.power_fail_irq, power_fail, IRQF_TRIGGER_HIGH,
			  pdev->name, &device);
	if (ret < 0) {
		dev_err(dev, "Failed to get irq\n");
		return ret;
	}

	pm_power_off = atlas_power_off;
	init_procfs();
	gpioirqenable(device.power_fail_gpio);

	schedule_delayed_work(&device.power_check_work, msecs_to_jiffies(5000));

	dev_dbg(dev, "probed\n");

	return 0;
}

static int __devexit power_remove(struct platform_device *pdev)
{
	cancel_delayed_work_sync(&device.power_check_work);
	disable_irq(device.power_fail_irq);
	free_irq(device.power_fail_irq, &device);
	del_timer_sync(&device.timer);
	
	dev_dbg(device.dev, "removed\n");
	
	return 0;
}

static struct platform_driver power_driver = {
	.driver		= {
		.name	= "atlas-power",
		.owner	= THIS_MODULE,
	},
	.probe		= power_probe,
	.remove		= __devexit_p(power_remove),
};

static int __init power_init(void)
{
	int ret;

	ret = platform_driver_register(&power_driver);
	if (ret)
		printk(KERN_ERR "%s: failed to add power driver\n", __func__);

	return ret;
}

arch_initcall(power_init);