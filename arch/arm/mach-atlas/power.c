//#define DEBUG
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
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/pins.h>

#define ATLAS_CAPS_VOLTAGE_CHANNEL   14
#define ATLAS_VUNREG_VOLTAGE_CHANNEL 15

#define CAPS_MAX_VOLTAGE_mV 2500
#define CAPS_MIN_VOLTAGE_mV 2400
#define CAPS_MAX_TEMPERATURE_mC 50000
#define VUNREG_MIN_LEVEL_mV 9000

struct power_sensors {
	int caps_voltage;
	int vunreg_voltage;
	int mcu_temperature;
};

struct power_device {
	struct device *dev;
	struct delayed_work work;
	struct power_sensors sensors;
	int charging;
};

static struct power_device device;

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
	
	schedule_delayed_work(&device.work, msecs_to_jiffies(5000));
}

#if 0
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
#endif

static int power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	device.dev = dev;
	INIT_DELAYED_WORK(&device.work, power_check_work);

	platform_set_drvdata(pdev, &device);

	dev_dbg(dev, "%s probed\n", __func__);
	
	schedule_delayed_work(&device.work, msecs_to_jiffies(5000));

	return 0;
}

static int __devexit power_remove(struct platform_device *pdev)
{
	cancel_delayed_work_sync(&device.work);
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