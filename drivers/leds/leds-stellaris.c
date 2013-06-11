/* drivers/leds/leds-lm3s.c
 *
 * (C) Max Nekludov <macscomp@gmail.com>
 *
 * Based on:
 * drivers/leds/leds-s3c24xx.c
 *
 * (c) 2006 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * LM3S - LEDs GPIO driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <mach/leds.h>

/* our context */

struct stellaris_gpio_led {
	struct led_classdev		 cdev;
	struct stellaris_led_platdata	*pdata;
};

static inline struct stellaris_gpio_led *pdev_to_gpio(struct platform_device *dev)
{
	return platform_get_drvdata(dev);
}

static inline struct stellaris_gpio_led *to_gpio(struct led_classdev *led_cdev)
{
	return container_of(led_cdev, struct stellaris_gpio_led, cdev);
}

static void led_set(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	struct stellaris_gpio_led *led = to_gpio(led_cdev);
	struct stellaris_led_platdata *pd = led->pdata;

	/* there will be a short delay between setting the output and
	 * going from output to input when using tristate. */

	gpiowrite(pd->gpio, (value ? 1 : 0) ^ (pd->flags & STELLARIS_LEDS_ACTLOW));
}

static int led_remove(struct platform_device *dev)
{
	struct stellaris_gpio_led *led = pdev_to_gpio(dev);

	led_classdev_unregister(&led->cdev);
	kfree(led);

	return 0;
}

static int led_probe(struct platform_device *dev)
{
	struct stellaris_led_platdata *pdata = dev->dev.platform_data;
	struct stellaris_gpio_led *led;
	int ret;

	led = kzalloc(sizeof(struct stellaris_gpio_led), GFP_KERNEL);
	if (led == NULL) {
		dev_err(&dev->dev, "No memory for device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(dev, led);

	led->cdev.brightness_set = led_set;
	led->cdev.default_trigger = pdata->def_trigger;
	led->cdev.name = pdata->name;
	led->cdev.flags |= LED_CORE_SUSPENDRESUME;

	led->pdata = pdata;

	/* no point in having a pull-up if we are always driving */

	configgpio(pdata->gpio);
	gpiowrite(pdata->gpio, pdata->flags & STELLARIS_LEDS_ACTLOW ? 1 : 0);

	/* register our new led device */

	ret = led_classdev_register(&dev->dev, &led->cdev);
	if (ret < 0) {
		dev_err(&dev->dev, "led_classdev_register failed\n");
		kfree(led);
		return ret;
	}

	return 0;
}

static struct platform_driver led_driver = {
	.probe		= led_probe,
	.remove		= led_remove,
	.driver		= {
		.name		= "stellaris-led",
		.owner		= THIS_MODULE,
	},
};

static int __init led_init(void)
{
	return platform_driver_register(&led_driver);
}

static void __exit led_exit(void)
{
	platform_driver_unregister(&led_driver);
}

module_init(led_init);
module_exit(led_exit);

MODULE_AUTHOR("Max Nekludov <macscomp@gmail.com>");
MODULE_DESCRIPTION("Stellaris LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:stellaris-led");
