#define DEBUG
#define VERBOSE_DEBUG 1

#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#include <mach/hardware.h>
#include <mach/pins.h>

#include <net/telit_he910.h>

#define POWER_PROC_ENTRY "driver/telit_pwr"
#define MAX_PROC_SIZE 100

struct telit_modem {
	struct spi_device *spidev;
	struct proc_dir_entry *power_proc_entry;
	struct workqueue_struct *work_queue;
	struct work_struct work_power_on;
	struct work_struct work_power_off;
	int pwr_mon_gpio;
	int pwr_on_gpio;
	int if_en_gpio;
};

static inline int get_power_status(struct telit_modem *priv)
{
	return gpioread(priv->pwr_mon_gpio, 0);
}

static inline void interface_control(struct telit_modem *priv, int ctrl)
{
	gpiowrite(priv->if_en_gpio, ctrl);
}

#ifdef CONFIG_PROC_FS
static int read_proc(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
	struct telit_modem *priv = data;
	int len = 0;
	int power = get_power_status(priv);
	len = snprintf(buf, count, "%s", power ? "on" : "off");
	return len;
}

static int write_proc(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	struct telit_modem *priv = data;
	char proc_data[MAX_PROC_SIZE];
	
	if(count > MAX_PROC_SIZE)
		count = MAX_PROC_SIZE;
	
	if(copy_from_user(proc_data, buf, count))
		return -EFAULT;
	
	if( strncmp(proc_data, "on", 2) == 0 )
		return queue_work(priv->work_queue, &priv->work_power_on);
	else if( strncmp(proc_data, "off", 2) == 0 )
		return queue_work(priv->work_queue, &priv->work_power_off);

	return count;
}

static int __devinit init_procfs(struct telit_modem *priv)
{
	priv->power_proc_entry = create_proc_entry(POWER_PROC_ENTRY, 0666, NULL);
	if (!priv->power_proc_entry) {
		printk(KERN_ERR "Error creating proc entry");
		return -ENOMEM;
	}
	
	priv->power_proc_entry->read_proc = read_proc;
	priv->power_proc_entry->write_proc = write_proc;
	priv->power_proc_entry->data = priv;
	
	dev_dbg(&priv->spidev->dev, "Proc entry created\n");
	
	return 0;
}
#endif

static void telit_power_on(struct work_struct *work)
{
	struct telit_modem *priv = container_of(work, struct telit_modem, work_power_on);
	
	dev_dbg(&priv->spidev->dev, "Switching modem on...\n");
	
	gpiowrite(priv->pwr_on_gpio, 0);
	dev_dbg(&priv->spidev->dev, "Hold PWR_ON\n");
	msleep(5000);
	gpiowrite(priv->pwr_on_gpio, 1);
	dev_dbg(&priv->spidev->dev, "Release PWR_ON\n");
	
	msleep(200);
	interface_control(priv, 1);
	dev_dbg(&priv->spidev->dev, "Power: %s\n", get_power_status(priv) ? "ON" : "OFF");
}

static void telit_power_off(struct work_struct *work)
{
	struct telit_modem *priv = container_of(work, struct telit_modem, work_power_off);
	
	dev_dbg(&priv->spidev->dev, "Switching modem off...\n");
	
	interface_control(priv, 0);
	
	gpiowrite(priv->pwr_on_gpio, 0);
	dev_dbg(&priv->spidev->dev, "Hold PWR_ON\n");
	msleep(3000);
	gpiowrite(priv->pwr_on_gpio, 1);
	dev_dbg(&priv->spidev->dev, "Release PWR_ON\n");
	
	msleep(3000);
	interface_control(priv, 1);
	dev_dbg(&priv->spidev->dev, "Power: %s\n", get_power_status(priv) ? "ON" : "OFF");
	interface_control(priv, 0);
}

static int __devinit telit_probe(struct spi_device *spi)
{
	struct telit_modem *priv;
	int ret;
	struct telit_platform_data *plat = spi->dev.platform_data;

	spi->bits_per_word = 8;

	priv = kmalloc(sizeof(struct telit_modem), GFP_KERNEL);
	if (priv == 0)
		return -ENOMEM;

	priv->spidev = spi;
	priv->pwr_mon_gpio = plat->pwr_mon_gpio;
	priv->pwr_on_gpio = plat->pwr_on_gpio;
	priv->if_en_gpio = plat->if_en_gpio;

	dev_set_drvdata(&spi->dev, priv);
	
	priv->work_queue = create_workqueue("telit");
	if (priv->work_queue == 0)
	{
		printk(KERN_ERR "Fail to create workqueue\n");
		kfree(priv);
		return -ENOMEM;
	}
	
	INIT_WORK(&priv->work_power_on, telit_power_on);
	INIT_WORK(&priv->work_power_off, telit_power_off);

#ifdef CONFIG_PROC_FS
	ret = init_procfs(priv);
	if( ret )
	{
		destroy_workqueue(priv->work_queue);
		kfree(priv);
		return ret;
	}
#endif

	dev_dbg(&spi->dev, "probed\n");

	return 0;
}

static int __devexit telit_remove(struct spi_device *spi)
{
	struct telit_modem *priv = dev_get_drvdata(&spi->dev);
	dev_info(&spi->dev, "remove\n");
	kfree(priv);
	return 0;
}

static struct spi_driver telit_driver = {
	.driver = {
		.name = "telit_he910",
		.owner = THIS_MODULE,
	},
	.probe = telit_probe,
	.remove = __devexit_p(telit_remove),
	//.suspend = telit_suspend,
	//.resume = telit_resume,
};

static int __init telit_init(void)
{
	return spi_register_driver(&telit_driver);
}

static void __exit telit_exit(void)
{
	spi_unregister_driver(&telit_driver);
}


module_init(telit_init);
module_exit(telit_exit);

MODULE_DESCRIPTION("Telit Modem HE910 SPI driver");
MODULE_AUTHOR("Max Nekludov <Max.Nekludov@us.elster.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:telit_he910");