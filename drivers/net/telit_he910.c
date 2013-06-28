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
#define AT_PROC_ENTRY    "driver/telit_at"
#define MAX_PROC_SIZE 100

enum telit_state {
	switched_off,
	switching_off,
	switching_on,
	switched_on
};

struct telit_modem {
	struct spi_device *spidev;
	struct proc_dir_entry *power_proc_entry;
	struct proc_dir_entry *at_proc_entry;
	struct workqueue_struct *work_queue;
	struct work_struct work_power_on;
	struct work_struct work_power_off;
	int pwr_mon_gpio;
	int pwr_on_gpio;
	int if_en_gpio;
	int spi_srdy_gpio;
	struct spi_message	spi_msg;
	struct spi_transfer	spi_xfer;
	enum telit_state state;
};

static inline int get_power_status(struct telit_modem *priv)
{
	return gpioread(priv->pwr_mon_gpio, 0);
}

static inline void interface_control(struct telit_modem *priv, int ctrl)
{
	gpiowrite(priv->if_en_gpio, ctrl);
}

static inline int is_modem_ready(struct telit_modem *priv)
{
	return !gpioread(priv->spi_srdy_gpio, 0);
}

static int send_at(struct telit_modem *priv, char *command, int size)
{
	struct spi_transfer *xfer = &priv->spi_xfer;
	struct spi_message *msg = &priv->spi_msg;

	if (priv->state != switched_on)
		return -EFAULT;

	if (!is_modem_ready(priv))
	{
		printk(KERN_ERR "Modem is busy\n");
		return -EBUSY;
	}

	xfer->tx_buf = command;
	xfer->rx_buf = NULL;
	xfer->len = size;

	return spi_sync(priv->spidev, msg);
}

#ifdef CONFIG_PROC_FS
static int read_proc_pwr(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
	struct telit_modem *priv = data;
	int len;
	char *state = 0;
	
	if (offset)
		return 0;
	
	switch(priv->state)
	{
		case switched_off:
			state = "off";
			break;
		case switching_off:
			state = "switching off";
			break;
		case switching_on:
			state = "switching on";
			break;
		case switched_on:
			state = "on";
			break;
	};
	len = sprintf(buf, "%s", state);
	*eof = 1;
	
	return len;
}

static int write_proc_pwr(struct file *file, const char __user *buf, unsigned long count, void *data)
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

static int read_proc_at(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
//	struct telit_modem *priv = data;

	
	return 0;
}

static int write_proc_at(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	struct telit_modem *priv = data;
	char cmd[64];
	int ret;
	size_t cmd_size = min((size_t)count, sizeof(cmd));

	if (copy_from_user(cmd, buf, min((size_t)count, sizeof(cmd))))
		return -EFAULT;
	
	cmd_size = strnlen(cmd, cmd_size);

	if (cmd[cmd_size-1] == '\n')
		cmd[cmd_size-1] = '\r';

	ret = send_at(priv, cmd, cmd_size);
	if (ret)
		return ret;

	return count;
}

static int __devinit init_procfs(struct telit_modem *priv)
{
	priv->power_proc_entry = create_proc_entry(POWER_PROC_ENTRY, 0666, NULL);
	if (!priv->power_proc_entry) {
		printk(KERN_ERR "Error creating proc entry: " POWER_PROC_ENTRY);
		return -ENOMEM;
	}

	priv->power_proc_entry->read_proc = read_proc_pwr;
	priv->power_proc_entry->write_proc = write_proc_pwr;
	priv->power_proc_entry->data = priv;

	priv->at_proc_entry = create_proc_entry(AT_PROC_ENTRY, 0666, NULL);
	if (!priv->at_proc_entry) {
		printk(KERN_ERR "Error creating proc entry: " AT_PROC_ENTRY);
		return -ENOMEM;
	}
	
	priv->at_proc_entry->read_proc = read_proc_at;
	priv->at_proc_entry->write_proc = write_proc_at;
	priv->at_proc_entry->data = priv;
	
	dev_dbg(&priv->spidev->dev, "init_procfs: done\n");
	
	return 0;
}
#endif

static void telit_power_on(struct work_struct *work)
{
	struct telit_modem *priv = container_of(work, struct telit_modem, work_power_on);
	int power_status;

	dev_dbg(&priv->spidev->dev, "Switching modem on...\n");
	priv->state = switching_on;

	gpiowrite(priv->pwr_on_gpio, 0);
	dev_dbg(&priv->spidev->dev, "Hold PWR_ON\n");
	msleep(5000);
	gpiowrite(priv->pwr_on_gpio, 1);
	dev_dbg(&priv->spidev->dev, "Release PWR_ON\n");

	msleep(200);
	//interface_control(priv, 1);
	//power_status = get_power_status(priv);
	power_status = 1;
	dev_dbg(&priv->spidev->dev, "Power: %s\n", power_status ? "ON" : "OFF");

	priv->state = power_status ? switched_on : switched_off;
}

static void telit_power_off(struct work_struct *work)
{
	struct telit_modem *priv = container_of(work, struct telit_modem, work_power_off);
	int i;
	int power_status;

	dev_dbg(&priv->spidev->dev, "Switching modem off...\n");
	priv->state = switching_off;

	//interface_control(priv, 0);

	gpiowrite(priv->pwr_on_gpio, 0);
	dev_dbg(&priv->spidev->dev, "Hold PWR_ON\n");
	msleep(3000);
	gpiowrite(priv->pwr_on_gpio, 1);
	dev_dbg(&priv->spidev->dev, "Release PWR_ON\n");

	for (i = 0; i < 15; i++ )
	{
		msleep(1000);
		//interface_control(priv, 1);
		//power_status = get_power_status(priv);
		//interface_control(priv, 0);
		power_status = 0;
		if (!power_status)
			break;
	}

	priv->state = power_status ? switched_on : switched_off;
	dev_dbg(&priv->spidev->dev, "Power: %s\n", power_status ? "ON" : "OFF");
}

static int __devinit telit_probe(struct spi_device *spi)
{
	struct telit_modem *priv;
	int ret;
	struct telit_platform_data *plat = spi->dev.platform_data;

	spi->bits_per_word = 8;

	priv = kzalloc(sizeof(struct telit_modem), GFP_KERNEL);
	if (priv == 0)
		return -ENOMEM;

	priv->spidev = spi;
	priv->pwr_mon_gpio = plat->pwr_mon_gpio;
	priv->pwr_on_gpio = plat->pwr_on_gpio;
	priv->if_en_gpio = plat->if_en_gpio;
	priv->spi_srdy_gpio = plat->spi_srdy_gpio;
	priv->state = switched_off;

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

	spi_message_init(&priv->spi_msg);
	spi_message_add_tail(&priv->spi_xfer, &priv->spi_msg);

#ifdef CONFIG_TELIT_HE910_POWER_ON
	if (!queue_work(priv->work_queue, &priv->work_power_on))
		dev_err(&spi->dev, "fail to power on\n");
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