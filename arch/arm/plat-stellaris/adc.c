//#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>

#include <linux/proc_fs.h>

#include <mach/hardware.h>
#include <mach/pins.h>

struct adc_device {
	struct device *dev;
	struct mutex lock;
};

static struct adc_device device;
static struct proc_dir_entry *proc_entry;

int adc_convert_ex(uint32_t* values, int *channels, int count)
{
	uint32_t actss;
	uint32_t ssmux = 0;
	uint32_t ssctl = ADC_SSCTL_END;
	int i;

	if (channels == NULL || values == NULL || count < 1 || count > 8)
		return -EINVAL;

	for (i = 0; ; i++, channels++) {
		if (*channels == STLR_ADC_TEMPERATURE) {
			ssctl |= ADC_SSCTL_TS;
			dev_dbg(device.dev, "%s: capture temperature\n", __func__);
		}
		else {
			ssmux |= *channels;
			dev_dbg(device.dev, "%s: capture AIN%i\n", __func__, *channels);
		}
		
		if ( (i + 1) >= count)
			break;
		
		ssmux <<= 4;
		ssctl <<= 4;
	}
	
	if (mutex_lock_interruptible(&device.lock))
		return -EINTR;
	
	adc_clock_ctrl(0, SYS_ENABLE_CLOCK);

	actss = getreg32(STLR_ADC0_ACTSS);
	actss &= ~ADC_ACTSS_ASEN0;
	putreg32(actss, STLR_ADC0_ACTSS);

	putreg32(ssmux, STLR_ADC0_SSMUX0);
	putreg32(ssctl, STLR_ADC0_SSCTL0);

	actss |= ADC_ACTSS_ASEN0;
	putreg32(actss, STLR_ADC0_ACTSS);

	dev_dbg(device.dev, "%s: sampling\n", __func__);
	putreg32(ADC_PSSI_SS0, STLR_ADC0_PSSI);

	while (getreg32(STLR_ADC0_ACTSS) & ADC_ACTSS_BUSY) {}

	values += count - 1;
	for (i = 0; i < count; ++i, --values)
	{
		uint32_t value = getreg32(STLR_ADC0_SSFIFO0) & ADC_SSFIFO_DATA_MASK;
		dev_dbg(device.dev, "%s: read value %u\n", __func__, value);
		*values = value;
	}

	adc_clock_ctrl(0, SYS_DISABLE_CLOCK);
	
	mutex_unlock(&device.lock);

	return count;
}
EXPORT_SYMBOL_GPL(adc_convert_ex);

int adc_convert(uint32_t *value, int channel)
{
	return adc_convert_ex(value, &channel, 1);
}
EXPORT_SYMBOL_GPL(adc_convert);

static int read_proc(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
	int res;
	int channels[] = {14, 15, STLR_ADC_TEMPERATURE};
	uint32_t values[ARRAY_SIZE(channels)];
	
	if (offset)
		return 0;
	
	res = adc_convert_ex(values, channels, ARRAY_SIZE(channels));
	if( res <= 0 )
		return res;
	
	res = sprintf(buf, "caps-charge: %i mV, vunreg: %i mV temp: %i mC\n", 
	              adc_to_mV(values[0]), adc_to_mV(values[1]) * 10, adc_to_mC(values[2]));
	
	*eof = 1;
	
	return res;
}

static int write_proc(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	return count;
}

static int __devinit init_procfs(void)
{
	proc_entry = create_proc_entry("charge", 0666, NULL);
	if (!proc_entry) {
		printk(KERN_ERR "Error creating proc entry");
		return -ENOMEM;
	}
	
	proc_entry->read_proc = read_proc;
	proc_entry->write_proc = write_proc;
	
	return 0;
}

static int stlr_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	
	device.dev = dev;
	mutex_init(&device.lock);

	platform_set_drvdata(pdev, &device);
	
	init_procfs();
	
	dev_dbg(dev, "%s probed\n", __func__);

	return 0;
}

static int __devexit stlr_adc_remove(struct platform_device *pdev)
{
	//struct adc_device *adc = platform_get_drvdata(pdev);
	return 0;
}

#ifdef CONFIG_PM
static int stlr_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	//struct adc_device *adc = platform_get_drvdata(pdev);
	return 0;
}

static int stlr_adc_resume(struct platform_device *pdev)
{
	//struct adc_device *adc = platform_get_drvdata(pdev);
	return 0;
}

#else
#define stlr_adc_suspend NULL
#define stlr_adc_resume NULL
#endif

static struct platform_driver stlr_adc_driver = {
	.driver		= {
		.name	= "stellaris-adc",
		.owner	= THIS_MODULE,
	},
	.probe		= stlr_adc_probe,
	.remove		= __devexit_p(stlr_adc_remove),
	.suspend	= stlr_adc_suspend,
	.resume		= stlr_adc_resume,
};

static int __init adc_init(void)
{
	int ret;

	ret = platform_driver_register(&stlr_adc_driver);
	if (ret)
		printk(KERN_ERR "%s: failed to add adc driver\n", __func__);

	return ret;
}

arch_initcall(adc_init);