/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/types.h>
#include <linux/msm_dsps.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <asm/arch_timer.h>
#include <linux/uaccess.h>

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <soc/qcom/subsystem_restart.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

#define IMAGE_LOAD_CMD 1
#define IMAGE_UNLOAD_CMD 0
#define CLASS_NAME	"ssc"
#define DRV_NAME	"sensors"
#define DRV_VERSION	"2.00"
#ifdef CONFIG_COMPAT
#define DSPS_IOCTL_READ_SLOW_TIMER32 _IOR(DSPS_IOCTL_MAGIC, 3, compat_uint_t)
#endif

//#define ALS_VDD_SUPPORT //device tree control L19
#ifdef ALS_VDD_SUPPORT
#define ALS_VDD_MIN_uV 3050000
#define ALS_VDD_MAX_uV 3050000
#define ALS_VDD_MAX_ua 50000
#define ALS_VDD_ENABLE 1
#define ALS_VDD_DISABLE 0
#define ALS_VDD_SETTING 2
struct regulator *ALS_VDD;
#endif

#define ALS_GPIO_SUPPORT
#ifdef ALS_GPIO_SUPPORT
#define ALS_GPIO_ENABLE 1
#define ALS_GPIO_DISABLE 0
//#define ALS_INT_TEST
static int ALS_GPIO;
#endif

#ifdef CONFIG_ASUS_SENSOR_ENG_CMD
#define ALS_DEV_NAME	"proximity"
#define ATTRIBUTES_PERMISSION  0664
#endif

struct sns_ssc_control_s {
	struct class *dev_class;
	dev_t dev_num;
	struct device *dev;
	struct cdev *cdev;
#ifdef ALS_DEV_NAME
	dev_t als_num;
	struct device *als_dev;
	struct cdev *als_cdev;
#endif

};
static struct sns_ssc_control_s sns_ctl;

#ifdef ALS_INT_TEST
static struct als_sensor_str {
	int status;
	int enable;
	spinlock_t mALSLock;
	struct wake_lock wake_lock;
	struct input_dev *als_indev;
	struct delayed_work als_sensor_work;
	int irq_trigger;
	int default_value;
	int debounce;
	int sleep;
}* als_sensor_dev;
#endif

static ssize_t slpi_boot_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf, size_t count);

struct slpi_loader_private {
	void *pil_h;
	struct kobject *boot_slpi_obj;
	struct attribute_group *attr_group;
	struct regulator *vdd;
};

static struct kobj_attribute slpi_boot_attribute =
	__ATTR(boot, 0220, NULL, slpi_boot_store);

static struct attribute *attrs[] = {
	&slpi_boot_attribute.attr,
	NULL,
};

static struct platform_device *slpi_private;

static void slpi_loader_do(struct platform_device *pdev)
{

	struct slpi_loader_private *priv = NULL;

	if (!pdev) {
		dev_err(&pdev->dev, "%s: Platform device null\n", __func__);
		goto fail;
	}

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev,
			"%s: Device tree information missing\n", __func__);
		goto fail;
	}

	priv = platform_get_drvdata(pdev);
	if (!priv) {
		dev_err(&pdev->dev,
		" %s: Private data get failed\n", __func__);
		goto fail;
	}

	priv->pil_h = subsystem_get("slpi");
	if (IS_ERR(priv->pil_h)) {
		dev_err(&pdev->dev, "%s: pil get failed,\n",
			__func__);
		goto fail;
	}

	dev_err(&pdev->dev, "%s: SLPI image is loaded\n", __func__);
	return;

fail:
	dev_err(&pdev->dev, "%s: SLPI image loading failed\n", __func__);
}

static void slpi_loader_unload(struct platform_device *pdev)
{
	struct slpi_loader_private *priv = NULL;

	priv = platform_get_drvdata(pdev);

	if (!priv)
		return;

	if (priv->pil_h) {
		dev_dbg(&pdev->dev, "%s: calling subsystem put\n", __func__);
		subsystem_put(priv->pil_h);
		priv->pil_h = NULL;
	}
}

static ssize_t slpi_boot_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf,
	size_t count)
{
	int boot = 0;

	if (sscanf(buf, "%du", &boot) != 1)
		return -EINVAL;

	if (boot == IMAGE_LOAD_CMD) {
		pr_debug("%s: going to call slpi_loader_do\n", __func__);
		slpi_loader_do(slpi_private);
	} else if (boot == IMAGE_UNLOAD_CMD) {
		pr_debug("%s: going to call slpi_unloader\n", __func__);
		slpi_loader_unload(slpi_private);
	}
	return count;
}

static int slpi_loader_init_sysfs(struct platform_device *pdev)
{
	int ret = -EINVAL;
	struct slpi_loader_private *priv = NULL;

	slpi_private = NULL;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	priv->pil_h = NULL;
	priv->boot_slpi_obj = NULL;
	priv->attr_group = devm_kzalloc(&pdev->dev,
				sizeof(*(priv->attr_group)),
				GFP_KERNEL);
	if (!priv->attr_group) {
		dev_err(&pdev->dev, "%s: malloc attr_group failed\n",
						__func__);
		ret = -ENOMEM;
		goto error_return;
	}

	priv->attr_group->attrs = attrs;

	priv->boot_slpi_obj = kobject_create_and_add("boot_slpi", kernel_kobj);
	if (!priv->boot_slpi_obj) {
		dev_err(&pdev->dev, "%s: sysfs create and add failed\n",
						__func__);
		ret = -ENOMEM;
		goto error_return;
	}

	ret = sysfs_create_group(priv->boot_slpi_obj, priv->attr_group);
	if (ret) {
		dev_err(&pdev->dev, "%s: sysfs create group failed %d\n",
							__func__, ret);
		goto error_return;
	}

	slpi_private = pdev;

	return 0;

error_return:

	if (priv->boot_slpi_obj) {
		kobject_del(priv->boot_slpi_obj);
		priv->boot_slpi_obj = NULL;
	}

	return ret;
}

static int slpi_loader_remove(struct platform_device *pdev)
{
	struct slpi_loader_private *priv = NULL;

	priv = platform_get_drvdata(pdev);

	if (!priv)
		return 0;

	if (priv->pil_h) {
		subsystem_put(priv->pil_h);
		priv->pil_h = NULL;
	}

	if (priv->boot_slpi_obj) {
		sysfs_remove_group(priv->boot_slpi_obj, priv->attr_group);
		kobject_del(priv->boot_slpi_obj);
		priv->boot_slpi_obj = NULL;
	}

	return 0;
}

/*
 * Read QTimer clock ticks and scale down to 32KHz clock as used
 * in DSPS
 */
static u32 sns_read_qtimer(void)
{
	u64 val;
	val = arch_counter_get_cntpct();
	/*
	 * To convert ticks from 19.2 Mhz clock to 32768 Hz clock:
	 * x = (value * 32768) / 19200000
	 * This is same as first left shift the value by 4 bits, i.e. mutiply
	 * by 16, and then divide by 9375. The latter is preferable since
	 * QTimer tick (value) is 56-bit, so (value * 32768) could overflow,
	 * while (value * 16) will never do
	 */
	val <<= 4;
	do_div(val, 9375);

	return (u32)val;
}

static int sensors_ssc_open(struct inode *ip, struct file *fp)
{
	return 0;
}

static int sensors_ssc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long sensors_ssc_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	u32 val = 0;

	switch (cmd) {
	case DSPS_IOCTL_READ_SLOW_TIMER:
#ifdef CONFIG_COMPAT
	case DSPS_IOCTL_READ_SLOW_TIMER32:
#endif
		val = sns_read_qtimer();
		ret = put_user(val, (u32 __user *) arg);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

const struct file_operations sensors_ssc_fops = {
	.owner = THIS_MODULE,
	.open = sensors_ssc_open,
	.release = sensors_ssc_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl = sensors_ssc_ioctl,
#endif
	.unlocked_ioctl = sensors_ssc_ioctl
};

#ifdef ALS_VDD_SUPPORT
static void als_regulator_control(int enable)
{
	int ret;
	if (IS_ERR(ALS_VDD)) {
		printk("ssc als-vdd cannot get regulator.\n");
	} else if(enable == ALS_VDD_ENABLE) {
		ret = regulator_enable(ALS_VDD);
		if (ret < 0) {
			printk("ssc als-vdd cannot enable.\n");
		}
	}	else if(enable == ALS_VDD_SETTING) {
		ret = regulator_set_voltage(ALS_VDD, ALS_VDD_MIN_uV, ALS_VDD_MAX_uV);
		if (ret) {
			printk("ssc als-vdd cannot set_vtg ret=%d.\n", ret);
		}

		ret =  regulator_set_optimum_mode(ALS_VDD, ALS_VDD_MAX_ua);
		if (ret< 0) {
			printk("ssc als-vdd cannot set_load ret=%d.\n", ret);
		}
	}  else if(enable == ALS_VDD_DISABLE) {
		ret = regulator_disable(ALS_VDD);
		if (ret < 0) {
			printk("ssc als-vdd cannot disable.\n");
		}
	} else {
		printk("ssc als_regulator_control unknown cmd.\n");
	}
}
#endif

#ifdef ALS_GPIO_SUPPORT
#ifdef ALS_INT_TEST
static irqreturn_t als_sensor_interrupt_handler(int irq, void *dev_id)
{
	printk("als_sensor_interrupt_handler.\n");
	return IRQ_HANDLED;
}
#endif

static void als_gpio_control(int enable)
{
	int ret,irq;
	ret = 0, irq = 0;
	if(ALS_GPIO<=0 ){
		printk("cannot get_named_gpio (%d).\n", ALS_GPIO);
	} else if(enable==1) {
#ifdef ALS_INT_TEST
		ret = gpio_request(ALS_GPIO, "als_gpio");
		if (ret) {
			printk("Unable to request als_gpio(%d).\n", ALS_GPIO);
		}

		ret = gpio_direction_input(ALS_GPIO);
		if (ret < 0) {
			printk("Unable to set the direction of als_gpio(%d).\n", ALS_GPIO);
		}

		irq=gpio_to_irq(ALS_GPIO);
		if (irq < 0) {
			printk("als_gpio(%d) gpio_to_irq error.\n",ALS_GPIO);
		}

        ret = request_threaded_irq(irq, NULL, als_sensor_interrupt_handler,IRQF_TRIGGER_RISING| IRQF_ONESHOT,"als_int", als_sensor_dev);
#endif
	} else if(enable==0) {
		gpio_free(ALS_GPIO);
	} else {
		printk("ssc als_gpio_control unknown cmd.\n");
	}
}

static void set_pinctrl(struct device *dev)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;

	key_pinctrl = devm_pinctrl_get(dev);
	set_state = pinctrl_lookup_state(key_pinctrl, "als_gpio_high");
	ret = pinctrl_select_state(key_pinctrl, set_state);
	if(ret < 0) {
		printk("pinctrl_select_state = %d.\n",  ret);
	}
}
#endif

#ifdef ALS_DEV_NAME
static int sensors_als_open(struct inode *ip, struct file *fp)
{
	return 0;
}

static int sensors_als_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long sensors_als_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	switch (cmd) {
		case DSPS_IOCTL_ON:
			als_regulator_control(ALS_VDD_ENABLE);
			als_gpio_control(ALS_GPIO_ENABLE);
			break;
		case DSPS_IOCTL_OFF:
			als_regulator_control(ALS_VDD_DISABLE);
			als_gpio_control(ALS_GPIO_DISABLE);
			break;

		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}
const struct file_operations sensors_als_fops = {
	.owner = THIS_MODULE,
	.open = sensors_als_open,
	.release = sensors_als_release,
	.compat_ioctl = sensors_als_ioctl,
	.unlocked_ioctl = sensors_als_ioctl
};

static ssize_t als_show_mode(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", regulator_is_enabled(ALS_VDD));
}

static ssize_t als_ps_enable(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
	int ret;

	ret = sscanf(buf, "%lu", &value);
	if (ret != 1)
		return -EINVAL;

	switch (value) {
		case 1:
			als_regulator_control(ALS_VDD_ENABLE);
			als_gpio_control(ALS_GPIO_ENABLE);
			break;
		case 0:
			als_regulator_control(ALS_VDD_DISABLE);
			als_gpio_control(ALS_GPIO_DISABLE);
			break;
		default:
			return -EINVAL;
	}
	return count;
}

static struct device_attribute psensor_attributes[] = {
	__ATTR(enable, ATTRIBUTES_PERMISSION, als_show_mode, als_ps_enable),
};
#endif

static int sensors_ssc_probe(struct platform_device *pdev)
{
	int ret;
#ifdef ALS_DEV_NAME
	int i=0;
#endif

#ifdef ALS_VDD_SUPPORT
	ALS_VDD = devm_regulator_get(&pdev->dev, "vdd");
	als_regulator_control(ALS_VDD_SETTING);
	als_regulator_control(ALS_VDD_ENABLE);
#endif

#ifdef ALS_GPIO_SUPPORT
	ALS_GPIO = of_get_named_gpio(pdev->dev.of_node, "qcom,als-gpio", 0);
	als_gpio_control(ALS_GPIO_ENABLE);
	set_pinctrl(&pdev->dev);
#endif

	ret = slpi_loader_init_sysfs(pdev);

	if (ret != 0) {
		dev_err(&pdev->dev, "%s: Error in initing sysfs\n", __func__);
		return ret;
	}

	sns_ctl.dev_class = class_create(THIS_MODULE, CLASS_NAME);
	if (sns_ctl.dev_class == NULL) {
		pr_err("%s: class_create fail.\n", __func__);
		goto res_err;
	}

	ret = alloc_chrdev_region(&sns_ctl.dev_num, 0, 1, DRV_NAME);
	if (ret) {
		pr_err("%s: alloc_chrdev_region fail.\n", __func__);
		goto alloc_chrdev_region_err;
	}

	sns_ctl.dev = device_create(sns_ctl.dev_class, NULL,
				     sns_ctl.dev_num,
				     &sns_ctl, DRV_NAME);
	if (IS_ERR(sns_ctl.dev)) {
		pr_err("%s: device_create fail.\n", __func__);
		goto device_create_err;
	}

	sns_ctl.cdev = cdev_alloc();
	if (sns_ctl.cdev == NULL) {
		pr_err("%s: cdev_alloc fail.\n", __func__);
		goto cdev_alloc_err;
	}
	cdev_init(sns_ctl.cdev, &sensors_ssc_fops);
	sns_ctl.cdev->owner = THIS_MODULE;

	ret = cdev_add(sns_ctl.cdev, sns_ctl.dev_num, 1);
	if (ret) {
		pr_err("%s: cdev_add fail.\n", __func__);
		goto cdev_add_err;
	}

#ifdef ALS_DEV_NAME
	ret = alloc_chrdev_region(&sns_ctl.als_num, 0, 1, ALS_DEV_NAME);
	if (ret) {
		pr_err("%s: alloc als_num region fail.\n", __func__);
		goto als_alloc_chrdev_region_err;
	}

	sns_ctl.als_dev = device_create(sns_ctl.dev_class, NULL,
				     sns_ctl.als_num,
				     &sns_ctl, ALS_DEV_NAME);
	if (IS_ERR(sns_ctl.als_dev)) {
		pr_err("%s: als_dev create fail.\n", __func__);
		goto als_device_create_err;
	}

	sns_ctl.als_cdev = cdev_alloc();
	if (sns_ctl.als_cdev == NULL) {
		pr_err("%s: als_cdev alloc fail.\n", __func__);
		goto als_cdev_alloc_err;
	}
	cdev_init(sns_ctl.als_cdev, &sensors_als_fops);
	sns_ctl.als_cdev->owner = THIS_MODULE;
	ret = cdev_add(sns_ctl.als_cdev, sns_ctl.als_num, 1);
	if (ret) {
		pr_err("%s: als_cdev add fail.\n", __func__);
		goto als_cdev_add_err;
	}

	for (i = 0; i < ARRAY_SIZE(psensor_attributes); i++){
		if (device_create_file(sns_ctl.als_dev, psensor_attributes + i)) {
			pr_err("als device_create_file fail \n");
			goto als_create_file_error;
		}
	}
#endif
	return 0;

#ifdef ALS_DEV_NAME
als_create_file_error:
	for ( ; i >= 0; i--)
	device_remove_file(sns_ctl.als_dev, psensor_attributes + i);
als_cdev_add_err:
	kfree(sns_ctl.als_cdev);
als_cdev_alloc_err:
	device_destroy(sns_ctl.dev_class, sns_ctl.als_num);
als_device_create_err:
	unregister_chrdev_region(sns_ctl.als_num, 1);
als_alloc_chrdev_region_err:
	class_destroy(sns_ctl.dev_class);
#endif

cdev_add_err:
	kfree(sns_ctl.cdev);
cdev_alloc_err:
	device_destroy(sns_ctl.dev_class, sns_ctl.dev_num);
device_create_err:
	unregister_chrdev_region(sns_ctl.dev_num, 1);
alloc_chrdev_region_err:
	class_destroy(sns_ctl.dev_class);
res_err:
	return -ENODEV;
}

static int sensors_ssc_remove(struct platform_device *pdev)
{
	slpi_loader_remove(pdev);

#ifdef ALS_VDD_SUPPORT
	als_regulator_control(ALS_VDD_DISABLE);
#endif

#ifdef ALS_GPIO_SUPPORT
	als_gpio_control(ALS_GPIO_DISABLE);
#endif

#ifdef ALS_DEV_NAME
	cdev_del(sns_ctl.als_cdev);
	kfree(sns_ctl.als_cdev);
	sns_ctl.als_cdev = NULL;
	device_destroy(sns_ctl.dev_class, sns_ctl.als_num);
	unregister_chrdev_region(sns_ctl.als_num, 1);
#endif

	cdev_del(sns_ctl.cdev);
	kfree(sns_ctl.cdev);
	sns_ctl.cdev = NULL;
	device_destroy(sns_ctl.dev_class, sns_ctl.dev_num);
	unregister_chrdev_region(sns_ctl.dev_num, 1);
	class_destroy(sns_ctl.dev_class);

	return 0;
}

static const struct of_device_id msm_ssc_sensors_dt_match[] = {
	{.compatible = "qcom,msm-ssc-sensors"},
	{},
};
MODULE_DEVICE_TABLE(of, msm_ssc_sensors_dt_match);

static struct platform_driver sensors_ssc_driver = {
	.driver = {
		.name = "sensors-ssc",
		.owner = THIS_MODULE,
		.of_match_table = msm_ssc_sensors_dt_match,
	},
	.probe = sensors_ssc_probe,
	.remove = sensors_ssc_remove,
};

static int __init sensors_ssc_init(void)
{
	int rc;

	pr_debug("%s driver version %s.\n", DRV_NAME, DRV_VERSION);
	rc = platform_driver_register(&sensors_ssc_driver);
	if (rc) {
		pr_err("%s: Failed to register sensors ssc driver\n",
			__func__);
		return rc;
	}

	return 0;
}

static void __exit sensors_ssc_exit(void)
{
	platform_driver_unregister(&sensors_ssc_driver);
}

module_init(sensors_ssc_init);
module_exit(sensors_ssc_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Sensors SSC driver");
