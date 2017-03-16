#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>


/*************************/
/* Debug Switch System */
/************************/
//#define HALL_DEBUG
#ifdef HALL_DEBUG
#define dbg(s,args...)	{printk("[HALL]:[%s],",__func__); printk(s,## args);}
#else
#define dbg(s,args...) {}
#endif

/*****************************/
/* Hall Sensor Configuration */
/****************************/
#define DRIVER_NAME 		"hall_sensor"
#define IRQ_Name			"hall_gpio"
#define INT_NAME			"hall_int"
#define KERNEL_OBJECT	    "hall_kobject"

/**************************/
/* Driver Data Structure */
/*************************/
static struct hall_sensor_str {
	int status;
	int enable;
	spinlock_t mHallSensorLock;
	struct wake_lock wake_lock;
	struct input_dev *hall_indev;
	struct delayed_work hall_sensor_work;
	int irq_trigger;
	int default_value;
	int debounce;
	int sleep;
}* hall_sensor_dev;

/*******************************/
/* Hall Sensor Global Variables */
/******************************/
static int 							HALL_SENSOR_GPIO;
static int 							HALL_SENSOR_IRQ;
static struct workqueue_struct 	*hall_sensor_wq;
static struct platform_device 	*hall_pdev;

/*===============================
*|| Interrupt Service Routine part ||
*===============================
*/

static void hall_sensor_report_function(struct work_struct *dat)
{
	unsigned long flags;
	msleep(50);
	if(!hall_sensor_dev->enable){
		dbg(" hall sensor is disable!\n");
		wake_unlock(&hall_sensor_dev->wake_lock);
		return;
	}

	spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
	if (gpio_get_value(HALL_SENSOR_GPIO) > 0)
	hall_sensor_dev->status = 1;
	else
	hall_sensor_dev->status = 0;
	spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);

	input_report_switch(hall_sensor_dev->hall_indev, SW_LID, !hall_sensor_dev->status);
	input_sync(hall_sensor_dev->hall_indev);
	wake_unlock(&hall_sensor_dev->wake_lock);
	dbg(" report value = %d\n", !hall_sensor_dev->status);

}

static irqreturn_t hall_sensor_interrupt_handler(int irq, void *dev_id)
{
	dbg(" hall_sensor_interrupt = %d\n",HALL_SENSOR_IRQ);
	queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
	wake_lock(&hall_sensor_dev->wake_lock);
	return IRQ_HANDLED;
}

static void debounce_hall_sensor_report_function(struct work_struct *dat)
{
	unsigned long flags;
	int counter, counter_trigger = 0, initial_status;
	int de_bounce=0;
	int sleep_time=0;

	dbg(" default_value:%d \n",hall_sensor_dev->default_value);
	if(hall_sensor_dev->default_value==0) {
		input_report_switch(hall_sensor_dev->hall_indev, SW_LID, 0);
		input_sync(hall_sensor_dev->hall_indev);
		return;
	} else if(hall_sensor_dev->default_value==1) {
		input_report_switch(hall_sensor_dev->hall_indev, SW_LID, 1);
		input_sync(hall_sensor_dev->hall_indev);
		return;
	}

	dbg(" debounce:%d \n",hall_sensor_dev->debounce);
	dbg(" sleep_time:%d \n",hall_sensor_dev->sleep);
	if(hall_sensor_dev->debounce<50) {
		dbg(" debounce<50  \n");
		de_bounce=50;
	} else {
		de_bounce=hall_sensor_dev->debounce;
	}

	if(hall_sensor_dev->sleep<10  || hall_sensor_dev->sleep>hall_sensor_dev->debounce){
		dbg(" sleep_time<10 or sleep>debounce \n");
		sleep_time=10;
	} else {
		sleep_time=hall_sensor_dev->sleep;
	}

	if(!hall_sensor_dev->enable){
		dbg(" hall sensor is disable!\n");
		wake_unlock(&hall_sensor_dev->wake_lock);
		return;
	}
	initial_status =hall_sensor_dev->status;
	dbg(" initial_status:%d \n",hall_sensor_dev->status);
	dbg(" de_bounce:%d \n",de_bounce);
	for (counter = 0;counter < ((de_bounce/sleep_time));counter++) {
		msleep(sleep_time);
		dbg(" counter:%d \n",counter);
		spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
		if (gpio_get_value(HALL_SENSOR_GPIO) == hall_sensor_dev->irq_trigger) {
			hall_sensor_dev->status = 0;
			counter_trigger++;
			dbg(" gpio_get_value 0 \n");
		}else{
			hall_sensor_dev->status = 1;
			dbg(" gpio_get_value 1 \n");
		}
		spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
	}
	dbg(" counter_trigger:%d \n",counter_trigger);
	if( (counter_trigger > 0) && (counter_trigger < (de_bounce/sleep_time))){
		dbg(" SW_LID do not report to framework.\n");
		hall_sensor_dev->status = initial_status;
		wake_unlock(&hall_sensor_dev->wake_lock);
		return;
	}
	input_report_switch(hall_sensor_dev->hall_indev, SW_LID, !hall_sensor_dev->status);
	input_sync(hall_sensor_dev->hall_indev);

	wake_unlock(&hall_sensor_dev->wake_lock);
	printk("hall report value = %d\n", !hall_sensor_dev->status);

}

/*===========================
*|| sysfs DEVICE_Hall sensor part ||
*===========================
*
*/

static ssize_t show_action_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(!hall_sensor_dev)
	return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->status);
}

static ssize_t store_action_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	unsigned long flags;

	//if(!hall_sensor_dev)
	//         return sprintf(buf, "Hall sensor does not exist!\n");
	sscanf(buf, "%du", &request);

	spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
	if (!request)
	hall_sensor_dev->status = 0;
	else
	hall_sensor_dev->status = 1;
	spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);

	printk("[Hall sensor] status rewite value = %d\n",!hall_sensor_dev->status);
	return count;
}

static ssize_t show_hall_sensor_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(!hall_sensor_dev)
	return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->enable);
}


static ssize_t store_hall_sensor_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;

	if(!hall_sensor_dev) {
		printk("Hall sensor does not exist!\n");
		return 0;
	}

	sscanf(buf, "%du", &request);

	if(request==hall_sensor_dev->enable){
		return count;
	}
	else {
		unsigned long flags;
		if(0 == request) {
			// Turn off
			printk("[Hall sensor] Turn off.\n");

			spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
			hall_sensor_dev->enable=request;
			spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);

		}else if(1 == request){
			// Turn on
			printk("[Hall sensor] Turn on. \n");

			spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
			hall_sensor_dev->enable=request;
			spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);

		}else{
			printk("[Hall sensor] Enable/Disable Error, can not recognize (%d)", request);
		}

	}
	return count;
}

static DEVICE_ATTR(status, 0660, show_action_status, store_action_status);
static DEVICE_ATTR(switch, 0660,show_hall_sensor_enable, store_hall_sensor_enable);

static struct attribute *hall_sensor_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_switch.attr,
	NULL
};

static struct attribute_group hall_sensor_group = {
	.name = "hall_sensor",
	.attrs = hall_sensor_attrs
};


/*====================
*|| Initialization Part ||
*====================
*
*/

static int init_input_event(void)
{
	int ret = 0;

	hall_sensor_dev->hall_indev = input_allocate_device();
	if(!hall_sensor_dev->hall_indev){
		printk(" can't to allocate input event device\n");
		return -ENOMEM;
	}

	hall_sensor_dev->hall_indev->name = "hall_input";
	hall_sensor_dev->hall_indev->phys= "/dev/input/hall_indev";
	hall_sensor_dev->hall_indev->dev.parent= NULL;
	input_set_capability(hall_sensor_dev->hall_indev, EV_SW, SW_LID);

	ret = input_register_device(hall_sensor_dev->hall_indev);
	if (ret) {
		printk("[Input] Failed to register input event device\n");
		return -1;
	}

	dbg("[Input] Input Event registration Success!\n");
	return 0;
}

static int init_data(void)
{
	int ret = 0;

	/* Memory allocation for data structure */
	hall_sensor_dev = kzalloc(sizeof (struct hall_sensor_str), GFP_KERNEL);
	if (!hall_sensor_dev) {
		printk("kzalloc can't allocate for hall \n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	spin_lock_init(&hall_sensor_dev->mHallSensorLock);
	wake_lock_init(&hall_sensor_dev->wake_lock, WAKE_LOCK_SUSPEND, "Hall_wake_lock");

	hall_sensor_dev->enable = 1;

	return 0;
init_data_err:
	printk("Init Data ERROR\n");
	return ret;
}

static int set_pinctrl(struct device *dev)
{
	int ret = 0;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;

	key_pinctrl = devm_pinctrl_get(dev);
	set_state = pinctrl_lookup_state(key_pinctrl, "hall_gpio_high");
	ret = pinctrl_select_state(key_pinctrl, set_state);
	dbg("%s: pinctrl_select_state = %d\n", __FUNCTION__, ret);
	return ret;
}

static int init_irq_data (void)
{
	int ret = 0;

	/* GPIO to IRQ */
	HALL_SENSOR_IRQ = gpio_to_irq(HALL_SENSOR_GPIO);

	if (HALL_SENSOR_IRQ < 0) {
		printk(" gpio_to_irq ERROR, irq=%d.\n", HALL_SENSOR_IRQ);
	}else {
		dbg(" gpio_to_irq IRQ %d successed on GPIO:%d\n", HALL_SENSOR_IRQ, HALL_SENSOR_GPIO);
	}

	/*Request IRQ */
	ret = request_threaded_irq(HALL_SENSOR_IRQ, NULL, hall_sensor_interrupt_handler,
	IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	INT_NAME, hall_sensor_dev);

	if (ret < 0)
	printk(" request_irq() ERROR %d.\n", ret);
	else {
		dbg(" Enable irq !! \n");
		enable_irq_wake(HALL_SENSOR_IRQ);
	}

	return 0;
}

static ssize_t hall_send_cmd_read(struct file *dev, char *buf, size_t count, loff_t *ppos) {
	int ret;
	ret = sprintf(buf, " hall_default_value:%d \n",hall_sensor_dev->default_value);
	return ret;
}

static ssize_t hall_send_cmd_write(struct file *dev,const char *buf, size_t count, loff_t *ppos){
	int send_buff = -1;
	sscanf(buf, "%d", &send_buff);
	printk(" hall_send_cmd_write send_buff:%d \n",send_buff);
	if(send_buff==0) {
		hall_sensor_dev->default_value=0;
	} else if(send_buff==1) {
		hall_sensor_dev->default_value=1;
	} else {
		hall_sensor_dev->default_value=-1;
	}
	return count;
}


static const struct file_operations proc_hall_send = {
	.read       = hall_send_cmd_read,
	.write      = hall_send_cmd_write,
};

static ssize_t debounce_send_cmd_read(struct file *dev, char *buf, size_t count, loff_t *ppos) {
	int ret;
	ret = sprintf(buf, " debounce:%d \n",hall_sensor_dev->debounce);
	return ret;
}

static ssize_t debounce_send_cmd_write(struct file *dev,const char *buf, size_t count, loff_t *ppos){
	int send_buff = -1;
	sscanf(buf, "%d", &send_buff);
	printk(" debounce_send_cmd_write send_buff:%d \n",send_buff);
	hall_sensor_dev->debounce=send_buff;
	return count;
}

static const struct file_operations proc_debounce_send = {
	.read       = debounce_send_cmd_read,
	.write      = debounce_send_cmd_write,
};


static ssize_t sleep_send_cmd_read(struct file *dev, char *buf, size_t count, loff_t *ppos) {
	int ret;
	ret = sprintf(buf, " sleep:%d \n",hall_sensor_dev->debounce);
	return ret;
}

static ssize_t sleep_send_cmd_write(struct file *dev,const char *buf, size_t count, loff_t *ppos){
	int send_buff = -1;
	sscanf(buf, "%d", &send_buff);
	printk(" sleep_send_cmd_write send_buff:%d \n",send_buff);
	hall_sensor_dev->sleep=send_buff;
	return count;
}

static const struct file_operations proc_sleep_send = {
	.read       = sleep_send_cmd_read,
	.write      = sleep_send_cmd_write,
};


static int init_gpio_data (void)
{
	int ret = 0;

	HALL_SENSOR_GPIO = of_get_named_gpio(hall_pdev->dev.of_node, "qcom,hall-gpio", 0);
	ret = gpio_request(HALL_SENSOR_GPIO, IRQ_Name);
	if (ret) {
		printk(" Unable to request gpio %s(%d)\n", IRQ_Name, HALL_SENSOR_GPIO);
		goto probe_err;
	}

	ret = gpio_direction_input(HALL_SENSOR_GPIO);
	if (ret < 0) {
		printk(" Unable to set the direction of gpio %d\n", HALL_SENSOR_GPIO);
		goto probe_err;
	}
	return 0;

probe_err:
	return ret;
}

static int hall_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct proc_dir_entry* proc_hall_data = NULL;
	struct proc_dir_entry* proc_debounce_data = NULL;
	struct proc_dir_entry* proc_sleep_data = NULL;
	dbg("Probe \n");

#ifdef CONFIG_SENSOR_ENG_CMD
	proc_hall_data = proc_create("hall_sensor_cmd", 0666, NULL, &proc_hall_send);
	proc_debounce_data = proc_create("hall_sensor_debounce", 0666, NULL, &proc_debounce_send);
	proc_sleep_data = proc_create("hall_sensor_sleep", 0666, NULL, &proc_sleep_send);
#else
	proc_hall_data=NULL;
	proc_debounce_data=NULL;
	proc_sleep_data=NULL;
#endif

	ret = init_data();
	if (ret < 0)
		goto probe_err;

	ret = set_pinctrl(&pdev->dev);
	if (ret < 0)
		goto probe_err;
	
	hall_pdev = pdev;
	init_gpio_data();
	/* sysfs */

	ret = sysfs_create_group(&pdev->dev.kobj, &hall_sensor_group);
	if (ret) {
		printk("Hall sensor sysfs_create_group ERROR.\n");
		goto probe_err;
	}

	ret = init_input_event();
	if (ret < 0)
		goto probe_err;

	hall_sensor_dev->default_value=-1;
	hall_sensor_dev->irq_trigger=0;//default trigger low
	hall_sensor_dev->debounce=-1;
	hall_sensor_dev->sleep=-1;

	hall_sensor_dev->debounce=60;
	hall_sensor_dev->sleep=30;
	
	ret = init_irq_data();
	if (ret < 0)
		goto probe_err;

	hall_sensor_wq = create_singlethread_workqueue("hall_sensor_wq");
	if(1) {
		INIT_DEFERRABLE_WORK(&hall_sensor_dev->hall_sensor_work, debounce_hall_sensor_report_function);
	} else   {
		INIT_DEFERRABLE_WORK(&hall_sensor_dev->hall_sensor_work, hall_sensor_report_function);
	}

	queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
	return 0;

probe_err:
	printk("hall sensor probe fail\n");
	return ret;
}

static const struct platform_device_id hall_id_table[] = {
	{DRIVER_NAME, 1},
};

static struct of_device_id hallsensor_match_table[] = {
	{ .compatible = "qcom,hall",},
	{},
};

static struct platform_driver hall_sensor_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = hallsensor_match_table,
	},
	.probe      = hall_sensor_probe,
	.id_table	= hall_id_table,
};

static int __init hall_sensor_init(void)
{
	int err = 0;
	dbg("Driver INIT \n");

	err = platform_driver_register(&hall_sensor_driver);
	if (err != 0) {
		printk(" platform_driver_register fail, Error : %d\n", err);
	}
	printk(" platform_driver_register success \n" );
	return err;
}

static void __exit hall_sensor_exit(void)
{
	dbg("Driver EXIT \n");
#ifdef CONFIG_SENSOR_ENG_CMD
	if(proc_hall_data !=NULL) {
		remove_proc_entry("hall_sensor_cmd", NULL);
	}
	if(proc_debounce_data !=NULL) {
		remove_proc_entry("hall_sensor_debounce", NULL);
	}
	if(proc_sleep_data !=NULL) {
		remove_proc_entry("hall_sensor_sleep", NULL);
	}
#endif
	free_irq(HALL_SENSOR_IRQ, hall_sensor_dev);
	gpio_free(HALL_SENSOR_GPIO);
	kfree(hall_sensor_dev);
	hall_sensor_dev=NULL;
	wake_lock_destroy(&hall_sensor_dev->wake_lock);
	platform_driver_unregister(&hall_sensor_driver);
}

module_init(hall_sensor_init);
module_exit(hall_sensor_exit);

MODULE_DESCRIPTION("Hall Sensor");
MODULE_LICENSE("GPL v2");