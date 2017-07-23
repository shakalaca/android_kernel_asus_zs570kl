/* drivers/input/misc/cm3323e.c - cm3323e optical sensors driver
 *
 * Copyright (C) 2015 Vishay Capella Microsystems Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <asm/setup.h>
#include <linux/jiffies.h>
#include <linux/asus_project.h>
#include "lightsensor.h"
#include "cm3323e.h"

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define LS_POLLING_DELAY 50

#define REL_RED		REL_X
#define REL_GREEN	REL_Y
#define REL_BLUE	REL_Z
#define REL_WHITE	REL_MISC

#define ASUS_RGB_SENSOR_DATA_SIZE	5
#define ASUS_RGB_SENSOR_NAME_SIZE	32
#define ASUS_RGB_SENSOR_IOC_MAGIC                      ('C')	///< RGB sensor ioctl magic number 
#define ASUS_RGB_SENSOR_IOCTL_DATA_READ           _IOR(ASUS_RGB_SENSOR_IOC_MAGIC, 1, int[ASUS_RGB_SENSOR_DATA_SIZE])	///< RGB sensor ioctl command - Read data RGBW
#define ASUS_RGB_SENSOR_IOCTL_IT_SET           _IOW(ASUS_RGB_SENSOR_IOC_MAGIC, 2, int)	///< RGB sensor ioctl command - Set integration time
#define ASUS_RGB_SENSOR_IOCTL_DEBUG_MODE           _IOW(ASUS_RGB_SENSOR_IOC_MAGIC, 3, int)	///< RGB sensor ioctl command - Get debug mode
#define ASUS_RGB_SENSOR_IOCTL_MODULE_NAME           _IOR(ASUS_RGB_SENSOR_IOC_MAGIC, 4, char[ASUS_RGB_SENSOR_NAME_SIZE])	///< RGB sensor ioctl command - Get module name

#define READ_RGB_TEMP_DATA          //ASUS_BSP +++, Leong, Create Thread to read RGB Data

static void report_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(report_work, report_do_work);

static bool g_debugMode;

struct cm3323e_info {
	struct class *cm3323e_class;
	struct device *ls_dev;
	struct input_dev *ls_input_dev;
	struct regulator *reg_vdd;
	struct regulator *reg_vdd_i2c;

	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int als_enable;
	uint16_t it_setting;
	int (*power)(int, uint8_t); /* power to the chip */
	int it_time;
	int lightsensor_opened;
	int polling_delay;
};

struct cm3323e_status {
	bool need_wait;
	long long int start_time_ms;
	long long int end_time_ms;
	u32 start_time_jiffies;
	u32 end_time_jiffies;
	u16 delay_time_ms;
	u16 delay_time_jiffies;
};


struct cm3323e_info *lp_info;
struct cm3323e_status g_rgb_status = {false, 0, 0};

int enable_log = 0;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static int lightsensor_enable(struct cm3323e_info *lpi);
static int lightsensor_disable(struct cm3323e_info *lpi);

static uint16_t cm3323e_adc_red, cm3323e_adc_green, cm3323e_adc_blue, cm3323e_adc_white;

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	uint8_t subaddr[] = {cmd};

	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = subaddr,
		 },
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 2) > 0)
			break;

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[ERR][CM3323E error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0; 
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[ERR][CM3323E error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm3323e_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0) {
		pr_err(
			"[ERR][CM3323E error]%s: I2C_RxData fail [0x%x, 0x%x]\n",
			__func__, slaveAddr, cmd);
		return ret;
	}

	*pdata = (buffer[1] << 8) | buffer[0];
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM3323E] %s: I2C_RxData[0x%x, 0x%x] = 0x%x\n",
		__func__, slaveAddr, cmd, *pdata);
#endif
	return ret;
}

static int _cm3323e_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[CM3323E] %s: _cm3323e_I2C_Write_Word[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif
	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data & 0xff);
	buffer[2] = (uint8_t)((data & 0xff00) >> 8);

	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		pr_err("[ERR][CM3323E error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static int _cm3323e_I2C_Mask_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t mask, uint16_t data)
{
	s32 rc;
	uint16_t adc_data;

	rc = _cm3323e_I2C_Read_Word(SlaveAddress, cmd, &adc_data);
	if (rc) {
		pr_err("[ERR][CM3323E error]%s: Failed to read reg 0x%02x, rc=%d\n", __func__, cmd, rc);
		goto out;
	}
	adc_data &= ~mask;
	adc_data |= data & mask;
	rc = _cm3323e_I2C_Write_Word(SlaveAddress, cmd, adc_data);
	if (rc) {
		pr_err("[ERR][CM3323E error]%s: Failed to write reg 0x%02x, rc=%d\n", __func__, cmd, rc);
	}
out:
	return rc;
}

static void report_lsensor_input_event(struct cm3323e_info *lpi, bool resume)
{/*when resume need report a data, so the paramerter need to quick reponse*/
	//uint32_t r_val, g_val, b_val, w_val;

	mutex_lock(&als_get_adc_mutex);

	_cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_R_DATA, &cm3323e_adc_red);
	_cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_G_DATA, &cm3323e_adc_green);
	_cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_B_DATA, &cm3323e_adc_blue);
	_cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_W_DATA, &cm3323e_adc_white);	

	input_report_rel(lpi->ls_input_dev, REL_RED,   (int) cm3323e_adc_red);
	input_report_rel(lpi->ls_input_dev, REL_GREEN, (int) cm3323e_adc_green);
	input_report_rel(lpi->ls_input_dev, REL_BLUE,  (int) cm3323e_adc_blue);
	input_report_rel(lpi->ls_input_dev, REL_WHITE, (int) cm3323e_adc_white);
	input_sync(lpi->ls_input_dev);
	//printk("[LS][CM3323E] %s %x %x %x %x \n", __func__, cm3323e_adc_red, cm3323e_adc_green, cm3323e_adc_blue, cm3323e_adc_white);
	  
	mutex_unlock(&als_get_adc_mutex);
}

static void report_do_work(struct work_struct *work)
{
	struct cm3323e_info *lpi = lp_info;
	
 	if (enable_log)
		D("[CM3323E] %s\n", __func__);

	report_lsensor_input_event(lpi, 0);

	queue_delayed_work(lpi->lp_wq, &report_work, lpi->polling_delay);
}

static int als_power(int enable)
{
	struct cm3323e_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

static void dev_init(struct cm3323e_info *lpi)
{
	int ret = 0;
	ret = regulator_enable(lpi->reg_vdd);
	if (ret){
		pr_err("%s:%d failed to enable vdd\n", __func__, __LINE__);
	}
	ret = regulator_enable(lpi->reg_vdd_i2c);
	if (ret){
		pr_err("%s:%d failed to enable vdd_i2c\n", __func__, __LINE__);
	}
}

static void dev_deinit(struct cm3323e_info *lpi)
{
	int ret = 0;
	ret = regulator_disable(lpi->reg_vdd);
	if (ret){
		pr_err("%s:%d failed to disable vdd\n", __func__, __LINE__);
	}
	ret = regulator_disable(lpi->reg_vdd_i2c);
	if (ret){
		pr_err("%s:%d failed to disable vdd_i2c\n", __func__, __LINE__);
	}
}

void lightsensor_doDelay(u32 delay_time_ms)
{
	//RGB_DBG("%s: delay time = %u(ms)\n", __func__, delay_time_ms);
	if (delay_time_ms < 0 || delay_time_ms > g_rgb_status.delay_time_ms) {
		printk("[LS][CM3323E] %s: param wrong, delay default time = %u(ms)\n", __func__, g_rgb_status.delay_time_ms);
		if( g_rgb_status.delay_time_ms > 0 && g_rgb_status.delay_time_ms <= 1600 ) 
			msleep(g_rgb_status.delay_time_ms);
	} else{
		printk("[LS][CM3323E] %s: delay time = %u(ms)\n", __func__, delay_time_ms);
		if( delay_time_ms > 0 && delay_time_ms <= 1600 )  
			msleep(delay_time_ms);
	}
}

void lightsensor_checkStatus(void)
{
	struct timeval current_time; 
	long long int current_time_ms = 0;
	do_gettimeofday(&current_time);
	current_time_ms = current_time.tv_sec * 1000LL + current_time.tv_usec / 1000LL;

	if (g_rgb_status.need_wait) {
		/*handling jiffies overflow*/
		printk("[LS][CM3323E] %s: start at %lld, end at %lld, current at %lld, delay = %u(ms)\n", __func__,
			g_rgb_status.start_time_ms, g_rgb_status.end_time_ms, current_time_ms, g_rgb_status.delay_time_ms);
		/*normal case*/
		if (g_rgb_status.end_time_ms > g_rgb_status.start_time_ms) {
			/*between start and end - need delay*/
			if (current_time_ms >= g_rgb_status.start_time_ms && current_time_ms < g_rgb_status.end_time_ms)
				lightsensor_doDelay(g_rgb_status.end_time_ms - current_time_ms);
			else  // overflow or don't neet to delay
				lightsensor_doDelay(0);
		} 
		g_rgb_status.need_wait = false;
	}
}

static void lightsensor_setDelay(void)
{
	struct cm3323e_info *lpi = lp_info;
	struct timeval start_time; 
	g_rgb_status.need_wait = true;
	do_gettimeofday(&start_time);
	if (lpi->it_time >= 0 && lpi->it_time <= 5) {
		g_rgb_status.delay_time_ms = (40 << lpi->it_time) * 5 / 4;
		printk("[LS][CM3323E]%s: set delay time to %d ms\n", __func__, g_rgb_status.delay_time_ms);
	} else{
		g_rgb_status.delay_time_ms = 200;
		printk("[LS][CM3323E]%s: wrong IT time - %d, set delay time to 200ms \n", __func__, lpi->it_time);
	}
	g_rgb_status.start_time_ms = start_time.tv_sec * 1000LL + start_time.tv_usec / 1000LL;
	
	g_rgb_status.end_time_ms = g_rgb_status.start_time_ms + g_rgb_status.delay_time_ms;
}

static void lightsensor_itSet_ms(int input_ms)
{
	struct cm3323e_info *lpi = lp_info;
	int it_time;
	if (input_ms < 80) {
		it_time = CM3323E_CONF_IT_40MS;
	} else if (input_ms < 160) {
		it_time = CM3323E_CONF_IT_80MS;
	} else if (input_ms < 320) {
		it_time = CM3323E_CONF_IT_160MS;
	} else if (input_ms < 640) {
		it_time = CM3323E_CONF_IT_320MS;
	} else if (input_ms < 1280) {
		it_time = CM3323E_CONF_IT_640MS;
	} else{
		it_time = CM3323E_CONF_IT_1280MS;
	}
	lpi->it_time = it_time;
	lpi->polling_delay = msecs_to_jiffies( (40 << lpi->it_time) * 5 / 4 );
	printk("[LS][CM3323E] %s: write config - it time = %d, it set = %d, polling_delay = %d(%dms)\n", __func__, input_ms, lpi->it_time , lpi->polling_delay, (40 << lpi->it_time) * 5 / 4);
	if (lpi->als_enable == 1) {
		_cm3323e_I2C_Mask_Write_Word(CM3323E_ADDR, CM3323E_CONF, CM3323E_CONF_IT_MASK, lpi->it_time << 4);
	} else{
		printk("[LS][CM3323E] %s: write config - it time = %d, it set = %d\n", __func__, input_ms, lpi->it_time);
	}
}

static int lightsensor_enable(struct cm3323e_info *lpi)
{
	int ret = 0;

	mutex_lock(&als_enable_mutex);
	D("[LS][CM3323E] %s\n", __func__);
  
	dev_init(lpi);
	msleep(5);

	lpi->it_setting = lpi->it_time << 4;
	printk("[LS][CM3323E] %s setting %x \n", __func__, lpi->it_setting); 
	ret = _cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, lpi->it_setting);
	lightsensor_setDelay();
	if (ret < 0) {
		pr_err(
		"[LS][CM3323E error]%s: set auto light sensor fail\n",
		__func__);
	} 
	/*else {
		msleep(200); // wait for 200 ms for the first report
		report_lsensor_input_event(lpi, 1);// resume, IOCTL and DEVICE_ATTR
	}*/
#ifdef READ_RGB_TEMP_DATA			  //ASUS_BSP +++, Leong, Create Thread to read RGB Data
	queue_delayed_work(lpi->lp_wq, &report_work, lpi->polling_delay);
#endif
	lpi->als_enable = 1;
	mutex_unlock(&als_enable_mutex);
	
	return ret;
}

static int lightsensor_disable(struct cm3323e_info *lpi)
{
	int ret = 0;

	mutex_lock(&als_disable_mutex);

	D("[LS][CM3323E] %s\n", __func__);

    // reset to 40ms, to speed up next detecting cycle.
    lpi->it_time = CM3323E_CONF_IT_40MS;  // CM3323E_CONF_IT_160MS  CM3323E_CONF_IT_40MS  CM3323E_CONF_IT_640MS
    lpi->it_setting = CM3323E_CONF_DEFAULT | lpi->it_time << 4;
	ret = _cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, lpi->it_setting | CM3323E_CONF_SD );
	if (ret < 0){
		pr_err("[LS][CM3323E error]%s: disable auto light sensor fail\n",
			__func__);
	}
#ifdef READ_RGB_TEMP_DATA		  //ASUS_BSP +++, Leong, Create Thread to read RGB Data
	cancel_delayed_work_sync(&report_work); 	
#endif
	lpi->als_enable = 0;

	dev_deinit(lpi);

	mutex_unlock(&als_disable_mutex);
	
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct cm3323e_info *lpi = lp_info;
	int rc = 0;

	D("[LS][CM3323E] %s\n", __func__);
	if (lpi->lightsensor_opened) {
		pr_err("[LS][CM3323E error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	rc =  lightsensor_enable(lpi);
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct cm3323e_info *lpi = lp_info;
	int rc = 0;

	D("[LS][CM3323E] %s\n", __func__);
	lpi->lightsensor_opened = 0;
	rc = lightsensor_disable(lpi);
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct cm3323e_info *lpi = lp_info;
#if 1  //dit
	int m_ioctlData[ASUS_RGB_SENSOR_DATA_SIZE];
	char module_name[ASUS_RGB_SENSOR_NAME_SIZE] = CM3323E_I2C_NAME;
#endif

	switch (cmd) {
#if 1		  // for dit interface
		case ASUS_RGB_SENSOR_IOCTL_DATA_READ:

			lightsensor_checkStatus();

#ifndef READ_RGB_TEMP_DATA			  //ASUS_BSP +++, Leong, Create Thread to read RGB Data
			report_lsensor_input_event(lpi, 1);
#endif

			m_ioctlData[0] = cm3323e_adc_red;
			m_ioctlData[1] = cm3323e_adc_green;
			m_ioctlData[2] = cm3323e_adc_blue;
			m_ioctlData[3] = cm3323e_adc_white;
			m_ioctlData[4] = 0;
          //  printk("%s@\t ASUS_RGB_SENSOR_IOCTL_DATA_READ R = %d , G = %d , B = %d , W = %d  \n"
          //  	,__func__, m_ioctlData[0], m_ioctlData[1], m_ioctlData[2], m_ioctlData[3]);
            if( copy_to_user((int *)arg, &m_ioctlData, sizeof(int) * ASUS_RGB_SENSOR_DATA_SIZE) ) {
				printk("%s@\t commond fail !!\n", __func__);
				return -EFAULT;
			}
            return 0;
        case ASUS_RGB_SENSOR_IOCTL_IT_SET:
            //printk("%s@\t ASUS_RGB_SENSOR_IOCTL_IT_SET \n",__func__);
            if( copy_from_user(&val, (int *)arg, sizeof(int)) ) {
				printk("%s@\t commond fail !!\n", __func__);
				return -EFAULT;
			}
      //      printk("%s@\t ASUS_RGB_SENSOR_IOCTL_IT_SET - %d  \n",__func__,val);
            lightsensor_itSet_ms(val);

     //       lpi->it_setting = CM3323E_CONF_DEFAULT | cm3323e_it_mapping(val);
		//	if (lpi->als_enable) {
		//		_cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, lpi->it_setting);
		//	}
            return 0;
        case ASUS_RGB_SENSOR_IOCTL_DEBUG_MODE:
            printk("%s@\t ASUS_RGB_SENSOR_IOCTL_DEBUG_MODE \n",__func__);
            val = (g_debugMode) ? 1 : 0;
            if( copy_to_user((int*)arg, &val, sizeof(int)) ) {
				printk("%s@\t commond fail !!\n", __func__);
				return -EFAULT;
			}
            printk("%s@\t %d  \n",__func__,val);
            return 0;
        case ASUS_RGB_SENSOR_IOCTL_MODULE_NAME:
            printk("%s@\t ASUS_RGB_SENSOR_IOCTL_MODULE_NAME \n",__func__);
            if( copy_to_user((char *)arg, &module_name, sizeof(char) * ASUS_RGB_SENSOR_NAME_SIZE) ) {
				printk("%s@\t commond fail !!\n", __func__);
				return -EFAULT;
			}
            return 0;
#endif 
		case LIGHTSENSOR_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			D("[LS][CM3323E] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
				__func__, val);
			rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
			break;
		case LIGHTSENSOR_IOCTL_GET_ENABLED:
			val = lpi->als_enable;
			D("[LS][CM3323E] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
				__func__, val);
			rc = put_user(val, (unsigned long __user *)arg);
			break;
		default:
			pr_err("[LS][CM3323E error]%s: invalid cmd %d\n",
				__func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl,
	.compat_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = CM3323E_I2C_NAME,
	.fops = &lightsensor_fops
};

static int asusRgbDebug_proc_read(struct seq_file *buf, void *v)
{
    int result = 0;
    if (g_debugMode) {
        result = 1;
    } else{
        result = 0;
    }
    return seq_printf(buf, "%d\n", result);
}

static int asusRgbDebug_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, asusRgbDebug_proc_read, NULL);
}

static ssize_t asusRgbDebug_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
    int val;
    char messages[256];
    if (len > 256) {
        len = 256;
    }
    if (copy_from_user(messages, buff, len)) {
        return -EFAULT;
    }

    val = (int)simple_strtol(messages, NULL, 10);
    if (val == 0) {
        g_debugMode = false;
    } else{
        g_debugMode = true;
    }
    printk("%s: debug_value:%d\n",__func__, val);
    return len;
}

static const struct file_operations asus_rgb_debug_fops = {
    .owner = THIS_MODULE,
    .open =  asusRgbDebug_proc_open,
    .write = asusRgbDebug_proc_write,
    .read = seq_read,
};

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm3323e_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm3323e_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1)
		return -EINVAL;

	if (ls_auto) {
		ret = lightsensor_enable(lpi);
	} else {
		ret = lightsensor_disable(lpi);
	}

	D("[LS][CM3323E] %s: lpi->als_enable = %d, ls_auto = %d\n",
		__func__, lpi->als_enable, ls_auto);

	if (ret < 0)
		pr_err(
		"[LS][CM3323E error]%s: set auto light sensor fail\n",
		__func__);

	return count;
}

static ssize_t ls_poll_delay_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm3323e_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Poll Delay = %d ms\n",
			jiffies_to_msecs(lpi->polling_delay));

	return ret;
}

static ssize_t ls_poll_delay_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int new_delay;
	struct cm3323e_info *lpi = lp_info;

	sscanf(buf, "%d", &new_delay);
  
	D("new delay = %d ms, old delay = %d ms\n", 
		new_delay, jiffies_to_msecs(lpi->polling_delay));

	lpi->polling_delay = msecs_to_jiffies(new_delay);

	if( lpi->als_enable ){
		lightsensor_disable(lpi); 
		lightsensor_enable(lpi);
	}

	return count;
}

static uint16_t CONF_SETTING = 0;
static ssize_t ls_conf_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "CONF_SETTING = %x\n", CONF_SETTING);
}
static ssize_t ls_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int value = 0;
	sscanf(buf, "0x%x", &value);

	CONF_SETTING = value;
	printk(KERN_INFO "[LS]set CM3323E_CONF = %x\n", CONF_SETTING);
	_cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, CONF_SETTING);
	return count;
}

static ssize_t ls_red_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", cm3323e_adc_red);
}

static ssize_t ls_green_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", cm3323e_adc_green);
}

static ssize_t ls_blue_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", cm3323e_adc_blue);
}

static ssize_t ls_white_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", cm3323e_adc_white);
}

static struct device_attribute dev_attr_light_enable =
__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP, ls_enable_show, ls_enable_store);

static struct device_attribute dev_attr_light_poll_delay =
__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP, ls_poll_delay_show, ls_poll_delay_store);

static struct device_attribute dev_attr_light_conf =
__ATTR(conf, S_IRUGO | S_IWUSR | S_IWGRP, ls_conf_show, ls_conf_store);

static struct device_attribute dev_attr_light_red =
__ATTR(in_intensity_red, S_IRUGO | S_IRUSR | S_IRGRP, ls_red_show, NULL);

static struct device_attribute dev_attr_light_green =
__ATTR(in_intensity_green, S_IRUGO | S_IRUSR | S_IRGRP, ls_green_show, NULL);

static struct device_attribute dev_attr_light_blue =
__ATTR(in_intensity_blue, S_IRUGO | S_IRUSR | S_IRGRP, ls_blue_show, NULL);

static struct device_attribute dev_attr_light_white =
__ATTR(in_intensity_white, S_IRUGO | S_IRUSR | S_IRGRP, ls_white_show, NULL);

static struct attribute *light_sysfs_attrs[] = {
&dev_attr_light_enable.attr,
&dev_attr_light_poll_delay.attr,
&dev_attr_light_conf.attr,
&dev_attr_light_red.attr,
&dev_attr_light_green.attr,
&dev_attr_light_blue.attr,
&dev_attr_light_white.attr,
NULL
};

static struct attribute_group light_attribute_group = {
.attrs = light_sysfs_attrs,
};

static int ATD_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	int ret = 0;
	uint16_t conf_data;

	dev_init(lp_info);

	ret = _cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_CONF, &conf_data);
	seq_printf(buf, "%d\n", ret < 0 ? 0 : 1);

	dev_deinit(lp_info);
	return 0;
}

static int ATD_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_I2C_status_check_proc_read, NULL);
}

static const struct file_operations ATD_I2C_status_check_fops = {
    .owner = THIS_MODULE,
    .open = ATD_I2C_status_check_proc_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
 };

static void create_proc_file( void )
{
    proc_create(STATUS_PROC_FILE, 0664, NULL, &ATD_I2C_status_check_fops);
    proc_create(RGBDEBUG_PROC_FILE, 0664, NULL, &asus_rgb_debug_fops);
}

static int lightsensor_setup(struct cm3323e_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err(
			"[LS][CM3323E error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "cm3323e-ls";
	
	input_set_capability(lpi->ls_input_dev, EV_REL, REL_RED);
	input_set_capability(lpi->ls_input_dev, EV_REL, REL_GREEN);
	input_set_capability(lpi->ls_input_dev, EV_REL, REL_BLUE);
	input_set_capability(lpi->ls_input_dev, EV_REL, REL_WHITE);
	
	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("[LS][CM3323E error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	return ret;

err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int cm3323e_setup(struct cm3323e_info *lpi)
{
	int ret = 0;
 	uint16_t adc_data;

	als_power(1);
	msleep(5);

	// Shut down CM3323E
	ret = _cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, lpi->it_setting | CM3323E_CONF_SD);
	if(ret < 0)
		return ret;
		
	// Enable CM3323E
	ret = _cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, lpi->it_setting);
	if(ret < 0)
		return ret;

	msleep(200);
	
	// Get initial RED light data
	ret = _cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_R_DATA, &adc_data);
	if (ret < 0) {
		pr_err(
			"[LS][CM3323E error]%s: _cm3323e_I2C_Read_Word for RED fail\n",
			__func__);
		return -EIO;
	}
	
	// Get initial GREEN light data
	ret = _cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_G_DATA, &adc_data);
	if (ret < 0) {
		pr_err(
			"[LS][CM3323E error]%s: _cm3323e_I2C_Read_Word for GREEN fail\n",
			__func__);
		return -EIO;
	}	

	// Get initial BLUE light data
	ret = _cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_B_DATA, &adc_data);
	if (ret < 0) {
		pr_err(
			"[LS][CM3323E error]%s: _cm3323e_I2C_Read_Word for BLUE fail\n",
			__func__);
		return -EIO;
	}

	// Get initial WHITE light data
	ret = _cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_W_DATA, &adc_data);
	if (ret < 0) {
		pr_err(
			"[LS][CM3323E error]%s: _cm3323e_I2C_Read_Word for WHITE fail\n",
			__func__);
		return -EIO;
	}

	// Shut down CM3323E
	ret = _cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, CM3323E_CONF_DEFAULT | CM3323E_CONF_IT_160MS | CM3323E_CONF_SD);

	return ret;
}

static int cm3323e_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm3323e_info *lpi;

	D("[CM3323E] %s: Probe start\n", __func__);

	lpi = kzalloc(sizeof(struct cm3323e_info), GFP_KERNEL);
	if (!lpi) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	lpi->reg_vdd = devm_regulator_get(&client->dev, "vdd");
	lpi->reg_vdd_i2c = devm_regulator_get(&client->dev, "vdd_i2c");

	switch (asus_HW_ID) {
		case HW_ID_EVB:
		case HW_ID_EVB2:
		case HW_ID_SR1:
		case HW_ID_SR2:
			// for stage before ER2, vdd should be always enabled.
			ret = regulator_enable(lpi->reg_vdd);
			if (ret){
				pr_err("%s:%d failed to enable vdd\n", __func__, __LINE__);
			}
			break;

		default:
			break;
	}

    g_debugMode = false;

	dev_init(lpi);

	lpi->i2c_client = client;

	i2c_set_clientdata(client, lpi);

	lpi->power = NULL; //if necessary, add power function here for sensor chip

	lpi->polling_delay = msecs_to_jiffies(LS_POLLING_DELAY);
	lp_info = lpi;
	lpi->it_setting = CM3323E_CONF_DEFAULT | CM3323E_CONF_IT_160MS;

	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		pr_err("[LS][CM3323E error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}

	lpi->lp_wq = create_singlethread_workqueue("cm3323e_wq");
	if (!lpi->lp_wq) {
		pr_err("[CM3323E error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	ret = cm3323e_setup(lpi);
	if (ret < 0) {
		pr_err("[ERR][CM3323E error]%s: cm3323e_setup error!\n", __func__);
		goto err_cm3323e_setup;
	}

	// create /dev node
	ret = misc_register(&lightsensor_misc);
    if (ret != 0) {
        pr_err("[ERR][CM3323E error]%s: cannot register miscdev on minor=11 (err=%d)\n", __func__,ret);
        goto err_create_misc;
    }

	lpi->cm3323e_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(lpi->cm3323e_class)) {
		ret = PTR_ERR(lpi->cm3323e_class);
		lpi->cm3323e_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->cm3323e_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}
	/* register the attributes */
	ret = sysfs_create_group(&lpi->ls_input_dev->dev.kobj,
	&light_attribute_group);
	if (ret) {
		pr_err("[LS][CM3323E error]%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_light;
	}

	create_proc_file();

	lpi->als_enable = 0;
	dev_deinit(lpi);
	D("[CM3323E] %s: Probe success!\n", __func__);

	return ret;

err_sysfs_create_group_light:
	device_unregister(lpi->ls_dev);
err_create_ls_device:
	class_destroy(lpi->cm3323e_class);
err_create_class:
	misc_deregister(&lightsensor_misc);
err_create_misc:
err_cm3323e_setup:
	destroy_workqueue(lpi->lp_wq);
err_create_singlethread_workqueue:
	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
err_lightsensor_setup:
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_get_adc_mutex);
	dev_deinit(lpi);
	kfree(lpi);
	regulator_disable(lpi->reg_vdd);
	return ret;
}

static const struct i2c_device_id cm3323e_i2c_id[] = {
	{CM3323E_I2C_NAME, 0},
	{}
};

#ifdef CONFIG_OF
  static struct of_device_id cm3323e_match_table[] = {
          { .compatible = "capella,cm3323e",},
          { },
  };
#else
  #define cm3323e_match_table NULL
#endif

static struct i2c_driver cm3323e_driver = {
	.id_table = cm3323e_i2c_id,
	.probe = cm3323e_probe,
	.driver = {
		.name = CM3323E_I2C_NAME,
		.owner = THIS_MODULE,
 		.of_match_table = of_match_ptr(cm3323e_match_table),
	},
};

static int __init cm3323e_init(void)
{
	return i2c_add_driver(&cm3323e_driver);
}

static void __exit cm3323e_exit(void)
{
	i2c_del_driver(&cm3323e_driver);
}

module_init(cm3323e_init);
module_exit(cm3323e_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("RGB sensor device driver for CM3323E");
MODULE_AUTHOR("Frank Hsieh <frank.hsieh@vishay.com>");
