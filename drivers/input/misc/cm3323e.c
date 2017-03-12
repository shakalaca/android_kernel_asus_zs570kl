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
#include <asm/uaccess.h>
//#include <asm/mach-types.h>
#include <linux/cm3323e.h>
#include <asm/setup.h>
#include <linux/jiffies.h>
#include <linux/regulator/consumer.h>
#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define LS_POLLING_DELAY 600

#define REL_RED		REL_X
#define REL_GREEN	REL_Y
#define REL_BLUE	REL_Z
#define REL_WHITE	REL_MISC
#define CM3323_VDD_MIN_UV	3000000
#define CM3323_VDD_MAX_UV	3000000
#define CM3323_VI2C_MIN_UV	1800000
#define CM3323_VI2C_MAX_UV	1800000
static void report_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(report_work, report_do_work);

struct cm3323e_info {
	struct class *cm3323e_class;
	struct device *ls_dev;
	struct input_dev *ls_input_dev;

	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int als_enable;
	int (*power)(int, uint8_t); /* power to the chip */
	int rgb_debug;
	int rgbsensor_opened;
	int polling_delay;
	struct regulator *vdd;
    	struct regulator *vio;
};
struct cm3323e_info *lp_info;
int enable_log = 0;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static int rgbsensor_enable(struct cm3323e_info *lpi);
static int rgbsensor_disable(struct cm3323e_info *lpi);

static uint16_t cm3323e_adc_red, cm3323e_adc_green, cm3323e_adc_blue, cm3323e_adc_white;
int rgb_data[5] = {0};
static int cm3323_power_set(struct cm3323e_info * info, bool on);
static uint16_t CONF_SETTING = 0;

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	uint8_t subaddr[1];

	
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
	subaddr[0] = cmd;
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
	//pr_err("[LS][CM3323E test]%s: anna 11test cm3323 slaveAddr=%d ,cmd =%d \n",__func__,slaveAddr,cmd);
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
	//pr_err("[LS][CM3323E test]%s: anna 11test cm3323 slaveAddr=%d ,cmd =%d ,data=%d \n",__func__,SlaveAddress,cmd,data);
	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		pr_err("[ERR][CM3323E error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}
//wxtest
	if (cmd == CM3323E_CONF) CONF_SETTING = data;
//wxtest

	return ret;
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
	//D("[LS][CM3323E] %s %x %x %x %x \n", __func__, cm3323e_adc_red, cm3323e_adc_green, cm3323e_adc_blue, cm3323e_adc_white);
	  
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


static int cm3323_power_set(struct cm3323e_info *info, bool on)
{
	int rc;

	if (on) {
/*
		info->vdd = regulator_get(&info->i2c_client->dev, "vdd");
		if (IS_ERR(info->vdd)) {
			rc = PTR_ERR(info->vdd);
			dev_err(&info->i2c_client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			goto err_vdd_get;
		}

		if (regulator_count_voltages(info->vdd) > 0) {
			rc = regulator_set_voltage(info->vdd,
					CM3323_VDD_MIN_UV, CM3323_VDD_MAX_UV);
			if (rc) {
				dev_err(&info->i2c_client->dev,
					"Regulator set failed vdd rc=%d\n", rc);
				goto err_vdd_set_vtg;
			}
		}
*/
		info->vio = regulator_get(&info->i2c_client->dev, "vio");
		if (IS_ERR(info->vio)) {
			rc = PTR_ERR(info->vio);
			dev_err(&info->i2c_client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto err_vio_get;
		}

		if (regulator_count_voltages(info->vio) > 0) {
			rc = regulator_set_voltage(info->vio,
				CM3323_VI2C_MIN_UV, CM3323_VI2C_MAX_UV);
			if (rc) {
				dev_err(&info->i2c_client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set_vtg;
			}
		}
/*
		rc = regulator_enable(info->vdd);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_ena;
		}
*/
		rc = regulator_enable(info->vio);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_ena;
		}
	} else {
/*
		rc = regulator_disable(info->vdd);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(info->vdd) > 0)
			regulator_set_voltage(info->vdd, 0, CM3323_VDD_MAX_UV);

		regulator_put(info->vdd);
*/
		rc = regulator_disable(info->vio);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(info->vio) > 0)
			regulator_set_voltage(info->vio, 0,
					CM3323_VI2C_MAX_UV);

		regulator_put(info->vio);
	}

	dev_err(&info->i2c_client->dev,"CM3323 set power ok");
	return 0;

err_vio_ena:
/*
	regulator_disable(info->vdd);
err_vdd_ena:
*/
	if (regulator_count_voltages(info->vio) > 0)
		regulator_set_voltage(info->vio, 0, CM3323_VI2C_MAX_UV);
err_vio_set_vtg:
	regulator_put(info->vio);
err_vio_get:
/*
	if (regulator_count_voltages(info->vdd) > 0)
		regulator_set_voltage(info->vdd, 0, CM3323_VDD_MAX_UV);
err_vdd_set_vtg:
	regulator_put(info->vdd);
err_vdd_get:
*/
	return rc;
}





static int rgbsensor_enable(struct cm3323e_info *lpi)
{
	int ret = 0;
	uint16_t setting;

	mutex_lock(&als_enable_mutex);
	pr_err("[LS][CM3323E] %s\n", __func__);
  
	setting = CM3323E_CONF_DEFAULT | CM3323E_CONF_IT_40MS;
	ret = _cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, setting);
	//pr_err("[LS][CM3323E] %s  anna test setting =%d\n", __func__,setting);
	if (ret < 0) {
		pr_err(
		"[LS][CM3323E error]%s: set auto rgb sensor fail\n",
		__func__);
	} else {
		msleep(50);/*wait for 200 ms for the first report*/	
		report_lsensor_input_event(lpi, 1);/*resume, IOCTL and DEVICE_ATTR*/	
	}
	
	queue_delayed_work(lpi->lp_wq, &report_work, lpi->polling_delay);
	lpi->als_enable = 1;
	mutex_unlock(&als_enable_mutex);
	
	return ret;
}

static int rgbsensor_disable(struct cm3323e_info *lpi)
{
	int ret = 0;
	uint16_t setting;

	mutex_lock(&als_disable_mutex);

	D("[LS][CM3323E] %s\n", __func__);
//wxtest
	setting = CONF_SETTING | CM3323E_CONF_SD;
//wxtest
	ret = _cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, setting);
	if (ret < 0){
		pr_err("[LS][CM3323E error]%s: disable auto rgb sensor fail\n",
			__func__);
	}

	cancel_delayed_work_sync(&report_work); 	
	lpi->als_enable = 0;
	mutex_unlock(&als_disable_mutex);
	
	return ret;
}

static int rgbsensor_open(struct inode *inode, struct file *file)
{
	struct cm3323e_info *lpi = lp_info;
	int rc = 0;

	D("[LS][CM3323E] %s\n", __func__);
	if (lpi->rgbsensor_opened) {
		pr_err("[LS][CM3323E error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->rgbsensor_opened = 1;
	return rc;
}

static int rgbsensor_release(struct inode *inode, struct file *file)
{
	struct cm3323e_info *lpi = lp_info;

	D("[LS][CM3323E] %s\n", __func__);
	lpi->rgbsensor_opened = 0;
	return 0;
}

static long rgbsensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	int rc=0, val,time,delaytime=0;
	struct cm3323e_info *lpi = lp_info;
	void __user *argp = (void __user *)arg;
        uint16_t config_setting = 0;
	char modelName[ASUS_RGB_SENSOR_NAME_SIZE]; 
	switch (cmd) {
		case ASUS_RGB_SENSOR_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			D("[LS][CM3323E] %s rgbSENSOR_IOCTL_ENABLE, value = %d\n",
				__func__, val);
			rc = val ? rgbsensor_enable(lpi) : rgbsensor_disable(lpi);
			break;
		case ASUS_RGB_SENSOR_IOCTL_GET_ENABLED:
			val = lpi->als_enable;
			D("[LS][CM3323E] %s rgbSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
				__func__, val);
			rc = put_user(val, (unsigned long __user *)arg);
			break;
		case ASUS_RGB_SENSOR_IOCTL_IT_SET:
			//D("%s:ASUS_RGB_SENSOR_IOCTL_IT_SET begin \n", __func__);
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			if((0<=val) && ( val <= 60))
				time = 0;
			else if ((60<val) && ( val <= 120))
				time= 1;
			else if ((120<val) && ( val <= 240))
				time= 2;
			else if ((240<val) && ( val <= 480))
				time= 3;
			else if ((480<val) && ( val <= 960))
				time= 4;
			else if(960<val)
				time= 5;
//wxtest
//			_cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_CONF, &config_setting);
			
//			config_setting  &= ~(BIT4+BIT5+BIT6);

			config_setting = CONF_SETTING & CM3323E_CONF_SD; //keep enable/disable conf
			if ((CONF_SETTING & ~(BIT4+BIT5+BIT6)) != (time << 4)) { //write CM3323E_CONF if IT changes
			rgbsensor_disable(lpi);
//wxtest
			config_setting  |= time<<4;
			D("%s:ASUS_RGB_SENSOR_IOCTL_IT_SET CONFIG=0x%x, val=%d\n", __func__,config_setting,val);
			rc= _cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, config_setting);
			if(rc <0){
				pr_err("%s:ASUS failed to write RGB TIME.\n",__func__);
				rc = -EFAULT;			
				break;
			}
			
			rc = 0;
			delaytime = val *125/100;
			mdelay(delaytime);
//wxtest
			}
//wxtest
			//D("%s:ASUS_RGB_SENSOR_IOCTL_IT_SET end \n", __func__);
			break;
		case ASUS_RGB_SENSOR_IOCTL_DEBUG_MODE:
			//D("%s:ASUS_RGB_SENSOR_IOCTL_DEBUG_MODE begin\n", __func__);
			val=lpi->rgb_debug;
			
			//D("%s:ASUS debug mode argp= %p, val = %p, sizeof(val)=%lx\n",__func__,argp,&val,sizeof(val));
			if ( copy_to_user(argp, &val, sizeof(val) ) ) {
	     			pr_err("%s:ASUS failed to copy RBG debug mode to user space.\n",__func__);
				rc = -EFAULT;			
				break;
	    		}
			rc = 0;	
			D("%s:ASUS_RGB_SENSOR_IOCTL_DEBUG_MODE end \n", __func__);
			break;
		case ASUS_RGB_SENSOR_IOCTL_MODULE_NAME:
			//D("%s:ASUS_RGB_SENSOR_IOCTL_MODULE_NAME begin \n", __func__);
			
			switch(asus_project_id)
				{
				case ASUS_ZC552KL://ASUS_ZC552KL
					strcpy(modelName,"ZC552KL");
					//modelName="ZC552KL";
					break;
				case ASUS_ZS550KL://ASUS_ZS550KL
					strcpy(modelName,"ZS550KL");
					//modelName="ZS550KL";
					break;
				case ASUS_ZD552KL: //ASUS_ZD552KL
					strcpy(modelName,"ZD552KL");
					//modelName="ZD552KL";
					break;
				default:
					strcpy(modelName,"ZD552KL");
					//modelName="ZD552KL";
					break;
				} 

		  	//D("%s:ASUS model name argp= %p, val = %p sizeof(val)=%lx\n",__func__,argp,&modelName,sizeof(modelName));
			if ( copy_to_user(argp, &modelName, sizeof(modelName) ) ) {
	     			pr_err("%s:ASUS failed to copy model name  to user space.\n",__func__);
				rc = -EFAULT;			
				break;
	    		}	
			rc = 0;
			//D("%s:ASUS_RGB_SENSOR_IOCTL_MODULE_NAME %s end \n", __func__,modelName);
			
			break;
		case ASUS_RGB_SENSOR_IOCTL_DATA_READ:
			//D("%s:ASUS_RGB_SENSOR_IOCTL_DATA_READ11 begin \n", __func__);
			// rgbsensor_enable(lpi);
			mutex_lock(&als_get_adc_mutex);
			
			_cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_R_DATA, &cm3323e_adc_red);
			_cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_G_DATA, &cm3323e_adc_green);
			_cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_B_DATA, &cm3323e_adc_blue);
			_cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_W_DATA, &cm3323e_adc_white);
			
			//D("[LS][CM3323E] %s ASUS_RGB_SENSOR_IOCTL_DATA_READ, r = %d,g = %d,b = %d,w = %d\n",__func__, cm3323e_adc_red,cm3323e_adc_green,cm3323e_adc_blue,cm3323e_adc_white);
			rgb_data[0]= cm3323e_adc_red;
			rgb_data[1]= cm3323e_adc_green;
			rgb_data[2]= cm3323e_adc_blue;
			rgb_data[3]= cm3323e_adc_white;
			rgb_data[4]= 0;
			if ( copy_to_user(argp, &rgb_data, sizeof(rgb_data) ) ) {
	     			pr_err("%s:ASUS failed to copy RBG data to user space.\n",__func__);
				rc = -EFAULT;			
				break;
	    		}
			mutex_unlock(&als_get_adc_mutex);
			//rgbsensor_disable(lpi);
			rc = 0;
			//D("%s:ASUS_RGB_SENSOR_IOCTL_DATA_READ end \n", __func__);
			break;
		default:
			pr_err("[LS][CM3323E error]%s: invalid cmd %d\n",
				__func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}
	
	return rc;
}

static const struct file_operations rgbsensor_fops = {
	.owner = THIS_MODULE,
	.open = rgbsensor_open,
	.release = rgbsensor_release,
	.unlocked_ioctl = rgbsensor_ioctl,
	.compat_ioctl = rgbsensor_ioctl,
};


static struct miscdevice rgbsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "asusRgbSensor",
	.fops = &rgbsensor_fops
};

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm3323e_info *lpi = lp_info;

	ret = sprintf(buf, "rgb sensor Auto Enable = %d\n",
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
	pr_err("[LS][CM3323E error]%s: anna test node11 ls_auto=%d \n",__func__,ls_auto);
	if (ls_auto) {
		ret = rgbsensor_enable(lpi);
	} else {
		ret = rgbsensor_disable(lpi);
	}


	pr_err("[LS][CM3323E] %s: lpi->als_enable = %d, ls_auto = %d\n",
		__func__, lpi->als_enable, ls_auto);

	if (ret < 0)
		pr_err(
		"[LS][CM3323E error]%s: set auto rgb sensor fail\n",
		__func__);

	return count;
}

static ssize_t ls_poll_delay_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm3323e_info *lpi = lp_info;

	ret = sprintf(buf, "rgb sensor Poll Delay = %d ms\n",
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
		rgbsensor_disable(lpi); 
		rgbsensor_enable(lpi);
	}

	return count;
}

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
	uint16_t conf_value;
	sscanf(buf, "0x%x", &value);

	conf_value = value;
	printk(KERN_INFO "[LS]set CM3323E_CONF = %x\n", conf_value);
	_cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, conf_value);
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



static ssize_t ls_rgb_debug_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm3323e_info *lpi = lp_info;

	ret = sprintf(buf, "RGB sensor debug enable = %d\n",
			lpi->rgb_debug);

	return ret;
}

static ssize_t ls_rgb_debug_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ls_auto;
	struct cm3323e_info *lpi = lp_info;
	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1)
		return -EINVAL;
	lpi->rgb_debug = ls_auto;
        pr_err("[LS][CM3323E ]%s: anna test lpi->rgb_debug =%d \n",__func__,lpi->rgb_debug );

	return count;
}
//anna add for atd spec
static ssize_t ls_rgb_status(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0 ,ret1=0;
	 uint16_t config_setting = 0;
	ret1= _cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_CONF, &config_setting);
	//struct cm3323e_info *lpi = lp_info;
	if(ret1 < 0){
		ret = sprintf(buf, "0 \n");
	}else{
		ret = sprintf(buf, "1 \n");
	}
	
	return ret;
}
//anna add for atd spec
static DEVICE_ATTR(rgb_enable, S_IRUGO | S_IWUSR | S_IWGRP,ls_enable_show, ls_enable_store);
static DEVICE_ATTR(rgb_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,ls_poll_delay_show, ls_poll_delay_store);
static DEVICE_ATTR(rgb_conf , S_IRUGO | S_IWUSR | S_IWGRP,ls_conf_show, ls_conf_store);
static DEVICE_ATTR(rgb_red, S_IRUGO | S_IRUSR | S_IRGRP,ls_red_show, NULL);
static DEVICE_ATTR(rgb_green, S_IRUGO | S_IRUSR | S_IRGRP,ls_green_show, NULL);
static DEVICE_ATTR(rgb_blue, S_IRUGO | S_IRUSR | S_IRGRP,ls_blue_show, NULL);
static DEVICE_ATTR(rgb_white, S_IRUGO | S_IRUSR | S_IRGRP,ls_white_show, NULL);
static DEVICE_ATTR(rgb_debug_enable, S_IRUGO | S_IWUSR | S_IWGRP,ls_rgb_debug_show, ls_rgb_debug_store);
static DEVICE_ATTR(rgb_status, S_IRUGO | S_IRUSR | S_IRGRP,ls_rgb_status, NULL);
/*
static struct device_attribute dev_attr_rgb_enable =
__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP, ls_enable_show, ls_enable_store);

static struct device_attribute dev_attr_rgb_poll_delay =
__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP, ls_poll_delay_show, ls_poll_delay_store);

static struct device_attribute dev_attr_rgb_conf =
__ATTR(conf, S_IRUGO | S_IWUSR | S_IWGRP, ls_conf_show, ls_conf_store);

static struct device_attribute dev_attr_rgb_red =
__ATTR(in_intensity_red, S_IRUGO | S_IRUSR | S_IRGRP, ls_red_show, NULL);

static struct device_attribute dev_attr_rgb_green =
__ATTR(in_intensity_green, S_IRUGO | S_IRUSR | S_IRGRP, ls_green_show, NULL);

static struct device_attribute dev_attr_rgb_blue =
__ATTR(in_intensity_blue, S_IRUGO | S_IRUSR | S_IRGRP, ls_blue_show, NULL);

static struct device_attribute dev_attr_rgb_white =
__ATTR(in_intensity_white, S_IRUGO | S_IRUSR | S_IRGRP, ls_white_show, NULL);
*/
static struct attribute *rgb_sysfs_attrs[] = {
&dev_attr_rgb_enable.attr,
&dev_attr_rgb_poll_delay.attr,
&dev_attr_rgb_conf.attr,
&dev_attr_rgb_red.attr,
&dev_attr_rgb_green.attr,
&dev_attr_rgb_blue.attr,
&dev_attr_rgb_white.attr,
&dev_attr_rgb_debug_enable.attr,
&dev_attr_rgb_status.attr,
NULL
};

static struct attribute_group rgb_attribute_group = {
.attrs = rgb_sysfs_attrs,
};

static int rgbsensor_setup(struct cm3323e_info *lpi)
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


	ret = misc_register(&rgbsensor_misc);
	if (ret < 0) {
		pr_err("[LS][CM3323E error]%s: can not register ls misc device\n", __func__);

		goto err_free_misc_device;
	}

		//pr_err("[LS][CM3323E test]%s:anna  test \n",__func__);
	return ret;
	
err_free_misc_device:
misc_deregister(&rgbsensor_misc);
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
	ret = _cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, CM3323E_CONF_DEFAULT | CM3323E_CONF_IT_160MS | CM3323E_CONF_SD);
	if(ret < 0)
		return ret;
	//	pr_err("[LS][CM3323E test]%s: anna 11test cm3323\n",__func__);
	// Enable CM3323E
	ret = _cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, CM3323E_CONF_DEFAULT | CM3323E_CONF_IT_160MS);
	if(ret < 0)
		return ret;

// Get initial RED rgb data
	ret = _cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_CONF, &adc_data);
	if (ret < 0) {
		pr_err("[LS][CM3323E error]%s: _cm3323e_I2C_Read_Word for RED fail\n",__func__);
		return -EIO;
	}
	//pr_err("[LS][CM3323E test]%s: anna 22test cm3323 CM3323E_CONF=%d, adc_data=%d\n",__func__,CM3323E_CONF,adc_data);
	

	msleep(200);
	
	// Get initial RED rgb data
	ret = _cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_R_DATA, &adc_data);
	if (ret < 0) {
		pr_err("[LS][CM3323E error]%s: _cm3323e_I2C_Read_Word for RED fail\n",__func__);
		return -EIO;
	}
//	pr_err("[LS][CM3323E test]%s: anna 22test cm3323 red= adc_data=%d\n",__func__,adc_data);
	// Get initial GREEN rgb data
	ret = _cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_G_DATA, &adc_data);
	if (ret < 0) {
		pr_err(
			"[LS][CM3323E error]%s: _cm3323e_I2C_Read_Word for GREEN fail\n",
			__func__);
		return -EIO;
	}	
//	pr_err("[LS][CM3323E test]%s: anna 33test cm3323 ,green=adc_data=%d\n",__func__,adc_data);
	// Get initial BLUE rgb data
	ret = _cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_B_DATA, &adc_data);
	if (ret < 0) {
		pr_err(
			"[LS][CM3323E error]%s: _cm3323e_I2C_Read_Word for BLUE fail\n",
			__func__);
		return -EIO;
	}
//	pr_err("[LS][CM3323E test]%s: anna 33test cm3323 ,,bule=adc_data=%d\n",__func__,adc_data);
	// Get initial WHITE rgb data
	ret = _cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_W_DATA, &adc_data);
	if (ret < 0) {
		pr_err(
			"[LS][CM3323E error]%s: _cm3323e_I2C_Read_Word for WHITE fail\n",
			__func__);
		return -EIO;
	}
//	pr_err("[LS][CM3323E test]%s: anna 44test cm3323 ,CM3323E_W_DATA,adc_data=%d\n",__func__,adc_data);
	return ret;
}

static int cm3323e_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm3323e_info *lpi;

	D("[CM3323E] %s\n", __func__);

	lpi = kzalloc(sizeof(struct cm3323e_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	lpi->i2c_client = client;

	i2c_set_clientdata(client, lpi);

	lpi->power = NULL; //if necessary, add power function here for sensor chip

	lpi->polling_delay = msecs_to_jiffies(LS_POLLING_DELAY);
	lpi->rgb_debug = 0;
	lp_info = lpi;
	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);

	ret = cm3323_power_set(lpi, true);
	if (ret < 0) {	
		pr_err("[CM3323E error]%s:cm3323 power on error!\n", __func__);
		goto err_cm3323_power_on;
	}

	ret = cm3323e_setup(lpi);
	if (ret < 0) {
		pr_err("[ERR][CM3323E error]%s: cm3323e_setup error!\n", __func__);
		goto err_cm3323e_setup;
	}

	ret = rgbsensor_setup(lpi);
	if (ret < 0) {
		pr_err("[LS][CM3323E error]%s: rgbsensor_setup error!!\n",
			__func__);
		goto err_rgbsensor_setup;
	}

	lpi->lp_wq = create_singlethread_workqueue("cm3323e_wq");
	if (!lpi->lp_wq) {
		pr_err("[CM3323E error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	lpi->cm3323e_class = class_create(THIS_MODULE, "rgb_sensors");
	if (IS_ERR(lpi->cm3323e_class)) {
		ret = PTR_ERR(lpi->cm3323e_class);
		lpi->cm3323e_class = NULL;
		goto err_create_class;
	}
	
	lpi->ls_dev = device_create(lpi->cm3323e_class,
				NULL, 0, "%s", "rgbsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}
	//pr_err("[LS][CM3323E test]%s: anna 444test cm3323 \n",__func__);
	/* register the attributes */
	ret = sysfs_create_group(&lpi->ls_input_dev->dev.kobj,
	&rgb_attribute_group);
	if (ret) {
		pr_err("[LS][CM3323E error]%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_rgb;
	}


	ret = device_create_file(lpi->ls_dev, &dev_attr_rgb_enable);
	ret = device_create_file(lpi->ls_dev, &dev_attr_rgb_poll_delay);
	ret = device_create_file(lpi->ls_dev, &dev_attr_rgb_conf);
	ret = device_create_file(lpi->ls_dev, &dev_attr_rgb_red);
	ret = device_create_file(lpi->ls_dev, &dev_attr_rgb_green);
	ret = device_create_file(lpi->ls_dev, &dev_attr_rgb_blue);
	ret = device_create_file(lpi->ls_dev, &dev_attr_rgb_white);
	ret = device_create_file(lpi->ls_dev, &dev_attr_rgb_debug_enable);
	ret = device_create_file(lpi->ls_dev, &dev_attr_rgb_status);
//	D("[CM3323E] %s: anna test node ok\n", __func__);
	if (ret)
		goto err_create_ls_device_file;
	lpi->als_enable = 0;
	lp_info = lpi;
	D("[CM3323E] %s: Probe success!\n", __func__);

	return ret;
err_create_ls_device_file:
err_sysfs_create_group_rgb:
	device_unregister(lpi->ls_dev);
err_create_ls_device:
	class_destroy(lpi->cm3323e_class);
err_create_class:
	destroy_workqueue(lpi->lp_wq);
	input_free_device(lpi->ls_input_dev);
err_create_singlethread_workqueue:
err_rgbsensor_setup:
err_cm3323e_setup:
	cm3323_power_set(lpi, false);
err_cm3323_power_on:
	mutex_destroy(&als_get_adc_mutex);
	mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_enable_mutex);
	kfree(lpi);
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
