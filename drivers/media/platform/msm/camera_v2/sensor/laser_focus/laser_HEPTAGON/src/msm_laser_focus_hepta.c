/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include "msm_laser_focus.h"
#include "laser_log.h"
#include "HPTG_debug.h"
#include "HPTG_interface.h"
#include "HPTG_factory_func.h"
#include "HPTG_shipping_func.h"
#include "laser_focus_hepta.h"
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>


#define LASER_NAME "qcom,laser"
static struct i2c_driver olivia_i2c_driver;

static int ATD_status;

#define DO_CAL true
#define NO_CAL false


/*
Range2 for keeping continuous measuring mode data
*/

#define HEPT_DMAX	400
int ErrCode1 = 0;
int ErrCode2 = 0;
uint16_t Range1 =0;
uint16_t Range2 =0;

/*
continuous measuring mode, make sure ioctrl_close base on timedMeasure is true
*/
//extern int g_ftm_mode;
int g_ftm_mode_HEP=0;
extern int g_factory;

extern int Laser_log_cnt;
int proc_log_cnt=0;

extern bool timedMeasure;
bool ioctrl_close = true;
bool close_done = false;
struct delayed_work		keepMeasure;
struct workqueue_struct*	Measure_wq;
struct work_struct			Measure_wk;

bool repairing_state = false;

struct msm_laser_focus_ctrl_t *laura_t;

uint16_t Settings[NUMBER_OF_SETTINGS];

#if 0
#enable on
static bool camera_on_flag = false;
#endif

static bool calibration_flag = true;


static bool load_calibration_data = false;

static int i2c_status=0;

static int client=0;

extern int Laser_Product;
extern int FirmWare;
extern uint16_t chipID;

struct msm_laser_focus_ctrl_t *get_laura_ctrl(void){
	return laura_t;
}

bool device_state_invalid(void){
	return (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF);
}

void HPTG_DataInit(void){

	Settings[AMBIENT] = DEFAULT_AMBIENT;
	Settings[CONFIDENCE10] = DEFAULT_CONFIDENCE10;
	Settings[CONFIDENCE_THD] = DEFAULT_CONFIDENCE_THD;
	Settings[IT] = DEFAULT_IT;
	Settings[CONFIDENCE_FACTOR] = DEFAULT_CONFIDENCE_FACTOR;
	Settings[DISTANCE_THD] = DEFAULT_DISTANCE_THD;
	Settings[NEAR_LIMIT] = DEFAULT_NEAR_LIMIT;
	Settings[TOF_NORMAL_0] = DEFAULT_TOF_NORMAL_0;
	Settings[TOF_NORMAL_1] = DEFAULT_TOF_NORMAL_1;
	Settings[TOF_NORMAL_2] = DEFAULT_TOF_NORMAL_2;
	Settings[TOF_NORMAL_3] = DEFAULT_TOF_NORMAL_3;
	Settings[TOF_NORMAL_4] = DEFAULT_TOF_NORMAL_4;
	Settings[TOF_NORMAL_5] = DEFAULT_TOF_NORMAL_5;
	Settings[TOF_K10_0] = DEFAULT_TOF_K10_0;
	Settings[TOF_K10_1] = DEFAULT_TOF_K10_1;
	Settings[TOF_K10_2] = DEFAULT_TOF_K10_2;
	Settings[TOF_K10_3] = DEFAULT_TOF_K10_3;
	Settings[TOF_K10_4] = DEFAULT_TOF_K10_4;
	Settings[TOF_K10_5] = DEFAULT_TOF_K10_5;
	Settings[TOF_K40_0] = DEFAULT_TOF_K40_0;
	Settings[TOF_K40_1] = DEFAULT_TOF_K40_1;
	Settings[TOF_K40_2] = DEFAULT_TOF_K40_2;
	Settings[TOF_K40_3] = DEFAULT_TOF_K40_3;
	Settings[TOF_K40_4] = DEFAULT_TOF_K40_4;
	Settings[TOF_K40_5] = DEFAULT_TOF_K40_5;

}

void keep_measure_work(struct work_struct *work){

	int status=0;

	if(timedMeasure){
		do{
			status = Laser_measurement_interface(laura_t, load_calibration_data, &calibration_flag);
			if(status<0){
				printk("keep: repair\n");
				dev_cci_deinit(laura_t);
				dev_cci_init(laura_t);
				Laser_power_up_init_interface(laura_t, DO_CAL, &calibration_flag);
			}

		}while(status<0);

		if(ioctrl_close)
			close_done = true;
	}
	else{
		do{
			if(repairing_state){
				printk("single: repair\n");
				dev_cci_deinit(laura_t);
				dev_cci_init(laura_t);
				Laser_power_up_init_interface(laura_t, DO_CAL, &calibration_flag);
			}
			status = Laser_measurement_interface(laura_t, load_calibration_data, &calibration_flag);
		}while(status<0);
	}
}

int WaitWorkCloseDone(void){

	int cnt=0;

	ioctrl_close =true;
	while(1){
		if(close_done){
			LOG_Handler(LOG_CDBG,"keep_measure_work reply close done\n");
			break;
		}
		else if(cnt++ >500){
			LOG_Handler(LOG_ERR,"keep_measure_work fail replying close done\n");
			break;
		}
		else if(laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF){
			LOG_Handler(LOG_CDBG,"device was not opened\n");
			break;
		}
		msleep(2);
	}
	close_done = false;

	return 0;
}

int Laser_Disable(enum msm_laser_focus_atd_device_trun_on_type val){

		int rc=0;

		if(timedMeasure)
			WaitWorkCloseDone();

		laura_t->device_state = val;
		load_calibration_data=false;

		ErrCode1 = RANGE_ERR_NOT_ADAPT;
		Range1 = OUT_OF_RANGE;

		rc = dev_cci_deinit(laura_t);
		if (rc < 0)
			LOG_Handler(LOG_ERR, "%s Deinit Device fail(%d), rc(%d)\n", __func__, laura_t->device_state, rc);
		else
			LOG_Handler(LOG_CDBG, "%s Deinit Device success(%d)\n", __func__, laura_t->device_state);

		return rc;
}

int Laser_Enable(enum msm_laser_focus_atd_device_trun_on_type val){

		int rc=0;

		//for safety
		if (laura_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF)
			LOG_Handler(LOG_CDBG, "%s device status is not off (%d)\n", __func__, laura_t->device_state);

		dev_cci_init(laura_t);

		msleep(1); //wait for power stable

		if(val == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION)
			rc = Laser_power_up_init_interface(laura_t, DO_CAL, &calibration_flag);
		else
			rc = Laser_power_up_init_interface(laura_t, NO_CAL, &calibration_flag);

		if (rc < 0)
			return rc;

		laura_t->device_state = val;
		load_calibration_data = (val == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION?true:false);
		LOG_Handler(LOG_CDBG, "%s Init Device (%d)\n", __func__, laura_t->device_state);

		if(timedMeasure){
			ioctrl_close =false;
			LOG_Handler(LOG_CDBG,"kick off read interface for keep-measurement mode\n");
			//schedule_delayed_work(&keepMeasure, 0);
			queue_work(Measure_wq, &Measure_wk);
		}

		return rc;
}

void HPTG_Customize(void){

	int inputData[NUMBER_OF_SETTINGS];
	bool keepDefault[NUMBER_OF_SETTINGS];
	int i=0;

	if(Sysfs_read_word_seq("/factory/HPTG_Settings.txt",inputData,NUMBER_OF_SETTINGS)>=0){
		for(i=0; i<NUMBER_OF_SETTINGS; i++){
			if(inputData[i]!=0){
				Settings[i] = inputData[i];
				keepDefault[i] = false;
			}
			else
				keepDefault[i] = true;
		}
		LOG_Handler(LOG_CDBG,"Measure settings in dec, Tof in hex; (1) means default\n");


		LOG_Handler(LOG_CDBG,"Measure settings: %d(%d) %d(%d) %d(%d) %d(%d) %d(%d) %d(%d) %d(%d) \n",
			Settings[AMBIENT], keepDefault[AMBIENT],
			Settings[CONFIDENCE10], keepDefault[CONFIDENCE10],
			Settings[CONFIDENCE_THD], keepDefault[CONFIDENCE_THD],
			Settings[IT], keepDefault[IT],
			Settings[CONFIDENCE_FACTOR], keepDefault[CONFIDENCE_FACTOR],
			Settings[DISTANCE_THD], keepDefault[DISTANCE_THD],
			Settings[NEAR_LIMIT], keepDefault[NEAR_LIMIT]);

		LOG_Handler(LOG_CDBG,"Tof normal: %04x(%d) %04x(%d) %04x(%d) %04x(%d) %04x(%d) %04x(%d) \n",
			Settings[TOF_NORMAL_0], keepDefault[TOF_NORMAL_0],
			Settings[TOF_NORMAL_1], keepDefault[TOF_NORMAL_1],
			Settings[TOF_NORMAL_2], keepDefault[TOF_NORMAL_2],
			Settings[TOF_NORMAL_3], keepDefault[TOF_NORMAL_3],
			Settings[TOF_NORMAL_4], keepDefault[TOF_NORMAL_4],
			Settings[TOF_NORMAL_5], keepDefault[TOF_NORMAL_5]);

		LOG_Handler(LOG_CDBG,"Tof K10: %04x(%d) %04x(%d) %04x(%d) %04x(%d) %04x(%d) %04x(%d) \n",
			Settings[TOF_K10_0], keepDefault[TOF_K10_0],
			Settings[TOF_K10_1], keepDefault[TOF_K10_1],
			Settings[TOF_K10_2], keepDefault[TOF_K10_2],
			Settings[TOF_K10_3], keepDefault[TOF_K10_3],
			Settings[TOF_K10_4], keepDefault[TOF_K10_4],
			Settings[TOF_K10_5], keepDefault[TOF_K10_5]);

		LOG_Handler(LOG_CDBG,"Tof K40: %04x(%d) %04x(%d) %04x(%d) %04x(%d) %04x(%d) %04x(%d) \n",
			Settings[TOF_K40_0], keepDefault[TOF_K40_0],
			Settings[TOF_K40_1], keepDefault[TOF_K40_1],
			Settings[TOF_K40_2], keepDefault[TOF_K40_2],
			Settings[TOF_K40_3], keepDefault[TOF_K40_3],
			Settings[TOF_K40_4], keepDefault[TOF_K40_4],
			Settings[TOF_K40_5], keepDefault[TOF_K40_5]);
	}



}


static ssize_t ATD_Laser_enable_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, rc = 0;
	char messages[8]="";

	LOG_Handler(LOG_DBG, "%s: Enter\n", __func__);

	len =(len > 8 ?8:len);
	if (copy_from_user(messages, buff, len)) {
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);

	if(laura_t->device_state == val){
		LOG_Handler(LOG_ERR, "%s device state same as command(%d)\n", __func__,val);
		return len;
	}

	mutex_ctrl(laura_t, MUTEX_LOCK);
	switch (val) {
		case MSM_LASER_FOCUS_DEVICE_OFF:
			LOG_Handler(LOG_CDBG,"%s: client leave via proc\n", __func__);

			rc = Laser_Disable(val);
			PowerDown(laura_t);
			if (rc < 0)
				goto DEVICE_SWITCH_ERROR;

			break;

		case MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION:
		case MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION:
			HPTG_DataInit();
			HPTG_Customize();
			LOG_Handler(LOG_CDBG,"%s: client enter via proc, type (%d)\n", __func__, val);

			if(laura_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF){
				Laser_Disable(val);
				PowerDown(laura_t);
			}
			PowerUp(laura_t);
			rc = Laser_Enable(val);
			if (rc < 0)
				goto DEVICE_SWITCH_ERROR;
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command invalid\n", __func__);
			break;
	}

DEVICE_SWITCH_ERROR:

	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	if(rc<0 && val != MSM_LASER_FOCUS_DEVICE_OFF)
		dev_cci_deinit(laura_t);

	if(rc<0)
		LOG_Handler(LOG_ERR, "%s: command is not done due to device switching fail\n", __func__);
	else
		LOG_Handler(LOG_DBG, "%s: command (%d) done\n",__func__,val);

	return len;
}

static int ATD_Laser_enable_read(struct seq_file *buf, void *v){
	seq_printf(buf, "%d\n", laura_t->device_state);
	return 0;
}

static int ATD_Laser_enable_open(struct inode *inode, struct  file *file){
	return single_open(file, ATD_Laser_enable_read, NULL);
}

const struct file_operations ATD_laser_focus_device_enable_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laser_enable_open,
	.write = ATD_Laser_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


int HEPT_ReadRangeByProc(struct seq_file * buf, bool for_DIT){

	int Range = 0, time=0;
	struct timeval start,now;
	O_get_current_time(&start);

	mutex_ctrl(laura_t, MUTEX_LOCK);
	if(!for_DIT)
		proc_log_cnt++;

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF){
		LOG_Handler(LOG_ERR, "Device without turn on\n");
		seq_printf(buf, "%d\n", 0);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return -EBUSY;
	}

	if(!timedMeasure)
		Range = Laser_measurement_interface(laura_t, load_calibration_data, &calibration_flag);
	else
		Range = Range1;	//need more condition

	if (Range >= OUT_OF_RANGE) {
		LOG_Handler(LOG_DBG, "%s: OUT_OF_RANGE(%d), errCode(%d)\n", __func__, Range,ErrCode1);
		Range = OUT_OF_RANGE;
	}

	if(for_DIT)
		seq_printf(buf, "%d#%d#%d\n", Range, HEPT_DMAX, ErrCode1);
	else
		seq_printf(buf, "%d\n", Range);

	LOG_Handler(LOG_DBG, "%s : Get range (%d)  Device (%d)\n", __func__, Range , laura_t->device_state);

	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	O_get_current_time(&now);
	DeltaTime_ms(start, now, &time);

	if(!for_DIT)
	LOG_Handler(LOG_CDBG,"time consumption of reading range data: %ld\n", time);


	return 0;


}
static int ATD_Laura_device_get_range_read(struct seq_file *buf, void *v)
{
	return HEPT_ReadRangeByProc(buf, 0);
}

static int ATD_Laura_device_get_range_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_Laura_device_get_range_read, NULL);
}

const struct file_operations ATD_laser_focus_device_get_range_fos = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_get_range_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_device_get_range_read_more_info(struct seq_file *buf, void *v)
{
	return HEPT_ReadRangeByProc(buf, 1);
}

static int ATD_Laura_device_get_range_more_info_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_Laura_device_get_range_read_more_info, NULL);
}

const struct file_operations ATD_laser_focus_device_get_range_more_info_fos = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_get_range_more_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int dummy_read(struct seq_file *buf, void *v)
{
	return 0;
}

static int dummy_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dummy_read, NULL);
}

static ssize_t Laser_calibration_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, ret = 0;
	char messages[8]="";
	LOG_Handler(LOG_CDBG, "%s: Enter\n", __func__);

	if (device_state_invalid()){
		LOG_Handler(LOG_ERR, "%s: Device without turning on: \n", __func__);
		return -EBUSY;
	}
	len = (len>8?8:len);
	if (copy_from_user(messages, buff, len)){
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	LOG_Handler(LOG_DBG, "%s command : %d\n", __func__, val);

	switch (val) {
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Laser_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			break;
		default:
			LOG_Handler(LOG_ERR, "%s command fail(%d) !!\n", __func__, val);
			break;
	}

	LOG_Handler(LOG_CDBG, "%s: Exit, rc=%d\n", __func__,ret);
	return len;
}


const struct file_operations Laser_calibration_fops = {
	.owner = THIS_MODULE,
	.open = dummy_open,
	.write = Laser_calibration_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int Laser_get_raw_Kdata_proc_open(struct inode *inode, struct  file *file)
{

	return single_open(file, Laser_get_raw_Kdata_interface, NULL);
}

const struct file_operations Laser_get_raw_Kdata_fops = {
	.owner = THIS_MODULE,
	.open = Laser_get_raw_Kdata_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	int result = 0;
	mutex_ctrl(laura_t, MUTEX_LOCK);
	ATD_status = HPTG_I2C_status_check(laura_t, MSM_CAMERA_I2C_WORD_DATA);
	//temp
	//keepMeasuring = !keepMeasuring;
	if (ATD_status) {
		result = 1;
	} else{
		result = 0;
	}
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	//seq_printf(buf, "%d\n", ATD_status);

	return seq_printf(buf, "%d\n", result);
}

static int ATD_Laura_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_Laura_I2C_status_check_proc_read, NULL);
}

static const struct file_operations ATD_I2C_status_check_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_I2C_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int Laser_status_check_proc_read(struct seq_file *buf, void *v)
{

	mutex_ctrl(laura_t, MUTEX_LOCK);
	i2c_status = HPTG_I2C_status_check(laura_t, MSM_CAMERA_I2C_WORD_DATA);
	LOG_Handler(LOG_CDBG, "HW id: 0x%x expected id 0x%x\n",chipID, laura_t->sensordata->slave_info->sensor_id);
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	seq_printf(buf, "%d\n", i2c_status?(I2C_STATUS_PASS):(I2C_STATUS_FAIL));

	return 0;
}

static int Laser_status_check_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, Laser_status_check_proc_read, NULL);
}

const struct file_operations Laser_status_check_fops = {
	.owner = THIS_MODULE,
	.open = Laser_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int Laser_check_producer_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", i2c_status);
	return 0;
}

static int Laser_check_producer_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, Laser_check_producer_proc_read, NULL);
}

const struct file_operations Laser_check_producer_fops = {
	.owner = THIS_MODULE,
	.open = Laser_check_producer_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_register_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_laura_register_read, NULL);
}

const struct file_operations dump_laser_focus_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int dump_debug_register_read(struct seq_file *buf, void *v)
{
	debug_dump(buf, v);
	return 0;
}

int dump_laser_focus_debug_register_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_debug_register_read, NULL);
}

const struct file_operations dump_HPTG_debug_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_laser_focus_debug_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


/*
//ATD will tell how to impliment it
static int dump_Laura_value_check_read(struct seq_file *buf, void *v)
{
	timedMeasure = !timedMeasure;
	printk("timedMeasure changed\n");
	seq_printf(buf,"PASS\n");
       return 0;
}

static int dump_Laura_laser_focus_value_check_open(struct inode *inode, struct  file *file)
{
        return single_open(file, dump_Laura_value_check_read, NULL);
}

static const struct file_operations dump_laser_focus_value_check_fops = {
        .owner = THIS_MODULE,
        .open = dump_Laura_laser_focus_value_check_open,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

*/
/*----------CE Debug----------*/


int Laura_laser_focus_set_K_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_DBG, "%s: Enter and Exit, calibration: %d\n", __func__, calibration_flag);
	seq_printf(buf,"%d",calibration_flag);
	return 0;
}
int Laura_laser_focus_set_K_open(struct inode *inode, struct file *file)
{
	//LOG_Handler(LOG_FUN, "%s: Enter and Exit\n", __func__);
	return single_open(file, Laura_laser_focus_set_K_read, NULL);
}

ssize_t Laura_laser_focus_set_K_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;
	int val;
	char messages[8]="";
	LOG_Handler(LOG_CDBG, "%s: Enter\n", __func__);

	len = (len>8?8:len);
	if (copy_from_user(messages, buff, len)){
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);

	switch(val){
		case 0:
			calibration_flag = false;
			break;
		case 1:
			calibration_flag = true;
			break;
		default:
			LOG_Handler(LOG_DBG, "command '%d' is not valid\n", val);
	}

	LOG_Handler(LOG_CDBG, "%s: Exit, calibration: %d\n", __func__, calibration_flag);
	return rc;
}

const struct file_operations laser_focus_set_K_fops = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_set_K_open,
	.write = Laura_laser_focus_set_K_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


int Laura_laser_focus_product_family_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_DBG, "%s: Enter and Exit, Product Family: %d\n", __func__, Laser_Product);
	seq_printf(buf,"%d",Laser_Product);
	return 0;
}
int Laura_laser_focus_product_family_open(struct inode *inode, struct file *file)
{
	//LOG_Handler(LOG_FUN, "%s: Enter and Exit\n", __func__);
	return single_open(file, Laura_laser_focus_product_family_read, NULL);
}

const struct file_operations laser_focus_product_family = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_product_family_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


#define MODULE_NAME "LaserSensor"
#define ASUS_LASER_NAME_SIZE	32
#define ASUS_LASER_DATA_SIZE	4
#define OLIVIA_IOC_MAGIC                      ('W')
#define ASUS_LASER_SENSOR_MEASURE     _IOR(OLIVIA_IOC_MAGIC  , 0, unsigned int[ASUS_LASER_DATA_SIZE])
#define ASUS_LASER_SENSOR_GET_NAME	_IOR(OLIVIA_IOC_MAGIC  , 4, char[ASUS_LASER_NAME_SIZE])

int Olivia_get_measurement(int* distance){
	int RawRange = 0;

	if(repairing_state){
		Range1 = OUT_OF_RANGE;
		ErrCode1 = RANGE_ERR_NOT_ADAPT;
		*distance = Range1;
		return 0;
	}
	mutex_ctrl(laura_t, MUTEX_LOCK);

	if(device_state_invalid()){
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return -EBUSY;
	}

	if(!timedMeasure){
		RawRange = Laser_measurement_interface(laura_t, load_calibration_data, &calibration_flag);
		if(RawRange<0)
			schedule_delayed_work(&keepMeasure, 0);
	}
	else
		RawRange = Range1;

	if(RawRange < 0){
		LOG_Handler(LOG_ERR, "%s: Read_range(%d) failed\n", __func__, RawRange);
		RawRange = 0;
	}
	*distance = RawRange;

	mutex_ctrl(laura_t, MUTEX_UNLOCK);
	return 0;


}

static int Olivia_misc_open(struct inode *inode, struct file *file){

	int rc = 0, cnt = 0;

	mutex_ctrl(laura_t, MUTEX_LOCK);
	timedMeasure = true;
	HPTG_DataInit();
	HPTG_Customize();
	client++;
	LOG_Handler(LOG_CDBG,"%s: client enter via ioctrl(%d)\n", __func__, client);

	if(client == 1){
		PowerUp(laura_t);
		if(chipID == CHIP_ID_PREVIOUS)
			rc = Laser_Enable(MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION);
		else
			rc = Laser_Enable(MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION);
	}
	while(rc < 0 && (cnt++ < 3)){
		dev_cci_deinit(laura_t);
		rc = Laser_Enable(MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION);
		LOG_Handler(LOG_ERR,"%s: retry laser enable, rc(%d), retry(%d)\n", __func__, rc, cnt);
	}
	if(rc < 0){
		client--;
		LOG_Handler(LOG_ERR,"%s: retry open fail, client(%d)\n", __func__, client);
	}
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	return 0;
}

static int Olivia_misc_release(struct inode *inode, struct file *file)
{
	int rc = 0, cnt = 0;
	mutex_ctrl(laura_t, MUTEX_LOCK);

	client--;
	LOG_Handler(LOG_CDBG,"%s: client leave via ioctrl(%d)\n", __func__, client);

	if(client ==0){
		rc = Laser_Disable(MSM_LASER_FOCUS_DEVICE_OFF);
		while(rc < 0 && (cnt++ < 3)){
			dev_cci_init(laura_t);
			rc = Laser_Disable(MSM_LASER_FOCUS_DEVICE_OFF);
			LOG_Handler(LOG_ERR,"%s: retry laser deinit, rc(%d), retry(%d)\n", __func__, rc, cnt);
		}
		if(rc < 0)
			LOG_Handler(LOG_ERR,"%s: retry release fail\n", __func__);

		PowerDown(laura_t);
	}
	else if(client < 0){
		client =0;
		LOG_Handler(LOG_CDBG,"%s: dummy leave, reset client to %d\n", __func__, client);
	}
	timedMeasure = false;
	mutex_ctrl(laura_t, MUTEX_UNLOCK);
	return 0;
}

static long Olivia_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

 	unsigned int dist[4] = {0,0,0,0};
	char name[ASUS_LASER_NAME_SIZE];
	int distance=Range1;
	int ret = 0, time=0;
	//static int  cnt=0;
	struct timeval start,now;

	Laser_log_cnt++;
	if(Laser_log_cnt >= LOG_SAMPLE_RATE)
		O_get_current_time(&start);

	if (_IOC_TYPE(cmd) != OLIVIA_IOC_MAGIC)
		return 0;

	switch (cmd) {
		case ASUS_LASER_SENSOR_MEASURE:
			Olivia_get_measurement(&distance);
			dist[0] = distance;
			dist[1] = ErrCode1;
			dist[2] = HEPT_DMAX;
			dist[3] = calibration_flag;
			ret = copy_to_user((int __user*)arg, dist, sizeof(dist));
			break;

		case ASUS_LASER_SENSOR_GET_NAME:
			snprintf(name, ASUS_LASER_NAME_SIZE, MODULE_NAME);
			ret = copy_to_user((int __user*)arg, &name, sizeof(name));
			break;

		default:
			LOG_Handler(LOG_ERR, "%s: ioctrl command is not valid  commod  %d \n", __func__, cmd);
	}

	if(Laser_log_cnt>=LOG_SAMPLE_RATE){
		O_get_current_time(&now);
		DeltaTime_ms(start, now, &time);
		LOG_Handler(LOG_CDBG,"time consumption of reading range data: %ld\t [%d,%d]\n", time, distance, ErrCode1);
		Laser_log_cnt=0;
	}

	return 0;
}

static struct file_operations Olivia_fops = {
  .owner = THIS_MODULE,
  .open = Olivia_misc_open,
  .release = Olivia_misc_release,
  .unlocked_ioctl = Olivia_misc_ioctl,
  .compat_ioctl = Olivia_misc_ioctl
};

struct miscdevice Olivia_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = MODULE_NAME,
  .fops = &Olivia_fops
};

static int register_ioctrl(int Product)
{
	int rtn = 0;

	if(Product != PRODUCT_OLIVIA){
		LOG_Handler(LOG_CDBG, "LaserFocus is not supported, ProductFamily is not Olivia (%d)",Product);
		return rtn;
	}

	rtn = misc_register(&Olivia_misc);
	if (rtn < 0) {
		LOG_Handler(LOG_ERR,"Unable to register misc devices\n");
		misc_deregister(&Olivia_misc);
	}
	return rtn;
}



#define proc(file,mod,fop)	\
	LOG_Handler(LOG_DBG,"proc %s %s\n",file,proc_create(file,mod,NULL,fop)? "success":"fail");
#if 0
static void Olivia_Create_proc(void)
{
	proc(STATUS_PROC_FILE, 0664, &ATD_I2C_status_check_fops);
	proc(STATUS_PROC_FILE, 0664, &Laser_status_check_fops);
	proc(STATUS_PROC_FILE_FOR_CAMERA, 0664, &Laser_check_producer_fops);
	proc(DEVICE_TURN_ON_FILE, 0664, &ATD_laser_focus_device_enable_fops);
	proc(DEVICE_GET_VALUE, 0664, &ATD_laser_focus_device_get_range_fos);
	proc(DEVICE_GET_VALUE_MORE_INFO, 0664, &ATD_laser_focus_device_get_range_more_info_fos);


	proc(DEVICE_SET_CALIBRATION, 0664, &Laser_calibration_fops);
	proc(DEVICE_GET_CALIBRATION_INPUT_DATA, 0664, &Laser_get_raw_Kdata_fops);



	//for debug
	proc(DEVICE_DUMP_REGISTER_VALUE, 0664, &dump_laser_focus_register_fops);
	proc(DEVICE_DUMP_DEBUG_VALUE, 0664, &dump_HPTG_debug_register_fops);
	//proc(DEVICE_LOG_CTRL_FILE, 0664, &laser_focus_log_contorl_fops);
	//for debug

/*
	//for ce
	proc(DEVICE_DEBUG_VALUE1, 0664, &dump_debug_Kvalue1_fops);
	proc(DEVICE_DEBUG_VALUE2, 0664, &dump_debug_Kvalue2_fops);
	proc(DEVICE_DEBUG_VALUE3, 0664, &dump_debug_Kvalue3_fops);
	proc(DEVICE_DEBUG_VALUE4, 0664, &dump_debug_Kvalue4_fops);
	proc(DEVICE_DEBUG_VALUE5, 0664, &dump_debug_Kvalue5_fops);
	proc(DEVICE_DEBUG_VALUE6, 0664, &dump_debug_Kvalue6_fops);
	proc(DEVICE_DEBUG_VALUE7, 0664, &dump_debug_Kvalue7_fops);
	proc(DEVICE_DEBUG_VALUE8, 0664, &dump_debug_Kvalue8_fops);
	proc(DEVICE_DEBUG_VALUE9, 0664, &dump_debug_Kvalue9_fops);
	proc(DEVICE_VALUE_CHECK, 0664, &dump_laser_focus_value_check_fops);
	//for ce

	//for dit
	proc(DEVICE_IOCTL_SET_K, 0664, &laser_focus_set_K_fops);
	proc(DEVICE_IOCTL_PRODUCT_FAMILY, 0444, &laser_focus_product_family);
	//for dit
*/
}

#endif

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
	.i2c_poll =  msm_camera_qup_i2c_poll,
};

static const struct v4l2_subdev_internal_ops msm_laser_focus_internal_ops;

static struct v4l2_subdev_core_ops msm_laser_focus_subdev_core_ops = {
	.ioctl = NULL,
};

static struct v4l2_subdev_ops msm_laser_focus_subdev_ops = {
	.core = &msm_laser_focus_subdev_core_ops,
};

static struct msm_camera_i2c_client msm_laser_focus_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};
static const struct of_device_id msm_laser_focus_dt_match[] = {
	{.compatible = LASER_NAME, .data = NULL},
	{}
};
static const struct i2c_device_id laser_i2c_id[] = {
	{ LASER_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(of, msm_laser_focus_dt_match);


static int match_Olivia(struct platform_device *pdev){

	const struct of_device_id *match;

	match = of_match_device(msm_laser_focus_dt_match, &pdev->dev);
	if (!match) {
		pr_err("device not match\n");
		return -EFAULT;
	}

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	laura_t = kzalloc(sizeof(struct msm_laser_focus_ctrl_t),GFP_KERNEL);
	if (!laura_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}


	/* Set platform device handle */
	laura_t->pdev = pdev;

	return 0;
}

static int Init_Chip_Status(struct msm_laser_focus_ctrl_t *dev_t){

	int rc = 0;
	PowerUp(dev_t);
	dev_cci_init(laura_t);

	rc = Laser_power_up_init_interface(laura_t, NO_CAL, &calibration_flag);
	if (rc < 0)
		LOG_Handler(LOG_ERR, "%s Device init fail !! rc(%d)\n", __func__, rc);

	dev_cci_deinit(laura_t);
	PowerDown(dev_t);
	LOG_Handler(LOG_CDBG, "%s: done\n", __func__);

	return rc;
}

static int msm_laser_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc;
	LOG_Handler(LOG_CDBG, "%s: Probe Start\n", __func__);
	if (!id) {
		pr_err("%s: id is NULL\n", __func__);
		id = laser_i2c_id;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c_check_functionality failed\n", __func__);
		goto probe_failure;
	}

	if (!client->dev.of_node) {
		pr_err("%s: of_node NULL\n", __func__);
		return -EINVAL;
	}

	laura_t = kzalloc(sizeof(struct msm_laser_focus_ctrl_t),GFP_KERNEL);
	if (!laura_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	rc = get_dtsi_data(client->dev.of_node, laura_t);
	if (rc < 0) {
		pr_err("%s get_dtsi_data failed line %d\n", __func__, __LINE__);
		return rc;
	}

	/* Assign name for sub device */
	snprintf(laura_t->msm_sd.sd.name, sizeof(laura_t->msm_sd.sd.name),
			"%s", laura_t->sensordata->sensor_name);

	laura_t->act_v4l2_subdev_ops = &msm_laser_focus_subdev_ops;

	/* Set device type as platform device */
	laura_t->act_device_type = MSM_CAMERA_I2C_DEVICE;
	laura_t->i2c_client = &msm_laser_focus_i2c_client;
	laura_t->i2c_client->client = client;
	if (NULL == laura_t->i2c_client) {
		pr_err("%s i2c_client NULL\n", __func__);
		return -EFAULT;
	}
	if (laura_t->sensordata->slave_info->sensor_slave_addr)
		laura_t->i2c_client->client->addr =
			laura_t->sensordata->slave_info->sensor_slave_addr;
	pr_info("laura_t->i2c_client->client->addr=0x%x\n", laura_t->i2c_client->client->addr);

	if (!laura_t->i2c_client->i2c_func_tbl)
		laura_t->i2c_client->i2c_func_tbl = &msm_sensor_qup_func_tbl;

	set_subdev(laura_t, &msm_laser_focus_internal_ops);
	set_laser_state(laura_t);

	HPTG_DataInit();

	/* Check I2C status */
	i2c_status = HPTG_I2C_status_check(laura_t, MSM_CAMERA_I2C_WORD_DATA);
	if(i2c_status ==I2C_STATUS_FAIL)
		goto probe_failure;

	/* Init mutex */
       mutex_ctrl(laura_t, MUTEX_ALLOCATE);
	mutex_ctrl(laura_t, MUTEX_INIT);

	Init_Chip_Status(laura_t);

	//loose criteria
	if(Laser_Product !=PRODUCT_OLIVIA){
		Laser_Product = PRODUCT_OLIVIA;
		LOG_Handler(LOG_CDBG, "%s: WARNING: module ID is not Olivia\n", __func__);
	}

	rc = register_ioctrl(Laser_Product);
	if (rc < 0)
		goto probe_failure;

	HEPTAGON_create_proc_file();
	//Olivia_Create_proc();

	//INIT_DELAYED_WORK(&keepMeasure, keep_measure_work);
	Measure_wq = create_singlethread_workqueue("Laser_wq");

	INIT_WORK(&Measure_wk, keep_measure_work);

	g_factory = g_ftm_mode_HEP;
	if(g_factory){
		//Enable_DBG();
		timedMeasure = false;
	}
	LOG_Handler(LOG_CDBG, "%s: timedMeasure(%d), factory_mode(%d)\n", __func__,timedMeasure, g_factory);

	LOG_Handler(LOG_CDBG, "%s: Probe Success\n", __func__);
	return 0;

probe_failure:
	LOG_Handler(LOG_CDBG, "%s: Probe failed, rc = %d\n", __func__, rc);
	return rc;
}

static void __exit msm_laser_i2c_remove(void)
{
	i2c_del_driver(&olivia_i2c_driver);
	return;
}

static struct i2c_driver olivia_i2c_driver = {
	.id_table = laser_i2c_id,
	.probe  = msm_laser_i2c_probe,
	.remove = __exit_p(msm_laser_i2c_remove),
	.driver = {
		.name = LASER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = msm_laser_focus_dt_match,
	},
};

static int32_t Olivia_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	LOG_Handler(LOG_CDBG, "%s: Probe Start\n", __func__);

	rc = match_Olivia(pdev);
	if(rc < 0)
	{
		pr_err("%s:%d [LASER] match_Olivia faill\n ",__func__,__LINE__);
		goto probe_failure;
	}

	rc = get_dtsi_data(pdev->dev.of_node, laura_t);
	if (rc < 0)
	{
		pr_err("%s:%d [LASER] get_dtsi_data faill\n ",__func__,__LINE__);
		goto probe_failure;
	}

	rc = set_i2c_client(laura_t, &msm_laser_focus_i2c_client);
	if (rc < 0)
	{
		pr_err("%s:%d [LASER] set_i2c_client faill\n ",__func__,__LINE__);
		goto probe_failure;
	}

	set_cci_client(laura_t);
	set_subdev(laura_t, &msm_laser_focus_internal_ops);
	set_laser_state(laura_t);

	HPTG_DataInit();

	/* Check I2C status */
	i2c_status = HPTG_I2C_status_check(laura_t, MSM_CAMERA_I2C_WORD_DATA);
	if(i2c_status ==I2C_STATUS_FAIL)
		goto probe_failure;

	/* Init mutex */
       mutex_ctrl(laura_t, MUTEX_ALLOCATE);
	mutex_ctrl(laura_t, MUTEX_INIT);

	Init_Chip_Status(laura_t);

	//loose criteria
	if(Laser_Product !=PRODUCT_OLIVIA){
		Laser_Product = PRODUCT_OLIVIA;
		LOG_Handler(LOG_CDBG, "%s: WARNING: module ID is not Olivia\n", __func__);
	}

	rc = register_ioctrl(Laser_Product);
	if (rc < 0)
		goto probe_failure;

	HEPTAGON_create_proc_file();
	//Olivia_Create_proc();

	//INIT_DELAYED_WORK(&keepMeasure, keep_measure_work);
	Measure_wq = create_singlethread_workqueue("Laser_wq");

	INIT_WORK(&Measure_wk, keep_measure_work);

	g_factory = g_ftm_mode_HEP;
	if(g_factory){
		//Enable_DBG();
		timedMeasure = false;
	}
	LOG_Handler(LOG_CDBG, "%s: timedMeasure(%d), factory_mode(%d)\n", __func__,timedMeasure, g_factory);

	LOG_Handler(LOG_CDBG, "%s: Probe Success\n", __func__);
	return 0;

probe_failure:
	LOG_Handler(LOG_CDBG, "%s: Probe failed, rc = %d\n", __func__, rc);
	return rc;
}


static struct platform_driver msm_laser_focus_platform_driver = {
	.driver = {
		.name = LASER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = msm_laser_focus_dt_match,
	},
};

static int __init Laura_init_module(void)
{
	int32_t rc = 0;

	LOG_Handler(LOG_DBG, "%s:Enter\n", __func__);
	//pr_info("Enter hepta , g_ASUS_hwID %d \n",g_ASUS_hwID);

	rc = platform_driver_probe(&msm_laser_focus_platform_driver, Olivia_platform_probe);
	if (rc == 0) {
		pr_info("laser platform_driver_register success\n");
		return rc;
	} else if (rc != 0) {
		pr_err("laser platform_driver_register failed\n");
		rc = i2c_add_driver(&olivia_i2c_driver);
		if (!rc)
			pr_err("laser i2c_add_driver success\n");
	}
	LOG_Handler(LOG_DBG, "%s rc %d\n", __func__, rc);

	return rc;
}
static void __exit Laura_driver_exit(void)
{
	platform_driver_unregister(&msm_laser_focus_platform_driver);
	return;
}

#if 0

static int set_i2c_client(struct platform_device *pdev){

	/* Assign name for sub device */
	snprintf(laura_t->msm_sd.sd.name, sizeof(laura_t->msm_sd.sd.name),
			"%s", laura_t->sensordata->sensor_name);

	laura_t->act_v4l2_subdev_ops = &msm_laser_focus_subdev_ops;

	/* Set device type as platform device */
	laura_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	laura_t->i2c_client = &msm_laser_focus_i2c_client;
	if (NULL == laura_t->i2c_client) {
		pr_err("%s i2c_client NULL\n",
			__func__);
		return -EFAULT;

	}
	if (!laura_t->i2c_client->i2c_func_tbl)
		laura_t->i2c_client->i2c_func_tbl = &msm_sensor_cci_func_tbl;

	laura_t->i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!laura_t->i2c_client->cci_client) {
		kfree(laura_t->vreg_cfg.cam_vreg);
		kfree(laura_t);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	return 0;
}

static void set_cci_client(struct platform_device *pdev){

	struct msm_camera_cci_client *cci_client = NULL;

	cci_client = laura_t->i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = laura_t->cci_master;
	if (laura_t->sensordata->slave_info->sensor_slave_addr)
		cci_client->sid = laura_t->sensordata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;

}

static void set_subdev(struct platform_device *pdev){

	v4l2_subdev_init(&laura_t->msm_sd.sd, laura_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&laura_t->msm_sd.sd, laura_t);

	laura_t->msm_sd.sd.internal_ops = &msm_laser_focus_internal_ops;
	laura_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(laura_t->msm_sd.sd.name,
		ARRAY_SIZE(laura_t->msm_sd.sd.name), "msm_laser_focus");

	media_entity_init(&laura_t->msm_sd.sd.entity, 0, NULL, 0);
	laura_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	//laura_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_LASER_FOCUS;
	laura_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&laura_t->msm_sd);

}

static void set_laser_config(struct platform_device *pdev){

	/* Init data struct */
	laura_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	laura_t->laser_focus_cross_talk_offset_value = 0;
	laura_t->laser_focus_offset_value = 0;
	laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;

}

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};

static struct v4l2_subdev_core_ops msm_laser_focus_subdev_core_ops = {
	.ioctl = NULL,
};

static struct v4l2_subdev_ops msm_laser_focus_subdev_ops = {
	.core = &msm_laser_focus_subdev_core_ops,
};

#endif
module_init(Laura_init_module);
module_exit(Laura_driver_exit);
MODULE_DESCRIPTION("MSM LASER_FOCUS");
MODULE_LICENSE("GPL v2");
