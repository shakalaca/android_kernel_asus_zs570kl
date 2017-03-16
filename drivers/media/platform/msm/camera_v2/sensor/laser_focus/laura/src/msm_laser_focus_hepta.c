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
#include "show_log.h"
#include "laura_debug.h"
#include "laura_interface.h"
#include "laura_factory_func.h"
#include "laura_shipping_func.h"
#include "laser_focus_hepta.h"
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/asus_project.h>

#define DO_CAL true
#define NO_CAL false
#define DO_MEASURE true
#define NO_MEASURE false

#define LASER_SENSOR_NAME	"olivia"

struct regulator *reg_vdd;
struct regulator *reg_vdd_i2c;

static int DMax = 400;
int ErrCode1 = 0;
uint16_t Range1 = 0;

/*
continuous measuring mode, make sure ioctrl_close base on keepMeasuring is true
*/
//extern int g_ftm_mode;
bool keepMeasuring = false;
bool ioctrl_close = true;
bool close_done = false;
struct workqueue_struct*	Measure_wq;
struct work_struct		Measure_wk;

bool repairing_state = false;

struct msm_laser_focus_ctrl_t *laura_t = NULL;

static bool camera_on_flag = false;

static bool calibration_flag = true;

bool CSC_mode_flag = false;

static int laser_focus_enforce_ctrl = 0;

static bool load_calibration_data = false;

static int ATD_status;

static int client=0;

extern int Laser_Product;
extern int FirmWare;
extern uint16_t module_id[34];
extern uint16_t module_verion[2];
extern uint16_t chipID;

extern int MIN_CONF;
extern int MAX_DISTANCE;
extern int CONF_A;
extern int ITB;
extern int CONF_C;
extern int Ambient_value;
static bool gotVarietydata = false;
extern uint16_t config_normal_olivia[];
extern uint16_t config_cal_10_olivia[];
extern uint16_t config_cal_40_olivia[];

struct msm_laser_focus_ctrl_t *get_laura_ctrl(void){
	return laura_t;
}

bool OLI_device_invalid(void){
	return (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
			laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI);
}	

#if 0
void keep_measure_work(struct work_struct *work){

	int status=0;

	if(keepMeasuring){	
		do{
			//LOG_Handler(LOG_CDBG,"rekick off read interface for keep-measurement mode\n");
			status = Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag);   

			//seems need not wait standby
			//WaitMCPUStandby(laura_t);

			if(status<0){
		printk("keep: repair\n");
		dev_deinit(laura_t);
		power_down(laura_t);
		power_up(laura_t);
		dev_init(laura_t);
		Laura_device_power_up_init_interface(laura_t, DO_CAL, &calibration_flag);

		}

	}while(status<0);

	//repairing_state = false;
		if(ioctrl_close)
			close_done = true;
	}
	else{

	do{
	if(repairing_state){
		printk("single: repair\n");
		dev_deinit(laura_t);
		power_down(laura_t);
		power_up(laura_t);
		dev_init(laura_t);
		Laura_device_power_up_init_interface(laura_t, DO_CAL, &calibration_flag);
		//repairing_state = false;
	}

	status = Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag);   
	}while(status<0);



	}
}
#else
void keep_measure_work(struct work_struct *work){

	#define CONTINOUS_MEASURE_TIMEOUT_MS	90
	int status=0;
	struct timeval start,now;
	O_get_current_time(&start);

	ErrCode1 = 0;
	Range1 = 0;

	printk("[LASER_FOCUS] work queue start\n");

	while(keepMeasuring) {
		O_get_current_time(&now);
		if(is_timeout(start,now,CONTINOUS_MEASURE_TIMEOUT_MS) ) {
			O_get_current_time(&start);
			mutex_ctrl(laura_t, MUTEX_LOCK);
			if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_INIT_CCI) {
				status = Olivia_device_read_range(laura_t);
				if(status<0){
					printk("[LASER_FOCUS] continuous measure error: %d\n", status);
					(void) Laura_device_power_up_init_interface(laura_t, DO_CAL, &calibration_flag);
					(void) WaitMCPUOn(laura_t);
					(void) control_signal(laura_t);
				}
			}
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
		}
		msleep(10);
	}

	printk("[LASER_FOCUS] work queue release\n");
}
#endif

int enable_i2c_pull_up_power(void) {
        return regulator_enable(reg_vdd_i2c);
}

int disable_i2c_pull_up_power(void) {
        return regulator_disable(reg_vdd_i2c);
}

int enable_laser_power(void) {
        return regulator_enable(reg_vdd);
}

int disable_laser_power(void) {
        return regulator_disable(reg_vdd);
}

void Load_variety_data(void) {

	#define SIZE_OF_OLIVIA_VARIETY_DATA_SIZE	6
	#define SIZE_OF_OLIVIA_TOF_CONFIG_SIZE		18
	uint16_t data[SIZE_OF_OLIVIA_VARIETY_DATA_SIZE] = {0};
	uint16_t data1[SIZE_OF_OLIVIA_TOF_CONFIG_SIZE] = {0};
	int rc;
	int i;

	if (!gotVarietydata || CSC_mode_flag) {
		rc = Olivia_Read_variety_Data_From_File(data, SIZE_OF_OLIVIA_VARIETY_DATA_SIZE);
		if (rc == 0) {
			MIN_CONF = data[0];
			MAX_DISTANCE = data[1];
			CONF_A = data[2];
			ITB = data[3];
			CONF_C = data[4];
			Ambient_value = data[5];
		}
		rc = Olivia_Read_TOF_Config_From_File(data1, SIZE_OF_OLIVIA_TOF_CONFIG_SIZE);
		if (rc == 0) {
			for (i=0; i<6; i++) {
				config_normal_olivia[i] = data1[i];
				config_cal_10_olivia[i] = data1[i+6];
				config_cal_40_olivia[i] = data1[i+12];
			}
		}
		gotVarietydata = true;
	}

	LOG_Handler(LOG_DBG, "MIN_CONF:(%d) MAX_DISTANCE:(%d), CONF_A:(%d), ITB:(%d), CONF_C:(%d), Ambient_value:(%d)\n ",
				MIN_CONF, MAX_DISTANCE, CONF_A, ITB, CONF_C, Ambient_value);


	for (i=0; i<6; i++) {
		LOG_Handler(LOG_DBG, "config_normal_olivia[%d]:0x%04x\n", i, config_normal_olivia[i]);

	}
	for (i=0; i<6; i++) {
		LOG_Handler(LOG_DBG, "config_cal_10_olivia[%d]:0x%04x\n", i, config_cal_10_olivia[i]);

	}
	for (i=0; i<6; i++) {
		LOG_Handler(LOG_DBG, "config_cal_40_olivia[%d]:0x%04x\n", i, config_cal_40_olivia[i]);

	}
}

int Laser_Disable(enum msm_laser_focus_atd_device_trun_on_type val){

		int rc=0;
	
		mutex_ctrl(laura_t, MUTEX_LOCK);
		if(camera_on_flag){
			LOG_Handler(LOG_DBG, "%s: Camera is running, do nothing!!\n ", __func__);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			return 9999;
		}

		#if 0
		if(keepMeasuring){
			ioctrl_close =true;
			while(1){				
				if(close_done){
					LOG_Handler(LOG_CDBG,"keep_measure_work reply close done\n");
					break;
				}
				else if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
					 laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
					LOG_Handler(LOG_CDBG,"device was not opened\n");
					break;
				}
				msleep(1);
			}
			close_done = false;
		}
		#endif
		
		laura_t->device_state = val;
		load_calibration_data=false;
		gotVarietydata = false;
		
		rc = dev_deinit(laura_t);
		if (rc < 0)
			LOG_Handler(LOG_ERR, "%s Deinit Device fail(%d), rc(%d)\n", __func__, laura_t->device_state, rc);
		else
			LOG_Handler(LOG_CDBG, "%s Deinit Device success(%d)\n", __func__, laura_t->device_state);
	
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		//LOG_Handler(LOG_CDBG, "%s Deinit Device (%d)\n", __func__, laura_t->device_state);
		return rc;
}

int Laser_Enable(enum msm_laser_focus_atd_device_trun_on_type val){

		int rc=0;
		mutex_ctrl(laura_t, MUTEX_LOCK);
		
		if(camera_on_flag){
	              	LOG_Handler(LOG_DBG, "%s: Camera is running, do nothing!!\n ", __func__);
	              	return rc;
             	}

		#if 0 // not necessary
		if (laura_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF){
			rc = dev_deinit(laura_t);
			laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		}
		#endif
		
		dev_init(laura_t);
		msleep(1); // wait for power stable
		if(val == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION)
			rc = Laura_device_power_up_init_interface(laura_t, DO_CAL, &calibration_flag);
		else
			rc = Laura_device_power_up_init_interface(laura_t, NO_CAL, &calibration_flag);			
		if (rc < 0){
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			return rc;
		}

		Load_variety_data();
		
		laura_t->device_state = val;
		load_calibration_data = (val == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION?true:false);	
		LOG_Handler(LOG_CDBG, "%s Init Device (%d)\n", __func__, laura_t->device_state);

		#if 0
		if(keepMeasuring){
			ioctrl_close =false;
			LOG_Handler(LOG_CDBG,"kick off read interface for keep-measurement mode\n");		
			queue_work(Measure_wq, &Measure_wk);
		}
		#endif
	
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return rc;	
}

int Laser_Enable_by_Camera(enum msm_laser_focus_atd_device_trun_on_type val){

	int rc=0;
	int retry_i = 0;
	mutex_ctrl(laura_t, MUTEX_LOCK);
	laura_t->device_state = val;
	rc = dev_init(laura_t);
	msleep(1); // wait for power stable
	rc = Laura_device_power_up_init_interface(laura_t, DO_CAL, &calibration_flag);
	if (rc < 0) {
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return rc;
	}
	//?
	Load_variety_data();

	laura_t->device_state = val;
	load_calibration_data = true;
	camera_on_flag = true;

	keepMeasuring = true;
	ioctrl_close = false;

retry_on:
	rc = WaitMCPUOn(laura_t);
	if (rc < 0) {
		if (++retry_i < 3) goto retry_on;
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return rc;
	}

	control_signal(laura_t);

	queue_work(Measure_wq, &Measure_wk);

	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_CDBG, "%s Init Device (%d)\n", __func__, laura_t->device_state);
	return rc;
}

int Laser_Disable_by_Camera(enum msm_laser_focus_atd_device_trun_on_type val){

	int rc=0;
	mutex_ctrl(laura_t, MUTEX_LOCK);
	(void) WaitMCPUStandby(laura_t);
	rc = dev_deinit(laura_t);
	if (rc < 0){
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return rc;
	}
	laura_t->device_state = val;
	load_calibration_data = false;
	camera_on_flag = false;
	keepMeasuring = false;
	ioctrl_close = true;
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_CDBG, "%s Deinit Device (%d)\n", __func__, laura_t->device_state);
	return rc;
}

static ssize_t ATD_Laura_device_enable_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
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
	if (laura_t->device_state == val)	{
		LOG_Handler(LOG_ERR, "%s Setting same command (%d) !!\n", __func__, val);
		return -EINVAL;
	}
	
	switch (val) {
		case MSM_LASER_FOCUS_DEVICE_OFF:
			client--;
			LOG_Handler(LOG_CDBG,"%s: client leave via proc (%d)\n", __func__, client);
			if(client>=0){
				rc = Laser_Disable(val);
				if (rc == 9999) {
					client++; //recovery last client status
					return len;
				} else if (rc < 0)
					goto DEVICE_TURN_ON_ERROR;
				client = 0;
				power_down(laura_t);
			}
			else if(client < 0){
				client = 0;
				LOG_Handler(LOG_CDBG,"%s: client dummy leave\n", __func__, client);
			}
			break;
			
		case MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION:
		case MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION:	
			client++;
			LOG_Handler(LOG_CDBG,"%s: client enter via proc (%d), type (%d)\n", __func__, client, val);
			if(client==1 || laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF
				|| laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
				power_up(laura_t);		
				rc = Laser_Enable(val);
				if (rc < 0)
					goto DEVICE_TURN_ON_ERROR;
			}
			break;
		case MSM_LASER_FOCUS_DEVICE_INIT_CCI:	
			client++;
			if(client==1 || laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF
				|| laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
				power_up(laura_t);		
				rc = Laser_Enable_by_Camera(val);
				if (rc < 0)
					goto DEVICE_TURN_ON_ERROR;
			}
			break;
			
		case MSM_LASER_FOCUS_DEVICE_DEINIT_CCI:
			client--;
			if(client>=0){
				client = 0;
				rc = Laser_Disable_by_Camera(val);
				if (rc < 0)
					goto DEVICE_TURN_ON_ERROR;
				power_down(laura_t);
			}
			else if(client < 0){
				client = 0;
				LOG_Handler(LOG_CDBG,"%s: client dummy leave\n", __func__, client);
			}
			break;
		default:
			LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
			break;
	}


	LOG_Handler(LOG_DBG, "%s: command (%d) done\n",__func__,val);
	return len;
	
DEVICE_TURN_ON_ERROR:

	rc = dev_deinit(laura_t);
	if (rc < 0) 
		LOG_Handler(LOG_ERR, "%s Laura_deinit failed %d\n", __func__, __LINE__);
	rc = power_down(laura_t);
	if (rc < 0)
		LOG_Handler(LOG_ERR, "%s Laura power off failed %d\n", __func__, __LINE__);
	laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
	
	LOG_Handler(LOG_DBG, "%s: Exit due to device trun on fail !!\n", __func__);
	return -EIO;
}

static int ATD_Laura_device_enable_read(struct seq_file *buf, void *v){
	seq_printf(buf, "%d\n", laura_t->device_state);
	return 0;
}

static int ATD_Laura_device_enable_open(struct inode *inode, struct  file *file){
	return single_open(file, ATD_Laura_device_enable_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_enable_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_enable_open,
	.write = ATD_Laura_device_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_device_get_range_read(struct seq_file *buf, void *v)
{
	int Range = 0;

	if (keepMeasuring) {
		LOG_Handler(LOG_DBG, "Get raw:0x%x,confidence:0x%x,range:(%d),error(%d),Device(%d)\n", get_debug_raw_range(), get_debug_raw_confidence(), Range1, ErrCode1, laura_t->device_state);
		seq_printf(buf, "%d\n", Range1);
	} else {

		mutex_ctrl(laura_t, MUTEX_LOCK);
		if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
			laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
			LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
			seq_printf(buf, "%d\n", 0);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			//return -EBUSY;
			return 0;
		}

		#if 0
		if(laser_focus_enforce_ctrl != 0){
			seq_printf(buf, "%d\n", laser_focus_enforce_ctrl);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			return 0;
		}
		if(!keepMeasuring)
			Range = Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag);
		else
			Range = Range1;	//need more condition
		#else
		Range = Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag);
		#endif

		if (Range >= OUT_OF_RANGE) {
			LOG_Handler(LOG_ERR, "%s: Read_range(%d) failed\n", __func__, Range);
			Range = OUT_OF_RANGE;
		}
		
		LOG_Handler(LOG_CDBG, "%s : Get range (%d)  Device (%d)\n", __func__, Range , laura_t->device_state);

		seq_printf(buf, "%d\n", Range);

		mutex_ctrl(laura_t, MUTEX_UNLOCK);
	}
	
	return 0;
}
 
static int ATD_Laura_device_get_range_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_Laura_device_get_range_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_get_range_fos = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_get_range_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_device_get_range_read_more_info(struct seq_file *buf, void *v)
{
	int RawRange = 0;

	if (keepMeasuring) {
		LOG_Handler(LOG_DBG, "Get raw:0x%x,confidence:0x%x,range:(%d),error(%d),Device(%d)\n", get_debug_raw_range(), get_debug_raw_confidence(), Range1, ErrCode1, laura_t->device_state);
		seq_printf(buf, "%d#%d#%d\n", Range1, DMax, ErrCode1);
	} else {
	
		mutex_ctrl(laura_t, MUTEX_LOCK);

		if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
			laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
			LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
			seq_printf(buf, "%d\n", 0);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			return -EBUSY;
		}

		#if 0
		if(laser_focus_enforce_ctrl != 0){
			seq_printf(buf, "%d#%d#%d\n", laser_focus_enforce_ctrl, DMax, ErrCode1);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			return 0;
		}
		#endif

		RawRange = (int) Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag);

		if (RawRange >= OUT_OF_RANGE) {
			LOG_Handler(LOG_ERR, "%s: Read_range(%d) failed\n", __func__, RawRange);
			RawRange = OUT_OF_RANGE;
		}
		
		LOG_Handler(LOG_CDBG, "Get range(%d) error(%d) Device(%d)\n", RawRange, ErrCode1, laura_t->device_state);

		seq_printf(buf, "%d#%d#%d\n", RawRange, DMax, ErrCode1);

		mutex_ctrl(laura_t, MUTEX_UNLOCK);
	}
	return 0;
}
 
static int ATD_Laura_device_get_range_more_info_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_Laura_device_get_range_read_more_info, NULL);
}

static const struct file_operations ATD_laser_focus_device_get_range_more_info_fos = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_get_range_more_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t ATD_Laura_device_calibration_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, ret = 0;
	char messages[8];

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		return -EBUSY;
	}

	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);

	LOG_Handler(LOG_DBG, "%s command : %d\n", __func__, val);
	
	switch (val) {
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Laura_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Laura_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Laura_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		default:
			LOG_Handler(LOG_ERR, "%s command fail(%d) !!\n", __func__, val);
			return -EINVAL;
			break;
	}

	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return len;
}

static int ATD_Olivia_dummy_read(struct seq_file *buf, void *v)
{
	return 0;
}

static int ATD_Olivia_dummy_open(struct inode *inode, struct  file *file)
{
	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, ATD_Olivia_dummy_read, NULL);
}

static ssize_t ATD_Olivia_device_calibration_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, ret = 0;
	char messages[8]="";
	LOG_Handler(LOG_CDBG, "%s: Enter\n", __func__);

	if (OLI_device_invalid()){
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
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
		//case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Olivia_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		default:
			LOG_Handler(LOG_ERR, "%s command fail(%d) !!\n", __func__, val);
			return -EINVAL;
			break;
	}

	LOG_Handler(LOG_CDBG, "%s: Exit\n", __func__);	
	return len;
}


static const struct file_operations ATD_olivia_calibration_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Olivia_dummy_open,
	.write = ATD_Olivia_device_calibration_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations ATD_laser_focus_device_calibration_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_get_range_open,
	.write = ATD_Laura_device_calibration_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_get_calibration_input_data_proc_open(struct inode *inode, struct  file *file)
{
	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, Laura_get_calibration_input_data_interface, NULL);
}

static const struct file_operations ATD_Laura_get_calibration_input_data_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_get_calibration_input_data_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Olivia_get_calibration_input_data_proc_open(struct inode *inode, struct  file *file)
{
	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, Olivia_get_calibration_input_data_interface, NULL);
}

static const struct file_operations ATD_Olivia_get_calibration_input_data_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Olivia_get_calibration_input_data_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	ATD_status = dev_I2C_status_check(laura_t, MSM_CAMERA_I2C_WORD_DATA);

	LOG_Handler(LOG_CDBG,"FW version %d.%d\n",module_verion[0],module_verion[1]);
	LOG_Handler(LOG_CDBG,"FW id: %02x %02x %02x %02x \n",module_id[0],module_id[1],module_id[2],module_id[3]);

	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	seq_printf(buf, "%d\n", ATD_status?1:0);

	return 0;
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

static int ATD_Laura_I2C_status_check_proc_read_for_camera(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", ATD_status);
	return 0;
}

static int ATD_Laura_I2C_status_check_proc_open_for_camera(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_Laura_I2C_status_check_proc_read_for_camera, NULL);
}

static const struct file_operations ATD_I2C_status_check_for_camera_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_I2C_status_check_proc_open_for_camera,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_register_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_laura_register_read, NULL);
}

static const struct file_operations dump_laser_focus_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_register_read(struct seq_file *buf, void *v)
{	
	//mutex_ctrl(laura_t, MUTEX_LOCK);
	laura_debug_dump(buf, v);
	//mutex_ctrl(laura_t, MUTEX_UNLOCK);
	return 0;
}

static int dump_Laura_laser_focus_debug_register_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_Laura_debug_register_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int Laura_laser_focus_enforce_read(struct seq_file *buf, void *v)
{
	return 0;
}

static int Laura_laser_focus_enforce_open(struct inode *inode, struct file *file)
{
	return single_open(file, Laura_laser_focus_enforce_read, NULL);
}

static ssize_t Laura_laser_focus_enforce_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	rc = Laser_Focus_enforce(laura_t, buff, len, &laser_focus_enforce_ctrl);
	mutex_ctrl(laura_t, MUTEX_UNLOCK);
	return rc;
}

static const struct file_operations laser_focus_enforce_fops = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_enforce_open,
	.write = Laura_laser_focus_enforce_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int Laura_laser_focus_log_contorl_read(struct seq_file *buf, void *v)
{
	return 0;
}

static int Laura_laser_focus_log_contorl_open(struct inode *inode, struct file *file)
{
	return single_open(file, Laura_laser_focus_log_contorl_read, NULL);
}

static ssize_t Laura_laser_focus_log_contorl_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;
	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	rc = Laser_Focus_log_contorl(buff, len);
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	return rc;
}

static const struct file_operations laser_focus_log_contorl_fops = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_log_contorl_open,
	.write = Laura_laser_focus_log_contorl_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*++++++++++CE Debug++++++++++*/
static int dump_Laura_debug_value1_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

	seq_printf(buf,"%d\n",cal_data[3]);
	return 0;
}

static int dump_Laura_laser_focus_debug_value1_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_Laura_debug_value1_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value1_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value1_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value2_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

	seq_printf(buf,"%d\n",cal_data[20]);
	return 0;
}

static int dump_Laura_laser_focus_debug_value2_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_Laura_debug_value2_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value2_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value2_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value3_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

	seq_printf(buf,"%d\n",cal_data[29]);
	return 0;
}

static int dump_Laura_laser_focus_debug_value3_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_Laura_debug_value3_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value3_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value3_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value4_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

	seq_printf(buf,"%d\n",cal_data[22]);
	return 0;
}

static int dump_Laura_laser_focus_debug_value4_open(struct inode *inode, struct  file *file)
{

	return single_open(file, dump_Laura_debug_value4_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value4_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value4_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value5_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

	seq_printf(buf,"%d\n",cal_data[31]);
	return 0;
}

static int dump_Laura_laser_focus_debug_value5_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_Laura_debug_value5_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value5_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value5_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int dump_Laura_debug_value6_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

	seq_printf(buf,"%d\n",cal_data[23]);
	return 0;
}

static int dump_Laura_laser_focus_debug_value6_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_Laura_debug_value6_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value6_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value6_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value7_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

	seq_printf(buf,"%d\n",cal_data[32]);
	return 0;
}

static int dump_Laura_laser_focus_debug_value7_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_Laura_debug_value7_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value7_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value7_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value8_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

	seq_printf(buf,"%d\n",cal_data[24]);
	return 0;
}

static int dump_Laura_laser_focus_debug_value8_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_Laura_debug_value8_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value8_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value8_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value9_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

	seq_printf(buf,"%d\n",cal_data[33]);
	return 0;
}

static int dump_Laura_laser_focus_debug_value9_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_Laura_debug_value9_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value9_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value9_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
//ATD will tell how to impliment it
static int dump_Laura_value_check_read(struct seq_file *buf, void *v)
{
	keepMeasuring = !keepMeasuring;
	printk("keepMeasuring changed\n");
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
/*----------CE Debug----------*/

//CSC mode
static int laser_focus_read_csc_mode(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_CDBG, "%s: Enter and Exit, calibration: %d\n", __func__, CSC_mode_flag);
	seq_printf(buf,"%d\n",CSC_mode_flag);
	return 0;
}
static int laser_focus_set_csc_mode_open(struct inode *inode, struct file *file)
{
	//LOG_Handler(LOG_FUN, "%s: Enter and Exit\n", __func__);
	return single_open(file, laser_focus_read_csc_mode, NULL);
}

static ssize_t laser_focus_set_csc_mode(struct file *filp, const char __user *buff, size_t len, loff_t *data)
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
			CSC_mode_flag = false;
			break;
		case 1:
			CSC_mode_flag = true;
			break;
		default:
			LOG_Handler(LOG_DBG, "command '%d' is not valid\n", val);
	}

	LOG_Handler(LOG_CDBG, "%s: Exit, CSC mode: %d\n", __func__, CSC_mode_flag);
	return rc;
}

static const struct file_operations laser_focus_cscmode_fops = {
	.owner = THIS_MODULE,
	.open = laser_focus_set_csc_mode_open,
	.write = laser_focus_set_csc_mode,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*----------CSC mode----------*/


static int Laura_laser_focus_set_K_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_DBG, "%s: Enter and Exit, calibration: %d\n", __func__, calibration_flag);
	seq_printf(buf,"%d\n",calibration_flag);
	return 0;
}
static int Laura_laser_focus_set_K_open(struct inode *inode, struct file *file)
{
	//LOG_Handler(LOG_FUN, "%s: Enter and Exit\n", __func__);
	return single_open(file, Laura_laser_focus_set_K_read, NULL);
}

static ssize_t Laura_laser_focus_set_K_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
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

static const struct file_operations laser_focus_set_K_fops = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_set_K_open,
	.write = Laura_laser_focus_set_K_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int Laura_laser_focus_product_family_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_DBG, "%s: Enter and Exit, Product Family: %d\n", __func__, Laser_Product);
	seq_printf(buf,"%d",Laser_Product);
	return 0;
}
static int Laura_laser_focus_product_family_open(struct inode *inode, struct file *file)
{
	//LOG_Handler(LOG_FUN, "%s: Enter and Exit\n", __func__);
	return single_open(file, Laura_laser_focus_product_family_read, NULL);
}

static const struct file_operations laser_focus_product_family = {
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

int Olivia_get_measure(int* distance){
	int RawRange = 0;

	if(repairing_state){
		Range1 = OUT_OF_RANGE;
		ErrCode1 = RANGE_ERR_NOT_ADAPT;
		return 0;
	}
	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	mutex_ctrl(laura_t, MUTEX_LOCK);

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return -EBUSY;
	}

	if(laser_focus_enforce_ctrl != 0){
		*distance = laser_focus_enforce_ctrl;
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return 0;
	}
if(!keepMeasuring){
	RawRange = Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag);
	if(RawRange<0)
		queue_work(Measure_wq, &Measure_wk);
}
else
	RawRange = Range1;	//need more condition
	if (RawRange < 0) {
		LOG_Handler(LOG_ERR, "%s: Read_range(%d) failed\n", __func__, RawRange);
		RawRange = 0;
	}
	
	//LOG_Handler(LOG_DBG, "%s : Get range (%d)  Device (%d)\n", __func__, RawRange , laura_t->device_state);
	*distance = RawRange;



	
	mutex_ctrl(laura_t, MUTEX_UNLOCK);
	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);	
	return 0;


}

static int Olivia_misc_open(struct inode *inode, struct file *file){

	int rc = 0, cnt = 0;

	mutex_ctrl(laura_t, MUTEX_LOCK);
	client++;
	LOG_Handler(LOG_CDBG,"%s: client enter via ioctrl(%d)\n", __func__, client);

	if(client == 1){
		power_up(laura_t);
		rc = Laser_Enable(MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION);
	}
	while(rc < 0 && (cnt++ < 3)){
		dev_deinit(laura_t);
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
			rc = dev_init(laura_t);
			LOG_Handler(LOG_ERR,"%s: retry laser deinit, rc(%d), retry(%d)\n", __func__, rc, cnt);	
		}
		if(rc < 0)
			LOG_Handler(LOG_ERR,"%s: retry release fail\n", __func__);			

		power_down(laura_t);
	}
	else if(client < 0){
		client =0;
		LOG_Handler(LOG_CDBG,"%s: dummy leave, reset client to %d\n", __func__, client);
	}
	
	mutex_ctrl(laura_t, MUTEX_UNLOCK);
	return 0;
}

static long Olivia_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

 	unsigned int dist[4] = {0,0,0,0};
	char name[ASUS_LASER_NAME_SIZE];
	int distance;
	int ret = 0;
	static int  cnt=0;
	struct timeval start,now;
	
	cnt++;
	if(cnt >= 50)
		O_get_current_time(&start);	
	
	switch (cmd) {	
		case ASUS_LASER_SENSOR_MEASURE:
			Olivia_get_measure(&distance);
			//__put_user(dist, (int* __user*)arg);
			dist[0] = distance;
			dist[1] = ErrCode1;
			dist[2] = DMax;
			dist[3] = calibration_flag;
			ret = copy_to_user((int __user*)arg, dist, sizeof(dist));
			break;
			
		case ASUS_LASER_SENSOR_GET_NAME:
			snprintf(name, ASUS_LASER_NAME_SIZE, MODULE_NAME);
			//__put_user(MODULE_NAME, (int __user*)arg);
			ret = copy_to_user((int __user*)arg, &name, sizeof(name));			
			break;
			
		default:
			LOG_Handler(LOG_ERR,"%s: ioctrl command is not valid\n", __func__);
	}
	
	if(cnt>=50){
		O_get_current_time(&now);
		LOG_Handler(LOG_CDBG,"read data stime duration: %ld\t [%d,%d]\n",(((now.tv_sec*1000000)+now.tv_usec)-((start.tv_sec*1000000)+start.tv_usec))/1000,distance,ErrCode1);
		cnt=0;
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

static int Olivia_misc_register(int Product)
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


static void Olivia_Create_proc(void)
{
	proc(STATUS_PROC_FILE, 0664, &ATD_I2C_status_check_fops);
	proc(STATUS_PROC_FILE_FOR_CAMERA, 0664, &ATD_I2C_status_check_for_camera_fops);
	proc(DEVICE_TURN_ON_FILE, 0666, &ATD_laser_focus_device_enable_fops);
	proc(DEVICE_GET_VALUE, 0664, &ATD_laser_focus_device_get_range_fos);
	proc(DEVICE_GET_VALUE_MORE_INFO, 0664, &ATD_laser_focus_device_get_range_more_info_fos);

if(Laser_Product == PRODUCT_OLIVIA){
	proc(DEVICE_SET_CALIBRATION, 0664, &ATD_olivia_calibration_fops);
	proc(DEVICE_GET_CALIBRATION_INPUT_DATA, 0664, &ATD_Olivia_get_calibration_input_data_fops);
}
else{
	proc(DEVICE_SET_CALIBRATION, 0664, &ATD_laser_focus_device_calibration_fops);
	proc(DEVICE_GET_CALIBRATION_INPUT_DATA, 0664, &ATD_Laura_get_calibration_input_data_fops);
}
	
	proc(DEVICE_DUMP_REGISTER_VALUE, 0664, &dump_laser_focus_register_fops);
	proc(DEVICE_DUMP_DEBUG_VALUE, 0664, &dump_laser_focus_debug_register_fops);
	//proc(DEVICE_ENFORCE_FILE, 0664, &laser_focus_enforce_fops);
	proc(DEVICE_LOG_CTRL_FILE, 0664, &laser_focus_log_contorl_fops);

	//for ce
	proc(DEVICE_DEBUG_VALUE1, 0664, &dump_laser_focus_debug_value1_fops);
	proc(DEVICE_DEBUG_VALUE2, 0664, &dump_laser_focus_debug_value2_fops);
	proc(DEVICE_DEBUG_VALUE3, 0664, &dump_laser_focus_debug_value3_fops);
	proc(DEVICE_DEBUG_VALUE4, 0664, &dump_laser_focus_debug_value4_fops);
	proc(DEVICE_DEBUG_VALUE5, 0664, &dump_laser_focus_debug_value5_fops);	
	proc(DEVICE_DEBUG_VALUE6, 0664, &dump_laser_focus_debug_value6_fops);
	proc(DEVICE_DEBUG_VALUE7, 0664, &dump_laser_focus_debug_value7_fops);
	proc(DEVICE_DEBUG_VALUE8, 0664, &dump_laser_focus_debug_value8_fops);
	proc(DEVICE_DEBUG_VALUE9, 0664, &dump_laser_focus_debug_value9_fops);
	proc(DEVICE_VALUE_CHECK, 0664, &dump_laser_focus_value_check_fops);
	proc(DEVICE_CSC_MODE, 0664, &laser_focus_cscmode_fops);
	//for ce

	//for dit
	proc(DEVICE_IOCTL_SET_K, 0664, &laser_focus_set_K_fops);
	proc(DEVICE_IOCTL_PRODUCT_FAMILY, 0444, &laser_focus_product_family);
	//for dit
}

static const struct v4l2_subdev_internal_ops msm_laser_focus_internal_ops;

static struct v4l2_subdev_core_ops msm_laser_focus_subdev_core_ops = {
	.ioctl = NULL,
	//.s_power = msm_laser_focus_power,
};

static struct v4l2_subdev_ops msm_laser_focus_subdev_ops = {
	.core = &msm_laser_focus_subdev_core_ops,
};

#if 0
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

	LOG_Handler(LOG_FUN, "%s: done\n", __func__);
	return 0;
}
#endif

static void set_subdev(void){
	laura_t->act_v4l2_subdev_ops = &msm_laser_focus_subdev_ops;

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

	LOG_Handler(LOG_FUN, "%s: done\n", __func__);
}

static void set_laser_config(void){

	/* Init data struct */
	laura_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	laura_t->laser_focus_cross_talk_offset_value = 0;
	laura_t->laser_focus_offset_value = 0;
	laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;

}

static int Laura_Init_Chip_Status_On_Boot(struct msm_laser_focus_ctrl_t *dev_t){
	       int rc = 0, chip_status = 0;
	      
		     LOG_Handler(LOG_CDBG, "%s: Enter Init Chip Status\n", __func__);    
		
			    mutex_ctrl(laura_t, MUTEX_LOCK);
		    
			      rc = dev_init(laura_t);
			    
			      rc = Laura_device_power_up_init_interface(laura_t, NO_CAL, &calibration_flag);
			      chip_status = WaitMCPUStandby(dev_t);
			      if (rc < 0 || chip_status < 0){
			      LOG_Handler(LOG_ERR, "%s Device init fail !! (rc,status):(%d,%d)\n", __func__, rc, chip_status);
			     } else {
			    LOG_Handler(LOG_CDBG, "%s Init init success !! (rc,status):(%d,%d)\n", __func__, rc,chip_status);
			     }
			   
			    rc = dev_deinit(laura_t);
			  
			       mutex_ctrl(laura_t, MUTEX_UNLOCK);
			    
			    LOG_Handler(LOG_CDBG, "%s: Exit Init Chip Status\n", __func__);
			    
			    return rc;
}

static int olivia_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc;

	laura_t = kzalloc(sizeof(struct msm_laser_focus_ctrl_t),GFP_KERNEL);
	if (!laura_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	LOG_Handler(LOG_CDBG, "%s: done, addr: 0x%x\n", __func__, client->addr);


	reg_vdd = devm_regulator_get(&client->dev, "vdd");
	reg_vdd_i2c = devm_regulator_get(&client->dev, "vdd_i2c");
	//regulator_set_voltage (reg_vdd, 3000000, 3000000);

	(void) enable_laser_power();
	(void) dev_init(laura_t);
	msleep(1);
	
	rc = i2c_smbus_read_word_data(client, 0x28);
	if (rc < 0) {
		pr_err("%s:%d i2c bus is not correct\n", __func__, __LINE__);
		goto probe_failure;
	}
	LOG_Handler(LOG_CDBG, "%s: chip ID 0x%x\n", __func__, rc);

	/* Set platform device handle */
	laura_t->blsp_i2c_client = client;

	set_laser_config();

	/* Init mutex */
	mutex_ctrl(laura_t, MUTEX_ALLOCATE);
	mutex_ctrl(laura_t, MUTEX_INIT);

	set_subdev();

	rc = Olivia_misc_register(Laser_Product);
	if (rc < 0)
		goto probe_failure;

	switch (asus_HW_ID) {
		case HW_ID_EVB:
		case HW_ID_EVB2:
		case HW_ID_SR1:
		case HW_ID_SR2:
			Laura_Init_Chip_Status_On_Boot(laura_t);
			break;

		default:
			// turn off power
			break;
	}
	
	ATD_status = 2;

	Olivia_Create_proc();

	Measure_wq = create_singlethread_workqueue("Laser_wq");
	INIT_WORK(&Measure_wk, keep_measure_work);
	
        dev_deinit(laura_t);
	switch (asus_HW_ID) {
		case HW_ID_EVB:
		case HW_ID_EVB2:
		case HW_ID_SR1:
		case HW_ID_SR2:
			// always power on
			break;

		default:
			(void) disable_laser_power();
			break;
	}
	
	LOG_Handler(LOG_CDBG, "%s: Probe Success\n", __func__);
	return 0;

probe_failure:
	LOG_Handler(LOG_CDBG, "%s: Probe failed, rc = %d\n", __func__, rc);
	kfree(laura_t);
	(void) dev_deinit(laura_t);
	(void) disable_laser_power();
        return rc;

}

static const struct i2c_device_id olivia_i2c_id[] = {
	{LASER_SENSOR_NAME, 0},
	{}
};


static struct of_device_id olivia_match_table[] = {
	{ .compatible = "hptg,olivia",},
	{ },
};

static struct i2c_driver olivia_i2c_driver = {
	.id_table = olivia_i2c_id,
	.probe = olivia_i2c_probe,
	.driver = {
		.name = LASER_SENSOR_NAME,
		.owner = THIS_MODULE,
 		.of_match_table = of_match_ptr(olivia_match_table),
	},
};

static int __init olivia_init_module(void){
	int32_t rc = 0;
	LOG_Handler(LOG_DBG, "%s: Enter\n", __func__);
	rc = i2c_add_driver(&olivia_i2c_driver);
	LOG_Handler(LOG_DBG, "%s rc %d\n", __func__, rc);
	return rc;
}

static void __exit olivia_driver_exit(void){
	kfree(laura_t);
	misc_deregister(&Olivia_misc);
	i2c_del_driver(&olivia_i2c_driver);
}

module_init(olivia_init_module);
module_exit(olivia_driver_exit);
MODULE_DESCRIPTION("MSM LASER_FOCUS");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Hugo Lin <Hugo_Lin@asus.com>");

