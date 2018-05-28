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
#include <asm/setup.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include <media/msm_cam_sensor.h>
#include "../io/msm_camera_dt_util.h"
#include "../../common/msm_camera_io_util.h"
#include "cm3323e.h"
#include "lightsensor.h"

#define ASUS_RGB_SENSOR_DATA_SIZE	5
#define ASUS_RGB_SENSOR_NAME_SIZE	32
#define ASUS_RGB_SENSOR_IOC_MAGIC                      ('C')	///< RGB sensor ioctl magic number
#define ASUS_RGB_SENSOR_IOCTL_DATA_READ           _IOR(ASUS_RGB_SENSOR_IOC_MAGIC, 1, int[ASUS_RGB_SENSOR_DATA_SIZE])	///< RGB sensor ioctl command - Read data RGBW
#define ASUS_RGB_SENSOR_IOCTL_IT_SET           _IOW(ASUS_RGB_SENSOR_IOC_MAGIC, 2, int)	///< RGB sensor ioctl command - Set integration time
#define ASUS_RGB_SENSOR_IOCTL_DEBUG_MODE           _IOW(ASUS_RGB_SENSOR_IOC_MAGIC, 3, int)	///< RGB sensor ioctl command - Get debug mode
#define ASUS_RGB_SENSOR_IOCTL_MODULE_NAME           _IOR(ASUS_RGB_SENSOR_IOC_MAGIC, 4, char[ASUS_RGB_SENSOR_NAME_SIZE])	///< RGB sensor ioctl command - Get module name

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10
#define IT_TIME_LOG_SAMPLE_RATE 500

#define LS_POLLING_DELAY 600

#define REL_RED		REL_X
#define REL_GREEN	REL_Y
#define REL_BLUE	REL_Z
#define REL_WHITE	REL_MISC

#define CFG_IT_MASK            0x0070
#define CFG_SD_MASK            0x0001

#define RGB_INDEX_R 0
#define RGB_INDEX_G 1
#define RGB_INDEX_B 2
#define RGB_INDEX_W 3

#define RGB_SENSOR_FILE		"/factory/rgbSensor_data"
extern int g_ftm_mode;
int g_ftm_mode_RGB=0;

static struct mutex als_enable_mutex;
struct msm_rgb_sensor_vreg {
	struct camera_vreg_t *cam_vreg;
	void *data[CAM_VREG_MAX];
	int num_vreg;
};
struct cm3323e_info {
	struct class *cm3323e_class;
	struct device *ls_dev;
	struct input_dev *ls_input_dev;

	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;
	struct msm_rgb_sensor_vreg vreg_cfg;

	int als_enable;
	int (*power)(int, uint8_t); /* power to the chip */

	int lightsensor_opened;
	int polling_delay;
	int it_time;
	bool using_calib;
};
struct cm3323e_status {
	bool need_wait;
	u32 start_time_jiffies;
	u32 end_time_jiffies;
	u16 delay_time_ms;
	u16 delay_time_jiffies;
};
struct cm3323e_info *cm_lp_info;
struct cm3323e_status g_rgb_status = {false, 0, 0};


static int calibration_data[] = {
	1, 1, 1, 1
};
static int config_data[] = {
	1, 1, 1, 1
};

enum RGB_data {
	RGB_DATA_R = 0,
	RGB_DATA_G,
	RGB_DATA_B,
	RGB_DATA_W,
};
enum RGB_it {
	RGB_IT_40MS = 0,
	RGB_IT_80MS,
	RGB_IT_160MS,
	RGB_IT_320MS,
	RGB_IT_640MS,
	RGB_IT_1280MS,
};
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
		if (i2c_transfer(cm_lp_info->i2c_client->adapter, msg, 2) > 0)
			break;

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		RGB_DBG_E("%s retry over %d\n", __func__, I2C_RETRY_COUNT);
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
		if (i2c_transfer(cm_lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		RGB_DBG_E("%s retry over %d\n", __func__, I2C_RETRY_COUNT);
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
		RGB_DBG_E("%s: I2C_RxData fail [0x%x, 0x%x]\n", __func__, slaveAddr, cmd);
		return ret;
	}

	*pdata = (buffer[1] << 8) | buffer[0];
#if 0
	/* Debug use */
	RGB_DBG("%s: I2C_RxData[0x%x, 0x%x] = 0x%x\n", __func__, slaveAddr, cmd, *pdata);
#endif
	return ret;
}

static int _cm3323e_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	RGB_DBG("%s: _cm3323e_I2C_Write_Word[0x%x, 0x%x, 0x%x]\n", __func__, SlaveAddress, cmd, data);
#endif
	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data & 0xff);
	buffer[2] = (uint8_t)((data & 0xff00) >> 8);

	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		RGB_DBG_E("%s: I2C_TxData fail\n", __func__);
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
		RGB_DBG_E("%s: Failed to read reg 0x%02x, rc=%d\n", __func__, cmd, rc);
		goto out;
	}
	adc_data &= ~mask;
	adc_data |= data & mask;
	rc = _cm3323e_I2C_Write_Word(SlaveAddress, cmd, adc_data);
	if (rc) {
		RGB_DBG_E("%s: Failed to write reg 0x%02x, rc=%d\n", __func__, cmd, rc);
	}
out:
	return rc;
}

static int32_t als_power(int config)
{
	int rc = 0, i, cnt;
	struct msm_rgb_sensor_vreg *vreg_cfg;
	if (!cm_lp_info) {
		RGB_DBG_E("%s: null pointer, probe might not finished!\n", __func__);
		return -1;
	}
	vreg_cfg = &cm_lp_info->vreg_cfg;
	cnt = vreg_cfg->num_vreg;
	if (!cnt)
		return 0;

	if (cnt >= CAM_VREG_MAX) {
		pr_err("%s failed %d cnt %d\n", __func__, __LINE__, cnt);
		return -EINVAL;
	}

	for (i = 0; i < cnt; i++) {
		rc = msm_camera_config_single_vreg(&(cm_lp_info->i2c_client->dev),
			&vreg_cfg->cam_vreg[i],
			(struct regulator **)&vreg_cfg->data[i],
			config);
	}
	return rc;
}
void rgbSensor_setDefaultCalibrationData(void)
{
	int i = 0;
	for (i = 0; i < 4; i++) {
		config_data[i] = 1;
		calibration_data[i] = 1;
	}
}

void rgbSensor_setCalibrationData(int config_local[], int calibration_local[])
{
	int i = 0;
	for (i = 0; i < 4; i++) {
		if (config_local[i] <= 0 || calibration_local[i] <= 0) {
			RGB_DBG_E("%s: wrong data!\n", __func__);
			rgbSensor_setDefaultCalibrationData();
			return;
		}
	}
	for (i = 0; i < 4; i++) {
		config_data[i] = config_local[i];
		calibration_data[i] = calibration_local[i];
	}
	RGB_DBG("%s: calibration data updated!\n", __func__);
}

int rgbSensor_getUserSpaceData(char* buf, int len)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	int readlen = 0;

	fp = filp_open(RGB_SENSOR_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		RGB_DBG_E("%s: open %s fail\n", __func__, RGB_SENSOR_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, buf, len, &pos_lsts);
		buf[readlen] = '\0';
	} else {
		RGB_DBG_E("%s: f_op=NULL or op->read=NULL\n", __func__);
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
	return 0;
}
void rgbSensor_updateCalibrationData(void)
{
	int ret = 0;
	char buf[64];
	int config_local[4] = {
		0, 0, 0, 0
	};
	int calibration_local[4] = {
		0, 0, 0, 0
	};
	ret = rgbSensor_getUserSpaceData(buf, 64);

	if (ret >= 0) {
		sscanf(buf, "%d/%d\n%d/%d\n%d/%d\n%d/%d\n",
			&config_local[0], &calibration_local[0],
			&config_local[1], &calibration_local[1],
			&config_local[2], &calibration_local[2],
			&config_local[3], &calibration_local[3]);
		RGB_DBG("%s: config: %d,%d,%d,%d. calib data: %d,%d,%d,%d\n", __func__,
			config_local[0], config_local[1], config_local[2], config_local[3],
			calibration_local[0], calibration_local[1], calibration_local[2], calibration_local[3]);
	}
	rgbSensor_setCalibrationData(config_local, calibration_local);
}
static void rgbSensor_setDelay(void)
{
	g_rgb_status.need_wait = true;
	g_rgb_status.start_time_jiffies = jiffies;
	if (cm_lp_info->it_time >= 0 && cm_lp_info->it_time <= 5) {
		if (g_ftm_mode_RGB)
			g_rgb_status.delay_time_ms = (40 << cm_lp_info->it_time) * 2;
		else
			g_rgb_status.delay_time_ms = (40 << cm_lp_info->it_time) * 5 / 4;
		//RGB_DBG("%s: set delay time to %d ms\n", __func__, g_rgb_status.delay_time_ms);
	} else{
		g_rgb_status.delay_time_ms = 200;
		RGB_DBG_E("%s: wrong IT time - %d, set delay time to 200ms \n", __func__, cm_lp_info->it_time);
	}
	g_rgb_status.delay_time_jiffies = g_rgb_status.delay_time_ms * HZ / 1000;
	g_rgb_status.end_time_jiffies = g_rgb_status.start_time_jiffies + g_rgb_status.delay_time_jiffies;
}
static int rgbSensor_doEnable(bool enabled)
{
	int ret = 0;
	int l_it_time;
	RGB_DBG("%s: enabled = %d, IT time = %d\n", __func__, enabled ? 1 : 0, cm_lp_info->it_time);
	l_it_time = cm_lp_info->it_time << 4;
	if (enabled) {
		cm_lp_info->als_enable = 1;
		als_power(1);
		msleep(5);
		ret = _cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, l_it_time);
		rgbSensor_setDelay();
	} else{
		cm_lp_info->als_enable = 0;
		if (g_ftm_mode_RGB) {
			/* Rgb sensor would use last time IT to read first data and then use current IT to read data afterwards */
			/* so set IT to minimum 40ms when closing rgb sensor */
			cm_lp_info->it_time = 0;
			l_it_time = cm_lp_info->it_time << 4;
		}
		ret = _cm3323e_I2C_Write_Word(CM3323E_ADDR, CM3323E_CONF, l_it_time | CM3323E_CONF_SD);
		als_power(0);
	}
	return ret;
}
static int rgbSensor_setEnable(bool enabled)
{
	int ret = 0;
	static int l_count = 0;
	bool do_enabled = false;

	if (!cm_lp_info) {
		RGB_DBG_E("%s: null pointer, probe might not finished!\n", __func__);
		return -1;
	}
	RGB_DBG("%s: count = %d, enabled = %d\n", __func__, l_count, enabled ? 1 : 0);
	mutex_lock(&als_enable_mutex);
	if ((enabled && l_count == 0) || (!enabled && l_count == 1)) {
		ret = rgbSensor_doEnable(enabled);
		do_enabled = enabled;
	}
	if (enabled) {
		l_count++;
	} else{
		l_count--;
	}
	mutex_unlock(&als_enable_mutex);
	if (enabled) {
		rgbSensor_updateCalibrationData();
	}

	return ret;
}
#if 0
static bool rgbSensor_getEnable(void)
{
	bool result;
	if (!cm_lp_info) {
		result = false;
	} else{
		mutex_lock(&als_enable_mutex);
		if (cm_lp_info->als_enable == 1) {
			result = true;
		} else{
			result = false;
		}
		mutex_unlock(&als_enable_mutex);
	}
	return result;
}
#endif
void rgbSensor_doDelay(u32 delay_time_jiffies)
{
	u32 delay_time_ms = delay_time_jiffies * 1000 / HZ;
	//RGB_DBG("%s: delay time = %u(ms)\n", __func__, delay_time_ms);
	if (delay_time_ms < 0 || delay_time_ms > g_rgb_status.delay_time_ms) {
		RGB_DBG_E("%s: param wrong, delay default time = %u(ms)\n", __func__, g_rgb_status.delay_time_ms);
		msleep(g_rgb_status.delay_time_ms);
	} else{
		msleep(delay_time_ms);
	}
}
void rgbSensor_checkStatus(void)
{
	u32 current_tme_l = jiffies;
	if (g_rgb_status.need_wait) {
		/*handling jiffies overflow*/
		//RGB_DBG("%s: start at %u, end at %u, current at %u, delay = %u(jiffies)\n", __func__,
			//g_rgb_status.start_time_jiffies, g_rgb_status.end_time_jiffies, current_tme_l, g_rgb_status.delay_time_jiffies);
		/*normal case*/
		if (g_rgb_status.end_time_jiffies > g_rgb_status.start_time_jiffies) {
			/*between start and end - need delay*/
			if (current_tme_l >= g_rgb_status.start_time_jiffies && current_tme_l < g_rgb_status.end_time_jiffies) {
				rgbSensor_doDelay(g_rgb_status.end_time_jiffies - current_tme_l);
			} else{
			}
		/*overflow case*/
		} else{
			/*after end - don't need to delay*/
			if (current_tme_l < g_rgb_status.start_time_jiffies && current_tme_l >= g_rgb_status.end_time_jiffies) {
			} else{
				if (current_tme_l >= g_rgb_status.start_time_jiffies) {
					rgbSensor_doDelay(g_rgb_status.delay_time_jiffies - (current_tme_l - g_rgb_status.start_time_jiffies));
				} else{
					rgbSensor_doDelay(g_rgb_status.end_time_jiffies - current_tme_l);
				}
			}
		}
		g_rgb_status.need_wait = false;
	}
}
static int get_rgb_data(int rgb_data, u16 *pdata, bool l_needDelay)
{
	int ret = 0;
	u8 cmd;
	uint16_t adc_data;
	if (cm_lp_info == NULL || cm_lp_info->als_enable == 0) {
		RGB_DBG_E("%s: als not enabled yet!\n", __func__);
		ret = -1;
		goto end;
	}
	switch (rgb_data) {
	case RGB_DATA_R:
		cmd = CM3323E_R_DATA;
		break;
	case RGB_DATA_G:
		cmd = CM3323E_G_DATA;
		break;
	case RGB_DATA_B:
		cmd = CM3323E_B_DATA;
		break;
	case RGB_DATA_W:
		cmd = CM3323E_W_DATA;
		break;
	default:
		RGB_DBG_E("%s: unknown cmd(%d)\n", __func__, rgb_data);
		ret = -1;
		goto end;
	}
	if (l_needDelay) {
		rgbSensor_checkStatus();
	}
	ret = _cm3323e_I2C_Read_Word(CM3323E_ADDR, cmd, &adc_data);
	if (ret < 0) {
		RGB_DBG_E("%s: _cm3323e_I2C_Read_Word at %x fail\n", __func__, cmd);
		goto end;
	}
	*pdata = adc_data;
end:
	return ret;
}

static int rgbSensor_setup(struct cm3323e_info *lpi)
{
	int ret = 0;
	uint16_t adc_data;

	lpi->it_time = RGB_IT_160MS;

	// Enable CM3323E
	ret = rgbSensor_setEnable(true);
	if(ret < 0) {
		return ret;
	}

	// Get initial RED light data
	ret = get_rgb_data(RGB_DATA_R, &adc_data, false);
	if (ret < 0) {
		return -EIO;
	}

	// Get initial GREEN light data
	ret = get_rgb_data(RGB_DATA_G, &adc_data, false);
	if (ret < 0) {
		return -EIO;
	}

	// Get initial BLUE light data
	ret = get_rgb_data(RGB_DATA_B, &adc_data, false);
	if (ret < 0) {
		return -EIO;
	}

	// Get initial WHITE light data
	ret = get_rgb_data(RGB_DATA_W, &adc_data, false);
	if (ret < 0) {
		return -EIO;
	}

	rgbSensor_setEnable(false);
	return ret;
}
/*+++BSP David ASUS Interface+++*/

u8 rgbReg_CMD_Table[] = {
	0x00, 0x08, 0x09, 0x0a, 0x0b
};

void rgbRegDump(char tempString[], int length)
{
	int i = 0;
	uint16_t reg_value = -1;
	if (cm_lp_info != NULL && cm_lp_info->als_enable == 1) {
		for (i = 0; i < ARRAY_SIZE(rgbReg_CMD_Table); i++) {
			if (i > 0) {
				snprintf(tempString, length, "%s\n", tempString);
			}
			_cm3323e_I2C_Read_Word(CM3323E_ADDR, rgbReg_CMD_Table[i], &reg_value);
			snprintf(tempString, length, "%s0x%02x=%u", tempString, rgbReg_CMD_Table[i], reg_value);
		}
	}
}
static bool rgbSensor_checkI2C(void)
{
	int ret = -1;
	uint16_t adc_data;
	// Get initial RED light data
	rgbSensor_setEnable(true);
	ret = _cm3323e_I2C_Read_Word(CM3323E_ADDR, CM3323E_R_DATA, &adc_data);
	rgbSensor_setEnable(false);
	if (ret < 0) {
		RGB_DBG_E("%s: fail\n", __func__);
		return false;
	} else{
		return true;
	}
}
static void rgbSensor_itSet_ms(int input_ms)
{
	static uint16_t l_count = 0;
	int it_time;
	if (input_ms < 80) {
		it_time = 0;
	} else if (input_ms < 160) {
		it_time = 1;
	} else if (input_ms < 320) {
		it_time = 2;
	} else if (input_ms < 640) {
		it_time = 3;
	} else if (input_ms < 1280) {
		it_time = 4;
	} else{
		it_time = 5;
	}
	if (l_count % IT_TIME_LOG_SAMPLE_RATE == 0) {
		RGB_DBG("%s: it time = %d, it set = %d, log rate = %d\n", __func__, input_ms, it_time, IT_TIME_LOG_SAMPLE_RATE);
		l_count = 0;
	}
	l_count++;
	cm_lp_info->it_time = it_time;
	if (cm_lp_info->als_enable == 1) {
		_cm3323e_I2C_Mask_Write_Word(CM3323E_ADDR, CM3323E_CONF, CFG_IT_MASK, it_time << 4);
		if (g_ftm_mode_RGB)
			rgbSensor_setDelay();
	} else{
		RGB_DBG("%s: write config - it time = %d, it set = %d\n", __func__, input_ms, it_time);
	}
}
/*+++BSP David proc rgbSensor_dump Interface+++*/
static int rgbSensor_dump_proc_read(struct seq_file *buf, void *v)
{
	char rgbReg[128]="";
	rgbSensor_setEnable(true);
	rgbRegDump(rgbReg, 128);
	rgbSensor_setEnable(false);
	return seq_printf(buf, "%s\n", rgbReg);
}
static int rgbSensor_dump_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_dump_proc_read, NULL);
}
static void create_rgbSensor_dump_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_dump_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_dump", 0444, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_dump Interface---*/
/*+++BSP David proc rgbSensor_status Interface+++*/
static int rgbSensor_status_proc_read(struct seq_file *buf, void *v)
{
	int result = 0;
	if (rgbSensor_checkI2C()) {
		result = 1;
	} else{
		result = 0;
	}
	RGB_DBG("%s: %d\n", __func__, result);
	return seq_printf(buf, "%d\n", result);
}
static int rgbSensor_status_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_status_proc_read, NULL);
}

static void create_rgbSensor_status_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_status_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_status", 0444, NULL, &proc_fops);
	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_status Interface---*/
/*+++BSP David proc asusRgbCalibEnable Interface+++*/
static int asusRgbCalibEnable_proc_read(struct seq_file *buf, void *v)
{
	int result = 0;
	if (cm_lp_info->using_calib) {
		result = 1;
	} else{
		result = 0;
	}
	return seq_printf(buf, "%d\n", result);
}
static int asusRgbCalibEnable_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, asusRgbCalibEnable_proc_read, NULL);
}

static ssize_t asusRgbCalibEnable_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
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
		cm_lp_info->using_calib = false;
	} else{
		cm_lp_info->using_calib = true;
	}
	RGB_DBG("%s: %d\n", __func__, val);
	return len;
}
static void create_asusRgbCalibEnable_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  asusRgbCalibEnable_proc_open,
		.write = asusRgbCalibEnable_proc_write,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/asusRgbCalibEnable", 0664, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc asusRgbCalibEnable Interface---*/
/*+++BSP David proc asusRgbDebug Interface+++*/
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

static ssize_t asusRgbDebug_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
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
	RGB_DBG("%s: %d\n", __func__, val);
	return len;
}
static void create_asusRgbDebug_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  asusRgbDebug_proc_open,
		.write = asusRgbDebug_proc_write,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/asusRgbDebug", 0664, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc asusRgbDebug Interface---*/
/*+++BSP David proc asusRgbSetIT Interface+++*/
static int asusRgbSetIT_proc_read(struct seq_file *buf, void *v)
{
	int result = 0;
	result = 40 << cm_lp_info->it_time;
	return seq_printf(buf, "%d\n", result);
}
static int asusRgbSetIT_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, asusRgbSetIT_proc_read, NULL);
}

static ssize_t asusRgbSetIT_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256]={'\0'};
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
        RGB_DBG("%s: messages : %s\n", __func__, messages);
	val = (int)simple_strtol(messages, NULL, 10);
        RGB_DBG("%s: val : %d\n", __func__, val);

	rgbSensor_itSet_ms(val);
	RGB_DBG("%s: %d\n", __func__, val);
	return len;
}
static void create_asusRgbSetIT_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  asusRgbSetIT_proc_open,
		.write = asusRgbSetIT_proc_write,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/asusRgbSetIT", 0664, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc asusRgbSetIT Interface---*/
#if 0
/*+++BSP David proc rgbSensor_enable Interface+++*/
static ssize_t rgbSensor_enable_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	char messages[256];
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	if (messages[0] == '1') {
		rgbSensor_setEnable(true);
	} else{
		rgbSensor_setEnable(false);
	}

	RGB_DBG("%s: %s\n", __func__, messages);
	return len;
}
static int rgbSensor_enable_proc_read(struct seq_file *buf, void *v)
{
	char tempString[16];
	if (rgbSensor_getEnable()) {
		snprintf(tempString, 16, "%s", "enabled");
	} else{
		snprintf(tempString, 16, "%s", "disabled");
	}
	RGB_DBG("%s: %s\n", __func__, tempString);
	return seq_printf(buf, "%s\n", tempString);
}
static int rgbSensor_enable_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_enable_proc_read, NULL);
}
static void create_rgbSensor_enable_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_enable_proc_open,
		.write = rgbSensor_enable_proc_write,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_enable", 0664, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_enable Interface---*/
/*+++BSP David proc rgbSensor_update_calibration_data Interface+++*/
static int rgbSensor_update_calibration_data_proc_read(struct seq_file *buf, void *v)
{
	int result = 1;
	rgbSensor_updateCalibrationData();
	RGB_DBG("%s: %d\n", __func__, result);
	return seq_printf(buf, "%d\n", result);
}
static int rgbSensor_update_calibration_data_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_update_calibration_data_proc_read, NULL);
}

static void create_rgbSensor_update_calibration_data_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_update_calibration_data_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_update_calibration_data", 0440, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_update_calibration_data Interface---*/
/*+++BSP David proc rgbSensor_itTime Interface+++*/
static int rgbSensor_itTime_proc_read(struct seq_file *buf, void *v)
{
	int result = -1;
	if (cm_lp_info != NULL) {
		result = cm_lp_info->it_time;
	}
	RGB_DBG("%s: %d\n", __func__, result);
	return seq_printf(buf, "%d\n", result);
}
static int rgbSensor_itTime_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_itTime_proc_read, NULL);
}

static ssize_t rgbSensor_itTime_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
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
	if (val >= 0 && val <= 5) {
		cm_lp_info->it_time = val;
		if (cm_lp_info != NULL && cm_lp_info->als_enable == 1) {
			RGB_DBG("%s: %d => %x\n", __func__, val, val << 4);
			_cm3323e_I2C_Mask_Write_Word(CM3323E_ADDR, CM3323E_CONF, CFG_IT_MASK, val << 4);
		} else{
			RGB_DBG_E("%s: als not enabled yet, val = %d\n", __func__, val);
		}
	} else{
		RGB_DBG_E("%s : parameter out of range(%d)\n", __func__, val);
	}

	return len;
}
static void create_rgbSensor_itTime_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_itTime_proc_open,
		.write = rgbSensor_itTime_proc_write,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_itTime", 0664, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_itTime Interface---*/
/*+++BSP David proc rgbSensor_raw_r Interface+++*/
static int rgbSensor_raw_r_proc_read(struct seq_file *buf, void *v)
{
	u16 adc_data = 0;

	get_rgb_data(RGB_DATA_R, false, &adc_data);
	RGB_DBG("%s: %u\n", __func__, adc_data);
	return seq_printf(buf, "%u\n", adc_data);
}
static int rgbSensor_raw_r_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_raw_r_proc_read, NULL);
}

static void create_rgbSensor_raw_r_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_raw_r_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_raw_r", 0444, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_raw_r Interface---*/
/*+++BSP David proc rgbSensor_raw_g Interface+++*/
static int rgbSensor_raw_g_proc_read(struct seq_file *buf, void *v)
{
	u16 adc_data = 0;

	get_rgb_data(RGB_DATA_G, false, &adc_data);
	RGB_DBG("%s: %u\n", __func__, adc_data);
	return seq_printf(buf, "%u\n", adc_data);
}
static int rgbSensor_raw_g_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_raw_g_proc_read, NULL);
}

static void create_rgbSensor_raw_g_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_raw_g_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_raw_g", 0444, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_raw_g Interface---*/
/*+++BSP David proc rgbSensor_raw_b Interface+++*/
static int rgbSensor_raw_b_proc_read(struct seq_file *buf, void *v)
{
	u16 adc_data = 0;

	get_rgb_data(RGB_DATA_B, false, &adc_data);
	RGB_DBG("%s: %u\n", __func__, adc_data);
	return seq_printf(buf, "%u\n", adc_data);
}
static int rgbSensor_raw_b_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_raw_b_proc_read, NULL);
}

static void create_rgbSensor_raw_b_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_raw_b_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_raw_b", 0444, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_raw_b Interface---*/
/*+++BSP David proc rgbSensor_raw_w Interface+++*/
static int rgbSensor_raw_w_proc_read(struct seq_file *buf, void *v)
{
	u16 adc_data = 0;

	get_rgb_data(RGB_DATA_W, false, &adc_data);
	RGB_DBG("%s: %u\n", __func__, adc_data);
	return seq_printf(buf, "%u\n", adc_data);
}
static int rgbSensor_raw_w_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_raw_w_proc_read, NULL);
}

static void create_rgbSensor_raw_w_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_raw_w_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_raw_w", 0444, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_raw_w Interface---*/
/*+++BSP David proc rgbSensor_r Interface+++*/
static int rgbSensor_r_proc_read(struct seq_file *buf, void *v)
{
	u16 adc_data = 0;

	get_rgb_data(RGB_DATA_R, true, &adc_data);
	RGB_DBG("%s: %u\n", __func__, adc_data);
	return seq_printf(buf, "%u\n", adc_data);
}
static int rgbSensor_r_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_r_proc_read, NULL);
}

static void create_rgbSensor_r_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_r_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_r", 0444, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_r Interface---*/
/*+++BSP David proc rgbSensor_g Interface+++*/
static int rgbSensor_g_proc_read(struct seq_file *buf, void *v)
{
	u16 adc_data = 0;

	get_rgb_data(RGB_DATA_G, true, &adc_data);
	RGB_DBG("%s: %u\n", __func__, adc_data);
	return seq_printf(buf, "%u\n", adc_data);
}
static int rgbSensor_g_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_g_proc_read, NULL);
}

static void create_rgbSensor_g_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_g_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_g", 0444, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_g Interface---*/
/*+++BSP David proc rgbSensor_b Interface+++*/
static int rgbSensor_b_proc_read(struct seq_file *buf, void *v)
{
	u16 adc_data = 0;

	get_rgb_data(RGB_DATA_B, true, &adc_data);
	RGB_DBG("%s: %u\n", __func__, adc_data);
	return seq_printf(buf, "%u\n", adc_data);
}
static int rgbSensor_b_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_b_proc_read, NULL);
}

static void create_rgbSensor_b_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_b_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_b", 0444, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_b Interface---*/
/*+++BSP David proc rgbSensor_w Interface+++*/
static int rgbSensor_w_proc_read(struct seq_file *buf, void *v)
{
	u16 adc_data = 0;

	get_rgb_data(RGB_DATA_W, true, &adc_data);
	RGB_DBG("%s: %u\n", __func__, adc_data);
	return seq_printf(buf, "%u\n", adc_data);
}
static int rgbSensor_w_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rgbSensor_w_proc_read, NULL);
}

static void create_rgbSensor_w_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  rgbSensor_w_proc_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/rgbSensor_w", 0444, NULL, &proc_fops);

	if (!proc_file) {
		RGB_DBG_E("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP David proc rgbSensor_w Interface---*/
#endif
/*---BSP David ASUS Interface---*/


static int rgbSensor_miscOpen(struct inode *inode, struct file *file)
{
	RGB_DBG("%s\n", __func__);
	rgbSensor_setEnable(true);
	return 0;
}

static int rgbSensor_miscRelease(struct inode *inode, struct file *file)
{
	RGB_DBG("%s\n", __func__);
	rgbSensor_setEnable(false);
	return 0;
}
static long rgbSensor_miscIoctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int it_time = 0;
	int ret = 0;
	int dataI[ASUS_RGB_SENSOR_DATA_SIZE];
	char dataS[ASUS_RGB_SENSOR_NAME_SIZE];
 	uint16_t adc_data;
	int l_debug_mode = 0;
	switch (cmd) {
		case ASUS_RGB_SENSOR_IOCTL_IT_SET:
			if (cm_lp_info->als_enable == 1) {
				ret = copy_from_user(&it_time, (int __user*)arg, sizeof(it_time));
				rgbSensor_itSet_ms(it_time);
				if (cm_lp_info->it_time >= 0 && cm_lp_info->it_time <= 5) {
					RGB_DBG_API("%s: cmd = IT_SET, time = %dms\n", __func__, 40 << cm_lp_info->it_time);
				} else{
					RGB_DBG_E("%s: cmd = IT_SET, time error(%d)\n", __func__, cm_lp_info->it_time);
				}
			} else{
				RGB_DBG_E("%s: als not enabled yet!\n", __func__);
			}
			break;
		case ASUS_RGB_SENSOR_IOCTL_DATA_READ:
			get_rgb_data(RGB_DATA_R, &adc_data, true);
			dataI[0] = adc_data;
			get_rgb_data(RGB_DATA_G, &adc_data, true);
			dataI[1] = adc_data;
			get_rgb_data(RGB_DATA_B, &adc_data, true);
			dataI[2] = adc_data;
			get_rgb_data(RGB_DATA_W, &adc_data, true);
			dataI[3] = adc_data;
			dataI[4] = 1;
			RGB_DBG_API("%s: cmd = DATA_READ, data[0] = %d, data[1] = %d, data[2] = %d, data[3] = %d, data[4] = %d\n"
				, __func__, dataI[0], dataI[1], dataI[2], dataI[3], dataI[4]);
			ret = copy_to_user((int __user*)arg, &dataI, sizeof(dataI));
			break;
		case ASUS_RGB_SENSOR_IOCTL_MODULE_NAME:
			snprintf(dataS, sizeof(dataS), "%s", CM3323E_I2C_NAME);
			RGB_DBG_API("%s: cmd = MODULE_NAME, name = %s\n", __func__, dataS);
			ret = copy_to_user((int __user*)arg, &dataS, sizeof(dataS));
			break;
		case ASUS_RGB_SENSOR_IOCTL_DEBUG_MODE:
			if (g_debugMode) {
				l_debug_mode = 1;
			} else{
				l_debug_mode = 0;
			}
			RGB_DBG_API("%s: cmd = DEBUG_MODE, result = %d\n", __func__, l_debug_mode);
			ret = copy_to_user((int __user*)arg, &l_debug_mode, sizeof(l_debug_mode));
			break;
		default:
			RGB_DBG_E("%s: default\n", __func__);
	}
	return 0;
}
static struct file_operations cm3323e_fops = {
  .owner = THIS_MODULE,
  .open = rgbSensor_miscOpen,
  .release = rgbSensor_miscRelease,
  .unlocked_ioctl = rgbSensor_miscIoctl,
  .compat_ioctl = rgbSensor_miscIoctl
};
struct miscdevice cm3323e_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = "asusRgbSensor",
  .fops = &cm3323e_fops
};
static int rgbSensor_miscRegister(void)
{
	int rtn = 0;
	rtn = misc_register(&cm3323e_misc);
	if (rtn < 0) {
		RGB_DBG_E("[%s] Unable to register misc deive\n", __func__);
		misc_deregister(&cm3323e_misc);
	}
	return rtn;
}

static int32_t rgbSensor_getDt(struct device_node *of_node,
		struct cm3323e_info *fctrl)
{
	struct msm_rgb_sensor_vreg *vreg_cfg = NULL;
	int rc;

	if (of_find_property(of_node,
			"qcom,cam-vreg-name", NULL)) {
		vreg_cfg = &fctrl->vreg_cfg;
		rc = msm_camera_get_dt_vreg_data(of_node,
			&vreg_cfg->cam_vreg, &vreg_cfg->num_vreg);
		if (rc < 0) {
			RGB_DBG_E("%s: failed rc %d\n", __func__, rc);
		}
	} else{
		RGB_DBG_E("%s: can't find regulator\n", __func__);
		rc = -1;
	}
	return rc;
}
static int cm3323e_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm3323e_info *lpi;

	RGB_DBG("%s start\n", __func__);

	lpi = kzalloc(sizeof(struct cm3323e_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	lpi->i2c_client = client;

	i2c_set_clientdata(client, lpi);

	lpi->power = NULL; //if necessary, add power function here for sensor chip

	lpi->polling_delay = msecs_to_jiffies(LS_POLLING_DELAY);
	lpi->using_calib = true;
	cm_lp_info = lpi;
	g_debugMode = false;
	rgbSensor_getDt(client->dev.of_node, lpi);
	mutex_init(&als_enable_mutex);
	ret = rgbSensor_setup(lpi);
	if (ret < 0) {
		RGB_DBG_E("%s: rgbSensor_setup error!\n", __func__);
		goto err_rgbSensor_setup;
	}
	ret = rgbSensor_miscRegister();
	if (ret < 0) {
		goto err_rgbSensor_miscRegister;
	}

	create_rgbSensor_dump_proc_file();
	create_rgbSensor_status_proc_file();
	create_asusRgbCalibEnable_proc_file();
	create_asusRgbDebug_proc_file();
	create_asusRgbSetIT_proc_file();

#if 0
	create_rgbSensor_enable_proc_file();
	create_rgbSensor_itTime_proc_file();
	create_rgbSensor_update_calibration_data_proc_file();
	create_rgbSensor_r_proc_file();
	create_rgbSensor_g_proc_file();
	create_rgbSensor_b_proc_file();
	create_rgbSensor_w_proc_file();
	create_rgbSensor_raw_r_proc_file();
	create_rgbSensor_raw_g_proc_file();
	create_rgbSensor_raw_b_proc_file();
	create_rgbSensor_raw_w_proc_file();
#endif

	RGB_DBG("%s: Probe success!\n", __func__);

	return ret;

err_rgbSensor_miscRegister:
err_rgbSensor_setup:
	rgbSensor_setEnable(false);
	mutex_destroy(&als_enable_mutex);
	kfree(lpi);
	return ret;
}

static int cm3323e_remove(struct i2c_client *client)
{
	misc_deregister(&cm3323e_misc);
	return 0;
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
	.remove = cm3323e_remove,
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
