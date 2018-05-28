/*
 * msm_debugfs.c
 *
 *  Created on: Jan 13, 2015
 *      Author: charles
 *      Email: Weiche_Tsai@asus.com
 */
#include <media/v4l2-device.h>
#include "msm_debugfs.h"
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define MAX_CAMERA 4
static int number_of_camera = 0;
static struct debugfs dbgfs[MAX_CAMERA];
static struct msm_actuator_ctrl_t *s_ctrl_vcm;
static struct msm_flash_ctrl_t *s_ctrl_flash;
static struct msm_ois_ctrl_t *o_ctrl_ois;
static struct msm_sensor_ctrl_t *s_ctrl_rear;
static struct msm_sensor_ctrl_t *s_ctrl_front;
static struct msm_sensor_ctrl_t *s_ctrl_imx318;

static int imx318_power_on;
#define OTP_SIZE 32
static unsigned char imx318_otp[OTP_SIZE];
static unsigned char ov8856_otp[OTP_SIZE];
static unsigned char ov8856_otp_1[OTP_SIZE];
static unsigned char ov8856_otp_2[OTP_SIZE];
static unsigned char ov7251_otp[OTP_SIZE];
#define	CAMERA_RES_PROC_FILE		"driver/camera_res"

static int ois_mode_err;
static int ois_accel_gain_err;
#define	OIS_RW_PROC_FILE		"driver/ois_rw"

#define DEFAULT_UID "000000000000000000000000"
#define DEFAULT_OTP "NO available OTP"
#define MAX_TORCH_CURRENT 200
#define FLASH_MODULE_DEFAULT_TORCH_CURRENT 200
#define FLASH_MODULE_MAX_TORCH_CURRENT 250
#define FLASH_MODULE_MIN_TORCH_CURRENT 25
#define FLASH_MODULE_STEP_TORCH_CURRENT 25


#define MAX_FLASH_CURRENT 900
#define FLASH_MODULE_DEFAULT_CURRENT 1000
#define FLASH_MODULE_MAX_CURRENT 1500
#define FLASH_MODULE_MIN_CURRENT 250
#define FLASH_MODULE_STEP_CURRENT 50


/* eeprom max memory size 131072 = 128KB */
#define MAX_eeprom 131072
#define MAX_I2C_retry 10;
static struct msm_sensor_ctrl_t *s_ctrl_depth;
static uint8_t *s_ptr;
static uint8_t *pmd;
static int ver;
static int p_size,p_size2;
static int SPC_size,Pro_size,Len_size,Effi_size;
static uint8_t *lensdata;
static uint8_t *efficiency;
static uint8_t *prod_code;
static int VendorId;

typedef enum{
  FLASH_TURN_OFF_ALL,
  FLASH_SET_CURRENT_FLASH,
  FLASH_SET_CURRENT_TORCH,
  FLASH_TURN_ON_HIGH,
  FLASH_TURN_ON_LOW,
  FLASH_DUMP_I2C_REG
}CFG_CAMERA_Flash_INTERFACE;

typedef enum{
  FLASH_LOW_TURN_ON = 1,
  FLASH_HIGH_TURN_ON,
  FLASH_I2C_MANUAL,
  FLASH_TURN_OFF
}CFG_CAMERA_Flash;

static int dbg_dump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_front_turn_on_power_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)

{
	char debug_buf[256];
	int cnt;
	int val1 = 0;
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info;
	struct msm_camera_i2c_client *sensor_i2c_client;
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%d", &val1);
	pr_err("%s: turn_on_power = %d\n", __func__, val1);
  power_info = &s_ctrl_front->sensordata->power_info;
  sensor_i2c_client = s_ctrl_front->sensor_i2c_client;
	if (val1 == 1) {
    rc = msm_camera_power_up(power_info, s_ctrl_front->sensor_device_type,
			sensor_i2c_client);
	}
  else {
    rc = msm_camera_power_down(power_info, s_ctrl_front->sensor_device_type,
		sensor_i2c_client);
  }

	*ppos += count;	/* increase offset */
	return count;
}

static const struct file_operations dbg_front_turn_on_power_fops = {
	.open		= dbg_dump_open,
	.write	= dbg_front_turn_on_power_write,
};

static ssize_t dbg_rear_turn_on_power_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)

{
	char debug_buf[256];
	int cnt;
	int val1 = 0;
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info;
	struct msm_camera_i2c_client *sensor_i2c_client;
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%d", &val1);
	pr_err("%s: turn_on_power = %d\n", __func__, val1);
  power_info = &s_ctrl_rear->sensordata->power_info;
  sensor_i2c_client = s_ctrl_rear->sensor_i2c_client;
	if (val1 == 1) {
    rc = msm_camera_power_up(power_info, s_ctrl_rear->sensor_device_type,
			sensor_i2c_client);
	}
  else {
    rc = msm_camera_power_down(power_info, s_ctrl_rear->sensor_device_type,
		sensor_i2c_client);
  }

	*ppos += count;	/* increase offset */
	return count;
}

static const struct file_operations dbg_rear_turn_on_power_fops = {
	.open		= dbg_dump_open,
	.write	= dbg_rear_turn_on_power_write,
};

static ssize_t dbg_dump_imx318_otp_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = 0;
	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", imx318_otp[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", imx318_otp[i]);
	}
	pr_err("OTP=%s\n", debug_buf);

#if 0
	len = snprintf(bp, dlen,
		"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		otp_data.af_inf[0], otp_data.af_inf[1],
		otp_data.af_30cm[0], otp_data.af_30cm[1],
		otp_data.af_5cm[0], otp_data.af_5cm[1],
		otp_data.start_current[0], otp_data.start_current[1],
		otp_data.module_id, otp_data.vendor_id);
#endif

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_imx318_otp_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_imx318_otp_read,
};

static ssize_t dbg_dump_imx318_uid_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = 0;
	for (i = 0; i < 12; i++)
		len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "%02X", imx318_otp[10 + i]);
	pr_debug("UID=%s\n", debug_buf);

#if 0
	len = snprintf(bp, dlen,
		"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		otp_data.dc[0], otp_data.dc[1],
		otp_data.dc[2], otp_data.dc[3],
		otp_data.sn[0], otp_data.sn[1],
		otp_data.sn[2], otp_data.sn[3],
		otp_data.pn[0], otp_data.pn[1],
		otp_data.pn[2], otp_data.pn[3]);
#endif

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_imx318_uid_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_imx318_uid_read,
};

static ssize_t dbg_dump_ov8856_otp_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[768];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = 0;
	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", ov8856_otp[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov8856_otp[i]);
	}
	len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "\n");
	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", ov8856_otp_1[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov8856_otp_1[i]);
	}
	len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "\n");
	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", ov8856_otp_2[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov8856_otp_2[i]);
	}
	pr_debug("OTP=%s\n", debug_buf);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_ov8856_otp_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_ov8856_otp_read,
};

static ssize_t dbg_dump_ov8856_uid_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = 0;
	for (i = 0; i < 12; i++)
		len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "%02X", ov8856_otp[10 + i]);
	pr_debug("UID=%s\n", debug_buf);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_ov8856_uid_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_ov8856_uid_read,
};

static ssize_t dbg_dump_ov7251_otp_read(
        struct file *file,
        char __user *buf,
        size_t count,
        loff_t *ppos)
{
        int len = 0;
        int tot = 0;
        char debug_buf[256];
        int dlen = sizeof(debug_buf);
        char *bp = debug_buf;
        int i;

        pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
                __func__, buf, (int)count, ppos, (int)*ppos);

        if (*ppos)
                return 0;       /* the end */

        len = 0;
        for (i = 0; i < OTP_SIZE; i++)
                len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov7251_otp[i]);
        pr_debug("OTP=%s\n", debug_buf);

        tot += len; bp += len; dlen -= len;

        if (copy_to_user(buf, debug_buf, tot))
                return -EFAULT;

        if (tot < 0)
                return 0;
        *ppos += tot;   /* increase offset */
        return tot;
}

static const struct file_operations dbg_dump_ov7251_otp_fops = {
        .open           = dbg_dump_open,
        .read           = dbg_dump_ov7251_otp_read,
};

static ssize_t dbg_dump_ov7251_uid_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
//read fisheye vendor id
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
//	int i;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = 0;

		len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "%02X", ov7251_otp[30]);
	pr_debug("Vendor ID=%s\n", debug_buf);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_ov7251_uid_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_ov7251_uid_read,
};


static ssize_t dbg_default_otp_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen,"%s\n", DEFAULT_OTP);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_default_otp_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_default_otp_read,
};

static ssize_t dbg_default_uid_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen,"%s\n", DEFAULT_UID);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_default_uid_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_default_uid_read,
};

static ssize_t dbg_dump_vcm_test_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	unsigned int val1 = 0, val2 = 0;
	int ret = 0;
	uint16_t status = 0;
	//uint16_t read_data = 0;
	int i=0;

	struct msm_camera_i2c_client client = s_ctrl_vcm->i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = s_ctrl_vcm->i2c_client.i2c_func_tbl->i2c_write;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = s_ctrl_vcm->i2c_client.i2c_func_tbl->i2c_read;

  if (!imx318_power_on) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}
	/*
	pr_info("%s: buf=%p, count=%d, ppos=%p\n", __func__, buf, count, ppos);
	*/
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%x %x", &val1, &val2);
	pr_err("%s: val1=0x%x, val2=0x%x\n", __func__, val1, val2);

	if (s_ctrl_vcm==NULL)
		pr_err("%s: s_ctrl_vcm is null\n", __func__);
	if (i2c_write==NULL)
		pr_err("%s: i2c_write is null\n", __func__);

	client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	i2c_read(&client, 0x3304, &status, 1);
	pr_err("%s: 0x3304 status=%d\n", __func__, status);

	ret |= i2c_write(&client, 0xB315, 0x00, 1);	// 1 means data type is byte

	i2c_read(&client, 0xB973, &status, 1);
	pr_err("%s: init status=%d\n", __func__, status);
	if (status == 4) {
		pr_err("%s: communication reset\n", __func__);
		ret |= i2c_write(&client, 0x3370, 0x84, 1);	// 1 means data type is byte
		for (i=0; i<300; i++) {
			i2c_read(&client, 0x3370, &status, 1);
			pr_err("%s: %d, init send status polling=0x%x\n", __func__, i, status);
			if ((status&0x80)==0)
				break;
			usleep_range(10000, 11000);
		}
	}

#if 0 /* test read data*/
	ret |= i2c_write(&client, 0x3373, 0x01, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3374, 0x02, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3378, 0x72, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3379, 0xF0, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3370, 0x82, 1);	// 1 means data type is byte
	i2c_read(&client, 0xB973, &status, 1);
	i2c_read(&client, 0x337a, &read_data, 1);
	pr_err("%s: read1 status=%d, read_data=%x\n", __func__, status, read_data);
#endif

	ret |= i2c_write(&client, 0x3374, 0x03, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3378, 0x72, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3379, 0xE0, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x337a, 0x01, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3370, 0x81, 1);	// 1 means data type is byte
	for (i=0; i<300; i++) {
		i2c_read(&client, 0x3370, &status, 1);
		pr_err("%s: %d, init send status polling=0x%x\n", __func__, i, status);
		if ((status&0x80)==0)
			break;
		usleep_range(10000, 11000);
	}
	i2c_read(&client, 0xB973, &status, 1);
	pr_err("%s: init status=%d\n", __func__, status);
	if (status == 4) {
		pr_err("%s: communication reset\n", __func__);
		ret |= i2c_write(&client, 0x3370, 0x84, 1);	// 1 means data type is byte
		for (i=0; i<300; i++) {
			i2c_read(&client, 0x3370, &status, 1);
			pr_err("%s: %d, init send status polling=0x%x\n", __func__, i, status);
			if ((status&0x80)==0)
				break;
			usleep_range(10000, 11000);
		}
	}

	ret |= i2c_write(&client, 0x3374, 0x04, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3378, 0x72, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3379, 0xA0, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x337a, val1, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x337b, val2, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3370, 0x81, 1);	// 1 means data type is byte
	for (i=0; i<300; i++) {
		i2c_read(&client, 0x3370, &status, 1);
		pr_err("%s: %d, command send status polling=0x%x\n", __func__, i, status);
		if ((status&0x80)==0)
			break;
		usleep_range(10000, 11000);
	}
	i2c_read(&client, 0xB973, &status, 1);
	pr_err("%s: init status=%d\n", __func__, status);
	if (status == 4) {
		pr_err("%s: communication reset\n", __func__);
		ret |= i2c_write(&client, 0x3370, 0x84, 1);	// 1 means data type is byte
		for (i=0; i<300; i++) {
			i2c_read(&client, 0x3370, &status, 1);
			pr_err("%s: %d, init send status polling=0x%x\n", __func__, i, status);
			if ((status&0x80)==0)
				break;
			usleep_range(10000, 11000);
		}
	}

	return count;
}

static const struct file_operations dbg_dump_vcm_test_fops = {
	.open		= dbg_dump_open,
	.write		= dbg_dump_vcm_test_write,
};

static ssize_t dbg_ois_mode_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	unsigned int ois_mode = 0;

	struct msm_camera_i2c_client client = o_ctrl_ois->i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_write;

  if (!imx318_power_on || o_ctrl_ois == NULL) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%d", &ois_mode);
	pr_err("%s: set ois_mode to %d\n", __func__, ois_mode);

	if (o_ctrl_ois==NULL)
		pr_err("%s: o_ctrl_ois is null\n", __func__);
	if (i2c_write==NULL)
		pr_err("%s: i2c_write is null\n", __func__);

	switch (ois_mode) {
	case 0: /* turn off ois, only centering on */
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0C0C, 2);
		if (ois_mode_err < 0)
			pr_err("ois_mode %d set fail\n", ois_mode);
		break;
	case 1: /* movie mode */
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0C0C, 2);
		ois_mode_err |= i2c_write(&client, 0x8436, 0xFd7F, 2);
		ois_mode_err |= i2c_write(&client, 0x8440, 0xF07F, 2);
		ois_mode_err |= i2c_write(&client, 0x8443, 0xB41E, 2);
		ois_mode_err |= i2c_write(&client, 0x841B, 0x8000, 2);
		ois_mode_err |= i2c_write(&client, 0x84B6, 0xFd7F, 2);
		ois_mode_err |= i2c_write(&client, 0x84C0, 0xF07F, 2);
		ois_mode_err |= i2c_write(&client, 0x84C3, 0xB41E, 2);
		ois_mode_err |= i2c_write(&client, 0x849B, 0x8000, 2);
		ois_mode_err |= i2c_write(&client, 0x8438, 0x020D, 2);
		ois_mode_err |= i2c_write(&client, 0x84B8, 0x020D, 2);
		ois_mode_err |= i2c_write(&client, 0x8447, 0x862E, 2);
		ois_mode_err |= i2c_write(&client, 0x84C7, 0x862E, 2);
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0D0D, 2);
		if (ois_mode_err < 0)
			pr_err("ois_mode %d set fail\n", ois_mode);
		break;
	case 2: /* still mode */
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0C0C, 2);
		ois_mode_err |= i2c_write(&client, 0x8436, 0xFd7F, 2);
		ois_mode_err |= i2c_write(&client, 0x8440, 0xF07F, 2);
		ois_mode_err |= i2c_write(&client, 0x8443, 0xB41E, 2);
		ois_mode_err |= i2c_write(&client, 0x841B, 0x8000, 2);
		ois_mode_err |= i2c_write(&client, 0x84B6, 0xFd7F, 2);
		ois_mode_err |= i2c_write(&client, 0x84C0, 0xF07F, 2);
		ois_mode_err |= i2c_write(&client, 0x84C3, 0xB41E, 2);
		ois_mode_err |= i2c_write(&client, 0x849B, 0x8000, 2);
		ois_mode_err |= i2c_write(&client, 0x8438, 0x020D, 2);
		ois_mode_err |= i2c_write(&client, 0x84B8, 0x020D, 2);
		ois_mode_err |= i2c_write(&client, 0x8447, 0x862E, 2);
		ois_mode_err |= i2c_write(&client, 0x84C7, 0x862E, 2);
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0D0D, 2);
		if (ois_mode_err < 0)
			pr_err("ois_mode %d set fail\n", ois_mode);
		break;
	case 3: /* test mode */
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0C0C, 2);
		ois_mode_err |= i2c_write(&client, 0x8436, 0xFF7F, 2);
		ois_mode_err |= i2c_write(&client, 0x8440, 0xFF7F, 2);
		ois_mode_err |= i2c_write(&client, 0x8443, 0xFF7F, 2);
		ois_mode_err |= i2c_write(&client, 0x841B, 0x8000, 2);
		ois_mode_err |= i2c_write(&client, 0x84B6, 0xFF7F, 2);
		ois_mode_err |= i2c_write(&client, 0x84C0, 0xFF7F, 2);
		ois_mode_err |= i2c_write(&client, 0x84C3, 0xFF7F, 2);
		ois_mode_err |= i2c_write(&client, 0x849B, 0x8000, 2);
		ois_mode_err |= i2c_write(&client, 0x8438, 0x5209, 2);
		ois_mode_err |= i2c_write(&client, 0x84B8, 0x5209, 2);
		ois_mode_err |= i2c_write(&client, 0x8447, 0xF240, 2);
		ois_mode_err |= i2c_write(&client, 0x84C7, 0xF240, 2);
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0D0D, 2);
		if (ois_mode_err < 0)
			pr_err("ois_mode %d set fail\n", ois_mode);
		break;
	default:
		pr_err("do not support this ois_mode %d\n", ois_mode);
		break;
	}

	return count;
}

static ssize_t dbg_ois_mode_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen,"%d\n", ois_mode_err);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}


static const struct file_operations dbg_ois_mode_fops = {
	.open		= dbg_dump_open,
	.write		= dbg_ois_mode_write,
	.read		= dbg_ois_mode_read,
};

static ssize_t dbg_ois_gyro_x_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int ret = 0;
	uint16_t gyro_x = 0;

	struct msm_camera_i2c_client client = o_ctrl_ois->i2c_client;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_read;

  if (!imx318_power_on || o_ctrl_ois == NULL) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	if (*ppos)
		return 0;	/* the end */

	ret |= i2c_read(&client, 0x8455, &gyro_x, 2);
	if (ret < 0)
		pr_err("%s: ret=%d\n", __func__, ret);

	len = snprintf(bp, dlen,"%d\n", gyro_x);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_ois_gyro_x_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_ois_gyro_x_read,
};

static ssize_t dbg_ois_gyro_y_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int ret = 0;
	uint16_t gyro_y = 0;

	struct msm_camera_i2c_client client = o_ctrl_ois->i2c_client;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_read;

  if (!imx318_power_on || o_ctrl_ois == NULL) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}
	if (*ppos)
		return 0;	/* the end */

	ret |= i2c_read(&client, 0x8456, &gyro_y, 2);
	if (ret < 0)
		pr_err("%s: ret=%d\n", __func__, ret);

	len = snprintf(bp, dlen,"%d\n", gyro_y);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_ois_gyro_y_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_ois_gyro_y_read,
};

static ssize_t dbg_ois_accel_gain_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	unsigned int accel_gain_x = 0, accel_gain_y = 0;
	unsigned int accel_gain_x_swap = 0, accel_gain_y_swap = 0;

	struct msm_camera_i2c_client client = o_ctrl_ois->i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_write;

	if (!imx318_power_on || o_ctrl_ois == NULL) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%d %d", &accel_gain_x, &accel_gain_y);
	pr_err("%s: set accel_gain_x to %d, accel_gain_y=%d\n", __func__, accel_gain_x, accel_gain_y);

	if (o_ctrl_ois==NULL)
		pr_err("%s: o_ctrl_ois is null\n", __func__);
	if (i2c_write==NULL)
		pr_err("%s: i2c_write is null\n", __func__);

	/* should write low byte first */
	accel_gain_x_swap = ((accel_gain_x & 0xFF) << 8) | ((accel_gain_x & 0xFF00) >> 8 );
	accel_gain_y_swap = ((accel_gain_y & 0xFF) << 8) | ((accel_gain_y & 0xFF00) >> 8 );

	ois_accel_gain_err |= i2c_write(&client, 0x828B, accel_gain_x_swap, 2);
	ois_accel_gain_err |= i2c_write(&client, 0x82CB, accel_gain_y_swap, 2);
	if (ois_accel_gain_err < 0)
		pr_err("ois_accel_gain set fail\n");

	return count;
}

static ssize_t dbg_ois_accel_gain_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen,"%d\n", ois_accel_gain_err);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_ois_accel_gain_fops = {
	.open		= dbg_dump_open,
	.write		= dbg_ois_accel_gain_write,
	.read		= dbg_ois_accel_gain_read,
};

static int g_reg, g_value;
#define DBG_TXT_BUF_SIZE 256
static char debugTxtBuf[DBG_TXT_BUF_SIZE];
static int ois_rw_proc_read(struct seq_file *buf, void *v)
{
	pr_err("ois_rw_proc_read reg 0x%x=0x%x\n", g_reg, g_value);
	seq_printf(buf, "%x\n", g_value);
	return 0;
}

static ssize_t ois_rw_proc_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	int reg = -1, value = -1, temp;
	int rc, len;

	struct msm_camera_i2c_client client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type);
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type);

	if (!imx318_power_on || o_ctrl_ois == NULL) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	client = o_ctrl_ois->i2c_client;
	i2c_write = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_write;
	i2c_read = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_read;

	len = (count > DBG_TXT_BUF_SIZE - 1) ? (DBG_TXT_BUF_SIZE-1) : (count);
	if (copy_from_user(debugTxtBuf, buf, len))
		return -EFAULT;
	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%x %x", &reg, &value);
	*ppos=len;
	if (reg != -1 && value != -1) {
		pr_err("ois write reg=0x%x value=0x%x\n", reg, value);
		/* write to ois IC low byte first */
		temp = ((value & 0xFF) << 8) | ((value & 0xFF00) >> 8);
		rc = i2c_write(&client, (uint32_t)reg, (uint16_t)temp, 2);
		if (rc < 0) {
			pr_err("%s: failed to write 0x%x = 0x%x\n",
				 __func__, reg, value);
			return rc;
		}
	} else if (reg != -1) {
		rc = i2c_read(&client, (uint32_t)reg, (uint16_t *)&value, 2);
		pr_err("ois read reg=0x%x value=0x%x\n", reg, value);
		if (rc < 0) {
			pr_err("%s: failed to read 0x%x\n",
				 __func__, reg);
			return rc;
		}
	}
	g_reg = reg;
	g_value = value & 0xFFFF;

	return len;
}

static int ois_rw_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_rw_proc_read, NULL);
}

static const struct file_operations ois_rw_proc_fops = {
	.owner = THIS_MODULE,
	.open = ois_rw_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = ois_rw_proc_write,
};
static int g_ois_read_times_status;
static int ois_read_times_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", g_ois_read_times_status);
	return 0;
}

static ssize_t ois_read_times_proc_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	int times = -1, i;
	int rc, len;
	static uint16_t OIS_GYRO_ACC_VALUE[4] = {
		0x8455,  /*Gyro X*/
		0x8456,  /*Gyro Y*/
		0x8280,  /*ACC X*/
		0x82C0   /*ACC Y*/
	};
	int gyro_x = 0, gyro_y = 0, accel_x = 0, accel_y = 0;
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buffer[5];
	char *filename = "/data/data/OIS_debug";

	struct msm_camera_i2c_client client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type);
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type);

	g_ois_read_times_status = 1;

	if (!imx318_power_on || o_ctrl_ois == NULL) {
		pr_err("please power on imx318 first");
		g_ois_read_times_status = 0;
		return -EPERM;
	}

	client = o_ctrl_ois->i2c_client;
	i2c_write = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_write;
	i2c_read = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_read;

	len = (count > DBG_TXT_BUF_SIZE - 1) ? (DBG_TXT_BUF_SIZE-1) : (count);
	if (copy_from_user(debugTxtBuf, buf, len)) {
		g_ois_read_times_status = 0;
		return -EFAULT;
	}
	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%d", &times);
	*ppos=len;

	if (times < 0) {
		pr_err("%s: times %d is invalid\n", __func__, times);
		g_ois_read_times_status = 0;
		return false;
	}

	/* Open file */
	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("%s: openfail line = %d\n", __func__, __LINE__);
		g_ois_read_times_status = 0;
		return false;
	}
	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		for (i = 0; i < times; i++) {
			rc = i2c_read(&client, (uint32_t)OIS_GYRO_ACC_VALUE[0], (uint16_t *)&gyro_x, 2);
			pr_debug("ois read reg=0x%x value=0x%x\n", OIS_GYRO_ACC_VALUE[0], gyro_x);
			if (rc < 0) {
				pr_err("%s: failed to read 0x%x\n", __func__, OIS_GYRO_ACC_VALUE[0]);
				g_ois_read_times_status = 0;
				return rc;
			}
			rc = i2c_read(&client, (uint32_t)OIS_GYRO_ACC_VALUE[1], (uint16_t *)&gyro_y, 2);
			pr_debug("ois read reg=0x%x value=0x%x\n", OIS_GYRO_ACC_VALUE[1], gyro_y);
			if (rc < 0) {
				pr_err("%s: failed to read 0x%x\n", __func__, OIS_GYRO_ACC_VALUE[1]);
				g_ois_read_times_status = 0;
				return rc;
			}
			rc = i2c_read(&client, (uint32_t)OIS_GYRO_ACC_VALUE[2], (uint16_t *)&accel_x, 2);
			pr_debug("ois read reg=0x%x value=0x%x\n", OIS_GYRO_ACC_VALUE[2], accel_x);
			if (rc < 0) {
				pr_err("%s: failed to read 0x%x\n", __func__, OIS_GYRO_ACC_VALUE[2]);
				g_ois_read_times_status = 0;
				return rc;
			}
			rc = i2c_read(&client, (uint32_t)OIS_GYRO_ACC_VALUE[3], (uint16_t *)&accel_y, 2);
			pr_debug("ois read reg=0x%x value=0x%x\n", OIS_GYRO_ACC_VALUE[3], accel_y);
			if (rc < 0) {
				pr_err("%s: failed to read 0x%x\n", __func__, OIS_GYRO_ACC_VALUE[3]);
				g_ois_read_times_status = 0;
				return rc;
			}

			sprintf(buffer, "%04x", gyro_x);
			buffer[4] = ',';
			fp->f_op->write(fp, buffer, 5, &fp->f_pos);
			sprintf(buffer, "%04x", gyro_y);
			buffer[4] = ',';
			fp->f_op->write(fp, buffer, 5, &fp->f_pos);
			sprintf(buffer, "%04x", accel_x);
			buffer[4] = ',';
			fp->f_op->write(fp, buffer, 5, &fp->f_pos);
			sprintf(buffer, "%04x", accel_y);
			buffer[4] = '\n';
			fp->f_op->write(fp, buffer, 5, &fp->f_pos);
		}
	} else {
		/* Set addr_limit of the current process back to its own */
		set_fs(old_fs);

		/* Close file */
		filp_close(fp, NULL);
		pr_err("%s: f_op = null or write = null, fail line = %d\n", __func__, __LINE__);

		return false;
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	/* Close file */
	filp_close(fp, NULL);

	return len;
}

static int ois_read_times_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_read_times_proc_read, NULL);
}

static const struct file_operations ois_read_times_proc_fops = {
	.owner = THIS_MODULE,
	.open = ois_read_times_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = ois_read_times_proc_write,
};

static int camera_res_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "8M-23M");
	return 0;
}
static int camera_res_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, camera_res_proc_read, NULL);
}

static const struct file_operations camera_res_proc_fops = {
	.owner = THIS_MODULE,
	.open = camera_res_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int imx318_rw_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "reg 0x%x=0x%x\n", g_reg, g_value);
	return 0;
}

static ssize_t imx318_rw_proc_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	int reg = -1, value = -1;
	int rc, len;
	struct msm_camera_i2c_client *client = s_ctrl_imx318->sensor_i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = s_ctrl_imx318->sensor_i2c_client->i2c_func_tbl->i2c_write;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = s_ctrl_imx318->sensor_i2c_client->i2c_func_tbl->i2c_read;

	if (!imx318_power_on) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	len = (count > DBG_TXT_BUF_SIZE - 1) ? (DBG_TXT_BUF_SIZE-1) : (count);
	if (copy_from_user(debugTxtBuf, buf, len))
		return -EFAULT;
	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%x %x", &reg, &value);
	*ppos=len;
	if (reg != -1 && value != -1) {
		pr_info("imx318 write reg=0x%x value=0x%x\n", reg, value);
		rc = i2c_write(client, (uint32_t)reg, (uint16_t)value, 1);
		if (rc < 0) {
			pr_err("%s: failed to write 0x%x = 0x%x\n",
				 __func__, reg, value);
			return rc;
		}
	} else if (reg != -1) {
		rc = i2c_read(client, (uint32_t)reg, (uint16_t *)&value, 1);
		pr_info("imx318 read reg=0x%x value=0x%x\n", reg, value);
		if (rc < 0) {
			pr_err("%s: failed to read 0x%x\n",
				 __func__, reg);
			return rc;
		}
	}
	g_reg = reg;
	g_value = value & 0xFF;

	return len;
}

static int imx318_rw_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, imx318_rw_proc_read, NULL);
}

static const struct file_operations imx318_rw_proc_fops = {
	.owner = THIS_MODULE,
	.open = imx318_rw_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = imx318_rw_proc_write,
};


static int rear_otp_proc_read(struct seq_file *buf, void *v)
{
	int len = 0;
	char debug_buf[256];
	int i;

	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", imx318_otp[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", imx318_otp[i]);
	}
	pr_debug("OTP=%s\n", debug_buf);

	seq_printf(buf, "%s", debug_buf);
	return 0;
}

static int rear_otp_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, rear_otp_proc_read, NULL);
}

static const struct file_operations rear_otp_proc_fops = {
	.owner = THIS_MODULE,
	.open = rear_otp_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int front_otp_proc_read(struct seq_file *buf, void *v)
{
	int len = 0;
	char debug_buf[768];
	int i;

	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", ov8856_otp[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov8856_otp[i]);
	}
	len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "\n");
	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", ov8856_otp_1[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov8856_otp_1[i]);
	}
	len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "\n");
	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", ov8856_otp_2[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov8856_otp_2[i]);
	}
	pr_debug("OTP=%s\n", debug_buf);

	seq_printf(buf, "%s", debug_buf);
	return 0;
}

static int front_otp_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, front_otp_proc_read, NULL);
}

static const struct file_operations front_otp_proc_fops = {
	.owner = THIS_MODULE,
	.open = front_otp_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int rear_thermal_proc_read(struct seq_file *buf, void *v)
{
	int ret = 0;
	uint16_t temp = 0;
	int temp_translate = 0;

	struct msm_camera_i2c_client *client = s_ctrl_imx318->sensor_i2c_client;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = s_ctrl_imx318->sensor_i2c_client->i2c_func_tbl->i2c_read;

	if (!imx318_power_on) {
		seq_printf(buf, "0\n");
		return 0;
	}

	ret |= i2c_read(client, 0x013a, &temp, 1);
	if (ret < 0)
		pr_err("%s: ret=%d\n", __func__, ret);

	if (temp <= 0x77)
		temp_translate = (int)temp;
	else if (temp >= 0x78 && temp <= 0x7f)
		temp_translate = 120;
	else if (temp >= 0x81 && temp <= 0xEC)
		temp_translate = -20;
	else if (temp >= 0xED && temp <= 0xFF)
		temp_translate = temp - 256;
	pr_debug("temp=0x%x, temp_translate=%d\n", temp, temp_translate);

	seq_printf(buf, "%d\n", temp_translate);
	return 0;
}

static int rear_thermal_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, rear_thermal_proc_read, NULL);
}

static const struct file_operations rear_thermal_proc_fops = {
	.owner = THIS_MODULE,
	.open = rear_thermal_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int i2c_read_thu_imx318(uint8_t *value, int num, uint8_t *read_data, int read_num) {
	int ret = 0, i = 0;
	uint16_t status_0x3370 = 0;
	uint16_t status = 0;
	struct msm_camera_i2c_client *client = s_ctrl_rear->sensor_i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = s_ctrl_rear->sensor_i2c_client->i2c_func_tbl->i2c_write;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = s_ctrl_rear->sensor_i2c_client->i2c_func_tbl->i2c_read;

	/* to make sure the channel is ok */
	i2c_read(client, 0xB973, &status, 1);
	if (status == 4) { // 2 is normal, 4 is communication error
		pr_err("%s: communication reset\n", __func__);
		ret |= i2c_write(client, 0x3370, 0x84, 1);
		for (i=0; i<300; i++) {
			i2c_read(client, 0x3370, &status, 1);
			pr_err("%s: %d, init send status polling=0x%x\n", __func__, i, status);
			if ((status&0x80)==0)
				break;
			usleep_range(10000, 11000);
		}
	}

	/* start to transfer data to imx318, then imx318 send to vcm */
	ret |= i2c_write(client, 0x3373, read_num, 1); // 1 means data type is byte
	ret |= i2c_write(client, 0x3374, num, 1);
	for (i = 0; i < num; i++) {
		//pr_err("%d write 0x%x 0x%x\n", i, 0x3378+i, (uint16_t)value[i]);
		ret |= i2c_write(client, 0x3378+i, (uint16_t)value[i], 1);
	}
	ret |= i2c_write(client, 0x3370, 0x82, 1);
	usleep_range(100, 110);

	/* polling to make sure the communication is done */
	for (i = 0; i < 300; i++) {
		i2c_read(client, 0x3370, &status_0x3370, 1);
		if ((status&0x80)==0)
			break;
		pr_err("%s: %d, command send status polling=0x%x\n", __func__, i, status_0x3370);
		usleep_range(10000, 11000);
	}
	/* read data from vcm */
	for (i = 0; i < read_num; i++) {
		i2c_read(client, 0x3378+num+i, (uint16_t*)&(read_data[i]), 1);
		pr_debug("%s: read_data[%d]=0x%x\n", __func__, i, read_data[i]);
	}

	return ret;
}

static int i2c_write_thu_imx318(uint8_t *value, int num) {
	int ret = 0, i = 0;
	uint16_t status = 0;
	struct msm_camera_i2c_client *client = s_ctrl_imx318->sensor_i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = s_ctrl_imx318->sensor_i2c_client->i2c_func_tbl->i2c_write;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = s_ctrl_imx318->sensor_i2c_client->i2c_func_tbl->i2c_read;

	/* to make sure the channel is ok */
	i2c_read(client, 0xB973, &status, 1);
	if (status == 4) { // 2 is normal, 4 is communication error
		pr_err("%s: communication reset\n", __func__);
		ret |= i2c_write(client, 0x3370, 0x84, 1);
		for (i=0; i<300; i++) {
			i2c_read(client, 0x3370, &status, 1);
			pr_err("%s: %d, init send status polling=0x%x\n", __func__, i, status);
			if ((status&0x80)==0)
				break;
			usleep_range(10000, 11000);
		}
	}

	/* start to transfer data to imx318, then imx318 send to vcm */
	ret |= i2c_write(client, 0x3373, 0, 1); // 1 means data type is byte
	ret |= i2c_write(client, 0x3374, num, 1); // 1 means data type is byte
	for (i = 0; i < num; i++)
		ret |= i2c_write(client, 0x3378+i, (uint16_t)value[i], 1);
	ret |= i2c_write(client, 0x3370, 0x81, 1);

	/* polling to make sure the communication is done */
	for (i = 0; i < 300; i++) {
		i2c_read(client, 0x3370, &status, 1);
		if ((status&0x80)==0)
			break;
		pr_err("%s: %d, command send status polling=0x%x\n", __func__, i, status);
		usleep_range(10000, 11000);
	}
	return ret;
}

void update_vcm_fw(int version)
{
	uint8_t value_init1[3] = {0x72, 0x84, 0x00};
	uint8_t value_init2[3] = {0x72, 0x87, 0x00};
	uint8_t value_init3[3] = {0x72, 0x8E, 0x00};
	uint8_t value_read_fw_id[2] = {0x73, 0x48};
	uint8_t read_data[32] = {0};

	uint8_t value_eeprom_w_enable1[3] = {0x72, 0x9C, 0xAF};
	uint8_t value_eeprom_w_enable2[3] = {0x72, 0x9D, 0x80};
	uint8_t value_read_status[2] = {0x72, 0xED};
	uint8_t value1_47[10] = {0x73, 0x20, 0x1F, 0x00, 0x19, 0x8B, 0x40, 0xFF, 0x40, 0x0E};
	uint8_t value2_47[10] = {0x73, 0x48, 0x47, 0xF0, 0x7F, 0xF0, 0x8E, 0xC0, 0x71, 0xD0};
	uint8_t value3_47[10] = {0x73, 0x50, 0x63, 0x10, 0x41, 0xB0, 0x28, 0x00, 0x00, 0x00};
	uint8_t value4_47[10] = {0x73, 0x58, 0x50, 0xC0, 0xC6, 0xD0, 0x7F, 0xF0, 0x06, 0x20};
	uint8_t value5_47[10] = {0x73, 0x60, 0x04, 0xA0, 0x76, 0xB0, 0x7F, 0xF0, 0xA8, 0x00};
	uint8_t value_eeprom_w_disable1[3] = {0x72, 0x9C, 0x00};
	uint8_t value_eeprom_w_disable2[3] = {0x72, 0x9D, 0x00};
	int wait_count = 0;
	int i = 0;
	int mismatch_count = 0;
	uint8_t read_value1[2] = {0x73, 0x20};
	uint8_t read_value2[2] = {0x73, 0x48};
	uint8_t read_value3[2] = {0x73, 0x50};
	uint8_t read_value4[2] = {0x73, 0x58};
	uint8_t read_value5[2] = {0x73, 0x60};
	uint8_t value1_40[10] = {0x73, 0x20, 0x1F, 0x00, 0x19, 0x8B, 0x40, 0xFF, 0x40, 0x18};
	uint8_t value2_40[10] = {0x73, 0x48, 0x40, 0x30, 0x7F, 0xF0, 0x8F, 0x70, 0x71, 0x30};
	uint8_t value3_40[10] = {0x73, 0x50, 0x61, 0xB0, 0x40, 0x30, 0x28, 0x00, 0x00, 0x00};
	uint8_t value4_40[10] = {0x73, 0x58, 0x50, 0xC0, 0xBF, 0xD0, 0x7F, 0xF0, 0x06, 0x20};
	uint8_t value5_40[10] = {0x73, 0x60, 0x09, 0x90, 0x6C, 0xF0, 0x7F, 0xF0, 0xA8, 0x00};

	uint8_t value1[10] = {0x73, 0x20, 0x1F, 0x00, 0x19, 0x8B, 0x40, 0xFF, 0x40, 0x0E};
	uint8_t value2[10] = {0x73, 0x48, 0x47, 0xF0, 0x7F, 0xF0, 0x8E, 0xC0, 0x71, 0xD0};
	uint8_t value3[10] = {0x73, 0x50, 0x63, 0x10, 0x41, 0xB0, 0x28, 0x00, 0x00, 0x00};
	uint8_t value4[10] = {0x73, 0x58, 0x50, 0xC0, 0xC6, 0xD0, 0x7F, 0xF0, 0x06, 0x20};
	uint8_t value5[10] = {0x73, 0x60, 0x04, 0xA0, 0x76, 0xB0, 0x7F, 0xF0, 0xA8, 0x00};

	if (version == 0x40) {
		memcpy(value1,  value1_40, sizeof(value1));
		memcpy(value2,  value2_40, sizeof(value2));
		memcpy(value3,  value3_40, sizeof(value3));
		memcpy(value4,  value4_40, sizeof(value4));
		memcpy(value5,  value5_40, sizeof(value5));
	} else if (version == 0x47) {
		memcpy(value1,  value1_47, sizeof(value1));
		memcpy(value2,  value2_47, sizeof(value2));
		memcpy(value3,  value3_47, sizeof(value3));
		memcpy(value4,  value4_47, sizeof(value4));
		memcpy(value5,  value5_47, sizeof(value5));
	}

	/* init and read vcm fw id */
	i2c_write_thu_imx318(value_init1, 3);
	i2c_write_thu_imx318(value_init2, 3);
	i2c_write_thu_imx318(value_init3, 3);
	i2c_read_thu_imx318(value_read_fw_id, 2, read_data, 1);

	/* update fw */
	i2c_write_thu_imx318(value_init1, 3);
	i2c_write_thu_imx318(value_init2, 3);
	i2c_write_thu_imx318(value_init3, 3);
	i2c_write_thu_imx318(value_eeprom_w_enable1, 3);
	i2c_write_thu_imx318(value_eeprom_w_enable2, 3);
	i2c_write_thu_imx318(value1, 10);
	wait_count = 0;
	do {
		usleep_range(3000, 3500);
		i2c_read_thu_imx318(value_read_status, 2, read_data, 1);
		wait_count++;
	} while (read_data[0] != 0x00 && wait_count < 5);
	i2c_write_thu_imx318(value2, 10);
	wait_count = 0;
	do {
		usleep_range(3000, 3500);
		i2c_read_thu_imx318(value_read_status, 2, read_data, 1);
		wait_count++;
	} while (read_data[0] != 0x00 && wait_count < 5);
	i2c_write_thu_imx318(value3, 10);
	wait_count = 0;
	do {
		usleep_range(3000, 3500);
		i2c_read_thu_imx318(value_read_status, 2, read_data, 1);
		wait_count++;
	} while (read_data[0] != 0x00 && wait_count < 5);
	i2c_write_thu_imx318(value4, 10);
	wait_count = 0;
	do {
		usleep_range(3000, 3500);
		i2c_read_thu_imx318(value_read_status, 2, read_data, 1);
		wait_count++;
	} while (read_data[0] != 0x00 && wait_count < 5);
	i2c_write_thu_imx318(value5, 10);
	wait_count = 0;
	do {
		usleep_range(3000, 3500);
		i2c_read_thu_imx318(value_read_status, 2, read_data, 1);
		wait_count++;
	} while (read_data[0] != 0x00 && wait_count < 5);
	i2c_write_thu_imx318(value_eeprom_w_disable1, 3);
	i2c_write_thu_imx318(value_eeprom_w_disable2, 3);

	/* verify fw write success */
	i2c_write_thu_imx318(value_init1, 3);
	i2c_write_thu_imx318(value_init2, 3);
	i2c_write_thu_imx318(value_init3, 3);
	mismatch_count = 0;
	i2c_read_thu_imx318(read_value1, 2, read_data, 8);
	for (i = 0; i < 8; i++) {
		if (read_data[i] != value1[i+2]) {
			pr_err("%s: value1 %d expect 0x%x, actual 0x%x\n", __func__, i, value1[i+2], read_data[i]);
			mismatch_count++;
		}
	}
	i2c_read_thu_imx318(read_value2, 2, read_data, 8);
	for (i = 0; i < 8; i++) {
		if (read_data[i] != value2[i+2]) {
			pr_err("%s: value2 %d expect 0x%x, actual 0x%x\n", __func__, i, value2[i+2], read_data[i]);
			mismatch_count++;
		}
	}
	i2c_read_thu_imx318(read_value3, 2, read_data, 8);
	for (i = 0; i < 8; i++) {
		if (read_data[i] != value3[i+2]) {
			pr_err("%s: value3 %d expect 0x%x, actual 0x%x\n", __func__, i, value3[i+2], read_data[i]);
			mismatch_count++;
		}
	}
	i2c_read_thu_imx318(read_value4, 2, read_data, 8);
	for (i = 0; i < 8; i++) {
		if (read_data[i] != value4[i+2]) {
			pr_err("%s: value4 %d expect 0x%x, actual 0x%x\n", __func__, i, value4[i+2], read_data[i]);
			mismatch_count++;
		}
	}
	i2c_read_thu_imx318(read_value5, 2, read_data, 8);
	for (i = 0; i < 8; i++) {
		if (read_data[i] != value5[i+2]) {
			pr_err("%s: value5 %d expect 0x%x, actual 0x%x\n", __func__, i, value5[i+2], read_data[i]);
			mismatch_count++;
		}
	}
	pr_err("%s, mismatch_count=%d\n", __func__, mismatch_count);

}

uint8_t read_vcm_fw_id(void)
{
	uint8_t value_init1[3] = {0x72, 0x84, 0x00};
	uint8_t value_init2[3] = {0x72, 0x87, 0x00};
	uint8_t value_init3[3] = {0x72, 0x8E, 0x00};
	uint8_t value_read_fw_id[2] = {0x73, 0x48};
	uint8_t read_data[32] = {0};

	/* init and read vcm fw id */
	i2c_write_thu_imx318(value_init1, 3);
	i2c_write_thu_imx318(value_init2, 3);
	i2c_write_thu_imx318(value_init3, 3);
	i2c_read_thu_imx318(value_read_fw_id, 2, read_data, 1);

	pr_err("%s: fw_id=0x%x\n", __func__, read_data[0]);
	return read_data[0];
}

static ssize_t dbg_dump_vcm_fw_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	uint8_t fw_id = 0;

	if (*ppos)
		return 0;	/* the end */

	if (!imx318_power_on) {
		pr_err("please power on imx318 first");
		fw_id = 0;
	} else
		fw_id = read_vcm_fw_id();

	len = snprintf(bp, dlen,"0x%x\n", fw_id);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_dump_vcm_fw_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt = 0;
	unsigned int fw_id = 0;

	if (!imx318_power_on) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "0x%x", &fw_id);
	pr_err("%s: fw_id=0x%x\n", __func__, fw_id);

	if (s_ctrl_vcm == NULL)
		pr_err("%s: s_ctrl_vcm is null\n", __func__);

	update_vcm_fw(fw_id);

	return count;
}

static const struct file_operations dbg_dump_vcm_fw_fops = {
	.open		= dbg_dump_open,
	.write		= dbg_dump_vcm_fw_write,
	.read		= dbg_dump_vcm_fw_read,
};




static int vcm_z1;
static int vcm_z2;
static int vcm_Z1_Z2_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "0x%02x 0x%02x 0x%02x 0x%02x\n", (vcm_z1 & 0xFF00) >> 8, vcm_z1 & 0xFF, (vcm_z2 & 0xFF00) >> 8, vcm_z2 & 0xFF);
	return 0;
}

static int vcm_Z1_Z2_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, vcm_Z1_Z2_proc_read, NULL);
}

static const struct file_operations vcm_Z1_Z2_proc_fops = {
	.owner = THIS_MODULE,
	.open = vcm_Z1_Z2_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void vcm_Z1_Z2_read()
{
	int ret = 0;
	uint8_t value1[2] = {0x72, 0xF0};
	uint8_t value2[2] = {0x73, 0x2E};
	uint8_t value3[2] = {0x73, 0x2F};
	uint8_t value4[2] = {0x73, 0x30};
	uint8_t value5[2] = {0x73, 0x31};
	uint8_t read_data[2] = {0};
	uint8_t read_data2[2] = {0};
	int vcm_z1_temp = 0;
	int vcm_z2_temp = 0;

	if (!imx318_power_on || s_ctrl_rear == NULL) {
		pr_err("imx318 not power on\n");
		return;
	}

	ret = i2c_read_thu_imx318(value1, 2, read_data, 1);
	pr_err("%s: read 0xF0=0x%x\n", __func__, read_data[0]);
	ret = i2c_read_thu_imx318(value2, 2, read_data, 1);
	ret = i2c_read_thu_imx318(value3, 2, read_data2, 1);
	pr_err("%s: read 0x2E, 0x2F={ 0x%x, 0x%x }\n", __func__, read_data[0], read_data2[0]);
	vcm_z1_temp = (read_data[0] << 8) | read_data2[0];
	vcm_z1 = (((vcm_z1_temp << 4)+ 0x8000)&0xffff) >> 4;
	pr_err("%s: convert Z1 to %d, 0x%x\n", __func__, vcm_z1, vcm_z1);
	ret = i2c_read_thu_imx318(value4, 2, read_data, 1);
	ret = i2c_read_thu_imx318(value5, 2, read_data2, 1);
	pr_err("%s: read 0x30, 0x31={ 0x%x, 0x%x }\n", __func__, read_data[0], read_data2[0]);
	vcm_z2_temp = (read_data[0] << 8) | read_data2[0];
	vcm_z2 = (((vcm_z2_temp << 4)+ 0x8000)&0xffff) >> 4;
	pr_err("%s: convert Z2 to %d, 0x%x\n", __func__, vcm_z2, vcm_z2);
}

static int32_t dbg_dump_flash_init(
	struct msm_flash_ctrl_t *flash_ctrl)
{
	struct msm_camera_i2c_reg_setting conf_array;
	uint16_t reg_data = 0;
	unsigned i = 0;
	int32_t rc = 0;
  struct msm_camera_i2c_reg_array data[]=
			{
				{0x04, 0x00, 0x00}, /*disable LED1, 2,3*/
				{0x00, 0x0B, 0x01}, /*set Flash 1 current*/
				{0x01, 0x0B, 0x01}, /*set Flash 2 current*/
				{0x02, 0xff, 0x01}, /*set Flash 1,2 timer*/
				{0x03, 0x77, 0x01}, /*set LED 1,2 current*/
				{0x05, 0x01, 0x01}, /*enable Input voltage monitor*/
				{0x06, 0x20, 0x01},
			};

  rc = msm_camera_request_gpio_table(
		flash_ctrl->power_info.gpio_conf->cam_gpio_req_tbl,
		flash_ctrl->power_info.gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_info("%s: request gpio failed\n", __func__);
		//return rc;
	}
  if (flash_ctrl->power_info.gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_RESET] == 1)
		gpio_set_value_cansleep(
			flash_ctrl->power_info.gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
	gpio_set_value_cansleep(
		flash_ctrl->power_info.gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	rc = msm_camera_power_up(&flash_ctrl->power_info,
		flash_ctrl->flash_device_type,
		&flash_ctrl->flash_i2c_client);
	if (rc < 0) {
		pr_err("%s msm_camera_power_up failed %d\n",
			__func__, __LINE__);
		return rc;
	}

	conf_array.reg_setting = data;
	conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
	conf_array.delay = 0;
	conf_array.size = 7;

	rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(&flash_ctrl->flash_i2c_client, &conf_array);
	if(rc < 0)
		pr_err("%s:%d i2c write err on %d\n", __func__, __LINE__, i);
	flash_ctrl->flash_i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	for (i = 0; i < conf_array.size; i++){
		rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_read(
			&flash_ctrl->flash_i2c_client, data[i].reg_addr, &reg_data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
			pr_err("%s:%d i2c_read err on %d, rc = %d\n", __func__, __LINE__, i, rc);
		else
		  pr_info("%s:%d i2c_read data[%d] = %d, rc = %d\n", __func__, __LINE__, i, reg_data, rc);
	}

	flash_ctrl->flash_state = MSM_CAMERA_FLASH_INIT;
	return rc;
}

static int32_t dbg_dump_flash_manual(
	struct msm_flash_ctrl_t *flash_ctrl, int addr1, int data1)
{
	struct msm_camera_i2c_reg_setting conf_array;
	uint16_t reg_data = 0;
	unsigned i = 0;
	int32_t rc = 0;
	struct msm_camera_i2c_reg_array data;

	conf_array.reg_setting = &data;
	conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
	conf_array.delay = 0;
	conf_array.reg_setting->reg_addr = addr1;
	conf_array.reg_setting->reg_data = data1;
	conf_array.reg_setting->delay = 0x00;
	conf_array.size = 1;
  if (conf_array.addr_type != flash_ctrl->flash_i2c_client.addr_type) {
    flash_ctrl->flash_i2c_client.addr_type = conf_array.addr_type;
  }
	rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
		&flash_ctrl->flash_i2c_client, &conf_array);
	if(rc < 0)
		pr_err("%s:%d i2c write err on %d\n", __func__, __LINE__, i);
	for (i = 0; i < conf_array.size; i++){
		rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_read(
			&flash_ctrl->flash_i2c_client, data.reg_addr, &reg_data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
			pr_err("%s:%d i2c_read err on %d, rc = %d\n", __func__, __LINE__, i, rc);
	}
	return rc;
}


static int32_t dbg_dump_flash_high(
	struct msm_flash_ctrl_t *flash_ctrl)
{
	struct msm_camera_i2c_reg_setting conf_array;
	uint16_t reg_data = 0;
	unsigned i = 0;
	int32_t rc = 0;
	struct msm_camera_i2c_reg_array data;

	conf_array.reg_setting = &data;
	conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
	conf_array.delay = 0;
	conf_array.reg_setting->reg_addr = 0x04;
	conf_array.reg_setting->reg_data = 0x02;
	conf_array.reg_setting->delay = 0x00;
	conf_array.size = 1;
  if (conf_array.addr_type != flash_ctrl->flash_i2c_client.addr_type) {
    flash_ctrl->flash_i2c_client.addr_type = conf_array.addr_type;
  }
	rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
		&flash_ctrl->flash_i2c_client, &conf_array);
	if(rc < 0)
		pr_err("%s:%d i2c write err on %d\n", __func__, __LINE__, i);
	for (i = 0; i < conf_array.size; i++){
		rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_read(
			&flash_ctrl->flash_i2c_client, data.reg_addr, &reg_data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
			pr_err("%s:%d i2c_read err on %d, rc = %d\n", __func__, __LINE__, i, rc);
	}
	return rc;
}
static int32_t dbg_dump_flash_low(
	struct msm_flash_ctrl_t *flash_ctrl)
{
	struct msm_camera_i2c_reg_setting conf_array;
	uint16_t reg_data = 0;
	unsigned i = 0;
	int32_t rc = 0;
	struct msm_camera_i2c_reg_array data;

	conf_array.reg_setting = &data;

	conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;

	conf_array.delay = 0;
	conf_array.reg_setting->reg_addr = 0x04;
	conf_array.reg_setting->reg_data = 0x01;
	conf_array.reg_setting->delay = 0x00;
	conf_array.size = 1;

  if (conf_array.addr_type != flash_ctrl->flash_i2c_client.addr_type) {
    flash_ctrl->flash_i2c_client.addr_type = conf_array.addr_type;
  }
	rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
		&flash_ctrl->flash_i2c_client, &conf_array);
	if(rc < 0)
		pr_err("%s:%d i2c write err on %d\n", __func__, __LINE__, i);
	for (i = 0; i < conf_array.size; i++){
		rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_read(
			&flash_ctrl->flash_i2c_client, data.reg_addr, &reg_data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
			pr_err("%s:%d i2c_read err on %d, rc = %d\n", __func__, __LINE__, i, rc);
	}
	return rc;
}
static int32_t dbg_dump_flash_off(
	struct msm_flash_ctrl_t *flash_ctrl)
{
	struct msm_camera_i2c_reg_setting conf_array;
	uint16_t reg_data = 0;
	unsigned i = 0;
	int32_t rc = 0;
	struct msm_camera_i2c_reg_array data;

	conf_array.reg_setting = &data;
	conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;

	conf_array.delay = 0;
	conf_array.reg_setting->reg_addr = 0x04;
	conf_array.reg_setting->reg_data = 0x00;
	conf_array.reg_setting->delay = 0x00;
	conf_array.size = 1;

  if (conf_array.addr_type != flash_ctrl->flash_i2c_client.addr_type) {
    flash_ctrl->flash_i2c_client.addr_type = conf_array.addr_type;
  }
	rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
		&flash_ctrl->flash_i2c_client, &conf_array);
	if(rc < 0)
		pr_err("%s:%d i2c write err on %d\n", __func__, __LINE__, i);
	for (i = 0; i < conf_array.size; i++){
		rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_read(
			&flash_ctrl->flash_i2c_client, data.reg_addr, &reg_data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
			pr_err("%s:%d i2c_read err on %d, rc = %d\n", __func__, __LINE__, i, rc);
	}
	return rc;
}
static int32_t dbg_dump_flash_release(
	struct msm_flash_ctrl_t *flash_ctrl)
{
	int32_t rc = 0;

	if (!(&flash_ctrl->power_info) || !(&flash_ctrl->flash_i2c_client)) {
		pr_err("%s:%d failed: %p %p\n",
			__func__, __LINE__, &flash_ctrl->power_info,
			&flash_ctrl->flash_i2c_client);
		return -EINVAL;
	}
  if (flash_ctrl->power_info.gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_RESET] == 1)
		gpio_set_value_cansleep(
			flash_ctrl->power_info.gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_LOW);
	gpio_set_value_cansleep(
		flash_ctrl->power_info.gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	rc = msm_camera_power_down(&flash_ctrl->power_info,
		flash_ctrl->flash_device_type,
		&flash_ctrl->flash_i2c_client);
	if (rc < 0) {
		pr_err("%s msm_camera_power_down failed %d\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	flash_ctrl->flash_state = MSM_CAMERA_FLASH_RELEASE;
	return 0;
}

static ssize_t dbg_dump_flash_set_status(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	int val1 = 0;
	unsigned int val2 = 0;
	unsigned int val3 = 0;
	int rc = 0;
  int state = 0;
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%d %x %x", &val1, &val2, &val3);
	pr_info("%s: val1 = %d, val2 = 0x%x, val3 = 0x%x\n", __func__, val1, val2, val3);
  state = gpio_get_value_cansleep(
			s_ctrl_flash->power_info.gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN]);
	pr_info("%s: SENSOR_GPIO_FL_EN state is %d\n", __func__, state);
  if (val1 != FLASH_TURN_OFF &&
     (!state || s_ctrl_flash->flash_state != MSM_CAMERA_FLASH_INIT)) {
	  s_ctrl_flash->flash_state = MSM_CAMERA_FLASH_RELEASE;
    rc = dbg_dump_flash_init(s_ctrl_flash);
		if (rc < 0) {
			pr_err("%s flash_init failed line %d\n", __func__, __LINE__);
		}
  }
	switch (val1) {
	case FLASH_LOW_TURN_ON:
		rc = dbg_dump_flash_low(s_ctrl_flash);
		if (rc < 0) {
			pr_err("%s flash_low failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case FLASH_HIGH_TURN_ON:
		rc = dbg_dump_flash_high(s_ctrl_flash);
		if (rc < 0) {
			pr_err("%s flash_high failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case FLASH_I2C_MANUAL:
	  rc = dbg_dump_flash_manual(s_ctrl_flash, val2, val3);
		if (rc < 0) {
			pr_err("%s send flash manual i2c command failed line %d\n", __func__, __LINE__);
			return rc;
		}
	  break;
	case FLASH_TURN_OFF:
		rc = dbg_dump_flash_off(s_ctrl_flash);
		if (rc < 0) {
			pr_err("%s flash_off failed line %d\n", __func__, __LINE__);
			//return rc;
		}
		rc = dbg_dump_flash_release(s_ctrl_flash);
		if (rc < 0) {
			pr_err("%s flash_release failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	default:
		rc = -1;
		break;
	}
	return count;
}

static const struct file_operations dbg_dump_flash_test_fops = {
	.open		= dbg_dump_open,
	.write		= dbg_dump_flash_set_status,
};

static int32_t translate_flash_current_to_reg_data(int32_t flash_current,
int8_t flash_mode)
{
  int32_t                     rc = -EINVAL;
  int32_t                     flash_default_reg_current = 15; //15 means 1000mA
  switch (flash_mode) {
    case 1: // FLASH
      if (flash_current > FLASH_MODULE_MAX_CURRENT
      || flash_current < FLASH_MODULE_MIN_CURRENT
      || flash_current % FLASH_MODULE_STEP_CURRENT != 0
      || (flash_current > FLASH_MODULE_DEFAULT_CURRENT
           && flash_current % (2 * FLASH_MODULE_STEP_CURRENT)) != 0) {
        pr_err("%s Invalid flash_current %d\n", __func__, flash_current);
        return rc;
      }
      if (flash_current <= FLASH_MODULE_DEFAULT_CURRENT) {
        rc = (flash_current - FLASH_MODULE_MIN_CURRENT) / FLASH_MODULE_STEP_CURRENT;
        pr_info("%s original flash_current %d, reg flash_current %d\n", __func__, flash_current, rc);
      }
      else if (flash_current > FLASH_MODULE_DEFAULT_CURRENT) {
        rc = (flash_current - FLASH_MODULE_DEFAULT_CURRENT) / (2 * FLASH_MODULE_STEP_CURRENT) + flash_default_reg_current;
        pr_info("%s original flash_current %d, reg flash_current %d\n", __func__, flash_current, rc);
      }
      break;
    case 2: //TORCH
      if (flash_current > FLASH_MODULE_MAX_TORCH_CURRENT
          || flash_current < FLASH_MODULE_MIN_TORCH_CURRENT
          || flash_current % FLASH_MODULE_STEP_TORCH_CURRENT != 0) {
        pr_err("%s Invalid torch_current %d\n", __func__, flash_current);
        return rc;
      }
      rc = (flash_current - FLASH_MODULE_MIN_TORCH_CURRENT)/ FLASH_MODULE_STEP_TORCH_CURRENT;
      pr_info("%s original torch_current %d, reg torch_current %d\n", __func__, flash_current, rc);
      break;
  }
  return rc;
}
static int32_t dbg_low_flash_turn_on(
	struct msm_flash_ctrl_t *flash_ctrl, int light_choice)
{
	struct msm_camera_i2c_reg_setting conf_array;
	uint16_t reg_data = 0;
	unsigned i = 0;
	int32_t rc = 0;
	struct msm_camera_i2c_reg_array data;

	conf_array.reg_setting = &data;
	conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
	conf_array.delay = 0;
	conf_array.reg_setting->delay = 0x00;
	conf_array.size = 1;

	switch (light_choice) {
    case 1:
      conf_array.reg_setting->reg_addr = 0x04;
      conf_array.reg_setting->reg_data = 0x01;
      break;
    case 2:
      conf_array.reg_setting->reg_addr = 0x04;
      conf_array.reg_setting->reg_data = 0x10;
      break;
    case 3:
      conf_array.reg_setting->reg_addr = 0x04;
      conf_array.reg_setting->reg_data = 0x11;
      break;
    default:
      pr_err("%s:%d wrong light_choice %d\n", __func__, __LINE__, light_choice);
	}
  if (conf_array.addr_type != flash_ctrl->flash_i2c_client.addr_type) {
    flash_ctrl->flash_i2c_client.addr_type = conf_array.addr_type;
  }
	rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
		&flash_ctrl->flash_i2c_client, &conf_array);
	if(rc < 0)
		pr_err("%s:%d i2c write err on %d\n", __func__, __LINE__, i);
	for (i = 0; i < conf_array.size; i++){
		rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_read(
			&flash_ctrl->flash_i2c_client, conf_array.reg_setting->reg_addr, &reg_data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
			pr_err("%s:%d i2c_read err on %d, rc = %d\n", __func__, __LINE__, i, rc);
		else
		  pr_err("%s:%d i2c_read data.reg_addr 0x%x data[%d] = 0x%x, rc = %d\n",
          __func__, __LINE__, conf_array.reg_setting->reg_addr, i, reg_data, rc);
	}
	return rc;
}

static int32_t dbg_high_flash_turn_on(
	struct msm_flash_ctrl_t *flash_ctrl, int light_choice)
{
	struct msm_camera_i2c_reg_setting conf_array;
	uint16_t reg_data = 0;
	unsigned i = 0;
	int32_t rc = 0;
	struct msm_camera_i2c_reg_array data;

	conf_array.reg_setting = &data;
	conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
	conf_array.delay = 0;
	conf_array.reg_setting->delay = 0x00;
	conf_array.size = 1;

	switch (light_choice) {
    case 1:
      conf_array.reg_setting->reg_addr = 0x04;
      conf_array.reg_setting->reg_data = 0x02;
      break;
    case 2:
      conf_array.reg_setting->reg_addr = 0x04;
      conf_array.reg_setting->reg_data = 0x20;
      break;
    case 3:
      conf_array.reg_setting->reg_addr = 0x04;
      conf_array.reg_setting->reg_data = 0x22;
      break;
    default:
      pr_err("%s:%d wrong light_choice %d\n", __func__, __LINE__, light_choice);
	}
  if (conf_array.addr_type != flash_ctrl->flash_i2c_client.addr_type) {
    flash_ctrl->flash_i2c_client.addr_type = conf_array.addr_type;
  }
	rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
		&flash_ctrl->flash_i2c_client, &conf_array);
	if(rc < 0)
		pr_err("%s:%d i2c write err on %d\n", __func__, __LINE__, i);
	for (i = 0; i < conf_array.size; i++){
		rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_read(
			&flash_ctrl->flash_i2c_client, conf_array.reg_setting->reg_addr, &reg_data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
			pr_err("%s:%d i2c_read err on %d, rc = %d\n", __func__, __LINE__, i, rc);
		else
		  pr_debug("%s:%d i2c_read data.reg_addr 0x%x data[%d] = 0x%x, rc = %d\n",
        __func__, __LINE__, conf_array.reg_setting->reg_addr, i, reg_data, rc);
	}
	return rc;
}

static int32_t dbg_flash_set_current(
	struct msm_flash_ctrl_t *flash_ctrl, int current1, int current2, int mode)
{
	struct msm_camera_i2c_reg_setting conf_array;
	uint16_t reg_data = 0;
	unsigned i = 0;
	int32_t rc = 0;
	struct msm_camera_i2c_reg_array data;
  int addr1, data1;
  switch (mode) {
    case 1: //Torch
      addr1 = 0x03;
      data1 = ((translate_flash_current_to_reg_data(current2, 2) & 0xf) << 4)
            | ((translate_flash_current_to_reg_data(current1, 2) & 0xf));
      conf_array.reg_setting = &data;
      conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
      conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
      conf_array.delay = 0;
      conf_array.reg_setting->reg_addr = addr1;
      conf_array.reg_setting->reg_data = data1;
      conf_array.reg_setting->delay = 0x00;
      conf_array.size = 1;
      if (conf_array.addr_type != flash_ctrl->flash_i2c_client.addr_type) {
        flash_ctrl->flash_i2c_client.addr_type = conf_array.addr_type;
      }
      rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
        &flash_ctrl->flash_i2c_client, &conf_array);
      rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_read(
        &flash_ctrl->flash_i2c_client, conf_array.reg_setting->reg_addr, &reg_data, MSM_CAMERA_I2C_BYTE_DATA);
      if(rc < 0)
        pr_err("%s:%d i2c_read err on %d, rc = %d\n", __func__, __LINE__, i, rc);
      else
        pr_info("%s:%d i2c_read data.reg_addr 0x%x data[%d] = 0x%x, rc = %d\n",
            __func__, __LINE__, conf_array.reg_setting->reg_addr, i, reg_data, rc);
      break;
    case 2: //Flash
      if (current1 != 0 || current2 != 0) {
        conf_array.reg_setting = &data;
        conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
        conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
        conf_array.delay = 0;
        conf_array.reg_setting->delay = 0x00;
        conf_array.size = 1;
      }
      if (current1 != 0) {
        addr1 = 0x00;
        data1 = ((translate_flash_current_to_reg_data(current1, 1) & 0x1f));
        pr_info("%s:%d reg_addr: 0x%x, reg_data: 0x%x\n", __func__, __LINE__,
          addr1, data1);
        conf_array.reg_setting->reg_addr = addr1;
        conf_array.reg_setting->reg_data = data1;
        if (conf_array.addr_type != flash_ctrl->flash_i2c_client.addr_type) {
          flash_ctrl->flash_i2c_client.addr_type = conf_array.addr_type;
        }
        rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
          &flash_ctrl->flash_i2c_client, &conf_array);
        rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_read(
          &flash_ctrl->flash_i2c_client, conf_array.reg_setting->reg_addr, &reg_data, MSM_CAMERA_I2C_BYTE_DATA);
        if(rc < 0)
          pr_err("%s:%d i2c_read err on %d, rc = %d\n", __func__, __LINE__, i, rc);
        else
          pr_err("%s:%d i2c_read data.reg_addr 0x%x data[%d] = 0x%x, rc = %d\n",
          __func__, __LINE__, conf_array.reg_setting->reg_addr, i, reg_data, rc);
      }

      if (current2 != 0) {
        addr1 = 0x01;
        data1 = ((translate_flash_current_to_reg_data(current2, 1) & 0x1f));
        pr_info("%s:%d reg_addr: 0x%x, reg_data: 0x%x\n", __func__, __LINE__,
          addr1, data1);
        conf_array.reg_setting->reg_addr = addr1;
        conf_array.reg_setting->reg_data = data1;
        if (conf_array.addr_type != flash_ctrl->flash_i2c_client.addr_type) {
          flash_ctrl->flash_i2c_client.addr_type = conf_array.addr_type;
        }
        rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
          &flash_ctrl->flash_i2c_client, &conf_array);
        rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_read(
          &flash_ctrl->flash_i2c_client, conf_array.reg_setting->reg_addr, &reg_data, MSM_CAMERA_I2C_BYTE_DATA);
        if(rc < 0)
          pr_err("%s:%d i2c_read err on %d, rc = %d\n", __func__, __LINE__, i, rc);
        else
          pr_err("%s:%d i2c_read data.reg_addr 0x%x data[%d] = 0x%x, rc = %d\n",
            __func__, __LINE__, conf_array.reg_setting->reg_addr, i, reg_data, rc);
      }
      break;
      default:
        pr_err("%s:%d Wrong mode choice %d\n", __func__, __LINE__, mode);
  }
	return rc;
}
static int32_t dbg_flash_dump_reg(
	struct msm_flash_ctrl_t *flash_ctrl, int start, int end)
{
	uint16_t reg_data = 0;
	unsigned i = 0;
	int32_t rc = 0;
	for (i = start; i <= end; i++) {
		rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_read(
			&flash_ctrl->flash_i2c_client, i, &reg_data, MSM_CAMERA_I2C_BYTE_DATA);
		pr_info("%s:%d reg_addr: 0x%x, reg_data: 0x%x\n", __func__, __LINE__, i, reg_data);
		if (rc < 0)
			pr_err("%s:%d i2c_read err on addr %d, rc = %d\n", __func__, __LINE__, i, rc);
	}
	return rc;
}

static ssize_t dbg_dump_flash_interface(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	int val1 = 0;
	unsigned int val2 = 0;
	unsigned int val3 = 0;
	int rc = 0;
  int state = 0;
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%d %d %d", &val1, &val2, &val3);
	pr_info("%s: val1 = %d, val2 = %d, val3 = %d\n", __func__, val1, val2, val3);
  state = gpio_get_value_cansleep(
			s_ctrl_flash->power_info.gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN]);
	pr_info("%s: SENSOR_GPIO_FL_EN state is %d\n", __func__, state);
  if (val1 != FLASH_TURN_OFF &&
     (!state || s_ctrl_flash->flash_state != MSM_CAMERA_FLASH_INIT)) {
	  s_ctrl_flash->flash_state = MSM_CAMERA_FLASH_RELEASE;
    rc = dbg_dump_flash_init(s_ctrl_flash);
		if (rc < 0) {
			pr_err("%s flash_init failed line %d\n", __func__, __LINE__);
		}
	}
	pr_info("%s:%d: val1 = %d\n", __func__, __LINE__, val1);
	switch (val1) {
	case FLASH_TURN_OFF_ALL:
		rc = dbg_dump_flash_off(s_ctrl_flash);
		if (rc < 0) {
			pr_err("%s flash_off failed line %d\n", __func__, __LINE__);
			//return rc;
		}
		rc = dbg_dump_flash_release(s_ctrl_flash);
		if (rc < 0) {
			pr_err("%s flash_release failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case FLASH_SET_CURRENT_FLASH:
		rc = dbg_flash_set_current(s_ctrl_flash, val2, val3, 2);
		if (rc < 0) {
			pr_err("%s flash_set_current_high failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case FLASH_SET_CURRENT_TORCH:
		rc = dbg_flash_set_current(s_ctrl_flash, val2, val3, 1);
		if (rc < 0) {
			pr_err("%s flash_set_current_low failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case FLASH_TURN_ON_HIGH:
		rc = dbg_high_flash_turn_on(s_ctrl_flash, val2);
		if (rc < 0) {
			pr_err("%s flash_turn_on_high failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case FLASH_TURN_ON_LOW:
		rc = dbg_low_flash_turn_on(s_ctrl_flash, val2);
		if (rc < 0) {
			pr_err("%s flash_turn_on_low failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case FLASH_DUMP_I2C_REG:
		pr_info("%s:%d: FLASH_DUMP_I2C_REG\n", __func__, __LINE__);
		rc = dbg_flash_dump_reg(s_ctrl_flash, 0, 0xff);
	default:
		rc = -1;
		break;
	}
	return count;
}

static const struct file_operations dbg_flash_control_fops = {
	.open		= dbg_dump_open,
	.write		= dbg_dump_flash_interface,
};

int msm_debugfs_flash_init(struct msm_flash_ctrl_t *flash_ctrl)
{
	int32_t rc = -EFAULT;
  s_ctrl_flash = flash_ctrl;
  if(!s_ctrl_flash) {
    pr_err("%s flash_ctrl is NULL %d\n",
			__func__, __LINE__);
		return rc;
  }
	(void) debugfs_create_file("camera_flash", S_IRUGO | S_IWUGO,
			NULL, NULL, &dbg_dump_flash_test_fops);
	(void) debugfs_create_file("camera_flash_interface", S_IRUGO | S_IWUGO,
			NULL, NULL, &dbg_flash_control_fops);
	return rc;
}


static int32_t dbg_dump_eeprom_parseData(void)
{
	int rc = -1;
	int i = 0;
	int ver2;
	int crc,crc2;
	uint8_t header2[16];
	int SPC_ID,Pro_ID,Len_ID,Effi_ID;

	pr_err("[eeprom] Parse File...\n");
	if (s_ptr == NULL)
	{
		pr_err("[eeprom] File is  NULL\n");
		return rc;
	}

	pr_info("[eeprom] File Header...\n");

	/*
	* 'PMDTEC'
	* HEX '50 4D 44 54 45 43'
	* DEX '80 77 68 84 69 67'
	*/
	if ((s_ptr[i] == 'P') && (s_ptr[i+1] == 'M') && (s_ptr[i+2] == 'D')
			&& (s_ptr[i+3] == 'T') && (s_ptr[i+4] == 'E') && (s_ptr[i+5] == 'C') )
	{
		pr_info("[eeprom] Magic Number 'PMDTEC' okay\n");
		rc = 0;
	}

	if (rc < 0)
	{
		pr_err("%s:%d [eeprom] Magic Number is incorrect... \n"
			, __func__, __LINE__);
		return rc;
	}
	i +=6;

	/* version */
	ver = s_ptr[i]+s_ptr[i+1]*256;
	pr_info("[eeprom] Version number = %d \n", ver);
	i +=2;

	/* crc */
	crc = (((s_ptr[i+3]*256) + s_ptr[i+2])*256 + s_ptr[i+1]*256) + s_ptr[i];
	pr_info("[eeprom] CRC = 0x%02X%02X%02X%02X \n"
		,s_ptr[i+3],s_ptr[i+2],s_ptr[i+1],s_ptr[i]);
	i +=4;

	/* p_size */
	p_size=(((s_ptr[i+3]*256) + s_ptr[i+2])*256 + s_ptr[i+1])*256 + s_ptr[i];
	pr_info("[eeprom] p_size = %d bytes \n",p_size);
	i +=4;


	switch(ver)
	{
		case 3:
		{
			pr_info("[eeprom] parsing Data of version 3\n");

			/* pmd.spc */
			pmd = &s_ptr[i];
			i += p_size;

			/* header2 */
			memcpy(header2, s_ptr, 16);

			/* PMDTEC */
			rc = -1;
			if ((s_ptr[i] == 'P') && (s_ptr[i+1] == 'M') && (s_ptr[i+2] == 'D')
				&& (s_ptr[i+3] == 'T') && (s_ptr[i+4] == 'E') && (s_ptr[i+5] == 'C') )
			{
				pr_info("[eeprom] Magic Number Buffer#2 okay\n");
				rc = 0;
			}
			if (rc < 0)
			{
				pr_err("%s:%d[eeprom] Magic Number Buffer#2 is incorrect... \n"
					, __func__, __LINE__);
			return rc;
			}
			i +=6;

			/* version 2 */
			ver2 = s_ptr[i] + s_ptr[i+1]*256;
			pr_info("[eeprom] Version number of buffer #2= %d \n",ver2);
			i +=2;

			/* crc2 */
			crc2 = (((s_ptr[i+3]*256) + s_ptr[i+2])*256 + s_ptr[i+1])*256 + s_ptr[i];
			pr_info("[eeprom] CRC = 0x%02X%02X%02X%02X \n"
				,s_ptr[i+3],s_ptr[i+2],s_ptr[i+1],s_ptr[i]);
			i +=4;

			/* p_size2 */
			p_size2 = (((s_ptr[i+3]*256) + s_ptr[i+2])*256 + s_ptr[i+1])*256 + s_ptr[i];
			pr_info("[eeprom] p_size2 = %d bytes \n",p_size2);
			i +=4;

			/*
			* lensdata.dat
			* data size 44 bytes
			*/
			lensdata = &s_ptr[i];
			i += 44;

			/*
			* efficiency.dat
			* data size 8 bytes
			*/
			efficiency = &s_ptr[i];
			i += 8;

			return 0;
		}
		case 6:
		{
			pr_info("[eeprom] parsing Data of version 6\n");

			SPC_ID = s_ptr[i+1]*256 + s_ptr[i];
			SPC_size = (((s_ptr[i+5]*256) + s_ptr[i+4])*256 + s_ptr[i+3])*256 + s_ptr[i+2];
			pr_info("[eeprom] SPC_ID %d SPC_size %d \n",SPC_ID,SPC_size);

			/* Block_size = header size + data size */
			i += 6;

			/* pmd.spc */

			pmd = &s_ptr[i];
			i += SPC_size;

			/*
			* ProductCode, prod_code.dat
			* Block size = 13 bytes = header size (6) + data size (7)
			*/
			Pro_ID = s_ptr[i+1]*256 + s_ptr[i];
			i += 2 ;

			Pro_size = (((s_ptr[i+3]*256) + s_ptr[i+2])*256 + s_ptr[i+1])*256 + s_ptr[i];
			pr_info("[eeprom] Pro_ID %d Pro_size %d \n",Pro_ID,Pro_size);
			i += 4 ;

			pr_info("[eeprom] product code revision: %02X,vendor id: %02X, VCSEL type: %02X, "
				"VCSEL diffusor: %02X, lens type: %02X, project stage: %02X, project revision: %02X\n"
				,s_ptr[i],s_ptr[i+1],s_ptr[i+2],
				s_ptr[i+3],s_ptr[i+4],s_ptr[i+5],s_ptr[i+6]);

			prod_code = &s_ptr[i];

			VendorId = s_ptr[i+1];

			i += 7;

			/*
			* lensdata.dat ( tango.bin )
			* Block_size2 = header size (6) + data size (44)
			*/
			Len_ID = s_ptr[i+1]*256 + s_ptr[i];
			i += 2 ;

			Len_size = (((s_ptr[i+3]*256) + s_ptr[i+2])*256 + s_ptr[i+1])*256 + s_ptr[i];
			i += 4;


			pr_info("[eeprom] Len_ID: %02X, Len_size: %02X \n"
				,Len_ID,Len_size);
			lensdata = &s_ptr[i];
			i += Len_size;

			/*
			* efficiency.dat ( scale.spc )
			* Block_size3 = header size (6) + data size (8)
			*/
			Effi_ID = s_ptr[i+1]*256 + s_ptr[i];
			Effi_size = (((s_ptr[i+5]*256) + s_ptr[i+4])*256 + s_ptr[i+3])*256 + s_ptr[i+2];
			i += 6;

			pr_info("[eeprom] Effi_ID: %02X, Effi_size: %02X \n"
				,Effi_ID,Effi_size);
			efficiency = &s_ptr[i];
			i += Effi_size;

			return 0;
		}
		default:
		{
			rc = -1;
			pr_err("%s:%d [eeprom] current only support version 3 and 6 , early return \n",__func__, __LINE__);
			break;
		}
	}
	return rc;
}


static int32_t dbg_dump_eeprom_i2c_read(void)
{
	int rc = -1;
	uint32_t addr = 0x00;
	uint32_t i=0;
	uint16_t i2c_slave_address;
	uint8_t *data_ptr;
	int i2c_retry;
	struct msm_camera_i2c_client *sensor_i2c_client;
	int prc = -1;

	/* CCI_I2C_MAX_READ = 8192 */
	data_ptr = kzalloc(8192, GFP_KERNEL);
	if (!data_ptr) {
		pr_err("[eeprom]%s:%d data_ptr Alloc memory Fail\n", __func__, __LINE__);
		kfree(data_ptr);
		return rc;
	}
	pr_info("%s:%d [eeprom]data_ptr Alloc memory sucess\n",__func__, __LINE__);


	/* Power up */
	prc = msm_sensor_power_up(s_ctrl_depth);
	if (prc < 0) {
		pr_err("%s:%d [eeprom] power up failed \n", __func__, __LINE__);
		kfree(data_ptr);
		return prc;
	}
	pr_info("%s:%d [eeprom] Power up\n", __func__, __LINE__);

	sensor_i2c_client = s_ctrl_depth->sensor_i2c_client;
	i2c_slave_address = sensor_i2c_client->cci_client->sid;
	sensor_i2c_client->cci_client->sid = 0x56;

		for ( i=0 ; i < 8 ; i++)
		{
			i2c_retry = MAX_I2C_retry;
			do
			{
				pr_info("[eeprom] i2c_read_seq  i=%d \n",i);
				rc = sensor_i2c_client->i2c_func_tbl->i2c_read_seq(
				sensor_i2c_client, addr, data_ptr, 8192);
				if (rc < 0)
				{
					i2c_retry--;
				}
			}while(rc < 0 && i2c_retry > 0);

			if (rc < 0)
			{
				pr_err("%s:%d [eeprom] i2c read fail i2c_retry = %d   i= %d  rc =%d \n",__func__, __LINE__,i2c_retry,i,rc);
				goto END;
			}

			memcpy(&s_ptr[i*8192], data_ptr, 8192);
			addr +=8192;
		}

		sensor_i2c_client->cci_client->sid = 0x57;
		addr = 0x00;
		pr_info("%s:%d[eeprom] dump_eeprom_i2c_read Finish 64KB/128KB \n",__func__, __LINE__);

		for ( i=8 ; i < 16 ; i++)
		{
			i2c_retry = MAX_I2C_retry;
			do
			{
				pr_info("[eeprom] i2c_read_seq  i=%d \n",i);
				rc = sensor_i2c_client->i2c_func_tbl->i2c_read_seq(
				sensor_i2c_client, addr, data_ptr, 8192);
				if (rc < 0)
				{
					i2c_retry--;
				}
			}while(rc < 0 && i2c_retry > 0);

			if (rc < 0)
			{
				pr_err("%s:%d [eeprom] i2c read fail i2c_retry = %d   i= %d  rc =%d \n",__func__, __LINE__,i2c_retry,i,rc);
				goto END;
			}

			memcpy(&s_ptr[i*8192], data_ptr, 8192);
			addr +=8192;
		}

END:
	sensor_i2c_client->cci_client->sid = i2c_slave_address;

	/* Power down */
	prc = msm_sensor_power_down(s_ctrl_depth);
	if (rc < 0)
	{
		pr_err("[eeprom] power down failed\n");
		kfree(data_ptr);
		return prc;
	}
	pr_info("%s:%d [eeprom] Power down \n", __func__, __LINE__);

	kfree(data_ptr);
	return rc;
}

static ssize_t dbg_dump_eeprom_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
    /* copy_to_user buffer */
	if (*ppos >= MAX_eeprom)
		return 0;	/* the end */

	if (*ppos + count > MAX_eeprom)
		count = MAX_eeprom - *ppos;

	if (copy_to_user(buf, s_ptr + *ppos, count))
		return -EFAULT;

	*ppos += count;	/* increase offset */
	return count;
}

static ssize_t dbg_dump_eeprom_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int rc =0;
	char debug_buf[256];
	unsigned int input_value = 0;

	pr_info("%s:%d [eeprom] start write \n", __func__, __LINE__);
	if (count > sizeof(debug_buf))
		return -EFAULT;

	if (*ppos + count > sizeof(debug_buf))
        count = sizeof(debug_buf) - *ppos;

	if (copy_from_user(debug_buf + *ppos, buf, count))
		return -EFAULT;

	debug_buf[count] = '\0'; /* end of string */
	sscanf(debug_buf, "%d", &input_value);

	/*
	* case 1 : I2C read eeprom
	* case 2 : parseData
	*/
	switch (input_value)
	{
		case 1:
		{
			rc = dbg_dump_eeprom_i2c_read();
			break;
		}
		case 2:
		{
			rc = dbg_dump_eeprom_parseData();
			break;
		}
		default:
		{
			rc = -1;
			break;
		}
	}
	*ppos += count;
	return count;
}

static const struct file_operations dbg_dump_eeprom_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_eeprom_read,
	.write		= dbg_dump_eeprom_write,
};


static ssize_t dbg_dump_eeprom_efficiency_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int data_size = 0;

	switch (ver)
	{
		case 3:
		{
			data_size = 8;
			break;
		}
		case 6:
		{
			data_size = Effi_size;
			break;
		}
		default:
		{
			pr_err("%s:%d[eeprom]Not support version: %d\n", __func__, __LINE__,ver);
			return 0;
		}
	}

	if (!efficiency) {
		pr_err("%s:%d [eeprom] efficiency no data\n", __func__, __LINE__);
		return 0;
	}

	/* copy_to_user buffer */
	if (*ppos >= data_size)
	{
		return 0;	/* the end */
	}

	if (*ppos + count > data_size)
		count = data_size - *ppos;

	if (copy_to_user(buf, efficiency + *ppos, count))
		return -EFAULT;

	*ppos += count;	/* increase offset */

	return count;
}
static const struct file_operations dbg_dump_eeprom_efficiency = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_eeprom_efficiency_read,
};

static ssize_t dbg_dump_eeprom_lensdata_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int data_size = 0;

	switch (ver)
	{
		case 3:
		{
			data_size = 44;
			break;
		}
		case 6:
		{
			data_size = Len_size;
			break;
		}
		default:
		{
			pr_err("%s:%d[eeprom]Not support version: %d\n", __func__, __LINE__,ver);
			return 0;
		}
	}

	if (!lensdata) {
		pr_err("%s:%d [eeprom]lensdata no data \n", __func__, __LINE__);
		return 0;
	}

	/* copy_to_user buffer */
	if (*ppos >= data_size)
	{
		return 0;	/* the end */
	}

	if (*ppos + count > data_size)
		count = data_size - *ppos;

	if (copy_to_user(buf, lensdata + *ppos, count))
		return -EFAULT;

	*ppos += count;	/* increase offset */

	return count;
}

static const struct file_operations dbg_dump_eeprom_lensdata = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_eeprom_lensdata_read,
};

static ssize_t dbg_dump_eeprom_pmd_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int data_size = 0;

	switch (ver)
	{
		case 3:
		{
			data_size = p_size;
			break;
		}
		case 6:
		{
			data_size = SPC_size;
			break;
		}
		default:
		{
			pr_err("%s:%d[eeprom]Not support version: %d\n", __func__, __LINE__,ver);
			return 0;
		}
	}

	if (!pmd) {
		pr_err("%s:%d[eeprom] pmd no data\n", __func__, __LINE__);
		return 0;
	}

	/* copy_to_user buffer */
	if (*ppos >= data_size)
	{
		return 0;	/* the end */
	}

	if (*ppos + count > data_size)
		count = data_size - *ppos;

	if (copy_to_user(buf, pmd + *ppos, count))
		return -EFAULT;

	*ppos += count;	/* increase offset */

	return count;
}

static const struct file_operations dbg_dump_eeprom_pmd = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_eeprom_pmd_read,
};

static ssize_t dbg_dump_eeprom_ProductCode_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int data_size = 0;

	switch (ver)
	{
		case 3:
		{
			data_size = Pro_size;
			break;
		}
		case 6:
		{
			data_size = Pro_size;
			break;
		}
		default:
		{
			pr_err("%s:%d[eeprom]Not support version: %d\n", __func__, __LINE__,ver);
			return 0;
		}
	}

	if (!prod_code) {
		pr_err("%s:%d[eeprom] prod_code no data\n", __func__, __LINE__);
		return 0;
	}

	/* copy_to_user buffer */
	if (*ppos >= data_size)
	{
		return 0;	/* the end */
	}

	if (*ppos + count > data_size)
		count = data_size - *ppos;

	if (copy_to_user(buf, prod_code + *ppos, count))
		return -EFAULT;

	*ppos += count;	/* increase offset */

	return count;
}

static const struct file_operations dbg_dump_eeprom_ProductCode = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_eeprom_ProductCode_read,
};

static int dbg_VendorId_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d \n", VendorId);
	return 0;
}

static int dbg_VendorId_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dbg_VendorId_read, NULL);
}

static const struct file_operations dbg_dump_eeprom_VendorId = {
	.owner = THIS_MODULE,
	.open = dbg_VendorId_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


int msm_debugfs_eeprom_init(struct msm_sensor_ctrl_t *s_ctrl)
{
	s_ptr = kzalloc(MAX_eeprom,GFP_KERNEL);
	if (!s_ptr) {
		pr_err("[eeprom]%s:%d Mem Alloc Fail\n", __func__, __LINE__);
		kfree(s_ptr);
	}
	pr_info("%s:%d [eeprom]Mem Alloc sucess\n",__func__, __LINE__);

	s_ctrl_depth = s_ctrl;

	return 1;
}

int msm_debugfs_init(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_camera_sensor_slave_info *slave_info)
{
	struct dentry *debugfs_dir;
//	int camera_id = slave_info->camera_id;
	int chip_id = slave_info->sensor_id_info.sensor_id;
	const char *sensor_name = s_ctrl->sensordata->sensor_name;
	int index = number_of_camera++;

	if (index >= MAX_CAMERA) {
		pr_err("Invalid! number of camera (%d) > MAX Camera (%d)",
				number_of_camera, MAX_CAMERA);
		return -1;
	}

  if (!strcmp(sensor_name,"imx318"))
    debugfs_dir = debugfs_create_dir("camera0", NULL);
  else if (!strcmp(sensor_name,"ov7251"))
    debugfs_dir = debugfs_create_dir("camera1", NULL);
  else if (!strcmp(sensor_name,"ov8856"))
    debugfs_dir = debugfs_create_dir("camera2", NULL);
  else
    debugfs_dir = debugfs_create_dir("camera3", NULL);

	dbgfs[index].sensor_id = chip_id;

	debugfs_create_x16("sensor_id", 0644, debugfs_dir,
			&dbgfs[index].sensor_id);
	debugfs_create_u8("exposure_return0", 0666, debugfs_dir,
			&dbgfs[index].exposure_return0);

	if (!strcmp(sensor_name,"ov7251")) {
		debugfs_create_u8("fisheye_status",
				0644, debugfs_dir, &dbgfs[index].status);
	}

	if (!strcmp(sensor_name,"irs10x0c")) {
		debugfs_create_u8("depthsensor_status",
				0644, debugfs_dir, &dbgfs[index].status);
		(void) debugfs_create_file("eeprom", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_eeprom_fops);
		(void) debugfs_create_file("scale.spc", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_eeprom_efficiency);
		(void) debugfs_create_file("tango.bin", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_eeprom_lensdata);
		(void) debugfs_create_file("pmd.spc", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_eeprom_pmd);
		(void) debugfs_create_file("prod_code.dat", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_eeprom_ProductCode);
		(void) debugfs_create_file("VendorId.txt", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_eeprom_VendorId);


	}

	if (!strcmp(sensor_name,"imx318")) {
	  s_ctrl_rear = s_ctrl;
	  (void) debugfs_create_u8("camera_status",
			0644, debugfs_dir, &dbgfs[index].status);
		(void) debugfs_create_file("vcm_test", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_vcm_test_fops);
		(void) debugfs_create_file("vcm_fw", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_vcm_fw_fops);
		(void) debugfs_create_file("ois_gyro_x", S_IRUGO,
			debugfs_dir, NULL, &dbg_ois_gyro_x_fops);
		(void) debugfs_create_file("ois_gyro_y", S_IRUGO,
			debugfs_dir, NULL, &dbg_ois_gyro_y_fops);
		/*Dangerous!! It should limit on ENG and Userdebug, but does not limit now*/
		(void) debugfs_create_file("ois_mode", S_IRUGO | S_IWUGO,
				debugfs_dir, NULL, &dbg_ois_mode_fops);
		(void) debugfs_create_file("ois_accel_gain", S_IRUGO | S_IWUGO,
				debugfs_dir, NULL, &dbg_ois_accel_gain_fops);
    (void) debugfs_create_file("CameraTurnOnPower", S_IRUGO,
      debugfs_dir, NULL, &dbg_rear_turn_on_power_fops);
		(void) debugfs_create_file("CameraOTP", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_imx318_otp_fops);
		(void) debugfs_create_file("Camera_Unique_ID", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_imx318_uid_fops);
	  proc_create(OIS_RW_PROC_FILE, 0666, NULL, &ois_rw_proc_fops);
	  proc_create(OIS_READ_TIMES_PROC_FILE, 0666, NULL, &ois_read_times_proc_fops);
		proc_create(REAR_OTP_PROC_FILE, 0664, NULL, &rear_otp_proc_fops);
		proc_create(REAR_OTP_THERMAL_FILE, 0664, NULL, &rear_thermal_proc_fops);
		proc_create(VCM_Z1_Z2_PROC_FILE, 0664, NULL, &vcm_Z1_Z2_proc_fops);
		proc_create(IMX318_RW_PROC_FILE, 0664, NULL, &imx318_rw_proc_fops);
		proc_create(CAMERA_RES_PROC_FILE, 0664, NULL, &camera_res_proc_fops);
	} else if (!strcmp(sensor_name,"ov8856")) {
	  s_ctrl_front = s_ctrl;
	  (void) debugfs_create_u8("vga_status",
			0644, debugfs_dir, &dbgfs[index].status);
		(void) debugfs_create_file("CameraOTP", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_ov8856_otp_fops);
		(void) debugfs_create_file("Camera_Unique_ID", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_ov8856_uid_fops);
    (void) debugfs_create_file("CameraTurnOnPower", S_IRUGO,
      debugfs_dir, NULL, &dbg_front_turn_on_power_fops);
		proc_create(FRONT_OTP_PROC_FILE, 0664, NULL, &front_otp_proc_fops);
	} else if (!strcmp(sensor_name,"ov7251")) {
		(void) debugfs_create_file("CameraOTP", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_ov7251_otp_fops);
		(void) debugfs_create_file("Camera_Unique_ID", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_ov7251_uid_fops);
	}

	return index;
}

void msm_debugfs_set_status(unsigned int index, unsigned int status)
{
	if (index < MAX_CAMERA)
		dbgfs[index].status = status;
}

static int ov8856_read_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	int  i;
	u16 local_data, package;
	u8 buf[32];
	u16 start_addr, end_addr;
	struct msm_camera_i2c_client *client = s_ctrl->sensor_i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read;
	int32_t (*i2c_read_seq)(struct msm_camera_i2c_client *, uint32_t,
		uint8_t *, uint32_t) = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq;

	 /* make sure reset sensor as default */
	i2c_write(client, 0x0103, 0x01, 1);
	/*set 0x5001[3] to 0 */
	i2c_read(client, 0x5001, &local_data, 1);
	i2c_write(client, 0x5001, ((0x00 & 0x08) | (local_data & (~0x08))), 1);

	for (package = 2; package >= 0; package--) {
		if (package == 0) {
			start_addr = 0x7010;
			end_addr   = 0x702f;
		} else if (package == 1) {
			start_addr = 0x7030;
			end_addr   = 0x704f;
		} else if (package == 2) {
			start_addr = 0x7050;
			end_addr   = 0x706f;
		}
		/* [6] Manual mode(partial) */
		i2c_write(client, 0x3d84, 0xc0, 1);
		/* rst default:13 */
		i2c_write(client, 0x3d85, 0x06, 1);
		/*otp start addr*/
		i2c_write(client, 0x3d88, (start_addr >> 8) & 0xff, 1);
		i2c_write(client, 0x3d89, start_addr & 0xff, 1);
		/*otp end addr*/
		i2c_write(client, 0x3d8A, (end_addr >> 8) & 0xff, 1);
		i2c_write(client, 0x3d8B, end_addr & 0xff, 1);
		/* trigger auto_load */
		i2c_write(client, 0x0100, 0x01, 1);
		/*load otp into buffer*/
		i2c_write(client, 0x3d81, 0x01, 1);
		msleep(5);

		i2c_read_seq(client, start_addr, buf, 32);

		if (buf[8] != 0 || buf[9] != 0) {
			memcpy(&ov8856_otp, (u8 *)&buf, sizeof(buf));
			pr_info("ov8856 otp read success\n");
			goto out;
		}
	}
	pr_err("ov8856 otp read failed\n");
out:
	for (i = start_addr ; i <= end_addr ; i++)
		i2c_write(client, i, 0x00, 1);
	i2c_read(client, 0x5001, &local_data, 1);
	i2c_write(client, 0x5001, (0x08 & 0x08) | (local_data & (~0x08)), 1);
	i2c_write(client, 0x0100, 0x00, 1);
	return 0;
}

static int ov7251_read_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
        u8 buf[32];
        u16 start_addr, end_addr;
        int i,otp_retry;
        struct msm_camera_i2c_client *client = s_ctrl->sensor_i2c_client;
        int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
                enum msm_camera_i2c_data_type) = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write;
        int32_t (*i2c_read_seq)(struct msm_camera_i2c_client *, uint32_t,
                uint8_t *, uint32_t) = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq;
        pr_info("ov7251_read_otp\n");
        otp_retry = 3;
        msleep(30);
        pr_info("ov7251_read_otp1\n");
        i2c_write(client, 0x0100, 0x01, 1);
        msleep(100);
        pr_info("ov7251_read_otp2\n");
        i2c_write(client, 0x3018, 0x10, 1);
        pr_info("ov7251_read_otp3\n");
        msleep(30);
        i2c_write(client, 0x3d81, 0x01, 1);
        pr_info("ov7251_read_otp4\n");
        /*wait 30ms*/
        msleep(100);

        /*retry 3 times */
        for(i = 0; i < otp_retry; i++) {
                /* make sure reset sensor as default */
                /*wait 30ms*/
                /*set start and end address*/
                start_addr = 0x3d00;
                end_addr   = 0x3d1f;
                pr_info("ov7251_read_otp5\n");
                /*read data from OTP*/
                i2c_read_seq(client, start_addr, buf, 32);
                memcpy(ov7251_otp, (u8 *)buf, sizeof(buf));
                if(ov7251_otp[16]!=0){
                        pr_info("ov7251 read otp sucesss");
                        break;
                }
        msleep(30);
        }
        pr_info("ov7251_read_otp6\n");
        i2c_write(client, 0x0100, 0x00, 1);
        msleep(30);
        pr_info("ov7251_read_otp end\n");
        return 0;
}

int msm_read_otp(struct msm_sensor_ctrl_t *s_ctrl, struct msm_camera_sensor_slave_info *slave_info)
{
	const char *sensor_name = s_ctrl->sensordata->sensor_name;

	if (!s_ctrl) {
		pr_err("%s:%d failed: %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}
	if (!strcmp(sensor_name,"ov8856"))
		ov8856_read_otp(s_ctrl);

	if (!strcmp(sensor_name,"imx318")) {
		s_ctrl_imx318 = s_ctrl;
	}

	if (!strcmp(sensor_name,"ov7251"))
                ov7251_read_otp(s_ctrl);

	return 0;
}

void msm_set_actuator_ctrl(struct msm_actuator_ctrl_t *s_ctrl)
{
	s_ctrl_vcm = s_ctrl;
}
void msm_set_ois_ctrl(struct msm_ois_ctrl_t *o_ctrl)
{
	o_ctrl_ois = o_ctrl;
}
void get_eeprom_OTP(struct msm_eeprom_memory_block_t *block)
{
	memcpy(imx318_otp, block->mapdata, OTP_SIZE);
}
void imx318_power_state(int power_on)
{
	imx318_power_on = power_on;
}
