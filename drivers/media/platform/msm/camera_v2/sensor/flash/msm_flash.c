/* Copyright (c) 2009-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/of_gpio.h>
#include "msm_flash.h"
#include "msm_camera_dt_util.h"
#include "msm_cci.h"
#include "../debugfs/msm_debugfs.h"
#include <linux/fs.h>
#include <linux/proc_fs.h>

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
//ASUS_BSP+++
#define LED1_TORCH_ENABLE		1
#define LED1_TORCH_DISABLE 		0x3E  // 111110
#define LED2_TORCH_ENABLE		( 1 << 2 )
#define LED2_TORCH_DISABLE 		0x3B  // 111011
#define LED3_TORCH_ENABLE		( 1 << 4 )
#define LED3_TORCH_DISABLE 		0x2F  // 101111
#define LED123_TORCH_ENABLE		LED1_TORCH_ENABLE | LED2_TORCH_ENABLE | LED3_TORCH_ENABLE
#define LED123_TORCH_DISABLE 	0x2A  // 101010
#define LED1_FLASH_ENABLE		( 1 << 1 )
#define LED2_FLASH_ENABLE		( 1 << 3 )
#define LED3_FLASH_ENABLE		( 1 << 5 )
#define LED123_FLASH_ENABLE		LED1_FLASH_ENABLE | LED2_FLASH_ENABLE | LED3_FLASH_ENABLE

#define FLASH_CURRENT_STEP 12

#define LED1_FLASH_CURRENT_ADDR 0x00
#define LED2_FLASH_CURRENT_ADDR 0x01
#define LED3_FLASH_CURRENT_ADDR 0x02

#define LED1_TORCH_CURRENT_ADDR 0x05
#define LED2_TORCH_CURRENT_ADDR 0x06
#define LED3_TORCH_CURRENT_ADDR 0x07

//ASUS_BSP---

#define MAX_TORCH_CURRENT 135

DEFINE_MSM_MUTEX(msm_flash_mutex);

//ASUS_BSP+++
static struct msm_flash_ctrl_t *gfctrl = NULL;
static struct i2c_driver sky81298_i2c_driver;
//ASUS_BSP---

static struct v4l2_file_operations msm_flash_v4l2_subdev_fops;
static struct led_trigger *torch_trigger;

#define DBG_TXT_BUF_SIZE 256
static char debugTxtBuf[DBG_TXT_BUF_SIZE];

#define MAX_FLASH_CURRENT 750
#define MAX_FLASH_DURATION 800

#define FLASH_MODULE_DEFAULT_TORCH_CURRENT 200
#define FLASH_MODULE_MAX_TORCH_CURRENT 250
#define FLASH_MODULE_MIN_TORCH_CURRENT 25

#define FLASH_MODULE_DEFAULT_CURRENT 1000
#define FLASH_MODULE_MAX_CURRENT 1500
#define FLASH_MODULE_MIN_CURRENT 250
#define FLASH_MODULE_STEP_CURRENT 12 //Use both torch and Flash

#define ISSMALLER(a,b) (((a)<(b))?(true):(false))

static void create_proc_file(void);
static int32_t msm_flash_i2c_write_custom_current(struct msm_flash_ctrl_t *flash_ctrl, struct msm_flash_cfg_data_t *flash_data);

static struct v4l2_file_operations msm_flash_v4l2_subdev_fops;
static struct led_trigger *torch_trigger;

static const struct of_device_id msm_flash_dt_match[] = {
	{.compatible = "qcom,camera-flash", .data = NULL},
	{}
};

static struct msm_flash_table msm_i2c_flash_table;
static struct msm_flash_table msm_gpio_flash_table;
static struct msm_flash_table msm_pmic_flash_table;

static struct msm_flash_table *flash_table[] = {
	&msm_i2c_flash_table,
	&msm_gpio_flash_table,
	&msm_pmic_flash_table
};


//ASUS_BSP+++
static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq = msm_camera_qup_i2c_write_seq,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
	.i2c_poll =  msm_camera_qup_i2c_poll,
};
//ASUS_BSP---
//ASUS_BSP+++

struct msm_camera_i2c_reg_setting_array settings_init  = {
	.reg_setting_a =
	{
		{0x00, 0x43, 0x00},
		{0x01, 0x43, 0x00},
		{0x02, 0x11, 0x00},
		{0x03, 0xFF, 0x00},
		{0x05, 0x11, 0x00},
		{0x06, 0x11, 0x00},
		{0x07, 0x11, 0x00},
		{0x0A, 0xA2, 0x00},
	},
	.size = 8,
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
struct msm_camera_i2c_reg_setting_array settings_brightness  = {
	.reg_setting_a =
	{
		{0x08, 0x01, 0x00},
	},
	.size = 1,
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
struct msm_camera_i2c_reg_setting_array settings_off  = {
	.reg_setting_a =
	{
		{0x08, 0x00, 0x00},
	},
	.size = 1,
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
//ASUS_BSP---


void msm_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (!torch_trigger) {
		pr_err("No torch trigger found, can't set brightness\n");
		return;
	}

	led_trigger_event(torch_trigger, value);
};

static struct led_classdev msm_torch_led[MAX_LED_TRIGGERS] = {
	{
		.name		= "torch-light0",
		.brightness_set	= msm_torch_brightness_set,
		.brightness	= LED_OFF,
	},
	{
		.name		= "torch-light1",
		.brightness_set	= msm_torch_brightness_set,
		.brightness	= LED_OFF,
	},
	{
		.name		= "torch-light2",
		.brightness_set	= msm_torch_brightness_set,
		.brightness	= LED_OFF,
	},
};

static int32_t msm_torch_create_classdev(struct platform_device *pdev,
				void *data)
{
	int32_t rc = 0;
	int32_t i = 0;
	struct msm_flash_ctrl_t *fctrl =
		(struct msm_flash_ctrl_t *)data;

	if (!fctrl) {
		pr_err("Invalid fctrl\n");
		return -EINVAL;
	}

	for (i = 0; i < fctrl->torch_num_sources; i++) {
		if (fctrl->torch_trigger[i]) {
			torch_trigger = fctrl->torch_trigger[i];
			CDBG("%s:%d msm_torch_brightness_set for torch %d",
				__func__, __LINE__, i);
			msm_torch_brightness_set(&msm_torch_led[i],
				LED_OFF);

			rc = led_classdev_register(&pdev->dev,
				&msm_torch_led[i]);
			if (rc) {
				pr_err("Failed to register %d led dev. rc = %d\n",
						i, rc);
				return rc;
			}
		} else {
			pr_err("Invalid fctrl->torch_trigger[%d]\n", i);
			return -EINVAL;
		}
	}

	return 0;
};

static int32_t msm_flash_get_subdev_id(
	struct msm_flash_ctrl_t *flash_ctrl, void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("%s Enter\n",__func__);	//ASUS_BSP+++
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (flash_ctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		*subdev_id = flash_ctrl->pdev->id;
	else
		*subdev_id = flash_ctrl->subdev_id;

	CDBG("Exit\n");
	return 0;
}

static int32_t msm_flash_i2c_write_table(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_camera_i2c_reg_setting_array *settings)
{
	struct msm_camera_i2c_reg_setting conf_array;
	int i = 0;
	conf_array.addr_type = settings->addr_type;
	conf_array.data_type = settings->data_type;
	conf_array.delay = settings->delay;
	conf_array.reg_setting = settings->reg_setting_a;
	conf_array.size = settings->size;
	for (i = 0; i < conf_array.size; i++) {
	  pr_info("%s conf_array[%d] reg_addr: 0x%x, reg_data: 0x%x", __func__, i, conf_array.reg_setting[i].reg_addr, conf_array.reg_setting[i].reg_data);
	}
	/* Validate the settings size */
	if ((!conf_array.size) || (conf_array.size > MAX_I2C_REG_SET)) {
		pr_err("failed: invalid size %d", conf_array.size);
		return -EINVAL;
	}

	return flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
		&flash_ctrl->flash_i2c_client, &conf_array);
}

#ifdef CONFIG_COMPAT
static void msm_flash_copy_power_settings_compat(
	struct msm_sensor_power_setting *ps,
	struct msm_sensor_power_setting32 *ps32, uint32_t size)
{
	uint16_t i = 0;

	for (i = 0; i < size; i++) {
		ps[i].config_val = ps32[i].config_val;
		ps[i].delay = ps32[i].delay;
		ps[i].seq_type = ps32[i].seq_type;
		ps[i].seq_val = ps32[i].seq_val;
	}
}
#endif

static int32_t msm_flash_i2c_init(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int32_t rc = 0;
	struct msm_flash_init_info_t *flash_init_info =
		flash_data->cfg.flash_init_info;
	struct msm_camera_i2c_reg_setting_array *settings = NULL;
	struct msm_camera_cci_client *cci_client = NULL;
#ifdef CONFIG_COMPAT
	struct msm_sensor_power_setting_array32 *power_setting_array32 = NULL;
#endif
	if (!flash_init_info || !flash_init_info->power_setting_array) {
		pr_err("%s:%d failed: Null pointer\n", __func__, __LINE__);
		return -EFAULT;
	}
	printk("[LED_FLASH]@%s\t \n", __func__);	//ASUS_BSP+++

#ifdef CONFIG_COMPAT
	if (is_compat_task()) {
		power_setting_array32 = kzalloc(
			sizeof(struct msm_sensor_power_setting_array32),
			GFP_KERNEL);
		if (!power_setting_array32) {
			pr_err("%s mem allocation failed %d\n",
				__func__, __LINE__);
			return -ENOMEM;
		}

		if (copy_from_user(power_setting_array32,
			(void *)flash_init_info->power_setting_array,
			sizeof(struct msm_sensor_power_setting_array32))) {
			pr_err("%s copy_from_user failed %d\n",
				__func__, __LINE__);
			kfree(power_setting_array32);
			return -EFAULT;
		}

		flash_ctrl->power_setting_array.size =
			power_setting_array32->size;
		flash_ctrl->power_setting_array.size_down =
			power_setting_array32->size_down;
		flash_ctrl->power_setting_array.power_down_setting =
			compat_ptr(power_setting_array32->power_down_setting);
		flash_ctrl->power_setting_array.power_setting =
			compat_ptr(power_setting_array32->power_setting);

		/* Validate power_up array size and power_down array size */
		if ((!flash_ctrl->power_setting_array.size) ||
			(flash_ctrl->power_setting_array.size >
			MAX_POWER_CONFIG) ||
			(!flash_ctrl->power_setting_array.size_down) ||
			(flash_ctrl->power_setting_array.size_down >
			MAX_POWER_CONFIG)) {

			pr_err("failed: invalid size %d, size_down %d",
				flash_ctrl->power_setting_array.size,
				flash_ctrl->power_setting_array.size_down);
			kfree(power_setting_array32);
			power_setting_array32 = NULL;
			return -EINVAL;
		}
		/* Copy the settings from compat struct to regular struct */
		msm_flash_copy_power_settings_compat(
			flash_ctrl->power_setting_array.power_setting_a,
			power_setting_array32->power_setting_a,
			flash_ctrl->power_setting_array.size);

		msm_flash_copy_power_settings_compat(
			flash_ctrl->power_setting_array.power_down_setting_a,
			power_setting_array32->power_down_setting_a,
			flash_ctrl->power_setting_array.size_down);
	} else
#endif
	if (copy_from_user(&flash_ctrl->power_setting_array,
		(void *)flash_init_info->power_setting_array,
		sizeof(struct msm_sensor_power_setting_array))) {
		pr_err("%s copy_from_user failed %d\n", __func__, __LINE__);
		return -EFAULT;
	}

	if (flash_ctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = flash_ctrl->flash_i2c_client.cci_client;
		cci_client->sid = flash_init_info->slave_addr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->i2c_freq_mode = flash_init_info->i2c_freq_mode;
	}

	flash_ctrl->power_info.power_setting =
		flash_ctrl->power_setting_array.power_setting_a;
	flash_ctrl->power_info.power_down_setting =
		flash_ctrl->power_setting_array.power_down_setting_a;
	flash_ctrl->power_info.power_setting_size =
		flash_ctrl->power_setting_array.size;
	flash_ctrl->power_info.power_down_setting_size =
		flash_ctrl->power_setting_array.size_down;

	if ((flash_ctrl->power_info.power_setting_size > MAX_POWER_CONFIG) ||
	(flash_ctrl->power_info.power_down_setting_size > MAX_POWER_CONFIG)) {
		pr_err("%s:%d invalid power setting size=%d size_down=%d\n",
			__func__, __LINE__,
			flash_ctrl->power_info.power_setting_size,
			flash_ctrl->power_info.power_down_setting_size);
		rc = -EINVAL;
		goto msm_flash_i2c_init_fail;
	}

	rc = msm_camera_power_up(&flash_ctrl->power_info,
		flash_ctrl->flash_device_type,
		&flash_ctrl->flash_i2c_client);
	if (rc < 0) {
		pr_err("%s msm_camera_power_up failed %d\n",
			__func__, __LINE__);
		goto msm_flash_i2c_init_fail;
	}

	if (flash_data->cfg.flash_init_info->settings) {
		settings = kzalloc(sizeof(
			struct msm_camera_i2c_reg_setting_array), GFP_KERNEL);
		if (!settings) {
			pr_err("%s mem allocation failed %d\n",
				__func__, __LINE__);
			return -ENOMEM;
		}

		if (copy_from_user(settings, (void *)flash_init_info->settings,
			sizeof(struct msm_camera_i2c_reg_setting_array))) {
			kfree(settings);
			pr_err("%s copy_from_user failed %d\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		rc = msm_flash_i2c_write_table(flash_ctrl, settings);
		kfree(settings);

		if (rc < 0) {
			pr_err("%s:%d msm_flash_i2c_write_table rc %d failed\n",
				__func__, __LINE__, rc);
		}
	}

	return 0;

msm_flash_i2c_init_fail:
	return rc;
}

static int32_t msm_flash_gpio_init(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int32_t i = 0;
	int32_t rc = 0;

	CDBG("Enter");
	for (i = 0; i < flash_ctrl->flash_num_sources; i++)
		flash_ctrl->flash_op_current[i] = LED_FULL;

	for (i = 0; i < flash_ctrl->torch_num_sources; i++)
		flash_ctrl->torch_op_current[i] = LED_HALF;

	for (i = 0; i < flash_ctrl->torch_num_sources; i++) {
		if (!flash_ctrl->torch_trigger[i]) {
			if (i < flash_ctrl->flash_num_sources)
				flash_ctrl->torch_trigger[i] =
					flash_ctrl->flash_trigger[i];
			else
				flash_ctrl->torch_trigger[i] =
					flash_ctrl->flash_trigger[
					flash_ctrl->flash_num_sources - 1];
		}
	}

	rc = flash_ctrl->func_tbl->camera_flash_off(flash_ctrl, flash_data);

	CDBG("Exit");
	return rc;
}

static int32_t msm_flash_i2c_release(
	struct msm_flash_ctrl_t *flash_ctrl)
{
	int32_t rc = 0;

	printk("[LED_FLASH]@%s\t \n", __func__);	//ASUS_BSP+++
	if (!(&flash_ctrl->power_info) || !(&flash_ctrl->flash_i2c_client)) {
		pr_err("%s:%d failed: %pK %pK\n",
			__func__, __LINE__, &flash_ctrl->power_info,
			&flash_ctrl->flash_i2c_client);
		return -EINVAL;
	}

	rc = msm_camera_power_down(&flash_ctrl->power_info,
		flash_ctrl->flash_device_type,
		&flash_ctrl->flash_i2c_client);
	if (rc < 0) {
		pr_err("%s msm_camera_power_down failed %d\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	flash_ctrl->flash_state = MSM_CAMERA_FLASH_RELEASE;	//ASUS_BSP+++
	return 0;
}

static int32_t msm_flash_off(struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int32_t i = 0;

	printk("[LED_FLASH]@%s\t \n", __func__);	//ASUS_BSP+++

	for (i = 0; i < flash_ctrl->flash_num_sources; i++)
		if (flash_ctrl->flash_trigger[i])
			led_trigger_event(flash_ctrl->flash_trigger[i], 0);

	for (i = 0; i < flash_ctrl->torch_num_sources; i++)
		if (flash_ctrl->torch_trigger[i])
			led_trigger_event(flash_ctrl->torch_trigger[i], 0);
	if (flash_ctrl->switch_trigger)
		led_trigger_event(flash_ctrl->switch_trigger, 0);

	CDBG("Exit\n");
	return 0;
}
static int32_t msm_flash_i2c_turn_on_flash(
  struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data, int32_t bulb_ctrl)
{
	struct msm_camera_i2c_reg_setting conf_array;
	struct msm_camera_i2c_reg_array current_data;
	int32_t rc = -EINVAL;
	if (!flash_data) {
		return 0;
	}
	current_data.reg_addr = 0x08;
	current_data.delay = 0x00;

	conf_array.size = 1;
	conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
	conf_array.delay = 0;
	switch (bulb_ctrl) {
			/* Flash case 1: turn on white bulb, 2: turn on yellow bulb, 3: turn on both bulbs */
			/* Torch case 4: turn on white bulb, 5: turn on yellow bulb, 6: turn on both bulbs */
			case 1:
					current_data.reg_data = 0x08;
					break;
			case 2:
					current_data.reg_data = 0x02;
					break;
			case 3:
					current_data.reg_data = 0x0A;
					break;
			case 4:
					current_data.reg_data = 0x04;
					break;
			case 5:
					current_data.reg_data = 0x01;
					break;
			case 6:
					current_data.reg_data = 0x05;
					break;
			default:
					pr_err("Unrecogized flash bulb mode!");
	}
	conf_array.reg_setting = &current_data;
	rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
	&flash_ctrl->flash_i2c_client, &conf_array);
  return rc;
}
static int32_t msm_flash_set_bulb_ctrl(int32_t *bulb_ctrl, struct msm_flash_cfg_data_t *flash_data) {
		int flash_cmd = 0;
		if (!flash_data) {
				return 0;
		}
		flash_cmd = flash_data->cfg_type;
		switch (flash_cmd) {
    case CFG_FLASH_LOW:
			if (flash_data->flash_current[0] == -1 ) {
					*bulb_ctrl = 4; // Turn on white bulb
					CDBG("bulb_ctrl = 4");
					flash_data->flash_current[0] = 0;
			} else if (flash_data->flash_current[1] == -1) {
					*bulb_ctrl = 5; // Turn on yellow bulb
					CDBG("bulb_ctrl = 5");
					flash_data->flash_current[1] = 0;
			} else {
					*bulb_ctrl = 6; // Turn on both bulb
					CDBG("bulb_ctrl = 6");
			}
			break;
    case CFG_FLASH_HIGH:
      if (flash_data->flash_current[0] == -1 ) {
					*bulb_ctrl = 1; // Turn on white bulb
					CDBG("bulb_ctrl = 1");
					flash_data->flash_current[0] = 0;
			} else if (flash_data->flash_current[1] == -1) {
					*bulb_ctrl = 2; // Turn on yellow bulb
					CDBG("bulb_ctrl = 2");
					flash_data->flash_current[1] = 0;
			} else {
					*bulb_ctrl = 3; // Turn on both bulb
					CDBG("bulb_ctrl = 3");
			}
			break;
    default:
      pr_debug("%s:%d cfg_type %d do not need process here.\n", __func__, __LINE__,flash_data->cfg_type);
      break;
  }
  return 0;
}
//ASUS_BSP+++
static uint32_t get_flash_current_addr(int flashIndex) {
	switch(flashIndex) {
		case 1:
			return LED2_FLASH_CURRENT_ADDR;
		case 2:
			return LED3_FLASH_CURRENT_ADDR;
		case 0:
		default:
			return LED1_FLASH_CURRENT_ADDR;
	}
}

static uint32_t get_torch_current_addr(int flashIndex) {
	switch(flashIndex) {
		case 1:
			return LED2_TORCH_CURRENT_ADDR;
		case 2:
			return LED3_TORCH_CURRENT_ADDR;
		case 0:
		default:
			return LED1_TORCH_CURRENT_ADDR;
	}
}
//ASUS_BSP---

static int32_t msm_flash_i2c_write_setting_array(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int32_t rc = 0;
	struct msm_camera_i2c_reg_setting_array *settings = NULL;
	int32_t bulb_ctrl = -1;
	if (!flash_data->cfg.settings) {
		pr_err("%s:%d failed: Null pointer\n", __func__, __LINE__);
		return -EFAULT;
	}
	msm_flash_set_bulb_ctrl(&bulb_ctrl, flash_data);
	msm_flash_i2c_write_custom_current(flash_ctrl, flash_data);
	settings = kzalloc(sizeof(struct msm_camera_i2c_reg_setting_array),
		GFP_KERNEL);
	if (!settings) {
		pr_err("%s mem allocation failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (copy_from_user(settings, (void *)flash_data->cfg.settings,
		sizeof(struct msm_camera_i2c_reg_setting_array))) {
		kfree(settings);
		pr_err("%s copy_from_user failed %d\n", __func__, __LINE__);
		return -EFAULT;
	}
	if (bulb_ctrl == -1) {
		rc = msm_flash_i2c_write_table(flash_ctrl, settings);
		kfree(settings);
	} else {
		rc = msm_flash_i2c_turn_on_flash(flash_ctrl, flash_data, bulb_ctrl);
	if (rc < 0) {
		pr_err("%s:%d msm_flash_i2c_write_table rc = %d failed\n",
			__func__, __LINE__, rc);
		}
		kfree(settings);
	}
	return rc;
}

static int32_t msm_flash_init(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	uint32_t i = 0;
	int32_t rc = -EFAULT;
	enum msm_flash_driver_type flash_driver_type = FLASH_DRIVER_DEFAULT;

	printk("[LED_FLASH]@%s\t \n", __func__);	//ASUS_BSP+++

	if (flash_ctrl->flash_state == MSM_CAMERA_FLASH_INIT) {
		pr_err("%s:%d Invalid flash state = %d",
			__func__, __LINE__, flash_ctrl->flash_state);
		return 0;
	}

	if (flash_data->cfg.flash_init_info->flash_driver_type ==
		FLASH_DRIVER_DEFAULT) {
		flash_driver_type = flash_ctrl->flash_driver_type;
		for (i = 0; i < MAX_LED_TRIGGERS; i++) {
			flash_data->flash_current[i] =
				flash_ctrl->flash_max_current[i];
			flash_data->flash_duration[i] =
				flash_ctrl->flash_max_duration[i];
		}
	} else if (flash_data->cfg.flash_init_info->flash_driver_type ==
		flash_ctrl->flash_driver_type) {
		flash_driver_type = flash_ctrl->flash_driver_type;
		for (i = 0; i < MAX_LED_TRIGGERS; i++) {
			flash_ctrl->flash_max_current[i] =
				flash_data->flash_current[i];
			flash_ctrl->flash_max_duration[i] =
					flash_data->flash_duration[i];
		}
	}

	if (flash_driver_type == FLASH_DRIVER_DEFAULT) {
		pr_err("%s:%d invalid flash_driver_type", __func__, __LINE__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(flash_table); i++) {
		if (flash_driver_type == flash_table[i]->flash_driver_type) {
			flash_ctrl->func_tbl = &flash_table[i]->func_tbl;
			rc = 0;
		}
	}

	if (rc < 0) {
		pr_err("%s:%d failed invalid flash_driver_type %d\n",
			__func__, __LINE__,
			flash_data->cfg.flash_init_info->flash_driver_type);
	}

	if (flash_ctrl->func_tbl->camera_flash_init) {
		rc = flash_ctrl->func_tbl->camera_flash_init(
				flash_ctrl, flash_data);
		if (rc < 0) {
			pr_err("%s:%d camera_flash_init failed rc = %d",
				__func__, __LINE__, rc);
			return rc;
		}
	}

	flash_ctrl->flash_state = MSM_CAMERA_FLASH_INIT;

	CDBG("Exit");
	return 0;
}

static int32_t msm_flash_init_prepare(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
#ifdef CONFIG_COMPAT
	struct msm_flash_cfg_data_t flash_data_k;
	struct msm_flash_init_info_t flash_init_info;
	int32_t i = 0;

	if (!is_compat_task()) {
		/*for 64-bit usecase,it need copy the data to local memory*/
		flash_data_k.cfg_type = flash_data->cfg_type;
		for (i = 0; i < MAX_LED_TRIGGERS; i++) {
			flash_data_k.flash_current[i] =
				flash_data->flash_current[i];
			flash_data_k.flash_duration[i] =
				flash_data->flash_duration[i];
		}

		flash_data_k.cfg.flash_init_info = &flash_init_info;
		if (copy_from_user(&flash_init_info,
			(void __user *)(flash_data->cfg.flash_init_info),
			sizeof(struct msm_flash_init_info_t))) {
			pr_err("%s copy_from_user failed %d\n",
				__func__, __LINE__);
			return -EFAULT;
		}
		return msm_flash_init(flash_ctrl, &flash_data_k);
	}
	/*
	 * for 32-bit usecase,it already copy the userspace
	 * data to local memory in msm_flash_subdev_do_ioctl()
	 * so here do not need copy from user
	 */
	return msm_flash_init(flash_ctrl, flash_data);
#else
	struct msm_flash_cfg_data_t flash_data_k;
	struct msm_flash_init_info_t flash_init_info;
	int32_t i = 0;
	flash_data_k.cfg_type = flash_data->cfg_type;
	for (i = 0; i < MAX_LED_TRIGGERS; i++) {
		flash_data_k.flash_current[i] =
			flash_data->flash_current[i];
		flash_data_k.flash_duration[i] =
			flash_data->flash_duration[i];
	}

	flash_data_k.cfg.flash_init_info = &flash_init_info;
	if (copy_from_user(&flash_init_info,
		(void __user *)(flash_data->cfg.flash_init_info),
		sizeof(struct msm_flash_init_info_t))) {
		pr_err("%s copy_from_user failed %d\n",
			__func__, __LINE__);
		return -EFAULT;
	}
	return msm_flash_init(flash_ctrl, &flash_data_k);
#endif
}

static int32_t msm_flash_low(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	uint32_t curr = 0, max_current = 0;
	int32_t i = 0;

	printk("[LED_FLASH]@%s\t \n", __func__);	//ASUS_BSP+++
	/* Turn off flash triggers */
	for (i = 0; i < flash_ctrl->flash_num_sources; i++)
		if (flash_ctrl->flash_trigger[i])
			led_trigger_event(flash_ctrl->flash_trigger[i], 0);

	/* Turn on flash triggers */
	for (i = 0; i < flash_ctrl->torch_num_sources; i++) {
		if (flash_ctrl->torch_trigger[i]) {
			max_current = flash_ctrl->torch_max_current[i];
			if (flash_data->flash_current[i] >= 0 &&
				flash_data->flash_current[i] <
				max_current) {
				curr = flash_data->flash_current[i];
			} else {
				curr = flash_ctrl->torch_op_current[i];
				pr_debug("LED current clamped to %d\n",
					curr);
			}
			CDBG("low_flash_current[%d] = %d", i, curr);
			led_trigger_event(flash_ctrl->torch_trigger[i],
				curr);
		}
	}
	if (flash_ctrl->switch_trigger)
		led_trigger_event(flash_ctrl->switch_trigger, 1);
	CDBG("Exit\n");
	return 0;
}

static int32_t msm_flash_high(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int32_t curr = 0;
	int32_t max_current = 0;
	int32_t i = 0;

	printk("[LED_FLASH]@%s\t \n", __func__);	//ASUS_BSP+++

	/* Turn off torch triggers */
	for (i = 0; i < flash_ctrl->torch_num_sources; i++)
		if (flash_ctrl->torch_trigger[i])
			led_trigger_event(flash_ctrl->torch_trigger[i], 0);

	/* Turn on flash triggers */
	for (i = 0; i < flash_ctrl->flash_num_sources; i++) {
		if (flash_ctrl->flash_trigger[i]) {
			max_current = flash_ctrl->flash_max_current[i];
			if (flash_data->flash_current[i] >= 0 &&
				flash_data->flash_current[i] <
				max_current) {
				curr = flash_data->flash_current[i];
			} else {
				curr = flash_ctrl->flash_op_current[i];
				pr_debug("LED flash_current[%d] clamped %d\n",
					i, curr);
			}
			CDBG("high_flash_current[%d] = %d", i, curr);
			led_trigger_event(flash_ctrl->flash_trigger[i],
				curr);
		}
	}
	if (flash_ctrl->switch_trigger)
		led_trigger_event(flash_ctrl->switch_trigger, 1);
	return 0;
}

static int32_t msm_flash_release(
	struct msm_flash_ctrl_t *flash_ctrl)
{
	int32_t rc = 0;

	rc = flash_ctrl->func_tbl->camera_flash_off(flash_ctrl, NULL);
	if (rc < 0) {
		pr_err("%s:%d camera_flash_init failed rc = %d",
			__func__, __LINE__, rc);
		return rc;
	}
	flash_ctrl->flash_state = MSM_CAMERA_FLASH_RELEASE;
	return 0;
}

static int32_t msm_flash_config(struct msm_flash_ctrl_t *flash_ctrl,
	void __user *argp)
{
	int32_t rc = 0;
	struct msm_flash_cfg_data_t *flash_data =
		(struct msm_flash_cfg_data_t *) argp;

	mutex_lock(flash_ctrl->flash_mutex);

	pr_info("[LED_FLASH]@Enter %s type %d\n", __func__, flash_data->cfg_type);	//ASUS_BSP+++

	switch (flash_data->cfg_type) {
	case CFG_FLASH_INIT:
		rc = msm_flash_init_prepare(flash_ctrl, flash_data);
		break;
	case CFG_FLASH_RELEASE:
		if (flash_ctrl->flash_state != MSM_CAMERA_FLASH_RELEASE) {
			rc = flash_ctrl->func_tbl->camera_flash_release(
				flash_ctrl);
		} else {
			CDBG(pr_fmt("Invalid state : %d\n"),
				flash_ctrl->flash_state);
		}
		break;
	case CFG_FLASH_OFF:
		if ((flash_ctrl->flash_state != MSM_CAMERA_FLASH_RELEASE) &&
			(flash_ctrl->flash_state != MSM_CAMERA_FLASH_OFF)) {
			rc = flash_ctrl->func_tbl->camera_flash_off(
				flash_ctrl, flash_data);
			if (!rc)
				flash_ctrl->flash_state = MSM_CAMERA_FLASH_OFF;
		} else {
			CDBG(pr_fmt("Invalid state : %d\n"),
				flash_ctrl->flash_state);
		}
		break;
	case CFG_FLASH_LOW:
		if ((flash_ctrl->flash_state == MSM_CAMERA_FLASH_OFF) ||
			(flash_ctrl->flash_state == MSM_CAMERA_FLASH_INIT)) {
			rc = flash_ctrl->func_tbl->camera_flash_low(
				flash_ctrl, flash_data);
			if (!rc)
				flash_ctrl->flash_state = MSM_CAMERA_FLASH_LOW;
		} else {
			CDBG(pr_fmt("Invalid state : %d\n"),
				flash_ctrl->flash_state);
		}
		break;
	case CFG_FLASH_HIGH:
		if ((flash_ctrl->flash_state == MSM_CAMERA_FLASH_OFF) ||
			(flash_ctrl->flash_state == MSM_CAMERA_FLASH_INIT)) {
			rc = flash_ctrl->func_tbl->camera_flash_high(
				flash_ctrl, flash_data);
			if (!rc)
				flash_ctrl->flash_state = MSM_CAMERA_FLASH_HIGH;
		} else {
			CDBG(pr_fmt("Invalid state : %d\n"),
				flash_ctrl->flash_state);
		}
		break;
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(flash_ctrl->flash_mutex);

	CDBG("Exit %s type %d\n", __func__, flash_data->cfg_type);

	return rc;
}

static long msm_flash_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	struct msm_flash_ctrl_t *fctrl = NULL;
	void __user *argp = (void __user *)arg;

	CDBG("[LED_FLASH]@%s\t \n", __func__);	//ASUS_BSP+++

	if (!sd) {
		pr_err("sd NULL\n");
		return -EINVAL;
	}
	fctrl = v4l2_get_subdevdata(sd);
	if (!fctrl) {
		pr_err("fctrl NULL\n");
		return -EINVAL;
	}
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_flash_get_subdev_id(fctrl, argp);
	case VIDIOC_MSM_FLASH_CFG:
		return msm_flash_config(fctrl, argp);
	case MSM_SD_NOTIFY_FREEZE:
		return 0;
	case MSM_SD_UNNOTIFY_FREEZE:
		return 0;
	case MSM_SD_SHUTDOWN:
		if (!fctrl->func_tbl) {
			pr_err("fctrl->func_tbl NULL\n");
			return -EINVAL;
		} else {
			return fctrl->func_tbl->camera_flash_release(fctrl);
		}
	default:
		pr_err_ratelimited("invalid cmd %d\n", cmd);
		return -ENOIOCTLCMD;
	}
	CDBG("Exit\n");
}

static struct v4l2_subdev_core_ops msm_flash_subdev_core_ops = {
	.ioctl = msm_flash_subdev_ioctl,
};

static struct v4l2_subdev_ops msm_flash_subdev_ops = {
	.core = &msm_flash_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops msm_flash_internal_ops;

static int32_t msm_flash_get_pmic_source_info(
	struct device_node *of_node,
	struct msm_flash_ctrl_t *fctrl)
{
	int32_t rc = 0;
	uint32_t count = 0, i = 0;
	struct device_node *flash_src_node = NULL;
	struct device_node *torch_src_node = NULL;
	struct device_node *switch_src_node = NULL;

	switch_src_node = of_parse_phandle(of_node, "qcom,switch-source", 0);
	if (!switch_src_node) {
		CDBG("%s:%d switch_src_node NULL\n", __func__, __LINE__);
	} else {
		rc = of_property_read_string(switch_src_node,
			"qcom,default-led-trigger",
			&fctrl->switch_trigger_name);
		if (rc < 0) {
			rc = of_property_read_string(switch_src_node,
				"linux,default-trigger",
				&fctrl->switch_trigger_name);
			if (rc < 0)
				pr_err("default-trigger read failed\n");
		}
		of_node_put(switch_src_node);
		switch_src_node = NULL;
		if (!rc) {
			CDBG("switch trigger %s\n",
				fctrl->switch_trigger_name);
			led_trigger_register_simple(
				fctrl->switch_trigger_name,
				&fctrl->switch_trigger);
		}
	}

	if (of_get_property(of_node, "qcom,flash-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("invalid count\n");
			return -EINVAL;
		}
		fctrl->flash_num_sources = count;
		CDBG("%s:%d flash_num_sources = %d",
			__func__, __LINE__, fctrl->flash_num_sources);
		for (i = 0; i < count; i++) {
			flash_src_node = of_parse_phandle(of_node,
				"qcom,flash-source", i);
			if (!flash_src_node) {
				pr_err("flash_src_node NULL\n");
				continue;
			}

			rc = of_property_read_string(flash_src_node,
				"qcom,default-led-trigger",
				&fctrl->flash_trigger_name[i]);
			if (rc < 0) {
				rc = of_property_read_string(flash_src_node,
					"linux,default-trigger",
					&fctrl->flash_trigger_name[i]);
				if (rc < 0) {
					pr_err("default-trigger read failed\n");
					of_node_put(flash_src_node);
					continue;
				}
			}

			CDBG("default trigger %s\n",
				fctrl->flash_trigger_name[i]);

			/* Read operational-current */
			rc = of_property_read_u32(flash_src_node,
				"qcom,current",
				&fctrl->flash_op_current[i]);
			if (rc < 0) {
				pr_err("current: read failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			/* Read max-current */
			rc = of_property_read_u32(flash_src_node,
				"qcom,max-current",
				&fctrl->flash_max_current[i]);
			if (rc < 0) {
				pr_err("current: read failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			/* Read max-duration */
			rc = of_property_read_u32(flash_src_node,
				"qcom,duration",
				&fctrl->flash_max_duration[i]);
			if (rc < 0) {
				pr_err("duration: read failed\n");
				of_node_put(flash_src_node);
				/* Non-fatal; this property is optional */
			}

			of_node_put(flash_src_node);

			CDBG("max_current[%d] %d\n",
				i, fctrl->flash_op_current[i]);

			led_trigger_register_simple(
				fctrl->flash_trigger_name[i],
				&fctrl->flash_trigger[i]);
		}
		if (fctrl->flash_driver_type == FLASH_DRIVER_DEFAULT)
			fctrl->flash_driver_type = FLASH_DRIVER_PMIC;
		CDBG("%s:%d fctrl->flash_driver_type = %d", __func__, __LINE__,
			fctrl->flash_driver_type);
	}

	if (of_get_property(of_node, "qcom,torch-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("invalid count\n");
			return -EINVAL;
		}
		fctrl->torch_num_sources = count;
		CDBG("%s:%d torch_num_sources = %d",
			__func__, __LINE__, fctrl->torch_num_sources);
		for (i = 0; i < count; i++) {
			torch_src_node = of_parse_phandle(of_node,
				"qcom,torch-source", i);
			if (!torch_src_node) {
				pr_err("torch_src_node NULL\n");
				continue;
			}

			rc = of_property_read_string(torch_src_node,
				"qcom,default-led-trigger",
				&fctrl->torch_trigger_name[i]);
			if (rc < 0) {
				rc = of_property_read_string(torch_src_node,
					"linux,default-trigger",
					&fctrl->torch_trigger_name[i]);
				if (rc < 0) {
					pr_err("default-trigger read failed\n");
					of_node_put(torch_src_node);
					continue;
				}
			}

			CDBG("default trigger %s\n",
				fctrl->torch_trigger_name[i]);

			/* Read operational-current */
			rc = of_property_read_u32(torch_src_node,
				"qcom,current",
				&fctrl->torch_op_current[i]);
			if (rc < 0) {
				pr_err("current: read failed\n");
				of_node_put(torch_src_node);
				continue;
			}

			/* Read max-current */
			rc = of_property_read_u32(torch_src_node,
				"qcom,max-current",
				&fctrl->torch_max_current[i]);
			if (rc < 0) {
				pr_err("current: read failed\n");
				of_node_put(torch_src_node);
				continue;
			}

			of_node_put(torch_src_node);

			CDBG("max_current[%d] %d\n",
				i, fctrl->torch_op_current[i]);

			led_trigger_register_simple(
				fctrl->torch_trigger_name[i],
				&fctrl->torch_trigger[i]);
		}
		if (fctrl->flash_driver_type == FLASH_DRIVER_DEFAULT)
			fctrl->flash_driver_type = FLASH_DRIVER_PMIC;
		CDBG("%s:%d fctrl->flash_driver_type = %d", __func__, __LINE__,
			fctrl->flash_driver_type);
	}

	return 0;
}

static int32_t msm_flash_get_dt_data(struct device_node *of_node,
	struct msm_flash_ctrl_t *fctrl)
{
	int32_t rc = 0;

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	/* Read the sub device */
//ASUS_BSP+++
	if (fctrl->pdev) {
		rc = of_property_read_u32(of_node, "cell-index", &fctrl->pdev->id);
	} else {
		rc = of_property_read_u32(of_node, "cell-index", &fctrl->subdev_id);
	}
//ASUS_BSP---
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	CDBG("subdev id %d\n", fctrl->subdev_id);

	fctrl->flash_driver_type = FLASH_DRIVER_DEFAULT;

	/* Read the CCI master. Use M0 if not available in the node */
	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&fctrl->cci_i2c_master);
	CDBG("%s qcom,cci-master %d, rc %d\n", __func__, fctrl->cci_i2c_master,
		rc);
	if (rc < 0) {
		/* Set default master 0 */
		fctrl->cci_i2c_master = MASTER_0;
		rc = 0;
	} else {
		fctrl->flash_driver_type = FLASH_DRIVER_I2C;
	}

	/* Read the flash and torch source info from device tree node */
	rc = msm_flash_get_pmic_source_info(of_node, fctrl);
	if (rc < 0) {
		pr_err("%s:%d msm_flash_get_pmic_source_info failed rc %d\n",
			__func__, __LINE__, rc);
		return rc;
	}

	/* Read the gpio information from device tree */
	rc = msm_sensor_driver_get_gpio_data(
		&(fctrl->power_info.gpio_conf), of_node);
	if (-ENODEV == rc) {
		pr_notice("No valid flash GPIOs data\n");
		rc = 0;
	} else if (rc < 0) {
		pr_err("Error flash GPIOs rc %d\n", rc);
		return rc;
	}

	if (fctrl->flash_driver_type == FLASH_DRIVER_DEFAULT)
		fctrl->flash_driver_type = FLASH_DRIVER_GPIO;
	CDBG("%s:%d fctrl->flash_driver_type = %d", __func__, __LINE__,
		fctrl->flash_driver_type);

	return rc;
}

#ifdef CONFIG_COMPAT
static long msm_flash_subdev_do_ioctl(
	struct file *file, unsigned int cmd, void *arg)
{
	int32_t i = 0;
	int32_t rc = 0;
	struct video_device *vdev;
	struct v4l2_subdev *sd;
	struct msm_flash_cfg_data_t32 *u32;
	struct msm_flash_cfg_data_t flash_data;
	struct msm_flash_init_info_t32 flash_init_info32;
	struct msm_flash_init_info_t flash_init_info;

	CDBG("[[LED_FLASH]]@%s\t \n", __func__);	//ASUS_BSP+++

	if (!file || !arg) {
		pr_err("%s:failed NULL parameter\n", __func__);
		return -EINVAL;
	}
	vdev = video_devdata(file);
	sd = vdev_to_v4l2_subdev(vdev);
	u32 = (struct msm_flash_cfg_data_t32 *)arg;

	switch (cmd) {
	case VIDIOC_MSM_FLASH_CFG32:
		flash_data.cfg_type = u32->cfg_type;
		for (i = 0; i < MAX_LED_TRIGGERS; i++) {
			flash_data.flash_current[i] = u32->flash_current[i];
			flash_data.flash_duration[i] = u32->flash_duration[i];
		}
		cmd = VIDIOC_MSM_FLASH_CFG;
		switch (flash_data.cfg_type) {
		case CFG_FLASH_OFF:
		case CFG_FLASH_LOW:
		case CFG_FLASH_HIGH:
			flash_data.cfg.settings = compat_ptr(u32->cfg.settings);
			break;
		case CFG_FLASH_INIT:
			flash_data.cfg.flash_init_info = &flash_init_info;
			if (copy_from_user(&flash_init_info32,
				(void *)compat_ptr(u32->cfg.flash_init_info),
				sizeof(struct msm_flash_init_info_t32))) {
				pr_err("%s copy_from_user failed %d\n",
					__func__, __LINE__);
				return -EFAULT;
			}
			flash_init_info.flash_driver_type =
				flash_init_info32.flash_driver_type;
			flash_init_info.slave_addr =
				flash_init_info32.slave_addr;
			flash_init_info.i2c_freq_mode =
				flash_init_info32.i2c_freq_mode;
			flash_init_info.settings =
				compat_ptr(flash_init_info32.settings);
			flash_init_info.power_setting_array =
				compat_ptr(
				flash_init_info32.power_setting_array);
			break;
		default:
			break;
		}
		break;
	case VIDIOC_MSM_FLASH_CFG:
		pr_err("invalid cmd 0x%x received\n", cmd);
		return -EINVAL;
	default:
		return msm_flash_subdev_ioctl(sd, cmd, arg);
	}

	rc =  msm_flash_subdev_ioctl(sd, cmd, &flash_data);
	for (i = 0; i < MAX_LED_TRIGGERS; i++) {
		u32->flash_current[i] = flash_data.flash_current[i];
		u32->flash_duration[i] = flash_data.flash_duration[i];
	}
	CDBG("Exit");
	return rc;
}

static long msm_flash_subdev_fops_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_flash_subdev_do_ioctl);
}
#endif
//ASUS_BSP+++

static void __exit msm_flash_sky81298_i2c_remove(void)
{
	i2c_del_driver(&sky81298_i2c_driver);
	return;
}

static const struct of_device_id sky81298_trigger_dt_match[] = {
	{.compatible = "qcom,flash-qup", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, sky81298_trigger_dt_match);

static const struct i2c_device_id sky81298_i2c_id[] = {
	{"qcom,flash-qup", (kernel_ulong_t)NULL},
	{ }
};

static int msm_flash_sky81298_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int32_t rc = 0;
	struct msm_flash_ctrl_t *flash_ctrl = NULL;

	pr_info("[LED_FLASH]@%s entry\n", __func__);
	if (client == NULL) {
		pr_err("%s: client is null\n", __func__);
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c_check_functionality failed\n", __func__);
		rc = -EINVAL;
	}

	flash_ctrl = kzalloc(sizeof(struct msm_flash_ctrl_t), GFP_KERNEL);
	if (!flash_ctrl) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	memset(flash_ctrl, 0, sizeof(struct msm_flash_ctrl_t));

	rc = msm_flash_get_dt_data(client->dev.of_node, flash_ctrl);
	if (rc < 0) {
		pr_err("%s:%d msm_flash_get_dt_data failed\n",
			__func__, __LINE__);
		kfree(flash_ctrl);
		return -EINVAL;
	}

	flash_ctrl->flash_state = MSM_CAMERA_FLASH_RELEASE;
	flash_ctrl->power_info.dev = &client->dev;
	flash_ctrl->flash_device_type = MSM_CAMERA_I2C_DEVICE;
	flash_ctrl->flash_mutex = &msm_flash_mutex;
	flash_ctrl->flash_i2c_client.i2c_func_tbl = &msm_sensor_qup_func_tbl;

	flash_ctrl->flash_i2c_client.client = client;
	flash_ctrl->flash_i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	// since the rx/tx function in msm_camera_qup_i2c will shift the slave addr right by 1 bit,
	// shift the address left by 1-bit here to prevent mis-match when using.
	flash_ctrl->flash_i2c_client.client->addr <<= 1;
	flash_ctrl->flash_driver_type = FLASH_DRIVER_I2C;

	/* Initialize sub device */
	v4l2_i2c_subdev_init(&flash_ctrl->msm_sd.sd, client, &msm_flash_subdev_ops);
	v4l2_set_subdevdata(&flash_ctrl->msm_sd.sd, flash_ctrl);

	flash_ctrl->msm_sd.sd.internal_ops = &msm_flash_internal_ops;
	flash_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	media_entity_init(&flash_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	flash_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	flash_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_FLASH;
	flash_ctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x1;
	msm_sd_register(&flash_ctrl->msm_sd);

	CDBG("%s:%d flash sd name = %s", __func__, __LINE__,
		flash_ctrl->msm_sd.sd.entity.name);

	msm_cam_copy_v4l2_subdev_fops(&msm_flash_v4l2_subdev_fops);
#ifdef CONFIG_COMPAT
	msm_flash_v4l2_subdev_fops.compat_ioctl32 =
		msm_flash_subdev_fops_ioctl;
#endif
	flash_ctrl->msm_sd.sd.devnode->fops = &msm_flash_v4l2_subdev_fops;

	gfctrl = flash_ctrl;

	create_proc_file();
	msm_debugfs_flash_init(flash_ctrl);
	pr_info("%s end\n", __func__);
	return 0;
}

static struct i2c_driver sky81298_i2c_driver = {
	.id_table = sky81298_i2c_id,
	.probe  = msm_flash_sky81298_i2c_probe,
	.remove = __exit_p(msm_flash_sky81298_i2c_remove),
	.driver = {
		.name = "qcom,flash-qup",
		.owner = THIS_MODULE,
		.of_match_table = sky81298_trigger_dt_match,
	},
};
//ASUS_BSP---


static int32_t msm_flash_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_flash_ctrl_t *flash_ctrl = NULL;
	struct msm_camera_cci_client *cci_client = NULL;

	CDBG("Enter");
	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	flash_ctrl = kzalloc(sizeof(struct msm_flash_ctrl_t), GFP_KERNEL);
	if (!flash_ctrl) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	memset(flash_ctrl, 0, sizeof(struct msm_flash_ctrl_t));

	flash_ctrl->pdev = pdev;

	rc = msm_flash_get_dt_data(pdev->dev.of_node, flash_ctrl);
	if (rc < 0) {
		pr_err("%s:%d msm_flash_get_dt_data failed\n",
			__func__, __LINE__);
		kfree(flash_ctrl);
		return -EINVAL;
	}

	flash_ctrl->flash_state = MSM_CAMERA_FLASH_RELEASE;
	flash_ctrl->power_info.dev = &flash_ctrl->pdev->dev;
	flash_ctrl->flash_device_type = MSM_CAMERA_I2C_DEVICE;	//ASUS_BSP+++
	flash_ctrl->flash_mutex = &msm_flash_mutex;
	flash_ctrl->flash_i2c_client.i2c_func_tbl = &msm_sensor_qup_func_tbl;	//ASUS_BSP+++
	flash_ctrl->flash_i2c_client.cci_client = kzalloc(
		sizeof(struct msm_camera_cci_client), GFP_KERNEL);
	if (!flash_ctrl->flash_i2c_client.cci_client) {
		kfree(flash_ctrl);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	cci_client = flash_ctrl->flash_i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = flash_ctrl->cci_i2c_master;

	/* Initialize sub device */
	v4l2_subdev_init(&flash_ctrl->msm_sd.sd, &msm_flash_subdev_ops);
	v4l2_set_subdevdata(&flash_ctrl->msm_sd.sd, flash_ctrl);

	flash_ctrl->msm_sd.sd.internal_ops = &msm_flash_internal_ops;
	flash_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(flash_ctrl->msm_sd.sd.name,
		ARRAY_SIZE(flash_ctrl->msm_sd.sd.name),
		"msm_camera_flash");
	media_entity_init(&flash_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	flash_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	flash_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_FLASH;
	flash_ctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x1;
	msm_sd_register(&flash_ctrl->msm_sd);

	CDBG("%s:%d flash sd name = %s", __func__, __LINE__,
		flash_ctrl->msm_sd.sd.entity.name);
	msm_cam_copy_v4l2_subdev_fops(&msm_flash_v4l2_subdev_fops);
#ifdef CONFIG_COMPAT
	msm_flash_v4l2_subdev_fops.compat_ioctl32 =
		msm_flash_subdev_fops_ioctl;
#endif
	flash_ctrl->msm_sd.sd.devnode->fops = &msm_flash_v4l2_subdev_fops;

	if (flash_ctrl->flash_driver_type == FLASH_DRIVER_PMIC)
		rc = msm_torch_create_classdev(pdev, flash_ctrl);

	CDBG("probe success\n");
	return rc;
}


MODULE_DEVICE_TABLE(of, msm_flash_dt_match);

static struct platform_driver msm_flash_platform_driver = {
	.probe = msm_flash_platform_probe,
	.driver = {
		.name = "qcom,camera-flash",
		.owner = THIS_MODULE,
		.of_match_table = msm_flash_dt_match,
	},
};

static int __init msm_flash_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = platform_driver_register(&msm_flash_platform_driver);
	if (rc)
		pr_err("platform probe for flash failed");

	return i2c_add_driver(&sky81298_i2c_driver);	//ASUS_BSP+++

}

static void __exit msm_flash_exit_module(void)
{
  platform_driver_unregister(&msm_flash_platform_driver);
  i2c_del_driver(&sky81298_i2c_driver); //ASUS_BSP+++
  return;
}

#define	FLASH_BRIGHTNESS_PROC_FILE	"driver/asus_flash_brightness"
#define	FLASH_STATUS_PROC_FILE	"driver/flash_status"
#define	ASUS_FLASH_CONTROL	"driver/asus_flash_control"
#define	ASUS_TORCH_CONTROL	"driver/asus_torch_control"

static struct proc_dir_entry *flash_brightness_proc_file;
static struct proc_dir_entry *flash_status_proc_file;

static int last_torch_val1;
static int last_torch_val2;
static int last_torch_val3;

static int flash_status;
static bool custom_cci_init = false;
static int32_t msm_flash_i2c_write_custom_current(
  struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
  struct msm_camera_i2c_reg_setting conf_array;
	unsigned i = 0;
	int32_t rc = -EINVAL;
	struct msm_camera_i2c_reg_array flash_current_data[3];
	struct msm_camera_i2c_reg_array torch_current_data[3];
	int flash_cmd = 0;
	if (!flash_data) {
		return 0;
	}
	flash_cmd = flash_data->cfg_type;
	switch (flash_cmd) {
		case CFG_FLASH_LOW:
			for(i = 0; i < 3; i++) {
				torch_current_data[i].reg_addr = 0x05 + i;
				torch_current_data[i].reg_data = ((flash_data->flash_current[i] & 0x1f));
				torch_current_data[i].delay = 0x00;
				pr_info("%s:%d reg_addr: 0x%x, reg_data: 0x%x\n", __func__, __LINE__,
						torch_current_data[i].reg_addr, torch_current_data[i].reg_data);
			}
			conf_array.reg_setting = torch_current_data;
			conf_array.size = 3;
			conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
			conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
			conf_array.delay = 0;
			rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
		&flash_ctrl->flash_i2c_client, &conf_array);
			if (rc < 0)
				pr_err("%s:%d I2C write fail.\n", __func__, __LINE__);
			break;
		case CFG_FLASH_HIGH:
			for(i = 0; i < 3; i++) {
				flash_current_data[i].reg_addr = 0x00 + i;
				flash_current_data[i].reg_data = ((flash_data->flash_current[i] & 0x7f));
				flash_current_data[i].delay = 0x00;
				pr_info("%s:%d reg_addr: 0x%x, reg_data: 0x%x\n", __func__, __LINE__,
						flash_current_data[i].reg_addr, flash_current_data[i].reg_data);
			}
			conf_array.reg_setting = flash_current_data;
			conf_array.size = 3;
			conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
			conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
			conf_array.delay = 0;

			rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
		&flash_ctrl->flash_i2c_client, &conf_array);
			if (rc < 0)
				pr_err("%s:%d I2C write fail.\n", __func__, __LINE__);
			break;
		default:
			pr_debug("%s:%d cfg_type %d do not need process here.\n", __func__, __LINE__,flash_data->cfg_type);
			break;
	}
	return rc;
}

static int32_t translate_current_to_reg_data(int32_t flash_current, int8_t flash_mode)
{
	int32_t rc = -EINVAL;
	switch (flash_mode) {
		case 1: // FLASH
			if (flash_current > FLASH_MODULE_MAX_CURRENT) {
				pr_err("%s Invalid flash_current %d\n", __func__, flash_current);
				return rc;
			}
			rc = flash_current / FLASH_MODULE_STEP_CURRENT;
		break;
		case 2: //TORCH
			if (flash_current > FLASH_MODULE_MAX_TORCH_CURRENT) {
				pr_err("%s Invalid torch_current %d\n", __func__, flash_current);
				return rc;
			}
			rc = flash_current / FLASH_MODULE_STEP_CURRENT;
			pr_info("%s original torch_current %d, reg torch_current %d\n", __func__, flash_current, rc);
		break;
	}
	return rc;
}

static int32_t msm_flash_custom_init(struct msm_flash_ctrl_t *flash_ctrl)
{
	struct msm_camera_i2c_reg_setting conf_array;
	uint16_t reg_data = 0;
	unsigned i = 0;
	int32_t rc = 0;
  struct msm_camera_i2c_reg_array data[]=
			{
			  {0x00, 0x43, 0x00},
				{0x01, 0x43, 0x00},
				{0x02, 0x11, 0x00},
				{0x03, 0xFF, 0x00},
				{0x05, 0x11, 0x00},
				{0x06, 0x11, 0x00},
				{0x07, 0x11, 0x00},
				{0x08, 0x00, 0x00},
				{0x0A, 0xA2, 0x00}
			};
	custom_cci_init = true;

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
	conf_array.size = 9;

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

static int32_t msm_flash_custom_off(struct msm_flash_ctrl_t *flash_ctrl)
{
	struct msm_camera_i2c_reg_setting conf_array;
	uint16_t reg_data = 0;
	unsigned i = 0;
	int32_t rc = 0;
	struct msm_camera_i2c_reg_array data;
	last_torch_val1 = 0;
	last_torch_val2 = 0;
	last_torch_val3 = 0;

	conf_array.reg_setting = &data;
	conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;

	conf_array.delay = 0;
	conf_array.reg_setting->reg_addr = 0x08;
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

	custom_cci_init = false;
	return rc;
}
static int32_t msm_flash_asus_led_i2c_write(
  struct msm_flash_ctrl_t *flash_ctrl)
{
  struct msm_camera_i2c_reg_setting conf_array;
	int32_t rc = -EINVAL;
	struct msm_camera_i2c_reg_array torch_current_data;
	torch_current_data.reg_addr = 0x08;
	torch_current_data.reg_data = 0x15;
	torch_current_data.delay = 0x00;
	pr_info("%s:%d reg_addr: 0x%x, reg_data: 0x%x\n", __func__, __LINE__,
	torch_current_data.reg_addr, torch_current_data.reg_data);
	conf_array.reg_setting = &torch_current_data;
	conf_array.size = 1;
	conf_array.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	conf_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
	conf_array.delay = 0;
	rc = flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
				&flash_ctrl->flash_i2c_client, &conf_array);
	if (rc < 0)
		pr_err("%s:%d I2C write fail.\n", __func__, __LINE__);
	return rc;
}

static int msm_flash_brightness_proc_read(struct seq_file *buf, void *v)
{
		seq_printf(buf, "%d %d %d\n", last_torch_val1, last_torch_val2, last_torch_val3);
		return 0;
}

static int msm_flash_brightness_proc_open(struct inode *inode, struct  file *file)
{
		return single_open(file, msm_flash_brightness_proc_read, NULL);
}

static ssize_t msm_flash_brightness_proc_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
		int cur_torch_val1 = -1, cur_torch_val2 = -1, cur_torch_val3 = -1;
		int MAX_FLASHLIGHT_CURRENT = MAX_TORCH_CURRENT;
		int rc = 0, len;
		struct msm_flash_ctrl_t *flash_ctrl = gfctrl;
		struct msm_flash_cfg_data_t flash_data;
		len = (count > DBG_TXT_BUF_SIZE - 1)? (DBG_TXT_BUF_SIZE-1):(count);
		if (copy_from_user(debugTxtBuf, buf, len))
			return -EFAULT;
		debugTxtBuf[len] = 0; //add string end
		sscanf(debugTxtBuf, "%d %d %d", &cur_torch_val1, &cur_torch_val2, &cur_torch_val3);
		*ppos=len;
		if (cur_torch_val1 <= 0 && cur_torch_val2 <= 0 && cur_torch_val3 <= 0) {
			if (flash_ctrl->flash_state == MSM_CAMERA_FLASH_INIT && custom_cci_init) {
				rc = msm_flash_custom_off(flash_ctrl);
				if (rc < 0) {
					pr_debug("%s:%d [AsusLedInterface] camera_flash_off failed rc = %d",
							__func__, __LINE__, rc);
					return rc;
				}
				rc = msm_flash_i2c_release(flash_ctrl);
				if (rc < 0) {
					pr_debug("%s:%d [AsusLedInterface] camera_flash_release failed rc = %d",
							__func__, __LINE__, rc);
					return rc;
				}
				flash_status = 0;
			}
			return len;
		}

		if (last_torch_val1 == cur_torch_val1 && last_torch_val2 == cur_torch_val2 && last_torch_val3 == cur_torch_val3) {
				pr_info("[Asus Torch] input value does not change, do nothing\n");
			return len;
		}
		if (last_torch_val1 > 99 || last_torch_val2 > 99 || last_torch_val3 > 99) {
				pr_info("[Asus Torch] input value over the limit, do nothing\n");
		}
		last_torch_val1 = cur_torch_val1;
		last_torch_val2 = cur_torch_val2;
		last_torch_val3 = cur_torch_val3;
		if (cur_torch_val1 != 0 || cur_torch_val2 != 0 || cur_torch_val3 != 0)
				flash_status = 1;

		if (flash_ctrl->flash_state != MSM_CAMERA_FLASH_INIT || !custom_cci_init) {
				rc = msm_flash_custom_init(flash_ctrl);
				if (rc < 0) {
						pr_debug("%s:%d [AsusLedInterface] camera_flash_init failed rc = %d", __func__, __LINE__, rc);
						rc = msm_flash_i2c_release(flash_ctrl);
						if (rc < 0) {
								pr_debug("%s:%d [AsusLedInterface] camera_flash_release failed rc = %d",
										__func__, __LINE__, rc);
								return rc;
						}
				return rc;
				}
		}
		/*Set input current mA +++*/
		{
				cur_torch_val1 = cur_torch_val1 * MAX_FLASHLIGHT_CURRENT / 99;
				cur_torch_val2 = cur_torch_val2 * MAX_FLASHLIGHT_CURRENT / 99;
				cur_torch_val3 = cur_torch_val3 * MAX_FLASHLIGHT_CURRENT / 99;
		}
		/*Set input current mA ---*/

		if (0 < cur_torch_val1 || 0 < cur_torch_val2 || 0 < cur_torch_val3) {
				flash_data.flash_current[0] = translate_current_to_reg_data(cur_torch_val1, 2);
				flash_data.flash_current[1] = translate_current_to_reg_data(cur_torch_val2, 2);
				flash_data.flash_current[2] = translate_current_to_reg_data(cur_torch_val3, 2);
				flash_data.cfg_type = CFG_FLASH_LOW;
				if (flash_data.flash_current[1] < 0) {
						pr_debug("%s:%d [AsusLedInterface] flash_current reg_data invalid",__func__, __LINE__);
						return len;
				}
				if (flash_ctrl->flash_state != MSM_CAMERA_FLASH_INIT || !custom_cci_init) {
						rc = msm_flash_custom_off(flash_ctrl);
						if (rc < 0) {
								pr_err("%s:%d [AsusLedInterface] camera_flash_off failed rc = %d",
										__func__, __LINE__, rc);
								return rc;
						}
				} else {
						msm_flash_i2c_write_custom_current(flash_ctrl, &flash_data);
						msm_flash_asus_led_i2c_write(flash_ctrl);
				}
		}
		return len;
}

static const struct file_operations flash_brightness_fops = {
	.owner = THIS_MODULE,
	.open = msm_flash_brightness_proc_open,
	.read = seq_read,
	.write = msm_flash_brightness_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int flash_status_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", flash_status);
	return 0;
}

static int flash_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, flash_status_read, NULL);
}

static const struct file_operations flash_status_proc_fops = {
	.open		= flash_status_open,
	.read		= seq_read,
};
static ssize_t enable_flash(struct msm_flash_ctrl_t* flash_ctrl, struct msm_camera_power_ctrl_t *power_info) {
	int32_t rc = 0;
	if( flash_ctrl->flash_state != MSM_CAMERA_FLASH_INIT ){
		//  set gpio high , to write i2c
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_HIGH);

		rc = msm_flash_i2c_write_table(flash_ctrl, &settings_init);
		if (rc < 0) {
			pr_err("%s:%d msm_flash_i2c_write_table rc = %d failed\n",
				__func__, __LINE__, rc);
			 return rc;
		}
		flash_ctrl->flash_state = MSM_CAMERA_FLASH_INIT;
	}
	return 0;
}

static ssize_t disable_flash(struct msm_flash_ctrl_t* flash_ctrl, struct msm_camera_power_ctrl_t *power_info) {
	int32_t rc = 0;
	if( flash_ctrl->flash_state != MSM_CAMERA_FLASH_RELEASE )
	{
		rc = msm_flash_i2c_write_table(flash_ctrl, &settings_off);
		if (rc < 0) {
			pr_err("%s:%d msm_flash_i2c_write_table rc = %d failed\n",
				__func__, __LINE__, rc);
			return rc;
		}
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_LOW);
		flash_ctrl->flash_state = MSM_CAMERA_FLASH_RELEASE;
	}
	return 0;
}

static ssize_t flash_control_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
	int32_t rc = 0, len;
	struct msm_flash_ctrl_t *flash_ctrl = gfctrl;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int flash_current = -1, i = 0;
	u32 led_ctrl = 0;
	u32 led_setting[3] = {0};
	u32 flash_control = 0;
	len = (count > DBG_TXT_BUF_SIZE - 1)? (DBG_TXT_BUF_SIZE-1):(count);
	if (copy_from_user(debugTxtBuf, buf, len))
			return -EFAULT;
	debugTxtBuf[len] = 0; //add string end
	/* led_ctrl : Open how many leds.
	*             1. led_ctrl = 2 means turn on white/yellow leds simultaneously.
	*             2. led_ctrl = 1 means turn on white only
	*             3. led_ctrl = 0 means turn off the leds.
	*  led_setting[0]: current for led 1 (Yellow)
	*  led_setting[1]: current for led 2 (White)
	*  led_setting[2]: current for led 3 (No use)  */
	sscanf(debugTxtBuf, "%d %d %d %d", &led_ctrl, &led_setting[0], &led_setting[1], &led_setting[2]);
	*ppos=len;
	printk("[LED_FLASH]@%s:%d led_ctrl = %d, led_setting[0] = %d , led_setting[1] = %d, led_setting[2] = %d \n",
	         __func__, __LINE__, led_ctrl, led_setting[0], led_setting[1], led_setting[2]);

	if( led_ctrl > 2 || led_ctrl < 0 )
	{
		pr_err("%s:%d led_ctrl > 1 || led_ctrl < 0\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (!flash_ctrl) {
		pr_err("%s:%d flash_ctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	power_info = &(flash_ctrl->power_info);
	if(!power_info) {
		pr_err("%s:%d power_info NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if( led_ctrl == 1) {
		//  set led init
		rc = enable_flash(flash_ctrl, power_info);
		if (rc < 0) return rc;

		if (led_setting[0] == 1) flash_control |= LED1_FLASH_ENABLE;
		if (led_setting[1] == 1) flash_control |= LED2_FLASH_ENABLE;
		if (led_setting[2] == 1) flash_control |= LED3_FLASH_ENABLE;
		settings_brightness.reg_setting_a[0].reg_data = flash_control;
		rc = msm_flash_i2c_write_table(flash_ctrl, &settings_brightness);
		if (rc < 0) {
			pr_err("%s:%d msm_flash_i2c_write_table rc = %d failed\n",
				__func__, __LINE__, rc);
			return rc;
		}
	}
	else if( led_ctrl == 0)
	{
		rc = disable_flash(flash_ctrl, power_info);
		if (rc < 0) return rc;
	}
	else if(led_ctrl == 2)
	{
		rc = enable_flash(flash_ctrl, power_info);
		if (rc < 0) return rc;

		for (i = 0; i < 3; i++) {
			if (led_setting[i] > 0) {
				flash_current = led_setting[i] / FLASH_CURRENT_STEP;
				flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write(
					&flash_ctrl->flash_i2c_client,
					get_flash_current_addr(i), flash_current,
					MSM_CAMERA_I2C_BYTE_DATA);
			}
		}
	}

	return len;
}

static const struct file_operations flash_control_fops = {
	.owner = THIS_MODULE,
	.open = msm_flash_brightness_proc_open,
	.read = seq_read,
	.write = flash_control_write,
	.llseek = seq_lseek,
	.release = single_release,
};
static ssize_t torch_control_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
	// gfctrl
	int32_t rc = 0, len;
	struct msm_flash_ctrl_t *flash_ctrl = gfctrl;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int flash_current = -1, i = 0;
	u32 led_ctrl = 0;
	u32 led_setting[3] = {0};
	u32 flash_control = 0;
	len = (count > DBG_TXT_BUF_SIZE - 1)? (DBG_TXT_BUF_SIZE-1):(count);
	if (copy_from_user(debugTxtBuf, buf, len))
			return -EFAULT;
	debugTxtBuf[len] = 0; //add string end
	sscanf(debugTxtBuf, "%d %d %d %d", &led_ctrl, &led_setting[0], &led_setting[1], &led_setting[2]);
	*ppos=len;
	printk("[LED_FLASH]@%s:%d led_ctrl = %d, led_setting[0] = %d , led_setting[1] = %d, led_setting[2] = %d \n",
	    __func__, __LINE__, led_ctrl, led_setting[0], led_setting[1], led_setting[2]);

	if( led_ctrl > 2 || led_ctrl < 0 )
	{
		pr_err("%s:%d led_ctrl > 1 || led_ctrl < 0\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (!flash_ctrl) {
		pr_err("%s:%d flash_ctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	power_info = &(flash_ctrl->power_info);
	if(!power_info) {
		pr_err("%s:%d power_info NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if( led_ctrl == 1){
		//  set led init
		rc = enable_flash(flash_ctrl, power_info);
		if (rc < 0) return rc;

		if (led_setting[0] == 1) flash_control |= LED1_TORCH_ENABLE;
		if (led_setting[1] == 1) flash_control |= LED2_TORCH_ENABLE;
		if (led_setting[2] == 1) flash_control |= LED3_TORCH_ENABLE;
		settings_brightness.reg_setting_a[0].reg_data = flash_control;
		rc = msm_flash_i2c_write_table(flash_ctrl, &settings_brightness);
		if (rc < 0) {
			pr_err("%s:%d msm_flash_i2c_write_table rc = %d failed\n",
				__func__, __LINE__, rc);
			return rc;
		}
	}
	else if( led_ctrl == 0)
	{
		//  set led off
		rc = disable_flash(flash_ctrl, power_info);
		if (rc < 0) return rc;
	}
	else if(led_ctrl == 2)
	{
		rc = enable_flash(flash_ctrl, power_info);
		if (rc < 0) return rc;

		for (i = 0; i < 3; i++) {
			if (led_setting[i] > 0) {
				flash_current = led_setting[i] / FLASH_CURRENT_STEP;
				flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write(
					&flash_ctrl->flash_i2c_client,
					get_torch_current_addr(i), flash_current,
					MSM_CAMERA_I2C_BYTE_DATA);
			}
		}
	}

	return len;
}

static const struct file_operations torch_control_fops = {
	.owner = THIS_MODULE,
	.open = msm_flash_brightness_proc_open,
	.read = seq_read,
	.write = torch_control_write,
	.llseek = seq_lseek,
	.release = single_release,
};


static void create_proc_file(void)
{
		flash_brightness_proc_file = proc_create(FLASH_BRIGHTNESS_PROC_FILE, 0666, NULL, &flash_brightness_fops);
		if (flash_brightness_proc_file) {
				printk("%s flash_brightness_proc_file sucessed!\n", __func__);
		} else {
				printk("%s flash_brightness_proc_file failed!\n", __func__);
		}
		flash_status_proc_file = proc_create(FLASH_STATUS_PROC_FILE, 0666, NULL, &flash_status_proc_fops);
		if (flash_status_proc_file) {
				printk("%s flash_status_proc_file sucessed!\n", __func__);
		} else {
				printk("%s flash_status_proc_file failed!\n", __func__);
		}
		proc_create(ASUS_FLASH_CONTROL, 0666, NULL, &flash_control_fops);
		proc_create(ASUS_TORCH_CONTROL, 0666, NULL, &torch_control_fops);
}

static struct msm_flash_table msm_pmic_flash_table = {
	.flash_driver_type = FLASH_DRIVER_PMIC,
	.func_tbl = {
		.camera_flash_init = NULL,
		.camera_flash_release = msm_flash_release,
		.camera_flash_off = msm_flash_off,
		.camera_flash_low = msm_flash_low,
		.camera_flash_high = msm_flash_high,
	},
};

static struct msm_flash_table msm_gpio_flash_table = {
	.flash_driver_type = FLASH_DRIVER_GPIO,
	.func_tbl = {
		.camera_flash_init = msm_flash_gpio_init,
		.camera_flash_release = msm_flash_release,
		.camera_flash_off = msm_flash_off,
		.camera_flash_low = msm_flash_low,
		.camera_flash_high = msm_flash_high,
	},
};

static struct msm_flash_table msm_i2c_flash_table = {
	.flash_driver_type = FLASH_DRIVER_I2C,
	.func_tbl = {
		.camera_flash_init = msm_flash_i2c_init,
		.camera_flash_release = msm_flash_i2c_release,
		.camera_flash_off = msm_flash_i2c_write_setting_array,
		.camera_flash_low = msm_flash_i2c_write_setting_array,
		.camera_flash_high = msm_flash_i2c_write_setting_array,
	},
};

module_init(msm_flash_init_module);
module_exit(msm_flash_exit_module);
MODULE_DESCRIPTION("MSM FLASH");
MODULE_LICENSE("GPL v2");
