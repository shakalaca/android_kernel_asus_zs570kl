/* Copyright (c) 2014-2017, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/firmware.h>
#include "msm_sd.h"
#include "msm_ois.h"
#include "msm_cci.h"
#include "../debugfs/msm_debugfs.h"

DEFINE_MSM_MUTEX(msm_ois_mutex);
extern struct mutex *msm_cci0_sensor_mutex;

/*#define MSM_OIS_DEBUG*/
#undef CDBG
#ifdef MSM_OIS_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif
#define MAX_POLL_COUNT 100

#define PROGRAM_DOWNLOAD_OIS_OP_CODE 0x80
#define COEFFICIENT_DOWNLOAD_OIS_OP_CODE 0x88
#define PROGRAM_DOWNLOAD_TRNS_SIZE 32

static int PROGRAM_DOWNLOAD_OIS_FW[5000];
static int COEFFICIENT_DOWNLOAD_OIS_FW[1000];
static int PROGRAM_DOWNLOAD_OIS_FW_LENGTH;
static int COEFFICIENT_DOWNLOAD_OIS_FW_LENGTH;

static struct v4l2_file_operations msm_ois_v4l2_subdev_fops;
static int32_t msm_ois_power_up(struct msm_ois_ctrl_t *o_ctrl);
static int32_t msm_ois_power_down(struct msm_ois_ctrl_t *o_ctrl);
static int Sysfs_read_byte_seq(char *filename, int *value, int size);

static struct i2c_driver msm_ois_i2c_driver;

static struct reg_settings_ois_t ois_disable_setting_array[] = {
	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x0C0C, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
};

static struct reg_settings_ois_t ois_movie_setting_array[] = {
#if 1
	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x0C0C, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8436, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xF87F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8440, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xF07F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8443, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xB41E, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x841B, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xB000, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84B6, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xF87F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84C0, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xF07F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84C3, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xB41E, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x849B, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xB000, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8438, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x051A, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84B8, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x051A, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8447, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x4317, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84C7, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x4317, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
#endif
	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x0D0D, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
};

static struct reg_settings_ois_t ois_still_setting_array[] = {
#if 1
	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x0C0C, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8436, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xF87F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8440, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xF07F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8443, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xB41E, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x841B, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xB000, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84B6, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xF87F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84C0, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xF07F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84C3, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xB41E, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x849B, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xB000, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8438, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x051A, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84B8, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x051A, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8447, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x4317, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84C7, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x4317, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
#endif
	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x0D0D, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
};

static struct reg_settings_ois_t ois_test_setting_array[] = {
#if 0
	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x0C0C, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8436, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xFF7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8440, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xFF7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8443, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xFF7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x841B, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x8000, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84B6, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xFF7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84C0, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xFF7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84C3, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xFF7F, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x849B, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x8000, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8438, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x5209, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84B8, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x5209, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x8447, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xF240, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
	{.reg_addr = 0x84C7, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0xF240, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
#endif
	{.reg_addr = 0x847F, .addr_type = MSM_CAMERA_I2C_WORD_ADDR, 0x0D0D, .data_type = MSM_CAMERA_I2C_WORD_DATA, .i2c_operation = MSM_OIS_WRITE, .delay = 0},
};

static int32_t msm_ois_download(struct msm_ois_ctrl_t *o_ctrl)
{
	uint16_t bytes_in_tx = 0;
	uint16_t total_bytes = 0;
	uint8_t *ptr = NULL;
	int32_t rc = 0;
	const struct firmware *fw = NULL;
	const char *fw_name_prog = NULL;
	const char *fw_name_coeff = NULL;
	char name_prog[MAX_SENSOR_NAME] = {0};
	char name_coeff[MAX_SENSOR_NAME] = {0};
	struct device *dev = &(o_ctrl->pdev->dev);
	enum msm_camera_i2c_reg_addr_type save_addr_type;

	CDBG("Enter\n");
	save_addr_type = o_ctrl->i2c_client.addr_type;
	o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;

	snprintf(name_coeff, MAX_SENSOR_NAME, "%s.coeff",
		o_ctrl->oboard_info->ois_name);

	snprintf(name_prog, MAX_SENSOR_NAME, "%s.prog",
		o_ctrl->oboard_info->ois_name);

	/* cast pointer as const pointer*/
	fw_name_prog = name_prog;
	fw_name_coeff = name_coeff;

	/* Load FW */
	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		dev_err(dev, "Failed to locate %s\n", fw_name_prog);
		o_ctrl->i2c_client.addr_type = save_addr_type;
		return rc;
	}

	total_bytes = fw->size;
	for (ptr = (uint8_t *)fw->data; total_bytes;
		total_bytes -= bytes_in_tx, ptr += bytes_in_tx) {
		bytes_in_tx = (total_bytes > 10) ? 10 : total_bytes;
		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(
			&o_ctrl->i2c_client, o_ctrl->oboard_info->opcode.prog,
			 ptr, bytes_in_tx);
		if (rc < 0) {
			pr_err("Failed: remaining bytes to be downloaded: %d",
				bytes_in_tx);
			/* abort download fw and return error*/
			goto release_firmware;
		}
	}
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		dev_err(dev, "Failed to locate %s\n", fw_name_coeff);
		o_ctrl->i2c_client.addr_type = save_addr_type;
		return rc;
	}
	total_bytes = fw->size;
	for (ptr = (uint8_t *)fw->data; total_bytes;
		total_bytes -= bytes_in_tx, ptr += bytes_in_tx) {
		bytes_in_tx = (total_bytes > 10) ? 10 : total_bytes;
		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(
			&o_ctrl->i2c_client, o_ctrl->oboard_info->opcode.coeff,
			ptr, bytes_in_tx);
		if (rc < 0) {
			pr_err("Failed: remaining bytes to be downloaded: %d",
				total_bytes);
			/* abort download fw*/
			break;
		}
	}
release_firmware:
	release_firmware(fw);
	o_ctrl->i2c_client.addr_type = save_addr_type;

	return rc;
}

static int32_t msm_ois_data_config(struct msm_ois_ctrl_t *o_ctrl,
	struct msm_ois_slave_info *slave_info)
{
	int rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;

	CDBG("Enter\n");
	if (!slave_info) {
		pr_err("failed : invalid slave_info ");
		return -EINVAL;
	}
	/* fill ois slave info*/
	if (strlcpy(o_ctrl->oboard_info->ois_name, slave_info->ois_name,
		sizeof(o_ctrl->oboard_info->ois_name)) < 0) {
		pr_err("failed: copy_from_user");
		return -EFAULT;
	}
	memcpy(&(o_ctrl->oboard_info->opcode), &(slave_info->opcode),
		sizeof(struct msm_ois_opcode));
	o_ctrl->oboard_info->i2c_slaveaddr = slave_info->i2c_addr;

	/* config cci_client*/
	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = o_ctrl->i2c_client.cci_client;
		cci_client->sid =
			o_ctrl->oboard_info->i2c_slaveaddr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->cci_i2c_master = o_ctrl->cci_master;
	} else {
		o_ctrl->i2c_client.client->addr =
			o_ctrl->oboard_info->i2c_slaveaddr;
	}
	o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	CDBG("Exit\n");
	return rc;
}

static int32_t msm_ois_write_settings(struct msm_ois_ctrl_t *o_ctrl,
	uint16_t size, struct reg_settings_ois_t *settings)
{
	int32_t rc = -EFAULT;
	int32_t i = 0;
	struct msm_camera_i2c_seq_reg_array *reg_setting;
	struct msm_camera_i2c_seq_reg_array *reg_setting_seg;
	int16_t j = 0;
	uint16_t read_num = 0;
	uint16_t block_cnt = 0;
	uint16_t total_cnt = 0;
	uint16_t register_array_size = 0;
	uint16_t fw_array_size = 0;
	CDBG("Enter\n");

	for (i = 0; i < size; i++) {
		switch (settings[i].i2c_operation) {
		case MSM_OIS_WRITE: {
			switch (settings[i].data_type) {
			case MSM_CAMERA_I2C_BYTE_DATA:
			case MSM_CAMERA_I2C_WORD_DATA:
				pr_debug("addr=0x%x, reg_data=0x%x, data_type=%d, addr_type=%d, sid=0x%x\n",
							settings[i].reg_addr, settings[i].reg_data,
							settings[i].data_type, o_ctrl->i2c_client.addr_type,
							o_ctrl->i2c_client.cci_client->sid);

				rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
							&o_ctrl->i2c_client,
							settings[i].reg_addr,
							settings[i].reg_data,
							settings[i].data_type);
				break;
			case MSM_CAMERA_I2C_DWORD_DATA:
			reg_setting =
			kzalloc(sizeof(struct msm_camera_i2c_seq_reg_array),
				GFP_KERNEL);
				if (!reg_setting)
					return -ENOMEM;

				reg_setting->reg_addr = settings[i].reg_addr;
				reg_setting->reg_data[0] = (uint8_t)
					((settings[i].reg_data &
					0xFF000000) >> 24);
				reg_setting->reg_data[1] = (uint8_t)
					((settings[i].reg_data &
					0x00FF0000) >> 16);
				reg_setting->reg_data[2] = (uint8_t)
					((settings[i].reg_data &
					0x0000FF00) >> 8);
				reg_setting->reg_data[3] = (uint8_t)
					(settings[i].reg_data & 0x000000FF);
				reg_setting->reg_data_size = 4;
				rc = o_ctrl->i2c_client.i2c_func_tbl->
					i2c_write_seq(&o_ctrl->i2c_client,
					reg_setting->reg_addr,
					reg_setting->reg_data,
					reg_setting->reg_data_size);
				kfree(reg_setting);
				reg_setting = NULL;
				if (rc < 0)
					return rc;
				break;
			case MSM_CAMERA_I2C_NO_DATA:
				pr_debug("%s: MSM_CAMERA_I2C_NO_DATA +++\n", __func__);
				o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
				settings[i].reg_addr = (settings[i].reg_addr & 0xFF00) >> 8;
				settings[i].data_type = MSM_CAMERA_I2C_BYTE_DATA;
				pr_debug("addr=0x%x, reg_data=0x%x, data_type=%d, addr_type=%d, sid=0x%x\n",
						settings[i].reg_addr, settings[i].reg_data,
						settings[i].data_type, o_ctrl->i2c_client.addr_type,
						o_ctrl->i2c_client.cci_client->sid);
				rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
							&o_ctrl->i2c_client,
							settings[i].reg_addr,
							settings[i].reg_data,
							settings[i].data_type);
				o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
				break;
			case MSM_CAMERA_I2C_WRITE_FW_DATA:
				pr_debug("%s: MSM_CAMERA_I2C_WRITE_FW_DATA +++\n", __func__);
				reg_setting_seg = kzalloc(sizeof(struct msm_camera_i2c_seq_reg_array), GFP_KERNEL);
				if (!reg_setting_seg)
					return -ENOMEM;
				if(settings[i].reg_data == 0x0001) {
				   fw_array_size = PROGRAM_DOWNLOAD_OIS_FW_LENGTH;
				} else if(settings[i].reg_data == 0x0002) {
				   fw_array_size = COEFFICIENT_DOWNLOAD_OIS_FW_LENGTH;
				}
				block_cnt = fw_array_size / PROGRAM_DOWNLOAD_TRNS_SIZE + 1;
				total_cnt = block_cnt;
				pr_err("%s: fw_array_size = %d, block_cnt = %d, total_cnt = %d\n", __func__, fw_array_size, block_cnt, total_cnt);
				//usleep(50);
				while(block_cnt > 0) {
					if(block_cnt == 1) {
						register_array_size = fw_array_size % PROGRAM_DOWNLOAD_TRNS_SIZE;
					} else
						register_array_size = PROGRAM_DOWNLOAD_TRNS_SIZE;

					if(register_array_size != 0) {
						if(settings[i].reg_data == 0x0001) {
							reg_setting_seg->reg_addr = PROGRAM_DOWNLOAD_OIS_OP_CODE << 8 |
								PROGRAM_DOWNLOAD_OIS_FW[(total_cnt - block_cnt) * PROGRAM_DOWNLOAD_TRNS_SIZE];
							for(j = 1; j < register_array_size; j++) {
								reg_setting_seg->reg_data[j - 1] = (uint8_t)(0xFF & PROGRAM_DOWNLOAD_OIS_FW[(total_cnt - block_cnt) * PROGRAM_DOWNLOAD_TRNS_SIZE + j]);
							}
						} else if(settings[i].reg_data == 0x0002) {
							reg_setting_seg->reg_addr = COEFFICIENT_DOWNLOAD_OIS_OP_CODE << 8 |
								COEFFICIENT_DOWNLOAD_OIS_FW[(total_cnt - block_cnt) * PROGRAM_DOWNLOAD_TRNS_SIZE];
							for(j = 1; j < register_array_size; j++) {
								reg_setting_seg->reg_data[j - 1] = (uint8_t)(0xFF & COEFFICIENT_DOWNLOAD_OIS_FW[(total_cnt - block_cnt) * PROGRAM_DOWNLOAD_TRNS_SIZE + j]);
							}
						}
						reg_setting_seg->reg_data_size = register_array_size - 1;
						if(register_array_size == 1) {
							o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
							if(settings[i].reg_data == 0x0001) {
								reg_setting_seg->reg_addr = PROGRAM_DOWNLOAD_OIS_OP_CODE;
								reg_setting_seg->reg_data[0] = PROGRAM_DOWNLOAD_OIS_FW[(total_cnt - block_cnt) * PROGRAM_DOWNLOAD_TRNS_SIZE];
							} else if(settings[i].reg_data == 0x0002) {
								reg_setting_seg->reg_addr = COEFFICIENT_DOWNLOAD_OIS_OP_CODE;
								reg_setting_seg->reg_data[0] = COEFFICIENT_DOWNLOAD_OIS_FW[(total_cnt - block_cnt) * PROGRAM_DOWNLOAD_TRNS_SIZE];
							}
							reg_setting_seg->reg_data_size = 1;
						}
						if(settings[i].reg_data == 0x0001 || settings[i].reg_data == 0x0002) {
							rc = o_ctrl->i2c_client.i2c_func_tbl->
								i2c_write_seq(&o_ctrl->i2c_client,
								reg_setting_seg->reg_addr,
								reg_setting_seg->reg_data,
								reg_setting_seg->reg_data_size);
							if (rc < 0) {
								if(register_array_size == 1) {
									o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
								}
								pr_err("%s: MSM_CAMERA_I2C_WRITE_FW_DATA block_cnt = %d rc = %d fail ---\n", __func__, block_cnt, rc);
								kfree(reg_setting_seg);
								reg_setting_seg = NULL;
								return rc;
							}
						} else {
							rc = 0;
						}
						if(register_array_size == 1) {
						    o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
						}
					}
					block_cnt--;
				}
				kfree(reg_setting_seg);
				reg_setting_seg = NULL;
				pr_debug("%s: MSM_CAMERA_I2C_WRITE_FW_DATA ---\n", __func__);
				break;
			default:
				pr_err("Unsupport data type: %d\n",
					settings[i].data_type);
				break;
			}
			if (settings[i].delay > 20)
				msleep(settings[i].delay);
			else if (0 != settings[i].delay)
				usleep_range(settings[i].delay * 1000,
					(settings[i].delay * 1000) + 1000);
		}
			break;
		case MSM_OIS_READ: {
				switch (settings[i].data_type) {
				case MSM_CAMERA_I2C_READ_FW_DATA:
					pr_debug("%s: MSM_CAMERA_I2C_READ_FW_DATA +++\n", __func__);
					if(settings[i].reg_data == 0x0001) {
						PROGRAM_DOWNLOAD_OIS_FW_LENGTH = Sysfs_read_byte_seq("/system/etc/firmware/OIS_ProgramFW.bin", PROGRAM_DOWNLOAD_OIS_FW, ARRAY_SIZE(PROGRAM_DOWNLOAD_OIS_FW));
						pr_err("%s: PROGRAM_DOWNLOAD_OIS_FW_LENGTH = %d\n", __func__, PROGRAM_DOWNLOAD_OIS_FW_LENGTH);
					} else if(settings[i].reg_data == 0x0002) {
						COEFFICIENT_DOWNLOAD_OIS_FW_LENGTH = Sysfs_read_byte_seq("/system/etc/firmware/OIS_CoefficientFW.mem", COEFFICIENT_DOWNLOAD_OIS_FW, ARRAY_SIZE(COEFFICIENT_DOWNLOAD_OIS_FW));
						pr_err("%s: COEFFICIENT_DOWNLOAD_OIS_FW_LENGTH = %d\n", __func__, COEFFICIENT_DOWNLOAD_OIS_FW_LENGTH);
					}
					pr_debug("%s: MSM_CAMERA_I2C_READ_FW_DATA ---\n", __func__);
					rc = 0;
					break;
				case MSM_CAMERA_I2C_BYTE_DATA:
				case MSM_CAMERA_I2C_WORD_DATA:
					rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
						&o_ctrl->i2c_client,
						settings[i].reg_addr,
						&read_num,
						settings[i].data_type);
					pr_err("%s: ois read[0x%x] = 0x%x\n", __func__, settings[i].reg_addr, read_num);
					if(settings[i].reg_data != 0xFFFF &&
						settings[i].reg_data != read_num) {
						pr_err("%s: ois read_num no match expected value = 0x%x\n", __func__, settings[i].reg_data);
					}
					break;
				case MSM_CAMERA_I2C_DWORD_DATA:
					reg_setting_seg =
					kzalloc(sizeof(struct msm_camera_i2c_seq_reg_array),
					GFP_KERNEL);
					if (!reg_setting_seg)
						return -ENOMEM;
					reg_setting_seg->reg_addr = settings[i].reg_addr;
					reg_setting_seg->reg_data[0] = (uint8_t)
						((settings[i].reg_data &
						0xFF000000) >> 24);
					reg_setting_seg->reg_data[1] = (uint8_t)
						((settings[i].reg_data &
						0x00FF0000) >> 16);
					reg_setting_seg->reg_data[2] = (uint8_t)
						((settings[i].reg_data &
						0x0000FF00) >> 8);
					reg_setting_seg->reg_data[3] = (uint8_t)
						(settings[i].reg_data & 0x000000FF);
					reg_setting_seg->reg_data_size = 4;
					rc = o_ctrl->i2c_client.i2c_func_tbl->
						i2c_read_seq(&o_ctrl->i2c_client,
						reg_setting_seg->reg_addr,
						reg_setting_seg->reg_data,
						reg_setting_seg->reg_data_size);
					kfree(reg_setting_seg);
					reg_setting_seg = NULL;
					if (rc < 0)
						return rc;
					break;

				default:
					pr_err("Unsupport data type: %d\n",
						settings[i].data_type);
					break;
				}
				if (settings[i].delay > 20)
					msleep(settings[i].delay);
				else if (0 != settings[i].delay)
					usleep_range(settings[i].delay * 1000,
						(settings[i].delay * 1000) + 1000);
			}
				break;

		case MSM_OIS_POLL: {
			switch (settings[i].data_type) {
			case MSM_CAMERA_I2C_BYTE_DATA:
			case MSM_CAMERA_I2C_WORD_DATA:

				rc = o_ctrl->i2c_client.i2c_func_tbl
					->i2c_poll(&o_ctrl->i2c_client,
					settings[i].reg_addr,
					settings[i].reg_data,
					settings[i].data_type,
					settings[i].delay);
				break;

			default:
				pr_err("Unsupport data type: %d\n",
					settings[i].data_type);
				break;
			}
		}
		}

		if (rc < 0)
			break;
	}

	CDBG("Exit\n");
	return rc;
}

static int32_t msm_ois_vreg_control(struct msm_ois_ctrl_t *o_ctrl,
							int config)
{
	int rc = 0, i, cnt;
	struct msm_ois_vreg *vreg_cfg;

	vreg_cfg = &o_ctrl->vreg_cfg;
	cnt = vreg_cfg->num_vreg;
	if (!cnt)
		return 0;

	if (cnt >= MSM_OIS_MAX_VREGS) {
		pr_err("%s failed %d cnt %d\n", __func__, __LINE__, cnt);
		return -EINVAL;
	}

	for (i = 0; i < cnt; i++) {
		rc = msm_camera_config_single_vreg(&(o_ctrl->pdev->dev),
			&vreg_cfg->cam_vreg[i],
			(struct regulator **)&vreg_cfg->data[i],
			config);
	}
	return rc;
}

static int32_t msm_ois_power_down(struct msm_ois_ctrl_t *o_ctrl)
{
	int32_t rc = 0;
	enum msm_sensor_power_seq_gpio_t gpio;

	CDBG("Enter\n");
	if (o_ctrl->ois_state != OIS_DISABLE_STATE) {

		rc = msm_ois_vreg_control(o_ctrl, 0);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return rc;
		}

		for (gpio = SENSOR_GPIO_AF_PWDM; gpio < SENSOR_GPIO_MAX;
			gpio++) {
			if (o_ctrl->gconf &&
				o_ctrl->gconf->gpio_num_info &&
				o_ctrl->gconf->
					gpio_num_info->valid[gpio] == 1) {
				gpio_set_value_cansleep(
					o_ctrl->gconf->gpio_num_info
						->gpio_num[gpio],
					GPIOF_OUT_INIT_LOW);

				if (o_ctrl->cam_pinctrl_status) {
					rc = pinctrl_select_state(
						o_ctrl->pinctrl_info.pinctrl,
						o_ctrl->pinctrl_info.
							gpio_state_suspend);
					if (rc < 0)
						pr_err("ERR:%s:%d cannot set pin to suspend state: %d",
							__func__, __LINE__, rc);
					devm_pinctrl_put(
						o_ctrl->pinctrl_info.pinctrl);
				}
				o_ctrl->cam_pinctrl_status = 0;
				rc = msm_camera_request_gpio_table(
					o_ctrl->gconf->cam_gpio_req_tbl,
					o_ctrl->gconf->cam_gpio_req_tbl_size,
					0);
				if (rc < 0)
					pr_err("ERR:%s:Failed in selecting state in ois power down: %d\n",
						__func__, rc);
			}
		}

		o_ctrl->i2c_tbl_index = 0;
		o_ctrl->ois_state = OIS_OPS_INACTIVE;
	}
	CDBG("Exit\n");
	return rc;
}

static int msm_ois_init(struct msm_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	CDBG("Enter\n");

	if (!o_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	msm_set_ois_ctrl(o_ctrl);
	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&o_ctrl->i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
	o_ctrl->ois_state = OIS_OPS_ACTIVE;
	CDBG("Exit\n");
	return rc;
}
static int32_t msm_ois_check_status(struct msm_ois_ctrl_t *o_ctrl, bool isBeforeWriteSettings)
{
	int32_t rc = 0;
	struct reg_settings_ois_t settings;
	uint16_t reg_data;
	if (!o_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	settings.reg_addr = 0x8200;
	settings.reg_data = 0x0;
	settings.data_type = MSM_CAMERA_I2C_WORD_DATA;
	rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
							&o_ctrl->i2c_client,
							settings.reg_addr,
							&reg_data,
							settings.data_type);
	settings.reg_data = reg_data;
	if (isBeforeWriteSettings) {
		pr_debug("Before write settings");
		pr_debug("reg_addr: 0x8200, reg_data: 0x%x", settings.reg_data);
	} else {
		pr_debug("After write settings");
		pr_debug("reg_addr: 0x8200, reg_data: 0x%x", settings.reg_data);
	}
	return rc;
}

static int32_t msm_ois_control(struct msm_ois_ctrl_t *o_ctrl,
	struct msm_ois_set_info_t *set_info)
{
	struct reg_settings_ois_t *settings = NULL;
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	CDBG("Enter\n");

	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = o_ctrl->i2c_client.cci_client;
		cci_client->sid =
			set_info->ois_params.i2c_addr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->cci_i2c_master = o_ctrl->cci_master;
		cci_client->i2c_freq_mode = set_info->ois_params.i2c_freq_mode;
	} else {
		o_ctrl->i2c_client.client->addr =
			set_info->ois_params.i2c_addr;
	}
	o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;


	if (set_info->ois_params.setting_size > 0 &&
		set_info->ois_params.setting_size
		< MAX_OIS_REG_SETTINGS) {
		settings = kmalloc(
			sizeof(struct reg_settings_ois_t) *
			(set_info->ois_params.setting_size),
			GFP_KERNEL);
		if (settings == NULL) {
			pr_err("Error allocating memory\n");
			return -EFAULT;
		}
		if (copy_from_user(settings,
			(void *)set_info->ois_params.settings,
			set_info->ois_params.setting_size *
			sizeof(struct reg_settings_ois_t))) {
			kfree(settings);
			pr_err("Error copying\n");
			return -EFAULT;
		}
		msm_ois_check_status(o_ctrl, true);
		rc = msm_ois_write_settings(o_ctrl,
			set_info->ois_params.setting_size,
			settings);
		msm_ois_check_status(o_ctrl, false);
		kfree(settings);
		if (rc < 0) {
			pr_err("Error\n");
			return -EFAULT;
		}
	}

	CDBG("Exit\n");

	return rc;
}


static int32_t msm_ois_config(struct msm_ois_ctrl_t *o_ctrl,
	void __user *argp)
{
	struct msm_ois_cfg_data *cdata =
		(struct msm_ois_cfg_data *)argp;
	int32_t rc = 0;
	mutex_lock(o_ctrl->ois_mutex);
	CDBG("Enter\n");
	CDBG("%s type %d\n", __func__, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_OIS_INIT:
		rc = msm_ois_init(o_ctrl);
		if (rc < 0)
			pr_err("msm_ois_init failed %d\n", rc);
		break;
	case CFG_OIS_POWERDOWN:
		rc = msm_ois_power_down(o_ctrl);
		if (rc < 0)
			pr_err("msm_ois_power_down failed %d\n", rc);
		break;
	case CFG_OIS_POWERUP:
		rc = msm_ois_power_up(o_ctrl);
		if (rc < 0)
			pr_err("Failed ois power up%d\n", rc);
		break;
	case CFG_OIS_CONTROL:
		rc = msm_ois_control(o_ctrl, &cdata->cfg.set_info);
		if (rc < 0)
			pr_err("Failed ois control%d\n", rc);
		break;
	case CFG_OIS_I2C_WRITE_SEQ_TABLE: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

#ifdef CONFIG_COMPAT
		if (is_compat_task()) {
			memcpy(&conf_array,
				(void *)cdata->cfg.settings,
				sizeof(struct msm_camera_i2c_seq_reg_setting));
		} else
#endif
		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.settings,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size ||
			conf_array.size > I2C_SEQ_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = o_ctrl->i2c_client.i2c_func_tbl->
			i2c_write_seq_table(&o_ctrl->i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_OIS_I2C_WRITE_MODE: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
#ifdef CONFIG_COMPAT
		if (is_compat_task()) {
			memcpy(&conf_array,
				(void *)cdata->cfg.settings,
				sizeof(struct msm_camera_i2c_seq_reg_setting));
		} else
#endif
		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.settings,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if(conf_array.delay == 0) {
			rc = msm_ois_write_settings(o_ctrl, ARRAY_SIZE(ois_disable_setting_array),
				ois_disable_setting_array);
		} else if(conf_array.delay == 1) {
			rc = msm_ois_write_settings(o_ctrl, ARRAY_SIZE(ois_movie_setting_array),
				ois_movie_setting_array);
		} else if(conf_array.delay == 2) {
			rc = msm_ois_write_settings(o_ctrl, ARRAY_SIZE(ois_still_setting_array),
				ois_still_setting_array);
		} else if(conf_array.delay == 3) {
			rc = msm_ois_write_settings(o_ctrl, ARRAY_SIZE(ois_test_setting_array),
				ois_test_setting_array);
		}
		break;
	}
	default:
		break;
	}
	mutex_unlock(o_ctrl->ois_mutex);
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_ois_config_download(struct msm_ois_ctrl_t *o_ctrl,
	void __user *argp)
{
	struct msm_ois_cfg_download_data *cdata =
		(struct msm_ois_cfg_download_data *)argp;
	int32_t rc = 0;

	if (!o_ctrl || !cdata) {
		pr_err("failed: Invalid data\n");
		return -EINVAL;
	}
	mutex_lock(o_ctrl->ois_mutex);
	CDBG("Enter\n");
	CDBG("%s type %d\n", __func__, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_OIS_DATA_CONFIG:
		rc = msm_ois_data_config(o_ctrl, &cdata->slave_info);
		if (rc < 0)
			pr_err("Failed ois data config %d\n", rc);
		break;
	case CFG_OIS_DOWNLOAD:
		rc = msm_ois_download(o_ctrl);
		if (rc < 0)
			pr_err("Failed ois download %d\n", rc);
		break;
	default:
		break;
	}
	mutex_unlock(o_ctrl->ois_mutex);
	CDBG("Exit\n");
	return rc;
}


static int32_t msm_ois_get_subdev_id(struct msm_ois_ctrl_t *o_ctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("Enter\n");
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		*subdev_id = o_ctrl->pdev->id;
	else
		*subdev_id = o_ctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("Exit\n");
	return 0;
}
/** @brief read many byte from file
*
*	@param filename the file to write
*	@param value the byte which will store the calibration data from read file
*	@param size the size of write data
*
*/
static int Sysfs_read_byte_seq(char *filename, int *value, int size)
{
	int i = 0;
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[4];
	ssize_t read_size = 0;

	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("%s: open %s fail\n", __func__, filename);
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/

	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		for(i = 0; i < size; i++){
			read_size = fp->f_op->read(fp, buf, 4, &pos_lsts);
			buf[2]='\0';
			if(read_size == 0) {
				break;
			}
			sscanf(buf, "%x", &value[i]);
			//if (i < 20)
			//	pr_err("%s: %s, value[i]=0x%x\n", __func__, buf, value[i]);
		}
	} else {
		/* Set addr_limit of the current process back to its own */
		set_fs(old_fs);

		/* close file */
		filp_close(fp, NULL);
		pr_err("%s: f_op = null or write = null, fail line = %d\n", __func__, __LINE__);

		return -ENXIO;	/*No such device or address*/
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	/* close file */
	filp_close(fp, NULL);

	return i;
}

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq = msm_camera_qup_i2c_write_seq,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
	.i2c_poll = msm_camera_qup_i2c_poll,
};

static int msm_ois_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_ois_ctrl_t *o_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("Enter\n");
	if (!o_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	mutex_lock(o_ctrl->ois_mutex);
	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE &&
		o_ctrl->ois_state != OIS_DISABLE_STATE) {
		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&o_ctrl->i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
	o_ctrl->ois_state = OIS_DISABLE_STATE;
	mutex_unlock(o_ctrl->ois_mutex);
	CDBG("Exit\n");
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_ois_internal_ops = {
	.close = msm_ois_close,
};

static long msm_ois_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	int rc;
	struct msm_ois_ctrl_t *o_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;
	CDBG("Enter\n");
	CDBG("%s:%d o_ctrl %pK argp %pK\n", __func__, __LINE__, o_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_ois_get_subdev_id(o_ctrl, argp);
	case VIDIOC_MSM_OIS_CFG:
		return msm_ois_config(o_ctrl, argp);
	case VIDIOC_MSM_OIS_CFG_DOWNLOAD:
		return msm_ois_config_download(o_ctrl, argp);
	case MSM_SD_SHUTDOWN:
		if (!o_ctrl->i2c_client.i2c_func_tbl) {
			pr_err("o_ctrl->i2c_client.i2c_func_tbl NULL\n");
			return -EINVAL;
		}
		mutex_lock(o_ctrl->ois_mutex);
		rc = msm_ois_power_down(o_ctrl);
		if (rc < 0) {
			pr_err("%s:%d OIS Power down failed\n",
				__func__, __LINE__);
		}
		mutex_unlock(o_ctrl->ois_mutex);
		return msm_ois_close(sd, NULL);
	default:
		return -ENOIOCTLCMD;
	}
}

static int32_t msm_ois_power_up(struct msm_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	enum msm_sensor_power_seq_gpio_t gpio;

	CDBG("%s called\n", __func__);

	rc = msm_ois_vreg_control(o_ctrl, 1);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	for (gpio = SENSOR_GPIO_AF_PWDM;
		gpio < SENSOR_GPIO_MAX; gpio++) {
		if (o_ctrl->gconf && o_ctrl->gconf->gpio_num_info &&
			o_ctrl->gconf->gpio_num_info->valid[gpio] == 1) {
			rc = msm_camera_request_gpio_table(
				o_ctrl->gconf->cam_gpio_req_tbl,
				o_ctrl->gconf->cam_gpio_req_tbl_size, 1);
			if (rc < 0) {
				pr_err("ERR:%s:Failed in selecting state for ois: %d\n",
					__func__, rc);
				return rc;
			}
			if (o_ctrl->cam_pinctrl_status) {
				rc = pinctrl_select_state(
					o_ctrl->pinctrl_info.pinctrl,
					o_ctrl->pinctrl_info.gpio_state_active);
				if (rc < 0)
					pr_err("ERR:%s:%d cannot set pin to active state: %d",
						__func__, __LINE__, rc);
			}

			gpio_set_value_cansleep(
				o_ctrl->gconf->gpio_num_info->gpio_num[gpio],
				1);
		}
	}

	o_ctrl->ois_state = OIS_ENABLE_STATE;
	CDBG("Exit\n");
	return rc;
}

static struct v4l2_subdev_core_ops msm_ois_subdev_core_ops = {
	.ioctl = msm_ois_subdev_ioctl,
};

static struct v4l2_subdev_ops msm_ois_subdev_ops = {
	.core = &msm_ois_subdev_core_ops,
};

static const struct i2c_device_id msm_ois_i2c_id[] = {
	{"qcom,ois", (kernel_ulong_t)NULL},
	{ }
};

static int32_t msm_ois_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_ois_ctrl_t *ois_ctrl_t = NULL;
	CDBG("Enter\n");

	if (client == NULL) {
		pr_err("msm_ois_i2c_probe: client is null\n");
		return -EINVAL;
	}

	ois_ctrl_t = kzalloc(sizeof(struct msm_ois_ctrl_t),
		GFP_KERNEL);
	if (!ois_ctrl_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		rc = -EINVAL;
		goto probe_failure;
	}

	CDBG("client = 0x%pK\n",  client);

	rc = of_property_read_u32(client->dev.of_node, "cell-index",
		&ois_ctrl_t->subdev_id);
	CDBG("cell-index %d, rc %d\n", ois_ctrl_t->subdev_id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		goto probe_failure;
	}

	ois_ctrl_t->i2c_driver = &msm_ois_i2c_driver;
	ois_ctrl_t->i2c_client.client = client;
	/* Set device type as I2C */
	ois_ctrl_t->ois_device_type = MSM_CAMERA_I2C_DEVICE;
	ois_ctrl_t->i2c_client.i2c_func_tbl = &msm_sensor_qup_func_tbl;
	ois_ctrl_t->ois_v4l2_subdev_ops = &msm_ois_subdev_ops;
	ois_ctrl_t->ois_mutex = &msm_ois_mutex;
	/* Assign name for sub device */
	snprintf(ois_ctrl_t->msm_sd.sd.name, sizeof(ois_ctrl_t->msm_sd.sd.name),
		"%s", ois_ctrl_t->i2c_driver->driver.name);

	/* Initialize sub device */
	v4l2_i2c_subdev_init(&ois_ctrl_t->msm_sd.sd,
		ois_ctrl_t->i2c_client.client,
		ois_ctrl_t->ois_v4l2_subdev_ops);
	v4l2_set_subdevdata(&ois_ctrl_t->msm_sd.sd, ois_ctrl_t);
	ois_ctrl_t->msm_sd.sd.internal_ops = &msm_ois_internal_ops;
	ois_ctrl_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&ois_ctrl_t->msm_sd.sd.entity, 0, NULL, 0);
	ois_ctrl_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	ois_ctrl_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_OIS;
	ois_ctrl_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&ois_ctrl_t->msm_sd);
	ois_ctrl_t->ois_state = OIS_DISABLE_STATE;
	pr_info("msm_ois_i2c_probe: succeeded\n");
	CDBG("Exit\n");

probe_failure:
	kfree(ois_ctrl_t);
	return rc;
}

#ifdef CONFIG_COMPAT
static long msm_ois_subdev_do_ioctl(
	struct file *file, unsigned int cmd, void *arg)
{
	long rc = 0;
	struct video_device *vdev;
	struct v4l2_subdev *sd;
	struct msm_ois_cfg_data32 *u32;
	struct msm_ois_cfg_data ois_data;
	void *parg;
	struct msm_camera_i2c_seq_reg_setting settings;
	struct msm_camera_i2c_seq_reg_setting32 settings32;

	if (!file || !arg) {
		pr_err("%s:failed NULL parameter\n", __func__);
		return -EINVAL;
	}
	vdev = video_devdata(file);
	sd = vdev_to_v4l2_subdev(vdev);
	u32 = (struct msm_ois_cfg_data32 *)arg;
	parg = arg;

	switch (cmd) {
	case VIDIOC_MSM_OIS_CFG32:
		cmd = VIDIOC_MSM_OIS_CFG;
		ois_data.cfgtype = u32->cfgtype;

		switch (u32->cfgtype) {
		case CFG_OIS_CONTROL:
			ois_data.cfg.set_info.ois_params.setting_size =
				u32->cfg.set_info.ois_params.setting_size;
			ois_data.cfg.set_info.ois_params.i2c_addr =
				u32->cfg.set_info.ois_params.i2c_addr;
			ois_data.cfg.set_info.ois_params.i2c_freq_mode =
				u32->cfg.set_info.ois_params.i2c_freq_mode;
			ois_data.cfg.set_info.ois_params.i2c_addr_type =
				u32->cfg.set_info.ois_params.i2c_addr_type;
			ois_data.cfg.set_info.ois_params.i2c_data_type =
				u32->cfg.set_info.ois_params.i2c_data_type;
			ois_data.cfg.set_info.ois_params.settings =
				compat_ptr(u32->cfg.set_info.ois_params.
				settings);
			parg = &ois_data;
			break;
		case CFG_OIS_I2C_WRITE_SEQ_TABLE:
			if (copy_from_user(&settings32,
				(void *)compat_ptr(u32->cfg.settings),
				sizeof(
				struct msm_camera_i2c_seq_reg_setting32))) {
				pr_err("copy_from_user failed\n");
				return -EFAULT;
			}

			settings.addr_type = settings32.addr_type;
			settings.delay = settings32.delay;
			settings.size = settings32.size;
			settings.reg_setting =
				compat_ptr(settings32.reg_setting);

			ois_data.cfg.settings = &settings;
			parg = &ois_data;
			break;
		case CFG_OIS_I2C_WRITE_MODE:
			if (copy_from_user(&settings32,
				(void *)compat_ptr(u32->cfg.settings),
				sizeof(
				struct msm_camera_i2c_seq_reg_setting32))) {
				pr_err("copy_from_user failed\n");
				return -EFAULT;
			}

			settings.addr_type = settings32.addr_type;
			settings.delay = settings32.delay;
			settings.size = settings32.size;
			settings.reg_setting =
				compat_ptr(settings32.reg_setting);

			ois_data.cfgtype = u32->cfgtype;
			ois_data.cfg.settings = &settings;
			parg = &ois_data;
			break;
		default:
			parg = &ois_data;
			break;
		}
		break;
	case VIDIOC_MSM_OIS_CFG:
		pr_err("%s: invalid cmd 0x%x received\n", __func__, cmd);
		return -EINVAL;
	}
	rc = msm_ois_subdev_ioctl(sd, cmd, parg);

	return rc;
}

static long msm_ois_subdev_fops_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_ois_subdev_do_ioctl);
}
#endif

static int32_t msm_ois_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_ois_ctrl_t *msm_ois_t = NULL;
	struct msm_ois_vreg *vreg_cfg;
	CDBG("Enter\n");

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	msm_ois_t = kzalloc(sizeof(struct msm_ois_ctrl_t),
		GFP_KERNEL);
	if (!msm_ois_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	msm_ois_t->oboard_info = kzalloc(sizeof(
		struct msm_ois_board_info), GFP_KERNEL);
	if (!msm_ois_t->oboard_info) {
		kfree(msm_ois_t);
		return -ENOMEM;
	}

	rc = of_property_read_u32((&pdev->dev)->of_node, "cell-index",
		&pdev->id);
	CDBG("cell-index %d, rc %d\n", pdev->id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		goto release_memory;
	}

	rc = of_property_read_u32((&pdev->dev)->of_node, "qcom,cci-master",
		&msm_ois_t->cci_master);
	CDBG("qcom,cci-master %d, rc %d\n", msm_ois_t->cci_master, rc);
	if (rc < 0 || msm_ois_t->cci_master >= MASTER_MAX) {
		pr_err("failed rc %d\n", rc);
		goto release_memory;
	}

	if (of_find_property((&pdev->dev)->of_node,
			"qcom,cam-vreg-name", NULL)) {
		vreg_cfg = &msm_ois_t->vreg_cfg;
		rc = msm_camera_get_dt_vreg_data((&pdev->dev)->of_node,
			&vreg_cfg->cam_vreg, &vreg_cfg->num_vreg);
		if (rc < 0) {
			pr_err("failed rc %d\n", rc);
			goto release_memory;
		}
	}

	rc = msm_sensor_driver_get_gpio_data(&(msm_ois_t->gconf),
		(&pdev->dev)->of_node);
	if (-ENODEV == rc) {
		pr_notice("No valid OIS GPIOs data\n");
	} else if (rc < 0) {
		pr_err("Error OIS GPIO\n");
	} else {
		msm_ois_t->cam_pinctrl_status = 1;
		rc = msm_camera_pinctrl_init(
			&(msm_ois_t->pinctrl_info), &(pdev->dev));
		if (rc < 0) {
			pr_err("ERR: Error in reading OIS pinctrl\n");
			msm_ois_t->cam_pinctrl_status = 0;
		}
	}

	msm_ois_t->ois_v4l2_subdev_ops = &msm_ois_subdev_ops;
#if 1
	msm_ois_t->ois_mutex = msm_cci0_sensor_mutex;
#else
	msm_ois_t->ois_mutex = &msm_ois_mutex;
#endif

	/* Set platform device handle */
	msm_ois_t->pdev = pdev;
	/* Set device type as platform device */
	msm_ois_t->ois_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	msm_ois_t->i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	msm_ois_t->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!msm_ois_t->i2c_client.cci_client) {
		kfree(msm_ois_t->vreg_cfg.cam_vreg);
		rc = -ENOMEM;
		goto release_memory;
	}

	cci_client = msm_ois_t->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = msm_ois_t->cci_master;
	v4l2_subdev_init(&msm_ois_t->msm_sd.sd,
		msm_ois_t->ois_v4l2_subdev_ops);
	v4l2_set_subdevdata(&msm_ois_t->msm_sd.sd, msm_ois_t);
	msm_ois_t->msm_sd.sd.internal_ops = &msm_ois_internal_ops;
	msm_ois_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(msm_ois_t->msm_sd.sd.name,
		ARRAY_SIZE(msm_ois_t->msm_sd.sd.name), "msm_ois");
	media_entity_init(&msm_ois_t->msm_sd.sd.entity, 0, NULL, 0);
	msm_ois_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	msm_ois_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_OIS;
	msm_ois_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&msm_ois_t->msm_sd);
	msm_ois_t->ois_state = OIS_DISABLE_STATE;
	msm_cam_copy_v4l2_subdev_fops(&msm_ois_v4l2_subdev_fops);
#ifdef CONFIG_COMPAT
	msm_ois_v4l2_subdev_fops.compat_ioctl32 =
		msm_ois_subdev_fops_ioctl;
#endif
	msm_ois_t->msm_sd.sd.devnode->fops =
		&msm_ois_v4l2_subdev_fops;

	CDBG("Exit\n");
	return rc;
release_memory:
	kfree(msm_ois_t->oboard_info);
	kfree(msm_ois_t);
	return rc;
}

static const struct of_device_id msm_ois_i2c_dt_match[] = {
	{.compatible = "qcom,ois"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_ois_i2c_dt_match);

static struct i2c_driver msm_ois_i2c_driver = {
	.id_table = msm_ois_i2c_id,
	.probe  = msm_ois_i2c_probe,
	.remove = __exit_p(msm_ois_i2c_remove),
	.driver = {
		.name = "qcom,ois",
		.owner = THIS_MODULE,
		.of_match_table = msm_ois_i2c_dt_match,
	},
};

static const struct of_device_id msm_ois_dt_match[] = {
	{.compatible = "qcom,ois", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, msm_ois_dt_match);

static struct platform_driver msm_ois_platform_driver = {
	.probe = msm_ois_platform_probe,
	.driver = {
		.name = "qcom,ois",
		.owner = THIS_MODULE,
		.of_match_table = msm_ois_dt_match,
	},
};

static int __init msm_ois_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = platform_driver_register(&msm_ois_platform_driver);
	if (!rc)
		return rc;
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&msm_ois_i2c_driver);
}

static void __exit msm_ois_exit_module(void)
{
	platform_driver_unregister(&msm_ois_platform_driver);
	i2c_del_driver(&msm_ois_i2c_driver);
	return;
}

module_init(msm_ois_init_module);
module_exit(msm_ois_exit_module);
MODULE_DESCRIPTION("MSM OIS");
MODULE_LICENSE("GPL v2");
