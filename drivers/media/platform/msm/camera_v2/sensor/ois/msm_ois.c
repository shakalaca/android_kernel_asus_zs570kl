/* Copyright (c) 2014 - 2016, The Linux Foundation. All rights reserved.
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
#include <linux/proc_fs.h>
#include <linux/firmware.h>
#include "msm_sd.h"
#include "msm_ois.h"
#include "msm_cci.h"

DEFINE_MSM_MUTEX(msm_ois_mutex);
#define ADDRESS_OF_ACCEL_GAIN_X						0x828B
#define ADDRESS_OF_ACCEL_GAIN_Y						0x82CB
#define OIS_PROGRAM_FW_NAME							"/etc/firmware/OIS_ProgramFW.bin" //   "OIS_ProgramFW.bin" 
#define OIS_COEFFICIENT_FW_NAME						"/etc/firmware/OIS_CoefficientFW.mem" //   "OIS_CoefficientFW.mem"

#define OIS_PROGRAM_FW_SIZE							4615
#define OIS_COEFFICIENT_FW_SIZE						481
#define OIS_PROGRAM_DOWNLOAD_OP_CODE 0x80
#define OIS_COEFFICIENT_DOWNLOAD_OP_CODE 0x88
#define OIS_SEQUENCE_WRITE_SIZE 	64

#define OIS_SPECIAL_CMD_SETTING						8
#define OIS_CENTERING_OFF_SETTING  					7
#define OIS_CENTERING_ON_SETTING  					6
#define OIS_TEST_MODE_SETTING  						5
#define OIS_CAPTURE_MODE_SETTING  					4
#define OIS_MOVIE_MODE_SETTING 						3
#define OIS_DISABLE_SETTING  						2
#define OIS_ENABLE_SETTING  						1
#define OIS_INIT_SETTING  							0

extern struct mutex *msm_sensor_global_mutex; //ASUS_BSP, jungchi for Pass vcm's cmd through imx318

uint8_t *program_fw;
uint8_t *coefficient_fw;
/*#define MSM_OIS_DEBUG*/
#undef CDBG
#ifdef MSM_OIS_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

static struct v4l2_file_operations msm_ois_v4l2_subdev_fops;
static int32_t msm_ois_power_up(struct msm_ois_ctrl_t *o_ctrl);
static int32_t msm_ois_power_down(struct msm_ois_ctrl_t *o_ctrl);

static struct i2c_driver msm_ois_i2c_driver;
static struct msm_ois_ctrl_t *gfctrl;
static struct class *ois_debug_class;
static struct device *ois_debug_dev;

static ssize_t ois_write_word_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret = 0;
    return ret;
}

static ssize_t ois_write_word_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    uint32_t reg_addr, cmd;
	uint16_t reg_data;
	int32_t rc = 0;

	if( gfctrl == NULL || gfctrl->ois_state != OIS_CCI_READY)
		return count;

#if 1 //ASUS_BSP, jungchi for Pass vcm's cmd through imx318
	mutex_lock(msm_sensor_global_mutex);
#else
	mutex_lock(gfctrl->ois_mutex); 
#endif

    cmd = -1;
    sscanf(buf, "%x", &cmd);
	reg_data = cmd & 0xFFFF;
	reg_addr = (cmd >> 16) & 0xFFFF;

	rc = gfctrl->i2c_client.i2c_func_tbl->i2c_write(&gfctrl->i2c_client, reg_addr, reg_data, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
#if 1 //ASUS_BSP, jungchi for Pass vcm's cmd through imx318
		mutex_unlock(msm_sensor_global_mutex);
#else
		mutex_unlock(gfctrl->ois_mutex);
#endif
		return rc;
	}
	else
    	pr_err("[8996_ois]@%s\t reg_addr is 0x%x, reg_data is 0x%x\n",__func__, reg_addr, reg_data);
#if 1 //ASUS_BSP, jungchi for Pass vcm's cmd through imx318
    mutex_unlock(msm_sensor_global_mutex);
#else
    mutex_unlock(gfctrl->ois_mutex);
#endif
    return count;
}
DEVICE_ATTR(ois_debug_write_word, 0664, ois_write_word_show, ois_write_word_store);

static int GYRO_MAX_NEGATIVE_VALUE = 0x8000; /* 0x8000 = -32768 */
static int readData = 0;
static int readDataAddr = 0x8455; /* default: gyro x address */
static ssize_t ois_read_word_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	int tmp = 0, trans_value = 0;

	tmp = readData - GYRO_MAX_NEGATIVE_VALUE;
	if(tmp >= 0) {
		trans_value = GYRO_MAX_NEGATIVE_VALUE - tmp;
		trans_value = 0 - trans_value;
	} else {
		trans_value = readData;
	}
    printk("[8996_ois], @addr=0x%x, read_data(Hex)=%x, trans_value(Dex)=%d", readDataAddr, readData, trans_value);
    return sprintf(buf, "%d,%04X\n", trans_value, readData);
}

static ssize_t ois_read_word_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    uint32_t reg_addr, cmd;
	uint16_t reg_data;
	int32_t rc = 0;

	if( gfctrl == NULL  || gfctrl->ois_state != OIS_CCI_READY)
		return count;
#if 1 //ASUS_BSP, jungchi for Pass vcm's cmd through imx318
	mutex_lock(msm_sensor_global_mutex);
#else
	mutex_lock(gfctrl->ois_mutex);
#endif
    cmd = -1;
    sscanf(buf, "%x", &cmd);
	reg_data = cmd & 0xFFFF;
	reg_addr = (cmd >> 16) & 0xFFFF;
	readDataAddr = reg_addr;
	rc = gfctrl->i2c_client.i2c_func_tbl->i2c_read(&gfctrl->i2c_client, reg_addr, &reg_data, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
#if 1 //ASUS_BSP, jungchi for Pass vcm's cmd through imx318
		mutex_unlock(msm_sensor_global_mutex);
#else
		mutex_unlock(gfctrl->ois_mutex);
#endif
		return rc;
	}
	else {
    	pr_err("[8996_ois]@%s\t reg_addr is 0x%x, reg_data is 0x%x\n",__func__, reg_addr, reg_data);
    	readData = reg_data;
	}
#if 1 //ASUS_BSP, jungchi for Pass vcm's cmd through imx318
	mutex_unlock(msm_sensor_global_mutex);
#else
	mutex_unlock(gfctrl->ois_mutex);
#endif

    return count;
}
DEVICE_ATTR(ois_debug_read_word, 0664, ois_read_word_show, ois_read_word_store);



static ssize_t ois_check_special_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	bool ois_special_cmd_finished = msm_camera_cci_i2c_check_ois_special_cmd();

    return sprintf(buf, "%d\n", ois_special_cmd_finished);
}

static ssize_t ois_check_special_cmd_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
	return count;
}

DEVICE_ATTR(ois_debug_check_special_cmd, 0664, ois_check_special_cmd_show, ois_check_special_cmd_store);


static uint8_t convert_data( uint8_t input_data)
{
	uint8_t output_data = 0;
	switch(input_data)
	{
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			output_data = input_data - 48;
			break;
		case 'A':
		case 'B':
		case 'C':
		case 'D':
		case 'E':
		case 'F':
			output_data = input_data - 65 + 10;
			break;
		case 'a':
		case 'b':
		case 'c':
		case 'd':
		case 'e':
		case 'f':
			output_data = input_data - 97 + 10;
			break;
		default:
			output_data = 0;
			break;
	}
	return output_data;
}

static uint8_t *
msm_ois_load_firmware( const char * name , int size)
{
	uint8_t *memptr;
	uint8_t data1, data2;
	int i, read_size, ret;
	mm_segment_t old_fs;
	struct file *fp = NULL;

	fp = filp_open(name, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		ret = (int)PTR_ERR(fp);
		pr_err("[8996_ois]%s\t  file (%s) fail %d \n",__func__ , name, ret );
		return NULL;	/*No such file or directory*/
	}
	printk("[8996_ois] filename: %s  \n",name);

	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	memptr = kzalloc( size, GFP_KERNEL);
	if (!memptr)
	{
		pr_err("[8996_ois]%s\t %d Error in kzalloc %d \n", __func__, __LINE__, size);
		goto kzalloc_fail;
	}

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		/*
		file format--
		00
		12
		33
		*/
		for( i = 0 ; i < size ; ++i )
		{
			// 00  ,  12  ,  33
			read_size = fp->f_op->read(fp,&data1,1, &fp->f_pos);
			read_size = fp->f_op->read(fp,&data2,1, &fp->f_pos);
			memptr[i] = (convert_data(data1) & 0xF) << 4 | (convert_data(data2) & 0xF);
#if 1  // debug
			if( i < 15 )
				printk("[8996_ois] -- %x %x %x \n",memptr[i],data1, data2);
#endif
			// line of end 
			read_size = fp->f_op->read(fp,&data1,1, &fp->f_pos);
			read_size = fp->f_op->read(fp,&data2,1, &fp->f_pos);
		}
//		printk("[8996_ois]%s\t read_size %d \n", __func__,read_size);
	} else {
		pr_err("[8996_ois]%s\t File (%s) \n", __func__, name);
		goto read_fail;
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	for( i = 0 ; i < 15 ; ++i )
		printk("[8996_ois] ++ %x \n",memptr[size - 1 - i]);

	/* close file */
	filp_close(fp, NULL);

	return memptr;

read_fail:
	kfree(memptr);
kzalloc_fail:
	set_fs(old_fs);
	filp_close(fp, NULL);
	return NULL;
}

static int32_t msm_ois_accel_gain_write(int32_t input_data)
{
	uint32_t x_reg_addr = ADDRESS_OF_ACCEL_GAIN_X , y_reg_addr = ADDRESS_OF_ACCEL_GAIN_Y;
	uint16_t accel_gain = 0;
	int32_t rc = 0;
	uint16_t reg_data = 0;

	if( gfctrl == NULL  || gfctrl->ois_state != OIS_CCI_READY)
		return rc;
	
	if( (input_data < 0x1A44 || input_data > 0x41AB) && input_data != 0x0 )   //  8 cm ~ 20 cm and default value
		return rc;

	accel_gain =  ((input_data & 0xFF ) << 8) | ((input_data >> 8 ) & 0xFF);
	CDBG("[8996_ois_gain]@ 0x%x , 0x%x \n", input_data, accel_gain);

	rc = gfctrl->i2c_client.i2c_func_tbl->i2c_write(&gfctrl->i2c_client, x_reg_addr, accel_gain, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("[8996_ois]@%s failed %d\n", __func__, __LINE__);
		return rc;
	}
	else 
		CDBG("[8996_ois]@%s\t reg_addr is 0x%x, reg_data is 0x%x\n",__func__, x_reg_addr, accel_gain);

	rc = gfctrl->i2c_client.i2c_func_tbl->i2c_write(&gfctrl->i2c_client, y_reg_addr, accel_gain, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("[8996_ois]@%s failed %d\n", __func__, __LINE__);
		return rc;
	}
	else 
		CDBG("[8996_ois]@%s\t reg_addr is 0x%x, reg_data is 0x%x\n",__func__, y_reg_addr, accel_gain);

	//  check data 
	rc = gfctrl->i2c_client.i2c_func_tbl->i2c_read(&gfctrl->i2c_client, y_reg_addr, &reg_data, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
	else 
		if( reg_data != input_data )
    		pr_err("[8996_ois]@%s\t reg_addr is 0x%x, reg_data is 0x%x  input_data is 0x%x \n",__func__, y_reg_addr, reg_data, input_data);

	return rc;
}


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
	uint32_t reg_addr = 0x8200;   // check ois status
	uint16_t reg_data = 0;
	int32_t j = 0, block_count = 0;
	enum msm_camera_i2c_reg_addr_type addr_type_temp;
	struct msm_camera_i2c_seq_reg_array *reg_setting;
	CDBG("Enter\n");

	for (i = 0; i < size; i++) {
		switch (settings[i].i2c_operation) {
		case MSM_OIS_WRITE: {
			switch (settings[i].data_type) {
			case MSM_CAMERA_I2C_BYTE_DATA:
			case MSM_CAMERA_I2C_WORD_DATA:
				if( settings[i].reg_addr == 0x8c && settings[i].reg_data == 0x1 )
					pr_err("[8996_ois_special]%s addr = 0x%x data = 0x%x !!!!!!  \n", __func__, settings[i].reg_addr, settings[i].reg_data);

				rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&o_ctrl->i2c_client,
					settings[i].reg_addr,
					settings[i].reg_data,
					settings[i].data_type);
				// pr_err("%s addr = 0x%x data = 0x%x\n", __func__, settings[i].reg_addr, settings[i].reg_data);
				break;
			case MSM_CAMERA_I2C_DWORD_DATA:
			reg_setting =
			kzalloc(sizeof(struct msm_camera_i2c_seq_reg_array),
				GFP_KERNEL);
				if (!reg_setting)
				{
					pr_err("[8996_ois]%s MSM_CAMERA_I2C_DWORD_DATA - kzalloc fail (%d, %d) \n", __func__, i, size);
					return -ENOMEM;
				}

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
				{
					pr_err("[8996_ois_check]%s MSM_CAMERA_I2C_DWORD_DATA - i2c fail rc %d  (%d) \n", __func__, rc, size);
					return rc;
				}
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
        break;

        case MSM_OIS_READ: {
            uint16_t readData = 0;
			switch (settings[i].data_type) {
			case MSM_CAMERA_I2C_BYTE_DATA:
			case MSM_CAMERA_I2C_WORD_DATA:
				rc = o_ctrl->i2c_client.i2c_func_tbl
						->i2c_read(&o_ctrl->i2c_client,
						settings[i].reg_addr,
						&readData,
						settings[i].data_type);
                if (rc != 0) {
                    pr_err("[8996_ois]MSM_OIS_READ failed");
                } else {
                    pr_err("[8996_ois]%s MSM_OIS_READ: 0x%x , rc %d \n", __func__, readData, rc);
                }
				break;

			default:
				pr_err("Unsupport data type: %d\n",
					settings[i].data_type);
				break;
            }
		}
        break;

        }
        #if 1
				if( settings[i].reg_addr == 0x8261 && settings[i].reg_data == 0x0988 )
				{

					addr_type_temp = o_ctrl->i2c_client.addr_type;
					o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
					pr_err("[8996_ois]%s i = %d !!!!!!!!!!!! \n", __func__, i);
				// PROGRAM
					if( program_fw )
					{
						reg_addr = OIS_PROGRAM_DOWNLOAD_OP_CODE;   // check ois status
						block_count = OIS_PROGRAM_FW_SIZE / OIS_SEQUENCE_WRITE_SIZE;
						pr_err("[8996_ois]%s 0x%x , size %d block_count %d !!!!!!!!!!!! \n", __func__, OIS_PROGRAM_DOWNLOAD_OP_CODE, OIS_PROGRAM_FW_SIZE, block_count);
						for( j = 0 ; j < block_count ; ++j)
						{
							rc = o_ctrl->i2c_client.i2c_func_tbl->
										i2c_write_seq(&o_ctrl->i2c_client,
										reg_addr,
										program_fw + OIS_SEQUENCE_WRITE_SIZE*j,
										OIS_SEQUENCE_WRITE_SIZE );
						}
						if( OIS_PROGRAM_FW_SIZE % OIS_SEQUENCE_WRITE_SIZE != 0 )
						{
							rc = o_ctrl->i2c_client.i2c_func_tbl->
										i2c_write_seq(&o_ctrl->i2c_client,
										reg_addr,
										program_fw + OIS_SEQUENCE_WRITE_SIZE*j,
										OIS_PROGRAM_FW_SIZE % OIS_SEQUENCE_WRITE_SIZE );
						}
					}

				// COEFFICIENT
					if( coefficient_fw )
					{
						reg_addr = OIS_COEFFICIENT_DOWNLOAD_OP_CODE;   // check ois status
						block_count = OIS_COEFFICIENT_FW_SIZE / OIS_SEQUENCE_WRITE_SIZE;
						pr_err("[8996_ois]%s 0x%x , size %d block_count %d !!!!!!!!!!!!!  \n", __func__, OIS_COEFFICIENT_DOWNLOAD_OP_CODE, OIS_COEFFICIENT_FW_SIZE, block_count);
						for( j = 0 ; j < block_count ; ++j)
						{
							rc = o_ctrl->i2c_client.i2c_func_tbl->
										i2c_write_seq(&o_ctrl->i2c_client,
										reg_addr,
										coefficient_fw + OIS_SEQUENCE_WRITE_SIZE*j,
										OIS_SEQUENCE_WRITE_SIZE );
						}
						if( OIS_COEFFICIENT_FW_SIZE % OIS_SEQUENCE_WRITE_SIZE != 0 )
						{
							rc = o_ctrl->i2c_client.i2c_func_tbl->
										i2c_write_seq(&o_ctrl->i2c_client,
										reg_addr,
										coefficient_fw + OIS_SEQUENCE_WRITE_SIZE*j,
										OIS_COEFFICIENT_FW_SIZE % OIS_SEQUENCE_WRITE_SIZE );
						}
					}

					o_ctrl->i2c_client.addr_type = addr_type_temp;
				}
				#endif
		if (rc < 0)
			break;
	}

	reg_addr = 0x8200;   // check ois status
	rc = o_ctrl->i2c_client.i2c_func_tbl
						->i2c_read(&o_ctrl->i2c_client,
						reg_addr,
						&reg_data,
						MSM_CAMERA_I2C_WORD_DATA);
	if( reg_addr == 0x8200 && reg_data == 0xe75 )
		o_ctrl->ois_state = OIS_CCI_READY;
		
	pr_err("[8996_ois_check]%s addr = 0x%x data = 0x%x  state = %d !!!!!!  \n", __func__, reg_addr, reg_data , o_ctrl->ois_state);
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_ois_vreg_control(struct msm_ois_ctrl_t *o_ctrl,
							int config)
{
#if 0
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
#else
	return 0;
#endif
}

static int32_t msm_ois_power_down(struct msm_ois_ctrl_t *o_ctrl)
{
	int32_t rc = 0;
//	enum msm_sensor_power_seq_gpio_t gpio;

	CDBG("Enter\n");
	if (o_ctrl->ois_state != OIS_DISABLE_STATE) {

		rc = msm_ois_vreg_control(o_ctrl, 0);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return rc;
		} 
#if 0      // BSP Leong
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
#endif
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

	if (o_ctrl->ois_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&o_ctrl->i2c_client, MSM_CCI_INIT);
		if (rc < 0)
		{
			pr_err("cci_init failed\n");
			return rc;
		}
	}
	o_ctrl->ois_state = OIS_OPS_ACTIVE;
	CDBG("Exit\n");
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

		rc = msm_ois_write_settings(o_ctrl,
			set_info->ois_params.setting_size,
			settings);
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

	if( !program_fw )
		program_fw = msm_ois_load_firmware( OIS_PROGRAM_FW_NAME, OIS_PROGRAM_FW_SIZE );
	if( !coefficient_fw )
		coefficient_fw = msm_ois_load_firmware( OIS_COEFFICIENT_FW_NAME, OIS_COEFFICIENT_FW_SIZE );
	switch (cdata->cfgtype) {
	case CFG_OIS_INIT:
	printk("[8996_ois]@%s\t type CFG_OIS_INIT %d\n", __func__, cdata->cfgtype);
		rc = msm_ois_init(o_ctrl);
		if (rc < 0)
			pr_err("msm_ois_init failed %d\n", rc);
		break;
	case CFG_OIS_POWERDOWN:
	printk("[8996_ois]@%s\t type CFG_OIS_POWERDOWN %d\n", __func__, cdata->cfgtype);
		rc = msm_ois_power_down(o_ctrl);
		if (rc < 0)
			pr_err("msm_ois_power_down failed %d\n", rc);
		break;
	case CFG_OIS_POWERUP:
	printk("[8996_ois]@%s\t type CFG_OIS_POWERUP %d\n", __func__, cdata->cfgtype);
		rc = msm_ois_power_up(o_ctrl);
		if (rc < 0)
			pr_err("Failed ois power up%d\n", rc);
		break;
	case CFG_OIS_CONTROL:
	printk("[8996_ois]@%s\t type CFG_OIS_CONTROL %d - %d \n", __func__, cdata->cfgtype, cdata->cfg.set_info.ois_params.setting_size);
		rc = msm_ois_control(o_ctrl, &cdata->cfg.set_info);
		if (rc < 0)
			pr_err("Failed ois control%d\n", rc);
		break;
	case CFG_OIS_CONTROL_ACCEL_GAIN:
	CDBG("[8996_ois]@%s\t type CFG_OIS_CONTROL_ACCLE_GAIN %d - %d \n", __func__, cdata->cfgtype, cdata->cfg.data);
		rc = msm_ois_accel_gain_write(cdata->cfg.data);
		if (rc < 0)
			pr_err("Failed ois control%d\n", rc);
		break;
	case CFG_OIS_I2C_WRITE_SEQ_TABLE: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;
	printk("[8996_ois]@%s\t type CFG_OIS_I2C_WRITE_SEQ_TABLE %d\n", __func__, cdata->cfgtype);
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

		if (!conf_array.size) {
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
	/* reset read data */
	readData = 0;
	readDataAddr = 0x8455; /* default: gyro x address */
	msm_camera_cci_i2c_reset_ois_special_cmd();
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
		rc = msm_ois_power_down(o_ctrl);
		if (rc < 0) {
			pr_err("%s:%d OIS Power down failed\n",
				__func__, __LINE__);
		}
		return msm_ois_close(sd, NULL);
	default:
		return -ENOIOCTLCMD;
	}
}

static int32_t msm_ois_power_up(struct msm_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
//	enum msm_sensor_power_seq_gpio_t gpio;

	CDBG("%s called\n", __func__);

	rc = msm_ois_vreg_control(o_ctrl, 1);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}
#if 0      // BSP Leong
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
#endif
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
	printk("[8996_ois]@%s\t  Enter\n", __func__);

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
#if 1 //ASUS_BSP, jungchi for Pass vcm's cmd through imx318
	ois_ctrl_t->ois_mutex = msm_sensor_global_mutex;
#else
	ois_ctrl_t->ois_mutex = &msm_ois_mutex;
#endif
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
	printk("[8996_ois]@%s\t  Exit\n", __func__);

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

	ois_data.cfgtype = u32->cfgtype;

	switch (cmd) {
	case VIDIOC_MSM_OIS_CFG32:
		cmd = VIDIOC_MSM_OIS_CFG;

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

			ois_data.cfgtype = u32->cfgtype;
			ois_data.cfg.settings = &settings;
			parg = &ois_data;
			break;

		case CFG_OIS_CONTROL_ACCEL_GAIN:
			CDBG("[8996_ois]%s\t %d \n", __func__, u32->cfg.data);
			ois_data.cfgtype = u32->cfgtype;
			ois_data.cfg.data = u32->cfg.data;
			parg = &ois_data;
		default:
			parg = &ois_data;
			break;
		}
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
	printk("[8996_ois]@%s\t  Enter\n", __func__);

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
#if 0
	rc = msm_sensor_driver_get_gpio_data(&(msm_ois_t->gconf),
		(&pdev->dev)->of_node);
	if (rc < 0) {
		pr_err("%s: No/Error OIS GPIO\n", __func__);
	} else {
		msm_ois_t->cam_pinctrl_status = 1;
		rc = msm_camera_pinctrl_init(
			&(msm_ois_t->pinctrl_info), &(pdev->dev));
		if (rc < 0) {
			pr_err("ERR:%s: Error in reading OIS pinctrl\n",
				__func__);
			msm_ois_t->cam_pinctrl_status = 0;
		}
	}
#endif
	msm_ois_t->ois_v4l2_subdev_ops = &msm_ois_subdev_ops;
#if 1 //ASUS_BSP, jungchi for Pass vcm's cmd through imx318
	msm_ois_t->ois_mutex = msm_sensor_global_mutex;
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
		pr_err("ERR:%s: failed no memory\n",__func__);
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


    ois_debug_class = class_create(THIS_MODULE, "ois_debug");
    ois_debug_dev = device_create(ois_debug_class,
            NULL, 0, "%s", "ois_debug");
	(void) device_create_file(ois_debug_dev, &dev_attr_ois_debug_write_word);
	(void) device_create_file(ois_debug_dev, &dev_attr_ois_debug_read_word);
	(void) device_create_file(ois_debug_dev, &dev_attr_ois_debug_check_special_cmd);

	gfctrl = msm_ois_t;
	printk("[8996_ois]@%s\t  Exit\n", __func__);
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
