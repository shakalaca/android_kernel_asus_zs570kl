#include "msm_camera_i2c.h"

static struct msm_camera_i2c_reg_array g_reg_array[] = 
{
    {
	   .reg_addr = 0x3378,
	   .delay = 0,
	},
	{
	   .reg_addr = 0x3379,
	   .delay = 0,
	},
	{
	   .reg_addr = 0x337A,
	   .delay = 0,
	},
	{
	   .reg_addr = 0x337B,
	   .delay = 0,
	},
	{
	   .reg_addr = 0x3374,
	   .reg_data = 0x04,
	   .delay = 0,
	},
	{
	   .reg_addr = 0x3370,
	   .reg_data = 0x81,
	   .delay = 0,
	},
};

static struct msm_camera_i2c_reg_setting imx318_bypass_g_reg_setting = 
{
    .reg_setting = &g_reg_array[0],
    .size = 6,
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};