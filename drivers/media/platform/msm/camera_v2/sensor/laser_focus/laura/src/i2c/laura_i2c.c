/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-08
*
*/

#include "laura_i2c.h"
#include "show_log.h"
#include <linux/i2c.h>

#define I2C_RETRY_COUNT 3
#define I2C_SUPPORT_INDIRECT_MODE	1
#define I2C_SUPPORT_SPECIAL_COMMAND	1

#if 0
/** @brief laura indirect address read
*
*       @param dev_t the laser focus controller
*
*/
int Laura_device_indirect_addr_read(struct msm_laser_focus_ctrl_t *dev_t, uint16_t *i2c_read_data, uint32_t num_word){
       int status = 0;
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
       return status;
}
#endif

/** @brief laura indirect address write
*
*	@param dev_t the laser focus controller
*	@param i_reg_addr_lsb the lsb register address which store lsb indirect address
*	@param i_reg_addr_msb the msb register address which store msb indirect address
*	@param indirect_addr the indirect address
*	@param register_addr the register address which store data
*	@param i2c_write_data the write data
*	@param num_word the size of write data
*
*/
int Laura_device_indirect_addr_write(struct msm_laser_focus_ctrl_t *dev_t,
										uint16_t i_reg_addr_lsb, uint16_t i_reg_addr_msb, uint16_t indirect_addr, 
										uint32_t register_addr, uint16_t *i2c_write_data, uint32_t num_word){
	int status;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	status = Laura_write_seq(dev_t, i_reg_addr_lsb, i_reg_addr_msb, indirect_addr, register_addr, i2c_write_data, num_word);
	if (status < 0) {
		LOG_Handler(LOG_ERR, "%s: write register(0x%x) failed\n", __func__, register_addr);
		return status;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

#if I2C_SUPPORT_INDIRECT_MODE
static int I2C_TxData(struct msm_laser_focus_ctrl_t *dev_t, uint8_t *txData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msg[] = {
		{
		 .addr = dev_t->blsp_i2c_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(dev_t->blsp_i2c_client->adapter, msg, 1) > 0)
			break;

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[olivia] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}
#endif

#if I2C_SUPPORT_SPECIAL_COMMAND
static int special_I2C_TxData(struct msm_laser_focus_ctrl_t *dev_t, uint8_t *txData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msg[] = {
		{
		 .addr = 0x45,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(dev_t->blsp_i2c_client->adapter, msg, 1) > 0)
			break;

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[olivia] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

int special_command_write_byte(struct msm_laser_focus_ctrl_t *dev_t, uint8_t data) {
	LOG_Handler(LOG_DBG, "%s: 0x%02x\n", __func__, data);
	return special_I2C_TxData(dev_t, &data,1);
}


int special_command_write_word(struct msm_laser_focus_ctrl_t *dev_t, uint16_t data) {
	uint8_t buf[2];
	buf[0] = data & 0x00FF;
	buf[1] = data >> 8;
	LOG_Handler(LOG_DBG, "%s: 0x%04x\n", __func__, data);
	return special_I2C_TxData(dev_t, buf,2);
}
#endif

/** @brief laura write sequence bytes for indirect address
*
*     @param client
*	@param i_reg_addr_lsb the lsb register address which store lsb indirect address
*	@param i_reg_addr_msb the msb register address which store msb indirect address
*	@param indirect_addr the indirect address
*	@param addr the register address which store data
*	@param data the write data
*	@param num_word the size of write data
*
*/
int32_t Laura_write_seq(struct msm_laser_focus_ctrl_t *dev_t, 
							uint16_t i_reg_addr_lsb, uint16_t i_reg_addr_msb, uint16_t indirect_addr, 
							uint32_t reg_addr, uint16_t *data, uint32_t num_word)
{
	#if 0
	int32_t rc = -EFAULT;
	uint8_t i = 0, j = 0;
	int reg_conf_tbl_size = (num_word*2)+2; 
	struct msm_camera_cci_ctrl cci_ctrl;
	struct msm_camera_i2c_reg_array reg_conf_tbl[reg_conf_tbl_size];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if ((client->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& client->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| num_word == 0)
		return rc;

	LOG_Handler(LOG_DBG, "%s reg addr = 0x%x num bytes: %d\n", __func__, reg_addr, reg_conf_tbl_size);
	
	memset(reg_conf_tbl, 0, (reg_conf_tbl_size) * sizeof(struct msm_camera_i2c_reg_array));

	/* Assing indirect address LSB byte */
	reg_conf_tbl[0].reg_addr = i_reg_addr_lsb;
	reg_conf_tbl[0].reg_data = (indirect_addr&0xFF00)>>8;
	reg_conf_tbl[0].delay = 0;
	/* Assing indirect address MSB byte */
	reg_conf_tbl[1].reg_addr = i_reg_addr_msb;
	reg_conf_tbl[1].reg_data = indirect_addr&0x00FF;
	reg_conf_tbl[1].delay = 0;
	
	if (reg_conf_tbl_size > I2C_SEQ_REG_DATA_MAX) {
		LOG_Handler(LOG_ERR, "%s: num bytes=%d clamped to max supported %d\n", __func__, reg_conf_tbl_size, I2C_SEQ_REG_DATA_MAX);
		reg_conf_tbl_size = I2C_SEQ_REG_DATA_MAX;
	}
	for (i = 2; i < reg_conf_tbl_size; i = i+2) {
		/* Assign data LSB byte */
		reg_conf_tbl[i].reg_addr = reg_addr;
		reg_conf_tbl[i].reg_data = (data[j]&0xFF00)>>8;
		reg_conf_tbl[i].delay = 0;
		/* Assign data MSB byte */
		reg_conf_tbl[i+1].reg_addr = reg_addr;
		reg_conf_tbl[i+1].reg_data = data[j]&0x00FF;
		reg_conf_tbl[i+1].delay = 0;
		j++;
	}
	
	cci_ctrl.cmd = MSM_CCI_I2C_WRITE;
	cci_ctrl.cci_info = client->cci_client;
	cci_ctrl.cfg.cci_i2c_write_cfg.reg_setting = reg_conf_tbl;
	cci_ctrl.cfg.cci_i2c_write_cfg.data_type = MSM_CAMERA_I2C_BYTE_DATA;
	cci_ctrl.cfg.cci_i2c_write_cfg.addr_type = client->addr_type;
	cci_ctrl.cfg.cci_i2c_write_cfg.size = reg_conf_tbl_size;
	rc = v4l2_subdev_call(client->cci_client->cci_subdev, core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	
	rc = cci_ctrl.status;

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
	#else
	int i = 0;
#if I2C_SUPPORT_INDIRECT_MODE
	int cnt, cycle;
	uint8_t TxData[32];
	#define MAX_WORD_LEN	15
#endif

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

#if !I2C_SUPPORT_INDIRECT_MODE
	/* Check patch memory write */
	(void) CCI_I2C_WrByte(dev_t, i_reg_addr_lsb, indirect_addr >> 8);
	(void) CCI_I2C_WrByte(dev_t, i_reg_addr_msb, indirect_addr & 0xFF);

	for (i = 0; i < num_word; i++) {
		CCI_I2C_WrWord(dev_t, I2C_DATA_PORT, data[i]);
	}
#else
	/* Check patch memory write */
	(void) CCI_I2C_WrByte(dev_t, i_reg_addr_lsb, indirect_addr >> 8);
	(void) CCI_I2C_WrByte(dev_t, i_reg_addr_msb, indirect_addr & 0xFF);

	TxData[0] = 0x1A;
	cnt = num_word;
	cycle = 0;
	while (cnt) {
		if (num_word > MAX_WORD_LEN) num_word = MAX_WORD_LEN; // the max data length should not over 32 bytes.
		for (i = 0; i < num_word; i++) {
			// swap data
			TxData[i*2+1]   = data[i + cycle*MAX_WORD_LEN] & 0xFF;
			TxData[i*2+1+1] = data[i + cycle*MAX_WORD_LEN] >> 8;
			LOG_Handler(LOG_REG, "calibration data(%d):(0x%04x)\n", i + cycle*MAX_WORD_LEN, data[i + cycle*MAX_WORD_LEN]);
		}
		I2C_TxData(dev_t, TxData, num_word*2+1);
		cnt -= num_word;
		num_word = cnt;
		cycle++;
	}

#endif

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
	#endif
}
