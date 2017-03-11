/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-08
*
*/

#include "laura_interface.h"
#include "show_log.h"

#define DO_MEASURE true

extern int Laser_Product;

extern int ErrCode1;
extern uint16_t Range1;

extern bool keepMeasuring;
extern bool ioctrl_close;

extern bool repairing_state;

/* Calibration input data */
/*
int16_t cal_data_10[CAL_MSG_LEN*2]={3863,3863,3863,3863,3863,3863,3863,3863,3863,2039,2039,2039,2039,2039,2039,2039,2039,2039,2909,1098,539,12850,12850,12850,12850,12850,5,3857,3857,3857,3857,3857,3857,3857,3857,3857,2039,2039,2039,2039,2039,2039,2039,2039,2039,2910,1100,540,12850,12850,12850,12850,12850,5};
int16_t cal_data_40[CAL_MSG_LEN*2]={21,21,21,21,21,21,21,21,21,45,45,45,45,45,45,45,45,45,6500,1094,531,12850,12850,12850,12850,12850,5,21,21,21,21,21,21,21,21,21,45,45,45,45,45,45,45,45,45,6500,1088,536,12850,12850,12850,12850,12850,5};
*/
int16_t cal_data_10[CAL_MSG_LEN*2];
int16_t cal_data_40[CAL_MSG_LEN*2];
int16_t cal_data_inf[CAL_MSG_LEN*2];	/* Calibration infinity input data */
int16_t cal_data_f0[OLIVIA_F0_DATA_LEN];
uint16_t conf_level_10cm;
#if 0
int16_t cal_data_10[12] = {-1231,364,3721,-1029,824,12814,
                            -1222,356,3676,-1000,785,12814};
int16_t cal_data_40[12] =  {-27,-29,6500,-1021,816,12814,
                            -26,-29,6500,-972,764,12814};
int16_t cal_data_inf[12] = {-6,-2,6500,-1025,823,12814,
                            -6,-2,6500,-975,769,12814};
#endif

/** @brief laura calibration interface
*
*	@param dev_t the laser focus controller
*	@param load_cal if load calibration data
*	@param calibration_flag the calibration flag
*	@param ctrl the calibration controller
*
*/

int Laura_device_calibration_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *calibration_flag, int ctrl){
	int status = 0;

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Power up initialization */
	status = Laura_device_wake_up_interface(dev_t, load_cal, calibration_flag);

	switch(ctrl){
		/* Do 10cm calibration */
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			/* Set TOF configuration */
			status = Laura_device_tof_configuration_interface(dev_t, ctrl);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
			/* Do calibration */
			status = Laura_device_calibration(dev_t, cal_data_10);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
			break;
		/* Do 40cm calibration */
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			/* Set TOF configuration */
			status = Laura_device_tof_configuration_interface(dev_t, ctrl);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
			/* Do calibration */
			status = Laura_device_calibration(dev_t, cal_data_40);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
			break;
		/* Do infinity calibration */
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			/* Set TOF configuration */
			status = Laura_device_tof_configuration_interface(dev_t, ctrl);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
			/* Do calibration */
			status = Laura_device_calibration(dev_t, cal_data_inf);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
#if 0
			status = Larua_Create_Calibration_Data(cal_data_10, cal_data_40, cal_data_inf);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
#endif
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command fail (%d) !!\n", __func__, ctrl);
			/* Go MCPU to standby mode */
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
			return -1;
	}

	/* Go MCPU to standby mode */
	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}



/** @brief laura read calibration input data from calibration file interface
*
*       @param buf
*       @param v
*	@param cal_data the calibration data
*
*/
int Laura_read_calibration_data_interface(struct seq_file *buf, void *v, uint16_t *cal_data){
	int status = 0;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* read calibration data */
	status = Laura_Read_Calibration_Value_From_File(buf, cal_data);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

int Olivia_read_calibration_data_interface(struct seq_file *buf, void *v, uint16_t *cal_data){
	int status = 0;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* read calibration data */
	status = Olivia_Read_Calibration_Value_From_File(buf, cal_data);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

int Laura_get_module_id_interface(struct msm_laser_focus_ctrl_t *dev_t, struct seq_file *buf, void *v){
        int status = 0;

        LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

        /* Get module id */
        Laura_Get_Module_ID(dev_t,buf);

        LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
        return status;
}


int	Olivia_Do_TOF_and_Calibration(struct msm_laser_focus_ctrl_t *dev_t, enum laura_configuration_ctrl  cal_type, int16_t* cal_data){

	int status = 0;
	
	/* Set TOF configuration */
	status = Olivia_device_tof_configuration_interface(dev_t, cal_type);
	if(status < 0)
		return status;
	
	/* Do calibration */
	status = Laura_device_calibration(dev_t, cal_data);
	if(status < 0)
		return status;

	return status;
}



int Olivia_device_calibration_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *calibration_flag, int ctrl){
	int status = 0;
	static int calib[2] = {0,0};
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Power up initialization */
	status = Laura_device_wake_up_interface(dev_t, load_cal, calibration_flag);

	/* Do 10,40,inf cm calibration */
	switch(ctrl){		
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			status = Olivia_Do_TOF_and_Calibration(dev_t, ctrl, cal_data_10);
			if(status < 0)
				goto err;

			(void) CCI_I2C_RdWord(dev_t, 0x0A, &conf_level_10cm);
			conf_level_10cm = (conf_level_10cm&0x7ff0)>>4;
			LOG_Handler(LOG_CDBG, "10cm confidence level=(%d)\n", conf_level_10cm);

			calib[0] = 1;
			break;
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			status = Olivia_Do_TOF_and_Calibration(dev_t, ctrl, cal_data_40);
			if(status < 0)
				goto err;
			calib[1] = 1;
			break;
		//has block 70 outside, here wont be run
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			status = Olivia_Do_TOF_and_Calibration(dev_t, ctrl, cal_data_inf);
			if(status < 0)
				goto err;
#if 0
			status = Larua_Create_Calibration_Data(cal_data_10, cal_data_40, cal_data_inf);
			if(status < 0){
				/* Go MCPU to standby mode */
				Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
				return status;
			}
#endif
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command fail (%d) !!\n", __func__, ctrl);
			/* Go MCPU to standby mode */
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
			return -1;
	}

	//do tof for safety, not necessary
	if(calib[0]&&calib[1]){
		calib[0] = 0;
		calib[1] = 0;
		Olivia_DumpKdata(dev_t, cal_data_f0);
		Olivia_device_tof_configuration_interface(dev_t, MSM_LASER_FOCUS_APPLY_NORMAL_CALIBRATION);
	}


	/* Go MCPU to standby mode */
	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;

err:

	LOG_Handler(LOG_ERR, "%s: fail (%d) !!\n", __func__, status);	
	/* Go MCPU to standby mode */
	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
	return status;	
}


/** @brief laura get calibration input data interface
*
*	@param buf
*	@param v
*
*/
int Laura_get_calibration_input_data_interface(struct seq_file *buf, void *v){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Get calibration input data */
	status = Laura_get_calibration_input(buf, v, cal_data_10, cal_data_40, cal_data_inf);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}


int Olivia_get_calibration_input_data_interface(struct seq_file *buf, void *v){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Get calibration input data */
	status = Laser_get_calibration_input(buf, v, cal_data_10, cal_data_40, cal_data_f0, conf_level_10cm);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}
/** @brief laura read range interface
*
*	@param dev_t the laser focus controller
*	@param load_cal if load calibration data
*	@param calibration_flag the calibration flag
*
*/
int Laura_device_read_range_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *calibration_flag){
	int status = 0;

	/* Power up initialization */
	status = Laura_device_wake_up_interface(dev_t, load_cal, calibration_flag);
	if(status < 0){
		repairing_state = true;
		goto procedure_break;
	}
	
	status = Olivia_device_read_range(dev_t);	
	if(status < 0){
		repairing_state = true;
		goto procedure_break;
	}

procedure_break:
	
	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);

	#if 0
	if(keepMeasuring){
		ErrCode1 = RANGE_ERR_NOT_ADAPT;
		Range1 = OUT_OF_RANGE;		

	}
	#endif

	return status;
}

void control_signal(struct msm_laser_focus_ctrl_t *dev_t)
{
	uint16_t indirect_addr;
	uint16_t data,data_verify;

	indirect_addr = 0x1007;
	data = 0xD618;
	Laura_device_indirect_addr_write(dev_t, 0x18, 0x19, indirect_addr, I2C_DATA_PORT, &data, 1);

	indirect_addr = 0x1207;
	data = 0xD586;
	Laura_device_indirect_addr_write(dev_t, 0x18, 0x19, indirect_addr, I2C_DATA_PORT, &data, 1);

	indirect_addr = 0x3007;
	data = 0x3C01;
	Laura_device_indirect_addr_write(dev_t, 0x18, 0x19, indirect_addr, I2C_DATA_PORT, &data, 1);

	indirect_addr = 0x3207;
	data = 0xE500;
	Laura_device_indirect_addr_write(dev_t, 0x18, 0x19, indirect_addr, I2C_DATA_PORT, &data, 1);

	indirect_addr = 0x0007;
	data = 0x0003;
	Laura_device_indirect_addr_write(dev_t, 0x18, 0x19, indirect_addr, I2C_DATA_PORT, &data, 1);

#if 1 // for debug
	/* Check patch memory write */
	CCI_I2C_WrByte(dev_t, 0x18, 0x10);
	CCI_I2C_WrByte(dev_t, 0x19, 0x07);
	CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &data_verify);
	LOG_Handler(LOG_DBG, "%s: 0x0710: 0x%04x\n", __func__, data_verify);

	CCI_I2C_WrByte(dev_t, 0x18, 0x12);
	CCI_I2C_WrByte(dev_t, 0x19, 0x07);
	CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &data_verify);
	LOG_Handler(LOG_DBG, "%s: 0x0712: 0x%04x\n", __func__, data_verify);

	CCI_I2C_WrByte(dev_t, 0x18, 0x30);
	CCI_I2C_WrByte(dev_t, 0x19, 0x07);
	CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &data_verify);
	LOG_Handler(LOG_DBG, "%s: 0x0730: 0x%04x\n", __func__, data_verify);

	CCI_I2C_WrByte(dev_t, 0x18, 0x32);
	CCI_I2C_WrByte(dev_t, 0x19, 0x07);
	CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &data_verify);
	LOG_Handler(LOG_DBG, "%s: 0x0732: 0x%04x\n", __func__, data_verify);

	CCI_I2C_WrByte(dev_t, 0x18, 0x00);
	CCI_I2C_WrByte(dev_t, 0x19, 0x07);
	CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &data_verify);
	LOG_Handler(LOG_DBG, "%s: 0x0700: 0x%04x\n", __func__, data_verify);
#endif
}


/** @brief laura power up initialization interface (verify firmware)
*
*	@param dev_t the laser focus controller
*	@param load_cal if load calibration data
*	@param calibration_flag the calibration flag
*	@param do_measure if do measure
*
*/
int Laura_device_power_up_init_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *cal_flag){
	int status = 0;


	status = WaitMCPUStandby(dev_t);
	if(status < 0){
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		return status;
	}

	/* For current issue (15->600 microA) */
	status = CCI_I2C_WrWord(dev_t, ICSR, PAD_INT_MODE);
       if (status < 0){
	   	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
             return status;
       }
	   
	status = Config_I2C_Interface(dev_t);
	if(status < 0){
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		return status;
	}

#if 1
	/* Load default calibration value or not*/
	if(load_cal && *cal_flag){
		status = Laura_Apply_Calibration(dev_t);
		if(status < 0){
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
			return status;
		}
	}
#endif

	WaitMCPUOn(dev_t);
	if(status < 0){
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		return status;
	}

	Olivia_device_tof_configuration_interface(dev_t, MSM_LASER_FOCUS_APPLY_NORMAL_CALIBRATION);
	   
	status = WaitMCPUStandby(dev_t);
	if(status < 0){
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		return status;
	}
	
	
	return status;
}

/** @brief laura power up initialization interface (non-verify firmware)
*
*	@param dev_t the laser focus controller
*	@param load_cal if load calibration data
*	@param calibration_flag the calibration flag
*	@param do_measure if do measure
*
*/
int Laura_device_wake_up_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *cal_flag){
	#define CHECK_CALIBRATION_DATA	0
	int status = 0;
#if CHECK_CALIBRATION_DATA
	int i = 0;
	uint16_t data_verify;
#endif

	//for safety
	status = WaitMCPUStandby(dev_t);
	if(status < 0){
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		return status;
	}

#if 0
	/* Load default calibration value or not*/
	if(load_cal && *cal_flag){
		status = Laura_Apply_Calibration(dev_t);
		if(status < 0){
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
			return status;
		}
	}
#endif

#if CHECK_CALIBRATION_DATA
	status = WaitMCPUOff(dev_t);
	if(status < 0){
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		return status;
	}
	/* Check patch memory write */
	CCI_I2C_WrByte(dev_t, 0x18, 0x10);
	CCI_I2C_WrByte(dev_t, 0x19, 0xC0);
	for(i = 0; i < SIZE_OF_OLIVIA_CALIBRATION_DATA; i++){
		CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &data_verify);
		LOG_Handler(LOG_DBG, "%s: 0x1A: 0x%04x\n", __func__, data_verify);
	}
#endif
	
	status = WaitMCPUOn(dev_t);
	control_signal(dev_t);
	return status;
}


extern int Laser_Product;
/** @brief laura configuration interface
*
*	@param dev_t the laser focus controller
*	@param ctrl the tof configuration controller
*
*/
int Laura_device_tof_configuration_interface(struct msm_laser_focus_ctrl_t *dev_t, int ctrl){
	int status = 0;

	/* TOF configuratino default value */
	uint16_t config_normal_laura[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0xE002, 0xA041, 0x4580};
	uint16_t config_cal_10_laura[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0xE002, 0xAC81, 0x4580};
	uint16_t config_cal_40_laura[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0xE002, 0xAC81, 0x4580};
	uint16_t config_cal_inf_laura[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0xE000, 0xFC81, 0x4500};

		
	switch(ctrl){
		/* These are set TOF configuration */
		case MSM_LASER_FOCUS_APPLY_NORMAL_CALIBRATION:			
			status = Laura_device_UpscaleRegInit(dev_t, config_normal_laura);
			break;
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			status = Laura_device_UpscaleRegInit(dev_t, config_cal_10_laura);
			break;
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			status = Laura_device_UpscaleRegInit(dev_t, config_cal_40_laura);
			break;
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			status = Laura_device_UpscaleRegInit(dev_t, config_cal_inf_laura);
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command fail (%d) !!\n", __func__, ctrl);
			return -1;
	}
	
	return status;
}


/* TOF configuratino default value */
uint16_t config_normal_olivia[LAURA_CONFIG_SIZE] = {0xE100, 0x10FF, 0x07F0, 0x5010, 0xA081, 0x4594};
uint16_t config_cal_10_olivia[LAURA_CONFIG_SIZE] = {0xE100, 0x10FF, 0x07D0, 0x5008, 0x1C81, 0x4114};
uint16_t config_cal_40_olivia[LAURA_CONFIG_SIZE] = {0xE100, 0x10FF, 0x07D0, 0x5008, 0xFC81, 0x4114};
uint16_t config_cal_inf_olivia[LAURA_CONFIG_SIZE] = {0xE100, 0x30FF, 0x07D0, 0xE000, 0xFC81, 0x4500};
int Olivia_device_tof_configuration_interface(struct msm_laser_focus_ctrl_t *dev_t, int ctrl){
	int status = 0;

	switch(ctrl){
		/* These are set TOF configuration */
		case MSM_LASER_FOCUS_APPLY_NORMAL_CALIBRATION:			
			status = Laura_device_UpscaleRegInit(dev_t, config_normal_olivia);
			break;
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			status = Laura_device_UpscaleRegInit(dev_t, config_cal_10_olivia);
			break;
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			status = Laura_device_UpscaleRegInit(dev_t, config_cal_40_olivia);
			break;
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			status = Laura_device_UpscaleRegInit(dev_t, config_cal_inf_olivia);
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command fail (%d) !!\n", __func__, ctrl);
			return -1;
	}
	
	return status;
}

