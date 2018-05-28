/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-08
*
*/

#include "HPTG_interface.h"
#include "laser_log.h"

extern int Laser_Product;

extern int ErrCode1;
extern int ErrCode2;
extern uint16_t Range1;
extern uint16_t Range2;

extern bool timedMeasure;
extern bool ioctrl_close;

extern struct delayed_work		keepMeasure;
extern bool repairing_state;

extern int g_factory;
extern uint16_t Settings[NUMBER_OF_SETTINGS];

/* Calibration input data */
/*
int16_t cal_data_10[CAL_MSG_LEN*2]={3863,3863,3863,3863,3863,3863,3863,3863,3863,2039,2039,2039,2039,2039,2039,2039,2039,2039,2909,1098,539,12850,12850,12850,12850,12850,5,3857,3857,3857,3857,3857,3857,3857,3857,3857,2039,2039,2039,2039,2039,2039,2039,2039,2039,2910,1100,540,12850,12850,12850,12850,12850,5};
int16_t cal_data_40[CAL_MSG_LEN*2]={21,21,21,21,21,21,21,21,21,45,45,45,45,45,45,45,45,45,6500,1094,531,12850,12850,12850,12850,12850,5,21,21,21,21,21,21,21,21,21,45,45,45,45,45,45,45,45,45,6500,1088,536,12850,12850,12850,12850,12850,5};
*/
int16_t cal_data_10[CAL_MSG_LEN*2];
int16_t cal_data_40[CAL_MSG_LEN*2];
int16_t cal_data_inf[CAL_MSG_LEN*2];	/* Calibration infinity input data */
int16_t cal_data_f0[CAL_MSG_F0];
int16_t cal_data_confidence[CAL_CONFIDENCE_LEN];

/** @brief laura read calibration input data from calibration file interface
*
*       @param buf
*       @param v
*	@param cal_data the calibration data
*
*/
int Laser_read_Kdata_interface(struct seq_file *buf, void *v, uint16_t *cal_data){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* read calibration data */
	status = Read_Kdata_From_File(buf, cal_data);

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


int	Do_TOF_and_Calibration(struct msm_laser_focus_ctrl_t *dev_t, enum laura_configuration_ctrl  cal_type, int16_t* cal_data){

	int status = 0;

	/* Set TOF configuration */
	status = Tof_configuration_interface(dev_t, cal_type);
	if(status < 0)
		return status;

	/* Do calibration */
	status = Laura_device_calibration(dev_t, cal_data);
	if(status < 0)
		return status;

	return status;
}



int Laser_calibration_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *calibration_flag, int ctrl){
	int status = 0;
	static int calib[2] = {0,0};
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	status = Laser_wake_up_interface(dev_t, load_cal, calibration_flag);

	/* Do 10,40,inf cm calibration */
	switch(ctrl){
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			status = Do_TOF_and_Calibration(dev_t, ctrl, cal_data_10);
			Tof_configuration_interface(dev_t, MSM_LASER_FOCUS_APPLY_NORMAL_CALIBRATION);
			if(status==0)
			status = ComputeConfidencePoint(dev_t);
			if(status < 0)
				goto err;
			calib[0] = 1;
			break;
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			status = Do_TOF_and_Calibration(dev_t, ctrl, cal_data_40);
			if(status < 0)
				goto err;
			calib[1] = 1;
			break;

		default:
			LOG_Handler(LOG_ERR, "%s: command fail (%d) !!\n", __func__, ctrl);
			MCPU_Controller(dev_t, MCPU_STANDBY);
			return -1;
	}

	//ISSUE: Do tof due to K-ToF cost more time -> may cause timeout
	if(calib[0]&&calib[1]){
		calib[0] = 0;
		calib[1] = 0;
		Olivia_DumpKdata(dev_t, cal_data_f0, CAL_MSG_F0);
		Tof_configuration_interface(dev_t, MSM_LASER_FOCUS_APPLY_NORMAL_CALIBRATION);
	}

	MCPU_Controller(dev_t, MCPU_STANDBY);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return status;

err:
	MCPU_Controller(dev_t, MCPU_STANDBY);
	LOG_Handler(LOG_ERR, "%s: fail (%d) !!\n", __func__, status);
	return status;
}


/** @brief laura get calibration input data interface
*
*	@param buf
*	@param v
*
*/

int Laser_get_raw_Kdata_interface(struct seq_file *buf, void *v){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Get calibration input data */
	status = Laser_get_calibration_input(buf, v, cal_data_10, cal_data_40, cal_data_f0, cal_data_confidence);

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

int Laser_measurement_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *calibration_flag){
	int status = 0;

	/* Power up initialization */
	status = Laser_wake_up_interface(dev_t, load_cal, calibration_flag);
	if(status < 0){
		repairing_state = true;
		goto procedure_break;
	}

	status = Perform_measurement(dev_t);
	if(status < 0){
		repairing_state = true;
		goto procedure_break;
	}

procedure_break:

	MCPU_Controller(dev_t, MCPU_STANDBY);

	if(timedMeasure){
		ErrCode1 = RANGE_ERR_NOT_ADAPT;
		Range1 = OUT_OF_RANGE;

	}

	return status;
}

/** @brief laura power up initialization interface (verify firmware)
*
*	@param dev_t the laser focus controller
*	@param load_cal if load calibration data
*	@param calibration_flag the calibration flag
*	@param do_measure if do measure
*
*/
int Laser_power_up_init_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *cal_flag){
	int status = 0;

		status = WaitMCPUStandby(dev_t);

		/* For current issue (15->600 microA) */
	if(status ==0)
		status = CCI_I2C_WrWord(dev_t, ICSR, PAD_INT_MODE);

	if(status ==0)
		status = Config_I2C_Interface(dev_t);

	if(status ==0 && load_cal)
		status = Apply_Calibration_Data(dev_t);


	if(status ==0)
		status = WaitMCPUOn(dev_t);

	if(status ==0)
		status = Tof_configuration_interface(dev_t, MSM_LASER_FOCUS_APPLY_NORMAL_CALIBRATION);

	if(status ==0)
		status = WaitMCPUStandby(dev_t);

	if(status !=0)
		MCPU_Controller(dev_t, MCPU_STANDBY);

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
int Laser_wake_up_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *cal_flag){
	int status = 0;

		//for safety
		status = WaitMCPUStandby(dev_t);

	/* Load K data or not*/
	if(load_cal && *cal_flag && (status==0))
		status = Apply_Calibration_Data(dev_t);

	if(status ==0)
		status = WaitMCPUOn(dev_t);

	if(status ==0)
		status = ambient_setting(dev_t);

	if(status !=0)
		MCPU_Controller(dev_t, MCPU_STANDBY);

	return status;
}



/** @brief laura configuration interface
*
*	@param dev_t the laser focus controller
*	@param ctrl the tof configuration controller
*
*/
int Tof_configuration_interface(struct msm_laser_focus_ctrl_t *dev_t, int ctrl){

	int status = 0;
	/* TOF configuratino default value */
	/*
	uint16_t config_normal[LAURA_CONFIG_SIZE] = {0xE100, 0x10FF, 0x07F0, 0x5008, 0xA041, 0x4594};
	uint16_t config_K10[LAURA_CONFIG_SIZE] = {0xE100, 0x10FF, 0x07D0, 0x5008, 0x1C81, 0x4154};
	uint16_t config_K40[LAURA_CONFIG_SIZE] = {0xE100, 0x10FF, 0x07D0, 0x5008, 0xFC81, 0x4154};
	*/
/*
	if(g_factory){
	getTOF(config_normal,config_K10,config_K40,LAURA_CONFIG_SIZE);
	}
*/
	uint16_t *config_normal = &Settings[TOF_NORMAL_0];
	uint16_t *config_K10 = &Settings[TOF_K10_0];
	uint16_t *config_K40 = &Settings[TOF_K40_0];
#if 0
	// for debug
	LOG_Handler(LOG_DBG, "%s: 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x)\n", __func__,
		config_normal[0], config_normal[1], config_normal[2], config_normal[3], config_normal[4], config_normal[5]);

	LOG_Handler(LOG_DBG, "%s: 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x)\n", __func__,
		config_K10[0], config_K10[1], config_K10[2], config_K10[3], config_K10[4], config_K10[5]);

	LOG_Handler(LOG_DBG, "%s: 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x)\n", __func__,
		config_K40[0], config_K40[1], config_K40[2], config_K40[3], config_K40[4], config_K40[5]);
#endif

	switch(ctrl){
		/* TOF configuration */
		case MSM_LASER_FOCUS_APPLY_NORMAL_CALIBRATION:
			status = settingTOF(dev_t, config_normal);
			break;
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			status = settingTOF(dev_t, config_K10);
			break;
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			status = settingTOF(dev_t, config_K40);
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command fail (%d) !!\n", __func__, ctrl);
			return -1;
	}

	return status;
}

