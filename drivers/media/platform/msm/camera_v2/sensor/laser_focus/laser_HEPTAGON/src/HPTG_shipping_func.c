/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-07
*
*/

#include "HPTG_shipping_func.h"
#include "laser_log.h"
#include <linux/delay.h>
#include <linux/unistd.h>
/* Log count */
int read_range_log_count = 0; // Read range log count

/* Module id */
static bool module_id_flag = false;
uint16_t module_id[34];
uint16_t FW_version[2];
uint16_t chipID;
uint16_t f0_data[CAL_MSG_F0*2];

bool newKdata = true;

extern int ErrCode1;
extern int ErrCode2;
extern uint16_t Range1;
extern uint16_t Range2;

extern bool timedMeasure;
extern bool ioctrl_close;
extern bool repairing_state;

extern bool CSCmode;

static uint16_t debug_raw_range = 0;
static uint16_t debug_raw_confidence = 0;

extern int g_factory;
extern int Laser_log_cnt;
extern int proc_log_cnt;
extern uint16_t Settings[NUMBER_OF_SETTINGS];

static void init_debug_raw_data(void){
		debug_raw_range = 0;
		debug_raw_confidence = 0;
}

uint16_t get_debug_raw_range(void){
	return debug_raw_range;
}

uint16_t get_debug_raw_confidence(void){
	return debug_raw_confidence;
}


/** @brief Swap high and low of the data (e.g 0x1234 => 0x3412)
*
*	@param register_data the data which will be swap
*
*/
void swap_data(uint16_t* register_data){
	*register_data = ((*register_data >> 8) | ((*register_data & 0xff) << 8)) ;
}


/** @brief Mailbox: create calibration data
*		  This mailbox command is used to retrieve data to be used for the computation of calibration parameters.
*		  This is a singleentry MBX command with MBX message response with Msg_len = 6
*
*	@param dev_t the laser focus controller
*	@param cal_data the calibration ouput data
*
*/
int Check_Ready_via_ICSR(struct msm_laser_focus_ctrl_t *dev_t){
	int status =0;
	uint16_t i2c_read_data, i2c_read_data2;

	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0)
			return status;

		if((i2c_read_data&(NEW_DATA_IN_MBX)) == GO_AHEAD)
			return status;

		/* Busy pending MCPU Msg */
		status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data2);
		if (status < 0)
			return status;

msleep(1);
LOG_Handler(LOG_ERR, "%s: register(0x00, 0x10): (0x%x, 0x%x)\n", __func__,  i2c_read_data, i2c_read_data2);

	}
}

int Wait_for_Notification_from_ICSR(struct msm_laser_focus_ctrl_t *dev_t){
	int status =0;
	uint16_t i2c_read_data;

	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0 || (i2c_read_data&NEW_DATA_IN_MBX))
			return status;
	}
}

int Olivia_Mailbox_Command(struct msm_laser_focus_ctrl_t *dev_t, int16_t cal_data[]){
	int status = 0, msg_index = 0, M2H_Msg_Len = 0;
	uint16_t i2c_read_data;

	status = Check_Ready_via_ICSR(dev_t);
	//if(status !=0)

	//single entry message? -> Cmd_len=0
	status = CCI_I2C_WrWord(dev_t, H2M_MBX, GET_CALIBRATION);
       if (status < 0){
               return status;
       }

	status = Wait_for_Notification_from_ICSR(dev_t);
	//if(status !=0)


	status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data);
	if (status < 0){
		return status;
       }

	M2H_Msg_Len = (i2c_read_data & CMD_LEN_MASK)>>8;
	if(M2H_Msg_Len != CAL_MSG_LEN)
		LOG_Handler(LOG_ERR,"Message length is not in expect\n");

	if((i2c_read_data&MESSAGE_ID_MASK) == 0xCC) {
		for(msg_index=0; msg_index<CAL_MSG_LEN; msg_index++){
			status = Wait_for_Notification_from_ICSR(dev_t);
			//if(status !=0)
       		status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data);
			if (status < 0){
				LOG_Handler(LOG_CDBG, "break %d \n",msg_index);
              		return status;
       		}
			/* Append to previosly saved data */
			cal_data[msg_index] = i2c_read_data;
			//LOG_Handler(LOG_DBG, "%s: Calibration data[%d]: 0x%x\n", __func__, msg_index, cal_data[msg_index]);
			//print  these at proc debug dump
		}
		LOG_Handler(LOG_DBG, "end%d \n",msg_index);
	}
	else{
		LOG_Handler(LOG_ERR, "%s: M2H_MBX(7:0): 0x%x, Msg_Len: %d\n", __func__, i2c_read_data&0xFF, M2H_Msg_Len);
		return -1;
	}
	return status;
}

int ambient_setting(struct msm_laser_focus_ctrl_t *dev_t){

	int status=0;
	uint16_t i2c_read_data;
	int cnt=0;
	/*
	int buf[1]={0x0640}; //dec 1600

	if(g_factory){
	Sysfs_read_word_seq("/factory/Olivia_ambient.txt",buf,1);
	}
	*/
	LOG_Handler(LOG_DBG, "%s: ambient setting(%d)\n",__func__,Settings[AMBIENT]);


	status = CCI_I2C_WrWord(dev_t, H2M_MBX, 0x81C7);

	while(1){
		CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if(i2c_read_data&MCPU_HAVE_READ_MBX){
			cnt=0;
			break;
		}
		if(++cnt > 50){
			LOG_Handler(LOG_ERR, "%s: timeout 1\n",__func__);
			return -1;
		}
		msleep(1);
	}

	status = CCI_I2C_WrWord(dev_t, H2M_MBX, Settings[AMBIENT]);

	while(1){
		CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if(i2c_read_data&MCPU_HAVE_READ_MBX){
			cnt=0;
			break;
		}
		if(++cnt > 50){
			LOG_Handler(LOG_ERR, "%s: timeout 2\n",__func__);
			return -1;
		}
		msleep(1);
	}

	return status;
}

/** @brief Load calibration data
*
*	@param dev_t the laser focus controller
*
*/
int Laura_device_Load_Calibration_Value(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t indirect_addr, data[SIZE_OF_LAURA_CALIBRATION_DATA];
#if DEBUG_LOG_FLAG
	int i = 0;
	uint16_t data_verify;
#endif
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);


	/* Read Calibration data, addr is swapped */
	indirect_addr = 0x10C0;

	status = Larua_Read_Calibration_Data_From_File(data, SIZE_OF_LAURA_CALIBRATION_DATA);
	if(status < 0){
		LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
		return status;
	}

	Laura_device_indirect_addr_write(dev_t, 0x18, 0x19, indirect_addr, I2C_DATA_PORT, data, 10);

	/* Check patch memory write */
	CCI_I2C_WrByte(dev_t, 0x18, 0x10);
	CCI_I2C_WrByte(dev_t, 0x19, 0xC0);
#if DEBUG_LOG_FLAG
	for(i = 0; i < 20; i++){
		CCI_I2C_RdByte(dev_t, I2C_DATA_PORT, &data_verify);
		LOG_Handler(LOG_DBG, "%s: 0x1A: 0x%x\n", __func__, data_verify);
	}
#endif
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}


int Olivia_device_Load_Calibration_Value(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t indirect_addr;
	static uint16_t data[SIZE_OF_OLIVIA_CALIBRATION_DATA+CONFIDENCE_LENGTH];
	static bool gotKdata = false;
	static int cnt=0;
	uint16_t header_verify0,header_verify1;
	uint16_t header_verify[3];
	int i = 0;
#if DEBUG_LOG_FLAG
	uint16_t data_verify;
#endif

	/* Read Calibration data, addr is swapped */
	indirect_addr = 0x10C0;


    //notice that CSCmode changed at running time
    if(CSCmode)
        gotKdata = false;

	if(!gotKdata){
		status = Larua_Read_Calibration_Data_From_File(data, SIZE_OF_OLIVIA_CALIBRATION_DATA+CONFIDENCE_LENGTH);
		if(status < 0){
			if(++cnt>=50){
				LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
				cnt=0;
			}
			return status;
		}
		//swap confidence data back
		swap_data(&data[SIZE_OF_OLIVIA_CALIBRATION_DATA]);
		swap_data(&data[SIZE_OF_OLIVIA_CALIBRATION_DATA+1]);

		//FAE suggest filtering-range
		if(500 < data[SIZE_OF_OLIVIA_CALIBRATION_DATA] &&
			2047 > data[SIZE_OF_OLIVIA_CALIBRATION_DATA] &&
			2500 < data[SIZE_OF_OLIVIA_CALIBRATION_DATA+1] &&
			10235 > data[SIZE_OF_OLIVIA_CALIBRATION_DATA+1]){
			Settings[CONFIDENCE10] =  data[SIZE_OF_OLIVIA_CALIBRATION_DATA];
			Settings[CONFIDENCE_THD] = data[SIZE_OF_OLIVIA_CALIBRATION_DATA+1];
			newKdata = true;
		}
		else{
			data[SIZE_OF_OLIVIA_CALIBRATION_DATA] = DEFAULT_CONFIDENCE10;
			data[SIZE_OF_OLIVIA_CALIBRATION_DATA+1] = DEFAULT_CONFIDENCE_THD;
			newKdata = false;
		}

		if(g_factory)
			gotKdata = false;
		else
			gotKdata = true;
	}
	Settings[CONFIDENCE10] =  data[SIZE_OF_OLIVIA_CALIBRATION_DATA];
	Settings[CONFIDENCE_THD] = data[SIZE_OF_OLIVIA_CALIBRATION_DATA+1];

	Laura_device_indirect_addr_write(dev_t, 0x18, 0x19, indirect_addr, I2C_DATA_PORT, data, SIZE_OF_OLIVIA_CALIBRATION_DATA);

	/* Check patch memory write */
	CCI_I2C_WrByte(dev_t, 0x18, 0x10);
	CCI_I2C_WrByte(dev_t, 0x19, 0xC0);
#if DEBUG_LOG_FLAG
	for(i = 0; i < 2*SIZE_OF_OLIVIA_CALIBRATION_DATA; i++){
		CCI_I2C_RdByte(dev_t, I2C_DATA_PORT, &data_verify);
		LOG_Handler(LOG_DBG, "%s: 0x1A: 0x%x\n", __func__, data_verify);
	}
#endif

	for(i=0; i<3; i++){
		CCI_I2C_RdByte(dev_t, I2C_DATA_PORT, &header_verify0);
		CCI_I2C_RdByte(dev_t, I2C_DATA_PORT, &header_verify1);
		header_verify0 &= 0x00ff;
		header_verify1 &= 0x00ff;
		header_verify[i] = (header_verify1<<8)|header_verify0;
	}
	LOG_Handler(LOG_DBG, "%s: header 0x%04x 0x%04x 0x%04x, expect CA1B 0026 0301\n",
					__func__, header_verify[0],header_verify[1],header_verify[2]);


	return status;
}


int Read_Kdata_From_File(struct seq_file *vfile, uint16_t *cal_data){
	int status = 0, i = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	status = Larua_Read_Calibration_Data_From_File(cal_data, SIZE_OF_OLIVIA_CALIBRATION_DATA);
        if(status < 0){
                LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
		if(vfile!=NULL){
			seq_printf(vfile,"No calibration data!!\n");
		}
                return status;
        }

	for(i = 0; i < SIZE_OF_OLIVIA_CALIBRATION_DATA; i++){
                swap_data(cal_data+i);
        }

	LOG_Handler(LOG_DBG,"part Cal data: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
		cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4],
		cal_data[5], cal_data[6], cal_data[7], cal_data[8], cal_data[9]);

	if(vfile!=NULL){
		for(i=0; i<SIZE_OF_OLIVIA_CALIBRATION_DATA; i++)
			seq_printf(vfile,"%04x",cal_data[i]);
		seq_printf(vfile,"\n");

	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}


#define	MCPU_SWITCH_TIMEOUT_ms		15
#define	MEASURE_TIME_OUT_ms		120

int Verify_Range_Data_Ready(struct msm_laser_focus_ctrl_t *dev_t){

	uint16_t i2c_read_data = 0;
	int status=0;
	struct timeval start,now;
	O_get_current_time(&start);

       while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0)
       		break;

		if(i2c_read_data & NEW_DATA_IN_RESULT_REG){
			LOG_Handler(LOG_DBG, "%s: range data ready\n", __func__);
			break;
		}

		O_get_current_time(&now);
	       if(is_timeout(start,now,MEASURE_TIME_OUT_ms)){
			LOG_Handler(LOG_ERR, "%s: fail (time out)\n", __func__);
	             status = -(OUT_OF_RANGE);
			break;
	      }

		/* Delay: waitting laser sensor sample ready */
		msleep(3);
       }

	return status;
}


bool compareConfidence(uint16_t confidence, uint16_t Range, uint16_t thd, uint16_t limit){
	return  		confidence <= thd*limit/Range;
}


uint16_t thd = 16;
uint16_t limit = 1500;
uint8_t thd_near_mm = 0;

int	Read_Range_Data(struct msm_laser_focus_ctrl_t *dev_t){

	uint16_t RawRange = 0, Range = 0, error_status =0;
	uint16_t RawConfidence = 0, confidence_level =0;
	int status;
	int errcode;
	int confirm=0;

	uint16_t IT_verify;

	int confA = 1300;
	int confC = 5600;
	int ItB = 5000;

	thd = Settings[CONFIDENCE_FACTOR];
	limit = Settings[DISTANCE_THD];
	thd_near_mm = Settings[NEAR_LIMIT];

	CCI_I2C_WrByte(dev_t, 0x18, 0x06);
	CCI_I2C_WrByte(dev_t, 0x19, 0x20);
	CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &IT_verify);

	confA = Settings[CONFIDENCE10];
	confC = Settings[CONFIDENCE_THD];
	ItB =  Settings[IT];

	init_debug_raw_data();

      	status = CCI_I2C_RdWord(dev_t, RESULT_REGISTER, &RawRange);
	if (status < 0)
             	return status;

	debug_raw_range = RawRange;
	Range = (RawRange&DISTANCE_MASK)>>2;

	error_status = RawRange&ERROR_CODE_MASK;

	CCI_I2C_RdWord(dev_t, 0x0A, &RawConfidence);
	if (status < 0)
            	return status;

	debug_raw_confidence = RawConfidence;
	confidence_level = (RawConfidence&0x7ff0)>>4;


	if(RawRange&VALID_DATA){
		if(!confirm){
			if(IT_verify < ItB){
				if(confidence_level < confA){
					confirm =1;
					errcode = RANGE_ERR_NOT_ADAPT;
					Range =	OUT_OF_RANGE;
				}
			}
			else{
				if(( ItB*(confidence_level) < ItB*confA- (IT_verify - ItB)*(confC - confA) )){
					confirm =2;
					errcode = RANGE_ERR_NOT_ADAPT;
					Range =	OUT_OF_RANGE;
				}
			}
		}
		if(!confirm && error_status==NO_ERROR && Range == 0){
			confirm=3;
			errcode = RANGE_ERR_NOT_ADAPT;
			Range =OUT_OF_RANGE;
		}
		if(!confirm&& error_status==NO_ERROR && compareConfidence(confidence_level,Range,thd,limit)){
			confirm=4;
			errcode = RANGE_ERR_NOT_ADAPT;
			Range =OUT_OF_RANGE;
		}
		if(!confirm&&error_status==GENERAL_ERROR){
			confirm=5;
			errcode = RANGE_ERR_NOT_ADAPT;
			Range =OUT_OF_RANGE;
		}
		if(!confirm&&error_status==NEAR_FIELD){
			confirm=6;
			errcode = RANGE_ADAPT;
			Range =thd_near_mm;
		}
		if(!confirm&&error_status==FAR_FIELD){
			confirm=7;
			errcode = RANGE_ADAPT;
			Range =OUT_OF_RANGE;
		}
		if(!confirm&&Range==2047){
			confirm=8;
			errcode = RANGE_ADAPT;
			Range =OUT_OF_RANGE;
		}
		if(!confirm&&Range>=limit){
			confirm=9;
			errcode = RANGE_ADAPT;
			Range =OUT_OF_RANGE;
		}
		if(!confirm){
			confirm =0;
			if(Range <= 100 /*&& !g_factory*/)
				errcode = RANGE_ADAPT_WITH_LESS_ACCURACY;
			else
				errcode = RANGE_ADAPT;
		}

	}
	else{
		confirm=10;
		Range=OUT_OF_RANGE;
		errcode = RANGE_ERR_NOT_ADAPT;
	}


	ErrCode1 = errcode;
	Range1 = Range;

	if((Laser_log_cnt==LOG_SAMPLE_RATE-1)||(Laser_log_cnt==LOG_SAMPLE_RATE-2)||proc_log_cnt){
		LOG_Handler(LOG_CDBG,"%s: conf(%d)  confA(%d) confC(%d) ItB(%d) IT_verify(0x2006:%d)\n", __func__, confidence_level, confA,confC,ItB,IT_verify);
		LOG_Handler(LOG_CDBG, "%s: status(%d) thd(%d) limit(%d) near(%d) Confidence(%d)\n", __func__,
			error_status>>13, thd, limit, thd_near_mm, confidence_level);
		LOG_Handler(LOG_CDBG, "%s: Range(%d) ErrCode(%d) RawRange(%d) case(%d)\n", __func__,Range,errcode,(RawRange&DISTANCE_MASK)>>2, confirm);
	}

	//reset cnt for case that use both proc and ioctrl
	proc_log_cnt=0;

	return Range;


}

int	Read_Range_Data_OldKdata(struct msm_laser_focus_ctrl_t *dev_t){

	uint16_t RawRange = 0, Range = 0, error_status =0;
	uint16_t RawConfidence = 0, confidence_level =0;
	int status;
	int errcode;
	int confirm=0;

	uint16_t IT_verify;

	int confA = 1300;
	int confC = 5600;
	int ItB = 5000;

	thd = Settings[CONFIDENCE_FACTOR];
	limit = Settings[DISTANCE_THD];
	thd_near_mm = Settings[NEAR_LIMIT];

	CCI_I2C_WrByte(dev_t, 0x18, 0x06);
	CCI_I2C_WrByte(dev_t, 0x19, 0x20);
	CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &IT_verify);

	confA = Settings[CONFIDENCE10];
	confC = Settings[CONFIDENCE_THD];
	ItB =  Settings[IT];

	init_debug_raw_data();

      	status = CCI_I2C_RdWord(dev_t, RESULT_REGISTER, &RawRange);
	if (status < 0)
             	return status;

	debug_raw_range = RawRange;
	Range = (RawRange&DISTANCE_MASK)>>2;

	error_status = RawRange&ERROR_CODE_MASK;

	CCI_I2C_RdWord(dev_t, 0x0A, &RawConfidence);
	if (status < 0)
            	return status;

	debug_raw_confidence = RawConfidence;
	confidence_level = (RawConfidence&0x7ff0)>>4;


	if(RawRange&VALID_DATA){

		if((error_status==NO_ERROR)){
			errcode = 0;

			if(IT_verify < ItB){
				if(!confirm && confidence_level < confA){
					errcode = RANGE_ERR_NOT_ADAPT;
					Range =	OUT_OF_RANGE;
					confirm = 15;
				}
			}
			else{
				if(!confirm&&( ItB*(confidence_level) < ItB*confA- (IT_verify - ItB)*(confC - confA) )){
					errcode = RANGE_ERR_NOT_ADAPT;
					Range =	OUT_OF_RANGE;
					confirm = 16;
				}
			}

			if(!confirm && Range == 0){
				errcode = RANGE_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm = 8;
			}

			if(!confirm && Range==2047){
				errcode = RANGE_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm = 12;
			}


			if(!confirm && Range < limit && compareConfidence(confidence_level,Range,thd,limit)){
				errcode = RANGE_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm =9;
			}

			if(!confirm && Range >= limit && confidence_level >= thd){
				errcode = RANGE_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm =10;
			}

			if(!confirm && Range >= limit && confidence_level < thd){
				errcode = RANGE_ERR_NOT_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm =13;
			}

			if(!confirm && Range<100 && !g_factory){
				errcode = RANGE_ADAPT_WITH_LESS_ACCURACY;
				confirm =11;
			}


			if(!confirm){
				errcode = RANGE_ADAPT;
				confirm =11;
			}

		}
		else{

			if(error_status==NEAR_FIELD){
				errcode = RANGE_ADAPT;
				Range = 0;
				confirm = 1;
			}

			if(error_status==FAR_FIELD && confidence_level==0){
				errcode = RANGE_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm = 2;
			}

			if(error_status==FAR_FIELD && confidence_level > 0){
				errcode = RANGE_ERR_NOT_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm = 3;
			}

			if(error_status==GENERAL_ERROR){
				errcode = RANGE_ERR_NOT_ADAPT;
				Range =	OUT_OF_RANGE;
				confirm = 4;
			}

		}
	}
	else {
		if(error_status==NEAR_FIELD){
			Range = 0;
			confirm =5;
		}
		else if(error_status==NO_ERROR){
			Range = OUT_OF_RANGE;
			confirm =6;
		}
		else{
			Range = OUT_OF_RANGE;
			confirm =7;
		}
		errcode = RANGE_ERR_NOT_ADAPT;

	}



	ErrCode1 = errcode;
	Range1 = Range;

	if((Laser_log_cnt==LOG_SAMPLE_RATE-1)||(Laser_log_cnt==LOG_SAMPLE_RATE-2)||proc_log_cnt){
		LOG_Handler(LOG_CDBG,"%s: conf(%d)  confA(%d) confC(%d) ItB(%d) IT_verify(0x2006:%d)\n", __func__, confidence_level, confA,confC,ItB,IT_verify);
		LOG_Handler(LOG_CDBG, "%s: status(%d) thd(%d) limit(%d) near(%d) Confidence(%d)\n", __func__,
			error_status>>13, thd, limit, thd_near_mm, confidence_level);
		LOG_Handler(LOG_CDBG, "%s: Range(%d) ErrCode(%d) RawRange(%d) case(%d)\n", __func__,Range,errcode,(RawRange&DISTANCE_MASK)>>2, confirm);
	}

	//reset cnt for case that use both proc and ioctrl
	proc_log_cnt=0;

	return Range;


}

int Perform_measurement(struct msm_laser_focus_ctrl_t *dev_t)
{
	int status, Range=0;

	do{
		/* Trigger single measure */
	       status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (SINGLE_MEASURE|VALIDATE_CMD));
	       if (status < 0){
	            	return status;
	       }
		status = Verify_Range_Data_Ready(dev_t);
		if(status < 0) goto read_err;

		if(newKdata)
			Range = Read_Range_Data(dev_t);
		else
			Range = Read_Range_Data_OldKdata(dev_t);

		if(Range < 0){
			ErrCode1 = RANGE_ERR_NOT_ADAPT;
			Range1 = OUT_OF_RANGE;
			status = Range;
			goto read_err;
		}

		repairing_state = false;

		if(timedMeasure && ioctrl_close){
			LOG_Handler(LOG_DBG,"ioctrl close, stop measuring\n");
			return status;
		}
		else if(!timedMeasure)
			return Range;
		else
			msleep(40);

	}while(timedMeasure);


read_err:
	LOG_Handler(LOG_ERR, "%s: Exit with Error: %d\n", __func__, status);
	return status;

}



/** @brief MCPU Contorller
*
*	@param dev_t the laser focus controller
*	@param mode the MCPU go to status
*
*/
int MCPU_Controller(struct msm_laser_focus_ctrl_t *dev_t, int mode){
	int status;

	LOG_Handler(LOG_DBG, "%s: procdure (%d)\n", __func__, mode);
	switch(mode){
		case MCPU_ON:
			status = CCI_I2C_WrWord(dev_t, PMU_CONFIG, (PATCH_MEM_EN|MCPU_INIT_STATE));
			if(status >= 0)
				status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (MCPU_TO_ON|VALIDATE_CMD));
			break;

		case MCPU_OFF:
			// Enable patch memory
			status = CCI_I2C_WrWord(dev_t, PMU_CONFIG, (PATCH_MEM_EN|PATCH_CODE_LD_EN));
			if(status >= 0)
				status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (GO_MCPU_OFF|VALIDATE_CMD));
			break;

		case MCPU_STANDBY:
			status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (GO_STANDBY|VALIDATE_CMD));
			break;

		default:
			LOG_Handler(LOG_ERR, "%s MCPU mode invalid (%d)\n", __func__, mode);
			break;
	}

	udelay(MCPU_DELAY_TIME);

	return status;
}

int getTOF(uint16_t* config_normal,uint16_t* config_K10,uint16_t* config_K40,int LAURA_CONFIG_SIZE){

	int buf[3*LAURA_CONFIG_SIZE];
	int i=0;
	int status=0;

	for(i=0; i<3*LAURA_CONFIG_SIZE; i++)
		buf[i]=0;

	status = Sysfs_read_word_seq("/factory/Olivia_conf.txt",buf,3*LAURA_CONFIG_SIZE);
	if(status<0)
		return status;

	for(i=0; i< LAURA_CONFIG_SIZE; i++)
		config_normal[i]=buf[i]&0xffff;

	for(i=0; i< LAURA_CONFIG_SIZE; i++)
		config_K10[i]=buf[i+LAURA_CONFIG_SIZE]&0xffff;

	for(i=0; i< LAURA_CONFIG_SIZE; i++)
		config_K40[i]=buf[i+(2*LAURA_CONFIG_SIZE)]&0xffff;

	return 0;




}


/** @brief Initialize Laura tof configure
*
*	@param dev_t the laser focus controller
*	@param config the configuration param
*
*/
int settingTOF(struct msm_laser_focus_ctrl_t *dev_t, uint16_t *config)
{
	int status = 0;

//#if DEBUG_LOG_FLAG
	LOG_Handler(LOG_DBG, "%s: 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n", __func__,
		config[0], config[1], config[2], config[3], config[4], config[5]);
//#endif


	/* Change the default VCSEL threshold and VCSEL peak */
       	status = CCI_I2C_WrWord(dev_t, 0x0C, config[0]);

	if(status >= 0)
       	status = CCI_I2C_WrWord(dev_t, 0x0E, config[1]);

	if(status >= 0)
       	status = CCI_I2C_WrWord(dev_t, 0x20, config[2]);

	if(status >= 0)
		status = CCI_I2C_WrWord(dev_t, 0x22, config[3]);

	if(status >= 0)
		status = CCI_I2C_WrWord(dev_t, 0x24, config[4]);

	if(status >= 0)
		status = CCI_I2C_WrWord(dev_t, 0x26, config[5]);

	return status;
}


int WaitMCPUOn(struct msm_laser_focus_ctrl_t *dev_t){
	uint16_t i2c_read_data = 0;
	int status=0;
	struct timeval start,now;
	O_get_current_time(&start);

	MCPU_Controller(dev_t, MCPU_ON);
	while(1){
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0)
       		break;

		//include MCPU_ON
		if(i2c_read_data == STATUS_MEASURE_ON){
			LOG_Handler(LOG_DBG, "%s: in MCPU_ON\n", __func__);
			break;
		}

		 O_get_current_time(&now);
              if(is_timeout(start,now,MCPU_SWITCH_TIMEOUT_ms)){
			LOG_Handler(LOG_ERR, "%s:  time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
			status = -TIMEOUT_VAL;
                  	break;
              }
	}

	return status;
}

#define	RETRY_STANDBY		50

int WaitMCPUStandby(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t i2c_read_data = 0;
	int cnt=0;

	struct timeval start, now;
	O_get_current_time(&start);

	/* Wait MCPU standby */
	MCPU_Controller(dev_t, MCPU_STANDBY);
	while(1){
		cnt++;

		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0)	break;

		i2c_read_data &= STATUS_MASK;

		if(cnt >= RETRY_STANDBY){
			LOG_Handler(LOG_ERR, "%s:  retry %d times, reg(0x06): 0x%x\n", __func__, RETRY_STANDBY, i2c_read_data);
			MCPU_Controller(dev_t, MCPU_STANDBY);
		}

		if(i2c_read_data == STATUS_STANDBY){
			LOG_Handler(LOG_DBG, "%s: in STANDBY MODE, reg(0x06): 0x%x\n", __func__, i2c_read_data);
			break;
		}

		//Check time out
		O_get_current_time(&now);
             	if(is_timeout(start,now,MCPU_SWITCH_TIMEOUT_ms)){
			LOG_Handler(LOG_ERR, "%s:  time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
              	status = -TIMEOUT_VAL;
			break;
             	}

		msleep(1);
	}

	return status;
}

int WaitMCPUOff(struct msm_laser_focus_ctrl_t *dev_t){

	int status = 0;
	uint16_t i2c_read_data = 0;
	struct timeval start, now;
	O_get_current_time(&start);

	// Set then Verify status is MCPU off
	MCPU_Controller(dev_t, MCPU_OFF);
	while(1){
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0)
             		break;

		i2c_read_data &= STATUS_MASK;
		if(i2c_read_data == STATUS_MCPU_OFF){
			LOG_Handler(LOG_DBG, "%s: in OFF MODE, reg(0x06): 0x%x\n", __func__, i2c_read_data);
			break;
		}

             	if(is_timeout(start,now,MCPU_SWITCH_TIMEOUT_ms)){
			LOG_Handler(LOG_ERR, "%s: time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
              	status = -TIMEOUT_VAL;
			break;
             	}
		udelay(500);
	}

	return status;
}


/** @brief Configure i2c interface
*
*	@param dev_t the laser focus controller
*
*/
int Config_I2C_Interface(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Configure I2C interface */
	//include enable auto-increment
	status = CCI_I2C_WrByte(dev_t, 0x1C, 0x65);
       if (status < 0){
               return status;
       }

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief Power up initialization without applying calibration data
*
*	@param dev_t the laser focus controller
*
*/
int Laura_No_Apply_Calibration(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	status = MCPU_Controller(dev_t, MCPU_ON);
	return status;
}

/** @brief Power up initialization which apply calibration data
*
*	@param dev_t the laser focus controller
*
*/
int Apply_Calibration_Data(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;

	status = WaitMCPUOff(dev_t);

	/* Load calibration data */
	Olivia_device_Load_Calibration_Value(dev_t);


	/* Control Signal Setting*/
	control_signal(dev_t);

       return status;
}

//update: Libra ER
#define VER_MAJOR	1
#define VER_MINOR	0

void Verify_FW_Version(void){

	LOG_Handler(LOG_CDBG,"FW version %d.%d\n",FW_version[0],FW_version[1]);
	if(FW_version[0]!=VER_MAJOR|| FW_version[1]!=VER_MINOR)
		LOG_Handler(LOG_ERR,"FW vesrion not %d.%d\n",VER_MAJOR,VER_MINOR);

}


/** @brief Get module id from chip
*
*       @param dev_t the laser focus controller
*
*/
#define MODULE_ID_LEN	34
void Get_ModuleID(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0, i = 0;

if(!module_id_flag){
	MCPU_Controller(dev_t, MCPU_OFF);

	status = CCI_I2C_WrByte(dev_t, 0x18, 0xc0);
       status = CCI_I2C_WrByte(dev_t, 0x19, 0xff);

	CCI_I2C_RdByte(dev_t, 0x1A, &FW_version[0]);
	CCI_I2C_RdByte(dev_t, 0x1A, &FW_version[1]);

	FW_version[0] &= 0x001f;
	FW_version[1] &= 0x000f;

	Verify_FW_Version();

	status = CCI_I2C_WrByte(dev_t, 0x18, 0x04);
       status = CCI_I2C_WrByte(dev_t, 0x19, 0xC8);

	for(i = 0; i < MODULE_ID_LEN; i++){
               	CCI_I2C_RdByte(dev_t, 0x1A, &module_id[i]);
        }
	i=0;
	LOG_Handler(LOG_CDBG,"Module ID(Hex):%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x\n",
                	module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
                	module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
                	module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
                	module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
                	module_id[32],module_id[33]);

	module_id_flag=true;
}

	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
}

/** @brief Get Chip from driver
*
*       @param dev_t the laser focus controller
*       @param vfile
*
*/
void Laura_Get_Module_ID(struct msm_laser_focus_ctrl_t *dev_t, struct seq_file *vfile){



	if(!module_id_flag){
		MCPU_Controller(dev_t, MCPU_OFF);
		Get_ModuleID(dev_t);
	}
	else{
		LOG_Handler(LOG_DBG,"Module ID(Hex):%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x\n",
                	module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
                	module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
                	module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
                	module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
                	module_id[32],module_id[33]);
	}

	if(vfile!=NULL){
                seq_printf(vfile,"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",
                        module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
                        module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
                        module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
                        module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
                        module_id[32],module_id[33]);

		seq_printf(vfile,"FW vesrion: %d.%d\n",FW_version[0],FW_version[1]);
        }

}


/** @brief Verify firmware version
*
*	@param dev_t the laser focus controller
*
*/
bool Laura_FirmWare_Verify(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t fw_major_version, fw_minor_version;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);



#if 0
	/* wait hardware handling (least 500us) */
	usleep(MCPU_DELAY_TIME);
#endif

	status = CCI_I2C_WrByte(dev_t, 0x18, 0xC0);
	status = CCI_I2C_WrByte(dev_t, 0x19, 0xFF);

	status = CCI_I2C_RdByte(dev_t, I2C_DATA_PORT, &fw_major_version);
	fw_major_version = fw_major_version & 0x3F;
	status = CCI_I2C_RdByte(dev_t, I2C_DATA_PORT, &fw_minor_version);

	LOG_Handler(LOG_DBG, "%s: LSB: 0x%x ; MSB: 0x%x\n", __func__, fw_major_version, fw_minor_version);

	if( fw_major_version >= 0 && fw_minor_version >= 14 ){
		/* Can do calibraion */
		LOG_Handler(LOG_DBG, "%s: It can do calibration!!\n", __func__);
		return true;
	}
	else{
		/* Can not do calibraion */
		LOG_Handler(LOG_DBG, "%s: The fireware is too old, it can not do calibration!!\n", __func__);
		return false;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return false;
}

int Olivia_DumpKdata(struct msm_laser_focus_ctrl_t *dev_t, int16_t* cal_data, uint16_t len ){
	int status = 0, i=0;
	uint16_t i2c_read_data;
	uint16_t len_byte = len*2;
	MCPU_Controller(dev_t, MCPU_STANDBY);
msleep(1);
	status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
	if (status < 0){
             	return status;
       }
	LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);


	MCPU_Controller(dev_t, MCPU_OFF);
	msleep(2);

	status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
	if (status < 0){
             	return status;
       }
	LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);

	status = CCI_I2C_WrByte(dev_t, 0x18, 0x48);
	status = CCI_I2C_WrByte(dev_t, 0x19, 0xC8);

	for(i=0; i < len_byte; i++){
               	CCI_I2C_RdByte(dev_t, 0x1A, &f0_data[i]);
       }

	for(i=0;i < len; i++){
		cal_data[i] = (f0_data[2*i] | f0_data[2*i+1]<<8);
	}

		LOG_Handler(LOG_DBG,"dump part f0_data:%04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  %04x  \n",
                	cal_data[0],cal_data[1],cal_data[2],cal_data[3],cal_data[4],cal_data[5],cal_data[6],cal_data[7],
                	cal_data[8],cal_data[9],cal_data[10]);

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