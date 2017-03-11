/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-07
*
*/

#include "laura_shipping_func.h"
#include "show_log.h"
#include <linux/delay.h>
#include <linux/unistd.h>

/* Module id */
static bool module_id_flag = false;
uint16_t module_id[34];
uint16_t module_verion[2];
uint16_t chipID;
//uint8_t module_id_2[34];

extern int ErrCode1;
extern uint16_t Range1;

extern bool keepMeasuring;
extern bool ioctrl_close;
extern bool repairing_state;
extern bool CSC_mode_flag;

static uint16_t debug_raw_range = 0;
static uint16_t debug_raw_confidence = 0;

static bool gotKdata = false;

int MIN_CONF=16;
int MAX_DISTANCE=1500;
int CONF_A=1300;
int ITB=5000;
int CONF_C=5600;
int Ambient_value=1600;

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
int Mailbox_Command(struct msm_laser_focus_ctrl_t *dev_t, int16_t cal_data[]){
	int status = 0, msg_len = 0, M2H_Msg_Len = 0;
	uint16_t i2c_read_data, i2c_read_data2;
	struct timeval start;//, now;

	start = get_current_time();
	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0){
			return status;
       	}
		
		if((i2c_read_data&NEW_DATA_IN_MBX) == 0x00){
			break;
		}

		/* Busy pending MCPU Msg */
		status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data2);
		if (status < 0){
			return status;
       	}
		LOG_Handler(LOG_ERR, "%s: register(0x00, 0x10): (0x%x, 0x%x)\n", __func__,  i2c_read_data, i2c_read_data2);
#if 0
		/* Check if time out */
		now = get_current_time();
              if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Verify ICSR(2:1) time out - register(0x10): 0x%x\n", __func__, i2c_read_data);
                    	return -TIMEOUT_VAL;
              }
#endif
		//usleep_range(DEFAULT_DELAY_TIME, DEFAULT_DELAY_TIME);
	}

	status = CCI_I2C_WrWord(dev_t, H2M_MBX, 0x0004);
       if (status < 0){
               return status;
       }

	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0){
			return status;
       	}
		
		if((i2c_read_data&0x20) == 0x20){
			break;
		}

#if 0
		/* Check if time out */
		now = get_current_time();
              if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Verify ICSR(2:1) time out - register(0x10): 0x%x\n", __func__, i2c_read_data);
                    	return -TIMEOUT_VAL;
              }
#endif
		//usleep_range(DEFAULT_DELAY_TIME, DEFAULT_DELAY_TIME);
	}


	status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data);
	LOG_Handler(LOG_DBG, "%s: Verify M2H_MBX(1)  register(0x12): 0x%x\n", __func__, i2c_read_data);
	if (status < 0){
		return status;
       }
	M2H_Msg_Len = (i2c_read_data & CMD_LEN_MASK)>>8;
	LOG_Handler(LOG_DBG, "%s: Verify M2H_MBX(1) M2H_Msg_Len: %d\n", __func__, M2H_Msg_Len);

	if(((i2c_read_data&0xFF) == 0xCC) && (M2H_Msg_Len == CAL_MSG_LEN)){
		for(msg_len=0; msg_len<M2H_Msg_Len; msg_len++){
			start = get_current_time();
			while(1){
				status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
				if (status < 0){
					return status;
       			}
				//LOG_Handler(LOG_ERR, "%s: Verify ICSR(1)  register(0x00): 0x%x\n", __func__, i2c_read_data);
			
				if((i2c_read_data&NEW_DATA_IN_MBX)){
					break;
				}
#if 0
				/* Check if time out */
				now = get_current_time();
              		if(is_timeout(start,now,TIMEOUT_VAL)){
					LOG_Handler(LOG_ERR, "%s: Verify ICSR(1) time out - register(0x00): 0x%x\n", __func__, i2c_read_data);
                    			return -TIMEOUT_VAL;
              		}
#endif
		 		//usleep_range(DEFAULT_DELAY_TIME, DEFAULT_DELAY_TIME);
			}

       		status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data);
			if (status < 0){
              		return status;
       		}

			/* Append to previosly saved data */
			cal_data[msg_len] = i2c_read_data;
			LOG_Handler(LOG_DBG, "%s: Calibration data[%d]: 0x%x\n", __func__, msg_len, cal_data[msg_len]);
		}
	}
	else{
		LOG_Handler(LOG_ERR, "%s: M2H_MBX(7:0): 0x%x, Msg_Len: %d\n", __func__, i2c_read_data&0xFF, M2H_Msg_Len);
		return -1;
	}
	return status;
}


int Check_Ready_via_ICSR(struct msm_laser_focus_ctrl_t *dev_t){
	int status =0;
	uint16_t i2c_read_data, i2c_read_data2;
	struct timeval start,now;
	O_get_current_time(&start);
	
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

		usleep_range(100, 100);
		LOG_Handler(LOG_ERR, "%s: register(0x00, 0x10): (0x%x, 0x%x)\n", __func__,  i2c_read_data, i2c_read_data2);

		/* Check if time out */
		now = get_current_time();
		if(is_timeout(start,now,100)){
			LOG_Handler(LOG_ERR, "%s: timeout\n", __func__);
			return -1;
		}
	}
}

int Wait_for_Notification_from_ICSR(struct msm_laser_focus_ctrl_t *dev_t){
	int status =0;
	uint16_t i2c_read_data;
	struct timeval start,now;
	O_get_current_time(&start);

	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0 || (i2c_read_data&NEW_DATA_IN_MBX))
			return status;

		/* Check if time out */
		now = get_current_time();
		if(is_timeout(start,now,100)){
			LOG_Handler(LOG_ERR, "%s: timeout\n", __func__);
			return -1;
		}
	}
}

int Olivia_Mailbox_Command(struct msm_laser_focus_ctrl_t *dev_t, int16_t cal_data[]){
	int status = 0, msg_index = 0, M2H_Msg_Len = 0;
	uint16_t i2c_read_data;

	status = Check_Ready_via_ICSR(dev_t);
	if(status < 0) {
		LOG_Handler(LOG_ERR, "%s: %d timeout\n", __func__, __LINE__);
		return -1;
	}

	//single entry message? -> Cmd_len=0
	status = CCI_I2C_WrWord(dev_t, H2M_MBX, GET_CALIBRATION);
       if (status < 0){
		return status;
       }

	gotKdata = false; // apply next calibration data

	//special command
	status = CCI_I2C_WrWord(dev_t, I2C_INIT_CONFIG, I2C_SPECIAL_COMMAND_START);
	special_command_write_byte(dev_t, 0x87);
	special_command_write_word(dev_t, 0x0000);
	special_command_write_byte(dev_t, 0x85);
	special_command_write_word(dev_t, 0x0120);
	special_command_write_byte(dev_t, 0x86);
	special_command_write_word(dev_t, 0x5A80);
	special_command_write_byte(dev_t, 0xC4);
	special_command_write_byte(dev_t, 0x03);

	status = Wait_for_Notification_from_ICSR(dev_t);
	if (status < 0){
		LOG_Handler(LOG_ERR, "%s: %d timeout\n", __func__, __LINE__);
		return -1;
	}

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
			if (status < 0){
				LOG_Handler(LOG_ERR, "%s: %d timeout\n", __func__, __LINE__);
				return -1;
			}
			status = CCI_I2C_RdWord(dev_t, M2H_MBX, &i2c_read_data);
			if (status < 0){
				LOG_Handler(LOG_CDBG, "break %d \n",msg_index);
              		return status;
       		}
			/* Append to previosly saved data */
			cal_data[msg_index] = i2c_read_data;
			LOG_Handler(LOG_DBG, "%s: Calibration data[%d]: 0x%x\n", __func__, msg_index, cal_data[msg_index]);
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

#if DEBUG_LOG_FLAG
	/* Check patch memory write */
	CCI_I2C_WrByte(dev_t, 0x18, 0x10);
	CCI_I2C_WrByte(dev_t, 0x19, 0xC0);
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
	static uint16_t data[SIZE_OF_OLIVIA_CALIBRATION_DATA];
	static int cnt=0;
#if DEBUG_LOG_FLAG
	int i = 0;
	uint16_t data_verify;
#endif

	/* Read Calibration data, addr is swapped */
	indirect_addr = 0x10C0;

	if(!gotKdata || CSC_mode_flag){
		status = Larua_Read_Calibration_Data_From_File(data, SIZE_OF_OLIVIA_CALIBRATION_DATA);
		if(status < 0){
			if(++cnt>=50){
				LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
				cnt=0;
			}	
			return status;
		}
		gotKdata = true;
	}
	Laura_device_indirect_addr_write(dev_t, 0x18, 0x19, indirect_addr, I2C_DATA_PORT, data, SIZE_OF_OLIVIA_CALIBRATION_DATA);

#if DEBUG_LOG_FLAG
	/* Check patch memory write */
	CCI_I2C_WrByte(dev_t, 0x18, 0x10);
	CCI_I2C_WrByte(dev_t, 0x19, 0xC0);
	for(i = 0; i < SIZE_OF_OLIVIA_CALIBRATION_DATA; i++){
		CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &data_verify);
		LOG_Handler(LOG_DBG, "%s: 0x1A: 0x%04x\n", __func__, data_verify);
	}
#endif

	return status;
}

int Laura_Read_Calibration_Value_From_File(struct seq_file *vfile, uint16_t *cal_data){
	int status = 0, i = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	status = Larua_Read_Calibration_Data_From_File(cal_data, SIZE_OF_LAURA_CALIBRATION_DATA);
        if(status < 0){
                LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
		if(vfile!=NULL){
			seq_printf(vfile,"No calibration data!!\n");
		}
                return status;
        }

	for(i = 0; i < SIZE_OF_LAURA_CALIBRATION_DATA; i++){
                swap_data(cal_data+i);
        }

	LOG_Handler(LOG_CDBG,"Cal data: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
		cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4],
		cal_data[5], cal_data[6], cal_data[7], cal_data[8], cal_data[9]);	

	if(vfile!=NULL){
		seq_printf(vfile,"%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
               		cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4],
                	cal_data[5], cal_data[6], cal_data[7], cal_data[8], cal_data[9]);

	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

int Olivia_Read_Calibration_Value_From_File(struct seq_file *vfile, uint16_t *cal_data){
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
//                swap_data(cal_data+i);
		LOG_Handler(LOG_CDBG,"part Cal data(%d): 0x%04x(%d) \n", i, cal_data[i], cal_data[i]);
        }

	if(vfile!=NULL){
		for(i=0; i<SIZE_OF_OLIVIA_CALIBRATION_DATA; i++)
			seq_printf(vfile,"%04x",cal_data[i]);
		seq_printf(vfile,"\n");
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

#define	MCPU_ON_TIME_OUT_ms		15
#define	MEASURE_TIME_OUT_ms		100

int WaitMCPUOn(struct msm_laser_focus_ctrl_t *dev_t){
	uint16_t i2c_read_data = 0;
	int status=0;	
	struct timeval start,now;
	O_get_current_time(&start);

	Laura_MCPU_Controller(dev_t, MCPU_ON);
	while(1){
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0)
			return status;
       		
		//include MCPU_ON
		if(i2c_read_data == STATUS_MEASURE_ON){
			LOG_Handler(LOG_DBG, "%s: in MCPU_ON\n", __func__);		
			break;
		}

		O_get_current_time(&now);
		if(is_timeout(start,now,MCPU_ON_TIME_OUT_ms)) {
			LOG_Handler(LOG_ERR, "%s: fail (time out):%d\n", __func__, __LINE__);
			return OUT_OF_RANGE;
		}
	}

	CCI_I2C_WrWord(dev_t, H2M_MBX, 0x81C7);

	O_get_current_time(&start);
	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0)
			return status;

		if((i2c_read_data&MCPU_HAVE_READ_MBX)){
			LOG_Handler(LOG_DBG, "%s: patch mailbox 0x81C7\n", __func__);
			break;
		}
		O_get_current_time(&now);
		if(is_timeout(start,now,MCPU_ON_TIME_OUT_ms)) {
			LOG_Handler(LOG_ERR, "%s: fail (time out):%d\n", __func__, __LINE__);
			return OUT_OF_RANGE;
		}
		msleep(1);
	}

	CCI_I2C_WrWord(dev_t, H2M_MBX, Ambient_value);

	O_get_current_time(&start);
	while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0)
			return status;

		if((i2c_read_data&MCPU_HAVE_READ_MBX)){
			LOG_Handler(LOG_DBG, "%s: ambient value: %d\n", __func__, Ambient_value);
			break;
		}
		O_get_current_time(&now);
		if(is_timeout(start,now,MCPU_ON_TIME_OUT_ms)) {
			LOG_Handler(LOG_ERR, "%s: fail (time out):%d\n", __func__, __LINE__);
			return OUT_OF_RANGE;
		}
		msleep(1);
	}

	return status;
}

int Verify_Range_Data_Ready(struct msm_laser_focus_ctrl_t *dev_t){
	
	uint16_t i2c_read_data = 0;
	int status=0;	
	struct timeval start,now;
	uint16_t test;
	O_get_current_time(&start);
	
       while(1){
		status = CCI_I2C_RdWord(dev_t, ICSR, &i2c_read_data);
		if (status < 0)
       		return status;
	
		if(i2c_read_data & NEW_DATA_IN_RESULT_REG){
			LOG_Handler(LOG_DBG, "%s: range data ready\n", __func__);	
			break;
		}

		O_get_current_time(&now);
		if(is_timeout(start,now,MEASURE_TIME_OUT_ms)){
			LOG_Handler(LOG_ERR, "%s: fail (time out)\n", __func__);
			(void) CCI_I2C_RdWord(dev_t, 0x06, &test);
			LOG_Handler(LOG_ERR, "%s: CK_test[0x06]:0x%x\n", __func__, test);
			return -(OUT_OF_RANGE);
		}
             	
		/* Delay: waitting laser sensor sample ready */
		usleep_range(READ_DELAY_TIME, READ_DELAY_TIME);
       }
	return status;
	

}

int	Read_Range_Data(struct msm_laser_focus_ctrl_t *dev_t){
	uint16_t RawRange = 0, Range = 0, error_status =0;
	uint16_t RawConfidence = 0, confidence_level =0;
	int status;
	int errcode;
	int confidence_level_formula, confidence_level_formula3;
	uint16_t INT_Time = 0;
	int err_case = 0;

	init_debug_raw_data();

	//(void) CCI_I2C_WrByte(dev_t, 0x18, 0x06);
	//(void) CCI_I2C_WrByte(dev_t, 0x19, 0x20);
	(void) CCI_I2C_WrWord(dev_t, 0x18, 0x2006);
	(void) CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &INT_Time);
	LOG_Handler(LOG_DBG, "INT time:%d\n",INT_Time);

      	status = CCI_I2C_RdWord(dev_t, RESULT_REGISTER, &RawRange);
	if (status < 0)
             	return status;
	debug_raw_range = RawRange;

	Range = (RawRange&DISTANCE_MASK)>>2;
	LOG_Handler(LOG_DBG, "%s:%d\n",__func__,Range);
	error_status = RawRange&ERROR_CODE_MASK;

	if(RawRange&VALID_DATA){

		CCI_I2C_RdWord(dev_t, 0x0A, &RawConfidence);
		if (status < 0)
			return status;

		debug_raw_confidence = RawConfidence;
		confidence_level = (RawConfidence&0x7ff0)>>4;
		LOG_Handler(LOG_DBG,"%s: confidence level is: %d (Raw data:%d)\n", __func__, confidence_level, RawConfidence);
		Range = (RawRange&DISTANCE_MASK)>>2;
		errcode = 0;
		
		confidence_level_formula = (long)((long)MIN_CONF*(long)MAX_DISTANCE) / Range;
		confidence_level_formula3 = ((long)ITB*(long)CONF_A - ((long)(INT_Time - ITB) * (long)(CONF_C - CONF_A))) / ITB;
		LOG_Handler(LOG_DBG, "confidence_level_formula:%d, confidence_level_formula3:%d\n",confidence_level_formula,confidence_level_formula3);

		if ( INT_Time < ITB && confidence_level < CONF_A) {
			Range = OUT_OF_RANGE;
			errcode = RANGE_ERR_NOT_ADAPT;
			err_case = 1;
		} else if ( INT_Time >= ITB  && confidence_level < confidence_level_formula3) {
			Range = OUT_OF_RANGE;
			errcode = RANGE_ERR_NOT_ADAPT;
			err_case = 2;
		} else if ( Range == 0 && error_status == NO_ERROR) {
			Range = OUT_OF_RANGE;
			errcode = RANGE_ERR_NOT_ADAPT;
			err_case = 3;
		} else if (confidence_level <= confidence_level_formula && error_status == NO_ERROR) {
			Range = OUT_OF_RANGE;
			errcode = RANGE_ERR_NOT_ADAPT;
			err_case = 4;
		} else if (error_status == GENERAL_ERROR) {
			Range = OUT_OF_RANGE;
			errcode = RANGE_ERR_NOT_ADAPT;
			err_case = 5;
		} else if (error_status == NEAR_FIELD) {
			Range =	0; // TBD
			err_case = 6;
		} else if(error_status == FAR_FIELD) {
			Range =	OUT_OF_RANGE;
			err_case = 7;
		} else if(Range == 2047) {
			Range = OUT_OF_RANGE;
			err_case = 8;
		} else if ( Range >= MAX_DISTANCE) {
			Range = OUT_OF_RANGE;
			err_case = 9;
		} else if ( Range < 100) {
			errcode = 64;
		}

		/* if no error we trust it! */
		
	} else {
		Range = OUT_OF_RANGE;
		errcode = RANGE_ERR_NOT_ADAPT;
		err_case = 10;
	}

	ErrCode1 = errcode;
	Range1 = Range;
	LOG_Handler(LOG_DBG, "%s: range:(%d), error code:(%d) error case:(%d)\n", __func__, Range, errcode, err_case);
	
	return Range;
}

int Olivia_device_read_range(struct msm_laser_focus_ctrl_t *dev_t)
{
	int status, Range=0;

	//do{
		/* Trigger single measure */
	       status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (SINGLE_MEASURE|VALIDATE_CMD));
	       if (status < 0){
	            	return status;
	       }
		status = Verify_Range_Data_Ready(dev_t);
		if(status != 0) goto read_err;			

		Range = Read_Range_Data(dev_t);
		if(Range < 0){
			ErrCode1 = RANGE_ERR_NOT_ADAPT;
			Range1 = OUT_OF_RANGE;
			status = Range;
			goto read_err;
		}

		#if 0
		repairing_state = false;

		if(keepMeasuring && ioctrl_close){
			LOG_Handler(LOG_DBG,"ioctrl close, stop measuring\n");
			return status;
		}	
		else if(!keepMeasuring)
			return Range;
		else
			msleep(40);
		#else
		return Range;
		#endif
		
	//}while(keepMeasuring);


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
int Laura_MCPU_Controller(struct msm_laser_focus_ctrl_t *dev_t, int mode){
	int status;
#if DEBUG_LOG_FLAG
	uint16_t i2c_read_data = 0;
#endif


	switch(mode){
		case MCPU_ON:
			/* Enable MCPU to run coming out of standby */
			//LOG_Handler(LOG_DBG, "%s: MCPU ON procdure\n", __func__);
			status = CCI_I2C_WrWord(dev_t, PMU_CONFIG, (PATHC_CODE_RETAIN|PATCH_MEM_EN|MCPU_INIT_STATE));
       		if (status < 0){
             			return status;
      			}
			
			/* Wake up MCPU to ON mode */
			status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (MCPU_TO_ON|VALIDATE_CMD));
			if (status < 0){
       			return status;
       		}
			break;
			
		case MCPU_OFF:
			/* Enable patch memory */
			//LOG_Handler(LOG_DBG, "%s: MCPU OFF procdure\n", __func__);
			status = CCI_I2C_WrWord(dev_t, PMU_CONFIG, (PATHC_CODE_RETAIN|PATCH_MEM_EN|PATCH_CODE_LD_EN));
       		if (status < 0){
             			return status;
      			}
			
			/* Go MCPU to OFF status */
			status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (GO_MCPU_OFF|VALIDATE_CMD));
        		if (status < 0){
              		return status;
        		}
			break;
			
		case MCPU_STANDBY:
			/* Change MCPU to standby mode */
			//LOG_Handler(LOG_DBG, "%s: MCPU STANDBY procdure\n", __func__);
			status = CCI_I2C_WrWord(dev_t, COMMAND_REGISTER, (GO_STANDBY|VALIDATE_CMD));
       		if (status < 0){
       			return status;
     			}
			break;
			
		default:
			//LOG_Handler(LOG_ERR, "%s MCPU mode invalid (%d)\n", __func__, mode);
			break;
	}

	/* wait hardware booting(least 500us) */
	usleep_range(MCPU_DELAY_TIME, MCPU_DELAY_TIME);
	
#if DEBUG_LOG_FLAG
	/* Verify MCPU status */
	status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
	if (status < 0){
             	return status;
       }
	//LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);
#endif

	return status;
}

/** @brief Initialize Laura tof configure
*
*	@param dev_t the laser focus controller
*	@param config the configuration param
*
*/
int Laura_device_UpscaleRegInit(struct msm_laser_focus_ctrl_t *dev_t, uint16_t *config)
{
	int status = 0;

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

#if DEBUG_LOG_FLAG
	LOG_Handler(LOG_DBG, "%s: config:(0x%x,0x%x,0x%x,0x%x,0x%x,0x%x)\n", __func__,
		config[0], config[1], config[2], config[3], config[4], config[5]);
#endif

	/* Drive INT_PAD high */
	status = CCI_I2C_WrWord(dev_t, ICSR, PAD_INT_MODE);
       if (status < 0){
               return status;
       }

	/* Change the default VCSEL threshold and VCSEL peak */
       status = CCI_I2C_WrWord(dev_t, 0x0C, config[0]);
       if (status < 0){
               return status;
       }
	   
       status = CCI_I2C_WrWord(dev_t, 0x0E, config[1]);
       if (status < 0){
               return status;
       }
	   
       status = CCI_I2C_WrWord(dev_t, 0x20, config[2]);
       if (status < 0){
               return status;
       }
	   
       status = CCI_I2C_WrWord(dev_t, 0x22, config[3]);
       if (status < 0){
               return status;
       }
	   
      status = CCI_I2C_WrWord(dev_t, 0x24, config[4]);
       if (status < 0){
               return status;
       }
	   
       status = CCI_I2C_WrWord(dev_t, 0x26, config[5]);
       if (status < 0){
               return status;
       }

	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return status;
} 

/** @brief Wait device go to standby mode
*
*	@param dev_t the laser focus controller
*
*/
int WaitMCPUStandby(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t i2c_read_data = 0;
	//uint16_t i2c_read_data_2 = 0;
	int cnt=0;

	struct timeval start, now;
	O_get_current_time(&start);


	/* Wait MCPU standby */
	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
	while(1){
		cnt++;
	
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0){
              	break;
		}
		i2c_read_data = (i2c_read_data & STATUS_MASK);
		
		if(cnt >=50){
			LOG_Handler(LOG_ERR, "%s: %s in STANDBY MODE, reg(0x06): 0x%x\n", __func__
				, (i2c_read_data==STATUS_STANDBY)?"":"Not", i2c_read_data);
			cnt =0;
			//CCI_I2C_RdWord(dev_t, RESULT_REGISTER, &i2c_read_data_2);
			Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		}
		
		if(i2c_read_data == STATUS_STANDBY){
			LOG_Handler(LOG_DBG, "%s: in STANDBY MODE, reg(0x06): 0x%x\n", __func__, i2c_read_data);		
			break;
		}


		/* Check if time out */
		now = get_current_time();
             	if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Wait chip standby time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
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

	/* Set then Verify status is MCPU off */
	Laura_MCPU_Controller(dev_t, MCPU_OFF);
	while(1){		
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0){
             		return status;
		}
		i2c_read_data = (i2c_read_data & STATUS_MASK);

		if(i2c_read_data == STATUS_MCPU_OFF){
			break;
		}
		udelay(100);

		O_get_current_time(&now);
		if(is_timeout(start,now,MCPU_ON_TIME_OUT_ms)) {
			LOG_Handler(LOG_ERR, "%s: fail (time out)\n", __func__);
			return -1;
		}
	}
	
	return 0;
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
	status = Laura_MCPU_Controller(dev_t, MCPU_ON);
	return status;
}

/** @brief Power up initialization which apply calibration data
*
*	@param dev_t the laser focus controller
*
*/
int Laura_Apply_Calibration(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;

	status = WaitMCPUOff(dev_t);

	/* Load calibration data */
	Olivia_device_Load_Calibration_Value(dev_t);

	return status;
}

/** @brief Go to standby mode when do not do measure
*
*	@param dev_t the laser focus controller
*
*/
int Laura_non_measures_go_standby(struct msm_laser_focus_ctrl_t *dev_t)
{
	int status = 0;
	uint16_t i2c_read_data = 0;
	struct timeval start, now;
	O_get_current_time(&start);

	while(1){
		status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
		if (status < 0){
             		return status;
       	}

		//include MCPU_ON
		if(i2c_read_data==STATUS_MEASURE_ON){
			break;
		}

		/* Check if time out */	
		now = get_current_time();
             	if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Wait MCPU on time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
              	return -TIMEOUT_VAL;
       	}
		LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);
		
msleep(1);
	}

	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);	
	return status;
}


/** @brief Get module id from chip
*
*       @param dev_t the laser focus controller
*
*/
void Laura_Get_Module_ID_From_Chip(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0, i = 0;
	Laura_MCPU_Controller(dev_t, MCPU_OFF);

	status = CCI_I2C_WrByte(dev_t, 0x18, 0xc0);
       status = CCI_I2C_WrByte(dev_t, 0x19, 0xff);

	CCI_I2C_RdByte(dev_t, 0x1A, &module_verion[0]);
	CCI_I2C_RdByte(dev_t, 0x1A, &module_verion[1]);

	module_verion[0] &= 0x001f;
	module_verion[1] &= 0x000f;

	LOG_Handler(LOG_CDBG,"FW version %d.%d\n",module_verion[0],module_verion[1]);
	if(module_verion[0]!=1 || module_verion[1]!=0)
		LOG_Handler(LOG_ERR,"FW vesrion not 1.0\n");
	
	status = CCI_I2C_WrByte(dev_t, 0x18, 0x04);
       status = CCI_I2C_WrByte(dev_t, 0x19, 0xC8);

	for(i = 0; i < 34; i++){
               	CCI_I2C_RdByte(dev_t, 0x1A, &module_id[i]);
        }

	LOG_Handler(LOG_CDBG,"Module ID(Hex):%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x\n",
		module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
		module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
		module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
		module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
		module_id[32],module_id[33]);

	module_id_flag=true;


	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
}

/** @brief Get Chip from driver
*
*       @param dev_t the laser focus controller
*       @param vfile
*
*/
void Laura_Get_Module_ID(struct msm_laser_focus_ctrl_t *dev_t, struct seq_file *vfile){

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if(!module_id_flag){
		Laura_MCPU_Controller(dev_t, MCPU_OFF);
		Laura_Get_Module_ID_From_Chip(dev_t);	
	}
	else{
		LOG_Handler(LOG_DBG,"Module ID:%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",
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
        }

	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
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
	usleep_range(MCPU_DELAY_TIME, MCPU_DELAY_TIME);
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

int Olivia_DumpKdata(struct msm_laser_focus_ctrl_t *dev_t, int16_t cal_data[OLIVIA_F0_DATA_LEN]){
	int status = 0, i=0;
	uint16_t i2c_read_data;
	uint16_t f0_data[OLIVIA_F0_DATA_LEN*2];
	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
	usleep_range(10000, 10000);
	status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
	if (status < 0){
             	return status;
       }	
	LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);

	
	Laura_MCPU_Controller(dev_t, MCPU_OFF);	
	msleep(2);
	
	usleep_range(10000, 10000);
	status = CCI_I2C_RdWord(dev_t, DEVICE_STATUS, &i2c_read_data);
	if (status < 0){
             	return status;
       }	
	LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);

	
	status = CCI_I2C_WrByte(dev_t, 0x18, 0x48);
	status = CCI_I2C_WrByte(dev_t, 0x19, 0xC8);

	LOG_Handler(LOG_DBG,"dump f0_data(byte) start:\n");
	for(i = 0; i < OLIVIA_F0_DATA_LEN * 2; i++){
               	CCI_I2C_RdByte(dev_t, 0x1A, &f0_data[i]);
		LOG_Handler(LOG_DBG," f0_data btye(%d), data(0x%02x)\n",i,f0_data[i]);
       }

	LOG_Handler(LOG_DBG,"dump f0_data(word) start:\n");
	for(i=0;i < OLIVIA_F0_DATA_LEN; i++){
		cal_data[i] = (f0_data[2*i] | f0_data[2*i+1]<<8);
		LOG_Handler(LOG_DBG," f0_data word(%d), data(0x%04x)\n",i,cal_data[i]);
	}
	
	return status;
}
