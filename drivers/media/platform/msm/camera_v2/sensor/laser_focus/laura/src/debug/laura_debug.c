/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-07
*
*/

#include "laura_debug.h"
#include "laura_interface.h"
#include "show_log.h"
#include "laura_shipping_func.h"
#include "laura_factory_func.h"

extern int Laser_Product;
extern bool keepMeasuring;

int laura_debug_dump(struct seq_file *vfile, void *v){
	struct msm_laser_focus_ctrl_t *dev_t = get_laura_ctrl();
	uint16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if (dev_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		dev_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, dev_t->device_state);
		seq_printf(vfile,"%s: Device state: (%d) \n", __func__, dev_t->device_state);
		return -EBUSY;
	}

	/* The module id */
	seq_printf(vfile,"Module id: ");
	Laura_get_module_id_interface(dev_t,vfile,v);


	
	/* The calibration input data */
	seq_printf(vfile,"cal input data: ");
	//if(Laser_Product == PRODUCT_OLIVIA)
		Olivia_get_calibration_input_data_interface(vfile,v);
	//else
		//Laura_get_calibration_input_data_interface(vfile,v);	
	
	/* The final calibration data */
	seq_printf(vfile,"cal data: ");
	//if(Laser_Product == PRODUCT_OLIVIA)
        	Olivia_read_calibration_data_interface(vfile,v,cal_data);	
	//else
        	//Laura_read_calibration_data_interface(vfile,v,cal_data);	


	/* The laster raw range and confidence */
	seq_printf(vfile, "Raw data(range,confidence): (%d,%d)\n", get_debug_raw_range(), get_debug_raw_confidence());
	
	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

/** @brief Dump register value 
*	
*	@param vfile virtual file which provide information to the user or system administrator
*	@param v
*
*/
int dump_laura_register_read(struct seq_file *vfile, void *v){
	int rc = 0;
	int i = 0;
	uint16_t data_verify;
	struct msm_laser_focus_ctrl_t *dev_t = get_laura_ctrl();

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	mutex_ctrl(dev_t, MUTEX_LOCK);
	
	/* Dump register value */
	rc = dump_register(vfile, dev_t, DUMP_REGISTER_RANGE_MIN, DUMP_REGISTER_RANGE_MAX);

	(void) WaitMCPUStandby(dev_t);

	rc = WaitMCPUOff(dev_t);
	if(rc < 0){
		LOG_Handler(LOG_ERR, "%s: can not enter off mode", __func__);
		goto leave;
	}
	/* Check patch memory write */
	CCI_I2C_WrByte(dev_t, 0x18, 0x10);
	CCI_I2C_WrByte(dev_t, 0x19, 0xC0);
	for(i = 0; i < SIZE_OF_OLIVIA_CALIBRATION_DATA; i++){
		CCI_I2C_RdWord(dev_t, I2C_DATA_PORT, &data_verify);
		LOG_Handler(LOG_DBG, "%s: 0x1A: 0x%04x\n", __func__, data_verify);
		seq_printf(vfile, "read calibration data(%d): 0x%04x\n", i, data_verify);
	}

leave:
	if (keepMeasuring) {
		(void) WaitMCPUOn(dev_t);
	} else {
		(void) WaitMCPUStandby(dev_t);
	}
	
	mutex_ctrl(dev_t, MUTEX_UNLOCK);

	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}
