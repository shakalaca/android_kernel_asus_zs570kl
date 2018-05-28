/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-08
*
*/

#ifndef __LINUX_SHOW_LAURA_SENSOR_INTERFACE_H
#define __LINUX_SHOW_LAURA_SENSOR_INTERFACE_H

#include "msm_laser_focus.h"
#include "HPTG_shipping_func.h"
#include "HPTG_factory_func.h"
#include "HPTG_i2c.h"

/* Laura tof configure size */
#define LAURA_CONFIG_SIZE 6

/* Laura configuration control */
enum laura_configuration_ctrl {
        LAURA_READ_CONFIG,
        LAURA_CALIBRATION_10_CONFIG, /* Calibration 10 cm */
        LAURA_CALIBRATION_40_CONFIG, /* Calibration 40 cm */
        LAURA_CALIBRATION_INF_CONFIG, /* Calibration infinity */
};

/* calibration interface */
int Laser_calibration_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *calibration_flag, int ctrl);
/* Laura get calibration input data interface */

int Laser_get_raw_Kdata_interface(struct seq_file *buf, void *v);

/* Laura read range interface */
int Laser_measurement_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *calibration_flag);
/* Laura power up initialization interface (verify firmware) */
int Laser_power_up_init_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *cal_flag);
/* Laura power up initialization interface (non-verify firmware) */
int Laser_wake_up_interface(struct msm_laser_focus_ctrl_t *dev_t, bool load_cal, bool *cal_flag);
/* Laura configuration interface */

int Tof_configuration_interface(struct msm_laser_focus_ctrl_t *dev_t, int ctrl);

//int Do_TOF_and_Calibration(struct msm_laser_focus_ctrl_t *dev_t, enum laura_configuration_ctrl  cal_type, int16_t* cal_data);

/* Laura read calibration input data from calibration file interface */
int Laser_read_Kdata_interface(struct seq_file *buf, void *v, uint16_t *cal_data);
/* Laura get module id interface */
int Laura_get_module_id_interface(struct msm_laser_focus_ctrl_t *dev_t, struct seq_file *buf, void *v);

#endif
