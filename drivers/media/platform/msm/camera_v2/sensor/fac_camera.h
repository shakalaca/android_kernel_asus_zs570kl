#ifndef  FAC_CAMERA_H_INCLUDED
#define FAC_CAMERA_H_INCLUDED

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "msm_sensor.h"

#define  sheldon_debug      pr_debug

#define SENSOR_ID_IMX298  0x0298
#define SENSOR_ID_T4K37   0x1c21
#define SENSOR_ID_T4K35   0x1481
#define SENSOR_ID_OV5670  0x5670
#define SENSOR_ID_OV8856  0x885a


#define OTP_DATA_LEN_WORD (16)
#define OTP_DATA_LEN_BYTE (32)

#define	PROC_FILE_STATUS_REAR	"driver/rear_otp"
#define	PROC_FILE_STATUS_FRONT	"driver/front_otp"
#define	PROC_FILE_STATUS_BANK1_REAR	"driver/rear_otp_bank1"
#define	PROC_FILE_STATUS_BANK1_FRONT	"driver/front_otp_bank1"
#define	PROC_FILE_STATUS_BANK2_REAR	"driver/rear_otp_bank2"
#define	PROC_FILE_STATUS_BANK2_FRONT	"driver/front_otp_bank2"
#define	PROC_FILE_STATUS_BANK3_REAR	"driver/rear_otp_bank3"
#define	PROC_FILE_STATUS_BANK3_FRONT	"driver/front_otp_bank3"
#define	PROC_FILE_REARMODULE	"driver/RearModule"
#define	PROC_FILE_FRONTMODULE	"driver/FrontModule"

#define REAR_PROC_FILE_PDAF_INFO	"driver/PDAF_value_more_info"
#define FRONT_PROC_FILE_PDAF_INFO	"driver/PDAF_value_more_info_1"

#define   PROC_FILE_THERMAL_REAR  "driver/camera_temp"
#define   PROC_FILE_THERMAL_FRONT "driver/camera_temp_front"


//ralf++
void set_sensor_info(int camera_id,struct msm_sensor_ctrl_t* ctrl,uint16_t sensor_id);
void create_otp_proc_file(struct msm_camera_sensor_slave_info *slave_info); 
//ralf--

//panpan++
 void create_rear_otp_proc_file(void);
 void create_front_otp_proc_file(void);
 int rear_proc_read(struct seq_file *buf, void *v);
 int front_proc_read(struct seq_file *buf, void *v);
 //panpan--
 
 void remove_file(void);

 void create_rear_bank1_proc_file(void);
 void create_front_bank1_proc_file(void);
 int rear_bank1_proc_read(struct seq_file *buf, void *v);
 int front_bank1_proc_read(struct seq_file *buf, void *v);
 void remove_bank1_file(void);


 void create_rear_bank2_proc_file(void);
 void create_front_bank2_proc_file(void);
 int rear_bank2_proc_read(struct seq_file *buf, void *v);
 int front_bank2_proc_read(struct seq_file *buf, void *v);
 void remove_bank2_file(void);

 void create_rear_bank3_proc_file(void);
 void create_front_bank3_proc_file(void);
 int rear_bank3_proc_read(struct seq_file *buf, void *v);
 int front_bank3_proc_read(struct seq_file *buf, void *v);
 void remove_bank3_file(void);

 void create_rear_module_proc_file(void);
 void create_front_module_proc_file(void);
 int rear_module_proc_read(struct seq_file *buf, void *v);
 int front_module_proc_read(struct seq_file *buf, void *v);
 void remove_module_file(void);

 ssize_t rear_otp_proc_show_asus(struct device *dev,
				struct device_attribute *attr, char *buf);
 ssize_t front_otp_proc_show_asus(struct device *dev,
				struct device_attribute *attr, char *buf);

 int read_rear_otp_asus(unsigned short sensor_id);
 int read_front_otp_asus(unsigned short sensor_id);


//sheldon++
 int fcamera_power_up( struct msm_sensor_ctrl_t *s_ctrl);
 int fcamera_power_down( struct msm_sensor_ctrl_t *s_ctrl);
void create_thermal_file(int camNum);
void create_dump_eeprom_file(int camNum);
int imx298_otp_read(int camNum);
int ov8856_otp_read(void);
int t4k37_35_otp_read(void);
//sheldon--


 int create_rear_status_proc_file(void);
 int create_front_status_proc_file(void);
 int create_rear_resolution_proc_file(void);
 int create_front_resolution_proc_file(void);

 void remove_proc_file(void);
 void remove_resolution_file(void);

 void create_proc_pdaf_info(void);
 void clear_proc_pdaf_info(void);

#endif
