/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012-2015 Synaptics Incorporated. All rights reserved.
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/input/synaptics_dsx.h>
#include "synaptics_dsx_core.h"
#include <linux/i2c.h>
#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
//<ASUS_led+>
#ifdef ASUS_FACTORY_BUILD
#include "../drivers/leds/leds-qpnp.h"
#endif
//<ASUS_led->
#include <linux/asus_project.h>
//<ASUS_focal+>
#include <linux/fs.h>
#include "test_lib.h"
#include "Test_FT6X36.h"
//<ASUS_focal->

#define INPUT_PHYS_NAME "synaptics_dsx/touch_input"
#define STYLUS_PHYS_NAME "synaptics_dsx/stylus"
#define DEBUGFS_DIR_NAME "ts_debug"

#define VIRTUAL_KEY_MAP_FILE_NAME "virtualkeys." PLATFORM_DRIVER_NAME

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif

#define WAKEUP_GESTURE true

#define ASUS_TOUCH_PROXIMITY_NODE	//<ASUS_Proximity+>

//<ASUS_Proximity+>
#ifdef ASUS_TOUCH_PROXIMITY_NODE	
#define PROXIMITY_NAME "asus_touch_proximity_status"
#endif
//<ASUS_Proximity->

#define NO_0D_WHILE_2D
#define REPORT_2D_Z
#define REPORT_2D_W
/*
#define REPORT_2D_PRESSURE
*/

#define F12_DATA_15_WORKAROUND

#define IGNORE_FN_INIT_FAILURE
/*
#define FB_READY_RESET
#define FB_READY_WAIT_MS 100
#define FB_READY_TIMEOUT_S 30
*/
#define RPT_TYPE (1 << 0)
#define RPT_X_LSB (1 << 1)
#define RPT_X_MSB (1 << 2)
#define RPT_Y_LSB (1 << 3)
#define RPT_Y_MSB (1 << 4)
#define RPT_Z (1 << 5)
#define RPT_WX (1 << 6)
#define RPT_WY (1 << 7)
#define RPT_DEFAULT (RPT_TYPE | RPT_X_LSB | RPT_X_MSB | RPT_Y_LSB | RPT_Y_MSB)

#define REBUILD_WORK_DELAY_MS 500 /* ms */

#define EXP_FN_WORK_DELAY_MS 500 /* ms */
#define MAX_F11_TOUCH_WIDTH 15
#define MAX_F12_TOUCH_WIDTH 255
#define MAX_F12_TOUCH_PRESSURE 255

#define CHECK_STATUS_TIMEOUT_MS 100

#define F01_STD_QUERY_LEN 21
#define F01_BUID_ID_OFFSET 18

#define STATUS_NO_ERROR 0x00
#define STATUS_RESET_OCCURRED 0x01
#define STATUS_INVALID_CONFIG 0x02
#define STATUS_DEVICE_FAILURE 0x03
#define STATUS_CONFIG_CRC_FAILURE 0x04
#define STATUS_FIRMWARE_CRC_FAILURE 0x05
#define STATUS_CRC_IN_PROGRESS 0x06

#define NORMAL_OPERATION (0 << 0)
#define SENSOR_SLEEP (1 << 0)
#define NO_SLEEP_OFF (0 << 2)
#define NO_SLEEP_ON (1 << 2)
#define CONFIGURED (1 << 7)

#define F11_CONTINUOUS_MODE 0x00
#define F11_WAKEUP_GESTURE_MODE 0x04
#define F12_CONTINUOUS_MODE 0x00
#define F12_WAKEUP_GESTURE_MODE 0x02
#define F12_UDG_DETECT 0x0f
#define F34_FLASH_CTRL00 0x0A
#define F51_CUSTOM_CTRL 0x407

#ifdef WAKEUP_GESTURE
#define  KEY_GESTURE_E		KEY_E//KEY_F21//KEY_E
#define  KEY_GESTURE_C		KEY_C//KEY_F20//KEY_C
//#define  KEY_GESTURE_M	KEY_M 
//#define  KEY_GESTURE_L	KEY_L
#define  KEY_GESTURE_W		KEY_W//KEY_F23//KEY_W
#define  KEY_GESTURE_S		KEY_S//KEY_F22//KEY_S 
#define  KEY_GESTURE_V		KEY_V//KEY_F18//KEY_V
#define  KEY_GESTURE_Z		KEY_Z//KEY_F19//KEY_Z
#define  KEY_GESTURE_DCLICK	KEY_POWER
#define  KEY_GESTURE_SWIPE_UP	KEY_WAKEUP

#define GESTURE_SWITCH		0x40//1000000
#define GESTURE_VEE_REG		0x0A
#define GESTURE_OTHERS		0x0B
#define GESTURE_DOUBLECLICK	0x03
#define GESTURE_SWIPE	0x07
#define GESTURE_E		0x65
#define GESTURE_C		0x63
//#define GESTURE_M		0x6D
//#define GESTURE_L		0x44
#define GESTURE_W		0x77
#define GESTURE_S		0x73
#define GESTURE_V		0x02
#define GESTURE_Z		0x7A
#endif

extern int fw_update_state;

//<ASUS_cap_sensor+>
int btn_back;
int btn_menu;
int btn_home;
//<ASUS_cap_sensor->

//<ASUS_DTP+>
int dclick_mode = 0;
//<ASUS_DTP->
//<ASUS_SWIPE+>
int swipe_mode = 0;
//<ASUS_SWIPE->
//<ASUS_Gesture+>
int gesture_mode = 0;
//<ASUS_Gesture->
//<ASUS_Glove+>
int glove_mode = 0;
//<ASUS_Glove->
//<ASUS_COVER+>
int cover_mode = 0;
//<ASUS_COVER->

//<ASUS_focal+>
static int fts_btn_back;
static int fts_btn_menu;
static int fts_btn_home;
static unsigned char cap_addr = FTS_CAP_I2C_ADDR;
unsigned int cap_sel_status = 1;    //cap_sel_status(description cap chip) -> 0:micro chip 1:focal chip
static unsigned int buf_count_add=0;
static unsigned int buf_count_neg=0;
u8 buf_touch_data[30*POINT_READ_BUF] = { 0 };
extern int fts_6336GU_ctpm_fw_upgrade(struct synaptics_rmi4_data *rmi4_data, u8 *pbt_buf, u32 dw_lenth);
int fts_a5_flag = 0;
static struct proc_dir_entry *fts_proc_entry;
static unsigned char proc_operate_mode = PROC_UPGRADE;
//<ASUS_focal->

#ifdef ASUS_TOUCH_PROXIMITY_NODE
struct proc_dir_entry *tp_proximity_proc = NULL; //<ASUS_Proximity+>
#endif

//ASUS_FACTORY+
#if defined(ASUS_FACTORY_BUILD)
struct proc_dir_entry *tp_entry = NULL;
static int touch_proc_show(struct seq_file *m, void *v) {

     if(!tp_entry)

     return seq_printf(m, "-1\n");

     else

     return seq_printf(m, "1\n");

}

static int  touch_proc_open(struct inode *inode, struct  file *file) {

  return single_open(file, touch_proc_show, NULL);

}

static const struct file_operations touch_status_fops = {
	.owner = THIS_MODULE,
    .open =  touch_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int create_asusproc_tp_status_entry( void )
{   
   tp_entry = proc_create("asus_tp_status", S_IWUGO| S_IRUGO, NULL, &touch_status_fops);
    if (!tp_entry)
        return -ENOMEM;
	return 0;
}
#endif
//ASUS_FACTORY-

static int synaptics_rmi4_check_status(struct synaptics_rmi4_data *rmi4_data,
		bool *was_in_bl_mode);
static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data,
		bool rebuild);

#ifdef CONFIG_FB
static int synaptics_rmi4_fb_notifier_cb(struct notifier_block *self,
		unsigned long event, void *data);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#ifndef CONFIG_FB
#define USE_EARLYSUSPEND
#endif
#endif

#ifdef USE_EARLYSUSPEND
static void synaptics_rmi4_early_suspend(struct early_suspend *h);

static void synaptics_rmi4_late_resume(struct early_suspend *h);
#endif

static int synaptics_rmi4_suspend(struct device *dev);

static int synaptics_rmi4_resume(struct device *dev);

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_wake_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_wake_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_virtual_key_map_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf);

static int synaptics_rmi4_sw_reset(struct synaptics_rmi4_data *rmi4_data);

//<ASUS_cap_sensor+>
static ssize_t synaptics_rmi4_cap_sensor_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_cap_sensor_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_cap1_status_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_cap2_status_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_cap3_status_show(struct device *dev,
		struct device_attribute *attr, char *buf);		
//<ASUS_cap_sensor->

//<ASUS_DTP+>
static ssize_t synaptics_rmi4_dclick_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_dclick_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
//<ASUS_DTP->
//<ASUS_SWIPE+>
static ssize_t synaptics_rmi4_swipe_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_swipe_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
//<ASUS_SWIPE->
//<ASUS_Gesture+>
static ssize_t synaptics_rmi4_gesture_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_gesture_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
//<ASUS_Gesture->
//<ASUS_Glove+>
static ssize_t synaptics_rmi4_glove_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_glove_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t synaptics_rmi4_set_glove_param(struct synaptics_rmi4_data *rmi4_data);
//<ASUS_Glove->
//<ASUS_COVER+>
static ssize_t synaptics_rmi4_cover_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_cover_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t synaptics_rmi4_set_cover_param(struct synaptics_rmi4_data *rmi4_data);
//<ASUS_COVER->
//<ASUS_focal+>
static ssize_t fts_fwupgradeapp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t fts_ftsselftest_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_ftsselftest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t cap_chip_select_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t fts_fw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t fts_fts_sw_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
//<ASUS_focal->

//<ASUS_Proximity+>
#ifdef ASUS_TOUCH_PROXIMITY_NODE
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
static ssize_t tp_proximity_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static ssize_t tp_proximity_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
#else
static int tp_proximity_proc_read(char *buf, char **start, off_t offset, int request, int *eof, void *data);
static int tp_proximity_proc_write(struct file *file, const char *buffer, unsigned long count, void *data);
#endif

static unsigned int touch_proximity_at_phone = 0;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
static const struct file_operations tp_proximity_proc_fops = {
	.owner = THIS_MODULE,
	.read = tp_proximity_proc_read,
	.write = tp_proximity_proc_write,
};
#endif
#endif
//<ASUS_Proximity->

struct synaptics_rmi4_data *gb_rmi4_data;
		
struct synaptics_rmi4_f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f11_query_0_5 {
	union {
		struct {
			/* query 0 */
			unsigned char f11_query0_b0__2:3;
			unsigned char has_query_9:1;
			unsigned char has_query_11:1;
			unsigned char has_query_12:1;
			unsigned char has_query_27:1;
			unsigned char has_query_28:1;

			/* query 1 */
			unsigned char num_of_fingers:3;
			unsigned char has_rel:1;
			unsigned char has_abs:1;
			unsigned char has_gestures:1;
			unsigned char has_sensitibity_adjust:1;
			unsigned char f11_query1_b7:1;

			/* query 2 */
			unsigned char num_of_x_electrodes;

			/* query 3 */
			unsigned char num_of_y_electrodes;

			/* query 4 */
			unsigned char max_electrodes:7;
			unsigned char f11_query4_b7:1;

			/* query 5 */
			unsigned char abs_data_size:2;
			unsigned char has_anchored_finger:1;
			unsigned char has_adj_hyst:1;
			unsigned char has_dribble:1;
			unsigned char has_bending_correction:1;
			unsigned char has_large_object_suppression:1;
			unsigned char has_jitter_filter:1;
		} __packed;
		unsigned char data[6];
	};
};

struct synaptics_rmi4_f11_query_7_8 {
	union {
		struct {
			/* query 7 */
			unsigned char has_single_tap:1;
			unsigned char has_tap_and_hold:1;
			unsigned char has_double_tap:1;
			unsigned char has_early_tap:1;
			unsigned char has_flick:1;
			unsigned char has_press:1;
			unsigned char has_pinch:1;
			unsigned char has_chiral_scroll:1;

			/* query 8 */
			unsigned char has_palm_detect:1;
			unsigned char has_rotate:1;
			unsigned char has_touch_shapes:1;
			unsigned char has_scroll_zones:1;
			unsigned char individual_scroll_zones:1;
			unsigned char has_multi_finger_scroll:1;
			unsigned char has_multi_finger_scroll_edge_motion:1;
			unsigned char has_multi_finger_scroll_inertia:1;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f11_query_9 {
	union {
		struct {
			unsigned char has_pen:1;
			unsigned char has_proximity:1;
			unsigned char has_large_object_sensitivity:1;
			unsigned char has_suppress_on_large_object_detect:1;
			unsigned char has_two_pen_thresholds:1;
			unsigned char has_contact_geometry:1;
			unsigned char has_pen_hover_discrimination:1;
			unsigned char has_pen_hover_and_edge_filters:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f11_query_12 {
	union {
		struct {
			unsigned char has_small_object_detection:1;
			unsigned char has_small_object_detection_tuning:1;
			unsigned char has_8bit_w:1;
			unsigned char has_2d_adjustable_mapping:1;
			unsigned char has_general_information_2:1;
			unsigned char has_physical_properties:1;
			unsigned char has_finger_limit:1;
			unsigned char has_linear_cofficient_2:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f11_query_27 {
	union {
		struct {
			unsigned char f11_query27_b0:1;
			unsigned char has_pen_position_correction:1;
			unsigned char has_pen_jitter_filter_coefficient:1;
			unsigned char has_group_decomposition:1;
			unsigned char has_wakeup_gesture:1;
			unsigned char has_small_finger_correction:1;
			unsigned char has_data_37:1;
			unsigned char f11_query27_b7:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f11_ctrl_6_9 {
	union {
		struct {
			unsigned char sensor_max_x_pos_7_0;
			unsigned char sensor_max_x_pos_11_8:4;
			unsigned char f11_ctrl7_b4__7:4;
			unsigned char sensor_max_y_pos_7_0;
			unsigned char sensor_max_y_pos_11_8:4;
			unsigned char f11_ctrl9_b4__7:4;
		} __packed;
		unsigned char data[4];
	};
};

struct synaptics_rmi4_f11_data_1_5 {
	union {
		struct {
			unsigned char x_position_11_4;
			unsigned char y_position_11_4;
			unsigned char x_position_3_0:4;
			unsigned char y_position_3_0:4;
			unsigned char wx:4;
			unsigned char wy:4;
			unsigned char z;
		} __packed;
		unsigned char data[5];
	};
};

struct synaptics_rmi4_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query6;
			struct {
				unsigned char ctrl0_is_present:1;
				unsigned char ctrl1_is_present:1;
				unsigned char ctrl2_is_present:1;
				unsigned char ctrl3_is_present:1;
				unsigned char ctrl4_is_present:1;
				unsigned char ctrl5_is_present:1;
				unsigned char ctrl6_is_present:1;
				unsigned char ctrl7_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl8_is_present:1;
				unsigned char ctrl9_is_present:1;
				unsigned char ctrl10_is_present:1;
				unsigned char ctrl11_is_present:1;
				unsigned char ctrl12_is_present:1;
				unsigned char ctrl13_is_present:1;
				unsigned char ctrl14_is_present:1;
				unsigned char ctrl15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl16_is_present:1;
				unsigned char ctrl17_is_present:1;
				unsigned char ctrl18_is_present:1;
				unsigned char ctrl19_is_present:1;
				unsigned char ctrl20_is_present:1;
				unsigned char ctrl21_is_present:1;
				unsigned char ctrl22_is_present:1;
				unsigned char ctrl23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl24_is_present:1;
				unsigned char ctrl25_is_present:1;
				unsigned char ctrl26_is_present:1;
				unsigned char ctrl27_is_present:1;
				unsigned char ctrl28_is_present:1;
				unsigned char ctrl29_is_present:1;
				unsigned char ctrl30_is_present:1;
				unsigned char ctrl31_is_present:1;
			} __packed;
		};
		unsigned char data[5];
	};
};

struct synaptics_rmi4_f12_query_8 {
	union {
		struct {
			unsigned char size_of_query9;
			struct {
				unsigned char data0_is_present:1;
				unsigned char data1_is_present:1;
				unsigned char data2_is_present:1;
				unsigned char data3_is_present:1;
				unsigned char data4_is_present:1;
				unsigned char data5_is_present:1;
				unsigned char data6_is_present:1;
				unsigned char data7_is_present:1;
			} __packed;
			struct {
				unsigned char data8_is_present:1;
				unsigned char data9_is_present:1;
				unsigned char data10_is_present:1;
				unsigned char data11_is_present:1;
				unsigned char data12_is_present:1;
				unsigned char data13_is_present:1;
				unsigned char data14_is_present:1;
				unsigned char data15_is_present:1;
			} __packed;
			struct {
				unsigned char data16_is_present:1;
				unsigned char data17_is_present:1;
				unsigned char data18_is_present:1;
				unsigned char data19_is_present:1;
				unsigned char data20_is_present:1;
				unsigned char data21_is_present:1;
				unsigned char data22_is_present:1;
				unsigned char data23_is_present:1;
			} __packed;
		};
		unsigned char data[4];
	};
};

struct synaptics_rmi4_f12_ctrl_8 {
	union {
		struct {
			unsigned char max_x_coord_lsb;
			unsigned char max_x_coord_msb;
			unsigned char max_y_coord_lsb;
			unsigned char max_y_coord_msb;
			unsigned char rx_pitch_lsb;
			unsigned char rx_pitch_msb;
			unsigned char tx_pitch_lsb;
			unsigned char tx_pitch_msb;
			unsigned char low_rx_clip;
			unsigned char high_rx_clip;
			unsigned char low_tx_clip;
			unsigned char high_tx_clip;
			unsigned char num_of_rx;
			unsigned char num_of_tx;
		};
		unsigned char data[14];
	};
};

//<ASUS_Glove+>
struct synaptics_rmi4_f12_ctrl_10 {
	union {
		struct {
			unsigned char noise_floor;
			unsigned char min_peak_amp;
			unsigned char peak_merge_threshold;
			unsigned char drumming_acc_threshold;
			unsigned char min_drumming_separation;
			unsigned char noise_peak_filter;
			unsigned char reserved;
		};
		unsigned char data[7];
	};	
};
//<ASUS_Glove->

//<ASUS_COVER+>
struct synaptics_rmi4_f12_ctrl_15 {
	union {
		struct {
			unsigned char finger_amp_threshold;
			unsigned char small_finger_amp_threshold;
			unsigned char small_finger_border_size;
			unsigned char negative_finger_amp_threshold;
			unsigned char palm_amp_threshold;
			unsigned char palm_area;
			unsigned char negative_finger_low_ground_mass_threshold;
		};
		unsigned char data[7];
	};	
};
//<ASUS_COVER->

//<ASUS_DTP+>
struct synaptics_rmi4_f12_ctrl_18 {
	union {
		struct {
			unsigned char double_tap_x0_LSB;
			unsigned char double_tap_x0_MSB;
			unsigned char double_tap_y0_LSB;
			unsigned char double_tap_y0_MSB;
			unsigned char double_tap_x1_LSB;
			unsigned char double_tap_x1_MSB;
			unsigned char double_tap_y1_LSB;
			unsigned char double_tap_y1_MSB;
			unsigned char max_tap_time;
			unsigned char max_tap_distance;
			unsigned char swipe_x0_LSB;
			unsigned char swipe_x0_MSB;
			unsigned char swipe_y0_LSB;
			unsigned char swipe_y0_MSB;
			unsigned char swipe_x1_LSB;
			unsigned char swipe_x1_MSB;
			unsigned char swipe_y1_LSB;
			unsigned char swipe_y1_MSB;
			unsigned char swipe_min_disance;
			unsigned char swipe_min_speed;
			unsigned char circle_max_endpoint_gap;
			unsigned char circle_min_speed;
			unsigned char triangle_max_endpoint_gap;
			unsigned char triangle_min_speed;
			unsigned char triangle_min_angle;
			unsigned char up_vee:1;
			unsigned char down_vee:1;
			unsigned char left_vee:1;
			unsigned char right_vee:1;
			unsigned char f12_ctr18_05_b4__7:4;
			unsigned char vee_min_endpoint_gap;
			unsigned char vee_min_speed;
			unsigned char vee_angle_tolerance;
			unsigned char unicode_max_endpoint_gap;
			unsigned char unicode_min_speed;
			unsigned char f12_ctr18_06_orientation:3;
			unsigned char f12_ctr18_06_b3__7:5;
		};
		unsigned char data[32];
	};	
};
//<ASUS_DTP->

struct synaptics_rmi4_f12_ctrl_23 {
	union {
		struct {
			unsigned char finger_enable:1;
			unsigned char active_stylus_enable:1;
			unsigned char palm_enable:1;
			unsigned char unclassified_object_enable:1;
			unsigned char hovering_finger_enable:1;
			unsigned char gloved_finger_enable:1;
			unsigned char f12_ctr23_00_b6__7:2;
			unsigned char max_reported_objects;
			unsigned char f12_ctr23_02_b0:1;
			unsigned char report_active_stylus_as_finger:1;
			unsigned char report_palm_as_finger:1;
			unsigned char report_unclassified_object_as_finger:1;
			unsigned char report_hovering_finger_as_finger:1;
			unsigned char report_gloved_finger_as_finger:1;
			unsigned char report_narrow_object_swipe_as_finger:1;
			unsigned char report_handedge_as_finger:1;
			unsigned char cover_enable:1;
			unsigned char stylus_enable:1;
			unsigned char eraser_enable:1;
			unsigned char small_object_enable:1;
			unsigned char f12_ctr23_03_b4__7:4;
			unsigned char report_cover_as_finger:1;
			unsigned char report_stylus_as_finger:1;
			unsigned char report_eraser_as_finger:1;
			unsigned char report_small_object_as_finger:1;
			unsigned char f12_ctr23_04_b4__7:4;
		};
		unsigned char data[5];
	};
};

struct synaptics_rmi4_f12_ctrl_31 {
	union {
		struct {
			unsigned char max_x_coord_lsb;
			unsigned char max_x_coord_msb;
			unsigned char max_y_coord_lsb;
			unsigned char max_y_coord_msb;
			unsigned char rx_pitch_lsb;
			unsigned char rx_pitch_msb;
			unsigned char rx_clip_low;
			unsigned char rx_clip_high;
			unsigned char wedge_clip_low;
			unsigned char wedge_clip_high;
			unsigned char num_of_p;
			unsigned char num_of_q;
		};
		unsigned char data[12];
	};
};

struct synaptics_rmi4_f12_finger_data {
	unsigned char object_type_and_status;
	unsigned char x_lsb;
	unsigned char x_msb;
	unsigned char y_lsb;
	unsigned char y_msb;
#ifdef REPORT_2D_Z
	unsigned char z;
#endif
#ifdef REPORT_2D_W
	unsigned char wx;
	unsigned char wy;
#endif
};

struct synaptics_rmi4_f1a_query {
	union {
		struct {
			unsigned char max_button_count:3;
			unsigned char reserved:5;
			unsigned char has_general_control:1;
			unsigned char has_interrupt_enable:1;
			unsigned char has_multibutton_select:1;
			unsigned char has_tx_rx_map:1;
			unsigned char has_perbutton_threshold:1;
			unsigned char has_release_threshold:1;
			unsigned char has_strongestbtn_hysteresis:1;
			unsigned char has_filter_strength:1;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f1a_control_0 {
	union {
		struct {
			unsigned char multibutton_report:2;
			unsigned char filter_mode:2;
			unsigned char reserved:4;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f1a_control {
	struct synaptics_rmi4_f1a_control_0 general_control;
	unsigned char button_int_enable;
	unsigned char multi_button;
	unsigned char *txrx_map;
	unsigned char *button_threshold;
	unsigned char button_release_threshold;
	unsigned char strongest_button_hysteresis;
	unsigned char filter_strength;
};

struct synaptics_rmi4_f1a_handle {
	int button_bitmask_size;
	unsigned char max_count;
	unsigned char valid_button_count;
	unsigned char *button_data_buffer;
	unsigned char *button_map;
	struct synaptics_rmi4_f1a_query button_query;
	struct synaptics_rmi4_f1a_control button_control;
};

struct synaptics_rmi4_exp_fhandler {
	struct synaptics_rmi4_exp_fn *exp_fn;
	bool insert;
	bool remove;
	struct list_head link;
};

struct synaptics_rmi4_exp_fn_data {
	bool initialized;
	bool queue_work;
	struct mutex mutex;
	struct list_head list;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct synaptics_rmi4_data *rmi4_data;
};

static struct synaptics_rmi4_exp_fn_data exp_data;

static struct synaptics_dsx_button_map *vir_button_map;

static struct device_attribute attrs[] = {
	__ATTR(reset, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			synaptics_rmi4_f01_reset_store),
	__ATTR(productinfo, S_IRUGO,
			synaptics_rmi4_f01_productinfo_show,
			synaptics_rmi4_store_error),
	__ATTR(buildid, S_IRUGO,
			synaptics_rmi4_f01_buildid_show,
			synaptics_rmi4_store_error),
	__ATTR(flashprog, S_IRUGO,
			synaptics_rmi4_f01_flashprog_show,
			synaptics_rmi4_store_error),
	__ATTR(0dbutton, (S_IRUGO | S_IWUSR | S_IWGRP),
			synaptics_rmi4_0dbutton_show,
			synaptics_rmi4_0dbutton_store),
	__ATTR(suspend, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			synaptics_rmi4_suspend_store),
	__ATTR(wake_gesture, (S_IRUGO | S_IWUSR | S_IWGRP),
			synaptics_rmi4_wake_gesture_show,
			synaptics_rmi4_wake_gesture_store),
	//<ASUS_cap_sensor+>	
	__ATTR(cap_sensor, (S_IRUGO | S_IWUSR | S_IWGRP),
			synaptics_rmi4_cap_sensor_show,
			synaptics_rmi4_cap_sensor_store),	
	__ATTR(cap1_status, (S_IRUGO | S_IWUSR | S_IWGRP),
			synaptics_rmi4_cap1_status_show,
			NULL),
	__ATTR(cap2_status, (S_IRUGO | S_IWUSR | S_IWGRP),
			synaptics_rmi4_cap2_status_show,
			NULL),
	__ATTR(cap3_status, (S_IRUGO | S_IWUSR | S_IWGRP),
			synaptics_rmi4_cap3_status_show,
			NULL),	
	//<ASUS_cap_sensor->
	//<ASUS_DTP+>
	__ATTR(dclickmode, (S_IRUGO | S_IWUSR | S_IWGRP),
			synaptics_rmi4_dclick_mode_show,
			synaptics_rmi4_dclick_mode_store),	
	//<ASUS_DTP->
	//<ASUS_SWIPE+>
	__ATTR(swipemode, (S_IRUGO | S_IWUSR | S_IWGRP),
			synaptics_rmi4_swipe_mode_show,
			synaptics_rmi4_swipe_mode_store),	
	//<ASUS_SWIPE->
	//<ASUS_Gesture+>
	__ATTR(gesturemode, (S_IRUGO | S_IWUSR | S_IWGRP),
			synaptics_rmi4_gesture_mode_show,
			synaptics_rmi4_gesture_mode_store),
	//<ASUS_Gesture->
	//<ASUS_Glove+>
	__ATTR(glovemode, (S_IRUGO | S_IWUSR | S_IWGRP),
			synaptics_rmi4_glove_mode_show,
			synaptics_rmi4_glove_mode_store),
	//<ASUS_Glove->
	//<ASUS_COVER+>
	__ATTR(covermode, (S_IRUGO | S_IWUSR | S_IWGRP),
			synaptics_rmi4_cover_mode_show,
			synaptics_rmi4_cover_mode_store),
	//<ASUS_COVER->
    //<ASUS_focal+>
    __ATTR(fts_selftest, (S_IRUGO | S_IWUSR | S_IWGRP),
			fts_ftsselftest_show,
			fts_ftsselftest_store),
    __ATTR(fwupgradeapp, (S_IRUGO | S_IWUSR | S_IWGRP),
        NULL,
        fts_fwupgradeapp_store),
    __ATTR(cap_chip_status, (S_IRUGO | S_IWUSR | S_IWGRP),
			cap_chip_select_show,
			NULL),
    __ATTR(fts_fw_info, (S_IRUGO | S_IWUSR | S_IWGRP),
			fts_fw_version_show,
			NULL),
    __ATTR(fts_sw_reset, (S_IRUGO | S_IWUSR | S_IWGRP),
			NULL,
			fts_fts_sw_reset_store),
    //<ASUS_focal->
};

//<ASUS_COVER+>
static ssize_t synaptics_rmi4_set_cover_param(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;	
	struct synaptics_rmi4_f12_ctrl_10 *ctrl_10 = NULL;
	struct synaptics_rmi4_f12_ctrl_15 *ctrl_15 = NULL;
	struct synaptics_rmi4_f12_ctrl_18 *ctrl_18 = NULL;

    unsigned char buf;
    /*unsigned char buf_val[2] = {0};

	if(cap_sel_status == 1) {
		buf_val[0] = 0xa5;
        buf_val[1] = 0x00;
        retval = cap_i2c_write(rmi4_data, buf_val, 2);
        if(retval < 0){
            printk("[cap] write reg_addr=0x%02x, retval =%d\n", buf_val[0], retval);
        }
        msleep(5);
  }*/


	ctrl_10 = kmalloc(sizeof(*ctrl_10), GFP_KERNEL);
	if (!ctrl_10) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for ctrl_10\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}
	
	ctrl_15 = kmalloc(sizeof(*ctrl_15), GFP_KERNEL);
	if (!ctrl_15) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for ctrl_15\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}
	
	//<ASUS_DTP+>
	ctrl_18 = kmalloc(sizeof(*ctrl_18), GFP_KERNEL);
	if (!ctrl_18) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for ctrl_18\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}
	//<ASUS_DTP->
	
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl10_base_addr,
			ctrl_10->data,
			sizeof(ctrl_10->data));
	if (retval < 0)
		return -EINVAL;
	dev_dbg(rmi4_data->pdev->dev.parent,
				"%s enable cover,old min peak = 0x%x, ctrl_10->data[1]=0x%x\n", __func__, ctrl_10->min_peak_amp, ctrl_10->data[1]);
				
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl15_base_addr,
			ctrl_15->data,
			sizeof(ctrl_15->data));
	if (retval < 0)
		return -EINVAL;	
	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s old ctrl_15->data=0x%02x, finger_thrshold=0x%02x\n", __func__, ctrl_15->data[0], ctrl_15->finger_amp_threshold);
			
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl18_base_addr,
			ctrl_18->data,
			sizeof(ctrl_18->data));
	if (retval < 0)
		return -EINVAL;
			
	//ctrl18+		
	/*
	printk("%s ctrl18 double_tap_x0_LSB=0x%02x\n", __func__, ctrl_18->double_tap_x0_LSB);
	printk("%s ctrl18 double_tap_x0_MSB=0x%02x\n", __func__, ctrl_18->double_tap_x0_MSB);
	printk("%s ctrl18 double_tap_y0_LSB=0x%02x\n", __func__, ctrl_18->double_tap_y0_LSB);
	printk("%s ctrl18 double_tap_y0_MSB=0x%02x\n", __func__, ctrl_18->double_tap_y0_MSB);
	printk("%s ctrl18 double_tap_x1_LSB=0x%02x\n", __func__, ctrl_18->double_tap_x1_LSB);
	printk("%s ctrl18 double_tap_x1_MSB=0x%02x\n", __func__, ctrl_18->double_tap_x1_MSB);
	printk("%s ctrl18 double_tap_y1_LSB=0x%02x\n", __func__, ctrl_18->double_tap_y1_LSB);
	printk("%s ctrl18 double_tap_y1_MSB=0x%02x\n", __func__, ctrl_18->double_tap_y1_MSB);
	
	printk("%s ctrl18 swipe_x0_LSB=0x%02x\n", __func__, ctrl_18->swipe_x0_LSB);
	printk("%s ctrl18 swipe_x0_MSB=0x%02x\n", __func__, ctrl_18->swipe_x0_MSB);
	printk("%s ctrl18 swipe_y0_LSB=0x%02x\n", __func__, ctrl_18->swipe_y0_LSB);
	printk("%s ctrl18 swipe_y0_MSB=0x%02x\n", __func__, ctrl_18->swipe_y0_MSB);
	printk("%s ctrl18 swipe_x1_LSB=0x%02x\n", __func__, ctrl_18->swipe_x1_LSB);
	printk("%s ctrl18 swipe_x1_MSB=0x%02x\n", __func__, ctrl_18->swipe_x1_MSB);
	printk("%s ctrl18 swipe_y1_LSB=0x%02x\n", __func__, ctrl_18->swipe_y1_LSB);
	printk("%s ctrl18 swipe_y1_MSB=0x%02x\n", __func__, ctrl_18->swipe_y1_MSB);
	*/
	//ctrl18-
		
	//Enable cover mode
    
	if (cover_mode == 1) {
        printk("%s into cover mode\n", __func__);
        //Active Zone control
        buf = 0x01;
        retval = synaptics_rmi4_reg_write(rmi4_data,
			F51_CUSTOM_CTRL,
			&buf,
			sizeof(buf));
		if (retval < 0)
			return -EINVAL;

		//ctrl10+
        if(glove_mode == 1) {
            ctrl_10->min_peak_amp = 0x0A;
		}else {
            ctrl_10->min_peak_amp = 0x0F;
        }
		
		retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f12_ctrl10_base_addr,
			ctrl_10->data,
			sizeof(ctrl_10->data));
		if (retval < 0)
			return -EINVAL;
		retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl10_base_addr,
			ctrl_10->data,
			sizeof(ctrl_10->data));
		if (retval < 0)
			return -EINVAL;
		
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s enable cover, min peak = 0x%x, ctrl_10->data[1]=0x%x\n", __func__, ctrl_10->min_peak_amp, ctrl_10->data[1]);
		//ctrl10-
		
		//ctrl15+				
		ctrl_15->finger_amp_threshold = 0x2C;
		retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f12_ctrl15_base_addr,
			ctrl_15->data,
			sizeof(ctrl_15->data));
 		if (retval < 0)
			return -EINVAL;
			
		retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl15_base_addr,
			ctrl_15->data,
			sizeof(ctrl_15->data));
 		if (retval < 0)
 			return -EINVAL;
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s ctrl_15->data=0x%02x, finger_thrshold=0x%02x\n", __func__, ctrl_15->data[0], ctrl_15->finger_amp_threshold);
		//ctrl15-
		
		//ctrl18+
		ctrl_18->double_tap_x0_LSB = 0x64;
		ctrl_18->double_tap_x0_MSB = 0x00;
		ctrl_18->double_tap_y0_LSB = 0x32;
		ctrl_18->double_tap_y0_MSB = 0x00;
		ctrl_18->double_tap_x1_LSB = 0xe8;
		ctrl_18->double_tap_x1_MSB = 0x03;
		ctrl_18->double_tap_y1_LSB = 0xe8;
		ctrl_18->double_tap_y1_MSB = 0x03;
		
		ctrl_18->swipe_x0_LSB = 0x64;
		ctrl_18->swipe_x0_MSB = 0x00;
		ctrl_18->swipe_y0_LSB = 0x32;
		ctrl_18->swipe_y0_MSB = 0x00;
		ctrl_18->swipe_x1_LSB = 0xe8;
		ctrl_18->swipe_x1_MSB = 0x03;
		ctrl_18->swipe_y1_LSB = 0xe8;
		ctrl_18->swipe_y1_MSB = 0x03;
		retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f12_ctrl18_base_addr,
			ctrl_18->data,
			sizeof(ctrl_18->data));
 		if (retval < 0)
 			return -EINVAL;
		//ctrl18-
	} else {
		//disable cover mode
        printk("%s not into cover mode\n", __func__);
        //Active Zone control
        buf = 0x00;
        retval = synaptics_rmi4_reg_write(rmi4_data,
			F51_CUSTOM_CTRL,
			&buf,
			sizeof(buf));
		if (retval < 0)
			return -EINVAL;
        
		//ctrl10+		
        if(glove_mode == 1) {
            ctrl_10->min_peak_amp = 0x0C;
        }else {
            ctrl_10->min_peak_amp = 0x14;
        }
		
		retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f12_ctrl10_base_addr,
			ctrl_10->data,
			sizeof(ctrl_10->data));
		if (retval < 0)
			return -EINVAL;
		
		retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl10_base_addr,
			ctrl_10->data,
			sizeof(ctrl_10->data));
		if (retval < 0)
			return -EINVAL;
		
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s disable cover, min peak = 0x%x, ctrl_10->data[1]=0x%x\n", __func__, ctrl_10->min_peak_amp, ctrl_10->data[1]);
		//ctrl10-
		//ctrl15+
		ctrl_15->finger_amp_threshold = 0x48;
		retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f12_ctrl15_base_addr,
			ctrl_15->data,
			sizeof(ctrl_15->data));
		if (retval < 0)
			return -EINVAL;
	
		retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl15_base_addr,
			ctrl_15->data,
			sizeof(ctrl_15->data));
		if (retval < 0)
			return -EINVAL;	
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s ctrl_15->data=0x%02x, finger_thrshold=0x%02x\n", __func__, ctrl_15->data[0], ctrl_15->finger_amp_threshold);
		//ctrl15-
		
		//ctrl18+
		ctrl_18->double_tap_x0_LSB = 0x9a;
		ctrl_18->double_tap_x0_MSB = 0x00;
		ctrl_18->double_tap_y0_LSB = 0x93;
		ctrl_18->double_tap_y0_MSB = 0x00;
		ctrl_18->double_tap_x1_LSB = 0x9e;
		ctrl_18->double_tap_x1_MSB = 0x03;
		ctrl_18->double_tap_y1_LSB = 0x80;
		ctrl_18->double_tap_y1_MSB = 0x07;
		
		ctrl_18->swipe_x0_LSB = 0x9a;
		ctrl_18->swipe_x0_MSB = 0x00;
		ctrl_18->swipe_y0_LSB = 0x93;
		ctrl_18->swipe_y0_MSB = 0x00;
		ctrl_18->swipe_x1_LSB = 0x9e;
		ctrl_18->swipe_x1_MSB = 0x03;
		ctrl_18->swipe_y1_LSB = 0x80;
		ctrl_18->swipe_y1_MSB = 0x07;
		retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f12_ctrl18_base_addr,
			ctrl_18->data,
			sizeof(ctrl_18->data));
 		if (retval < 0)
 			return -EINVAL;
		//ctrl18-
	}
	
exit:
	kfree(ctrl_10);
	kfree(ctrl_15);
	kfree(ctrl_18);
	
	return retval;
}
//<ASUS_COVER->
//<ASUS_Glove+>
static ssize_t synaptics_rmi4_set_glove_param(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
    unsigned char cap_buf[2] = {0};
	struct synaptics_rmi4_f12_ctrl_10 *ctrl_10 = NULL;
	struct synaptics_rmi4_f12_ctrl_23 *ctrl_23 = NULL;
	
	ctrl_10 = kmalloc(sizeof(*ctrl_10), GFP_KERNEL);
	if (!ctrl_10) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for ctrl_10\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}
	
	ctrl_23 = kmalloc(sizeof(*ctrl_23), GFP_KERNEL);
	if (!ctrl_23) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for ctrl_23\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}
	
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl10_base_addr,
			ctrl_10->data,
			sizeof(ctrl_10->data));
	if (retval < 0)
		return -EINVAL;	
		
	dev_dbg(rmi4_data->pdev->dev.parent,
				"%s old glove, min peak = 0x%x, ctrl_10->data[1]=0x%x\n", __func__, ctrl_10->min_peak_amp, ctrl_10->data[1]);
	
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl23_base_addr,
			ctrl_23->data,
			sizeof(ctrl_23->data));
	if (retval < 0)
		return -EINVAL;	
		
	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s old ctrl23 data[0] = 0x%x\n", __func__, ctrl_23->data[0]);
	//Enable glove mode
	if (glove_mode == 1) {
        printk("%s into glove mode\n", __func__);
		if (rmi4_data->usb_status == 1) {
            if(cap_sel_status == 0) { //<ASUS_focal+>
                printk("[cap] golve=1 usb=1 init\n");
                cap_sensor_glove_usb_mode_init(rmi4_data);
            }
		} else {
            if(cap_sel_status == 0) { //<ASUS_focal+>
                printk("[cap] golve=1 usb=0 init\n");
                cap_sensor_glove_init(rmi4_data);
            }
		}
        
        //<ASUS_focal+>
        if(cap_sel_status == 1) {            
            cap_buf[0] = 0xc0;
            cap_buf[1] = 0x01;
            retval = cap_i2c_write(rmi4_data, cap_buf, 2);
            if(retval < 0)
                printk("%s [fts] retval = %d glove mode write failure.\n", __func__, retval);
        }
        //<ASUS_focal->
        
		ctrl_23->gloved_finger_enable |= 0x01;		
		retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f12_ctrl23_base_addr,
			ctrl_23->data,
			sizeof(ctrl_23->data));
		if (retval < 0)
			return -EINVAL;		
		retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl23_base_addr,
			ctrl_23->data,
			sizeof(ctrl_23->data));
		if (retval < 0)
			return -EINVAL;	
			
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s enable glove =0x%x, ctrl23 data[0] = 0x%x\n", 
				__func__, ctrl_23->gloved_finger_enable, ctrl_23->data[0]);
                
        if(cover_mode == 1) {
            ctrl_10->min_peak_amp = 0x0A;
		}else {
            ctrl_10->min_peak_amp = 0x0C;
        }
		
		retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f12_ctrl10_base_addr,
			ctrl_10->data,
			sizeof(ctrl_10->data));
		if (retval < 0)
			return -EINVAL;
		retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl10_base_addr,
			ctrl_10->data,
			sizeof(ctrl_10->data));
		if (retval < 0)
			return -EINVAL;
		
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s enable glove, min peak = 0x%x, ctrl_10->data[1]=0x%x\n", __func__, ctrl_10->min_peak_amp, ctrl_10->data[1]);
	} else {
		//disable glove mode
        printk("%s not into glove mode\n", __func__);
		if (rmi4_data->usb_status == 1) { 
            if(cap_sel_status == 0) { //<ASUS_focal+>
                printk("[cap] golve=0 usb=1 init\n");
                cap_sensor_usb_init(rmi4_data);
            }
		} else {
            if(cap_sel_status == 0) { //<ASUS_focal+>
                printk("[cap] golve=0 usb=0 init\n");
                cap_sensor_reg_init(rmi4_data);
            }
		}
		
        //<ASUS_focal+>
        if(cap_sel_status == 1) {            
            cap_buf[0] = 0xc0;
            cap_buf[1] = 0x00;
            retval = cap_i2c_write(rmi4_data, cap_buf, 2);
            if(retval < 0)
                printk("%s [fts] retval = %d glove mode write failure.\n", __func__, retval);
        }
        //<ASUS_focal->
        
		ctrl_23->gloved_finger_enable &= 0x00;		
		retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f12_ctrl23_base_addr,
			ctrl_23->data,
			sizeof(ctrl_23->data));
		if (retval < 0)
			return -EINVAL;
		
		retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl23_base_addr,
			ctrl_23->data,
			sizeof(ctrl_23->data));
		if (retval < 0)
			return -EINVAL;	
		
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s disable glove =0x%x, ctrl23 data[0] = 0x%x\n", __func__, ctrl_23->gloved_finger_enable, ctrl_23->data[0]);
		
        if(cover_mode == 1) {
            ctrl_10->min_peak_amp = 0x0F;
        }else {
            ctrl_10->min_peak_amp = 0x14;
        }
		
		retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f12_ctrl10_base_addr,
			ctrl_10->data,
			sizeof(ctrl_10->data));
		if (retval < 0)
			return -EINVAL;
		
		retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f12_ctrl10_base_addr,
			ctrl_10->data,
			sizeof(ctrl_10->data));
		if (retval < 0)
			return -EINVAL;
		
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s disable glove, min peak = 0x%x, ctrl_10->data[1]=0x%x\n", __func__, ctrl_10->min_peak_amp, ctrl_10->data[1]);
	}
	
exit:
	kfree(ctrl_10);
	kfree(ctrl_23);
	
	return retval;
}
//<ASUS_Glove->
//<ASUS_cap_sensor+>
static ssize_t synaptics_rmi4_cap_sensor_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t synaptics_rmi4_cap_sensor_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);	
	ssize_t num_read_chars = 0;
	int retval;
	//u32 wmreg=0;
	long unsigned int wmreg=0;
	u8 buf_val[8]={0};
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5]={0};
	memset(valbuf, 0, sizeof(valbuf));

	mutex_lock(&(rmi4_data->cap_mutex));
	num_read_chars = count - 1;

	if(num_read_chars!=2)
	{
		if(num_read_chars!=4)
		{
			dev_err(dev, "please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = kstrtoul(valbuf, 16, &wmreg);
	if (0 != retval)
	{
		dev_err(dev, "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
		goto error_return;
	}

	if(2 == num_read_chars)
	{
		//read register
		regaddr = wmreg;
		if(cap_i2c_Read(rmi4_data, &regaddr, 1, &regvalue, 1) < 0)
			dev_err(dev, "Could not read the register(0x%02x)\n", regaddr);
		else
			printk("[cap] the register(0x%02x) is 0x%02x\n", regaddr, regvalue);
	}
	else
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		buf_val[0] = regaddr;
		buf_val[1] = regvalue;
		if(cap_i2c_write(rmi4_data, buf_val, 2) < 0)
			dev_err(dev, "Could not write the register(0x%02x)\n", regaddr);
		else
			printk("[cap] Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
	}
error_return:
	mutex_unlock(&(rmi4_data->cap_mutex));

	return count;
}

static ssize_t synaptics_rmi4_cap1_status_show(struct device *dev,
struct device_attribute *attr, char *buf) {
    //<ASUS_focal+>
	int local_back;
    if(cap_sel_status == 1) {
        local_back = fts_btn_back;
        printk("%s [FTS] back=%d\n", __func__, fts_btn_back);
    }else if(cap_sel_status == 0){
        local_back = btn_back;        
        printk("%s [cap] back=%d\n", __func__, btn_back);
    }
	return snprintf(buf, PAGE_SIZE, "%u\n",
			local_back);
    //<ASUS_focal->
}

static ssize_t synaptics_rmi4_cap2_status_show(struct device *dev,
struct device_attribute *attr, char *buf) {
    //<ASUS_focal+>
	int local_home;
    if(cap_sel_status == 1) {
        local_home = fts_btn_home;
        printk("%s [FTS] home=%d\n", __func__, fts_btn_home);
    }else if(cap_sel_status == 0){
        local_home = btn_home;        
        printk("%s [cap] home=%d\n", __func__, btn_home);
    }    
	return snprintf(buf, PAGE_SIZE, "%u\n",
			local_home);
    //<ASUS_focal->
}

static ssize_t synaptics_rmi4_cap3_status_show(struct device *dev,
struct device_attribute *attr, char *buf) {
    //<ASUS_focal+>
	int local_menu;
    if(cap_sel_status == 1) {
        local_menu = fts_btn_menu;
        printk("%s [FTS] menu=%d\n", __func__, fts_btn_menu);
    }else if(cap_sel_status == 0){
        local_menu = btn_menu;        
        printk("%s [cap] menu=%d\n", __func__, btn_menu);
    }    
	return snprintf(buf, PAGE_SIZE, "%u\n",
			local_menu);
    //<ASUS_focal->
}
//<ASUS_cap_sensor->

//<ASUS_DTP+>
static ssize_t synaptics_rmi4_dclick_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",
			dclick_mode);
}

static ssize_t synaptics_rmi4_dclick_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;
		
	dclick_mode = input;
	printk("[synaptics] dclick mode: %d\n", dclick_mode);

	return count;
}
//<ASUS_DTP->
//<ASUS_SWIPE+>
static ssize_t synaptics_rmi4_swipe_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",
			swipe_mode);
}

static ssize_t synaptics_rmi4_swipe_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;
		
	swipe_mode = input;
	printk("[synaptics] swipe mode: %d\n", swipe_mode);

	return count;
}
//<ASUS_SWIPE->
//<ASUS_Gesture+>
static ssize_t synaptics_rmi4_gesture_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",
			gesture_mode);
}

static ssize_t synaptics_rmi4_gesture_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char input[7] = {0};
	int value=0, i;
	
	if (sscanf(buf, "%s", input) != 1)
		return -EINVAL;

	for (i=0;i<7;i++)
	{
		value += (((int)input[i])-48)<<(6-i);
	}
	gesture_mode = value;
	printk("[synaptics] gesture mode: %d\n", gesture_mode);

	return count;
}
//<ASUS_Gesture->

//<ASUS_Glove+>
static ssize_t synaptics_rmi4_glove_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",
			glove_mode);
}

static ssize_t synaptics_rmi4_glove_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;	
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	
	
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;
		
	glove_mode = input;
	printk("[synaptics] glove_mode: %d\n", glove_mode);
		
	if(!fw_update_state)
		synaptics_rmi4_set_glove_param(rmi4_data);
	
	return count;
}
//<ASUS_Glove->

//<ASUS_COVER+>
static ssize_t synaptics_rmi4_cover_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",
			cover_mode);
}

static ssize_t synaptics_rmi4_cover_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;
		
	cover_mode = input;
	printk("[synaptics] cover_mode: %d\n", cover_mode);
		
	if(!fw_update_state)
		synaptics_rmi4_set_cover_param(rmi4_data);
	
	return count;
}
//<ASUS_COVER->

//<ASUS_focal+>
#define FTS_CFG_FILEPATH "/data/"
static int fts_ReadFirmware(char *firmware_name,unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FTS_CFG_FILEPATH, firmware_name);
    printk("%s [fts] filepath = %s\n" ,__func__, filepath);
	if (NULL == pfile)
	{
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) 
	{
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

static int fts_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	memset(filepath, 0, sizeof(filepath)); 
    sprintf(filepath, "%s%s", FTS_CFG_FILEPATH, firmware_name);
    printk("%s [fts] filepath = %s\n" ,__func__, filepath);
	if (NULL == pfile)
	{
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) 
	{
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static ssize_t fts_fwupgradeapp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char fwname[128];
    u8 *pbt_buf = NULL;
    int i_ret = 0;    
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    int fwsize;
    printk("%s [fts] start\n", __func__);
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

    fwsize = fts_GetFirmwareSize(fwname);
    printk("%s [fts] fw name = %s\n", __func__, fwname);
    if (fwsize <= 0) 
	{
		printk("%s ERROR:Get firmware size failed\n",__func__);
		return -EIO;
	}    
	
	disable_irq(CAP_INT_GET_PIN(CAP_INT_PIN));
    pbt_buf = (unsigned char *)kmalloc(fwsize + 1, GFP_KERNEL);
	if (fts_ReadFirmware(fwname, pbt_buf)) 
	{
        printk("%s() - ERROR: request_firmware failed\n",__func__);
		kfree(pbt_buf);
		return -EIO;        
    }
    i_ret = fts_6336GU_ctpm_fw_upgrade(rmi4_data, pbt_buf, fwsize);
    if (i_ret != 0)
		printk("%s() - ERROR:[FTS] upgrade failed..\n",
					__func__);
                    
	enable_irq(CAP_INT_GET_PIN(CAP_INT_PIN));
    kfree(pbt_buf);
	return count;
}

int FTS_I2c_Read(unsigned char *wBuf, int wLen, unsigned char *rBuf, int rLen)
{
	return cap_i2c_Read(gb_rmi4_data, wBuf, wLen, rBuf, rLen);
}

int FTS_I2c_Write(unsigned char *wBuf, int wLen)
{
	return cap_i2c_write(gb_rmi4_data, wBuf, wLen);
}

static int fts_GetInISize(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FTS_CFG_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("[FTS][TOUCH_ERR] %s : error occured while opening file %s.\n", __func__, filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int fts_ReadInIData(char *config_name, char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FTS_CFG_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("[FTS][TOUCH_ERR] %s : error occured while opening file %s.\n", __func__, filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

static int fts_get_testparam_from_ini(char *config_name)
{
	char *config_data = NULL;
	int file_size;

	file_size = fts_GetInISize(config_name);

	pr_err("[FTS][Touch] %s : inisize = %d\n ", __func__, file_size);
	if (file_size <= 0) {
		pr_err("[FTS][TOUCH_ERR] %s : ERROR : Get firmware size failed\n", __func__);
		return -EIO;
	}

	config_data = kmalloc(file_size + 1, GFP_KERNEL);

	if (fts_ReadInIData(config_name, config_data)) {
		pr_err("[FTS][TOUCH_ERR] %s() - ERROR: request_firmware failed\n", __func__);
		kfree(config_data);
		return -EIO;
	} else {
		pr_info("[FTS][Touch] %s : fts_ReadInIData successful\n", __func__);
	}

	set_param_data(config_data);

	return 0;
}

int selft_test_result = 0;
static ssize_t fts_ftsselftest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    if(cap_sel_status == 1) {
        if (selft_test_result) {
            printk("[FTS] %s: Selftest FAIL\n", __func__);
        } else {
            printk("[FTS] %s : Selftest PASS\n", __func__);
        }
        return sprintf(buf, "[FTS] : Selftest %s\n", selft_test_result ? "FAIL" : "PASS");
    }else {
        return sprintf(buf, "[cap] Is not focal chip\n");
    }	
}

static mm_segment_t oldfs;

static struct file *fts_selftest_file_open(void)
{

	struct file *filp = NULL;
	char filepath[128];
	int err = 0;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FTS_CFG_FILEPATH, "test_result.csv");

	oldfs = get_fs();
	set_fs(get_ds());

	filp = filp_open(filepath, O_WRONLY|O_CREAT, 0644);
	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}

	return filp;
}

int fts_selftest_file_write(struct file *file, unsigned char *data, int len)
{

	int ret;

	ret = file->f_op->write(file, data, len, &file->f_pos);

	return ret;
}

void fts_selftest_file_close(struct file *file)
{

	set_fs(oldfs);
	filp_close(file, NULL);
}

static void fts_sw_reset(struct synaptics_rmi4_data *rmi4_data)
{
    int i;
    u8 auc_i2c_write_buf[10];
    u8 buf[2] = {0};
    u8 reg_val[2] = {0};
    
    for(i =0;i<5;i++) {
        /*********Step 1:Reset  CTPM *****/
        buf[0] = FTS_RST_CMD_REG2;
        buf[1] = FTS_UPGRADE_AA;
        cap_i2c_write(rmi4_data, buf, 2);
        msleep(FTS_DELAY_TIME_AA);
        
        buf[0] = FTS_RST_CMD_REG2;
        buf[1] = FTS_UPGRADE_55;
        cap_i2c_write(rmi4_data, buf, 2);
        msleep(FTS_DELAY_TIME_55);
        /*********Step 2:Enter upgrade mode *****/
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        cap_i2c_write(rmi4_data, auc_i2c_write_buf, 1);
        auc_i2c_write_buf[0] = FTS_UPGRADE_AA;
        cap_i2c_write(rmi4_data, auc_i2c_write_buf, 1);
        msleep(10);    
        
        /*********Step 3:check READ-ID***********************/		
		auc_i2c_write_buf[0] = FTS_READ_ID_REG;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		cap_i2c_Read(rmi4_data, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == FTS_UPGRADE_ID_1
			&& reg_val[1] == FTS_UPGRADE_ID_2) 
		{
			printk("%s [FTS] : GET CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
				__func__, reg_val[0], reg_val[1]);
			break;
		} 
		else 
		{
			printk("%s [FTS] : GET CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				__func__, reg_val[0], reg_val[1]);
		}
    }
    
    /*********Step 7: reset the new FW***********************/
	auc_i2c_write_buf[0] = 0x07;
	cap_i2c_write(rmi4_data, auc_i2c_write_buf, 1);
	//msleep(200);
}

static ssize_t fts_fts_sw_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    ssize_t num_read_chars = 0;
    
    num_read_chars = count - 1;
    
    if(1 == num_read_chars) {
        printk("%s [fts] excution sw reset\n", __func__);
        fts_sw_reset(rmi4_data);
    }
    
    return count;
}

static ssize_t fts_ftsselftest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	char config_file[128], result_buf[64];
	char *w_buf;
	struct file *w_file;
	int w_len;

	//mutex_lock(&g_device_mutex);
    if(cap_sel_status == 1) {
        disable_irq(CAP_INT_GET_PIN(CAP_INT_PIN));

        selft_test_result = 0;

        memset(config_file, 0, sizeof(config_file));
        sprintf(config_file, "%s", buf);
        config_file[count-1] = '\0';
        
        if (fts_get_testparam_from_ini(config_file) < 0) {
        }        

        if (fts_get_testparam_from_ini(config_file) < 0) {
            printk("[FTS][TOUCH_ERR] %s : get testparam from ini failure\n", __func__);
        } else {
            printk("[FTS][Touch] %s : tp test Start...\n", __func__);

            if (start_test_tp()) {
                printk("[FTS][Touch] %s : tp test pass\n", __func__);
                selft_test_result = 0;
            } else {
                printk("[FTS][Touch] %s : tp test failure\n", __func__);
                selft_test_result = 1;
            }

            /*for (i = 0; i < 3; i++) {
                if (cap_i2c_write(rmi4_data, 0x00, 1) >= 0)
                    break;
                else
                    msleep(200);
            }*/

            w_file = fts_selftest_file_open();
            if (!w_file)
                printk("[FTS][Touch] %s : Open log file fail !\n", __func__);
            else{
                w_buf =kmalloc(1024*80,  GFP_KERNEL);
                if (!w_buf)
                    printk("[FTS][Touch] %s : allocate memory fail !\n", __func__);
                else{
                    w_len = get_test_data(w_buf);
                    fts_selftest_file_write(w_file, w_buf, w_len);
                    w_len = sprintf(result_buf, "[FTS] : Selftest %s\n", selft_test_result ? "FAIL" : "PASS");
                    fts_selftest_file_write(w_file, result_buf, w_len);

                    kfree(w_buf);
                }
                fts_selftest_file_close(w_file);
                FreeStoreAllData();
            }

            free_test_param_data();
        }
        fts_sw_reset(rmi4_data);
        enable_irq(CAP_INT_GET_PIN(CAP_INT_PIN));
        //mutex_unlock(&g_device_mutex);
        
    }else if(cap_sel_status == 0) {
        printk("%s [cap] Is not focal chip\n", __func__);
    }
    return count;
}

static ssize_t cap_chip_select_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
    if (cap_sel_status == 0) {
		printk("%s [cap]: micro chip\n", __func__);
	} else if(cap_sel_status == 1){
		printk("%s [cap][FTS]: focal chip\n", __func__);
	}

	return sprintf(buf, "%d\n", cap_sel_status);
}

static ssize_t fts_fw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    unsigned char addr=0x00;
    unsigned char buf_val=0x00;
    
    addr = 0xa6;
    cap_i2c_Read(rmi4_data, &addr, 1, &buf_val, 1);
    printk("%s [FTS] focal cap fw version = 0x%02x\n", __func__, buf_val);
    
    return sprintf(buf, "[FTS] focal cap fw version = 0x%02x\n", buf_val);
}
//<ASUS_focal->

//<ASUS_Proximity+>
#ifdef ASUS_TOUCH_PROXIMITY_NODE
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
static ssize_t tp_proximity_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
	{
	char *str;
	int len, retval;

	if (touch_proximity_at_phone == 1) {		//No Touch
		printk("[Synaptics] Touch is disabled now\n");
		str = "Touch is disabled now\n";
	} else if (touch_proximity_at_phone == 0) {	//Touch
		printk("[Synaptics] Touch is enabled now\n");
		str = "Touch is enabled now\n";
	}

	len = strlen(str);
	if(copy_to_user(buf, str, len))
		retval = -EFAULT;
	else if (*ppos == 0)
 		*ppos += len;
	else
		len = 0;
	return len;
}
static ssize_t tp_proximity_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char str[128];
	if (count > PAGE_SIZE) //procfs write and read has PAGE_SIZE limit
		count = 128;

    if (copy_from_user(str, buf, count))
	{
		printk("copy_from_user failed!\n");
		return -EFAULT;
        }

	if (count > 1)
	{
		str[count-1] = '\0';
	}

	if ((int)(str[0]) == (1+48)) {		//No Touch
		touch_proximity_at_phone = 1;
		printk("[Synaptics] Disable Touch\n");
	} else {				//Touch
		touch_proximity_at_phone = 0;
		printk("[Synaptics] Enable Touch\n");
	}

	return count;
}
#else
static int tp_proximity_proc_read(char *buf, char **start, off_t offset, int request,
				     int *eof, void *data)
{
	if (touch_proximity_at_phone == 1) {		//No Touch
		printk("[Synaptics] Touch is disabled now\n");
		return sprintf(buf, "Touch is disabled now\n");
	} else if (touch_proximity_at_phone == 0) {	//Touch
		printk("[Synaptics] Touch is enabled now\n");
		return sprintf(buf, "Touch is enabled now\n");
	}

	return 0;
}
static int tp_proximity_proc_write(struct file *file, const char *buffer,
				      unsigned long count, void *data)
{
        //struct device_private *dpp = container_of(data, struct device_private, driver_data);
	//struct device *dev = container_of(dpp, struct device, p);

	if ((int)(*buffer) == (1+48)) {		//No Touch
		touch_proximity_at_phone = 1;
		printk("[Synaptics] Disable Touch\n");
	} else {				//Touch
		touch_proximity_at_phone = 0;
		printk("[Synaptics] Enable Touch\n");
	}

	return count;
}
#endif
#endif
//<ASUS_Proximity->

static int synaptics_rmi4_debug_suspend_set(void *_data, u64 val)
{
	struct synaptics_rmi4_data *rmi4_data = _data;

	if (val)
		synaptics_rmi4_suspend(&rmi4_data->input_dev->dev);
	else
		synaptics_rmi4_resume(&rmi4_data->input_dev->dev);

	return 0;
}

static int synaptics_rmi4_debug_suspend_get(void *_data, u64 *val)
{
	struct synaptics_rmi4_data *rmi4_data = _data;

	*val = rmi4_data->suspend;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, synaptics_rmi4_debug_suspend_get,
			synaptics_rmi4_debug_suspend_set, "%lld\n");

static struct kobj_attribute virtual_key_map_attr = {
	.attr = {
		.name = VIRTUAL_KEY_MAP_FILE_NAME,
		.mode = S_IRUGO,
	},
	.show = synaptics_rmi4_virtual_key_map_show,
};

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int reset;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	retval = synaptics_rmi4_reset_device(rmi4_data, false);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		return retval;
	}

	return count;
}

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
			(rmi4_data->rmi4_mod_info.product_info[0]),
			(rmi4_data->rmi4_mod_info.product_info[1]));
}

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u_%x\n",
			rmi4_data->firmware_id, rmi4_data->config_id); //<ASUS_buildid+>
}

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	struct synaptics_rmi4_f01_device_status device_status;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			device_status.data,
			sizeof(device_status.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read device status, error = %d\n",
				__func__, retval);
		return retval;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n",
			device_status.flash_prog);
}

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->button_0d_enabled);
}

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	unsigned char ii;
	unsigned char intr_enable;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (rmi4_data->button_0d_enabled == input)
		return count;

	if (list_empty(&rmi->support_fn_list))
		return -ENODEV;

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
			ii = fhandler->intr_reg_num;

			retval = synaptics_rmi4_reg_read(rmi4_data,
					rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;

			if (input == 1)
				intr_enable |= fhandler->intr_mask;
			else
				intr_enable &= ~fhandler->intr_mask;

			retval = synaptics_rmi4_reg_write(rmi4_data,
					rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;
		}
	}

	rmi4_data->button_0d_enabled = input;

	return count;
}

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input == 1)
		synaptics_rmi4_suspend(dev);
	else if (input == 0)
		synaptics_rmi4_resume(dev);
	else
		return -EINVAL;

	return count;
}

static ssize_t synaptics_rmi4_wake_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->enable_wakeup_gesture);
}

static ssize_t synaptics_rmi4_wake_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (rmi4_data->f11_wakeup_gesture || rmi4_data->f12_wakeup_gesture)
		rmi4_data->enable_wakeup_gesture = input;

	return count;
}

static ssize_t synaptics_rmi4_virtual_key_map_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ii;
	int cnt;
	int count = 0;

	for (ii = 0; ii < vir_button_map->nbuttons; ii++) {
		cnt = snprintf(buf, PAGE_SIZE - count, "0x01:%d:%d:%d:%d:%d\n",
				vir_button_map->map[ii * 5 + 0],
				vir_button_map->map[ii * 5 + 1],
				vir_button_map->map[ii * 5 + 2],
				vir_button_map->map[ii * 5 + 3],
				vir_button_map->map[ii * 5 + 4]);
		buf += cnt;
		count += cnt;
	}

	return count;
}

static int synaptics_rmi4_f11_abs_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char reg_index;
	unsigned char finger;
	unsigned char fingers_supported;
	unsigned char num_of_finger_status_regs;
	unsigned char finger_shift;
	unsigned char finger_status;
	unsigned char finger_status_reg[3];
	unsigned char detected_gestures;
	unsigned short data_addr;
	unsigned short data_offset;
	int x;
	int y;
	int wx;
	int wy;
	int temp;
	struct synaptics_rmi4_f11_data_1_5 data;
	struct synaptics_rmi4_f11_extra_data *extra_data;

	/*
	 * The number of finger status registers is determined by the
	 * maximum number of fingers supported - 2 bits per finger. So
	 * the number of finger status registers to read is:
	 * register_count = ceil(max_num_of_fingers / 4)
	 */
	fingers_supported = fhandler->num_of_data_points;
	num_of_finger_status_regs = (fingers_supported + 3) / 4;
	data_addr = fhandler->full_addr.data_base;

	extra_data = (struct synaptics_rmi4_f11_extra_data *)fhandler->extra;

	if (rmi4_data->suspend && rmi4_data->enable_wakeup_gesture) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				data_addr + extra_data->data38_offset,
				&detected_gestures,
				sizeof(detected_gestures));
		if (retval < 0)
			return 0;

		if (detected_gestures) {
			input_report_key(rmi4_data->input_dev, KEY_POWER, 1);
			input_sync(rmi4_data->input_dev);
			input_report_key(rmi4_data->input_dev, KEY_POWER, 0);
			input_sync(rmi4_data->input_dev);
			rmi4_data->suspend = false;
		}

		return 0;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr,
			finger_status_reg,
			num_of_finger_status_regs);
	if (retval < 0)
		return 0;

	mutex_lock(&(rmi4_data->rmi4_report_mutex));

	for (finger = 0; finger < fingers_supported; finger++) {
		reg_index = finger / 4;
		finger_shift = (finger % 4) * 2;
		finger_status = (finger_status_reg[reg_index] >> finger_shift)
				& MASK_2BIT;

		/*
		 * Each 2-bit finger status field represents the following:
		 * 00 = finger not present
		 * 01 = finger present and data accurate
		 * 10 = finger present but data may be inaccurate
		 * 11 = reserved
		 */
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, finger_status);
#endif

		if (finger_status) {
			data_offset = data_addr +
					num_of_finger_status_regs +
					(finger * sizeof(data.data));
			retval = synaptics_rmi4_reg_read(rmi4_data,
					data_offset,
					data.data,
					sizeof(data.data));
			if (retval < 0) {
				touch_count = 0;
				goto exit;
			}

			x = (data.x_position_11_4 << 4) | data.x_position_3_0;
			y = (data.y_position_11_4 << 4) | data.y_position_3_0;
			wx = data.wx;
			wy = data.wy;

			if (rmi4_data->hw_if->board_data->swap_axes) {
				temp = x;
				x = y;
				y = temp;
				temp = wx;
				wx = wy;
				wy = temp;
			}

			if (rmi4_data->hw_if->board_data->x_flip)
				x = rmi4_data->sensor_max_x - x;
			if (rmi4_data->hw_if->board_data->y_flip)
				y = rmi4_data->sensor_max_y - y;

			input_report_key(rmi4_data->input_dev,
					BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,
					BTN_TOOL_FINGER, 1);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif

			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: Finger %d: "
					"status = 0x%02x, "
					"x = %d, "
					"y = %d, "
					"wx = %d, "
					"wy = %d\n",
					__func__, finger,
					finger_status,
					x, y, wx, wy);

			touch_count++;
		}
	}

	if (touch_count == 0) {
		input_report_key(rmi4_data->input_dev,
				BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,
				BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
#endif
	}

	input_sync(rmi4_data->input_dev);

exit:
	mutex_unlock(&(rmi4_data->rmi4_report_mutex));

	return touch_count;
}

#ifdef WAKEUP_GESTURE
static void synaptics_check_gesture(struct synaptics_rmi4_data *rmi4_data, unsigned char gesture_type, unsigned char gesture_alphabet)
{		
	switch(gesture_type) {
			case GESTURE_DOUBLECLICK:
				if (dclick_mode == 1) {
					printk("%s double click wake up\n", __func__);
					input_report_key(rmi4_data->input_dev, KEY_GESTURE_DCLICK, 1);
					input_sync(rmi4_data->input_dev);
					input_report_key(rmi4_data->input_dev, KEY_GESTURE_DCLICK, 0);
					input_sync(rmi4_data->input_dev);	
				}
				break;
			case GESTURE_SWIPE:
				if (swipe_mode == 1) {
					printk("%s swipe wake up\n", __func__);
					input_report_key(rmi4_data->input_dev, KEY_GESTURE_SWIPE_UP, 1);
					input_sync(rmi4_data->input_dev);
					input_report_key(rmi4_data->input_dev, KEY_GESTURE_SWIPE_UP, 0);
					input_sync(rmi4_data->input_dev);
				}
				break;
			case GESTURE_VEE_REG:
				if ((gesture_mode & GESTURE_SWITCH) && (gesture_mode & 0x01) && (gesture_alphabet == 0x02)) {
					printk("%s gesture v wake up\n", __func__);
					input_report_key(rmi4_data->input_dev, KEY_GESTURE_V, 1);
					input_sync(rmi4_data->input_dev);
					input_report_key(rmi4_data->input_dev, KEY_GESTURE_V, 0);
					input_sync(rmi4_data->input_dev);
				}
				break;
			case GESTURE_OTHERS:
				if (gesture_alphabet == GESTURE_E) {
					if ((gesture_mode & GESTURE_SWITCH) && (gesture_mode & 0x08)) {
						printk("%s gesture e wake up\n", __func__);
						input_report_key(rmi4_data->input_dev, KEY_GESTURE_E, 1);
						input_sync(rmi4_data->input_dev);
						input_report_key(rmi4_data->input_dev, KEY_GESTURE_E, 0);
						input_sync(rmi4_data->input_dev);
					}
				}else if (gesture_alphabet == GESTURE_C) {
					if ((gesture_mode & GESTURE_SWITCH) && (gesture_mode & 0x04)) {
						printk("%s gesture c wake up\n", __func__);
						input_report_key(rmi4_data->input_dev, KEY_GESTURE_C, 1);
						input_sync(rmi4_data->input_dev);
						input_report_key(rmi4_data->input_dev, KEY_GESTURE_C, 0);
						input_sync(rmi4_data->input_dev);
					}
				}else if (gesture_alphabet == GESTURE_W) {
					if ((gesture_mode & GESTURE_SWITCH) && (gesture_mode & 0x20)) {
						printk("%s gesture w wake up\n", __func__);
						input_report_key(rmi4_data->input_dev, KEY_GESTURE_W, 1);
						input_sync(rmi4_data->input_dev);
						input_report_key(rmi4_data->input_dev, KEY_GESTURE_W, 0);
						input_sync(rmi4_data->input_dev);
					}
				}else if (gesture_alphabet == GESTURE_S) {
					if ((gesture_mode & GESTURE_SWITCH) && (gesture_mode & 0x10)) {
						printk("%s gesture s wake up\n", __func__);
						input_report_key(rmi4_data->input_dev, KEY_GESTURE_S, 1);
						input_sync(rmi4_data->input_dev);
						input_report_key(rmi4_data->input_dev, KEY_GESTURE_S, 0);
						input_sync(rmi4_data->input_dev);
					}
				}else if (gesture_alphabet == GESTURE_Z) {
					if ((gesture_mode & GESTURE_SWITCH) && (gesture_mode & 0x02)) {
						printk("%s gesture z wake up\n", __func__);
						input_report_key(rmi4_data->input_dev, KEY_GESTURE_Z, 1);
						input_sync(rmi4_data->input_dev);
						input_report_key(rmi4_data->input_dev, KEY_GESTURE_Z, 0);
						input_sync(rmi4_data->input_dev);
					}
				}
				break;
			default:
				printk("%s gesture can't parse gesture type =0x%02x\n gesture gesture_alphabet =0x%02x", __func__, gesture_alphabet, gesture_alphabet);
				break;
		}		
}
#endif

static int synaptics_rmi4_f12_abs_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char index;
	unsigned char finger;
	unsigned char fingers_to_process;
	unsigned char finger_status;
	unsigned char size_of_2d_data;
	unsigned char gesture_type;
	unsigned short data_addr;
#ifdef WAKEUP_GESTURE
	unsigned char gesture_alphabet;
#endif
	int x;
	int y;
	int wx;
	int wy;
	int temp;
#ifdef REPORT_2D_PRESSURE
	int pressure;
#endif
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_finger_data *data;
	struct synaptics_rmi4_f12_finger_data *finger_data;
	static unsigned char finger_presence;
	static unsigned char stylus_presence;
#ifdef F12_DATA_15_WORKAROUND
	static unsigned char objects_already_present;
#endif

	fingers_to_process = fhandler->num_of_data_points;
	data_addr = fhandler->full_addr.data_base;
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

	if (rmi4_data->suspend && rmi4_data->enable_wakeup_gesture) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				data_addr + extra_data->data4_offset,
				rmi4_data->gesture_detection,
				sizeof(rmi4_data->gesture_detection));
		if (retval < 0)
			return 0;
		gesture_type = rmi4_data->gesture_detection[0];
		gesture_alphabet = rmi4_data->gesture_detection[2];
		
		synaptics_check_gesture(rmi4_data, gesture_type, gesture_alphabet);
		/* //synaptics original code double tap
		if (gesture_type && gesture_type != F12_UDG_DETECT) {
			input_report_key(rmi4_data->input_dev, KEY_POWER, 1);
			input_sync(rmi4_data->input_dev);
			input_report_key(rmi4_data->input_dev, KEY_POWER, 0);
			input_sync(rmi4_data->input_dev);
			rmi4_data->suspend = false;
		}*/

		return 0;
	}

	/* Determine the total number of fingers to process */
	if (extra_data->data15_size) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				data_addr + extra_data->data15_offset,
				extra_data->data15_data,
				extra_data->data15_size);
		if (retval < 0)
			return 0;

		/* Start checking from the highest bit */
		index = extra_data->data15_size - 1; /* Highest byte */
		finger = (fingers_to_process - 1) % 8; /* Highest bit */
		do {
			if (extra_data->data15_data[index] & (1 << finger))
				break;

			if (finger) {
				finger--;
			} else if (index > 0) {
				index--; /* Move to the next lower byte */
				finger = 7;
			}

			fingers_to_process--;
		} while (fingers_to_process);

		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Number of fingers to process = %d\n",
			__func__, fingers_to_process);
	}

#ifdef F12_DATA_15_WORKAROUND
	fingers_to_process = max(fingers_to_process, objects_already_present);
#endif

	if (!fingers_to_process) {
		synaptics_rmi4_free_fingers(rmi4_data);
		finger_presence = 0;
		stylus_presence = 0;
		return 0;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr + extra_data->data1_offset,
			(unsigned char *)fhandler->data,
			fingers_to_process * size_of_2d_data);
	if (retval < 0)
		return 0;

	data = (struct synaptics_rmi4_f12_finger_data *)fhandler->data;

#ifdef REPORT_2D_PRESSURE
	if (rmi4_data->report_pressure) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				data_addr + extra_data->data23_offset,
				extra_data->data23_data,
				fingers_to_process);
		if (retval < 0)
			return 0;
	}
#endif

	mutex_lock(&(rmi4_data->rmi4_report_mutex));

	for (finger = 0; finger < fingers_to_process; finger++) {
		finger_data = data + finger;
		finger_status = finger_data->object_type_and_status;

//<ASUS_Proximity+>
#ifdef ASUS_TOUCH_PROXIMITY_NODE
		if (touch_proximity_at_phone == 1) {
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(rmi4_data->input_dev, finger);
			input_mt_report_slot_state(rmi4_data->input_dev,
			MT_TOOL_FINGER, 0);
#endif		
		} else {
#endif
//<ASUS_Proximity->
#ifdef F12_DATA_15_WORKAROUND
		objects_already_present = finger + 1;
#endif

		x = (finger_data->x_msb << 8) | (finger_data->x_lsb);
		y = (finger_data->y_msb << 8) | (finger_data->y_lsb);
#ifdef REPORT_2D_W
		wx = finger_data->wx;
		wy = finger_data->wy;
#endif

		if (rmi4_data->hw_if->board_data->swap_axes) {
			temp = x;
			x = y;
			y = temp;
			temp = wx;
			wx = wy;
			wy = temp;
		}

		if (rmi4_data->hw_if->board_data->x_flip)
			x = rmi4_data->sensor_max_x - x;
		if (rmi4_data->hw_if->board_data->y_flip)
			y = rmi4_data->sensor_max_y - y;

        //<ASUS_COVER+>
        if(cover_mode == 1 && ((y >= 940 && y <=1919) || (y >= 0 && y <=20))) {
            input_mt_slot(rmi4_data->input_dev, finger);
			input_mt_report_slot_state(rmi4_data->input_dev,
					MT_TOOL_FINGER, 0);
            
        }else {
        //<ASUS_COVER->

		switch (finger_status) {
		case F12_FINGER_STATUS:
		case F12_GLOVED_FINGER_STATUS:
			if (stylus_presence) /* Stylus has priority over fingers */
				break;
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(rmi4_data->input_dev, finger);
			input_mt_report_slot_state(rmi4_data->input_dev,
					MT_TOOL_FINGER, 1);
#endif

			input_report_key(rmi4_data->input_dev,
					BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,
					BTN_TOOL_FINGER, 1);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_W
			if (rmi4_data->wedge_sensor) {
				input_report_abs(rmi4_data->input_dev,
						ABS_MT_TOUCH_MAJOR, wx);
				input_report_abs(rmi4_data->input_dev,
						ABS_MT_TOUCH_MINOR, wx);
			} else {
				input_report_abs(rmi4_data->input_dev,
						ABS_MT_TOUCH_MAJOR,
						max(wx, wy));
				input_report_abs(rmi4_data->input_dev,
						ABS_MT_TOUCH_MINOR,
						min(wx, wy));
			}
#endif
#ifdef REPORT_2D_PRESSURE
			if (rmi4_data->report_pressure) {
				pressure = extra_data->data23_data[finger];
				input_report_abs(rmi4_data->input_dev,
						ABS_MT_PRESSURE, pressure);
			}
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif
			input_sync(rmi4_data->input_dev);

			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: Finger %d: "
					"status = 0x%02x, "
					"x = %d, "
					"y = %d, "
					"wx = %d, "
					"wy = %d\n",
					__func__, finger,
					finger_status,
					x, y, wx, wy);

			finger_presence = 1;
			touch_count++;
			break;
		case F12_PALM_STATUS:
			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: Finger %d: "
					"x = %d, "
					"y = %d, "
					"wx = %d, "
					"wy = %d\n",
					__func__, finger,
					x, y, wx, wy);
			break;
		case F12_STYLUS_STATUS:
		case F12_ERASER_STATUS:
			if (finger_presence) { /* Stylus has priority over fingers */
				mutex_unlock(&(rmi4_data->rmi4_report_mutex));
				synaptics_rmi4_free_fingers(rmi4_data);
				mutex_lock(&(rmi4_data->rmi4_report_mutex));
				finger_presence = 0;
			}
			if (stylus_presence) {/* Allow one stylus at a timee */
				if (finger + 1 != stylus_presence)
					break;
			}
			input_report_key(rmi4_data->stylus_dev,
					BTN_TOUCH, 1);
			if (finger_status == F12_STYLUS_STATUS) {
				input_report_key(rmi4_data->stylus_dev,
						BTN_TOOL_PEN, 1);
			} else {
				input_report_key(rmi4_data->stylus_dev,
						BTN_TOOL_RUBBER, 1);
			}
			input_report_abs(rmi4_data->stylus_dev,
					ABS_X, x);
			input_report_abs(rmi4_data->stylus_dev,
					ABS_Y, y);
			input_sync(rmi4_data->stylus_dev);

			stylus_presence = finger + 1;
			touch_count++;
			break;
		default:
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(rmi4_data->input_dev, finger);
			input_mt_report_slot_state(rmi4_data->input_dev,
					MT_TOOL_FINGER, 0);
#endif
			break;
            }
        }//<ASUS_COVER+>
//<ASUS_Proximity+>
#ifdef ASUS_TOUCH_PROXIMITY_NODE
		}
#endif
//<ASUS_Proximity->
	}

	if (touch_count == 0) {
		finger_presence = 0;
#ifdef F12_DATA_15_WORKAROUND
		objects_already_present = 0;
#endif
		input_report_key(rmi4_data->input_dev,
				BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,
				BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
#endif
		input_sync(rmi4_data->input_dev);

		if (rmi4_data->stylus_enable) {
			stylus_presence = 0;
			input_report_key(rmi4_data->stylus_dev,
					BTN_TOUCH, 0);
			input_report_key(rmi4_data->stylus_dev,
					BTN_TOOL_PEN, 0);
			if (rmi4_data->eraser_enable) {
				input_report_key(rmi4_data->stylus_dev,
						BTN_TOOL_RUBBER, 0);
			}
			input_sync(rmi4_data->stylus_dev);
		}
	}

	mutex_unlock(&(rmi4_data->rmi4_report_mutex));

	return touch_count;
}

static void synaptics_rmi4_f1a_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0;
	unsigned char button;
	unsigned char index;
	unsigned char shift;
	unsigned char status;
	unsigned char *data;
	unsigned short data_addr = fhandler->full_addr.data_base;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	static unsigned char do_once = 1;
	static bool current_status[MAX_NUMBER_OF_BUTTONS];
#ifdef NO_0D_WHILE_2D
	static bool before_2d_status[MAX_NUMBER_OF_BUTTONS];
	static bool while_2d_status[MAX_NUMBER_OF_BUTTONS];
#endif

	if (do_once) {
		memset(current_status, 0, sizeof(current_status));
#ifdef NO_0D_WHILE_2D
		memset(before_2d_status, 0, sizeof(before_2d_status));
		memset(while_2d_status, 0, sizeof(while_2d_status));
#endif
		do_once = 0;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr,
			f1a->button_data_buffer,
			f1a->button_bitmask_size);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read button data registers\n",
				__func__);
		return;
	}

	data = f1a->button_data_buffer;

	mutex_lock(&(rmi4_data->rmi4_report_mutex));

	for (button = 0; button < f1a->valid_button_count; button++) {
		index = button / 8;
		shift = button % 8;
		status = ((data[index] >> shift) & MASK_1BIT);

		if (current_status[button] == status)
			continue;
		else
			current_status[button] = status;

		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Button %d (code %d) ->%d\n",
				__func__, button,
				f1a->button_map[button],
				status);
#ifdef NO_0D_WHILE_2D
		if (rmi4_data->fingers_on_2d == false) {
			if (status == 1) {
				before_2d_status[button] = 1;
			} else {
				if (while_2d_status[button] == 1) {
					while_2d_status[button] = 0;
					continue;
				} else {
					before_2d_status[button] = 0;
				}
			}
			touch_count++;
			input_report_key(rmi4_data->input_dev,
					f1a->button_map[button],
					status);
		} else {
			if (before_2d_status[button] == 1) {
				before_2d_status[button] = 0;
				touch_count++;
				input_report_key(rmi4_data->input_dev,
						f1a->button_map[button],
						status);
			} else {
				if (status == 1)
					while_2d_status[button] = 1;
				else
					while_2d_status[button] = 0;
			}
		}
#else
		touch_count++;
		input_report_key(rmi4_data->input_dev,
				f1a->button_map[button],
				status);
#endif
	}

	if (touch_count)
		input_sync(rmi4_data->input_dev);

	mutex_unlock(&(rmi4_data->rmi4_report_mutex));

	return;
}

static void synaptics_rmi4_report_touch(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	unsigned char touch_count_2d;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Function %02x reporting\n",
			__func__, fhandler->fn_number);

	switch (fhandler->fn_number) {
	case SYNAPTICS_RMI4_F11:
		touch_count_2d = synaptics_rmi4_f11_abs_report(rmi4_data,
				fhandler);

		if (touch_count_2d)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;
	case SYNAPTICS_RMI4_F12:
		touch_count_2d = synaptics_rmi4_f12_abs_report(rmi4_data,
				fhandler);

		if (touch_count_2d)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;
	case SYNAPTICS_RMI4_F1A:
		synaptics_rmi4_f1a_report(rmi4_data, fhandler);
		break;
	default:
		break;
	}

	return;
}

static void synaptics_rmi4_sensor_report(struct synaptics_rmi4_data *rmi4_data,
		bool report)
{
	int retval;
	unsigned char data[MAX_INTR_REGISTERS + 1];
	unsigned char *intr = &data[1];
	bool was_in_bl_mode;
	struct synaptics_rmi4_f01_device_status status;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;
    //unsigned char buf_val[2]={0};
	rmi = &(rmi4_data->rmi4_mod_info);
    /*
    if(rmi4_data->suspend && rmi4_data->enable_wakeup_gesture){
        if(cap_sel_status == 1) {            
            buf_val[0] = 0xa5;
            buf_val[1] = 0x00;
            retval = cap_i2c_write(rmi4_data, buf_val, 2);
            if(retval < 0){
                printk("[cap] write reg_addr=0x%02x, retval =%d\n", buf_val[0], retval);
            }
            msleep(5);
        }        
    }*/
    
	/*
	 * Get interrupt status information from F01 Data1 register to
	 * determine the source(s) that are flagging the interrupt.
	 */
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			data,
			rmi4_data->num_of_intr_regs + 1);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read interrupt status\n",
				__func__);
		return;
	}

	status.data[0] = data[0];
	if (status.status_code == STATUS_CRC_IN_PROGRESS) {
		retval = synaptics_rmi4_check_status(rmi4_data,
				&was_in_bl_mode);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to check status\n",
					__func__);
			return;
		}
		retval = synaptics_rmi4_reg_read(rmi4_data,
				rmi4_data->f01_data_base_addr,
				status.data,
				sizeof(status.data));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read device status\n",
					__func__);
			return;
		}
	}
	if (status.unconfigured && !status.flash_prog) {
		//pr_notice("%s: spontaneous reset detected\n", __func__);
		retval = synaptics_rmi4_reinit_device(rmi4_data);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to reinit device\n",
					__func__);
		}
	}

	if (!report)
		return;

	/*
	 * Traverse the function handler list and service the source(s)
	 * of the interrupt accordingly.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				if (fhandler->intr_mask &
						intr[fhandler->intr_reg_num]) {
					synaptics_rmi4_report_touch(rmi4_data,
							fhandler);
				}
			}
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (!exp_fhandler->insert &&
					!exp_fhandler->remove &&
					(exp_fhandler->exp_fn->attn != NULL))
				exp_fhandler->exp_fn->attn(rmi4_data, intr[0]);
		}
	}
	mutex_unlock(&exp_data.mutex);

	return;
}

//<ASUS_cap_sensor+>
int cap_i2c_Read(struct synaptics_rmi4_data *rmi4_data, char *writebuf,
	int writelen, char *readbuf, int readlen)
{
	int ret;
	int retry = 0, retrycount = 5;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	unsigned char addr = cap_addr; //<ASUS_focal+>
	mutex_lock(&i2c_rw_access); //<ASUS_focal+>
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		for (retry = 0; retry < retrycount; retry++) {
			ret = i2c_transfer(i2c->adapter, msgs, 2);
			if(ret < 0) {
				printk("[cap] %s: retry=%d, addr=0x%02x\n", __func__, retry, writebuf[0]);
			}else {
				break;
			}
			msleep(1);
		}
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		for (retry = 0; retry < retrycount; retry++) {
			ret = i2c_transfer(i2c->adapter, msgs, 1);
			if (ret >= 0)
				break;
			msleep(1);
		}
	}
    mutex_unlock(&i2c_rw_access);
	retry=0;
	return ret;
}

int cap_i2c_write(struct synaptics_rmi4_data *rmi4_data, unsigned char *writebuf, int writelen)
{
	int ret;
	int retry = 0, retrycount = 5;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	unsigned char addr = cap_addr;	//<ASUS_focal+>
	
    mutex_lock(&i2c_rw_access); //<ASUS_focal+>
    
    if (writelen > 0) {
        struct i2c_msg msg[] = {
            {
                .addr = addr,
                .flags = 0,
                .len = writelen,
                .buf = writebuf,
            },
        };
        
        for (retry = 0; retry < retrycount; retry++) {
            ret = i2c_transfer(i2c->adapter, msg, 1);
            if(ret < 0) {
                printk("[cap] %s: retry=%d, addr=0x%02x, val=0x%02x\n", __func__, retry, writebuf[0], writebuf[1]);
            }else {
                break;
            }
            msleep(1);
        }
    }
    mutex_unlock(&i2c_rw_access); //<ASUS_focal+>
	retry=0;
	return ret;
}

#ifdef ASUS_FACTORY_BUILD
static int pwrkey_mode=1 ;
static unsigned int caphome_keycode,capmenu_keycode;

static int pwrkeyMode_function(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	int old_val = pwrkey_mode;

	if (ret)
		return ret;

	if (pwrkey_mode > 0xf) {
		pwrkey_mode = old_val;
		return -EINVAL;
	}

	ret = param_set_int(val, kp);

	if (pwrkey_mode == 1) {
		caphome_keycode = KEY_HOME;
		capmenu_keycode = KEY_MENU;
		printk("[mid_powerbtn] Normal_Mode! \n");
		printk("[mid_powerbtn] CaphomeKeyCode = %d , CapmenuKeyCode = %d\n", caphome_keycode , capmenu_keycode);

	} else if (pwrkey_mode == 0) {
		caphome_keycode = KEY_B;
		capmenu_keycode = KEY_C;
		printk("[mid_powerbtn] Debug_Mode! \n");
		printk("[mid_powerbtn] CaphomeKeyCode = %d , CapmenuKeyCode = %d\n", caphome_keycode , capmenu_keycode);

	}
	return 0;
}

module_param_call(pwrkey_mode, pwrkeyMode_function, param_get_int, &pwrkey_mode, 0644);
#endif

static void cap_sensor_report(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char buf_val=0x00;
	unsigned char cap_read_addr = 0x00;
	//<ASUS_cap_delta_count+>
	unsigned char delta_count_addr = 0x00;
	unsigned char delta_count_addr_2 = 0x00;
	unsigned char delta_val = 0x00;
	unsigned char delta_val_2 = 0x00;
	unsigned char delta_back_LSB = 0x00;
	unsigned char delta_back_MSB = 0x00;
	unsigned char delta_home_LSB = 0x00;
	unsigned char delta_home_MSB = 0x00;
	unsigned char delta_menu_LSB = 0x00;
	unsigned char delta_menu_MSB = 0x00;
	//<ASUS_cap_delta_count->
	//<ASUS_led+>
	#ifdef ASUS_FACTORY_BUILD
	int ret;
	unsigned long onesec;
	onesec = msecs_to_jiffies(2000);
	#endif
	//<ASUS_led->
	
	cap_i2c_Read(rmi4_data, &cap_read_addr, 1, &buf_val, 1);
	printk("[cap] %s : buf_val=%d\n", __func__, buf_val);
	if(buf_val == 1) {	
		printk("[cap] %s : home key press buf_val=%d\n", __func__, buf_val);
		//<ASUS_cap_delta_count+>
		delta_count_addr = 0x10;
		cap_i2c_Read(rmi4_data, &delta_count_addr, 1, &delta_val, 1);
		delta_count_addr_2 = 0x11;
		cap_i2c_Read(rmi4_data, &delta_count_addr_2, 1, &delta_val_2, 1);
		printk("[cap]%s : addr=0x%02x%02x, CS1_delta=0x%02x%02x\n", __func__, delta_count_addr, delta_count_addr_2, delta_val, delta_val_2);
		//<ASUS_cap_delta_count->
        if(!touch_proximity_at_phone) { //<ASUS_proximity+>
#ifdef ASUS_FACTORY_BUILD
		//<ASUS_led+>
		set_button_backlight(true);
		ret = work_busy(&rmi4_data->led_delay_work.work);
		if(ret == WORK_BUSY_PENDING) {
			cancel_delayed_work_sync(&rmi4_data->led_delay_work);					
		}
		//<ASUS_led->
		if(pwrkey_mode == 1){
			input_report_key(rmi4_data->input_dev, KEY_HOME, 1);
		}else{
			input_report_key(rmi4_data->input_dev, KEY_B, 1);
		}
#endif
#ifndef ASUS_FACTORY_BUILD
		input_report_key(rmi4_data->input_dev, KEY_HOME, 1);
#endif
		input_sync(rmi4_data->input_dev);
        } //<ASUS_proximity+>
		btn_home=1;
	}else if(buf_val == 2) {		
		printk("[cap] %s : back key press buf_val=%d\n", __func__, buf_val);
		//<ASUS_cap_delta_count+>
		delta_count_addr = 0x12;
		cap_i2c_Read(rmi4_data, &delta_count_addr, 1, &delta_val, 1);
		delta_count_addr_2 = 0x13;
		cap_i2c_Read(rmi4_data, &delta_count_addr_2, 1, &delta_val_2, 1);
		printk("[cap]%s : addr=0x%02x%02x, CS2_delta=0x%02x%02x\n", __func__, delta_count_addr, delta_count_addr_2, delta_val, delta_val_2);
		//<ASUS_cap_delta_count->
        if(!touch_proximity_at_phone) { //<ASUS_proximity+>
		#ifdef ASUS_FACTORY_BUILD
		//<ASUS_led+>
		set_button_backlight(true);
		ret = work_busy(&rmi4_data->led_delay_work.work);
		if(ret == WORK_BUSY_PENDING) {
			cancel_delayed_work_sync(&rmi4_data->led_delay_work);					
		}
		//<ASUS_led->
		#endif
		input_report_key(rmi4_data->input_dev, KEY_BACK, 1);
		input_sync(rmi4_data->input_dev);
        } //<ASUS_proximity+>
		btn_back=1;
	}else if(buf_val == 4){
		printk("[cap] %s : menu key press buf_val=%d\n", __func__, buf_val);
		//<ASUS_cap_delta_count+>
		delta_count_addr = 0x14;
		cap_i2c_Read(rmi4_data, &delta_count_addr, 1, &delta_val, 1);
		delta_count_addr_2 = 0x15;
		cap_i2c_Read(rmi4_data, &delta_count_addr_2, 1, &delta_val_2, 1);
		printk("[cap]%s : addr=0x%02x%02x, CS3_delta=0x%02x%02x\n", __func__, delta_count_addr, delta_count_addr_2, delta_val, delta_val_2);
		//<ASUS_cap_delta_count->
        if(!touch_proximity_at_phone) { //<ASUS_proximity+>
#ifdef ASUS_FACTORY_BUILD
		//<ASUS_led+>
		set_button_backlight(true);
		ret = work_busy(&rmi4_data->led_delay_work.work);
		if(ret == WORK_BUSY_PENDING) {
			cancel_delayed_work_sync(&rmi4_data->led_delay_work);					
		}
		//<ASUS_led->
		if(pwrkey_mode == 1){
			input_report_key(rmi4_data->input_dev, KEY_MENU, 1);
		}else{
			input_report_key(rmi4_data->input_dev, KEY_C, 1);
		}
#endif
#ifndef ASUS_FACTORY_BUILD
		input_report_key(rmi4_data->input_dev, KEY_MENU, 1);
#endif
		input_sync(rmi4_data->input_dev);
        } //<ASUS_proximity+>
		btn_menu=1;
	}else if (buf_val == 6){ //Screen unpinning
        if(!touch_proximity_at_phone) { //<ASUS_proximity+>
		input_report_key(rmi4_data->input_dev, KEY_BACK, 1);
		input_report_key(rmi4_data->input_dev, KEY_MENU, 1);
		input_sync(rmi4_data->input_dev);
        } //<ASUS_proximity+>
		btn_menu=1;
		btn_back=1;
	} else if (buf_val == 5){
		//<ASUS_cap_delta_count+>
		delta_count_addr = 0x10;
		cap_i2c_Read(rmi4_data, &delta_count_addr, 1, &delta_home_MSB, 1);
		delta_count_addr_2 = 0x11;
		cap_i2c_Read(rmi4_data, &delta_count_addr_2, 1, &delta_home_LSB, 1);		
		delta_count_addr = 0x14;
		cap_i2c_Read(rmi4_data, &delta_count_addr, 1, &delta_menu_MSB, 1);
		delta_count_addr_2 = 0x15;
		cap_i2c_Read(rmi4_data, &delta_count_addr_2, 1, &delta_menu_LSB, 1);
		printk("[cap]%s : CS1_delta=0x%02x%02x, CS3_delta=0x%02x%02x\n", __func__
							, delta_home_MSB, delta_home_LSB
							, delta_menu_MSB, delta_menu_LSB);		
		//<ASUS_cap_delta_count->			
	} else if (buf_val == 7){
		//<ASUS_cap_delta_count+>
		delta_count_addr = 0x10;
		cap_i2c_Read(rmi4_data, &delta_count_addr, 1, &delta_home_MSB, 1);
		delta_count_addr_2 = 0x11;
		cap_i2c_Read(rmi4_data, &delta_count_addr_2, 1, &delta_home_LSB, 1);
		delta_count_addr = 0x12;
		cap_i2c_Read(rmi4_data, &delta_count_addr, 1, &delta_back_MSB, 1);
		delta_count_addr_2 = 0x13;
		cap_i2c_Read(rmi4_data, &delta_count_addr_2, 1, &delta_back_LSB, 1);
		delta_count_addr = 0x14;
		cap_i2c_Read(rmi4_data, &delta_count_addr, 1, &delta_menu_MSB, 1);
		delta_count_addr_2 = 0x15;
		cap_i2c_Read(rmi4_data, &delta_count_addr_2, 1, &delta_menu_LSB, 1);
		printk("[cap]%s : CS1_delta=0x%02x%02x, CS2_delta=0x%02x%02x, CS3_delta=0x%02x%02x\n", __func__
							, delta_home_MSB, delta_home_LSB
							, delta_back_MSB, delta_back_LSB
							, delta_menu_MSB, delta_menu_LSB);		
		//<ASUS_cap_delta_count->		
 	}else {
		if(btn_home == 1){
			printk("[cap] %s : home key release buf_val=%d\n", __func__, buf_val);
            if(!touch_proximity_at_phone) { //<ASUS_proximity+>
#ifdef ASUS_FACTORY_BUILD
			//<ASUS_led+>
			ret = queue_delayed_work(rmi4_data->led_wq, &rmi4_data->led_delay_work, onesec);
			printk("[cap] led ret(delay_wq) = %d\n", ret);
			//<ASUS_led->
			if(pwrkey_mode)
				input_report_key(rmi4_data->input_dev, KEY_HOME, 0);
			else
				input_report_key(rmi4_data->input_dev, KEY_B, 0);
#endif
#ifndef ASUS_FACTORY_BUILD
			input_report_key(rmi4_data->input_dev, KEY_HOME, 0);
#endif
			input_sync(rmi4_data->input_dev);
            } //<ASUS_proximity+>
			btn_home=0;
		}
		if(btn_back == 1){
			printk("[cap] %s : back key release buf_val=%d\n", __func__, buf_val);
            if(!touch_proximity_at_phone) { //<ASUS_proximity+>
			#ifdef ASUS_FACTORY_BUILD
			//<ASUS_led+>
			ret = queue_delayed_work(rmi4_data->led_wq, &rmi4_data->led_delay_work, onesec);
			printk("[cap] led ret(delay_wq) = %d\n", ret);
			//<ASUS_led->
			#endif
			input_report_key(rmi4_data->input_dev, KEY_BACK, 0);
			input_sync(rmi4_data->input_dev);
            } //<ASUS_proximity+>
			btn_back=0;
		}
		if(btn_menu == 1){
			printk("[cap] %s : menu key release buf_val=%d\n", __func__, buf_val);
            if(!touch_proximity_at_phone) { //<ASUS_proximity+>
			#ifdef ASUS_FACTORY_BUILD
			//<ASUS_led+>
			ret = queue_delayed_work(rmi4_data->led_wq, &rmi4_data->led_delay_work, onesec);
			printk("[cap] led ret(delay_wq) = %d\n", ret);
			//<ASUS_led->
			#endif
#ifdef ASUS_FACTORY_BUILD
			if(pwrkey_mode)
				input_report_key(rmi4_data->input_dev, KEY_MENU, 0);
			else
				input_report_key(rmi4_data->input_dev, KEY_C, 0);
#endif
#ifndef ASUS_FACTORY_BUILD
			input_report_key(rmi4_data->input_dev, KEY_MENU, 0);
#endif
			input_sync(rmi4_data->input_dev);
            } //<ASUS_proximity+>
			btn_menu=0;
		}
		if(btn_menu == 1 && btn_back == 1) {//screen unpinning
            if(!touch_proximity_at_phone) { //<ASUS_proximity+>
			input_report_key(rmi4_data->input_dev, KEY_BACK, 1);
			input_report_key(rmi4_data->input_dev, KEY_MENU, 1);
			input_sync(rmi4_data->input_dev);
            } //<ASUS_proximity+>
			btn_menu=0;
			btn_back=0;
		}
	}
}

//<ASUS_led+>
#ifdef ASUS_FACTORY_BUILD
static void led_delay_work_func(struct work_struct *led_delay_work)
{
	set_button_backlight(false);
	printk("[cap] set led lo\n");
}
#endif
//<ASUS_led->

static void cap_work_func(struct work_struct *cap_work)
{
	struct synaptics_rmi4_data *rmi4_data = container_of(cap_work, struct synaptics_rmi4_data, cap_work);
	int ret = 0;
	
	mutex_lock(&(rmi4_data->cap_mutex));
	
	if (ret == 0)
		cap_sensor_report(rmi4_data);
	
	mutex_unlock(&(rmi4_data->cap_mutex));
	enable_irq(CAP_INT_GET_PIN(CAP_INT_PIN));
	return;	
	
}

static irqreturn_t cap_sensor_interrupt(int irq, void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;

	printk("[cap] %s\n", __func__);
				
	if(queue_work(rmi4_data->cap_wq, &rmi4_data->cap_work))
	{
		disable_irq_nosync(CAP_INT_GET_PIN(CAP_INT_PIN));
	}
	
	return IRQ_HANDLED;
}
//<ASUS_cap_sensor->

//<ASUS_focal+>
static int fts_read_Touchdata(struct synaptics_rmi4_data *rmi4_data)
{
    int ret = -1;
    u8 buf[POINT_READ_BUF] = { 0 };    
    
    ret = cap_i2c_Read(rmi4_data, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		printk("%s read touchdata failed.\n", __func__);
		return ret;
	}

	buf_count_add++;
	memcpy( buf_touch_data+(((buf_count_add-1)%30)*POINT_READ_BUF), buf, sizeof(u8)*POINT_READ_BUF );
    
    return 0;
}

static void fts_report_value(struct synaptics_rmi4_data *rmi4_data)
{
    int i;
    u16 au16_x[FTS_MAX_POINTS] = {0};	/*x coordinate */
	u16 au16_y[FTS_MAX_POINTS] = {0};	/*y coordinate */
	u8 au8_touch_event[FTS_MAX_POINTS] = {0};	/*touch event:
					0 -- down; 1-- up; 2 -- contact */
	u8 pointid = FTS_MAX_ID;	
	u8 buf[POINT_READ_BUF] = { 0 };
	buf_count_neg++;
	
	memcpy( buf,buf_touch_data+(((buf_count_neg-1)%30)*POINT_READ_BUF), sizeof(u8)*POINT_READ_BUF );
    
    for (i = 0; i < FTS_MAX_POINTS; i++) {
		pointid = (buf[FTS_TOUCH_ID_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		
		au16_x[i] =
		    (s16) (buf[FTS_TOUCH_X_H_POS + FTS_ONE_TCH_LEN * i] & 0x0F) <<
		    8 | (s16) buf[FTS_TOUCH_X_L_POS + FTS_ONE_TCH_LEN * i];
		au16_y[i] =
		    (s16) (buf[FTS_TOUCH_Y_H_POS + FTS_ONE_TCH_LEN * i] & 0x0F) <<
		    8 | (s16) buf[FTS_TOUCH_Y_L_POS + FTS_ONE_TCH_LEN * i];
		au8_touch_event[i] =
		    buf[FTS_TOUCH_EVENT_POS + FTS_ONE_TCH_LEN * i] >> 6;

		/*if((au8_touch_event[i]==0 || au8_touch_event[i]==2))
			return;*/
	}
    
    //printk("%s [FTS] x=%d, y=%d\n", __func__, au16_x[0], au16_y[0]);
    //key down
    for (i = 0; i < FTS_MAX_POINTS; i++)
	{
		if (au8_touch_event[i] == FTS_TOUCH_DOWN)// || au8_touch_event[i] == FTS_TOUCH_CONTACT)
		{
			if((au16_x[i] == KEY_BACK_X_AREA) && (au16_y[i] == KEY_BACK_Y_AREA))
			{		
                if(!touch_proximity_at_phone) {
                    printk("[%s][FTS][cap] back press\n", __func__);
                    fts_btn_back = 1;
	                input_report_key(rmi4_data->input_dev, KEY_BACK, true);
	                input_sync(rmi4_data->input_dev);
                }
			}
			
			if((au16_x[i] == KEY_HOME_X_AREA) && (au16_y[i] == KEY_HOME_Y_AREA))
			{		
                if(!touch_proximity_at_phone) {
                    printk("[%s][FTS][cap] home press\n", __func__);
                    fts_btn_home = 1;
	                input_report_key(rmi4_data->input_dev, KEY_HOME, true);
	                input_sync(rmi4_data->input_dev);
                }
			}

			if((au16_x[i] == KEY_MENU_X_AREA) && (au16_y[i] == KEY_MENU_Y_AREA))
			{		
                if(!touch_proximity_at_phone) {
                    printk("[%s][FTS][cap] menu press\n", __func__);
                    fts_btn_menu = 1;
	                input_report_key(rmi4_data->input_dev, KEY_MENU, true);
	                input_sync(rmi4_data->input_dev);
                }
			}
        }
	}
    
    //key up
    for (i = 0; i < FTS_MAX_POINTS; i++)
	{
		if (au8_touch_event[i] == FTS_TOUCH_UP)
		{
			if((au16_x[i] == KEY_BACK_X_AREA) && (au16_y[i] == KEY_BACK_Y_AREA))
			{		
                if(!touch_proximity_at_phone) {
                    printk("[%s][FTS][cap] back relese\n", __func__);
                    fts_btn_back = 0;
	                input_report_key(rmi4_data->input_dev, KEY_BACK, false);
	                input_sync(rmi4_data->input_dev);
                }
			}
			
			if((au16_x[i] == KEY_HOME_X_AREA) && (au16_y[i] == KEY_HOME_Y_AREA))
			{		
                if(!touch_proximity_at_phone) {
                    printk("[%s][FTS][cap] home relese\n", __func__);
                    fts_btn_home = 0;
	                input_report_key(rmi4_data->input_dev, KEY_HOME, false);
	                input_sync(rmi4_data->input_dev);
                }
			}

			if((au16_x[i] == KEY_MENU_X_AREA) && (au16_y[i] == KEY_MENU_Y_AREA))
			{		
                if(!touch_proximity_at_phone) {
                    printk("[%s][FTS][cap] menu relese\n", __func__);
                    fts_btn_menu = 0;
	                input_report_key(rmi4_data->input_dev, KEY_MENU, false);
	                input_sync(rmi4_data->input_dev);
                }
			}
        }
	}
    
    return;
}

static void fts_touch_irq_work(struct work_struct *work)
{
    struct synaptics_rmi4_data *rmi4_data = container_of(work, struct synaptics_rmi4_data, fts_touch_event_work);
	int ret;
    ret = -1;
    
	ret = fts_read_Touchdata(rmi4_data);
	if (ret == 0)
		fts_report_value(rmi4_data);
	
	enable_irq(CAP_INT_GET_PIN(CAP_INT_PIN));
    return;
}

static irqreturn_t fts_ts_interrupt(int irq, void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;

    if(queue_work(rmi4_data->fts_workqueue, &rmi4_data->fts_touch_event_work))
	{
		disable_irq_nosync(CAP_INT_GET_PIN(CAP_INT_PIN));
	}

	return IRQ_HANDLED;
}

static void fts_glove_delay_work_func(struct work_struct *work)
{
    int retval;
    unsigned char cap_buf[2] = {0};
    struct delayed_work *delayed_work = container_of(work, struct delayed_work, work);
    struct synaptics_rmi4_data *rmi4_data =	container_of(delayed_work, struct synaptics_rmi4_data, fts_glove_delay_work);
    
    if(cap_sel_status == 1) {
        if(glove_mode == 1) {
            cap_buf[0] = 0xc0;
            cap_buf[1] = 0x01;
            retval = cap_i2c_write(rmi4_data, cap_buf, 2);
            if(retval < 0)
                printk("%s [fts] retval = %d glove mode write failure.\n", __func__, retval);
        }else {
            cap_buf[0] = 0xc0;
            cap_buf[1] = 0x00;
            retval = cap_i2c_write(rmi4_data, cap_buf, 2);
            if(retval < 0)
                printk("%s [fts] retval = %d glove mode write failure.\n", __func__, retval);
        }
    }
    printk("%s set glove parameter done\n", __func__);
}
//<ASUS_focal->

//<ASUS_cap_calibration+>
/*
static void calibration_work_function(struct work_struct *cap_cal_wq) {
	int ret;
    unsigned char buf_val=0x00;
	unsigned char cap_read_addr = 0x00;
	unsigned char buf_val_write[2]={0x06, 0x0F};
	struct synaptics_rmi4_data *rmi4_data = container_of((struct delayed_work *) cap_cal_wq, struct synaptics_rmi4_data, calibration_work);
	
    cap_i2c_Read(rmi4_data, &cap_read_addr, 1, &buf_val, 1);	
	if (glove_mode == 1) {
		if (rmi4_data->usb_status == 1) { //(glove, usb) = (1,1)
			buf_val_write[1] = 0x21;
		}else {							  //(glove, usb) = (1,0)
			buf_val_write[1] = 0x21;
		}
	}else {
		if (rmi4_data->usb_status == 1) { //(glove, usb) = (0,1)
			buf_val_write[1] = 0x20;
		} else {
			buf_val_write[1] = 0x18;	  //(glove, usb) = (0,0)
		}
	}
	
    if (buf_val == 0) {
        ret = cap_i2c_write(rmi4_data, buf_val_write, 2);
		printk("[cap] %s set sampling configuration write value=0x%x.\n", __func__, buf_val_write[1]);
    }
    queue_delayed_work(rmi4_data->cap_cal_wq, &rmi4_data->calibration_work, msecs_to_jiffies(60000));
}
*/
//<ASUS_cap_calibration->

//<ASUS_usb_cable_status+>
void synaptics_usb_detection(bool plugin)
{
	if (gb_rmi4_data == NULL) {
		printk("%s : global rmi4_data variable is null, skip \n", __func__);
		return;
	}
	
	if (plugin) {
		gb_rmi4_data->usb_status = 1; /*AC plug in*/
		printk("%s usb_status=%d\n", __func__, gb_rmi4_data->usb_status);
	} else {
		gb_rmi4_data->usb_status = 0; /*no AC */
		printk("%s usb_status=%d\n", __func__, gb_rmi4_data->usb_status);
	}
	queue_work(gb_rmi4_data->usb_wq, &gb_rmi4_data->usb_detect_work);
}
static void synaptics_cable_statue(struct work_struct *work)
{
	struct synaptics_rmi4_data *rmi4_data = container_of(work, struct synaptics_rmi4_data, usb_detect_work);
	
	int status = rmi4_data->usb_status;
	
	if (!fw_update_state) {
		if (status == 0 && cap_sel_status == 0) {	/*no AC */ //<ASUS_focal+>
			if (glove_mode == 1) {
				printk("%s set cap_sensor_glove_init\n", __func__);
				cap_sensor_glove_init(rmi4_data);
			} else {
				printk("%s set reg_init\n", __func__);
				cap_sensor_reg_init(rmi4_data);
			}
		} else if (status == 1 && cap_sel_status == 0) {	/*AC plug in*/	//<ASUS_focal+>
			if (glove_mode == 1) {
				printk("%s set cap_sensor_glove_usb_init\n", __func__);
				cap_sensor_glove_usb_mode_init(rmi4_data);
			} else {
				printk("%s set usb_init\n", __func__);
				cap_sensor_usb_init(rmi4_data);
			}
		}
	}
	
	return;
}
//<ASUS_usb_cable_status->

static irqreturn_t synaptics_rmi4_irq(int irq, void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (gpio_get_value(bdata->irq_gpio) != bdata->irq_on_state)
		goto exit;

	synaptics_rmi4_sensor_report(rmi4_data, true);

exit:
	return IRQ_HANDLED;
}

static int synaptics_rmi4_int_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval = 0;
	unsigned char ii;
	unsigned char zero = 0x00;
	unsigned char *intr_mask;
	unsigned short intr_addr;

	intr_mask = rmi4_data->intr_mask;

	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (intr_mask[ii] != 0x00) {
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			if (enable) {
				retval = synaptics_rmi4_reg_write(rmi4_data,
						intr_addr,
						&(intr_mask[ii]),
						sizeof(intr_mask[ii]));
				if (retval < 0)
					return retval;
			} else {
				retval = synaptics_rmi4_reg_write(rmi4_data,
						intr_addr,
						&zero,
						sizeof(zero));
				if (retval < 0)
					return retval;
			}
		}
	}

	return retval;
}

static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable, bool attn_only)
{
	int retval = 0;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (attn_only) {
		retval = synaptics_rmi4_int_enable(rmi4_data, enable);
		return retval;
	}

	if (enable) {
		if (rmi4_data->irq_enabled)
			return retval;

		retval = synaptics_rmi4_int_enable(rmi4_data, false);
		if (retval < 0)
			return retval;

		/* Process and clear interrupts */
		synaptics_rmi4_sensor_report(rmi4_data, false);

		retval = request_threaded_irq(rmi4_data->irq, NULL,
				synaptics_rmi4_irq, bdata->irq_flags,
				PLATFORM_DRIVER_NAME, rmi4_data);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to create irq thread\n",
					__func__);
			return retval;
		}

		retval = synaptics_rmi4_int_enable(rmi4_data, true);
		if (retval < 0)
			return retval;

		rmi4_data->irq_enabled = true;
	} else {
		if (rmi4_data->irq_enabled) {
			disable_irq(rmi4_data->irq);
			free_irq(rmi4_data->irq, rmi4_data);
			rmi4_data->irq_enabled = false;
		}
	}

	return retval;
}

static void synaptics_rmi4_set_intr_mask(struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	unsigned char ii;
	unsigned char intr_offset;

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
			ii < (fd->intr_src_count + intr_offset);
			ii++)
		fhandler->intr_mask |= 1 << ii;

	return;
}

static int synaptics_rmi4_f01_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->data = NULL;
	fhandler->extra = NULL;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	rmi4_data->f01_query_base_addr = fd->query_base_addr;
	rmi4_data->f01_ctrl_base_addr = fd->ctrl_base_addr;
	rmi4_data->f01_data_base_addr = fd->data_base_addr;
	rmi4_data->f01_cmd_base_addr = fd->cmd_base_addr;

	return 0;
}

static int synaptics_rmi4_f11_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	int temp;
	unsigned char offset;
	unsigned char fingers_supported;
	struct synaptics_rmi4_f11_extra_data *extra_data;
	struct synaptics_rmi4_f11_query_0_5 query_0_5;
	struct synaptics_rmi4_f11_query_7_8 query_7_8;
	struct synaptics_rmi4_f11_query_9 query_9;
	struct synaptics_rmi4_f11_query_12 query_12;
	struct synaptics_rmi4_f11_query_27 query_27;
	struct synaptics_rmi4_f11_ctrl_6_9 control_6_9;
	const struct synaptics_dsx_board_data *bdata =
				rmi4_data->hw_if->board_data;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->extra = kmalloc(sizeof(*extra_data), GFP_KERNEL);
	if (!fhandler->extra) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for fhandler->extra\n",
				__func__);
		return -ENOMEM;
	}
	extra_data = (struct synaptics_rmi4_f11_extra_data *)fhandler->extra;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base,
			query_0_5.data,
			sizeof(query_0_5.data));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	if (query_0_5.num_of_fingers <= 4)
		fhandler->num_of_data_points = query_0_5.num_of_fingers + 1;
	else if (query_0_5.num_of_fingers == 5)
		fhandler->num_of_data_points = 10;

	rmi4_data->num_of_fingers = fhandler->num_of_data_points;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + 6,
			control_6_9.data,
			sizeof(control_6_9.data));
	if (retval < 0)
		return retval;

	/* Maximum x and y */
	rmi4_data->sensor_max_x = control_6_9.sensor_max_x_pos_7_0 |
			(control_6_9.sensor_max_x_pos_11_8 << 8);
	rmi4_data->sensor_max_y = control_6_9.sensor_max_y_pos_7_0 |
			(control_6_9.sensor_max_y_pos_11_8 << 8);
	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);

	rmi4_data->max_touch_width = MAX_F11_TOUCH_WIDTH;

	if (bdata->swap_axes) {
		temp = rmi4_data->sensor_max_x;
		rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
		rmi4_data->sensor_max_y = temp;
	}

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	fhandler->data = NULL;

	offset = sizeof(query_0_5.data);

	/* query 6 */
	if (query_0_5.has_rel)
		offset += 1;

	/* queries 7 8 */
	if (query_0_5.has_gestures) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.query_base + offset,
				query_7_8.data,
				sizeof(query_7_8.data));
		if (retval < 0)
			return retval;

		offset += sizeof(query_7_8.data);
	}

	/* query 9 */
	if (query_0_5.has_query_9) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.query_base + offset,
				query_9.data,
				sizeof(query_9.data));
		if (retval < 0)
			return retval;

		offset += sizeof(query_9.data);
	}

	/* query 10 */
	if (query_0_5.has_gestures && query_7_8.has_touch_shapes)
		offset += 1;

	/* query 11 */
	if (query_0_5.has_query_11)
		offset += 1;

	/* query 12 */
	if (query_0_5.has_query_12) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.query_base + offset,
				query_12.data,
				sizeof(query_12.data));
		if (retval < 0)
			return retval;

		offset += sizeof(query_12.data);
	}

	/* query 13 */
	if (query_0_5.has_jitter_filter)
		offset += 1;

	/* query 14 */
	if (query_0_5.has_query_12 && query_12.has_general_information_2)
		offset += 1;

	/* queries 15 16 17 18 19 20 21 22 23 24 25 26*/
	if (query_0_5.has_query_12 && query_12.has_physical_properties)
		offset += 12;

	/* query 27 */
	if (query_0_5.has_query_27) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.query_base + offset,
				query_27.data,
				sizeof(query_27.data));
		if (retval < 0)
			return retval;

		rmi4_data->f11_wakeup_gesture = query_27.has_wakeup_gesture;
	}

	if (!rmi4_data->f11_wakeup_gesture)
		return retval;

	/* data 0 */
	fingers_supported = fhandler->num_of_data_points;
	offset = (fingers_supported + 3) / 4;

	/* data 1 2 3 4 5 */
	offset += 5 * fingers_supported;

	/* data 6 7 */
	if (query_0_5.has_rel)
		offset += 2 * fingers_supported;

	/* data 8 */
	if (query_0_5.has_gestures && query_7_8.data[0])
		offset += 1;

	/* data 9 */
	if (query_0_5.has_gestures && (query_7_8.data[0] || query_7_8.data[1]))
		offset += 1;

	/* data 10 */
	if (query_0_5.has_gestures &&
			(query_7_8.has_pinch || query_7_8.has_flick))
		offset += 1;

	/* data 11 12 */
	if (query_0_5.has_gestures &&
			(query_7_8.has_flick || query_7_8.has_rotate))
		offset += 2;

	/* data 13 */
	if (query_0_5.has_gestures && query_7_8.has_touch_shapes)
		offset += (fingers_supported + 3) / 4;

	/* data 14 15 */
	if (query_0_5.has_gestures &&
			(query_7_8.has_scroll_zones ||
			query_7_8.has_multi_finger_scroll ||
			query_7_8.has_chiral_scroll))
		offset += 2;

	/* data 16 17 */
	if (query_0_5.has_gestures &&
			(query_7_8.has_scroll_zones &&
			query_7_8.individual_scroll_zones))
		offset += 2;

	/* data 18 19 20 21 22 23 24 25 26 27 */
	if (query_0_5.has_query_9 && query_9.has_contact_geometry)
		offset += 10 * fingers_supported;

	/* data 28 */
	if (query_0_5.has_bending_correction ||
			query_0_5.has_large_object_suppression)
		offset += 1;

	/* data 29 30 31 */
	if (query_0_5.has_query_9 && query_9.has_pen_hover_discrimination)
		offset += 3;

	/* data 32 */
	if (query_0_5.has_query_12 &&
			query_12.has_small_object_detection_tuning)
		offset += 1;

	/* data 33 34 */
	if (query_0_5.has_query_27 && query_27.f11_query27_b0)
		offset += 2;

	/* data 35 */
	if (query_0_5.has_query_12 && query_12.has_8bit_w)
		offset += fingers_supported;

	/* data 36 */
	if (query_0_5.has_bending_correction)
		offset += 1;

	/* data 37 */
	if (query_0_5.has_query_27 && query_27.has_data_37)
		offset += 1;

	/* data 38 */
	if (query_0_5.has_query_27 && query_27.has_wakeup_gesture)
		extra_data->data38_offset = offset;

	return retval;
}

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28)
{
	int retval;
	static unsigned short ctrl_28_address;

	if (ctrl28)
		ctrl_28_address = ctrl28;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			ctrl_28_address,
			&rmi4_data->report_enable,
			sizeof(rmi4_data->report_enable));
	if (retval < 0)
		return retval;

	return retval;
}

static int synaptics_rmi4_f12_ctrl_sub(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_f12_query_5 *query_5,
		unsigned char ctrlreg, unsigned char subpacket)
{
	int retval;
	unsigned char cnt;
	unsigned char regnum;
	unsigned char bitnum;
	unsigned char q5_index;
	unsigned char q6_index;
	unsigned char offset;
	unsigned char max_ctrlreg;
	unsigned char *query_6;

	max_ctrlreg = (sizeof(query_5->data) - 1) * 8 - 1;

	if (ctrlreg > max_ctrlreg) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Control register number (%d) over limit\n",
				__func__, ctrlreg);
		return -EINVAL;
	}

	q5_index = ctrlreg / 8 + 1;
	bitnum = ctrlreg % 8;
	if ((query_5->data[q5_index] & (1 << bitnum)) == 0x00) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Control %d is not present\n",
				__func__, ctrlreg);
		return -EINVAL;
	}

	query_6 = kmalloc(query_5->size_of_query6, GFP_KERNEL);
	if (!query_6) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for query 6\n",
				__func__);
		return -ENOMEM;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base + 6,
			query_6,
			query_5->size_of_query6);
	if (retval < 0)
		goto exit;

	q6_index = 0;

	for (regnum = 0; regnum < ctrlreg; regnum++) {
		q5_index = regnum / 8 + 1;
		bitnum = regnum % 8;
		if ((query_5->data[q5_index] & (1 << bitnum)) == 0x00)
			continue;

		if (query_6[q6_index] == 0x00)
			q6_index += 3;
		else
			q6_index++;

		while (query_6[q6_index] & ~MASK_7BIT)
			q6_index++;

		q6_index++;
	}

	cnt = 0;
	q6_index++;
	offset = subpacket / 7;
	bitnum = subpacket % 7;

	do {
		if (cnt == offset) {
			if (query_6[q6_index + cnt] & (1 << bitnum))
				retval = 1;
			else
				retval = 0;
			goto exit;
		}
		cnt++;
	} while (query_6[q6_index + cnt - 1] & ~MASK_7BIT);

	retval = 0;

exit:
	kfree(query_6);

	return retval;
}

static int synaptics_rmi4_f12_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval = 0;
	int temp;
	unsigned char subpacket;
	unsigned char ctrl_23_size;
	unsigned char size_of_2d_data;
	unsigned char size_of_query8;
	unsigned char ctrl_8_offset;
	unsigned char ctrl_10_offset;
	unsigned char ctrl_15_offset; //<ASUS_COVER+>
	unsigned char ctrl_18_offset; //<ASUS_DTP+>
	unsigned char ctrl_20_offset;
	unsigned char ctrl_23_offset;
	unsigned char ctrl_28_offset;
	unsigned char ctrl_31_offset;
	unsigned char num_of_fingers;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_query_5 *query_5 = NULL;
	struct synaptics_rmi4_f12_query_8 *query_8 = NULL;
	struct synaptics_rmi4_f12_ctrl_8 *ctrl_8 = NULL;
	struct synaptics_rmi4_f12_ctrl_10 *ctrl_10 = NULL; //<ASUS_Glove+>
	struct synaptics_rmi4_f12_ctrl_15 *ctrl_15 = NULL; //<ASUS_COVER+>
	struct synaptics_rmi4_f12_ctrl_18 *ctrl_18 = NULL; //<ASUS_DTP+>
	struct synaptics_rmi4_f12_ctrl_23 *ctrl_23 = NULL;
	struct synaptics_rmi4_f12_ctrl_31 *ctrl_31 = NULL;
	const struct synaptics_dsx_board_data *bdata =
				rmi4_data->hw_if->board_data;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->extra = kmalloc(sizeof(*extra_data), GFP_KERNEL);
	if (!fhandler->extra) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for fhandler->extra\n",
				__func__);
		return -ENOMEM;
	}
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

	query_5 = kmalloc(sizeof(*query_5), GFP_KERNEL);
	if (!query_5) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for query_5\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}

	query_8 = kmalloc(sizeof(*query_8), GFP_KERNEL);
	if (!query_8) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for query_8\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}

	ctrl_8 = kmalloc(sizeof(*ctrl_8), GFP_KERNEL);
	if (!ctrl_8) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for ctrl_8\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}
	//<ASUS_Glove+>
	ctrl_10 = kmalloc(sizeof(*ctrl_10), GFP_KERNEL);
	if (!ctrl_10) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for ctrl_10\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}
	//<ASUS_Glove->

	//<ASUS_COVER+>
	ctrl_15 = kmalloc(sizeof(*ctrl_15), GFP_KERNEL);
	if (!ctrl_15) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for ctrl_15\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}
	//<ASUS_COVER->
	
	//<ASUS_DTP+>
	ctrl_18 = kmalloc(sizeof(*ctrl_18), GFP_KERNEL);
	if (!ctrl_18) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for ctrl_18\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}
	//<ASUS_DTP->
	
	ctrl_23 = kmalloc(sizeof(*ctrl_23), GFP_KERNEL);
	if (!ctrl_23) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for ctrl_23\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}

	ctrl_31 = kmalloc(sizeof(*ctrl_31), GFP_KERNEL);
	if (!ctrl_31) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for ctrl_31\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base + 5,
			query_5->data,
			sizeof(query_5->data));
	if (retval < 0)
		goto exit;

	ctrl_8_offset = query_5->ctrl0_is_present +
			query_5->ctrl1_is_present +
			query_5->ctrl2_is_present +
			query_5->ctrl3_is_present +
			query_5->ctrl4_is_present +
			query_5->ctrl5_is_present +
			query_5->ctrl6_is_present +
			query_5->ctrl7_is_present;

	ctrl_10_offset = ctrl_8_offset +
			query_5->ctrl8_is_present +
			query_5->ctrl9_is_present;
	
	ctrl_15_offset = ctrl_10_offset +
			query_5->ctrl10_is_present +
			query_5->ctrl11_is_present +
			query_5->ctrl12_is_present +
			query_5->ctrl13_is_present +
			query_5->ctrl14_is_present;
	
	ctrl_18_offset = ctrl_15_offset +
			query_5->ctrl15_is_present +
			query_5->ctrl16_is_present +
			query_5->ctrl17_is_present;
	
	ctrl_20_offset = ctrl_8_offset +
			query_5->ctrl8_is_present +
			query_5->ctrl9_is_present +
			query_5->ctrl10_is_present +
			query_5->ctrl11_is_present +
			query_5->ctrl12_is_present +
			query_5->ctrl13_is_present +
			query_5->ctrl14_is_present +
			query_5->ctrl15_is_present +
			query_5->ctrl16_is_present +
			query_5->ctrl17_is_present +
			query_5->ctrl18_is_present +
			query_5->ctrl19_is_present;

	ctrl_23_offset = ctrl_20_offset +
			query_5->ctrl20_is_present +
			query_5->ctrl21_is_present +
			query_5->ctrl22_is_present;

	ctrl_28_offset = ctrl_23_offset +
			query_5->ctrl23_is_present +
			query_5->ctrl24_is_present +
			query_5->ctrl25_is_present +
			query_5->ctrl26_is_present +
			query_5->ctrl27_is_present;

	ctrl_31_offset = ctrl_28_offset +
			query_5->ctrl28_is_present +
			query_5->ctrl29_is_present +
			query_5->ctrl30_is_present;

	ctrl_23_size = 2;
	for (subpacket = 2; subpacket <= 4; subpacket++) {
		retval = synaptics_rmi4_f12_ctrl_sub(rmi4_data,
				fhandler, query_5, 23, subpacket);
		if (retval == 1)
			ctrl_23_size++;
		else if (retval < 0)
			goto exit;
	}
	
	//<ASUS_Glove+>
	rmi4_data->f12_ctrl10_base_addr = fhandler->full_addr.ctrl_base + ctrl_10_offset;
	rmi4_data->f12_ctrl23_base_addr = fhandler->full_addr.ctrl_base + ctrl_23_offset;
	//<ASUS_Glove->
	//<ASUS_COVER+>
	rmi4_data->f12_ctrl15_base_addr = fhandler->full_addr.ctrl_base + ctrl_15_offset;
	//<ASUS_COVER->	
	//<ASUS_DTP+>
	rmi4_data->f12_ctrl18_base_addr = fhandler->full_addr.ctrl_base + ctrl_18_offset;
	printk("%s ctrl 18 addr=0x%02x\n", __func__, rmi4_data->f12_ctrl18_base_addr);
	//<ASUS_DTP->	
	
	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_23_offset,
			ctrl_23->data,
			ctrl_23_size);
	if (retval < 0)
		goto exit;

	/* Maximum number of fingers supported */
	fhandler->num_of_data_points = min(ctrl_23->max_reported_objects,
			(unsigned char)F12_FINGERS_TO_SUPPORT);

	num_of_fingers = fhandler->num_of_data_points;
	rmi4_data->num_of_fingers = num_of_fingers;

	rmi4_data->stylus_enable = false;//ctrl_23->stylus_enable;
	printk("%s stylus_enable=%d\n", __func__, rmi4_data->stylus_enable);
	rmi4_data->eraser_enable = ctrl_23->eraser_enable;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base + 7,
			&size_of_query8,
			sizeof(size_of_query8));
	if (retval < 0)
		goto exit;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base + 8,
			query_8->data,
			size_of_query8);
	if (retval < 0)
		goto exit;

	/* Determine the presence of the Data0 register */
	extra_data->data1_offset = query_8->data0_is_present;

	if ((size_of_query8 >= 3) && (query_8->data15_is_present)) {
		extra_data->data15_offset = query_8->data0_is_present +
				query_8->data1_is_present +
				query_8->data2_is_present +
				query_8->data3_is_present +
				query_8->data4_is_present +
				query_8->data5_is_present +
				query_8->data6_is_present +
				query_8->data7_is_present +
				query_8->data8_is_present +
				query_8->data9_is_present +
				query_8->data10_is_present +
				query_8->data11_is_present +
				query_8->data12_is_present +
				query_8->data13_is_present +
				query_8->data14_is_present;
		extra_data->data15_size = (num_of_fingers + 7) / 8;
	} else {
		extra_data->data15_size = 0;
	}

#ifdef REPORT_2D_PRESSURE
	if ((size_of_query8 >= 4) && (query_8->data23_is_present)) {
		extra_data->data23_offset = query_8->data0_is_present +
				query_8->data1_is_present +
				query_8->data2_is_present +
				query_8->data3_is_present +
				query_8->data4_is_present +
				query_8->data5_is_present +
				query_8->data6_is_present +
				query_8->data7_is_present +
				query_8->data8_is_present +
				query_8->data9_is_present +
				query_8->data10_is_present +
				query_8->data11_is_present +
				query_8->data12_is_present +
				query_8->data13_is_present +
				query_8->data14_is_present +
				query_8->data15_is_present +
				query_8->data16_is_present +
				query_8->data17_is_present +
				query_8->data18_is_present +
				query_8->data19_is_present +
				query_8->data20_is_present +
				query_8->data21_is_present +
				query_8->data22_is_present;
		extra_data->data23_size = num_of_fingers;
		rmi4_data->report_pressure = true;
	} else {
		extra_data->data23_size = 0;
		rmi4_data->report_pressure = false;
	}
#endif

	rmi4_data->report_enable = RPT_DEFAULT;
#ifdef REPORT_2D_Z
	rmi4_data->report_enable |= RPT_Z;
#endif
#ifdef REPORT_2D_W
	rmi4_data->report_enable |= (RPT_WX | RPT_WY);
#endif

	retval = synaptics_rmi4_f12_set_enables(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_28_offset);
	if (retval < 0)
		goto exit;

	if (query_5->ctrl8_is_present) {
		rmi4_data->wedge_sensor = false;

		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.ctrl_base + ctrl_8_offset,
				ctrl_8->data,
				sizeof(ctrl_8->data));
		if (retval < 0)
			goto exit;

		/* Maximum x and y */
		rmi4_data->sensor_max_x =
				((unsigned int)ctrl_8->max_x_coord_lsb << 0) |
				((unsigned int)ctrl_8->max_x_coord_msb << 8);
		rmi4_data->sensor_max_y =
				((unsigned int)ctrl_8->max_y_coord_lsb << 0) |
				((unsigned int)ctrl_8->max_y_coord_msb << 8);

		rmi4_data->max_touch_width = MAX_F12_TOUCH_WIDTH;
	} else {
		rmi4_data->wedge_sensor = true;

		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.ctrl_base + ctrl_31_offset,
				ctrl_31->data,
				sizeof(ctrl_31->data));
		if (retval < 0)
			goto exit;

		/* Maximum x and y */
		rmi4_data->sensor_max_x =
				((unsigned int)ctrl_31->max_x_coord_lsb << 0) |
				((unsigned int)ctrl_31->max_x_coord_msb << 8);
		rmi4_data->sensor_max_y =
				((unsigned int)ctrl_31->max_y_coord_lsb << 0) |
				((unsigned int)ctrl_31->max_y_coord_msb << 8);

		rmi4_data->max_touch_width = MAX_F12_TOUCH_WIDTH;
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);

	if (bdata->swap_axes) {
		temp = rmi4_data->sensor_max_x;
		rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
		rmi4_data->sensor_max_y = temp;
	}

	rmi4_data->f12_wakeup_gesture = query_5->ctrl27_is_present;
	if (rmi4_data->f12_wakeup_gesture) {
		extra_data->ctrl20_offset = ctrl_20_offset;
		extra_data->data4_offset = query_8->data0_is_present +
				query_8->data1_is_present +
				query_8->data2_is_present +
				query_8->data3_is_present;
	}

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	/* Allocate memory for finger data storage space */
	fhandler->data_size = num_of_fingers * size_of_2d_data;
	fhandler->data = kmalloc(fhandler->data_size, GFP_KERNEL);
	if (!fhandler->data) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for fhandler->data\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}

exit:
	kfree(query_5);
	kfree(query_8);
	kfree(ctrl_8);
	kfree(ctrl_15);
	kfree(ctrl_18);
	kfree(ctrl_23);
	kfree(ctrl_31);

	return retval;
}

static int synaptics_rmi4_f1a_alloc_mem(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	struct synaptics_rmi4_f1a_handle *f1a;

	f1a = kzalloc(sizeof(*f1a), GFP_KERNEL);
	if (!f1a) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for function handle\n",
				__func__);
		return -ENOMEM;
	}

	fhandler->data = (void *)f1a;
	fhandler->extra = NULL;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base,
			f1a->button_query.data,
			sizeof(f1a->button_query.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read query registers\n",
				__func__);
		return retval;
	}

	f1a->max_count = f1a->button_query.max_button_count + 1;

	f1a->button_control.txrx_map = kzalloc(f1a->max_count * 2, GFP_KERNEL);
	if (!f1a->button_control.txrx_map) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for tx rx mapping\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_bitmask_size = (f1a->max_count + 7) / 8;

	f1a->button_data_buffer = kcalloc(f1a->button_bitmask_size,
			sizeof(*(f1a->button_data_buffer)), GFP_KERNEL);
	if (!f1a->button_data_buffer) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for data buffer\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_map = kcalloc(f1a->max_count,
			sizeof(*(f1a->button_map)), GFP_KERNEL);
	if (!f1a->button_map) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for button map\n",
				__func__);
		return -ENOMEM;
	}

	return 0;
}

static int synaptics_rmi4_f1a_button_map(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char ii;
	unsigned char mapping_offset = 0;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	mapping_offset = f1a->button_query.has_general_control +
			f1a->button_query.has_interrupt_enable +
			f1a->button_query.has_multibutton_select;

	if (f1a->button_query.has_tx_rx_map) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.ctrl_base + mapping_offset,
				f1a->button_control.txrx_map,
				f1a->max_count * 2);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read tx rx mapping\n",
					__func__);
			return retval;
		}

		rmi4_data->button_txrx_mapping = f1a->button_control.txrx_map;
	}

	if (!bdata->cap_button_map) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: cap_button_map is NULL in board file\n",
				__func__);
		return -ENODEV;
	} else if (!bdata->cap_button_map->map) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Button map is missing in board file\n",
				__func__);
		return -ENODEV;
	} else {
		if (bdata->cap_button_map->nbuttons != f1a->max_count) {
			f1a->valid_button_count = min(f1a->max_count,
					bdata->cap_button_map->nbuttons);
		} else {
			f1a->valid_button_count = f1a->max_count;
		}

		for (ii = 0; ii < f1a->valid_button_count; ii++)
			f1a->button_map[ii] = bdata->cap_button_map->map[ii];
	}

	return 0;
}

static void synaptics_rmi4_f1a_kfree(struct synaptics_rmi4_fn *fhandler)
{
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

	if (f1a) {
		kfree(f1a->button_control.txrx_map);
		kfree(f1a->button_data_buffer);
		kfree(f1a->button_map);
		kfree(f1a);
		fhandler->data = NULL;
	}

	return;
}

static int synaptics_rmi4_f1a_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	retval = synaptics_rmi4_f1a_alloc_mem(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	retval = synaptics_rmi4_f1a_button_map(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	rmi4_data->button_0d_enabled = 1;

	return 0;

error_exit:
	synaptics_rmi4_f1a_kfree(fhandler);

	return retval;
}

static void synaptics_rmi4_empty_fn_list(struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_fn *fhandler_temp;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry_safe(fhandler,
				fhandler_temp,
				&rmi->support_fn_list,
				link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
				synaptics_rmi4_f1a_kfree(fhandler);
			} else {
				kfree(fhandler->extra);
				kfree(fhandler->data);
			}
			list_del(&fhandler->link);
			kfree(fhandler);
		}
	}
	INIT_LIST_HEAD(&rmi->support_fn_list);

	return;
}

static int synaptics_rmi4_check_status(struct synaptics_rmi4_data *rmi4_data,
		bool *was_in_bl_mode)
{
	int retval;
	int timeout = CHECK_STATUS_TIMEOUT_MS;
	struct synaptics_rmi4_f01_device_status status;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			status.data,
			sizeof(status.data));
	if (retval < 0)
		return retval;

	while (status.status_code == STATUS_CRC_IN_PROGRESS) {
		if (timeout > 0)
			msleep(20);
		else
			return -EINVAL;

		retval = synaptics_rmi4_reg_read(rmi4_data,
				rmi4_data->f01_data_base_addr,
				status.data,
				sizeof(status.data));
		if (retval < 0)
			return retval;

		timeout -= 20;
	}

	if (timeout != CHECK_STATUS_TIMEOUT_MS)
		*was_in_bl_mode = true;

	if (status.flash_prog == 1) {
		rmi4_data->flash_prog_mode = true;
		pr_notice("%s: In flash prog mode, status = 0x%02x\n",
				__func__,
				status.status_code);
	} else {
		rmi4_data->flash_prog_mode = false;
	}

	return 0;
}

static void synaptics_rmi4_set_configured(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to set configured\n",
				__func__);
		return;
	}

	rmi4_data->no_sleep_setting = device_ctrl | NO_SLEEP_ON;
	device_ctrl |= CONFIGURED | rmi4_data->no_sleep_setting;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to set configured\n",
				__func__);
	}

	return;
}

static int synaptics_rmi4_alloc_fh(struct synaptics_rmi4_fn **fhandler,
		struct synaptics_rmi4_fn_desc *rmi_fd, int page_number)
{
	*fhandler = kmalloc(sizeof(**fhandler), GFP_KERNEL);
	if (!(*fhandler))
		return -ENOMEM;

	(*fhandler)->full_addr.data_base =
			(rmi_fd->data_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.ctrl_base =
			(rmi_fd->ctrl_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.cmd_base =
			(rmi_fd->cmd_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.query_base =
			(rmi_fd->query_base_addr |
			(page_number << 8));

	return 0;
}

static int synaptics_rmi4_query_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char page_number;
	unsigned char intr_count;
	unsigned char *f01_query;
	unsigned char config_id[4];			//<ASUS_buildid+>
	unsigned short pdt_entry_addr;
	bool f01found;
	bool f35found;
	bool was_in_bl_mode;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

rescan_pdt:
	f01found = false;
	f35found = false;
	was_in_bl_mode = false;
	intr_count = 0;
	INIT_LIST_HEAD(&rmi->support_fn_list);

	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
				pdt_entry_addr -= PDT_ENTRY_SIZE) {
			pdt_entry_addr |= (page_number << 8);

			retval = synaptics_rmi4_reg_read(rmi4_data,
					pdt_entry_addr,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			pdt_entry_addr &= ~(MASK_8BIT << 8);

			fhandler = NULL;

			if (rmi_fd.fn_number == 0) {
				dev_dbg(rmi4_data->pdev->dev.parent,
						"%s: Reached end of PDT\n",
						__func__);
				break;
			}

			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: F%02x found (page %d)\n",
					__func__, rmi_fd.fn_number,
					page_number);

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				if (rmi_fd.intr_src_count == 0)
					break;

				f01found = true;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f01_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;

				retval = synaptics_rmi4_check_status(rmi4_data,
						&was_in_bl_mode);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to check status\n",
							__func__);
					return retval;
				}

				if (was_in_bl_mode) {
					kfree(fhandler);
					fhandler = NULL;
					goto rescan_pdt;
				}

				if (rmi4_data->flash_prog_mode)
					goto flash_prog_mode;

				break;
			case SYNAPTICS_RMI4_F11:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f11_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F12:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f12_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F1A:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f1a_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0) {
#ifdef IGNORE_FN_INIT_FAILURE
					kfree(fhandler);
					fhandler = NULL;
#else
					return retval;
#endif
				}
				break;
			case SYNAPTICS_RMI4_F35:
				f35found = true;
				break;
			}

			/* Accumulate the interrupt count */
			intr_count += rmi_fd.intr_src_count;

			if (fhandler && rmi_fd.intr_src_count) {
				list_add_tail(&fhandler->link,
						&rmi->support_fn_list);
			}
		}
	}

	if (!f01found) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to find F01\n",
				__func__);
		if (!f35found) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to find F35\n",
					__func__);
			return -EINVAL;
		} else {
			pr_notice("%s: In microbootloader mode\n",
					__func__);
			return 0;
		}
	}

flash_prog_mode:
	rmi4_data->num_of_intr_regs = (intr_count + 7) / 8;
	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Number of interrupt registers = %d\n",
			__func__, rmi4_data->num_of_intr_regs);

	f01_query = kmalloc(F01_STD_QUERY_LEN, GFP_KERNEL);
	if (!f01_query) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for f01_query\n",
				__func__);
		return -ENOMEM;
	}
	
	retval = synaptics_rmi4_reg_read(rmi4_data,
		rmi4_data->f01_query_base_addr,
		f01_query,
		F01_STD_QUERY_LEN);

	
	if (retval < 0) {
		kfree(f01_query);
		return retval;
	}

	/* RMI Version 4.0 currently supported */
	rmi->version_major = 4;
	rmi->version_minor = 0;

	rmi->manufacturer_id = f01_query[0];
	rmi->product_props = f01_query[1];
	rmi->product_info[0] = f01_query[2];
	rmi->product_info[1] = f01_query[3];
	retval = secure_memcpy(rmi->product_id_string,
			sizeof(rmi->product_id_string),
			&f01_query[11],
			F01_STD_QUERY_LEN - 11,
			PRODUCT_ID_SIZE);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to copy product ID string\n",
				__func__);
	}

	kfree(f01_query);

	if (rmi->manufacturer_id != 1) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Non-Synaptics device found, manufacturer ID = %d\n",
				__func__, rmi->manufacturer_id);
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_query_base_addr + F01_BUID_ID_OFFSET,
			rmi->build_id,
			sizeof(rmi->build_id));
	if (retval < 0)
		return retval;

	rmi4_data->firmware_id = (unsigned int)rmi->build_id[0] +
			(unsigned int)rmi->build_id[1] * 0x100 +
			(unsigned int)rmi->build_id[2] * 0x10000;

	//<ASUS_buildid+>
	retval = synaptics_rmi4_reg_read(rmi4_data,
			F34_FLASH_CTRL00,
			config_id,
			sizeof(config_id));

	if (retval < 0)
		return retval;

	rmi4_data->config_id = (((unsigned int)config_id[0]) << 24) +
			(((unsigned int)config_id[1]) << 16) +
			(((unsigned int)config_id[2]) << 8) +
			((unsigned int)config_id[3]);
	//<ASUS_buildid->
			
	memset(rmi4_data->intr_mask, 0x00, sizeof(rmi4_data->intr_mask));

	/*
	 * Map out the interrupt bit masks for the interrupt sources
	 * from the registered function handlers.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				rmi4_data->intr_mask[fhandler->intr_reg_num] |=
						fhandler->intr_mask;
			}
		}
	}

	if (rmi4_data->f11_wakeup_gesture || rmi4_data->f12_wakeup_gesture)
		rmi4_data->enable_wakeup_gesture = WAKEUP_GESTURE;
	else
		rmi4_data->enable_wakeup_gesture = false;

	synaptics_rmi4_set_configured(rmi4_data);

	return 0;
}

static int synaptics_rmi4_gpio_setup(int gpio, bool config, int dir, int state)
{
	int retval = 0;
	unsigned char buf[16];

	if (config) {
		snprintf(buf, PAGE_SIZE, "dsx_gpio_%u\n", gpio);

		retval = gpio_request(gpio, buf);
		if (retval) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, retval);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return retval;
}

static void synaptics_rmi4_set_params(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char ii;
	struct synaptics_rmi4_f1a_handle *f1a;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_X, 0,
			rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, 0,
			rmi4_data->sensor_max_y, 0, 0);
#ifdef REPORT_2D_W
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, 0,
			rmi4_data->max_touch_width, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MINOR, 0,
			rmi4_data->max_touch_width, 0, 0);
#endif

#ifdef REPORT_2D_PRESSURE
	if (rmi4_data->report_pressure) {
		input_set_abs_params(rmi4_data->input_dev,
				ABS_MT_PRESSURE, 0,
				MAX_F12_TOUCH_PRESSURE, 0, 0);
	}
#endif

#ifdef TYPE_B_PROTOCOL
#ifdef KERNEL_ABOVE_3_6
	input_mt_init_slots(rmi4_data->input_dev,
			rmi4_data->num_of_fingers, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(rmi4_data->input_dev,
			rmi4_data->num_of_fingers);
#endif
#endif

	f1a = NULL;
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				f1a = fhandler->data;
		}
	}

	if (f1a) {
		for (ii = 0; ii < f1a->valid_button_count; ii++) {
			set_bit(f1a->button_map[ii],
					rmi4_data->input_dev->keybit);
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, f1a->button_map[ii]);
		}
	}

	if (vir_button_map->nbuttons) {
		for (ii = 0; ii < vir_button_map->nbuttons; ii++) {
			set_bit(vir_button_map->map[ii * 5],
					rmi4_data->input_dev->keybit);
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, vir_button_map->map[ii * 5]);
		}
	}

	if (rmi4_data->f11_wakeup_gesture || rmi4_data->f12_wakeup_gesture) {
		set_bit(KEY_POWER, rmi4_data->input_dev->keybit);
		input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_POWER);
	}

	return;
}

static int synaptics_rmi4_set_input_dev(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	const struct synaptics_dsx_board_data *bdata =
				rmi4_data->hw_if->board_data;

	rmi4_data->input_dev = input_allocate_device();
	if (rmi4_data->input_dev == NULL) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to allocate input device\n",
				__func__);
		retval = -ENOMEM;
		goto err_input_device;
	}

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to query device\n",
				__func__);
		goto err_query_device;
	}

	rmi4_data->input_dev->name = PLATFORM_DRIVER_NAME;
	rmi4_data->input_dev->phys = INPUT_PHYS_NAME;
	rmi4_data->input_dev->id.product = SYNAPTICS_DSX_DRIVER_PRODUCT;
	rmi4_data->input_dev->id.version = SYNAPTICS_DSX_DRIVER_VERSION;
	rmi4_data->input_dev->dev.parent = rmi4_data->pdev->dev.parent;
	input_set_drvdata(rmi4_data->input_dev, rmi4_data);

	set_bit(EV_SYN, rmi4_data->input_dev->evbit);
	set_bit(EV_KEY, rmi4_data->input_dev->evbit);
	set_bit(EV_ABS, rmi4_data->input_dev->evbit);
	set_bit(BTN_TOUCH, rmi4_data->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, rmi4_data->input_dev->keybit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);
#endif

	//<ASUS_cap_sensor+>
	set_bit(KEY_MENU, rmi4_data->input_dev->keybit);
	set_bit(KEY_BACK, rmi4_data->input_dev->keybit);
	set_bit(KEY_HOME, rmi4_data->input_dev->keybit);
#ifdef ASUS_FACTORY_BUILD
	set_bit(KEY_B, rmi4_data->input_dev->keybit);
	set_bit(KEY_C, rmi4_data->input_dev->keybit);
#endif
	//<ASUS_cap_sensor->

#ifdef WAKEUP_GESTURE
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_DCLICK);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_SWIPE_UP); 
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_E); 
	//input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_M); 
	//input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_S); 
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_GESTURE_C);
	
	__set_bit(KEY_GESTURE_DCLICK, rmi4_data->input_dev->keybit);
	__set_bit(KEY_GESTURE_SWIPE_UP, rmi4_data->input_dev->keybit);
	__set_bit(KEY_GESTURE_E, rmi4_data->input_dev->keybit);
	//__set_bit(KEY_GESTURE_M, rmi4_data->input_dev->keybit);
	//__set_bit(KEY_GESTURE_L, rmi4_data->input_dev->keybit);
	__set_bit(KEY_GESTURE_W, rmi4_data->input_dev->keybit);
	__set_bit(KEY_GESTURE_S, rmi4_data->input_dev->keybit);
	__set_bit(KEY_GESTURE_V, rmi4_data->input_dev->keybit);
	__set_bit(KEY_GESTURE_Z, rmi4_data->input_dev->keybit);
	__set_bit(KEY_GESTURE_C, rmi4_data->input_dev->keybit);
#endif
	
	if (bdata->max_y_for_2d >= 0)
		rmi4_data->sensor_max_y = bdata->max_y_for_2d;

	synaptics_rmi4_set_params(rmi4_data);

	retval = input_register_device(rmi4_data->input_dev);
	if (retval) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to register input device\n",
				__func__);
		goto err_register_input;
	}
	
	/*if (!rmi4_data->stylus_enable)
		return 0;

	rmi4_data->stylus_dev = input_allocate_device();
	if (rmi4_data->stylus_dev == NULL) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to allocate stylus device\n",
				__func__);
		retval = -ENOMEM;
		goto err_stylus_device;
	}

	rmi4_data->stylus_dev->name = STYLUS_DRIVER_NAME;
	rmi4_data->stylus_dev->phys = STYLUS_PHYS_NAME;
	rmi4_data->stylus_dev->id.product = SYNAPTICS_DSX_DRIVER_PRODUCT;
	rmi4_data->stylus_dev->id.version = SYNAPTICS_DSX_DRIVER_VERSION;
	rmi4_data->stylus_dev->dev.parent = rmi4_data->pdev->dev.parent;
	input_set_drvdata(rmi4_data->stylus_dev, rmi4_data);

	set_bit(EV_KEY, rmi4_data->stylus_dev->evbit);
	set_bit(EV_ABS, rmi4_data->stylus_dev->evbit);
	set_bit(BTN_TOUCH, rmi4_data->stylus_dev->keybit);
	set_bit(BTN_TOOL_PEN, rmi4_data->stylus_dev->keybit);
	if (rmi4_data->eraser_enable)
		set_bit(BTN_TOOL_RUBBER, rmi4_data->stylus_dev->keybit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, rmi4_data->stylus_dev->propbit);
#endif

	input_set_abs_params(rmi4_data->stylus_dev, ABS_X, 0,
			rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->stylus_dev, ABS_Y, 0,
			rmi4_data->sensor_max_y, 0, 0);

	retval = input_register_device(rmi4_data->stylus_dev);
	if (retval) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to register stylus device\n",
				__func__);
		goto err_register_stylus;
	}*/

	return 0;

/*err_register_stylus:
	rmi4_data->stylus_dev = NULL;

err_stylus_device:
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;*/

err_register_input:
err_query_device:
	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_free_device(rmi4_data->input_dev);

err_input_device:
	return retval;
}

static int synaptics_dsx_pinctrl_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	rmi4_data->ts_pinctrl = devm_pinctrl_get((rmi4_data->pdev->dev.parent));
	if (IS_ERR_OR_NULL(rmi4_data->ts_pinctrl)) {
		retval = PTR_ERR(rmi4_data->ts_pinctrl);
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	rmi4_data->pinctrl_state_active
		= pinctrl_lookup_state(rmi4_data->ts_pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(rmi4_data->pinctrl_state_active)) {
		retval = PTR_ERR(rmi4_data->pinctrl_state_active);
		dev_err(rmi4_data->pdev->dev.parent,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	rmi4_data->pinctrl_state_suspend
		= pinctrl_lookup_state(rmi4_data->ts_pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(rmi4_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(rmi4_data->pinctrl_state_suspend);
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}
	
	rmi4_data->pinctrl_state_release
		= pinctrl_lookup_state(rmi4_data->ts_pinctrl, "pmx_ts_release");
	if (IS_ERR_OR_NULL(rmi4_data->pinctrl_state_release)) {
		retval = PTR_ERR(rmi4_data->pinctrl_state_release);
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(rmi4_data->ts_pinctrl);
err_pinctrl_get:
	rmi4_data->ts_pinctrl = NULL;
	return retval;
}

static int synaptics_rmi4_set_gpio(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	printk("%s start\n", __func__);
	
	retval = synaptics_rmi4_gpio_setup(
			bdata->irq_gpio,
			true, 0, 0);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to configure attention GPIO\n",
				__func__);
		goto err_gpio_irq;
	}

	/*if (bdata->power_gpio >= 0) {
		retval = synaptics_rmi4_gpio_setup(
				bdata->power_gpio,
				true, 1, !bdata->power_on_state);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to configure power GPIO\n",
					__func__);
			goto err_gpio_power;
		}
	}*/

	if (bdata->reset_gpio >= 0) {
		retval = synaptics_rmi4_gpio_setup(
				bdata->reset_gpio,
				true, 1, !bdata->reset_on_state);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to configure reset GPIO\n",
					__func__);
			goto err_gpio_reset;
		}
	}

	/*if (bdata->power_gpio >= 0) {
		gpio_set_value(bdata->power_gpio, bdata->power_on_state);
		msleep(bdata->power_delay_ms);
	}*/

	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		msleep(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
		msleep(bdata->reset_delay_ms);
	}
	if(cap_sel_status == 0) {
        //<ASUS_cap_sensor+>
        retval = gpio_request(CAP_INT_PIN, CAP_INT_PIN_NAME);
        if (retval < 0) {
                dev_err(rmi4_data->pdev->dev.parent,
                        "[cap] %s: Failed to configure cap-int GPIO\n",
                        __func__);
                return retval;
        }
        gpio_direction_input(CAP_INT_PIN);
        pr_info("[cap] %s: cap-sensor gpio_request int gpio Num: %d\n", __func__,CAP_INT_PIN);
        //<ASUS_cap_sensor->
    }else if(cap_sel_status == 1){
        //<ASUS_focal+>
        retval = gpio_request(CAP_INT_PIN, FTS_CAP_INT_PIN_NAME);
        if (retval < 0) {
                dev_err(rmi4_data->pdev->dev.parent,
                        "[FTS][cap] %s: Failed to configure cap-int GPIO\n",
                        __func__);
                return retval;
        }
        gpio_direction_input(CAP_INT_PIN);
        pr_info("[FTS][cap] %s: cap-sensor gpio_request int gpio Num: %d\n", __func__,CAP_INT_PIN);
        //<ASUS_focal->
	}
	
	printk("%s end\n", __func__);

	return 0;

err_gpio_reset:
	if (bdata->power_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->power_gpio, false, 0, 0);

/*err_gpio_power:
	synaptics_rmi4_gpio_setup(bdata->irq_gpio, false, 0, 0);*/

err_gpio_irq:
	return retval;
	
}

static int synaptics_rmi4_get_reg(struct synaptics_rmi4_data *rmi4_data,
		bool get)
{
	int retval;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}

	if ((bdata->pwr_reg_name != NULL) && (*bdata->pwr_reg_name != 0)) {
		rmi4_data->pwr_reg = regulator_get(rmi4_data->pdev->dev.parent,
				bdata->pwr_reg_name);
		if (IS_ERR(rmi4_data->pwr_reg)) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to get power regulator\n",
					__func__);
			retval = PTR_ERR(rmi4_data->pwr_reg);
			goto regulator_put;
		}
	}

	if ((bdata->bus_reg_name != NULL) && (*bdata->bus_reg_name != 0)) {
		rmi4_data->bus_reg = regulator_get(rmi4_data->pdev->dev.parent,
				bdata->bus_reg_name);
		if (IS_ERR(rmi4_data->bus_reg)) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to get bus pullup regulator\n",
					__func__);
			retval = PTR_ERR(rmi4_data->bus_reg);
			goto regulator_put;
		}
	}

	return 0;

regulator_put:
	if (rmi4_data->pwr_reg) {
		regulator_put(rmi4_data->pwr_reg);
		rmi4_data->pwr_reg = NULL;
	}

	if (rmi4_data->bus_reg) {
		regulator_put(rmi4_data->bus_reg);
		rmi4_data->bus_reg = NULL;
	}

	return retval;
}

static int synaptics_rmi4_enable_reg(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (!enable) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (rmi4_data->bus_reg) {
		retval = regulator_enable(rmi4_data->bus_reg);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to enable bus pullup regulator\n",
					__func__);
			goto exit;
		}
	}

	if (rmi4_data->pwr_reg) {
		retval = regulator_enable(rmi4_data->pwr_reg);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to enable power regulator\n",
					__func__);
			goto disable_bus_reg;
		}
		msleep(bdata->power_delay_ms);
	}

	return 0;

disable_pwr_reg:
	if (rmi4_data->pwr_reg)
		regulator_disable(rmi4_data->pwr_reg);

disable_bus_reg:
	if (rmi4_data->bus_reg)
		regulator_disable(rmi4_data->bus_reg);

exit:
	return retval;
}

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char ii;

	mutex_lock(&(rmi4_data->rmi4_report_mutex));

#ifdef TYPE_B_PROTOCOL
	for (ii = 0; ii < rmi4_data->num_of_fingers; ii++) {
		input_mt_slot(rmi4_data->input_dev, ii);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(rmi4_data->input_dev,
			BTN_TOUCH, 0);
	input_report_key(rmi4_data->input_dev,
			BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(rmi4_data->input_dev);
#endif
	input_sync(rmi4_data->input_dev);

	if (rmi4_data->stylus_enable) {
		input_report_key(rmi4_data->stylus_dev,
				BTN_TOUCH, 0);
		input_report_key(rmi4_data->stylus_dev,
				BTN_TOOL_PEN, 0);
		if (rmi4_data->eraser_enable) {
			input_report_key(rmi4_data->stylus_dev,
					BTN_TOOL_RUBBER, 0);
		}
		input_sync(rmi4_data->stylus_dev);
	}

	mutex_unlock(&(rmi4_data->rmi4_report_mutex));

	rmi4_data->fingers_on_2d = false;

	return 0;
}

static int synaptics_rmi4_sw_reset(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char command = 0x01;

    printk("%s start\n", __func__);
	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0)
		return retval;

	msleep(rmi4_data->hw_if->board_data->reset_delay_ms);

	if (rmi4_data->hw_if->ui_hw_init) {
		retval = rmi4_data->hw_if->ui_hw_init(rmi4_data);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static void synaptics_rmi4_rebuild_work(struct work_struct *work)
{
	int retval;
	unsigned char attr_count;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct delayed_work *delayed_work =
			container_of(work, struct delayed_work, work);
	struct synaptics_rmi4_data *rmi4_data =
			container_of(delayed_work, struct synaptics_rmi4_data,
			rb_work);

	mutex_lock(&(rmi4_data->rmi4_reset_mutex));

	mutex_lock(&exp_data.mutex);

	synaptics_rmi4_irq_enable(rmi4_data, false, false);

	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->remove != NULL)
				exp_fhandler->exp_fn->remove(rmi4_data);
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	synaptics_rmi4_free_fingers(rmi4_data);
	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;
	if (rmi4_data->stylus_enable) {
		input_unregister_device(rmi4_data->stylus_dev);
		rmi4_data->stylus_dev = NULL;
	}

	retval = synaptics_rmi4_sw_reset(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to issue reset command\n",
				__func__);
		goto exit;
	}

	retval = synaptics_rmi4_set_input_dev(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to set up input device\n",
				__func__);
		goto exit;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			goto exit;
		}
	}

	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->init != NULL)
				exp_fhandler->exp_fn->init(rmi4_data);
	}

	retval = 0;

exit:
	synaptics_rmi4_irq_enable(rmi4_data, true, false);

	mutex_unlock(&exp_data.mutex);

	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));

	return;
}

static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	mutex_lock(&(rmi4_data->rmi4_reset_mutex));

	synaptics_rmi4_free_fingers(rmi4_data);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F12) {
				synaptics_rmi4_f12_set_enables(rmi4_data, 0);
				break;
			}
		}
	}

	retval = synaptics_rmi4_int_enable(rmi4_data, true);
	if (retval < 0)
		goto exit;

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->reinit != NULL)
				exp_fhandler->exp_fn->reinit(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	synaptics_rmi4_set_configured(rmi4_data);

	retval = 0;

exit:
	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
	return retval;
}

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data,
		bool rebuild)
{
	int retval;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;

	if (rebuild) {
		queue_delayed_work(rmi4_data->rb_workqueue,
				&rmi4_data->rb_work,
				msecs_to_jiffies(REBUILD_WORK_DELAY_MS));
		return 0;
	}

	mutex_lock(&(rmi4_data->rmi4_reset_mutex));

	rmi4_data->touch_stopped = true;
	synaptics_rmi4_irq_enable(rmi4_data, false, false);

	retval = synaptics_rmi4_sw_reset(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to issue reset command\n",
				__func__);
		goto exit;
	}

	synaptics_rmi4_free_fingers(rmi4_data);

	synaptics_rmi4_empty_fn_list(rmi4_data);

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to query device\n",
				__func__);
		goto exit;
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->reset != NULL)
				exp_fhandler->exp_fn->reset(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	retval = 0;

exit:
	synaptics_rmi4_irq_enable(rmi4_data, true, false);

	rmi4_data->touch_stopped = false;
	
	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));

	return retval;
}

#ifdef FB_READY_RESET
static void synaptics_rmi4_reset_work(struct work_struct *work)
{
	int retval;
	unsigned int timeout;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(work, struct synaptics_rmi4_data,
			reset_work);

	timeout = FB_READY_TIMEOUT_S * 1000 / FB_READY_WAIT_MS + 1;

	while (!rmi4_data->fb_ready) {
		msleep(FB_READY_WAIT_MS);
		timeout--;
		if (timeout == 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Timed out waiting for FB ready\n",
					__func__);
			return;
		}
	}

	mutex_lock(&rmi4_data->rmi4_exp_init_mutex);

	retval = synaptics_rmi4_reset_device(rmi4_data, false);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to issue reset command\n",
				__func__);
	}

	mutex_unlock(&rmi4_data->rmi4_exp_init_mutex);

	return;
}
#endif

static void synaptics_rmi4_sleep_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval;
	unsigned char device_ctrl;
	unsigned char no_sleep_setting = rmi4_data->no_sleep_setting;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read device control\n",
				__func__);
		return;
	}

	device_ctrl = device_ctrl & ~MASK_3BIT;
	if (enable) {
		if ((dclick_mode == 1) || (swipe_mode == 1) || (gesture_mode & GESTURE_SWITCH)) {
			device_ctrl = NO_SLEEP_OFF;
		} else {
			device_ctrl = device_ctrl | NO_SLEEP_OFF | SENSOR_SLEEP;
		}
	}
	else
		device_ctrl = device_ctrl | no_sleep_setting | NORMAL_OPERATION | NO_SLEEP_ON;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write device control\n",
				__func__);
		return;
	}

	rmi4_data->sensor_sleep = enable;

	return;
}

//<ASUS_SDev+>
static ssize_t touch_switch_name(struct switch_dev *sdev, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = container_of(sdev, struct synaptics_rmi4_data, touch_sdev);

	return sprintf(buf, "%u_%x_0x%02x\n",rmi4_data->firmware_id, rmi4_data->config_id, rmi4_data->cap_fw_id);
}
//<ASUS_SDev->

static void synaptics_rmi4_exp_fn_work(struct work_struct *work)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler_temp;
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;

	mutex_lock(&rmi4_data->rmi4_exp_init_mutex);
	mutex_lock(&rmi4_data->rmi4_reset_mutex);
	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry_safe(exp_fhandler,
				exp_fhandler_temp,
				&exp_data.list,
				link) {
			if ((exp_fhandler->exp_fn->init != NULL) &&
					exp_fhandler->insert) {
				exp_fhandler->exp_fn->init(rmi4_data);
				exp_fhandler->insert = false;
			} else if ((exp_fhandler->exp_fn->remove != NULL) &&
					exp_fhandler->remove) {
				exp_fhandler->exp_fn->remove(rmi4_data);
				list_del(&exp_fhandler->link);
				kfree(exp_fhandler);
			}
		}
	}
	mutex_unlock(&exp_data.mutex);
	mutex_unlock(&rmi4_data->rmi4_reset_mutex);
	mutex_unlock(&rmi4_data->rmi4_exp_init_mutex);

	return;
}

void synaptics_rmi4_new_function(struct synaptics_rmi4_exp_fn *exp_fn,
		bool insert)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;

	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = true;
	}

	mutex_lock(&exp_data.mutex);
	if (insert) {
		exp_fhandler = kzalloc(sizeof(*exp_fhandler), GFP_KERNEL);
		if (!exp_fhandler) {
			pr_err("%s: Failed to alloc mem for expansion function\n",
					__func__);
			goto exit;
		}
		exp_fhandler->exp_fn = exp_fn;
		exp_fhandler->insert = true;
		exp_fhandler->remove = false;
		list_add_tail(&exp_fhandler->link, &exp_data.list);
	} else if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (exp_fhandler->exp_fn->fn_type == exp_fn->fn_type) {
				exp_fhandler->insert = false;
				exp_fhandler->remove = true;
				goto exit;
			}
		}
	}

exit:
	mutex_unlock(&exp_data.mutex);

	if (exp_data.queue_work) {
		queue_delayed_work(exp_data.workqueue,
				&exp_data.work,
				msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
	}

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_new_function);

static int synaptics_dsx_regulator_configure(struct synaptics_rmi4_data
			*rmi4_data)
{
	int retval;
	rmi4_data->regulator_vdd = regulator_get(rmi4_data->pdev->dev.parent,
			"vdd");
	if (IS_ERR(rmi4_data->regulator_vdd)) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to get regulator vdd\n",
				__func__);
		retval = PTR_ERR(rmi4_data->regulator_vdd);
		return retval;
	}
	rmi4_data->regulator_avdd = regulator_get(rmi4_data->pdev->dev.parent,
			"avdd");
	if (IS_ERR(rmi4_data->regulator_avdd)) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to get regulator avdd\n",
				__func__);
		retval = PTR_ERR(rmi4_data->regulator_avdd);
		regulator_put(rmi4_data->regulator_vdd);
		return retval;
	}

	return 0;
};

static int synaptics_dsx_regulator_enable(struct synaptics_rmi4_data
			*rmi4_data, bool on)
{
	int retval;

	if (on) {
		retval = regulator_enable(rmi4_data->regulator_vdd);
		if (retval) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to enable regulator vdd\n",
				__func__);
			return retval;
		}
		retval = regulator_enable(rmi4_data->regulator_avdd);
		if (retval) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to enable regulator avdd\n",
				__func__);
			regulator_disable(rmi4_data->regulator_vdd);
			return retval;
		}
		//msleep(rmi4_data->hw_if->board_data->power_delay_ms);
	} else {
		regulator_disable(rmi4_data->regulator_vdd);
		regulator_disable(rmi4_data->regulator_avdd);
	}

	return 0;
}

//<ASUS_cap_sensor+>
void cap_sensor_glove_init(struct synaptics_rmi4_data *rmi4_data)
{
	int ret,i;
	unsigned char buf_val[2]={0};
	unsigned char buf_reg[]={0x02, 0x03, 0x04, 0x06, 0x0b};
	unsigned char buf_reg_val[]={0x3e, 0x3e, 0x3e, 0x29, 0x0a};
	printk("[cap] cap sensor glove init start\n");	
	
	for(i=0; i<(sizeof(buf_reg)/sizeof(buf_reg[0])); i++) {
		buf_val[0] = buf_reg[i];
		buf_val[1] = buf_reg_val[i];
		ret = cap_i2c_write(rmi4_data, buf_val, 2);
		if(ret < 0){
			printk("[%s] write reg_addr=%d, ret =%d\n", __func__, buf_val[0], ret);
		}
		printk("[%s] cap sensor write buf[0] =%x, buf[1]=%x\n", __func__, buf_val[0], buf_val[1]);
	}
}
void cap_sensor_glove_usb_mode_init(struct synaptics_rmi4_data *rmi4_data)
{
	int ret,i;
	unsigned char buf_val[2]={0};
	unsigned char buf_reg[]={0x02, 0x03, 0x04, 0x06, 0x0b};
	unsigned char buf_reg_val[]={0x3e, 0x3e, 0x3e, 0x29, 0x0a};
	printk("[cap] cap sensor glove usb mode init start\n");	
	
	for(i=0; i<(sizeof(buf_reg)/sizeof(buf_reg[0])); i++) {
		buf_val[0] = buf_reg[i];
		buf_val[1] = buf_reg_val[i];
		ret = cap_i2c_write(rmi4_data, buf_val, 2);
		if(ret < 0){
			printk("[%s] write reg_addr=%d, ret =%d\n", __func__, buf_val[0], ret);
		}
		printk("[%s] cap sensor write buf[0] =%x, buf[1]=%x\n", __func__, buf_val[0], buf_val[1]);
	}
}
void cap_sensor_usb_init(struct synaptics_rmi4_data *rmi4_data)
{
	int ret,i;
	unsigned char buf_val[2]={0};
	unsigned char buf_reg[]={0x01, 0x02, 0x03, 0x04, 
							   0x05, 0x06, 0x07, 0x08, 
							   0x09, 0x0a, 0x0b};
	unsigned char buf_reg_val_ER[] = {0x00, 0x4b, 0x4b, 0x4b,
									   0x07, 0x20, 0x05, 0x01,
									   0x1e, 0x04, 0x1a};	//ER parameter
	unsigned char buf_reg_val_PR[]={0x00, 0x4b, 0x4b, 0x4b,
									   0x07, 0x20, 0x05, 0x01,
									   0x1e, 0x04, 0x1a}; //PR parameter
	
	printk("[cap] cap sensor usb init start\n");	
	
	if (asus_HW_ID == HW_ID_ER1) {
		for(i=0; i<(sizeof(buf_reg)/sizeof(buf_reg[0])); i++) {
			buf_val[0] = buf_reg[i];
			buf_val[1] = buf_reg_val_ER[i];
			ret = cap_i2c_write(rmi4_data, buf_val, 2);
			if(ret < 0){
				printk("[%s] write reg_addr=%d, ret =%d\n", __func__, buf_val[0], ret);
			}
			printk("[%s] cap sensor write buf[0] =%x, buf[1]=%x\n", __func__, buf_val[0], buf_val[1]);
		}		
	} else {	
		for(i=0; i<(sizeof(buf_reg)/sizeof(buf_reg[0])); i++) {
			buf_val[0] = buf_reg[i];
			buf_val[1] = buf_reg_val_PR[i];
			ret = cap_i2c_write(rmi4_data, buf_val, 2);
			if(ret < 0){
				printk("[%s] write reg_addr=%d, ret =%d\n", __func__, buf_val[0], ret);
			}
			printk("[%s] cap sensor write buf[0] =%x, buf[1]=%x\n", __func__, buf_val[0], buf_val[1]);
		}
	}
}
void cap_sensor_reg_init(struct synaptics_rmi4_data *rmi4_data)
{
	int ret,i;
	unsigned char buf_val[2]={0};
	unsigned char buf_reg[]={0x01, 0x02, 0x03, 0x04, 
							   0x05, 0x06, 0x07, 0x08, 
							   0x09, 0x0a, 0x0b};
	unsigned char buf_reg_val_ER[] = {0x00, 0x4b, 0x4b, 0x4b,
									   0x07, 0x20, 0x05, 0x01,
									   0x1e, 0x04, 0x1a};	//ER parameter
	unsigned char buf_reg_val_PR[]={0x00, 0x4b, 0x4b, 0x4b,
									   0x07, 0x20, 0x05, 0x01,
									   0x1e, 0x04, 0x1a}; //PR parameter
	
	printk("[cap] cap sensor init start\n");	
	
	if (asus_HW_ID == HW_ID_ER1) {
		for(i=0; i<(sizeof(buf_reg)/sizeof(buf_reg[0])); i++) {
			buf_val[0] = buf_reg[i];
			buf_val[1] = buf_reg_val_ER[i];
			ret = cap_i2c_write(rmi4_data, buf_val, 2);
			if(ret < 0){
				printk("[%s] write reg_addr=%d, ret =%d\n", __func__, buf_val[0], ret);
			}
			printk("[%s] cap sensor write buf[0] =%x, buf[1]=%x\n", __func__, buf_val[0], buf_val[1]);
		}
	} else {	
		for(i=0; i<(sizeof(buf_reg)/sizeof(buf_reg[0])); i++) {
			buf_val[0] = buf_reg[i];
			buf_val[1] = buf_reg_val_PR[i];
			ret = cap_i2c_write(rmi4_data, buf_val, 2);
			if(ret < 0){
				printk("[%s] write reg_addr=%d, ret =%d\n", __func__, buf_val[0], ret);
			}
			printk("[%s] cap sensor write buf[0] =%x, buf[1]=%x\n", __func__, buf_val[0], buf_val[1]);
		}
	}
}
//<ASUS_cap_sensor->

//<ASUS_focal+>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		printk("%s[FTS][CAP]:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
//#if 1
			printk("%s[FTS][CAP] Please use adb app upgrade\n", __func__);
//#else			
			/*char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			printk("%s[FTS][CAP]%s\n", __fuc__, upgrade_file_path);
			disable_irq(IRQ NUMBER???);
			
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			enable_irq(IRQ NUMBER???);
			if (ret < 0) {
				printk("%s[FTS]{CAP]:upgrade failed.\n", __func__);
				return ret;
			}*/
//#endif
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		
		//change to your i2c write function
		ret = FTS_I2c_Write(writebuf + 1, writelen);
		if (ret < 0) {
			printk("%s[FTS][CAP]:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		//change to your i2c write function
		ret = FTS_I2c_Write(writebuf + 1, writelen);
		if (ret < 0) {
			printk("%s[FTS][CAP]:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		printk("%s[FTS][CAP]: autoclb, not need to do this for FT3267\n", __func__);
		//not need to do this
		//fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		//change to your i2c write function
		ret = FTS_I2c_Write(writebuf + 1, writelen);
		if (ret < 0) {
			printk("[FTS][CAP]%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	

	return count;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	unsigned char buf[READ_BUF_SIZE];
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		//change to your i2c read function
		ret = FTS_I2c_Read(NULL, 0, &regaddr, regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		//change to your i2c read function
		ret = FTS_I2c_Read(NULL, 0, buf, readlen);
		if (ret < 0) {
			printk("%s[FTS][CAP]:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		//change to your i2c read function
		ret = FTS_I2c_Read(NULL, 0, buf, readlen);
		if (ret < 0) {
			printk("%s[FTS][CAP]:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	if (copy_to_user(buff, buf, num_read_chars)) {
		printk("%s[FTS][CAP]:copy to user error\n", __func__);
		return -EFAULT;
	}

	return num_read_chars;
}

static const struct file_operations fts_proc_fops = {
		.owner = THIS_MODULE,
		.read = fts_debug_read,
		.write = fts_debug_write,
		
};
#else
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static int fts_debug_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		printk("%s[FTS][CAP]:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

    printk("%s enter\n", __fuc__);
    printk("%s proc_operate_mode = %d\n", __fuc__, proc_operate_mode);
    
	switch (proc_operate_mode) {
	
	case PROC_UPGRADE:
		{
//#if 1	
			printk("%s[FTS][CAP] Please use adb app upgrade\n", __fuc__);
//#else		
			/*char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			printk("%s[FTS][CAP]%s\n",__fuc__, upgrade_file_path);
			disable_irq(IRQ number ???);
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			enable_irq(IRQ number ???);
			if (ret < 0) {
				printk("%s[FTS][CAP]:upgrade failed.\n", __func__);
				return ret;
			}*/
//#endif
		}		
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		//change to your i2c write function
		ret = FTS_I2c_Write(writebuf + 1, writelen);
		if (ret < 0) {
			printk("%s[FTS][CAP]:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		//change to your i2c write function
		ret = FTS_I2c_Write(writebuf + 1, writelen);
		if (ret < 0) {
			printk("%s[FTS][CAP]:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		printk("%s[FTS][CAP]: autoclb, not need to do this for FT3267\n", __func__);
		//not need to do this
		//fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		//change to your i2c write function
		ret = FTS_I2c_Write(writebuf + 1, writelen);
		if (ret < 0) {
			printk("%s[FTS][CAP]:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	

	return len;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static int fts_debug_read( char *page, char **start,
	off_t off, int count, int *eof, void *data )
{
	int ret = 0;
	unsigned char buf[READ_BUF_SIZE];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	
    printk("%s enter\n", __fuc__);
    printk("%s proc_operate_mode = %d\n", __fuc__, proc_operate_mode);
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		//change to your i2c read function
		ret = fts_read_reg(regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		//change to your i2c read function
		ret = FTS_I2c_Read(NULL, 0, buf, readlen);
		if (ret < 0) {
			printk("%s[FTS][CAP]:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		//change to your i2c read function
		ret = FTS_I2c_Read(NULL, 0, buf, readlen);
		if (ret < 0) {
			printk("%s[FTS][CAP]:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	memcpy(page, buf, num_read_chars);
	return num_read_chars;
}
#endif
/************************************************************************
* Name: fts_create_apk_debug_channel
* Brief:  create apk debug channel
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_apk_debug_channel(void)
{	
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
		fts_proc_entry = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);		
	#else
		fts_proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
	#endif
	if (NULL == fts_proc_entry) 
	{
		printk("%s[FTS][CAP] Couldn't create proc entry!\n",__func__);
		
		return -ENOMEM;
	} 
	else 
	{
		printk("%s[FTS][CAP] Create proc entry success!\n",__func__);
		
		#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
			fts_proc_entry->write_proc = fts_debug_write;
			fts_proc_entry->read_proc = fts_debug_read;
		#endif
	}
	return 0;
}
/************************************************************************
* Name: fts_release_apk_debug_channel
* Brief:  release apk debug channel
* Input: no
* Output: no
* Return: no
***********************************************************************/
void fts_release_apk_debug_channel(void)
{
	
	if (fts_proc_entry)
		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
			proc_remove(fts_proc_entry);
		#else
			remove_proc_entry(NULL, fts_proc_entry);
		#endif
}
//<ASUS_focal->

static int synaptics_rmi4_probe(struct platform_device *pdev)
{
	int retval, len;
    int i;//<ASUS_focal+>
	unsigned char attr_count;
	struct synaptics_rmi4_data *rmi4_data;
	const struct synaptics_dsx_hw_interface *hw_if;
	const struct synaptics_dsx_board_data *bdata;
	struct dentry *temp;
    u8 regaddr=0x9F,regvalue=0xff;//<ASUS_cap_sensor+>
	
	printk("%s: start\n", __func__);
	hw_if = pdev->dev.platform_data;
	if (!hw_if) {
		dev_err(&pdev->dev,
				"%s: No hardware interface found\n",
				__func__);
		return -EINVAL;
	}

	bdata = hw_if->board_data;
	if (!bdata) {
		dev_err(&pdev->dev,
				"%s: No board data found\n",
				__func__);
		return -EINVAL;
	}

	rmi4_data = kzalloc(sizeof(*rmi4_data), GFP_KERNEL);
	if (!rmi4_data) {
		dev_err(&pdev->dev,
				"%s: Failed to alloc mem for rmi4_data\n",
				__func__);
		return -ENOMEM;
	}

	rmi4_data->pdev = pdev;
	rmi4_data->current_page = MASK_8BIT;
	rmi4_data->hw_if = hw_if;
	rmi4_data->touch_stopped = false;
	rmi4_data->suspend = false;
	rmi4_data->irq_enabled = false;
	rmi4_data->fingers_on_2d = false;
	rmi4_data->usb_status = 0;
	
	rmi4_data->reset_device = synaptics_rmi4_reset_device;
	rmi4_data->irq_enable = synaptics_rmi4_irq_enable;
	rmi4_data->sleep_enable = synaptics_rmi4_sleep_enable;

	mutex_init(&(rmi4_data->rmi4_reset_mutex));
	mutex_init(&(rmi4_data->rmi4_report_mutex));
	mutex_init(&(rmi4_data->rmi4_io_ctrl_mutex));
	mutex_init(&(rmi4_data->rmi4_exp_init_mutex));
	mutex_init(&(rmi4_data->cap_mutex));
	mutex_init(&(rmi4_data->rmi4_fw_mutex));

	retval = synaptics_dsx_regulator_configure(rmi4_data);
	if (retval) {
		dev_err(&pdev->dev,
			"%s: regulator configuration failed\n", __func__);
		goto err_get_reg;
	}
	retval = synaptics_dsx_regulator_enable(rmi4_data, true);
	if (retval) {
		dev_err(&pdev->dev,
			"%s: regulator enable failed\n", __func__);
		goto err_regulator_enable;
	}
	
	platform_set_drvdata(pdev, rmi4_data);

	vir_button_map = bdata->vir_button_map;

	retval = synaptics_rmi4_get_reg(rmi4_data, true);
	if (retval < 0) {
		dev_err(&pdev->dev,
				"%s: Failed to get regulators\n",
				__func__);
		goto err_get_reg;
	}

	retval = synaptics_rmi4_enable_reg(rmi4_data, true);
	if (retval < 0) {
		dev_err(&pdev->dev,
				"%s: Failed to enable regulators\n",
				__func__);
		goto err_enable_reg;
	}

	retval = synaptics_dsx_pinctrl_init(rmi4_data);
		if (!retval && rmi4_data->ts_pinctrl) {
			/*
			* Pinctrl handle is optional. If pinctrl handle is found
			* let pins to be configured in active state. If not
			* found continue further without error.
			*/
			retval = pinctrl_select_state(rmi4_data->ts_pinctrl,
					rmi4_data->pinctrl_state_active);
			if (retval < 0) {
				dev_err(&pdev->dev,
					"%s: Failed to select %s pinstate %d\n",
					__func__, PINCTRL_STATE_ACTIVE, retval);
			}
		}
	
	printk("%s irq gpio =%d, reset gpio =%d\n", __func__,  bdata->irq_gpio, bdata->reset_gpio);
	retval = synaptics_rmi4_set_gpio(rmi4_data);
	if (retval < 0) {
		dev_err(&pdev->dev,
				"%s: Failed to set up GPIO's\n",
				__func__);
		goto err_set_gpio;
	}

	
	if (hw_if->ui_hw_init) {
		retval = hw_if->ui_hw_init(rmi4_data);
		if (retval < 0) {
			dev_err(&pdev->dev,
					"%s: Failed to initialize hardware interface\n",
					__func__);
			goto err_ui_hw_init;
		}
	}

	printk("[synaptics] %s fw name = %s\n", __func__, bdata->fw_name);
	if (bdata->fw_name) {
		len = strlen(bdata->fw_name);
		if (len > SYNA_FW_NAME_MAX_LEN - 1) {
			dev_err(&pdev->dev, "Invalid firmware name\n");
			goto err_set_input_dev;
		}

		strlcpy(rmi4_data->fw_name, bdata->fw_name, len + 1);
	}
	
	retval = synaptics_rmi4_set_input_dev(rmi4_data);
	if (retval < 0) {
		dev_err(&pdev->dev,
				"%s: Failed to set up input device\n",
				__func__);
		goto err_set_input_dev;
	}

    //<ASUS_cap_sensor+>
    //<ASUS_focal+>
    for(i=0; i<5; i++) {
        retval = cap_i2c_Read(rmi4_data, &regaddr, 1, &regvalue, 1);
        printk("%s [FTS][cap]retval = %d, regvalue =0x%02x\n", __func__, retval, regvalue);
    }
    if(retval > 0) {
        cap_sel_status = 1;
    }
    
    regaddr = 0x01;
    cap_addr = CAP_I2C_ADDR;
    for(i=0; i<5; i++) {
        retval = cap_i2c_Read(rmi4_data, &regaddr, 1, &regvalue, 1);
        printk("%s [cap]retval = %d, regvalue =0x%02x\n", __func__, retval, regvalue);
    }
    if(retval > 0) {
        cap_sel_status = 0;
    }
    
    if(cap_sel_status == 1) {
        init_i2c_read_func(FTS_I2c_Read);
        init_i2c_write_func(FTS_I2c_Write);
        cap_addr = FTS_CAP_I2C_ADDR;
        fts_a5_flag = 0;
        regaddr = 0xa6;
        retval = cap_i2c_Read(rmi4_data, &regaddr, 1, &regvalue, 1);
        if(retval < 0)
            printk("%s [FTS][cap]retval = %d, regvalue =0x%02x\n", __func__, retval, regvalue);
        printk("%s [fts] cap fw version = 0x%02x\n", __func__, regvalue);
        rmi4_data->cap_fw_id = regvalue;
    }else if(cap_sel_status == 0){
        cap_addr = CAP_I2C_ADDR;
        rmi4_data->cap_fw_id = 0x88;
    }
    printk("%s [cap] cap_sel_status = %d, cap_addr = 0x%02x\n", __func__, cap_sel_status, cap_addr);
    //<ASUS_focal->
    //<ASUS_cap_sensor->
    
#ifdef CONFIG_FB
	rmi4_data->fb_notifier.notifier_call = synaptics_rmi4_fb_notifier_cb;
	retval = fb_register_client(&rmi4_data->fb_notifier);
	if (retval < 0) {
		dev_err(&pdev->dev,
				"%s: Failed to register fb notifier client\n",
				__func__);
	}
#endif

#ifdef USE_EARLYSUSPEND
	rmi4_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	rmi4_data->early_suspend.suspend = synaptics_rmi4_early_suspend;
	rmi4_data->early_suspend.resume = synaptics_rmi4_late_resume;
	register_early_suspend(&rmi4_data->early_suspend);
#endif

	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = true;
	}
	
	rmi4_data->irq = gpio_to_irq(bdata->irq_gpio);

	retval = synaptics_rmi4_irq_enable(rmi4_data, true, false);
	if (retval < 0) {
		dev_err(&pdev->dev,
				"%s: Failed to enable attention interrupt\n",
				__func__);
		goto err_enable_irq;
	}
	
	//<ASUS_led+>
	#ifdef ASUS_FACTORY_BUILD
	printk("cap led start");
	rmi4_data->led_wq = create_singlethread_workqueue("led_wq");
	if (!rmi4_data->led_wq) 
	{
		printk(KERN_ERR "\n[synaptics] %s: create led workqueue failed\n", __func__);		
		goto err_cap_sensor;
	}
	INIT_DELAYED_WORK(&rmi4_data->led_delay_work, led_delay_work_func);
	printk("\n[synaptics] Create delay workqueue success\n");
	printk("cap led end");
	#endif
	//<ASUS_led->

	//<ASUS_usb_cable_status+>
	printk("[synaptics]:create usb cable detect workqueue\n");
	rmi4_data->usb_wq = create_singlethread_workqueue("synaptics_usb_wq");
	if (!rmi4_data->usb_wq) 
	{
		printk(KERN_ERR "\n [synaptics] %s: create usb cable detect workqueue failed\n", __func__);		
		goto err_cap_sensor;
	}
	INIT_WORK(&rmi4_data->usb_detect_work, synaptics_cable_statue);
	printk("[synaptics]:create usb cable detect workqueue success\n");
	//<ASUS_usb_cable_status->
	
	//<ASUS_cap_sensor+>
    if(cap_sel_status == 0) {
        printk("cap sensor start\n");
        rmi4_data->cap_wq = create_singlethread_workqueue("cap_wq");
        if (!rmi4_data->cap_wq) 
        {
            printk(KERN_ERR "\n [cap] %s: create cap_sensor workqueue failed\n", __func__);
            goto err_cap_sensor;
        }
        INIT_WORK(&rmi4_data->cap_work, cap_work_func);

        printk("\n [cap] %s: Create cap_sensor workqueue success\n", __func__);
        
        retval = request_threaded_irq(CAP_INT_GET_PIN(CAP_INT_PIN), NULL, cap_sensor_interrupt,
            IRQF_TRIGGER_FALLING | IRQF_ONESHOT, PLATFORM_DRIVER_NAME,
            rmi4_data);
        if (retval < 0) {
            dev_err(&pdev->dev, "[cap]: cap-sensor request irq failed\n");
            goto err_cap_sensor;
        }
        printk("\n [cap] %s: request cap_sensor irq success\n", __func__);	
        
        cap_sensor_reg_init(rmi4_data);
        
        //<ASUS_cap_calibration+>
        /*printk("cap sensor calibration start\n");
        rmi4_data->cap_cal_wq = create_singlethread_workqueue("cap_cal_wq");
        if (!rmi4_data->cap_cal_wq) 
        {
            printk(KERN_ERR "\n [cap] %s: create cap_sensor calibration workqueue failed\n", __func__);		
            goto err_cap_sensor;
        }
        INIT_DELAYED_WORK(&rmi4_data->calibration_work, calibration_work_function);*/
        //<ASUS_cap_calibration->
	}else if(cap_sel_status == 1){
        //<ASUS_focal+>
        INIT_WORK(&rmi4_data->fts_touch_event_work, fts_touch_irq_work);
        rmi4_data->fts_workqueue = create_workqueue(FTS_WORKQUEUE_NAME);
        if (!rmi4_data->fts_workqueue)
        {
            printk(KERN_ERR "\n [FTS] %s: create fts cap workqueue failed\n", __func__);
            goto err_cap_sensor;
        }
        
        retval = request_threaded_irq(CAP_INT_GET_PIN(CAP_INT_PIN), NULL, fts_ts_interrupt,
                    IRQF_ONESHOT | IRQF_TRIGGER_FALLING, PLATFORM_DRIVER_NAME, rmi4_data);
        if (retval < 0) {
            dev_err(&pdev->dev, "[FTS]: cap-sensor request irq failed\n");
            goto err_cap_sensor;
        }
        fts_create_apk_debug_channel();
        rmi4_data->fts_glove_wq = create_singlethread_workqueue("fts_glove_wq");
        if (!rmi4_data->fts_glove_wq) 
        {
            printk(KERN_ERR "\n%s [FTS]: create fts glove workqueue failed\n", __func__);		
            goto err_cap_sensor;
        }
        INIT_DELAYED_WORK(&rmi4_data->fts_glove_delay_work, fts_glove_delay_work_func);
    }
err_cap_sensor:
    //<ASUS_focal->
	//<ASUS_cap_sensor->
	
	if (vir_button_map->nbuttons) {
		rmi4_data->board_prop_dir = kobject_create_and_add(
				"board_properties", NULL);
		if (!rmi4_data->board_prop_dir) {
			dev_err(&pdev->dev,
					"%s: Failed to create board_properties directory\n",
					__func__);
			goto err_virtual_buttons;
		} else {
			retval = sysfs_create_file(rmi4_data->board_prop_dir,
					&virtual_key_map_attr.attr);
			if (retval < 0) {
				dev_err(&pdev->dev,
						"%s: Failed to create virtual key map file\n",
						__func__);
				goto err_virtual_buttons;
			}
		}
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&pdev->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			goto err_sysfs;
		}
	}

	rmi4_data->rb_workqueue =
			create_singlethread_workqueue("dsx_rebuild_workqueue");
	INIT_DELAYED_WORK(&rmi4_data->rb_work, synaptics_rmi4_rebuild_work);

	exp_data.workqueue = create_singlethread_workqueue("dsx_exp_workqueue");
	INIT_DELAYED_WORK(&exp_data.work, synaptics_rmi4_exp_fn_work);
	exp_data.rmi4_data = rmi4_data;
	exp_data.queue_work = true;
	queue_delayed_work(exp_data.workqueue,
			&exp_data.work,
			0);
	
	rmi4_data->dir = debugfs_create_dir(DEBUGFS_DIR_NAME, NULL);
	if (rmi4_data->dir == NULL || IS_ERR(rmi4_data->dir)) {
		retval = rmi4_data->dir ? PTR_ERR(rmi4_data->dir) : -EIO;
		dev_err(&pdev->dev,
			"%s: Failed to create debugfs directory, rc = %d\n",
			__func__, retval);
		goto err_create_debugfs_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, rmi4_data->dir,
					rmi4_data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		retval = temp ? PTR_ERR(temp) : -EIO;
		dev_err(&pdev->dev,
			"%s: Failed to create suspend debugfs file, rc = %d\n",
			__func__, retval);
		goto err_create_debugfs_file;
	}

#ifdef FB_READY_RESET
	rmi4_data->reset_workqueue =
			create_singlethread_workqueue("dsx_reset_workqueue");
	INIT_WORK(&rmi4_data->reset_work, synaptics_rmi4_reset_work);
	queue_work(rmi4_data->reset_workqueue, &rmi4_data->reset_work);
#endif

//<ASUS_Proximity+>
#ifdef ASUS_TOUCH_PROXIMITY_NODE
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
	tp_proximity_proc = proc_create(PROXIMITY_NAME, 0664, NULL, &tp_proximity_proc_fops);
#else
	tp_proximity_proc = create_proc_entry(PROXIMITY_NAME, 0664, NULL);
#endif
	if (!tp_proximity_proc) {
		dev_err(&pdev->dev,
				"%s: Failed to create proc proximity node\n",
				__func__);
		goto err_sysfs;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
	} else {
		tp_proximity_proc->write_proc = tp_proximity_proc_write;
		tp_proximity_proc->read_proc = tp_proximity_proc_read;
		tp_proximity_proc->data = NULL;
#endif
	}
#endif
//<ASUS_Proximity->

	//<ASUS_SDev+>
	rmi4_data->touch_sdev.name = "touch";
	rmi4_data->touch_sdev.print_name = touch_switch_name;
	if(switch_dev_register(&rmi4_data->touch_sdev) < 0){
		printk("switch_dev_register failed!\n");
	}
	//<ASUS_SDev->
	
	//ASUS_FACTORY+
	#if defined(ASUS_FACTORY_BUILD)
	if(create_asusproc_tp_status_entry())
		printk("[%s] : ERROR to create tp proc entry\n",__func__);
	#endif
	//ASUS_FACTORY-
	
	//add global variable gb_rmi4_data for checking usb_cable_status
	gb_rmi4_data = rmi4_data;

	//queue_delayed_work(rmi4_data->cap_cal_wq, &rmi4_data->calibration_work, msecs_to_jiffies(60000));
	printk("%s: end\n", __func__);
	return retval;

err_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

err_create_debugfs_file:
	debugfs_remove_recursive(rmi4_data->dir);

err_create_debugfs_dir:
	cancel_delayed_work_sync(&exp_data.work);
	if (exp_data.workqueue != NULL) {
		flush_workqueue(exp_data.workqueue);
		destroy_workqueue(exp_data.workqueue);
	}
	synaptics_rmi4_irq_enable(rmi4_data, false, false);
	free_irq(rmi4_data->irq, rmi4_data);
	
err_virtual_buttons:
	if (rmi4_data->board_prop_dir) {
		sysfs_remove_file(rmi4_data->board_prop_dir,
				&virtual_key_map_attr.attr);
		kobject_put(rmi4_data->board_prop_dir);
	}

	synaptics_rmi4_irq_enable(rmi4_data, false, false);

err_enable_irq:
#ifdef CONFIG_FB
	fb_unregister_client(&rmi4_data->fb_notifier);
#endif

#ifdef USE_EARLYSUSPEND
	unregister_early_suspend(&rmi4_data->early_suspend);
#endif

	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;
	if (rmi4_data->stylus_enable) {
		input_unregister_device(rmi4_data->stylus_dev);
		rmi4_data->stylus_dev = NULL;
	}

err_set_input_dev:
	synaptics_rmi4_gpio_setup(bdata->irq_gpio, false, 0, 0);

	if (bdata->reset_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->reset_gpio, false, 0, 0);

	if (bdata->power_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->power_gpio, false, 0, 0);

err_ui_hw_init:
err_set_gpio:
	synaptics_rmi4_enable_reg(rmi4_data, false);
	if (rmi4_data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(rmi4_data->pinctrl_state_release)) {
			devm_pinctrl_put(rmi4_data->ts_pinctrl);
			rmi4_data->ts_pinctrl = NULL;
		} else {
			retval = pinctrl_select_state(
				rmi4_data->ts_pinctrl,
				rmi4_data->pinctrl_state_release);
			if (retval)
				dev_err(&pdev->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
		}
	}

err_enable_reg:
	synaptics_rmi4_get_reg(rmi4_data, false);

err_regulator_enable:
	regulator_put(rmi4_data->regulator_vdd);
	regulator_put(rmi4_data->regulator_avdd);
	
err_get_reg:
	kfree(rmi4_data);

	return retval;
}

static int synaptics_rmi4_remove(struct platform_device *pdev)
{
	int err;
	unsigned char attr_count;
	struct synaptics_rmi4_data *rmi4_data = platform_get_drvdata(pdev);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

#ifdef FB_READY_RESET
	cancel_work_sync(&rmi4_data->reset_work);
	flush_workqueue(rmi4_data->reset_workqueue);
	destroy_workqueue(rmi4_data->reset_workqueue);
#endif

	cancel_delayed_work_sync(&exp_data.work);
	flush_workqueue(exp_data.workqueue);
	destroy_workqueue(exp_data.workqueue);

	cancel_delayed_work_sync(&rmi4_data->rb_work);
	flush_workqueue(rmi4_data->rb_workqueue);
	destroy_workqueue(rmi4_data->rb_workqueue);
	
	//<ASUS_led+>
	#ifdef ASUS_FACTORY_BUILD
	cancel_delayed_work_sync(&rmi4_data->led_delay_work);
	flush_workqueue(rmi4_data->led_wq);
	destroy_workqueue(rmi4_data->led_wq);
	#endif
	//<ASUS_led->
	
	//<ASUS_usb_cable_status+>
	flush_workqueue(rmi4_data->usb_wq);
	destroy_workqueue(rmi4_data->usb_wq);
	//<ASUS_usb_cable_status->
	
	//<ASUS_cap_sensor+>
	flush_workqueue(rmi4_data->cap_wq);
	destroy_workqueue(rmi4_data->cap_wq);
	//<ASUS_cap_sensor->
    
    //<ASUS_focal+>
    flush_workqueue(rmi4_data->fts_workqueue);
	destroy_workqueue(rmi4_data->fts_workqueue);
    fts_release_apk_debug_channel();
    
    flush_workqueue(rmi4_data->fts_glove_wq);
	destroy_workqueue(rmi4_data->fts_glove_wq);
    //<ASUS_focal->

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	if (rmi4_data->board_prop_dir) {
		sysfs_remove_file(rmi4_data->board_prop_dir,
				&virtual_key_map_attr.attr);
		kobject_put(rmi4_data->board_prop_dir);
	}

	synaptics_rmi4_irq_enable(rmi4_data, false, false);

#ifdef CONFIG_FB
	fb_unregister_client(&rmi4_data->fb_notifier);
#endif

#ifdef USE_EARLYSUSPEND
	unregister_early_suspend(&rmi4_data->early_suspend);
#endif

	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;
	if (rmi4_data->stylus_enable) {
		input_unregister_device(rmi4_data->stylus_dev);
		rmi4_data->stylus_dev = NULL;
	}

	synaptics_rmi4_gpio_setup(bdata->irq_gpio, false, 0, 0);

	if (bdata->reset_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->reset_gpio, false, 0, 0);

	if (bdata->power_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->power_gpio, false, 0, 0);

	synaptics_rmi4_enable_reg(rmi4_data, false);
	synaptics_rmi4_get_reg(rmi4_data, false);
	
	if (rmi4_data->ts_pinctrl) {
			if (IS_ERR_OR_NULL(rmi4_data->pinctrl_state_release)) {
				devm_pinctrl_put(rmi4_data->ts_pinctrl);
				rmi4_data->ts_pinctrl = NULL;
			} else {
				err = pinctrl_select_state(
					rmi4_data->ts_pinctrl,
					rmi4_data->pinctrl_state_release);
				if (err)
					dev_err(&pdev->dev,
						"Failed to select release pinctrl state %d\n",
						err);
			}
		}
		
//<ASUS_Proximity+>
#ifdef ASUS_TOUCH_PROXIMITY_NODE
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
	proc_remove(tp_proximity_proc);
#else
	remove_proc_entry(PROXIMITY_NAME, NULL);
#endif
#endif
//<ASUS_Proximity->

	kfree(rmi4_data);

	return 0;
}

static void synaptics_rmi4_f11_wg(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval;
	unsigned char reporting_control;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F11)
			break;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base,
			&reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change reporting mode\n",
				__func__);
		return;
	}

	reporting_control = (reporting_control & ~MASK_3BIT);
	if (enable)
		reporting_control |= F11_WAKEUP_GESTURE_MODE;
	else
		reporting_control |= F11_CONTINUOUS_MODE;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base,
			&reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change reporting mode\n",
				__func__);
		return;
	}

	return;
}

static void synaptics_rmi4_f12_wg(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval;
	unsigned char offset;
	unsigned char reporting_control[3];
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F12)
			break;
	}

	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	offset = extra_data->ctrl20_offset;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + offset,
			reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change reporting mode\n",
				__func__);
		return;
	}

	if (enable)
		reporting_control[2] = F12_WAKEUP_GESTURE_MODE;
	else
		reporting_control[2] = F12_CONTINUOUS_MODE;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base + offset,
			reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change reporting mode\n",
				__func__);
		return;
	}

	return;
}

static void synaptics_rmi4_wakeup_gesture(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	if (rmi4_data->f11_wakeup_gesture)
		synaptics_rmi4_f11_wg(rmi4_data, enable);
	else if (rmi4_data->f12_wakeup_gesture)
		synaptics_rmi4_f12_wg(rmi4_data, enable);

	return;
}

#ifdef CONFIG_FB
static int synaptics_rmi4_fb_notifier_cb(struct notifier_block *self,
		unsigned long event, void *data)
{
	int *transition;
	struct fb_event *evdata = data;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(self, struct synaptics_rmi4_data,
			fb_notifier);

	if (evdata && evdata->data && rmi4_data) {
		if (event == FB_EVENT_BLANK) {
			transition = evdata->data;
			if (*transition == FB_BLANK_POWERDOWN) {
				//printk(KERN_EMERG "%s FB_BLANK_POWERDOWN\n", __func__);
				synaptics_rmi4_suspend(&rmi4_data->pdev->dev);
				rmi4_data->fb_ready = false;
			} else if (*transition == FB_BLANK_UNBLANK) {
				//printk(KERN_EMERG "%s FB_BLANK_UNBLANK\n", __func__);
				if (fw_update_state == 1) {
				} else {
					synaptics_rmi4_resume(&rmi4_data->pdev->dev);
					rmi4_data->fb_ready = true;
				}
			} else if (*transition == FB_BLANK_NORMAL) {
				//printk(KERN_EMERG "%s FB_BLANK_NORMAL\n", __func__);
                synaptics_rmi4_suspend(&rmi4_data->pdev->dev);
                rmi4_data->fb_ready = false;
            }
		}
	}

	return 0;
}
#endif

#ifdef USE_EARLYSUSPEND
static void synaptics_rmi4_early_suspend(struct early_suspend *h)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(h, struct synaptics_rmi4_data,
			early_suspend);

	if (rmi4_data->stay_awake)
		return;

	if (rmi4_data->enable_wakeup_gesture) {
		synaptics_rmi4_wakeup_gesture(rmi4_data, true);
		enable_irq_wake(rmi4_data->irq);
		goto exit;
	}

	rmi4_data->touch_stopped = true;
	synaptics_rmi4_irq_enable(rmi4_data, false, false);
	synaptics_rmi4_sleep_enable(rmi4_data, true);
	synaptics_rmi4_free_fingers(rmi4_data);

exit:
	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->early_suspend != NULL)
				exp_fhandler->exp_fn->early_suspend(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	rmi4_data->suspend = true;

	return;
}

static void synaptics_rmi4_late_resume(struct early_suspend *h)
{
#ifdef FB_READY_RESET
	int retval;
#endif
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(h, struct synaptics_rmi4_data,
			early_suspend);

	if (rmi4_data->stay_awake)
		return;

	if (rmi4_data->enable_wakeup_gesture) {
		synaptics_rmi4_wakeup_gesture(rmi4_data, false);
		disable_irq_wake(rmi4_data->irq);
		goto exit;
	}

	rmi4_data->current_page = MASK_8BIT;

	if (rmi4_data->suspend) {
		synaptics_rmi4_sleep_enable(rmi4_data, false);
		synaptics_rmi4_irq_enable(rmi4_data, true, false);
	}

exit:
#ifdef FB_READY_RESET
	if (rmi4_data->suspend) {
		retval = synaptics_rmi4_reset_device(rmi4_data, false);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to issue reset command\n",
					__func__);
		}
	}
#endif
	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->late_resume != NULL)
				exp_fhandler->exp_fn->late_resume(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	rmi4_data->touch_stopped = false;
	rmi4_data->suspend = false;

	return;
}
#endif

static int synaptics_rmi4_suspend(struct device *dev)
{
	int retval,i;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	unsigned char buf_val[2]={0};
	unsigned char buf_reg[]={0x02, 0x03, 0x04, 0x05, 0x06};
	unsigned char buf_reg_val[]={0xFF, 0xFF, 0xFF, 0x00, 0x01};
	
    printk("%s suspend start\n", __func__);
    if(rmi4_data->suspend) 
        return 0;
    //<ASUS_cap_sensor+>
    //<ASUS_focal+>    
	if(cap_sel_status == 0) {
        printk("%s [cap] Disable cap_button virtualkey\n", __func__);
        disable_irq(CAP_INT_GET_PIN(CAP_INT_PIN));
        
        for(i=0; i<(sizeof(buf_reg)/sizeof(buf_reg[0])); i++) {
            buf_val[0] = buf_reg[i];
            buf_val[1] = buf_reg_val[i];
            retval = cap_i2c_write(rmi4_data, buf_val, 2);
            if(retval < 0){
                printk("[cap] write reg_addr=%d, retval =%d\n", buf_val[0], retval);
            }
            //printk("[cap] cap sensor write buf[0] =%x, buf[1]=%x\n", buf_val[0], buf_val[1]);
        }
    }else if(cap_sel_status == 1) {
        printk("%s [FTS] Disable cap_button virtualkey\n", __func__);
        disable_irq(CAP_INT_GET_PIN(CAP_INT_PIN));
    }
    //<ASUS_cap_sensor->
    //<ASUS_focal->
	
	if (rmi4_data->stay_awake)
		return 0;

	if (rmi4_data->enable_wakeup_gesture && ((dclick_mode == 1) || (swipe_mode == 1) || (gesture_mode & GESTURE_SWITCH))) {//<ASUS_DTP+>
		//printk("%s gesture suspend\n", __func__);
        synaptics_rmi4_free_fingers(rmi4_data);
		synaptics_rmi4_sleep_enable(rmi4_data, true);        
		synaptics_rmi4_wakeup_gesture(rmi4_data, true);
		enable_irq_wake(rmi4_data->irq);
		goto exit;
	}
		
	if (!rmi4_data->suspend) {
		if (rmi4_data->ts_pinctrl) {
		retval = pinctrl_select_state(rmi4_data->ts_pinctrl,
				rmi4_data->pinctrl_state_suspend);
		if (retval < 0) {
			dev_err(dev, "Cannot get idle pinctrl state\n");
			goto err_pinctrl_select_suspend;
		}
		}
		rmi4_data->touch_stopped = true;
		synaptics_rmi4_irq_enable(rmi4_data, false, false);
		synaptics_rmi4_sleep_enable(rmi4_data, true);
		synaptics_rmi4_free_fingers(rmi4_data);
	}

exit:
	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->suspend != NULL)
				exp_fhandler->exp_fn->suspend(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	rmi4_data->suspend = true;
	
    mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);
    if(cap_sel_status == 1) {        
        fts_a5_flag = 1;
        buf_val[0] = 0xa5;
        buf_val[1] = 0x03;
        retval = cap_i2c_write(rmi4_data, buf_val, 2);        
        if(retval < 0){
            printk("[cap] write reg_addr=%d, retval =%d\n", buf_val[0], retval);
            fts_a5_flag = 0;
        }
        msleep(20);
    }
    mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);
    printk("%s suspend end\n", __func__);
    
	return 0;
	
err_pinctrl_select_suspend:
//reserved vdd and avdd power regulator
	if (rmi4_data->sensor_sleep) {
		synaptics_rmi4_sleep_enable(rmi4_data, false);
		synaptics_rmi4_irq_enable(rmi4_data, true, true);
		rmi4_data->touch_stopped = false;
	}
	
	return retval;
}

static int synaptics_rmi4_resume(struct device *dev)
{
#ifdef FB_READY_RESET
	int retval;
#endif
	int retval,i;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	unsigned char buf_val[2]={0};
	unsigned char cap_read_addr = 0x00;
	unsigned char cap_status = 0x00;
	unsigned char buf_reg[]={0x06, 0x05, 0x02, 0x03, 0x04};
	unsigned char buf_reg_val_ER[]={0x20, 0x07, 0x4b, 0x4b, 0x4b}; //ER parameter
	unsigned char buf_reg_val_PR[]={0x20, 0x07, 0x4b, 0x4b, 0x4b}; //PR parameter
	unsigned char buf_reg_val_PR_usb[] = {0x20, 0x07, 0x4c, 0x47, 0x40}; //usb parameter
	unsigned char buf_reg_val_PR_glove[] = {0x29, 0x07, 0x3e, 0x3e, 0x3e}; //glove parameter
	unsigned char buf_reg_val_PR_glove_usb[] = {0x29, 0x07, 0x3e, 0x3e, 0x3e}; //glove and usb parameter	
    unsigned long onesec = msecs_to_jiffies(1500);

	
	if(cap_sel_status == 0) {
        printk("%s [cap] enable cap_button virtualkey\n", __func__);
        if (asus_HW_ID == HW_ID_ER1) {
            for(i=0; i<(sizeof(buf_reg)/sizeof(buf_reg[0])); i++) {
                buf_val[0] = buf_reg[i];
                buf_val[1] = buf_reg_val_ER[i];
                retval = cap_i2c_write(rmi4_data, buf_val, 2);
                if(retval < 0){
                    printk("[cap] write reg_addr=%d, retval =%d\n", buf_val[0], retval);
                }
                //printk("[cap] cap sensor write buf[0] =%x, buf[1]=%x\n", buf_val[0], buf_val[1]);
            }
        } else{
            if (glove_mode == 1) {
                if (rmi4_data->usb_status == 1) {
                    for(i=0; i<(sizeof(buf_reg)/sizeof(buf_reg[0])); i++) {
                        buf_val[0] = buf_reg[i];
                        buf_val[1] = buf_reg_val_PR_glove_usb[i];
                        retval = cap_i2c_write(rmi4_data, buf_val, 2);
                        if(retval < 0){
                            printk("[cap] write reg_addr=%d, retval =%d\n", buf_val[0], retval);
                        }
                        //printk("[cap] cap sensor write buf[0] =%x, buf[1]=%x\n", buf_val[0], buf_val[1]);
                    }
                }else {
                    for(i=0; i<(sizeof(buf_reg)/sizeof(buf_reg[0])); i++) {
                        buf_val[0] = buf_reg[i];
                        buf_val[1] = buf_reg_val_PR_glove[i];
                        retval = cap_i2c_write(rmi4_data, buf_val, 2);
                        if(retval < 0){
                            printk("[cap] write reg_addr=%d, retval =%d\n", buf_val[0], retval);
                        }
                        //printk("[cap] cap sensor write buf[0] =%x, buf[1]=%x\n", buf_val[0], buf_val[1]);
                    }
                }
            } else {
                if (rmi4_data->usb_status == 1){
                    for(i=0; i<(sizeof(buf_reg)/sizeof(buf_reg[0])); i++) {
                        buf_val[0] = buf_reg[i];
                        buf_val[1] = buf_reg_val_PR_usb[i];
                        retval = cap_i2c_write(rmi4_data, buf_val, 2);
                        if(retval < 0){
                            printk("[cap] write reg_addr=%d, retval =%d\n", buf_val[0], retval);
                        }
                        //printk("[cap] cap sensor write buf[0] =%x, buf[1]=%x\n", buf_val[0], buf_val[1]);
                    }			
                } else {
                    for(i=0; i<(sizeof(buf_reg)/sizeof(buf_reg[0])); i++) {
                        buf_val[0] = buf_reg[i];
                        buf_val[1] = buf_reg_val_PR[i];
                        retval = cap_i2c_write(rmi4_data, buf_val, 2);
                        if(retval < 0){
                            printk("[cap] write reg_addr=%d, retval =%d\n", buf_val[0], retval);
                        }
                        //printk("[cap] cap sensor write buf[0] =%x, buf[1]=%x\n", buf_val[0], buf_val[1]);
                    }
                }
            }
        }		
	enable_irq(CAP_INT_GET_PIN(CAP_INT_PIN));
    }else if(cap_sel_status == 1){        
        buf_val[0] = 0xa5;
        buf_val[1] = 0x00;
        retval = cap_i2c_write(rmi4_data, buf_val, 2);
        if(retval < 0){
            printk("[cap] write reg_addr=0x%02x, retval =%d\n", buf_val[0], retval);
        }
        msleep(5);
        printk("%s [FTS] enable cap_button virtualkey\n", __func__);
        fts_sw_reset(rmi4_data);        
        enable_irq(CAP_INT_GET_PIN(CAP_INT_PIN));
    }
	
	synaptics_rmi4_sw_reset(rmi4_data);
	
	//calibration+
    if(cap_sel_status == 0) {
        cap_i2c_Read(rmi4_data, &cap_read_addr, 1, &cap_status, 1);
        buf_val[0] = 0x06;
        buf_val[1] = 0x0F;
        if (glove_mode == 1) {
            if (rmi4_data->usb_status == 1) { //(glove, usb) = (1,1)
                buf_val[1] = 0x29;
            }else {							  //(glove, usb) = (1,0)
                buf_val[1] = 0x29;
            }
        }else {
            if (rmi4_data->usb_status == 1) { //(glove, usb) = (0,1)
                buf_val[1] = 0x20;
            } else {
                buf_val[1] = 0x20;	  //(glove, usb) = (0,0)
            }
        }
        if (buf_val == 0) {		
            retval = cap_i2c_write(rmi4_data, buf_val, 2);
            printk("[cap] %s set sampling configuration write value=0x%x.\n", __func__, buf_val[1]);
        }
    }
	//calibration-
	
	//<ASUS_COVER+>
	synaptics_rmi4_set_cover_param(rmi4_data);
	//<ASUS_COVER->
	//<ASUS_Glove+>
	synaptics_rmi4_set_glove_param(rmi4_data);
    if(cap_sel_status == 1) {
        retval = work_busy(&rmi4_data->fts_glove_delay_work.work);
        if(retval == WORK_BUSY_PENDING) {
            cancel_delayed_work_sync(&rmi4_data->fts_glove_delay_work);					
        }
        retval = queue_delayed_work(rmi4_data->fts_glove_wq, &rmi4_data->fts_glove_delay_work, onesec);
        if(retval < 0)
            printk("%s ret = %d glove delay queue failure\n", __func__, retval);
    }
	//<ASUS_Glove->
	
	if (rmi4_data->stay_awake)
		return 0;

	if (rmi4_data->enable_wakeup_gesture && ((dclick_mode == 1) || (swipe_mode == 1) || (gesture_mode & GESTURE_SWITCH))) {//<ASUS_DTP+>
		//printk("%s gesture resume\n", __func__);
		synaptics_rmi4_sleep_enable(rmi4_data, false);
		synaptics_rmi4_wakeup_gesture(rmi4_data, false);
		disable_irq_wake(rmi4_data->irq);
		goto exit;
	}

	rmi4_data->current_page = MASK_8BIT;

	if (rmi4_data->ts_pinctrl) {
		retval = pinctrl_select_state(rmi4_data->ts_pinctrl,
				rmi4_data->pinctrl_state_active);
		if (retval < 0)
			dev_err(dev, "Cannot get default pinctrl state\n");
	}
	synaptics_rmi4_sleep_enable(rmi4_data, false);
	synaptics_rmi4_irq_enable(rmi4_data, true, false);

exit:
#ifdef FB_READY_RESET
	retval = synaptics_rmi4_reset_device(rmi4_data, false);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to issue reset command\n",
				__func__);
	}
#endif
	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->resume != NULL)
				exp_fhandler->exp_fn->resume(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	rmi4_data->touch_stopped = false;
	rmi4_data->suspend = false;

	return 0;
}

#ifdef CONFIG_PM
static const struct dev_pm_ops synaptics_rmi4_dev_pm_ops = {
#ifndef CONFIG_FB
	.suspend = synaptics_rmi4_suspend,
	.resume = synaptics_rmi4_resume,
#endif
};
#endif

static struct platform_driver synaptics_rmi4_driver = {
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &synaptics_rmi4_dev_pm_ops,
#endif
	},
	.probe = synaptics_rmi4_probe,
	.remove = synaptics_rmi4_remove,
};

static int __init synaptics_rmi4_init(void)
{
	int retval;
	extern char* androidboot_mode;

	if (strcmp(androidboot_mode,"charger")==0) {
		printk("[Power] %s: skip this driver in charger mode\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_bus_init();
	if (retval)
		return retval;

	return platform_driver_register(&synaptics_rmi4_driver);
}

static void __exit synaptics_rmi4_exit(void)
{
	//ASUS_FACTORY+
	#if defined(ASUS_FACTORY_BUILD)
	if (tp_entry)
		remove_proc_entry("asus_tp_status", NULL);
	#endif
	//ASUS_FACTORY-

	platform_driver_unregister(&synaptics_rmi4_driver);

	synaptics_rmi4_bus_exit();

	return;
}

module_init(synaptics_rmi4_init);
module_exit(synaptics_rmi4_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX Touch Driver");
MODULE_LICENSE("GPL v2");
