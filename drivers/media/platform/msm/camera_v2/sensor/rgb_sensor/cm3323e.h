/* include/linux/cm3323e.h
 *
 * Copyright (C) 2012 Capella Microsystems Inc.
 * Author: Frank Hsieh <pengyueh@gmail.com>  
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_CM3323E_H
#define __LINUX_CM3323E_H

#define CM3323E_I2C_NAME		"cm3323ee"

#define	CM3323E_ADDR			0x20 >> 1

#define STATUS_PROC_FILE                "driver/RGB_status"      /* Status */
#define RGBDEBUG_PROC_FILE              "driver/asusRgbDebug"    /* rgb debug */

/* cm3323e command code */
#define	CM3323E_CONF			0x00
#define CM3323E_R_DATA		0x08
#define CM3323E_G_DATA		0x09
#define CM3323E_B_DATA		0x0A
#define CM3323E_W_DATA		0x0B

/* cm3323e CONF command code */
#define CM3323E_CONF_SD			1
#define CM3323E_CONF_AF			(1 << 1)
#define CM3323E_CONF_TRIG		(1 << 2)
#define CM3323E_CONF_IT_40MS		0
#define CM3323E_CONF_IT_80MS		1
#define CM3323E_CONF_IT_160MS	2
#define CM3323E_CONF_IT_320MS	3
#define CM3323E_CONF_IT_640MS	4
#define CM3323E_CONF_IT_1280MS	5
#define CM3323E_CONF_DEFAULT		0

#define CM3323E_CONF_IT_MASK            0x0070
 
#define LS_PWR_ON					(1 << 0)

struct cm3323e_platform_data {
	int (*power)(int, uint8_t); /* power to the chip */
	uint16_t RGB_slave_address;
};

#endif
