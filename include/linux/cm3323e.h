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

#define CM3323E_I2C_NAME		"cm3323e"

#define	CM3323E_ADDR			0x20 >> 1

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
#define CM3323E_CONF_IT_40MS		(0 << 4)
#define CM3323E_CONF_IT_80MS		(1 << 4)
#define CM3323E_CONF_IT_160MS	(2 << 4)
#define CM3323E_CONF_IT_320MS	(3 << 4)
#define CM3323E_CONF_IT_640MS	(4 << 4)
#define CM3323E_CONF_IT_1280MS	(5 << 4)
#define CM3323E_CONF_DEFAULT		0

#define LS_PWR_ON					(1 << 0)

struct cm3323e_platform_data {
	int (*power)(int, uint8_t); /* power to the chip */
	uint16_t RGB_slave_address;
};

#define BIT6				0x00000040
#define BIT5				0x00000020
#define BIT4				0x00000010


#define ASUS_RGB_SENSOR_DATA_SIZE	5
#define ASUS_RGB_SENSOR_NAME_SIZE	32
#define ASUS_RGB_SENSOR_IOC_MAGIC                      ('C')		///< RGB sensor ioctl magic number 
#define ASUS_RGB_SENSOR_IOCTL_DATA_READ           	 _IOR(ASUS_RGB_SENSOR_IOC_MAGIC, 1, int[ASUS_RGB_SENSOR_DATA_SIZE])	///< RGB sensor ioctl command - Read data RGBW
#define ASUS_RGB_SENSOR_IOCTL_IT_SET          		 _IOW(ASUS_RGB_SENSOR_IOC_MAGIC, 2, int)	///< RGB sensor ioctl command - Set integration time
#define ASUS_RGB_SENSOR_IOCTL_DEBUG_MODE           _IOW(ASUS_RGB_SENSOR_IOC_MAGIC, 3, int)	///< RGB sensor ioctl command - Get debug mode
#define ASUS_RGB_SENSOR_IOCTL_MODULE_NAME         _IOR(ASUS_RGB_SENSOR_IOC_MAGIC, 4, char[ASUS_RGB_SENSOR_NAME_SIZE])	///< RGB sensor ioctl command - Get module name
#define ASUS_RGB_SENSOR_IOCTL_GET_ENABLED		 _IOR(ASUS_RGB_SENSOR_IOC_MAGIC, 5, int *)
#define ASUS_RGB_SENSOR_IOCTL_ENABLE 			 _IOW(ASUS_RGB_SENSOR_IOC_MAGIC, 7, int)


#endif
