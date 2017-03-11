/*
 * File:   fusb30x_driver.c
 * Author: Tim Bremm <tim.bremm@fairchildsemi.com>
 * Company: Fairchild Semiconductor
 *
 * Created on September 2, 2015, 10:22 AM
 */

/* Standard Linux includes */
#include <linux/init.h>                                                         // __init, __initdata, etc
#include <linux/module.h>                                                       // Needed to be a module
#include <linux/kernel.h>                                                       // Needed to be a kernel module
#include <linux/i2c.h>                                                          // I2C functionality
#include <linux/slab.h>                                                         // devm_kzalloc
#include <linux/types.h>                                                        // Kernel datatypes
#include <linux/errno.h>                                                        // EINVAL, ERANGE, etc
#include <linux/of_device.h>                                                    // Device tree functionality

/* Driver-specific includes */
#include "fusb30x_global.h"                                                     // Driver-specific structures/types
#include "platform_helpers.h"                                                   // I2C R/W, GPIO, misc, etc

#include "../core/fusb30X.h"

/* ASUS_BSP : For GPIO/adb(proc file)/log in FUSB302 +++ */
	/* include for proc file/kernel log */
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
	/* include for pmic */
#include <linux/regulator/consumer.h>
	/* include for smbcharger */
#include <linux/power_supply.h>
        /* include for delay */
#include <linux/delay.h>
        /* usb orientation */
extern FSC_BOOL blnCCPinIsCC1;
extern FSC_BOOL blnCCPinIsCC2;
extern DeviceReg_t  Registers;
        /* PD factory informations dump */
extern FSC_BOOL USBPDEnabled;
        /* usb variable for OTG charger */
struct power_supply *usb_psy;   // for OTG set property
struct power_supply *usb_parallel_psy;	// for OTG set property
bool   flag_is_otg_present = 0; // for qpnp-smbcharger.c : function "platform_fusb302_is_otg_present" flag
/* ASUS_BSP : For GPIO/adb(proc file)/log in FUSB302 --- */

#ifdef FSC_DEBUG
#include "../core/core.h"                                                       // GetDeviceTypeCStatus
#endif // FSC_DEBUG

#include "fusb30x_driver.h"

/******************************************************************************
*                        Create Proc  : For Factory --- I2C report                                                                             *
******************************************************************************/
ssize_t fusb302_i2c_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len= 0;
	ssize_t ret=0;
	char *buff;
	//extern DeviceReg_t Registers;
	USB_FUSB302_331_INFO("%s +++\n", __func__);
	buff = kmalloc(300,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;

        //  Dump USB I2C checking Informations in adb shell : when i2c communication success , cc pin values will be updated
	if((blnCCPinIsCC1==1) && (blnCCPinIsCC2==0))						// cc1
		len += sprintf(buff + len, "USB Type-C I2C = 1\n");
	else if((blnCCPinIsCC1==0) && (blnCCPinIsCC2==1))					// cc2
		len += sprintf(buff + len, "USB Type-C I2C = 1\n");
	else if((blnCCPinIsCC1==0) && (blnCCPinIsCC2==0))					// None
        {/*
		len += sprintf(buff + len, "CC pin value is none.\n");
                len += sprintf(buff + len, "Please check the usb port.\n");
                len += sprintf(buff + len, "Do you plug-in usb test device into usb port? \n");
                len += sprintf(buff + len, "If yes ---> USB Type-C I2C = 0\n");
                len += sprintf(buff + len, "If no, please plug-in usb test device.\n");
        */
                len += sprintf(buff + len, "USB Type-C I2C = 0\n");
        }
	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
        kfree(buff);
	return ret;
}

int init_asus_for_i2c_fusb302(void)
{
	struct proc_dir_entry *entry=NULL;
	static struct file_operations fusb302_PD_fop = {
		.read  = fusb302_i2c_read,
	};
	USB_FUSB302_331_INFO("init_asus_for_i2c_fusb302\n");
	entry = proc_create("factory_i2c", 0666,NULL, &fusb302_PD_fop);
	if(!entry)
	{
		USB_FUSB302_331_INFO("create /proc/factory_i2c fail\n");
	}

	return 0;
}


/******************************************************************************
*                        Create Proc  : For Factory --- PD                                                                             *
******************************************************************************/
ssize_t fusb302_PD_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len= 0;
	ssize_t ret=0;
	char *buff;
	//extern DeviceReg_t Registers;
	USB_FUSB302_331_INFO("%s +++\n", __func__);
	buff = kmalloc(300,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;

        //  Dump USB PD Informations in adb shell
	if(USBPDEnabled == TRUE)			// PD enable
		len += sprintf(buff + len, "Type-C PD = %d\n",USBPDEnabled);
	else if(!USBPDEnabled == FALSE)			// PD disable
		len += sprintf(buff + len, "Type-C PD = %d\n",USBPDEnabled);

	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
        kfree(buff);
	return ret;

}

int init_asus_for_PD_fusb302(void)
{
	struct proc_dir_entry *entry=NULL;
	static struct file_operations fusb302_PD_fop = {
		.read  = fusb302_PD_read,
	};
	USB_FUSB302_331_INFO("init_asus_for_PD_fusb302\n");
	entry = proc_create("factory_PD", 0666,NULL, &fusb302_PD_fop);
	if(!entry)
	{
		USB_FUSB302_331_INFO("create /proc/factory_PD fail\n");
	}

	return 0;
}
/******************************************************************************
*                        Create Proc  : For Factory --- CC pin orientation                                                                              *
******************************************************************************/
ssize_t fusb302_factory_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len= 0;
	ssize_t ret=0;
	char *buff;
	//extern DeviceReg_t Registers;
	USB_FUSB302_331_INFO("%s +++\n", __func__);
	buff = kmalloc(300,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;

        //  Dump USB Orientation Informations in adb shell
	if((blnCCPinIsCC1==1) && (blnCCPinIsCC2==0))					        // cc1
		len += sprintf(buff + len, "USB Type-C Orientation = 1\n");
	else if((blnCCPinIsCC1==0) && (blnCCPinIsCC2==1))					// cc2
		len += sprintf(buff + len, "USB Type-C Orientation = 2\n");
	else if((blnCCPinIsCC1==0) && (blnCCPinIsCC2==0))					// None
		len += sprintf(buff + len, "USB Type-C Orientation = 0(None)\n");
	else											// Error : cc1=cc2=1
		len += sprintf(buff + len, "Something Error In USB Type-C(CC1=%d,cc2=%d)\n",blnCCPinIsCC1,blnCCPinIsCC2);

	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
        kfree(buff);
	return ret;

}

int init_asus_for_factory_fusb302(void)
{
	struct proc_dir_entry *entry=NULL;
	static struct file_operations fusb302_factory_fop = {
		.read  = fusb302_factory_read,
	};
	USB_FUSB302_331_INFO("init_asus_for_factory_fusb302\n");
	entry = proc_create("factory_typec", 0666,NULL, &fusb302_factory_fop);
	if(!entry)
	{
		USB_FUSB302_331_INFO("create /proc/factory_typec fail\n");
	}

	return 0;
}

/******************************************************************************
*                        Create Proc  : For Engineer                                                                              *
******************************************************************************/
ssize_t fusb302_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len= 0;
	ssize_t ret=0;
	int GPIOVal;
	int RegVal;
	char *buff;
	//extern DeviceReg_t Registers;
	USB_FUSB302_331_INFO("%s +++\n", __func__);
	buff = kmalloc(300,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;

       //  Dump MUX GPIO 25 Details in adb shell
	GPIOVal = gpio_get_value(USB_MUX_GPIO);
	len += sprintf(buff + len, "GPIO %d state = %d , Pin CC1 state = %d , Pin CC2 state = %d\n", USB_MUX_GPIO ,GPIOVal,blnCCPinIsCC1,blnCCPinIsCC2);
	   // Dump Register values in adb shell
	RegVal = DeviceRead(regDeviceID, 1, &Registers.DeviceID.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regDeviceID ,Registers.DeviceID.byte);
	RegVal = DeviceRead(regSwitches0, 1, &Registers.Switches.byte[0]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regSwitches0 ,Registers.Switches.byte[0]);
	RegVal = DeviceRead(regSwitches1, 1, &Registers.Switches.byte[1]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regSwitches1 ,Registers.Switches.byte[1]);
	RegVal = DeviceRead(regMeasure, 1, &Registers.Measure.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regMeasure ,Registers.Measure.byte);
	RegVal = DeviceRead(regSlice, 1, &Registers.Slice.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regSlice ,Registers.Slice.byte);
	RegVal = DeviceRead(regControl0, 1, &Registers.Control.byte[0]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regControl0 ,Registers.Control.byte[0]);
	RegVal = DeviceRead(regControl1, 1, &Registers.Control.byte[1]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regControl1 ,Registers.Control.byte[1]);
	RegVal = DeviceRead(regControl2, 1, &Registers.Control.byte[2]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regControl2 ,Registers.Control.byte[2]);
	RegVal = DeviceRead(regControl3, 1, &Registers.Control.byte[3]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regControl3 ,Registers.Control.byte[3]);
	RegVal = DeviceRead(regMask, 1, &Registers.Mask.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regMask ,Registers.Mask.byte);
	RegVal = DeviceRead(regPower, 1, &Registers.Power.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regPower ,Registers.Power.byte);
	RegVal = DeviceRead(regReset, 1, &Registers.Reset.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regReset ,Registers.Reset.byte);
	RegVal = DeviceRead(regOCPreg, 1, &Registers.OCPreg.byte);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regOCPreg ,Registers.OCPreg.byte);
	RegVal = DeviceRead(regMaska, 1, &Registers.MaskAdv.byte[0]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regMaska ,Registers.MaskAdv.byte[0]);
	RegVal = DeviceRead(regMaskb, 1, &Registers.MaskAdv.byte[1]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regMaskb ,Registers.MaskAdv.byte[1]);
	RegVal = DeviceRead(regStatus0a, 1, &Registers.Status.byte[0]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regStatus0a ,Registers.Status.byte[0]);
	RegVal = DeviceRead(regStatus1a, 1, &Registers.Status.byte[1]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regStatus1a ,Registers.Status.byte[1]);
	RegVal = DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regStatus0 ,Registers.Status.byte[4]);
	RegVal = DeviceRead(regStatus1, 1, &Registers.Status.byte[5]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regStatus1 ,Registers.Status.byte[5]);
	RegVal = DeviceRead(regInterrupt, 1, &Registers.Status.byte[6]);
	len += sprintf(buff + len, "0x0%x  0x%02x\n", regInterrupt ,Registers.Status.byte[6]);

	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
        kfree(buff);
	return ret;

}

int proc_reg;
int proc_value;
char  proc_buf[64];
ssize_t fusb302_write(struct file *filp,const char __user *buffer, size_t count, loff_t *ppos)
{
	//int ret = 0;
	/* turn to number */
	int adb_com = 999;
	char adb_val;
    /* check command details */
    int result = -1;
	USB_FUSB302_331_INFO("%s +++\n", __func__);
    if (copy_from_user(proc_buf, buffer, count)) {
        USB_FUSB302_331_INFO("read data from user space error\n");
        return -EFAULT;
    }
    sscanf(proc_buf, "%x %x\n", &proc_reg, &proc_value);
    USB_FUSB302_331_INFO("%d,%d\n", proc_reg, proc_value);



	adb_com = proc_reg;
	adb_val = (char) proc_value;

	USB_FUSB302_331_INFO("adb_com = %x , adb_val = %x\n", adb_com, adb_val);



	USB_FUSB302_331_INFO("write register %x with value %x\n", adb_com, adb_val);

	/* start to process command(write register value in adb shell) */

    if( (adb_com == regDeviceID))
	{
		Registers.DeviceID.byte = adb_val;
		result = DeviceWrite(regDeviceID, 1, &Registers.DeviceID.byte);
	}
    if( (adb_com == regSwitches0))
	{
		Registers.Switches.byte[0] = adb_val;
		result = DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
	}
    if( (adb_com == regSwitches1))
	{
		Registers.Switches.byte[1] = adb_val;
		result = DeviceWrite(regSwitches1, 1, &Registers.Switches.byte[1]);
	}
    if( (adb_com == regMeasure))
	{
		Registers.Measure.byte = adb_val;
		result = DeviceWrite(regMeasure, 1, &Registers.Measure.byte);
	}
    if( (adb_com == regSlice))
	{
		Registers.Slice.byte = adb_val;
		result = DeviceWrite(regSlice, 1, &Registers.Slice.byte);
	}
    if( (adb_com == regControl0))
	{
		Registers.Control.byte[0] = adb_val;
		result = DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);
	}
    if( (adb_com == regControl1))
	{
		Registers.Control.byte[1] = adb_val;
		result = DeviceWrite(regControl1, 1, &Registers.Control.byte[1]);
	}
    if( (adb_com == regControl2))
	{
		Registers.Control.byte[2] = adb_val;
		result = DeviceWrite(regControl2, 1, &Registers.Control.byte[2]);
	}
    if( (adb_com == regControl3))
	{
		Registers.Control.byte[3] = adb_val;
		result = DeviceWrite(regControl3, 1, &Registers.Control.byte[3]);
	}
    if( (adb_com == regMask))
	{
		Registers.Mask.byte = adb_val;
		result = DeviceWrite(regMask, 1, &Registers.Mask.byte);
	}
    if( (adb_com == regPower))
	{
		Registers.Power.byte = adb_val;
		result = DeviceWrite(regPower, 1, &Registers.Power.byte);
	}
    if( (adb_com == regReset))
	{
	    USB_FUSB302_331_INFO("adb_com = %x , adb_val = %x\n", adb_com, adb_val);
		Registers.Reset.byte = adb_val;
		result = DeviceWrite(regReset, 1, &Registers.Reset.byte);
		USB_FUSB302_331_INFO("register 0x%2x = 0x%2x\n", regReset ,result);
	}
/*
    if( (adb_com == regOCPreg))
	{
		Registers.OCPreg.byte = adb_val;
		result = DeviceWrite(regOCPreg, 1, &Registers.OCPreg.byte);
	}
    if( (adb_com == regMaska))
	{
		Registers.MaskAdv.byte[0] = adb_val;
		result = DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
	}
    if( (adb_com == regMaskb))
	{
		Registers.MaskAdv.byte[1] = adb_val;
		result = DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
	}
    if( (adb_com == regStatus0a))
	{
		Registers.Status.byte[0] = adb_val;
		result = DeviceWrite(regStatus0a, 1, &Registers.Status.byte[0]);
	}
    if( (adb_com == regStatus1a))
	{
		Registers.Status.byte[1] = adb_val;
		result = DeviceWrite(regStatus1a, 1, &Registers.Status.byte[1]);
	}
*/
	//DeviceRead(regInterrupta, 1, &Registers.Status.byte[2]); //Disabled reading interrupts
	//DeviceRead(regInterruptb, 1, &Registers.Status.byte[3]);
/*
    if( (adb_com == regStatus0))
	{
		Registers.Status.byte[4] = adb_val;
		result = DeviceWrite(regStatus0, 1, &Registers.Status.byte[4]);
	}
    if( (adb_com == regStatus1))
	{
		Registers.Status.byte[5] = adb_val;
		result = DeviceWrite(regStatus1, 1, &Registers.Status.byte[5]);
    }
*/
	return count;
}

int init_asus_for_fusb302(void)
{
    struct proc_dir_entry *entry=NULL;
	static struct file_operations fusb302Reg_fop = {
	    .read  = fusb302_read,
	    .write = fusb302_write,
	};
	USB_FUSB302_331_INFO("init_asus_for_fusb302\n");
	entry = proc_create("fusb302", 0666,NULL, &fusb302Reg_fop);
	if(!entry)
	{
		USB_FUSB302_331_INFO("create /proc/fusb302 fail\n");
	}

	return 0;
}

/******************************************************************************
*                        Create Proc  : For USB Switch Speed Test(EE)                                                                              *
******************************************************************************/
ssize_t fusb302_speed_2_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len= 0;
	ssize_t ret=0;
	int GPIO_16_Val; // Re-driver GPIO
	int GPIO_96_Val; // USB 2.0 GPIO
	char *buff;
	//extern DeviceReg_t Registers;
	USB_FUSB302_331_INFO("%s +++\n", __func__);
	buff = kmalloc(300,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;

        //  Dump Re-driver GPIO 16 / USB 2.0 GPIO 96 Details in adb shell
	GPIO_16_Val = gpio_get_value(16);
	GPIO_96_Val = gpio_get_value(96);
	len += sprintf(buff + len, "GPIO 16 state = %d , GPIO 96 state = %d\n", GPIO_16_Val ,GPIO_96_Val);

	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
        kfree(buff);
	return ret;
}

int proc_value_2;
char  proc_buf_2[64];
ssize_t fusb302_speed_2_write(struct file *filp,const char __user *buffer, size_t count, loff_t *ppos)
{
	//int ret = 0;
	int adb_val = 0;
	/* turn to number */
        /* check command details */
	USB_FUSB302_331_INFO("%s +++\n", __func__);
        if (copy_from_user(proc_buf_2, buffer, count)) {
                USB_FUSB302_331_INFO("read data from user space error\n");
                return -EFAULT;
        }
        sscanf(proc_buf_2, "%x\n", &proc_value_2);
	adb_val = proc_value_2;
        USB_FUSB302_331_INFO("proc_value_2 = %d\n", adb_val);

	gpio_set_value(96,adb_val);

	return count;
}


int init_asus_for_usb_2_speed(void)
{
        struct proc_dir_entry *entry=NULL;
	static struct file_operations fusb302_speed_2_fop = {
	    .read  = fusb302_speed_2_read,
	    .write = fusb302_speed_2_write,
	};
	USB_FUSB302_331_INFO("init_asus_for_usb_2_speed\n");
	entry = proc_create("fusb302_speed_2", 0666,NULL, &fusb302_speed_2_fop);
	if(!entry)
	{
		USB_FUSB302_331_INFO("create /proc/fusb302_speed_2 fail\n");
	}

	return 0;
}

ssize_t fusb302_speed_3_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len= 0;
	ssize_t ret=0;
	int GPIO_16_Val; // Re-driver GPIO
	int GPIO_96_Val; // USB 2.0 GPIO
	char *buff;
	//extern DeviceReg_t Registers;
	USB_FUSB302_331_INFO("%s +++\n", __func__);
	buff = kmalloc(300,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;

       //  Dump Re-driver GPIO 16 / USB 2.0 GPIO 96 Details in adb shell
	GPIO_16_Val = gpio_get_value(16);
	GPIO_96_Val = gpio_get_value(96);
	len += sprintf(buff + len, "GPIO 16 state = %d , GPIO 96 state = %d\n", GPIO_16_Val ,GPIO_96_Val);

	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
        kfree(buff);
	return ret;
}

int proc_value_3;
char  proc_buf_3[64];
ssize_t fusb302_speed_3_write(struct file *filp,const char __user *buffer, size_t count, loff_t *ppos)
{
	//int ret = 0;
	int adb_val = 0;
	/* turn to number */
        /* check command details */
	USB_FUSB302_331_INFO("%s +++\n", __func__);
        if (copy_from_user(proc_buf_3, buffer, count)) {
                USB_FUSB302_331_INFO("read data from user space error\n");
                return -EFAULT;
        }
        sscanf(proc_buf_3, "%x\n", &proc_value_3);
	adb_val = proc_value_3;
        USB_FUSB302_331_INFO("proc_value_3 = %d\n", adb_val);

        gpio_set_value(16,adb_val);

	return count;
}


int init_asus_for_usb_3_speed(void)
{
        struct proc_dir_entry *entry=NULL;
	static struct file_operations fusb302_speed_3_fop = {
	    .read  = fusb302_speed_3_read,
	    .write = fusb302_speed_3_write,
	};
	USB_FUSB302_331_INFO("init_asus_for_usb_3_speed\n");
	entry = proc_create("fusb302_speed_3", 0666,NULL, &fusb302_speed_3_fop);
	if(!entry)
	{
		USB_FUSB302_331_INFO("create /proc/fusb302_speed_3 fail\n");
	}

	return 0;
}
/******************************************************************************
*                        Create Proc  : For Type-C Driver Version                                                                              *
******************************************************************************/
ssize_t fusb302_version_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len= 0;
	ssize_t ret=0;
	char *buff;
	//extern DeviceReg_t Registers;
	USB_FUSB302_331_INFO("%s +++\n", __func__);
	buff = kmalloc(300,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;

	len += sprintf(buff + len, "FUSB302 version = 3.3.1\n");

	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
        kfree(buff);
	return ret;
}

int init_asus_typec_version(void)
{
        struct proc_dir_entry *entry=NULL;
	static struct file_operations fusb302_version_fop = {
	    .read  = fusb302_version_read,
	};
	USB_FUSB302_331_INFO("init_asus_typec_version\n");
	entry = proc_create("fusb302_version", 0666,NULL, &fusb302_version_fop);
	if(!entry)
	{
		USB_FUSB302_331_INFO("create /proc/fusb302_version fail\n");
	}

	return 0;
}
/******************************************************************************
* Driver functions
******************************************************************************/
static int __init fusb30x_init(void)
{
        pr_debug("FUSB  %s - Start driver initialization...\n", __func__);

	return i2c_add_driver(&fusb30x_driver);
}

static void __exit fusb30x_exit(void)
{
	i2c_del_driver(&fusb30x_driver);
        pr_debug("FUSB  %s - Driver deleted...\n", __func__);
}

/* ASUS_BSP : apply device tree setting --- pull up interrupt pin(58) +++ */
static void set_pinctrl(struct device *dev)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;

	key_pinctrl = devm_pinctrl_get(dev);
	set_state = pinctrl_lookup_state(key_pinctrl, "fusb_int_active");
	ret = pinctrl_select_state(key_pinctrl, set_state);
}
/* ASUS_BSP : apply device tree setting --- pull up interrupt pin(58) --- */

static int fusb30x_probe (struct i2c_client* client,
                          const struct i2c_device_id* id)
{
        int ret = 0;
        struct fusb30x_chip* chip;
        struct i2c_adapter* adapter;
	int ldo_var = 0;
        //int reset_result = 0;
	int proc_ret =0;
	struct power_supply *battery_psy = NULL;
        struct regulator *regulator_usbvdd;
	/* ASUS_BSP : for charger ready +++ */
	battery_psy = power_supply_get_by_name("battery");
	if (!battery_psy) {
		USB_FUSB302_331_INFO("USB supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}
	/* ASUS_BSP : for charger ready --- */
	/* ASUS_BSP : set OTG property "usb" +++ */
        usb_psy = power_supply_get_by_name("usb");
        if (!usb_psy) {
                USB_FUSB302_331_INFO("USB supply not found, deferring probe\n");
        }
	usb_parallel_psy= power_supply_get_by_name("usb-parallel");
        if (!usb_parallel_psy) {
                USB_FUSB302_331_INFO("USB(parallel) supply not found, deferring probe\n");
        }
        /* ASUS_BSP : set OTG property "usb" --- */
        /* ASUS_BSP : always turn on usb power(LDO 22) +++ */
        regulator_usbvdd = regulator_get(&client->dev, " usbvdd ");
	ldo_var = regulator_enable(regulator_usbvdd);
        /* ASUS_BSP : always turn on usb power(LDO 22) --- */
	/* ASUS_BSP : apply gpio setting +++ */
	set_pinctrl(&client->dev);
	/* ASUS_BSP : apply gpio setting --- */
        //USB_FUSB302_331_INFO("sleep_1_start\n");
        //msleep(200);
        //USB_FUSB302_331_INFO("sleep_1_end\n");
        /* ASUS_BSP : make sure interrupt pin reset to high +++ */
        //Registers.Reset.byte = 0x01;
        //reset_result = DeviceWrite(regReset, 1, &Registers.Reset.byte);
        //USB_FUSB302_331_INFO("[Reset] register 0x0%x = 0x0%x\n", regReset ,reset_result);
        //USB_FUSB302_331_INFO("[Reset] interrupt GPIO 58 value = %d\n",gpio_get_value_cansleep(58));
        /* ASUS_BSP : make sure interrupt pin reset to high --- */
        //USB_FUSB302_331_INFO("sleep_2_start\n");
        //msleep(200);
        //USB_FUSB302_331_INFO("sleep_2_end\n");
	USB_FUSB302_331_INFO("[probe] fusb30x start !\n");

        if (!client)
        {
                pr_err("FUSB  %s - Error: Client structure is NULL!\n", __func__);
                return -EINVAL;
        }
        dev_info(&client->dev, "%s\n", __func__);

        /* Make sure probe was called on a compatible device */
	if (!of_match_device(fusb30x_dt_match, &client->dev))
	{
		dev_err(&client->dev, "FUSB  %s - Error: Device tree mismatch!\n", __func__);
		return -EINVAL;
	}
        pr_debug("FUSB  %s - Device tree matched!\n", __func__);

        /* Allocate space for our chip structure (devm_* is managed by the device) */
        chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
        if (!chip)
	{
		dev_err(&client->dev, "FUSB  %s - Error: Unable to allocate memory for g_chip!\n", __func__);
		return -ENOMEM;
	}
        chip->client = client;                                                      // Assign our client handle to our chip
        fusb30x_SetChip(chip);                                                      // Set our global chip's address to the newly allocated memory
        pr_debug("FUSB  %s - Chip structure is set! Chip: %p ... g_chip: %p\n", __func__, chip, fusb30x_GetChip());

        /* Initialize the chip lock */
        mutex_init(&chip->lock);

        /* Initialize the chip's data members */
        fusb_InitChipData();
        pr_debug("FUSB  %s - Chip struct data initialized!\n", __func__);

        /* Verify that the system has our required I2C/SMBUS functionality (see <linux/i2c.h> for definitions) */
        adapter = to_i2c_adapter(client->dev.parent);
        if (i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_BLOCK_REQUIRED_FUNC))
        {
                chip->use_i2c_blocks = true;
        }
        else
        {
        // If the platform doesn't support block reads, try with block writes and single reads (works with eg. RPi)
        // NOTE: It is likely that this may result in non-standard behavior, but will often be 'close enough' to work for most things
                dev_warn(&client->dev, "FUSB  %s - Warning: I2C/SMBus block read/write functionality not supported, checking single-read mode...\n", __func__);
                if (!i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_REQUIRED_FUNC))
                {
                        dev_err(&client->dev, "FUSB  %s - Error: Required I2C/SMBus functionality not supported!\n", __func__);
                        dev_err(&client->dev, "FUSB  %s - I2C Supported Functionality Mask: 0x%x\n", __func__, i2c_get_functionality(adapter));
                        return -EIO;
                }
        }
        pr_debug("FUSB  %s - I2C Functionality check passed! Block reads: %s\n", __func__, chip->use_i2c_blocks ? "YES" : "NO");

        /* Assign our struct as the client's driverdata */
        i2c_set_clientdata(client, chip);
        pr_debug("FUSB  %s - I2C client data set!\n", __func__);

        /* Verify that our device exists and that it's what we expect */
        if (!fusb_IsDeviceValid())
        {
                dev_err(&client->dev, "FUSB  %s - Error: Unable to communicate with device!\n", __func__);
                return -EIO;
        }
        pr_debug("FUSB  %s - Device check passed!\n", __func__);

        /* Initialize the platform's GPIO pins and IRQ */
        ret = fusb_InitializeGPIO();
        if (ret)
        {
                dev_err(&client->dev, "FUSB  %s - Error: Unable to initialize GPIO!\n", __func__);
                return ret;
        }
        pr_debug("FUSB  %s - GPIO initialized!\n", __func__);

        /* Initialize our timer */
        fusb_InitializeTimer();
        pr_debug("FUSB  %s - Timers initialized!\n", __func__);

#ifdef FSC_DEBUG
        /* Initialize debug sysfs file accessors */
        fusb_Sysfs_Init();
        pr_debug("FUSB  %s - Sysfs device file created!\n", __func__);
#endif // FSC_DEBUG

#ifdef FSC_INTERRUPT_TRIGGERED
    /* Enable interrupts after successful core/GPIO initialization */
    ret = fusb_EnableInterrupts();
    if (ret)
    {
        dev_err(&client->dev, "FUSB  %s - Error: Unable to enable interrupts! Error code: %d\n", __func__, ret);
        return -EIO;
    }

    /* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now)
    *  Interrupt must be enabled before starting 302 initialization */
    fusb_InitializeCore();
    pr_debug("FUSB  %s - Core is initialized!\n", __func__);
#else
    /* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now) */
    fusb_InitializeCore();
    pr_debug("FUSB  %s - Core is initialized!\n", __func__);

    /* Init our workers, but don't start them yet */
    fusb_InitializeWorkers();
    /* Start worker threads after successful initialization */
    fusb_ScheduleWork();
    pr_debug("FUSB  %s - Workers initialized and scheduled!\n", __func__);
#endif  // ifdef FSC_POLLING elif FSC_INTERRUPT_TRIGGERED

        dev_info(&client->dev, "FUSB  %s - FUSB30X Driver loaded successfully!\n", __func__);

        /* ASUS_BSP : make sure interrupt GPIO value after intilization +++ */
        //USB_FUSB302_331_INFO("[After_initilization] interrupt GPIO 58 value = %d\n",gpio_get_value_cansleep(58));
        /* ASUS_BSP : make sure interrupt GPIO value after intilization --- */
	/* for create proc */
	proc_ret = init_asus_for_fusb302();
	if (proc_ret) {
		USB_FUSB302_331_INFO("Unable to create proc init_asus_for_fusb302\n");
	}

	proc_ret = 0;
	proc_ret = init_asus_for_factory_fusb302();
	if (proc_ret) {
		USB_FUSB302_331_INFO("Unable to create proc init_asus_for_factory_fusb302\n");
	}

        proc_ret = 0;
        proc_ret = init_asus_for_PD_fusb302();
        if(proc_ret){
                USB_FUSB302_331_INFO("Unable to create proc init_asus_for_PD_fusb302\n");
        }

        proc_ret = 0;
        proc_ret = init_asus_for_i2c_fusb302();
        if(proc_ret){
                USB_FUSB302_331_INFO("Unable to create proc init_asus_for_i2c_fusb302\n");
        }

        proc_ret = 0;
        proc_ret = init_asus_for_usb_2_speed();
        if(proc_ret){
                USB_FUSB302_331_INFO("Unable to create proc init_asus_for_usb_2_speed\n");
        }

        proc_ret = 0;
        proc_ret = init_asus_for_usb_3_speed();
        if(proc_ret){
                USB_FUSB302_331_INFO("Unable to create proc init_asus_for_usb_3_speed\n");
        }

        proc_ret = 0;
        proc_ret = init_asus_typec_version();
        if(proc_ret){
                USB_FUSB302_331_INFO("Unable to create proc init_asus_typec_version\n");
        }

	return ret;
}

static int fusb30x_remove(struct i2c_client* client)
{
        pr_debug("FUSB  %s - Removing fusb30x device!\n", __func__);

#ifndef FSC_INTERRUPT_TRIGGERED // Polling mode by default
        fusb_StopThreads();
#endif  // !FSC_INTERRUPT_TRIGGERED

        fusb_StopTimers();
        fusb_GPIO_Cleanup();
        pr_debug("FUSB  %s - FUSB30x device removed from driver...\n", __func__);
        return 0;
}

/*******************************************************************************
 * Driver macros
 ******************************************************************************/
late_initcall(fusb30x_init);                                                    // Defines the module's delayed entrance function
module_exit(fusb30x_exit);                                                      // Defines the module's exit function

MODULE_LICENSE("GPL");                                                          // Exposed on call to modinfo
MODULE_DESCRIPTION("Fairchild FUSB30x Driver");                                 // Exposed on call to modinfo
MODULE_AUTHOR("Tim Bremm<tim.bremm@fairchildsemi.com>");                        // Exposed on call to modinfo
