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
#include <linux/usb/typec.h>
#include <linux/usb/class-dual-role.h>
#include <linux/completion.h>

/* Driver-specific includes */
#include "fusb30x_global.h"                                                     // Driver-specific structures/types
#include "platform_helpers.h"                                                   // I2C R/W, GPIO, misc, etc

#include "../core/fusb30X.h"

#include "../core/PD_Types.h"

static enum typec_attached_state fusb302_attatched_state_detect(void);

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
/* ASUS_BSP : variables/function for charger set result into PD +++ */
extern doDataObject_t   CapsSink[7];
extern sopMainHeader_t  CapsHeaderSink;
extern PolicyState_t    PolicyState;
extern FSC_BOOL         isVBUSOverVoltage(FSC_U8 vbusMDAC);
extern FSC_U32          SinkRequestMaxVoltage;
extern FSC_U32          SinkRequestMaxPower;
extern FSC_U32          SinkRequestOpPower;
/* ASUS_BSP : variables/function for charger set result into PD --- */
/* ASUS_BSP : refresh traffic monitor timer +++ */
union power_supply_propval refresh_timer = {0,};
/* ASUS_BSP : refresh traffic monitor timer --- */

//#define INIT_COMPLETION(x)      ((x).done = 0)
#ifdef FSC_DEBUG
#include "../core/core.h"                                                       // GetDeviceTypeCStatus
#endif // FSC_DEBUG

#include "fusb30x_driver.h"

/******************************************************************************
 *                       Create Proc  : For Factory --- Moisture              *
 ******************************************************************************/
ssize_t fusb302_moisture_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
        int len= 0;
        ssize_t ret=0;
        char *buff;
        bool result=false;

        USB_FUSB302_331_INFO("%s +++\n", __func__);
        buff = kmalloc(300,GFP_KERNEL);
        if(!buff)
                return -ENOMEM;

        result = platform_check_for_connector_fault();
        if(result)
                len += sprintf(buff + len, "Moisture = True(Moisture fault is found)\n");//Moisture fault condition is found
        else
                len += sprintf(buff + len, "Moisture = False(No Moisture fault)\n");//No Moisture fault condition

        ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
        kfree(buff);
        return ret;
}

int init_asus_moisture_fusb302(void)
{
        struct proc_dir_entry *entry=NULL;
        static struct file_operations fusb302_moisture_fop = {
                .read  = fusb302_moisture_read,
        };
        USB_FUSB302_331_INFO("init_asus_moisture_fusb302\n");
        entry = proc_create("fusb_moisture", 0666,NULL, &fusb302_moisture_fop);
        if(!entry){
                USB_FUSB302_331_INFO("create /proc/fusb_moisture fail\n");
        }

        return 0;
}

/******************************************************************************
*                        Create Proc  : For Factory --- I2C report            *
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
*                        Create Proc  : For Factory --- PD                    *
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
*                        Create Proc  : For Factory --- CC pin orientation    *
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
*                        Create Proc  : For Engineer                          *
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

        // Dump MUX GPIO 25 Details in adb shell
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
ssize_t fusb302_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
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
*                        Create Proc  : For USB Switch Speed Test(EE)         *
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
        /* ASUS_BSP : refresh traffic moni timer +++ */
        if(adb_val == 1)
                usb_psy->set_property(usb_psy,POWER_SUPPLY_PROP_REFRESH_TIMER, &refresh_timer);
        /* ASUS_BSP : refresh traffic moni timer --- */

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
*                        Create Proc  : For Type-C Driver Version             *
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

	len += sprintf(buff + len, "FUSB302 version = 3.3.2\n");

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
*                    Driver functions                                         *
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

/*******************************************************************************
 * Function:        fusb302_update_sink_capabilities
 * Input:           Pointer to an array of new Sink PD capabilities (50mV/10mA units)
 * Return:          None
 * Description:     The Charger/PMIC can call this to update the PD sink
 *                  capabilities.  It will update the number of available
 *                  PD profiles based on the number of non-zero voltage values.
 *                  This function will also request to re-negotiate the
 *                  PD contract with the attached source.
 * *******************************************************************************/
void fusb302_update_sink_capabilities(unsigned int * PDSinkCaps)
{
	FSC_U8 inMsgBuffer[64] = {0};
	FSC_U8 outMsgBuffer[64] = {0};
	FSC_U8 capsCnt = 0;
	FSC_U8 i, j;
	doDataObject_t	tempPDO = {0};
	sopMainHeader_t tempHeader = {0};

        inMsgBuffer[4] = 0x0B;				// 0x0B : update sink capabilities

        SinkRequestMaxVoltage = PDSinkCaps[0];                    // 50mV * PDSinkCaps[0]
        SinkRequestMaxPower   = PDSinkCaps[1]*PDSinkCaps[0];      // PDSinkCaps[1]*10mA*PDSinkCaps[0]*50mV
        SinkRequestOpPower = PDSinkCaps[1]*PDSinkCaps[0];         // Operating power the sink will request (sed to calculate current as well)
        USB_FUSB302_331_INFO("%s - Charger write back to PD : SinkRequestMaxVoltage = %dV , SinkRequestMaxPower = %dW , SinkRequestOpPower = %dW\n", __func__,SinkRequestMaxVoltage/20,SinkRequestMaxPower/2000,SinkRequestOpPower/2000);
        for (i = 0; i < 7; i++)
	{
		// PDSinkCaps[(i * 2)] = voltage in 50mV units
		// PDSinkCaps[(i * 2) + 1] = current in 10mA units
		tempPDO = CapsSink[i];							// copy current PDO
		tempPDO.FPDOSink.Voltage = PDSinkCaps[(i * 2)];				// update PDO voltage
		tempPDO.FPDOSink.OperationalCurrent = PDSinkCaps[(i * 2) + 1];		// update PDO current
		for (j = 0; j < 4; j++)                                         	// Loop through each byte of the object
			inMsgBuffer[7 + i + j] = tempPDO.byte[j];               	// Set the actual bytes
		if (PDSinkCaps[(i * 2)] > 0)
			capsCnt++;

		USB_FUSB302_331_INFO("%s - charger request %d : tempPDO.FPDOSink.Voltage = %d\n", __func__, i, tempPDO.FPDOSink.Voltage);
                USB_FUSB302_331_INFO("%s - charger request %d : tempPDO.FPDOSink.OperationalCurrent = %d\n", __func__, i, tempPDO.FPDOSink.OperationalCurrent);
	}

        tempHeader = CapsHeaderSink;			// copy the current CapsHeaderSink data
        tempHeader.NumDataObjects = capsCnt;		// update number of sink PDOs
        USB_FUSB302_331_INFO("%s - CapsHeaderSink.byte[0] : 0x%02x; CapsHeaderSink.byte[1] : 0x%02x\n", __func__, CapsHeaderSink.byte[0], CapsHeaderSink.byte[1]);
        //inMsgBuffer[5] = tempHeader.byte[0];
        //inMsgBuffer[6] = tempHeader.byte[1];
        inMsgBuffer[5] = 0x44;
        inMsgBuffer[6] = (FSC_U8)((capsCnt & 0x0F) << 4);

        core_process_typec_pd_control(inMsgBuffer, outMsgBuffer);
        PolicyState = peSinkGetSourceCap;		// get ready to re-negotiate PD contract
        isVBUSOverVoltage(VBUS_MDAC_15P96);		// this will trigger INT_N so that isr will run
}
EXPORT_SYMBOL(fusb302_update_sink_capabilities);
/* ASUS_BSP : power supply mode file operations +++ */
 /* Callback for "cat /sys/class/dual_role_usb/otg_default/<property>" */
static int fusb302_dual_role_get_local_prop(struct dual_role_phy_instance *dual_role,
					    enum dual_role_property prop,
					    unsigned int *val)
{
	struct fusb30x_chip* chip = dual_role_get_drvdata(dual_role);
	enum typec_attached_state attached_state;

	if (!chip)
	return -EINVAL;

	attached_state = fusb302_attatched_state_detect();

	/* TODO.
	 * Should we need to have modification if we are full PD capability? */
	if (attached_state == TYPEC_ATTACHED_AS_DFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_DFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SRC;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_HOST;
		else
			return -EINVAL;
	} else if (attached_state == TYPEC_ATTACHED_AS_UFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_UFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SNK;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_DEVICE;
		else
			return -EINVAL;
	} else {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_NONE;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_NONE;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_NONE;
		else
			return -EINVAL;
	}

	return 0;
}

/* 1. Check to see if current attached_state is same as requested state
 * if yes, then, return.
 * 2. Disonect current session
 * 3. Set approrpriate mode (dfp or ufp)
 * 4. wait for 1.5 secs to see if we get into the corresponding target state
 * if yes, return
 * 5. if not, fallback to Try.SNK
 * 6. wait for 1.5 secs to see if we get into one of the attached states
 * 7. return -EIO
 * Also we have to fallback to Try.SNK state machine on cable disconnect
 *
 * Callback for "echo <value> > /sys/class/dual_role_usb/otg_default/<property>"
 *    'prop' may be one of 'mode', 'data_role', 'power_role'
 *          DUAL_ROLE_PROP_MODE: 'val' could be
 *               0: DUAL_ROLE_PROP_MODE_UFP
 *               1: DUAL_ROLE_PROP_MODE_DFP
 *               2: DUAL_ROLE_PROP_MODE_NONE
 *               3: DUAL_ROLE_PROP_MODE_TOTAL (invalid)
 *
 *          DUAL_ROLE_PROP_PR: 'val' could be
 *               0: DUAL_ROLE_PROP_PR_SRC
 *               1: DUAL_ROLE_PROP_PR_SNK
 *               2: DUAL_ROLE_PROP_PR_NONE
 *               3: DUAL_ROLE_PROP_PR_TOTAL (invalid)
 *
 *          DUAL_ROLE_PROP_DR: 'val' could be
 *               0: DUAL_ROLE_PROP_DR_HOST
 *               1: DUAL_ROLE_PROP_DR_DEVICE
 *               2: DUAL_ROLE_PROP_DR_NONE
 *               3: DUAL_ROLE_PROP_DR_TOTAL (invalid)
 */
#define DUAL_ROLE_SET_MODE_WAIT_MS 3000
static int fusb302_dual_role_set_mode_prop(struct dual_role_phy_instance *dual_role,
				   enum dual_role_property prop,
				   const unsigned int *val)
{
	struct fusb30x_chip* chip = dual_role_get_drvdata(dual_role);
	enum typec_attached_state attached_state = TYPEC_NOT_ATTACHED;
	int timeout = 0;
	int ret = 0;

	if (!chip)
		return -EINVAL;

	if (prop != DUAL_ROLE_PROP_MODE) {
		pr_err("unsupport prop setter - prop:%d with value:%d\n", prop, *val);
		return -EINVAL;
	}
	if (*val != DUAL_ROLE_PROP_MODE_DFP && *val != DUAL_ROLE_PROP_MODE_UFP)
		return -EINVAL;

	attached_state = fusb302_attatched_state_detect();

	if (attached_state != TYPEC_ATTACHED_AS_DFP
	    && attached_state != TYPEC_ATTACHED_AS_UFP)
		return 0;	/* NOP if attached state is unattached or accessory */

	if (attached_state == TYPEC_ATTACHED_AS_DFP
	    && *val == DUAL_ROLE_PROP_MODE_DFP)
		return 0;	/* ignore due to current attached_state is same as requested state */

	if (attached_state == TYPEC_ATTACHED_AS_UFP
	    && *val == DUAL_ROLE_PROP_MODE_UFP)
		return 0;	/* ignore due to current attached_state is same as requested state */

	pr_info("%s: start\n", __func__);

	/* AS DFP now, try reversing, form Source to Sink */
	if (attached_state == TYPEC_ATTACHED_AS_DFP) {

		pr_err("%s: try reversing, form Source to Sink\n", __func__);
		fusb_enable(false, FUSB_EN_F_HOST_IRQ|FUSB_EN_F_MASK_IRQ|FUSB_EN_F_SM,
			    TYPEC_MODE_ACCORDING_TO_PROT);
		//fusb_vbus_off();
		power_supply_set_usb_otg(usb_psy,0);
		power_supply_set_usb_otg(usb_parallel_psy,0);
		fusb_disabled_state_enter();
		chip->reverse_state = REVERSE_ATTEMPT;
		fusb_enable(true, FUSB_EN_F_ALL, TYPEC_UFP_MODE);
	}
	/* AS UFP now, try reversing, form Source to Sink */
	else if (attached_state == TYPEC_ATTACHED_AS_UFP) {

		pr_err("%s: try reversing, form Sink to Source\n", __func__);
		fusb_enable(false, FUSB_EN_F_HOST_IRQ|FUSB_EN_F_MASK_IRQ|FUSB_EN_F_SM,
			    TYPEC_MODE_ACCORDING_TO_PROT);
		//fusb_vbus_off();
		power_supply_set_usb_otg(usb_psy,0);
		power_supply_set_usb_otg(usb_parallel_psy,0);
		fusb_disabled_state_enter();
		chip->reverse_state = REVERSE_ATTEMPT;
		fusb_enable(true, FUSB_EN_F_ALL, TYPEC_DFP_MODE);
	} else {
		pr_err("%s: attached state is not ether ATTACHED_AS_DFP or ATTACHED_AS_UFP, but got %d\n",
		       __func__, attached_state);
	}

	INIT_COMPLETION(chip->reverse_completion);
	timeout =
	    wait_for_completion_timeout(&chip->reverse_completion,
					msecs_to_jiffies
					(DUAL_ROLE_SET_MODE_WAIT_MS));
	if (!timeout) {
		/* If falling back to here, disable everything and reinitialie them within DRP */
		fusb_enable(false, FUSB_EN_F_HOST_IRQ|FUSB_EN_F_MASK_IRQ|FUSB_EN_F_SM,
			    TYPEC_MODE_ACCORDING_TO_PROT);
		pr_err("%s: reverse failed, set mode to DRP\n", __func__);
		chip->reverse_state = 0;
		fusb_reinitialize(TYPEC_DRP_MODE); /* to DRP mode */
		INIT_COMPLETION(chip->reverse_completion);
		wait_for_completion_timeout(&chip->reverse_completion,
					    msecs_to_jiffies
					    (DUAL_ROLE_SET_MODE_WAIT_MS));

		ret = -EIO;
	}
        else {
                /* reverse complete, set DRP mode */
                fusb_revert_to_drp_mode();
        }
	pr_err("%s: end ret = %d\n", __func__, ret);

	return ret;
}

/* Callback for "echo <value> >
 *                      /sys/class/dual_role_usb/<name>/<property>"
 * Block until the entire final state is reached.
 * Blocking is one of the better ways to signal when the operation
 * is done.
 * This function tries to switch to Attached.SRC or Attached.SNK
 * by forcing the mode into SRC or SNK.
 * On failure, we fall back to Try.SNK state machine.
 */
static int fusb302_dual_role_set_prop(struct dual_role_phy_instance *dual_role,
				      enum dual_role_property prop,
				      const unsigned int *val)
{
	if (prop == DUAL_ROLE_PROP_MODE)
		return fusb302_dual_role_set_mode_prop(dual_role, prop, val);
	else
		/* TODO.
		 * Implement PR_swap and DR_swap here. */
		return -EINVAL;
}

static enum dual_role_property fusb302_drp_properties[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
};

/* Decides whether userspace can change a specific property */
static int fusb302_dual_role_is_writeable(struct dual_role_phy_instance *drp,
					  enum dual_role_property prop)
{
	/* TODO.
	 * MODE switch only. Because DP is under developing */
	if (prop == DUAL_ROLE_PROP_MODE)
		return 1;
	else
		return 0;
}

static enum typec_current_mode fusb302_current_mode_detect(void)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return TYPEC_CURRENT_MODE_DEFAULT;
}

static enum typec_attached_state fusb302_attatched_state_detect(void)
{
	return fusb_get_connecting_state();
}

static enum typec_current_mode fusb302_current_advertise_get(void)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return TYPEC_CURRENT_MODE_DEFAULT;
}

static int fusb302_current_advertise_set(enum typec_current_mode current_mode)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return 0;
}
/* call from 'cat /sys/class/typec/typec_device/port_mode_ctrl'
 * Besides BSP/ATD tools, no one will access it. */
static enum typec_port_mode fusb302_port_mode_get(void)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return TYPEC_MODE_ACCORDING_TO_PROT;
}
/* call from 'echo 0|1|2|3 > /sys/class/typec/typec_device/port_mode_ctrl'
 *  0: TYPEC_MODE_ACCORDING_TO_PROT
 *  1: TYPEC_UFP_MODE
 *  2: TYPEC_DFP_MODE
 *  3: TYPEC_DRP_MODE
 * Besides BSP/ATD tools, no one will access it.
 * */
static int fusb302_port_mode_set(enum typec_port_mode port_mode)
{
	WARN(1, "unimplement foo - %s\n", __func__);
return 0;
}
/* call from 'cat > /sys/class/typec/typec_device/dump_regs'
 * Besides BSP/ATD tools, no one will access it.
 * */
static ssize_t fusb302_dump_regs(char *buf)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return 0;
}
/* call from 'cat > /sys/class/typec/typec_device/i2c_status'
 * Besides BSP/ATD tools, no one will access it.
 * */
//static ssize_t fusb302_i2c_status(char *buf)
static bool fusb302_i2c_status(void)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	//return 0;
        return false;
}
/* call from 'cat > /sys/class/typec/typec_device/cc_status'
 * Besides BSP/ATD tools, no one will access it.
 * */
//static ssize_t fusb302_cc_status(char *buf)
static int fusb302_cc_status(void)
{
	WARN(1, "unimplement foo - %s\n", __func__);
	return 0;
}

struct typec_device_ops fusb302_typec_ops = {
	.current_detect = fusb302_current_mode_detect,
	.attached_state_detect = fusb302_attatched_state_detect,
	.current_advertise_get = fusb302_current_advertise_get,
	.current_advertise_set = fusb302_current_advertise_set,
	.port_mode_get = fusb302_port_mode_get,
	.port_mode_set = fusb302_port_mode_set,
	.dump_regs = fusb302_dump_regs,
	.i2c_status = fusb302_i2c_status,
	.cc_status = fusb302_cc_status,
};

void register_typec_device(struct fusb30x_chip* chip)
{
	int ret = 0;
	struct dual_role_phy_desc *desc;
	struct dual_role_phy_instance *dual_role;

	chip->dev = &chip->client->dev;

	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		desc = devm_kzalloc(chip->dev, sizeof(struct dual_role_phy_desc),
				    GFP_KERNEL);
		if (!desc) {
			pr_err("unable to allocate dual role descriptor\n");
		return;
		}

		desc->name = "otg_default";
		desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
		desc->get_property = fusb302_dual_role_get_local_prop;
		desc->set_property = fusb302_dual_role_set_prop;
		desc->properties = fusb302_drp_properties;
		desc->num_properties = ARRAY_SIZE(fusb302_drp_properties);
		desc->property_is_writeable = fusb302_dual_role_is_writeable;
		dual_role =
		    devm_dual_role_instance_register(chip->dev, desc);
		dual_role->drv_data = chip;
		chip->dual_role = dual_role;
		chip->desc = desc;
	}

	ret = add_typec_device(chip->dev, &fusb302_typec_ops);
	if (ret < 0) {
		pr_err("%s: add_typec_device fail\n", __func__);
	}
}
/* ASUS_BSP : power supply mode file --- */
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
        USB_FUSB302_331_INFO("[USB] FUSB302 version = 3.3.2\n");
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

	init_completion(&chip->reverse_completion);
	
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
	register_typec_device(chip);
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

        proc_ret = 0;
        proc_ret = init_asus_moisture_fusb302();
        if(proc_ret){
                USB_FUSB302_331_INFO("Unable to create proc init_asus_moisture_fusb302\n");
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
