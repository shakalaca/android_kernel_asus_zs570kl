/*
 * This file is part of the AP3045 sensor driver.
 * AP3045 is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: John Huang <john.huang@dyna-image.com>
 * Contact: Leo Tsai <LeoHS.Tsai@dyna-image.com>
 * Contact: Vicky Lee <Vicky.lee@dyna-image.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3045.c
 *
 * Summary:
 *	AP3045 device driver.
 *
 * Modification History:
 * Date(MM/DD/YY)     By       Summary
 * -------- -------- -------------------------------------------------------
 * 09/24/15		Vicky      First release
 *
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/ap3045.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/hrtimer.h>
#include <linux/regulator/consumer.h>

#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#define INTEL_SOFIA
#define I2C_RETRY_DELAY 50
#define I2C_RETRIES 5
#define AP3045_DRV_NAME		"ap3045"

#define DRIVER_VERSION		"1"
#define CONFIG_ASUS_FACTORY_SENSOR_MODE  1

#define PL_TIMER_DELAY 200
/* POWER SUPPLY VOLTAGE RANGE */
#define AP3426_VDD_MIN_UV	2700000
#define AP3426_VDD_MAX_UV	3300000
//#define AP3426_VI2C_MIN_UV	1750000
//#define AP3426_VI2C_MAX_UV	1950000

#define LSC_DBG
#define Driverversion  "1.0.0"  //<ASUS-annacheng20150129+>
#define VENDOR  "AP3426"    
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk("LDBG: func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif
static void plsensor_work_handler(struct work_struct *w);
//static void pl_timer_callback(unsigned long pl_data);
static int ap3045_get_int_stat(struct i2c_client *client);
static unsigned int ap3045_als_autogain(struct i2c_client *client,unsigned int ADC);


static u8 ap3045_reg[] =
{0x00,0x01,0x02,0x06,0x07,0x08,0x0A,0x0C,0x0D,0x0F,0x10,0x1B,0x26,0x27,0x2C,0x2D,0x2E,0x2F,
    0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C};


struct ap3045_data {
    struct i2c_client *client;
    u8 reg_cache[(sizeof(ap3045_reg)/sizeof(ap3045_reg[0]))];//TO-DO
    u8 power_state_before_suspend;
    int irq;
    struct input_dev	*psensor_input_dev;
    struct input_dev	*lsensor_input_dev;
//    struct input_dev	*hsensor_input_dev;
    struct workqueue_struct *plsensor_wq;
    struct work_struct plsensor_work;
    struct workqueue_struct *psensor_wq;
    struct work_struct psensor_work;
    struct timer_list pl_timer;
    struct regulator *vdd;
    int ps_opened;
	int ls_opened;
	uint8_t status_calling;
    struct regulator *vio;
};

static struct ap3045_data *private_pl_data = NULL;
static int ap3045_power_set(struct ap3045_data * info, bool on);
 //<ASUS-annacheng20150129+>>>>+
struct proc_dir_entry *ap3045_lightsensor_entry = NULL;
struct proc_dir_entry *ap3045_proximitysensor_entry = NULL;
 //<ASUS-annacheng20150129+><<<<<+

// AP3045 range
static int ap3045_range[4] = {1,4,16,110};
static int ALS_Max_Count = 65535;
static int ALS_Gain_MapIndex = 0; 
static int ALS_Gain_MapCase[8] = {0,4,1,5,2,6,3,7};
//static u16 ap3045_threshole[8] = {28,444,625,888,1778,3555,7222,0xffff};
//static u16 ap3045_threshole[10] = {100,220,444,888,1788,3555,7222,19555,35555,0xffff};
static u8 *reg_array;
static int *range;
static int reg_num = 0;
static int als_data_ch0 = 0;
//static int hal_value = 0;

static int cali = 100;
static int misc_ps_opened = 0;
static int misc_ls_opened = 0;
static DECLARE_WORK(ap3045_irq_work, plsensor_work_handler);

#define ADD_TO_IDX(addr,idx)	{														\
    int i;												\
    for(i = 0; i < reg_num; i++)						\
    {													\
	if (addr == reg_array[i])						\
	{												\
	    idx = i;									\
	    break;										\
	}												\
    }													\
}

#ifdef INTEL_SOFIA
bool EnLSensorConfig_flag =0 ;
bool EnPSensorConfig_flag =0 ;
int lSensor_CALIDATA[2] = {0}; //input calibration data . Format : "200 lux -->lux value ; 1000 lux -->lux value"
int pSensor_CALIDATA[2] = {0}; //input calibration data . Format : "near 3cm :--> value ; far  5cm :--> value"
static s32 ap3045_i2c_read_byte_data(const struct i2c_client *client, u8 command)
{
	int err;
	int tries = 0;
         
        u8 buf[2] = {0,0};

	struct i2c_msg	msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags | I2C_M_NOSTART,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = client->flags | I2C_M_RD,
			.len = 1,
			.buf = buf,
		},
	};
#ifdef DEBUG
        //pr_info("%s; before transfer: len:%d; [0]=0x%x,[1]=0x%x,[2]=0x%x,[3]=0x%x",__func__,len,buf[0],buf[1],buf[2],buf[3]);
    
#endif
       msgs[0].buf[0] = command;
	do {
		err = i2c_transfer(client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));
#ifdef DEBUG
        //pr_info("%s; after transfer: err:%d; [0]=0x%x,[1]=0x%x,[2]=0x%x,[3]=0x%x",__func__,err,buf[0],buf[1],buf[2],buf[3]);
#endif
	if (err != 2) {
		dev_err(&client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = msgs[1].buf[0];
	}

	return err;
}
#else
#define ap3045_i2c_read_byte_data i2c_smbus_read_byte_data
#endif
/*
 * register access helpers
 */

static int __ap3045_read_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift)
{
    struct ap3045_data *data = i2c_get_clientdata(client);
    u8 idx = 0xff;

    ADD_TO_IDX(reg,idx);
	
	return (data->reg_cache[idx] & mask) >> shift;
}

static int __ap3045_write_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift, u8 val)
{
    struct ap3045_data *data = i2c_get_clientdata(client);
    int ret = 0;
    u8 tmp;
    u8 idx = 0xff;

    ADD_TO_IDX(reg,idx)
	if (idx >= reg_num)
	    return -EINVAL;

    tmp = data->reg_cache[idx];
    tmp &= ~mask;
    tmp |= val << shift;

    ret = i2c_smbus_write_byte_data(client, reg, tmp);
    if (!ret)
	data->reg_cache[idx] = tmp;

    return ret;
}

/*
 * internally used functions
 */

/* range = Gain */
static int ap3045_get_range(struct i2c_client *client)
{
    u8 idx = __ap3045_read_reg(client, AP3045_REG_ALS_GAIN_CONF,
	    AP3045_ALS_RANGE_MASK, AP3045_ALS_RANGE_SHIFT);
    
    ap3045_als_autogain(client,als_data_ch0);    	
   // LDBG("Vicky_ap3024_get_range = %d \n",idx);
    return idx;
    
}

static int ap3045_set_range(struct i2c_client *client, int range)
{
    return __ap3045_write_reg(client, AP3045_REG_ALS_GAIN_CONF,
	    AP3045_ALS_RANGE_MASK, AP3045_ALS_RANGE_SHIFT, range);
}


/* mode */
static int ap3045_get_mode(struct i2c_client *client)
{
    int ret;

    ret = __ap3045_read_reg(client, AP3045_REG_SYS_CONF,
	    AP3045_REG_SYS_CONF_MASK, AP3045_REG_SYS_CONF_SHIFT);
			LDBG("Vicky_ap3045_get_mode = %d..\n",ret);
    return ret;
}

static int ap3045_set_mode(struct i2c_client *client, int mode)
{
    int ret;
    if(mode == AP3045_SYS_PS_ENABLE) {
	misc_ps_opened = 1;
    } else if(mode == AP3045_SYS_ALS_ENABLE) {
	misc_ls_opened = 1;
    } else if(mode == AP3045_SYS_ALS_PS_ENABLE) {
	misc_ps_opened = 1;
	misc_ls_opened = 1;
    } else if(mode == AP3045_SYS_DEV_DOWN) {
	misc_ps_opened = 0;
	misc_ls_opened = 0;
    }



    ret = __ap3045_write_reg(client, AP3045_REG_SYS_CONF,
	    AP3045_REG_SYS_CONF_MASK, AP3045_REG_SYS_CONF_SHIFT, mode);
    pr_err("Vicky_ap3045_set_mode [AP3045_REG_SYS_CONF]= %d   [mode]= %d..\n",AP3045_REG_SYS_CONF,mode);
    pr_err("Vicky_ap3045_set_mode [misc_ps_opened]= %d   [misc_ls_opened]= %d..\n",misc_ps_opened,misc_ls_opened);
    return ret;
}

/* ALS low threshold */
static int ap3045_get_althres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3045_read_reg(client, AP3045_REG_ALS_THDL_L,
	    AP3045_REG_ALS_THDL_L_MASK, AP3045_REG_ALS_THDL_L_SHIFT);
    msb = __ap3045_read_reg(client, AP3045_REG_ALS_THDL_H,
	    AP3045_REG_ALS_THDL_H_MASK, AP3045_REG_ALS_THDL_H_SHIFT);
		LDBG("Vicky_ap3045_get_althres lsb= %d   lsb= %d\n",lsb,msb);
    return ((msb << 8) | lsb);
}

static int ap3045_set_althres(struct i2c_client *client, int val)
{

    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3045_REG_ALS_THDL_L_MASK;

    err = __ap3045_write_reg(client, AP3045_REG_ALS_THDL_L,
	    AP3045_REG_ALS_THDL_L_MASK, AP3045_REG_ALS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3045_write_reg(client, AP3045_REG_ALS_THDL_H,
	    AP3045_REG_ALS_THDL_H_MASK, AP3045_REG_ALS_THDL_H_SHIFT, msb);

    return err;
}

/* ALS high threshold */
static int ap3045_get_ahthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3045_read_reg(client, AP3045_REG_ALS_THDH_L,
	    AP3045_REG_ALS_THDH_L_MASK, AP3045_REG_ALS_THDH_L_SHIFT);
    msb = __ap3045_read_reg(client, AP3045_REG_ALS_THDH_H,
	    AP3045_REG_ALS_THDH_H_MASK, AP3045_REG_ALS_THDH_H_SHIFT);
	LDBG("Vicky_ap3045_get_ahthres lsb= %d   lsb= %d\n",lsb,msb);
    return ((msb << 8) | lsb);
}

static int ap3045_set_ahthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3045_REG_ALS_THDH_L_MASK;

    err = __ap3045_write_reg(client, AP3045_REG_ALS_THDH_L,
	    AP3045_REG_ALS_THDH_L_MASK, AP3045_REG_ALS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3045_write_reg(client, AP3045_REG_ALS_THDH_H,
	    AP3045_REG_ALS_THDH_H_MASK, AP3045_REG_ALS_THDH_H_SHIFT, msb);

    return err;
}

/* PX low threshold */
static int ap3045_get_plthres(struct i2c_client *client)
{
    int lsb, msb;

    lsb = __ap3045_read_reg(client, AP3045_REG_PS_THDL_L,
	    AP3045_REG_PS_THDL_L_MASK, AP3045_REG_PS_THDL_L_SHIFT);
    msb = __ap3045_read_reg(client, AP3045_REG_PS_THDL_H,
	    AP3045_REG_PS_THDL_H_MASK, AP3045_REG_PS_THDL_H_SHIFT);
	LDBG("Vicky_ap3045_get_plthres lsb= %d   lsb= %d\n",lsb,msb);
    return ((msb << 8) | lsb);
}

static int ap3045_set_plthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3045_REG_PS_THDL_L_MASK;

    err = __ap3045_write_reg(client, AP3045_REG_PS_THDL_L,
	    AP3045_REG_PS_THDL_L_MASK, AP3045_REG_PS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3045_write_reg(client, AP3045_REG_PS_THDL_H,
	    AP3045_REG_PS_THDL_H_MASK, AP3045_REG_PS_THDL_H_SHIFT, msb);

    return err;
}

/* PX high threshold */
static int ap3045_get_phthres(struct i2c_client *client)
{
    int lsb, msb;

    lsb = __ap3045_read_reg(client, AP3045_REG_PS_THDH_L,
	    AP3045_REG_PS_THDH_L_MASK, AP3045_REG_PS_THDH_L_SHIFT);
    msb = __ap3045_read_reg(client, AP3045_REG_PS_THDH_H,
	    AP3045_REG_PS_THDH_H_MASK, AP3045_REG_PS_THDH_H_SHIFT);
	LDBG("Vicky_ap3045_get_phthres lsb= %d   lsb= %d\n",lsb,msb);
    return ((msb << 8) | lsb);
}

static int ap3045_set_phthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3045_REG_PS_THDH_L_MASK;

    err = __ap3045_write_reg(client, AP3045_REG_PS_THDH_L,
	    AP3045_REG_PS_THDH_L_MASK, AP3045_REG_PS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3045_write_reg(client, AP3045_REG_PS_THDH_H,
	    AP3045_REG_PS_THDH_H_MASK, AP3045_REG_PS_THDH_H_SHIFT, msb);

    return err;
}


static int ap3045_set_crosstalk(struct i2c_client *client, int val)
{
/*
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_PS_CAL_L_MASK;
//	printk("%s:ASUS_PSENSOR SETCALI_DATA : msb :  %d ,lsb:  %d \n", 
//			__func__, msb,lsb);
    err = __ap3426_write_reg(client, AP3426_REG_PS_CAL_L,
	    AP3426_REG_PS_CAL_L_MASK, AP3426_REG_PS_CAL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_PS_CAL_H,
	    AP3426_REG_PS_CAL_H_MASK, AP3426_REG_PS_CAL_H_SHIFT, msb);
*/
    return 0;
}

/* ALS autogain */
static unsigned int ap3045_als_autogain(struct i2c_client *client,unsigned int ADC)
{
    int i;
    unsigned int HIGH_THRS,LOWS_THRS,r_Gain;
    unsigned char INT_Status;
	
    HIGH_THRS = ALS_Max_Count * 3 / 4;
    LOWS_THRS = ALS_Max_Count * 1 / 8;
    INT_Status = ap3045_get_int_stat(client);
    r_Gain = __ap3045_read_reg(client,AP3045_REG_ALS_GAIN_CONF,0x07,0x00);
    for(i=0;i<8;i++)
    {
	if(r_Gain == ALS_Gain_MapCase[i]){
	    ALS_Gain_MapIndex = i;
	    break;
	}
     }

 if ((ADC > HIGH_THRS) || ((INT_Status & AP3045_REG_SYS_INT_ERR_MASK) == AP3045_REG_SYS_INT_ERR_MASK)){
        if (ALS_Gain_MapIndex > 0){
            ALS_Gain_MapIndex--;
            __ap3045_write_reg(client, AP3045_REG_ALS_GAIN_CONF, 0x07, 0x00, ALS_Gain_MapCase[ALS_Gain_MapIndex]);
            LDBG("vicky_[autogain]ALS_Gain_MapIndex-- [%d]%d\n",ALS_Gain_MapIndex,ALS_Gain_MapCase[ALS_Gain_MapIndex]);
        }
    }else if ((ADC < LOWS_THRS) && ((INT_Status & AP3045_REG_SYS_INT_ERR_MASK) != AP3045_REG_SYS_INT_ERR_MASK)){
        if (ALS_Gain_MapIndex < 7){
            ALS_Gain_MapIndex++;
            __ap3045_write_reg(client, AP3045_REG_ALS_GAIN_CONF, 0x07, 0x00, ALS_Gain_MapCase[ALS_Gain_MapIndex]);
            LDBG("vicky_[autogain]ALS_Gain_MapIndex++ [%d]%d\n",ALS_Gain_MapIndex,ALS_Gain_MapCase[ALS_Gain_MapIndex]);
        }
    }
   //  __ap3045_write_reg(client, AP3045_REG_SYS_INT_STATUS,0xFF, 0x00, 0);
   return 1;
}



static int ap3045_get_adc_value(struct i2c_client *client)
{
    unsigned int lsb, msb;
    //int i=0;
    int als_lux;
	
#ifdef LSC_DBG
    unsigned int tmp,range;
#endif

    lsb = ap3045_i2c_read_byte_data(client, AP3045_REG_ALS_L_DATA_LOW);
	//LDBG("ALS val lsb =%d lux\n",lsb);
    if (lsb < 0) {
	return lsb;
    }

    msb = ap3045_i2c_read_byte_data(client, AP3045_REG_ALS_L_DATA_HIGH);
	//LDBG("ALS val msb =%d lux\n",msb);
    if (msb < 0)
	return msb;
#ifdef LSC_DBG
    range = ap3045_get_range(client);
    tmp = (((msb << 8) | lsb) * range) >> 16;
    tmp = tmp * cali / 100;
  //  LDBG("ALS val temp =%d lux\n",tmp);
#endif
    als_lux = msb << 8 | lsb;
  // LDBG("ALS anna test als_lux=%d lux  \n",als_lux);
    return als_lux;
}



static int ap3045_get_object(struct i2c_client *client)
{
    int val;

    val = ap3045_i2c_read_byte_data(client, AP3045_REG_SYS_INT_STATUS);
  //  LDBG("val=%x\n", val);
    val &= AP3045_OBJ_MASK;
    //LDBG("Vicky_ap3045_get_object val= %x\n",val);
    return val >> AP3045_OBJ_SHIFT;
}

static int ap3045_get_int_stat(struct i2c_client *client)
{
    int val;

    val = ap3045_i2c_read_byte_data(client, AP3045_REG_SYS_INT_STATUS);
    val &= AP3045_REG_SYS_INT_MASK;
  //  LDBG("Vicky_ap3045_get_int_stat= %d\n",val);
    return val >> AP3045_REG_SYS_INT_SHIFT;
}


static int ap3045_get_px_value(struct i2c_client *client)
{
    int lsb, msb;

	
    lsb = ap3045_i2c_read_byte_data(client, AP3045_REG_PS_DATA_LOW);

    if (lsb < 0) {
		return lsb;
    }

    //LDBG("%s, IR = %d\n", __func__, (u32)(lsb));
    msb = ap3045_i2c_read_byte_data(client, AP3045_REG_PS_DATA_HIGH);

    if (msb < 0)
	return msb;

    //LDBG("%s, IR = %d\n", __func__, (u32)(msb));

    return (u32)(((msb & AL3045_REG_PS_DATA_HIGH_MASK) << 8) | (lsb & AL3045_REG_PS_DATA_LOW_MASK));
}

//static void ap3045_change_ls_threshold(struct i2c_client *client);


//anna-

static int ap3045_lsensor_enable(struct i2c_client *client)
{
	//struct ap3045_data *data = i2c_get_clientdata(client);
    int ret = 0,mode;

	LDBG("Vicky_ap3045_lsensor_enable\n");
    mode = ap3045_get_mode(client);
    if((mode & AP3045_SYS_ALS_ENABLE) == 0){
		mode |= AP3045_SYS_ALS_ENABLE;
		ret = ap3045_set_mode(client,mode);
		LDBG("Vicky_ap3045_lsensor_enable_mode= %d \n",mode);
		LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
		//ret = mod_timer(&data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));
    }

    return ret;
}

static int ap3045_lsensor_disable(struct i2c_client *client)
{
    //struct ap3045_data *data = i2c_get_clientdata(client);
    int ret = 0,mode;
    
    LDBG("Vicky_ap3045_lsensor_disable\n");
    
    mode = ap3045_get_mode(client);
    if(mode & AP3045_SYS_ALS_ENABLE){
		mode &= ~AP3045_SYS_ALS_ENABLE;
		if(mode == AP3045_SYS_DEV_RESET)
	    	mode = 0;
		ret = ap3045_set_mode(client,mode);
    }
    
   // del_timer(&data->pl_timer);

    return ret;
}


static ssize_t  ap3045_lsensor_enable_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{

struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
	unsigned long val;
    int ret;
	    if ((kstrtol(buf, 10, &val) < 0) || (val > 1))
	return -EINVAL;

	if(val == 1) {
		ret = ap3045_lsensor_enable(data->client);		
	    } else {
	    	ret = ap3045_lsensor_disable(data->client);
	    }
	
    if (ret < 0)
	return ret;

    return count;
/*
   struct input_dev *input = to_input_dev(dev);
    //struct i2c_client *client = to_i2c_client(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret = 0;

    if ((kstrtol(buf, 10, &val) < 0) || (val > 1))
		return -EINVAL;
	
    if(val == 1) {
		ret = ap3045_lsensor_enable(data->client);		
    } else {
    	ret = ap3045_lsensor_disable(data->client);
    }
	
    LDBG("ret = %d\n", ret); */
    return count;
}

static int lsensor_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	printk("[LS][AP3426] %s\n", __func__);
	
	if (private_pl_data -> ls_opened) {
		pr_err("[LS][AP3045 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}

	private_pl_data -> ls_opened = 1;
	return rc;
}

static int lsensor_release(struct inode *inode, struct file *file)
{
	printk("[LS][AP3045] %s\n", __func__);
	private_pl_data -> ls_opened = 0;
	return 0;
}
static ssize_t  ap3045_lsensor_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    //struct i2c_client *client = to_i2c_client(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    //unsigned long val;
    //int ret;
    int enabled = 0;
    int mode = ap3045_get_mode(data->client);
    if((mode & AP3045_SYS_ALS_ENABLE) == AP3045_SYS_ALS_ENABLE)
          enabled = 1;
    else enabled = 0;
    
    return sprintf(buf, "%i\n", enabled);
}

static DEVICE_ATTR(lsensor_enable, S_IWUSR | S_IRUGO, ap3045_lsensor_enable_show, ap3045_lsensor_enable_store);
//static DEVICE_ATTR(lsensor_enable, S_IWUSR | S_IRUGO, ap3045_lsensor_enable_show, NULL);

//<asus-annacheng20150129>>>>>>>>>>>>>>>+

static int ap3045_lightsensor_proc_show(struct seq_file *m, void *v) {
	
	//if(!lightsensor_entry){
		int ret = 0;
		int idreg;
		//uint8_t i;				
		
		struct ap3045_data *lpi = private_pl_data;   
		 idreg = i2c_smbus_read_byte_data(lpi->client, 0x04);//0x04 spec not found

		printk("anna-sensor idreg : 0x04=0x%d \n", idreg);
	      
	  	if(idreg<0){
			ret =seq_printf(m," ERROR: i2c r/w test fail\n");	
		}else{
			ret =seq_printf(m," ACK: i2c r/w test ok\n");	
		}
		if(Driverversion != NULL){
			ret =seq_printf(m," Driver version:%s\n",Driverversion);
		}
		else{
			ret =seq_printf(m," Driver version:NULL\n");
		}
	  	if(idreg==0x0B){
	  		ret =seq_printf(m," Vendor:%s(%d)\n", VENDOR,idreg);
		}else{
			ret =seq_printf(m," Vendor:%s\n", VENDOR);
		}

		 idreg = i2c_smbus_read_byte_data(lpi->client, 0x04);
		if(idreg<0){	  		
			ret =seq_printf(m," Device status:error\n");
		}else{
			ret =seq_printf(m," Device status:ok\n");			
		}
	     	    
   	return ret;

}

static int ap3045_Proximitysensor_proc_show(struct seq_file *m, void *v) {
	
	//if(!lightsensor_entry){
		int ret = 0;
		int idreg;
		//uint8_t i;				
		
	   	struct ap3045_data *lpi = private_pl_data;   
		 idreg = ap3045_i2c_read_byte_data(lpi->client, 0x04);

		printk("anna-sensor idreg : 0x0d=0x%d \n", idreg);
	      
	  	if(idreg<0){
			ret =seq_printf(m," ERROR: i2c r/w test fail\n");	
		}else{
			ret =seq_printf(m," ACK: i2c r/w test ok\n");	
		}
		if(Driverversion != NULL){
			ret =seq_printf(m," Driver version:%s\n",Driverversion);
		}
		else{
			ret =seq_printf(m," Driver version:NULL\n");
		}
	  	if(idreg==0x0B){
	  		ret =seq_printf(m," Vendor:%s(%d)\n", VENDOR,idreg);
		}else{
			ret =seq_printf(m," Vendor:%s\n", VENDOR);
		}

	      idreg = ap3045_i2c_read_byte_data(lpi->client, 0x04);

		if(idreg<0){	  		
			ret =seq_printf(m," Device status:error\n");
			//return seq_printf(m, " ERROR: i2c r/w test fail\n Driver version:1.0.0\n Vendor:0x0%x\n Device status error.\n",idReg);
		}else{
			ret =seq_printf(m," Device status:ok\n");			
		}
	     	    
   	return ret;

}

static int ap3045_lightsensor_proc_open(struct inode *inode, struct  file *file) {
  return single_open(file, ap3045_lightsensor_proc_show, NULL);
}

static const struct file_operations ap3045_lightsensor_proc_fops = {
  .owner = THIS_MODULE,
  .open = ap3045_lightsensor_proc_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

static int ap3045_Proximitysensor_proc_open(struct inode *inode, struct  file *file) {
  return single_open(file, ap3045_Proximitysensor_proc_show, NULL);
}

static const struct file_operations ap3045_Proximitysensor_proc_fops = {
  .owner = THIS_MODULE,
  .open = ap3045_Proximitysensor_proc_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

int create_ap3045_asusproc_lightsensor_status_entry(void){
	ap3045_lightsensor_entry = proc_create("lightsensor_status", S_IWUGO| S_IRUGO, NULL,&ap3045_lightsensor_proc_fops);
 	if (!ap3045_lightsensor_entry)
       		 return -ENOMEM;

    	return 0;
}

int create_ap3045_asusproc_Proximitysensor_status_entry(void){
	ap3045_proximitysensor_entry = proc_create("Proximitysensor_status", S_IWUGO| S_IRUGO, NULL,&ap3045_Proximitysensor_proc_fops);
	if (!ap3045_proximitysensor_entry)
       		 return -ENOMEM;

    	return 0;
}
//<asus-<asus-annacheng20150129>><<<<<<<<<<<<<+



//<-------ward_du------->
//<-- ASUS-Bevis_Chen - -->

//=========================================

//     Calibration Formula:

//     y = f(x) 

//  -> ax - by = constant_k

//     a is f(x2) - f(x1) , b is x2 - x1

////=========================================            

int static calibration_light_ap3045(int x_big, int x_small, int report_lux){        

                    int y_big = 1000;
                    int y_small = 200;
                    int constant_k;

					    if (x_small == 1) {
					        x_small = 0;
					        y_small = 0;
					    }
			 constant_k = (y_big - y_small)*x_small - (x_big - x_small)*y_small;
			 if ( report_lux*(y_big - y_small) < constant_k){
                              return 0; 
                        }else {   
                             return ((report_lux*(y_big - y_small) - constant_k) / (x_big - x_small));			
                        }
}

static long lsensor_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int rc, val;
	char encalibration_flag = 0 ;
	 int adc_value = 0 ;
	uint16_t report_lux=0;	
	void __user *argp = (void __user *)arg;
	/*D("[ap3045] %s cmd %d\n", __func__, _IOC_NR(cmd));*/
	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		printk("[LS][AP045] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
			__func__, val);

		rc = val ? ap3045_lsensor_enable(private_pl_data -> client) : ap3045_lsensor_disable(private_pl_data -> client);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = misc_ls_opened;
	   	printk("[LS][AP3045] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
			__func__, val);

		rc = put_user(val, (unsigned long __user *)arg);
		break;
	//<----------- ASUS-Bevis_Chen + --------------->
	/*case ASUS_LIGHTSENSOR_IOCTL_START:	
	        printk("%s:ASUS ASUS_LIGHTSENSOR_IOCTL_START  \n", __func__);
	        break;
	case ASUS_LIGHTSENSOR_IOCTL_CLOSE:				
	 	  printk("%s:ASUS ASUS_LIGHTSENSOR_IOCTL_CLOSE \n", __func__);
	        break;*/
	case ASUS_LIGHTSENSOR_IOCTL_GETDATA:
        	printk("%s:ASUS ASUS_LIGHTSENSOR_IOCTL_GETDATA \n", __func__);
		rc = 0 ;
		
       		 adc_value = ap3045_get_adc_value(private_pl_data -> client);

		 report_lux = (uint16_t)adc_value;		
		if(EnLSensorConfig_flag == 1 ){//calibration enable 
			if(lSensor_CALIDATA[0] > 0&&lSensor_CALIDATA[1] > 0 
  	        	&&lSensor_CALIDATA[1] >lSensor_CALIDATA[0] ){ //in case of zero divisor error
                            
            	printk("AP3045---Before calibration, ASUS_LIGHTSENSOR_IOCTL_GETDATA,  report_lux is %d\n", report_lux);
				//report_lux = calibration_light_ap3045(lSensor_CALIDATA[1], lSensor_CALIDATA[0], report_lux * 100 / ( 6553600/ap3045_get_range(private_pl_data -> client)));
				report_lux = calibration_light_ap3045(lSensor_CALIDATA[1], lSensor_CALIDATA[0], report_lux);
				report_lux =report_lux * 100 / ( 6553600/ap3045_get_range(private_pl_data -> client));//sensitivity =0.1309 
				printk("AP3045---After calibration, ASUS_LIGHTSENSOR_IOCTL_GETDATA, report_lux is %d\n", report_lux);
            }else{
				rc = -EINVAL;
				printk("%s:ASUS input lSensor_CALIDATA was invalid .error !!!!!\n",__func__);
			}
		}else{
#ifndef CONFIG_ASUS_FACTORY_SENSOR_MODE // in user build and no calibration data			   
        		report_lux = report_lux*7;
			printk("AP3045--- NO calibration data, use default config\n"); 
#else
	        printk("AP3045--- NO calibration data, Factory branch , NO default config\n"); 	
#endif // end of CONFIG_ASUS_FACTORY_SENSOR_MODE				
			report_lux = report_lux * 100 / ( 6553600/ap3045_get_range(private_pl_data -> client));//sensitivity =0.1309  
			printk("AP3045---After convertion to lux, ASUS_LIGHTSENSOR_IOCTL_GETDATA, report_lux is %d\n", report_lux);
		}

        if ( copy_to_user(argp, &report_lux, sizeof(report_lux) ) ) {
        	printk("%s:ASUS failed to copy lightsense data to user space.\n",__func__);
            rc = -EFAULT;			
            goto end;
	    }		
		printk("%s:ASUS_LIGHTSENSOR_IOCTL_GETDATA end\n", __func__);
		break;
	case ASUS_LIGHTSENSOR_SETCALI_DATA:

		printk("%s:ASUS ASUS_LIGHTSENSOR_SETCALI_DATA \n", __func__);
		rc = 0 ;
		memset(lSensor_CALIDATA, 0, 2*sizeof(int));

		if (copy_from_user(lSensor_CALIDATA, argp, sizeof(lSensor_CALIDATA)))
		{
			rc = -EFAULT;
			goto end;
		}	

		printk("%s:ASUS_LIGHTSENSOR SETCALI_DATA : lSensor_CALIDATA[0] :  %d ,lSensor_CALIDATA[1]:  %d \n", 
			__func__, lSensor_CALIDATA[0],lSensor_CALIDATA[1]);

		if(lSensor_CALIDATA[0] <= 0||lSensor_CALIDATA[1] <= 0 
			||lSensor_CALIDATA[0] >= lSensor_CALIDATA[1] )
			rc =  -EINVAL;


		break;
	case ASUS_LIGHTSENSOR_EN_CALIBRATION:
		printk("%s:ASUS ASUS_LIGHTSENSOR_EN_CALIBRATION \n", __func__);
		rc = 0 ;
		if (copy_from_user(&encalibration_flag , argp, sizeof(encalibration_flag )))
		{
			rc = -EFAULT;
			goto end;
		}	
		EnLSensorConfig_flag =  encalibration_flag ;

		printk("%s: ASUS_LIGHTSENSOR_EN_CALIBRATION : EnLSensorConfig_flag is : %d  \n",__func__,EnLSensorConfig_flag); 
		break;		
	//<----------- ASUS-Bevis_Chen - ------------->

	default:
		pr_err("[LS][AP3045 error]%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

end:
	
    return rc;
}


static const struct file_operations lsensor_fops = {
	.owner = THIS_MODULE,
	.open = lsensor_open,
	.release = lsensor_release,
	.unlocked_ioctl = lsensor_ioctl
};

static struct miscdevice lsensor_misc_ap3045 = {
	.minor = AP3045_MINOR,
	.name = "lightsensor",
	.fops = &lsensor_fops
};
static int ap3045_register_lsensor_device(struct i2c_client *client, struct ap3045_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device lsensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for lsensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->lsensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "ASUS Lightsensor";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_MISC, 0, 8, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for lsensor\n", __FUNCTION__);
	goto done;
    }

	rc = misc_register(&lsensor_misc_ap3045);

	if (rc < 0) {
		pr_err( "[PS][AP3045 error]%s: could not register ps misc device\n", __func__);
	goto done;
    }
done:
    return rc;
}


static void ap3045_unregister_lsensor_device(struct i2c_client *client, struct ap3045_data *data)
{
    input_unregister_device(data->lsensor_input_dev);
}


static void ap3045_change_ls_threshold(struct i2c_client *client)
{
    struct ap3045_data *data = i2c_get_clientdata(client);
    int value;
	//int i,lowTH,highTH,temp=10;
    value = ap3045_get_adc_value(client);

//    printk("AP3045--- lux =%d\n",value); 
/*
	if(value > 100){

		for (i=1;i<10;i++)
		{
			if(value <=ap3045_threshole[i])break;
		}
		temp = temp *i;
	
		if(value >(0xffff -(9*temp*3)) ){
			highTH = 0xffff ;
			lowTH = value -temp*3;
		}else{
			if(i ==8 ){
				//printk( "[	LS]anna adc  temp=%d \n",temp);
				temp =  temp*2 ;//need big 
			}else if(i ==9){
				temp =  temp*3;
			}
			
			highTH = value + temp;
			lowTH = value -temp;
			
			if(lowTH < 100 ){  
				lowTH = 100;
			}
		}
		
    }
    else if((value>40)&&(value <= 100)){

		lowTH  = value -5;
		highTH = value+ 5;
		if(lowTH<40) lowTH = 40;
		if(highTH > 100)highTH =100;
    }else{
		if(value <= 1){
			lowTH = 0;
		}else{
			lowTH  = value -1;
		}
		highTH =value+ 1;
	}
	ap3045_set_althres(client,lowTH);
       ap3045_set_ahthres(client,highTH);
	if(EnLSensorConfig_flag == 1 ){//calibration enable 
		if(lSensor_CALIDATA[0] > 0&&lSensor_CALIDATA[1] > 0 
  	        	&&lSensor_CALIDATA[1] >lSensor_CALIDATA[0] ){ //in case of zero divisor error
		//pr_err( "[LS]anna LUX  value=%d \n", value * 100 / ( 6553600/ap3045_get_range(private_pl_data -> client)));
           	 value = calibration_light_ap3045(lSensor_CALIDATA[1], lSensor_CALIDATA[0], value);
		//pr_err( "[	LS]anna EnLSensorConfig_flag  LUX value=%d \n",value);
		 value =value * 100 / ( 6553600/ap3045_get_range(private_pl_data -> client));//sensitivity =0.1309  

        	}else{
		}
	}else{
#ifndef CONFIG_ASUS_FACTORY_SENSOR_MODE // in user build and no calibration data			   
//        value = value*15;
//		printk("AP3045--- NO calibration data, use default config\n"); 
#else
//	    printk("AP3045--- NO calibration data, Factory branch , NO default config\n"); 	
#endif 			
		value = value * 100 / ( 6553600/ap3045_get_range(private_pl_data -> client))*(2000/30)/10;
		//pr_err( "[	LS]anna EnLSensorConfig_flag=0  value=%d \n",value);

	}
*/
		if (value % 2 == 0)
			value += 1;

  input_report_abs(data->lsensor_input_dev, ABS_MISC, value);
  //  input_report_abs(data->lsensor_input_dev, ABS_MISC, hal_value);
 // pr_err("%s:anna test repoet ,value = %d \n", __FUNCTION__,value);
    input_sync(data->lsensor_input_dev);
//	pr_err("%s:anna test repoet444 \n", __FUNCTION__);
}

//anna-

static int ap3045_psensor_enable(struct i2c_client *client)
{
    int ret = 0,mode;
    LDBG("Vicky_ap3045_psensor_enable\n");
    mode = ap3045_get_mode(client);
    if((mode & AP3045_SYS_PS_ENABLE) == 0){
	mode |= AP3045_SYS_PS_ENABLE;
	ret = ap3045_set_mode(client,mode);
	 LDBG("Vicky_ap3045_psensor_enable_mode= %d \n",mode);
		//enable_irq(client->irq);
    }

    return ret;
}

static int ap3045_psensor_disable(struct i2c_client *client)
{
    int ret = 0,mode;
    LDBG("Vicky_ap3045_psensor_disable\n");
    mode = ap3045_get_mode(client);
    if(mode & AP3045_SYS_PS_ENABLE){
	mode &= ~AP3045_SYS_PS_ENABLE;
	if(mode == AP3045_SYS_DEV_RESET)
	    mode = AP3045_SYS_DEV_DOWN;
	ret = ap3045_set_mode(client,mode);
    }
    return ret;
}

static int psensor_open(struct inode *inode, struct file *file)
{
	printk("[PS][AP3045] %s\n", __func__);
	if (private_pl_data -> ps_opened)
		return -EBUSY;

	private_pl_data -> ps_opened = 1;
	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	printk("[PS][AP3045] %s\n", __func__);
	private_pl_data -> ps_opened = 0;

//	return ap3426_psensor_disable(private_pl_data -> client);
	return 0;
}
static ssize_t  ap3045_psensor_enable_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
   struct ap3045_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret = 0;

    if ((kstrtol(buf, 10, &val) < 0) || (val > 1))
	return -EINVAL;
    if(val == 1)
       ret = ap3045_psensor_enable(data->client);
    else ret = ap3045_psensor_disable(data->client);
  //ret = (ret == 0)?count:ret;
  	if (ret < 0) return ret;
    return count;
}

static ssize_t ap3045_psensor_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
   // struct i2c_client *client = to_i2c_client(dev);
    struct ap3045_data *data = input_get_drvdata(input);
   // unsigned long val;
  //  int ret;
    int enabled = 0;
    int mode = ap3045_get_mode(data->client);
    if((mode & AP3045_SYS_PS_ENABLE)  == AP3045_SYS_PS_ENABLE)
          enabled = 1;
    else enabled = 0;
    
	    LDBG("Vicky_ap3045_psensor_disable = %d \n",enabled);
    return sprintf(buf, "%i\n", enabled);
}

static DEVICE_ATTR(psensor_enable, S_IWUSR | S_IRUGO, ap3045_psensor_enable_show, ap3045_psensor_enable_store);


static long psensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val;
	int rc ;
	uint16_t px_value;
	int ret;
	char enPcalibration_flag = 0 ;
	void __user *argp = (void __user *)arg;
	printk("[PS][AP3045] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case PROXIMITYSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;

		if (val){
			return ap3045_psensor_enable(private_pl_data -> client);
		}else{
			return ap3045_psensor_disable(private_pl_data -> client);
		}

		break;
	case PROXIMITYSENSOR_IOCTL_GET_ENABLED:
		return put_user(misc_ps_opened, (unsigned long __user *)arg);
		break;
	case ASUS_PSENSOR_IOCTL_GETDATA:
		printk("%s:ASUS ASUS_PSENSOR_IOCTL_GETDATA \n", __func__);
		rc = 0 ;
		
		ret = ap3045_get_px_value(private_pl_data -> client);
		if (ret < 0) {
			printk("%s:ASUS failed to get_px_value. \n",__func__);
			rc = -EIO;
			goto pend;
		}

		px_value = (uint16_t)ret;
		if ( copy_to_user(argp, &px_value, sizeof(px_value) ) ) {
	     		printk("%s:ASUS failed to copy psense data to user space.\n",__func__);
			rc = -EFAULT;			
			goto pend;
	    	}		

		printk("%s:ASUS_PSENSOR_IOCTL_GETDATA end\n", __func__);
		break;
	case ASUS_PSENSOR_SETCALI_DATA:
		printk("%s:ASUS ASUS_PSENSOR_SETCALI_DATA \n", __func__);
		rc = 0 ;
		memset(pSensor_CALIDATA, 0, 2*sizeof(int));
		if (copy_from_user(pSensor_CALIDATA, argp, sizeof(pSensor_CALIDATA))){
			rc = -EFAULT;
			goto pend;
		}	

		printk("%s:ASUS_PSENSOR SETCALI_DATA : pSensor_CALIDATA[0] :  %d ,pSensor_CALIDATA[1]:  %d \n", 
			__func__, pSensor_CALIDATA[0],pSensor_CALIDATA[1]);

		if( (pSensor_CALIDATA[1] == 0) && ( pSensor_CALIDATA[0] >= 0  && pSensor_CALIDATA[0] < 512 ) ) {

			ap3045_set_plthres(private_pl_data -> client,0X0A);//5m
			ap3045_set_phthres(private_pl_data -> client,0X10);//3
			
			if (ap3045_set_crosstalk(private_pl_data -> client, pSensor_CALIDATA[0])) {
				rc = -EFAULT;
				pr_err("[PS][AP3045 error]%s: ap3045_set_plthres error\n", __func__);
				goto pend;
			}
			goto pend;
		}

		if(pSensor_CALIDATA[0] <= 0||pSensor_CALIDATA[1] <= 0 
			||pSensor_CALIDATA[0] <= pSensor_CALIDATA[1] ) {
			rc =  -EINVAL;
			goto pend;
		}

		if (ap3045_set_plthres(private_pl_data -> client, pSensor_CALIDATA[1])) {
			rc = -EFAULT;
			pr_err("[PS][AP3045 error]%s: ap3045_set_plthres error\n", __func__);
			goto pend;
		}
		
	 	if (ap3045_set_phthres(private_pl_data -> client, pSensor_CALIDATA[0])) {
			rc = -EFAULT;
			pr_err("[PS][AP3045 error]%s: ap3045_set_phthres error\n", __func__);
			goto pend;
		}
		break;

	case ASUS_PSENSOR_EN_CALIBRATION:
		printk("%s:ASUS ASUS_PSENSOR_EN_CALIBRATION \n", __func__);
		rc = 0 ;
		if (copy_from_user(&enPcalibration_flag , argp, sizeof(enPcalibration_flag ))){
			rc = -EFAULT;
			goto pend;
		}	
		EnPSensorConfig_flag =  enPcalibration_flag ;
		if(EnPSensorConfig_flag == 0){
			
			ap3045_set_crosstalk(private_pl_data -> client, EnPSensorConfig_flag);
		}
		
		printk("%s: ASUS_PSENSOR_EN_CALIBRATION : EnPSensorConfig_flag is : %d  \n",__func__,EnPSensorConfig_flag); 
		break;		
	//<----------- ASUS-Bevis_Chen - ------------->
	default:
		pr_err("[PS][AP3045 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		return -EINVAL;
	}

	pend:
 		return rc;
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

struct miscdevice psensor_misc_ap3045 = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cm3602",
	.fops = &psensor_fops
};
static int ap3045_register_psensor_device(struct i2c_client *client, struct ap3045_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device psensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for psensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->psensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "ASUS Proximitysensor";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for psensor\n", __FUNCTION__);
	goto done;
    }

    rc = misc_register(&psensor_misc_ap3045);

    if (rc < 0) {
    	pr_err( "[PS][AP3045 error]%s: could not register ps misc device\n", __func__);
 	goto done;
    }
    return 0;

done:
    return rc;
}

static void ap3045_unregister_psensor_device(struct i2c_client *client, struct ap3045_data *data)
{
    input_unregister_device(data->psensor_input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend ap3045_early_suspend;
static void ap3045_suspend(struct early_suspend *h)
{

    if (misc_ps_opened)
	ap3045_psensor_disable(private_pl_data -> client);
    if (misc_ls_opened)
	ap3045_lsensor_disable(private_pl_data -> client);
}

static void ap3045_resume(struct early_suspend *h)
{

    if (misc_ls_opened)
	ap3045_lsensor_enable(private_pl_data -> client);
    if (misc_ps_opened)
	ap3045_psensor_enable(private_pl_data -> client);
}
#endif


/* range */
static ssize_t ap3045_show_range(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
	
    return sprintf(buf, "%d\n", ap3045_get_range(data->client));
//    return sprintf(buf, "0x%x\n", ap3045_get_range(data->client));
}

static ssize_t ap3045_store_range(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;
    
    LDBG("Vicky_ap3045_store_range \n");
    
    if ((kstrtol(buf, 10, &val) < 0) || (val > 3))
	return -EINVAL;

    ret = ap3045_set_range(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(range, S_IWUSR | S_IRUGO, ap3045_show_range, ap3045_store_range);

/* mode */

static ssize_t ap3045_show_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    
    LDBG("Vicky_ap3045_show_mode \n");
    return sprintf(buf, "%d\n", ap3045_get_mode(data->client));
}


static ssize_t ap3045_store_mode(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{

    struct i2c_client *client = to_i2c_client(dev);
    struct ap3045_data *data = i2c_get_clientdata(client);
      unsigned long val;
      int ret;
	int lastmode;
	int pl_enabled;
	
 if ((kstrtol(buf, 10, &val) < 0) || (val % 10 > 7) || (val / 10 > 2))
    	{
		return -EINVAL;
    	}
	
	pl_enabled = val / 10;
	val %= 10;
	lastmode = ap3045_get_mode(data->client);
	pr_err("anna test stror mode =%ld \n",val);
        ret = ap3045_set_mode(data->client, val);
	pr_err("anna test stror mode =%ld \n",val);
    if (ret < 0)
	return ret;
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
   // ret = mod_timer(&data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");

    return count;
}


static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO,
	ap3045_show_mode, ap3045_store_mode);

//static DEVICE_ATTR(mode, S_IRUGO ,ap3045_show_mode, NULL);
/* lux */
static ssize_t ap3045_show_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	long int lux;
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    
    LDBG("Vicky_ap3045_show_lux \n");
    
    /* No LUX data if power down */
    if (ap3045_get_mode(data->client) == AP3045_SYS_DEV_DOWN)
		return sprintf((char*) buf, "%s\n", "Please power up first!");

	lux = ap3045_get_adc_value(data->client);

    return sprintf(buf, "%ld\n", lux);
}

static DEVICE_ATTR(lux, S_IRUGO, ap3045_show_lux, NULL);


/* Px data */
static ssize_t ap3045_show_pxvalue(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    
    /* No Px data if power down */
    if (ap3045_get_mode(data->client) == AP3045_SYS_DEV_DOWN)
	return -EBUSY;

    return sprintf(buf, "%d\n", ap3045_get_px_value(data->client));
}

static DEVICE_ATTR(pxvalue, S_IRUGO, ap3045_show_pxvalue, NULL);


/* proximity object detect */
static ssize_t ap3045_show_object(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3045_get_object(data->client));
}

static DEVICE_ATTR(object, S_IRUGO, ap3045_show_object, NULL);


/* ALS low threshold */
static ssize_t ap3045_show_althres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3045_get_althres(data->client));
}

static ssize_t ap3045_store_althres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (kstrtol(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3045_set_althres(data->client, val);
    
    LDBG("Vicky_ap3045_store_althres =%d \n",ret);	
    
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(althres, S_IWUSR | S_IRUGO,
	ap3045_show_althres, ap3045_store_althres);


/* ALS high threshold */
static ssize_t ap3045_show_ahthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3045_get_ahthres(data->client));
}

static ssize_t ap3045_store_ahthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (kstrtol(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3045_set_ahthres(data->client, val);
    
    LDBG("Vicky_ap3045_store_ahthres =%d \n",ret);	
    
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(ahthres, S_IWUSR | S_IRUGO,
	ap3045_show_ahthres, ap3045_store_ahthres);

/* Px low threshold */
static ssize_t ap3045_show_plthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3045_get_plthres(data->client));
}

static ssize_t ap3045_store_plthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (kstrtol(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3045_set_plthres(data->client, val);
    
    LDBG("Vicky_ap3045_store_plthres =%d \n",ret);	
    
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(plthres, S_IWUSR | S_IRUGO,
	ap3045_show_plthres, ap3045_store_plthres);

/* Px high threshold */
static ssize_t ap3045_show_phthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3045_get_phthres(data->client));
}

static ssize_t ap3045_store_phthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (kstrtol(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3045_set_phthres(data->client, val);
    
    LDBG("Vicky_ap3045_store_phthres =%d \n",ret);	
    
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(phthres, S_IWUSR | S_IRUGO,
	ap3045_show_phthres, ap3045_store_phthres);


/* calibration */
static ssize_t ap3045_show_calibration_state(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    return sprintf(buf, "%d\n", cali);
}

static ssize_t ap3045_store_calibration_state(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3045_data *data = input_get_drvdata(input);
    int stdls; int lux; 
    char tmp[10];

    LDBG("DEBUG ap3045_store_calibration_state..\n");

    /* No LUX data if not operational */
    if (ap3045_get_mode(data->client) == AP3045_SYS_DEV_DOWN)
    {
	printk("Please power up first!");
	return -EINVAL;
    }

    cali = 100;
    sscanf(buf, "%d %s", &stdls, tmp);

    if (!strncmp(tmp, "-setcv", 6))
    {
	cali = stdls;
	return -EBUSY;
    }

    if (stdls < 0)
    {
	printk("Std light source: [%d] < 0 !!!\nCheck again, please.\n\
		Set calibration factor to 100.\n", stdls);
	return -EBUSY;
    }

    lux = ap3045_get_adc_value(data->client);
    cali = stdls * 1000000 * 100 / lux;

    return -EBUSY;
}

static DEVICE_ATTR(calibration, S_IWUSR | S_IRUGO,
	ap3045_show_calibration_state, ap3045_store_calibration_state);

#ifdef LSC_DBG
/* engineer mode */
static ssize_t ap3045_em_read(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ap3045_data *data = i2c_get_clientdata(client);
    int i;
    u8 tmp;
	int tmp_index = 0;

    LDBG("DEBUG ap3045_em_read..\n");
	tmp_index = sprintf(buf,"[reg add]:[reg value]\n");
	reg_num = sizeof(ap3045_reg)/sizeof(ap3045_reg[0]);
	LDBG("DEBUG reg_num = %d\n",reg_num);

    for (i = 0; i < reg_num; i++) {
		tmp = ap3045_i2c_read_byte_data(data->client, reg_array[i]);
		tmp_index += sprintf(buf+tmp_index,"0x%02x:0x%02x\n",reg_array[i],tmp);
		LDBG("Reg[0x%x] Val[0x%x]\n", reg_array[i], tmp);
    }

	return tmp_index;
}

static ssize_t ap3045_em_write(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ap3045_data *data = i2c_get_clientdata(client);
    u32 addr,val,idx=0;
    int ret = 0;

    LDBG("DEBUG ap3045_em_write..\n");

    sscanf(buf, "%x%x", &addr, &val);

    printk("Write [%x] to Reg[%x]...\n",val,addr);

    ret = i2c_smbus_write_byte_data(data->client, addr, val);
    ADD_TO_IDX(addr,idx)
	if (!ret)
	    data->reg_cache[idx] = val;

    return count;
}
static DEVICE_ATTR(em, S_IWUSR |S_IRUGO,
	ap3045_em_read, ap3045_em_write);
#endif

static ssize_t als_ch0_data_read(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
  //  struct ap3045_data *data = i2c_get_clientdata(client);
   // int i;
    //u8 tmp;
 //   int tmp_index = 0;
    int lsb,msb,als_ch_data;
    
    LDBG("DEBUG als_ch0_data..\n");

	/* ch0 data */
	lsb = ap3045_i2c_read_byte_data(client, AP3045_REG_ALS_CH0_DATA_LOW);
	msb = ap3045_i2c_read_byte_data(client, AP3045_REG_ALS_CH0_DATA_HIGH);
	als_ch_data = msb<<8|lsb;
	als_data_ch0 = als_ch_data;
	LDBG("Vicky_als_ch0_data..[%d]\n",als_ch_data);

	return sprintf(buf, "%d\n", als_ch_data);
}
static DEVICE_ATTR(als_ch0_data, S_IWUSR |S_IRUGO,
	als_ch0_data_read, NULL);

static ssize_t als_ch1_data_read(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    //struct ap3045_data *data = i2c_get_clientdata(client);
    //int i;
    //u8 tmp;
	//int tmp_index = 0;
	int lsb,msb,als_ch_data;

	LDBG("DEBUG als_ch1_data..\n");

	/* ch1 data */
	lsb = ap3045_i2c_read_byte_data(client, AP3045_REG_ALS_CH1_DATA_LOW);
	msb = ap3045_i2c_read_byte_data(client, AP3045_REG_ALS_CH1_DATA_HIGH);
	als_ch_data = msb<<8|lsb;
	LDBG("Vicky_als_ch1_data..[%d]\n",als_ch_data);
	return sprintf(buf, "%d\n", als_ch_data);
}
static DEVICE_ATTR(als_ch1_data, S_IWUSR |S_IRUGO,
	als_ch1_data_read, NULL);

static struct attribute *ap3045_attributes[] = {
   &dev_attr_range.attr,
    &dev_attr_mode.attr,
    &dev_attr_lux.attr,
    &dev_attr_object.attr,
    &dev_attr_pxvalue.attr,
    &dev_attr_althres.attr,
    &dev_attr_ahthres.attr,
    &dev_attr_plthres.attr,
    &dev_attr_phthres.attr,
    &dev_attr_calibration.attr,
#ifdef LSC_DBG
    &dev_attr_em.attr,
#endif
   &dev_attr_lsensor_enable.attr,
    &dev_attr_psensor_enable.attr,
    &dev_attr_als_ch0_data.attr,
    &dev_attr_als_ch1_data.attr, 
    NULL
};

static const struct attribute_group ap3045_attr_group = {
    .attrs = ap3045_attributes,
};

/*******************************************************************************
* Function    :  ap3045_pls_reg_init
* Description :  set ap3045 registers
* Parameters  :  none
* Return      :  void
*******************************************************************************/
#define OF_PS_LOW_THRES		"intel,ps-low-thres"
#define OF_PS_HIGH_THRES	"intel,ps-hight-thres"
#define OF_ALS_Time	 	"intel,als-time"
#define	OF_ALS_PERSIS_TIME	"intel,als-persis-time"
#define	OF_INT_CTRL		"intel,int-ctrl"   //only PS INT
#define	OF_PS_GAIN		"intel,ps-gain"
#define	OF_PS_LED_CTRL		"intel,ps-led-control"
#define	OF_PS_Time		"intel,ps-time"
#define OF_ALS_GAIN		"intel,als-gain"
static int ap3045_pls_reg_init(struct i2c_client *client, struct device *dev)
{
	int ret=0;
	struct device_node *np = dev->of_node;
	int of_pro_value, of_pro_lsb, of_pro_msb;
	 unsigned int MaxCount;
	 
	 
	/*PORINT*/
	i2c_smbus_write_byte_data(client,0x01,0);	//vicky  2015/09/08


	/* PS_LOW_THRES 70 */
	if ( of_property_read_u32(np, OF_PS_LOW_THRES, &of_pro_value) < 0 ) {
		printk("%s, read intel,ps-low-thres fail\n",__func__);
		of_pro_lsb = 0x00;
		of_pro_msb = 0x00;	
	} else {
		of_pro_lsb = of_pro_value/5 & AP3045_REG_PS_THDL_L_MASK;
		of_pro_msb = (of_pro_value >> 8) & AP3045_REG_PS_THDL_H_MASK;
		LDBG("Vicky_ap3045_pls_reg_init[PS_LOW_THRES][lsb]=%d [msb]=%d\n",of_pro_lsb,of_pro_msb);
	}
	if(i2c_smbus_write_byte_data(client,/*AP3045_PX_LTHL*/AP3045_REG_PS_THDL_L ,of_pro_lsb)<0) {
		printk("%s: I2C Write PS_THDL_L Failed\n", __func__);
		ret = -1;}
	if(i2c_smbus_write_byte_data(client,/*AP3045_PX_LTHH*/AP3045_REG_PS_THDL_H ,of_pro_msb)<0) {
		printk("%s: I2C Write PS_THDL_H Failed\n", __func__);
		ret = -1;}


	/* PS_HIGH_THRES 80 */
	if ( of_property_read_u32(np, OF_PS_HIGH_THRES, &of_pro_value) < 0 ) {
		printk("%s, read intel,ps-high-thres fail\n",__func__);
		of_pro_lsb = 0xFF;
		of_pro_msb = 0x03;
	} else {
		of_pro_lsb = of_pro_value/5 & AP3045_REG_PS_THDH_L_MASK;
		of_pro_msb = (of_pro_value >> 8) & AP3045_REG_PS_THDH_H_MASK;
		LDBG("Vicky_ap3045_pls_reg_init[PS_HIGH_THRES][lsb]=%d [msb]=%d\n",of_pro_lsb,of_pro_msb);
	}
	if(i2c_smbus_write_byte_data(client,/*AP3045_PX_HTHL*/AP3045_REG_PS_THDH_L ,of_pro_lsb)<0) {
		printk("%s: I2C Write PS_THDH_L Failed\n", __func__);
		ret = -1;}
	if(i2c_smbus_write_byte_data(client,/*AP3045_PX_HTHH*/AP3045_REG_PS_THDH_H ,of_pro_msb)<0) {
		printk("%s: I2C Write PS_THDH_H Failed\n", __func__);
		ret = -1;}
		

	// ALS Time
	if ( of_property_read_u32(np, OF_ALS_Time, &of_pro_value) < 0 ) {
		printk("%s, read intel,als-time fail\n",__func__);
		of_pro_value = 0X00;
	}
		LDBG("Vicky_ap3045_pls_reg_init[ALS Time][value]=%d\n",of_pro_value);
	if (i2c_smbus_write_byte_data(client,AP3045_REG_ALS_TIME,of_pro_value)<0){
		printk("%s: I2C Write ALS_Time Failed\n", __func__);
		ret = -1;	
	}
		MaxCount = 1024 * ((int)of_pro_value + 1) -1;
    	if (MaxCount < 65535)
        	ALS_Max_Count = MaxCount;
    	else
        	ALS_Max_Count = 65535;
		

	// ALS Persistence	
	if ( of_property_read_u32(np, OF_ALS_PERSIS_TIME, &of_pro_value) < 0 ) {
		printk("%s, read intel,als-persis-time fail\n",__func__);
		of_pro_value = 0X01;
	}
		LDBG("Vicky_ap3045_pls_reg_init[ALS Persistence][value]=%d\n",of_pro_value);
	if (i2c_smbus_write_byte_data(client,/*AP3045_AIF_COMMAND*/AP3045_REG_ALS_PERSIS,of_pro_value)<0){
		printk("%s: I2C Write ALS_PERSIS Failed\n", __func__);
		ret = -1;	
	}


	// INT controll register, only ps INT
	if ( of_property_read_u32(np, OF_INT_CTRL, &of_pro_value) < 0 ) {
		printk("%s, read intel,int-ctrl fail\n",__func__);
		of_pro_value = 0X00;
	}
		LDBG("Vicky_ap3045_pls_reg_init[ps INT]=%d\n",of_pro_value);	
	if (i2c_smbus_write_byte_data(client,AP3045_REG_SYS_INT_CTRL,of_pro_value)<0){
		printk("%s: I2C Write SYS_INT_CTRL Failed\n", __func__);
		ret = -1;	
	}


	/*PS gain*/
	if ( of_property_read_u32(np, OF_PS_GAIN, &of_pro_value) < 0 ) {
		printk("%s, read intel,ps-gain fail\n",__func__);
		of_pro_value = 0X00;
	}
		LDBG("Vicky_ap3045_pls_reg_init[PS gain]=%d\n",of_pro_value);	
	if(i2c_smbus_write_byte_data(client,/*AP3045_PX_CONFIGURE*/AP3045_REG_PS_GAIN, of_pro_value)<0) {
		printk("%s: I2C Write PS_CONF Failed\n", __func__);
		ret = -1;
	}
	
	/*PS LED control*/  //vicky  2015/09/08
	if ( of_property_read_u32(np, OF_PS_LED_CTRL, &of_pro_value) < 0 ) {
		printk("%s, read intel,PS LED control fail\n",__func__);
		of_pro_value = 0X00;
	}
		LDBG("Vicky_ap3045_pls_reg_init[PS LED control]=%d\n",of_pro_value);	
	if(i2c_smbus_write_byte_data(client,AP3045_REG_PS_LED_CONTROL, of_pro_value)<0) {
		printk("%s: I2C Write PS LED control Failed\n", __func__);
		ret = -1;
	}
		
	/*PS Time*/		
	if ( of_property_read_u32(np, OF_PS_Time, &of_pro_value) < 0 ) {
		printk("%s, read intel,PS Time fail\n",__func__);
		of_pro_value = 0X00;
	}
		LDBG("Vicky_ap3045_pls_reg_init[PS Time]=%d\n",of_pro_value);	
	if(i2c_smbus_write_byte_data(client,AP3045_REG_PS_Time, of_pro_value)<0) {
		printk("%s: I2C Write PS Time Failed\n", __func__);
		ret = -1;
	}

	ap3045_set_althres(client,0); //anna test set
       ap3045_set_ahthres(client,1);


	/* ALS gain */
	if ( of_property_read_u32(np, OF_ALS_GAIN, &of_pro_value) < 0 ) {
		printk("%s, read intel,als-gain fail\n",__func__);
		of_pro_value = 0X00; 	// B2 axgain:0 x1 half, B1B0 again:00 x1
	}
	ap3045_set_range(client, of_pro_value);
	ap3045_set_mode(client, AP3045_SYS_DEV_DOWN);


#ifdef PLS_DEBUG
	ap3045_pls_reg_dump();
#endif
	
	return ret;
}



static int ap3045_power_set(struct ap3045_data *info, bool on)
{
	int rc;

	if (on) {
		info->vdd = regulator_get(&info->client->dev, "vdd");
		if (IS_ERR(info->vdd)) {
			rc = PTR_ERR(info->vdd);
			dev_err(&info->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			goto err_vdd_get;
		}

		if (regulator_count_voltages(info->vdd) > 0) {
			rc = regulator_set_voltage(info->vdd,
					AP3426_VDD_MIN_UV, AP3426_VDD_MAX_UV);
			if (rc) {
				dev_err(&info->client->dev,
					"Regulator set failed vdd rc=%d\n", rc);
				goto err_vdd_set_vtg;
			}
		}

		/*
		info->vio = regulator_get(&info->client->dev, "vio");
		if (IS_ERR(info->vio)) {
			rc = PTR_ERR(info->vio);
			dev_err(&info->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto err_vio_get;
		}

		if (regulator_count_voltages(info->vio) > 0) {
			rc = regulator_set_voltage(info->vio,
				AP3426_VI2C_MIN_UV, AP3426_VI2C_MAX_UV);
			if (rc) {
				dev_err(&info->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set_vtg;
			}
		}
*/
		rc = regulator_enable(info->vdd);
		if (rc) {
			dev_err(&info->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_ena;
		}
/*
		rc = regulator_enable(info->vio);
		if (rc) {
			dev_err(&info->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_ena;
		}
		*/

	} else {
		rc = regulator_disable(info->vdd);
		if (rc) {
			dev_err(&info->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(info->vdd) > 0)
			regulator_set_voltage(info->vdd, 0, AP3426_VDD_MAX_UV);

		regulator_put(info->vdd);

		/*rc = regulator_disable(info->vio);
		if (rc) {
			dev_err(&info->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(info->vio) > 0)
			regulator_set_voltage(info->vio, 0,
					AP3426_VI2C_MAX_UV);

		regulator_put(info->vio); */
	}

	dev_err(&info->client->dev,"ap3045 set power ok");
	return 0;

//err_vio_ena:
//	regulator_disable(info->vdd);
err_vdd_ena:
	regulator_disable(info->vdd);
//err_vio_set_vtg:
//	regulator_put(info->vio);
//err_vio_get:
//	if (regulator_count_voltages(info->vdd) > 0)
//		regulator_set_voltage(info->vdd, 0, AP3426_VDD_MAX_UV);
err_vdd_set_vtg:
	regulator_put(info->vdd);
err_vdd_get:
	return rc;
}


static int ap3045_init_client(struct i2c_client *client)
{
    struct ap3045_data *data = i2c_get_clientdata(client);
    int i;

    LDBG("DEBUG ap3045_init_client..\n");

	ap3045_pls_reg_init(client,&client->dev);

    /* read all the registers once to fill the cache.
     * if one of the reads fails, we consider the init failed */
    for (i = 0; i < reg_num; i++) {
		int v = ap3045_i2c_read_byte_data(client, reg_array[i]);
		if (v < 0)
	    	return -ENODEV;

		data->reg_cache[i] = v;
		printk("%s, reg_array[%d] = 0x%x, data->reg_cache[%d] = 0x%x\n",__func__,i,reg_array[i],i,data->reg_cache[i]);
    }
  	
    return 0;
}


void pl_timer_callback(unsigned long pl_data)
{
    struct ap3045_data *data;
    int ret =0;

    data = private_pl_data;
    queue_work(data->plsensor_wq, &data->plsensor_work);
   // LDBG("queue work");
    ret = mod_timer(&private_pl_data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");

}

/*
static void psensor_work_handler(struct work_struct *w)
{

    struct ap3045_data *data =
	container_of(w, struct ap3045_data, psensor_work);
    u8 int_stat;
    int pxvalue;
    int distance;
    int_stat = ap3045_get_int_stat(data->client);
    
    LDBG("%s, int_stat = 0x%x\n",__func__, int_stat);
    if (int_stat & AP3045_REG_SYS_INT_PMASK) {
    	distance = ap3045_get_object(data->client);
    	pxvalue = ap3045_get_px_value(data->client);
    	LDBG("PXVALUE=%d\n",pxvalue); 
    	LDBG("work handler, distance = %d\n", distance);
    	input_report_abs(data->psensor_input_dev, ABS_DISTANCE, distance);
    	input_sync(data->psensor_input_dev);
    }
	LDBG("Vicky_work_handler[end]\n");

    //enable_irq(data->client->irq);
    i2c_smbus_write_byte_data(data->client,0x01,0x00);

   if(i2c_smbus_write_byte_data(data->client,0x01,0x00)<0)
    {
	LDBG("**[Failed]** 0x01 -> 0x00 \n");	
    }
    distance = ap3045_get_object(data->client);
    input_report_abs(data->psensor_input_dev, ABS_DISTANCE, distance);
    input_sync(data->psensor_input_dev);
    LDBG("Vicky_work handler[end]=%d\n",distance);
	

} 
*/
static void plsensor_work_handler(struct work_struct *w)
{

  //  struct ap3045_data *data =container_of(w, struct ap3045_data, plsensor_work);
  	struct ap3045_data *data = private_pl_data;
    u8 int_stat;
    int pxvalue;
    int distance;
	
	//LDBG("%s\n",__func__);
    int_stat = ap3045_get_int_stat(data->client);
    // LDBG("Vicky_plsensor_work_handler..\n");
  //  LDBG("%s, int_stat = 0x%x\n",__func__, int_stat);
    if (int_stat & AP3045_REG_SYS_INT_PMASK) {
    	distance = ap3045_get_object(data->client);
    	pxvalue = ap3045_get_px_value(data->client);
    	//LDBG("PXVALUE=%d\n",pxvalue); 
    	//LDBG("work handler, distance = %d\n", distance);
    	if(distance == 0)
    	{
    		   	distance =1;
    	}else {
			distance = 0;
	}
    	input_report_abs(data->psensor_input_dev, ABS_DISTANCE, distance);
    	input_sync(data->psensor_input_dev);

	  //__ap3045_write_reg(data->client, AP3045_REG_SYS_INT_STATUS,0xff, 0x00, 0);
	//	ap3045_change_ls_threshold(data->client);
    }
	//als int
   if (int_stat & AP3045_REG_SYS_INT_AMASK){

	ap3045_change_ls_threshold(data->client);
	
	
 }	
	//  __ap3045_write_reg(data->client, AP3045_REG_SYS_INT_STATUS,0xff, 0x00, 0);
   	
      i2c_smbus_write_byte_data(data->client,0x01,0x00);
	//int_stat = ap3045_get_int_stat(data->client);
	// LDBG("%s, anna handler end int_stat = 0x%x\n",__func__, int_stat);
     enable_irq(data->client->irq);
#if 0
    // this is incorrect,not intialized
    pxvalue = ap3045_get_px_value(data->client); 
    input_report_abs(data->hsensor_input_dev, ABS_WHEEL, pxvalue);
    input_sync(data->hsensor_input_dev);
#endif
#if 0
    // ALS int
    if (int_stat & AP3045_REG_SYS_INT_AMASK)
    {
	ap3045_change_ls_threshold(data->client);
    }

    // PX int
    if (int_stat & AP3045_REG_SYS_INT_PMASK)
    {
	Pval = ap3045_get_object(data->client);
	LDBG("%s\n", Pval ? "obj near":"obj far");
	input_report_abs(data->psensor_input_dev, ABS_DISTANCE, Pval);
	input_sync(data->psensor_input_dev);
    }

    enable_irq(data->client->irq);
#endif
}


/*
 * I2C layer
 */

static irqreturn_t ap3045_irq(int irq, void *data_)
{
    struct ap3045_data *data = data_;

   // LDBG("%s\n",__func__);
   // LDBG("data->psensor_wq = 0x%p,data->psensor_work=0x%p",data->psensor_wq,data->psensor_work);
    disable_irq_nosync(data->client->irq);
    //queue_work(data->psensor_wq, &data->psensor_work);
   // queue_work(data->plsensor_wq, &data->plsensor_work);
	 queue_work(data->plsensor_wq, &ap3045_irq_work);
	//LDBG("%s end anna \n",__func__);
 //  enable_irq(data->client->irq);
    return IRQ_HANDLED;
}

static int  ap3045_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct ap3045_data *data;
    int err = 0;

    LDBG("ap3045_probe\n");

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)){
		err = -EIO;
		goto exit_free_gpio;
    }

    reg_array = ap3045_reg;
    range = ap3045_range;
    reg_num = sizeof(ap3045_reg)/sizeof(ap3045_reg[0]);

    data = kzalloc(sizeof(struct ap3045_data), GFP_KERNEL);
    if (!data){
	err = -ENOMEM;
	goto exit_free_gpio;
    }

    data->client = client;
    i2c_set_clientdata(client, data);
    data->irq = client->irq;

    err = ap3045_power_set(data, true);
	if (err < 0) {	
		dev_err(&client->dev, "%s:ap3045 power on error!\n", __func__);
		goto exit_kfree;
	}

    if (ap3045_i2c_read_byte_data(client, 0x00) < 0) {
    	dev_err(&client->dev, "%s:ap3045 is not present!\n", __func__);
    	goto exit_power_on;
    }

    /* initialize the AP3045 chip */
    err = ap3045_init_client(client);
    if (err)
	goto exit_power_on;

    err = ap3045_register_lsensor_device(client,data);
    if (err){
	dev_err(&client->dev, "failed to register_lsensor_device\n");
	goto exit_power_on;
    }

    err = ap3045_register_psensor_device(client, data);
    if (err) {
	dev_err(&client->dev, "failed to register_psensor_device\n");
	goto exit_free_ls_device;
    }

	


#if 1
    /* register sysfs hooks */
    err = sysfs_create_group(&data->client->dev.kobj, &ap3045_attr_group);
    if (err)
	goto exit_free_ps_device;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    LDBG("define the [CONFIG_HAS_EARLYSUSPEND]\n");
    ap3045_early_suspend.suspend = ap3045_suspend;
    ap3045_early_suspend.resume  = ap3045_resume;
    ap3045_early_suspend.level   = 0x02;
    register_early_suspend(&ap3045_early_suspend);
#endif

#if 0
    err = request_threaded_irq(client->irq, NULL, ap3045_irq,
	    IRQF_TRIGGER_FALLING,
	    "ap3045", data);

   data->psensor_wq = create_singlethread_workqueue("psensor_wq");   
    if (!data->psensor_wq) {
	LDBG("%s: create workqueue failed\n", __func__);
	err = -ENOMEM;
	goto err_create_wq_failed;
    }
#else

    data->plsensor_wq = create_singlethread_workqueue("plsensor_wq");	
    if (!data->plsensor_wq) {
	LDBG("%s: create workqueue failed\n", __func__);
	err = -ENOMEM;
	goto err_create_wq_failed;
    }
	
    err = request_irq(client->irq, ap3045_irq, IRQF_TRIGGER_FALLING | IRQF_ONESHOT ,"ap3045",data); 
    //err = request_irq(client->irq, ap3045_irq, /*IRQF_TRIGGER_FALLING*/IRQF_TRIGGER_LOW|IRQF_ONESHOT,
    //		"ap3045",data); 
	
#endif
    if (err) {
	dev_err(&client->dev, "ret: %d, could not get IRQ %d\n",err,client->irq);
	goto exit_free_ps_device;
    }

   
	



    //INIT_WORK(&data->plsensor_work, plsensor_work_handler);
    //LDBG("Timer module installing\n");
   // INIT_WORK(&data->psensor_work, psensor_work_handler);
  //anna-   setup_timer(&data->pl_timer, pl_timer_callback, 0);


   // LDBG("psensor_work_handler[END]\n");

    private_pl_data = data;
//<ASUS-<asus-annacheng20150129>>>>>>>>>>>>>>+
	if(create_ap3045_asusproc_lightsensor_status_entry( ))
		printk("[%s] : ERROR to create lightsensor proc entry\n",__func__);

	if(create_ap3045_asusproc_Proximitysensor_status_entry( ))
		printk("[%s] : ERROR to create Proximitysensor proc entry\n",__func__);
//<ASUS-<asus-annacheng20150129><<<<<<<<<<<<+
    dev_info(&client->dev, "ap3045 Driver version %s enabled\n", DRIVER_VERSION);
    return 0;
err_create_wq_failed:
  //  if(&data->pl_timer != NULL)
//	del_timer(&data->pl_timer);
    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
exit_free_ps_device:
    ap3045_unregister_psensor_device(client,data);
    LDBG("ap3045_unregister\n");

exit_free_ls_device:
    ap3045_unregister_lsensor_device(client,data);

exit_power_on:
    ap3045_power_set(data, false);

exit_kfree:
    kfree(data);

exit_free_gpio:
    return err;
}

static int  ap3045_remove(struct i2c_client *client)
{
   
    struct ap3045_data *data = i2c_get_clientdata(client);
	 LDBG("ap3045_remove\n");
    free_irq(data->irq, data);

    sysfs_remove_group(&data->client->dev.kobj, &ap3045_attr_group);
    ap3045_unregister_psensor_device(client,data);
    ap3045_unregister_lsensor_device(client,data);

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ap3045_early_suspend);
#endif

    ap3045_set_mode(client, 0);
    kfree(i2c_get_clientdata(client));

    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
    //if(&data->pl_timer)
	//del_timer(&data->pl_timer);
    return 0;
}

static const struct i2c_device_id ap3045_id[] = {
    { AP3045_DRV_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, ap3045_id);


static struct i2c_driver ap3045_driver = {
    .driver = {
	.name	= AP3045_DRV_NAME,
	.owner	= THIS_MODULE,
    },
    .probe	= ap3045_probe,
    .remove	= ap3045_remove,
    .id_table = ap3045_id,
};

static int __init ap3045_init(void)
{
    int ret;

    LDBG("ap3045_init\n");
    ret = i2c_add_driver(&ap3045_driver);
	LDBG("ret %d\n",ret);
    return ret;
}

static void __exit ap3045_exit(void)
{
    i2c_del_driver(&ap3045_driver);
//<asus-annacheng20150129>>>>>>>>>>>>>>>>>>+	
	if(ap3045_proximitysensor_entry)
		remove_proc_entry("Proximitysensor_status", NULL);
	if(ap3045_lightsensor_entry)
		remove_proc_entry("lightsensor_status", NULL);
//<asus-annacheng20150129><<<<<<<<<<<<<<<<<+
}

MODULE_AUTHOR("Leo Tsai Dyna-Image Corporation.");
MODULE_DESCRIPTION("AP3045 driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(ap3045_init);
module_exit(ap3045_exit);



