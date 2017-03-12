#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include "../video/msm/mdss/mdss_panel.h"

#define AIN1	0x03
#define AIN3	0x05
extern  struct  mdss_panel_data *gdata;
struct us5587 {
	struct i2c_client	*client;
	struct device		*dev;	
	int adc_en_gpio;
	int adc_sw_en_gpio;
	struct delayed_work polling_temp_work;
	int thermal_temp[6];  //off_temp1 off_temp2  off_temp3 on_temp1  on_temp2  on_temp3
	int thermal_level;
};

struct us5587 *us5587_chip;

#define CHIP_ID 0xB2
int us5587_read_reg(int reg, u8 *val)
{
	s32 ret;
	if(us5587_chip==NULL){
		*val=0;
		return -1;
	}
		
	ret = i2c_smbus_read_byte_data(us5587_chip->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}
	printk("Reading 0x%02x=0x%02x\n", reg, *val);
	return 0;
}

/*static int us5587_write_reg(struct us5587 *chip, int reg, u8 val)
{
	s32 ret;
	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	pr_debug("Writing 0x%02x=0x%02x\n", reg, val);
	return 0;
}*/

u8 ChargerTempTable[]= {0xff,0xff,0xfb,0xf4,0xee,0xe7,0xe1,0xdb,0xd4,0xce,0xc9,0xc3,0xbd,0xb8,0xb2,0xad,\
						     0xa8,0xa3,0x9e,0x99,0x94,0x90,0x8c,0x87,0x83,0x7f,0x7b,0x77,0x74,0x70,0x6d,0x69,\
						     0x66,0x63,0x60,0x5d,0x5a,0x57,0x54,0x52,0x4f,0x4d,0x4a,0x48,0x46,0x44,0x42,0x40,\
						     0x3e,0x3c,0x3a,0x38,0x37,0x35,0x33,0x32,0x30,0x2f,0x2d,0x2c,0x2b,0x29,0x28,0x27,\
						     0x26,0x25,0x24,0x23,0x22,0x21,0x20,0x1f,0x1e,0x1d,0x1c,0x1b,0x1b,0x1a,0x19,0x18};
static u8 Convert_Voltage2temp(u8 data){
	u8 i,temp;

	for(i=0;i<sizeof(ChargerTempTable);i++){
		if((ChargerTempTable[i] >= data) &&(ChargerTempTable[i+1] <= data))
			break;
	}

	if(data < ChargerTempTable[sizeof(ChargerTempTable) -1]){
		temp = 110;
		printk(" date over temp table, data:%d,temp:%d\n",data,temp);
	}
	
	if(data == ChargerTempTable[i])
		temp = 31+i;
	else{
		if(data == ChargerTempTable[i+1])
			temp = 31+i+1;
		else{
			if(ChargerTempTable[i] == ChargerTempTable[i+1])
				temp = 31+i;
			else{
				if( (10*data) >= (10*ChargerTempTable[i+1] + 10*(ChargerTempTable[i] - ChargerTempTable[i+1])/2)  )
					temp = 31+i;
				else
					temp = 31+i+1;
			}
			
		}
	}
	printk("Convert_Voltage2temp i:%d,data:%x,temp:%d\n",i,data,temp);
	return temp;

}

/*
	type: 0 pmi8952 chaeger , 1 parallel charger;
*/
u8 ASUS_Charger_Temp(bool type)
{
	u8 data,temp;
	s32 ret;
	
	if(type){
		ret = us5587_read_reg(AIN3,&data);
		if (ret < 0) {
			pr_err("AIN3 read fail %d\n", ret);
			return ret;
		}
	}else{
		ret = us5587_read_reg(AIN1,&data);
		if (ret < 0) {
			pr_err("AIN1 read fail %d\n", ret);
			return ret;
		}
	}
	
	temp = Convert_Voltage2temp(data);
	return temp;
}

//PRT8301 AIN1
static int PRT8301_proc_read(struct seq_file *buf, void *data)
{
	u8 reg,temp;
	s32 ret;

	ret = us5587_read_reg(AIN1,&reg);
	if (ret < 0) {
		pr_err("AIN1 read fail %d\n", ret);
		return ret;
	}	
	temp = Convert_Voltage2temp(reg);
	seq_printf(buf, "%d\n", temp);
	return 0;
}
static int PRT8301_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, PRT8301_proc_read, NULL);
}
static const struct file_operations PRT8301_fops = {
	.owner = THIS_MODULE,
	.open = PRT8301_proc_open,
	.read = seq_read,
	.release = single_release,
};
static void create_prt8301_proc_file(void)
{
	struct proc_dir_entry *PRT8301_proc_file = proc_create("PRT8301_temp", 0666, NULL, &PRT8301_fops);

	if (PRT8301_proc_file) {
		printk("PRT8301_proc_file create ok!\n");
	} else{
		printk("PRT8301_proc_file create failed!\n");
	}
}
//PRT8302 AIN3
static int PRT8302_proc_read(struct seq_file *buf, void *data)
{
	u8 reg,temp;
	s32 ret;

	ret = us5587_read_reg(AIN3,&reg);
	if (ret < 0) {
		pr_err("AIN3 read fail %d\n", ret);
		return ret;
	}	
	temp = Convert_Voltage2temp(reg);
	seq_printf(buf, "%d\n", temp);
	return 0;
}
static int PRT8302_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, PRT8302_proc_read, NULL);
}
static const struct file_operations PRT8302_fops = {
	.owner = THIS_MODULE,
	.open = PRT8302_proc_open,
	.read = seq_read,
	.release = single_release,
};
static void create_prt8302_proc_file(void)
{
	struct proc_dir_entry *PRT8302_proc_file = proc_create("PRT8302_temp", 0666, NULL, &PRT8302_fops);

	if (PRT8302_proc_file) {
		printk("PRT8302_proc_file create ok!\n");
	} else{
		printk("PRT8302_proc_file create failed!\n");
	}
}

#define ASUS_US5587_PROC_FILE "US5587_Status"

static int us5587_proc_read(struct seq_file *buf, void *data)
{
	u8 reg, ret;	
	ret = us5587_read_reg(0x00, &reg);
	if(ret < 0)
		ret = 0;
	else
		ret = 1;
	seq_printf(buf, "%d\n", ret);
	return 0;
}
static int us5587_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, us5587_proc_read, NULL);
}
static const struct file_operations us5587_fops = {
	.owner = THIS_MODULE,
	.open = us5587_proc_open,
	.read = seq_read,
	.release = single_release,
};
static void create_us5587_proc_file(void)
{
	struct proc_dir_entry *asus_us5587_proc_file = proc_create(ASUS_US5587_PROC_FILE, 0666, NULL, &us5587_fops);

	if (asus_us5587_proc_file) {
		printk("create_us5587_proc_file create ok!\n");
	} else{
		printk("create_us5587_proc_file create failed!\n");
	}
}

int asus_judge_thermal_level(struct us5587 *chip,int temp,bool on){

	int level,pre_level;

	pre_level = chip->thermal_level;
		
	printk("[US5587]panel %s,temp =%d\n",on ? "on":"off",temp);
		
	if(temp < chip->thermal_temp[on*3+0]){
			level = 0;
	}else if(temp < chip->thermal_temp[on*3+1]){
			level = 1;
	}else if(temp < chip->thermal_temp[on*3+2]){
			level = 2;
	}else{
			level = 3;
	}

	//judge debounce 2
	if (level ==0 && pre_level ==1){
		if(temp > chip->thermal_temp[on*3+0]-2)
				level =1;
	}else if(level ==1 && pre_level ==2){
		if(temp > chip->thermal_temp[on*3+1]-2)
				level =2;
	}else if(level ==2 && pre_level ==3){
		if(temp > chip->thermal_temp[on*3+2]-2)
				level =3;
	}

	if (level ==3 && pre_level ==2){
		if(temp < chip->thermal_temp[on*3+2]+2)
				level =2;
	}else if(level ==2 && pre_level ==1){
		if(temp < chip->thermal_temp[on*3+1]+2)
				level =1;
	}else if(level ==1 && pre_level ==0){
		if(temp < chip->thermal_temp[on*3+0]+2)
			level =0;
	}		
	printk("[US5587]thermal level now is %d,pre is %d\n",level,pre_level);
	return level;
}

void asus_polling_temp_work(struct work_struct *work){
	struct us5587 *chip = container_of(work,
			struct us5587,
			polling_temp_work.work);

	union power_supply_propval value;
	struct power_supply *batt_chg;
	u8 temp = ASUS_Charger_Temp(0);
	bool panel_on =gdata->panel_info.panel_power_state;
	
	batt_chg = power_supply_get_by_name("battery");
	if (!batt_chg) {
		printk("[US5587]charge supply not found, could not set thermal levelto it\n");
	}

	chip->thermal_level = asus_judge_thermal_level(chip,temp,panel_on);
	value.intval= chip->thermal_level ;

	printk("[CHARGE][US5587]set charge thermal level =%d\n",value.intval);
	if(batt_chg){
		batt_chg->set_property(batt_chg,POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,&value);
	}

	if(gdata->panel_info.panel_power_state ==1){
		printk("[US5587]panel on, next polling temp 10s\n");
		schedule_delayed_work(&chip->polling_temp_work,10*HZ);
	}else{
		printk("[US5587]panel off, next polling temp 60s\n");
		schedule_delayed_work(&chip->polling_temp_work,60*HZ);
	}
}
void us5587_thermal_policy(bool run_if)
{
	union power_supply_propval value;
	struct power_supply *batt_chg;
	batt_chg = power_supply_get_by_name("battery");
	if (!batt_chg) {
		printk("[US5587]charge supply not found, could not set thermal levelto it\n");
	}
	
	if(us5587_chip){
		if(run_if){
			printk("[CHARGE][US5587]now thermal_policy start");
			cancel_delayed_work(&us5587_chip->polling_temp_work);
			schedule_delayed_work(&us5587_chip->polling_temp_work,20*HZ);
		}else{
			printk("[CHARGE][US5587]now thermal_policy stop");
			us5587_chip->thermal_level=0;
			cancel_delayed_work(&us5587_chip->polling_temp_work);
			value.intval= 0 ;
			printk("[CHARGE][US5587]set charge thermal level =%d\n",value.intval);
			if(batt_chg){
				batt_chg->set_property(batt_chg,POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,&value);
			}
		}
	}else{
		printk("[US6687] us5587 not in\n");
	}
}

static int thermal_temp_proc_read(struct seq_file *buf, void *v)
{
	if(us5587_chip)
		seq_printf(buf, "T1_on =%d, T2_on =%d, T3_on =%d, T1_off =%d, T2_off =%d, T3_off =%d\n",
		us5587_chip->thermal_temp[3],us5587_chip->thermal_temp[4],us5587_chip->thermal_temp[5],
		us5587_chip->thermal_temp[0],us5587_chip->thermal_temp[1],us5587_chip->thermal_temp[2]);

	return 0;
}

static ssize_t thermal_temp_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	char messages[256];
	int on;
	int t1,t2,t3;

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	if(us5587_chip){
		sscanf(buff,"%d %d %d %d", &on,&t1,&t2,&t3);

		if(on){
			us5587_chip->thermal_temp[3] =t1;
			us5587_chip->thermal_temp[4] =t2;
			us5587_chip->thermal_temp[5] =t3;
		}else{
			us5587_chip->thermal_temp[0] =t1;
			us5587_chip->thermal_temp[1] =t2;
			us5587_chip->thermal_temp[2] =t3;
		}
	}

	return len;
}

static int thermal_temp_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, thermal_temp_proc_read, NULL);
}

static const struct file_operations thermal_temp_fops = {
	.owner = THIS_MODULE,
	.open =  thermal_temp_proc_open,
	.write = thermal_temp_proc_write,
	.read = seq_read,
	.release = single_release,
};

static void create_thermal_temp_proc_file(void)
{
	struct proc_dir_entry *thermal_temp_proc_file = proc_create("driver/thermal_temp", 0666, NULL, &thermal_temp_fops);

	if (thermal_temp_proc_file) {
		printk("thermal_temp_proc_file create ok!\n");
	} else{
		printk("thermal_temp_proc_file create failed!\n");
	}
	return;
}
bool us5587_adc_enable =0;
int  asus_us5587_adc(bool enable)
{
	if(us5587_chip==NULL){
		pr_info("us5587_chip NULL\n");
		return 0;
	}

	pr_info("%s enable:%d\n",__FUNCTION__,enable);
	us5587_adc_enable = enable;
	if(enable)
		gpio_direction_output(us5587_chip->adc_en_gpio, 0);
	else
		gpio_direction_output(us5587_chip->adc_en_gpio, 1);
	return true;
}

static int us5587_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc;
	struct us5587 *chip;
	u8 reg = 0;
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;

	printk("us5587 probe\n");
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pr_err("Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	us5587_chip=chip;

	/* probe the device to check if its actually connected */
	rc = us5587_read_reg(CHIP_ID, &reg);
	if (rc) {
		pr_err("Failed to detect us5587, device may be absent\n");
		return -ENODEV;
	}
	printk("us5587 chip revision is %x\n", reg);
	create_us5587_proc_file();
	create_prt8301_proc_file();
	create_prt8302_proc_file();
		//request gpio for adc 
	chip->adc_en_gpio= of_get_named_gpio(node, "us5587_adc_en", 0);
	printk("us5587_adc_en_gpio = %d\n", chip->adc_en_gpio);
	
	if ((!gpio_is_valid(chip->adc_en_gpio))) {
		printk("%s: adc_pwr_en_gpio is not valid!\n", __FUNCTION__);
		return -EINVAL;
	}
	rc = gpio_request(chip->adc_en_gpio,"us5587_adc_en");
	if (rc < 0) {
		printk("%s: request us5587_adc_en gpio fail!\n", __FUNCTION__);
	}

	gpio_direction_output(chip->adc_en_gpio, 0);
	if (rc < 0) {
		printk("%s: set direction of us5587-adc-en fail!\n", __FUNCTION__);
	}

	chip->thermal_temp[3]=45;
	chip->thermal_temp[4]=55;
	chip->thermal_temp[5]=60;
	chip->thermal_temp[0]=50;
	chip->thermal_temp[1]=55;
	chip->thermal_temp[2]=60;
	chip->thermal_level=0;
	INIT_DELAYED_WORK(&chip->polling_temp_work,asus_polling_temp_work);
	create_thermal_temp_proc_file();
	if(!us5587_adc_enable)
		gpio_direction_output(chip->adc_en_gpio, 1);
	printk("us5587 probe ok\n");
	return 0;
}

static int us5587_remove(struct i2c_client *client)
{
	/*struct us5587 *chip=i2c_get_clientdata(client);
	if (gpio_is_valid(chip->adc_sw_en_gpio)) {
		gpio_free(chip->adc_sw_en_gpio);
	}
	if (gpio_is_valid(chip->adc_pwr_en_gpio)) {
		gpio_free(chip->adc_pwr_en_gpio);
	}*/
	return 0;
}

#define us5587_I2C_NAME          "us5587-adc"
static const struct i2c_device_id us5587_id[] = {
    { us5587_I2C_NAME, 0 },
    { }
};
static struct of_device_id us5587_match_table[] = {
	{ .compatible = "us5587-adc",},
	{ },
};

static struct i2c_driver us5587_driver = {
    .probe      = us5587_probe,
    .remove     = us5587_remove,
    .id_table   = us5587_id,
    .driver = {
        .name     = us5587_I2C_NAME,
        .owner    = THIS_MODULE,
        .of_match_table	=us5587_match_table,
    },
};

static int  us5587_init(void){ 
	s32 ret;

	ret = i2c_add_driver(&us5587_driver);
	printk("us5587_init!\n");
	return ret;        
}
static void  us5587_exit(void) {
	
	i2c_del_driver(&us5587_driver);       
}
module_init(us5587_init);
module_exit(us5587_exit);
