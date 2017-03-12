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

struct tps6128 {
	struct i2c_client	*client;
	struct device		*dev;	
	int adc_pwr_en_gpio;
	int adc_sw_en_gpio;
};

struct tps6128 *tps6128_chip;

#define STATUS 0x05
int tps6128_read_reg(int reg, u8 *val)
{
	s32 ret;
	if(tps6128_chip==NULL){
		*val=0;
		return 0;
	}
		
	ret = i2c_smbus_read_byte_data(tps6128_chip->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}
	//printk("Reading 0x%02x=0x%02x\n", reg, *val);
	return 0;
}

static int tps6128_write_reg(int reg, u8 val)
{
	s32 ret;

	if(tps6128_chip==NULL){
		printk("tps6128chip is null\n");
		return 0;
	}
	ret = i2c_smbus_write_byte_data(tps6128_chip->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	pr_info("Writing 0x%02x=0x%02x\n", reg, val);
	return 0;
}

#define ASUS_TPS6128x_PROC_FILE "TPS6128x_Status"

static int tps6128x_proc_read(struct seq_file *buf, void *data)
{
	u8 reg, ret;	
	ret = tps6128_read_reg(STATUS, &reg);
	if(ret < 0)
		ret = 0;
	else
		ret = 1;
	seq_printf(buf, "%d\n", ret);
	return 0;
}
static int tps6128x_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tps6128x_proc_read, NULL);
}
static const struct file_operations tps6128x_fops = {
	.owner = THIS_MODULE,
	.open = tps6128x_proc_open,
	.read = seq_read,
	.release = single_release,
};
static void create_tps6128x_proc_file(void)
{
	struct proc_dir_entry *asus_tps6128x_proc_file = proc_create(ASUS_TPS6128x_PROC_FILE, 0666, NULL, &tps6128x_fops);

	if (asus_tps6128x_proc_file) {
		printk("create_tps6128x_proc_file create ok!\n");
	} else{
		printk("create_tps6128x_proc_file create failed!\n");
	}
}

static int tps6128_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc;
	struct tps6128 *chip;
	u8 reg[6];
	u8 address;
	//struct device *dev = &client->dev;
	//struct device_node *node = dev->of_node;

	printk("tps6128 probe\n");
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pr_err("Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	tps6128_chip=chip;

	/* probe the device to check if its actually connected */
	printk("[TPS6128]reg when system start\n");
	for(address =0x1;address < 0x6;address++)
		{
			rc = tps6128_read_reg(address, &reg[address-1]);
				if (rc) {
					pr_err("Failed to detect tps6128, device may be absent\n");
					return -ENODEV;
				}
		}
	
	rc = tps6128_read_reg(0xff, &reg[5]);
	if (rc) {
		pr_err("Failed to detect tps6128, device may be absent\n");
		return -ENODEV;
	}
	printk("[0x01]=0x%02x; [0x02]=0x%02x; [0x03]=0x%02x; [0x04]=0x%02x; [0x05]=0x%02x; [0xff]=0x%02x; ",
		reg[0],reg[1],reg[2],reg[3],reg[4],reg[5]);
	//printk("tps6128 chip status is %x\n", reg);
	//tps6128_write_reg(0xff,0x80);
	tps6128_write_reg(0x03,0x0f);
	create_tps6128x_proc_file();
	printk("[TPS6128]reg after porting\n");
	for(address =0x1;address < 0x6;address++)
		{			
			rc = tps6128_read_reg(address, &reg[address-1]);
				if (rc) {
					pr_err("Failed to detect tps6128, device may be absent\n");
					return -ENODEV;
				}
		}
	
	rc = tps6128_read_reg(0xff, &reg[5]);
	if (rc) {
		pr_err("Failed to detect tps6128, device may be absent\n");
		return -ENODEV;
	}
	printk("[0x01]=0x%02x; [0x02]=0x%02x; [0x03]=0x%02x; [0x04]=0x%02x; [0x05]=0x%02x; [0xff]=0x%02x; ",
		reg[0],reg[1],reg[2],reg[3],reg[4],reg[5]);


	printk("tps6128 probe ok\n");
	return 0;
}

static int tps6128_remove(struct i2c_client *client)
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

#define tps6128_I2C_NAME          "tps6128"
static const struct i2c_device_id tps6128_id[] = {
    { tps6128_I2C_NAME, 0 },
    { }
};
static struct of_device_id tps6128_match_table[] = {
	{ .compatible = "tps6128",},
	{ },
};

static struct i2c_driver tps6128_driver = {
    .probe      = tps6128_probe,
    .remove     = tps6128_remove,
    .id_table   = tps6128_id,
    .driver = {
        .name     = tps6128_I2C_NAME,
        .owner    = THIS_MODULE,
        .of_match_table	=tps6128_match_table,
    },
};

static int  tps6128_init(void){ 
	s32 ret;

	ret = i2c_add_driver(&tps6128_driver);
	printk("tps6128_init!\n");
	return ret;        
}
static void  tps6128_exit(void) {
	
	i2c_del_driver(&tps6128_driver);       
}
module_init(tps6128_init);
module_exit(tps6128_exit);
