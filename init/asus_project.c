/* NOTICE: 
*This file is for asus project id pins, 
*some id pin export in kernel by asus_XXXX , before use these global variable , 
*you need inlude <linux/asus_project.h> in your driver.
*And this file also export these id to user space by /proc/apxx.
*The value meaning of each id is described in asus_project.h 
*@wigman_sun
*/
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/asus_project.h>


//+++ ASUS_BSP :  Add for charger mode @wigman_sun
char* androidboot_mode = "main";
EXPORT_SYMBOL(androidboot_mode);
static int get_androidboot_mode(char *str)
{
	androidboot_mode = str;
	printk("androidboot.mode = %s\n", androidboot_mode);

    return 0;
}
__setup("androidboot.mode=", get_androidboot_mode);
//--- ASUS_BSP :  Add for project id


int strtoint(char *str){
	int i = 0, val = 0;
	while(str[i] != '\0')
		val = val*10 + (int)str[i++]-48;
	return val;
}

//+++ ASUS_BSP :  Add for project id @wigman_sun
int asus_PRJ_ID = 0;
EXPORT_SYMBOL(asus_PRJ_ID);
static int set_project_id(char *str)
{
	asus_PRJ_ID = strtoint(str);
	printk("asus_PRJ_ID = %d\n", asus_PRJ_ID);

    return 0;
}
__setup("PRJ_ID=", set_project_id);
//--- ASUS_BSP :  Add for project id

//+++ ASUS_BSP :  Add for asus_project_RFsku @wigman_sun
int asus_RF_SKU = 0;
EXPORT_SYMBOL(asus_RF_SKU);

static int get_prj_RFsku(char *str)
{
    asus_RF_SKU = strtoint(str);
	printk("asus_RF_SKU = %d\n ", asus_RF_SKU);

    return 0;
}
__setup("RF_SKU=", get_prj_RFsku);
//--- ASUS_BSP :  Add for asus_project_RFsku

//+++ ASUS_BSP :  Add for asus_hw_id @wigman_sun
int asus_HW_ID = 0;
EXPORT_SYMBOL(asus_HW_ID);

static int get_prj_HW_ID(char *str)
{
    asus_HW_ID = strtoint(str);
	printk("asus_HW_ID = %d\n ", asus_HW_ID);

    return 0;
}
__setup("HW_ID=", get_prj_HW_ID);
//--- ASUS_BSP :  Add for asus_hw_id

static struct proc_dir_entry *project_id_proc_file;
static int project_id_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", asus_PRJ_ID);
	return 0;
}

static int project_id_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_id_proc_read, NULL);
}


static struct file_operations project_id_proc_ops = {
	.open = project_id_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_id_proc_file(void)
{
    printk("create_project_id_proc_file\n");
    project_id_proc_file = proc_create("apid", 0444,NULL, &project_id_proc_ops);
    if(project_id_proc_file){
        printk("create project_id_proc_file sucessed!\n");
    }else{
		printk("create project_id_proc_file failed!\n");
    }
}

static struct proc_dir_entry *hw_id_proc_file;
static int hw_id_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", asus_HW_ID);
	return 0;
}

static int hw_id_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, hw_id_proc_read, NULL);
}


static struct file_operations hw_id_proc_ops = {
	.open = hw_id_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_hw_id_proc_file(void)
{
    printk("create_hw_id_proc_file\n");
    hw_id_proc_file = proc_create("aphw", 0444,NULL, &hw_id_proc_ops);
    if(hw_id_proc_file){
        printk("create create_hw_id_proc_file sucessed!\n");
    }else{
		printk("create create_hw_id_proc_file failed!\n");
    }
}

static struct proc_dir_entry *project_RFsku_proc_file;
static int project_RFsku_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", asus_RF_SKU);
	return 0;
}

static int project_RFsku_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, project_RFsku_proc_read, NULL);
}


static struct file_operations project_RFsku_proc_ops = {
	.open = project_RFsku_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_project_RFsku_proc_file(void)
{
    printk("create_project_RFsku_proc_file\n");
    project_RFsku_proc_file = proc_create("aprf", 0444,NULL, &project_RFsku_proc_ops);
    if(project_RFsku_proc_file){
        printk("create project_RFsku_proc_file sucessed!\n");
    }else{
		printk("create project_RFsku_proc_file failed!\n");
    }
}

static int __init proc_asusPRJ_init(void)
{
	create_project_id_proc_file();
	create_project_RFsku_proc_file();
	create_hw_id_proc_file();
	return 0;
}
module_init(proc_asusPRJ_init);

