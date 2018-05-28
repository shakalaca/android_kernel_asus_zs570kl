/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/
//#include "Laura_shipping_func.h"
#include "msm_laser_focus.h"
#include "laser_log.h"
#include "HPTG_debug.h"
#include "HPTG_interface.h"
#include "HPTG_factory_func.h"
#include "HPTG_shipping_func.h"
#include "laser_focus_hepta.h"
#include <linux/of.h>
#include <linux/of_gpio.h>
#include "laser_focus_hepta.h"


//Laser vars  +++


int g_factory = 0;			//default for shipping
bool timedMeasure = true; //default for shipping
bool CSCmode = false;

uint16_t Laser_log_cnt=0;
//Laser vars ---



/* HEPT VARS +++ */
int Laser_Product = PRODUCT_UNKNOWN;
int FirmWare;
extern uint16_t module_id[34];

extern struct msm_laser_focus_ctrl_t *laura_t;

//Error table
extern uint16_t thd;
extern uint16_t limit;
extern uint8_t thd_near_mm;
/* HEPT VARS --- */


/* ST VARS +++ */
//ST long range param
extern uint8_t		g_preRange;
extern uint8_t		g_finalRange;
extern uint32_t 	g_signal_Kcps;
extern uint32_t 	g_sigma_mm;
extern uint32_t 	g_time_budget_ms;
/* ST VARS --- */

#define proc(file,mod,fop)	\
	LOG_Handler(LOG_DBG,"proc %s %s\n",file,proc_create(file,mod,NULL,fop)? "success":"fail");

int IDexistance(struct msm_laser_focus_ctrl_t *dev_t){

	if (!dev_t) {
		LOG_Handler(LOG_ERR, "%s: failed: %p\n", __func__, dev_t);
		return -EINVAL;
	}
	if (!(dev_t->i2c_client && dev_t->sensordata->slave_info && dev_t->sensordata->sensor_name)) {
		LOG_Handler(LOG_ERR, "%s: failed: %p %p %p\n", __func__,
			dev_t->i2c_client,
			dev_t->sensordata->slave_info,
			dev_t->sensordata->sensor_name);
		return -EINVAL;
	}
	return 0;
}
extern uint16_t chipID;


int VerifyID(struct msm_laser_focus_ctrl_t *dev_t, int chip_id_size){

	int rc = 0;
	uint16_t chip_id = 0;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;
	sensor_i2c_client = dev_t->i2c_client;
	slave_info = dev_t->sensordata->slave_info;
	sensor_name = dev_t->sensordata->sensor_name;

	// Get ID
	rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client,
		slave_info->sensor_id_reg_addr, &chip_id, chip_id_size);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: %s read id failed\n", __func__, sensor_name);
		return rc;
	}
	chipID=chip_id;

	// Verify ID,  &&0x01AD for HEPT and wont disturb ST
	printk( "%s: read id: 0x%x expected id 0x%x\n", __func__, chip_id, slave_info->sensor_id);
	if (chip_id != slave_info->sensor_id && (chip_id != 0x01AD)) {
		LOG_Handler(LOG_ERR, "%s: chip id doesnot match\n", __func__);
		return -ENODEV;
	}
	return rc;
}

int Laser_Match_ID(struct msm_laser_focus_ctrl_t *dev_t, int chip_id_size)
{
	int rc = 0;

	rc = IDexistance(dev_t);
	if(rc==0)
		rc = VerifyID(dev_t, chip_id_size);

	return rc;
}

/** @brief Voltage control
*
*	@param dev_t the laser focus controller
*	@param config the configuration (>0:enable; 0:disable)
*
*/
int32_t vreg_control(struct msm_laser_focus_ctrl_t *dev_t, int config)
{
	int rc = 0, i, cnt;
	struct msm_laser_focus_vreg *vreg_cfg;

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	vreg_cfg = &dev_t->vreg_cfg;
	cnt = vreg_cfg->num_vreg;
	if (!cnt)
		return 0;

	if (cnt >= CAM_VREG_MAX) {
		LOG_Handler(LOG_ERR, "%s: failed %d cnt %d\n", __func__, __LINE__, cnt);
		return -EINVAL;
	}

	for (i = 0; i < cnt; i++) {
		if (dev_t->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = msm_camera_config_single_vreg(&(dev_t->pdev->dev),
				&vreg_cfg->cam_vreg[i], (struct regulator **)&vreg_cfg->data[i], config);
		} else {
			rc = msm_camera_config_single_vreg(&(dev_t->i2c_client->client->dev),
				&vreg_cfg->cam_vreg[i], (struct regulator **)&vreg_cfg->data[i], config);
		}
	}

	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}

/** @brief Power on component
*
*	@param dev_t the laser focus controller
*
*/
int32_t PowerUp(struct msm_laser_focus_ctrl_t *dev_t)
{
	int rc = 0;

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Enable voltage */
	rc = vreg_control(dev_t, ENABLE_VREG);
	if (rc < 0) {
		LOG_Handler(LOG_ERR, "%s: failed %d\n", __func__, __LINE__);
		return rc;
	}

	/* Set current status to power on state */
	dev_t->laser_focus_state = LASER_FOCUS_POWER_UP;
	if (dev_t->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		/* Set GPIO vdig to high for sku4 */

		GPIO_Handler(dev_t, SENSOR_GPIO_VDIG, GPIO_HIGH);

		//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	}
	return rc;
}


/** @brief Power off component
*
*	@param dev_t the laser focus controller
*
*/
int32_t PowerDown(struct msm_laser_focus_ctrl_t *dev_t)
{
	int32_t rc = 0;

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Check device status */
	if (dev_t->laser_focus_state != LASER_FOCUS_POWER_DOWN) {

		/* Disable voltage */
		rc = vreg_control(dev_t, DISABLE_VREG);
		if (rc < 0) {
			LOG_Handler(LOG_ERR, "%s failed %d\n", __func__, __LINE__);
			return rc;
		}

		/* release memory for i2c table */
		kfree(dev_t->i2c_reg_tbl);
		dev_t->i2c_reg_tbl = NULL;
		dev_t->i2c_tbl_index = 0;
		/* Set current status to power off state*/
		dev_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	}

	if (dev_t->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		/* Set GPIO vdig to low for sku4 */
		GPIO_Handler(dev_t, SENSOR_GPIO_VDIG, GPIO_LOW);

		//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	}
	return rc;
}

/** @brief Initialize device
*
*	@param dev_t the laser focus controller
*
*/
int dev_cci_init(struct msm_laser_focus_ctrl_t *dev_t)
{
	int rc = 0;
	if(!dev_t){
		LOG_Handler(LOG_ERR, "%s: failed\n", __func__);
		return -EINVAL;
	}

	// CCI initialize
	if (dev_t->act_device_type == MSM_CAMERA_PLATFORM_DEVICE){
		rc = dev_t->i2c_client->i2c_func_tbl->i2c_util(dev_t->i2c_client, MSM_CCI_INIT);
		if (rc != 0)
			LOG_Handler(LOG_ERR, "%s: cci_init failed, rc= %d\n", __func__, rc);
	}

	return rc;
}


/** @brief Deinitialize device
*
*	@param dev_t the laser focus controller
*
*/
int dev_cci_deinit(struct msm_laser_focus_ctrl_t *dev_t)
{
	int rc = 0;

	if (!dev_t) {
		LOG_Handler(LOG_ERR, "%s: failed\n", __func__);
		return -EINVAL;
	}
	// CCI deinitialize
	if (dev_t->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = dev_t->i2c_client->i2c_func_tbl->i2c_util(dev_t->i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			LOG_Handler(LOG_ERR, "%s: cci_deinit failed, rc = %d\n", __func__, rc);

	}
	return rc;
}


#define CPE_LEN	5
#define PRODUCT_VERSION	5

void Product_Family(struct msm_laser_focus_ctrl_t *dev_t){
	char Laura[CPE_LEN] = {'0', 'M', 'G', 'B', 'X'};
	char Olivia[CPE_LEN] = {'0', 'M', 'L', 'A', 'X'};
	int PV,i;

	PV = module_id[PRODUCT_VERSION];

	for(i=1; i < CPE_LEN; i++)
		if(module_id[i] != Olivia[i])
			break;

	if(i == CPE_LEN){
		Laser_Product = PRODUCT_OLIVIA;
		if(PV == 0)
			FirmWare = Laser_Product;
		else
			LOG_Handler(LOG_DBG, "%s: unknown FW version: %d\n", __func__, PV);
	}

	for(i=1; i < CPE_LEN; i++)
		if(module_id[i] != Laura[i])
			break;

	if(i==CPE_LEN){
		Laser_Product = PRODUCT_LAURA;
		if(PV == 0)
			FirmWare = Laser_Product;
		else if(PV == 2)
			LOG_Handler(LOG_CDBG, "%s: FW version is OLIVIA\n", __func__, PV);
		else
			LOG_Handler(LOG_ERR, "%s: unknown FW version: %d\n", __func__, PV);

	}


}

extern void Get_ModuleID(struct msm_laser_focus_ctrl_t *dev_t);

void HPTG_Match_Module(struct msm_laser_focus_ctrl_t *dev_t){

	Get_ModuleID(dev_t);

	Product_Family(dev_t);

	LOG_Handler(LOG_CDBG, "%s: %d\n", __func__, Laser_Product);
}


/** @brief Check device status
*
*	@param dev_t the laser focus controller
*
*/
int HPTG_I2C_status_check(struct msm_laser_focus_ctrl_t *dev_t, int chip_id_size){
	int32_t rc=0;

	PowerUp(dev_t);
	dev_cci_init(dev_t);

	msleep(1); // wait for power stable

	rc = Laser_Match_ID(dev_t, chip_id_size);
	if (rc < 0)
		LOG_Handler(LOG_ERR,"%s fail, rc (%d)\n", __func__, rc);

	HPTG_Match_Module(dev_t);

	dev_cci_deinit(dev_t);
	PowerDown(dev_t);

	if(rc<0)
		return I2C_STATUS_FAIL;
	else
		return PRODUCER_HPTG;
}

int ST_I2C_status_check(struct msm_laser_focus_ctrl_t *s_ctrl, int chip_id_size){
	int32_t rc=0;

	PowerUp(s_ctrl);
	dev_cci_init(s_ctrl);

	rc = Laser_Match_ID(s_ctrl, chip_id_size);
	if (rc < 0)
		LOG_Handler(LOG_ERR,"%s fail, rc (%d)\n", __func__, rc);

	dev_cci_deinit(s_ctrl);
	PowerDown(s_ctrl);

	if(rc<0)
		return I2C_STATUS_FAIL;
	else
		return PRODUCER_ST;
}

/** @brief Mutex controller handles mutex action
*
*	@param dev_t the laser focus controller
*	@param ctrl the action of mutex
*
*/
int mutex_ctrl(struct msm_laser_focus_ctrl_t *dev_t, int ctrl)
{
	int rc = 0;

	//LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if(!dev_t){
		LOG_Handler(LOG_ERR, "%s: fail dev_t %p is NULL\n", __func__, dev_t);
		return -EFAULT;
	}

	switch(ctrl){
		/* Allocate mutex */
		case MUTEX_ALLOCATE:
			rc = _mutex_allocate(&dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_allocate\n", __func__);
				return rc;
			}
			break;
		/* Initialize mutex */
		case MUTEX_INIT:
			_mutex_init(dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_init\n", __func__);
				return rc;
			}
			break;
		/* Lock mutex */
		case MUTEX_LOCK:
			_mutex_lock(dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_lock\n", __func__);
				return rc;
			}
			break;
		/* Try lock mutex*/
		case MUTEX_TRYLOCK:
			_mutex_trylock(dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_trylock\n", __func__);
				return rc;
			}
			break;
		/* Unlock mutex */
		case MUTEX_UNLOCK:
			_mutex_unlock(dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_unlock\n", __func__);
				return rc;
			}
			break;
		/* Destroy mutex */
		case MUTEX_DESTROY:
			_mutex_destroy(dev_t->laser_focus_mutex);
			if(rc < 0){
				LOG_Handler(LOG_ERR, "%s: fail mutex_destroy\n", __func__);
				return rc;
			}
			break;
		default:
			LOG_Handler(LOG_ERR, "%s: command fail !!\n", __func__);
			break;
	}

	//LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;;
}

static ssize_t Laser_CSCmode_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
        int val;
        char messages[8]="";
        if (len > 8)    len = 8;

        if (copy_from_user(messages, buff, len)) {
                printk("%s commond fail !!\n", __func__);
                return -EFAULT;
        }
        val = (int)simple_strtol(messages, NULL, 10);


        if(val)
                CSCmode = true;
        else
                CSCmode = false;

        printk("Laser CSCmode(%d)\n",CSCmode);

        return len;
}

int Laser_CSCmode_read(struct seq_file *buf, void *v)
{
       seq_printf(buf,"CSCmode(%d)\n", (int)CSCmode);

       return 0;
}

int Laser_CSCmode_open(struct inode *inode, struct  file *file)
{
        return single_open(file, Laser_CSCmode_read, NULL);
}

const struct file_operations Laser_CSCmode_fops = {
        .owner = THIS_MODULE,
        .open = Laser_CSCmode_open,
        .write = Laser_CSCmode_write,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};


static ssize_t dump_Laser_value_check_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val;
	char messages[8]="";
	if (len > 8)	len = 8;

	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);


	g_factory = val;

	printk("Laser g_factory(%d)\n",g_factory);

	return len;
}


int dump_Laser_value_check_read(struct seq_file *buf, void *v)
{
	timedMeasure = !timedMeasure;
	printk("timedMeasure switch to (%d)\n", (int)timedMeasure);
	seq_printf(buf,"timedMeasure switch to (%d)\n", (int)timedMeasure);
	seq_printf(buf,"PASS\n");
       return 0;
}

int dump_Laser_value_check_open(struct inode *inode, struct  file *file)
{
        return single_open(file, dump_Laser_value_check_read, NULL);
}

const struct file_operations dump_Laser_value_check_fops = {
        .owner = THIS_MODULE,
        .open = dump_Laser_value_check_open,
        .write = dump_Laser_value_check_write,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

int Laura_laser_focus_log_contorl_read(struct seq_file *buf, void *v)
{
	return 0;
}

int Laura_laser_focus_log_contorl_open(struct inode *inode, struct file *file)
{
	return single_open(file, Laura_laser_focus_log_contorl_read, NULL);
}

ssize_t Laura_laser_focus_log_contorl_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;

	rc = Laser_Focus_log_contorl(buff, len);

	return rc;
}

const struct file_operations laser_focus_log_contorl_fops = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_log_contorl_open,
	.write = Laura_laser_focus_log_contorl_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};



extern int Read_Kdata_From_File(struct seq_file *vfile, uint16_t *cal_data);
//HPTG proc for CE catch part of K data (10 procs)+++

static ssize_t dump_CE_debug_value1_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{

	char messages[8]="";

	len =(len > 8 ?8:len);
	if (copy_from_user(messages, buff, len)) {
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	thd = (int)simple_strtol(messages, NULL, 10);

	printk("HEPT Threshold %d\n",thd);

	return len;
}

static int dump_CE_debug_value1_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);

	seq_printf(buf,"%d\n",cal_data[3]);
	return 0;
}

static int dump_debug_Kvalue1_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value1_read, NULL);
}

static const struct file_operations dump_debug_Kvalue1_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue1_open,
	.write = dump_CE_debug_value1_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t dump_CE_debug_value2_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{

	char messages[8]="";

	len =(len > 8 ?8:len);
	if (copy_from_user(messages, buff, len)) {
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	limit = (int)simple_strtol(messages, NULL, 10);

	printk("HEPT Limit %d\n",limit);

	return len;
}

static int dump_CE_debug_value2_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);

       seq_printf(buf,"%d\n",cal_data[4]);
	return 0;
}

static int dump_debug_Kvalue2_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value2_read, NULL);
}

static const struct file_operations dump_debug_Kvalue2_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue2_open,
	.write = dump_CE_debug_value2_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static ssize_t dump_CE_debug_value3_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{

	char messages[8]="";

	len =(len > 8 ?8:len);
	if (copy_from_user(messages, buff, len)) {
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	thd_near_mm = (int)simple_strtol(messages, NULL, 10);

	printk("HEPT Limit %d\n",thd_near_mm);

	return len;
}


static int dump_CE_debug_value3_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);

       seq_printf(buf,"%d\n",cal_data[18]);
	return 0;
}

static int dump_debug_Kvalue3_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value3_read, NULL);
}

static const struct file_operations dump_debug_Kvalue3_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue3_open,
	.write = dump_CE_debug_value3_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_CE_debug_value4_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);

       seq_printf(buf,"%d\n",cal_data[20]);
	return 0;
}

static int dump_debug_Kvalue4_open(struct inode *inode, struct  file *file)
{

	return single_open(file, dump_CE_debug_value4_read, NULL);
}

static const struct file_operations dump_debug_Kvalue4_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue4_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_CE_debug_value5_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);

       seq_printf(buf,"%d\n",cal_data[29]);
	return 0;
}

static int dump_debug_Kvalue5_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value5_read, NULL);
}

static const struct file_operations dump_debug_Kvalue5_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue5_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int dump_CE_debug_value6_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);

       seq_printf(buf,"%d\n",cal_data[23]);
	return 0;
}

static int dump_debug_Kvalue6_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value6_read, NULL);
}

static const struct file_operations dump_debug_Kvalue6_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue6_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_CE_debug_value7_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	Read_Kdata_From_File(NULL, cal_data);

       seq_printf(buf,"%d\n",cal_data[32]);
	return 0;
}

static int dump_debug_Kvalue7_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value7_read, NULL);
}

static const struct file_operations dump_debug_Kvalue7_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue7_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_CE_debug_value8_read(struct seq_file *buf, void *v)
{
	int16_t rc = 0, RawConfidence = 0, confidence_level = 0;;

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		return -EBUSY;
	}

	/* Read result confidence level */
	rc = CCI_I2C_RdWord(laura_t, 0x0A, &RawConfidence);
	if (rc < 0){
		return rc;
	}
	confidence_level = (RawConfidence&0x7fff)>>4;

	seq_printf(buf,"%d\n",confidence_level);
	return 0;
}

static int dump_debug_Kvalue8_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value8_read, NULL);
}

static const struct file_operations dump_debug_Kvalue8_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue8_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_CE_debug_value9_read(struct seq_file *buf, void *v)
{
	seq_printf(buf,"-9999\n");
	return 0;
}

static int dump_debug_Kvalue9_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_CE_debug_value9_read, NULL);
}

static const struct file_operations dump_debug_Kvalue9_fops = {
	.owner = THIS_MODULE,
	.open = dump_debug_Kvalue9_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

//HPTG proc for CE catch part of K data (10 procs)---



static int who_am_i_read(struct seq_file *buf, void *v){

	seq_printf(buf, "PASS\n");
	return 0;

}

static int who_am_i_open(struct inode *inode, struct  file *file)
{
	return single_open(file, who_am_i_read, NULL);
}

static const struct file_operations laser_who_am_i_fops = {
	.owner = THIS_MODULE,
	.open = who_am_i_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


/******************** HEPTAGON PROCS +++ ***********************/

extern const struct file_operations Laser_status_check_fops;
extern const struct file_operations Laser_check_producer_fops;

extern const struct file_operations ATD_laser_focus_device_enable_fops;
extern const struct file_operations ATD_laser_focus_device_get_range_fos;
extern const struct file_operations ATD_laser_focus_device_get_range_more_info_fos;

//Laser_get_raw_Kdata_fops: transfer raw Kdata, which derived from mailbox, to library
extern const struct file_operations Laser_calibration_fops;
extern const struct file_operations Laser_get_raw_Kdata_fops;

extern const struct file_operations dump_laser_focus_register_fops;
extern const struct file_operations dump_HPTG_debug_register_fops;


//setK and product_family are seldom to be used
extern const struct file_operations laser_focus_set_K_fops;
extern const struct file_operations laser_focus_product_family;

void HEPTAGON_create_proc_file(void)
{
	proc(STATUS_PROC_FILE, 0664, &Laser_status_check_fops);
	proc(STATUS_PROC_FILE_FOR_CAMERA, 0664, &Laser_check_producer_fops);


	proc(DEVICE_TURN_ON_FILE, 0664, &ATD_laser_focus_device_enable_fops);
	proc(DEVICE_GET_VALUE, 0664, &ATD_laser_focus_device_get_range_fos);
	proc(DEVICE_GET_VALUE_MORE_INFO, 0664, &ATD_laser_focus_device_get_range_more_info_fos);


	proc(DEVICE_SET_CALIBRATION, 0664, &Laser_calibration_fops);
	proc(DEVICE_GET_CALIBRATION_INPUT_DATA, 0664, &Laser_get_raw_Kdata_fops);

	//for debug+
	proc(DEVICE_DUMP_REGISTER_VALUE, 0664, &dump_laser_focus_register_fops);
	proc(DEVICE_DUMP_DEBUG_VALUE, 0664, &dump_HPTG_debug_register_fops);
	proc(DEVICE_LOG_CTRL_FILE, 0664, &laser_focus_log_contorl_fops);
	//for debug-

	//for ce+
	proc(DEVICE_DEBUG_VALUE1, 0664, &dump_debug_Kvalue1_fops);
	proc(DEVICE_DEBUG_VALUE2, 0664, &dump_debug_Kvalue2_fops);
	proc(DEVICE_DEBUG_VALUE3, 0664, &dump_debug_Kvalue3_fops);
	proc(DEVICE_DEBUG_VALUE4, 0664, &dump_debug_Kvalue4_fops);
	proc(DEVICE_DEBUG_VALUE5, 0664, &dump_debug_Kvalue5_fops);
	proc(DEVICE_DEBUG_VALUE6, 0664, &dump_debug_Kvalue6_fops);
	proc(DEVICE_DEBUG_VALUE7, 0664, &dump_debug_Kvalue7_fops);
	proc(DEVICE_DEBUG_VALUE8, 0664, &dump_debug_Kvalue8_fops);
	proc(DEVICE_DEBUG_VALUE9, 0664, &dump_debug_Kvalue9_fops);
	proc(DEVICE_VALUE_CHECK, 0664, &dump_Laser_value_check_fops);
	//for ce-
    proc(DEVICE_CSC_MODE, 0664, &Laser_CSCmode_fops);

	//for dit+
	proc(DEVICE_IOCTL_SET_K, 0664, &laser_focus_set_K_fops);
	proc(DEVICE_IOCTL_PRODUCT_FAMILY, 0444, &laser_focus_product_family);
	//for dit-
}

/******************** HEPTAGON PROCS --- ***********************/


static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};

//static const struct v4l2_subdev_internal_ops msm_laser_focus_internal_ops;

static struct v4l2_subdev_core_ops msm_laser_focus_subdev_core_ops = {
	.ioctl = NULL,
};

static struct v4l2_subdev_ops msm_laser_focus_subdev_ops = {
	.core = &msm_laser_focus_subdev_core_ops,
};




int set_i2c_client(struct msm_laser_focus_ctrl_t *dev_t, struct msm_camera_i2c_client *client){

	/* Assign name for sub device */
	snprintf(dev_t->msm_sd.sd.name, sizeof(dev_t->msm_sd.sd.name),
			"%s", dev_t->sensordata->sensor_name);

	dev_t->act_v4l2_subdev_ops = &msm_laser_focus_subdev_ops;

	/* Set device type as platform device */
	dev_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	dev_t->i2c_client = client;
	if (NULL == dev_t->i2c_client) {
		pr_err("%s i2c_client NULL\n",
			__func__);
		return -EFAULT;

	}
	if (!dev_t->i2c_client->i2c_func_tbl)
		dev_t->i2c_client->i2c_func_tbl = &msm_sensor_cci_func_tbl;

	dev_t->i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!dev_t->i2c_client->cci_client) {
		kfree(dev_t->vreg_cfg.cam_vreg);
		kfree(dev_t);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}
	return 0;

}

int set_cci_client(struct msm_laser_focus_ctrl_t *dev_t){

	struct msm_camera_cci_client *cci_client = NULL;

	cci_client = dev_t->i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = dev_t->cci_master;
	if (dev_t->sensordata->slave_info->sensor_slave_addr)
		cci_client->sid = dev_t->sensordata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;

	return 0;
}

int set_subdev(struct msm_laser_focus_ctrl_t *dev_t, const struct v4l2_subdev_internal_ops *internal_ops){

	v4l2_subdev_init(&dev_t->msm_sd.sd, dev_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&dev_t->msm_sd.sd, dev_t);

	dev_t->msm_sd.sd.internal_ops = internal_ops;
	dev_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(dev_t->msm_sd.sd.name,
		ARRAY_SIZE(dev_t->msm_sd.sd.name), "msm_laser_focus");

	media_entity_init(&dev_t->msm_sd.sd.entity, 0, NULL, 0);
	dev_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	//dev_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_LASER_FOCUS;
	dev_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&dev_t->msm_sd);

	return 0;
}

int set_laser_state(struct msm_laser_focus_ctrl_t *dev_t){

	dev_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	dev_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
	return 0;
}


