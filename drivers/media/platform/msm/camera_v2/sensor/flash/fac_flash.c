#include <fac_flash.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include "msm_flash.h"
#include "msm_camera_dt_util.h"
#include "msm_cci.h"
#include <linux/module.h>
#include <linux/of_gpio.h>
#include "msm_flash.h"
#include "msm_camera_dt_util.h"
#include "msm_cci.h"
#include <fac_flash.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/msm_cam_sensor.h>
#include <soc/qcom/camera2.h>
#include "msm_camera_i2c.h"
#include "msm_sd.h"
#include <linux/proc_fs.h>

#undef CDBG
#undef pr_fmt
#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

/*For ASUS FLASH---*/
//extern int asus_lcd_id;
extern int asus_project_id;
//extern int asus_hw_id;

int32_t msm_flash_i2c_write_table(struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_camera_i2c_reg_setting_array *settings);

int32_t msm_flash_i2c_release(
	struct msm_flash_ctrl_t *flash_ctrl);


int32_t msm_flash_init(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data);

int32_t msm_flash_off(struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data);

int32_t msm_flash_low(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data);
int32_t msm_flash_high(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data);
int32_t msm_flash_release(
	struct msm_flash_ctrl_t *flash_ctrl);
void create_brightness_proc_file(void* ctrl);
void create_asus_flash_trigger_time_proc_file(void* ctrl);

char* sky81296_dump_reg(struct msm_flash_ctrl_t *fctrl );

struct FAC_FLASH_INFO{
	struct msm_flash_ctrl_t * ctrl;
	uint32_t index;
	uint32_t target_index;
	uint32_t flash_op_current;
	uint32_t torch_op_current;
	bool 	 is_firsttime;
	struct proc_dir_entry *proc_file_flash;
};
struct FAC_FLASH_INFO  g_facflash[4]={{0},{0},{0},{0}};

bool g_is_otg_mode=false;
uint32_t g_front_flash_max_current=MAX_FLASH_CURRENT_PISCES_FRONT_OTG_0;
uint32_t g_front_torch_max_current=MAX_TORCH_CURRENT_PISCES_FRONT_OTG_0;
uint32_t g_rear_flash_max_current=MAX_FLASH_CURRENT_PISCES_REAR_OTG_0;
uint32_t g_rear_torch_max_current=MAX_TORCH_CURRENT_PISCES_REAR_OTG_0;

int g_led_flash_count=0;
struct	msm_camera_i2c_reg_setting_array sky81296_reg_off_settings=
{
	.reg_setting_a=
	{
		{0x04, 0x00,0x01},
	},
	.size=1,
	.addr_type=MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type=MSM_CAMERA_I2C_BYTE_DATA,
	.delay=0,
};
			
struct	msm_camera_i2c_reg_setting_array sky81296_reg_low_settings=
{
	.reg_setting_a=
	{
		{0x03, 0x11,0x01},
		{0x04, 0x11,0x01},
	},
	.size=2,
	.addr_type=MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type=MSM_CAMERA_I2C_BYTE_DATA,
	.delay=0,
};
struct	msm_camera_i2c_reg_setting_array sky81296_reg_high_settings=
{
	.reg_setting_a=
	{
		{0x00, 0x0A,0x01},
		{0x01, 0x0A,0x01},
		{0x02, 0xAA,0x01},
		{0x04, 0x22,0x01},
	},
	.size=4,
	.addr_type=MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type=MSM_CAMERA_I2C_BYTE_DATA,
	.delay=0,
};
struct	msm_camera_i2c_reg_setting_array sky81296_reg_init_setting=
{
	.reg_setting_a=
	{
		{0x04, 0x00,0x01},
		{0x06, 0x66,0x01},
		{0x00, 0x0A,0x01},
		{0x01, 0x0A,0x01},
		{0x02, 0xAA,0x01},
		{0x03, 0x77,0x01},
	},
	.size=6,
	.addr_type=MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type=MSM_CAMERA_I2C_BYTE_DATA,
	.delay=0,
};
struct msm_flash_init_info_t pmi8950_init_info=
{
	FLASH_DRIVER_DEFAULT,	
};
struct msm_flash_cfg_data_t pmi8950_init_cfg_data=
{
	CFG_FLASH_INIT,
	.flash_current={200,200,200},
	.flash_duration={1280,1280,1280},
	.cfg.flash_init_info=&pmi8950_init_info,
};
struct msm_flash_cfg_data_t pmi8950_cfg_data=
{
	CFG_FLASH_LOW,
	.flash_current={200,200,200},
	.flash_duration={1280,1280,1280},
	.cfg.flash_init_info=&pmi8950_init_info,
};
//////////////////////////////////////////////////////////
extern struct msm_flash_table msm_i2c_flash_table;
struct msm_sensor_power_setting_array sky81296_power_setting=
{
	.power_setting_a=
	{
		 {
		   .seq_type = SENSOR_GPIO,
		   .seq_val  = SENSOR_GPIO_FL_EN,
		   .config_val = GPIO_OUT_HIGH,
		   .delay = 10,
		},
	},
  .power_setting=NULL,
  .size = 1,
  .power_down_setting_a=
  {
	  {
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_FL_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	  },
  },
  .power_down_setting=NULL,
  .size_down = 1,
};



struct msm_flash_init_info_t sky81296_init_info=
{
	.flash_driver_type=FLASH_DRIVER_DEFAULT,
	.slave_addr=0x37<<1,
	.i2c_freq_mode=I2C_FAST_MODE,
	.power_setting_array=&sky81296_power_setting,
	//.settings=&sky81296_reg_setting,

};
struct msm_flash_cfg_data_t sky81296_init_data=
{
	.cfg_type=CFG_FLASH_INIT,
	.flash_current={200,200,200},
	.flash_duration={1280,1280,1280},
	.cfg.flash_init_info=&sky81296_init_info,
};
char * flash_type_string(int type)
{
	switch(type)
	{
		case FLASH_DRIVER_I2C:return "I2C";
		case FLASH_DRIVER_PMIC:return "PMIC";
		case FLASH_DRIVER_GPIO:return "GPIO";
		case FLASH_DRIVER_DEFAULT:return "DEFAULT";
		default:return "Unkown";
	}
}
int sky81296_off(struct msm_flash_ctrl_t * fctrl)
{
	int rc=-EFAULT;
	CDBG(" E\n");
	if(fctrl->flash_state == MSM_CAMERA_FLASH_INIT)
	{
		rc=msm_flash_i2c_write_table(fctrl,&sky81296_reg_off_settings);
		if (rc < 0)
			printk("%s:%d msm_flash_i2c_write_table failed\n",__func__,__LINE__);
	}
	return 0;
}

int sky81296_init(struct msm_flash_ctrl_t * fctrl)
{
	int rc=-EFAULT;
	int i=0;
	struct msm_camera_cci_client *cci_client = NULL;
	CDBG(" E\n");
	do
	{
		if(asus_project_id!=ASUS_ZD552KL)
		{
			printk("%s:%d invalid asus_project_id=%d \n",__func__,__LINE__,asus_project_id);
			rc=-EFAULT;break;
		}
		if(!fctrl  )
		{
			printk("%s:%d invalid parameter \n",__func__,__LINE__);
			rc=-EFAULT;break;
		}
		if(fctrl->flash_state == MSM_CAMERA_FLASH_INIT)
			{printk("%s:%d fctrl already inited\n",__func__,__LINE__);rc=0;break;}
		if( fctrl->flash_driver_type!=FLASH_DRIVER_I2C )
			{printk("%s:%d flash type mismatch\n",__func__,__LINE__);rc=-EFAULT;break;}
		fctrl->func_tbl = &msm_i2c_flash_table.func_tbl;
		memcpy(&fctrl->power_setting_array,(void *)&sky81296_power_setting,
			sizeof(struct msm_sensor_power_setting_array));
		
		if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			cci_client = fctrl->flash_i2c_client.cci_client;
			cci_client->sid = sky81296_init_info.slave_addr >> 1;
			cci_client->retries = 3;
			cci_client->id_map = 0;
			cci_client->i2c_freq_mode = sky81296_init_info.i2c_freq_mode;
		}
		
		fctrl->power_info.power_setting =
			fctrl->power_setting_array.power_setting_a;
		fctrl->power_info.power_down_setting =
			fctrl->power_setting_array.power_down_setting_a;
		fctrl->power_info.power_setting_size =
			fctrl->power_setting_array.size;
		fctrl->power_info.power_down_setting_size =
			fctrl->power_setting_array.size_down;
		/////////////////////////////////////////
		
		for(i=0;i<fctrl->flash_num_sources  &&  i<MAX_LED_TRIGGERS;i++)
		{
			fctrl->flash_max_current[i]=MAX_FLASH_CURRENT_SKY81296/2;
			fctrl->torch_max_current[i]=MAX_TORCH_CURRENT_SKY81296/2;
			fctrl->torch_op_current[i]=fctrl->torch_max_current[i];
			fctrl->flash_op_current[i]=fctrl->flash_max_current[i];
			fctrl->flash_max_duration[i]=MAX_FLASH_TIMEOUT_SKY81296;
		}
		rc=msm_camera_power_up(&fctrl->power_info,fctrl->flash_device_type,&fctrl->flash_i2c_client);
		if (rc < 0)
			{printk("%s:%d msm_camera_power_up failed\n",__func__,__LINE__);break;}
		rc=msm_flash_i2c_write_table(fctrl,&sky81296_reg_init_setting);
		if (rc < 0)
			{printk("%s:%d msm_flash_i2c_write_table failed\n",__func__,__LINE__);break;}
		fctrl->flash_state = MSM_CAMERA_FLASH_INIT;
		printk("%s:%d sky81296 init done.",__func__,__LINE__);
		rc=0;
	}while(0);
	
	return rc;
}
bool asus_flash_init(struct msm_flash_ctrl_t *fctrl)
{
	bool ret=false;
	CDBG(" E\n");
	do
	{
		if(!fctrl)
			{printk("%s fctrl is NULL\n",__func__);break;}
		if(fctrl->flash_driver_type==FLASH_DRIVER_I2C)
			{ret= ( 0==sky81296_init(fctrl));break;}
		else // if(fctrl->flash_driver_type==FLASH_DRIVER_PMIC)
			{ret=( 0==msm_flash_init(fctrl,&pmi8950_init_cfg_data));break;}
		printk("%s invalid flash type=%x\n",__func__,fctrl->flash_driver_type);
	}while(0);
	return ret;
}

int sky81296_current_set_low(struct msm_flash_ctrl_t *fctrl, uint32_t intensity1, uint32_t intensity2)
{
	int rc = 0,i = 0;
	uint32_t val[2]={0};
	uint32_t uMaxCurrent= MAX_TORCH_CURRENT_SKY81296;
	val[0] = intensity1;
	val[1] = intensity2;
	CDBG(" E\n");
	for (i=0;i<2;i++) {
		if(uMaxCurrent<val[i])val[i]=uMaxCurrent;
		val[i]=(val[i]/TORCH_MODE_STEP_SKY81296)&0xf;
		if(val[i]>0)val[i]--;
		fctrl->torch_op_current[i]=(val[i]+1)*TORCH_MODE_STEP_SKY81296;//set actual current
	}
	sky81296_reg_low_settings.reg_setting_a[0].reg_data=( (val[1]&0xf) << 4 | (val[0]&0xf) );
	sky81296_reg_low_settings.reg_setting_a[1].reg_data=0;
	sky81296_reg_low_settings.reg_setting_a[1].reg_data|= intensity1>0?0x1:0;
	sky81296_reg_low_settings.reg_setting_a[1].reg_data|= intensity2>0?0x10:0;

	for(i=0;i<sky81296_reg_low_settings.size;i++)
	{
		printk("%s %x=0x%x\n",__func__
			,sky81296_reg_low_settings.reg_setting_a[i].reg_addr,sky81296_reg_low_settings.reg_setting_a[i].reg_data
			);
	}
	rc=msm_flash_i2c_write_table(fctrl,&sky81296_reg_low_settings);
	if (rc < 0)
	{printk("%s:%d msm_flash_i2c_write_table failed\n",__func__,__LINE__);}
	return rc;
}

int sky81296_current_set_high(struct msm_flash_ctrl_t *fctrl,
	uint32_t intensity1,uint32_t intensity2 )
{
	int rc = 0,i = 0;
	uint32_t val[2]={0};
	uint32_t uMaxCurrent=MAX_FLASH_CURRENT_SKY81296;
	val[0] = intensity1;
	val[1] = intensity2;
	CDBG(" E\n");
	for (i=0;i<2;i++) {
		if(val[i]>uMaxCurrent)val[i]=uMaxCurrent;
		printk("%s val[%d]=%d\n",__func__,i,val[i]);
		if(val[i]<250)
			val[i]=0;
		else
			val[i]=(val[i]-250)/FLASH_MODE_STEP_SKY81296;
		fctrl->flash_op_current[i]=val[i]*FLASH_MODE_STEP_SKY81296+250;//set actual current
	}
	sky81296_reg_high_settings.reg_setting_a[0].reg_data=val[0];
	sky81296_reg_high_settings.reg_setting_a[1].reg_data=val[1];
	sky81296_reg_high_settings.reg_setting_a[3].reg_data=0;
	sky81296_reg_high_settings.reg_setting_a[3].reg_data|= intensity1>0?0x2:0;
	sky81296_reg_high_settings.reg_setting_a[3].reg_data|= intensity2>0?0x20:0;
	for(i=0;i<sky81296_reg_high_settings.size;i++)
	{
		printk("%s %x=0x%x\n",__func__
			,sky81296_reg_high_settings.reg_setting_a[i].reg_addr,sky81296_reg_high_settings.reg_setting_a[i].reg_data
			);
	}
	rc=msm_flash_i2c_write_table(fctrl,&sky81296_reg_high_settings);
	if (rc < 0)
	{printk("%s:%d msm_flash_i2c_write_table failed\n",__func__,__LINE__);}
	return rc;
}


void asus_flash_release(struct msm_flash_ctrl_t *fctrl )
{
	printk("%s:%d in flash_state=%d\n",__func__,__LINE__,fctrl->flash_state);
	if(fctrl->flash_driver_type==FLASH_DRIVER_I2C)
		msm_flash_i2c_release(fctrl);
	else
		msm_flash_release(fctrl);
}
char g_strBuffer[1000]={0};
char* sky81296_dump_reg(struct msm_flash_ctrl_t *fctrl )
{
	int rc=0;
	char temp[100]={0};
	CDBG(" E\n");
	if(asus_flash_init(fctrl))
	{
		//dump sky81296 regitster
		int i=0;
		char data_buf[12]={0};
		uint32_t num_byte = 12;
		if (fctrl->flash_i2c_client.i2c_func_tbl!=NULL)
		{
			rc = fctrl->flash_i2c_client.i2c_func_tbl->i2c_read_seq(
			&fctrl->flash_i2c_client,0,data_buf,num_byte);
			if (rc < 0){
				pr_err("%s:%d i2c_read_seq failed\n", __func__, __LINE__);
			}
			else{
				memset(g_strBuffer,0,100);
				for(i=0;i<12;i++)
				{
					sprintf(temp,"\nreg[%02d]=0x%02x",i,data_buf[i]);
					strcat(g_strBuffer,temp);
				}
			}	printk("%s %s \n",__func__,g_strBuffer);
		}
		asus_flash_release(fctrl);
	}
	return g_strBuffer;
}

static ssize_t asus_flash_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{//TODO:
	int ret=0;
	char buff[1024]={0};
	int desc = 0;
	uint32_t torch_op_current=0,flash_op_current=0;
	struct FAC_FLASH_INFO * facflash=(struct FAC_FLASH_INFO *)PDE_DATA(file_inode(dev));
	if(!facflash  ||  !facflash->ctrl)
	{
		desc+=sprintf(buff+desc,"Flash Control not inited\n");
	}else
	{
		torch_op_current=facflash->torch_op_current;
		flash_op_current=facflash->flash_op_current;
		desc+=sprintf(buff+desc,"flash type=%s\n",flash_type_string(facflash->ctrl->flash_driver_type));
		desc+=sprintf(buff+desc,"	 flash_current:%d\n",flash_op_current);
		desc+=sprintf(buff+desc,"	 torch_current:%d\n",torch_op_current);
		//desc+=sprintf(buff+desc,"	 du:led0=%d ,led1=%d\n"
		//	,facflash->ctrl->flash_max_duration[0],facflash->ctrl->flash_max_duration[1]);
		if(facflash->ctrl->flash_driver_type==FLASH_DRIVER_I2C)
			desc+=sprintf(buff+desc,"%s\n",sky81296_dump_reg(facflash->ctrl));
	}
	ret=simple_read_from_buffer(buffer,count,ppos,buff,desc);	
	return ret;
}
void asus_update_config(struct FAC_FLASH_INFO *facflash,uint32_t mode)
{
	struct msm_flash_ctrl_t *fctrl=NULL;
	CDBG(" E\n");
	if(!facflash)
	{printk("%s:%d invalid parameters\n",__func__,__LINE__);return;}
	fctrl=facflash->ctrl;
	if(asus_flash_init(fctrl))
	{
		printk("%s:%d [camera_flash] driver_type=%d,mode=%d,cur[0]=%d,cur[1]=%d\n"
			,__func__,__LINE__,fctrl->flash_driver_type,mode
			,fctrl->torch_op_current[0],fctrl->torch_op_current[1]);
		if(fctrl->flash_driver_type==FLASH_DRIVER_I2C)
		{
			if(mode==FAC_FLASH_MODE_TORCH)
			{
				if(fctrl->torch_op_current[0]==0  &&  fctrl->torch_op_current[1]==0)
					sky81296_off(fctrl);
				else
					sky81296_current_set_low(fctrl,fctrl->torch_op_current[0],fctrl->torch_op_current[1]);
				
			}
			if(mode==FAC_FLASH_MODE_FLASH)
			{
				sky81296_current_set_high(fctrl,fctrl->flash_op_current[0],fctrl->flash_op_current[1]);
			}
		}
		else //if(fctrl->flash_driver_type==FLASH_DRIVER_PMIC)
		{
			if(mode==FAC_FLASH_MODE_TORCH)
			{
				if(fctrl->torch_op_current[0]==0  &&  fctrl->torch_op_current[1]==0)
				{
					pmi8950_cfg_data.flash_current[0]=0;
					pmi8950_cfg_data.flash_current[1]=0;
					pmi8950_cfg_data.cfg_type=CFG_FLASH_OFF;
					msm_flash_off(fctrl,&pmi8950_cfg_data);
				}else
				{
					pmi8950_cfg_data.flash_current[0]=fctrl->torch_op_current[0];
					pmi8950_cfg_data.flash_current[1]=fctrl->torch_op_current[1];
					pmi8950_cfg_data.cfg_type=CFG_FLASH_LOW;
					msm_flash_low(fctrl,&pmi8950_cfg_data);
				}
				
			}
			if(mode==FAC_FLASH_MODE_FLASH)
			{
				pmi8950_cfg_data.flash_current[0]=fctrl->flash_op_current[0];
				pmi8950_cfg_data.flash_current[1]=fctrl->flash_op_current[1];
				pmi8950_cfg_data.cfg_type=CFG_FLASH_HIGH;
				msm_flash_high(fctrl,&pmi8950_cfg_data);
			}
		}
	}
}
void asus_flash_config(struct FAC_FLASH_INFO *facflash, uint32_t val0, uint32_t val1,int mode)
{
	uint32_t imax=0;
	uint32_t icurr0=0,icurr1=0;
	CDBG(" E\n");
	if(!facflash  ||  !facflash->ctrl)
	{
		printk("%s:%d invalid parameters\n",__func__,__LINE__);
		return;
	}
	{
		printk("%s:%d project=%d,index=%d,target_index=%d,val0=%d,val1=%d,type=%d,mode=%d,is_otg_mode=%d\n",__func__,__LINE__
			,asus_project_id,facflash->index
			,facflash->target_index
			,val0,val1
			,facflash->ctrl->flash_driver_type
			,mode,g_is_otg_mode
			);
		
		if(mode==FAC_FLASH_MODE_TORCH)
		{
			imax=asus_project_id==ASUS_ZD552KL?g_front_torch_max_current:g_rear_torch_max_current;
			val0=val0<imax?val0:imax;
			val1 =val1 <imax?val1:imax;
			icurr0=facflash->ctrl->torch_op_current[0];
			icurr1=facflash->ctrl->torch_op_current[1];
			facflash->ctrl->torch_op_current[0]=val0;
			facflash->ctrl->torch_op_current[1]=val1;
			facflash->torch_op_current=facflash->ctrl->torch_op_current\
				[facflash->target_index];
			g_facflash[facflash->index^1].torch_op_current=facflash->ctrl->torch_op_current\
				[facflash->target_index^1];
			asus_update_config(facflash,mode);
			facflash->ctrl->torch_op_current[0]=icurr0;
			facflash->ctrl->torch_op_current[1]=icurr1;
		}
		if(mode==FAC_FLASH_MODE_FLASH)
		{
			imax=asus_project_id==ASUS_ZD552KL?g_front_flash_max_current:g_rear_flash_max_current;
			val0=val0<imax?val0:imax;
			val1 =val1 <imax?val1:imax;
			icurr0=facflash->ctrl->torch_op_current[0];
			icurr1=facflash->ctrl->torch_op_current[1];
			facflash->ctrl->flash_op_current[0]=val0;
			facflash->ctrl->flash_op_current[1]=val1;
			facflash->flash_op_current=facflash->ctrl->flash_op_current\
				[facflash->target_index];
			g_facflash[facflash->index^1].torch_op_current=facflash->ctrl->torch_op_current\
				[facflash->target_index^1];
			asus_update_config(facflash,mode);
			facflash->ctrl->flash_op_current[0]=icurr0;
			facflash->ctrl->flash_op_current[1]=icurr1;
		}
		
	}
}

static ssize_t asus_flash_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	uint32_t mode = 0,val[2]={0},argc=0;
	struct FAC_FLASH_INFO * facflash=(struct FAC_FLASH_INFO *)PDE_DATA(file_inode(dev));
	CDBG(" E\n");
	argc=sscanf(buf, "%d %d %d",&mode, &val[0],&val[1]);
	if(argc<2)
	{
		printk("%s:%d invalid parameter\n",__func__,__LINE__);
	}else
	{
		if(!facflash  ||  !facflash->ctrl)
		{
			printk("%s:%d invalid internal parameter\n",__func__,__LINE__);
			return count;
		}
		if(argc==2)val[1]=val[0];
		if(facflash  &&  facflash->ctrl)
		{
			if (mode==FAC_FLASH_MODE_RELEASE)
			{
				asus_flash_release(facflash->ctrl);
				return count;
			}
			if(asus_flash_init(facflash->ctrl))
			{
				//set the other led current to last value or 0 for first time use
				if(argc==2)
					val[facflash->target_index^1]=0;
				asus_flash_config(facflash,val[0],val[1],mode);
			}
		}
	}
	return count;
}
void adjust_max_flash_current(bool is_otg_mode)
{
	int i=0;
	CDBG(" E is_otg_mode=%d\n",is_otg_mode);
	g_is_otg_mode=is_otg_mode;
	switch(asus_project_id)
	{
		case ASUS_ZD552KL:
			g_rear_flash_max_current=g_is_otg_mode?MAX_FLASH_CURRENT_PISCES_REAR_OTG_1:MAX_FLASH_CURRENT_PISCES_REAR_OTG_0;
			g_front_flash_max_current=g_is_otg_mode?MAX_FLASH_CURRENT_PISCES_FRONT_OTG_1:MAX_FLASH_CURRENT_PISCES_FRONT_OTG_0;
			g_rear_torch_max_current=g_is_otg_mode?MAX_TORCH_CURRENT_PISCES_REAR_OTG_1:MAX_TORCH_CURRENT_PISCES_REAR_OTG_0;
			g_front_torch_max_current=g_is_otg_mode?MAX_TORCH_CURRENT_PISCES_FRONT_OTG_1:MAX_TORCH_CURRENT_PISCES_FRONT_OTG_0;
			break;
		case ASUS_ZS550KL:
			//g_front_flash_max_current=g_is_otg_mode?MAX_FLASH_CURRENT_FRONT_OTG_1:MAX_FLASH_CURRENT_FRONT_OTG_0;
			//g_front_torch_max_current=g_is_otg_mode?MAX_TORCH_CURRENT_FRONT_OTG_1:MAX_TORCH_CURRENT_FRONT_OTG_0;
			g_rear_flash_max_current=g_is_otg_mode?MAX_FLASH_CURRENT_AQUARIUS_REAR_OTG_1:MAX_FLASH_CURRENT_AQUARIUS_REAR_OTG_0;
			g_rear_torch_max_current=g_is_otg_mode?MAX_TORCH_CURRENT_AQUARIUS_REAR_OTG_1:MAX_TORCH_CURRENT_AQUARIUS_REAR_OTG_0;
			break;
	}
	for(i=0;i<4;i++)
	{
		if(g_facflash[i].ctrl==NULL)
		{
			//printk("%s:%d ctrl not found\n",__func__,__LINE__);//TODO:remove this
		}else
		{
			if(g_facflash[i].ctrl->active)
			{
				asus_update_config(&g_facflash[i],FAC_FLASH_MODE_TORCH);
			}
		}
	}
}

static const struct file_operations asus_flash_proc_fops = {
	.read = asus_flash_show,
	.write = asus_flash_store,
	//.open=asus_flash_open,
};

void create_flash_proc_file(void* ctrl)
{
	char pro_file_name[100]={0};
	uint32_t driver_type=0;
	driver_type=((struct msm_flash_ctrl_t *)ctrl)->flash_driver_type;
	adjust_max_flash_current(false);
	if(((struct msm_flash_ctrl_t *)ctrl)->flash_driver_type == FLASH_DRIVER_I2C)
	{
		g_facflash[0].ctrl=ctrl;
		g_facflash[0].index=0;
		g_facflash[0].target_index=0;
		g_facflash[0].is_firsttime=1;
		g_facflash[0].torch_op_current=g_rear_torch_max_current;
		g_facflash[0].flash_op_current=g_rear_flash_max_current;
		snprintf(pro_file_name,ARRAY_SIZE(pro_file_name),"%s",PROC_FILE_ASUS_FLASH_1);
		g_facflash[0].proc_file_flash = proc_create_data(pro_file_name, 0666, NULL, &asus_flash_proc_fops, &g_facflash[0]);
		if(!g_facflash[0].proc_file_flash)
			printk("%s proc_create_data %s failed\n",__func__,pro_file_name);
		//////////////////////////////////////////////////////////////////////////////////////////////////////
		g_facflash[1].ctrl=ctrl;
		g_facflash[1].index=1;
		g_facflash[1].target_index=1;
		g_facflash[1].is_firsttime=1;
		g_facflash[1].torch_op_current=g_rear_torch_max_current;
		g_facflash[1].flash_op_current=g_rear_flash_max_current;
		snprintf(pro_file_name,ARRAY_SIZE(pro_file_name),"%s",PROC_FILE_ASUS_FLASH_2);
		g_facflash[1].proc_file_flash = proc_create_data(pro_file_name, 0666, NULL, &asus_flash_proc_fops, &g_facflash[1]);
		if(!g_facflash[1].proc_file_flash)
			printk("%s proc_create_data %s failed\n",__func__,pro_file_name);
		//create brightness proc file
		if(asus_project_id==ASUS_ZD552KL)
		{
			g_facflash[0].target_index=1;//for pisces hw connection
			g_facflash[1].target_index=0;
			create_brightness_proc_file(&g_facflash[0]);  //create proc fille for setting  brightness
			create_asus_flash_trigger_time_proc_file(&g_facflash[0]); 
		}
	}
	else //if(((struct msm_flash_ctrl_t *)ctrl)->flash_driver_type == FLASH_DRIVER_PMIC)    //change for CS codebase update 
	{
		g_facflash[2].ctrl=ctrl;
		g_facflash[2].index=2;
		g_facflash[2].target_index=0;
		g_facflash[2].is_firsttime=1;
		//create flash proc file
		if(asus_project_id==ASUS_ZD552KL)
		{
			g_facflash[2].torch_op_current=g_front_torch_max_current;
			g_facflash[2].flash_op_current=g_front_flash_max_current;
			snprintf(pro_file_name,ARRAY_SIZE(pro_file_name),"%s",PROC_FILE_ASUS_FLASH_3);
		}
		else
		{
			g_facflash[2].torch_op_current=g_rear_torch_max_current;
			g_facflash[2].flash_op_current=g_rear_flash_max_current;
			snprintf(pro_file_name,ARRAY_SIZE(pro_file_name),"%s",PROC_FILE_ASUS_FLASH_1);
			create_brightness_proc_file(&g_facflash[2]);  //create proc fille for setting  brightness
			create_asus_flash_trigger_time_proc_file(&g_facflash[2]); 
		}
		g_facflash[2].proc_file_flash = proc_create_data(pro_file_name, 0666, NULL, &asus_flash_proc_fops, &g_facflash[2]);
		if(!g_facflash[2].proc_file_flash)
			printk("%s proc_create_data %s failed\n",__func__,pro_file_name);
		///////////////////////////////////////////////////////////////////////////////////////////
		g_facflash[3].ctrl=ctrl;
		g_facflash[3].index=3;
		g_facflash[3].target_index=1;
		g_facflash[3].is_firsttime=1;
		if(asus_project_id==ASUS_ZD552KL)
		{
			g_facflash[3].torch_op_current=g_front_torch_max_current;
			snprintf(pro_file_name,ARRAY_SIZE(pro_file_name),"%s",PROC_FILE_ASUS_FLASH_4);
		}
		else
		{
			g_facflash[3].torch_op_current=g_rear_torch_max_current;
			snprintf(pro_file_name,ARRAY_SIZE(pro_file_name),"%s",PROC_FILE_ASUS_FLASH_2);
		}
		if(asus_project_id==ASUS_ZS550KL)
		{
			g_facflash[2].target_index=1;//for AQU hw connection
			g_facflash[3].target_index=0;
		}
		g_facflash[3].proc_file_flash = proc_create_data(pro_file_name, 0666, NULL, &asus_flash_proc_fops, &g_facflash[3]);
		if(!g_facflash[3].proc_file_flash)
			printk("%s proc_create_data %s failed\n",__func__,pro_file_name);
	}
}

//sheldon Add flash_brightness_proc +++
#define	FLASH_BRIGHTNESS_PROC_FILE	"driver/asus_flash_brightness"
static struct proc_dir_entry *flash_brightness_proc_file;
static int last_flash_brightness_value;



static int flash_brightness_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%d\n", last_flash_brightness_value);
    return 0;
}

static int flash_brightness_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, flash_brightness_proc_read, NULL);
}
static ssize_t flash_brightness_proc_write(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	uint32_t set_val0 = 0,set_val1=0,now_flash_brightness_value = 0;
	struct FAC_FLASH_INFO * facflash=(struct FAC_FLASH_INFO *)PDE_DATA(file_inode(dev));
	if(sscanf(buf, "%d", &now_flash_brightness_value)<1)
	{
		printk("%s invalid parameter\n",__func__);
	}
	else
	{
		if(now_flash_brightness_value>100)now_flash_brightness_value=100;
		set_val0 = 135*now_flash_brightness_value/100;
		last_flash_brightness_value = set_val0;
		set_val1=set_val0;
		if(asus_project_id==ASUS_ZS550KL)
			set_val0=0;
		else
			set_val1=0;
		asus_flash_config(facflash,set_val0,set_val1,FAC_FLASH_MODE_TORCH);
	}
	return count;
}

static const struct file_operations flash_brightness_fops = {
	.owner = THIS_MODULE,
	.open = flash_brightness_proc_open,
	.read = seq_read,
	.write = flash_brightness_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};
void create_brightness_proc_file(void* ctrl)
{
    last_flash_brightness_value = 0;
    flash_brightness_proc_file = proc_create_data(FLASH_BRIGHTNESS_PROC_FILE, 0666, NULL, &flash_brightness_fops,ctrl);
    if (flash_brightness_proc_file) {
    	printk("%s [AsusFlashBrightness]flash_brightness_proc_file sucessed!\n", __func__);
    } else {
    	printk("%s [AsusFlashBrightness]flash_brightness_proc_file failed!\n", __func__);
    }
}
//sheldon Add flash_brightness_proc ---

//asus bsp ralf>> Add zenflash contrl trigger +++
#define	FLASH_TRIGGER_PROC_FILE	"driver/asus_flash_trigger_time"
static struct proc_dir_entry *asus_flash_trigger_time_proc_file;
uint32_t g_asus_flash_tigger_current=500;
static int asus_flash_trigger_time_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%d\n", g_asus_flash_tigger_current);
    return 0;
}

static int asus_flash_trigger_time_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, asus_flash_trigger_time_proc_read, NULL);
}
#define ZENFLASH_DEFAULT_CURRENT 800
#define ZENFLASH_MAX_CURRENT     900
#define ZENFLASH_MAX_TIMEOUT     1000

static ssize_t asus_flash_trigger_time_proc_write(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	uint32_t timeout = 0,flash_current = ZENFLASH_DEFAULT_CURRENT,argc=0;
	struct FAC_FLASH_INFO * facflash=(struct FAC_FLASH_INFO *)PDE_DATA(file_inode(dev));
	argc=sscanf(buf, "%d %d", &timeout,&flash_current);
	if(argc<1)
	{
		printk("%s invalid parameter\n",__func__);
	}
	else
	{
		timeout=timeout>ZENFLASH_MAX_TIMEOUT?ZENFLASH_MAX_TIMEOUT:timeout;
		flash_current=argc==1?ZENFLASH_DEFAULT_CURRENT:flash_current;
		flash_current=flash_current>ZENFLASH_MAX_CURRENT?ZENFLASH_MAX_CURRENT:flash_current;
		g_asus_flash_tigger_current=flash_current;
		printk("%s:%d timeout=%d,current=%d\n",__func__,__LINE__,timeout,flash_current);
		if(asus_project_id==ASUS_ZS550KL)
			asus_flash_config(facflash,0,flash_current,FAC_FLASH_MODE_FLASH);
		else
			asus_flash_config(facflash,flash_current,0,FAC_FLASH_MODE_FLASH);
		msleep(timeout);
		asus_flash_config(facflash,0,0,FAC_FLASH_MODE_FLASH);
	}
	return count;
}

static const struct file_operations asus_flash_trigger_time_fops = {
	.owner = THIS_MODULE,
	.open = asus_flash_trigger_time_proc_open,
	.read = seq_read,
	.write = asus_flash_trigger_time_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};
void create_asus_flash_trigger_time_proc_file(void* ctrl)
{
    asus_flash_trigger_time_proc_file = proc_create_data(FLASH_TRIGGER_PROC_FILE, 0666, NULL, &asus_flash_trigger_time_fops,ctrl);
    if (asus_flash_trigger_time_proc_file) {
    	printk("%s [asus_flash_trigger_time]flash_brightness_proc_file sucessed!\n", __func__);
    } else {
    	printk("%s [asus_flash_trigger_time]flash_brightness_proc_file failed!\n", __func__);
    }
}
//sheldon Add flash_brightness_proc ---


/*For ASUS FLASH---*/

