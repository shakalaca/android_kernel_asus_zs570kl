#include "fac_camera.h"

u8 front_otp_data[OTP_DATA_LEN_BYTE];	//store front_otp data
u8 rear_otp_data[OTP_DATA_LEN_BYTE];	//store front_otp data
u8 front_otp_data_bank1[OTP_DATA_LEN_BYTE];	//store front_otp bank1 data
u8 rear_otp_data_bank1[OTP_DATA_LEN_BYTE];	//store rear_otp bank1 data
u8 front_otp_data_bank2[OTP_DATA_LEN_BYTE];	//store front_otp bank2 data
u8 rear_otp_data_bank2[OTP_DATA_LEN_BYTE];	//store rear_otp bank2 data
u8 front_otp_data_bank3[OTP_DATA_LEN_BYTE];	//store front_otp bank3 data
u8 rear_otp_data_bank3[OTP_DATA_LEN_BYTE];	//store rear_otp bank3 data

#ifdef FAC_DEBUG
#undef CDBG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#define PDAF_INFO_MAX_SIZE 1024
uint16_t g_sensor_id[MAX_CAMERAS]={0};
struct msm_sensor_ctrl_t *g_sensor_ctrl[MAX_CAMERAS]={0};

struct proc_dir_entry *pdaf_file;
char *pdaf_info = NULL;

// Sheldon_Li add for init ctrl++
int fcamera_power_up( struct msm_sensor_ctrl_t *s_ctrl)
{  
	  int rc = 0;
	  rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
	  /*rc = msm_camera_power_down(&s_ctrl->sensordata->power_info,
	  						      s_ctrl->sensor_device_type,
							      s_ctrl->sensor_i2c_client);*/
	  
	  if(rc == 0){
		s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
	  }
	  return rc;
}

int fcamera_power_down( struct msm_sensor_ctrl_t *s_ctrl)
{  
	  int rc = 0;
	  rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
	 /* rc = msm_camera_power_down(&s_ctrl->sensordata->power_info,
	  						      s_ctrl->sensor_device_type,
							      s_ctrl->sensor_i2c_client);*/					      
	 
	  if(rc == 0){
		s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
	  }
	  return rc;
}
// Sheldon_Li add for init ctrl--


void set_sensor_info(int camera_id,struct msm_sensor_ctrl_t* ctrl,uint16_t sensor_id)
{
	if(camera_id<MAX_CAMERAS)
	{
		g_sensor_id[camera_id]=sensor_id;
		g_sensor_ctrl[camera_id]=ctrl;
	}
}

void create_otp_proc_file(struct msm_camera_sensor_slave_info *slave_info)
{
	if(CAMERA_0 == slave_info->camera_id){
                        sheldon_debug("[sheldon]Create_rear_camera_otp_file for id = %x\n",slave_info->sensor_id_info.sensor_id);
			if(slave_info->sensor_id_info.sensor_id==SENSOR_ID_T4K37
				||slave_info->sensor_id_info.sensor_id==SENSOR_ID_T4K35){
				
				read_rear_otp_asus(SENSOR_ID_T4K37);
				
				create_rear_otp_proc_file();
				create_rear_bank1_proc_file();
				create_rear_bank2_proc_file();
				create_rear_bank3_proc_file();
			}
			else if(slave_info->sensor_id_info.sensor_id==SENSOR_ID_IMX298){
				create_dump_eeprom_file(0);
				create_thermal_file(0);
			 }
	}else if(CAMERA_1 == slave_info->camera_id){
			sheldon_debug("[sheldon]Create_front_camera_otp_file for id = %x\n",slave_info->sensor_id_info.sensor_id);
			if(slave_info->sensor_id_info.sensor_id==SENSOR_ID_OV8856){
				
				read_front_otp_asus(SENSOR_ID_OV8856);
				
				create_front_otp_proc_file();
				create_front_bank1_proc_file();
				create_front_bank2_proc_file();
				create_front_bank3_proc_file();
			}
			else if(slave_info->sensor_id_info.sensor_id==SENSOR_ID_IMX298){
				create_dump_eeprom_file(1);
				create_thermal_file(1);
			}		
	}
}

int msm_sensor_testi2c(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc;
	int power_self = 0;
	struct msm_camera_power_ctrl_t *power_info;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;
	uint16_t chipid;
	uint32_t retry = 0;

	if (!s_ctrl) {
		pr_err("%s:%d failed: %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}

	power_info = &s_ctrl->sensordata->power_info;
	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;

	if (!power_info || !sensor_i2c_client || !slave_info ||
		!sensor_name) {
		pr_err("%s:%d failed: %p %p %p %p\n",
			__func__, __LINE__, power_info,
			sensor_i2c_client, slave_info, sensor_name);
		return -EINVAL;
	}

	     if(s_ctrl->sensor_state == MSM_SENSOR_POWER_DOWN){
		   rc = fcamera_power_up(s_ctrl); //power on first
		   if(rc < 0){
	  		pr_err("%s:%d camera power up failed!\n",__func__, __LINE__);
			return -EINVAL;
		   }
		   power_self = 1;
	     }
	   
	for (retry = 0; retry < 5; retry++) {
	  rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
		sensor_i2c_client, 0x0100,
		&chipid, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0) {
			   rc = fcamera_power_down(s_ctrl); //power down if read failed
			   if(rc < 0){
		  		pr_err("%s:%d camera power down failed!\n",__func__, __LINE__);
				return rc;
			   }
			return -EINVAL;
		}

		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
		sensor_i2c_client, 0x0100,
		chipid, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0) {		
			   rc = fcamera_power_down(s_ctrl); //power down if write failed
			   if(rc < 0){
		  		pr_err("%s:%d camera power down failed!\n",__func__, __LINE__);
				return rc;
			   }
			return -EINVAL;
		}
	}
	   if(s_ctrl->sensor_state == MSM_SENSOR_POWER_UP && power_self == 1){
		   rc = fcamera_power_down(s_ctrl); //power down end
		   if(rc < 0){
	  		pr_err("%s:%d camera power down failed!\n",__func__, __LINE__);
			return rc;
		   }
		   power_self = 0;
	   }
	return rc;
}

  ssize_t rear_otp_proc_show_asus(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, buflen = 0;
	for( i = 0; i < OTP_DATA_LEN_WORD * 2; i++)
		buflen += sprintf(buf + i * 5, "0x%02X ",rear_otp_data[i]);
	buflen += sprintf(buf + i * 5, "\n");
	return buflen;
}

  ssize_t front_otp_proc_show_asus(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, buflen = 0;
	for( i = 0; i < OTP_DATA_LEN_WORD * 2; i++)
		buflen += sprintf(buf + i * 5, "0x%02X ",front_otp_data[i]);
	buflen += sprintf(buf + i * 5, "\n");
	return buflen;
}

int read_rear_otp_asus(unsigned short sensor_id)
{	
	int rc;
	
         if(sensor_id == SENSOR_ID_T4K37){
		rc = t4k37_35_otp_read();
		if(rc < 0){
  			pr_err("%s:%d t4k37 or t4k35 otp read failed!\n",__func__, __LINE__);
		}
	}
	 
	return 0;	
}

int read_front_otp_asus(unsigned short sensor_id)
{
         int rc;
		 
         if(sensor_id == SENSOR_ID_OV8856){
		rc = ov8856_otp_read();
		if(rc < 0){
  			pr_err("%s:%d ov8856_otp_read failed!\n",__func__, __LINE__);
		}
	}
	 
	return 0;
}

int t4k37_35_otp_read(void)
{
	struct msm_sensor_ctrl_t *s_ctrl = g_sensor_ctrl[CAMERA_0];
	struct msm_camera_i2c_client *sensor_i2c_client;
	const char *sensor_name;
	int rc = 0,i,j;
	//int flag = 3; //find the otp data index
	u16 reg_val = 0;
	u16 mFind=0;
	u16 mFlag=0;
	u8 read_value[OTP_DATA_LEN_BYTE];
	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	sensor_name = s_ctrl->sensordata->sensor_name;
	
		//1.open stream
		rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
			sensor_i2c_client, 0x0100,
			&reg_val, MSM_CAMERA_I2C_BYTE_DATA);
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
			sensor_i2c_client, 0x0100,
			reg_val|0x0001, MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);
	
		//2.get access to otp, and set read mode 1xxxx101
		
	
		//2.1 set clk
		rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
			sensor_i2c_client, 0x3545,
			&reg_val, MSM_CAMERA_I2C_BYTE_DATA);
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
			sensor_i2c_client, 0x3545,
			(reg_val|0x0c)&0xec, MSM_CAMERA_I2C_BYTE_DATA);
	
		//3.read otp ,it will spend one page(64 Byte) to store 24 * 8 bite
		for ( j =2 ; j >=0; j--){
			
			memset(read_value,0,sizeof(read_value));
				//3.1 set page num
			rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
					sensor_i2c_client, 0x3502,
					0x0000 + j, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)pr_err("select page failed\n");
			else pr_err("select page success\n");
			msleep(5);
			//3.2 get access to otp, and set read mode 1xxxx101
			rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
			sensor_i2c_client, 0x3500,
			&reg_val, MSM_CAMERA_I2C_BYTE_DATA);
			rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
			sensor_i2c_client, 0x3500,
			(reg_val|0x85)&0xfd, MSM_CAMERA_I2C_BYTE_DATA);
			msleep(5);
			for ( i = 0;i < OTP_DATA_LEN_WORD ; i++){
				reg_val = 0;
	
				//3.3 read page otp data
				rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
					sensor_i2c_client, 0x3504 + i * 2,
					&reg_val, MSM_CAMERA_I2C_WORD_DATA);
				read_value[i*2] = reg_val>>8;
				read_value[i*2 +1] = reg_val&0x00ff;
				
				if(rc<0)
				pr_err( "failed to read OTP data j:%d,i:%d\n",j,i);
				if(reg_val != 0)
				{
					if(mFind==0)
					{	
						mFind = 1;
						mFlag=j;
					}	
				}
			}
		
			if(j==0)
			memcpy(rear_otp_data_bank1, read_value, OTP_DATA_LEN_BYTE);
			else if(j==1)
			memcpy(rear_otp_data_bank2, read_value, OTP_DATA_LEN_BYTE);
			else 
			memcpy(rear_otp_data_bank3, read_value, OTP_DATA_LEN_BYTE);
		
		}
		if(mFlag==2)
			memcpy(rear_otp_data,rear_otp_data_bank3, OTP_DATA_LEN_BYTE);
		else if(mFlag==1)
			memcpy(rear_otp_data,rear_otp_data_bank2, OTP_DATA_LEN_BYTE);
		else
			memcpy(rear_otp_data,rear_otp_data_bank1, OTP_DATA_LEN_BYTE);
		pr_err("data ok and copy to rear_otp_data\n");
		//4.close otp access
		rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
				sensor_i2c_client, 0x3500,
				&reg_val, MSM_CAMERA_I2C_BYTE_DATA);
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
				sensor_i2c_client, 0x3500,
				reg_val&0x007f, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)pr_err("%s: %s: close otp access failed\n", __func__, sensor_name);
	
		//5.close stream
		rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
			sensor_i2c_client, 0x0100,
			&reg_val, MSM_CAMERA_I2C_BYTE_DATA);
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
			sensor_i2c_client, 0x0100,
			reg_val|0x0000, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)pr_err("%s: %s: close stream failed\n", __func__, sensor_name);
	
	 return rc;	
}

// Sheldon_Li add for imx298 ++
int imx298_otp_read(int camNum)
{
	   const char *sensor_name;
	   int rc = 0,i,j,flag,b1,b2,b3;
	   u16 reg_val = 0;
	   u8 read_value[OTP_DATA_LEN_BYTE];

	   struct msm_sensor_ctrl_t *s_ctrl = g_sensor_ctrl[camNum];
	   struct msm_camera_i2c_client *sensor_i2c_client;

	   if(!s_ctrl){
  		pr_err("%s:%d failed: %p\n",__func__, __LINE__, s_ctrl);
		return rc;
	   }

	   sensor_i2c_client = s_ctrl->sensor_i2c_client;
	   sensor_name = s_ctrl->sensordata->sensor_name;

	   rc = fcamera_power_up(s_ctrl); //power on first
	   if(rc < 0){
  		pr_err("%s:%d camera power up failed!\n",__func__, __LINE__);
		return -EINVAL;
	   }
	  //0.set clk
	   if(camNum == 0)
	   {
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
		 sensor_i2c_client, 0x0136, 0x1300, MSM_CAMERA_I2C_WORD_DATA);
		if(rc<0)  pr_err("%s :set clk_write INCK failed!\n",__func__);
		
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
		sensor_i2c_client, 0x4903,0x00c0, MSM_CAMERA_I2C_WORD_DATA);
		if(rc<0)  pr_err("%s :set clk_write  wrcnt failed!\n",__func__);
	   }
	   else
	   {
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
		 sensor_i2c_client, 0x0136, 0x1300, MSM_CAMERA_I2C_WORD_DATA);
		if(rc<0)  pr_err("%s :set clk_write INCK failed!\n",__func__);
		
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
		sensor_i2c_client, 0x4903,0x00c0, MSM_CAMERA_I2C_WORD_DATA);
		if(rc<0)  pr_err("%s : set clk_write  wrcnt failed!\n",__func__);
	   }
	  
	   msleep(1);
	   b1=0;b2=0;b3=0;
	   for(j = 0;  j < 18;  j++)
	   {
		        flag = 0; //clear flag
				
			//1.Set the NVM page number,N(=0-17)
			rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
				sensor_i2c_client, 0x0a02,
				(j), MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) pr_err("%s : Set the NVM page number failed!\n",__func__);
	
			msleep(5);
			
		   //2.Turn ON OTP Read MODE
			rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
				sensor_i2c_client, 0x0a00,
				&reg_val, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc<0)  pr_err("%s : Read for NVM read failed!\n",__func__);
			rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
				sensor_i2c_client, 0x0a00,
				(reg_val|0x01), MSM_CAMERA_I2C_BYTE_DATA);
			if(rc<0) pr_err("%s :  Set up for NVM read failed!\n",__func__);
			
			msleep(10);
		
			//3.Read NVM status registers.(not mandatory!) 
				//0d: = NVM access is in progress.
				//1d: = NVM read has completed.
				//5d: = NVM read has failed.
			 rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
				sensor_i2c_client, 0x0a01,
				&reg_val, MSM_CAMERA_I2C_WORD_DATA);
			  if(rc<0)	pr_err("%s :  Read for NVM read failed!\n",__func__);
			 
			  msleep(5); //wait a moment
			// sheldon_debug("[sheldon]Time : %d  Read NVM status registers : [0x%04x]\n", j, reg_val);
			/*  
			switch(reg_val & 0xff)	
			{
				case 0x0d:
				 sheldon_debug("[sheldon] NVM access is in progress!\n");
				break;
	
				case 0x1d:
				 sheldon_debug("[sheldon] NVM read has completed!\n");
				break;
				
				case 0x5d:
				 sheldon_debug("[sheldon] NVM read has failed!\n");
				break;
	
				default:
				sheldon_debug("[sheldon] Read for NVM Unknow data!\n");
				break;
			}
			*/
			//4.Read Data0-Data63 in the page N, as required.
			memset(read_value,0,sizeof(read_value));
			
				for ( i = 0; i < OTP_DATA_LEN_WORD ;  i++){
					reg_val = 0;
					// read page otp data
					rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
						sensor_i2c_client, 0x0a04 + i * 2,
						&reg_val, MSM_CAMERA_I2C_WORD_DATA);
					read_value[i*2] = reg_val>>8;
					read_value[i*2 +1] = reg_val&0x00ff;
					
					if(rc<0)	pr_err( "[%s]failed to read OTP data j:%d,i:%d\n",sensor_name,j,i);
					
					if(reg_val != 0)
					{
						 flag = 1;
					}
			  }
			
			if((flag == 1) && (b1 == 0))
			{
				if(camNum == 0)
				{
					memcpy(rear_otp_data_bank1, read_value, OTP_DATA_LEN_BYTE); 
				}
				else
				{
					memcpy(front_otp_data_bank1, read_value, OTP_DATA_LEN_BYTE);			
				}
				b1 = 1;
			}
			else if((flag == 1) && (b2 == 0))
			{
				if(camNum == 0)
				{
					memcpy(rear_otp_data_bank2, read_value, OTP_DATA_LEN_BYTE); 	
				}
				else
				{
					memcpy(front_otp_data_bank2, read_value, OTP_DATA_LEN_BYTE);	
				}
				b2 = 1;
			}
			else if((flag == 1) && (b3 == 0))
			{
				if(camNum == 0)
				{
					memcpy(rear_otp_data_bank3, read_value, OTP_DATA_LEN_BYTE); 
				}
				else
				{
					memcpy(front_otp_data_bank3, read_value, OTP_DATA_LEN_BYTE); 		
				}
				b3 = 1;
			}
	   }
	
	   if(camNum == 0)
	   {
		  if(b3 == 1)
			memcpy(rear_otp_data,rear_otp_data_bank3, OTP_DATA_LEN_BYTE); // rear_otp_data	
		  else if(b2 == 1)
			memcpy(rear_otp_data,rear_otp_data_bank2, OTP_DATA_LEN_BYTE); // rear_otp_data	
		  else if(b1 == 1)
			memcpy(rear_otp_data,rear_otp_data_bank1, OTP_DATA_LEN_BYTE); // rear_otp_data	
	   }
	   else
	   {
		  if(b3 == 1)
			memcpy(front_otp_data,front_otp_data_bank3, OTP_DATA_LEN_BYTE); // front_otp_data  
		  else if(b2 == 1)
			memcpy(front_otp_data,front_otp_data_bank2, OTP_DATA_LEN_BYTE); // front_otp_data  
		  else if(b1 == 1)
			memcpy(front_otp_data,front_otp_data_bank1, OTP_DATA_LEN_BYTE); // front_otp_data	
			
	   }
	   
	   rc = fcamera_power_down(s_ctrl); //power down end
	   if(rc < 0){
  		pr_err("%s:%d camera power down failed!\n",__func__, __LINE__);
		return rc;
	   }
	   return rc;
}
// Sheldon_Li add for imx298 --


// Sheldon_Li add for ov8856 ++
int ov8856_otp_read(void)
{
	   const char *sensor_name;
	   int rc = 0,i,j,flag,b1,b2,b3;
	   u16 reg_val = 0;
	   u16 start_addr, end_addr;
	   u8 read_value[OTP_DATA_LEN_BYTE];

	   struct msm_sensor_ctrl_t *s_ctrl = g_sensor_ctrl[CAMERA_1];
	   struct msm_camera_i2c_client *sensor_i2c_client;

	   if(!s_ctrl){
  		pr_err("%s:%d failed: %p\n",__func__, __LINE__, s_ctrl);
		return rc;
	   }

	   sensor_i2c_client = s_ctrl->sensor_i2c_client;
	   sensor_name = s_ctrl->sensordata->sensor_name;

	   rc = fcamera_power_up(s_ctrl); //power on first
	   if(rc < 0){
  		pr_err("%s:%d camera power up failed!\n",__func__, __LINE__);
		return -EINVAL;
	   }

	      
         //0. Reset sensor as default
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
	        	 	sensor_i2c_client, 0x0103, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc<0)  pr_err("%s : Reset sensor as default failed!\n",__func__);

	//1.open stream
		rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
			sensor_i2c_client, 0x0100,
			&reg_val, MSM_CAMERA_I2C_BYTE_DATA);
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
			sensor_i2c_client, 0x0100,
			reg_val|0x0001, MSM_CAMERA_I2C_BYTE_DATA);
	   
	  //2.otp_option_en = 1
		rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
		 sensor_i2c_client, 0x5001, &reg_val, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc<0)  pr_err("%s : read otp option failed!\n",__func__);
		
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
		sensor_i2c_client, 0x5001,((0x00 & 0x08) | (reg_val & (~0x08))), MSM_CAMERA_I2C_BYTE_DATA);
		if(rc<0)  pr_err("%s : write otp option failed!\n",__func__);
	  
	   msleep(1);
	   b1=0;b2=0;b3=0;
	   for(j = 3;  j > 0;  j--)
	   {
		    flag = 0; //clear flag
				
		   //3.Set the read bank number
			if (j == 1) {
				start_addr = 0x7010;
				end_addr   = 0x702f;
			} else if (j == 2) {
				start_addr = 0x7030;
				end_addr   = 0x704f;
			} else if (j == 3) {
				start_addr = 0x7050;
				end_addr   = 0x706f;
			}
			
		   //4.Set OTP MODE Bit[6] : 0=Auto mode 1=Manual mode
			rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
				sensor_i2c_client, 0x3d84,
			   	0xc0, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc<0) pr_err("%s : Write OTP_MODE_CTRL registor failed!\n",__func__);


		  //5.Set OTP_REG85 : Bit[2]=load data enable  Bit[1]= load setting enable
			rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
				sensor_i2c_client, 0x3d85,
				0x06, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc<0) pr_err("%s : Write OTP_REG85  failed!\n",__func__);

		  //6.Set OTP read address
		         //start address High
			rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
				sensor_i2c_client, 0x3d88,
				(start_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc<0) pr_err("%s : Write start address High  failed!\n",__func__);

			//start address Low
			rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
				sensor_i2c_client, 0x3d89,
				(start_addr & 0xff) , MSM_CAMERA_I2C_BYTE_DATA);
			if(rc<0) pr_err("%s : Write start address Low  failed!\n",__func__);

			 //end address High
			rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
				sensor_i2c_client, 0x3d8a,
				(end_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc<0) pr_err("%s : Write end address High  failed!\n",__func__);

			//end address Low
			rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
				sensor_i2c_client, 0x3d8b,
				(end_addr  & 0xff), MSM_CAMERA_I2C_BYTE_DATA);
			if(rc<0) pr_err("%s : Write end address Low  failed!\n",__func__);


		//7.Load OTP data enable
			rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
				sensor_i2c_client, 0x3d81,
				&reg_val, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc<0) pr_err("%s : Read data enable REG  failed!\n",__func__);

			rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
				sensor_i2c_client, 0x3d81,
				(reg_val | 0x01), MSM_CAMERA_I2C_BYTE_DATA);
			if(rc<0) pr_err("%s : Write data enable REG  failed!\n",__func__);
			
			msleep(20);
		
			memset(read_value,0,sizeof(read_value));
			
				for ( i = 0; i < OTP_DATA_LEN_WORD ;  i++){
					reg_val = 0;
					// read page otp data
					rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
						sensor_i2c_client, start_addr + i * 2,
						&reg_val, MSM_CAMERA_I2C_WORD_DATA);
					read_value[i*2] = reg_val>>8;
					read_value[i*2 +1] = reg_val&0x00ff;
					
					if(rc<0)	pr_err( "[%s]failed to read OTP data j:%d,i:%d\n",sensor_name,j,i);
					
					if(reg_val != 0)
					{
						 flag = 1;
					}
			  }
			
			if((flag == 1) && (b1 == 0))
			{
				memcpy(front_otp_data_bank1, read_value, OTP_DATA_LEN_BYTE);			
				b1 = 1;
			}
			else if((flag == 1) && (b2 == 0))
			{
				memcpy(front_otp_data_bank2, read_value, OTP_DATA_LEN_BYTE);	
				b2 = 1;
			}
			else if((flag == 1) && (b3 == 0))
			{
				memcpy(front_otp_data_bank3, read_value, OTP_DATA_LEN_BYTE); 		
				b3 = 1;
			}
	   }
	
		  if(b3 == 1)
			memcpy(front_otp_data,front_otp_data_bank3, OTP_DATA_LEN_BYTE); // front_otp_data  
		  else if(b2 == 1)
			memcpy(front_otp_data,front_otp_data_bank2, OTP_DATA_LEN_BYTE); // front_otp_data  
		  else if(b1 == 1)
			memcpy(front_otp_data,front_otp_data_bank1, OTP_DATA_LEN_BYTE); // front_otp_data
	
	 //8.close stream
		rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
			sensor_i2c_client, 0x0100,
			&reg_val, MSM_CAMERA_I2C_BYTE_DATA);
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
			sensor_i2c_client, 0x0100,
			reg_val|0x0000, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)pr_err("%s: %s: close stream failed\n", __func__, sensor_name);
		
	   
	   rc = fcamera_power_down(s_ctrl); //power down end
	   if(rc < 0){
  		pr_err("%s:%d camera power down failed!\n",__func__, __LINE__);
		return rc;
	   }
	   return rc;
}
// Sheldon_Li add for ov8856 --


struct proc_dir_entry *status_proc_file;

  int rear_proc_read(struct seq_file *buf, void *v)
{	
	int i;
	for( i = 0; i < OTP_DATA_LEN_WORD * 2; i++)
	{
		seq_printf(buf, "0x%02X ",rear_otp_data[i]);
		if(i==7||i==15||i==23||i==31)
		seq_printf(buf ,"\n");
	}
	return 0;
}

  int front_proc_read(struct seq_file *buf, void *v)
{
	int i;
	for( i = 0; i < OTP_DATA_LEN_WORD * 2; i++)
	{
		seq_printf(buf, "0x%02X ",front_otp_data[i]);
		if(i==7||i==15||i==23||i==31)
		seq_printf(buf ,"\n");
	}
	return 0;
}

  int rear_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rear_proc_read, NULL);
}

  int front_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, front_proc_read, NULL);
}

  const struct file_operations rear_status_fops = {
	.owner = THIS_MODULE,
	.open = rear_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

  const struct file_operations front_status_fops = {
	.owner = THIS_MODULE,
	.open = front_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

  void create_rear_otp_proc_file(void)
{
    status_proc_file = proc_create(PROC_FILE_STATUS_REAR, 0666, NULL, &rear_status_fops);
	if(status_proc_file) {
		CDBG("%s sucessed!\n", __func__);
    } else {
		pr_err("%s failed!\n", __func__);
    }   
}

  void create_front_otp_proc_file(void)
{
	status_proc_file = proc_create(PROC_FILE_STATUS_FRONT, 0666, NULL, &front_status_fops);
	
	if(status_proc_file) {
		CDBG("%s sucessed!\n", __func__);
    } else {
		pr_err("%s failed!\n", __func__);
    }
}
  
// Sheldon_Li add for imx298 thermal meter++
int imx298_thermal_meter_read(int camNum)
{
	   const char *sensor_name;
	   int rc = 0;
	   int reg_data = -1;
	   u16 reg_val = -1;  

	   struct msm_sensor_ctrl_t *s_ctrl = g_sensor_ctrl[camNum];
	   struct msm_camera_i2c_client *sensor_i2c_client;

	   if(!s_ctrl){
	         pr_err("%s:%d failed: %p\n",__func__, __LINE__, s_ctrl);
		return rc;
	   }

	   if(s_ctrl->sensor_state == MSM_SENSOR_POWER_DOWN){
		 pr_err("[imx298_thermal]Please open camera %d first !\n",camNum);
		 return -1;
	   }
	  
	   sensor_i2c_client = s_ctrl->sensor_i2c_client;
	   sensor_name = s_ctrl->sensordata->sensor_name;
	   
	  //0.set clk
                  //nominal
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
		 sensor_i2c_client, 0x0136, 0x13, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc<0)  pr_err("[imx298_thermal] set clk_write INCK failed!\n");
		//point
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
		 sensor_i2c_client, 0x0137, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc<0)  pr_err("[imx298_thermal] set clk_write INCK failed!\n");

		//msleep(15);
		
	 //1.open stream
		rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
			sensor_i2c_client, 0x0100,&reg_val, MSM_CAMERA_I2C_BYTE_DATA);
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
			sensor_i2c_client, 0x0100,reg_val|0x01, MSM_CAMERA_I2C_BYTE_DATA);

		//msleep(15);
		
	  //2.enable control
	         rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
	          sensor_i2c_client, 0x0138, &reg_val, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc<0)  pr_err("[imx298_thermal]read enable failed!\n");
		
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
		 sensor_i2c_client, 0x0138, (reg_val | 0x01), MSM_CAMERA_I2C_BYTE_DATA);
		if(rc<0)  pr_err("[imx298_thermal] set enable failed!\n");

		msleep(50);

	   //3.read temperature data 
	         rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
		sensor_i2c_client, 0x013a, &reg_val, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc<0)  pr_err("[imx298_thermal] read temperature failed!\n");

		reg_data =  (int)(reg_val&0xff);
		sheldon_debug("[imx298_thermal] temperature = 0x%02x\n",reg_data);

		//msleep(15);

	   //4.disable control
	         rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
	          sensor_i2c_client, 0x0138, &reg_val, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc<0)  pr_err("[imx298_thermal]read enable failed!\n");
		
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
		 sensor_i2c_client, 0x0138, (reg_val | 0x00), MSM_CAMERA_I2C_BYTE_DATA);
		if(rc<0)  pr_err("[imx298_thermal] set enable failed!\n");

		//msleep(15);

	    //5.close stream
		rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
			sensor_i2c_client, 0x0100,
			&reg_val, MSM_CAMERA_I2C_BYTE_DATA);
		rc = sensor_i2c_client->i2c_func_tbl->i2c_write(
			sensor_i2c_client, 0x0100,
			reg_val|0x0000, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)pr_err("%s: %s: close stream failed\n", __func__, sensor_name);
	   return reg_data;
}


int rear_thermal_read(struct seq_file *buf, void *v)
  {   
	 uint16_t temp = 0;
	sensor_read_temp(&temp);
	seq_printf(buf, "%d\n", temp);
	pr_err("Stimber: Temperature = %d\n", temp);

    return 0;
  }
  
int front_thermal_read(struct seq_file *buf, void *v)
  {
	  int temp=0;
	  int val=0;
	  
	  temp = imx298_thermal_meter_read(1);
	  
	  if(temp < 0 || temp > 0xec || temp == 0x80){
		if(temp == -1){
		     temp=0;	
		     seq_printf(buf ,"%d\n",temp);
		}else{
		     temp=-1;	
		     seq_printf(buf ,"%d\n",temp);
		}
	         return 0;
	  }
	  
	  if(temp < 0x50){
		val = temp;
	  } else if(temp >= 0x50 && temp < 0x80){
		val = 80;
	  }else  if(temp >= 0x81 && temp <= 0xec){
	  	val = -20;
	  }else{
	        temp -= 0xed;
		val = temp -19;
	  }
		
	  seq_printf(buf ,"%d\n",temp);
	  return 0;
  }
  
int rear_thermal_file_open(struct inode *inode, struct  file *file)
  {
	  return single_open(file, rear_thermal_read, NULL);
  }
  
int front_thermal_file_open(struct inode *inode, struct  file *file)
  {
	  return single_open(file, front_thermal_read, NULL);
  }
  
const struct file_operations rear_thermal_fops = {
	  .owner = THIS_MODULE,
	  .open = rear_thermal_file_open,
	  .read = seq_read,
	  .llseek = seq_lseek,
	  .release = single_release,
  };
  
const struct file_operations front_thermal_fops = {
	  .owner = THIS_MODULE,
	  .open = front_thermal_file_open,
	  .read = seq_read,
	  .llseek = seq_lseek,
	  .release = single_release,
  };
  
void create_thermal_file(int camNum)
  {
         if(camNum == 0){
	  status_proc_file = proc_create(PROC_FILE_THERMAL_REAR, 0666, NULL, &rear_thermal_fops);
	  if(status_proc_file) {
		  printk("%s 0 ,sucessed!\n", __func__);
	   } else {
		  pr_err("%s failed!\n", __func__);
	   }   
	}else if(camNum == 1){
	  status_proc_file = proc_create(PROC_FILE_THERMAL_FRONT, 0666, NULL, &front_thermal_fops);
	  if(status_proc_file) {
		  printk("%s 1 ,sucessed!\n", __func__);
	   } else {
		  pr_err("%s failed!\n", __func__);
	   }
	}
  }
// Sheldon_Li add for imx298 thermal meter--

  // Sheldon_Li add for imx298 eeprom map to otp++
  extern struct msm_eeprom_memory_block_t g_rear_eeprom_mapdata;
  extern struct msm_eeprom_memory_block_t g_front_eeprom_mapdata;

  int rear_eeprom_dump(struct seq_file *buf, void *v)
{	
	int i;
	for( i = 0; i < OTP_DATA_LEN_WORD * 2; i++)
	{
	        if(g_rear_eeprom_mapdata.mapdata != NULL){
			seq_printf(buf, "0x%02X ",g_rear_eeprom_mapdata.mapdata[i]);
			if((i&7) == 7)
			seq_printf(buf ,"\n");
		}else{
			seq_printf(buf ,"No rear EEprom!\n");
			break;
		}
		
	}
	return 0;
}

  int front_eeprom_dump(struct seq_file *buf, void *v)
{
	int i;
	for( i = 0; i < OTP_DATA_LEN_WORD * 2; i++)
	{
		if(g_front_eeprom_mapdata.mapdata != NULL){
			seq_printf(buf, "0x%02X ",g_front_eeprom_mapdata.mapdata[i]);
			if((i&7) == 7)
			seq_printf(buf ,"\n");
		}else{
			seq_printf(buf ,"No front EEprom!\n");
			break;
		}
	}
	return 0;
}

  int rear_eeprom_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rear_eeprom_dump, NULL);
}

  int front_eeprom_open(struct inode *inode, struct  file *file)
{
    return single_open(file, front_eeprom_dump, NULL);
}

  const struct file_operations rear_eeprom_fops = {
	.owner = THIS_MODULE,
	.open = rear_eeprom_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

  const struct file_operations front_eeprom_fops = {
	.owner = THIS_MODULE,
	.open = front_eeprom_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

  void create_dump_eeprom_file(int camNum)
{
	   if(camNum == 0){
		 status_proc_file = proc_create(PROC_FILE_STATUS_REAR, 0666, NULL, &rear_eeprom_fops);
		if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
	   	 } else {
			pr_err("%s failed!\n", __func__);
	    	}   

	   }else if(camNum == 1){
		status_proc_file = proc_create(PROC_FILE_STATUS_FRONT, 0666, NULL, &front_eeprom_fops);
		if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
	    	} else {
			pr_err("%s failed!\n", __func__);
	    	}
	 }
}
  // Sheldon_Li add for imx298 eeprom map to otp--

  int pdaf_info_proc_read(struct seq_file *buf, void *v)
  {
	  seq_printf(buf, "%s",pdaf_info);
	  seq_printf(buf ,"\n");
	  return 0;
  }

  ssize_t pdaf_info_proc_file_read (struct file *filp, char __user *buff, size_t size, loff_t *ppos){
	int ret;
	int count;
	count = copy_to_user(buff, pdaf_info, size);
	if(copy_to_user(buff, pdaf_info, size)){
		ret =  - EFAULT;
	}else{
		*ppos += count;
		ret = count;
	}
	return ret;
	}
  ssize_t pdaf_info_proc_file_write (struct file *filp, const char __user *buff, size_t size, loff_t *ppos){
	  int ret;
	  memset(pdaf_info, 0, PDAF_INFO_MAX_SIZE);
	  if(copy_from_user(pdaf_info, buff, size)){
		  ret =  - EFAULT;
		  pr_err("%s copy from us failed!\n", __func__);
		  }else{
		  *ppos += size;
		  ret = size;
		  }
	  return ret;
	}

  int pdaf_info_proc_open(struct inode *inode, struct  file *file)
  {
	  return single_open(file, pdaf_info_proc_read, NULL);
  }

  //rear and front use same ops
  const struct file_operations pdaf_info_fops = {
	  .owner = THIS_MODULE,
	  .open = pdaf_info_proc_open,
	  .read = seq_read,
	  //.read = pdaf_info_proc_file_read,
	  .write = pdaf_info_proc_file_write,
	  .llseek = seq_lseek,
	  .release = single_release,
  };

  void create_proc_pdaf_info(void)
  {
	if(!pdaf_info){
	  pdaf_info = kmalloc(PDAF_INFO_MAX_SIZE, GFP_KERNEL);
	  memset(pdaf_info,0,PDAF_INFO_MAX_SIZE);
	  pdaf_file = proc_create(REAR_PROC_FILE_PDAF_INFO, 0666, NULL, &pdaf_info_fops);
	  if(pdaf_file) {
		  CDBG("%s sucessed!\n", __func__);
	  } else {
		  pr_err("%s failed!\n", __func__);
	  }

	  pdaf_file = proc_create(FRONT_PROC_FILE_PDAF_INFO, 0666, NULL, &pdaf_info_fops);
	  if(pdaf_file) {
		  CDBG("%s sucessed!\n", __func__);
	  } else {
		  pr_err("%s failed!\n", __func__);
	  }
	}
  }

/*
	clear_proc_pdaf_info()
	Clear the pdaf data of node when close camera.
*/
void clear_proc_pdaf_info(void){
	memset(pdaf_info,0,PDAF_INFO_MAX_SIZE);
}
  void remove_file(void)
{
    extern struct proc_dir_entry proc_root;
    pr_info("remove_file\n");	
    remove_proc_entry(PROC_FILE_STATUS_REAR, &proc_root);
    remove_proc_entry(PROC_FILE_STATUS_FRONT, &proc_root);
    remove_proc_entry(REAR_PROC_FILE_PDAF_INFO,&proc_root);
    remove_proc_entry(FRONT_PROC_FILE_PDAF_INFO,&proc_root);
}

  int rear_bank1_proc_read(struct seq_file *buf, void *v)
{	
	int i;
	for( i = 0; i < OTP_DATA_LEN_WORD * 2; i++)
	{
		seq_printf(buf, "0x%02X ",rear_otp_data_bank1[i]);
		if(i==7||i==15||i==23||i==31)
		seq_printf(buf ,"\n");
	}
	return 0;
}

  int front_bank1_proc_read(struct seq_file *buf, void *v)
{
	int i;
	for( i = 0; i < OTP_DATA_LEN_WORD * 2; i++)
	{
		seq_printf(buf, "0x%02X ",front_otp_data_bank1[i]);
		if(i==7||i==15||i==23||i==31)
		seq_printf(buf ,"\n");
	}
	return 0;
}

  int rear_bank1_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rear_bank1_proc_read, NULL);
}

  int front_bank1_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, front_bank1_proc_read, NULL);
}

  const struct file_operations rear_status_fops_bank1 = {
	.owner = THIS_MODULE,
	.open = rear_bank1_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

  const struct file_operations front_status_fops_bank1 = {
	.owner = THIS_MODULE,
	.open = front_bank1_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

  void create_rear_bank1_proc_file(void)
{
     
        status_proc_file = proc_create(PROC_FILE_STATUS_BANK1_REAR, 0666, NULL, &rear_status_fops_bank1);
		if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
	    } else {
			pr_err("%s failed!\n", __func__);
	    }   
}

  void create_front_bank1_proc_file(void)
{
		status_proc_file = proc_create(PROC_FILE_STATUS_BANK1_FRONT, 0666, NULL, &front_status_fops_bank1);
	
	if(status_proc_file) {
		CDBG("%s sucessed!\n", __func__);
    } else {
		pr_err("%s failed!\n", __func__);
    }
}

  void remove_bank1_file(void)
{
    extern struct proc_dir_entry proc_root;
    pr_info("remove_bank1_file\n");	
    remove_proc_entry(PROC_FILE_STATUS_BANK1_REAR, &proc_root);
    remove_proc_entry(PROC_FILE_STATUS_BANK1_FRONT, &proc_root);
}



  int rear_bank2_proc_read(struct seq_file *buf, void *v)
{	
	int i;
	for( i = 0; i < OTP_DATA_LEN_WORD * 2; i++)
	{
		seq_printf(buf, "0x%02X ",rear_otp_data_bank2[i]);
		if(i==7||i==15||i==23||i==31)
		seq_printf(buf ,"\n");
	}
	return 0;
}

  int front_bank2_proc_read(struct seq_file *buf, void *v)
{
	int i;
	for( i = 0; i < OTP_DATA_LEN_WORD * 2; i++)
	{
		seq_printf(buf, "0x%02X ",front_otp_data_bank2[i]);
		if(i==7||i==15||i==23||i==31)
		seq_printf(buf ,"\n");
	}
	return 0;
}

  int rear_bank2_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rear_bank2_proc_read, NULL);
}

  int front_bank2_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, front_bank2_proc_read, NULL);
}

  const struct file_operations rear_status_fops_bank2 = {
	.owner = THIS_MODULE,
	.open = rear_bank2_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

  const struct file_operations front_status_fops_bank2 = {
	.owner = THIS_MODULE,
	.open = front_bank2_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

  void create_rear_bank2_proc_file(void)
{
     
        status_proc_file = proc_create(PROC_FILE_STATUS_BANK2_REAR, 0666, NULL, &rear_status_fops_bank2);
		if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
	    } else {
			pr_err("%s failed!\n", __func__);
	    }   
}

  void create_front_bank2_proc_file(void)
{
		status_proc_file = proc_create(PROC_FILE_STATUS_BANK2_FRONT, 0666, NULL, &front_status_fops_bank2);
	
	if(status_proc_file) {
		CDBG("%s sucessed!\n", __func__);
    } else {
		pr_err("%s failed!\n", __func__);
    }
}

  void remove_bank2_file(void)
{
    extern struct proc_dir_entry proc_root;
    pr_info("remove_bank2_file\n");	
    remove_proc_entry(PROC_FILE_STATUS_BANK2_REAR, &proc_root);
    remove_proc_entry(PROC_FILE_STATUS_BANK2_FRONT, &proc_root);
}


  int rear_bank3_proc_read(struct seq_file *buf, void *v)
{	
	int i;
	for( i = 0; i < OTP_DATA_LEN_WORD * 2; i++)
	{
		seq_printf(buf, "0x%02X ",rear_otp_data_bank3[i]);
		if(i==7||i==15||i==23||i==31)
		seq_printf(buf ,"\n");
	}
	return 0;
}

  int front_bank3_proc_read(struct seq_file *buf, void *v)
{
	int i;
	for( i = 0; i < OTP_DATA_LEN_WORD * 2; i++)
	{
		seq_printf(buf, "0x%02X ",front_otp_data_bank3[i]);
		if(i==7||i==15||i==23||i==31)
		seq_printf(buf ,"\n");
	}
	return 0;
}

  int rear_bank3_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rear_bank3_proc_read, NULL);
}

  int front_bank3_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, front_bank3_proc_read, NULL);
}

  const struct file_operations rear_status_fops_bank3 = {
	.owner = THIS_MODULE,
	.open = rear_bank3_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

  const struct file_operations front_status_fops_bank3 = {
	.owner = THIS_MODULE,
	.open = front_bank3_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

  void create_rear_bank3_proc_file(void)
{
     
        status_proc_file = proc_create(PROC_FILE_STATUS_BANK3_REAR, 0666, NULL, &rear_status_fops_bank3);
		if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
	    } else {
			pr_err("%s failed!\n", __func__);
	    }   
}

  void create_front_bank3_proc_file(void)
{
		status_proc_file = proc_create(PROC_FILE_STATUS_BANK3_FRONT, 0666, NULL, &front_status_fops_bank3);
	
	if(status_proc_file) {
		CDBG("%s sucessed!\n", __func__);
    } else {
		pr_err("%s failed!\n", __func__);
    }
}
  
  void remove_bank3_file(void)
{
    extern struct proc_dir_entry proc_root;
    pr_info("remove_bank3_file\n");	
    remove_proc_entry(PROC_FILE_STATUS_BANK3_REAR, &proc_root);
    remove_proc_entry(PROC_FILE_STATUS_BANK3_FRONT, &proc_root);
}
  
  int rear_module_proc_read(struct seq_file *buf, void *v)
{
	if(g_sensor_id[CAMERA_0]==SENSOR_ID_IMX298)
	return seq_printf(buf,"IMX298\n");
	if(g_sensor_id[CAMERA_0]==SENSOR_ID_T4K37)
	return seq_printf(buf,"T4K37\n");
	if(g_sensor_id[CAMERA_0]==SENSOR_ID_T4K35)
	return seq_printf(buf,"T4K35\n");
	return 0;
}

  int front_module_proc_read(struct seq_file *buf, void *v)
{
	if(g_sensor_id[CAMERA_1]==SENSOR_ID_IMX298)
	return seq_printf(buf,"IMX298F\n");
	if(g_sensor_id[CAMERA_1]==SENSOR_ID_OV5670)
	return seq_printf(buf,"OV5670\n");
	if(g_sensor_id[CAMERA_1]==SENSOR_ID_T4K37)
	return seq_printf(buf,"T4K37F\n");
	if(g_sensor_id[CAMERA_1]==SENSOR_ID_OV8856)
	return seq_printf(buf,"OV8856\n");
	return 0;
}

  int rear_module_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, rear_module_proc_read, NULL);
}

  int front_module_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, front_module_proc_read, NULL);
}

  const struct file_operations rear_module_fops = {
	.owner = THIS_MODULE,
	.open = rear_module_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

  const struct file_operations front_module_fops = {
	.owner = THIS_MODULE,
	.open = front_module_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

  void create_rear_module_proc_file(void)
{
           status_proc_file = proc_create(PROC_FILE_REARMODULE, 0666, NULL, &rear_module_fops);
	   if(status_proc_file) {
			CDBG("%s sucessed!\n", __func__);
	    } else {
			pr_err("%s failed!\n", __func__);
	    }   
}

  void create_front_module_proc_file(void)
{
	status_proc_file = proc_create(PROC_FILE_FRONTMODULE, 0666, NULL, &front_module_fops);
	
	if(status_proc_file) {
		CDBG("%s sucessed!\n", __func__);
   	 } else {
		pr_err("%s failed!\n", __func__);
        }
}

  void remove_module_file(void)
{
    extern struct proc_dir_entry proc_root;
    pr_info("remove_module_file\n");	
    remove_proc_entry(PROC_FILE_REARMODULE, &proc_root);
    remove_proc_entry(PROC_FILE_FRONTMODULE, &proc_root);
}

//static DEVICE_ATTR(camera_status, S_IRUGO | S_IWUSR | S_IWGRP ,rear_status_proc_read,NULL);
//static DEVICE_ATTR(vga_status, S_IRUGO | S_IWUSR | S_IWGRP ,front_status_proc_read,NULL);
//static DEVICE_ATTR(camera_resolution, S_IRUGO | S_IWUSR | S_IWGRP ,rear_resolution_read,NULL);
//static DEVICE_ATTR(vga_resolution, S_IRUGO | S_IWUSR | S_IWGRP ,front_resolution_read,NULL);

struct kobject *camera_status_camera;
EXPORT_SYMBOL_GPL(camera_status_camera);
struct kobject *camera_status_vga;
EXPORT_SYMBOL_GPL(camera_status_vga);
struct kobject *camera_resolution_camera;
EXPORT_SYMBOL_GPL(camera_resolution_camera);
struct kobject *camera_resolution_vga;
EXPORT_SYMBOL_GPL(camera_resolution_vga);
	
static ssize_t rear_status_proc_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	  struct msm_sensor_ctrl_t *s_ctrl = g_sensor_ctrl[CAMERA_0];

		  if(s_ctrl != NULL)
		  {
			  if((msm_sensor_testi2c(s_ctrl))<0)
			  return sprintf(buf, "%s\n%s\n", "ERROR: i2c r/w test fail","Driver version:");
			  else
			  return sprintf(buf, "%s\n%s\n%s0x%x\n", "ACK:i2c r/w test ok","Driver version:","sensor_id:",g_sensor_id[CAMERA_0]);	  
		  }
		  else{
		  	 return sprintf(buf, "%s\n%s\n", "ERROR: i2c r/w test fail","Driver version:");
		  }  
}

static const struct device_attribute dev_attr_camera_status = 
	__ATTR(camera_status, S_IRUGO | S_IWUSR | S_IWGRP, rear_status_proc_read, NULL);

static ssize_t front_status_proc_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	  struct msm_sensor_ctrl_t *s_ctrl = g_sensor_ctrl[CAMERA_1];
	  
		  if(s_ctrl != NULL)
		  {
			   if((msm_sensor_testi2c(s_ctrl))<0)
			   return sprintf(buf, "%s\n%s\n", "ERROR: i2c r/w test fail","Driver version:");
			   else
			   return sprintf(buf, "%s\n%s\n%s0x%x\n", "ACK:i2c r/w test ok","Driver version:","sensor_id:",g_sensor_id[CAMERA_1]);   
		  }
		   else{
		   	  return sprintf(buf, "%s\n%s\n", "ERROR: i2c r/w test fail","Driver version:");
		   }
}

static const struct device_attribute dev_attr_vga_status =
	__ATTR(vga_status, S_IRUGO | S_IWUSR | S_IWGRP, front_status_proc_read, NULL);

  int create_rear_status_proc_file(void)
{	
	int ret;
		camera_status_camera = kobject_create_and_add("camera_status_camera", NULL);
		if (!camera_status_camera)
			return -ENOMEM;
		ret = sysfs_create_file(camera_status_camera, &dev_attr_camera_status.attr);
		if (ret)
		{
			printk("%s: sysfs_create failed\n", __func__);
			return ret;
		}
		return 0;
}
  int create_front_status_proc_file(void)
{	
	int ret;
		camera_status_vga = kobject_create_and_add("camera_status_vga", NULL);
		if (!camera_status_vga)
			return -ENOMEM;
		ret = sysfs_create_file(camera_status_vga, &dev_attr_vga_status.attr);
		if (ret)
		{
			printk("%s: sysfs_create failed\n", __func__);
			return ret;
		}
		return 0;
}
  void remove_proc_file(void)
{
	sysfs_remove_file(camera_status_camera, &dev_attr_camera_status.attr);
	kobject_del(camera_status_camera);
	sysfs_remove_file(camera_status_vga, &dev_attr_vga_status.attr);
	kobject_del(camera_status_vga);
}

static ssize_t rear_resolution_read(struct device *dev, struct device_attribute *attr, char *buf)
  {
	  if(g_sensor_id[CAMERA_0]==SENSOR_ID_T4K37)
		  return sprintf(buf,"13M\n");
	  else if(g_sensor_id[CAMERA_0]==SENSOR_ID_IMX298)
		  return sprintf(buf,"16M\n");
	  else
		  return sprintf(buf,"None\n");
  }

static const struct device_attribute dev_attr_camera_resolution =
	__ATTR(camera_resolution, S_IRUGO | S_IWUSR | S_IWGRP, rear_resolution_read, NULL);  

static ssize_t front_resolution_read(struct device *dev, struct device_attribute *attr, char *buf)
  {
	  if(g_sensor_id[CAMERA_1]==SENSOR_ID_IMX298)
		  return sprintf(buf,"16M\n");
	  else  if(g_sensor_id[CAMERA_1]==SENSOR_ID_OV8856)
		  return sprintf(buf,"8M\n");
	  else
		  return sprintf(buf,"None\n");
  }

static const struct device_attribute dev_attr_vga_resolution =
	__ATTR(vga_resolution, S_IRUGO | S_IWUSR | S_IWGRP, front_resolution_read, NULL);

  int create_rear_resolution_proc_file(void)
{
		int ret;
			camera_resolution_camera = kobject_create_and_add("camera_resolution_camera", NULL);
			if (!camera_resolution_camera)
				return -ENOMEM;
			ret = sysfs_create_file(camera_resolution_camera, &dev_attr_camera_resolution.attr);
			if (ret)
			{
				printk("%s: sysfs_create failed\n", __func__);
				return ret;
			}
			return 0;
}
  int create_front_resolution_proc_file(void)
{	
		int ret;
			camera_resolution_vga = kobject_create_and_add("camera_resolution_vga", NULL);
			if (!camera_resolution_vga)
				return -ENOMEM;
			ret = sysfs_create_file(camera_resolution_vga, &dev_attr_vga_resolution.attr);
			if (ret)
			{
				printk("%s: sysfs_create failed\n", __func__);
				return ret;
			}
			return 0;
}
  void remove_resolution_file(void)
{
	sysfs_remove_file(camera_resolution_camera, &dev_attr_camera_resolution.attr);
	kobject_del(camera_resolution_camera);
	sysfs_remove_file(camera_resolution_vga, &dev_attr_vga_resolution.attr);
	kobject_del(camera_resolution_vga);
}


