/*
 * drivers/media/radio/si470x/radio-si470x-i2c.c
 *
 * I2C driver for radios with Silicon Labs Si470x FM Radio Receivers
 *
 * Copyright (c) 2009 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


/* driver definitions */
#define DRIVER_AUTHOR "Joonyoung Shim <jy0922.shim@samsung.com>";
#define DRIVER_CARD "Silicon Labs Si470x FM Radio Receiver"
#define DRIVER_DESC "I2C radio driver for Si470x FM Radio Receivers"
#define DRIVER_VERSION "1.0.2"

/* kernel includes */
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#include "radio-si470x.h"


/* I2C Device ID List */
static const struct i2c_device_id si470x_i2c_id[] = {
	/* Generic Entry */
	{ "si470x", 0 },
	/* Terminating entry */
	{ }
};
MODULE_DEVICE_TABLE(i2c, si470x_i2c_id);



/**************************************************************************
 * Module Parameters
 **************************************************************************/

/* Radio Nr */
static int radio_nr = -1;
module_param(radio_nr, int, 0444);
MODULE_PARM_DESC(radio_nr, "Radio Nr");

/* RDS buffer blocks */
static unsigned int rds_buf = 100;
module_param(rds_buf, uint, 0444);
MODULE_PARM_DESC(rds_buf, "RDS buffer entries: *100*");

/* RDS maximum block errors */
static unsigned short max_rds_errors = 1;
/* 0 means   0  errors requiring correction */
/* 1 means 1-2  errors requiring correction (used by original USBRadio.exe) */
/* 2 means 3-5  errors requiring correction */
/* 3 means   6+ errors or errors in checkword, correction not possible */
module_param(max_rds_errors, ushort, 0644);
MODULE_PARM_DESC(max_rds_errors, "RDS maximum block errors: *1*");



/**************************************************************************
 * I2C Definitions
 **************************************************************************/

/* Write starts with the upper byte of register 0x02 */
#define WRITE_REG_NUM		8
#define WRITE_INDEX(i)		(i + 0x02)

/* Read starts with the upper byte of register 0x0a */
#define READ_REG_NUM		RADIO_REGISTER_NUM
#define READ_INDEX(i)		((i + RADIO_REGISTER_NUM - 0x0a) % READ_REG_NUM)



/**************************************************************************
 * General Driver Functions - REGISTERs
 **************************************************************************/

static int silabs_fm_power_cfg(struct si470x_device *radio, bool on);

/*
 * si470x_get_register - read register
 */
int si470x_get_register(struct si470x_device *radio, int regnr)
{
	u16 buf[READ_REG_NUM];
	struct i2c_msg msgs[1] = {
		{
			.addr = radio->client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(u16) * READ_REG_NUM,
			.buf = (void *)buf
		},
	};

	if (i2c_transfer(radio->client->adapter, msgs, 1) != 1)
		return -EIO;

	radio->registers[regnr] = __be16_to_cpu(buf[READ_INDEX(regnr)]);

	return 0;
}


/*
 * si470x_set_register - write register
 */
int si470x_set_register(struct si470x_device *radio, int regnr)
{
	int i;
	u16 buf[WRITE_REG_NUM];
	struct i2c_msg msgs[1] = {
		{
			.addr = radio->client->addr,
			.len = sizeof(u16) * WRITE_REG_NUM,
			.buf = (void *)buf
		},
	};

	for (i = 0; i < WRITE_REG_NUM; i++)
		buf[i] = __cpu_to_be16(radio->registers[WRITE_INDEX(i)]);

	if (i2c_transfer(radio->client->adapter, msgs, 1) != 1)
		return -EIO;

	return 0;
}



/**************************************************************************
 * General Driver Functions - ENTIRE REGISTERS
 **************************************************************************/

/*
 * si470x_get_all_registers - read entire registers
 */
static int si470x_get_all_registers(struct si470x_device *radio)
{
	int i;
	u16 buf[READ_REG_NUM];
	struct i2c_msg msgs[1] = {
		{
			.addr = radio->client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(u16) * READ_REG_NUM,
			.buf = (void *)buf
		},
	};

	if (i2c_transfer(radio->client->adapter, msgs, 1) != 1)
		return -EIO;

	for (i = 0; i < READ_REG_NUM; i++)
		radio->registers[i] = __be16_to_cpu(buf[READ_INDEX(i)]);

	return 0;
}



/**************************************************************************
 * File Operations Interface
 **************************************************************************/

/*
 * si470x_fops_open - file open
 */
int si470x_fops_open(struct file *file)
{
	struct si470x_device *radio = video_get_drvdata(video_devdata(file));
    int retval = 0;

	if (unlikely(radio == NULL)) {
		FMDERR("%s:radio is null", __func__);
		return -EINVAL;
	}

	INIT_DELAYED_WORK(&radio->work_scan, si470x_scan);


	if (!atomic_dec_and_test(&radio->users)) {
		FMDBG("%s: Device already in use. Try again later", __func__);
		atomic_inc(&radio->users);
		return -EBUSY;
	}

	radio->irq = gpio_to_irq(radio->int_gpio);

	if (radio->irq < 0) {
		FMDERR("%s: gpio_to_irq returned %d\n", __func__, radio->irq);
	}

	FMDBG("irq number is = %d\n", radio->irq);

	return retval;
}


/*
 * si470x_fops_release - file release
 */
int si470x_fops_release(struct file *file)
{
	struct si470x_device *radio = video_drvdata(file);
    int retval = 0;

    FMDERR("%s()....\n", __func__);

	if (unlikely(radio == NULL))
		return -EINVAL;

	if (radio->mode == FM_RECV) {
		radio->mode = FM_OFF;
        retval = si470x_stop(radio);
		if (retval < 0)
			FMDERR("Err on disable FM %d\n", retval);
	}

	FMDBG("%s, Disabling the IRQs\n", __func__);

	atomic_inc(&radio->users);

	return retval;
}



/**************************************************************************
 * Video4Linux Interface
 **************************************************************************/

/*
 * si470x_vidioc_querycap - query device capabilities
 */
int si470x_vidioc_querycap(struct file *file, void *priv,
                           struct v4l2_capability *capability)
{
	strlcpy(capability->driver, DRIVER_NAME, sizeof(capability->driver));
	strlcpy(capability->card, DRIVER_CARD, sizeof(capability->card));
	capability->device_caps = V4L2_CAP_HW_FREQ_SEEK | V4L2_CAP_READWRITE |
		V4L2_CAP_TUNER | V4L2_CAP_RADIO | V4L2_CAP_RDS_CAPTURE;
	capability->capabilities = capability->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}


static int clear_stc_int(struct si470x_device *radio)
{
    int retval = 0;

	mutex_lock(&radio->lock);

    //Clear Seek Bit
    radio->registers[POWERCFG] &= ~POWERCFG_SEEK;
    retval = si470x_set_register(radio, POWERCFG);
    if (retval < 0)
        FMDERR("In %s, seek failed with error %d\n", __func__, retval);

	mutex_unlock(&radio->lock);
    return retval;
}


/**************************************************************************
 * I2C Interface
 **************************************************************************/

/*
 * si470x_i2c_interrupt - interrupt handler
 */
static irqreturn_t si470x_i2c_interrupt(int irq, void *dev_id)
{
	struct si470x_device *radio = dev_id;
	unsigned char regnr;
	unsigned char blocknum;
	unsigned short bler; /* rds block errors */
	unsigned short rds;
	unsigned char tmpbuf[3];
	int retval = 0;

	/* check Seek/Tune Complete */
	retval = si470x_get_register(radio, STATUSRSSI);
	if (retval < 0)
		goto end;

	if (radio->registers[STATUSRSSI] & STATUSRSSI_STC) {

        si470x_get_register(radio, STATUSRSSI);

		if (radio->seek_tune_status == TUNE_PENDING) {
			FMDERR("In %s, posting SILABS_EVT_TUNE_SUCC event\n",
				__func__);
			si470x_fm_q_event(radio, SILABS_EVT_TUNE_SUCC);
			radio->seek_tune_status = NO_SEEK_TUNE_PENDING;
            complete(&radio->completion);
		} else if (radio->seek_tune_status == SEEK_PENDING) {
			FMDERR("%s: posting SILABS_EVT_SEEK_COMPLETE event\n",
				__func__);
			si470x_fm_q_event(radio, SILABS_EVT_SEEK_COMPLETE);
			/* post tune comp evt since seek results in a tune.*/
			FMDERR("%s: posting SILABS_EVT_TUNE_SUCC\n",
				__func__);
			si470x_fm_q_event(radio, SILABS_EVT_TUNE_SUCC);
			radio->seek_tune_status = NO_SEEK_TUNE_PENDING;

		} else if (radio->seek_tune_status == SCAN_PENDING) {
			/*
			 * when scan is pending and STC int is set, signal
			 * so that scan can proceed
			 */
			FMDERR("In %s, signalling scan thread\n", __func__);
            complete(&radio->completion);
        }

        clear_stc_int(radio);
    }

	/* safety checks */
	if ((radio->registers[SYSCONFIG1] & SYSCONFIG1_RDS) == 0)
		goto end;

	/* Update RDS registers */
	for (regnr = 1; regnr < RDS_REGISTER_NUM; regnr++) {
		retval = si470x_get_register(radio, STATUSRSSI + regnr);
		if (retval < 0)
			goto end;
	}

	/* get rds blocks */
	if ((radio->registers[STATUSRSSI] & STATUSRSSI_RDSR) == 0)
		/* No RDS group ready, better luck next time */
		goto end;

	for (blocknum = 0; blocknum < 4; blocknum++) {
		switch (blocknum) {
		default:
			bler = (radio->registers[STATUSRSSI] &
					STATUSRSSI_BLERA) >> 9;
			rds = radio->registers[RDSA];
			break;
		case 1:
			bler = (radio->registers[READCHAN] &
					READCHAN_BLERB) >> 14;
			rds = radio->registers[RDSB];
			break;
		case 2:
			bler = (radio->registers[READCHAN] &
					READCHAN_BLERC) >> 12;
			rds = radio->registers[RDSC];
			break;
		case 3:
			bler = (radio->registers[READCHAN] &
					READCHAN_BLERD) >> 10;
			rds = radio->registers[RDSD];
			break;
		}

		/* Fill the V4L2 RDS buffer */
		put_unaligned_le16(rds, &tmpbuf);
		tmpbuf[2] = blocknum;		/* offset name */
		tmpbuf[2] |= blocknum << 3;	/* received offset */
		if (bler > max_rds_errors)
			tmpbuf[2] |= 0x80;	/* uncorrectable errors */
		else if (bler > 0)
			tmpbuf[2] |= 0x40;	/* corrected error(s) */

		/* copy RDS block to internal buffer */
		memcpy(&radio->buffer[radio->wr_index], &tmpbuf, 3);
		radio->wr_index += 3;

		/* wrap write pointer */
		if (radio->wr_index >= radio->buf_size)
			radio->wr_index = 0;

		/* check for overflow */
		if (radio->wr_index == radio->rd_index) {
			/* increment and wrap read pointer */
			radio->rd_index += 3;
			if (radio->rd_index >= radio->buf_size)
				radio->rd_index = 0;
		}
	}

	if (radio->wr_index != radio->rd_index)
		wake_up_interruptible(&radio->read_queue);

end:
	return IRQ_HANDLED;
}


static int fm_configure_gpios(struct si470x_device *radio, bool on)
{
	int rc = 0;
	int fm_reset_gpio = radio->reset_gpio;
	int fm_int_gpio = radio->int_gpio;

	if (on) {
		/*
		 * Reset pin configuration.
		 * write "0'' to make sure the chip is in reset.
		 */
		rc = gpio_direction_output(fm_reset_gpio, 0);
		if (rc) {
			FMDERR("Unable to set direction\n");
			return rc;
		}
		/* Wait for the value to take effect on gpio. */
		msleep(100);
		/* write "1" to bring the chip out of reset.*/
		rc = gpio_direction_output(fm_reset_gpio, 1);
		if (rc) {
			FMDERR("Unable to set direction\n");
			return rc;
		}
		/* Wait for the value to take effect on gpio. */
		msleep(100);

		rc = gpio_direction_input(fm_int_gpio);
		if (rc) {
			FMDERR("unable to set the gpio %d direction(%d)\n",
                   fm_int_gpio, rc);
			return rc;
		}
		/* Wait for the value to take effect on gpio. */
		msleep(100);
	} else {
		/*Turn OFF sequence */
		gpio_set_value(fm_reset_gpio, 0);

		rc = gpio_direction_input(fm_reset_gpio);
		if (rc)
			FMDERR("Unable to set direction\n");
		/* Wait for some time for the value to take effect. */
		msleep(100);
	}
	return rc;
}

static int silabs_fm_areg_cfg(struct si470x_device *radio, bool on)
{
	int rc = 0;
	struct fm_power_vreg_data *vreg;

	vreg = radio->areg;
	if (!vreg) {
		FMDERR("In %s, areg is NULL\n", __func__);
		return rc;
	}
	if (on) {
		FMDERR("vreg is : %s", vreg->name);
		if (vreg->set_voltage_sup) {
			rc = regulator_set_voltage(vreg->reg,
                                       vreg->low_vol_level,
                                       vreg->high_vol_level);
			if (rc < 0) {
				FMDERR("set_vol(%s) fail %d\n", vreg->name, rc);
				return rc;
			}
		}
		rc = regulator_enable(vreg->reg);
		if (rc < 0) {
			FMDERR("reg enable(%s) failed.rc=%d\n", vreg->name, rc);
			if (vreg->set_voltage_sup) {
				regulator_set_voltage(vreg->reg,
                                      0,
                                      vreg->high_vol_level);
			}
			return rc;
		}
		vreg->is_enabled = true;

	} else {
		rc = regulator_disable(vreg->reg);
		if (rc < 0) {
			FMDERR("reg disable(%s) fail rc=%d\n", vreg->name, rc);
			return rc;
		}
		vreg->is_enabled = false;

		if (vreg->set_voltage_sup) {
			/* Set the min voltage to 0 */
			rc = regulator_set_voltage(vreg->reg,
                                       0,
                                       vreg->high_vol_level);
			if (rc < 0) {
				FMDERR("set_vol(%s) fail %d\n", vreg->name, rc);
				return rc;
			}
		}
	}
	return rc;
}

static int silabs_fm_dreg_cfg(struct si470x_device *radio, bool on)
{
	int rc = 0;
	struct fm_power_vreg_data *vreg;

	vreg = radio->dreg;
	if (!vreg) {
		FMDERR("In %s, dreg is NULL\n", __func__);
		return rc;
	}

	if (on) {
		FMDERR("vreg is : %s", vreg->name);
		if (vreg->set_voltage_sup) {
			rc = regulator_set_voltage(vreg->reg,
                                       vreg->low_vol_level,
                                       vreg->high_vol_level);
			if (rc < 0) {
				FMDERR("set_vol(%s) fail %d\n", vreg->name, rc);
				return rc;
			}
		}

		rc = regulator_enable(vreg->reg);
		if (rc < 0) {
			FMDERR("reg enable(%s) failed.rc=%d\n", vreg->name, rc);
			if (vreg->set_voltage_sup) {
				regulator_set_voltage(vreg->reg,
                                      0,
                                      vreg->high_vol_level);
			}
			return rc;
		}
        vreg->is_enabled = true;
	} else {
		rc = regulator_disable(vreg->reg);
		if (rc < 0) {
			FMDERR("reg disable(%s) fail. rc=%d\n", vreg->name, rc);
			return rc;
		}
		vreg->is_enabled = false;

		if (vreg->set_voltage_sup) {
			/* Set the min voltage to 0 */
			rc = regulator_set_voltage(vreg->reg,
                                       0,
                                       vreg->high_vol_level);
			if (rc < 0) {
				FMDERR("set_vol(%s) fail %d\n", vreg->name, rc);
				return rc;
			}
		}
	}
	return rc;
}

static int silabs_fm_power_cfg(struct si470x_device *radio, bool on)
{
	int rc = 0;

	if (on) {
		/* Turn ON sequence */
		rc = silabs_fm_dreg_cfg(radio, on);
		if (rc < 0) {
			FMDERR("In %s, dreg cfg failed %x\n", __func__, rc);
			return rc;
		}
		rc = silabs_fm_areg_cfg(radio, on);
		if (rc < 0) {
			FMDERR("In %s, areg cfg failed %x\n", __func__, rc);
			silabs_fm_dreg_cfg(radio, false);
			return rc;
		}

		rc = fm_configure_gpios(radio, on);
		if (rc < 0) {
			FMDERR("fm_power gpio config failed\n");
			silabs_fm_dreg_cfg(radio, false);
			silabs_fm_areg_cfg(radio, false);
			return rc;
		}
	} else {
		/* Turn OFF sequence */
		rc = fm_configure_gpios(radio, on);
		if (rc < 0)
			FMDERR("fm_power gpio config failed");

		rc = silabs_fm_dreg_cfg(radio, on);
		if (rc < 0)
			FMDERR("In %s, dreg cfg failed %x\n", __func__, rc);
		rc = silabs_fm_areg_cfg(radio, on);
		if (rc < 0)
			FMDERR("In %s, areg cfg failed %x\n", __func__, rc);
	}
	return rc;
}

static int silabs_parse_dt(struct device *dev,
                           struct si470x_device *radio)
{
	int rc = 0;
	struct device_node *np = dev->of_node;

	radio->reset_gpio = of_get_named_gpio(np, "silabs,reset-gpio", 0);
	if (radio->reset_gpio < 0) {
		FMDERR("silabs-reset-gpio not provided in device tree");
		return radio->reset_gpio;
	}

	rc = gpio_request(radio->reset_gpio, "fm_rst_gpio_n");
	if (rc) {
		FMDERR("unable to request gpio %d (%d)\n",
               radio->reset_gpio, rc);
		return rc;
	}

	radio->int_gpio = of_get_named_gpio(np, "silabs,int-gpio", 0);
	FMDERR("radio->int_gpio = %d\n", radio->int_gpio);
	if (radio->int_gpio < 0) {
		FMDERR("silabs-int-gpio not provided in device tree");
		rc = radio->int_gpio;
		goto err_int_gpio;
	}

	rc = gpio_request(radio->int_gpio, "silabs_fm_int_n");
	if (rc) {
		FMDERR("unable to request gpio %d (%d)\n",
               radio->int_gpio, rc);
		goto err_int_gpio;
	}

	return rc;

	gpio_free(radio->int_gpio);
err_int_gpio:
	gpio_free(radio->reset_gpio);

	return rc;
}

static int silabs_dt_parse_vreg_info(struct device *dev,
                                     struct fm_power_vreg_data *vreg, const char *vreg_name)
{
	int ret = 0;
	u32 vol_suply[2];
	struct device_node *np = dev->of_node;

	ret = of_property_read_u32_array(np, vreg_name, vol_suply, 2);
	if (ret < 0) {
		FMDERR("Invalid property name\n");
		ret =  -EINVAL;
	} else {
		vreg->low_vol_level = vol_suply[0];
		vreg->high_vol_level = vol_suply[1];
	}
	return ret;
}


/*
 * si470x_i2c_probe - probe for the device
 */
static int si470x_i2c_probe(struct i2c_client *client,
                            const struct i2c_device_id *id)
{
	struct si470x_device *radio;
	struct regulator *vreg = NULL;
	int retval = 0;
    int i = 0;
	int kfifo_alloc_rc = 0;
	unsigned char version_warning = 0;
	extern char* androidboot_mode;

	if (strcmp(androidboot_mode,"charger")==0) {
		printk("[Power] %s: skip this driver in charger mode\n", __func__);
		return 0;
	}

	/* private data allocation and initialization */
	radio = kzalloc(sizeof(struct si470x_device), GFP_KERNEL);
	if (!radio) {
		retval = -ENOMEM;
		goto err_initial;
	}


	vreg = regulator_get(&client->dev, "va");

	if (IS_ERR(vreg)) {
		/*
		 * if analog voltage regulator, VA is not ready yet, return
		 * -EPROBE_DEFER to kernel so that probe will be called at
		 * later point of time.
		 */
		if (PTR_ERR(vreg) == -EPROBE_DEFER) {
			FMDERR("In %s, areg probe defer\n", __func__);
			return PTR_ERR(vreg);
		}
	} else {
        radio->areg = devm_kzalloc(&client->dev,
                                   sizeof(struct fm_power_vreg_data),
                                   GFP_KERNEL);
		if (!radio->areg) {
			FMDERR("%s: allocating memory for areg failed\n",
                   __func__);
			regulator_put(vreg);
			kfree(radio);
			return -ENOMEM;
		}

		radio->areg->reg = vreg;
		radio->areg->name = "va";
		radio->areg->is_enabled = 0;
		retval = silabs_dt_parse_vreg_info(&client->dev,
                                           radio->areg, "silabs,va-supply-voltage");
		if (retval < 0) {
			FMDERR("%s: parsing va-supply failed\n", __func__);
			goto mem_alloc_fail;
		}
	}

	retval = silabs_parse_dt(&client->dev, radio);
	if (retval) {
		FMDERR("%s: Parsing DT failed(%d)", __func__, retval);
		regulator_put(vreg);
		kfree(radio);
		return retval;
	}


	vreg = regulator_get(&client->dev, "vdd");

	if (IS_ERR(vreg)) {
		FMDERR("In %s, vdd supply is not provided\n", __func__);
	} else {
		radio->dreg = devm_kzalloc(&client->dev,
                                   sizeof(struct fm_power_vreg_data),
                                   GFP_KERNEL);
		if (!radio->dreg) {
			FMDERR("%s: allocating memory for dreg failed\n",
                   __func__);
			retval = -ENOMEM;
			regulator_put(vreg);
			goto mem_alloc_fail;
		}

		radio->dreg->reg = vreg;
		radio->dreg->name = "vdd";
		radio->dreg->is_enabled = 0;
		retval = silabs_dt_parse_vreg_info(&client->dev,
                                           radio->dreg, "silabs,vdd-supply-voltage");
		if (retval < 0) {
			FMDERR("%s: parsing vdd-supply failed\n", __func__);
			goto err_dreg;
		}
	}


	/* initial gpio pin config & Power up */
	retval = silabs_fm_power_cfg(radio, 1);
	if (retval) {
		FMDERR("%s: failed config gpio & pmic\n", __func__);
		goto err_dreg;
	}

	radio->client = client;
	radio->band = 1; /* Default to 76 - 108 MHz */
	mutex_init(&radio->lock);
	radio->seek_tune_status = 0;
	init_completion(&radio->completion);
	/* initialize wait queue for event read */
	init_waitqueue_head(&radio->event_queue);
	/* initialize wait queue for raw rds read */
	init_waitqueue_head(&radio->read_queue);

	for (i = 0; i < SILABS_FM_BUF_MAX; i++) {
	    spin_lock_init(&radio->buf_lock[i]);

		if (i == SILABS_FM_BUF_RAW_RDS)
			kfifo_alloc_rc = kfifo_alloc(&radio->data_buf[i],
                                         FM_RDS_BUF * 3, GFP_KERNEL);
		else if (i == SILABS_FM_BUF_RT_RDS)
			kfifo_alloc_rc = kfifo_alloc(&radio->data_buf[i],
                                         STD_BUF_SIZE * 2, GFP_KERNEL);
		else
			kfifo_alloc_rc = kfifo_alloc(&radio->data_buf[i],
                                         STD_BUF_SIZE, GFP_KERNEL);

		if (kfifo_alloc_rc != 0) {
			FMDERR("%s: failed allocating buffers %d\n",
                   __func__, kfifo_alloc_rc);
			retval = -ENOMEM;
			goto err_fifo_alloc;
        }
	}

	/* initializing the device count  */
	atomic_set(&radio->users, 1);

	radio->wqueue = NULL;
	radio->wqueue_scan = NULL;
	radio->wqueue_af = NULL;
	radio->wqueue_rds = NULL;

	/* video device allocation */
	radio->videodev = video_device_alloc();
	if (!radio->videodev) {
		FMDERR("radio->videodev is NULL\n");
		goto err_dreg;
	}

	/* video device initialization */
	memcpy(radio->videodev, &si470x_viddev_template, sizeof(si470x_viddev_template));
	strlcpy(radio->v4l2_dev.name, DRIVER_NAME,
            sizeof(radio->v4l2_dev.name));
	retval = v4l2_device_register(NULL, &radio->v4l2_dev);
	if (retval)
		goto err_dreg;

	radio->videodev->v4l2_dev = &radio->v4l2_dev;

	video_set_drvdata(radio->videodev, radio);
	

	/* power up : need 110ms */
	radio->registers[POWERCFG] = POWERCFG_ENABLE;
	if (si470x_set_register(radio, POWERCFG) < 0) {
		retval = -EIO;
		goto err_radio;
	}
	msleep(110);

	/* get device and chip versions */
	if (si470x_get_all_registers(radio) < 0) {
		retval = -EIO;
		goto err_radio;
	}
	dev_info(&client->dev, "DeviceID=0x%4.4hx ChipID=0x%4.4hx\n",
             radio->registers[DEVICEID], radio->registers[CHIPID]);
	if ((radio->registers[CHIPID] & CHIPID_FIRMWARE) < RADIO_FW_VERSION) {
		dev_warn(&client->dev,
                 "This driver is known to work with "
                 "firmware version %hu,\n", RADIO_FW_VERSION);
		dev_warn(&client->dev,
                 "but the device has firmware version %hu.\n",
                 radio->registers[CHIPID] & CHIPID_FIRMWARE);
		version_warning = 1;
	}

	/* give out version warning */
	if (version_warning == 1) {
		dev_warn(&client->dev,
                 "If you have some trouble using this driver,\n");
		dev_warn(&client->dev,
                 "please report to V4L ML at "
                 "linux-media@vger.kernel.org\n");
	}

	/* rds buffer allocation */
	radio->buf_size = rds_buf * 3;
	radio->buffer = kmalloc(radio->buf_size, GFP_KERNEL);
	if (!radio->buffer) {
		retval = -EIO;
		goto err_radio;
	}

	/* rds buffer configuration */
	radio->wr_index = 0;
	radio->rd_index = 0;

	radio->irq = gpio_to_irq(radio->int_gpio);

	if (radio->irq < 0) {
		FMDERR("%s: gpio_to_irq returned %d\n", __func__, radio->irq);
		goto err_rds;
	}

	retval = devm_request_threaded_irq(&client->dev, radio->irq, NULL, si470x_i2c_interrupt,
                                       IRQF_TRIGGER_FALLING|IRQF_ONESHOT, DRIVER_NAME, radio);
	if (retval) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_rds;
	}

	FMDBG("%s: creating work q for scan\n", __func__);
	radio->wqueue_scan  = create_singlethread_workqueue("sifmradioscan");

	if (!radio->wqueue_scan) {
		retval = -ENOMEM;
		goto err_wqueue_scan;
	}

	/* register video device */
	retval = video_register_device(radio->videodev, VFL_TYPE_RADIO,
                                   radio_nr);
	if (retval) {
		dev_warn(&client->dev, "Could not register video device\n");
		goto err_all;
	}
	i2c_set_clientdata(client, radio);

	radio->registers[POWERCFG] = POWERCFG_ENABLE | POWERCFG_DISABLE;
	if (si470x_set_register(radio, POWERCFG) < 0) {
		retval = -EIO;
		goto err_radio;
	}

	return 0;
err_all:
	free_irq(client->irq, radio);
err_rds:
	kfree(radio->buffer);
err_wqueue_scan:
	destroy_workqueue(radio->wqueue);
err_fifo_alloc:
	for (i--; i >= 0; i--)
		kfifo_free(&radio->data_buf[i]);
	video_device_release(radio->videodev);
err_dreg:
	if (radio->dreg && radio->dreg->reg) {
		regulator_put(radio->dreg->reg);
		devm_kfree(&client->dev, radio->dreg);
	}
mem_alloc_fail:
	if (radio->areg && radio->areg->reg) {
		regulator_put(radio->areg->reg);
		devm_kfree(&client->dev, radio->areg);
	}
err_radio:
	kfree(radio);
err_initial:
	return retval;
}


/*
 * si470x_i2c_remove - remove the device
 */
static int si470x_i2c_remove(struct i2c_client *client)
{
	struct si470x_device *radio = i2c_get_clientdata(client);

	free_irq(client->irq, radio);
	video_unregister_device(radio->videodev);
	kfree(radio);

	return 0;
}

static const struct of_device_id silabs_fm_match[] = {
	{.compatible = "silabs,si4705"},
	{}
};

/*
 * si470x_i2c_driver - i2c driver interface
 */
static struct i2c_driver si470x_i2c_driver = {
	.driver = {
		.name		= "si470x",
		.owner		= THIS_MODULE,
		.of_match_table = silabs_fm_match,
	},
	.probe			= si470x_i2c_probe,
	.remove			= si470x_i2c_remove,
	.id_table		= si470x_i2c_id,
};

module_i2c_driver(si470x_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
