/*
 *  drivers/media/radio/si470x/radio-si470x-common.c
 *
 *  Driver for radios with Silicon Labs Si470x FM Radio Receivers
 *
 *  Copyright (c) 2009 Tobias Lorenz <tobias.lorenz@gmx.net>
 *  Copyright (c) 2012 Hans de Goede <hdegoede@redhat.com>
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


/*
 * History:
 * 2008-01-12	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Version 1.0.0
 *		- First working version
 * 2008-01-13	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Version 1.0.1
 *		- Improved error handling, every function now returns errno
 *		- Improved multi user access (start/mute/stop)
 *		- Channel doesn't get lost anymore after start/mute/stop
 *		- RDS support added (polling mode via interrupt EP 1)
 *		- marked default module parameters with *value*
 *		- switched from bit structs to bit masks
 *		- header file cleaned and integrated
 * 2008-01-14	Tobias Lorenz <tobias.lorenz@gmx.net>
 * 		Version 1.0.2
 * 		- hex values are now lower case
 * 		- commented USB ID for ADS/Tech moved on todo list
 * 		- blacklisted si470x in hid-quirks.c
 * 		- rds buffer handling functions integrated into *_work, *_read
 * 		- rds_command in si470x_poll exchanged against simple retval
 * 		- check for firmware version 15
 * 		- code order and prototypes still remain the same
 * 		- spacing and bottom of band codes remain the same
 * 2008-01-16	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Version 1.0.3
 * 		- code reordered to avoid function prototypes
 *		- switch/case defaults are now more user-friendly
 *		- unified comment style
 *		- applied all checkpatch.pl v1.12 suggestions
 *		  except the warning about the too long lines with bit comments
 *		- renamed FMRADIO to RADIO to cut line length (checkpatch.pl)
 * 2008-01-22	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Version 1.0.4
 *		- avoid poss. locking when doing copy_to_user which may sleep
 *		- RDS is automatically activated on read now
 *		- code cleaned of unnecessary rds_commands
 *		- USB Vendor/Product ID for ADS/Tech FM Radio Receiver verified
 *		  (thanks to Guillaume RAMOUSSE)
 * 2008-01-27	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Version 1.0.5
 *		- number of seek_retries changed to tune_timeout
 *		- fixed problem with incomplete tune operations by own buffers
 *		- optimization of variables and printf types
 *		- improved error logging
 * 2008-01-31	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Oliver Neukum <oliver@neukum.org>
 *		Version 1.0.6
 *		- fixed coverity checker warnings in *_usb_driver_disconnect
 *		- probe()/open() race by correct ordering in probe()
 *		- DMA coherency rules by separate allocation of all buffers
 *		- use of endianness macros
 *		- abuse of spinlock, replaced by mutex
 *		- racy handling of timer in disconnect,
 *		  replaced by delayed_work
 *		- racy interruptible_sleep_on(),
 *		  replaced with wait_event_interruptible()
 *		- handle signals in read()
 * 2008-02-08	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Oliver Neukum <oliver@neukum.org>
 *		Version 1.0.7
 *		- usb autosuspend support
 *		- unplugging fixed
 * 2008-05-07	Tobias Lorenz <tobias.lorenz@gmx.net>
 *		Version 1.0.8
 *		- hardware frequency seek support
 *		- afc indication
 *		- more safety checks, let si470x_get_freq return errno
 *		- vidioc behavior corrected according to v4l2 spec
 * 2008-10-20	Alexey Klimov <klimov.linux@gmail.com>
 * 		- add support for KWorld USB FM Radio FM700
 * 		- blacklisted KWorld radio in hid-core.c and hid-ids.h
 * 2008-12-03	Mark Lord <mlord@pobox.com>
 *		- add support for DealExtreme USB Radio
 * 2009-01-31	Bob Ross <pigiron@gmx.com>
 *		- correction of stereo detection/setting
 *		- correction of signal strength indicator scaling
 * 2009-01-31	Rick Bronson <rick@efn.org>
 *		Tobias Lorenz <tobias.lorenz@gmx.net>
 *		- add LED status output
 *		- get HW/SW version from scratchpad
 * 2009-06-16   Edouard Lafargue <edouard@lafargue.name>
 *		Version 1.0.10
 *		- add support for interrupt mode for RDS endpoint,
 *                instead of polling.
 *                Improves RDS reception significantly
 */


/* kernel includes */
#include "radio-si4709.h"
#include <linux/delay.h>

static int cancel_seek(struct si470x_device *radio);

/**************************************************************************
 * Module Parameters
 **************************************************************************/

/* Spacing (kHz) */
/* 0: 200 kHz (USA, Australia) */
/* 1: 100 kHz (Europe, Japan) */
/* 2:  50 kHz */
static unsigned short space = 2;
module_param(space, ushort, 0444);
MODULE_PARM_DESC(space, "Spacing: 0=200kHz 1=100kHz *2=50kHz*");

/* De-emphasis */
/* 0: 75 us (USA) */
/* 1: 50 us (Europe, Australia, Japan) */
static unsigned short de = 1;
module_param(de, ushort, 0444);
MODULE_PARM_DESC(de, "De-emphasis: 0=75us *1=50us*");

/* Tune timeout */
static unsigned int tune_timeout = 3000;
module_param(tune_timeout, uint, 0644);
MODULE_PARM_DESC(tune_timeout, "Tune timeout: *3000*");

/* Seek timeout */
static unsigned int seek_timeout = 5000;
module_param(seek_timeout, uint, 0644);
MODULE_PARM_DESC(seek_timeout, "Seek timeout: *5000*");

static const struct v4l2_frequency_band bands[] = {
	{
		.type = V4L2_TUNER_RADIO,
		.index = 0,
		.capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO |
        V4L2_TUNER_CAP_RDS | V4L2_TUNER_CAP_RDS_BLOCK_IO |
        V4L2_TUNER_CAP_FREQ_BANDS |
        V4L2_TUNER_CAP_HWSEEK_BOUNDED |
        V4L2_TUNER_CAP_HWSEEK_WRAP,
		.rangelow   =  87500 * 16,
		.rangehigh  = 108000 * 16,
		.modulation = V4L2_BAND_MODULATION_FM,
	},
	{
		.type = V4L2_TUNER_RADIO,
		.index = 1,
		.capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO |
        V4L2_TUNER_CAP_RDS | V4L2_TUNER_CAP_RDS_BLOCK_IO |
        V4L2_TUNER_CAP_FREQ_BANDS |
        V4L2_TUNER_CAP_HWSEEK_BOUNDED |
        V4L2_TUNER_CAP_HWSEEK_WRAP,
		.rangelow   =  76000 * 16,
		.rangehigh  = 108000 * 16,
		.modulation = V4L2_BAND_MODULATION_FM,
	},
	{
		.type = V4L2_TUNER_RADIO,
		.index = 2,
		.capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO |
        V4L2_TUNER_CAP_RDS | V4L2_TUNER_CAP_RDS_BLOCK_IO |
        V4L2_TUNER_CAP_FREQ_BANDS |
        V4L2_TUNER_CAP_HWSEEK_BOUNDED |
        V4L2_TUNER_CAP_HWSEEK_WRAP,
		.rangelow   =  76000 * 16,
		.rangehigh  =  90000 * 16,
		.modulation = V4L2_BAND_MODULATION_FM,
	},
};

/* For bounds checking. */
const unsigned char MIN_RDS_STD = 0x00;
const unsigned char MAX_RDS_STD = 0x02;
const unsigned char MIN_SRCH_MODE = 0x00;
const unsigned char MAX_SRCH_MODE = 0x02;

/**************************************************************************
 * Generic Functions
 **************************************************************************/
static inline bool is_valid_chan_spacing(int spacing)
{
	if ((spacing == 0) ||
		(spacing == 1) ||
		(spacing == 2))
		return 1;
	else
		return 0;
}

static inline bool is_valid_srch_mode(int srch_mode)
{
	if ((srch_mode >= MIN_SRCH_MODE) &&
		(srch_mode <= MAX_SRCH_MODE))
		return 1;
	else
		return 0;
}

static bool is_enable_rx_possible(struct si470x_device *radio)
{
	bool retval = true;

	if (radio->mode == FM_OFF || radio->mode == FM_RECV)
		retval = false;

	return retval;
}

void si470x_fm_q_event(struct si470x_device *radio,
                              enum silabs_evt_t event)
{
	struct kfifo *data_b;
	unsigned char evt = event;

	data_b = &radio->data_buf[SILABS_FM_BUF_EVENTS];

	FMDERR("updating event_q with event %x\n", event);
	if (kfifo_in_locked(data_b,
                        &evt,
                        1,
                        &radio->buf_lock[SILABS_FM_BUF_EVENTS]))
		wake_up_interruptible(&radio->event_queue);
}

static void si470x_search(struct si470x_device *radio, bool on)
{
	int current_freq_khz;

	current_freq_khz = radio->tuned_freq_khz;

	if (on) {
		FMDBG("%s: Queuing the work onto scan work q\n", __func__);
		queue_delayed_work(radio->wqueue_scan, &radio->work_scan,
                           msecs_to_jiffies(SILABS_DELAY_MSEC));
	} else {
		cancel_seek(radio);
		si470x_fm_q_event(radio, SILABS_EVT_SEEK_COMPLETE);
	}
}

/*
 * si470x_set_spacing - set the spacing
 */
static int si470x_set_spacing(struct si470x_device *radio, int spacing)
{
    int retval = 0;

    retval = si470x_get_register(radio, SYSCONFIG2);
    if (retval < 0)
        return retval;

    radio->registers[SYSCONFIG2] &= ~SYSCONFIG2_SPACE;

    if (spacing == 0)
        radio->registers[SYSCONFIG2] &= ~SYSCONFIG2_SPACE;
    else if (spacing == 1)
        radio->registers[SYSCONFIG2] |= 1 << 4;
    else if (spacing == 2)
        radio->registers[SYSCONFIG2] |= 1 << 5;

    return si470x_set_register(radio, SYSCONFIG2);
}

/*
 * si470x_set_emphasis - set the emphasis
 */
static int si470x_set_emphasis(struct si470x_device *radio, int emp)
{
    int retval = 0;

    retval = si470x_get_register(radio, SYSCONFIG1);
    if (retval < 0)
        return retval;

    radio->registers[SYSCONFIG1] &= ~SYSCONFIG1_DE;

    if (emp == 1)
        radio->registers[SYSCONFIG1] |= SYSCONFIG1_DE;


    return si470x_set_register(radio, SYSCONFIG1);
}

static int set_hard_mute(struct si470x_device *radio, bool val)
{
	if (val == true)
        radio->registers[POWERCFG] &= ~POWERCFG_DMUTE;
    else
        radio->registers[POWERCFG] |= POWERCFG_DMUTE;
    
    return si470x_set_register(radio, POWERCFG);
}


/*
 * si470x_set_band - set the band
 */
static int si470x_set_band(struct si470x_device *radio, int band)
{
	if (radio->band == band)
		return 0;

	radio->band = band;
	radio->registers[SYSCONFIG2] &= ~SYSCONFIG2_BAND;
	radio->registers[SYSCONFIG2] |= radio->band << 6;
	return si470x_set_register(radio, SYSCONFIG2);
}

/*
 * si470x_set_chan - set the channel
 */
static int si470x_set_chan(struct si470x_device *radio, unsigned short chan)
{
	int retval;
	bool timed_out = false;

	/* start tuning */
	radio->registers[CHANNEL] &= ~CHANNEL_CHAN;
	radio->registers[CHANNEL] |= CHANNEL_TUNE | chan;
	retval = si470x_set_register(radio, CHANNEL);
	if (retval < 0)
		goto done;

	/* wait till tune operation has completed */
	reinit_completion(&radio->completion);
	retval = wait_for_completion_timeout(&radio->completion,
                                         msecs_to_jiffies(tune_timeout));
	if (!retval)
		timed_out = true;

	retval = si470x_get_register(radio, CHANNEL);
	if (retval < 0)
	    goto done;

	retval = si470x_get_register(radio, STATUSRSSI);
	if (retval < 0)
	    goto done;

	if ((radio->registers[STATUSRSSI] & STATUSRSSI_STC) == 0)
		dev_warn(&radio->videodev->dev, "tune does not complete\n");
	if (timed_out)
		dev_warn(&radio->videodev->dev,
                 "tune timed out after %u ms\n", tune_timeout);

	/* stop tuning */
	radio->registers[CHANNEL] &= ~CHANNEL_TUNE;
	retval = si470x_set_register(radio, CHANNEL);

done:
	return retval;
}

/*
 * si470x_get_step - get channel spacing
 */
static unsigned int si470x_get_step(struct si470x_device *radio)
{
	/* Spacing (kHz) */
	switch ((radio->registers[SYSCONFIG2] & SYSCONFIG2_SPACE) >> 4) {
        /* 0: 200 kHz (USA, Australia) */
	case 0:
		return 200 * 16;
        /* 1: 100 kHz (Europe, Japan) */
	case 1:
		return 100 * 16;
        /* 2:  50 kHz */
	default:
		return 50 * 16;
	}
}


/*
 * si470x_get_freq - get the frequency
 */
static int si470x_get_freq(struct si470x_device *radio, unsigned int *freq)
{
	int chan, retval;

	/* read channel */
	retval = si470x_get_register(radio, READCHAN);
	chan = radio->registers[READCHAN] & READCHAN_READCHAN;

	/* Frequency (MHz) = Spacing (kHz) x Channel + Bottom of Band (MHz) */
	*freq = chan * si470x_get_step(radio) + bands[radio->band].rangelow;

	return retval;
}


/*
 * si470x_set_freq - set the frequency
 */
int si470x_set_freq(struct si470x_device *radio, unsigned int freq)
{
	unsigned short chan;

	freq = clamp(freq, bands[radio->band].rangelow,
                 bands[radio->band].rangehigh);
	/* Chan = [ Freq (Mhz) - Bottom of Band (MHz) ] / Spacing (kHz) */
	chan = (freq - bands[radio->band].rangelow) / si470x_get_step(radio);

	return si470x_set_chan(radio, chan);
}

static int si470x_seek(struct si470x_device *radio, int dir, int wrap)
{
	int retval = 0;

	mutex_lock(&radio->lock);

	radio->registers[POWERCFG] |= POWERCFG_SEEK;
	if (wrap)
		radio->registers[POWERCFG] |= POWERCFG_SKMODE;
	else
		radio->registers[POWERCFG] &= ~POWERCFG_SKMODE;

	if (dir == SRCH_DIR_UP)
		radio->registers[POWERCFG] |= POWERCFG_SEEKUP;
	else
		radio->registers[POWERCFG] &= ~POWERCFG_SEEKUP;

	retval = si470x_set_register(radio, POWERCFG);
	if (retval < 0)
		FMDERR("In %s, seek failed with error %d\n", __func__, retval);

	mutex_unlock(&radio->lock);
	return retval;
}

static int cancel_seek(struct si470x_device *radio)
{
	int retval = 0;

	mutex_lock(&radio->lock);

	/* stop seeking */
	radio->registers[POWERCFG] &= ~POWERCFG_SEEK;
	retval = si470x_set_register(radio, POWERCFG);
	if (retval < 0)
		FMDERR("%s: cancel_seek failed, error %d\n", __func__, retval);

	mutex_unlock(&radio->lock);
	radio->is_search_cancelled = true;

	return retval;
}

/*
 * si470x_start - switch on radio
 */
int si470x_start(struct si470x_device *radio)
{
	int retval;

    mutex_lock(&radio->lock);

	/* powercfg */
	radio->registers[POWERCFG] =
		POWERCFG_DMUTE | POWERCFG_ENABLE | POWERCFG_RDSM;
	retval = si470x_set_register(radio, POWERCFG);
	if (retval < 0)
		goto done;

	msleep(110);

	/* sysconfig 1 */
	radio->registers[SYSCONFIG1] =
		(de << 11) & SYSCONFIG1_DE;		/* DE*/
	retval = si470x_set_register(radio, SYSCONFIG1);
	if (retval < 0)
		goto done;

	/* sysconfig 2 */
	radio->registers[SYSCONFIG2] =
		(0x5  << 8) |				/* SEEKTH */
		((radio->band << 6) & SYSCONFIG2_BAND) |/* BAND */
		((space << 4) & SYSCONFIG2_SPACE) |	/* SPACE */
		15;					/* VOLUME (max) */
	retval = si470x_set_register(radio, SYSCONFIG2);
	if (retval < 0)
		goto done;

	/* sysconfig 3 */
	radio->registers[SYSCONFIG3] =
		(0x1  << 4) |				/* SKSNR */
		1;					        /* SKCNT */
	retval = si470x_set_register(radio, SYSCONFIG3);
	if (retval < 0)
		goto done;

	/* /\* reset last channel *\/ */
	/* retval = si470x_set_chan(radio, */
    /*                          radio->registers[CHANNEL] & CHANNEL_CHAN); */

	/* enable RDS / STC interrupt */
	radio->registers[SYSCONFIG1] |= SYSCONFIG1_RDSIEN;
	radio->registers[SYSCONFIG1] |= SYSCONFIG1_STCIEN;
	radio->registers[SYSCONFIG1] |= SYSCONFIG1_RDS;
	radio->registers[SYSCONFIG1] |= 0x1 << 5;
	radio->registers[SYSCONFIG1] |= 0x1 << 2;
	radio->registers[SYSCONFIG1] |= 0x1 << 1;
	retval = si470x_set_register(radio, SYSCONFIG1);
    if (retval < 0)
        goto done;

    reset_rds(radio);
    if (radio->mode == FM_RECV_TURNING_ON) {
        si470x_fm_q_event(radio, SILABS_EVT_RADIO_READY);
        radio->mode = FM_RECV;
    }

    mutex_unlock(&radio->lock);
	
done:
	return retval;
}


/*
 * si470x_stop - switch off radio
 */
int si470x_stop(struct si470x_device *radio)
{
	int retval;

	/* sysconfig 1 */
	radio->registers[SYSCONFIG1] &= ~SYSCONFIG1_RDS;
	retval = si470x_set_register(radio, SYSCONFIG1);
	if (retval < 0)
		goto done;

	/* powercfg */
	radio->registers[POWERCFG] &= ~POWERCFG_DMUTE;
	/* POWERCFG_ENABLE has to automatically go low */
	radio->registers[POWERCFG] |= POWERCFG_ENABLE |	POWERCFG_DISABLE;
	retval = si470x_set_register(radio, POWERCFG);

	if (radio->mode == FM_TURNING_OFF || radio->mode == FM_RECV) {
        si470x_fm_q_event(radio, SILABS_EVT_RADIO_DISABLED);
        radio->mode = FM_OFF;
    }

done:
	return retval;
}


/*
 * si470x_rds_on - switch on rds reception
 */
static int si470x_rds_on(struct si470x_device *radio)
{
	int retval;

	/* sysconfig 1 */
	radio->registers[SYSCONFIG1] |= SYSCONFIG1_RDS;
	retval = si470x_set_register(radio, SYSCONFIG1);
	if (retval < 0)
		radio->registers[SYSCONFIG1] &= ~SYSCONFIG1_RDS;

	return retval;
}

void update_search_list(struct si470x_device *radio, int freq)
{
	int temp_freq = freq;

	temp_freq = temp_freq -
		(radio->recv_conf.band_low_limit * TUNE_STEP_SIZE);
	temp_freq = temp_freq / 50;
	radio->srch_list.rel_freq[radio->srch_list.num_stations_found].
        rel_freq_lsb = GET_LSB(temp_freq);
	radio->srch_list.rel_freq[radio->srch_list.num_stations_found].
        rel_freq_msb = GET_MSB(temp_freq);
	radio->srch_list.num_stations_found++;
}

void si470x_scan(struct work_struct *work)
{
	struct si470x_device *radio;
	int current_freq_khz;
	u8 bltf;
	u32 temp_freq_khz;
    u32 f1, f2;
	int retval = 0;
	struct kfifo *data_b;
	int len = 0;
    int chan;


	FMDBG("+%s, getting radio handle from work struct\n", __func__);
	radio = container_of(work, struct si470x_device, work_scan.work);

	if (unlikely(radio == NULL)) {
		FMDERR(":radio is null");
		return;
	}

	current_freq_khz = radio->tuned_freq_khz;
	FMDBG("current freq is %d\n", current_freq_khz);

    set_hard_mute(radio, true);

	radio->seek_tune_status = SCAN_PENDING;
	/* tune to lowest freq of the band */
	retval = si470x_set_freq(radio, radio->recv_conf.band_low_limit * TUNE_STEP_SIZE);
	if (retval < 0) {
		FMDERR("%s: Tune to lower band limit failed with error %d\n",
               __func__, retval);
		goto seek_tune_fail;
	}

	reinit_completion(&radio->completion);
	/* wait for tune to complete. */
	if (!wait_for_completion_timeout(&radio->completion,
                                     msecs_to_jiffies(tune_timeout)))
		FMDERR("In %s, didn't receive STC for tune\n", __func__);
	else
		FMDBG("In %s, received STC for tune\n", __func__);

	while (1) {
		/* If scan is cancelled or FM is not ON, break */
		if (radio->is_search_cancelled == true) {
			FMDBG("%s: scan cancelled\n", __func__);
			if (radio->g_search_mode == SCAN_FOR_STRONG)
				goto seek_tune_fail;
			else
				goto seek_cancelled;
		} else if (radio->mode != FM_RECV) {
			FMDERR("%s: FM is not in proper state\n", __func__);
			return;
		}

		retval = si470x_seek(radio, SRCH_DIR_UP, WRAP_ENABLE);
		if (retval < 0) {
			FMDERR("Scan operation failed with error %d\n", retval);
			goto seek_tune_fail;
		}
        reinit_completion(&radio->completion);
		/* wait for seek to complete */
		if (!wait_for_completion_timeout(&radio->completion,
                                         msecs_to_jiffies(seek_timeout))) {
			FMDERR("%s: didn't receive STC for seek\n", __func__);
			/* FM is not correct state or scan is cancelled */
			continue;
		} else {
            bltf = ((radio->registers[STATUSRSSI] & STATUSRSSI_SF) >> 12);
			FMDBG("%s: received STC for seek\n", __func__);
        }

		mutex_lock(&radio->lock);

        retval = si470x_get_register(radio, READCHAN);
        chan = radio->registers[READCHAN] & READCHAN_READCHAN;
        temp_freq_khz = chan * si470x_get_step(radio);
		mutex_unlock(&radio->lock);
		FMDERR("In %s, freq is %d, band_high_limit = %d, chan = %d\n", __func__, temp_freq_khz, radio->recv_conf.band_high_limit * 16 * 10, chan);

        f1 = radio->recv_conf.band_high_limit * 160;
        f2 = temp_freq_khz + bands[radio->band].rangelow;

        if (f2 > f1)
            break;

		if (radio->g_search_mode == SCAN) {
			FMDBG("val bit set, posting SILABS_EVT_TUNE_SUCC\n");
			si470x_fm_q_event(radio, SILABS_EVT_TUNE_SUCC);
		}

		if (bltf) {
			FMDBG("bltf bit is set\n");
			break;
		}
		/*
		 * If scan is cancelled or FM is not ON, break ASAP so that we
		 * don't need to sleep for dwell time.
		 */
		if (radio->is_search_cancelled == true) {
			FMDBG("%s: scan cancelled\n", __func__);
			if (radio->g_search_mode == SCAN_FOR_STRONG)
				goto seek_tune_fail;
			else
				goto seek_cancelled;
		} else if (radio->mode != FM_RECV) {
			FMDERR("%s: FM is not in proper state\n", __func__);
			return;
		}

		if (radio->g_search_mode == SCAN) {
			/* sleep for dwell period */
			msleep(radio->dwell_time_sec * 1000);
			/* need to queue the event when the seek completes */
			si470x_fm_q_event(radio, SILABS_EVT_SCAN_NEXT);
		} else if (radio->g_search_mode == SCAN_FOR_STRONG) {
			update_search_list(radio, temp_freq_khz);
		}
	}

seek_tune_fail:
	if (radio->g_search_mode == SCAN_FOR_STRONG) {
		len = radio->srch_list.num_stations_found * 2 +
			sizeof(radio->srch_list.num_stations_found);
		data_b = &radio->data_buf[SILABS_FM_BUF_SRCH_LIST];
		kfifo_in_locked(data_b, &radio->srch_list, len,
                        &radio->buf_lock[SILABS_FM_BUF_SRCH_LIST]);
		si470x_fm_q_event(radio, SILABS_EVT_NEW_SRCH_LIST);
	}
	/* tune to original frequency */
	retval = si470x_set_freq(radio, current_freq_khz);
	if (retval < 0)
		FMDERR("%s: Tune to orig freq failed with error %d\n",
               __func__, retval);
	else {
		if (!wait_for_completion_timeout(&radio->completion,
                                         msecs_to_jiffies(tune_timeout)))
			FMDERR("%s: didn't receive STC for tune\n", __func__);
		else
			FMDBG("%s: received STC for tune\n", __func__);
	}
seek_cancelled:
	si470x_fm_q_event(radio, SILABS_EVT_SEEK_COMPLETE);
	radio->seek_tune_status = NO_SEEK_TUNE_PENDING;

    set_hard_mute(radio, false);
}

/**************************************************************************
 * File Operations Interface
 **************************************************************************/

/*
 * si470x_fops_read - read RDS data
 */
static ssize_t si470x_fops_read(struct file *file, char __user *buf,
                                size_t count, loff_t *ppos)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;
	unsigned int block_count = 0;

	/* switch on rds reception */
	if ((radio->registers[SYSCONFIG1] & SYSCONFIG1_RDS) == 0)
		si470x_rds_on(radio);

	/* block if no new data available */
	while (radio->wr_index == radio->rd_index) {
		if (file->f_flags & O_NONBLOCK) {
			retval = -EWOULDBLOCK;
			goto done;
		}
		if (wait_event_interruptible(radio->read_queue,
                                     radio->wr_index != radio->rd_index) < 0) {
			retval = -EINTR;
			goto done;
		}
	}

	/* calculate block count from byte count */
	count /= 3;

	/* copy RDS block out of internal buffer and to user buffer */
	while (block_count < count) {
		if (radio->rd_index == radio->wr_index)
			break;

		/* always transfer rds complete blocks */
		if (copy_to_user(buf, &radio->buffer[radio->rd_index], 3))
			/* retval = -EFAULT; */
			break;

		/* increment and wrap read pointer */
		radio->rd_index += 3;
		if (radio->rd_index >= radio->buf_size)
			radio->rd_index = 0;

		/* increment counters */
		block_count++;
		buf += 3;
		retval += 3;
	}

done:
	return retval;
}


/*
 * si470x_fops_poll - poll RDS data
 */
static unsigned int si470x_fops_poll(struct file *file,
                                     struct poll_table_struct *pts)
{
	struct si470x_device *radio = video_drvdata(file);
	unsigned long req_events = poll_requested_events(pts);
	int retval = v4l2_ctrl_poll(file, pts);

	if (req_events & (POLLIN | POLLRDNORM)) {
		/* switch on rds reception */
		if ((radio->registers[SYSCONFIG1] & SYSCONFIG1_RDS) == 0)
			si470x_rds_on(radio);

		poll_wait(file, &radio->read_queue, pts);

		if (radio->rd_index != radio->wr_index)
			retval |= POLLIN | POLLRDNORM;
	}

	return retval;
}


/*
 * si470x_fops - file operations interface
 */
static const struct v4l2_file_operations si470x_fops = {
	.owner			= THIS_MODULE,
	.read			= si470x_fops_read,
	.poll			= si470x_fops_poll,
	.unlocked_ioctl		= video_ioctl2,
	.open			= si470x_fops_open,
	.release		= si470x_fops_release,
};



/**************************************************************************
 * Video4Linux Interface
 **************************************************************************/

static struct v4l2_queryctrl si470x_v4l2_queryctrl[] = {
	{
		.id	       = V4L2_CID_AUDIO_VOLUME,
		.type	       = V4L2_CTRL_TYPE_INTEGER,
		.name	       = "Volume",
		.minimum       = 0,
		.maximum       = 15,
		.step	       = 1,
		.default_value = 15,
	},
	{
		.id	       = V4L2_CID_AUDIO_BALANCE,
		.flags	       = V4L2_CTRL_FLAG_DISABLED,
	},
	{
		.id	       = V4L2_CID_AUDIO_BASS,
		.flags	       = V4L2_CTRL_FLAG_DISABLED,
	},
	{
		.id	       = V4L2_CID_AUDIO_TREBLE,
		.flags	       = V4L2_CTRL_FLAG_DISABLED,
	},
	{
		.id	       = V4L2_CID_AUDIO_MUTE,
		.type	       = V4L2_CTRL_TYPE_BOOLEAN,
		.name	       = "Mute",
		.minimum       = 0,
		.maximum       = 1,
		.step	       = 1,
		.default_value = 1,
	},
	{
		.id	       = V4L2_CID_AUDIO_LOUDNESS,
		.flags	       = V4L2_CTRL_FLAG_DISABLED,
	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_SRCHON,
		.type          = V4L2_CTRL_TYPE_BOOLEAN,
		.name          = "Search on/off",
		.minimum       = 0,
		.maximum       = 1,
		.step          = 1,
		.default_value = 1,

	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_STATE,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "radio 0ff/rx/tx/reset",
		.minimum       = 0,
		.maximum       = 3,
		.step          = 1,
		.default_value = 1,

	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_REGION,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "radio standard",
		.minimum       = 0,
		.maximum       = 2,
		.step          = 1,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_SIGNAL_TH,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Signal Threshold",
		.minimum       = 0x80,
		.maximum       = 0x7F,
		.step          = 1,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_EMPHASIS,
		.type          = V4L2_CTRL_TYPE_BOOLEAN,
		.name          = "Emphasis",
		.minimum       = 0,
		.maximum       = 1,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_RDS_STD,
		.type          = V4L2_CTRL_TYPE_BOOLEAN,
		.name          = "RDS standard",
		.minimum       = 0,
		.maximum       = 1,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_SPACING,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Channel spacing",
		.minimum       = 0,
		.maximum       = 2,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_RDSON,
		.type          = V4L2_CTRL_TYPE_BOOLEAN,
		.name          = "RDS on/off",
		.minimum       = 0,
		.maximum       = 1,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_RDSGROUP_MASK,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "RDS group mask",
		.minimum       = 0,
		.maximum       = 0xFFFFFFFF,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_RDSGROUP_PROC,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "RDS processing",
		.minimum       = 0,
		.maximum       = 0xFF,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_RDSD_BUF,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "RDS data groups to buffer",
		.minimum       = 1,
		.maximum       = 21,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_PSALL,
		.type          = V4L2_CTRL_TYPE_BOOLEAN,
		.name          = "pass all ps strings",
		.minimum       = 0,
		.maximum       = 1,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_LP_MODE,
		.type          = V4L2_CTRL_TYPE_BOOLEAN,
		.name          = "Low power mode",
		.minimum       = 0,
		.maximum       = 1,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_SILABS_ANTENNA,
		.type          = V4L2_CTRL_TYPE_BOOLEAN,
		.name          = "headset/internal",
		.minimum       = 0,
		.maximum       = 1,
		.default_value = 0,
	},

};

static int si470x_vidioc_queryctrl(struct file *file, void *priv,
		struct v4l2_queryctrl *qc)
{
	unsigned char i;
	int retval = -EINVAL;

	if (unlikely(qc == NULL)) {
		FMDERR("%s:qc is null", __func__);
		return -EINVAL;
	}


	for (i = 0; i < ARRAY_SIZE(si470x_v4l2_queryctrl); i++) {
		if (qc->id && qc->id == si470x_v4l2_queryctrl[i].id) {
			memcpy(qc, &(si470x_v4l2_queryctrl[i]),
				       sizeof(*qc));
			retval = 0;
			break;
		}
	}
	if (retval < 0)
		FMDERR("query conv4ltrol failed with %d\n", retval);

	return retval;
}

/*
 * si470x_vidioc_g_tuner - get tuner attributes
 */
static int si470x_vidioc_g_tuner(struct file *file, void *priv,
                                 struct v4l2_tuner *tuner)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = 0;

	if (tuner->index != 0)
		return -EINVAL;

	if (!radio->status_rssi_auto_update) {
		retval = si470x_get_register(radio, STATUSRSSI);
		if (retval < 0)
			return retval;
	}

	/* driver constants */
	strcpy(tuner->name, "FM");
	tuner->type = V4L2_TUNER_RADIO;
	tuner->capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO |
        V4L2_TUNER_CAP_RDS | V4L2_TUNER_CAP_RDS_BLOCK_IO |
        V4L2_TUNER_CAP_HWSEEK_BOUNDED |
        V4L2_TUNER_CAP_HWSEEK_WRAP;
	tuner->rangelow  =
		radio->recv_conf.band_low_limit * TUNE_STEP_SIZE * TUNE_PARAM;
	tuner->rangehigh =
		radio->recv_conf.band_high_limit * TUNE_STEP_SIZE * TUNE_PARAM;

	/* stereo indicator == stereo (instead of mono) */
	if ((radio->registers[STATUSRSSI] & STATUSRSSI_ST) == 0)
		tuner->rxsubchans = V4L2_TUNER_SUB_MONO;
	else
		tuner->rxsubchans = V4L2_TUNER_SUB_STEREO;
	/* If there is a reliable method of detecting an RDS channel,
	   then this code should check for that before setting this
	   RDS subchannel. */
	tuner->rxsubchans |= V4L2_TUNER_SUB_RDS;

	/* mono/stereo selector */
	if ((radio->registers[POWERCFG] & POWERCFG_MONO) == 0)
		tuner->audmode = V4L2_TUNER_MODE_STEREO;
	else
		tuner->audmode = V4L2_TUNER_MODE_MONO;

	/* min is worst, max is best; signal:0..0xffff; rssi: 0..0xff */
	/* measured in units of dbµV in 1 db increments (max at ~75 dbµV) */
	tuner->signal = (radio->registers[STATUSRSSI] & STATUSRSSI_RSSI);

	/* automatic frequency control: -1: freq to low, 1 freq to high */
	/* AFCRL does only indicate that freq. differs, not if too low/high */
	tuner->afc = (radio->registers[STATUSRSSI] & STATUSRSSI_AFCRL) ? 1 : 0;

	return retval;
}


/*
 * si470x_vidioc_s_tuner - set tuner attributes
 */
static int si470x_vidioc_s_tuner(struct file *file, void *priv,
                                 const struct v4l2_tuner *tuner)
{
	struct si470x_device *radio = video_drvdata(file);
	u16 top_band = 0, bottom_band = 0;

	if (unlikely(radio == NULL)) {
		FMDERR("%s:radio is null", __func__);
		return -EINVAL;
	}

	if (unlikely(tuner == NULL)) {
		FMDERR("%s:tuner is null", __func__);
		return -EINVAL;
	}

	if (tuner->index > 0)
		return -EINVAL;

	FMDERR("In %s, setting top and bottom band limits\n", __func__);

	bottom_band = (u16)((tuner->rangelow / TUNE_PARAM) / TUNE_STEP_SIZE);
	FMDERR("In %s, tuner->rangelow is %d, setting bottom band to %d\n",
		__func__, tuner->rangelow, bottom_band);
    radio->recv_conf.band_low_limit = bottom_band;

	top_band = (u16)((tuner->rangehigh / TUNE_PARAM) / TUNE_STEP_SIZE);
	FMDERR("In %s, tuner->rangehigh is %d, setting top band to %d\n",
		__func__, tuner->rangehigh, top_band);
    radio->recv_conf.band_high_limit = top_band;

    if (bottom_band == 8750 && top_band == 10800)
        si470x_set_band(radio, 0);
    else if (bottom_band == 7600 && top_band == 10800)
        si470x_set_band(radio, 1);
    else if (bottom_band == 7600 && top_band == 9500)
        si470x_set_band(radio, 1);

	/* mono/stereo selector */
	switch (tuner->audmode) {
	case V4L2_TUNER_MODE_MONO:
		radio->registers[POWERCFG] |= POWERCFG_MONO;  /* force mono */
		break;
	case V4L2_TUNER_MODE_STEREO:
	default:
		radio->registers[POWERCFG] &= ~POWERCFG_MONO; /* try stereo */
		break;
	}

	return si470x_set_register(radio, POWERCFG);
}


/*
 * si470x_vidioc_g_frequency - get tuner or modulator radio frequency
 */
static int si470x_vidioc_g_frequency(struct file *file, void *priv,
                                     struct v4l2_frequency *freq)
{
	struct si470x_device *radio = video_drvdata(file);

	if (unlikely(radio == NULL)) {
		FMDERR(":radio is null");
		return -EINVAL;
	}

	if (freq == NULL) {
		FMDERR("%s, v4l2 freq is null\n", __func__);
		return -EINVAL;
	}

	freq->type = V4L2_TUNER_RADIO;
	return si470x_get_freq(radio, &freq->frequency);
}

/*
 * si470x_vidioc_s_frequency - set tuner or modulator radio frequency
 */
static int si470x_vidioc_s_frequency(struct file *file, void *priv,
                                     const struct v4l2_frequency *freq)
{
	struct si470x_device *radio = video_drvdata(file);
	int retval = -1;
	u32 f = 0;

	if (unlikely(radio == NULL)) {
		FMDERR("%s:radio is null", __func__);
		return -EINVAL;
	}

	if (unlikely(freq == NULL)) {
		FMDERR("%s:freq is null", __func__);
		return -EINVAL;
	}

	if (freq->type != V4L2_TUNER_RADIO)
		return -EINVAL;

	f = (freq->frequency)/TUNE_PARAM;

	FMDBG("Calling tune with freq %u\n", f);

	radio->seek_tune_status = TUNE_PENDING;

    retval = si470x_set_freq(radio, freq->frequency);    

	/* save the current frequency if tune is successful. */
	if (retval > 0) {
		radio->tuned_freq_khz = f;

    }

	/* if (freq->frequency < bands[radio->band].rangelow || */
	/*     freq->frequency > bands[radio->band].rangehigh) { */
	/* 	/\* Switch to band 1 which covers everything we support *\/ */
	/* 	retval = si470x_set_band(radio, 1); */
	/* 	if (retval) */
	/* 		return retval; */
	/* } */

	return retval;
}


/*
 * si470x_vidioc_s_hw_freq_seek - set hardware frequency seek
 */
static int si470x_vidioc_s_hw_freq_seek(struct file *file, void *priv,
                                        const struct v4l2_hw_freq_seek *seek)
{
	struct si470x_device *radio = video_drvdata(file);
    int dir;
    int retval = 0;

	if (unlikely(radio == NULL)) {
		FMDERR("%s:radio is null", __func__);
		return -EINVAL;
	}

	if (unlikely(seek == NULL)) {
		FMDERR("%s:seek is null", __func__);
		return -EINVAL;
	}

	if (seek->seek_upward)
		dir = SRCH_DIR_UP;
	else
		dir = SRCH_DIR_DOWN;

	radio->is_search_cancelled = false;
    
	if (radio->g_search_mode == SEEK) {
		/* seek */
		FMDERR("starting seek\n");

		radio->seek_tune_status = SEEK_PENDING;

		retval = si470x_seek(radio, dir, WRAP_DISABLE);
	} else if ((radio->g_search_mode == SCAN) ||
               (radio->g_search_mode == SCAN_FOR_STRONG)) {
		/* scan */
		if (radio->g_search_mode == SCAN_FOR_STRONG) {
			FMDBG("starting search list\n");
			memset(&radio->srch_list, 0,
                   sizeof(struct silabs_srch_list_compl));
		} else {
			FMDBG("starting scan\n");
		}
		si470x_search(radio, START_SCAN);

	} else {
		retval = -EINVAL;
		FMDERR("In %s, invalid search mode %d\n",
               __func__, radio->g_search_mode);
	}

    return retval;
}

/*
 * si470x_vidioc_enum_freq_bands - enumerate supported bands
 */
static int si470x_vidioc_enum_freq_bands(struct file *file, void *priv,
                                         struct v4l2_frequency_band *band)
{
	if (band->tuner != 0)
		return -EINVAL;
	if (band->index >= ARRAY_SIZE(bands))
		return -EINVAL;
	*band = bands[band->index];
	return 0;
}

static int si470x_vidioc_dqbuf(struct file *file, void *priv,
                               struct v4l2_buffer *buffer)
{
    struct si470x_device *radio = video_get_drvdata(video_devdata(file));
    enum silabs_buf_t buf_type = -1;
    u8 buf_fifo[STD_BUF_SIZE] = {0};
    struct kfifo *data_fifo = NULL;
    u8 *buf = NULL;
    int len = 0, retval = -1;

    if ((radio == NULL) || (buffer == NULL)) {
        FMDERR("radio/buffer is NULL\n");
        return -ENXIO;
    }
    buf_type = buffer->index;
    buf = (u8 *)buffer->m.userptr;
    len = buffer->length;
    FMDBG("%s: requesting buffer %d\n", __func__, buf_type);

    if ((buf_type < SILABS_FM_BUF_MAX) && (buf_type >= 0)) {
        data_fifo = &radio->data_buf[buf_type];
        if (buf_type == SILABS_FM_BUF_EVENTS) {
            if (wait_event_interruptible(radio->event_queue,
                                         kfifo_len(data_fifo)) < 0) {
                return -EINTR;
            }
        }
    } else {
        FMDERR("invalid buffer type\n");
        return -EINVAL;
    }
    if (len <= STD_BUF_SIZE) {
        buffer->bytesused = kfifo_out_locked(data_fifo, &buf_fifo[0],
                                             len, &radio->buf_lock[buf_type]);
    } else {
        FMDERR("kfifo_out_locked can not use len more than 128\n");
        return -EINVAL;
    }

    retval = copy_to_user(buf, &buf_fifo[0], buffer->bytesused);
    if (retval > 0) {
        FMDERR("Failed to copy %d bytes of data\n", retval);
        return -EAGAIN;
    }
    
    return retval;
}


static int si470x_vidioc_g_ctrl(struct file *file, void *priv,
                                struct v4l2_control *ctrl)
{

    struct si470x_device *radio = video_get_drvdata(video_devdata(file));
    int retval = 0;

	pr_err("%s....\n", __func__);

    if (unlikely(radio == NULL)) {
        FMDERR(":radio is null");
        return -EINVAL;
    }

    if (ctrl == NULL) {
        FMDERR("%s, v4l2 ctrl is null\n", __func__);
        return -EINVAL;
    }

    switch (ctrl->id) {
    case V4L2_CID_AUDIO_VOLUME:
        break;
    case V4L2_CID_AUDIO_MUTE:
        break;

    case V4L2_CID_PRIVATE_SILABS_RDSGROUP_PROC:
        ctrl->value = 0;
        retval = 0;
        break;

    default:
        pr_err("%s: ctrl->id = 0x%x, ctrl->value = 0x%x\n", __func__, ctrl->id, ctrl->value);
        retval = -EINVAL;
        break;
    }

    if (retval < 0)
        FMDERR("get control failed with %d, id: %x\n",
               retval, ctrl->id);

    return retval;
}

static int si470x_vidioc_s_ctrl(struct file *file, void *priv,
                                struct v4l2_control *ctrl)
{
    struct si470x_device *radio = video_get_drvdata(video_devdata(file));
    int retval = 0;

    if (unlikely(radio == NULL)) {
        FMDERR("%s:radio is null", __func__);
        return -EINVAL;
    }

    if (unlikely(ctrl == NULL)) {
        FMDERR("%s:ctrl is null", __func__);
        return -EINVAL;
    }

    pr_err("%s: ctrl->id = 0x%x, ctrl->value = 0x%x\n", __func__, ctrl->id, ctrl->value);
    

    switch (ctrl->id) {
    case V4L2_CID_PRIVATE_SILABS_STATE:
        /* check if already on */
        if (ctrl->value == FM_RECV) {
            if (is_enable_rx_possible(radio) != 0) {
                FMDERR("%s: fm is not in proper state\n",
                       __func__);
                retval = -EINVAL;
                goto end;
            }
            radio->mode = FM_RECV_TURNING_ON;
            retval = si470x_start(radio);
            if (retval < 0) {
                FMDERR("Error while enabling RECV FM %d\n",
                       retval);
                goto end;
            }
        } else if (ctrl->value == FM_OFF) {
            flush_workqueue(radio->wqueue);
            cancel_work_sync(&radio->rds_worker);
            flush_workqueue(radio->wqueue_rds);
            radio->mode = FM_TURNING_OFF;
            retval = si470x_stop(radio);
            if (retval < 0) {
                FMDERR("Error while enabling RECV FM %d\n",
                       retval);
                goto end;
            }
        }

        break;

    case V4L2_CID_PRIVATE_SILABS_SPACING:
        retval = si470x_set_spacing(radio, ctrl->value);
        retval = 0;
        if (retval < 0) {
            FMDERR("Error in setting channel spacing\n");
            goto end;
        }
        break;

    case V4L2_CID_PRIVATE_SILABS_EMPHASIS:
        retval = si470x_set_emphasis(radio, ctrl->value);
        retval = 0;
        if (retval < 0) {
            FMDERR("Error in setting emphasis\n");
            goto end;
        }
        break;

    case V4L2_CID_PRIVATE_SILABS_SRCHMODE:
        if (is_valid_srch_mode(ctrl->value)) {
            radio->g_search_mode = ctrl->value;
        } else {
            FMDERR("%s: srch mode is not valid\n", __func__);
            retval = -EINVAL;
            goto end;
        }
        break;

	case V4L2_CID_PRIVATE_SILABS_SCANDWELL:
        if ((ctrl->value >= MIN_DWELL_TIME) &&
            (ctrl->value <= MAX_DWELL_TIME)) {
            radio->dwell_time_sec = ctrl->value;
        } else {
            FMDERR("%s: scandwell period is not valid\n", __func__);
            retval = -EINVAL;
        }
        break;

    case V4L2_CID_PRIVATE_SILABS_SRCHON:
		si470x_search(radio, (bool)ctrl->value);
        break;

    case V4L2_CID_PRIVATE_SILABS_ANTENNA:
    case V4L2_CID_PRIVATE_SILABS_SOFT_MUTE:
    case V4L2_CID_PRIVATE_SILABS_REGION:
    case V4L2_CID_PRIVATE_SILABS_SRCH_ALGORITHM:
    case V4L2_CID_PRIVATE_SILABS_SET_AUDIO_PATH:
    case V4L2_CID_PRIVATE_SILABS_SRCH_CNT:
    case V4L2_CID_PRIVATE_SILABS_RDS_STD:
    case V4L2_CID_PRIVATE_SILABS_RDSON:
        /*
         * These private controls are place holders to keep the
         * driver compatible with changes done in the frameworks
         * which are specific to TAVARUA.
         */
        retval = 0;
        break;

    case V4L2_CID_PRIVATE_SILABS_AF_RMSSI_SAMPLES:
    case V4L2_CID_PRIVATE_SILABS_AF_JUMP_RSSI_TH:
    case V4L2_CID_PRIVATE_SILABS_RSSI_TH:
    case V4L2_CID_PRIVATE_SILABS_SINR_THRESHOLD:
    case V4L2_CID_PRIVATE_SILABS_RDSD_BUF:
    case V4L2_CID_PRIVATE_SILABS_RDSGROUP_MASK:
    case V4L2_CID_PRIVATE_SILABS_RDSGROUP_PROC:
    case V4L2_CID_PRIVATE_SILABS_LP_MODE:
    case V4L2_CID_PRIVATE_SILABS_PSALL:
    case V4L2_CID_PRIVATE_SILABS_AF_JUMP:
        retval = 0;
        break;

    case V4L2_CID_AUDIO_MUTE:
		if (ctrl->value)
			radio->registers[POWERCFG] &= ~POWERCFG_DMUTE;
		else
			radio->registers[POWERCFG] |= POWERCFG_DMUTE;
		return si470x_set_register(radio, POWERCFG);

    default:
        retval = -EINVAL;
        break;
    }

end:
    return retval;
}

static int si470x_vidioc_g_fmt_type_private(struct file *file, void *priv,
                                            struct v4l2_format *f)
{
	return 0;

}

/*
 * si470x_ioctl_ops - video device ioctl operations
 */
static const struct v4l2_ioctl_ops si470x_ioctl_ops = {
	.vidioc_querycap	= si470x_vidioc_querycap,
	.vidioc_queryctrl   = si470x_vidioc_queryctrl,
	.vidioc_g_tuner		= si470x_vidioc_g_tuner,
	.vidioc_s_tuner		= si470x_vidioc_s_tuner,
	.vidioc_g_frequency	= si470x_vidioc_g_frequency,
	.vidioc_s_frequency	= si470x_vidioc_s_frequency,
	.vidioc_g_ctrl          = si470x_vidioc_g_ctrl,
	.vidioc_s_ctrl          = si470x_vidioc_s_ctrl,
	.vidioc_s_hw_freq_seek	= si470x_vidioc_s_hw_freq_seek,
	.vidioc_enum_freq_bands = si470x_vidioc_enum_freq_bands,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_dqbuf           = si470x_vidioc_dqbuf,
	.vidioc_g_fmt_type_private    = si470x_vidioc_g_fmt_type_private,
};


/*
 * si470x_viddev_template - video device interface
 */
struct video_device si470x_viddev_template = {
	.fops			= &si470x_fops,
	.name			= DRIVER_NAME,
	.release		= video_device_release_empty,
	.ioctl_ops		= &si470x_ioctl_ops,
};
