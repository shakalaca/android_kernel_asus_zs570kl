#include <vfsSpiDrv.h>

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <asm/uaccess.h>


#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/input.h>
#include <linux/fb.h>
#include <linux/notifier.h>

#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/timer.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/compat.h>

#include <asm-generic/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/jiffies.h>

#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/qseecomi.h>

#include <linux/poll.h>

#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include <linux/completion.h>

#define VFS_SENSOR_ORIENTATION_URDL   1 /* This is the usual configuation when the sensor is mounted Frontside */
#define VFS_SENSOR_ORIENTATION_LURD   2
#define VFS_SENSOR_ORIENTATION_DLUR   3
#define VFS_SENSOR_ORIENTATION_RDLU   4
#define VFS_SENSOR_ORIENTATION_ULDR   5  /* This is the usual configuation when the sensor is mounted backside */
#define VFS_SENSOR_ORIENTATION_LDRU   6
#define VFS_SENSOR_ORIENTATION_DRUL   7
#define VFS_SENSOR_ORIENTATION_RULD   8

#define VALIDITY_PART_NAME "validity_fingerprint"
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_mutex);
static struct class *vfsspi_device_class;
static int gpio_irq;
static int global_counter=0;

struct vfsspi_device_data {
	dev_t devt;
	struct cdev cdev;
	spinlock_t vfs_spi_lock;
	struct spi_device *spi;
	struct list_head device_entry;
	struct mutex buffer_mutex;
	unsigned int is_opened;
	unsigned char *buffer;
	unsigned char *null_buffer;
	unsigned char *stream_buffer;
	size_t stream_buffer_size;
	unsigned int drdy_pin;
	unsigned int sleep_pin;
	unsigned int isr_pin;
#if DO_CHIP_SELECT
	unsigned int cs_pin;
#endif
	int user_pid;
	int signal_id;
	int eUserPID;
	int eSignalID;
	struct notifier_block fb_notifier;
	unsigned int current_spi_speed;
	unsigned int is_drdy_irq_enabled;
	unsigned int drdy_ntf_type;
	struct mutex kernel_lock;
	struct wake_lock wake_lock;
	struct wake_lock wake_lock_irq;
	bool irq_wakeup_flag;
	struct delayed_work fp_sensor_work;
};
struct vfsspi_device_data *fp_device;
struct vfsspi_device_data *vfsSpiDevTmp = NULL;
static struct workqueue_struct 	*fp_sensor_wq;
#ifdef VFSSPI_32BIT
/*
 * Used by IOCTL compat command:
 *         VFSSPI_IOCTL_RW_SPI_MESSAGE
 *
 * @rx_buffer:pointer to retrieved data
 * @tx_buffer:pointer to transmitted data
 * @len:transmitted/retrieved data size
 */
struct vfsspi_compat_ioctl_transfer {
	compat_uptr_t rx_buffer;
	compat_uptr_t tx_buffer;
	unsigned int len;
};
#endif

//for screen status +++
int screen_status = 3; //0= off, 1=on, 3=unknown

static ssize_t screen_status_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        //int pos_ind= 0;
	int ret;
        printk("[FP] screen status = %d\n", screen_status);
	ret = sprintf(buf, "%d\n", screen_status);
        //return screen_status;
	return ret;
}

DEVICE_ATTR(screenState, (S_IWUSR|S_IRUGO), screen_status_show, NULL);
//for screen status  ---

static int vfsspi_sendDrdyEventFd(struct vfsspi_device_data *vfsSpiDev);
static int vfsspi_sendDrdyNotify(struct vfsspi_device_data *vfsSpiDev);


static int vfsspi_send_drdy_signal(struct vfsspi_device_data *vfsspi_device)
{
	struct task_struct *t;
	int ret = 0;

	printk("vfsspi_send_drdy_signal\n");

	if (vfsspi_device!=NULL && vfsspi_device->user_pid != 0) {
		rcu_read_lock();
		/* find the task_struct associated with userpid */
		printk("Searching task with PID=%08x\n",
			vfsspi_device->user_pid);
		t = pid_task(find_pid_ns(vfsspi_device->user_pid, &init_pid_ns),
			     PIDTYPE_PID);
		if (t == NULL) {
			printk("No such pid\n");
			rcu_read_unlock();
			return -ENODEV;
		}
		rcu_read_unlock();
		/* notify DRDY signal to user process */
		ret = send_sig_info(vfsspi_device->signal_id,
				    (struct siginfo *)1, t);
		if (ret < 0)
			printk("Error sending signal\n");

	} else {
		printk("pid not received yet\n");
	}

	return ret;
}

void vfsspi_screen_notify(void)
{
	struct task_struct *t;
#if 0
	struct file *efd_file = NULL;
	struct eventfd_ctx *efd_ctx = NULL;
#endif
	int ret = 0;

	printk("vfsspi screen status change to %d\n",screen_status);
#if 1 //for signal
	if(vfsSpiDevTmp!=NULL){
		if (vfsSpiDevTmp->eUserPID != 0) {
			rcu_read_lock();
			/* find the task_struct associated with userpid */
			printk("Searching task with PID=%08x\n", vfsSpiDevTmp->eUserPID);
			t = pid_task(find_pid_ns(vfsSpiDevTmp->eUserPID, &init_pid_ns),
				     PIDTYPE_PID);
			if (t == NULL) {
				printk("No such pid\n");
				rcu_read_unlock();
				return;
			}
			rcu_read_unlock();
			/* notify screen signal to user process */
			ret =
			    send_sig_info(vfsSpiDevTmp->eSignalID, (struct siginfo *)1,
					  t);
			if (ret < 0)
				printk("Error sending screen off signal\n");
			else
				printk("pid not received yet\n");
		}
	}
#else //for event
	if(vfsSpiDevTmp!=NULL){
	    if (vfsSpiDevTmp->eUserPID != 0) {
	        rcu_read_lock();
	        /* find the task_struct associated with userpid */
	        printk("Searching task with PID=%08x\n", vfsSpiDevTmp->eUserPID);
	        t = pid_task(find_pid_ns(vfsSpiDevTmp->eUserPID, &init_pid_ns),
	            PIDTYPE_PID);
	        if (t == NULL) {
	            printk("No such pid\n");
	            rcu_read_unlock();
	            return -ENODEV;
	        }
	        efd_file = fcheck_files(t->files, vfsSpiDevTmp->eSignalID);
	        rcu_read_unlock();

	        if (efd_file == NULL) {
	            printk("No such efd_file\n");
	            return -ENODEV;
	        }
	        
	        efd_ctx = eventfd_ctx_fileget(efd_file);
	        if (efd_ctx == NULL) {
	            printk("eventfd_ctx_fileget is failed\n");
	            return -ENODEV;
	        }

	        /* notify DRDY eventfd to user process */
	        eventfd_signal(efd_ctx, 1);

	        /* Release eventfd context */
	        eventfd_ctx_put(efd_ctx);
	    }
	}
#endif
}

#if 0 //disable for merge to one function
void vfsspi_screen_on(void)
{
	struct task_struct *t;
	int ret = 0;

	printk("vfsspi screen on\n");

#if 1 //for signal
	if(vfsSpiDevTmp!=NULL){
		if (vfsSpiDevTmp->eUserPID != 0) {
			rcu_read_lock();
			/* find the task_struct associated with userpid */
			printk("Searching task with PID=%08x\n", vfsSpiDevTmp->eUserPID);
			t = pid_task(find_pid_ns(vfsSpiDevTmp->eUserPID, &init_pid_ns),
				     PIDTYPE_PID);
			if (t == NULL) {
				printk("No such pid\n");
				rcu_read_unlock();
				return;
			}
			rcu_read_unlock();
			/* notify screen signal to user process */
			ret =
			    send_sig_info(vfsSpiDevTmp->eSignalID, (struct siginfo *)1,
					  t);
			if (ret < 0)
				printk("Error sending screen on signal\n");
			else
				printk("pid not received yet\n");
		}
	}
#else //for event
	if(vfsSpiDevTmp!=NULL){
		if (vfsSpiDevTmp->eUserPID != 0) {
			rcu_read_lock();
			/* find the task_struct associated with userpid */
			printk("Searching task with PID=%08x\n", vfsSpiDevTmp->eUserPID);
			t = pid_task(find_pid_ns(vfsSpiDevTmp->eUserPID, &init_pid_ns),
				PIDTYPE_PID);
			if (t == NULL) {
				printk("No such pid\n");
				rcu_read_unlock();
				return -ENODEV;
			}
			efd_file = fcheck_files(t->files, vfsSpiDevTmp->eSignalID);
			rcu_read_unlock();
	
			if (efd_file == NULL) {
				printk("No such efd_file\n");
				return -ENODEV;
			}
			
			efd_ctx = eventfd_ctx_fileget(efd_file);
			if (efd_ctx == NULL) {
				printk("eventfd_ctx_fileget is failed\n");
				return -ENODEV;
			}
	
			/* notify DRDY eventfd to user process */
			eventfd_signal(efd_ctx, 1);
	
			/* Release eventfd context */
			eventfd_ctx_put(efd_ctx);
		}
	}
#endif
}
#endif


static void fp_sensor_report_function(struct work_struct *dat)
{
	vfsspi_screen_notify();
	global_counter--;
}


// add for fb detection start
static int SYNA_fb_state_notify_callback(struct notifier_block *nb, unsigned long val, void *fbdata)
{
	struct fb_event *fb_data = fbdata;
	unsigned int fb_state;

	if (val != FB_EARLY_EVENT_BLANK)
		return 0;
	
	printk("SYNA_fb_state_notify_callback value = %d\n",(int)val);
	
	if (fb_data && fb_data->data) 
	{
		fb_state = *(int *)(fb_data->data);
		switch (fb_state) {
		case FB_BLANK_NORMAL:
		case FB_BLANK_POWERDOWN:
			screen_status = 0; //set the screen status to off
			printk("SYNA_fb_state_notify_callback FB_BLANK_POWERDOWN - vfsspi_screen_off\n");
			global_counter++;
			queue_delayed_work(fp_sensor_wq, &fp_device->fp_sensor_work, 0);
			break;
		case FB_BLANK_UNBLANK:
			screen_status = 1; //set the screen status to on
			printk("SYNA_fb_state_notify_callback FB_BLANK_UNBLANK - vfsspi_screen_on\n");
			global_counter++;
			queue_delayed_work(fp_sensor_wq, &fp_device->fp_sensor_work, 0);
			break;
		default:
			printk("SYNA_fb_state_notify_callback: defalut\n");
			break;
		}
	}
	return NOTIFY_OK;
}


static struct notifier_block SYNA_notify_block = {	
	.notifier_call = SYNA_fb_state_notify_callback,
};
// add for fb detection end
static int vfsspi_register_drdy_signal(struct vfsspi_device_data *vfsspi_device,
				       unsigned long arg)
{
	struct vfsspi_ioctl_register_signal usr_signal;
	if (copy_from_user(&usr_signal, (void __user *)arg, sizeof(usr_signal)) != 0) {
		printk("Failed copy from user.\n");
		return -EFAULT;
	} else {
		vfsspi_device->user_pid = usr_signal.user_pid;
		vfsspi_device->signal_id = usr_signal.signal_id;
	}
	return 0;
}

static int vfsspi_register_screen_signal(struct vfsspi_device_data *vfsspi_device,
				       unsigned long arg)
{
	struct vfsspi_ioctl_register_signal usrSignal_screen;
	printk("Enter vfsspi_register_screen_signal\n");
	if (copy_from_user(&usrSignal_screen, (void *)arg, sizeof(usrSignal_screen)) != 0) {
		printk("Failed copy from user.\n");
		return -EFAULT;
	} else {
		vfsspi_device->eUserPID = usrSignal_screen.user_pid;
		vfsspi_device->eSignalID = usrSignal_screen.signal_id;
	}
	printk("Exit vfsspi_register_screen_signal\n");
	return 0;
}

static irqreturn_t vfsspi_irq(int irq, void *context)
{
	struct vfsspi_device_data *vfsspi_device = context;
	/* Linux kernel is designed so that when you disable
	an edge-triggered interrupt, and the edge happens while
	the interrupt is disabled, the system will re-play the
	interrupt at enable time.
	Therefore, we are checking DRDY GPIO pin state to make sure
	if the interrupt handler has been called actually by DRDY
	interrupt and it's not a previous interrupt re-play */
	printk("vfsspi_irq\n");
	if (vfsspi_device != NULL && gpio_get_value(vfsspi_device->drdy_pin) == DRDY_ACTIVE_STATUS) {
		printk("vfsspi_wake_lock\n");
		wake_lock_timeout(&vfsspi_device->wake_lock, 150); //1.5s
		wake_lock_timeout(&vfsspi_device->wake_lock_irq, 150);
		vfsspi_sendDrdyNotify(vfsspi_device);
	}

	return IRQ_HANDLED;
}

static int vfsspi_sendDrdyEventFd(struct vfsspi_device_data *vfsSpiDev)
{
    struct task_struct *t;
    struct file *efd_file = NULL;
    struct eventfd_ctx *efd_ctx = NULL;	int ret = 0;

    printk("vfsspi_sendDrdyEventFd\n");

    if (vfsSpiDev!=NULL && vfsSpiDev->user_pid != 0) {
        rcu_read_lock();
        /* find the task_struct associated with userpid */
        printk("Searching task with PID=%08x\n", vfsSpiDev->user_pid);
        t = pid_task(find_pid_ns(vfsSpiDev->user_pid, &init_pid_ns),
            PIDTYPE_PID);
        if (t == NULL) {
            printk("No such pid\n");
            rcu_read_unlock();
            return -ENODEV;
        }
        efd_file = fcheck_files(t->files, vfsSpiDev->signal_id);
        rcu_read_unlock();

        if (efd_file == NULL) {
            printk("No such efd_file\n");
            return -ENODEV;
        }
        
        efd_ctx = eventfd_ctx_fileget(efd_file);
        if (efd_ctx == NULL) {
            printk("eventfd_ctx_fileget is failed\n");
            return -ENODEV;
        }

        /* notify DRDY eventfd to user process */
        eventfd_signal(efd_ctx, 1);

        /* Release eventfd context */
        eventfd_ctx_put(efd_ctx);
    }

    return ret;
}

static int vfsspi_sendDrdyNotify(struct vfsspi_device_data *vfsSpiDev)
{
    int ret = 0;

if(vfsSpiDev != NULL){
    if (vfsSpiDev->drdy_ntf_type == VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD) {
        ret = vfsspi_sendDrdyEventFd(vfsSpiDev);
    } else {
        ret = vfsspi_send_drdy_signal(vfsSpiDev);
    }
}
    return ret;
}

static int vfsspi_enableIrq(struct vfsspi_device_data *vfsspi_device)
{
	printk("vfsspi_enableIrq\n");
if(vfsspi_device != NULL){
	if (vfsspi_device->is_drdy_irq_enabled == DRDY_IRQ_ENABLE) {
		printk("DRDY irq already enabled\n");
		return -EINVAL;
	}

	enable_irq(gpio_irq);
	vfsspi_device->is_drdy_irq_enabled = DRDY_IRQ_ENABLE;
}
	return 0;
}

static int vfsspi_disableIrq(struct vfsspi_device_data *vfsspi_device)
{
	printk("vfsspi_disableIrq\n");
if(vfsspi_device != NULL){
	if (vfsspi_device->is_drdy_irq_enabled == DRDY_IRQ_DISABLE) {
		printk("DRDY irq already disabled\n");
		return -EINVAL;
	}

	disable_irq_nosync(gpio_irq);
	vfsspi_device->is_drdy_irq_enabled = DRDY_IRQ_DISABLE;
}
	return 0;
}
static int vfsspi_set_drdy_int(struct vfsspi_device_data *vfsspi_device,
			       unsigned long arg)
{
	unsigned short drdy_enable_flag;
	if (copy_from_user(&drdy_enable_flag, (void __user *)arg,
			   sizeof(drdy_enable_flag)) != 0) {
		printk("Failed copy from user.\n");
		return -EFAULT;
	}
	if (drdy_enable_flag == 0)
			vfsspi_disableIrq(vfsspi_device);
	else {
			vfsspi_enableIrq(vfsspi_device);
			/* Workaround the issue where the system
			  misses DRDY notification to host when
			  DRDY pin was asserted before enabling
			  device.*/
			if (gpio_get_value(vfsspi_device->drdy_pin) ==
				DRDY_ACTIVE_STATUS) {
				vfsspi_sendDrdyNotify(vfsspi_device);
			}
	}
	return 0;
}


static void vfsspi_hardReset(struct vfsspi_device_data *vfsspi_device)
{
	printk("vfsspi_hardReset\n");
	if (vfsspi_device != NULL) {
		gpio_set_value(vfsspi_device->sleep_pin, 0);
		mdelay(1);
		gpio_set_value(vfsspi_device->sleep_pin, 1);
		mdelay(5);
	}
}


static void vfsspi_suspend(struct vfsspi_device_data *vfsspi_device)
{
	printk("vfsspi_suspend\n");
	if (vfsspi_device != NULL) {
		spin_lock(&vfsspi_device->vfs_spi_lock);
		gpio_set_value(vfsspi_device->sleep_pin, 0);
		spin_unlock(&vfsspi_device->vfs_spi_lock);
	}
}

static long vfsspi_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	int ret_val = 0;
	struct vfsspi_device_data *vfsspi_device = NULL;

	printk("vfsspi_ioctl\n");

	if (_IOC_TYPE(cmd) != VFSSPI_IOCTL_MAGIC) {
		printk("invalid magic. cmd=0x%X Received=0x%X Expected=0x%X\n",
			cmd, _IOC_TYPE(cmd), VFSSPI_IOCTL_MAGIC);
		return -ENOTTY;
	}

	vfsspi_device = filp->private_data;
	vfsSpiDevTmp = vfsspi_device;
	
	mutex_lock(&vfsspi_device->buffer_mutex);
	switch (cmd) {
	case VFSSPI_IOCTL_DEVICE_RESET:
		printk("VFSSPI_IOCTL_DEVICE_RESET:\n");
		vfsspi_hardReset(vfsspi_device);
		break;
	case VFSSPI_IOCTL_DEVICE_SUSPEND:
	{
		printk("VFSSPI_IOCTL_DEVICE_SUSPEND:\n");
		vfsspi_suspend(vfsspi_device);
		break;
	}
	case VFSSPI_IOCTL_REGISTER_SCREEN_DETECTION_SIGNAL:
	{
		printk("VFSSPI_IOCTL_REGISTER_SCREEN_DETECTION_SIGNAL\n");
		ret_val = vfsspi_register_screen_signal(vfsspi_device, arg);
		break;
	}
	// register for screen on/off end
	
	case VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL:
		printk("VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL\n");
		ret_val = vfsspi_register_drdy_signal(vfsspi_device, arg);
		break;
	case VFSSPI_IOCTL_SET_DRDY_INT:
		printk("VFSSPI_IOCTL_SET_DRDY_INT\n");
		ret_val = vfsspi_set_drdy_int(vfsspi_device, arg);
		break;
	case VFSSPI_IOCTL_SELECT_DRDY_NTF_TYPE:
        {
            vfsspi_iocSelectDrdyNtfType_t drdyTypes;

            printk("VFSSPI_IOCTL_SELECT_DRDY_NTF_TYPE\n");

            if (copy_from_user(&drdyTypes, (void __user *)arg,
                sizeof(vfsspi_iocSelectDrdyNtfType_t)) != 0) {
                    printk("copy from user failed.\n");
                    ret_val = -EFAULT;
            } else {
                if (0 != (drdyTypes.supportedTypes & VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD)) {
                    vfsspi_device->drdy_ntf_type = VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD;
                } else {
                    vfsspi_device->drdy_ntf_type = VFSSPI_DRDY_NOTIFY_TYPE_SIGNAL;
                }
                drdyTypes.selectedType = vfsspi_device->drdy_ntf_type;
                if (copy_to_user((void __user *)arg, &(drdyTypes),
                    sizeof(vfsspi_iocSelectDrdyNtfType_t)) == 0) {
                        ret_val = 0;
                } else {
                    printk("copy to user failed\n");
                }
            }
            break;
        }
	case VFSSPI_IOCTL_GET_SENSOR_ORIENTATION:
	{
		pr_debug("VFSSPI_IOCTL_GET_SENSOR_ORIENTATION\n");
		if ((void __user *)arg != NULL)
		{
			*((unsigned int *) arg) = VFS_SENSOR_ORIENTATION_RULD;
			ret_val = 0;
		}
		else
		{
			pr_debug("VFSSPI_IOCTL_GET_SENSOR_ORIENTATION failed\n");
			*((unsigned int *)arg) = 0;
			ret_val = -EFAULT;
		}
		break;
	}
	case VFSSPI_IOCTL_FREE_SYSTEM_WAKE_LOCK:
	{
		printk("VFSSPI_IOCTL_FREE_SYSTEM_WAKE_LOCK\n");
		wake_unlock(&vfsspi_device->wake_lock);
		break;
	}
	default:
		ret_val = -EFAULT;
		break;
	}
	mutex_unlock(&vfsspi_device->buffer_mutex);
	return ret_val;
}

static int vfsspi_open(struct inode *inode, struct file *filp)
{
	struct vfsspi_device_data *vfsspi_device = NULL;
	int status = -ENXIO;

	printk("vfsspi_open\n");

	mutex_lock(&device_list_mutex);

	list_for_each_entry(vfsspi_device, &device_list, device_entry) {
		if (vfsspi_device->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status == 0) {
		mutex_lock(&vfsspi_device->kernel_lock);
		if (vfsspi_device->is_opened != 0) {
			status = -EBUSY;
			printk("vfsspi_open: is_opened != 0, -EBUSY");
			goto vfsspi_open_out;
		}
		vfsspi_device->user_pid = 0;
		vfsspi_device->drdy_ntf_type = VFSSPI_DRDY_NOTIFY_TYPE_SIGNAL;
		if (vfsspi_device->buffer != NULL) {
			printk("vfsspi_open: buffer != NULL");
			goto vfsspi_open_out;
		}
		vfsspi_device->null_buffer =
			kmalloc(DEFAULT_BUFFER_SIZE, GFP_KERNEL);
		if (vfsspi_device->null_buffer == NULL) {
			status = -ENOMEM;
			printk("vfsspi_open: null_buffer == NULL, -ENOMEM");
			goto vfsspi_open_out;
		}
		vfsspi_device->buffer =
			kmalloc(DEFAULT_BUFFER_SIZE, GFP_KERNEL);
		if (vfsspi_device->buffer == NULL) {
			status = -ENOMEM;
			kfree(vfsspi_device->null_buffer);
			printk("vfsspi_open: buffer == NULL, -ENOMEM");
			goto vfsspi_open_out;
		}
		vfsspi_device->is_opened = 1;
		filp->private_data = vfsspi_device;
		nonseekable_open(inode, filp);

vfsspi_open_out:
		mutex_unlock(&vfsspi_device->kernel_lock);
	}
	mutex_unlock(&device_list_mutex);
	return status;
}


static int vfsspi_release(struct inode *inode, struct file *filp)
{
	struct vfsspi_device_data *vfsspi_device = NULL;
	int                   status     = 0;

	printk("vfsspi_release\n");

	mutex_lock(&device_list_mutex);
	vfsspi_device = filp->private_data;
	filp->private_data = NULL;
	vfsspi_device->is_opened = 0;
	if (vfsspi_device->buffer != NULL) {
		kfree(vfsspi_device->buffer);
		vfsspi_device->buffer = NULL;
	}

	if (vfsspi_device->null_buffer != NULL) {
		kfree(vfsspi_device->null_buffer);
		vfsspi_device->null_buffer = NULL;
	}

	mutex_unlock(&device_list_mutex);
	return status;
}

static const struct file_operations vfsspi_fops = {
	.owner   = THIS_MODULE,
//	.write   = vfsspi_write,
//	.read    = vfsspi_read,
	.unlocked_ioctl   = vfsspi_ioctl,
	.open    = vfsspi_open,
	.release = vfsspi_release,
};

static int fp_sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct vfsspi_device_data *fp = dev_get_drvdata(&pdev->dev);
	struct timeval time_s, time_e;

		printk("[FP] sensor suspend +++\n");
		if(global_counter > 0){
			do_gettimeofday(&time_s);
			flush_workqueue(fp_sensor_wq);//flush all works
			do_gettimeofday(&time_e);
			if(global_counter > 0){ //still have work be scheduled after flush_workqueue.
	 			printk(KERN_ERR "\n[FP] global_counter INCORRECT %d\n",global_counter);
				printk(KERN_ERR "[FP] delay %lu\n", time_e.tv_usec - time_s.tv_usec);
				wake_lock_timeout(&fp->wake_lock_irq, 3);//30ms
				return -EBUSY;
			}else{
				printk(KERN_ERR "\n[FP] delay %lu\n", time_e.tv_usec - time_s.tv_usec);
			}
		}
		if (fp->is_drdy_irq_enabled & !fp->irq_wakeup_flag) {
			printk("[FP] enable irq wake up  \n");
			enable_irq_wake(fp->isr_pin);
			fp->irq_wakeup_flag = true;
		}
		printk("[FP] sensor suspend ---\n");

	return 0;
}

static int fp_sensor_resume(struct platform_device *pdev)
{
	struct vfsspi_device_data *fp = dev_get_drvdata(&pdev->dev);

		printk("[FP] sensor resume +++\n");

		if (fp->irq_wakeup_flag) {
			disable_irq_wake(fp->isr_pin);
			fp->irq_wakeup_flag = false;
		}
		flush_workqueue(fp_sensor_wq);//flush all works
		printk("[FP] sensor resume ---\n");

	return 0;
}

static int fp_gpio_init(struct vfsspi_device_data *pdata)
{

	int err = 0;
	/* request part */
	if (gpio_request(pdata->drdy_pin, "fp_drdy") < 0) {
		err = -EBUSY;
		printk("[FP][%s] gpio request failed ! \n", __func__);
		return err;
	}

	if (gpio_request(pdata->sleep_pin, "fp_sleep")) {
		err = -EBUSY;
		printk("[FP][%s] gpio request failed ! \n", __func__);
		goto fp_gpio_init_sleep_pin_failed;
	}

	err = gpio_direction_output(pdata->sleep_pin, 1);
	if (err < 0) {
		printk("[FP][%s] gpio_direction_output SLEEP failed ! \n", __func__);
		err = -EBUSY;
		goto fp_gpio_config_sleep_pin_failed;
	}

	return err;


fp_gpio_config_sleep_pin_failed:
	gpio_free(pdata->sleep_pin);
fp_gpio_init_sleep_pin_failed:
	gpio_free(pdata->drdy_pin);

	return err;
}

static int fp_pin_cfg(struct device *dev,
			struct vfsspi_device_data *pdata)
{

	pdata->sleep_pin = VFSSPI_SLEEP_PIN;

	if (pdata->sleep_pin < 0)
		return pdata->sleep_pin;

	pdata->drdy_pin = VFSSPI_DRDY_PIN;

	if (pdata->drdy_pin < 0)
		return pdata->drdy_pin;

	return 0;
}

static int fp_sensor_probe(struct platform_device *pdev)
{
	int status = 0;
	struct device *dev = &pdev->dev;
	//struct notifier_block fb_notifier;

	printk("[FP] Probe +++\n");
	fp_device = kzalloc(sizeof(*fp_device), GFP_KERNEL);
	if (fp_device == NULL) {
		printk("[FP] alloc Fingerprint data fail.\r\n");
		status = -1;
		goto fp_kmalloc_failed;
	}


	status = fp_pin_cfg(&pdev->dev, fp_device);
	fp_device->irq_wakeup_flag = false;

	if (fp_gpio_init(fp_device)) {
		printk("[FP] opps fp_gpio_init fail ! \n");
		status = -5;
		goto fp_pars_dt_failed;
	}

	dev_set_drvdata(&pdev->dev, fp_device);

	wake_lock_init(&fp_device->wake_lock, WAKE_LOCK_SUSPEND, "FP_wake_lock");
	wake_lock_init(&fp_device->wake_lock_irq, WAKE_LOCK_SUSPEND, "wake_lock_irq");

	spin_lock_init(&fp_device->vfs_spi_lock);
	mutex_init(&fp_device->buffer_mutex);
	mutex_init(&fp_device->kernel_lock);

	INIT_LIST_HEAD(&fp_device->device_entry);

	status = gpio_direction_input(fp_device->drdy_pin);
	if (status < 0) {
		printk("gpio_direction_input DRDY failed, status = %d \n", status);
		status = -8;
		goto fp_pars_dt_failed;
	}

	gpio_irq = gpio_to_irq(fp_device->drdy_pin);
	fp_device->isr_pin = gpio_irq;
	if (gpio_irq < 0) {
		printk("gpio_to_irq failed\n");
		status = -9;
		goto fp_pars_dt_failed;
	}

	if (request_irq(gpio_irq, vfsspi_irq, IRQF_TRIGGER_RISING,
			"vfsspi_irq", fp_device) < 0) {
		printk("request_irq failed \n");
		status = -10;
		goto fp_pars_dt_failed;
	}

	fp_device->is_drdy_irq_enabled = DRDY_IRQ_ENABLE;

	mutex_lock(&device_list_mutex);

	status = alloc_chrdev_region(&(fp_device->devt),
				     0, 1, VALIDITY_PART_NAME);
	if (status < 0) {
		printk("alloc_chrdev_region failed\n");
		goto fp_probe_alloc_chardev_failed;
	}

	cdev_init(&(fp_device->cdev), &vfsspi_fops);
	fp_device->cdev.owner = THIS_MODULE;
	status = cdev_add(&(fp_device->cdev), fp_device->devt, 1);
	if (status < 0) {
		printk("cdev_add failed\n");
		unregister_chrdev_region(fp_device->devt, 1);
		goto fp_probe_cdev_add_failed;
	}

	vfsspi_device_class = class_create(THIS_MODULE, "validity_fingerprint");

	if (IS_ERR(vfsspi_device_class)) {
		printk("vfsspi_init: class_create() is failed - unregister chrdev.\n");
		cdev_del(&(fp_device->cdev));
		unregister_chrdev_region(fp_device->devt, 1);
		status = PTR_ERR(vfsspi_device_class);
		goto vfsspi_probe_class_create_failed;
	}

	// register receiver for FB event
	fp_device->fb_notifier = SYNA_notify_block;
	fb_register_client(&fp_device->fb_notifier);
	// register receiver for FB event
	
	dev = device_create(vfsspi_device_class, &pdev->dev,
			    fp_device->devt, fp_device, "vfsspi");
	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if (status == 0)
		list_add(&fp_device->device_entry, &device_list);

        device_create_file(dev, &dev_attr_screenState);

	mutex_unlock(&device_list_mutex);

	fp_sensor_wq = create_singlethread_workqueue("fp_sensor_wq");
	INIT_DEFERRABLE_WORK(&fp_device->fp_sensor_work, fp_sensor_report_function);

	if (status != 0)
		goto vfsspi_probe_failed;

	return 0;


vfsspi_probe_failed:
vfsspi_probe_class_create_failed:
fp_probe_cdev_add_failed:
fp_probe_alloc_chardev_failed:
fp_pars_dt_failed:
	kfree(fp_device);
fp_kmalloc_failed:
	printk("[FP] Probe failed wieh error code %d ! \n", status);
	return status;
}

static void fp_sensor_shutdown(struct platform_device *dev)
{
	printk("[FP] Driver shutdown ---\n");
	fp_device->is_drdy_irq_enabled = DRDY_IRQ_DISABLE;
	free_irq(gpio_irq, fp_device);
}

static int fp_sensor_remove(struct platform_device *pdev)
{
	printk("[FP] Driver remove ---\n");
	fp_device->is_drdy_irq_enabled = DRDY_IRQ_DISABLE;
	free_irq(gpio_irq, fp_device);
	return 0;
}

static struct of_device_id fp_match_table[] = {
	{ .compatible = "asus,fingerprint",},
	{ },
};

static struct platform_driver fp_sensor_driver = {
	.driver = {
		.name = "metallica",
		.owner = THIS_MODULE,
		.of_match_table = fp_match_table,
	},
	.probe		= fp_sensor_probe,
	.remove		= fp_sensor_remove,
	.suspend	= fp_sensor_suspend,
	.resume		= fp_sensor_resume,
	.shutdown	= fp_sensor_shutdown,
};


static int __init fp_sensor_init(void)
{
	int err = 0;
	printk("[FP] Driver INIT +++\n");

	err = platform_driver_register(&fp_sensor_driver);
	if (err != 0) {
		printk("[FP] platform_driver_register fail, Error : %d\n", err);
	}

	printk("[FP] Driver INIT ---\n");

	return err;
}

static void __exit fp_sensor_exit(void)
{
	printk("[FP] Driver EXIT +++\n");

	platform_driver_unregister(&fp_sensor_driver);

	printk("[FP] Driver EXIT ---\n");
}


module_init(fp_sensor_init);
module_exit(fp_sensor_exit);
