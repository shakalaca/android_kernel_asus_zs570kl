/*
* File:   fusb30x_global.h
* Author: Tim Bremm <tim.bremm@fairchildsemi.com>
* Company: Fairchild Semiconductor
*
* Created on September 11, 2015, 15:28 AM
*/

#ifndef FUSB30X_TYPES_H
#define FUSB30X_TYPES_H

#include <linux/i2c.h>                              // i2c_client, spinlock_t
#include <linux/hrtimer.h>                          // hrtimer
#include <linux/completion.h>
#include "FSCTypes.h"                               // FUSB30x custom types
#include <linux/gpio.h>		                    // Asus GPIO
#include <linux/wakelock.h>                         // wake lock protection

/* ASUS_BSP : For GPIO/adb/log/early_suspend/OTG in FUSB302 +++ */
/* define GPIO/MUX state */
#define USB_ASUS_NUM_GPIOS 2
#define USB_RE_GPIO  16
#define USB_RE_HIGH 1
#define USB_RE_LOW  0
#define USB_MUX_GPIO 25
#define USB_MUX_HIGH 1
#define USB_MUX_LOW  0
/* define log */
#define USB_FUSB302_331_INFO(...)	printk(KERN_EMERG "[USB][fusb30x_driver] " __VA_ARGS__);	  // used for printing everything!
//#define USB_FUSB302_331_INFO(...)	printk("[USB][fusb30x_driver] " __VA_ARGS__);			// follows the dmesg priority level
/* add GPIO array */
extern struct gpio fusb302_gpios[];
/* usb variable for OTG charger */
extern struct power_supply *usb_psy;   // for OTG set property
extern struct power_supply *usb_parallel_psy;	// for OTG set property
/* PD factory informations dump */
extern FSC_BOOL USBPDEnabled;
/* for qpnp-smbcharger.c : function "platform_fusb302_is_otg_present" flag */
extern bool flag_is_otg_present;
/* ASUS_BSP : For GPIO/adb/log/early_suspend/OTG in FUSB302 --- */

#ifdef FSC_DEBUG
#define FSC_HOSTCOMM_BUFFER_SIZE    64              // Length of the hostcomm buffer
#endif // FSC_DEBUG

struct fusb30x_chip                                 // Contains data required by this driver
{
    struct mutex lock;                              // Synchronization lock

#ifdef FSC_DEBUG
    FSC_U8 dbgTimerTicks;                           // Count of timer ticks
    FSC_U8 dbgTimerRollovers;                       // Timer tick counter rollover counter
    FSC_U8 dbgSMTicks;                              // Count of state machine ticks
    FSC_U8 dbgSMRollovers;                          // State machine tick counter rollover counter
    FSC_S32 dbg_gpio_StateMachine;                  // Gpio that toggles every time the state machine is triggered
    FSC_BOOL dbg_gpio_StateMachine_value;           // Value of sm toggle state machine
    char HostCommBuf[FSC_HOSTCOMM_BUFFER_SIZE];     // Buffer used to communicate with HostComm
#endif // FSC_DEBUG

    /* Internal config data */
    FSC_S32 InitDelayMS;                            // Number of milliseconds to wait before initializing the fusb30x
    FSC_S32 numRetriesI2C;                          // Number of times to retry I2C reads/writes

    /* I2C */
    struct i2c_client* client;                      // I2C client provided by kernel
    FSC_BOOL use_i2c_blocks;                        // True if I2C_FUNC_SMBUS_I2C_BLOCK is supported

    /* GPIO */
    FSC_S32 gpio_VBus5V;                            // VBus 5V GPIO pin
    FSC_BOOL gpio_VBus5V_value;                     // true if active, false otherwise
    FSC_S32 gpio_VBusOther;                         // VBus other GPIO pin (eg. VBus 12V) (NOTE: Optional feature - if set to <0 during GPIO init, then feature is disabled)
    FSC_BOOL gpio_VBusOther_value;                  // true if active, false otherwise
    FSC_S32 gpio_IntN;                              // INT_N GPIO pin

	/* ASUS mux handling */
    FSC_U8 prev_orientation;                        // Track the previous orientation to determine when it has changed
    FSC_BOOL asus_gpios_are_valid;                  // Indicates if GPIO init succeeded and GPIOs are valid

#ifdef FSC_INTERRUPT_TRIGGERED
    FSC_S32 gpio_IntN_irq;                          // IRQ assigned to INT_N GPIO pin
#endif  // FSC_INTERRUPT_TRIGGERED
    
    /* Threads */
    struct delayed_work init_worker;                // Kicks off our runtime worker
    struct work_struct worker;                      // Main state machine actions

    /* Timers */
    struct hrtimer timer_state_machine;             // High-resolution timer for the state machine
    struct wake_lock fusb302_wakelock;              // wake lock protection
	struct device *dev;
	struct dual_role_phy_instance *dual_role;
	struct dual_role_phy_desc *desc;
	FSC_S32 reverse_state;
	struct completion reverse_completion;
};

extern struct fusb30x_chip* g_chip;

struct fusb30x_chip* fusb30x_GetChip(void);         // Getter for the global chip structure
void fusb30x_SetChip(struct fusb30x_chip* newChip); // Setter for the global chip structure

#endif /* FUSB30X_TYPES_H */
