#include "fusb30x_global.h"
#include <linux/gpio.h>		// struct gpio

/* ASUS_BSP : For GPIO/adb/log in FUSB302 --- */
struct gpio fusb302_gpios[USB_ASUS_NUM_GPIOS] = { /* add GPIO array here */
    { USB_RE_GPIO, GPIOF_OUT_INIT_LOW, "USB3_rD_en" }, /* default Re-driver to output LOW */
    { USB_MUX_GPIO, GPIOF_OUT_INIT_LOW, "USB3_MUX_SEL_SOC" }, /* default MUX to output LOW */
};

struct fusb30x_chip* g_chip = NULL;  // Our driver's relevant data

struct fusb30x_chip* fusb30x_GetChip(void)
{
    return g_chip;      // return a pointer to our structs
}

void fusb30x_SetChip(struct fusb30x_chip* newChip)
{
    g_chip = newChip;   // assign the pointer to our struct
}