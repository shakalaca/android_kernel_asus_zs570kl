#ifndef __LEDS_H_INCLUDED
#define __LEDS_H_INCLUDED

#include <linux/device.h>
#include <linux/rwsem.h>
#include <linux/leds.h>

//vkLed+++
void set_button_backlight(bool status);
//vkLed---

#endif	/* __LEDS_H_INCLUDED */
