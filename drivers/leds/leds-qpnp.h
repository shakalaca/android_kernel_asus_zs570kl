#ifndef __LEDS_H_INCLUDED
#define __LEDS_H_INCLUDED

#include <linux/device.h>
#include <linux/rwsem.h>
#include <linux/leds.h>

//led+++
void set_button_backlight(bool status);
//led---

#endif	/* __LEDS_H_INCLUDED */