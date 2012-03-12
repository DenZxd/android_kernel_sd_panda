//if your TP has button please define this
//#define BUTTON

#ifndef __PIXCIR_I2C_TS_H__
#define __PIXCIR_I2C_TS_H__

#define X_MAX 1023
#define Y_MAX 767

//#define BUTTON   //if have button on TP

#include <linux/gpio.h>

#define ATTB		38
#define get_attb_value  gpio_get_value

static int attb_read_val(void)
{
	return get_attb_value(ATTB);
}

struct pixcir_platform_data
{
	void (*rst)(void);
	void (*init)(void);
	unsigned long irq;
};

#endif
