/*
 * include/linux/ssd2533.h
 *
 * Copyright (C) 2010 - 2011 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef _LINUX_SSD2533_H
#define	_LINUX_SSD2533_H

/****** output debug information ******/
//#define SSL_DEBUG

#ifdef SSL_DEBUG
#define SSL_PRINT(x...)         printk("SSD253X:"x)
#else
#define SSL_PRINT(x...)
#endif

#define SSD2533_I2C_NAME    "SSD2533-TS"
#define SSD2533_I2C_ADDR    0x48
/****** auto-calibration conditions (only applied to SSD2531) ******/
/****** define auto-calibration threshold ******/
#define CAL_THRESHOLD -100
/****** define auto-calibration checking interval ******/
#define CAL_INTERVAL    50      //in ms
/****** define auto-calibration checking times ******/
#define CAL_COUNT               35      //check 45 seconds after boot-up/sleep out
/****** define delay time for calibration ******/
#define CAL_DELAY               300     //in ms
/****** define the number of finger that used to report coodinates ******/
#define FINGER_USED             10
/****** define delay time for bending restore ******/
#define BENDING_DELAY   200     //in ms

/****** define your RESET and IRQ input here ******/
#define TOUCH_INT_PIN           38
#define TOUCH_RES_PIN           37
#define GPIO_TS_SDA    91
#define GPIO_TS_SCL    90

/****** define your button codes ******/
#ifdef CONFIG_SMARTQ_T15
#define HAS_BUTTON
#define SIMULATED_BUTTON
#endif

#define BUTTON0 KEY_MENU      //menu
#define BUTTON1 KEY_BACK     //back
#define BUTTON2 217     //search
#define BUTTON3 KEY_HOME     //home

typedef struct{
        unsigned long x;        //center x of button
        unsigned long y;        //center y of button
        unsigned long dx;       //x range (x-dx)~(x+dx)
        unsigned long dy;       //y range (y-dy)~(y+dy)
        unsigned long code;     //key code for simulated key
}SKey_Info,*pSKey_Info;

#define REG_DEVICE_ID               0x02
#define FINGER_STATUS               0x79
#define FINGER_DATA                 0x7C
#define BUTTON_RAW                  0xB5
#define BUTTON_STATUS               0xB9

enum CMD_TYPE{CMD_DELAY=-1,CMD_ONLY=0,CMD_1B=1,CMD_2B=2};
typedef struct{
        enum CMD_TYPE type;
        unsigned int reg;
        unsigned char data1;
        unsigned char data2;
}Reg_Item,*pReg_Item;

/****** The following block is TP configuration, please get it from your TP vendor ******/
/****** TP configuration starts here ******/
#ifdef CONFIG_SMARTQ_T15
#define LCD_RANGE_X 1024
#define LCD_RANGE_Y 768
#else
#define LCD_RANGE_X 1280
#define LCD_RANGE_Y 800
#endif

struct ssd2533_platform_data
{
        uint32_t version;
        int (*power)(int on);
        void (*rst)(void);
        void (*irq_init)(void);
        void (*gpio_init)(void);
        void (*gpio_free)(void);
        int (*get_lock_state)(void);
        unsigned long irq;
        uint32_t flags;
        pReg_Item reg_array;
        int reg_size;
};

#endif /* _LINUX_SSD2533_H */
