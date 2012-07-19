/*
 * drivers/power/bq2416x_battery.c
 *
 * BQ24153 / BQ24156 battery charging driver
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Author: Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/bq2416x.h>
#include <linux/earlysuspend.h>
#include <linux/proc_fs.h>

#if defined(CONFIG_SMARTQ_S7) || defined(CONFIG_SMARTQ_K7)
  #define USE_USB_CHARGER
  #define USB_CHARGER_CURRENT 0x3c // 500mA current limit
#elif defined(CONFIG_SMARTQ_T16)
  #define USE_USB_CHARGER
  #define USB_CHARGER_CURRENT 0x5c // 1500mA current limit
#endif

struct charge_params {
	unsigned long		currentmA;
	unsigned long		voltagemV;
	unsigned long		term_currentmA;
	unsigned long		enable_iterm;
	bool			enable;
};

struct bq2416x_device_info {
	struct device		*dev;
	struct i2c_client	*client;
	struct charge_params	params;
	struct delayed_work	bq2416x_charger_work;
	struct notifier_block	nb;
	struct wake_lock	chrg_lock;
	struct early_suspend	early_suspend;

	unsigned short		status_reg;
	unsigned short		control_reg;
	unsigned short		voltage_reg;
	unsigned short		bqchip_version;
	unsigned short		current_reg;
	unsigned short		special_charger_reg;

	unsigned int		cin_limit;
	unsigned int		currentmA;
	unsigned int		voltagemV;
	unsigned int		max_currentmA;
	unsigned int		max_voltagemV;
	unsigned int		max_curLimit;
	unsigned int		term_currentmA;
	unsigned int		in_dpm;
	unsigned int		charger_current;
	unsigned int		standby_current;
	unsigned int		work_current;
	int			max_charger_voltagemV;

	int			timer_fault;
	bool			cfg_params;
	bool			enable_iterm;
	bool			active;
	bool			lock_flag;
};

static struct bq2416x_device_info *pdi;

static int bq2416x_write_block(struct bq2416x_device_info *di, u8 *value,
						u8 reg, unsigned num_bytes)
{
	struct i2c_msg msg[1];
	int ret;

	*value		= reg;

	msg[0].addr	= di->client->addr;
	msg[0].flags	= 0;
	msg[0].buf	= value;
	msg[0].len	= num_bytes + 1;

	ret = i2c_transfer(di->client->adapter, msg, 1);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 1) {
		dev_err(di->dev,
			"i2c_write failed to transfer all messages\n");
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
		return 0;
	}
}

static int bq2416x_read_block(struct bq2416x_device_info *di, u8 *value,
						u8 reg, unsigned num_bytes)
{
	struct i2c_msg msg[2];
	u8 buf;
	int ret;

	buf		= reg;

	msg[0].addr	= di->client->addr;
	msg[0].flags	= 0;
	msg[0].buf	= &buf;
	msg[0].len	= 1;

	msg[1].addr	= di->client->addr;
	msg[1].flags	= I2C_M_RD;
	msg[1].buf	= value;
	msg[1].len	= num_bytes;

	ret = i2c_transfer(di->client->adapter, msg, 2);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 2) {
		dev_err(di->dev,
			"i2c_write failed to transfer all messages\n");
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
		return 0;
	}
}

static int bq2416x_write_byte(struct bq2416x_device_info *di, u8 value, u8 reg)
{
	/* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
	u8 temp_buffer[2] = { 0 };

	/* offset 1 contains the data */
	temp_buffer[1] = value;
	return bq2416x_write_block(di, temp_buffer, reg, 1);
}

static int bq2416x_read_byte(struct bq2416x_device_info *di, u8 *value, u8 reg)
{
	return bq2416x_read_block(di, value, reg, 1);
}

int bq2416x_get_charge_status(void)
{
        u8 Reg0Val = 0;
        u8 stat = 0;
	int ret = 0;

	if (NULL == pdi)
		return 0;

	bq2416x_read_byte(pdi, &Reg0Val, Reg0Add);
        stat = (Reg0Val>>4) & 0x7;

        if (0x3 == stat || 0x1 == stat)
		ret = POWER_SUPPLY_STATUS_CHARGING;
#ifdef USE_USB_CHARGER
        else if (0x4 == stat || 0x2 == stat)
                ret = POWER_SUPPLY_STATUS_CHARGING | (1 << 16); //USB
#endif
        else if (0x5 == stat)
		ret = POWER_SUPPLY_STATUS_FULL;
        /*
        else if (0x1 == stat)
                return POWER_SUPPLY_STATUS_DISCHARGING;*/
        else
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return ret;
}
EXPORT_SYMBOL(bq2416x_get_charge_status);

static void bq2416x_set_reg00_status(struct bq2416x_device_info *di,unsigned short value)
{
	di->status_reg = value;
	bq2416x_write_byte(di, di->status_reg, REG_STATUS_CONTROL);

	return;
}

static void bq2416x_set_reg01_status(struct bq2416x_device_info *di,unsigned short value)
{
	di->status_reg = value;
	bq2416x_write_byte(di, di->status_reg, Reg1Add);
}

static void bq2416x_set_reg02_control(struct bq2416x_device_info *di,unsigned short value)
{
	di->status_reg = value;
	bq2416x_write_byte(di, di->status_reg, Reg2Add);

	return;
}

static void bq2416x_set_reg03_safety_reg(struct bq2416x_device_info *di,unsigned int max_curLimit,unsigned int max_voltagemV)
{
	int code = 0;
	int ilimbit;
	int vregbits = 0;
	u8 Reg3Val;

	if ((max_curLimit != IN_ILIM_1) && (max_curLimit != IN_ILIM_2))
		return;

	if((max_voltagemV < VOREG_MIN) || (max_voltagemV > VOREG_MAX))
		return;

	di->max_voltagemV = max_voltagemV;
	di->max_curLimit = max_curLimit;
	di->voltagemV = max_voltagemV;

	if (max_curLimit == IN_ILIM_1)
		ilimbit = 0;
	else {
		ilimbit = 1;
		ilimbit <<= IN_ILIM_LSHIFT;
	}
	code = ((max_voltagemV - VOREG_MIN)/VOREG_STEP);
	vregbits = code << VOREG_LSHIFT; //shift twice to left
	Reg3Val = vregbits | ilimbit;
	bq2416x_write_byte(di, Reg3Val, Reg3Add);

	return;
}

static int bq2416x_set_reg05_ChgCur(struct bq2416x_device_info *di, int ichg)
{
	int code = 0;
	int ichgbits = 0;
	u8 Reg5Val = 0;

	if((ichg < ICHG_MIN) | (ichg > ICHG_MAX))
        return -1;
	else {
		code = ((ichg - ICHG_MIN)/ICHG_STEP);
		ichgbits = code << ICHG_LSHIFT;
		bq2416x_read_byte(di, &Reg5Val, Reg5Add);
		Reg5Val = Reg5Val & ICHG_MASK;
		Reg5Val = ichgbits | Reg5Val;
		bq2416x_write_byte(di, Reg5Val, Reg5Add);
	}

	return 1;
}

static int bq2416x_set_reg05_TermCur(struct bq2416x_device_info *di, int iterm)
{
	int code = 0;
	int itermbits = 0;
	u8 Reg5Val;

	if((iterm < ITERM_MIN) | (iterm > ITERM_MAX))
	return -1;
	else {
		code = ((iterm - ITERM_MIN)/ITERM_STEP);
		itermbits = code << ITERM_LSHIFT;
		bq2416x_read_byte(di, &Reg5Val, Reg5Add);
		Reg5Val = Reg5Val & ITERM_MASK;
		Reg5Val = itermbits | Reg5Val;
		//Execute I2C Write Function
		bq2416x_write_byte(di, Reg5Val, Reg5Add);
	}
	return 1;
}

static int bq2416x_set_reg06_SetInVdpm(struct bq2416x_device_info *di, int vdpm)
{
	int code = 0;
	int invdpmbits = 0;
	u8 Reg6Val = 0;

	if((vdpm < IN_VDPM_MIN) | (vdpm > IN_VDPM_MAX))
		//Invalid vdpm
		return -1;
	else {
		code = ((vdpm - IN_VDPM_MIN)/IN_VDPM_STEP);
		invdpmbits = code << IN_VDPM_LSHIFT;
		bq2416x_read_byte(di, &Reg6Val, Reg6Add);
		Reg6Val = Reg6Val & IN_VDPM_MASK;
		Reg6Val = invdpmbits | Reg6Val;
		//Execute I2C Write Function
		bq2416x_write_byte(di, Reg6Val, Reg6Add);
	}
	return 1;
}

static int bq2416x_set_reg07_ThermalShutdown(struct bq2416x_device_info *di, int enable)
{
	int RegVal;
	u8 Reg7Val;

	bq2416x_read_byte(di, &Reg7Val, Reg7Add);
	RegVal = enable << TS_LSHIFT;
	Reg7Val = Reg7Val & TS_MASK;
	Reg7Val = RegVal | Reg7Val;
	Reg7Val = Reg7Val | 0x60;
	Reg7Val = Reg7Val & 0xf7;
	//Execute I2C Write Function
	bq2416x_write_byte(di, Reg7Val, Reg7Add);

	return 1;
}

void bq2416x_control_usb_charger(int usb_charger)
{
	u8 Reg1Val = 0;

	bq2416x_read_byte(pdi, &Reg1Val, Reg1Add);

	if(usb_charger == 1) {
		// No OTG supply present. Use USB input as normal.
		Reg1Val &= 0xf7;
		bq2416x_write_byte(pdi, Reg1Val, Reg1Add);
	} else if (usb_charger == 0) {
		// OTG supply present. Lockout USB input for charging
		Reg1Val |= 0x08;
		bq2416x_write_byte(pdi, Reg1Val, Reg1Add);
	}
}
EXPORT_SYMBOL(bq2416x_control_usb_charger);

static int bq2416x_check_reg02(struct bq2416x_device_info *di, int force)
{
        u8 Reg1Val = 0;
        u8 Reg2Val = 0;

        bq2416x_read_byte(di, &Reg1Val, Reg1Add);
        bq2416x_read_byte(di, &Reg2Val, Reg2Add);
        if ((Reg1Val >> 1) & 0x3) {
                if ((Reg2Val & 0x4) || force) {
                        //Reg2Val &= ~((1<<2) | (1<<7));
#ifdef USE_USB_CHARGER
                        bq2416x_write_byte(di, 0x38, Reg2Add);
#else
                        bq2416x_write_byte(di, 0x08, Reg2Add);
#endif
                }
        }
        else {
#ifdef USE_USB_CHARGER
                if ((Reg2Val != (USB_CHARGER_CURRENT | 0x80)) || force) {
                        //Reg2Val &= ~(1<<7);
                        //Reg2Val |= (1<<2);
                        bq2416x_write_byte(di, USB_CHARGER_CURRENT, Reg2Add);
                }
#else
                if (!(Reg2Val & 0x4)) {
                        bq2416x_write_byte(di, 0x0c, Reg2Add);
                }
#endif
        }
}

static int bq2416x_charger_chip_init(struct bq2416x_device_info *di)
{
	u8 read_reg[8] = {0};

	bq2416x_set_reg03_safety_reg(di,di->cin_limit,di->max_charger_voltagemV);
	bq2416x_set_reg05_ChgCur(di,di->charger_current);
	bq2416x_set_reg05_TermCur(di,di->term_currentmA);
	bq2416x_set_reg06_SetInVdpm(di,di->in_dpm);
	bq2416x_set_reg07_ThermalShutdown(di,TS_DIS);
	bq2416x_set_reg00_status(di,TIMER_RST);
	bq2416x_set_reg01_status(di,0x00);

	bq2416x_read_block(di, &read_reg[0], 0, 8);
	printk("%s: Status/Control Register = %x\n",__func__,read_reg[0]);
	printk("%s: Battery/Status Register = %x\n",__func__,read_reg[1]);
	printk("%s: Control Register = %x\n",__func__,read_reg[2]);
	printk("%s: Control/Battery Voltage Registe = %x\n",__func__,read_reg[3]);
	printk("%s: Vender/Part/Revision Register = %x\n",__func__,read_reg[4]);
	printk("%s: Battery Termination/Fast Charge Current Register = %x\n",__func__,read_reg[5]);
	printk("%s: VIN-DPM Voltage/ DPPM Status Register = %x\n",__func__,read_reg[6]);
	printk("%s: Safety Timer/ NTC Monitor Register = %x\n",__func__,read_reg[7]);

	return 0;
}

static void bq2416x_charger_update_status(struct bq2416x_device_info *di)
{
	u8 read_reg[8] = {0};

	bq2416x_read_block(di, &read_reg[0], 0, 8);

        bq2416x_check_reg02(di, 0);

	if(((read_reg[0] & 0x07) == 0x03) || ((read_reg[0] & 0x07) == 0x07) || ((read_reg[0] & 0x70) == 0x70))
	    bq2416x_charger_chip_init(di);

#if 0
	printk("\n----------------------------------------------------------\n");
	printk("%s: Status/Control Register = %x\n",__func__,read_reg[0]);
	printk("%s: Battery/Status Register = %x\n",__func__,read_reg[1]);
	printk("%s: Control Register = %x\n",__func__,read_reg[2]);
	printk("%s: Control/Battery Voltage Registe = %x\n",__func__,read_reg[3]);
	printk("%s: Vender/Part/Revision Register = %x\n",__func__,read_reg[4]);
	printk("%s: Battery Termination/Fast Charge Current Register = %x\n",__func__,read_reg[5]);
	printk("%s: VIN-DPM Voltage/ DPPM Status Register = %x\n",__func__,read_reg[6]);
	printk("%s: Safety Timer/ NTC Monitor Register = %x\n",__func__,read_reg[7]);

#endif

#if 0
	if(read_reg[3] != 0x80)
		bq2416x_set_reg03_safety_reg(di,di->cin_limit,di->max_charger_voltagemV);

	if(read_reg[5] != 0x43)
		bq2416x_set_reg05_ChgCur(di,di->charger_current);

#endif

	bq2416x_set_reg00_status(di,TIMER_RST);

	return;
}

void bq2416x_set_charger_current(int mode)
{
	if (!pdi)
		return ;
	int ret = bq2416x_get_charge_status();

	if (mode == SLEEP_MODE)
		bq2416x_set_reg05_ChgCur(pdi,pdi->standby_current);
	else
		bq2416x_set_reg05_ChgCur(pdi,pdi->work_current);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bq2416x_early_suspend(struct early_suspend *h)
{
	int ret;

	ret = bq2416x_get_charge_status();
	if(ret != POWER_SUPPLY_STATUS_NOT_CHARGING){
		pdi->lock_flag = true;
		wake_lock(&pdi->chrg_lock);
		pdi->charger_current = pdi->standby_current;
		bq2416x_set_reg05_ChgCur(pdi,pdi->charger_current);

	}
	bq2416x_set_charger_current(WORK_MODE);
}

static void bq2416x_late_resume(struct early_suspend *h)
{
	if(pdi->lock_flag){
		pdi->lock_flag = false;
		wake_unlock(&pdi->chrg_lock);
		pdi->charger_current = pdi->work_current;
		bq2416x_set_reg05_ChgCur(pdi,pdi->charger_current);
	}
}
#endif


static void bq2416x_charger_work(struct work_struct *work)
{
	struct bq2416x_device_info *di = container_of(work,struct bq2416x_device_info, bq2416x_charger_work.work);

	bq2416x_charger_update_status(di);
	schedule_delayed_work(&di->bq2416x_charger_work,msecs_to_jiffies(BQ2416x_WATCHDOG_TIMEOUT));
}

static int deb24161_proc_read(char* page, char** start, off_t off, int count,int* eof, void* data)
{
	data = (void *)page;
	u8 read_reg[8] = {0};

	bq2416x_read_block(pdi, &read_reg[0], 0, 8);

#if 1
	page += sprintf(page,"Status/Control Register: %x\n",read_reg[0]);
	page += sprintf(page,"Battery/ Supply Status Register: %x\n",read_reg[1]);
	page += sprintf(page,"Control Register = %x\n",read_reg[2]);
	page += sprintf(page,"Control/Battery Voltage Registe = %x\n",read_reg[3]);
	page += sprintf(page,"Vender/Part/Revision Register = %x\n",read_reg[4]);
	page += sprintf(page,"Battery Termination/Fast Charge Current Register = %x\n",read_reg[5]);
	page += sprintf(page,"VIN-DPM Voltage/ DPPM Status Register = %x\n",read_reg[6]);
	page += sprintf(page,"Safety Timer/ NTC Monitor Register = %x\n",read_reg[7]);
#endif
	return  ((page += sprintf(page, "\n")) - (char*)data);

}
//----------------------------------------------------
#define BQ24161_PROC_DEB "driver/deb24161"

static int bq24161_proc_init(void)
{
	struct proc_dir_entry* deb24161;

	if (!(deb24161 = create_proc_entry(BQ24161_PROC_DEB,S_IRUGO | S_IWUSR, NULL))){
		printk("Proc-FS interface for deb24161 failed\n");
		return -ENOMEM;
	}

	deb24161->read_proc = deb24161_proc_read;
	deb24161->write_proc = NULL;
	deb24161->data = NULL;

	return 0;
}

static void bq24161_proc_exit(void)
{
	remove_proc_entry(BQ24161_PROC_DEB,NULL);
}
//------------------------------------------------------

static int __devinit bq2416x_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq2416x_device_info *di;
	struct bq2416x_platform_data *pdata = client->dev.platform_data;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->dev = &client->dev;
	di->client = client;
	i2c_set_clientdata(client, di);
	pdi = di;

	di->term_currentmA = pdata->termination_currentmA;
	di->in_dpm = pdata->in_dpm;
	di->charger_current = pdata->work_current;
	di->work_current = pdata->work_current;
	di->standby_current = pdata->standby_current;
	di->max_charger_voltagemV = pdata->max_charger_voltagemV;
	di->cin_limit = pdata->cin_limit;

	bq2416x_check_reg02(di, 1);
	bq2416x_charger_chip_init(di);
	bq24161_proc_init();

	di->active = 0;
	di->params.enable = 1;
	di->cfg_params = 1;
	di->enable_iterm = 1;
	di->lock_flag = false;
	di->active = 1;

	di->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	di->early_suspend.suspend = bq2416x_early_suspend;
	di->early_suspend.resume = bq2416x_late_resume;
	register_early_suspend(&di->early_suspend);

	wake_lock_init(&di->chrg_lock, WAKE_LOCK_SUSPEND, "ac_chrg_wake_lock");

	INIT_DELAYED_WORK_DEFERRABLE(&di->bq2416x_charger_work,bq2416x_charger_work);
	schedule_delayed_work(&di->bq2416x_charger_work, 0);

	return 0;
}

static int __devexit bq2416x_charger_remove(struct i2c_client *client)
{
	struct bq2416x_device_info *di = i2c_get_clientdata(client);

	cancel_delayed_work(&di->bq2416x_charger_work);
	flush_scheduled_work();
	wake_lock_destroy(&di->chrg_lock);
	bq24161_proc_exit();

	kfree(di);

	return 0;
}

static int bq2416x_suspend(struct device *dev)
{

	return 0;
}

static int bq2416x_resume(struct device *dev)
{

	return 0;
}

static const struct dev_pm_ops bq2416x_pm_ops = {
	.suspend = bq2416x_suspend,
	.resume = bq2416x_resume,
};

static const struct i2c_device_id bq2416x_id[] = {
	{ "bq2416x", 0 },
	{},
};

static struct i2c_driver bq2416x_charger_driver = {
	.probe		= bq2416x_charger_probe,
	.remove		= __devexit_p(bq2416x_charger_remove),
	.id_table	= bq2416x_id,
	.driver		= {
		.name	= "bq2416x_charger",
#ifdef CONFIG_PM
		.pm     = &bq2416x_pm_ops,
#endif
	},
};

static int __init bq2416x_charger_init(void)
{
	return i2c_add_driver(&bq2416x_charger_driver);
}
module_init(bq2416x_charger_init);

static void __exit bq2416x_charger_exit(void)
{
	i2c_del_driver(&bq2416x_charger_driver);
}
module_exit(bq2416x_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments Inc");
