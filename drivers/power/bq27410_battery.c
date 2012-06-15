/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/skbuff.h>
#include <asm/unaligned.h>

#define DRIVER_VERSION			"0.0.1"

#define BQ27410_REG_CNTL        0x00
#define BQ27410_REG_TEMP		0x02
#define BQ27410_REG_VOLT		0x04
#define BQ27410_REG_AI			0x10
#define BQ27410_REG_FLAGS		0x06
#define BQ27410_REG_NAC			0x08 /* Nominal available capaciy */
#define BQ27410_REG_LMD			0x0e /* Last measured discharge */
#define BQ27410_REG_AE			0x16 /* Available enery */

#define BQ27410_REG_SOC			0x1C
#define BQ27410_REG_DCAP		0x3C /* Design capacity */

#define BQ27410_CNTL_INITCOMP   BIT(7)
#define BQ27410_FLAG_DSC		BIT(0)
#define BQ27410_FLAG_BAT_DET    BIT(3)
#define BQ27410_FLAG_FC			BIT(9)

#define BQ27410_REG_RM 0xc
#define BQ27410_REG_FAC 0xa

#define DOWN_MODE 0
#define WORK_MODE 1

struct bq27410_device_info;
static int bq27410_battery_get_property(struct power_supply *psy,
                                        enum power_supply_property psp,
                                        union power_supply_propval *val);
extern int bq2416x_get_charge_status(void);

struct bq27410_reg_cache {
	int temperature;
	int charge_full;
	int capacity;
	int flags;
	int bq24161_status;

	int current_now;
};

struct bq27410_device_info {
	struct device 		*dev;
	int			id;
	int         mode;
	int         supplies_done;

	struct bq27410_reg_cache cache;
	int charge_design_full;

	unsigned long last_update;
	struct delayed_work work;

	struct mutex lock;
};

struct bq27410_device_info *di;
typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_AC
} CB;

static enum power_supply_property bq27410_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static enum power_supply_property bq27410_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply bq27410_power_supplies[] = {
	{
		.name = "bq27410-battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = bq27410_battery_props,
		.num_properties = ARRAY_SIZE(bq27410_battery_props),
		.get_property = bq27410_battery_get_property,
	},
	{
		.name = "bq27410-ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.properties = bq27410_ac_properties,
		.num_properties = ARRAY_SIZE(bq27410_ac_properties),
		.get_property = bq27410_battery_get_property,
	},

};

static unsigned int poll_interval = 60 * 3;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
				"0 disables polling");

/*
 * Common code for BQ27x00 devices
 */

#define BQ27410_I2C_RETRY 5
static inline int bq27410_write_base(struct bq27410_device_info *di, unsigned char addr,
                                     int len, unsigned char *data)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[1];
	int ret, i2c_addr, i;

	if (!client->adapter)
		return -ENODEV;

	i2c_addr = client->addr;
	if (addr > 0)
		i2c_addr = addr;

	msg[0].addr = i2c_addr;
	msg[0].flags = 0;
	msg[0].len = len;
	msg[0].buf = data;

	for (i = 0; i < BQ27410_I2C_RETRY; i++) {
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (ret == ARRAY_SIZE(msg))
			break;
		else
			ret = -1;
	}

	return ret;
}

static inline int bq27410_read_base(struct bq27410_device_info *di, unsigned char addr,
                                    unsigned char reg, int len, unsigned char *data)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	int ret, i2c_addr, i;

	if (!client->adapter)
		return -ENODEV;

	i2c_addr = client->addr;
	if (addr > 0)
		i2c_addr = addr;

	msg[0].addr = i2c_addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = 1;
	msg[1].addr = i2c_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = len;

	for (i = 0; i < BQ27410_I2C_RETRY; i++) {
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (ret == ARRAY_SIZE(msg))
			break;
		else
			ret = -1;
	}

	return ret;
}

static inline int bq27410_read(struct bq27410_device_info *di, u8 reg,
                               bool single)
{
	unsigned char data[2];
	int ret, len;

	if (single)
		len = 1;
	else
		len = 2;

	ret = bq27410_read_base(di, 0, reg, len, data);
	if (ret < 0) {
		dev_err(di->dev, "%s read error\n", __func__);
		return ret;
	}

	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];

	return ret;
}

static int bq27410_battery_read_contrl(struct bq27410_device_info *di)
{
	unsigned char data[3] = { 0 };
	int contrl_status;

	bq27410_write_base(di, 0, 3, data);
	mdelay(20);
	contrl_status = bq27410_read(di, BQ27410_REG_CNTL, true);

	if (contrl_status < 0)
		dev_err(di->dev, "error reading contrl status\n");

	return contrl_status;
}

static void bq27410_battery_insert(struct bq27410_device_info *di)
{
	unsigned char data[3] = {BQ27410_REG_CNTL, 0xc, 0x0};

	bq27410_write_base(di, 0, 3, data);
}

static void bq27410_battery_detection(struct bq27410_device_info *di)
{
	int i;
	int flags, contrl_status;

	bq27410_battery_insert(di);

	for(i = 0; i < BQ27410_I2C_RETRY; i++) {
		flags = bq27410_read(di, BQ27410_REG_FLAGS, true);
		contrl_status = bq27410_battery_read_contrl(di);

		if ((flags & BQ27410_FLAG_BAT_DET) && (contrl_status & BQ27410_CNTL_INITCOMP)) {
			dev_err(di->dev, "bq27410 initialization Control = 0x%x Flag = 0x%x\n",contrl_status, flags);
			break;
		} else {
			msleep(200);
			bq27410_battery_insert(di);
		}
	}

	if (i >= BQ27410_I2C_RETRY)
		dev_err(di->dev, "bq27410 init failed\n");
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27410_battery_read_rsoc(struct bq27410_device_info *di)
{
	int rsoc;

	rsoc = bq27410_read(di, BQ27410_REG_SOC, false);

	if (rsoc < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return rsoc;
}

/*
 * Return a battery charge value in µAh
 * Or < 0 if something fails.
 */
static int bq27410_battery_read_charge(struct bq27410_device_info *di, u8 reg)
{
	int charge;

	charge = bq27410_read(di, reg, false);
	if (charge < 0) {
		dev_err(di->dev, "error reading nominal available capacity\n");
		return charge;
	}

		charge *= 1000;

	return charge;
}

/*
 * Return the battery Nominal available capaciy in µAh
 * Or < 0 if something fails.
 */
static inline int bq27410_battery_read_nac(struct bq27410_device_info *di)
{
	return bq27410_battery_read_charge(di, BQ27410_REG_NAC);
}

/*
 * Return the battery Last measured discharge in µAh
 * Or < 0 if something fails.
 */
static inline int bq27410_battery_read_lmd(struct bq27410_device_info *di)
{
	return bq27410_battery_read_charge(di, BQ27410_REG_LMD);
}

/*
 * Return the battery Initial last measured discharge in µAh
 * Or < 0 if something fails.
 */
static int bq27410_battery_read_ilmd(struct bq27410_device_info *di)
{
	int ilmd;

		ilmd = bq27410_read(di, BQ27410_REG_DCAP, false);

	if (ilmd < 0) {
		dev_err(di->dev, "error reading initial last measured discharge\n");
		return ilmd;
	}

		ilmd *= 1000;

	return ilmd;
}

static void bq27410_update(struct bq27410_device_info *di)
{
	struct bq27410_reg_cache cache = {0, };

	if (di->mode == DOWN_MODE)
		return ;

	cache.flags = bq27410_read(di, BQ27410_REG_FLAGS, true);
	if (cache.flags >= 0) {
		cache.capacity = bq27410_battery_read_rsoc(di);
		if (cache.capacity > 100)
			cache.capacity = 100;
		cache.temperature = bq27410_read(di, BQ27410_REG_TEMP, false);
		cache.charge_full = bq27410_battery_read_lmd(di);

		cache.current_now = (int)(s16)bq27410_read(di, BQ27410_REG_AI, false);

		/* We only have to read charge design full once */
		if (di->charge_design_full <= 0)
			di->charge_design_full = bq27410_battery_read_ilmd(di);
	}

	cache.bq24161_status = bq2416x_get_charge_status();

	/* Ignore current_now which is a snapshot of the current battery state
	 * and is likely to be different even between two consecutive reads */
	if ((memcmp(&di->cache, &cache, sizeof(cache) - sizeof(int)) != 0) && (di->supplies_done)) {
		di->cache = cache;
		power_supply_changed(&bq27410_power_supplies[CHARGER_BATTERY]);
		power_supply_changed(&bq27410_power_supplies[CHARGER_AC]);
	}

	di->last_update = jiffies;
}

static void bq27410_battery_poll(struct work_struct *work)
{
	struct bq27410_device_info *di =
		container_of(work, struct bq27410_device_info, work.work);

	bq27410_update(di);

	if (poll_interval > 0) {
		/* The timer does not have to be accurate. */
		set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
		schedule_delayed_work(&di->work, poll_interval * HZ);
	}
}


/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27410_battery_temperature(struct bq27410_device_info *di,
	union power_supply_propval *val)
{
	if (di->cache.temperature < 0)
		return di->cache.temperature;

		val->intval = di->cache.temperature - 2731;

	return 0;
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27410_battery_current(struct bq27410_device_info *di,
	union power_supply_propval *val)
{
	int curr;

	    curr = bq27410_read(di, BQ27410_REG_AI, false);

	if (curr < 0)
		return curr;

		/* bq27500 returns signed value */
		val->intval = (int)((s16)curr) * 1000;



	return 0;
}

static int battery_ac_status(struct bq27410_device_info *di)
{
	int status;

	if (di->last_update == 0)
		bq27410_update(di);

	status = di->cache.bq24161_status;
	if (status == POWER_SUPPLY_STATUS_UNKNOWN)
		status = bq2416x_get_charge_status();

	if (status == POWER_SUPPLY_STATUS_CHARGING && di->cache.capacity == 100)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (status == POWER_SUPPLY_STATUS_FULL && di->cache.capacity < 100)
		di->cache.capacity = 100;
	else if (status == POWER_SUPPLY_STATUS_UNKNOWN) {
		if (di->cache.flags & BQ27410_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27410_FLAG_DSC)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	}

	return status;
}

static int bq27410_battery_status(struct bq27410_device_info *di,
	union power_supply_propval *val)
{
	int status;

	status = battery_ac_status(di);
	val->intval = status;

	return 0;
}

static int bq27410_ac_status(struct bq27410_device_info *di,
                             union power_supply_propval *val)
{
	int status;

	status = battery_ac_status(di);

	if (status == POWER_SUPPLY_STATUS_FULL || status == POWER_SUPPLY_STATUS_CHARGING)
		val->intval = 1;
	else
		val->intval =0;

	return 0;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27410_battery_voltage(struct bq27410_device_info *di,
	union power_supply_propval *val)
{
	int volt;

	volt = bq27410_read(di, BQ27410_REG_VOLT, false);
	if (volt < 0)
		return volt;

	val->intval = volt * 1000;

	return 0;
}

/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 */
static int bq27410_battery_energy(struct bq27410_device_info *di,
	union power_supply_propval *val)
{
	int ae;

	ae = bq27410_read(di, BQ27410_REG_AE, false);
	if (ae < 0) {
		dev_err(di->dev, "error reading available energy\n");
		return ae;
	}

	ae *= 1000;

	val->intval = ae;

	return 0;
}


static int bq27410_simple_value(int value,
	union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

#define to_bq27410_device_info(x) container_of((x), \
				struct bq27410_device_info, bat);

static int bq27410_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;

	mutex_lock(&di->lock);
	if (time_is_before_jiffies(di->last_update + 5 * HZ)) {
		cancel_delayed_work_sync(&di->work);
		bq27410_battery_poll(&di->work.work);
	}
	mutex_unlock(&di->lock);

	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = bq27410_ac_status(di, val);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27410_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq27410_battery_voltage(di, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->cache.flags < 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq27410_battery_current(di, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = bq27410_simple_value(di->cache.capacity, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = bq27410_battery_temperature(di, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27410_simple_value(bq27410_battery_read_nac(di), val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27410_simple_value(di->cache.charge_full, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = bq27410_simple_value(di->charge_design_full, val);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = bq27410_battery_energy(di, val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

void bq27410_external_power_changed(void)
{

	if (!di)
		return ;
	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, 0);
}

static int bq27410_powersupply_init(struct bq27410_device_info *di)
{
	int ret;
	int i;


	INIT_DELAYED_WORK(&di->work, bq27410_battery_poll);
	mutex_init(&di->lock);

	for (i = 0; i < ARRAY_SIZE(bq27410_power_supplies); i++){
		ret = power_supply_register(di->dev, &bq27410_power_supplies[i]);
		if (ret) {
			dev_err(di->dev, "failed to register power supply: %d\n", ret);

			return ret;
		}

	}
	di->supplies_done = 1;

	bq27410_update(di);

	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;
}

static void bq27410_powersupply_unregister(struct bq27410_device_info *di)
{
	int i;
	cancel_delayed_work_sync(&di->work);

	for (i = 0; i < ARRAY_SIZE(bq27410_power_supplies); i++)
		power_supply_unregister(&bq27410_power_supplies[i]);

	mutex_destroy(&di->lock);
}

static int bq27410_suspend(struct device *dev)
{
	return 0;
}

static int bq27410_resume(struct device *dev)
{
	if (!di)
		return 0;
	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, 0);
	return 0;
}

static char bq27410fs_addr, bq27410fs_reg;
static int bq27410_proc_read(char* page, char** start, off_t off, int count,int* eof, void* data)
{
	struct bq27410_device_info *di = data;
	char *kdata;
	int i, ret;

	if (!(kdata = kmalloc(count, GFP_KERNEL))) {
		dev_err(di->dev,"%s : kmalloc failed\n",__func__);
		return -ENOMEM;
	}

	ret = bq27410_read_base(di, bq27410fs_addr, bq27410fs_reg, count, kdata);
	if (ret > 0) {
		for(i = 0; i < count; i++)
			*(page + i) = *(kdata + i);
	} else {
		dev_err(di->dev,"%s: bq27410_read_base failed reg = %x\n",__func__,bq27410fs_reg);
		kfree(kdata);
		return -ENOMEM;
	}

	kfree(kdata);
	return count;
}

static int bq27410_proc_write(struct file* file, const char* buffer,unsigned long count, void* data)
{
	struct bq27410_device_info *di = data;
	char *kdata;
	int ret;

	if (count < 1) {
		dev_err(di->dev,"%s : count invalid\n", __func__);
		return -1;
	}

	if (!(kdata = kmalloc(count + 1, GFP_KERNEL))) {
		dev_err(di->dev,"%s : kmalloc failed\n", __func__);
		return -ENOMEM;
	}

	if (copy_from_user(kdata, buffer, count + 1)){
		kfree(kdata);
		dev_err(di->dev,"%s: Implementation of copy_from_user error\n", __func__);
		return -EFAULT;
	}

	if (*(kdata) == 0xff) {
		if (*(kdata+1) == 0xff)	{
			di->mode = DOWN_MODE;
		} else {
			di->mode = WORK_MODE;
		}
		goto out;
	}

	if (*(kdata) == 0x16)
		bq27410fs_addr = *(kdata) >> 1;
	else
		bq27410fs_addr = *(kdata);
	bq27410fs_reg = *(kdata+1);

	if (count > 1) {
		ret = bq27410_write_base(di, bq27410fs_addr, count, (kdata + 1));
		if (ret < 0) {
			dev_err(di->dev,"%s: bq27410_write_base failed reg = %x\n",__func__,bq27410fs_reg);
			kfree(kdata);
			return -ENOMEM;
		}
	}

out:
	kfree(kdata);
	return count;
}

#define BQ27410_PROC_ENTRY "driver/bq27410"
static int bq27410_proc_init(struct bq27410_device_info *di)
{
	struct proc_dir_entry* bq27410_proc;
	if (!(bq27410_proc = create_proc_entry(BQ27410_PROC_ENTRY, 0666, NULL))) {
		dev_err(di->dev, "Proc-FS interface for bq27410 failed\n");
		return -1;
	}

	bq27410_proc->read_proc  = bq27410_proc_read;
	bq27410_proc->write_proc = bq27410_proc_write;
	bq27410_proc->data   = di;

	return 0;
}

static void bq27410_proc_exit(void)
{
	remove_proc_entry(BQ27410_PROC_ENTRY, NULL);
}
/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_MUTEX(battery_mutex);


static int bq27410_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int retval = 0;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di->dev = &client->dev;
	di->mode = WORK_MODE;

	bq27410_battery_detection(di);
	bq27410_proc_init(di);

	if (bq27410_powersupply_init(di))
		goto batt_failed_2;

	i2c_set_clientdata(client, di);

	return 0;

batt_failed_2:
	kfree(di);
batt_failed_1:
	return retval;
}

static int bq27410_battery_remove(struct i2c_client *client)
{
	struct bq27410_device_info *di = i2c_get_clientdata(client);

	bq27410_powersupply_unregister(di);
	bq27410_proc_exit();

	kfree(di);

	return 0;
}


/*
 * Module stuff
 */

static const struct i2c_device_id bq27410_id[] = {
    { "bq27410",0},
	{},
};

#ifdef CONFIG_PM
static const struct dev_pm_ops bq27410_pm_ops = {
	.suspend = bq27410_suspend,
	.resume = bq27410_resume,
};
#endif

MODULE_DEVICE_TABLE(i2c, bq27410_id);

static struct i2c_driver bq27410_battery_driver = {
	.driver = {
		.name = "bq27410-battery",
		.owner  = THIS_MODULE,
#ifdef CONFIG_PM
		.pm     = &bq27410_pm_ops,
#endif
	},
	.probe = bq27410_battery_probe,
	.remove = bq27410_battery_remove,
	.id_table = bq27410_id,
};

static int __init bq27410_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27410_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27410 i2c driver\n");

	return ret;
}

static void __exit bq27410_battery_exit(void)
{
	i2c_del_driver(&bq27410_battery_driver);
}

module_exit(bq27410_battery_exit);
module_init(bq27410_battery_init);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
