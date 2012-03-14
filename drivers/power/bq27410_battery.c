/*
 * BQ27410 battery driver
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
 * http://focus.ti.com/docs/prod/folders/print/bq27410.html
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/skbuff.h>

#include <asm/unaligned.h>

#include <linux/power/bq27410_battery.h>

static int ac_get_property(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val);
static int battery_get_property(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val);
extern int bq2416x_get_charge_status(void);

typedef enum {
    CHARGER_BATTERY = 0,
    CHARGER_AC
} CB;

static enum power_supply_property battery_properties[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property power_properties[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
    "bq27410-battery",
};

static struct power_supply power_supplies[] =
{
	{
		.name = "bq27410-battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = battery_properties,
		.num_properties = ARRAY_SIZE(battery_properties),
		.get_property = battery_get_property,
	},
	{
		.name = "bq27410-ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.properties = power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property = ac_get_property,
	},
};

static struct bq27410_device_info *the_bq27410;

static int bq27410_read_i2c(struct bq27410_device_info *di, u8 reg, bool single)
{
	struct i2c_client *client = di->myclient;
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;
	int count=0;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;

	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	for(count=0; count < 5; count++){
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (ret == ARRAY_SIZE(msg))
			break;
		else
			if(count == 4)
			{
				printk("i2c read err\n");
				return ret;
			}
	}
	//printk("reg = %x val = %x %x\n",reg,data[0],data[1]);
	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];

	return ret;
}

static int _bq27410_read_i2c(struct bq27410_device_info *di, u16 addr, u8 reg, u8 len, u8 *data)
{
	struct i2c_client *client = di->myclient;
	struct i2c_msg msg[2];
	int ret;
	int count = 0;
	u8 msgbuf0[1] = { reg };
	u16 slave = client->addr;
	u16 flags = 0;

	if(addr && (addr != 0x55))
		slave = 0xb;

	msg[0].addr = slave;
	msg[0].flags = flags;
	msg[0].len = 1;
	msg[0].buf = msgbuf0;
	msg[1].addr = slave;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	while(count < 5){
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if(ret == ARRAY_SIZE(msg))
			return ret;
		count++;
	}

	return 0;
}

static int _bq27410_write_i2c(struct bq27410_device_info *di, u16 addr, u8 reg, u8 len, u8 *data)
{
	struct i2c_client *client = di->myclient;
	struct i2c_msg msg[1];
	u16 slave = client->addr;
	u16 flags = 0;
	int count = 0;
	int ret;

	if(addr && (addr != 0x55))
		slave = 0xb;

	msg[0].addr = slave;
	msg[0].flags = flags;
	msg[0].len = len;
	msg[0].buf = data;

	while(count < 5)
	{
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if(ret == ARRAY_SIZE(msg))
			return ret;
		count++;
	}
	return 0;
}

static int bq27410_battery_get_version(struct bq27410_device_info *di)
{
	u8 data[4];

	data[0] = 0x00;
	data[1] = 0x02;
	data[2] = 0x00;

	//mdelay(1000);
	_bq27410_write_i2c(di,0,0x00,3,data);
	mdelay(20);
	_bq27410_read_i2c(di,0,0x00,2,data);
	//printk("FW version = %x.%x\n", data[1],data[0]);

	return (data[1]<<8)|data[0];
}

static int bq27410_GetControlStatus(struct bq27410_device_info *di)
{
	u8 data[4];

	data[0] = 0x00;
	data[1] = 0x00;
	data[2] = 0x00;

	_bq27410_write_i2c(di,0,0x00,3,data);
	mdelay(10);
	_bq27410_read_i2c(di,0,0x00,2,data);

	return (data[1]<<8)|data[0];
}

static int bq27410_GetFlags(struct bq27410_device_info *di)
{
	u8 data[2];
	data[0] = 0x00;
	data[1] = 0x00;

	_bq27410_read_i2c(di,0,0x06,2,data);

	return (data[1]<<8)|data[0];
}

static int bq27410_BatInsert(struct bq27410_device_info *di)
{
	u8 data[4];

	data[0] = 0x00;
	data[1] = 0x0c;
	data[2] = 0x00;
	_bq27410_write_i2c(di,0,0x00,3,&data[0]);

	return 0;
}

static int bq27410_BatRemove(struct bq27410_device_info *di)
{
	u8 data[4];

	data[0] = 0x00;
	data[1] = 0x0d;
	data[2] = 0x00;
	_bq27410_write_i2c(di,0,0x00,3,&data[0]);

	return 0;
}

static int OperationConfiguration(struct bq27410_device_info *di)
{
	u8 data[2];

	_bq27410_read_i2c(di,0,0x3a,2,data);
	printk("reg3a = %x\n",data[1]<<8|data[0]);
	mdelay(10);
//	data[0] |= 0x20;
	data[0] &= 0xdf;
	_bq27410_write_i2c(di,0,0x3a,3,data);
	printk("reg3a = %x\n",data[1]<<8|data[0]);

	//printk("FW version = %x.%x\n", data[1],data[0]);

	return (data[1]<<8)|data[0];

}

static int bq27410_polling(void)
{
	int value;
	struct bq27410_device_info *data = the_bq27410;

	if(data->mode)
		return 0;

	mutex_lock(&data->lock);

	data->control = BQ27410_REG_RSOC;
	value = bq27410_read_i2c(data,data->control,true);
	if(value < 0){
		value = 0;
		printk("%s: error reading capacity\n",__func__);
	}
	if(value > 100)
		value = 100;
	data->capacity = (value - BQ27410_RESE) * 100 / (100 - BQ27410_RESE);
	if(data->capacity <= 0)
		data->capacity = 0;
	if(data->capacity >= 100)
		data->capacity = 100;

	data->control = BQ27410_REG_VOLT;
	value = bq27410_read_i2c(data,data->control,false);
	if(value < 0){
		value = 0;
		printk("%s: error reading voltage \n",__func__);
	}

	if(value > 4200)
		value = 4200;
	data->voltage_now = value;

	data->control = BQ27410_REG_AI;
	value = bq27410_read_i2c(data,data->control,false);
	if(value < 0)
		printk("%s: error reading current \n",__func__);
	value = (int)((s16)value);
	data->current_now = value;

	data->control = BQ27410_REG_TEMP;
	value = bq27410_read_i2c(data,data->control,false);
	if(value < 0)
		printk("%s: error reading temperature \n",__func__);
	value /= 10;
	data->temperature = value - 273;

	data->control = BQ27410_REG_FACAP;
	value = bq27410_read_i2c(data,data->control,false);
	if(value < 0)
		printk("%s: error reading FullAvaCap \n",__func__);
	data->FullAvaCap = value;

	data->control = BQ27410_REG_FCCAP;
	value = bq27410_read_i2c(data,data->control,false);
	if(value < 0)
		printk("%s: error reading FullChaCap \n",__func__);
	data->FullChaCap = value;

	data->control =	BQ27410_REG_REMCAP;
	value = bq27410_read_i2c(data,data->control,false);
	if(value < 0)
		printk("%s: error reading RemCap \n",__func__);
	data->RemCap = value;

	data->control =	BQ27410_REG_FLAGS;
	value = bq27410_read_i2c(data,data->control,false);
	if(value < 0)
		printk("%s: error reading bat_flag \n",__func__);
	data->bat_flag = value;

	value  = bq27410_GetControlStatus(data);
	data->ControlStatus = value;

	mutex_unlock(&data->lock);

	return 0;
}

static int get_battery_info(void)
{
	struct battery_info *bi = &(the_bq27410->battery_data);

	if(!bi->PollCount){
		bi->PollCount = 15;
		bq27410_polling();
		//printk("%4x %4x %4d %4d %4d %4d %4d %4d\n",the_bq27410->ControlStatus,the_bq27410->bat_flag,the_bq27410->voltage_now, \
						the_bq27410->temperature,the_bq27410->current_now,the_bq27410->capacity,the_bq27410->RemCap, \
						the_bq27410->FullChaCap);
	}
	else{
		bi->PollCount--;
		bi->Power = bq2416x_get_charge_status();
		if(bi->Power != bi->PrevPower){
			if((bi->PrevPower = bi->Power) == POWER_SUPPLY_STATUS_FULL){
				the_bq27410->capacity = 100;
			}
			bi->Capacity = the_bq27410->capacity;
			return 1;
		}
		return 0;
	}

	bi->Voltage = the_bq27410->voltage_now;
	bi->Curr = the_bq27410->current_now;
	bi->Power = bq2416x_get_charge_status();

	if(!bi->SuspendFlag){
		bi->SuspendFlag = 1;
		bi->Capacity = the_bq27410->capacity;
		bi->PrevPower = bi->Power;
		return 1;
	}

	if((bi->Voltage <= 3405) && (bi->Power == POWER_SUPPLY_STATUS_NOT_CHARGING)){
		if(bi->LowCount >= 1){
			bi->Capacity = 0;
			bi->LowCount = 0;
			bi->PrevPower = bi->Power;
			bi->SuspendFlag = 0;
			return 1;
		}
		else
			++bi->LowCount;
	}

	if((bi->Voltage >= 3405) && (bi->Power == POWER_SUPPLY_STATUS_NOT_CHARGING) && \
								(the_bq27410->capacity == 0)){
		bi->Capacity = 1;
		bi->PrevPower = bi->Power;
		return 1;
	}

	if((bi->Curr < 200) && (bi->Voltage > 4150) && (bi->Power != POWER_SUPPLY_STATUS_NOT_CHARGING)){
		if(bi->FullCount >= 1){
			bi->FullCount = 0;
			bi->Capacity = 100;
			bi->PrevPower = bi->Power = POWER_SUPPLY_STATUS_FULL;
			return 1;
		}
		else
			++bi->FullCount;
	}

	if(bi->Capacity == the_bq27410->capacity){
		if(bi->PrevPower == bi->Power)
			return 0;
		else{
			if((bi->PrevPower = bi->Power) == POWER_SUPPLY_STATUS_FULL){
				the_bq27410->capacity = 100;
			}
			bi->Capacity = the_bq27410->capacity;
			return 1;
		}
	}
	else
	{
		if(((the_bq27410->capacity - bi->Capacity) > 2) || ((the_bq27410->capacity - bi->Capacity) < -2)){
			if(bi->Power != bi->PrevPower){
				if((bi->PrevPower = bi->Power) == POWER_SUPPLY_STATUS_FULL){
					the_bq27410->capacity = 100;
				}
				return 1;
			}
			else
			{
				bi->Capacity = the_bq27410->capacity;
				return 0;
			}
		}
		else
		{
			if((bi->PrevPower = bi->Power) == POWER_SUPPLY_STATUS_FULL)
				the_bq27410->capacity = 100;
			bi->Capacity = the_bq27410->capacity;
			return 1;
		}
	}
	return 0;
}

static int get_battery_status(void)
{
    int ret;
	struct battery_info *bi = &(the_bq27410->battery_data);

	ret = bi->Power;
	if(bi->Capacity == 100)
		ret = POWER_SUPPLY_STATUS_FULL;

	return ret;
}

static int battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct battery_info *bi = &(the_bq27410->battery_data);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	    val->intval = get_battery_status();
	    break;
	case POWER_SUPPLY_PROP_HEALTH:
	    val->intval = POWER_SUPPLY_HEALTH_GOOD;
	    break;
	case POWER_SUPPLY_PROP_PRESENT:
	    val->intval = 1;
	    break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
	    val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
	    break;
	case POWER_SUPPLY_PROP_CAPACITY:
	    val->intval = bi->Capacity;
	    break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	    val->intval = bi->Voltage;
	    break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	    val->intval = bi->Curr;
	    break;
	default:
	    return -EINVAL;
	}

	return 0;
}
//------------------------------
static int ac_get_property(struct power_supply *psy,
		enum power_supply_property psp,union power_supply_propval *val)
{
	struct battery_info *bi = &(the_bq27410->battery_data);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	    val->intval = (bi->Power == POWER_SUPPLY_STATUS_NOT_CHARGING) ? 0 : 1;
	    break;
	default:
	    return -EINVAL;
	}
	return 0;
}
//--------------------------------
static int do_battery_timer(void *time_data)
{
	mod_timer(&(the_bq27410->battery_timer), jiffies + HZ*2);
	schedule_work(&(the_bq27410->battery_work));
	return 0;
}
//---------------------------------
static void do_work_func(struct work_struct *work)
{
	mutex_init(&(the_bq27410->lock));

	if(get_battery_info()){
		power_supply_changed(&power_supplies[CHARGER_BATTERY]);
		power_supply_changed(&power_supplies[CHARGER_AC]);
	}

	mutex_unlock(&(the_bq27410->lock));
}

static int bq27410_charger_init(struct bq27410_device_info *di)
{
	int err,i;
	struct battery_info *bi = &(the_bq27410->battery_data);

	mutex_lock(&di->lock);

	bi->SuspendFlag = 0;
	bi->PollCount = 0;
	bi->LowCount = 0;
	for (i = 0; i < ARRAY_SIZE(power_supplies); i++){
		err = power_supply_register(di->dev, &power_supplies[i]);
		if (err){
			printk(KERN_ERR "Failed to register power supply (%d)\n", err);
			mutex_unlock(&di->lock);
			return -1;
		}
	}

	init_timer(&(di->battery_timer));
	di->battery_timer.function = &do_battery_timer;
	di->battery_timer.expires = jiffies + HZ*0;

	add_timer(&(di->battery_timer));
	INIT_WORK(&(di->battery_work), do_work_func);

	mutex_unlock(&di->lock);

	return 0;
}

static int bq27410_chip_init(struct bq27410_device_info *di)
{
	int count;
	u16 flag;
	u16 status;

	mutex_lock(&di->lock);
	//OperationConfiguration(di);
	bq27410_BatInsert(di);
	printk("%s: %x\n",__func__,bq27410_battery_get_version(di));
	//msleep(100);
	for(count = 0; count < 5; count++){
		flag = bq27410_GetFlags(di);
		status = bq27410_GetControlStatus(di);
		if((flag & 0x08) && (status & 0x80)){
			printk("%s: the bq27410 electricity meter initialization Control = 0x%x Flag = 0x%x\n",__func__,flag,status);
			break;
		}
		else if(count < 4){
			msleep(200);
			bq27410_BatInsert(di);
		}
		else{
			printk("%s: bq27410 fuel gauge failed to initialize Control = 0x%x Flag = 0x%x\n",__func__,flag,status);
			break;
		}
	}

	mutex_unlock(&di->lock);

	return 0;
}

static int bq27410_suspend(struct device *dev)
{
	struct battery_info *bi = &(the_bq27410->battery_data);

	bi->SuspendFlag = 0;
	bi->PollCount = 0;
	del_timer(&the_bq27410->battery_timer);

	return 0;
}

static int bq27410_resume(struct device *dev)
{
	the_bq27410->battery_timer.expires = jiffies + HZ*0;
	add_timer(&the_bq27410->battery_timer);

    return 0;
}

//------------------------------------------
static int bq27410_proc_read(char* page, char** start, off_t off, int count,int* eof, void* data)
{
	char *kdata;
	int len;
	int i;
	int kcount = count+8;

	if (!(kdata = kmalloc(kcount, GFP_KERNEL)))
		return -ENOMEM;

	len = _bq27410_read_i2c(the_bq27410,the_bq27410->AAddr,the_bq27410->Adreg,(u8)count,kdata);
	if(len){
		for(i=0; i<count; i++)
			*(page+i) = *(kdata+i);
	}
	else{
		printk("%s: _bq27410_read_i2c err reg = %x\n",__func__,the_bq27410->Adreg);
		if(kdata)
			kfree(kdata);
		return -ENOMEM;
	}

	if(kdata)
		kfree(kdata);

	return count;
}

static int bq27410_proc_write(struct file* file, const char* buffer,unsigned long count, void* data)
{
	char *kdata;
	int len;
	int kcount = count+8;

	if (!(kdata = kmalloc(kcount, GFP_KERNEL)))
		return -ENOMEM;

	if(copy_from_user(kdata,buffer,kcount)){
		if(kdata)
			kfree(kdata);
		printk("%s: Implementation of copy_from_user error\n",__func__);
		return -EFAULT;
	}

	if(*(kdata) == 0xff){
		if(*(kdata+1) == 0xff)
		{
			the_bq27410->mode = true;
			del_timer(&the_bq27410->battery_timer);
		}
		else
		{
			the_bq27410->mode = false;
			the_bq27410->battery_timer.expires = jiffies + HZ*5;
			add_timer(&the_bq27410->battery_timer);
		}
		goto loop;
	}

	if(count == 1){
		the_bq27410->AAddr = *(kdata);
		the_bq27410->Adreg = *(kdata+1);
	}
	else{
//		printk("%s: addr = %x reg = %x count = %d data = %x|%x|%x\n",__func__,*(kdata),*(kdata+1),count,*(kdata+2),*(kdata+3),*(kdata+4));
#if 1
		len = _bq27410_write_i2c(the_bq27410,*(kdata),*(kdata+1),count,(kdata+1));
		if(!len){
			printk("%s: _bq27410_write_i2c err reg = %x\n",__func__,*(kdata+1));
			if(kdata)
				kfree(kdata);
			return -ENOMEM;
		}
#endif
	}
loop:
	if(kdata)
		kfree(kdata);

	return count;
}

static int deb27410_proc_read(char* page, char** start, off_t off, int count,int* eof, void* data)
{
	struct bq27410_device_info *da = the_bq27410;
	data = (void *)page;

	bq27410_polling();
//	printk("cap = %d vol = %d cur = %d temp = %d FullAvaCap = %d FullChaCap = %d\n" \
					,da->capacity,da->voltage_now,da->current_now,da->temperature,da->FullAvaCap,da->FullChaCap);
#if 1
	page += sprintf(page,"cap: \t%-4d\t",da->capacity);
	page += sprintf(page,"vol: \t%-4d\t",da->voltage_now);
	page += sprintf(page,"cur: \t%-4d\t",da->current_now);
	page += sprintf(page,"tem: \t%-4d\n",da->temperature);
	page += sprintf(page,"fac: \t%-4d\t",da->FullAvaCap);
	page += sprintf(page,"fcc: \t%-4d\t",da->FullChaCap);
	page += sprintf(page,"rec: \t%-4d\n",da->RemCap);
	page += sprintf(page,"flg: \t%-4x\t",da->bat_flag);
	page += sprintf(page,"cts: \t%-4x\n",da->ControlStatus);
#endif
	return  ((page += sprintf(page, "\n")) - (char*)data);

}

//------------------------------------------
#define BQ27410_PROC_ENTRY "driver/bq27410"
#define BQ27410_PROC_DEB "driver/deb27410"

static int bq27410_proc_init(void)
{
	struct proc_dir_entry* bq27410;
	struct proc_dir_entry* deb27410;

	if (!(bq27410 = create_proc_entry(BQ27410_PROC_ENTRY,S_IRUGO | S_IWUSR, NULL))){
		printk("Proc-FS interface for bq27410 failed\n");
		return -ENOMEM;
	}

	bq27410->read_proc  = bq27410_proc_read;
	bq27410->write_proc = bq27410_proc_write;
	bq27410->data   = NULL;

	if (!(deb27410 = create_proc_entry(BQ27410_PROC_DEB,S_IRUGO | S_IWUSR, NULL))){
		printk("Proc-FS interface for deb27410 failed\n");
		return -ENOMEM;
	}

	deb27410->read_proc = deb27410_proc_read;
	deb27410->write_proc = NULL;
	deb27410->data = NULL;

	return 0;
}

static void bq27410_proc_exit(void)
{
	remove_proc_entry(BQ27410_PROC_ENTRY, NULL);
	remove_proc_entry(BQ27410_PROC_DEB,NULL);
}

//-------------------------------------------------
static int bq27410_battery_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct bq27410_device_info *di;
	int retval = 0;

	/* Get new ID for the new battery device */

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	the_bq27410 = di;
	the_bq27410->myclient = client;
	the_bq27410->mode = false;

	mutex_init(&di->lock);

	bq27410_proc_init();
	bq27410_chip_init(di);
	bq27410_charger_init(di);

	return 0;

batt_failed:

	kfree(di);
	return retval;
}

static int bq27410_battery_remove(struct i2c_client *client)
{
	struct bq27410_device_info *di = i2c_get_clientdata(client);

	bq27410_proc_exit();
	del_timer(&di->battery_timer);
	kfree(di);

	return 0;
}

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
	int ret = i2c_add_driver(&bq27410_battery_driver);
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

MODULE_AUTHOR("hanpin.wu <wuhanpin@hhcn.com>");
MODULE_DESCRIPTION("BQ27410 battery monitor driver");
MODULE_LICENSE("GPL");
