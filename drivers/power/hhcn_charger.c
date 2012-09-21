
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/slab.h>

#include <linux/power/hhcn_charger.h>
#include "oz8806.h"

#define TIME_POLL 14
#define MIN_VOLT 3400
//----------------------------------------------------------

typedef enum {
    CHARGER_BATTERY = 0,
    CHARGER_AC
}CB;

static enum power_supply_property hhcn_battery_properties[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property hhcn_power_properties[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
    "battery",
};

static struct power_supply hhtech_power_supplies[] = {
{
    .name = "battery",
    .type = POWER_SUPPLY_TYPE_BATTERY,
    .properties = hhcn_battery_properties,
    .num_properties = ARRAY_SIZE(hhcn_battery_properties),
    .get_property = hhcn_battery_get_property,
},
{
    .name = "ac",
    .type = POWER_SUPPLY_TYPE_MAINS,
    .supplied_to = supply_list,
    .num_supplicants = ARRAY_SIZE(supply_list),
    .properties = hhcn_power_properties,
    .num_properties = ARRAY_SIZE(hhcn_power_properties),
    .get_property = hhcn_ac_get_property,
},
};

static struct struct_battery_data *hhcn_battery_data;

#if defined(CONFIG_CHARGER_BQ2416X)
extern int bq2416x_get_charge_status(void);
static int battery_ac_status()
{
	int status;
	int i;

	for(i=0; i < 4; i++)
	{
		status = bq2416x_get_charge_status();
		if ((status == POWER_SUPPLY_STATUS_CHARGING) || (status == POWER_SUPPLY_STATUS_FULL) )
			return 1;
		else if (status == POWER_SUPPLY_STATUS_NOT_CHARGING)
			return 0;
		else{
			printk("%s: Read bq24161 battery ac status err\n",__func__);
			msleep(100);
		}
	}
	return OZ8806_PowerLoop();
}
#endif

static int hhcn_get_battery_info(void)
{
	struct hhcn_battery_info *bi = &(hhcn_battery_data->battery_data);
#if defined(CONFIG_CHARGER_BQ2416X)
	bi->Power = battery_ac_status();
#else
	struct pltdata_charger *pd = hhcn_battery_data->pdata_chg;
	bi->Power = pd->charger_sts();

#endif
	if(!bi->PollCount)
	{
		bi->PollCount = TIME_POLL;
		OZ8806_PollingLoop(bi->Power);
	}
	else
	{
		bi->PollCount--;
	}
	bi->Voltage = batt_info.fVolt;
	bi->Curr = batt_info.fCurr;
	if(!bi->SuspendFlag)
	{
		int count;
		if(batt_info.fRSOC <= 0){
			for(count = 0; count < 5; count++){
				OZ8806_PollingLoop(bi->Power);
				if(batt_info.fRSOC > 0)
					break;
				msleep(800);
			}
		}
		bi->Capacity = batt_info.fRSOC;
		bi->PrevPower = bi->Power;
		bi->SuspendFlag = 1;
		return 1;
	}
	if((bi->Voltage <= MIN_VOLT) && (!bi->Power) && (bi->PollCount == TIME_POLL))
	{
		if(bi->LowCount >= 3)
		{
			bi->Capacity = 0;
			bi->LowCount = 0;
			bi->PrevPower = bi->Power;
			bi->SuspendFlag = 0;
			return 1;
		}
		else
			++bi->LowCount;
	}
	if((bi->Voltage >= MIN_VOLT) && (!bi->Power) && (batt_info.fRSOC == 0))
	{
		bi->Capacity = 1;
		bi->PrevPower = bi->Power;
		return 1;
	}
	if(bi->Capacity == batt_info.fRSOC)
	{
		if(bi->Power != bi->PrevPower)
		{
			bi->Capacity = batt_info.fRSOC;
			bi->PrevPower = bi->Power;
			return 1;
		}
		return 0;
	}
	else
	{
		/*if(((batt_info.fRSOC - bi->Capacity) > 2) || ((batt_info.fRSOC - bi->Capacity) < -2))
		{
			if(bi->Power != bi->PrevPower)
			{
				bi->PrevPower = bi->Power;
				return 1;
			}
			else
				return 0;
		}
		else*/
		{
			bi->PrevPower = bi->Power;
			bi->Capacity = batt_info.fRSOC;
			return 1;
		}
	}
}

//--------------------------------
int hhcn_get_battery_start(void)
{
    int ret;
    struct hhcn_battery_info *bi = &(hhcn_battery_data->battery_data);

    if(bi->Power)
	    if(bi->Capacity != 100)
		    ret = POWER_SUPPLY_STATUS_CHARGING;
	    else
		    ret = POWER_SUPPLY_STATUS_FULL;
    else
	    ret = POWER_SUPPLY_STATUS_NOT_CHARGING;

    return ret;
}
EXPORT_SYMBOL(hhcn_get_battery_start);
//--------------------------------
static void hhcn_battery_timer(unsigned long time_data)
{
	mod_timer(&(hhcn_battery_data->battery_timer), jiffies + HZ*2);
	schedule_work(&(hhcn_battery_data->battery_work));
}
//---------------------------------
static int hhcn_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct hhcn_battery_info *bi = &(hhcn_battery_data->battery_data);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	    val->intval = hhcn_get_battery_start();
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
static int hhcn_ac_get_property(struct power_supply *psy,
		enum power_supply_property psp,union power_supply_propval *val)
{
	struct hhcn_battery_info *bi = &(hhcn_battery_data->battery_data);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	    val->intval = bi->Power;
	    break;
	default:
	    return -EINVAL;
	}
	return 0;
}
//-------------------------------
static void hhcn_work_func(struct work_struct *work)
{
	mutex_lock(&(hhcn_battery_data->battery_lock));
	if(hhcn_get_battery_info()){
		power_supply_changed(&hhtech_power_supplies[CHARGER_BATTERY]);
	}
	power_supply_changed(&hhtech_power_supplies[CHARGER_AC]);
	mutex_unlock(&(hhcn_battery_data->battery_lock));
}
//-------------------------------
static int __devinit hhcn_battery_probe(struct platform_device *pdev)
{
	int i, err;
	struct hhcn_battery_info *bi;

	hhcn_battery_data = kmalloc(sizeof(*hhcn_battery_data),GFP_KERNEL);
	if (!hhcn_battery_data) {
	    printk(KERN_ERR "%s : malloc failed\n",__func__);
	    return -1;
	}
	bi = &(hhcn_battery_data->battery_data);
#if defined(CONFIG_CHARGER_BQ2416X)
	bi->Power = battery_ac_status();
#else
	hhcn_battery_data->pdata_chg = pdev->dev.platform_data;
	bi->Power = hhcn_battery_data->pdata_chg->charger_sts();
#endif
	bi->SuspendFlag = 0;
	bi->PollCount = 0;
	bi->LowCount = 0;

	mutex_init(&(hhcn_battery_data->battery_lock));

	for (i = 0; i < ARRAY_SIZE(hhtech_power_supplies); i++) {
		err = power_supply_register(&pdev->dev, &hhtech_power_supplies[i]);
		if (err)
			printk(KERN_ERR "Failed to register power supply (%d)\n", err);
	}
	init_timer(&(hhcn_battery_data->battery_timer));
	hhcn_battery_data->battery_timer.function = &hhcn_battery_timer;
	hhcn_battery_data->battery_timer.expires = jiffies + HZ*1;
	add_timer(&(hhcn_battery_data->battery_timer));
	INIT_WORK(&(hhcn_battery_data->battery_work), hhcn_work_func);
	power_supply_changed(&hhtech_power_supplies[CHARGER_BATTERY]);

	return 0;
}
//-----------------------------
static int hhcn_battery_remove(struct platform_device *pdev)
{
	if (hhcn_battery_data)
	    kfree(hhcn_battery_data);
	return 0;
}
//-----------------------------
#ifdef CONFIG_PM
static int hhcn_bat_suspend(struct device *dev)
{

	struct hhcn_battery_info *bi = &(hhcn_battery_data->battery_data);
#if !defined(CONFIG_CHARGER_BQ2416X)
	struct pltdata_charger *pd = hhcn_battery_data->pdata_chg;
	if(!pd->charger_sts())
		pd->charger_crt(0);
#endif

	bi->SuspendFlag = 0;
	bi->PollCount = 0;
	del_timer(&(hhcn_battery_data->battery_timer));
	return 0;
}

static int hhcn_bat_resume(struct device *dev)
{

#if !defined(CONFIG_CHARGER_BQ2416X)
	struct pltdata_charger *pd = hhcn_battery_data->pdata_chg;
	pd->charger_crt(1);
#endif
    hhcn_battery_data->battery_timer.function = &hhcn_battery_timer;
    hhcn_battery_data->battery_timer.expires = jiffies;
    add_timer(&(hhcn_battery_data->battery_timer));
    return 0;
}

static const struct dev_pm_ops charger_ops = {
	.suspend = hhcn_bat_suspend,
	.resume = hhcn_bat_resume,
};
#endif
//----------------------------

static struct platform_driver hhcn_bat_driver = {
	.probe	= hhcn_battery_probe,
	.remove	= __devexit_p(hhcn_battery_remove),
	.driver	=	{
        .name	= "hhcn-charger",
#ifdef CONFIG_PM
		.pm     = &charger_ops,
#endif
		.owner	= THIS_MODULE,
	},
};

//---------------------------
static int __init hhcn_battery_init(void)
{
	return platform_driver_register(&hhcn_bat_driver);
}
//--------------------------
static void __exit hhcn_battery_exit(void)
{
	platform_driver_unregister(&hhcn_bat_driver);
}


late_initcall(hhcn_battery_init);
module_exit(hhcn_battery_exit);
MODULE_DESCRIPTION("HHcn Battery Driver");
MODULE_AUTHOR("hanpin.wu, Inc");
MODULE_LICENSE("GPL");
