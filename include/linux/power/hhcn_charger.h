
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/input.h>	/* Input device layer */

struct hhcn_battery_info {
    int Curr;
	int LowCount;
    int Power;
	int PrevPower;
	int PollCount;
	int Capacity;
	bool SuspendFlag;
    unsigned int Voltage;
    unsigned int Temp;
    unsigned int Energy;
};

struct pltdata_charger {
	void (*charger_crt)(int);
	int (*charger_sts)(void);
};

struct struct_battery_data {
    struct hhcn_battery_info battery_data;
    struct timer_list battery_timer;
    struct mutex battery_lock;
    struct work_struct battery_work;
	struct pltdata_charger *pdata_chg;
};

extern struct struct_batt_data batt_info;
static int hhcn_ac_get_property(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val);
static int hhcn_battery_get_property(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val);
