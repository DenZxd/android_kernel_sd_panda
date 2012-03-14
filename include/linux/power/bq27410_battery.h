#ifndef __LINUX_BQ27X00_BATTERY_H__
#define __LINUX_BQ27X00_BATTERY_H__

/**
 * struct bq27000_plaform_data - Platform data for bq27000 devices
 * @name: Name of the battery. If NULL the driver will fallback to "bq27000".
 * @read: HDQ read callback.
 *	This function should provide access to the HDQ bus the battery is
 *	connected to.
 *	The first parameter is a pointer to the battery device, the second the
 *	register to be read. The return value should either be the content of
 *	the passed register or an error value.
 */

#define BQ27410_REG_TEMP                0x02
#define BQ27410_REG_VOLT                0x04
#define BQ27410_REG_AI                  0x10
#define BQ27410_REG_FLAGS               0x06
#define BQ27410_REG_NAC                 0x08 /* Nominal available capaciy */
#define BQ27410_REG_LMD                 0x0E /* Last measured discharge */
#define BQ27410_REG_AE                  0x16 /* Available enery */

#define BQ27410_REG_FACAP		0x0a
#define BQ27410_REG_FCCAP		0x0e
#define BQ27410_REG_REMCAP		0x0c

#define BQ27410_REG_RSOC                0x1C /* Relative State-of-Charge */
#define BQ27410_REG_ILMD                0x76 /* Initial last measured discharge */
#define BQ27410_FLAG_CHGS               BIT(7)
#define BQ27410_FLAG_FC                 BIT(9)

#define BQ27410_RS                      20 /* Resistor sense */
#define BQ27410_RESE			10


struct bq27410_device_info;

struct battery_info {
	int Curr;
	int LowCount;
	int FullCount;
	int Power;
	int PrevPower;
	int PollCount;
	int Capacity;
	bool SuspendFlag;
	unsigned int Voltage;
	unsigned int Temp;
	unsigned int Energy;
};

struct bq27410_device_info {
	struct device *dev;
	struct mutex lock;
	struct i2c_client *myclient;
	struct timer_list battery_timer;
	struct work_struct battery_work;
	struct battery_info battery_data;

	int temperature;
	int voltage_now;
	int current_now;
	int capacity;
	int power;
	int FullAvaCap;
	int FullChaCap;
	int RemCap;
	int bat_flag;
	int ControlStatus;

	bool mode;
	u8 control;
	u8 Adreg;
	u16	AAddr;
};
#endif
