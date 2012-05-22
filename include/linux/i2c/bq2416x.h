/*
 * Copyright (C) 2010 Texas Instruments
 * Author: Balaji T K
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _LINUX_BQ2416X_I2C_H
#define _LINUX_BQ2416X_I2C_H

#define Reg0Add 0x00
#define Reg1Add 0x01
#define Reg2Add 0x02
#define Reg3Add 0x03
#define Reg4Add 0x04
#define Reg5Add 0x05
#define Reg6Add 0x06
#define Reg7Add 0x07

// Variables used on bqSetChgVolt Function
#define VOREG_LSHIFT 2
#define VOREG_MASK 0x03 //Bits Set to 1 on mask will remain unchanged
#define VOREG_STEP 20 // Value in mV
#define VOREG_MAX 4440 // Value in mV
#define VOREG_MIN 3500 // Value in mV

// Variables used on bqSetInCurLimit Function
#define IN_ILIM_1 1500 //value in mA
#define IN_ILIM_2 2500 //value in mA
#define IN_ILIM_MASK 0xFD //Bits Set to 1 on mask will remain unchanged
#define IN_ILIM_LSHIFT 1

// Variables used on bqDpDmDetect Function
#define DP_DM_DETECT_MASK 0xFE //Bits Set to 1 on mask will remain unchanged
#define DP_DM_DETECT_ENABLE_BIT 1
#define DP_DM_DETECT_LSHIFT 0

// Variables used on bqSetSupplyPriority Function
#define IN_SUPPLY 0
#define USB_SUPPLY 1
#define SUPPLY_PRIORITY_MASK 0xF7 //Bits Set to 1 on mask will remain unchanged
#define SUPPLY_PRIORITY_BIT 0x08

// Variables used on bqWatchDogRst Function
#define WATCH_DOG_RST 1
#define WATCH_DOG_RST_LSHIFT 7
#define WATCH_DOG_RST_MASK 0x7F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqUsbLockout Function
#define USB_LOCK 1
#define USB_UNLOCK 0
#define USB_LOCKOUT_LSHIFT 3
#define USB_LOCKOUT_MASK 0xF7 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqNoBatOp Function
#define NO_BAT_OP_EN 1
#define NO_BAT_OP_DIS 0
#define NO_BAT_OP_LSHIFT 0
#define NO_BAT_OP_MASK 0xFE //Bits Set to 1 on mask will remain unchanged

// Variables used on bqHiZMode Function
#define HI_Z_EN 1
#define HI_Z_DIS 0
#define HI_Z_LSHIFT 0
#define HI_Z_MASK 0x7E //Bits Set to 1 on mask will remain unchanged

// Variables used on bqChgEnable Function
#define CHG_EN 0
#define CHG_DIS 1
#define CHG_EN_LSHIFT 1
#define CHG_EN_MASK 0x7D //Bits Set to 1 on mask will remain unchanged

// Variables used on bqEnableChgCurTerm Function
#define CHG_CUR_TERM_EN 1
#define CHG_CUR_TERM_DIS 0
#define CHG_CUR_TERM_EN_LSHIFT 2
#define CHG_CUR_TERM_EN_MASK 0x7B //Bits Set to 1 on mask will remain unchanged

// Variables used on bqEnableStat Function
#define STAT_EN 1
#define STAT_DIS 0
#define STAT_EN_LSHIFT 3
#define STAT_EN_MASK 0x77 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetUsbCurLimit Function
#define USB2_H_100MA 0 //000-USB2.0 host with 100mA current limit
#define USB3_H_150MA 1 //001-USB3.0 host with 150mA current limit
#define USB2_H_500MA 2 //010 – USB2.0 host with 500mA current limit
#define USB_HC_800MA 3 //011 – USB host/charger with 800mA current limit
#define USB3_H_900MA 4 //100 – USB3.0 host with 900mA current limit
#define USB_HC_1500MA 5 //101 – USB host/charger with 1500mA current limit
#define USB_CUR_LIMIT_LSHIFT 4
#define USB_CUR_LIMIT_MASK 0x0F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqAllRegReset Function
#define RESET 1
#define RESET_LSHIFT 7
#define RESET_MASK 0x7F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetChgCur Function
#define ICHG_MIN 550 //value in mA
#define ICHG_MAX 2500 //value in mA
#define ICHG_STEP 75 //value in mA
#define ICHG_LSHIFT 3
#define ICHG_MASK 0x07 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetTermCur Function
#define ITERM_MIN 50 //value in mA
#define ITERM_MAX 400 //value in mA
#define ITERM_STEP 50 //value in mA
#define ITERM_LSHIFT 0
#define ITERM_MASK 0xF8 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetUsbVdpm Function
#define USB_VDPM_MIN 4200 //value in mA
#define USB_VDPM_MAX 4760 //value in mA
#define USB_VDPM_STEP 80 //value in mA
#define USB_VDPM_LSHIFT 3
#define USB_VDPM_MASK 0xC7 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetInVdpm Function
#define IN_VDPM_MIN 4200 //value in mA
#define IN_VDPM_MAX 4760 //value in mA
#define IN_VDPM_STEP 80 //value in mA
#define IN_VDPM_LSHIFT 0
#define IN_VDPM_MASK 0xF8 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqEnable2xSlowTimer Function
#define SLOW_TIMER_EN 1
#define SLOW_TIMER_DIS 0
#define SLOW_TIMER_LSHIFT 7
#define SLOW_TIMER_MASK 0x7F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetSafteyTimer Function
#define FAST_CHG_27_MIN 0
#define FAST_CHG_6_HR 1
#define FAST_CHG_9_HR 2
#define DISABLE 3
#define SAFETY_TIMER_LSHIFT 5
#define SAFETY_TIMER_MASK 0x9F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqThermalShutdown Function
#define TS_EN 1
#define TS_DIS 0
#define TS_LSHIFT 3
#define TS_MASK 0xF7 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqLowChg Function
#define LOW_CHG_EN 1
#define LOW_CHG_DIS 0
#define LOW_CHG_LSHIFT 0
#define LOW_CHG_MASK 0xFE //Bits Set to 1 on mask will remain unchanged

#define BQ2416x_START_CHARGING		(1 << 0)
#define BQ2416x_STOP_CHARGING		(1 << 1)
#define BQ2416x_CHARGER_FAULT		(1 << 2)

#define BQ2416x_CHARGE_DONE		0x20
#define BQ2416x_FAULT_VBUS_OVP		0x31
#define BQ2416x_FAULT_SLEEP		0x32
#define BQ2416x_FAULT_BAD_ADAPTOR	0x33
#define BQ2416x_FAULT_BAT_OVP		0x34
#define BQ2416x_FAULT_THERMAL_SHUTDOWN	0x35
#define BQ2416x_FAULT_TIMER		0x36
#define BQ2416x_FAULT_NO_BATTERY	0x37

/* not a bq generated event,we use this to reset the
 * the timer from the twl driver.
 */
#define BQ2416x_RESET_TIMER		0x38

/* BQ24153 / BQ24156 / BQ24158 */
/* Status/Control Register */
#define REG_STATUS_CONTROL		0x00
#define		TIMER_RST		(1 << 7)

/* Control Register */
#define REG_CONTROL_REGISTER		0x01
#define	INPUT_CURRENT_LIMIT_SHIFT	6
#define	ENABLE_ITERM_SHIFT		3

/* Control/Battery Voltage Register */
#define REG_BATTERY_VOLTAGE		0x02
#define	VOLTAGE_SHIFT			2

/* Vender/Part/Revision Register */
#define REG_PART_REVISION		0x04

/* Battery Termination/Fast Charge Current Register */
#define REG_BATTERY_CURRENT		0x04
#define	BQ24156_CURRENT_SHIFT		3
#define	BQ24153_CURRENT_SHIFT		4

/* Special Charger Voltage/Enable Pin Status Register */
#define REG_SPECIAL_CHARGER_VOLTAGE	0x05

/* Safety Limit Register */
#define REG_SAFETY_LIMIT		0x06
#define	MAX_CURRENT_SHIFT		4

#define BQ2416x (1 << 3)

#define BQ2416x_WATCHDOG_TIMEOUT	20000

#define SLEEP_MODE 0
#define WORK_MODE 1

struct bq2416x_platform_data {
	int cin_limit;
	int work_current;
	int standby_current;
	int charger_current;
	int in_dpm;
	int max_charger_voltagemV;
	int termination_currentmA;
};

#endif
