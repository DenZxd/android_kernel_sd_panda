/* OMAP Identity file for OMAP4 boards.
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * Based on
 * mach-omap2/board-44xx-tablet.c
 * mach-omap2/board-4430sdp.c
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/tps6130x.h>
#include "mux.h"
#include <linux/i2c/twl.h>
#include <linux/gpio.h>
#include <linux/cpufreq.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/omap-serial.h>
#include <linux/mfd/twl6040-codec.h>
#include "common-board-devices.h"

static struct regulator_consumer_supply vmmc_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data vmmc = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		}
	},
	.num_consumer_supplies  = ARRAY_SIZE(vmmc_supply),
	.consumer_supplies      = vmmc_supply,
};

static struct regulator_init_data vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data vusim = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_consumer_supply vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

static struct regulator_init_data vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies	= ARRAY_SIZE(vcxio_supply),
	.consumer_supplies	= vcxio_supply,
};

static struct regulator_consumer_supply vdac_supply[] = {
	{
		.supply = "hdmi_vref",
	},
};

static struct regulator_init_data vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = ARRAY_SIZE(vdac_supply),
	.consumer_supplies      = vdac_supply,
};

static struct regulator_consumer_supply vusb_supply[] = {
	REGULATOR_SUPPLY("vusb", "twl6030_usb"),
};

static struct regulator_init_data vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = ARRAY_SIZE(vusb_supply),
	.consumer_supplies      = vusb_supply,
};

static struct regulator_consumer_supply vaux_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1"),
};

static struct regulator_init_data vaux1 = {
	.constraints = {
		.min_uV			= 1000000,
#ifdef CONFIG_VENDOR_HHTECH
		.max_uV			= 3300000,
#else
		.max_uV			= 3000000,
#endif
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
	},
	.num_consumer_supplies  = ARRAY_SIZE(vaux_supply),
	.consumer_supplies      = vaux_supply,
};

static struct regulator_consumer_supply vaux2_supply[] = {
	REGULATOR_SUPPLY("av-switch", "soc-audio"),
#if defined(CONFIG_SMARTQ_S7) || defined(CONFIG_SMARTQ_K7) || \
    defined(CONFIG_SMARTQ_X7) || defined(CONFIG_SMARTQ_T16)
	{ .supply = "s7-tp-power", },
#endif
};

static struct regulator_init_data vaux2 = {
	.constraints = {
#if defined(CONFIG_SMARTQ_S7) || defined(CONFIG_SMARTQ_K7) || \
    defined(CONFIG_SMARTQ_X7) || defined(CONFIG_SMARTQ_T16)
		.always_on		= true,
		.min_uV			= 3000000,
		.max_uV			= 3000000,
#else
		.min_uV			= 1200000,
		.max_uV			= 2800000,
#endif
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		}
	},
	.num_consumer_supplies	= ARRAY_SIZE(vaux2_supply),
	.consumer_supplies	= vaux2_supply,
};

static struct regulator_consumer_supply cam2_supply[] = {
#ifdef CONFIG_VENDOR_HHTECH
#define COMMIX_VCC_ALWAYS_ON 1

	{	.supply = "commix_vcc",		},
#endif
	{
		.supply = "cam2pwr",
	},
};

static struct regulator_init_data vaux3 = {
	.constraints = {
#ifdef COMMIX_VCC_ALWAYS_ON
		.always_on		= true,
		.min_uV			= 3300000,
		.max_uV			= 3300000,
#else
		.min_uV			= 1000000,
		.max_uV			= 3000000,
#endif
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies = ARRAY_SIZE(cam2_supply),
	.consumer_supplies = cam2_supply,
};

/* clk32kg is a twl6030 32khz clock modeled as a regulator, used by WLAN on HHTech Board */
static struct regulator_consumer_supply clk32kg_supply[] = {
	{
		.supply = "clk32kgate",
	},
};
static struct regulator_init_data clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
#if !(defined(CONFIG_BT_BCM43XX) || defined(CONFIG_BT_BCM43XX_MODULE))
		.always_on		= true,
#endif
	},
	.num_consumer_supplies  = ARRAY_SIZE(clk32kg_supply),
	.consumer_supplies      = clk32kg_supply,
};

static struct regulator_init_data clk32kaudio = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_init_data v2v1 = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data sysen = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data regen1 = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data vcore1	= {
	.constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.always_on              = true,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data vcore2	= {
	.constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.always_on              = true,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data vcore3 = {
	.constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data vmem = {
	.constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

#ifndef CONFIG_VENDOR_HHTECH
static int batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925, /* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
	511, 504, 496 /* 60 - 62 */
};
#endif

/*static */struct twl4030_bci_platform_data bci_data = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= 1500,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
#ifndef CONFIG_VENDOR_HHTECH
	.battery_tmp_tbl		= batt_table,
	.tblsize			= ARRAY_SIZE(batt_table),
	.sense_resistor_mohm		= 10,
#endif
};

#ifdef CONFIG_VENDOR_HHTECH
#define GPIO_USB_POWER 151
#define GPIO_USB_VBUS  152

static int hhtech_phy_init(struct device *dev)
{
	if (0) omap_mux_init_signal("mcspi4_clk.gpio_151",
		OMAP_PIN_OUTPUT | OMAP_MUX_MODE3); else
	omap_mux_init_gpio(GPIO_USB_POWER, OMAP_PIN_OUTPUT);
	if (gpio_request(GPIO_USB_POWER, "USB power"))
		pr_err("Fail to request GPIO %d\n", GPIO_USB_POWER);
#ifdef CONFIG_SMARTQ_X7
	else gpio_direction_output(GPIO_USB_POWER, 0);
#else
	else gpio_direction_output(GPIO_USB_POWER, 1);
#endif

	if (0) omap_mux_init_signal("mcspi4_simo.gpio_152",
		OMAP_PIN_OUTPUT | OMAP_MUX_MODE3); else
	omap_mux_init_gpio(GPIO_USB_VBUS, OMAP_PIN_OUTPUT);
	if (gpio_request(GPIO_USB_VBUS, "USB vbus"))
		pr_err("Fail to request GPIO %d\n", GPIO_USB_VBUS);
	else gpio_direction_output(GPIO_USB_VBUS, 0);

	return omap4430_phy_init(dev);
}

static int hhtech_phy_exit(struct device *dev)
{
	gpio_free(GPIO_USB_VBUS);
	gpio_free(GPIO_USB_POWER);

	return omap4430_phy_exit(dev);
}

static int hhtech_phy_power(struct device *dev, int ID, int on)
{
	int value = on && ID;

#ifdef CONFIG_CPU_FREQ
	if (cpu_is_omap443x()) {	// XXX:
	    int cpu = smp_processor_id();
	    struct cpufreq_policy new_policy;
	    struct cpufreq_policy* policy = cpufreq_cpu_get(cpu);
	    extern int __cpufreq_set_policy(struct cpufreq_policy *data,
		    struct cpufreq_policy *policy);

	    if (!policy) goto OUT;
	    if (cpufreq_get_policy(&new_policy, policy->cpu)) {
		cpufreq_cpu_put(policy);
		goto OUT;
	    }

	    if ((ID + on) < 1)
		new_policy.min = policy->user_policy.min; else {
#if 0
		cpufreq_frequency_table* freq_table =
			cpufreq_frequency_get_table(policy->cpu);
		int idx = 1, cur_freq = new_policy.cur;

		new_policy.cur = new_policy.min;
		if (cpufreq_frequency_table_next_highest(new_policy,
			freq_table, &idx)) ;
		new_policy.min = freq_table[idx];
		new_policy.cur = cur_freq;
#else//	XXX:
		//new_policy.min = 600 * 1000;
		new_policy.min <<= 1;
#endif
	    }

	    __cpufreq_set_policy(policy, &new_policy);
	    cpufreq_cpu_put(policy);
OUT:	    ;
	}
#endif

	gpio_set_value(GPIO_USB_VBUS, value);
#ifndef CONFIG_SMARTQ_X7
	gpio_set_value(GPIO_USB_POWER, !value);
#endif

	return omap4430_phy_power(dev, ID, on);
}

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= hhtech_phy_init,
	.phy_exit	= hhtech_phy_exit,
	.phy_power	= hhtech_phy_power,
	.phy_suspend	= omap4430_phy_suspend,
};
#else
static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_suspend	= omap4430_phy_suspend,
};
#endif

static struct twl4030_codec_audio_data twl6040_audio = {
	/* single-step ramp for headset and handsfree */
	.hs_left_step   = 0x0f,
	.hs_right_step  = 0x0f,
	.hf_left_step   = 0x1d,
	.hf_right_step  = 0x1d,
	.vddhf_uV	= 4075000,
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	.max_timeout	= 15000,
	.initial_vibrate = 0,
#ifdef CONFIG_VENDOR_HHTECH
	.voltage_raise_speed = 0x32,
	.channels = TWL6040_VIBRA_CHR,
#else
	.voltage_raise_speed = 0x26,
	.channels = TWL6040_VIBRA_CHR | TWL6040_VIBRA_CHL,
#endif
};

static int twl6040_init(void)
{
	u8 rev = 0;
	int ret;

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE,
				&rev, TWL6040_REG_ASICREV);
	if (ret)
		return ret;

	/*
	 * ERRATA: Reset value of PDM_UL buffer logic is 1 (VDDVIO)
	 * when AUDPWRON = 0, which causes current drain on this pin's
	 * pull-down on OMAP side. The workaround consists of disabling
	 * pull-down resistor of ABE_PDM_UL_DATA pin
	 * Impacted revisions: ES1.1, ES1.2 (both share same ASICREV value),
	 * ES1.3, ES2.0 and ES2.2
	 */
	if ((rev == TWL6040_REV_1_1) ||
	    (rev == TWL6040_REV_1_3) ||
	    (rev == TWL6041_REV_2_0) ||
	    (rev == TWL6041_REV_2_2)) {
		omap_mux_init_signal("abe_pdm_ul_data.abe_pdm_ul_data",
			OMAP_PIN_INPUT);
	}

	return 0;
}

static struct twl4030_codec_data twl6040_codec = {
	.audio          = &twl6040_audio,
	.vibra          = &twl6040_vibra,
	.audpwron_gpio  = 127,
	.naudint_irq    = OMAP44XX_IRQ_SYS_2N,
	.irq_base       = TWL6040_CODEC_IRQ_BASE,
	.init		= twl6040_init,
};

#ifdef CONFIG_TWL6040_UNBOUND
struct i2c_board_info __initdata twl6040_boardinfo = {
	I2C_BOARD_INFO("twl6040", 0x4b),
	.platform_data = &twl6040_codec,
};
#endif

static struct twl4030_madc_platform_data twl6030_gpadc = {
	.irq_line = -1,
};

static struct twl4030_platform_data twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* TWL6030 regulators at OMAP443X/4460 based SOMs */
	.vmmc		= &vmmc,
	.vpp		= &vpp,
	.vusim		= &vusim,
	.vana		= &vana,
	.vcxio		= &vcxio,
	.vdac		= &vdac,
	.vusb		= &vusb,
#ifndef CONFIG_VENDOR_HHTECH
	.vaux1		= &vaux1,
#endif	// XXX:
	.vaux2		= &vaux2,
	.vaux3		= &vaux3,

	/* TWL6032 regulators at OMAP447X based SOMs */
	.ldo1		= &vpp,
#ifndef CONFIG_VENDOR_HHTECH
	.ldo2		= &vaux1,
#endif	// XXX:
	.ldo3		= &vaux3,
	.ldo4		= &vaux2,
	.ldo5		= &vmmc,
	.ldo6		= &vcxio,
	.ldo7		= &vusim,
	.ldoln		= &vdac,
	.ldousb		= &vusb,

	/* TWL6030/6032 common resources */
	.clk32kg	= &clk32kg,
	.clk32kaudio	= &clk32kaudio,

	/* SMPS */
	.vdd1		= &vcore1,
	.vdd2		= &vcore2,
	.v2v1		= &v2v1,

	/* children */
#if defined(CONFIG_TWL6030_BCI_BATTERY) || \
    defined(CONFIG_TWL6030_BCI_BATTERY_MODULE)
	.bci		= &bci_data,
#endif
	.usb		= &omap4_usbphy_data,
#ifndef CONFIG_TWL6040_UNBOUND
	.codec		= &twl6040_codec,
#endif
#if defined(CONFIG_TWL6030_GPADC) || defined(CONFIG_TWL6030_GPADC_MODULE)
	.madc		= &twl6030_gpadc,
#endif

	/* External control pins */
	.sysen		= &sysen,
	.regen1		= &regen1,
};

#ifndef COMMIX_VCC_ALWAYS_ON
void omap4_commix_vcc_power(int action)
{
	static struct regulator *omap4_commix_vcc_reg = NULL;

	if (IS_ERR_OR_NULL(omap4_commix_vcc_reg)) {
		omap4_commix_vcc_reg = regulator_get(NULL, "commix_vcc");
		if (IS_ERR_OR_NULL(omap4_commix_vcc_reg)) {
			pr_err("Can't get commix_vcc for system!\n");
			return;
		}
	}

	if (action < 0) {
		regulator_put(omap4_commix_vcc_reg);
		omap4_commix_vcc_reg = NULL;
	} else
	if (0 < action) regulator_enable(omap4_commix_vcc_reg);
	else regulator_disable(omap4_commix_vcc_reg);
}
#endif

void __init omap4_power_init(void)
{
	/*
	 * VCORE3 & VMEM are not used in 4460. By register it to regulator
	 * framework will ensures that resources are disabled.
	 */
	if (cpu_is_omap446x()) {
		twldata.vdd3 = &vcore3;
		twldata.vmem = &vmem;
	}

	omap4_pmic_init("twl6030", &twldata);
}

