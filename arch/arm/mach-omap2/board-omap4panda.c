/*
 * Board support file for OMAP4430 based PandaBoard.
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Author: David Anders <x0132446@ti.com>
 *
 * Based on mach-omap2/board-4430sdp.c
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/omapfb.h>
#include <linux/i2c/twl.h>
#include <linux/platform_device.h>
#include <linux/mfd/twl6040-codec.h>
#include <linux/regulator/tps6130x.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/pwm_backlight.h>
#include <linux/wl12xx.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/memblock.h>
#include <linux/gpio_keys.h>

#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/omap4-common.h>
#include <mach/dmm.h>
#include <mach/omap4_ion.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/android-display.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/pwm.h>
#include <plat/omap-pm.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/omap-serial.h>
#include <plat/remoteproc.h>
#include <plat/vram.h>

#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>

#include "timer-gp.h"
#include "board-panda.h"
#include "omap_ram_console.h"
#include "control.h"
#include "hsmmc.h"
#include "mux.h"
#include "pm.h"

#define GPIO_HUB_POWER		1
#define GPIO_HUB_NRESET		62
#define GPIO_WIFI_IRQ		53
#define HDMI_GPIO_CT_CP_HPD 60 /* HPD mode enable/disable */
#define HDMI_GPIO_LS_OE 41 /* Level shifter for HDMI */
#define HDMI_GPIO_HPD  63 /* Hotplug detect */
#ifdef CONFIG_VENDOR_HHTECH
#define TPS62361_GPIO   8
#define GPIO_WIFI_PMENA		40
#else
#define GPIO_WIFI_PMENA		43
#define TPS62361_GPIO   7 /* VCORE1 power control */
#endif

#if defined(CONFIG_PANEL_LG_IPS_7)
static struct platform_device lg_ips7_panel = {
   .name = "lg_ips7_panel",
   .id = -1,
};
#endif

#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
#define WILINK_BT_RESET     42
#define WILINK_UART_DEV_NAME    "/dev/ttyO1"
/* TODO: handle suspend/resume here.
 * Upon every suspend, make sure the wilink chip is
 * capable enough to wake-up the OMAP host.
 */
static int plat_wlink_kim_suspend(struct platform_device *pdev, pm_message_t
		state)
{
	return 0;
}

static int plat_wlink_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static bool uart_req;
static struct wake_lock st_wk_lock;
/* Call the uart disable of serial driver */
static int plat_uart_disable(void)
{
	int port_id = 0;
	int err = 0;
	if (uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_disable(port_id);
		if (!err)
			uart_req = false;
	}
	wake_unlock(&st_wk_lock);
	return err;
}

/* Call the uart enable of serial driver */
static int plat_uart_enable(void)
{
	int port_id = 0;
	int err = 0;
	if (!uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_enable(port_id);
		if (!err)
			uart_req = true;
	}
	wake_lock(&st_wk_lock);
	return err;
}

/* wl128x BT, FM, GPS connectivity chip */
static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = WILINK_BT_RESET,
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3686400,
	.suspend = plat_wlink_kim_suspend,
	.resume = plat_wlink_kim_resume,
	.chip_asleep = plat_uart_disable,
	.chip_awake  = plat_uart_enable,
	.chip_enable = plat_uart_enable,
	.chip_disable = plat_uart_disable,
};

static struct platform_device wl128x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};
#endif

static struct gpio_led gpio_leds[] = {
	{
		.name			= "pandaboard::status1",
		.default_trigger	= "heartbeat",
		.gpio			= 7,
	},
	{
		.name			= "pandaboard::status2",
		.default_trigger	= "mmc0",
		.gpio			= 8,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

#define GPIO_BL_PWM	94
#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
#ifdef CONFIG_OMAP_PWM
#define BL_PWM_GPT_ID	10
static struct omap2_pwm_platform_config pwm_config = {
	.timer_id   = BL_PWM_GPT_ID,
#if defined(CONFIG_SMARTQ_T20)
	.polarity   = 0,
#else
	.polarity   = 1,	// Active-high
#endif
};

static struct platform_device pwm_device = {
	.name	    = "omap-pwm",
	.id	    = 0,
	.dev	    = {
		.platform_data  = &pwm_config
	}
};

static int smartq_bl_init(struct device *dev)
{
    //omap_mux_init_signal("abe_dmic_din3.dmtimer9_pwm_evt", OMAP_MUX_MODE5);
    omap_mux_init_signal("usbb1_ulpitll_dat6.dmtimer10_pwm_evt",
	    OMAP_MUX_MODE1);
    if (0 && gpio_request(GPIO_BL_PWM, "backlight_pwm"))
	pr_err("Cannot request GPIO %d\n", GPIO_BL_PWM);
    return 0;
}
#endif

static struct platform_pwm_backlight_data smartq_backlight_data = {
#ifdef CONFIG_OMAP_PWM
	.init		= smartq_bl_init,
	.pwm_id		= BL_PWM_GPT_ID,
#else
	.pwm_id		= 2,
#endif
	.max_brightness	= 255,
	.dft_brightness	= 100,
	.pwm_period_ns	= 1000000000 / (32000),
};

static struct platform_device smartq_backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.platform_data = &smartq_backlight_data,
	},
};
#endif

#if defined(CONFIG_SWITCH_GPIO) || defined(CONFIG_SWITCH_GPIO_MODULE)
#include <linux/switch.h>
#define GPIO_KEY_LOCK		171

static struct gpio_switch_platform_data ts_key_switch_data = {
	.name		= "touch_key_lock",
	.gpio		= GPIO_KEY_LOCK,
	.active_low	= 0,
	//.debounce_interval = 20,
};

static struct platform_device ts_key_switch_device = {
	.name	= "switch-gpio",
	.dev.platform_data = &ts_key_switch_data,
};

static void __init omap4_gpio_switch_init(void)
{
	omap_mux_init_signal("kpd_col3.gpio_171",
			OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE3);
}

static int omap4_gpio_switch_state(void)
{
	return gpio_get_value(GPIO_KEY_LOCK);
}
#else
static void __init omap4_gpio_switch_init(void) { }
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_KEY_VOLUP		103
#define GPIO_KEY_VOLDOWN	101

static struct gpio_keys_button btn_button_table[] = {
	[0] = {
#if defined(CONFIG_SMARTQ_T20) || defined(CONFIG_SMARTQ_Q8) || \
    defined(CONFIG_SMARTQ_X7)
		.code           = KEY_VOLUMEUP,
#else
		.code           = KEY_VOLUMEDOWN,
#endif
		.gpio           = GPIO_KEY_VOLDOWN,
		.active_low     = 1,
		.desc           = "vol-",
		.type           = EV_KEY,
		.wakeup         = 0,
		//.debounce_interval = 10,
	},
	[1] = {
#if defined(CONFIG_SMARTQ_T20) || defined(CONFIG_SMARTQ_Q8) || \
    defined(CONFIG_SMARTQ_X7)
		.code           = KEY_VOLUMEDOWN,
#else
		.code           = KEY_VOLUMEUP,
#endif
		.gpio           = GPIO_KEY_VOLUP,
		.active_low     = 1,
		.desc           = "vol+",
		.type           = EV_KEY,
		.wakeup         = 0,
		//.debounce_interval = 10,
	},
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons  = btn_button_table,
	.nbuttons = ARRAY_SIZE(btn_button_table),
};

static struct platform_device gpio_keys = {
	.name = "gpio-keys",
	.id   = -1,
	.dev  = {
		.platform_data = &gpio_keys_data,
	},
};

static void __init omap4_gpio_keys_init(void)
{
    if (0) {
	int i;
	for (i = 0; i < ARRAY_SIZE(btn_button_table); ++i)
		omap_mux_init_gpio(btn_button_table[i].gpio,
			OMAP_PIN_INPUT_PULLUP);
	return;
    }

	omap_mux_init_signal("gpmc_ncs4.gpio_101",
		OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE3);
	omap_mux_init_signal("gpmc_ncs6.gpio_103",
		OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE3);
}
#else
static void __init omap4_gpio_keys_init(void) { }
#endif

#if defined(CONFIG_SMARTQ_ENCRYPT) || defined(CONFIG_SMARTQ_ENCRYPT_MODULE)
#define GPIO_ENCRYPT_SCL 44
#define GPIO_ENCRYPT_SDA 45

static void __init omap_encrypt_io_init(void)
{
    if (0) {
	omap_mux_init_signal("gpmc_a20.gpio_44",
		OMAP_PIN_INPUT | OMAP_MUX_MODE3);
	omap_mux_init_signal("gpmc_a21.gpio_45",
		OMAP_PIN_INPUT | OMAP_MUX_MODE3);
    } else {
	omap_mux_init_gpio(GPIO_ENCRYPT_SCL, OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_ENCRYPT_SDA, OMAP_PIN_INPUT);
    }
}
#else
static void __init omap_encrypt_io_init(void) { }
#endif

#if defined(CONFIG_CHARGER_HHCN) || defined(CONFIG_CHARGER_HHCN_MODULE)
#include <linux/power/hhcn_charger.h>

#ifdef CONFIG_CHARGER_BQ2416X
static void __init omap_charger_io_init(void) { }
#else
#define GPIO_CHARGER 137
#define GPIO_CHARGER_DETECT 95

static void __init omap_charger_io_init(void)
{
	int ret;

	omap_mux_init_gpio(GPIO_CHARGER, OMAP_PIN_OUTPUT);
	ret = gpio_request(GPIO_CHARGER, "charger ctrl");
	if (ret) {
		pr_err("Fail to request GPIO %d\n", GPIO_CHARGER);
		return ;
	}
	gpio_direction_output(GPIO_CHARGER, 1);

	omap_mux_init_gpio(GPIO_CHARGER_DETECT, OMAP_PIN_INPUT_PULLUP);
	ret = gpio_request(GPIO_CHARGER_DETECT, "charger detect");
	if (ret) {
		pr_err("Fail to request GPIO %d\n", GPIO_CHARGER_DETECT);
		return ;
	}
}

static void omap_charger_control(int val)
{
	if (val)
		gpio_direction_output(GPIO_CHARGER, 1);
	else
		gpio_direction_output(GPIO_CHARGER, 0);
}

static int omap_charger_get_status(void)
{
	return !gpio_get_value(GPIO_CHARGER_DETECT);
}

static struct pltdata_charger pdata_charger = {
	.charger_crt = omap_charger_control,
	.charger_sts = omap_charger_get_status,
};
#endif

static struct platform_device hhcn_charger = {
	.name	= "hhcn-charger",
	.id	= -1,
#ifndef CONFIG_CHARGER_BQ2416X
	.dev  = {
		.platform_data = &pdata_charger,
	},
#endif
};
#else
static void __init omap_charger_io_init(void) { }
#endif

#if defined(CONFIG_BT_BCM43XX) || defined(CONFIG_BT_BCM43XX_MODULE)
#include <linux/bcm43xx-bluetooth.h>
#include <mach/id.h>

#define BT_RESET_GPIO 42
#define BT_WAKE_GPIO  49
#define BT_HOST_WAKE_GPIO 46

/*
 * this uses the unique per-cpu info from the cpu fuses set at factory to
 * generate a 6-byte MAC address.  Two bits in the generated code are used
 * to elaborate the generated address into four, so it can be used on multiple
 * network interfaces.
 *
 * adopted from u-boot by mhfan
 */
static int omap4_die_id_to_ethernet_mac(u8 *mac, int subtype)
{
	unsigned idcode;
	struct omap_die_id odi;
	omap_get_die_id(&odi);

	idcode  = odi.id_0;
	idcode ^= odi.id_1;

	mac[0] = idcode >> 4;
	mac[1] = idcode >> 12;
	mac[2] = idcode >> 20;

	idcode  = odi.id_2;
	idcode ^= odi.id_3;

	mac[3] = idcode >> 4;
	mac[4] = idcode >> 12;
	mac[5] = idcode >> 20;

	/* allow four MACs from this same basic data */
	mac[1] = (mac[1] & ~0xc0) | ((subtype & 3) << 6);

	/* mark it as not multicast and outside official 80211 MAC namespace */
	mac[0] = (mac[0] & ~1) | 2;

	return 0;
}

static void bcm43xx_bluetooth_set_uart(int enable)
{
	static int bt_uart_enable = 0;

	if (bt_uart_enable == enable)
		return ;

	if (enable) omap_uart_enable(2); else
		    omap_uart_disable(2);

	bt_uart_enable = enable;
}

static struct bcm43xx_bt_platform_data bcm43xx_bluetooth_data = {
	.reset_gpio = BT_RESET_GPIO,
	.wake_gpio  = BT_WAKE_GPIO,
	.host_wake_gpio = BT_HOST_WAKE_GPIO,
	.set_uart  = bcm43xx_bluetooth_set_uart,
	.get_addr   = omap4_die_id_to_ethernet_mac,
	.wake_peer = NULL,
};

static struct platform_device bcm43xx_bluetooth = {
	.dev.platform_data = &bcm43xx_bluetooth_data,
	.name   = "bcm43xx_bluetooth",
	.id     = -1,
};

static void __init bcm43xx_bluetooth_io_init(void)
{
	omap_mux_init_gpio(BT_RESET_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(BT_WAKE_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(BT_HOST_WAKE_GPIO, OMAP_PIN_INPUT);
}

static void bcm43xx_wake_peer(struct uart_port *uport)
{
	if (bcm43xx_bluetooth_data.wake_peer)
		bcm43xx_bluetooth_data.wake_peer(uport);
}
#else
static void __init bcm43xx_bluetooth_io_init(void)
{
#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
        omap_mux_init_gpio(WILINK_BT_RESET, OMAP_PIN_OUTPUT);
#endif
}
#endif

static struct platform_device *panda_devices[] __initdata = {
#ifdef CONFIG_PANEL_LG_IPS_7
	&lg_ips7_panel,
#endif
#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
#ifdef CONFIG_OMAP_PWM
	&pwm_device,
#endif
	&smartq_backlight_device,
#endif
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	&gpio_keys,
#endif
#if defined(CONFIG_CHARGER_HHCN) || defined(CONFIG_CHARGER_HHCN_MODULE)
	&hhcn_charger,
#endif
#ifndef CONFIG_VENDOR_HHTECH
	&leds_gpio,
#endif
#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
	&wl128x_device,
	&btwilink_device,
#endif
#if defined(CONFIG_SWITCH_GPIO) || defined(CONFIG_SWITCH_GPIO_MODULE)
	&ts_key_switch_device,
#endif
#if defined(CONFIG_BT_BCM43XX) || defined(CONFIG_BT_BCM43XX_MODULE)
	&bcm43xx_bluetooth,
#endif
};

static void __init omap4_panda_init_early(void)
{
	extern unsigned long lpj_fine;
	char *mpu_clk_name;

	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif

	//if (cpu_is_omap24xx()) mpu_clk_name = "virt_prcm_set"; else
	//if (cpu_is_omap34xx()) mpu_clk_name = "dpll1_ck"; else
	if (cpu_is_omap443x()) mpu_clk_name = "dpll_mpu_ck"; else
	if (cpu_is_omap446x() || cpu_is_omap447x())
			       mpu_clk_name = "virt_dpll_mpu_ck";

	lpj_fine = clk_get_rate(clk_get(NULL, mpu_clk_name)) / HZ;
}

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static struct gpio panda_ehci_gpios[] __initdata = {
	{ GPIO_HUB_POWER,	GPIOF_OUT_INIT_LOW,  "hub_power"  },
	{ GPIO_HUB_NRESET,	GPIOF_OUT_INIT_LOW,  "hub_nreset" },
};

static void __init omap4_ehci_init(void)
{
	int ret;
	struct clk *phy_ref_clk;

	/* FREF_CLK3 provides the 19.2 MHz reference clock to the PHY */
	phy_ref_clk = clk_get(NULL, "auxclk3_ck");
	if (IS_ERR(phy_ref_clk)) {
		pr_err("Cannot request auxclk3\n");
		return;
	}
	clk_set_rate(phy_ref_clk, 19200000);
	clk_enable(phy_ref_clk);

	/* disable the power to the usb hub prior to init and reset phy+hub */
	ret = gpio_request_array(panda_ehci_gpios,
				 ARRAY_SIZE(panda_ehci_gpios));
	if (ret) {
		pr_err("Unable to initialize EHCI power/reset\n");
		return;
	}

	gpio_export(GPIO_HUB_POWER, 0);
	gpio_export(GPIO_HUB_NRESET, 0);
	gpio_set_value(GPIO_HUB_NRESET, 1);

	usbhs_init(&usbhs_bdata);

	/* enable power to hub */
	gpio_set_value(GPIO_HUB_POWER, 1);
}
#else
static void __init omap4_ehci_init(void) { }
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#else
	.mode			= MUSB_PERIPHERAL,
#endif
#ifdef CONFIG_VENDOR_HHTECH
	.power			= 500,
#else
	.power			= 200,
#endif
};

static struct omap2_hsmmc_info mmc[] = {
	{	// for iNAND/eMMC
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA
				| MMC_CAP_1_8V_DDR,
		.ocr_mask	= MMC_VDD_33_34, // | MMC_VDD_29_30,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.nonremovable	= true,
		.no_off_init	= true,
	},
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_1_8V_DDR
				, //| MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
#if defined(CONFIG_BCM4329) || defined(CONFIG_BCM4329_MODULE) || \
    defined(CONFIG_BCM4330) || defined(CONFIG_BCM4330_MODULE) || \
    defined(CONFIG_BCMDHD)  || defined(CONFIG_BCMDHD_MODULE)
	{
		.name		= "omap_wlan",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_33_34,
		.nonremovable	= false,
		.mmc_data	= &panda_wifi_data,
	},
#elif defined(CONFIG_WL12XX_SDIO) || defined(CONFIG_WL12XX_SDIO_MODULE)
	{
		.name		= "wl1271",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
#endif
	{}	/* Terminator */
};

#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
static struct regulator_consumer_supply omap4_panda_vmmc5_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.4",
};

static struct regulator_init_data panda_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap4_panda_vmmc5_supply,
};

static struct fixed_voltage_config panda_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 1800000, /* 1.8V */
	.gpio = GPIO_WIFI_PMENA,
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &panda_vmmc5,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &panda_vwlan,
	},
};
#endif

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	if (!pdata) {
		dev_err(dev, "%s: NULL platform data\n", __func__);
		return -EINVAL;
	}
	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		 if (ret)
			dev_err(dev, "%s: Error card detect config(%d)\n",
				__func__, ret);
		 else {
			pdata->slots[0].card_detect = twl6030_mmc_card_detect;
			pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
				MMCDETECT_INTR_OFFSET;
		 }
	} else
	/* Setting MMC5 SDIO card .built-in variable
	  * This is to make sure that if WiFi driver is not loaded
	  * at all, then the MMC/SD/SDIO driver does not keep
	  * turning on/off the voltage to the SDIO card
	  */
	if (pdev->id == 4) {
		ret = 0;
		pdata->slots[0].mmc_data.built_in = 1;
	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed omap4_twl6030_hsmmc_set_late_init\n");
		return;
	}
	pdata = dev->platform_data;

	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static void omap4_audio_conf(void)
{
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE);

#ifdef CONFIG_VENDOR_HHTECH
#define GPIO_POWER_AUDIO_AMP 119

	if (0) omap_mux_init_signal("abe_dmic_clk1.gpio_119",
		OMAP_PIN_OUTPUT | OMAP_MUX_MODE3); else
	omap_mux_init_gpio(GPIO_POWER_AUDIO_AMP, OMAP_PIN_OUTPUT);
	gpio_request_one(GPIO_POWER_AUDIO_AMP,
		GPIOF_OUT_INIT_HIGH, "Audio AMP");   // XXX:
#endif
}

static int tps6130x_enable(int on)
{
	u8 val = 0;
	int ret;

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &val, TWL6040_REG_GPOCTL);
	if (ret < 0) {
		pr_err("%s: failed to read GPOCTL %d\n", __func__, ret);
		return ret;
	}

	/* TWL6040 GPO2 connected to TPS6130X NRESET */
	if (on)
		val |= TWL6040_GPO2;
	else
		val &= ~TWL6040_GPO2;

	ret = twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, val, TWL6040_REG_GPOCTL);
	if (ret < 0)
		pr_err("%s: failed to write GPOCTL %d\n", __func__, ret);

	return ret;
}

static struct tps6130x_platform_data tps6130x_pdata = {
	.chip_enable	= tps6130x_enable,
};

static struct regulator_consumer_supply twl6040_vddhf_supply[] = {
	REGULATOR_SUPPLY("vddhf", "twl6040-codec"),
};

static struct regulator_init_data twl6040_vddhf = {
	.constraints = {
		.min_uV			= 4075000,
		.max_uV			= 4950000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(twl6040_vddhf_supply),
	.consumer_supplies	= twl6040_vddhf_supply,
	.driver_data		= &tps6130x_pdata,
};

static struct i2c_board_info __initdata tps6130x_boardinfo = {
	I2C_BOARD_INFO("tps6130x", 0x33),
	.platform_data = &twl6040_vddhf,
};

#ifdef CONFIG_VENDOR_HHTECH
#define GPIO_TOUCHSCREEN_IRQ 38
#define GPIO_TOUCHSCREEN_RST 37

static void touchscreen_rst(void)
{
#if defined(CONFIG_SMARTQ_T15) || defined(CONFIG_SMARTQ_S7) || \
    defined(CONFIG_SMARTQ_K7) || defined(CONFIG_SMARTQ_X7)
	gpio_direction_output(GPIO_TOUCHSCREEN_RST, 1);

	//gpio_set_value(GPIO_TOUCHSCREEN_RST, 1);
	msleep_interruptible(30);
	gpio_set_value(GPIO_TOUCHSCREEN_RST, 0);
#else
	gpio_direction_output(GPIO_TOUCHSCREEN_RST, 0);

	//gpio_set_value(GPIO_TOUCHSCREEN_RST, 0);
	msleep_interruptible(30);
	gpio_set_value(GPIO_TOUCHSCREEN_RST, 1);
#endif
}

static void touchscreen_irq_init(void)
{
	gpio_direction_input(GPIO_TOUCHSCREEN_IRQ);
}

static void __init touchscreen_gpio_init(void)
{
	int gpio = GPIO_TOUCHSCREEN_IRQ;
	if (0) omap_mux_init_signal("gpmc_ad14.gpio_38",
		OMAP_PIN_INPUT | OMAP_MUX_MODE3); else
	omap_mux_init_gpio(gpio, OMAP_PIN_INPUT);

	if (gpio_request(gpio, "touchscreen_irq"))
		pr_err("Cannot request GPIO %d\n", gpio);
	//gpio_direction_input(gpio);

	gpio = GPIO_TOUCHSCREEN_RST;
	if (0) omap_mux_init_signal("gpmc_ad13.gpio_37",
		OMAP_PIN_OUTPUT | OMAP_MUX_MODE3); else
	omap_mux_init_gpio(gpio, OMAP_PIN_OUTPUT);

	if (gpio_request(gpio, "touchscreen_rst"))
		pr_err("Cannot request GPIO %d\n", gpio);
	//gpio_direction_output(gpio, 0);
}

static void touchscreen_gpio_free(void)
{
        gpio_free(GPIO_TOUCHSCREEN_IRQ);
        gpio_free(GPIO_TOUCHSCREEN_RST);
}
#else
static void __init touchscreen_gpio_init(void) { }
#endif

#if defined(CONFIG_TOUCHSCREEN_GOODIX) || \
    defined(CONFIG_TOUCHSCREEN_GOODIX_MODULE)
#include <linux/goodix_touch.h>

struct goodix_i2c_rmi_platform_data goodix_pdata = {
	.rst = touchscreen_rst,
	.irq = OMAP_GPIO_IRQ(GPIO_TOUCHSCREEN_IRQ),
	.irq_gpio = GPIO_TOUCHSCREEN_IRQ,
	.irq_init = touchscreen_irq_init,
	//.gpio_init = touchscreen_gpio_init,
#ifdef CONFIG_SMARTQ_T15
#if defined(CONFIG_SWITCH_GPIO) || defined(CONFIG_SWITCH_GPIO_MODULE)
	.get_lock_state = omap4_gpio_switch_state,
#endif
#endif
};
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5X0X) || \
	defined(CONFIG_TOUCHSCREEN_FT5X0X_MODULE)
#include <linux/ft5x0x_i2c_ts.h>

struct ft5x0x_platform_data ft5x0x_pdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_TOUCHSCREEN_IRQ),
	.rst = touchscreen_rst
};
#endif

#if defined(CONFIG_TOUCHSCREEN_PIXCIR) || \
    defined(CONFIG_TOUCHSCREEN_PIXCIR_MODULE)
#include <linux/pixcir_i2c_ts.h>

struct pixcir_platform_data pixcir_pdata = {
	.rst = touchscreen_rst,
	.irq = OMAP_GPIO_IRQ(GPIO_TOUCHSCREEN_IRQ),
	.init = touchscreen_irq_init,
};
#endif

#if defined(CONFIG_NOVATEK_TOUCH) || \
    defined(CONFIG_NOVATEK_TOUCH_MODULE)
#include <linux/novatek_ts.h>

struct novatek_platform_data novatek_pdata = {
	.rst = touchscreen_rst,
	.irq = OMAP_GPIO_IRQ(GPIO_TOUCHSCREEN_IRQ),
	.irq_gpio = GPIO_TOUCHSCREEN_IRQ,
	.irq_init = touchscreen_irq_init,
	//.gpio_init = touchscreen_gpio_init,
};

#endif

#if defined(CONFIG_TOUCHSCREEN_SSD2533) || \
    defined(CONFIG_TOUCHSCREEN_SSD2533_MODULE)
#include <mach/ssd2533.h>

Reg_Item ssd2533_Init[]={
        {CMD_1B,0x04},         // Exit sleep mode
        {CMD_DELAY, 30},
        {CMD_1B,0x06,0x16},         // Drive No: 23
        {CMD_1B,0x07,0x23},         // Sense No: 36
        {CMD_2B,0x08,0x00,0x16},    // Set 1st drive line reg
        {CMD_2B,0x09,0x00,0x15},    // Set 2st drive line reg
        {CMD_2B,0x0a,0x00,0x14},    // Set 3st drive line reg
        {CMD_2B,0x0b,0x00,0x13},    // Set 4st drive line reg
        {CMD_2B,0x0c,0x00,0x12},    // Set 5st drive line reg
        {CMD_2B,0x0d,0x00,0x11},    // Set 6st drive line reg
        {CMD_2B,0x0e,0x00,0x10},    // Set 7st drive line reg
        {CMD_2B,0x0f,0x00,0x0f},    // Set 8st drive line reg
        {CMD_2B,0x10,0x00,0x0e},    // Set 9st drive line reg
        {CMD_2B,0x11,0x00,0x0d},    // Set 10st drive line reg
        {CMD_2B,0x12,0x00,0x0c},    // Set 11st drive line reg
        {CMD_2B,0x13,0x00,0x0b},    // Set 12st drive line reg
        {CMD_2B,0x14,0x00,0x0a},    // Set 13st drive line reg
        {CMD_2B,0x15,0x00,0x09},    // Set 14st drive line reg
        {CMD_2B,0x16,0x00,0x08},    // Set 15st drive line reg
        {CMD_2B,0x17,0x00,0x07},    // Set 16st drive line reg
        {CMD_2B,0x18,0x00,0x06},    // Set 17st drive line reg
        {CMD_2B,0x19,0x00,0x05},    // Set 18st drive line reg
        {CMD_2B,0x1a,0x00,0x04},    // Set 19st drive line reg
        {CMD_2B,0x1b,0x00,0x03},    // Set 20st drive line reg
        {CMD_2B,0x1c,0x00,0x02},    // Set 21st drive line reg
        {CMD_2B,0x1d,0x00,0x01},    // Set 22st drive line reg
        {CMD_2B,0x1e,0x00,0x00},    // Set 23st drive line reg
        {CMD_1B,0x27,0x1c},
        {CMD_1B,0x28,0x20},
        {CMD_1B,0x2a,0x03},
        {CMD_1B,0x2c,0x01},         // Median filter
        {CMD_1B,0x2e,0x13},         // Sub-frame drive pulse
        {CMD_1B,0x2f,0x01},         // Integration gain
        {CMD_1B,0x30,0x02},         // Start integrate
        {CMD_1B,0x31,0x09},         // Stop integrate
        {CMD_1B,0xd5,0x03},
        {CMD_1B,0xd7,0x04},         // ADC range
        {CMD_1B,0xd8,0x04},
        {CMD_1B,0xdb,0x04},         // Set integration cap
        {CMD_2B,0x33,0x00,0x01},    // min. finger area
        {CMD_2B,0x34,0x00,0x40},    // min finger level
        {CMD_2B,0x35,0x00,0x20},    // finger weight threshold
        {CMD_2B,0x36,0x00,0x20},    // max. finger area
        {CMD_1B,0x37,0x05},         // segmentation depth

        {CMD_1B,0x39,0x02},         // CG method
        {CMD_1B,0x3a,0x02},         // Hybrid mode select
        {CMD_1B,0x3d,0x01},
        {CMD_1B,0x40,0x50},
        {CMD_1B,0x44,0x01},
        {CMD_1B,0x53,0x04},         // Move tolerance
        {CMD_2B,0x54,0x00,0xc0},    // X tracking
        {CMD_2B,0x55,0x00,0xc0},    // Y tracking

        {CMD_1B,0x56,0x01},         // Moing average filter
        {CMD_1B,0x58,0x00},         // Finger weight scaling
        {CMD_1B,0x59,0x01},         // Enable random walk
        {CMD_1B,0x5a,0x00},         // Disable missing frame
        {CMD_1B,0x5b,0x10},         // Set random walk window
#ifdef CONFIG_SMARTQ_T15
        {CMD_1B,0x65,0x02},
        {CMD_2B,0x66,0x79,0xc0},
        {CMD_2B,0x67,0x85,0xe0},
#else
	{CMD_1B,0x65,0x01},         // Set X-Invert
	{CMD_2B,0x66,0x8f,0x10},    // X scaling
	{CMD_2B,0x67,0x8c,0x80},    // Y scaling
#endif
        {CMD_2B,0x7a,0xff,0xff},
        {CMD_2B,0x7b,0xc0,0x0b},
        {CMD_1B,0x8a,0x0a},
        {CMD_1B,0x8b,0x01},
        {CMD_1B,0x8c,0xc0},
        {CMD_1B,0xac,0x00},
        {CMD_1B,0xab,0x64},
        {CMD_1B,0xad,0x02},
        {CMD_1B,0xae,0x00},
        {CMD_1B,0xaf,0x50},
        {CMD_1B,0xb0,0x00},
        {CMD_1B,0xbb,0x00},
        {CMD_1B,0xbc,0x00},
        {CMD_1B,0x25,0x02},         // set scan mode, 100Hz
};

struct ssd2533_platform_data ssd2533_pdata = {
	.rst = touchscreen_rst,
	.irq = OMAP_GPIO_IRQ(GPIO_TOUCHSCREEN_IRQ),
	.irq_init = touchscreen_irq_init,
        //.gpio_init = touchscreen_gpio_init,
        .gpio_free = touchscreen_gpio_free,
        .reg_array = ssd2533_Init,
        .reg_size = ARRAY_SIZE(ssd2533_Init),
#ifdef CONFIG_SMARTQ_T15
#if defined(CONFIG_SWITCH_GPIO) || defined(CONFIG_SWITCH_GPIO_MODULE)
	.get_lock_state = omap4_gpio_switch_state,
#endif
#endif
};
#endif

#if defined(CONFIG_CHARGER_BQ2416X) || defined(CONFIG_CHARGER_BQ2416X_MODULE)
#include <linux/i2c/bq2416x.h>

static struct bq2416x_platform_data bq2416x_pdata = {
        .cin_limit = IN_ILIM_1,
        .work_current = 550,
	.standby_current = 1450,
        .in_dpm = IN_VDPM_MAX,
        .max_charger_voltagemV = 4200,
        .termination_currentmA = 200,
};
#endif

/*
 * Display monitor features are burnt in their EEPROM as EDID data. The EEPROM
 * is connected as I2C slave device, and can be accessed at address 0x50
 */
static struct i2c_board_info __initdata panda_i2c_bus3_boardinfo[] = {
#if defined(CONFIG_PANEL_DVI_OUTPUT) || defined(CONFIG_OMAP4_DSS_HDMI)
	{
		I2C_BOARD_INFO("eeprom", 0x50),
	},
#endif
};

static struct i2c_board_info __initdata panda_i2c_bus2_boardinfo[] = {
#if defined(CONFIG_MMA7455) || defined(CONFIG_MMA7455_MODULE)
	{
		I2C_BOARD_INFO("mma7455", 0x1D),
	},
#endif
#if defined(CONFIG_BATTERY_OZ8806) || defined(CONFIG_BATTERY_OZ8806_MODULE)
	{
		I2C_BOARD_INFO("OZ8806", 0x2f),
	},
#endif
#if defined(CONFIG_SND_SOC_CS42L52) || defined(CONFIG_SND_SOC_CS42L52_MODULE)
	{
		I2C_BOARD_INFO("cs42l52", 0x4b),
	},
#endif
#if defined(CONFIG_INPUT_BMA250) || defined(CONFIG_INPUT_BMA250_MODULE)
	{
		I2C_BOARD_INFO("bma250", 0x18),
	},
#endif
#if defined(CONFIG_INPUT_MMA8452) || defined(CONFIG_INPUT_MMA8452_MODULE)
	{
		I2C_BOARD_INFO("mma8452", 0x1C),
	},
#endif
#if defined(CONFIG_INPUT_ISL29023) || defined(CONFIG_INPUT_ISL29023_MODULE)
	{
		I2C_BOARD_INFO("isl29023", 0x44),
	},
#endif
#if defined(CONFIG_CHARGER_BQ2416X) || defined(CONFIG_CHARGER_BQ2416X_MODULE)
	{
		I2C_BOARD_INFO("bq2416x", 0x6b),
		.platform_data = &bq2416x_pdata,
	},
#endif
#if defined(CONFIG_BATTERY_BQ27410) || defined(CONFIG_BATTERY_BQ27x00_MODULE)
	{
		I2C_BOARD_INFO("bq27410", 0x55),
	},
#endif

};

static struct i2c_board_info __initdata panda_i2c_bus4_boardinfo[] = {
#if defined(CONFIG_TOUCHSCREEN_GOODIX) || \
    defined(CONFIG_TOUCHSCREEN_GOODIX_MODULE)
	{
		I2C_BOARD_INFO("Goodix-TS", 0x55),
		.platform_data = &goodix_pdata,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_SSD2533) || \
    defined(CONFIG_TOUCHSCREEN_SSD2533_MODULE)
        {
                I2C_BOARD_INFO(SSD2533_I2C_NAME, SSD2533_I2C_ADDR),
                .platform_data = &ssd2533_pdata,
        },
#endif
#if defined(CONFIG_TOUCHSCREEN_PIXCIR) || \
    defined(CONFIG_TOUCHSCREEN_PIXCIR_MODULE)
	{
		I2C_BOARD_INFO("pixcir_ts", 0x5c),
		.platform_data = &pixcir_pdata,
	},
#endif
#if defined(CONFIG_NOVATEK_TOUCH) || \
	defined(CONFIG_NOVATEK_TOUCH_MODULE)
	{
		I2C_BOARD_INFO("novatek-ts", 0x01),
		//     I2C_BOARD_INFO("novatek-ts", 0x5c),
		//     I2C_BOARD_INFO("novatek-ts", 0x09),
		.platform_data = &novatek_pdata,
	},
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5X0X) || \
	defined(CONFIG_TOUCHSCREEN_FT5X0X_MODULE)
	{
		I2C_BOARD_INFO(FT5X0X_NAME, I2C_CTPM_ADDRESS>>1),
		.platform_data= &ft5x0x_pdata,
	},
#endif
};

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
                                struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request();
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata panda_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata panda_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata panda_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata panda_i2c_4_bus_pdata;

void omap4_power_init(void);

static int __init omap4_panda_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &panda_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &panda_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &panda_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &panda_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &panda_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &panda_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &panda_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &panda_i2c_4_bus_pdata);

	omap4_power_init();

	//i2c_register_board_info(1, &tps6130x_boardinfo, 1);
	omap_register_i2c_bus(2, 100, panda_i2c_bus2_boardinfo,
					ARRAY_SIZE(panda_i2c_bus2_boardinfo));
#ifdef CONFIG_TWL6040_UNBOUND
    {	extern struct i2c_board_info twl6040_boardinfo;
	i2c_register_board_info(CONFIG_TWL6040_I2C_BUS_NO,
		&twl6040_boardinfo, 1);
    }
#endif
	/*
	 * Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz
	 */
	omap_register_i2c_bus(3, 100, panda_i2c_bus3_boardinfo,
					ARRAY_SIZE(panda_i2c_bus3_boardinfo));
	omap_register_i2c_bus(4, 400, panda_i2c_bus4_boardinfo,
					ARRAY_SIZE(panda_i2c_bus4_boardinfo));

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	/*
	 * Drive MSECURE high for TWL6030/6032 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
#ifndef CONFIG_VENDOR_HHTECH
	/* gpio 0 - TFP410 PD */
	OMAP4_MUX(KPD_COL1, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* WLAN IRQ - GPIO 53 */
	OMAP4_MUX(GPMC_NCS3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* WLAN POWER ENABLE - GPIO 43 */
	OMAP4_MUX(GPMC_A19, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
#endif
	/* WLAN SDIO: MMC5 CMD */
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 CLK */
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 DAT[0-3] */
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* dispc2_data23 */
	OMAP4_MUX(USBB2_ULPITLL_STP, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data22 */
	OMAP4_MUX(USBB2_ULPITLL_DIR, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data21 */
	OMAP4_MUX(USBB2_ULPITLL_NXT, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data20 */
	OMAP4_MUX(USBB2_ULPITLL_DAT0, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data19 */
	OMAP4_MUX(USBB2_ULPITLL_DAT1, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data18 */
	OMAP4_MUX(USBB2_ULPITLL_DAT2, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data15 */
	OMAP4_MUX(USBB2_ULPITLL_DAT3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data14 */
	OMAP4_MUX(USBB2_ULPITLL_DAT4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data13 */
	OMAP4_MUX(USBB2_ULPITLL_DAT5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data12 */
	OMAP4_MUX(USBB2_ULPITLL_DAT6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data11 */
	OMAP4_MUX(USBB2_ULPITLL_DAT7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data10 */
	OMAP4_MUX(DPM_EMU3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data9 */
	OMAP4_MUX(DPM_EMU4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data16 */
	OMAP4_MUX(DPM_EMU5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data17 */
	OMAP4_MUX(DPM_EMU6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_hsync */
	OMAP4_MUX(DPM_EMU7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_pclk */
	OMAP4_MUX(DPM_EMU8, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_vsync */
	OMAP4_MUX(DPM_EMU9, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_de */
	OMAP4_MUX(DPM_EMU10, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data8 */
	OMAP4_MUX(DPM_EMU11, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data7 */
	OMAP4_MUX(DPM_EMU12, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data6 */
	OMAP4_MUX(DPM_EMU13, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data5 */
	OMAP4_MUX(DPM_EMU14, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data4 */
	OMAP4_MUX(DPM_EMU15, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data3 */
	OMAP4_MUX(DPM_EMU16, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data2 */
	OMAP4_MUX(DPM_EMU17, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data1 */
	OMAP4_MUX(DPM_EMU18, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data0 */
	OMAP4_MUX(DPM_EMU19, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct omap_uart_port_info omap_serial_uart2_info[] = {
	{
		.use_dma    = 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
#if defined(CONFIG_BT_BCM43XX) || defined(CONFIG_BT_BCM43XX_MODULE)
		.wake_peer  = bcm43xx_wake_peer,
		.rts_mux_driver_control = 1,
#endif
	},
};

static inline void __init board_serial_init(void)
{
#ifndef CONFIG_VENDOR_HHTECH
	omap_serial_init();
#else
	omap_serial_init_port_pads(1, NULL, 0, &omap_serial_uart2_info);
	omap_serial_init_port_pads(2, NULL, 0, NULL);
#endif

}

#define GPIO_LVDS_POWER 34
#define GPIO_LCD_POWER  35

static int smartq_lcd_enable(struct omap_dss_device *dssdev)
{
#ifdef CONFIG_SMARTQ_Q8
#define GPIO_LCD_PWR_CTRL 113

	gpio_set_value(GPIO_LCD_PWR_CTRL, 0);
#endif

#ifdef CONFIG_SMARTQ_X7
#define GPIO_LCD_PWR_CTRL 134
#endif

	gpio_set_value(dssdev->reset_gpio >> 8, 1);
	gpio_set_value(dssdev->reset_gpio & 0xFF, 1);

    {	unsigned long long t = cpu_clock(0);
	do_div(t, 500000000);	// XXX:
	if (0 < (unsigned)t)
#ifdef CONFIG_SMARTQ_Q8
	    msleep_interruptible(200);
#else
	    msleep_interruptible(80);
#endif
    }

	return 0;
}

static void smartq_lcd_disable(struct omap_dss_device *dssdev)
{
#ifdef CONFIG_SMARTQ_Q8
	gpio_set_value(GPIO_LCD_PWR_CTRL, 1);
#endif

	gpio_set_value(dssdev->reset_gpio & 0xFF, 0);
	gpio_set_value(dssdev->reset_gpio >> 8, 0);
}

/* we must enable both gpio to power on the LCD and LVDS controler */
static int __init smartq_lcd_init(struct omap_dss_device *dssdev)
{
	int r, gpio;
#ifdef CONFIG_PANEL_LG_IPS_7
#define GPIO_SPI_CSB 134
#define GPIO_SPI_SCL 135
#define GPIO_SPI_SDA 136

	// LG 7 inch panel need initialization by spi_gpio
	omap_mux_init_gpio(GPIO_SPI_SCL, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_SPI_SDA, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_SPI_CSB, OMAP_PIN_OUTPUT);

	gpio_direction_output(GPIO_SPI_SDA, 0);
	gpio_direction_output(GPIO_SPI_SCL, 0);
	gpio_direction_output(GPIO_SPI_CSB, 0);
#endif

#ifdef CONFIG_SMARTQ_Q8
	gpio = GPIO_LCD_PWR_CTRL;
	omap_mux_init_gpio(gpio, OMAP_PIN_OUTPUT);
	gpio_request_one(gpio, GPIOF_OUT_INIT_LOW, "LCD_a PD");
#endif

#ifdef CONFIG_SMARTQ_X7
	gpio =  GPIO_LCD_PWR_CTRL;
	omap_mux_init_gpio(gpio, OMAP_PIN_OUTPUT);
	gpio_direction_output(gpio, 1);
#endif

	gpio = dssdev->reset_gpio & 0xFF;
	omap_mux_init_gpio(gpio, OMAP_PIN_OUTPUT);

	/* Requesting LCD/LVDS PD GPIO and disabling them, at bootup */
	r = gpio_request_one(gpio, GPIOF_OUT_INIT_LOW, "LCD PD");
	if (r) goto err;

	gpio = dssdev->reset_gpio >> 8;
	if (!gpio) return r;

	omap_mux_init_gpio(gpio, OMAP_PIN_OUTPUT);

	r = gpio_request_one(gpio, GPIOF_OUT_INIT_LOW, "LVDS PD");
	if (r) goto err;

	smartq_lcd_enable(dssdev);

#if 0
	gpio = GPIO_BL_PWM;
	if (0) omap_mux_init_signal("usbb1_ulpitll_dat6.dmtimer10_pwm_evt",
		OMAP_PIN_OUTPUT | OMAP_MUX_MODE3); else
	omap_mux_init_gpio(gpio, OMAP_PIN_OUTPUT);
	r = gpio_request_one(gpio, GPIOF_OUT_INIT_HIGH, "backlight_pwm");
	if (r) goto err; //else gpio_free(gpio);
#endif

	return r;
err:	pr_err("Cannot request GPIO %d\n", gpio);
	return r;
}

static struct panel_generic_dpi_data smartq_lcd_panel = {
#ifdef CONFIG_PANEL_LG_IPS_10
	.name			= "lg_ips10",
#elif defined(CONFIG_PANEL_CLAA070WP03)
	.name			= "claa070wp03_panel",
#elif defined(CONFIG_PANEL_QM_8)
        .name                   = "qm_8",
#elif defined(CONFIG_PANEL_LG_IPS_7)
	.name                   = "lg_ips7",
#elif defined(CONFIG_PANEL_HS_HSD101PWW1)
        .name                   = "hs_ips10.1",
#endif
	.platform_enable	= smartq_lcd_enable,
	.platform_disable	= smartq_lcd_disable,
};

struct omap_dss_device smartq_lcd_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "SmartQ_LCD",
// 9Apr2012 ctjing added for ld070ws2(SmartQ_S7)
#ifdef CONFIG_PANEL_LG_IPS_7
	.driver_name		= "lg_ld070ws2_panel",
#else
	.driver_name		= "generic_dpi_panel",
#endif
	.data			= &smartq_lcd_panel,
#if   defined(CONFIG_PANEL_LG_IPS_10)
	.phy.dpi.data_lines	= 18,
	.panel = {
		.timings = {
			.x_res = 1024,
			.y_res =  768,
		},
	},
#elif defined(CONFIG_PANEL_CLAA070WP03)
	.phy.dpi.data_lines	= 18,
	.panel = {
		.timings = {
			.x_res =  800,
			.y_res = 1280,
		},
	},
#elif defined(CONFIG_PANEL_QM_8)
	.phy.dpi.data_lines	= 24,
	.panel = {
		.timings = {
			.x_res = 1024,
			.y_res =  768,
		},
	},
#elif defined(CONFIG_PANEL_LG_IPS_7)
	.phy.dpi.data_lines	= 24,
	.panel = {
		.timings = {
			.x_res = 1024,
			.y_res =  600,
		},
	},
#elif defined(CONFIG_PANEL_HS_HSD101PWW1)
	.phy.dpi.data_lines     = 18,
	.panel = {
		.timings = {
			.x_res = 1280,
			.y_res =  800,
		},
	},
#endif
	.reset_gpio		= GPIO_LCD_POWER | (GPIO_LVDS_POWER << 8),
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};

#ifdef CONFIG_PANEL_DVI_OUTPUT
/* Display DVI */
#define PANDA_DVI_TFP410_POWER_DOWN_GPIO	0

static int omap4_panda_enable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(dssdev->reset_gpio, 1);
	return 0;
}

static void omap4_panda_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(dssdev->reset_gpio, 0);
}

/* Using generic display panel */
static struct panel_generic_dpi_data omap4_dvi_panel = {
	.name			= "generic_720p",
	.platform_enable	= omap4_panda_enable_dvi,
	.platform_disable	= omap4_panda_disable_dvi,
};

struct omap_dss_device omap4_panda_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "generic_dpi_panel",
	.data			= &omap4_dvi_panel,
	.phy.dpi.data_lines	= 24,
	.reset_gpio		= PANDA_DVI_TFP410_POWER_DOWN_GPIO,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};

static int __init omap4_panda_dvi_init(void)
{
	int r;

	/* Requesting TFP410 DVI GPIO and disabling it, at bootup */
	r = gpio_request_one(omap4_panda_dvi_device.reset_gpio,
				GPIOF_OUT_INIT_LOW, "DVI PD");
	if (r)
		pr_err("Failed to get DVI powerdown GPIO\n");

	return r;
}
#endif

#ifdef CONFIG_OMAP4_DSS_HDMI
static struct gpio panda_hdmi_gpios[] __initdata = {
	{ HDMI_GPIO_CT_CP_HPD,	GPIOF_OUT_INIT_HIGH, "hdmi_gpio_hpd"   },
	{ HDMI_GPIO_LS_OE,	GPIOF_OUT_INIT_HIGH, "hdmi_gpio_ls_oe" },
};

static void __init omap4_panda_hdmi_mux_init(void)
{
	u32 r;
	int status;
	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
	omap_mux_init_signal("hdmi_hpd.hdmi_hpd",
				OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("gpmc_wait2.gpio_100",
			OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("hdmi_cec.hdmi_cec",
			OMAP_PIN_INPUT_PULLUP);
	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
	omap_mux_init_signal("hdmi_ddc_scl.hdmi_ddc_scl",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hdmi_ddc_sda.hdmi_ddc_sda",
			OMAP_PIN_INPUT_PULLUP);

	/* strong pullup on DDC lines using unpublished register */
	r = ((1 << 24) | (1 << 28)) ;
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);

	gpio_request(HDMI_GPIO_HPD, NULL);
	omap_mux_init_gpio(HDMI_GPIO_HPD, OMAP_PIN_INPUT | OMAP_PULL_ENA);
	gpio_direction_input(HDMI_GPIO_HPD);

	status = gpio_request_array(panda_hdmi_gpios,
			ARRAY_SIZE(panda_hdmi_gpios));
	if (status)
		pr_err("%s: Cannot request HDMI GPIOs %x \n", __func__, status);
}

static struct omap_dss_device  omap4_panda_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
#if 0	// XXX:
	.panel = {
		.hdmi_default_cea_code = 34,
		.timings = {
			.x_res = 1920,
			.y_res = 1080,
			.pixel_clock = 74250,
			.hsw = 44,
			.hfp = 88,
			.hbp = 148,
			.vsw = 5,
			.vfp = 4,
			.vbp = 36,
		},
	},
#endif
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 15,
			.regm2	= 1,
		},
	},
	.hpd_gpio = HDMI_GPIO_HPD,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};
#endif

static struct omap_dss_device *omap4_panda_dss_devices[] = {
	&smartq_lcd_device,
#ifdef CONFIG_OMAP4_DSS_HDMI
	&omap4_panda_hdmi_device,
#endif
#ifdef CONFIG_PANEL_DVI_OUTPUT
	&omap4_panda_dvi_device,
#endif
};

static struct omap_dss_board_info omap4_panda_dss_data = {
	.num_devices	= ARRAY_SIZE(omap4_panda_dss_devices),
	.devices	= omap4_panda_dss_devices,
	.default_device	= &smartq_lcd_device,
};

/*
 * LPDDR2 Configeration Data:
 * The memory organisation is as below :
 *	EMIF1 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	EMIF2 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices = {
	.cs0_device = &lpddr2_elpida_2G_S4_dev,
	.cs1_device = &lpddr2_elpida_2G_S4_dev
};

/*
 * LPDDR2 Configuration Data for 4470 SOMs:
 * The memory organization is as below :
 *	EMIF1 - CS0 -	4 Gb
 *	EMIF2 - CS0 -	4 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices_4470 = {
	.cs0_device = &lpddr2_elpida_4G_S4_dev,
};

#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
static void __init omap4_panda_wifi_mux_init(void)
{
	omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT |
				OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(GPIO_WIFI_PMENA, OMAP_PIN_OUTPUT);

return;
	omap_mux_init_signal("sdmmc5_cmd.sdmmc5_cmd",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_clk.sdmmc5_clk",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat0.sdmmc5_dat0",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat1.sdmmc5_dat1",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat2.sdmmc5_dat2",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat3.sdmmc5_dat3",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
}

static struct wl12xx_platform_data omap4_panda_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = WL12XX_TCXOCLOCK_26,
};

static void __init omap4_panda_wifi_init(void)
{
	omap4_panda_wifi_mux_init();
	if (wl12xx_set_platform_data(&omap4_panda_wlan_data))
		pr_err("Error setting wl12xx data\n");
	platform_device_register(&omap_vwlan_device);
}
#endif

static void __init omap4_panda_display_init(void)
{
	int r;

	if ((r = smartq_lcd_init(&smartq_lcd_device)))
		pr_err("error initializing LCD\n");

#ifdef CONFIG_PANEL_DVI_OUTPUT
	r = omap4_panda_dvi_init();
	if (r)
		pr_err("error initializing panda DVI\n");
#endif

#ifdef CONFIG_OMAP4_DSS_HDMI
	omap4_panda_hdmi_mux_init();
#endif
	omap_display_init(&omap4_panda_dss_data);
}


#define PANDA_FB_RAM_SIZE                SZ_16M /* 1920×1080*4 * 2 */
static struct omapfb_platform_data panda_fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			[0] = {
				.size = PANDA_FB_RAM_SIZE,
			},
		},
	},
};

static struct dsscomp_platform_data dsscomp_config_hdmi_display = {
	.tiler1d_slotsz = (SZ_16M + SZ_2M + SZ_8M + SZ_1M),
};

extern void __init omap4_panda_android_init(void);

#ifdef CONFIG_VENDOR_HHTECH
static void __init omap4_panda_camera_mux_init(void)
{
	u32 r = 0;

#define GPIO_CAMERA_PWN_5640 	47
#define GPIO_CAMERA_RESET_5640 	48
#define GPIO_CAMERA_PWN_2655 	81
#define GPIO_CAMERA_RESET_2655 	82
#define GPIO_CAMERA_POWER	83

	omap_mux_init_gpio(GPIO_CAMERA_POWER, OMAP_PIN_OUTPUT);
	gpio_request_one(GPIO_CAMERA_POWER, GPIOF_OUT_INIT_HIGH,
		"camera_power");

	omap_mux_init_gpio(GPIO_CAMERA_PWN_5640, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_CAMERA_RESET_5640, OMAP_PIN_OUTPUT);
	gpio_request_one(GPIO_CAMERA_RESET_5640, GPIOF_OUT_INIT_HIGH,
		"camera_reset_5640");

	omap_mux_init_gpio(GPIO_CAMERA_PWN_2655, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_CAMERA_RESET_2655, OMAP_PIN_OUTPUT);
	gpio_request_one(GPIO_CAMERA_RESET_2655, GPIOF_OUT_INIT_HIGH,
		"camera_reset_2655");

	/* Enable CSI21 pads for T20 and T15 */
	if (1) {
		r = omap4_ctrl_pad_readl(
			OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_CAMERA_RX);
		r |= (0x7 << OMAP4_CAMERARX_CSI21_LANEENABLE_SHIFT);
		r |= (0x3 << OMAP4_CAMERARX_CSI22_LANEENABLE_SHIFT);
		omap4_ctrl_pad_writel(r,
			OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_CAMERA_RX);

		omap_mux_init_signal("csi21_dx0.csi21_dx0",
				OMAP_PIN_INPUT | OMAP_MUX_MODE0);
		omap_mux_init_signal("csi21_dy0.csi21_dy0",
				OMAP_PIN_INPUT | OMAP_MUX_MODE0);
		omap_mux_init_signal("csi21_dx1.csi21_dx1",
				OMAP_PIN_INPUT | OMAP_MUX_MODE0);
		omap_mux_init_signal("csi21_dy1.csi21_dy1",
				OMAP_PIN_INPUT | OMAP_MUX_MODE0);
		omap_mux_init_signal("csi21_dx2.csi21_dx2",
				OMAP_PIN_INPUT | OMAP_MUX_MODE0);
		omap_mux_init_signal("csi21_dy2.csi21_dy2",
				OMAP_PIN_INPUT | OMAP_MUX_MODE0);

		omap_mux_init_signal("csi22_dx0.csi22_dx0",
				OMAP_PIN_INPUT | OMAP_MUX_MODE0);
		omap_mux_init_signal("csi22_dy0.csi22_dy0",
				OMAP_PIN_INPUT | OMAP_MUX_MODE0);
		omap_mux_init_signal("csi22_dx1.csi22_dx1",
				OMAP_PIN_INPUT | OMAP_MUX_MODE0);
		omap_mux_init_signal("csi22_dy1.csi22_dy1",
				OMAP_PIN_INPUT | OMAP_MUX_MODE0);
	}

	omap_mux_init_signal("fref_clk2_out.fref_clk2_out", OMAP_MUX_MODE0);
}

#define GPIO_BOARD_ID0 153
#define GPIO_BOARD_ID1 154
#define GPIO_BOARD_ID2 155
#define GPIO_BOARD_ID3 156
unsigned char board_revision;

static void __init hhtech_boardrev_init(void)
{
	int ret;

	ret = gpio_request(GPIO_BOARD_ID0, "board_id0");
	if (ret) {
		pr_err("Cannot request GPIO %d\n", GPIO_BOARD_ID0);
		goto error1;
	}

	ret = gpio_request(GPIO_BOARD_ID1, "board_id1");
	if (ret) {
		pr_err("Cannot request GPIO %d\n", GPIO_BOARD_ID1);
		goto error2;
	}

	ret = gpio_request(GPIO_BOARD_ID2, "board_id2");
	if (ret) {
		pr_err("Cannot request GPIO %d\n", GPIO_BOARD_ID2);
		goto error3;
	}

	ret = gpio_request(GPIO_BOARD_ID3, "board_id3");
	if (ret) {
		pr_err("Cannot request GPIO %d\n", GPIO_BOARD_ID3);
		goto error4;
	}

	omap_mux_init_gpio(GPIO_BOARD_ID0, OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_BOARD_ID1, OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_BOARD_ID2, OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_BOARD_ID3, OMAP_PIN_INPUT);

	board_revision  = gpio_get_value(GPIO_BOARD_ID0);
	board_revision |= gpio_get_value(GPIO_BOARD_ID1) << 1;
	board_revision |= gpio_get_value(GPIO_BOARD_ID2) << 2;
	board_revision |= gpio_get_value(GPIO_BOARD_ID3) << 3;

	pr_info("HHTech Board Revision: %x\n", board_revision);

	return;

error4:
	gpio_free(GPIO_BOARD_ID2);
error3:
	gpio_free(GPIO_BOARD_ID1);
error2:
	gpio_free(GPIO_BOARD_ID0);
error1:
	board_revision = 0xf;
	pr_err("Unable to detemine Board Revision\n");
}
#else
static void __init omap4_panda_camera_mux_init(void) { }
#endif

static void __init omap4_panda_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	if (cpu_is_omap447x())
		omap_emif_setup_device_details(&emif_devices_4470,
					       &emif_devices_4470); else

	omap_emif_setup_device_details(&emif_devices, &emif_devices);

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, NULL, package);

#ifdef CONFIG_VENDOR_HHTECH
	hhtech_boardrev_init();
#endif

	omap_init_board_version(OMAP4_PANDA);
	omap4_create_board_props();

	omap4_gpio_keys_init();
	omap4_gpio_switch_init();
	touchscreen_gpio_init();
	omap_charger_io_init();
	omap_encrypt_io_init();
	omap4_panda_camera_mux_init();
	bcm43xx_bluetooth_io_init();

#ifdef CONFIG_VENDOR_HHTECH
	panda_wlan_init();
#endif
#if defined(CONFIG_WL12XX) || defined(CONFIG_WL12XX_MODULE)
        wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");
        omap4_panda_wifi_init();
#endif

	omap4_panda_i2c_init();
	omap4_register_ion();
	omap4_audio_conf();
	platform_add_devices(panda_devices, ARRAY_SIZE(panda_devices));
	board_serial_init();
	omap4_twl6030_hsmmc_init(mmc);
	omap4_ehci_init();
	usb_musb_init(&musb_board_data);

	omap_dmm_init();
	//omap_vram_set_sdram_vram(PANDA_FB_RAM_SIZE, 0);
	omapfb_set_platform_data(&panda_fb_pdata);
	omap4_panda_display_init();

	// XXX: PMIC mux setting
	omap_mux_init_signal("sys_nirq1",
		OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN);

	if (cpu_is_omap446x()
#ifdef CONFIG_SMARTQ_T30
		|| (1 && cpu_is_omap447x())	// XXX:
#endif
		) {
		/* Vsel0 = gpio, vsel1 = gnd */
		status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
#ifdef CONFIG_VENDOR_HHTECH
					OMAP_PIN_OFF_OUTPUT_LOW,
#else
					OMAP_PIN_OFF_OUTPUT_HIGH,
#endif
					-1);
		if (status)
			pr_err("TPS62361 initialization failed: %d\n", status);
	}
	omap_enable_smartreflex_on_init();
	omap_pm_enable_off_mode();	// XXX:
	/*
	 * 7X-38.400MBB-T oscillator uses:
	 * Up time = startup time(max 10ms) + enable time (max 100ns: round 1us)
	 * Down time = disable time (max 100ns: round 1us)
	 */
	omap_pm_set_osc_lp_time(11000, 1);
}

static void __init omap4_panda_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

static void __init omap4_panda_reserve(void)
{
	omap_init_ram_size();

#ifdef CONFIG_ION_OMAP
	omap_android_display_setup(&omap4_panda_dss_data,
				   NULL,
				   NULL,
				   &panda_fb_pdata,
				   get_omap_ion_platform_data());
	omap_ion_init();
#else
	omap_android_display_setup(&omap4_panda_dss_data,
				   NULL,
				   NULL,
				   &panda_fb_pdata,
				   NULL);
#endif

	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);

	/* do the static reservations first */
#ifdef CONFIG_SECURITY_MIDDLEWARE_COMPONENT
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	pr_info("%8x@%08x: reserved for SMC\n",
		PHYS_ADDR_SMC_SIZE, PHYS_ADDR_SMC_MEM);
#endif// XXX: useless for GP devices
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
	pr_info("%8x@%08x: reserved for Ducati\n",
		PHYS_ADDR_DUCATI_SIZE, PHYS_ADDR_DUCATI_MEM);
	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE +
					OMAP4_ION_HEAP_SECURE_OUTPUT_WFDHDCP_SIZE);

	omap_reserve();
}

MACHINE_START(OMAP4_PANDA, "OMAP4 Panda board")
	/* Maintainer: David Anders - Texas Instruments Inc */
	.boot_params	= 0x80000100,
	.reserve	= omap4_panda_reserve,
	.map_io		= omap4_panda_map_io,
	.init_early	= omap4_panda_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= omap4_panda_init,
	.timer		= &omap_timer,
MACHINE_END
