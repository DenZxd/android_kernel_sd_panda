/*
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <linux/if.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <plat/mmc.h>

#include <linux/random.h>
#include <linux/jiffies.h>

#include "hsmmc.h"
#include "control.h"
#include "mux.h"
#include "board-panda.h"

#if defined(CONFIG_BCM4329) || defined(CONFIG_BCM4329_MODULE) || \
    defined(CONFIG_BCM4330) || defined(CONFIG_BCM4330_MODULE) || \
    defined(CONFIG_BCMDHD)  || defined(CONFIG_BCMDHD_MODULE)
#define WL_RST_N 40
#define GPIO_WLAN_IRQ	53

#if defined(CONFIG_DHD_USE_STATIC_BUF)
#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *panda_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

int __init panda_init_wifi_mem(void)
{
	int i;

	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
}
#endif

static struct resource panda_wifi_resources[] = {
	[0] = {
		.name       = "bcmdhd_wlan_irq",
		.start      = OMAP_GPIO_IRQ(GPIO_WLAN_IRQ),
		.end        = OMAP_GPIO_IRQ(GPIO_WLAN_IRQ),
		.flags      = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL |
			      IORESOURCE_IRQ_SHAREABLE,
	},
};

static int panda_wifi_cd = 0; /* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static struct regulator *clk32kg_reg;

static int panda_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;

	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;

	return 0;
}

static unsigned int panda_wifi_status(struct device *dev)
{
	return panda_wifi_cd;
}

struct mmc_platform_data panda_wifi_data = {
	.ocr_mask		= MMC_VDD_165_195 | MMC_VDD_33_34,
	.built_in		= 1,
	.status			= panda_wifi_status,
	.card_present		= 0,
	.register_status_notify	= panda_wifi_status_register,
};

static int panda_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);

	panda_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid); else
		pr_warning("%s: Nobody to notify\n", __func__);

	return 0;
}
static int tuna_wifi_power_state;
static int panda_wifi_power(int on)
{
	if (!clk32kg_reg) {
		clk32kg_reg = regulator_get(0, "clk32kgate");
		if (IS_ERR(clk32kg_reg)) {
			pr_err("%s: clk32kgate reg not found!\n", __func__);
			clk32kg_reg = NULL;
		}
	}


	if (clk32kg_reg && on && !tuna_wifi_power_state)
		regulator_enable(clk32kg_reg);
	msleep_interruptible(100);	// XXX:

	pr_debug("%s: %d\n", __func__, on);

	gpio_set_value(WL_RST_N, on);

	msleep_interruptible(100);	// XXX:

	if (clk32kg_reg && !on && tuna_wifi_power_state)
		regulator_disable(clk32kg_reg);

	tuna_wifi_power_state = on;

	return 0;
}

static int panda_wifi_reset(int on)
{
	pr_debug("%s: %d\n", __func__, on);

	gpio_set_value(WL_RST_N, 1);

	return 0;
}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ	4
typedef struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
} cntry_locales_custom_t;

static cntry_locales_custom_t panda_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XY", 4},  /* universal */
	{"US", "US", 69}, /* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69}, /* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},  /* European union countries */
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},  /* input ISO "GB" to : EU regrev 05 */
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3},  /* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3}
};

static void *panda_wifi_get_country_code(char *ccode)
{
	int i;

	if (!ccode)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(panda_wifi_translate_custom_table); ++i)
		if (strcmp(panda_wifi_translate_custom_table[i].iso_abbrev,
				ccode) == 0)
			return &panda_wifi_translate_custom_table[i];

	return &panda_wifi_translate_custom_table[0];
}

static struct wifi_platform_data panda_wifi_control = {
	.set_power      = panda_wifi_power,
	.set_reset      = panda_wifi_reset,
	.set_carddetect = panda_wifi_set_carddetect,
#if defined(CONFIG_DHD_USE_STATIC_BUF)
	.mem_prealloc	= panda_wifi_mem_prealloc,
#endif
	.get_country_code = panda_wifi_get_country_code,
};

static struct platform_device panda_wifi_device = {
        .name           = "bcm4329_wlan",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(panda_wifi_resources),
        .resource       = panda_wifi_resources,
        .dev            = {
                .platform_data = &panda_wifi_control,
        },
};

int __init panda_wlan_init(void)
{
	int ret;

	omap_mux_init_gpio(WL_RST_N, OMAP_PIN_OUTPUT);

	ret = gpio_request(WL_RST_N, "wifi_reset");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__, WL_RST_N);
		return ret;
	} else gpio_direction_output(WL_RST_N, 0);

	omap_mux_init_gpio(GPIO_WLAN_IRQ, OMAP_PIN_INPUT);

	ret = gpio_request(GPIO_WLAN_IRQ, "wlan_irq");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n",
			__func__, GPIO_WLAN_IRQ);
		return ret;
	} else gpio_direction_input(GPIO_WLAN_IRQ);

#if defined(CONFIG_DHD_USE_STATIC_BUF)
	panda_init_wifi_mem();
#endif

	return platform_device_register(&panda_wifi_device);
}
#else
int __init panda_wlan_init(void) { }
#endif
