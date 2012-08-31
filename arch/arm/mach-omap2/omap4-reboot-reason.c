/*
 * Reboot reason special handler
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/notifier.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include "omap4-sar-layout.h"

#ifdef CONFIG_OMAP4_DPLL_CASCADING
static struct device fake_reset_dev;
#endif

#include <linux/i2c/twl.h>

#define START_ON_RTC (1 << 4)
#define PHOENIX_START_CONDITION 0x1F

static int twl_rtc_reboot(void)
{
    int ret;
    u8 start_state = 0;
    ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &start_state,
	    PHOENIX_START_CONDITION);
    if (ret)
	start_state = 0;

    if (start_state & START_ON_RTC) {
	start_state = 1;
	twl_i2c_write_u8(TWL6030_MODULE_ID0, start_state & ~START_ON_RTC,
		PHOENIX_START_CONDITION);
    } else
	start_state = 0;

    return start_state;
}

static int omap_reboot_notifier_call(struct notifier_block *this,
					unsigned long code, void *cmd)
{
	void __iomem *sar_base;
	char *reason = "normal";

#ifdef CONFIG_OMAP4_DPLL_CASCADING
	pr_info("%s: exit DPLL cascading\n", __func__);
	omap4_dpll_cascading_blocker_hold(&fake_reset_dev);
#endif

	sar_base = omap4_get_sar_ram_base();

	if (!sar_base)
		return notifier_from_errno(-ENOMEM);

	/* Save reboot mode in scratch memory */
	if (code == SYS_RESTART && cmd != NULL && strlen(cmd))
		reason = cmd;
	else if (code == SYS_POWER_OFF)
		reason = "off";

	strncpy(sar_base + OMAP_REBOOT_REASON_OFFSET,
			reason, OMAP_REBOOT_REASON_SIZE);
	return NOTIFY_DONE;
}

static struct notifier_block omap_reboot_notifier = {
	.notifier_call = omap_reboot_notifier_call,
};

char omap4_reboot_reason[OMAP_REBOOT_REASON_SIZE];

static int __init omap_reboot_reason_init(void)
{
	void __iomem *sar_base;

	sar_base = omap4_get_sar_ram_base();
	if (sar_base) {
		char* ptr = (char*)sar_base + OMAP_REBOOT_REASON_OFFSET;
		strncpy(omap4_reboot_reason, ptr, OMAP_REBOOT_REASON_SIZE);
		ptr[0] = '\0';
	}

	if (twl_rtc_reboot())
	    strncpy(omap4_reboot_reason, "rtc_reboot", OMAP_REBOOT_REASON_SIZE);

	return register_reboot_notifier(&omap_reboot_notifier);
}
late_initcall(omap_reboot_reason_init);
