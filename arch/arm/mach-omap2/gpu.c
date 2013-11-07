/*
 * Deassert reset for AM33xx graphics device(SGX) hwmod
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * Prathap MS <msprathap@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/err.h>
#include <linux/of_platform.h>
#include "omap_device.h"

void __init omap_sgx_init_of(void)
{
	struct device_node *node;
	struct platform_device *pdev;
	const char* dev_name = "pvrsrvkm";
	const char* oh_name = "gfx";
	struct omap_hwmod* oh;
	int ret = 0;

	node = of_find_compatible_node(NULL, NULL, "ti,sgx");
	if (!node)
		return;

	pdev = of_find_device_by_node(node);
	if (!pdev) {
		pr_warn("of_find_device_by_node() failed for sgx\n");
		return;
	}

	//of_property_read_string_index(node, "ti,hwmods", 0, &oh_name);
	//if (!oh_name) return;

	ret = omap_device_deassert_hardreset(pdev, oh_name);
	if (ret != 0)
		pr_warn("omap_device_deassert_hardreset() failed for sgx(gfx hwmod)\n");

	if (!(oh = omap_hwmod_lookup(oh_name))) return;
	pdev = omap_device_build(dev_name, -1, oh, NULL, 0, NULL, 0, 0);
	WARN(IS_ERR(pdev), "could not build omap_device for %s\n", dev_name);

	node = of_find_compatible_node(NULL, NULL, "ti,omap2-timer");
	if (!node)
		return;

	pdev = of_find_device_by_node(node);
	if (!pdev) {	// XXX: why failed here?
		pr_warn("of_find_device_by_node() failed for timer7\n");
		return;
	}

	ret = omap_device_deassert_hardreset(pdev, "timer7");
	if (ret != 0)
		pr_warn("omap_device_deassert_hardreset() failed for timer7\n");
}

