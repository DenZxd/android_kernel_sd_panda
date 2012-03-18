/*
 * arch/arm/mach-omap2/board-panda.h
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MACH_OMAP_BOARD_PANDA_H
#define _MACH_OMAP_BOARD_PANDA_H

void omap4_create_board_props(void);

int panda_wlan_init(void);

#if defined(CONFIG_BCM4329) || defined(CONFIG_BCM4329_MODULE) || \
    defined(CONFIG_BCM4330) || defined(CONFIG_BCM4330_MODULE) || \
    defined(CONFIG_BCMDHD)  || defined(CONFIG_BCMDHD_MODULE)
extern struct mmc_platform_data panda_wifi_data;
#endif

#endif
