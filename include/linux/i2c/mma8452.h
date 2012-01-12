/*
 * mma8452.h
 * MMA-8452 Accelerometer driver
 *
 * Copyright (C) 2012
 * Author: Juntao Yao <yaojuntao@hhcn.com>
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

#ifndef _LINUX_MMA8452_I2C_H
#define _LINUX_MMA8452_I2C_H

#define DEVICE_NAME					   "mma8452"

/* MMA8452 register address */
#define MMA8452_STATUS                 0x00
#define MMA8452_REG_CTRL               0x2a
#define MMA8452_REG_DATA               0x01
#define MMA8452_XYZ_DATA_CFG           0x0e

/* MMA8452 control bit */
#define MMA8452_STATUS_ZYXDR           0x08
#define MMA8452_CTRL_MODE_2G           0x00
#define MMA8452_CTRL_MODE_4G           0x01
#define MMA8452_CTRL_MODE_8G           0x02
#define MMA8452_CTRL_ACTIVE            0x01    /* ACTIVE */
#define MMA8452_CTRL_LNOISE            (0x1 << 2)

#define MMA_BW_2HZ          0x0
#define MMA_BW_7HZ          0x1
#define MMA_BW_13HZ         0x2
#define MMA_BW_50HZ         0x3
#define MMA_BW_100HZ        0x4
#define MMA_BW_200HZ        0x5
#define MMA_BW_400HZ        0x6
#define MMA_BW_800HZ        0x7

#define G_RANGE             2048
#endif
