/*
 * OmniVision OV2655 sensor driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
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

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include <media/ov2655.h>

/* OV2655 has only one fixed colorspace per pixelcode */
struct ov2655_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

enum ov2655_size {
	OV2655_SIZE_QVGA,
	OV2655_SIZE_CIF,
	OV2655_SIZE_VGA,
	OV2655_SIZE_SVGA,
	//OV2655_SIZE_XGA,
	//OV2655_SIZE_720P,
	OV2655_SIZE_SXGA,
	OV2655_SIZE_UXGA,
	OV2655_SIZE_LAST,
};

static const struct v4l2_frmsize_discrete ov2655_frmsizes[OV2655_SIZE_LAST] = {
	{  320,  240 },
	{  352,  288 },
	{  640,  480 },
	{  800,  600 },
	//{ 1024,  768 },
	//{ 1280,  720 },
	{ 1280, 1024 },
	{ 1600, 1200 },
};

struct ov2655_timing_cfg {
	u16 x_addr_start;
	u16 y_addr_start;
	u16 x_addr_end;
	u16 y_addr_end;
	u16 h_output_size;
	u16 v_output_size;
	u16 h_total_size;
	u16 v_total_size;
	u16 isp_h_offset;
	u16 isp_v_offset;
	u8 h_odd_ss_inc;
	u8 h_even_ss_inc;
	u8 v_odd_ss_inc;
	u8 v_even_ss_inc;
};

static const struct ov2655_timing_cfg timing_cfg[OV2655_SIZE_LAST] = {
	[OV2655_SIZE_QVGA] = {
		.x_addr_start = 0,
		.y_addr_start = 0,
		.x_addr_end = 2623,
		.y_addr_end = 1951,
		.h_output_size = 320,
		.v_output_size = 240,
		.h_total_size = 2844,
		.v_total_size = 1968,
		.isp_h_offset = 16,
		.isp_v_offset = 6,
		.h_odd_ss_inc = 1,
		.h_even_ss_inc = 1,
		.v_odd_ss_inc = 1,
		.v_even_ss_inc = 1,
	},
	[OV2655_SIZE_VGA] = {
		.x_addr_start = 0,
		.y_addr_start = 0,
		.x_addr_end = 2623,
		.y_addr_end = 1951,
		.h_output_size = 640,
		.v_output_size = 480,
		.h_total_size = 2844,
		.v_total_size = 1968,
		.isp_h_offset = 16,
		.isp_v_offset = 6,
		.h_odd_ss_inc = 1,
		.h_even_ss_inc = 1,
		.v_odd_ss_inc = 1,
		.v_even_ss_inc = 1,
	},
};

/* Find a frame size in an array */
static int ov2655_find_framesize(u32 width, u32 height)
{
	int i;

	for (i = 0; i < OV2655_SIZE_LAST; i++) {
		if ((ov2655_frmsizes[i].width >= width) &&
		    (ov2655_frmsizes[i].height >= height))
			break;
	}

	/* If not found, select biggest */
	if (i >= OV2655_SIZE_LAST)
		i = OV2655_SIZE_LAST - 1;

	return i;
}

struct ov2655 {
	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct v4l2_ctrl_handler ctrl_handler;

	const struct ov2655_platform_data *pdata;

	struct v4l2_ctrl *pixel_rate;
};

static inline struct ov2655 *to_ov2655(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov2655, subdev);
}

/**
 * struct ov2655_reg - ov2655 register format
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 * @length: length of the register
 *
 * Define a structure for OV2655 register initialization values
 */
struct ov2655_reg {
	u16	reg;
	u8	val;
};

/* TODO: Divide this properly */
static const struct ov2655_reg configscript_common1[] = {
	// IO & Clock & Analog Setup
	{ 0x308c, 0x80 }, 	// disable MIPI
	{ 0x308d, 0x0e }, 	// disable MIPI
	{ 0x360b, 0x00 }, 	// test mode off
	{ 0x30b0, 0xff }, 	// enable DVP
	{ 0x30b1, 0xff }, 	// enable DVP
	{ 0x30b2, 0x04 }, 	// enable DVP

	{ 0x300e, 0x34 }, 	// PLL
	{ 0x300f, 0xa6 }, 	// PLL
	{ 0x3010, 0x81 }, 	// PLL
	{ 0x3082, 0x01 },
	{ 0x30f4, 0x01 },
	{ 0x3090, 0x43 },
	{ 0x3091, 0xc0 },
	{ 0x30ac, 0x42 },

	{ 0x30d1, 0x08 },
	{ 0x30a8, 0x54 },
	{ 0x3015, 0x02 }, 	// VAEC ceiling
	{ 0x3093, 0x00 },
	{ 0x307e, 0xe5 }, 	// digital gain if gain >= 8x
	{ 0x3079, 0x00 },
	{ 0x30aa, 0x52 },
	{ 0x3017, 0x40 }, 	// disable drop frame
	{ 0x30f3, 0x83 },
	{ 0x306a, 0x0c },
	{ 0x306d, 0x00 }, 	// BLC control
	{ 0x336a, 0x3c }, 	// ISP CTRL
	{ 0x3076, 0x6a }, 	// Vsunc is drop when frame drop
	{ 0x30d9, 0x95 },
	{ 0x3016, 0x52 }, 	// Max exposure = Tfrme -2x2 + 1
	{ 0x3601, 0x30 }, 	// DVP output order D[9:2]
	{ 0x304e, 0x88 },
	{ 0x30f1, 0x82 },
	{ 0x306f, 0x14 },
	{ 0x302a, 0x02 },
	{ 0x302b, 0x6a },

	// XXX:
	{ 0x3012, 0x10 },
	{ 0x3011, 0x01 },	// 15 fps

	// AEC/AGC
	{ 0x3013, 0xf7 }, 	// AEC fast, big step, banding on, < band on
				// extreme exposure off, AGC on, AEC on
	{ 0x3018, 0x70 }, 	// stable high
	{ 0x3019, 0x60 }, 	// stable low
	{ 0x301a, 0xd4 }, 	// fast zone

	// XXX:
	{ 0x301c, 0x13 },
	{ 0x301d, 0x17 },
	{ 0x3070, 0x5d },
	{ 0x3072, 0x4d },

	// D5060
	{ 0x30af, 0x00 },
	{ 0x3048, 0x1f },
	{ 0x3049, 0x4e },
	{ 0x304a, 0x20 },
	{ 0x304f, 0x20 },
	{ 0x304b, 0x02 },
	{ 0x304c, 0x00 },
	{ 0x304d, 0x02 },
	{ 0x304f, 0x20 },
	{ 0x30a3, 0x10 },
	{ 0x3014, 0x64 }, 	// Manual 60hz, 50/60 detect on,
				// AddLT1F in AGC, Night mode off,
				// AEC smooth between 50/60,
				// extrem exposure auto,

	// XXX:
	{ 0x3071, 0x00 },
	{ 0x3070, 0xb9 },	// 0x5d
	{ 0x3073, 0x00 },
	{ 0x3072, 0x4d },
	{ 0x301c, 0x02 },	// 0x05
	{ 0x301d, 0x06 },

	{ 0x304d, 0x42 },
	{ 0x304a, 0x40 },
	{ 0x304f, 0x40 },
	{ 0x3095, 0x07 },
	{ 0x3096, 0x16 },
	{ 0x3097, 0x1d },

	// Window Setup XXX:
	{ 0x3020, 0x01 },
	{ 0x3021, 0x18 },
	{ 0x3022, 0x00 },
	{ 0x3023, 0x06 },
	{ 0x3024, 0x06 },
	{ 0x3025, 0x58 },
	{ 0x3026, 0x02 },
	{ 0x3027, 0x61 },
	{ 0x3088, 0x01 },
	{ 0x3089, 0x60 },
	{ 0x308a, 0x01 },
	{ 0x308b, 0x20 },
	{ 0x3316, 0x64 },
	{ 0x3317, 0x25 },
	{ 0x3318, 0x80 },
	{ 0x3319, 0x08 },
	{ 0x331a, 0x28 },
	{ 0x331b, 0x1e },
	{ 0x331c, 0x00 },
	{ 0x331d, 0x38 },
	{ 0x3100, 0x00 },

	// Lens correction
	{ 0x3350, 0x32 },
	{ 0x3351, 0x25 },
	{ 0x3352, 0x80 },
	{ 0x3353, 0x1e },
	{ 0x3354, 0x00 },
	{ 0x3355, 0x84 },
	{ 0x3356, 0x32 },
	{ 0x3357, 0x25 },
	{ 0x3358, 0x80 },
	{ 0x3359, 0x1b },
	{ 0x335a, 0x00 },
	{ 0x335b, 0x84 },
	{ 0x335c, 0x32 },
	{ 0x335d, 0x25 },
	{ 0x335e, 0x80 },
	{ 0x335f, 0x1b },
	{ 0x3360, 0x00 },
	{ 0x3361, 0x84 },
	{ 0x3363, 0x70 },
	{ 0x3364, 0x7f },
	{ 0x3365, 0x00 },
	{ 0x3366, 0x00 },

	// AWB
	{ 0x3320, 0xfa },
	{ 0x3321, 0x11 },
	{ 0x3322, 0x92 },
	{ 0x3323, 0x01 },
	{ 0x3324, 0x97 },
	{ 0x3325, 0x02 },
	{ 0x3326, 0xff },
	{ 0x3327, 0x10 },
	{ 0x3328, 0x10 },
	{ 0x3329, 0x1f },
	{ 0x332a, 0x58 },
	{ 0x332b, 0x50 },
	{ 0x332c, 0xbe },
	{ 0x332d, 0xce },
	{ 0x332e, 0x2e },
	{ 0x332f, 0x36 },
	{ 0x3330, 0x4d },
	{ 0x3331, 0x44 },
	{ 0x3332, 0xf0 },
	{ 0x3333, 0x0a },
	{ 0x3334, 0xf0 },
	{ 0x3335, 0xf0 },
	{ 0x3336, 0xf0 },
	{ 0x3337, 0x40 },
	{ 0x3338, 0x40 },
	{ 0x3339, 0x40 },
	{ 0x333a, 0x00 },
	{ 0x333b, 0x00 },

	// Gamma
	{ 0x3340, 0x09 },
	{ 0x3341, 0x19 },
	{ 0x3342, 0x2f },
	{ 0x3343, 0x45 },
	{ 0x3344, 0x5a },
	{ 0x3345, 0x69 },
	{ 0x3346, 0x75 },
	{ 0x3347, 0x7e },
	{ 0x3348, 0x88 },
	{ 0x3349, 0x96 },
	{ 0x334a, 0xa3 },
	{ 0x334b, 0xaf },
	{ 0x334c, 0xc4 },
	{ 0x334d, 0xd7 },
	{ 0x334e, 0xe8 },
	{ 0x334f, 0x20 },

	// Color Matrix
	{ 0x3380, 0x20 },
	{ 0x3381, 0x5b },
	{ 0x3382, 0x05 },
	{ 0x3383, 0x22 },
	{ 0x3384, 0x9d },
	{ 0x3385, 0xc0 },
	{ 0x3386, 0xb6 },
	{ 0x3387, 0xb5 },
	{ 0x3388, 0x02 },
	{ 0x3389, 0x98 },
	{ 0x338a, 0x00 },

	// UVadjust
	{ 0x3301, 0xff }, 	// SDE on, UV adjsut on, color matrix on,
				// sharpen on de-noise on, CIP on, BC on, WC on
	{ 0x338B, 0x1b },	// XXX:
	{ 0x338c, 0x1f }, 	// UV adjust TH1
	{ 0x338d, 0x40 },	// XXX:

	// SDE
	{ 0x3390, 0x45 }, 	// for brightness and sharpness
	{ 0x3391, 0x06 }, 	// saturation on, brightness/sharpness on
	{ 0x339a, 0x00 }, 	// brightness default
	{ 0x3398, 0x20 }, 	// contrast default
	{ 0x3399, 0x20 }, 	// contrast default
	{ 0x3394, 0x40 }, 	// saturation default
	{ 0x3395, 0x40 }, 	// saturation default

	// Sharpness/De-noise
	{ 0x3370, 0xd0 }, 	// de-noise threshold
	{ 0x3371, 0x00 }, 	// sharpness
	{ 0x3372, 0x00 }, 	// SHP TH Man
	{ 0x3373, 0x40 }, 	// DNS offset
	{ 0x3374, 0x10 }, 	// DNS th
	{ 0x3375, 0x10 }, 	// DNS slop
	{ 0x3376, 0x04 }, 	// SHP offset 1
	{ 0x3377, 0x00 }, 	// SHP offset 2
	{ 0x3378, 0x04 }, 	// SHP th1
	{ 0x3379, 0x80 }, 	// SHP th2

	// BLC
	{ 0x3069, 0x86 }, 	// BLC target
	{ 0x307c, 0x11 }, 	// mirror off, flip off, XXX: portrait
	{ 0x3087, 0x02 }, 	// disable always BLC

	// black sun
#if 1
	// Avdd 2.55~3.0V
	{ 0x3090, 0x03 },
	{ 0x30a8, 0x54 },
	{ 0x30aa, 0x82 },
	{ 0x30a3, 0x91 },
	{ 0x30a1, 0x41 },	// XXX:
#else
	// Avdd 2.45~2.7V
	{ 0x3090, 0x03 },
	{ 0x30a8, 0x52 },
	{ 0x30aa, 0x62 },
	{ 0x30a3, 0x91 },
	{ 0x30a1, 0x41 },
#endif

	// Other functions
	{ 0x3300, 0xfc }, 	// ISP on, raw gamma on, AWB stat on,
				// AWB gain on

	{ 0x3302, 0x11 },

	// lenc on, lenc low on, YUV output
	{ 0x3400, 0x00 }, 	// YUV 422 YUYV
	{ 0x3606, 0x20 }, 	// DVP enable
	{ 0x3601, 0x30 }, 	// output D[9:2]
	{ 0x300e, 0x34 }, 	// PLL
	{ 0x30f3, 0x83 },
	{ 0x304e, 0x88 },
#if 0
	{ 0x363b, 0x01 },
	{ 0x363c, 0xf2 },
	{ 0x3085, 0x20 },
#else// XXX:
	// MIPI
	{ 0x363b, 0x01 },
	{ 0x309e, 0x08 },
	{ 0x3606, 0x00 },
	{ 0x3630, 0x35 },

	{ 0x304e, 0x04 },
	{ 0x363b, 0x01 },
	{ 0x309e, 0x08 },
	{ 0x3606, 0x00 },
	{ 0x3084, 0x01 },
	{ 0x3010, 0x80 },	// 0x81
	{ 0x3011, 0x00 },
	{ 0x300e, 0x31 },	// 0x3a
	{ 0x300f, 0xa6 },	//
	{ 0x3634, 0x26 },
#endif

	{ 0x3086, 0x0f }, 	// sleep
	{ 0x3086, 0x00 }, 	// wake up
};

/* TODO: Divide this properly */
static const struct ov2655_reg configscript_common2[] = {
	// union1205 IQ Setting
	// cmx
	{ 0x3380, 0x27 },
	{ 0x3381, 0x5c },
	{ 0x3382, 0x0a },
	{ 0x3383, 0x2f },
	{ 0x3384, 0xc7 },
	{ 0x3385, 0xf7 },
	{ 0x3386, 0xea },
	{ 0x3387, 0xe7 },
	{ 0x3388, 0x03 },
	{ 0x3389, 0x98 },
	{ 0x338a, 0x01 },
	// awb
	{ 0x3320, 0xfa },
	{ 0x3321, 0x11 },
	{ 0x3322, 0x92 },
	{ 0x3323, 0x01 },
	{ 0x3324, 0x97 },
	{ 0x3325, 0x02 },
	{ 0x3326, 0xff },
	{ 0x3327, 0x0c },
	{ 0x3328, 0x10 },
	{ 0x3329, 0x13 },
	{ 0x332a, 0x58 },
	{ 0x332b, 0x5f },
	{ 0x332c, 0xbe },
	{ 0x332d, 0x9b },
	{ 0x332e, 0x3a },
	{ 0x332f, 0x36 },
	{ 0x3330, 0x4d },
	{ 0x3331, 0x44 },
	{ 0x3332, 0xf0 },
	{ 0x3333, 0x0a },
	{ 0x3334, 0xf0 },
	{ 0x3335, 0xf0 },
	{ 0x3336, 0xf0 },
	{ 0x3337, 0x40 },
	{ 0x3338, 0x40 },
	{ 0x3339, 0x40 },
	{ 0x333a, 0x00 },
	{ 0x333b, 0x00 },

	// lens correction
	{ 0x3300, 0xFC },
	// R
	{ 0x3350, 0x32 },
	{ 0x3351, 0x2a },
	{ 0x3352, 0x08 },
	{ 0x3353, 0x27 },
	{ 0x3354, 0x00 },
	{ 0x3355, 0x85 },
	// G
	{ 0x3356, 0x33 },
	{ 0x3357, 0x2a },
	{ 0x3358, 0x08 },
	{ 0x3359, 0x24 },
	{ 0x335a, 0x00 },
	{ 0x335b, 0x85 },
	// B
	{ 0x335c, 0x32 },
	{ 0x335d, 0x2a },
	{ 0x335e, 0x08 },
	{ 0x335f, 0x20 },
	{ 0x3360, 0x00 },
	{ 0x3361, 0x85 },

	{ 0x3363, 0x01 },
	{ 0x3364, 0x03 },
	{ 0x3365, 0x02 },
	{ 0x3366, 0x00 },

	// gamma
	{ 0x334f, 0x20 },
	{ 0x3340, 0x08 },
	{ 0x3341, 0x16 },
	{ 0x3342, 0x2f },
	{ 0x3343, 0x45 },
	{ 0x3344, 0x56 },
	{ 0x3345, 0x66 },
	{ 0x3346, 0x72 },
	{ 0x3347, 0x7c },
	{ 0x3348, 0x86 },
	{ 0x3349, 0x96 },
	{ 0x334a, 0xa3 },
	{ 0x334b, 0xaf },
	{ 0x334c, 0xc4 },
	{ 0x334d, 0xd7 },
	{ 0x334e, 0xe8 },

	// ae
	{ 0x3018, 0x80 },
	{ 0x3019, 0x70 },
	{ 0x301a, 0xd4 },
};

/**
 * ov2655_reg_read - Read a value from a register in an ov2655 sensor device
 * @client: i2c driver client structure
 * @reg: register address / offset
 * @val: stores the value that gets read
 *
 * Read a value from a register in an ov2655 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov2655_reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	u8 data[2] = {0};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 2,
		.buf	= data,
	};

	data[0] = (u8)(reg >> 8);
	data[1] = (u8)(reg & 0xff);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	msg.flags = I2C_M_RD;
	msg.len = 1;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	*val = data[0];
	return 0;

err:
	dev_err(&client->dev, "Failed reading register 0x%02x!\n", reg);
	return ret;
}

/**
 * Write a value to a register in ov2655 sensor device.
 * @client: i2c driver client structure.
 * @reg: Address of the register to read value from.
 * @val: Value to be written to a specific register.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov2655_reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { (u8)(reg >> 8), (u8)(reg & 0xff), val };
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 3,
		.buf	= data,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
}

/**
 * Initialize a list of ov2655 registers.
 * The list of registers is terminated by the pair of values
 * @client: i2c driver client structure.
 * @reglist[]: List of address of the registers to write data.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov2655_reg_writes(struct i2c_client *client,
			     const struct ov2655_reg reglist[],
			     int size)
{
	int err = 0, i;

	for (i = 0; i < size; i++) {
		err = ov2655_reg_write(client, reglist[i].reg,
				reglist[i].val);
		if (err)
			return err;
	}
	return 0;
}

static int ov2655_reg_set(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	u8 tmpval = 0;

	ret = ov2655_reg_read(client, reg, &tmpval);
	if (ret)
		return ret;

	return ov2655_reg_write(client, reg, tmpval | val);
}

static int ov2655_reg_clr(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	u8 tmpval = 0;

	ret = ov2655_reg_read(client, reg, &tmpval);
	if (ret)
		return ret;

	return ov2655_reg_write(client, reg, tmpval & ~val);
}

static int ov2655_config_timing(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2655 *ov2655 = to_ov2655(sd);
	int ret, i;

	i = ov2655_find_framesize(ov2655->format.width, ov2655->format.height);

	ret = ov2655_reg_write(client, 0x3088,
		(ov2655_frmsizes[i].width  >> 8) & 0x0f);
	if (ret) return ret;

	ret = ov2655_reg_write(client, 0x3089,
		 ov2655_frmsizes[i].width  & 0xff);
	if (ret) return ret;

	ret = ov2655_reg_write(client, 0x308a,
		(ov2655_frmsizes[i].height >> 8) & 0x07);
	if (ret) return ret;

	ret = ov2655_reg_write(client, 0x308b,
		 ov2655_frmsizes[i].height & 0xff);
	if (ret) return ret;

	// TODO:

	return ret;
}

static struct v4l2_mbus_framefmt *
__ov2655_get_pad_format(struct ov2655 *ov2655, struct v4l2_subdev_fh *fh,
			 unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov2655->format;
	default:
		return NULL;
	}
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

static int ov2655_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov2655 *ov2655 = to_ov2655(sd);
	int ret = 0;

	if (0 && !on) {
	    struct i2c_client *client = v4l2_get_subdevdata(sd);
	    ov2655_reg_write(client, 0x30ab, 0x00);
	    ov2655_reg_write(client, 0x30ad, 0x0a);
	    ov2655_reg_write(client, 0x30ae, 0x27);
	    //ov2655_reg_read(client, 0x363b, &saved);
	    ov2655_reg_write(client, 0x363b, 0x01);
	}

	ret = ov2655->pdata->s_power(sd, on);

	//if (on) ov2655_reg_write(client, 0x363b, saved);

	return ret;
}

static struct v4l2_subdev_core_ops ov2655_subdev_core_ops = {
	.s_power	= ov2655_s_power,
};

static int ov2655_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_format *format)
{
	struct ov2655 *ov2655 = to_ov2655(sd);

	format->format = *__ov2655_get_pad_format(ov2655, fh, format->pad,
						   format->which);

	return 0;
}

static int ov2655_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_format *format)
{
	struct ov2655 *ov2655 = to_ov2655(sd);
	struct v4l2_mbus_framefmt *__format;

	__format = __ov2655_get_pad_format(ov2655, fh, format->pad,
					    format->which);

	*__format = format->format;

	/* NOTE: This is always true for now, revisit later. */
	ov2655->pixel_rate->cur.val64 = 42000000;	// FIXME:

	return 0;
}

static int ov2655_enum_fmt(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= 2)
		return -EINVAL;

	switch (code->index) {
	case 0:
		code->code = V4L2_MBUS_FMT_UYVY8_1X16;
		break;
	case 1:
		code->code = V4L2_MBUS_FMT_YUYV8_1X16;
		break;
	}
	return 0;
}

static int ov2655_enum_framesizes(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if ((fse->index >= OV2655_SIZE_LAST) ||
	    (fse->code != V4L2_MBUS_FMT_UYVY8_1X16 &&
	     fse->code != V4L2_MBUS_FMT_YUYV8_1X16))
		return -EINVAL;

	fse->min_width = ov2655_frmsizes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ov2655_frmsizes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ov2655_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov2655 *ov2655 = to_ov2655(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (enable) {
		u8 fmtreg = 0;

		switch ((u32)ov2655->format.code) {
		case V4L2_MBUS_FMT_UYVY8_1X16:
			fmtreg = 0x02;
			break;
		case V4L2_MBUS_FMT_YUYV8_1X16:
			fmtreg = 0x00;
			break;
		default:
			/* This shouldn't happen */
			ret = -EINVAL;
			return ret;
		}

		ret = ov2655_reg_write(client, 0x3400, fmtreg);
		if (ret)
			return ret;

		ret = ov2655_config_timing(sd);
		if (ret)
			return ret;

return ret;
		ret = ov2655_reg_write(client, 0x30ad, 0x00);
		if (ret)
			goto out;
		ret = ov2655_reg_write(client, 0x3086, 0x00);
		if (ret)
			goto out;
	} else {
return ret;
		ret = ov2655_reg_write(client, 0x30ad, 0x0a);
		if (ret)
			goto out;
		ret = ov2655_reg_write(client, 0x3086, 0x0f);
		if (ret)
			goto out;
	}

out:
	return ret;
}

static struct v4l2_subdev_video_ops ov2655_subdev_video_ops = {
	.s_stream	= ov2655_s_stream,
};

static struct v4l2_subdev_pad_ops ov2655_subdev_pad_ops = {
	.enum_mbus_code = ov2655_enum_fmt,
	.enum_frame_size = ov2655_enum_framesizes,
	.get_fmt = ov2655_g_fmt,
	.set_fmt = ov2655_s_fmt,
};

static int ov2655_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	/* Quantity of initial bad frames to skip. Revisit. */
	*frames = 3;

	return 0;
}

static struct v4l2_subdev_sensor_ops ov2655_subdev_sensor_ops = {
	.g_skip_frames	= ov2655_g_skip_frames,
};

static struct v4l2_subdev_ops ov2655_subdev_ops = {
	.core	= &ov2655_subdev_core_ops,
	.video	= &ov2655_subdev_video_ops,
	.pad	= &ov2655_subdev_pad_ops,
	.sensor	= &ov2655_subdev_sensor_ops,
};

static int ov2655_registered(struct v4l2_subdev *subdev)
{
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	struct ov2655 *ov2655 = to_ov2655(subdev);
	u8 revision[2] = { 0 };
	int ret = 0;

	ret = ov2655_s_power(subdev, 1);
	if (ret < 0) {
		dev_err(&client->dev, "OV2655 power up failed\n");
		return ret;
	}

	ret = ov2655_reg_read(client, 0x300A, &revision[0]);	// = 0x26
	ret = ov2655_reg_read(client, 0x300B, &revision[1]);	// = 0x56
	if (ret) {
		dev_err(&client->dev, "Failure to detect OV2655 chip\n");
		goto out;
	}

	dev_info(&client->dev, "Detected a OV2655 chip, revision %x%x\n",
		 revision[0], revision[1]);

	/* SW Reset */
	ret = ov2655_reg_write(client, 0x3012, 0x80);
	if (ret)
		goto out;

	msleep(5);

#if 0
	/* sleep mode: SW Powerdown */
	ret = ov2655_reg_write(client, 0x30ad, 0x00);
	if (ret)
		goto out;
	ret = ov2655_reg_write(client, 0x3086, 0x00);
	if (ret)
		goto out;
#endif

	ret = ov2655_reg_writes(client, configscript_common1,
			ARRAY_SIZE(configscript_common1));
	if (ret)
		goto out;

	ret = ov2655_reg_writes(client, configscript_common2,
			ARRAY_SIZE(configscript_common2));
	if (ret)
		goto out;

	/* Init controls */
	ret = v4l2_ctrl_handler_init(&ov2655->ctrl_handler, 1);
	if (ret)
		goto out;

	ov2655->pixel_rate = v4l2_ctrl_new_std(
				&ov2655->ctrl_handler, NULL,
				V4L2_CID_IMAGE_PROC_PIXEL_RATE,
				0, 0, 1, 0);

	subdev->ctrl_handler = &ov2655->ctrl_handler;
out:
	ov2655_s_power(subdev, 0);

	return ret;
}

static int ov2655_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_get_try_format(fh, 0);
	format->code = V4L2_MBUS_FMT_UYVY8_1X16;
	format->width = ov2655_frmsizes[OV2655_SIZE_VGA].width;
	format->height = ov2655_frmsizes[OV2655_SIZE_VGA].height;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_JPEG;

	return 0;
}

static int ov2655_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	return 0;
}

static struct v4l2_subdev_internal_ops ov2655_subdev_internal_ops = {
	.registered = ov2655_registered,
	.open = ov2655_open,
	.close = ov2655_close,
};

static int ov2655_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct ov2655 *ov2655;
	int ret;

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "No platform data!!\n");
		return -ENODEV;
	}

	ov2655 = kzalloc(sizeof(*ov2655), GFP_KERNEL);
	if (!ov2655)
		return -ENOMEM;

	ov2655->pdata = client->dev.platform_data;

	ov2655->format.code = V4L2_MBUS_FMT_UYVY8_1X16;
	ov2655->format.width = ov2655_frmsizes[OV2655_SIZE_VGA].width;
	ov2655->format.height = ov2655_frmsizes[OV2655_SIZE_VGA].height;
	ov2655->format.field = V4L2_FIELD_NONE;
	ov2655->format.colorspace = V4L2_COLORSPACE_JPEG;

	v4l2_i2c_subdev_init(&ov2655->subdev, client, &ov2655_subdev_ops);
	ov2655->subdev.internal_ops = &ov2655_subdev_internal_ops;
	ov2655->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov2655->subdev.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ov2655->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&ov2655->subdev.entity, 1, &ov2655->pad, 0);
	if (ret < 0)
		goto err_mediainit;

	return ret;

err_mediainit:
	v4l2_device_unregister_subdev(&ov2655->subdev);
	kfree(ov2655);
	return ret;
}

static int ov2655_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ov2655 *ov2655 = to_ov2655(subdev);

	v4l2_ctrl_handler_free(&ov2655->ctrl_handler);
	v4l2_device_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);
	kfree(ov2655);
	return 0;
}

static const struct i2c_device_id ov2655_id[] = {
	{ "ov2655", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov2655_id);

static struct i2c_driver ov2655_i2c_driver = {
	.driver = {
		.name = "ov2655",
	},
	.probe		= ov2655_probe,
	.remove		= ov2655_remove,
	.id_table	= ov2655_id,
};

static int __init ov2655_mod_init(void)
{
	return i2c_add_driver(&ov2655_i2c_driver);
}

static void __exit ov2655_mod_exit(void)
{
	i2c_del_driver(&ov2655_i2c_driver);
}

module_init(ov2655_mod_init);
module_exit(ov2655_mod_exit);

MODULE_DESCRIPTION("OmniVision OV2655 Camera driver");
MODULE_AUTHOR("Sergio Aguirre <saaguirre@ti.com>");
MODULE_LICENSE("GPL v2");
