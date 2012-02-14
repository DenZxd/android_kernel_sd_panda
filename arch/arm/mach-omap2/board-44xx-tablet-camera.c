#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <plat/i2c.h>
#include <plat/omap-pm.h>

#include <asm/mach-types.h>

#include <media/ov5640.h>
#include <media/ov5650.h>

#include "devices.h"
#include "../../../drivers/media/video/omap4iss/iss.h"

#include "control.h"
#include "mux.h"

#define TABLET_GPIO_CAM1_PWRDN		37
#define TABLET_GPIO_CAM2_PWRDN		37
#define TABLET_GPIO_CAM_RESET		83

static struct clk *tablet_cam1_aux_clk;
static struct clk *tablet_cam2_aux_clk;

static int tablet_ov_cam1_power(struct v4l2_subdev *subdev, int on)
{
	struct device *dev = subdev->v4l2_dev->dev;

	if (on) {
		int ret;

		gpio_set_value(TABLET_GPIO_CAM1_PWRDN, 1);
		ret = clk_enable(tablet_cam1_aux_clk);
		if (ret) {
			dev_err(dev,
				"Error in clk_enable() in %s(%d)\n",
				__func__, on);
			gpio_set_value(TABLET_GPIO_CAM1_PWRDN, 0);
			return ret;
		}
		mdelay(2);
	} else {
		clk_disable(tablet_cam1_aux_clk);
		gpio_set_value(TABLET_GPIO_CAM1_PWRDN, 0);
	}

	return 0;
}

static int tablet_ov_cam2_power(struct v4l2_subdev *subdev, int on)
{
	struct device *dev = subdev->v4l2_dev->dev;

	if (on) {
		int ret;

		gpio_set_value(TABLET_GPIO_CAM2_PWRDN, 1);
		ret = clk_enable(tablet_cam2_aux_clk);
		if (ret) {
			dev_err(dev,
				"Error in clk_enable() in %s(%d)\n",
				__func__, on);
			gpio_set_value(TABLET_GPIO_CAM2_PWRDN, 0);
			return ret;
		}
		mdelay(2);
	} else {
		clk_disable(tablet_cam2_aux_clk);
		gpio_set_value(TABLET_GPIO_CAM2_PWRDN, 0);
	}

	return 0;
}

#define OV5640_I2C_ADDRESS   (0x3C)

static struct ov5640_platform_data ov5640_cam1_platform_data = {
      .s_power = tablet_ov_cam1_power,
};

static struct i2c_board_info ov5640_cam1_i2c_device = {
	I2C_BOARD_INFO("ov5640", OV5640_I2C_ADDRESS),
	.platform_data = &ov5640_cam1_platform_data,
};

static struct ov5640_platform_data ov5640_cam2_platform_data = {
      .s_power = tablet_ov_cam2_power,
};

static struct i2c_board_info ov5640_cam2_i2c_device = {
	I2C_BOARD_INFO("ov5640", OV5640_I2C_ADDRESS),
	.platform_data = &ov5640_cam2_platform_data,
};

#define OV5650_I2C_ADDRESS   (0x36)

static struct ov5650_platform_data ov5650_cam1_platform_data = {
      .s_power = tablet_ov_cam1_power,
};

static struct i2c_board_info ov5650_cam1_i2c_device = {
	I2C_BOARD_INFO("ov5650", OV5650_I2C_ADDRESS),
	.platform_data = &ov5650_cam1_platform_data,
};

static struct ov5650_platform_data ov5650_cam2_platform_data = {
      .s_power = tablet_ov_cam2_power,
};

static struct i2c_board_info ov5650_cam2_i2c_device = {
	I2C_BOARD_INFO("ov5650", OV5650_I2C_ADDRESS),
	.platform_data = &ov5650_cam2_platform_data,
};

static struct iss_subdev_i2c_board_info ov5640_cam1_subdevs[] = {
	{
		.board_info = &ov5640_cam1_i2c_device,
		.i2c_adapter_id = 3,
	},
	{ NULL, 0, },
};

static struct iss_subdev_i2c_board_info ov5650_cam1_subdevs[] = {
	{
		.board_info = &ov5650_cam1_i2c_device,
		.i2c_adapter_id = 3,
	},
	{ NULL, 0, },
};

static struct iss_subdev_i2c_board_info ov5640_cam2_subdevs[] = {
	{
		.board_info = &ov5640_cam2_i2c_device,
		.i2c_adapter_id = 2,
	},
	{ NULL, 0, },
};

static struct iss_subdev_i2c_board_info ov5650_cam2_subdevs[] = {
	{
		.board_info = &ov5650_cam2_i2c_device,
		.i2c_adapter_id = 2,
	},
	{ NULL, 0, },
};

static struct iss_v4l2_subdevs_group tablet_camera_subdevs[] = {
	{
		.subdevs = ov5640_cam1_subdevs,
		.interface = ISS_INTERFACE_CSI2A_PHY1,
		.bus = { .csi2 = {
			.lanecfg	= {
				.clk = {
					.pol = 0,
					.pos = 1,
				},
				.data[0] = {
					.pol = 0,
					.pos = 2,
				},
			},
		} },
	},
	{
		.subdevs = ov5650_cam1_subdevs,
		.interface = ISS_INTERFACE_CSI2A_PHY1,
		.bus = { .csi2 = {
			.lanecfg	= {
				.clk = {
					.pol = 0,
					.pos = 1,
				},
				.data[0] = {
					.pol = 0,
					.pos = 2,
				},
			},
		} },
	},
	{
		.subdevs = ov5640_cam2_subdevs,
		.interface = ISS_INTERFACE_CSI2B_PHY2,
		.bus = { .csi2 = {
			.lanecfg	= {
				.clk = {
					.pol = 0,
					.pos = 1,
				},
				.data[0] = {
					.pol = 0,
					.pos = 2,
				},
			},
		} },
	},
	{
		.subdevs = ov5650_cam2_subdevs,
		.interface = ISS_INTERFACE_CSI2B_PHY2,
		.bus = { .csi2 = {
			.lanecfg	= {
				.clk = {
					.pol = 0,
					.pos = 1,
				},
				.data[0] = {
					.pol = 0,
					.pos = 2,
				},
			},
		} },
	},
	{ },
};

static void tablet_omap4iss_set_constraints(struct iss_device *iss, bool enable)
{
	if (!iss)
		return;

	/* FIXME: Look for something more precise as a good throughtput limit */
	omap_pm_set_min_bus_tput(iss->dev, OCP_INITIATOR_AGENT,
				 enable ? 800000 : -1);
}

static struct iss_platform_data tablet_iss_platform_data = {
	.subdevs = tablet_camera_subdevs,
	.set_constraints = tablet_omap4iss_set_constraints,
};


static struct omap_device_pad omap4iss_pads[] = {
	/* CSI2-A */
	{
		.name   = "csi21_dx0.csi21_dx0",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi21_dy0.csi21_dy0",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi21_dx1.csi21_dx1",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi21_dy1.csi21_dy1",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi21_dx2.csi21_dx2",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi21_dy2.csi21_dy2",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	/* CSI2-B */
	{
		.name   = "csi22_dx0.csi22_dx0",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi22_dy0.csi22_dy0",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi22_dx1.csi22_dx1",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
	{
		.name   = "csi22_dy1.csi22_dy1",
		.enable = OMAP_MUX_MODE0 | OMAP_INPUT_EN,
	},
};

static struct omap_board_data omap4iss_data = {
	.id	    		= 1,
	.pads	 		= omap4iss_pads,
	.pads_cnt       	= ARRAY_SIZE(omap4iss_pads),
};

static int __init tablet_camera_init(void)
{
	if (!machine_is_omap_tabletblaze())
		return 0;

	tablet_cam1_aux_clk = clk_get(NULL, "auxclk1_ck");
	if (IS_ERR(tablet_cam1_aux_clk)) {
		printk(KERN_ERR "Unable to get auxclk1_ck\n");
		return -ENODEV;
	}

	if (clk_set_rate(tablet_cam1_aux_clk,
			clk_round_rate(tablet_cam1_aux_clk, 24000000))) {
		clk_put(tablet_cam1_aux_clk);
		return -EINVAL;
	}

	tablet_cam2_aux_clk = clk_get(NULL, "auxclk2_ck");
	if (IS_ERR(tablet_cam2_aux_clk)) {
		printk(KERN_ERR "Unable to get auxclk2_ck\n");
		clk_put(tablet_cam1_aux_clk);
		return -ENODEV;
	}

	if (clk_set_rate(tablet_cam2_aux_clk,
			clk_round_rate(tablet_cam2_aux_clk, 24000000))) {
		clk_put(tablet_cam1_aux_clk);
		clk_put(tablet_cam2_aux_clk);
		return -EINVAL;
	}

	/* Select GPIO 37 */
	omap_mux_init_gpio(TABLET_GPIO_CAM1_PWRDN, OMAP_PIN_OUTPUT);

	/* Select GPIO 38 */
	omap_mux_init_gpio(TABLET_GPIO_CAM2_PWRDN, OMAP_PIN_OUTPUT);

	/* Select GPIO 83 */
	omap_mux_init_gpio(TABLET_GPIO_CAM_RESET, OMAP_PIN_OUTPUT);

	/* Init FREF_CLK1_OUT */
	omap_mux_init_signal("fref_clk1_out", OMAP_PIN_OUTPUT);

	/* Init FREF_CLK2_OUT */
	omap_mux_init_signal("fref_clk2_out", OMAP_PIN_OUTPUT);

	if (gpio_request_one(TABLET_GPIO_CAM1_PWRDN, GPIOF_OUT_INIT_LOW,
			     "CAM1_PWRDN"))
		printk(KERN_WARNING "Cannot request GPIO %d\n",
			TABLET_GPIO_CAM1_PWRDN);

	if (gpio_request_one(TABLET_GPIO_CAM2_PWRDN, GPIOF_OUT_INIT_LOW,
			     "CAM2_PWRDN"))
		printk(KERN_WARNING "Cannot request GPIO %d\n",
			TABLET_GPIO_CAM2_PWRDN);

	if (gpio_request_one(TABLET_GPIO_CAM_RESET, GPIOF_OUT_INIT_HIGH,
			     "CAM_RESET"))
		printk(KERN_WARNING "Cannot request GPIO %d\n",
			TABLET_GPIO_CAM_RESET);

	omap4_init_camera(&tablet_iss_platform_data, &omap4iss_data);
	return 0;
}
late_initcall(tablet_camera_init);
