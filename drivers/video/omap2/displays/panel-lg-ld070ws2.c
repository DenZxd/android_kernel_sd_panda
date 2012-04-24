/*
 * LCD panel driver for LG.Philips ld070ws2
 *
 * Author: Chaotian Jing <jingchaotian6@126.com>
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

 * Derived work from panel-lg-ld070ws2.c from Xiaolu.Bao <bxl@hhcn.com>
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>

// 22March2012 ctjing added
#define GPIO_SPI_CSB 134
#define GPIO_SPI_SCL 135
#define GPIO_SPI_SDA 136

static void set_spi_sda(int val)
{
	gpio_set_value(GPIO_SPI_SDA, val);
}

static int get_spi_sda(void)
{
	return !!gpio_get_value(GPIO_SPI_SDA);
}

static void set_spi_clk(int val)
{
	gpio_set_value(GPIO_SPI_SCL, val);
}

static void set_spi_csb(int val)
{
	gpio_set_value(GPIO_SPI_CSB, val);
}

// 10 nsec delay is enough
#define N_USECONDS_DELAY  10
static int  __spi_write_byte(unsigned char data)
{
	signed int bit = 7;

	if (gpio_direction_output(GPIO_SPI_SDA, 0))
	{
		printk("Cannot set GPIO_SPI_SDA to gpio output!\n");
		return -1;
	}
	if (gpio_direction_output(GPIO_SPI_SCL, 0))
	{
		printk("Cannot set GPIO_SPI_SCL to gpio output!\n");
		return -1;
	}
	
	for (; bit >= 0; bit--)
	{
		set_spi_clk(0);
		set_spi_sda(data & (0x01 << bit));
		udelay(N_USECONDS_DELAY);
		set_spi_clk(1);
		udelay(N_USECONDS_DELAY);
	}
	return 0;
}

static int __spi_read_byte(unsigned char *data)
{
	signed int bit = 7;
	unsigned char read_back = 0;
	unsigned char temp = 0;

	if (gpio_direction_input(GPIO_SPI_SDA))
	{
		printk("Cannot set GPIO_SPI_SDA to gpio input!\n");
		return -1;
	}
	if (gpio_direction_output(GPIO_SPI_SCL, 0))
	{
		printk("Cannot set GPIO_SPI_SCL to gpio output!\n");
		return -1;
	}

	for (; bit >= 0; bit--)
	{
		set_spi_clk(0);
		udelay(N_USECONDS_DELAY);

		set_spi_clk(1);
		udelay(N_USECONDS_DELAY);

		read_back <<= 1;
		temp = get_spi_sda();
		//printk("get bit %d\n", temp);
		read_back |= temp;
		udelay(N_USECONDS_DELAY);
	}

	*data = read_back;

	return 0;
}


static void  write_spi_register(unsigned char reg, unsigned char data)
{
	unsigned char data1 = (reg << 2);

	set_spi_csb(0);
	__spi_write_byte(data1);
	__spi_write_byte(data);

	set_spi_csb(1); //delay for next transfer
}

static unsigned char read_spi_register(unsigned char reg)
{
	unsigned char ret = 0;
	unsigned char read_back = 0;
	unsigned char data1 = (reg << 2);
	data1 |= (0x01 << 1); //D9 read

	set_spi_csb(0);
	__spi_write_byte(data1);

	ret = __spi_read_byte(&read_back);
	if (ret)
	{
		printk(KERN_ERR "failed when reading spi register!\n");
	}

	set_spi_csb(1); //delay for next trasnsfer
	return read_back;
}

typedef struct reg_set {
	unsigned char reg;
	unsigned char val;
}reg_set_t;

// LD070WS2 initialize sequence
reg_set_t array_reg_set[] = {
	{0x00, 0x21},
	{0x00, 0xa5},
	{0x01, 0x30},
	{0x02, 0x40},
	{0x0e, 0x5f},
	{0x0f, 0xa4},
	{0x0d, 0x00},
	{0x02, 0x43},
	{0x0a, 0x28},
	{0x10, 0x41},
	{0x00, 0xad}
};

#define array_size(array) (sizeof(array) / sizeof(array[0]))
// invoked at probe
static void ld070ws2_reg_init(void)
{
	//initialize the registers see document
	int i = 0;
	while (i < array_size(array_reg_set))
	{
		write_spi_register(array_reg_set[i].reg, array_reg_set[i].val);
		i++;
	}
	// sleep 200 ms to avoid blink
	msleep_interruptible(200);
}

struct ld070ws2_data {
	struct mutex lock;
};

static struct omap_video_timings ld070ws2_timings = {
	.x_res = 1024,
	.y_res = 600,

	.pixel_clock	= 56888,

	.hsw		= 68,
	.hfp		= 160,
	.hbp		= 160,

	.vsw		= 8,
	.vfp		= 40,
	.vbp		= 23,
};

static inline struct panel_generic_dpi_data
*get_panel_data(const struct omap_dss_device *dssdev)
{
	return (struct panel_generic_dpi_data *) dssdev->data;
}

static int ld070ws2_panel_power_on(struct omap_dss_device *dssdev)
{
	int r;
	struct panel_generic_dpi_data *panel_data = get_panel_data(dssdev);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;
	if (panel_data->platform_enable) {
		r = panel_data->platform_enable(dssdev);
		if (r)
			goto err1;
	}
	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
		{
			goto err1;
		}
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void ld070ws2_panel_power_off(struct omap_dss_device *dssdev)
{
	struct panel_generic_dpi_data *panel_data = get_panel_data(dssdev);
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (panel_data->platform_disable)
		panel_data->platform_disable(dssdev);

	if (dssdev->platform_disable)
	{
		dssdev->platform_disable(dssdev);
	}

	omapdss_dpi_display_disable(dssdev);
}

static int ld070ws2_panel_probe(struct omap_dss_device *dssdev)
{
	struct ld070ws2_data *ld;
	int r;

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = ld070ws2_timings;

	ld = kzalloc(sizeof(*ld), GFP_KERNEL);
	if (!ld) {
		r = -ENOMEM;
		goto err;
	}
	mutex_init(&ld->lock);
	dev_set_drvdata(&dssdev->dev, ld);
	return 0;
err:
	return r;
}

static void ld070ws2_panel_remove(struct omap_dss_device *dssdev)
{
	struct ld070ws2_data *ld = dev_get_drvdata(&dssdev->dev);

	kfree(ld);
}

static int ld070ws2_panel_enable(struct omap_dss_device *dssdev)
{
	struct ld070ws2_data *ld = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&ld->lock);

	r = ld070ws2_panel_power_on(dssdev);
	if (r)
		goto err;
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&ld->lock);
	return 0;
err:
	mutex_unlock(&ld->lock);
	return r;
}

static void ld070ws2_panel_disable(struct omap_dss_device *dssdev)
{
	struct ld070ws2_data *ld = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&ld->lock);

	ld070ws2_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	mutex_unlock(&ld->lock);
}

static int ld070ws2_panel_suspend(struct omap_dss_device *dssdev)
{
	struct ld070ws2_data *ld = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&ld->lock);

	ld070ws2_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	/* avoid Leakage */
	gpio_set_value(GPIO_SPI_CSB, 0);
	gpio_set_value(GPIO_SPI_SCL, 0);
	gpio_set_value(GPIO_SPI_SDA, 0);

	mutex_unlock(&ld->lock);
	return 0;
}

static int ld070ws2_panel_resume(struct omap_dss_device *dssdev)
{
	struct ld070ws2_data *ld = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&ld->lock);

	//Note that must power on first, then reinitialize LCD registers by spi
	r = ld070ws2_panel_power_on(dssdev);
	if (r)
		goto err;

	// need reinit when resume
	ld070ws2_reg_init();

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&ld->lock);
	return 0;
err:
	mutex_unlock(&ld->lock);
	return r;
}

static struct omap_dss_driver ld070ws2_driver = {
	.probe		= ld070ws2_panel_probe,
	.remove		= ld070ws2_panel_remove,

	.enable		= ld070ws2_panel_enable,
	.disable	= ld070ws2_panel_disable,
	.suspend	= ld070ws2_panel_suspend,
	.resume		= ld070ws2_panel_resume,

	.driver         = {
		.name   = "lg_ld070ws2_panel",
		.owner  = THIS_MODULE,
	},
};


static int ld070ws2_panel_spi_probe(struct platform_device *pdev)
{
	ld070ws2_reg_init();
	return omap_dss_register_driver(&ld070ws2_driver);
}

static int ld070ws2_panel_spi_remove(struct platform_device *pdev)
{
	omap_dss_unregister_driver(&ld070ws2_driver);
	return 0;
}

static struct platform_driver ld070ws2_spi_driver = {
	.driver.name	= "lg_ips7_panel",
	.driver.owner	= THIS_MODULE,
	.probe     	= ld070ws2_panel_spi_probe,
	.remove		= __devexit_p(ld070ws2_panel_spi_remove),
};

static int  __init ld070ws2_panel_drv_init(void)
{
	int result = 0;
	result = platform_driver_register(&ld070ws2_spi_driver);
	return result;
}

static void  __exit ld070ws2_panel_drv_exit(void)
{
	platform_driver_unregister(&ld070ws2_spi_driver);
}

arch_initcall(ld070ws2_panel_drv_init);
module_exit(ld070ws2_panel_drv_exit);

MODULE_LICENSE("GPL");
