/* linux/arch/arm/mach-s3c2440/mach-smdk2440.c
 *
 * Copyright (c) 2004,2005 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * http://www.fluff.org/ben/smdk2440/
 *
 * Thanks to Dimity Andric and TomTom for the loan of an SMDK2440.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <linux/dm9000.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/gpio_keys.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/can/mcp251x.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/io.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/spi.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/leds-gpio.h>
#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>

#include <mach/idle.h>
#include <mach/fb.h>

#include <asm/plat-s3c/regs-serial.h>
#include <asm/plat-s3c24xx/s3c2410.h>
#include <asm/plat-s3c24xx/s3c2440.h>
#include <asm/plat-s3c24xx/clock.h>
#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/cpu.h>
#include <asm/plat-s3c24xx/udc.h>
#include <asm/plat-s3c24xx/mci.h>
#include <asm/plat-s3c24xx/pm.h>
#include <asm/plat-s3c24xx/ts.h>

#include <asm/plat-s3c24xx/common-smdk.h>

static struct map_desc smdk2440_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}
};

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg smdk2440_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	/* IR port */
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,	// XXX: mhfan
		.ufcon	     = 0x51,
	}
};

/* LCD driver info */

static struct s3c2410fb_display smdk2440_lcd_cfg __initdata = {

	.lcdcon5	= S3C2410_LCDCON5_FRM565 |
			  S3C2410_LCDCON5_INVVLINE |
			  S3C2410_LCDCON5_INVVFRAME |
			  S3C2410_LCDCON5_PWREN |
			  S3C2410_LCDCON5_HWSWP,

	.type		= S3C2410_LCDCON1_TFT,

	.width		= 240,
	.height		= 320,

	.pixclock	= 166667, /* HCLK 60 MHz, divisor 10 */
	.xres		= 240,
	.yres		= 320,
	.bpp		= 16,
	.left_margin	= 20,
	.right_margin	= 8,
	.hsync_len	= 4,
	.upper_margin	= 8,
	.lower_margin	= 7,
	.vsync_len	= 4,
};

static struct s3c2410fb_mach_info smdk2440_fb_info __initdata = {
	.displays	= &smdk2440_lcd_cfg,
	.num_displays	= 1,
	.default_display = 0,

#if 0
	/* currently setup by downloader */
	.gpccon		= 0xaa940659,
	.gpccon_mask	= 0xffffffff,
	.gpcup		= 0x0000ffff,
	.gpcup_mask	= 0xffffffff,
	.gpdcon		= 0xaa84aaa0,
	.gpdcon_mask	= 0xffffffff,
	.gpdup		= 0x0000faff,
	.gpdup_mask	= 0xffffffff,
#endif

	.lpcsel		= ((0xCE6) & ~7) | 1<<4,
};

static struct s3c2410fb_display hhs3c_lcd_cfg __initdata = {
	// XXX: for hhlcd
	.lcdcon5	= S3C2410_LCDCON5_FRM565 |
			  S3C2410_LCDCON5_PWREN |
			  S3C2410_LCDCON5_HWSWP,

	.type		= S3C2410_LCDCON1_TFT,

	.width		= 240,
	.height		= 320,

	//.pixclock	= 125000, /* HCLK 64 MHz, divisor 7, 8 MHz */
	.pixclock	= 166667,
	.xres		= 240,
	.yres		= 320,
	.bpp		= 16,

	.left_margin	= 30,
	.right_margin	= 20,
	.hsync_len	= 1,

	.upper_margin	= 1,
	.lower_margin	= 1,
	.vsync_len	= 1,
};

static struct s3c2410fb_mach_info hhs3c_fb_info __initdata = {
	.displays	= &hhs3c_lcd_cfg,
	.num_displays	= 1,
	.default_display = 0,

	.gpccon		= 0xaa8002a8,
	.gpccon_mask	= 0xffa003fa,
	.gpcup		= 0x0000f81e,
	.gpcup_mask	= 0xffffffff,

	.gpdcon		= 0xaa80aaa0,
	.gpdcon_mask	= 0xffa0fff0,
	.gpdup		= 0x0000f8fa,
	.gpdup_mask	= 0xffffffff,

	.lpcsel		= 0x02,
	//.lpcsel		= ((0xCE6) & ~7) | 1<<4,
};

static struct s3c2410_ts_mach_info hhs3c_ts_cfg __initdata = {
	.delay = 10000,
	.presc = 49,
	.oversampling_shift = 2,
};

/* DM9000AEP 10/100 ethernet controller */
static struct resource hhs3c_dm9k_resource[] = {
	[0] = {
		.start = S3C2410_CS1,
		.end   = S3C2410_CS1 + 3,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = S3C2410_CS1 + 4,
		.end   = S3C2410_CS1 + 7,
		.flags = IORESOURCE_MEM
	},
	[2] = {
		.start = IRQ_EINT0,
		.end   = IRQ_EINT0,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct dm9000_plat_data hhtech_dm9k_pdata = {
	.flags		= (DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM),
};

static struct platform_device hhtech_dm9k_dev = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(hhs3c_dm9k_resource),
	.resource	= hhs3c_dm9k_resource,
	.dev		= {
		.platform_data	= &hhtech_dm9k_pdata,
	},
};

static struct s3c2410_spi_info hhs3c_spi_info = {
	.bus_num    = 0,
	.num_cs	    = 1,
	.pin_cs	    = S3C2410_GPG2,	// XXX:
};

static struct s3c24xx_mci_pdata hhs3c_sdi_info = {
	.gpio_detect = S3C2410_GPG10,
	.gpio_wprotect = S3C2410_GPG9,
};

/* Micorchip mcp251x series CAN over spi controller */

static struct mcp251x_platform_data mcp251x_info = {
	.oscillator_frequency = 16000000,
	//.board_specific_setup = hhtech_mcp251x_initfunc,
	//.device_reset = hhtech_mcp251x_reset,
	//.transceiver_enable = NULL,
};

static struct spi_board_info hhs3c_spi_devs[] __initdata = {
	{
		.modalias	= "mcp251x",
		.platform_data	= &mcp251x_info,
		.max_speed_hz	= 8000000,
		.bus_num	= 0,
		.irq		= IRQ_EINT4,
		.chip_select	= 0,	// XXX: S3C2410_GPG2,
	},

};

static struct gpio_keys_button hhs3c_gpio_keys[] = {
	{
		.gpio		= S3C2410_GPF3,
		.code		= BTN_0,
		.desc		= "SW-PB/ICK1",
		.debounce_interval = 5,
		.active_low	= 1,
	}, {
		.gpio		= S3C2410_GPF2,
		.code		= BTN_1,
		.desc		= "SW-PB/ICK2",
		.debounce_interval = 5,
		.active_low	= 1,
	}, {
		.gpio		= S3C2410_GPF0,
		.code		= BTN_2,
		.desc		= "SW-PB/ICK3",
		.debounce_interval = 5,
		.active_low	= 1,
	}, {
		.gpio		= S3C2410_GPF1,
		.code		= BTN_3,
		.desc		= "SW-PB/ICK4",
		.debounce_interval = 5,
		.active_low	= 1,
	}, {
		.gpio		= S3C2410_GPF5,
		.code		= BTN_4,
		.desc		= "SW-PB/ICK5",
		.debounce_interval = 5,
		.active_low	= 1,
	},
};

static struct gpio_keys_platform_data hhtech_gpio_keys_data = {
	.buttons	= hhs3c_gpio_keys,
	.nbuttons	= ARRAY_SIZE(hhs3c_gpio_keys),
};

static struct platform_device hhtech_gpio_keys_dev = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &hhtech_gpio_keys_data,
	}
};

static struct s3c24xx_led_platdata hhs3c_led1 = {
        .gpio           = S3C2410_GPB0,
        .flags          = S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
        .name           = "led_1",
};

static struct s3c24xx_led_platdata hhs3c_led2 = {
        .gpio           = S3C2410_GPB1,
        .flags          = S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
        .name           = "led_2",
};

static struct platform_device hhs3c_led1_dev = {
        .name           = "s3c24xx_led",
        .id             = 0,
        .dev            = {
                .platform_data = &hhs3c_led1,
        },
};

static struct platform_device hhs3c_led2_dev = {
        .name           = "s3c24xx_led",
        .id             = 1,
        .dev            = {
                .platform_data = &hhs3c_led2,
        },
};

static struct gpio_led hhs3c_leds[] = {
	[0] = {
		.name = "led-1",
		.gpio = S3C2410_GPB0,
		.active_low = 1,
	},
	[1] = {
		.name = "led-2",
		.gpio = S3C2410_GPB1,
		.active_low = 1,
	},

	[2] = {
		.name = "led-3",
		.gpio = S3C2410_GPB10,
		.active_low = 1,
	},
	[3] = {
		.name = "led-4",
		.gpio = S3C2410_GPB9,
		.active_low = 1,
	},
};

static struct gpio_led_platform_data hhtech_leds_pdata = {
	.num_leds = ARRAY_SIZE(hhs3c_leds),
	.leds = hhs3c_leds,
};

static struct platform_device hhtech_leds_dev = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &hhtech_leds_pdata,
	},
};

static struct s3c2410_udc_mach_info hhs3c_udc_cfg = {
};

static struct i2c_board_info hhs3c_i2c_devs[] __initdata = {
	//{ I2C_BOARD_INFO("UDA1380", 0x1a), },	// XXX:
};

static struct platform_device *smdk2440_devices[] __initdata = {
	//&s3c_device_usb,
	&s3c_device_lcd,
	&s3c_device_wdt,
	&s3c_device_i2c,
	&s3c_device_iis,
	&s3c_device_sdi,
	&s3c_device_rtc,
	&s3c_device_adc,
	&s3c_device_ts,
	&s3c_device_spi0,
	&hhtech_dm9k_dev,
	&s3c_device_camif,
	&s3c_device_usbgadget,
	&hhtech_gpio_keys_dev,
	&hhtech_leds_dev,
	&hhs3c_led1_dev,
	&hhs3c_led2_dev,
};

static void __init smdk2440_map_io(void)
{
	s3c24xx_init_io(smdk2440_iodesc, ARRAY_SIZE(smdk2440_iodesc));
	s3c24xx_init_clocks(12000000);	// XXX: 16934400, by mhfan

	__raw_writel((__raw_readl(S3C2410_GPHCON) & ~0xffff) | 0xaaaa,
		S3C2410_GPHCON);	// XXX: enable UART 0/1/2
	__raw_writel((__raw_readl(S3C2410_GPHUP) | 0xff), S3C2410_GPHUP);

	s3c24xx_init_uarts(smdk2440_uartcfgs, ARRAY_SIZE(smdk2440_uartcfgs));
}

static void __init smdk2440_machine_init(void)
{
	s3c24xx_udc_set_platdata(&hhs3c_udc_cfg);
	//s3c24xx_fb_set_platdata(&smdk2440_fb_info);
	s3c24xx_fb_set_platdata(&hhs3c_fb_info);
	set_s3c2410ts_info(&hhs3c_ts_cfg);

	__raw_writel((__raw_readl(S3C2410_GPECON) & ~0x0fc0) | 0x0a80,
		S3C2410_GPECON);	// XXX: enable SPI0 pins
	__raw_writel((__raw_readl(S3C2410_GPEUP) | 0x38), S3C2410_GPEUP);

	s3c_device_sdi.dev.platform_data = &hhs3c_sdi_info;
	s3c_device_spi0.dev.platform_data = &hhs3c_spi_info;
	spi_register_board_info(hhs3c_spi_devs, ARRAY_SIZE(hhs3c_spi_devs));
	i2c_register_board_info(0, hhs3c_i2c_devs, ARRAY_SIZE(hhs3c_i2c_devs));

	platform_add_devices(smdk2440_devices, ARRAY_SIZE(smdk2440_devices));

	//smdk_machine_init();		// XXX: mhfan
	s3c2410_pm_init();
}

MACHINE_START(S3C2440, "SMDK2440")
	/* Maintainer: Ben Dooks <ben@fluff.org> */
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= smdk2440_map_io,
	.init_machine	= smdk2440_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
