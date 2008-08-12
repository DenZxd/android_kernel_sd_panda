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
#include <linux/interrupt.h>
#include <linux/dm9000.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/gpio_keys.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/can/mcp251x.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/io.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/plat-s3c/regs-serial.h>
#include <mach/leds-gpio.h>
#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>

#include <mach/idle.h>
#include <mach/fb.h>

#include <asm/plat-s3c24xx/s3c2410.h>
#include <asm/plat-s3c24xx/s3c2440.h>
#include <asm/plat-s3c24xx/clock.h>
#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/cpu.h>

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
		.ulcon	     = 0x43,
		.ufcon	     = 0x51,
	}
};

/* LCD driver info */

static struct s3c2410fb_display smdk2440_lcd_cfg __initdata = {

	// FIXME: for hhlcd
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

/* DM9000AEP 10/100 ethernet controller */

static struct resource hhs3c_dm9k_resource[] = {
	[0] = {
		.start = S3C2410_CS3,
		.end   = S3C2410_CS3 + 3,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = S3C2410_CS3 + 4,
		.end   = S3C2410_CS3 + 7,
		.flags = IORESOURCE_MEM
	},
	[2] = {
		.start = IRQ_EINT7,
		.end   = IRQ_EINT7,
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

/* Micorchip mcp251x series CAN over spi controller */

static struct mcp251x_platform_data mcp251x_info = {
	.oscillator_frequency = 19000000,
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
		//.irq		= 10,
		.chip_select	= 0,
	},

};

static struct gpio_keys_button hhs3c_gpio_keys[] = {
	{
		.gpio		= S3C2410_GPF3,
		.code		= BTN_0,
		.desc		= "SW-PB/ICK1",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPF2,
		.code		= BTN_1,
		.desc		= "SW-PB/ICK2",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPB10,
		.code		= BTN_2,
		.desc		= "SW-PB/ICK3",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPF1,
		.code		= BTN_3,
		.desc		= "SW-PB/ICK4",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPB9,
		.code		= BTN_4,
		.desc		= "SW-PB/ICK5",
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
        .name           = "led1/red",
};

static struct s3c24xx_led_platdata hhs3c_led2 = {
        .gpio           = S3C2410_GPB1,
        .flags          = S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
        .name           = "led2/green",
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
		.name = "led1:red",
		.gpio = S3C2410_GPB0,
		.active_low = 1,
	},
	[1] = {
		.name = "led1:green",
		.gpio = S3C2410_GPB1,
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

static struct platform_device *smdk2440_devices[] __initdata = {
	&s3c_device_usb,
	&s3c_device_lcd,
	&s3c_device_wdt,
	&s3c_device_i2c,
	&s3c_device_iis,
	&s3c_device_sdi,
	&s3c_device_spi0,
	&hhtech_dm9k_dev,
	&hhtech_leds_dev,
	&hhtech_gpio_keys_dev,
	//&hhtech_leds_dev,
	&hhs3c_led1_dev,
	&hhs3c_led2_dev,
};

static void __init smdk2440_map_io(void)
{
	s3c24xx_init_io(smdk2440_iodesc, ARRAY_SIZE(smdk2440_iodesc));
	s3c24xx_init_clocks(12000000);	// XXX: 16934400, by mhfan
	s3c24xx_init_uarts(smdk2440_uartcfgs, ARRAY_SIZE(smdk2440_uartcfgs));
}

static void __init smdk2440_machine_init(void)
{
	s3c24xx_fb_set_platdata(&smdk2440_fb_info);
	spi_register_board_info(hhs3c_spi_devs, ARRAY_SIZE(hhs3c_spi_devs));

	platform_add_devices(smdk2440_devices, ARRAY_SIZE(smdk2440_devices));
	smdk_machine_init();
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
