/*
 *  linux/drivers/mmc/s3cmci.h - Samsung S3C MCI driver
 *
 *  Copyright (C) 2004-2006 Thomas Kleffel, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* FIXME: DMA Resource management ?! */
#define S3CMCI_DMA 0	// XXX: mhfan

enum s3cmci_waitfor {
	COMPLETION_NONE,
	COMPLETION_FINALIZE,
	COMPLETION_CMDSENT,
	COMPLETION_RSPFIN,
	COMPLETION_XFERFINISH,
	COMPLETION_XFERFINISH_RSPFIN,
};

struct s3cmci_host {
	struct platform_device	*pdev;
	struct s3c24xx_mci_pdata *pdata;
	struct mmc_host		*mmc;
	struct resource		*mem;
	struct clk		*clk;
	void __iomem		*base;

	short			irq;
	short			irq_cd;

	unsigned long		clk_rate;
	unsigned long		real_rate;
	unsigned short		clk_div;

	u8			prescaler;

	u8			dma:1;
	u8			dodma:1;
	u8			is2440:1;

	u8			cmd_is_stop:1;
	u8			dma_complete:1;

	int			dmatogo;
	unsigned		sdiimsk;
	unsigned		sdidata;

	struct mmc_request	*mrq;

	spinlock_t		complete_lock;
	enum s3cmci_waitfor	complete_what;

	u32			pio_sgptr;
	u32			pio_words;
	u32			pio_count;
	u32			*pio_ptr;
#define XFER_NONE 0
#define XFER_READ 1
#define XFER_WRITE 2
	u16			pio_active;
	u16			bus_width;

#ifdef CONFIG_MMC_DEBUG
	char 			dbgmsg_cmd[301];
	char 			dbgmsg_dat[301];
#endif
	char			*status;

	unsigned int		ccnt, dcnt;
	struct tasklet_struct	pio_tasklet;
};
