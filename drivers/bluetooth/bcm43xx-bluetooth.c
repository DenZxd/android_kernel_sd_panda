/*
 * Bluetooth Broadcomm  and low power control via GPIO
 *
 *  Copyright (C) 2011 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/rfkill.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/bcm43xx-bluetooth.h>

static struct rfkill *bt_rfkill;
static struct proc_dir_entry *proc_entry;

static int bcm43xx_bt_rfkill_set_power(void *data, bool blocked)
{
	struct bcm43xx_bt_platform_data *pdata = data;

	// rfkill_ops callback. Turn transmitter on when blocked is false
	if (!blocked) {
		if (pdata->set_power) pdata->set_power(1);
		gpio_set_value(pdata->reset_gpio, 1);
	} else {
		if (pdata->set_power) pdata->set_power(0);
		gpio_set_value(pdata->reset_gpio, 0);
	}

	return 0;
}

static const struct rfkill_ops bcm43xx_bt_rfkill_ops = {
	.set_block = bcm43xx_bt_rfkill_set_power,
};

static int bcm43xx_bt_read_proc(char *page, char **start, off_t off, int count,
	int *eof, void *data)
{
	struct bcm43xx_bt_platform_data* pdata = data;
	unsigned char mac[6];

	*eof = 1;
	pdata->get_addr(mac, 1);

	return sprintf(page, "%02x:%02x:%02x:%02x:%02x:%02x",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static int bcm43xx_bluetooth_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct bcm43xx_bt_platform_data *pdata = pdev->dev.platform_data;

	rc = gpio_request_one(pdata->reset_gpio, GPIOF_OUT_INIT_LOW,
		"bcm43xx_nreset_gpio");
	if (unlikely(rc)) return rc;

	rc = gpio_request_one(pdata->wake_gpio, GPIOF_OUT_INIT_HIGH,
		"bcm43xx_wake_gpio");
	if (unlikely(rc)) return rc;

	bt_rfkill = rfkill_alloc("bcm43xx Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bcm43xx_bt_rfkill_ops,
				pdata);

	if (unlikely(!bt_rfkill)) {
		gpio_free(pdata->reset_gpio);
		gpio_free(pdata->wake_gpio);
		return -ENOMEM;
	}

	rc = rfkill_register(bt_rfkill);

	if (unlikely(rc)) {
		rfkill_destroy(bt_rfkill);
		gpio_free(pdata->reset_gpio);
		gpio_free(pdata->wake_gpio);
		return -1;
	}

	rfkill_set_states(bt_rfkill, true, false);
	bcm43xx_bt_rfkill_set_power(pdata, true);

	if (pdata->get_addr) {
		proc_entry = create_proc_entry("bt_addr", 0666, NULL);
		proc_entry->data = pdata;
		if (proc_entry == NULL) {
			printk("%s : Couldn't create proc entry!\n", __func__);
		} else {
			proc_entry->read_proc = bcm43xx_bt_read_proc;
		}
	}

	return rc;
}

static int bcm43xx_bluetooth_remove(struct platform_device *pdev)
{
	struct bcm43xx_bt_platform_data *pdata = pdev->dev.platform_data;

	if (bt_rfkill) {
		rfkill_unregister(bt_rfkill);
		rfkill_destroy(bt_rfkill);
	}

	gpio_free(pdata->reset_gpio);
	gpio_free(pdata->wake_gpio);

	if (proc_entry) remove_proc_entry("bt_addr", NULL);

	return 0;
}

int bcm43xx_bluetooth_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct bcm43xx_bt_platform_data *pdata = pdev->dev.platform_data;
	gpio_set_value(pdata->wake_gpio, 0);
	return 0;
}

int bcm43xx_bluetooth_resume(struct platform_device *pdev)
{
	struct bcm43xx_bt_platform_data *pdata = pdev->dev.platform_data;
	gpio_set_value(pdata->wake_gpio, 1);
	return 0;
}

static struct platform_driver bcm43xx_bluetooth_platform_driver = {
	.probe = bcm43xx_bluetooth_probe,
	.remove = bcm43xx_bluetooth_remove,
	.suspend = bcm43xx_bluetooth_suspend,
	.resume = bcm43xx_bluetooth_resume,
	.driver = {
		   .name = "bcm43xx_bluetooth",
		   .owner = THIS_MODULE,
		   },
};

static int __init bcm43xx_bluetooth_init(void)
{
	return platform_driver_register(&bcm43xx_bluetooth_platform_driver);
}

static void __exit bcm43xx_bluetooth_exit(void)
{
	platform_driver_unregister(&bcm43xx_bluetooth_platform_driver);
}

module_init(bcm43xx_bluetooth_init);
module_exit(bcm43xx_bluetooth_exit);

MODULE_ALIAS("platform:bcm43xx");
MODULE_DESCRIPTION("bcm43xx_bluetooth");
MODULE_LICENSE("GPL");
