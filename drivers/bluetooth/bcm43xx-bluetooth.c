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
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/rfkill.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/bcm43xx-bluetooth.h>
#include <linux/regulator/driver.h>

static struct rfkill *bt_rfkill;
static struct regulator *clk32kg_reg;
static struct proc_dir_entry *proc_entry;
static bool bt_enabled;
static bool host_wake_uart_enabled;
static bool wake_uart_enabled;

struct bcm_bt_lpm {
	int wake;
	int host_wake;

	struct hrtimer enter_lpm_timer;
	ktime_t enter_lpm_delay;

	struct uart_port *uport;

	struct wake_lock wake_lock;
	char wake_lock_name[100];

	struct bcm43xx_bt_platform_data *pdata;
} bt_lpm;

static int bcm43xx_bt_rfkill_set_power(void *data, bool blocked)
{
	struct bcm43xx_bt_platform_data *pdata = data;

	// rfkill_ops callback. Turn transmitter on when blocked is false
	if (!blocked) {
		if (clk32kg_reg && !bt_enabled)
			regulator_enable(clk32kg_reg);
		gpio_set_value(pdata->reset_gpio, 1);
	} else {
		gpio_set_value(pdata->reset_gpio, 0);
		if (clk32kg_reg && bt_enabled)
			regulator_disable(clk32kg_reg);
	}

	bt_enabled = !blocked;

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

static void set_wake_locked(int wake)
{
	bt_lpm.wake = wake;

	if (!wake)
		wake_unlock(&bt_lpm.wake_lock);

	if (!wake_uart_enabled && wake && bt_lpm.pdata->set_uart)
		bt_lpm.pdata->set_uart(1);

	gpio_set_value(bt_lpm.pdata->wake_gpio, wake);

	if (wake_uart_enabled && !wake && bt_lpm.pdata->set_uart)
		bt_lpm.pdata->set_uart(0);

	wake_uart_enabled = wake;
}

static enum hrtimer_restart enter_lpm(struct hrtimer *timer) {
	unsigned long flags;
	spin_lock_irqsave(&bt_lpm.uport->lock, flags);
	set_wake_locked(0);
	spin_unlock_irqrestore(&bt_lpm.uport->lock, flags);

	return HRTIMER_NORESTART;
}

void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport) {
	bt_lpm.uport = uport;

	hrtimer_try_to_cancel(&bt_lpm.enter_lpm_timer);

	set_wake_locked(1);

	hrtimer_start(&bt_lpm.enter_lpm_timer, bt_lpm.enter_lpm_delay,
		HRTIMER_MODE_REL);
}
EXPORT_SYMBOL(bcm_bt_lpm_exit_lpm_locked);

static void update_host_wake_locked(int host_wake)
{
	if (host_wake == bt_lpm.host_wake)
		return;

	bt_lpm.host_wake = host_wake;

	if (host_wake) {
		wake_lock(&bt_lpm.wake_lock);
		if (!host_wake_uart_enabled && bt_lpm.pdata->set_uart)
			bt_lpm.pdata->set_uart(1);
	} else  {
		if (host_wake_uart_enabled && bt_lpm.pdata->set_uart)
			bt_lpm.pdata->set_uart(0);
		// Take a timed wakelock, so that upper layers can take it.
		// The chipset deasserts the hostwake lock, when there is no
		// more data to send.
		wake_lock_timeout(&bt_lpm.wake_lock, HZ/2);
	}

	host_wake_uart_enabled = host_wake;

}
static irqreturn_t host_wake_isr(int irq, void *dev)
{
	int host_wake;
	unsigned long flags;
	struct bcm43xx_bt_platform_data *pdata = (struct bcm43xx_bt_platform_data *)dev;

	host_wake = gpio_get_value(pdata->host_wake_gpio);
	irq_set_irq_type(irq, host_wake ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);

	if (!bt_lpm.uport) {
		bt_lpm.host_wake = host_wake;
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&bt_lpm.uport->lock, flags);
	update_host_wake_locked(host_wake);
	spin_unlock_irqrestore(&bt_lpm.uport->lock, flags);

	return IRQ_HANDLED;
}

static int bcm_bt_lpm_init(struct platform_device *pdev)
{
	int irq;
	int ret;
	int rc;
	struct bcm43xx_bt_platform_data *pdata = pdev->dev.platform_data;

	rc = gpio_request(pdata->wake_gpio, "bcm4330_wake_gpio");
	if (unlikely(rc)) {
		return rc;
	}

	rc = gpio_request(pdata->host_wake_gpio, "bcm4330_host_wake_gpio");
	if (unlikely(rc)) {
		gpio_free(pdata->wake_gpio);
		return rc;
	}

	hrtimer_init(&bt_lpm.enter_lpm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bt_lpm.enter_lpm_delay = ktime_set(1, 0);  /* 1 sec */
	bt_lpm.enter_lpm_timer.function = enter_lpm;

	bt_lpm.host_wake = 0;
	bt_lpm.pdata = pdata;

	irq = gpio_to_irq(pdata->host_wake_gpio);
	ret = request_irq(irq, host_wake_isr, IRQF_TRIGGER_HIGH,
		"bt host_wake", pdata);
	if (ret) {
		gpio_free(pdata->wake_gpio);
		gpio_free(pdata->host_wake_gpio);
		return ret;
	}

	ret = irq_set_irq_wake(irq, 1);
	if (ret) {
		gpio_free(pdata->wake_gpio);
		gpio_free(pdata->host_wake_gpio);
		return ret;
	}

	gpio_direction_output(pdata->wake_gpio, 0);
	gpio_direction_input(pdata->host_wake_gpio);

	snprintf(bt_lpm.wake_lock_name, sizeof(bt_lpm.wake_lock_name),
			"BTLowPower");
	wake_lock_init(&bt_lpm.wake_lock, WAKE_LOCK_SUSPEND,
			 bt_lpm.wake_lock_name);

	return 0;
}

static int bcm43xx_bluetooth_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct bcm43xx_bt_platform_data *pdata = pdev->dev.platform_data;

	rc = gpio_request_one(pdata->reset_gpio, GPIOF_OUT_INIT_HIGH,
		"bcm43xx_nreset_gpio");
	if (unlikely(rc)) return rc;

	clk32kg_reg = regulator_get(0, "clk32kgate");
	if (IS_ERR(clk32kg_reg)) {
		pr_err("clk32kg reg not found!\n");
		clk32kg_reg = NULL;
	}

	bt_rfkill = rfkill_alloc("bcm43xx Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bcm43xx_bt_rfkill_ops,
				pdata);

	if (unlikely(!bt_rfkill)) {
		gpio_free(pdata->reset_gpio);
		return -ENOMEM;
	}

	rc = rfkill_register(bt_rfkill);

	if (unlikely(rc)) {
		rfkill_destroy(bt_rfkill);
		gpio_free(pdata->reset_gpio);
		return -1;
	}

	rfkill_set_states(bt_rfkill, true, false);
	bcm43xx_bt_rfkill_set_power(pdata, true);

	pdata->wake_peer = bcm_bt_lpm_exit_lpm_locked;
	rc = bcm_bt_lpm_init(pdev);
	if (rc) {
		rfkill_unregister(bt_rfkill);
		rfkill_destroy(bt_rfkill);

		gpio_free(pdata->reset_gpio);
	}

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
	gpio_free(pdata->host_wake_gpio);

	if (proc_entry) remove_proc_entry("bt_addr", NULL);

	regulator_put(clk32kg_reg);

	wake_lock_destroy(&bt_lpm.wake_lock);
	return 0;
}

int bcm43xx_bluetooth_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct bcm43xx_bt_platform_data *pdata = pdev->dev.platform_data;
	int irq = gpio_to_irq(pdata->host_wake_gpio);
	//int host_wake;

	disable_irq(irq);
	/*
	host_wake = gpio_get_value(pdata->host_wake_gpio);

	if (host_wake) {
		enable_irq(irq);
		return -EBUSY;
	}
	*/
	return 0;
}

int bcm43xx_bluetooth_resume(struct platform_device *pdev)
{
	struct bcm43xx_bt_platform_data *pdata = pdev->dev.platform_data;
	int irq = gpio_to_irq(pdata->host_wake_gpio);

	enable_irq(irq);
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
	bt_enabled = false;
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
