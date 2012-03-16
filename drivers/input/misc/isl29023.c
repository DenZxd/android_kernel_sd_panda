/**
 * ISL29023.c - ISL light sensor driver
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Dan Murphy <DMurphy@ti.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input-polldev.h>
#include <linux/input.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#define ISL29023_NAME "isl29023"
#define DEF_POLL_INTERVAL   2000
#define LUX_LEVEL       7

#define REGS_COMMAND_I          0x0
#define REGS_COMMAND_II         0x1
#define REGS_LBS_SENSOR         0x2
#define REGS_MBS_SENSOR         0x3

struct isl29023_data {
	int mode;
	unsigned long interval;
	struct i2c_client *client;
	struct input_dev *als_input_dev;
	struct delayed_work wq;

};

static u8 light_read(struct i2c_client *client, unsigned int reg)
{
        u8 data;
        data = reg & 0x00ff;

        if (i2c_master_send(client, &data, 1) == 1) {
                if (i2c_master_recv(client, &data, 1) == 1) {
                        return data;
                } else {
                        printk(KERN_EMERG"light_read: i2c_master_recv failed\n");
                        return -EIO;
                }
        } else {
                printk(KERN_EMERG"light_read: i2c_master_send failed\n");
                return -EIO;
        }
}

static int light_write(struct i2c_client *client, unsigned int reg, u8 value)
{
        u8 data[2];
        data[0] = reg & 0x00ff;
        data[1] = value;
        if (i2c_master_send(client, data, 2) == 2)
                return 0;
        else {
                printk(KERN_EMERG"light_write: i2c_master_send failed\n");
                return -EIO;
        }
}

static void isl_light_data_ready(struct isl29023_data *data)
{
        unsigned int dat;
        unsigned int MSB,LSB;

        LSB = light_read(data->client, REGS_LBS_SENSOR);
        MSB = light_read(data->client, REGS_MBS_SENSOR);


        dat = (MSB << 8) | LSB;

		input_event(data->als_input_dev, EV_LED, LED_MISC, dat);
		input_sync(data->als_input_dev);

}

static int isl29023_init_device(struct isl29023_data *data)
{
	struct i2c_client *client = data->client;

	light_write(client, REGS_COMMAND_I, 0x00);
	light_write(client, REGS_COMMAND_II, 0x00);

	return 0;
}

static void isl_light_device_worklogic(struct work_struct *work)
{
	struct isl29023_data *data = container_of((struct delayed_work *)work,
												struct isl29023_data, wq);
	if (data->mode) {
		isl_light_data_ready(data);
		schedule_delayed_work(&data->wq,
								msecs_to_jiffies(DEF_POLL_INTERVAL));
	}
}

static void isl29023_als_enable(struct isl29023_data *data, int val)
{
	if (val) {
		light_write(data->client, REGS_COMMAND_I, 0xa3);
		light_write(data->client, REGS_COMMAND_II, 0x02);
		schedule_delayed_work(&data->wq, msecs_to_jiffies(500));
	} else {
		light_write(data->client, REGS_COMMAND_I, 0x0);
		light_write(data->client, REGS_COMMAND_II, 0x0);
		cancel_delayed_work_sync(&data->wq);
	}
}

static ssize_t isl29023_show_attr_enable(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct isl29023_data *data = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", data->mode);

}

static ssize_t isl29023_store_attr_als_enable(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct isl29023_data *data = platform_get_drvdata(pdev);
	unsigned long val;
	int error, enable;

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;
	enable = !!val;

	if (data->mode == enable)
		return count;

	if (enable) {
		isl29023_als_enable(data, 1);
	} else {
		isl29023_als_enable(data, 0);
	}

	data->mode = enable;

	return count;
}

static ssize_t isl29023_show_attr_delay(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct isl29023_data *data = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", data->interval);
}

static ssize_t isl29023_store_attr_delay(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{

	struct platform_device *pdev = to_platform_device(dev);
	struct isl29023_data *data = platform_get_drvdata(pdev);

	unsigned long interval;
	int error = 0;

	error = strict_strtoul(buf, 0, &interval);
	if (error)
		return error;

	data->interval = interval;

	return count;
}

static DEVICE_ATTR(als_enable, 0666,
		isl29023_show_attr_enable, isl29023_store_attr_als_enable);

static DEVICE_ATTR(delay, 0666,
		isl29023_show_attr_delay, isl29023_store_attr_delay);

static struct attribute *isl29023_attrs[] = {
	&dev_attr_als_enable.attr,
	&dev_attr_delay.attr,
	NULL
};

static const struct attribute_group isl29023_attr_group = {
	.attrs = isl29023_attrs,
};

static int __devinit isl29023_driver_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct isl29023_data *data;

	pr_info("%s: Enter\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct isl29023_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto error;
	}

	data->mode = 0;
	data->interval = DEF_POLL_INTERVAL;
	data->client = client;
	i2c_set_clientdata(client, data);

	data->als_input_dev = input_allocate_device();
	if (data->als_input_dev == NULL) {
		ret = -ENOMEM;
		pr_err("%s:Failed to allocate als input device\n", __func__);
		goto als_input_error;
	}

	data->als_input_dev->name = "isl29023_als";
	data->als_input_dev->id.bustype = BUS_I2C;
	data->als_input_dev->dev.parent = &data->client->dev;

	input_set_capability(data->als_input_dev, EV_MSC, MSC_RAW);
	input_set_capability(data->als_input_dev, EV_LED, LED_MISC);
	input_set_drvdata(data->als_input_dev, data);

	ret = input_register_device(data->als_input_dev);
	if (ret) {
		pr_err("%s:Unable to register als device\n", __func__);
		goto als_register_fail;
	}

	ret = isl29023_init_device(data);
	if (ret) {
		pr_err("%s:ISL29023 device init failed\n", __func__);
		goto device_init_fail;
	}

	ret = sysfs_create_group(&client->dev.kobj, &isl29023_attr_group);
	if (ret) {
		pr_err("%s:Cannot create sysfs group\n", __func__);
		goto sysfs_create_fail;
	}

	INIT_DELAYED_WORK(&data->wq, isl_light_device_worklogic);

	return 0;

sysfs_create_fail:
device_init_fail:
als_register_fail:
	input_free_device(data->als_input_dev);
als_input_error:
	kfree(data);
error:
	return ret;
}

static int __devexit isl29023_driver_remove(struct i2c_client *client)
{
	struct isl29023_data *data = i2c_get_clientdata(client);
	int ret = 0;

	sysfs_remove_group(&client->dev.kobj, &isl29023_attr_group);
	cancel_delayed_work_sync(&data->wq);

	if (data->als_input_dev)
		input_free_device(data->als_input_dev);

	i2c_set_clientdata(client, NULL);
	kfree(data);

	return ret;
}

#ifdef CONFIG_PM
static int isl29023_driver_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct isl29023_data *data = platform_get_drvdata(pdev);

	if (data->mode)
		isl29023_als_enable(data, 0);

	return 0;
}

static int isl29023_driver_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct isl29023_data *data = platform_get_drvdata(pdev);

	if (data->mode)
		isl29023_als_enable(data, 1);

	return 0;
}

static const struct dev_pm_ops isl29023_pm_ops = {
	.suspend = isl29023_driver_suspend,
	.resume = isl29023_driver_resume,
};
#endif

static const struct i2c_device_id isl29023_idtable[] = {
	{ ISL29023_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, isl29023_idtable);

static struct i2c_driver isl29023_driver = {
	.probe		= isl29023_driver_probe,
	.remove		= isl29023_driver_remove,
	.id_table	= isl29023_idtable,
	.driver = {
		.name = ISL29023_NAME,
#ifdef CONFIG_PM
		.pm = &isl29023_pm_ops,
#endif
	},
};

static int __init isl29023_driver_init(void)
{
	return i2c_add_driver(&isl29023_driver);
}

static void __exit isl29023_driver_exit(void)
{
	i2c_del_driver(&isl29023_driver);
}

module_init(isl29023_driver_init);
module_exit(isl29023_driver_exit);

MODULE_DESCRIPTION("ISL29023 Driver");
MODULE_LICENSE("GPL");
