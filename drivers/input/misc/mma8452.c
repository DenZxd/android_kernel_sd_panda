/*
 * mma8452.c
 * MMA8452Q Accelerometer driver
 *
 * Copyright (C) 2012
 * Author: juntao Yao <yaojuntao@hhcn.com>
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
 *
 * Derived work from mma8452.c from Xiaolu.Bao <bxl@hhcn.com>
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/i2c/mma8452.h>
#include <linux/gpio.h>

#define MMA8452_DEBUG 1
#define MMA8452_RETRY_COUNT 5

struct mma8452_accel_data {
	int mode;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct delayed_work wq;
	uint32_t def_poll_rate;
};

static uint32_t accl_debug;
module_param_named(mma8452_debug, accl_debug, uint, 0664);

/* interval between samples for the different rates, in msecs */
static const unsigned int mma8452_measure_interval[] = {
	1000 / 2,  1000 / 7,  1000 / 13,  1000 / 50,
	1000 / 100, 1000 / 200, 1000 / 400, 1000 / 800
};

static int mma8452_read(struct mma8452_accel_data *data, char data_addr, char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msgs[] = {
		{
			.addr   = data->client->addr,
			.flags  = 0,
			.len    = 1,
			.buf    = buf,
		},
		{
			.addr   = data->client->addr,
			.flags  = I2C_M_RD,
			.len    = len,
			.buf    = buf,
		}
	};

	buf[0] = data_addr;

	for (i = 0; i < MMA8452_RETRY_COUNT; i++) {
		if (i2c_transfer(data->client->adapter, msgs, 2) > 0) {
			break;
		}
		msleep(10);
	}

	if (i >= MMA8452_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__,
			   MMA8452_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int mma8452_write(struct mma8452_accel_data *data, char reg, char val)
{
	uint8_t i;
	char data_buffer[2];
	struct i2c_msg msg[] = {
		{
			.addr   = data->client->addr,
			.flags  = 0,
			.len    = 2,
			.buf    = data_buffer,
		}
	};

	data_buffer[0] = reg;
	data_buffer[1] = val;

	for (i = 0; i < MMA8452_RETRY_COUNT; i++) {
		if (i2c_transfer(data->client->adapter, msg, 1) > 0) {
			break;
		}
		msleep(10);
	}

	if (i >= MMA8452_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__,
									MMA8452_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int mma8452_accel_device_hw_set_bandwidth(struct mma8452_accel_data *data,
																	int bandwidth)
{
	char reg_val;

	mma8452_read(data,MMA8452_REG_CTRL,&reg_val,1);
	reg_val = (reg_val & 0xC7) | ((bandwidth << 3) & 0x38);
	mma8452_write(data,MMA8452_REG_CTRL,reg_val);

	return 0;
}

static void mma8452_accel_device_sleep(struct mma8452_accel_data *data)
{
	char reg_val;

	mma8452_read(data,MMA8452_REG_CTRL,&reg_val,1);
	reg_val &= ~MMA8452_CTRL_ACTIVE;
	mma8452_write(data,MMA8452_REG_CTRL,reg_val);
}

static void mma8452_accel_device_wakeup(struct mma8452_accel_data *data)
{
	char reg_val;

	mma8452_read(data,MMA8452_REG_CTRL,&reg_val,1);
	reg_val |= MMA8452_CTRL_ACTIVE;
	mma8452_write(data,MMA8452_REG_CTRL,reg_val);
}
static short mma8452_accel_read_data(struct mma8452_accel_data *data,
									 short *x, short *y, short *z)
{
	int ret;
	char data_buffer[6];

	ret = mma8452_read(data, MMA8452_REG_DATA, data_buffer, 6);
	if (ret != 0) {
		dev_err(&data->client->dev,
			"mma8452_read_data_ready failed\n");
		return -1;
	}

	*x = (data_buffer[0] << 8) | data_buffer[1];
	*y = (data_buffer[2] << 8) | data_buffer[3];
	*z = (data_buffer[4] << 8) | data_buffer[5];

	*x >>= 4;
	*y >>= 4;
	*z >>= 4;

	return 0;
}

static void mma8452_accel_data_ready(struct mma8452_accel_data *data)
{
	int ret, count = -1;
	char data_status = 0;
	short x = 0;
	short y = 0;
	short z = 0;

	do {
		mma8452_read(data, MMA8452_STATUS, &data_status, 1);
		count ++;
	} while ((!(data_status & MMA8452_STATUS_ZYXDR)) && (count <= MMA8452_RETRY_COUNT));

	ret = mma8452_accel_read_data(data, &x, &y, &z);
	if (!ret) {
		if (accl_debug)
			pr_info("%s: X: 0x%X Y: 0x%X Z: 0x%X\n",
					__func__, x, y, z);

		input_report_abs(data->input_dev, ABS_X, x);
		input_report_abs(data->input_dev, ABS_Y, y);
		input_report_abs(data->input_dev, ABS_Z, z);
		input_sync(data->input_dev);
	}

}

static void mma8452_accel_device_worklogic(struct work_struct *work)
{
	struct mma8452_accel_data *data = container_of((struct delayed_work *)work,
				struct mma8452_accel_data, wq);

	if (data->mode) {
		mma8452_accel_data_ready(data);
		schedule_delayed_work(&data->wq,
			msecs_to_jiffies(data->def_poll_rate));
	}

}

static ssize_t mma8452_show_attr_enable(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mma8452_accel_data *data = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", data->mode);
}

static ssize_t mma8452_store_attr_enable(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mma8452_accel_data *data = platform_get_drvdata(pdev);
	unsigned long val;
	int error, enable;

	error = strict_strtoul(buf, 0, &val);
	if (error)
		return error;
	enable = !!val;

	if (data->mode == enable)
		return count;

	if (enable) {
		mma8452_accel_device_wakeup(data);
		schedule_delayed_work(&data->wq, 0);
	} else {
		mma8452_accel_device_sleep(data);
		cancel_delayed_work_sync(&data->wq);
	}

	data->mode = enable;

	return count;
}

static ssize_t mma8452_show_attr_delay(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mma8452_accel_data *data = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", data->def_poll_rate);
}

static ssize_t mma8452_store_attr_delay(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mma8452_accel_data *data = platform_get_drvdata(pdev);
	unsigned long interval;
	int error;
	int i = 0;

	error = strict_strtoul(buf, 0, &interval);
	if (error)
		return error;

	if (interval < 0)
		return -EINVAL;

	cancel_delayed_work_sync(&data->wq);

	if (interval >=
		mma8452_measure_interval[MMA_BW_2HZ])
		i = MMA_BW_2HZ;
	else if (interval >=
		mma8452_measure_interval[MMA_BW_7HZ])
		i = MMA_BW_7HZ;
	else if (interval >=
		mma8452_measure_interval[MMA_BW_13HZ])
		i = MMA_BW_13HZ;
	else if (interval >=
		mma8452_measure_interval[MMA_BW_50HZ])
		i = MMA_BW_50HZ;
	else if (interval >=
		mma8452_measure_interval[MMA_BW_100HZ])
		i = MMA_BW_100HZ;
	else if (interval >=
		mma8452_measure_interval[MMA_BW_200HZ])
		i = MMA_BW_200HZ;
	else if (interval >=
		mma8452_measure_interval[MMA_BW_400HZ])
		i = MMA_BW_400HZ;
	else
		i = MMA_BW_800HZ;

	data->def_poll_rate = interval;
	mma8452_accel_device_hw_set_bandwidth(data, MMA_BW_800HZ - i);

	schedule_delayed_work(&data->wq, 0);

	return count;

}
#ifdef MMA8452_DEBUG
static ssize_t mma8452_registers_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mma8452_accel_data *data = platform_get_drvdata(pdev);
	char data_buffer[6];
	unsigned n,i;

	mma8452_read(data, MMA8452_REG_DATA, data_buffer, 6);
	for(i = 0,n = 0; i < 6; i++) {
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%d = %d\n",
			       i, data_buffer[i]);
	}

	return n;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		mma8452_registers_show, NULL);
#endif
static DEVICE_ATTR(enable, 0666,
		mma8452_show_attr_enable, mma8452_store_attr_enable);

static DEVICE_ATTR(delay, 0666,
		mma8452_show_attr_delay, mma8452_store_attr_delay);

static struct attribute *mma8452_accel_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
#ifdef MMA8452_DEBUG
	&dev_attr_registers.attr,
#endif
	NULL
};

static const struct attribute_group mma8452_accel_attr_group = {
	.attrs = mma8452_accel_attrs,
};

static int mma8452_accel_device_hw_init(struct mma8452_accel_data *data)
{
	int ret = 0;

	ret = mma8452_write(data,MMA8452_XYZ_DATA_CFG,
					MMA8452_CTRL_MODE_2G);
	if (ret < 0)	return -EFAULT;

	ret = mma8452_write(data,MMA8452_REG_CTRL,!MMA8452_CTRL_ACTIVE | MMA8452_CTRL_LNOISE);
	if (ret < 0)	return -EFAULT;

	return ret;
}


static int __devinit mma8452_accel_driver_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct mma8452_accel_data *data;
	unsigned char tempvalue;
	int ret = 0;

	pr_info("%s: Enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

       /* read chip id */
#define MMA8452_CHIP_ID_REG     0x0D
#define MMA8452_CHIP_ID         0x2A
	tempvalue = i2c_smbus_read_byte_data(client, MMA8452_CHIP_ID_REG);
	if (tempvalue == MMA8452_CHIP_ID) {
	    printk(KERN_INFO "MMA8452 Device detected!\n");
	} else {
	    printk(KERN_INFO "MMA8452 Device not found\n");
	    return -ENODEV;
	}

	/* alloc memory for data structure */
	data = kzalloc(sizeof(struct mma8452_accel_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto error;
	}
	data->mode = 0;
	data->client = client;
	data->def_poll_rate = 0;
	i2c_set_clientdata(client, data);

	data->input_dev = input_allocate_device();
	if (data->input_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&data->client->dev,
			"Failed to allocate input device\n");
		goto error;
	}

	INIT_DELAYED_WORK(&data->wq, mma8452_accel_device_worklogic);

	data->input_dev->name = "mma8452";
	data->input_dev->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, data->input_dev->evbit);
	input_set_abs_params(data->input_dev, ABS_X,
				-G_RANGE, G_RANGE, 0, 0);
	input_set_abs_params(data->input_dev, ABS_Y,
				-G_RANGE, G_RANGE, 0, 0);
	input_set_abs_params(data->input_dev, ABS_Z,
				-G_RANGE, G_RANGE, 0, 0);

	data->input_dev->dev.parent = &data->client->dev;
	input_set_drvdata(data->input_dev, data);

	ret = input_register_device(data->input_dev);
	if (ret) {
		dev_err(&data->client->dev,
			"Unable to register input device\n");
	}


	ret = mma8452_accel_device_hw_init(data);
	if (ret)
		goto error_1;

	ret = sysfs_create_group(&client->dev.kobj, &mma8452_accel_attr_group);
	if (ret)
		goto error_1;

	return 0;

error_1:
	input_free_device(data->input_dev);
	kfree(data);
error:
	return ret;
}

static int __devexit mma8452_accel_driver_remove(struct i2c_client *client)
{
	struct mma8452_accel_data *data = i2c_get_clientdata(client);
	int ret = 0;

	sysfs_remove_group(&client->dev.kobj, &mma8452_accel_attr_group);

	if (data->client->irq)
		free_irq(data->client->irq, data);

	cancel_delayed_work_sync(&data->wq);

	if (data->input_dev)
		input_free_device(data->input_dev);

	i2c_set_clientdata(client, NULL);
	kfree(data);

	return ret;
}


#ifdef CONFIG_PM
static int mma8452_accel_driver_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mma8452_accel_data *data = platform_get_drvdata(pdev);

	if (data->mode)
		mma8452_accel_device_sleep(data);

	return 0;
}

static int mma8452_accel_driver_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mma8452_accel_data *data = platform_get_drvdata(pdev);

	if (data->mode)
		mma8452_accel_device_wakeup(data);

	return 0;
}

static const struct dev_pm_ops mma8452_pm_ops = {
	.suspend = mma8452_accel_driver_suspend,
	.resume = mma8452_accel_driver_resume,
};
#endif

static const struct i2c_device_id mma8452_accel_idtable[] = {
	{ DEVICE_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, mma8452_accel_idtable);

static struct i2c_driver mma8452_accel_driver = {
	.probe		= mma8452_accel_driver_probe,
	.remove		= mma8452_accel_driver_remove,
	.id_table	= mma8452_accel_idtable,
	.driver = {
		.name = DEVICE_NAME,
#ifdef CONFIG_PM
		.pm = &mma8452_pm_ops,
#endif
	},
};

static int __init mma8452_accel_driver_init(void)
{
	return i2c_add_driver(&mma8452_accel_driver);
}

static void __exit mma8452_accel_driver_exit(void)
{
	i2c_del_driver(&mma8452_accel_driver);
}

module_init(mma8452_accel_driver_init);
module_exit(mma8452_accel_driver_exit);

MODULE_DESCRIPTION("MMA8452 Accelerometer Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juntao YAO <yaojuntao@hhcn.com>");
