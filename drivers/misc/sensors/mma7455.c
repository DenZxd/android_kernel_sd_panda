/****************************************************************
 * $ID: smartq_mma7455.c Fri, 11 Nov 2011 17:48:25 +0800  root $ *
 *                                                              *
 * Description:                                                 *
 *                                                              *
 * Maintainer:  (JunTao Yao)  <yaojuntao@hhcn.com>            *
 *                                                              *
 * Copyright (C)  2009  HHTech                                  *
 *   www.hhcn.com, www.hhcn.org                                 *
 *   All rights reserved.                                       *
 *                                                              *
 * This file is free software;                                  *
 *   you are free to modify and/or redistribute it   	        *
 *   under the terms of the GNU General Public Licence (GPL).   *
 ****************************************************************/

#include <linux/module.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/apm_bios.h>
#include <linux/capability.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/freezer.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <asm/system.h>
#include <linux/i2c.h>
#include <linux/ctype.h>
#include <asm/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <mach/gpio.h>
#include <asm/io.h>
#include <linux/input.h>

#define MMA7455_IOC_MAGIC 'B'
#define MMA7455_READ_ACCEL_XYZ       _IOWR(MMA7455_IOC_MAGIC,26,signed char)
#define MMA7455_GET_OFFSET_XYZ       _IOWR(MMA7455_IOC_MAGIC,91,signed char)
#define MMA7455_CAL_XYZ				 _IOWR(MMA7455_IOC_MAGIC,92,signed short)
#define MMA7455_SET_CAL_XYZ			 _IOWR(MMA7455_IOC_MAGIC,93,signed short)

#define MMA7455_RETRY_COUNT    10
#define MAX_CAL_COUNT   2000
#define MMA7455_DEV_MINOR 249

static int sensor_used_count = 0;
static struct i2c_client *this_client;

int mma7455_read(unsigned char reg, unsigned char *buf)
{
	int i,ret;
	struct i2c_msg msg[] = {
		{
			.addr  = this_client->addr,
			.flags = 0,
			.len   = 1,
			.buf   = buf,
		},
		{
			.addr  = this_client->addr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = buf,
		}
	};

	*buf = reg;
	for (i = 0; i < MMA7455_RETRY_COUNT; i++) {
		ret = i2c_transfer(this_client->adapter, msg, 2);
		if (ret > 0)
			break;
	}

	return ret;
}

int mma7455_write(unsigned char reg, unsigned char data)
{
	int i,ret;
	unsigned char buf[2];

	struct i2c_msg msg[] = {
		{
			.addr  = this_client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = buf,
		}
	};

	buf[0] = reg;
	buf[1] = data;

	for (i = 0; i < MMA7455_RETRY_COUNT; i++) {
		ret = i2c_transfer(this_client->adapter, msg, 1);
		if (ret > 0)
			break;
	}

	return ret;
}

static int mma7455_open(struct inode *inode, struct file *filp)
{
	if (sensor_used_count == 0)
		mma7455_write(0x16,0x45); //2g and measurement Mode
	sensor_used_count++;

	return 0;
}

static int mma7455_close(struct inode *inode, struct file *flip)
{
	sensor_used_count--;
	if (sensor_used_count < 0) {
		printk("sensor :close error (over)\n");
		sensor_used_count = 0;
	}

	if (sensor_used_count <= 0) {
		mma7455_write(0x16,0x44); //sensor standby mode
	}

	return 0;
}

static unsigned char fabs(signed char a)
{
	if (a < 0)
		a = -a;

	return a;
}

static int mma7455_calibrate(signed short *cal)
{
	int ret;
	unsigned char drdy;
	int count = 0,valid_count = 0;
	signed char xdata,ydata,zdata;
	u8 xcal_h,xcal_l,ycal_h,ycal_l,zcal_h,zcal_l;
	signed long int xcalsum = 0,ycalsum = 0,zcalsum = 0;
	signed short xcal = 0,xcalp,ycal = 0,ycalp,zcal = 0,zcalp;

	mma7455_write(0x10,0);
	mma7455_write(0x11,0);
	mma7455_write(0x12,0);
	mma7455_write(0x13,0);
	mma7455_write(0x14,0);
	mma7455_write(0x15,0);

	while (1) {
		ret = mma7455_read(9,&drdy);
		if (ret < 0) {	count ++;	continue; }

		if ((drdy & 0x1) == 0) { continue; }

		ret = mma7455_read(0x06,&xdata);
		if (ret < 0) {	count ++;	continue; }
		ret = mma7455_read(0x07,&ydata);
		if (ret < 0) {	count ++;	continue; }
		ret = mma7455_read(0x08,&zdata);
		if (ret < 0) {	count ++;	continue; }

		xcal += -2 * xdata;
		ycal += -2 * ydata;

		if (zdata >= 0 && zdata <= 127)
			zcal += (64 - zdata) * 2;
		else if (zdata < 0 && zdata >= -128)
			zcal += -(64 + zdata) *2;

		if ((fabs(xcal)/2 > 128) || (fabs(xcal)/2 > 128) || (fabs(zcal)/2  > 128)) {
			xcal = 0;ycal = 0;zcal = 0;
			continue;
		}

		xcalp = xcal;
		if (xcalp < 0)	xcalp += 2048;

		ycalp = ycal;
		if (ycalp < 0)	ycalp += 2048;

		zcalp = zcal;
		if (zcalp < 0)	zcalp += 2048;

		xcal_l = (u8)(xcalp & 0x00ff);	xcal_h = (u8)((xcalp & 0xff00) >> 8);
		ycal_l = (u8)(ycalp & 0x00ff);	ycal_h = (u8)((ycalp & 0xff00) >> 8);
		zcal_l = (u8)(zcalp & 0x00ff);	zcal_h = (u8)((zcalp & 0xff00) >> 8);

		mma7455_write(0x10,xcal_l);
		mma7455_write(0x11,xcal_h);
		mma7455_write(0x12,ycal_l);
		mma7455_write(0x13,ycal_h);
		mma7455_write(0x14,zcal_l);
		mma7455_write(0x15,zcal_h);

		count++;

		if (fabs(xdata) < 4 && fabs(ydata) < 4 && fabs(fabs(zdata) - 64) < 4) {
			valid_count ++;
			xcalsum += xcal/2;	ycalsum += ycal/2;	zcalsum += zcal/2;

			if (valid_count >= 100) {
				xcal = xcalsum / valid_count;	ycal = ycalsum / valid_count;	zcal = zcalsum / valid_count;

				xcalp = xcal * 2;
			    if (xcalp < 0)	xcalp += 2048;
			    ycalp = ycal * 2;
			    if (ycalp < 0)	ycalp += 2048;
			    zcalp = zcal * 2;
			    if (zcalp < 0)	zcalp += 2048;

			    xcal_l = (u8)(xcalp & 0x00ff);	xcal_h = (u8)((xcalp & 0xff00) >> 8);
			    ycal_l = (u8)(ycalp & 0x00ff);	ycal_h = (u8)((ycalp & 0xff00) >> 8);
			    zcal_l = (u8)(zcalp & 0x00ff);	zcal_h = (u8)((zcalp & 0xff00) >> 8);

			    mma7455_write(0x10,xcal_l);
			    mma7455_write(0x11,xcal_h);
			    mma7455_write(0x12,ycal_l);
			    mma7455_write(0x13,ycal_h);
			    mma7455_write(0x14,zcal_l);
			    mma7455_write(0x15,zcal_h);

				cal[0] = xcal; cal[1] = ycal; cal[2] = zcal;
				return 0;
			}
		}

		if (count > MAX_CAL_COUNT)
			return -1;

		msleep(10);
	}
}

static void mma7455_fill_cal(signed short *cal)
{
	signed short xcal = 0,ycal = 0,zcal = 0;
	u8 xcal_h,xcal_l,ycal_h,ycal_l,zcal_h,zcal_l;

	xcal = cal[0] * 2;	ycal = cal[1] * 2;	zcal = cal[2] * 2;

	if (xcal < 0) xcal += 2048;
	if (ycal < 0) ycal += 2048;
	if (zcal < 0) zcal += 2048;

	xcal_l = (u8)(xcal & 0x00ff);	xcal_h = (u8)((xcal & 0xff00) >> 8);
	ycal_l = (u8)(ycal & 0x00ff);	ycal_h = (u8)((ycal & 0xff00) >> 8);
	zcal_l = (u8)(zcal & 0x00ff);	zcal_h = (u8)((zcal & 0xff00) >> 8);

	mma7455_write(0x10,xcal_l);
	mma7455_write(0x11,xcal_h);
	mma7455_write(0x12,ycal_l);
	mma7455_write(0x13,ycal_h);
	mma7455_write(0x14,zcal_l);
	mma7455_write(0x15,zcal_h);
	msleep(10);

}

static void mma7455_read_data(signed char *data)
{
	int i;
	char xyz_data[3];

	mma7455_read(0x6,&xyz_data[0]);
	mma7455_read(0x7,&xyz_data[1]);
	mma7455_read(0x8,&xyz_data[2]);

	for (i = 0; i < 3;i++)
		if (xyz_data[i] > 128) xyz_data[i] -= 256;

	data[0] = xyz_data[0];
	data[1] = xyz_data[1];
	data[2] = xyz_data[2];
}

static long mma7455_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;
	signed char data[3];
	signed short cal[3];
	void __user *pa = (void __user *)arg;

	switch (cmd) {
	case MMA7455_CAL_XYZ:
		ret = mma7455_calibrate(cal);
		if (ret < 0)
			return ret;
		if (copy_to_user(pa, cal, sizeof(cal)))
			return -EFAULT;
		break;
	case MMA7455_SET_CAL_XYZ:
		if(copy_from_user(cal, pa, sizeof(cal)))
			return -EFAULT;
		mma7455_fill_cal(cal);
		break;
	case MMA7455_READ_ACCEL_XYZ:
		mma7455_read_data(data);
		if (copy_to_user(pa, data, sizeof(data)))
			return -EFAULT;
		break;
	default:
		printk("sensor: unrecognized ioctl (0x%x)\n", cmd);
		return -EINVAL;
		break;
	}

	return 0;
}

static struct file_operations mma7455_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = mma7455_ioctl,
	.open = mma7455_open,
	.release = mma7455_close,
};

static struct miscdevice mma7455_device = {
	.minor = MMA7455_DEV_MINOR,
	.name  = "mma7455",
	.fops  = &mma7455_fops
};

void mma7455_init_client(void)
{
	mma7455_write(0x16,0x44); //2g and standby mode
}

static int mma7455_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc;
	if (!i2c_check_functionality(client->adapter,
								 I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "i2c bus does not support the mma7455\n");
		rc = -ENODEV;
		goto exit;
	}

	this_client = client;
	mma7455_init_client();

exit:
	return rc;
}

static int mma7455_suspend(struct i2c_client *client, pm_message_t mesg)
{
	mma7455_write(0x16,0x44);
	return 0;
}

static int mma7455_resume(struct i2c_client *client)
{
	mma7455_write(0x16,0x45);
	return 0;
}

static int mma7455_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id mma7455_id[] = {
	{"mma7455", 0 },
	{ }
};

static struct i2c_driver mma7455_driver = {
	.driver = {
		.name = "mma7455",
	},
	.probe  = mma7455_probe,
	.remove = mma7455_remove,
	.suspend = mma7455_suspend,
	.resume = mma7455_resume,
	.id_table = mma7455_id,
};

static int __init mma7455_init(void)
{
	misc_register(&mma7455_device);
	return i2c_add_driver(&mma7455_driver);
}

static void __exit mma7455_exit(void)
{
	misc_deregister(&mma7455_device);
	i2c_del_driver(&mma7455_driver);
}

MODULE_AUTHOR("JunTao Yao");
MODULE_DESCRIPTION("MMA7455 G-sensor driver");
MODULE_LICENSE("GPL");

module_init(mma7455_init);
module_exit(mma7455_exit);
