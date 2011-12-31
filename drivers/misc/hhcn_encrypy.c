
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/capability.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <mach/gpio.h>

typedef enum {
    SCL_HIGH,
    SCL_LOW,
    SDA_HIGH,
    SDA_LOW,
    SDA_DATA
} I2C_CMD;

#define GPIO_SCL   44
#define GPIO_SDA   45

/*
 * The smartq_encrypt device is one of the misc char devices.
 * This is its minor number.
 */
#define SMARTQ_ENCRYPT_DEV_MINOR	234

static int smartq_encrypt_open(struct inode * inode, struct file * filp)
{
    return 0;
}

static int smartq_encrypt_release(struct inode *inode, struct file *file)
{
    return 0;
}

static int smartq_encrypt_ioctl(struct file *file, unsigned int cmd,
                     unsigned long arg)
{
    int ret = 0;
    switch (cmd) {
	case SCL_HIGH:
	    gpio_direction_output(GPIO_SCL, 1);
	    break;
	case SCL_LOW:
	    gpio_direction_output(GPIO_SCL, 0);
	    break;
	case SDA_HIGH:
	    gpio_direction_input(GPIO_SDA); // important!
	    break;
	case SDA_LOW:
	    gpio_direction_output(GPIO_SDA, 0);
	    break;
	case SDA_DATA:
	    gpio_direction_input(GPIO_SDA);
	    ret = gpio_get_value(GPIO_SDA);
	    break;
    }
    return ret;
}

static const struct file_operations smartq_encrypt_fops = {
    .unlocked_ioctl =   smartq_encrypt_ioctl,
    .open	    =   smartq_encrypt_open,
    .release	    =   smartq_encrypt_release,
};

static struct miscdevice smartq_encrypt_device = {
    .minor	    =	SMARTQ_ENCRYPT_DEV_MINOR,
    .name	    =	"smartq_encrypt",
    .fops	    =	&smartq_encrypt_fops
};

static int __init smartq_encrypt_init(void)
{
    int status;
    status = gpio_request(GPIO_SCL, "GPIO_SCL");
    if (unlikely(status)) {
        printk("gpio %ld request failed\n",GPIO_SCL);
        return status;
    }
    status = gpio_request(GPIO_SDA, "GPIO_SDA");
    if (unlikely(status)) {
        printk("gpio %ld request failed\n",GPIO_SDA);
        return status;
    }
    return misc_register(&smartq_encrypt_device);
}

static void __exit smartq_encrypt_exit(void)
{
    misc_deregister(&smartq_encrypt_device);
}

module_init(smartq_encrypt_init);
module_exit(smartq_encrypt_exit);

MODULE_AUTHOR("hanpin.wu");
MODULE_DESCRIPTION("Encypt module for SmartQ");
MODULE_LICENSE("GPL");

