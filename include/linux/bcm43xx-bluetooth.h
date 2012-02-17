#ifndef BCM43XX_BLUETOOTH_H
#define BCM43XX_BLUETOOTH_H

struct bcm43xx_bt_platform_data {
	int reset_gpio;
	int wake_gpio;
	void (*set_power)(int enable);
	int (*get_addr)(unsigned char* mac, int subtype);
};

#endif //BCM43XX_BLUETOOTH_H
