#ifndef BCM43XX_BLUETOOTH_H
#define BCM43XX_BLUETOOTH_H

#include <linux/serial_core.h>

struct bcm43xx_bt_platform_data {
	int reset_gpio;
	int wake_gpio;
	int host_wake_gpio;
	void (*set_uart)(int enable);
	int (*get_addr)(unsigned char* mac, int subtype);
	void (*wake_peer)(struct uart_port *uport);
};

extern void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport);

#endif //BCM43XX_BLUETOOTH_H
