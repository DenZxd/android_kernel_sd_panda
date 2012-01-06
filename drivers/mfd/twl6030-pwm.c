/*
 * twl6030_pwm.c
 * Driver for PHOENIX (TWL6030) Pulse Width Modulator
 *
 * Copyright (C) 2010 Texas Instruments
 * Author: Hemanth V <hemanthv@ti.com>
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
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/slab.h>

#define LED_PWM_CTRL1	0xF4
#define LED_PWM_CTRL2	0xF5

/* TOGGLE3 register control PWMx */
#define REG_TOGGLE3	0x92

#define PWM2EN		(0x01 << 5)
#define PWM2S		(0x01 << 4)
#define PWM2R		(0x01 << 3)
#define PWM1EN		(0x01 << 2)
#define PWM1S		(0x01 << 1)
#define PWM1R		(0x01 << 0)

#define REG_PWDNSTATUS2	0x94

#define PWM2_CLK_EN	(0x01 << 3)
#define PWM2_STS	(0x01 << 2)
#define PWM1_CLK_EN	(0x01 << 1)
#define PWM1_STS	(0x01 << 0)

#define REG_PWM1ON	0xBA
#define REG_PWM1OFF	0xBB
#define REG_PWM2ON	0xBD
#define REG_PWM2OFF	0xBE

#define PWM_LENGTH_64	(0x01 << 7)

/* Max value for CTRL1 register */
#define PWM_CTRL1_MAX	255

/* Pull down disable */
#define PWM_CTRL2_DIS_PD	(1 << 6)

/* Current control 2.5 milli Amps */
#define PWM_CTRL2_CURR_02	(2 << 4)

/* LED supply source */
#define PWM_CTRL2_SRC_VAC	(1 << 2)

/* LED modes */
#define PWM_CTRL2_MODE_HW	(0 << 0)
#define PWM_CTRL2_MODE_SW	(1 << 0)
#define PWM_CTRL2_MODE_DIS	(2 << 0)

#define PWM_CTRL2_MODE_MASK	0x3

struct pwm_device {
	const char *label;
	unsigned int pwm_id;
};

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	u8 duty_cycle;
	int ret;

	if (pwm == NULL || period_ns == 0 || duty_ns > period_ns)
		return -EINVAL;

    if (pwm->pwm_id == 1) {
	duty_cycle = (duty_ns * PWM_CTRL1_MAX) / period_ns;

	ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, duty_cycle, LED_PWM_CTRL1);

	if (ret < 0) {
		pr_err("%s: Failed to configure PWM, Error %d\n",
			pwm->label, ret);
		return ret;
	}
    } else
    if (1 || 1 < pwm->pwm_id) {
	ret = twl_i2c_read_u8(TWL6030_MODULE_ID1, &duty_cycle,
		REG_PWM1ON + 3 * (pwm->pwm_id - 2));
	if (ret < 0) {
dtrace;
		return ret;
	}

	ret = ((duty_cycle & PWM_LENGTH_64) ? 64 : 128) - 1;

	duty_cycle = (duty_ns * ret) / period_ns;

	ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, duty_cycle,
		REG_PWM1OFF + 3 * (pwm->pwm_id - 2));
	if (ret < 0) {
dtrace;
		return ret;
	}
    }

	return 0;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwm)
{
	u8 val;
	int ret;

    if (pwm->pwm_id == 1) {
	ret = twl_i2c_read_u8(TWL6030_MODULE_ID1, &val, LED_PWM_CTRL2);
	if (ret < 0) {
		pr_err("%s: Failed to enable PWM, Error %d\n", pwm->label, ret);
		return ret;
	}

	/* Change mode to software control */
	val &= ~PWM_CTRL2_MODE_MASK;
	val |= PWM_CTRL2_MODE_SW;

	ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, val, LED_PWM_CTRL2);
	if (ret < 0) {
		pr_err("%s: Failed to enable PWM, Error %d\n", pwm->label, ret);
		return ret;
	}

	twl_i2c_read_u8(TWL6030_MODULE_ID1, &val, LED_PWM_CTRL2);
    } else
    if (1 || 1 < pwm->pwm_id) {
	val = (PWM1S | PWM1EN) << (3 * (pwm->pwm_id - 2));

	ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, val, REG_TOGGLE3);
	if (ret < 0) {
dtrace;
		return ret;
	}
    }

	return 0;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	u8 val;
	int ret;

    if (pwm->pwm_id == 1) {
	ret = twl_i2c_read_u8(TWL6030_MODULE_ID1, &val, LED_PWM_CTRL2);
	if (ret < 0) {
		pr_err("%s: Failed to disable PWM, Error %d\n",
			pwm->label, ret);
		return;
	}

	val &= ~PWM_CTRL2_MODE_MASK;
	val |= PWM_CTRL2_MODE_HW;

	ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, val, LED_PWM_CTRL2);
	if (ret < 0) {
		pr_err("%s: Failed to disable PWM, Error %d\n",
			pwm->label, ret);
		return;
	}
    } else
    if (1 || 1 < pwm->pwm_id) {
	val = PWM1R << (3 * (pwm->pwm_id - 2));

	ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, val, REG_TOGGLE3);
	if (ret < 0) {
dtrace;
		return ret;
	}
    }

	return;
}
EXPORT_SYMBOL(pwm_disable);

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	u8 val;
	int ret;
	struct pwm_device *pwm;

	if (3 < pwm_id || pwm_id < 1) {
		pr_err("Fail to request PWM with invalid ID(%d) for %s\n",
			pwm_id, label);
		return NULL;
	}

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		pr_err("%s: failed to allocate memory\n", label);
		return NULL;
	}

	pwm->label = label;
	pwm->pwm_id = pwm_id;

    if (pwm->pwm_id == 1) {
	/* Configure PWM */
	val = PWM_CTRL2_DIS_PD | PWM_CTRL2_CURR_02 | PWM_CTRL2_SRC_VAC |
		PWM_CTRL2_MODE_HW;

	ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, val, LED_PWM_CTRL2);

	if (ret < 0) {
		pr_err("%s: Failed to configure PWM, Error %d\n",
			 pwm->label, ret);

		kfree(pwm);
		return NULL;
	}
    } else
    if (1 || 1 < pwm->pwm_id) {
	ret = twl_i2c_read_u8(TWL6030_MODULE_ID1, &val, REG_PWDNSTATUS2);
	if (ret < 0) {
dtrace;
		kfree(pwm);	return NULL;
	}

	if (val & ((PWM1_CLK_EN | PWM1_STS) << (2 * (pwm->pwm_id - 2)))) {
		pr_err("PWM%d is already in use!\n", pwm->pwm_id);
		kfree(pwm);	return NULL;
	}

    if (0) {
	val = PWM1EN << (3 * (pwm->pwm_id - 2));

	ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, val, REG_TOGGLE3);
	if (ret < 0) {
dtrace;
		kfree(pwm);	return NULL;
	}
    }

	val = 0; //| PWM_LENGTH_64;	// we perfer to use 128 clk period.

	ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, val,
		REG_PWM1ON + 3 * (pwm->pwm_id - 2));
	if (ret < 0) {
dtrace;
		kfree(pwm);	return NULL;
	}
    }

	return pwm;
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwm)
{
	pwm_disable(pwm);
	kfree(pwm);
}
EXPORT_SYMBOL(pwm_free);
