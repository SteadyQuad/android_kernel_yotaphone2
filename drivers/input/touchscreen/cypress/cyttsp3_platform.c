/*
 * Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) platform module.
 * For use with Cypress Txx2xx and Txx3xx parts.
 * Supported parts include:
 * CY8CTST242
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009-2012 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/input/cyttsp3_touch_platform.h>

#define CY_USE_AUTOLOAD_FW

#ifdef CY_USE_AUTOLOAD_FW
#include "cyttsp3_img.h"
struct cyttsp3_touch_firmware cyttsp3_firmware = {
	.img = cyttsp3_img,
	.size = sizeof(cyttsp3_img),
	.ver = cyttsp3_ver,
	.vsize = sizeof(cyttsp3_ver),
};
#else
struct cyttsp3_touch_firmware cyttsp3_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif
EXPORT_SYMBOL_GPL(cyttsp3_firmware);

int cyttsp3_hw_reset(const struct cyttsp3_touch_platform_data *pdata)
{
	int rst_gpio = pdata->rst_gpio;
	int retval = 0;

	retval = gpio_request(rst_gpio, NULL);
	if (retval < 0) {
		pr_err("%s: Fail request RST pin r=%d\n", __func__, retval);
		pr_err("%s: Try free RST gpio=%d\n", __func__, rst_gpio);
		gpio_free(rst_gpio);
		retval = gpio_request(rst_gpio, NULL);
		if (retval < 0) {
			pr_err("%s: Fail 2nd request RST pin r=%d\n", __func__,
				retval);
		}
	}

	if (!(retval < 0)) {
		pr_info("%s: strobe RST(%d) pin\n", __func__, rst_gpio);
		gpio_direction_output(rst_gpio, 0);
		msleep(20);
		gpio_set_value(rst_gpio, 1);
		msleep(40);
		gpio_set_value(rst_gpio, 0);
		msleep(20);
		gpio_free(rst_gpio);
	}

	return retval;
}

EXPORT_SYMBOL_GPL(cyttsp3_hw_reset);

#define CY_WAKE_DFLT                99	/* causes wake strobe on INT line
					 * in sample board configuration
					 * platform data->hw_recov() function
					 */
int cyttsp3_hw_recov(const struct cyttsp3_touch_platform_data *pdata,
		int on)
{
	int irq_gpio = pdata->irq_gpio;
	int retval = 0;

	switch (on) {
	case 0:
		cyttsp3_hw_reset(pdata);
		retval = 0;
		break;
	case CY_WAKE_DFLT:
		retval = gpio_direction_output(irq_gpio, 0);
		if (retval < 0) {
			pr_err("%s: Fail switch IRQ pin to OUT r=%d\n",
				__func__, retval);
		} else {
			udelay(2000);
			retval = gpio_direction_input(irq_gpio);
			if (retval < 0) {
				pr_err("%s: Fail switch IRQ pin to IN"
					" r=%d\n", __func__, retval);
			}
		}
		break;
	default:
		retval = -ENOSYS;
		break;
	}

	return retval;
}
EXPORT_SYMBOL_GPL(cyttsp3_hw_recov);

int cyttsp3_irq_stat(const struct cyttsp3_touch_platform_data *pdata)
{
	int irq_gpio = pdata->irq_gpio;
	int irq_stat = 0;
	int retval = 0;
	printk(KERN_CRIT "inside cyttsp3_irq_stat\n");
	retval = gpio_request(irq_gpio, NULL);
	if (retval < 0) {
		pr_err("%s: Fail request IRQ pin r=%d\n", __func__, retval);
		pr_err("%s: Try free IRQ gpio=%d\n", __func__, irq_gpio);
		gpio_free(irq_gpio);
		retval = gpio_request(irq_gpio, NULL);
		if (retval < 0) {
			pr_err("%s: Fail 2nd request IRQ pin r=%d\n",
				__func__, retval);
		}
	}

	if (!(retval < 0)) {
		irq_stat = gpio_get_value(irq_gpio);
		gpio_free(irq_gpio);
	}

	return irq_stat;
}
EXPORT_SYMBOL_GPL(cyttsp3_irq_stat);
