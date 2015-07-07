/*
 * cyttsp5_platform.c
 * Cypress TrueTouch(TM) Standard Product V5 Platform Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2013 Cypress Semiconductor
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
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

/* cyttsp */
#include <linux/cyttsp5_bus.h>
#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_platform.h>

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
#include "cyttsp5_fw.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
static struct cyttsp5_touch_firmware cyttsp5_firmware_front = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};

static struct cyttsp5_touch_firmware cyttsp5_firmware_back = {
        .img = NULL,
        .size = 0,
        .ver = NULL,
        .vsize = 0,
};

#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
#include "linux/input/cyttsp5_front_params.h"
#include "linux/input/cyttsp5_back_params.h"

static struct touch_settings cyttsp5_sett_param_regs_front = {
	.data = (uint8_t *)&cyttsp4_param_regs_front[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_front),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_front = {
	.data = (uint8_t *)&cyttsp4_param_size_front[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_front),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_front = {
	.param_regs = &cyttsp5_sett_param_regs_front,
	.param_size = &cyttsp5_sett_param_size_front,
	.fw_ver = ttconfig_fw_ver_front,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_front),
};

static struct touch_settings cyttsp5_sett_param_regs_back = {
        .data = (uint8_t *)&cyttsp4_param_regs_back[0],
        .size = ARRAY_SIZE(cyttsp4_param_regs_back),
        .tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_back = {
        .data = (uint8_t *)&cyttsp4_param_size_back[0],
        .size = ARRAY_SIZE(cyttsp4_param_size_back),
        .tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_back = {
        .param_regs = &cyttsp5_sett_param_regs_back,
        .param_size = &cyttsp5_sett_param_size_back,
        .fw_ver = ttconfig_fw_ver_back,
        .fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_back),
};

#else
static struct cyttsp5_touch_config cyttsp5_ttconfig_front = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_back = {
        .param_regs = NULL,
        .param_size = NULL,
        .fw_ver = NULL,
        .fw_vsize = 0,
};
#endif

struct cyttsp5_loader_platform_data _cyttsp5_loader_platform_data_front = {
	.fw = &cyttsp5_firmware_front,
	.ttconfig = &cyttsp5_ttconfig_front,
	.flags = CY_LOADER_FLAG_NONE,
};

struct cyttsp5_loader_platform_data _cyttsp5_loader_platform_data_back = {
        .fw = &cyttsp5_firmware_back,
        .ttconfig = &cyttsp5_ttconfig_back,
        .flags = CY_LOADER_FLAG_NONE,
};

int cyttsp5_xres(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int rc = 0;
		gpio_set_value(rst_gpio, 1);
		msleep(20);
		gpio_set_value(rst_gpio, 0);
		msleep(40);
		gpio_set_value(rst_gpio, 1);
		msleep(20);
		dev_info(dev,
				"%s: RESET CYTTSP gpio=%d r=%d\n", __func__,
				pdata->rst_gpio, rc);
	return rc;
}

#if 0
static int power_init(struct cyttsp5_core_platform_data *data, bool on,
		struct device *dev)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vcc_i2c = regulator_get(dev->parent, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(dev, "Regulator get failed vcc_i2c rc=%d\n",
				rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, 1800000, 1800000);
		if (rc) {
			dev_err(dev, "Regulator set_vtg failed vcc_i2c rc=%d\n",
					rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, 1800000);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int power_on(struct cyttsp5_core_platform_data *data, bool on,
		struct device *dev)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(dev, "Regulator vcc_i2c enable failed rc=%d\n",
				rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(dev, "Regulator vcc_i2c disable failed rc=%d\n",
				rc);
		regulator_enable(data->vdd);
	}

	return rc;
}

#endif

int cyttsp5_init(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (on) {

#if 0
		rc = power_init(pdata, 1, dev);
		if (rc)
			dev_err(dev, "%s: power init failed\n", __func__);

		rc = power_on(pdata, 1, dev);
		if (rc)
			dev_err(dev, "%s: power on failed\n", __func__);

#endif
		rc = gpio_request(rst_gpio, "CTP_TMA568_RESET");
		if (rc < 0) {
			gpio_free(rst_gpio);
			rc = gpio_request(rst_gpio, "CTP_TMA568_RESET");
		}
		if (rc < 0) {
			dev_err(dev,
				"%s: Fail request gpio=%d\n", __func__,
				rst_gpio);
		} else {
			rc = gpio_direction_output(rst_gpio, 1);
			if (rc < 0) {
				pr_err("%s: Fail set output gpio=%d\n",
					__func__, rst_gpio);
				gpio_free(rst_gpio);
			} else {
				rc = gpio_request(irq_gpio, "CTP_TMA568_INT");
				if (rc < 0) {
					gpio_free(irq_gpio);
					rc = gpio_request(irq_gpio,
						NULL);
				}
				if (rc < 0) {
					dev_err(dev,
						"%s: Fail request gpio=%d\n",
						__func__, irq_gpio);
					gpio_free(rst_gpio);
				} else {
					gpio_direction_input(irq_gpio);
				}
			}
		}
	} else {
		gpio_free(rst_gpio);
		gpio_free(irq_gpio);
#if 0
		power_on(pdata, 0, dev);
		power_init(pdata, 0, dev);
#endif
	}

	dev_info(dev,
		"%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d r=%d\n",
		__func__, rst_gpio, irq_gpio, rc);
	return rc;
}


static int cyttsp5_wakeup(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (ignore_irq)
		atomic_set(ignore_irq, 1);
	rc = gpio_direction_output(irq_gpio, 0);
	if (rc < 0) {
		if (ignore_irq)
			atomic_set(ignore_irq, 0);
		dev_err(dev,
			"%s: Fail set output gpio=%d\n",
			__func__, irq_gpio);
	} else {
		udelay(2000);
		rc = gpio_direction_input(irq_gpio);
		if (ignore_irq)
			atomic_set(ignore_irq, 0);
		if (rc < 0) {
			dev_err(dev,
				"%s: Fail set input gpio=%d\n",
				__func__, irq_gpio);
		}
	}

	dev_info(dev,
		"%s: WAKEUP CYTTSP gpio=%d r=%d\n", __func__,
		irq_gpio, rc);
	return rc;
}

static int cyttsp5_sleep(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

int cyttsp5_power(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp5_wakeup(pdata, dev, ignore_irq);

	return cyttsp5_sleep(pdata, dev, ignore_irq);
}

int cyttsp5_irq_stat(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}
