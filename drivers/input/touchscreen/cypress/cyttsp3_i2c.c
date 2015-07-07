/*
 * Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) I2C touchscreen driver.
 * For use with Cypress Txx2xx and Txx3xx parts.
 * Supported parts include:
 * CY8CTST242
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009-2012 Cypress Semiconductor, Inc.
 * Copyright (C) 2010-2011 Motorola Mobility, Inc.
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

#include "cyttsp3_core.h"

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>

#define CY_I2C_DATA_SIZE  128

struct cyttsp_i2c {
	struct cyttsp_bus_ops ops;
	struct i2c_client *client;
	void *ttsp_client;
	u8 wr_buf[CY_I2C_DATA_SIZE];
};

static int ttsp_i2c_read_block_data(void *handle, u8 addr,
	u8 length, void *data)
{
	struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);
	int retval = 0;

	if (data == NULL) {
		pr_err("%s: packet data missing error\n", __func__);
		retval = -EINVAL;
		goto ttsp_i2c_read_block_data_exit;
	}

	if ((length == 0) || (length > CY_I2C_DATA_SIZE)) {
		pr_err("%s: packet length=%d error (max=%d)\n",
			__func__, length, CY_I2C_DATA_SIZE);
		retval = -EINVAL;
		goto ttsp_i2c_read_block_data_exit;
	}

	retval = i2c_master_send(ts->client, &addr, 1);
	if (retval < 0)
		goto ttsp_i2c_read_block_data_exit;
	else if (retval != 1) {
		retval = -EIO;
		goto ttsp_i2c_read_block_data_exit;
	}

	retval = i2c_master_recv(ts->client, data, length);

ttsp_i2c_read_block_data_exit:
	return (retval < 0) ? retval : retval != length ? -EIO : 0;
}

static int ttsp_i2c_write_block_data(void *handle, u8 addr,
	u8 length, const void *data)
{
	struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);
	int retval = 0;

	if (data == NULL) {
		pr_err("%s: packet data missing error\n", __func__);
		retval = -EINVAL;
		goto ttsp_i2c_write_block_data_exit;
	}

	if ((length == 0) || (length > CY_I2C_DATA_SIZE)) {
		pr_err("%s: packet length=%d error (max=%d)\n",
			__func__, length, CY_I2C_DATA_SIZE);
		retval = -EINVAL;
		goto ttsp_i2c_write_block_data_exit;
	}

	ts->wr_buf[0] = addr;
	memcpy(&ts->wr_buf[1], data, length);

	retval = i2c_master_send(ts->client, ts->wr_buf, length+1);

ttsp_i2c_write_block_data_exit:
	return (retval < 0) ? retval : retval != length+1 ? -EIO : 0;
}

#ifdef CONFIG_OF
static struct of_device_id cyttsp_i2c_of_match[] = {
	{ .compatible = "cy,cyttsp3-i2c", },
	{ }
};
MODULE_DEVICE_TABLE(of, cyttsp_i2c_of_match);
#endif

static int __devinit cyttsp_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct cyttsp_i2c *ts;
	int retval = 0;

	pr_info("%s: Starting %s probe...\n", __func__, CY_I2C_NAME);

	driver_fail = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: fail check I2C functionality\n", __func__);
		retval = -EIO;
		goto cyttsp_i2c_probe_exit;
	}

	/* allocate and clear memory */
	ts = kzalloc(sizeof(struct cyttsp_i2c), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("%s: Error, kzalloc.\n", __func__);
		retval = -ENOMEM;
		goto cyttsp_i2c_probe_exit;
	}

	/* register driver_data */
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->ops.write = ttsp_i2c_write_block_data;
	ts->ops.read = ttsp_i2c_read_block_data;
	ts->ops.dev = &client->dev;
	ts->ops.dev->bus = &i2c_bus_type;

	ts->ttsp_client = cyttsp_core_init(&ts->ops, &client->dev,
		client->irq, client->name, of_match_device(
			of_match_ptr(cyttsp_i2c_of_match), &client->dev));

	if (ts->ttsp_client == NULL) {
		kfree(ts);
		ts = NULL;
		retval = -ENODATA;
		pr_err("%s: Registration fail ret=%d\n", __func__, retval);
		goto cyttsp_i2c_probe_exit;
	}
	if (driver_fail) {
		pr_err("%s: Capsense controller is absent...\n",__func__);
		retval = -ENODATA;
		cyttsp_core_release(ts->ttsp_client);
		kfree(ts);
		goto cyttsp_i2c_probe_exit;
	}
	pr_info("%s: Registration complete\n", __func__);

cyttsp_i2c_probe_exit:
	return retval;
}

/* registered in driver struct */
static int __devexit cyttsp_i2c_remove(struct i2c_client *client)
{
/*	struct cyttsp_i2c *ts;
	int retval = 0;

	ts = i2c_get_clientdata(client);
	if (ts == NULL) {
		pr_err("%s: client pointer error\n", __func__);
		retval = -EINVAL;
	} else {
		cyttsp_core_release(ts->ttsp_client);
		kfree(ts);
	}
	return retval; */
	return 0;
}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int cyttsp_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	struct cyttsp_i2c *ts = i2c_get_clientdata(client);

	return cyttsp_suspend(ts);
}

static int cyttsp_i2c_resume(struct i2c_client *client)
{
	struct cyttsp_i2c *ts = i2c_get_clientdata(client);

	return cyttsp_resume(ts);
}
#endif

static const struct i2c_device_id cyttsp_i2c_id[] = {
	{ CY_I2C_NAME, 0 },  { }
};

static struct i2c_driver cyttsp_i2c_driver = {
	.driver = {
		.name = CY_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cyttsp_i2c_of_match),
	},
	.probe = cyttsp_i2c_probe,
	.remove = __devexit_p(cyttsp_i2c_remove),
	.id_table = cyttsp_i2c_id,
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = cyttsp_i2c_suspend,
	.resume = cyttsp_i2c_resume,
#endif
};

static int __init cyttsp_i2c_init(void)
{
	return i2c_add_driver(&cyttsp_i2c_driver);
}

static void __exit cyttsp_i2c_exit(void)
{
	return i2c_del_driver(&cyttsp_i2c_driver);
}

//module_init(cyttsp_i2c_init);
deferred_module_init_1(cyttsp_i2c_init);

module_exit(cyttsp_i2c_exit);

MODULE_ALIAS("i2c:cyttsp");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) I2C driver");
MODULE_AUTHOR("Cypress");
MODULE_DEVICE_TABLE(i2c, cyttsp_i2c_id);
