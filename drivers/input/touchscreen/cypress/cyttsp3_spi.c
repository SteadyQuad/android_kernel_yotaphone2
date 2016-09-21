/*
 * Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) SPI touchscreen driver.
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

#include "cyttsp3_core.h"

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#define CY_SPI_WR_OP      0x00 /* r/~w */
#define CY_SPI_RD_OP      0x01
#define CY_SPI_CMD_BYTES  4
#define CY_SPI_SYNC_BYTE  2
#define CY_SPI_SYNC_ACK1  0x62 /* from protocol v.2 */
#define CY_SPI_SYNC_ACK2  0x9D /* from protocol v.2 */
#define CY_SPI_DATA_SIZE  128
#define CY_SPI_DATA_BUF_SIZE (CY_SPI_CMD_BYTES + CY_SPI_DATA_SIZE)
#define CY_SPI_BITS_PER_WORD 8

#ifdef CONFIG_TOUCHSCREEN_DEBUG
#define cyttsp_spidbg(ts, l, f, a...) {\
	if (ts->bus_ops.tsdebug >= (l)) \
		pr_info(f, ## a);\
}
#else
#define cyttsp_spidbg(ts, l, f, a...)
#endif

struct cyttsp_spi {
	struct cyttsp_bus_ops bus_ops;
	struct spi_device *spi_client;
	void *ttsp_client;
	u8 wr_buf[CY_SPI_DATA_BUF_SIZE];
	u8 rd_buf[CY_SPI_DATA_BUF_SIZE];
};

static int cyttsp_spi_xfer(u8 op, struct cyttsp_spi *ts,
	u8 reg, u8 *buf, int length)
{
	struct spi_message msg;
	struct spi_transfer xfer[2];
	u8 *wr_buf = ts->wr_buf;
	u8 *rd_buf = ts->rd_buf;
	int retval;

	if (length > CY_SPI_DATA_SIZE) {
		cyttsp_spidbg(ts, CY_DBG_LVL_3,
			"%s: length %d is too big.\n",
			__func__, length);
		return -EINVAL;
	}

	memset(wr_buf, 0, CY_SPI_DATA_BUF_SIZE);
	memset(rd_buf, 0, CY_SPI_DATA_BUF_SIZE);

	wr_buf[0] = 0x00; /* header byte 0 */
	wr_buf[1] = 0xFF; /* header byte 1 */
	wr_buf[2] = reg;  /* reg index */
	wr_buf[3] = op;   /* r/~w */
	if (op == CY_SPI_WR_OP)
		memcpy(wr_buf + CY_SPI_CMD_BYTES, buf, length);

	memset((void *)xfer, 0, sizeof(xfer));
	spi_message_init(&msg);
	xfer[0].tx_buf = wr_buf;
	xfer[0].rx_buf = rd_buf;
	if (op == CY_SPI_WR_OP) {
		xfer[0].len = length + CY_SPI_CMD_BYTES;
		spi_message_add_tail(&xfer[0], &msg);
	} else if (op == CY_SPI_RD_OP) {
		xfer[0].len = CY_SPI_CMD_BYTES;
		spi_message_add_tail(&xfer[0], &msg);

		xfer[1].rx_buf = buf;
		xfer[1].len = length;
		spi_message_add_tail(&xfer[1], &msg);
	}

	retval = spi_sync(ts->spi_client, &msg);
	if (retval < 0) {
		cyttsp_spidbg(ts, CY_DBG_LVL_3,
			"%s: spi_sync() error %d, len=%d, op=%d\n",
			__func__, retval, xfer[1].len, op);

		/*
		 * do not return here since probably a bad ACK sequence
		 * let the following ACK check handle any errors and
		 * allow silent retries
		 */
	}

	if ((rd_buf[CY_SPI_SYNC_BYTE] == CY_SPI_SYNC_ACK1) &&
		(rd_buf[CY_SPI_SYNC_BYTE+1] == CY_SPI_SYNC_ACK2))
		retval = 0;
	else {
		int i;
		for (i = 0; i < (CY_SPI_CMD_BYTES); i++)
			cyttsp_spidbg(ts, CY_DBG_LVL_3,
				"%s: test rd_buf[%d]:0x%02x\n",
				__func__, i, rd_buf[i]);
		for (i = 0; i < (length); i++)
			cyttsp_spidbg(ts, CY_DBG_LVL_3,
				"%s: test buf[%d]:0x%02x\n",
				__func__, i, buf[i]);

		/* signal ACK error so silent retry */
		retval = 1;
	}

	return retval;
}

static int ttsp_spi_read_block_data(void *handle, u8 addr,
	u8 length, void *data)
{
	struct cyttsp_spi *ts =
		container_of(handle, struct cyttsp_spi, bus_ops);
	int retval = 0;

	if (data == NULL) {
		pr_err("%s: packet data missing error\n", __func__);
		retval = -EINVAL;
		goto ttsp_spi_read_block_data_exit;
	}

	if ((length == 0) || (length > CY_SPI_DATA_SIZE)) {
		pr_err("%s: packet length=%d error (max=%d)\n",
			__func__, length, CY_SPI_DATA_SIZE);
		retval = -EINVAL;
		goto ttsp_spi_read_block_data_exit;
	}

	retval = cyttsp_spi_xfer(CY_SPI_RD_OP, ts, addr, data, length);
	if (retval < 0)
		pr_err("%s: ttsp_spi_read_block_data failed\n",
			__func__);

	/*
	 * Do not print the above error if the data sync bytes were not found.
	 * This is a normal condition for the bootloader loader startup and need
	 * to retry until data sync bytes are found.
	 */
	if (retval > 0)
		retval = -EIO;	/* now signal fail; so retry can be done */

ttsp_spi_read_block_data_exit:
	return retval;
}

static int ttsp_spi_write_block_data(void *handle, u8 addr,
	u8 length, const void *data)
{
	struct cyttsp_spi *ts =
		container_of(handle, struct cyttsp_spi, bus_ops);
	int retval = 0;

	if (data == NULL) {
		pr_err("%s: packet data missing error\n", __func__);
		retval = -EINVAL;
		goto ttsp_spi_write_block_data_exit;
	}

	if ((length == 0) || (length > CY_SPI_DATA_SIZE)) {
		pr_err("%s: packet length=%d error (max=%d)\n",
			__func__, length, CY_SPI_DATA_SIZE);
		retval = -EINVAL;
		goto ttsp_spi_write_block_data_exit;
	}

	retval = cyttsp_spi_xfer(CY_SPI_WR_OP, ts, addr, (void *)data, length);
	if (retval < 0)
		pr_err("%s: ttsp_spi_write_block_data failed\n",
			__func__);

	/*
	 * Do not print the above error if the data sync bytes were not found.
	 * This is a normal condition for the bootloader loader startup and need
	 * to retry until data sync bytes are found.
	 */
	if (retval > 0)
		retval = -EIO;	/* now signal fail; so retry can be done */

ttsp_spi_write_block_data_exit:
	return retval;
}

#ifdef CONFIG_OF
static struct of_device_id cyttsp_spi_of_match[] = {
	{ .compatible = "cy,cyttsp3-spi", },
	{ }
};
MODULE_DEVICE_TABLE(of, cyttsp_spi_of_match);
#endif

static int __devinit cyttsp_spi_probe(struct spi_device *spi)
{
	struct cyttsp_spi *ts;
	int retval = 0;

	/* Set up SPI*/
	spi->bits_per_word = CY_SPI_BITS_PER_WORD;
	spi->mode = SPI_MODE_0;
	retval = spi_setup(spi);
	if (retval < 0) {
		pr_err("%s: SPI setup error %d\n",
			__func__, retval);
		goto cyttsp_spi_probe_exit;
	}

	ts = kzalloc(sizeof(struct cyttsp_spi), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("%s: Error, kzalloc.\n", __func__);
		retval = -ENOMEM;
		goto cyttsp_spi_probe_exit;
	}

	ts->spi_client = spi;
	dev_set_drvdata(&spi->dev, ts);
	ts->bus_ops.write = ttsp_spi_write_block_data;
	ts->bus_ops.read = ttsp_spi_read_block_data;
	ts->bus_ops.dev = &spi->dev;

	/* TODO: Add device tree support */
	ts->ttsp_client = cyttsp_core_init(&ts->bus_ops, &spi->dev,
		spi->irq, spi->modalias, of_match_device(
			of_match_ptr(cyttsp_spi_of_match), &spi->dev));
	if (ts->ttsp_client == NULL) {
		kfree(ts);
		ts = NULL;
		retval = -ENODATA;
		pr_err("%s: Registration fail ret=%d\n", __func__, retval);
		goto cyttsp_spi_probe_exit;
	}

	pr_info("%s: Registration complete\n", __func__);

cyttsp_spi_probe_exit:
	return retval;
}

static int __devexit cyttsp_spi_remove(struct spi_device *spi)
{
	struct cyttsp_spi *ts;
	int retval = 0;

	ts = dev_get_drvdata(&spi->dev);
	if (ts == NULL) {
		pr_err("%s: client pointer error\n", __func__);
		retval = -EINVAL;
	} else {
		cyttsp_core_release(ts->ttsp_client);
		kfree(ts);
	}
	return retval;
}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int cyttsp_spi_suspend(struct spi_device *spi, pm_message_t message)
{
	return cyttsp_suspend(dev_get_drvdata(&spi->dev));
}

static int cyttsp_spi_resume(struct spi_device *spi)
{
	return cyttsp_resume(dev_get_drvdata(&spi->dev));
}
#endif

static struct spi_driver cyttsp_spi_driver = {
	.driver = {
		.name = CY_SPI_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cyttsp_spi_of_match),
	},
	.probe = cyttsp_spi_probe,
	.remove = __devexit_p(cyttsp_spi_remove),
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = cyttsp_spi_suspend,
	.resume = cyttsp_spi_resume,
#endif
};

static int __init cyttsp_spi_init(void)
{
	return spi_register_driver(&cyttsp_spi_driver);
}

//module_init(cyttsp_spi_init);
deferred_module_init_1(cyttsp_spi_init);

static void __exit cyttsp_spi_exit(void)
{
	spi_unregister_driver(&cyttsp_spi_driver);
}
module_exit(cyttsp_spi_exit);

MODULE_ALIAS("spi:cyttsp");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) SPI driver");
MODULE_AUTHOR("Cypress");

