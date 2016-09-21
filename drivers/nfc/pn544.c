/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/nfc/pn544.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <asm-generic/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>

#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#define MAX_BUFFER_SIZE	512

#define PN544_DRIVER_NAME         "pn544"

struct pn544_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;

	unsigned int		ven_gpio;
	unsigned int		firm_gpio;
	unsigned int		irq_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
};

static struct pn544_dev    *pn544_dev = NULL;
static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		return IRQ_HANDLED;
	}

	pn544_disable_irq(pn544_dev);

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);
	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->client->irq);
		ret = wait_event_interruptible(pn544_dev->read_wq,
				gpio_get_value(pn544_dev->irq_gpio));

		pn544_disable_irq(pn544_dev);

		if (ret)
			goto fail;
	}

	/* Read data */
	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) {
		dev_dbg(&pn544_dev->client->dev, "i2c_master_recv returned %d\n",
			ret);
		return ret;
	}
	if (ret > count) {
		dev_dbg(&pn544_dev->client->dev,
			"received too many bytes from i2c (%d)\n", ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		dev_dbg(&pn544_dev->client->dev, "failed to copy to user space\n");
		return -EFAULT;
	}
	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		dev_dbg(&pn544_dev->client->dev, "failed to copy from user space\n");
		return -EFAULT;
	}

	/* Write data */
	ret = i2c_master_send(pn544_dev->client, tmp, count);

	if (ret == -ENOTCONN) {  /* Retry, chip was in standby */
		usleep_range(6000, 10000);
		ret = i2c_master_send(pn544_dev->client, tmp, count);
	}

	if (ret != count) {
		dev_dbg(&pn544_dev->client->dev, "i2c_master_send returned %d\n",
			ret);
		ret = -EIO;
	}
	return ret;
}

static int pn544_i2c_device_reset(struct pn544_dev *pn544_p)
{
	int ret, old_val, retry = 3;
	char rset_cmd[] = { 0x05, 0xF9, 0x04, 0x00, 0xC3, 0xE5 };
	int count = sizeof(rset_cmd);

	old_val = gpio_get_value_cansleep(pn544_p->ven_gpio);
	if (old_val) {
		gpio_set_value_cansleep(pn544_p->ven_gpio, 0);
		usleep_range(10000, 15000);
	}
	gpio_set_value_cansleep(pn544_p->ven_gpio, 1);

	/* send reset */
	while (retry--) {
		dev_dbg(&pn544_p->client->dev, "Sending reset cmd\n");
		ret = i2c_master_send(pn544_p->client, rset_cmd, count);
		if (ret == count) {
			ret = 0;
			break;
		}
		usleep_range(10000, 15000);
	}

	if (!retry && ret != count)
		dev_err(&pn544_p->client->dev, "device not responding\n");

	if (!old_val)
		gpio_set_value_cansleep(pn544_p->ven_gpio, 0);

	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	filp->private_data = pn544_dev;
	gpio_set_value_cansleep(pn544_dev->ven_gpio, 1);

	return pn544_i2c_device_reset(pn544_dev);
}

static int pn544_dev_close(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = filp->private_data;

	gpio_set_value_cansleep(pn544_dev->ven_gpio, 0);
	gpio_set_value_cansleep(pn544_dev->firm_gpio, 0);

	return 0;
}

static long pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;

	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			gpio_set_value_cansleep(pn544_dev->ven_gpio, 1);
			gpio_set_value_cansleep(pn544_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value_cansleep(pn544_dev->ven_gpio, 0);
			msleep(50);
			gpio_set_value_cansleep(pn544_dev->ven_gpio, 1);
			msleep(10);
		} else if (arg == 1) {
			/* power on */
			gpio_set_value_cansleep(pn544_dev->firm_gpio, 0);
			gpio_set_value_cansleep(pn544_dev->ven_gpio, 1);
			//irq_set_irq_wake(pn544_dev->client->irq, 1);
			msleep(10);
		} else  if (arg == 0) {
			/* power off */
			gpio_set_value_cansleep(pn544_dev->firm_gpio, 0);
			gpio_set_value_cansleep(pn544_dev->ven_gpio, 0);
			//irq_set_irq_wake(pn544_dev->client->irq, 0);
			msleep(10);
		} else {
			dev_dbg(&pn544_dev->client->dev, "bad arg %ld\n", arg);
			return -EINVAL;
		}
		break;
	default:
		dev_dbg(&pn544_dev->client->dev, "bad ioctl %d\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static void pn544_i2c_release_resources(struct i2c_client *client)
{
	disable_irq(client->irq);
	free_irq(client->irq, pn544_dev);

	if (pn544_dev->irq_gpio)
		gpio_free(pn544_dev->irq_gpio);

	if (pn544_dev->firm_gpio)
		gpio_free(pn544_dev->firm_gpio);

	if (pn544_dev->ven_gpio)
		gpio_free(pn544_dev->ven_gpio);

	pn544_dev->irq_gpio = 0;
	pn544_dev->firm_gpio = 0;
	pn544_dev->ven_gpio = 0;
}

static int pn544_i2c_of_request_resources(struct i2c_client *client)
{
	int rc;
	struct device_node *of_node = client->dev.of_node;

	if (!of_node)
		return -EINVAL;

	rc = of_get_named_gpio(of_node, "nxp,irq-gpio", 0);
	if (rc > 0) {
		pn544_dev->irq_gpio = rc;
	} else {
		dev_err(&client->dev,
			"of_property_read(irq_gpio) fail:%d\n", rc);
		goto comm_err;
	}

	rc = of_get_named_gpio(of_node, "nxp,firm-gpio", 0);
	if (rc > 0) {
		pn544_dev->firm_gpio = rc;
	} else {
		dev_err(&client->dev,
			"of_property_read(firm_gpio) fail:%d\n", rc);
		goto comm_err;
	}

	rc = of_get_named_gpio(of_node, "nxp,ven-gpio", 0);	//8974 mpp7
	if (rc > 0) {
		pn544_dev->ven_gpio = rc;
	} else {
		dev_err(&client->dev,
			"of_property_read(ven_gpio) fail:%d\n", rc);
		goto comm_err;
	}

	rc = gpio_tlmm_config(GPIO_CFG(pn544_dev->irq_gpio, 0,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		dev_err(&client->dev, "Could not configure nfc gpio %d\n",
			pn544_dev->irq_gpio);
		goto comm_err;
	}

	rc = gpio_request(pn544_dev->irq_gpio, "irq_gpio");
	if (rc) {
		dev_err(&client->dev,"unable to request nfc gpio %d (%d)\n",
			pn544_dev->irq_gpio, rc);
		goto comm_err;
	}

	rc = gpio_tlmm_config(GPIO_CFG(pn544_dev->firm_gpio, 0,
			      GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
			      GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	if (rc) {
		dev_err(&client->dev, "Could not configure nfc gpio %d\n",
			pn544_dev->firm_gpio);
		goto free_irq_gpio;
	}

	rc = gpio_request(pn544_dev->firm_gpio, "firm_gpio");
	if (rc) {
		dev_err(&client->dev, "unable to request nfc gpio %d (%d)\n",
					pn544_dev->firm_gpio, rc);
		goto free_irq_gpio;
	}

	gpio_direction_output(pn544_dev->firm_gpio, 0);

	/*ven gpio out*/
	rc = gpio_tlmm_config(GPIO_CFG(pn544_dev->ven_gpio, 0,
			      GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
			      GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	if (rc) {
		dev_err(&client->dev, "Could not configure nfc gpio %d\n",
			pn544_dev->ven_gpio);
		goto free_firm_gpio;
	}

	rc = gpio_request(pn544_dev->ven_gpio, "ven_gpio");
	if (rc) {
		dev_err(&client->dev, "unable to request nfc gpio %d (%d)\n",
			pn544_dev->ven_gpio, rc);
		goto free_firm_gpio;
	}

	gpio_direction_output(pn544_dev->ven_gpio, 0);

	/*
	 * request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	rc = request_irq(client->irq, pn544_dev_irq_handler,
			 IRQF_TRIGGER_HIGH, client->name, pn544_dev);
	if (rc) {
		dev_err(&client->dev, "Could request  gpio IRQ (%d)\n", rc);
		goto free_ven_gpio;
	} else {
		disable_irq_nosync(client->irq);
		pn544_dev->irq_enabled = false;
	}

	return 0;


free_ven_gpio:
	gpio_free(pn544_dev->ven_gpio);
free_firm_gpio:
	gpio_free(pn544_dev->firm_gpio);
free_irq_gpio:
	gpio_free(pn544_dev->irq_gpio);
comm_err:
	pn544_dev->irq_gpio = 0;
	pn544_dev->firm_gpio = 0;
	pn544_dev->ven_gpio = 0;

	return rc;
}

static const struct file_operations pn544_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.release = pn544_dev_close,
	.unlocked_ioctl  = pn544_dev_ioctl,
};


static int pn544_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;

	if (pn544_dev != NULL) {
		dev_err(&client->dev,
			"pn544_probe: multiple devices NOT supported\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	/* private data allocation */
	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}
	pn544_dev->client = client;

	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	ret = pn544_i2c_of_request_resources(client);
	if (ret < 0) {
		dev_err(&client->dev, "Cannot get platform resources\n");
		goto err_get_device_resources;
	}

	ret = pn544_i2c_device_reset(pn544_dev);
	if (ret != 0) {
		dev_err(&client->dev, "Could not detect nfc pn544 chip\n");
		goto err_misc_register;
	}

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = "pn544";
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		dev_err(&client->dev, "misc_register failed\n");
		goto err_misc_register;
	}

	i2c_set_clientdata(client, pn544_dev);
	return 0;

err_misc_register:
	pn544_i2c_release_resources(client);
err_get_device_resources:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
	pn544_dev = NULL;
	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	pn544_dev = i2c_get_clientdata(client);

	misc_deregister(&pn544_dev->pn544_device);
	pn544_i2c_release_resources(client);
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
	pn544_dev = NULL;

	return 0;
}

static const struct i2c_device_id pn544_id[] = {
	{ PN544_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id nfc_match_table[] = {
	{.compatible = "nxp,pn544",},
	{ },
};

static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= {
		.name   = "pn544",
		.owner = THIS_MODULE,
		.of_match_table = nfc_match_table,
	},
};

/*
 * module load/unload record keeping
 */
static int __init pn544_dev_init(void)
{
	pr_info("Loading pn544 driver\n");
	return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	pr_info("Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
