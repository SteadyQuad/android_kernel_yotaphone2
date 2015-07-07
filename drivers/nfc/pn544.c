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
//#include <linux/clk.h>

//#define pr_err printk
//#define pr_debug printk
//#define pr_warning printk

//#define DEBUG_NFC_DRIVER

#define MAX_BUFFER_SIZE	512

#define PN544_DRIVER_NAME         "pn544"

struct pn544_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	struct regulator        *regulator ;
	struct regulator        *regulator_s3 ;

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
	int ret, i;

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
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	for(i = 0; i < ret; i++){
		printk(" %02X", tmp[i]);
	}

	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	#ifdef DEBUG_NFC_DRIVER
	int i;
	#endif

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	#ifdef DEBUG_NFC_DRIVER
	printk("%s : writing %zu bytes.\n", __func__, count);
	printk("DH->NFCC:");
	for(i = 0; i < count; i++){
	        printk(" %02X", tmp[i]);
	}
	printk("\n");
	#endif

	/* Write data */
	ret = i2c_master_send(pn544_dev->client, tmp, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = container_of(filp->private_data,
						struct pn544_dev,
						pn544_device);

	filp->private_data = pn544_dev;
	pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));
        #ifdef DEBUG_NFC_DRIVER
        printk("%s: Before setting high :: ven_gpio : %d \n",__func__,gpio_get_value(pn544_dev->ven_gpio));
        #endif

        gpio_set_value(pn544_dev->ven_gpio, 1);

        #ifdef DEBUG_NFC_DRIVER
        printk("%s: After setting high :: ven_gpio : %d \n",__func__,gpio_get_value(pn544_dev->ven_gpio));
        #endif


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
			printk("%s power on with firmware\n", __func__);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(50);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
		} else if (arg == 1) {
			/* power on */
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			//irq_set_irq_wake(pn544_dev->client->irq, 1);
			msleep(10);
		} else  if (arg == 0) {
			/* power off */
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			//irq_set_irq_wake(pn544_dev->client->irq, 0);
			msleep(10);
		} else {
			printk("%s bad arg %ld\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		printk("%s bad ioctl %d\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}


static int nxp_pn544_reset(void)
{
	int rc;
	rc = gpio_tlmm_config(GPIO_CFG(pn544_dev->irq_gpio, 0,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	  if (rc) {
			printk( "%s: Could not configure nfc gpio %d\n",
					__func__, pn544_dev->irq_gpio);
			 return -EIO;
		   }

	 rc = gpio_request(pn544_dev->irq_gpio, "irq_gpio");
	 if (rc) {
			printk( "%s: unable to request nfc gpio %d (%d)\n",
					__func__, pn544_dev->irq_gpio, rc);
			 return -EIO;
		    }


	  rc = gpio_tlmm_config(GPIO_CFG(pn544_dev->firm_gpio, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	  //printk("pn544 config firmgpio pull down\n");
	  if (rc) {
			printk( "%s: Could not configure nfc gpio %d\n",
					__func__, pn544_dev->firm_gpio);
			 return -EIO;
		   }

	 rc = gpio_request(pn544_dev->firm_gpio, "firm_gpio");
	 if (rc) {
			printk( "%s: unable to request nfc gpio %d (%d)\n",
					__func__, pn544_dev->firm_gpio, rc);
			 return -EIO;
	 }
     gpio_direction_output(pn544_dev->firm_gpio, 0);
     /*ven gpio out*/
	 rc = gpio_tlmm_config(GPIO_CFG(pn544_dev->ven_gpio, 0,
	                                   GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
	                                   GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    //printk("pn544 config vengpio out put no pull\n");
    if (rc) {
        printk( "%s: Could not configure nfc gpio %d\n",
                __func__, pn544_dev->ven_gpio);
        return -EIO;
    }

	rc = gpio_request(pn544_dev->ven_gpio, "ven_gpio");
	if (rc) {
			printk( "%s: unable to request nfc gpio %d (%d)\n",
					__func__,pn544_dev->ven_gpio, rc);
			 return -EIO;
	}
	gpio_direction_output(pn544_dev->ven_gpio, 0);
	return 0;
}

static const struct file_operations pn544_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.unlocked_ioctl  = pn544_dev_ioctl,
};


static int pn544_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;

	  struct device_node *of_node = NULL;
	//static struct clk *bb_clk2;  /*added bb_clk2 for nfc , start */

	printk("pn544_probe(): start\n");
        if (pn544_dev != NULL)
	{
		 printk("pn544_probe: multiple devices NOT supported\n");
		 ret = -ENODEV;
		goto err_single_device;
	 }

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	/* private data allocation */
	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	pn544_dev->client   = client;


   if (client->dev.of_node) {

	of_node = client->dev.of_node;

	ret = of_get_named_gpio(of_node, "nxp,irq-gpio", 0);
	if (ret>0) {
		pn544_dev->irq_gpio=ret;
	} else{
		printk("pn544_probe: of_property_read(irq_gpio) fail:%d\n",ret);
		goto err_device_create_failed;
	}

	ret = of_get_named_gpio(of_node, "nxp,firm-gpio", 0);
	if (ret>0) {
		pn544_dev->firm_gpio=ret;
	} else{
		printk("pn544_probe: of_property_read(firm_gpio) fail:%d\n",ret);
		goto err_device_create_failed;
	}

	ret = of_get_named_gpio(of_node, "nxp,ven-gpio", 0);	//8974 mpp7
	if (ret>0) {
		pn544_dev->ven_gpio=ret;
	} else{
		printk("pn544_probe: of_property_read(ven_gpio) fail:%d\n",ret);
		goto err_device_create_failed;
	}


	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = "pn544";
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */

    ret =nxp_pn544_reset();
    if (ret < 0) {
        printk(  "can't reset device\n");
    //    goto err_device_create_file_failed;
    }


    /*bb_clk2 = clk_get(&client->dev, "bb_clk2");
	if (IS_ERR(bb_clk2)) {
		printk(KERN_ERR "%s: Error getting bb_clk2\n", __func__);
		bb_clk2 = NULL;
		//return -ENOENT;
	}
	else{
		printk("%s: start prepare bb_clk2\n", __func__);
        ret = clk_prepare_enable(bb_clk2);
		if(ret){
		    printk(KERN_ERR "%s: prepare bb_clk2 failed ret:%d\n", __func__, ret);
	    }
	}*/

	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	pn544_dev->irq_enabled = true;

	ret = request_irq(client->irq, pn544_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, pn544_dev);
	if (ret) {
		printk("request_irq failed\n");
		goto err_request_irq_failed;
	}

	pn544_disable_irq(pn544_dev);
	i2c_set_clientdata(client, pn544_dev);
	printk("nfc probe is ok\n");
	return 0;
   }
err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
err_device_create_failed:
   kfree(pn544_dev);
   pn544_dev = NULL;
//err_device_create_file_failed:
 //   device_destroy(pn544_dev_class, MKDEV(pn544_major, pn544_minor));
err_exit:
	//gpio_free(platform_data->firm_gpio);
err_single_device:
	return ret;

}

static int pn544_remove(struct i2c_client *client)
{
	pn544_dev = i2c_get_clientdata(client);
	pn544_disable_irq(pn544_dev);
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
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
