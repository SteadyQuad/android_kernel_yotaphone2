


#include <linux/init.h>
#include <linux/module.h>


#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/tcmd_driver.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/mfd/pm8xxx/ccadc.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/reboot.h>
#include <linux/i2c.h>
#include <linux/mhl_8334.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>

#define NAME "tcmd_driver"

static int gpio_map_size;
static struct gpio_mapping *gpio_map_table;

//extern int pm8921_disable_source_current_and_charging(bool disable);
#ifdef CONFIG_TCMD
extern int qpnp_disable_source_current(bool disable);

extern int qpnp_disable_usb_charging(bool disable);

extern int qpnp_set_max_battery_charge_current(bool enable);

extern int lm3559_flash_set_led_state(int flash_lm3559_state);

extern int qpnp_get_usb_max_current(int* usb_max_current);

extern int qpnp_get_bat_max_current(int* bat_max_current);

extern int qpnp_set_usb_max_current(int usb_max_current);

extern int qpnp_set_bat_max_current(int bat_max_current);

extern int qpnp_chg_priority(int enable);

extern struct qpnp_vadc_chip *vchip;

extern int get_slimport_chipid(void);
#endif


void __init gpio_mapping_init(struct gpio_mapping *table, int size)
{
	gpio_map_size = size;
	gpio_map_table = table;
}

int get_gpio_by_name(char *name)
{
	int i;

	for (i = 0; i < gpio_map_size; i++) {
		if (gpio_map_table[i].used == 0)
			continue;

		if (strncmp(name, gpio_map_table[i].name, GPIO_MAP_NAME_SIZE)
			== 0)
			return gpio_map_table[i].pin_num;
	}

	printk(KERN_ERR "Unable to get gpio pin num for %s\n", name);
	return -EINVAL;
}


static int tcmd_misc_open(struct inode *inode, struct file *file)
{
	int err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	return 0;
}

static long tcmd_misc_ioctl( struct file *file,
							unsigned int cmd, unsigned long arg)
{
#ifdef CONFIG_TCMD
	void __user *argp = (void __user *)arg;
	int gpio_enum = -1, irq = -1, gpio_state = -1, rc = 0;

	pr_info("tcmd_misc_ioctl is called");

	switch(cmd)
	{
		case TCMD_IOCTL_SET_REG:
		case TCMD_IOCTL_GET_REG:
			{
			tcmd_reg_arg reg_arg;
			struct regulator *reg_p;
			printk("TCMD_IOCTL_SET_REG or GET");
			if (copy_from_user(&reg_arg, argp, sizeof(tcmd_reg_arg)))
				return -EFAULT;

			reg_p = regulator_get(NULL,reg_arg.name );

			if (IS_ERR(reg_p))
			{
				printk("%s: VREG reg_p get failed\n", __func__);
				reg_p  = NULL;
				return -EFAULT;
			}


			if(cmd == TCMD_IOCTL_GET_REG ){
				printk("cmd == TCMD_IOCTL_GET_REG ");
				if(TCMD_REG_GET_VOLTAGE  == reg_arg.cmd)
				{	printk("cmd == TCMD_REG_GET_VOLTAGE  ");
					reg_arg.param = regulator_get_voltage(reg_p);
				}else if(TCMD_REG_GET_POWER_MODE  == reg_arg.cmd)
				{
					reg_arg.param = regulator_get_mode(reg_p);
				}
				if(copy_to_user((tcmd_reg_arg  *)argp, &reg_arg, sizeof(tcmd_reg_arg )))
				return -EFAULT;

			}
			else{
				switch(reg_arg.cmd)
				{
					case TCMD_REG_ENABLE :
						printk("TCMD_REGULATOR_ENABLE\n");
						rc = regulator_enable(reg_p);
						if (rc) {
							pr_info( " regulator_enable failed:%d\n", rc);
							return rc;
						}
						break;
					case     TCMD_REG_DISABLE:
						printk("TCMD_REGULATOR_DISABLE\n");
						rc = regulator_disable(reg_p);
						if (rc) {
							pr_info( " regulator_disable failed:%d\n", rc);
							return rc;
						}
						break;
					case    TCMD_REG_SET_VOLTAGE :
						pr_info("TCMD_REG_SET_VOLTAGE, reg_arg.param= %d\n",reg_arg.param);
						rc = regulator_set_voltage(reg_p, reg_arg.param, reg_arg.param);
						if (rc) {
							pr_info( " regulator_set_voltage failed:%d\n", rc);
							return rc;
						}
						break;
					case    TCMD_REG_SET_POWER_MODE :
						pr_info("TCMD_REG_SET_POWER_MODE, reg_arg.param= %d\n",reg_arg.param);
						rc = regulator_set_mode(reg_p, reg_arg.param);
						if (rc) {
							pr_info( " regulator_set_mode failed:%d\n", rc);
							return rc;
						}
						break;
					default:
						return -EINVAL;
					break;
				}

			}
		break;
		}
	case TCMD_IOCTL_CHARGER:
	{

		unsigned char charger_enable;
		pr_info("cmd == TCMD_IOCTL_CHARGER");

		if (copy_from_user(&charger_enable, argp, sizeof(unsigned char )))
			return -EFAULT;

		switch(charger_enable)
		{
			case 0:
				rc = qpnp_disable_source_current(true);
				if (rc)
				{
				 printk("TCMD :tcmd_driver : qpnp_disable_source_current failed:%d\n", rc);
				 return rc;
				}
				break;
			case 1:
				rc = qpnp_set_max_battery_charge_current(true);
				if (rc)
				{
				 pr_info("TCMD : tcmd_driver : qpnp_set_max_battery_charge_current(true) failed:%d\n", rc);
				 return rc;
				}
				rc = qpnp_disable_source_current(false);
				if (rc)
				{
				 pr_info("TCMD : tcmd_driver : qpnp_disable_source_current(false) failed:%d\n", rc);
				 return rc;
				}
				printk("TCMD : Enable charging with high current successful");
				break;
			case 2:
				rc = qpnp_set_max_battery_charge_current(false);
				if (rc)
				{
				printk("TCMD : tcmd_driver : qpnp_set_max_battery_charge_current(false) failed:%d\n", rc);
				 return rc;
				}
				rc = qpnp_disable_source_current(false);
				if (rc)
				{
				 pr_info("TCMD : tcmd_driver : qpnp_disable_source_current(false) failed:%d\n", rc);
				 return rc;
				}
				printk("TCMD : Enable charging with low current successful");
				break;
				break;
			default:
				return -EINVAL;
				break;
		}
		break;
	}
	case TCMD_IOCTL_USB_CHRGING:
				{
				unsigned char usb_charger_enable;
				pr_info("TCMD: cmd = TCMD_IOCTL_USB_CHRGING");
				if(copy_from_user(&usb_charger_enable, argp, sizeof(unsigned char)))
				return -EFAULT;

				switch(usb_charger_enable)
					{
					case 0:
						printk("TCMD: tcmd_driver : Disable USB charging\n");
						rc = qpnp_disable_usb_charging(!usb_charger_enable);
						if(rc) {
							printk("TCMD: qpnp-charger disable usb charging failed:%d\n", rc);
							return rc;
							}
							break;
					case 1:
						printk("TCMD: tcmd_driver : Enable USB charging\n");
						rc = qpnp_disable_usb_charging(!usb_charger_enable);
						if(rc) {
							printk("TCMD: qpnp-charger enable usb charging failed:%d\n", rc);
							return rc;
							}
						}
				break;
				}
       case TCMD_IOCTL_CHGPTH_PRIORITY:
                               {
                               unsigned char charging_path_priority;
                               pr_info("TCMD: cmd = TCMD_IOCTL_CHGPTH_PRIORITY\n");
                               if(copy_from_user(&charging_path_priority, argp, sizeof(unsigned char)))
                               return -EFAULT;

                               switch(charging_path_priority)
                                       {
                                       case 0:
                                               printk("TCMD: tcmd_driver : USB_IN priority \n");
                                               rc = qpnp_chg_priority(charging_path_priority);
                                               if(rc) {
                                                       printk("TCMD: qpnp_chg_priority USB_IN failed:%d\n", rc);
                                                       return rc;
                                                       }
                                                       break;
                                       case 1:
                                               printk("TCMD: tcmd_driver : DC_IN priority \n");
                                               rc = qpnp_chg_priority(charging_path_priority);
                                               if(rc) {
                                                       printk("TCMD: qpnp_chg_priority DC_IN failed:%d\n", rc);
                                                       return rc;
                                                       }
                                        }
                               break;
                               }
	/*case TCMD_IOCTL_SET_FLASH_LM3559:
	{
		int flash_lm3559_state;
		pr_info("TCMD_IOCTL_SET_FLASH_LM3559\n");

		if(copy_from_user(&flash_lm3559_state, argp, sizeof(int)))
			return -EFAULT;

		rc = lm3559_flash_set_led_state(flash_lm3559_state);

		if(rc){
			pr_info("Error on calling set_flash_lm3559\n");
			return rc;
		}
		break;
	}*/
	case TCMD_IOCTL_POWER_DOWN:
	{
		pr_info("TCMD_IOCTL_POWER_DOWN\n");
		kernel_power_off();
		break;
	}
	case TCMD_IOCTL_GET_ADC:
	{
		TCMD_QPNP_VADC_CHAN_PARAM  adc_arg;
                //struct qpnp_vadc_chip *vadc;
		int ibat_ua;

		printk("TCMD_IOCTL_GET_ADC");
		if (copy_from_user(&adc_arg, argp, sizeof(TCMD_QPNP_VADC_CHAN_PARAM)))
			return -EFAULT;
		printk("adc_arg.cmd: %d",adc_arg.cmd);
		if(!vchip)
		{
		printk("vchip is NULL\n");
		}
		if (adc_arg.cmd == TCMD_PMIC_ADC_BATTERY_CURRENT){
			printk("adc_arg.cmd == TCMD_PMIC_ADC_BATTERY_CURRENT");
			rc = tcmd_get_battery_current(&ibat_ua);
			adc_arg.adc_config.physical = ibat_ua;
			adc_arg.adc_config.measurement = adc_arg.adc_config.physical;
		}else{
			rc = qpnp_vadc_read(vchip, adc_arg.cmd, &adc_arg.adc_config);
		}

		if (rc) {
			pr_info("Error on calling qpnp_vadc_read");
			return rc;
		}

		if(copy_to_user((TCMD_QPNP_VADC_CHAN_PARAM *)argp, &adc_arg, sizeof(TCMD_QPNP_VADC_CHAN_PARAM)))
				return -EFAULT;

		break;
	}
	case TCMD_IOCTL_SET_GPIO:
	{
		TCMD_PM8XXX_GPIO_PARAM gpio_arg;

		printk("TCMD_IOCTL_SET_GPIO");
		if (copy_from_user(&gpio_arg, argp, sizeof(TCMD_PM8XXX_GPIO_PARAM)))
			return -EFAULT;
		printk("gpio_arg.cmd: %d",  gpio_arg.cmd);

		rc = pm8xxx_gpio_config(gpio_arg.cmd, &gpio_arg.gpio_config);

		if (rc) {
			pr_info("Error on calling pm8xxx_gpio_config");
			return rc;
		}

		break;
	}
	case TCMD_IOCTL_GET_USB_BAT_CURRENT:
	{
		int rc = -1;
		TCMD_REG_USB_BAT_CURRENT adc_arg;
		printk("TCMD_IOCTL_GET_USB_BAT_CURRENT");
		if (copy_from_user(&adc_arg, argp, sizeof(TCMD_REG_USB_BAT_CURRENT)))
		{
			return -EFAULT;
		}

		qpnp_get_usb_max_current(&adc_arg.usbPresentCurrent);
		rc = qpnp_get_bat_max_current(&adc_arg.batPresentCurrent);

		if(rc < 0)
		{
			return rc;
		}

		if(copy_to_user((TCMD_REG_USB_BAT_CURRENT *)argp, &adc_arg, sizeof(TCMD_REG_USB_BAT_CURRENT)))
				return -EFAULT;

		return rc;
	}
	case TCMD_IOCTL_SET_USB_BAT_CURRENT:
	{
		int rc = -1;
                TCMD_REG_USB_BAT_CURRENT adc_arg;
                printk("TCMD_IOCTL_GET_USB_BAT_CURRENT");
                if (copy_from_user(&adc_arg, argp, sizeof(TCMD_REG_USB_BAT_CURRENT)))
                {
                        return -EFAULT;
                }

		rc = qpnp_set_usb_max_current(adc_arg.usbPresentCurrent);
		if(rc < 0)
		{
			return rc;
		}

		rc = qpnp_set_bat_max_current(adc_arg.batPresentCurrent);
		return rc;
	}
	case TCMD_IOCTL_COINCELL_ENABLE_DISABLE:
	{
		int coincell_state;
		pr_info("TCMD_IOCTL_COINCELL_ENABLE_DISABLE\n");
		if(copy_from_user(&coincell_state, argp, sizeof(int)))
                        return -EFAULT;
		printk("TCMD:Set charger state entering with value : %d",coincell_state);
		rc = tcmd_qpnp_coincell_set_charger(coincell_state);
		if(rc){
                        pr_info("Error on calling  tcmd_qpnp_coincell_set_charger\n");
			printk("TCMD:Error on calling tcms_qpnp_coincell_set_charger : %d",rc);
                        return rc;
                }
		printk("TCMD:Set charger state out with value : %d",rc);
		break;
	}
	case TCMD_IOCTL_SET_COINCELL:
	{
		TCMD_QPNP_CC_CHG_PARAM coincell_arg;
		printk("TCMD_IOCTL_SET_COINCELL");
		if (copy_from_user(&coincell_arg, argp, sizeof(TCMD_QPNP_CC_CHG_PARAM)))
			return -EFAULT;

		//rc = pm8xxx_coincell_chg_config(&coincell_arg);
		printk("\nState - > %d \n Voltage - > %d \n Resistance -> %d \n",coincell_arg.state,coincell_arg.voltage,coincell_arg.resistance);
		if((rc = tcmd_qpnp_coincell_set_charger(coincell_arg.state))){
			printk("Error in configuring the state : tcmd_qpnp_coincell_set_charger : rc= %d",rc);
			return rc;
		}
		if((rc =  tcmd_qpnp_coincell_set_voltage(coincell_arg.voltage))){
			printk("Error in configuring Voltage :tcmd_qpnp_coincell_set_voltage: rc= %d",rc);
                        return rc;
		}
		if((rc = tcmd_qpnp_coincell_set_resistance(coincell_arg.resistance)))	{
			printk("Error in configuring Resistance :tcmd_qpnp_coincell_set_resistance : rc= %d",rc);
                        return rc;
		}

		if (rc) {
			pr_info("Error on calling pm8xxx_coincell_chg_config ");
			printk("value of rc = %d",rc);
			return rc;
		}
		break;
	}
	case TCMD_IOCTL_GET_SLIMPORT_CHIPID:
	{
		TCMD_SLIMPORT_PARAM slimport;
		slimport.chip_id = get_slimport_chipid();
		if(slimport.chip_id == -1)
			return -ENODEV;
		if(copy_to_user((TCMD_SLIMPORT_PARAM*)argp, &slimport, sizeof(TCMD_SLIMPORT_PARAM)))
			return -EFAULT;
		break;
	}
/*	case TCMD_IOCTL_SET_VIBRATOR:
	{
		TCMD_PM8XXX_VIB_PARAM vib_arg;

		printk("TCMD_IOCTL_SET_VIBRATOR");
		if (copy_from_user(&vib_arg, argp, sizeof(TCMD_PM8XXX_VIB_PARAM)))
			return -EFAULT;

		rc = pm8xxx_vibrator_config(&vib_arg);

		if (rc) {
		pr_info("Error on calling pm8xxx_vibrator_config");
		return rc;
		}

		break;

	}*/

	default:
	{
			struct tcmd_gpio_set_arg gpio_set_arg;

			if (cmd == TCMD_IOCTL_SET_INT) {
			if (copy_from_user(&gpio_set_arg, argp, 2))
				return -EFAULT;
			gpio_enum = gpio_set_arg.gpio;
			gpio_state = gpio_set_arg.gpio_state;
			}
		else if (copy_from_user(&gpio_enum, argp, sizeof(int)))
			return -EFAULT;

	if (gpio_enum < 0)
		return -EINVAL;

		switch (cmd) {
			case TCMD_IOCTL_MASK_INT:
			case TCMD_IOCTL_UNMASK_INT:
		irq = gpio_to_irq(gpio_enum);
				if (irq < 0)
					return -EINVAL;
				break;
			default:
				break;
			}

		switch (cmd) {
		case TCMD_IOCTL_MASK_INT:
			pr_info("tcmd mask interrupt: gpio = %d, irq = %d.\n",
						gpio_enum, irq);
			disable_irq(irq);
			break;
		case TCMD_IOCTL_UNMASK_INT:
			pr_info("tcmd unmask interrupt: gpio = %d, irq = %d.\n",
						gpio_enum, irq);
			enable_irq(irq);
			break;
		case TCMD_IOCTL_READ_INT:
		gpio_state = gpio_get_value(gpio_enum);
			pr_info("tcmd interrupt state: gpio = %d -> %d.\n",
						gpio_enum, gpio_state);
			if (copy_to_user(argp, &gpio_state, sizeof(int)))
				return -EFAULT;
			break;
		case TCMD_IOCTL_SET_INT:
			pr_info("tcmd set interrupt state: gpio = %d -> %d.\n",
						gpio_enum, gpio_state);
		gpio_set_value(gpio_enum, gpio_state);
		break;
		default:
			return -EINVAL;
		}
		break;
	}//default
	}
#endif
	return 0;
}

static const struct file_operations tcmd_misc_fops = {
	.owner = THIS_MODULE,
	.open = tcmd_misc_open,
	.unlocked_ioctl = tcmd_misc_ioctl,
};

static struct miscdevice tcmd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &tcmd_misc_fops,
};

static int __init tcmd_init(void)
{
	int error = misc_register(&tcmd_misc_device);
	if (error < 0) {
		pr_err("%s: tcmd misc register failed!\n", __func__);
		return error;
	}

	pr_info("tcmd probe\n");

	return 0;
}

static void __exit tcmd_exit(void)
{
	misc_deregister(&tcmd_misc_device);
}

module_init(tcmd_init);
module_exit(tcmd_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("tcmd");
