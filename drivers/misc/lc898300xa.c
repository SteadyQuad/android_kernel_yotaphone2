#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/i2c/isa1200.h>
#include "../staging/android/timed_output.h"
#include <linux/of_gpio.h>
#include <linux/vibrator-lc898300.h>

#define LC898300_REG_HBPW	0x01
#define LC898300_REG_RESOFRQ	0x02
#define LC898300_REG_STARTUP	0x03
#define LC898300_REG_BRAKE	0x04
#define LC898300_REG_STOPS	0x05

#define MIN_ON			5    //(200Hz -> 5ms)

struct lc898300xa_regulator {
        const char *name;
        u32     min_uV;
        u32     max_uV;
        u32     load_uA;
};

struct lc898300xa_platform_data {
        const char *name;
        unsigned int max_timeout;
        unsigned int hap_en_gpio;
       //unsigned int hap_len_gpio;
        unsigned int hap_standby_gpio;
        enum mode_control mode_ctrl; /* input/generation/wave */
        unsigned int chip_en;
        unsigned int duty;
        struct lc898300xa_regulator *regulator_info;
        u8 num_regulators;
        int (*power_on)(int on);
        int (*dev_setup)(bool on);
        int (*clk_enable)(bool on);
};

struct lc898300xa_chip {
	struct i2c_client *client;
	struct lc898300xa_platform_data *pdata;
	struct hrtimer timer;
	struct timed_output_dev dev;
	struct mutex lock;
	unsigned int period_ns;
	bool is_len_gpio_valid;
	struct regulator **regs;
	bool clk_on;
	unsigned int vibration_voltage;
};

#if 0 /*might be useful for debugging hence not removing */
static int lc898300xa_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
#endif
static int lc898300xa_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;
	unsigned retry_count = 3;

	do {
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d, retry count(%u)\n",
				__func__, ret, retry_count);
			udelay(200);
		}
		else
			break;
	} while (--retry_count);

	return ret;
}

static void lc898300xa_vib_set(struct lc898300xa_chip *haptic, int enable)
{
	if (enable) {
		/* if hen and len are seperate then enable hen
		 * otherwise set normal mode bit */
		if (haptic->is_len_gpio_valid == true)
			gpio_set_value(haptic->pdata->hap_en_gpio, 1);

	} else {
		/* if hen and len are seperate then pull down hen
		 * otherwise set power down bit */
		if (haptic->is_len_gpio_valid == true)
			gpio_set_value(haptic->pdata->hap_en_gpio, 0);
	}
	return;
}

static void lc898300xa_chip_enable(struct timed_output_dev *dev, int value)
{
	struct lc898300xa_chip *haptic = container_of(dev, struct lc898300xa_chip,
					dev);
	static int last_en_val = 0;

	mutex_lock(&haptic->lock);
	/* do not allow disable vibration during minimal enable time */
	if ((value == 0) && (last_en_val <= MIN_ON)) {
		mutex_unlock(&haptic->lock);
		return;
	}

	hrtimer_cancel(&haptic->timer);
	if (value != 0) {
		value = (value < MIN_ON) ? MIN_ON : value;
		value = (value > haptic->pdata->max_timeout ?
				haptic->pdata->max_timeout : value);
		last_en_val = value;
		lc898300xa_vib_set(haptic, 1);
		hrtimer_start(&haptic->timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	} else {
		lc898300xa_vib_set(haptic, 0);
	}

	mutex_unlock(&haptic->lock);
}

static int lc898300xa_chip_get_time(struct timed_output_dev *dev)
{
	struct lc898300xa_chip *haptic = container_of(dev, struct lc898300xa_chip,
					dev);

	if (hrtimer_active(&haptic->timer)) {
		ktime_t r = hrtimer_get_remaining(&haptic->timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	} else
		return 0;
}


static enum hrtimer_restart lc898300xa_vib_timer_func(struct hrtimer *timer)
{
	struct lc898300xa_chip *haptic = container_of(timer, struct lc898300xa_chip,
					timer);
	lc898300xa_vib_set(haptic, 0);

	return HRTIMER_NORESTART;
}

static int lc898300xa_reg_power(struct lc898300xa_chip *haptic, bool on)
{
	const struct lc898300xa_regulator *reg_info =
				haptic->pdata->regulator_info;
	u8 i, num_reg = haptic->pdata->num_regulators;
	int rc;
	for (i = 0; i < num_reg; i++) {
		rc = regulator_set_optimum_mode(haptic->regs[i],
					on ? reg_info[i].load_uA : 0);
		if (rc < 0) {
			pr_err("%s: regulator_set_optimum_mode failed(%d)\n",
							__func__, rc);
			goto regs_fail;
		}

		rc = on ? regulator_enable(haptic->regs[i]) :
			regulator_disable(haptic->regs[i]);
		if (rc < 0) {
			pr_err("%s: regulator %sable fail %d\n", __func__,
					on ? "en" : "dis", rc);
			regulator_set_optimum_mode(haptic->regs[i],
					!on ? reg_info[i].load_uA : 0);
			goto regs_fail;
		}
	}

	return 0;

regs_fail:
	while (i--) {
		regulator_set_optimum_mode(haptic->regs[i],
				!on ? reg_info[i].load_uA : 0);
		!on ? regulator_enable(haptic->regs[i]) :
			regulator_disable(haptic->regs[i]);
	}
	return rc;
}

static int lc898300xa_reg_setup(struct lc898300xa_chip *haptic, bool on)
{
	const struct lc898300xa_regulator *reg_info =
				haptic->pdata->regulator_info;
	u8 i, num_reg = haptic->pdata->num_regulators;
	int rc = 0;
	/* put regulators */
	if (on == false) {
		i = num_reg;
		goto put_regs;
	}

	haptic->regs = kzalloc(num_reg * sizeof(struct regulator *),
							GFP_KERNEL);
	if (!haptic->regs) {
		pr_err("unable to allocate memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < num_reg; i++) {
		haptic->regs[i] = regulator_get(&haptic->client->dev,
						reg_info[i].name);
		if (IS_ERR(haptic->regs[i])) {
			rc = PTR_ERR(haptic->regs[i]);
			pr_err("%s:regulator get failed(%d)\n", __func__, rc);
			goto put_regs;
		}

		if (regulator_count_voltages(haptic->regs[i]) > 0) {
			rc = regulator_set_voltage(haptic->regs[i],
				reg_info[i].min_uV, reg_info[i].max_uV);
			if (rc) {
				pr_err("%s: regulator_set_voltage failed(%d)\n",
								__func__, rc);
				regulator_put(haptic->regs[i]);
				goto put_regs;
			}
		}
	}

	return rc;

put_regs:
	while (i--) {
		if (regulator_count_voltages(haptic->regs[i]) > 0)
			regulator_set_voltage(haptic->regs[i], 0,
						reg_info[i].max_uV);
		regulator_put(haptic->regs[i]);
	}
	kfree(haptic->regs);
	return rc;
}

#ifdef CONFIG_OF
static int lc898300xa_parse_dt(struct device *dev,
			struct lc898300xa_platform_data *pdata)
{
	struct device_node *temp, *np = dev->of_node;
	struct lc898300xa_regulator *reg_info;
	enum of_gpio_flags hap_en_flags = OF_GPIO_ACTIVE_LOW;
	int rc = 0;
	u32 temp_val;
	const char *temp_string;

	rc = of_property_read_string(np, "label", &pdata->name);
	if (rc) {
		dev_err(dev, "Unable to read device name\n");
		return rc;
	}

	pdata->chip_en = of_property_read_bool(np, "sanyo,chip-en");

	pdata->hap_en_gpio = of_get_named_gpio_flags(np,
				"sanyo,hap-en-gpio", 0, &hap_en_flags);
	pdata->hap_standby_gpio = of_get_named_gpio_flags(np,
				"sanyo,hap-standby-gpio", 0, &hap_en_flags);
//	pdata->hap_len_gpio = of_get_named_gpio_flags(np,
//				"imagis,hap-len-gpio", 0, &hap_len_flags);

	rc = of_property_read_u32(np, "sanyo,max-timeout",
				&pdata->max_timeout);
	if (rc) {
		dev_err(dev, "Unable to read max timeout\n");
		return rc;
	}

	pdata->num_regulators = 0;
	temp = NULL;
	while ((temp = of_get_next_child(np, temp)))
		pdata->num_regulators++;

	if (!pdata->num_regulators)
		return 0;

	reg_info = devm_kzalloc(dev, pdata->num_regulators *
				sizeof(struct lc898300xa_regulator), GFP_KERNEL);
	if (!reg_info)
		return -ENOMEM;

	pdata->regulator_info = reg_info;

	for_each_child_of_node(np, temp) {
		rc = of_property_read_string(temp,
			"regulator-name", &temp_string);
		if (rc) {
			dev_err(dev, "Unable to read regulator name\n");
			return rc;
		} else
			reg_info->name = temp_string;

		rc = of_property_read_u32(temp, "regulator-max-microvolt",
			&temp_val);
		if (rc) {
			dev_err(dev, "Unable to read max uV\n");
			return rc;
		} else
			reg_info->max_uV = temp_val;

		rc = of_property_read_u32(temp, "regulator-min-microvolt",
			&temp_val);
		if (rc) {
			dev_err(dev, "Unable to read min uV\n");
			return rc;
		} else
			reg_info->min_uV = temp_val;

		rc = of_property_read_u32(temp, "regulator-max-microamp",
			&temp_val);
		if (rc) {
			dev_err(dev, "Unable to read load uA\n");
			return rc;
		} else
			reg_info->load_uA = temp_val;

		reg_info++;
	}

	return 0;
}
#else
static int lc898300xa_parse_dt(struct device *dev,
		struct lc898300xa_platform_data *pdata)
{
	return -ENODEV;
}
#endif


static ssize_t attr_intensity_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct lc898300xa_chip *haptic;

	haptic = container_of(tdev, struct lc898300xa_chip, dev);
	return sprintf(buf, "%u\n", haptic->vibration_voltage);
}

ssize_t attr_intensity_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned int level;
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct lc898300xa_chip *haptic;
	haptic = container_of(tdev, struct lc898300xa_chip, dev);

	if (kstrtouint(buf, 0, &level)) {
		dev_err(dev, "%s: error while storing new intensity\n",
			__func__);
		return -EINVAL;
	}

	/* make sure new intensity is in range */
	if(level > VIB_CMD_PWM_15_15)
		level = VIB_CMD_PWM_15_15;

	haptic->vibration_voltage = level;
	lc898300xa_write_reg(haptic->client, LC898300_REG_HBPW, level);

	dev_info(dev, "%s: new intensity: %u\n", __func__, level);

	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(intensity, S_IRUGO | S_IWUSR,
		attr_intensity_show, attr_intensity_store),
};

static int lc898300_create_sysfs_interfaces(struct device *dev)
{
	int i;
	int result;

	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		result = device_create_file(dev, &attributes[i]);
		if (result) {
			for (; i >= 0; i--)
				device_remove_file(dev, &attributes[i]);
			dev_err(dev, "%s: Failed to create sysfs interfaces\n",
				__func__);
			return result;
		}
	}

	return result;
}

static int __devinit lc898300xa_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lc898300xa_chip *haptic;
	struct lc898300xa_platform_data *pdata;
	int ret;
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "%s: no support for i2c read/write"
				"byte data\n", __func__);
		return -EIO;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct lc898300xa_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		ret = lc898300xa_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "Parsing DT failed(%d)", ret);
			return ret;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	if (pdata->dev_setup) {
		ret = pdata->dev_setup(true);
		if (ret < 0) {
			dev_err(&client->dev, "dev setup failed\n");
			return -EINVAL;
		}
	}

	haptic = kzalloc(sizeof(struct lc898300xa_chip), GFP_KERNEL);
	if (!haptic) {
		ret = -ENOMEM;
		goto mem_alloc_fail;
	}
	haptic->client = client;
	haptic->pdata = pdata;

	if (pdata->regulator_info) {
		ret = lc898300xa_reg_setup(haptic, true);
		if (ret) {
			dev_err(&client->dev, "%s: regulator setup failed\n",
							__func__);
			goto reg_setup_fail;
		}

		ret = lc898300xa_reg_power(haptic, true);
		if (ret) {
			dev_err(&client->dev, "%s: regulator power failed\n",
							__func__);
			goto reg_pwr_fail;
		}
	}

	if (pdata->power_on) {
		ret = pdata->power_on(1);
		if (ret) {
			dev_err(&client->dev, "%s: power-up failed\n",
							__func__);
			goto pwr_up_fail;
		}
	}

	mutex_init(&haptic->lock);
	haptic->clk_on = false;

	hrtimer_init(&haptic->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	haptic->timer.function = lc898300xa_vib_timer_func;

	/*register with timed output class*/
	haptic->dev.name = pdata->name;
	haptic->dev.get_time = lc898300xa_chip_get_time;
	haptic->dev.enable = lc898300xa_chip_enable;
	ret = timed_output_dev_register(&haptic->dev);
	if (ret < 0)
		goto timed_reg_fail;


	i2c_set_clientdata(client, haptic);

	ret = gpio_is_valid(pdata->hap_en_gpio);
	if (ret) {
		ret = gpio_request(pdata->hap_en_gpio, "haptic_en_gpio");
		if (ret) {
			dev_err(&client->dev, "%s: gpio %d request failed\n",
					__func__, pdata->hap_en_gpio);
		//	goto hen_gpio_fail;
			goto err0;
		}
	} else {
		dev_err(&client->dev, "%s: Invalid gpio %d\n", __func__,
					pdata->hap_en_gpio);
		//goto hen_gpio_fail;
	}


	ret = gpio_is_valid(pdata->hap_standby_gpio);
	if (ret) {
		ret = gpio_request(pdata->hap_standby_gpio, "hap_standby_gpio");
		if (ret) {
			dev_err(&client->dev, "%s: gpio %d request failed\n",
					__func__, pdata->hap_standby_gpio);
		//	goto hen_gpio_fail;
			goto err1;
		}
	} else {
		dev_err(&client->dev, "%s: Invalid gpio %d\n", __func__,
					pdata->hap_standby_gpio);
	//	goto hen_gpio_fail;
	}

	gpio_direction_output(pdata->hap_standby_gpio,0);
	gpio_direction_output(pdata->hap_en_gpio,0);
	udelay(200);
	gpio_set_value(pdata->hap_standby_gpio, 1);
	udelay(250);
	haptic->vibration_voltage = VIB_CMD_PWM_10_15;
	lc898300xa_write_reg(client, LC898300_REG_HBPW, VIB_CMD_PWM_10_15);
	lc898300xa_write_reg(client, LC898300_REG_RESOFRQ, VIB_CMD_FREQ_200);
	lc898300xa_write_reg(client, LC898300_REG_STARTUP, VIB_CMD_STTIME_1);

	haptic->is_len_gpio_valid = true;

	if (lc898300_create_sysfs_interfaces(haptic->dev.dev))
		dev_err(&client->dev, "%s: create sysfs failed", __func__);

	return 0;

#if 0

setup_fail:
	if (haptic->is_len_gpio_valid == true)
		gpio_free(pdata->hap_len_gpio);

len_gpio_fail:
	gpio_free(pdata->hap_en_gpio);
	gpio_free(pdata->hap_standby_gpio);

hen_gpio_fail:
	timed_output_dev_unregister(&haptic->dev);
#endif

timed_reg_fail:
	mutex_destroy(&haptic->lock);
	if (pdata->power_on)
		pdata->power_on(0);

err0:
	gpio_free(pdata->hap_en_gpio);

err1:
	gpio_free(pdata->hap_standby_gpio);

pwr_up_fail:
	if (pdata->regulator_info)
		lc898300xa_reg_power(haptic, false);
reg_pwr_fail:
	if (pdata->regulator_info)
		lc898300xa_reg_setup(haptic, false);
reg_setup_fail:
	kfree(haptic);
mem_alloc_fail:
	if (pdata->dev_setup)
		pdata->dev_setup(false);
	return ret;
}

static int __devexit lc898300xa_remove(struct i2c_client *client)
{
	struct lc898300xa_chip *haptic = i2c_get_clientdata(client);

	hrtimer_cancel(&haptic->timer);
	timed_output_dev_unregister(&haptic->dev);

	gpio_set_value_cansleep(haptic->pdata->hap_en_gpio, 0);
	gpio_set_value_cansleep(haptic->pdata->hap_standby_gpio, 0);
	gpio_free(haptic->pdata->hap_en_gpio);
	gpio_free(haptic->pdata->hap_standby_gpio);

	/* destroy mutex */
	mutex_destroy(&haptic->lock);

	/* power-off the chip */
	if (haptic->pdata->regulator_info) {
		lc898300xa_reg_power(haptic, false);
		lc898300xa_reg_setup(haptic, false);
	}

	if (haptic->pdata->power_on)
		haptic->pdata->power_on(0);

	if (haptic->pdata->dev_setup)
		haptic->pdata->dev_setup(false);

	kfree(haptic);
	return 0;
}

#ifdef CONFIG_PM
static int lc898300xa_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lc898300xa_chip *haptic = i2c_get_clientdata(client);
	int ret;

	hrtimer_cancel(&haptic->timer);

	gpio_set_value_cansleep(haptic->pdata->hap_en_gpio, 0);
	gpio_set_value_cansleep(haptic->pdata->hap_standby_gpio, 0);

	if (haptic->pdata->regulator_info)
		lc898300xa_reg_power(haptic, false);

	if (haptic->pdata->power_on) {
		ret = haptic->pdata->power_on(0);
		if (ret) {
			dev_err(&client->dev, "power-down failed\n");
			return ret;
		}
	}

	return 0;
}

static int lc898300xa_resume(struct i2c_client *client)
{
	struct lc898300xa_chip *haptic = i2c_get_clientdata(client);
	int ret;

	gpio_set_value_cansleep(haptic->pdata->hap_standby_gpio, 1);
	udelay(200);

	if (haptic->pdata->regulator_info)
		lc898300xa_reg_power(haptic, true);

	if (haptic->pdata->power_on) {
		ret = haptic->pdata->power_on(1);
		if (ret) {
			dev_err(&client->dev, "power-up failed\n");
			return ret;
		}
	}
	lc898300xa_write_reg(haptic->client ,LC898300_REG_HBPW,
			     haptic->vibration_voltage);
	lc898300xa_write_reg(haptic->client, LC898300_REG_RESOFRQ,
			     VIB_CMD_FREQ_200);
	lc898300xa_write_reg(haptic->client, LC898300_REG_STARTUP,
			     VIB_CMD_STTIME_1);

	return 0;
}
#else
#define lc898300xa_suspend		NULL
#define lc898300xa_resume		NULL
#endif

static const struct i2c_device_id lc898300xa_id[] = {
	{ "lc898300xa_1", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, lc898300xa_id);
#ifdef CONFIG_OF
static struct of_device_id lc898300xa_match_table[] = {
	{ .compatible = "sanyo,lc898300xa",},
	{ },
};
#else
#define lc898300xa_match_table NULL
#endif

static struct i2c_driver lc898300xa_driver = {
	.driver	= {
		.name	= "lc898300xa",
		.of_match_table = lc898300xa_match_table,
	},
	.probe		= lc898300xa_probe,
	.remove		= __devexit_p(lc898300xa_remove),
	.suspend	= lc898300xa_suspend,
	.resume		= lc898300xa_resume,
	.id_table	= lc898300xa_id,
};

static int __init lc898300xa_init(void)
{
	return i2c_add_driver(&lc898300xa_driver);
}

static void __exit lc898300xa_exit(void)
{
	i2c_del_driver(&lc898300xa_driver);
}

module_init(lc898300xa_init);
module_exit(lc898300xa_exit);
