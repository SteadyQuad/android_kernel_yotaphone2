/*-----------------------------------------------------------------------------

 linux/drivers/video/eink13522/s1d13522spi.c
 interface code for Epson S1D13522 LCD controllerframe buffer driver.

 Code based on E-Ink demo source code.

 Copyright(c) Seiko Epson Corporation 2009.
 All rights reserved.

 This file is subject to the terms and conditions of the GNU General Public
 License. See the file COPYING in the main directory of this archive for
 more details.

----------------------------------------------------------------------------*/
#ifdef CONFIG_FB_EPSON_SPI
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/firmware.h>
#include "s1d13522fb.h"

#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#define PMIC_READ 0xD1
#define PMIC_WRITE 0xD0

#define SPI_PRODUCT_NAME	"s1d13522"
#define SPI_PROD_ID	0x00

#define GPIO_SET(gpio, value)		gpio_set_value(gpio, value)
#define GPIO_GET(gpio)				gpio_get_value(gpio)

#define ERRORVAL	1

int systemSleep(void);
int systemRun(void);

int eink_spi_write(struct spi_device *spi, void *buf, size_t len);
int eink_spi_write_dma(struct spi_device *spi, void *buf, size_t len);
#define SPI_WRITE(spidev, buf, len)      eink_spi_write(spidev, buf, len)
#define SPI_WRITE_DMA(spidev, buf, len)  eink_spi_write_dma(spidev, buf, len)
#define SPI_READ(spidev, buf, len)	spi_read(spidev, buf, len)

#define GPIO_HRDY_DESC		"eink3522 HRDY Line"
#define GPIO_HDC_DESC		"eink3522 HDC Line"
#define GPIO_HIRQ_DESC		"eink3522 HIRQ Line"
#define GPIO_RESET_L_DESC	"eink3522 RESET_L Line"

#define DMA_MASK	DMA_BIT_MASK(32)
int GPIO_HDC;
int epdCheck;

static int set_cmd_mode(void);
static void command(u16 v);
static void data(u16 v);
static u16  data_get(void);
struct device *devlog;

#ifdef CONFIG_FB_EPSON_HRDY_OK
static int  wait_for_ready(void);
#endif

extern int wakeup_count;
extern struct fb_info *info;
static struct mutex ghwMutex;
static spinlock_t gHrdyIrqLock;
static struct completion gHrdyCompletion;


static const struct spi_device_id eink3522_spi_idtable[] = {
	{SPI_PRODUCT_NAME, SPI_PROD_ID},
	{}
};

MODULE_DEVICE_TABLE(spi, eink3522_spi_idtable);

struct spi_device *s1d13522_spidev;
int resetL_gpio;
int power_enable_gpio;
int hv_power_enable_gpio;
int pmic_enable_gpio;

int initGPIO(void)
{
	int hrdy_gpio;
	int hdc_gpio;
	int hirq_gpio;
	int rc;
	power_enable_gpio = of_get_named_gpio(s1d13522_spidev->dev.of_node,
			"qcom,power-enable", 0);
	hv_power_enable_gpio = of_get_named_gpio(s1d13522_spidev->dev.of_node,
			"qcom,hv-power-enable", 0);
	pmic_enable_gpio = of_get_named_gpio(s1d13522_spidev->dev.of_node,
			"qcom,pmic-enable", 0);

	rc = gpio_request(power_enable_gpio, "epd_pwr_en");
	if (rc) {
		pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"epd pwr en", power_enable_gpio, rc);
	}
	gpio_direction_output(power_enable_gpio, 0);

	rc = gpio_request(hv_power_enable_gpio, "hv_power_enable_gpio");
	if (rc) {
		pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"hv_power_enable_gpio", hv_power_enable_gpio, rc);
	}
	gpio_direction_output(hv_power_enable_gpio, 0);

	rc = gpio_request(pmic_enable_gpio, "pmic_enable_gpio");
	if (rc) {
		pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"pmic_enable_gpio", pmic_enable_gpio, rc);
	}
	gpio_direction_output(pmic_enable_gpio, 0);

	/* MSM GPIO Configuration */
	resetL_gpio = of_get_named_gpio(s1d13522_spidev->dev.of_node,
			"qcom,resetL-gpio", 0);
	rc = gpio_request(resetL_gpio, "resetL_gpio");
	if (rc) {
		pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"resetL_gpio", resetL_gpio, rc);
	}

	hrdy_gpio = of_get_named_gpio(s1d13522_spidev->dev.of_node,
			"qcom,hrdy", 0);
	rc = gpio_request(hrdy_gpio, "epd_hrdy_gpio");
	if (rc) {
		pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"hrdy_gpio", hrdy_gpio, rc);
	}

	hdc_gpio = of_get_named_gpio(s1d13522_spidev->dev.of_node,
			"qcom,hdc", 0);
	rc = gpio_request(hdc_gpio, "epd_hdc_gpio");
	if (rc) {
		pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"hdc_gpio", hdc_gpio, rc);
	}
	GPIO_HDC = hdc_gpio;

	hirq_gpio = of_get_named_gpio(s1d13522_spidev->dev.of_node,
			"qcom,hirq", 0);
	rc = gpio_request(hirq_gpio, "epd_hirq_gpio");
	if (rc) {
		pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"hirq_gpio", hirq_gpio, rc);
	}
	return 1;
}

int poweron(int on)
{
	gpio_set_value_cansleep(resetL_gpio, 0);
	if (on) {
		gpio_set_value_cansleep(pmic_enable_gpio, 0);
		gpio_set_value_cansleep(hv_power_enable_gpio, 0);
		gpio_set_value_cansleep(power_enable_gpio, 0);
		msleep(4);
		gpio_set_value_cansleep(resetL_gpio, 1);
		printk(KERN_ERR "%s(on): success\n", __func__);
	} else {
		gpio_set_value_cansleep(power_enable_gpio, 1);
		gpio_set_value_cansleep(hv_power_enable_gpio, 1);
		gpio_set_value_cansleep(pmic_enable_gpio, 1);
		msleep(4);
		gpio_set_value_cansleep(resetL_gpio, 1);
		printk(KERN_ERR "%s(off): success\n", __func__);
	}
	return 0;
}

static int __devinit eink3522_spi_probe(struct spi_device *spidev)
{
	int retVal = 0;
	spidev->bits_per_word = 2*8;
	retVal = spi_setup(spidev);
	if (retVal < 0) {
		printk("eink3522_spi_probe Failed.\n");
		return -ENODEV;
	}
	s1d13522_spidev = spidev;
	initGPIO();
	poweron(1);
	s1d13522fb_init();
	return 0;
}

static int __devexit eink3522_spi_remove(struct spi_device *spidev)
{
	s1d13522fb_exit();
	s1d13522_spidev = NULL;
	return 0;
}

static int eink3522_spi_suspend(struct spi_device *spidev, pm_message_t state)
{
	struct s1d13522fb_par *par;

	if (!info)
		return 0;
	else
		par = info->par;

	printk(KERN_INFO "%s, wakeup_count=%d\n", __func__, wakeup_count);
	mutex_lock(&(par->io_lock));
	if (wakeup_count) {
		wakeup_count++;
		if (systemSleep() < 0) {
			printk(KERN_ERR "%s: systemSleep error\n", __func__);
		}
	}
	mutex_unlock(&(par->io_lock));
	return 0;
}

static int eink3522_spi_resume(struct spi_device *spidev)
{
	struct s1d13522fb_par *par;

	if (!info)
		return 0;
	else
		par = info->par;

	printk(KERN_INFO "%s, wakeup_count=%d\n", __func__, wakeup_count);
	mutex_lock(&(par->io_lock));
	if (wakeup_count) {
		wakeup_count--;
		if (systemRun() < 0) {
			printk(KERN_ERR "%s: systemRun error\n", __func__);
		}
	}
	mutex_unlock(&(par->io_lock));
	return 0;
}

static struct of_device_id epd_match_table[] = {
        { .compatible = "epson,s1d13522",},
        { },
};

static struct spi_driver s1d13522spi_driver = {
	.driver = {
		.name = SPI_PRODUCT_NAME,
		.owner = THIS_MODULE,
		.of_match_table = epd_match_table,
	},
	.id_table	= eink3522_spi_idtable,
	.probe = eink3522_spi_probe,
	.remove = __devexit_p(eink3522_spi_remove),

	.suspend = eink3522_spi_suspend,
	.resume = eink3522_spi_resume,
};

static irqreturn_t s1d13522_hrdy_irq(int irq, void *data)
{
	complete(&gHrdyCompletion);
	return IRQ_HANDLED;
}

int s1d13522if_InterfaceInit(FB_INFO_S1D13522 *info)
{
	int retVal = 0;
	int hrdy_irq;
	mutex_init(&ghwMutex);
	spin_lock_init(&gHrdyIrqLock);
	init_completion(&gHrdyCompletion);
	hrdy_irq = gpio_to_irq(GPIO_HRDY);
	retVal = request_threaded_irq(hrdy_irq, NULL,
							s1d13522_hrdy_irq,
							IRQF_TRIGGER_RISING | IRQF_ONESHOT,
							"s1d13522_hrdy", NULL);
	if (retVal) {
		pr_err("s1d13522if_InterfaceInit: could not get IRQ %d\n", hrdy_irq);
	}
	disable_irq(hrdy_irq);
	retVal = set_cmd_mode();
	return retVal;
}

int s1d13522if_InterfaceReInit(FB_INFO_S1D13522 *info)
{
	int retVal = 0;
	retVal = set_cmd_mode();
	return retVal;
}

void s1d13522if_InterfaceTerminate(FB_INFO_S1D13522 *info)
{
}

int eink_spi_write_dma(struct spi_device *spi, void *buf, size_t len)
{
	u16 *bufD;
	u16 cntr = 0;
	u16 *buf16;

	bufD = (u16 *)buf;
	buf16  = kmalloc(len, GFP_KERNEL);

	for (cntr = 0; cntr < (len/2) ; cntr++) {
		buf16[cntr] = ((bufD[cntr] >> 8) | (bufD[cntr] << 8)) ;
	}
	spi_write(spi, (void *) buf16, len);
	kfree(buf16);
	return 0;
}


int eink_spi_write(struct spi_device *spi, void *buf, size_t len)
{
	u16 *bufD;
	u16 cntr = 0;
	u16 buf16;

	bufD = (u16 *)buf;
	for (cntr = 0; cntr < (len/2) ; cntr++) {
		buf16 = ((bufD[cntr] >> 8) | (bufD[cntr] << 8)) ;
		spi_write(spi, (void *) &buf16, sizeof(u16));
	}
	return 0;
}

int s1d13522if_WaitForHRDY(void)
{
#ifndef CONFIG_FB_EPSON_HRDY_OK
	int cnt = HRDY_TIMEOUT;
	u16 dummyRead = 0;
	mutex_lock(&ghwMutex);
	ndelay(20);
	command(RD_REG);
	GPIO_SET(GPIO_HDC, 1);
	data(0x0A);

	dummyRead = data_get();
	while (data_get() & 0x20) {
		if (--cnt <= 0)
			break;

		mdelay(1);
	}
	mutex_unlock(&ghwMutex);

	if (cnt <= 0) {
		printk(KERN_ERR "s1d13522if_WaitForHRDY: I/F busy bit stuck\n");
		return -ERRORVAL;
	}
	dbg_info("s1d13522if_WaitForHRDY: I/F Ready.\n");
	return 0;
#else
	int retval = 0;
	mutex_lock(&ghwMutex);
	retval = wait_for_ready();
	mutex_unlock(&ghwMutex);
	return retval;
#endif
}

u16 s1d13522if_ReadReg16(u16 Index)
{
	u16 dummyRead = 0;
	u16	Value = 0;

	mutex_lock(&ghwMutex);
	command(RD_REG);
	GPIO_SET(GPIO_HDC, 1);
	data(Index);
	dummyRead = data_get();
	Value = data_get();
	mutex_unlock(&ghwMutex);
	dbg_info("epson13522:spi %s(): Reg[%02xh]=%02xh\n", __FUNCTION__, Index, Value);
	return Value;
}

void s1d13522if_WriteReg16(u16 Index, u16 Value)
{
	dbg_info("%s(): %02x,%02x\n", __FUNCTION__, Index, Value);
	if (s1d13522if_WaitForHRDY() != 0)
		return ;

	mutex_lock(&ghwMutex);
	command(WR_REG);
	GPIO_SET(GPIO_HDC, 1);
	data(Index);
	data(Value);
	mutex_unlock(&ghwMutex);
}

int s1d13522if_cmd(unsigned ioctlcmd, s1d13522_ioctl_cmd_params *params, int numparams)
{
	int i = 0;
	unsigned cmd = ioctlcmd & 0xFF;

	dbg_info("epson13522:spi %s(): cmd=%xh numparams=%d\n", __FUNCTION__, cmd, numparams);
	if (s1d13522if_WaitForHRDY() != 0)
		return -ERRORVAL;

	mutex_lock(&ghwMutex);
	command(cmd);
	GPIO_SET(GPIO_HDC, 1);
	for (i = 0; i < numparams; i++)
		data(params->param[i]);
	dbg_info("Executed %s(): cmd=%xh numparams=%d\n", __FUNCTION__, cmd, numparams);
	mutex_unlock(&ghwMutex);
	return 0;
}

static void command(u16 v)
{
	u16 cmd16 = 0;
	cmd16 = v;
	GPIO_SET(GPIO_HDC, 0);
	SPI_WRITE(s1d13522_spidev, &cmd16, sizeof(u16));
}

static void data(u16 v)
{
	u16 data16 = 0;
	data16 = v;
	SPI_WRITE(s1d13522_spidev, &data16, sizeof(u16));
}

static u16 data_get(void)
{
	u16 data16 = 0;
	GPIO_SET(GPIO_HDC, 1);
	SPI_READ(s1d13522_spidev, &data16, sizeof(u16));
	return data16<<8 | data16>>8;
}

#ifdef CONFIG_FB_EPSON_HRDY_OK
static int wait_for_ready(void)
{
	int cnt = HRDY_TIMEOUT;
	int ret = -1;
	int irq = gpio_to_irq(GPIO_HRDY);

	INIT_COMPLETION(gHrdyCompletion);
	enable_irq(irq);
	while (GPIO_GET(GPIO_HRDY) == 0) {

		ret = wait_for_completion_timeout(&gHrdyCompletion, msecs_to_jiffies(15));

		if (ret == 0) {
			// timed out
			if (--cnt <= 0)	{
				printk(KERN_ERR "wait_for_ready: I/F busy bit stuck. ReInit the EPD Controller\n");
				disable_irq(irq);
				mutex_unlock(&ghwMutex);
				ret = s1d13522fb_re_init();
				mutex_lock(&ghwMutex);
				printk(KERN_ERR "wait_for_ready: I/F busy bit stuck. ReInit Completed\n");
				return -1;
			}
		}
	}
	disable_irq(irq);
	return 0;
}
#endif

static int set_cmd_mode(void)
{
#ifndef CONFIG_FB_EPSON_HRDY_OK
	int ticker = HRDY_TIMEOUT;
	u16 dummyRead = 0;
#endif
	GPIO_SET(GPIO_RESET_L, 0);
	mdelay(10);
	GPIO_SET(GPIO_RESET_L, 1);
#ifdef CONFIG_FB_EPSON_HRDY_OK
	return wait_for_ready();
#else
	command(RD_REG);
	GPIO_SET(GPIO_HDC, 1);
	data(0x0A);
	dummyRead = data_get();
	while (data_get() & 0x20) {
		if (--ticker <= 0)
			break;
		mdelay(1);
	}
	if (ticker <= 0) {
		printk("wait_for_ready: I/F busy bit stuck\n");
		return -ERRORVAL;
	}
	return 0;
#endif
}

static inline int spi_write_dma(struct spi_device *spidev,  const void *buf,  dma_addr_t dma_handle, size_t len)
{
	struct spi_message message;
	int ret = -1;
	struct spi_transfer	transfer = {
		.len		= len,
		.bits_per_word = 16,
	};

	spi_message_init(&message);
	transfer.tx_dma = dma_handle;
	transfer.tx_buf = buf;

	transfer.rx_dma = (dma_addr_t) NULL;
	transfer.rx_buf = NULL;

	message.is_dma_mapped = 1;

	spi_message_add_tail(&transfer, &message);
	ret = spi_sync(s1d13522_spidev, &message);
	return ret;
}

void waveformTransfer(u16 *ptr, unsigned copysize)
{
	dbg_info("%s()Transfering the waveform data, bytes[%d]\n", __func__, copysize);
	GPIO_SET(GPIO_HDC, 1);
	SPI_WRITE(s1d13522_spidev, (u16 *)ptr, (copysize));
}

void s1d13522if_BurstWrite16(u16 *ptr16, unsigned copysize16)
{
#ifdef USE_DMA
	u16 *pSource = ptr16;
	u64 mask;
	void *buf_for_tx;
	dma_addr_t dma_handle;
	struct device *dev = &(s1d13522_spidev->master->dev);

	mask = DMA_BIT_MASK(32);
	dev->dma_mask = &mask;
	dev->coherent_dma_mask = DMA_BIT_MASK(32);
	buf_for_tx = dma_alloc_coherent(dev, PAGE_ALIGN((copysize16<<1)), &dma_handle, GFP_KERNEL);
	if (!buf_for_tx) {
		printk("Error allocating memory\n");
		return;
	}
	memcpy(buf_for_tx, (const void *)pSource, (copysize16<<1));
	mutex_lock(&ghwMutex);
	GPIO_SET(GPIO_HDC, 1);
	spi_write_dma(s1d13522_spidev, buf_for_tx, dma_handle, (copysize16<<1));
	mutex_unlock(&ghwMutex);
	dma_free_coherent(dev, PAGE_ALIGN((copysize16<<1)), buf_for_tx, dma_handle);
#else
	GPIO_SET(GPIO_HDC, 1);
	SPI_WRITE(s1d13522_spidev, ptr16, (copysize16<<1));
#endif

}

void s1d13522if_BurstRead16(u16 *ptr16, unsigned copysize16)
{
	mutex_lock(&ghwMutex);
	GPIO_SET(GPIO_HDC, 1);
	while (copysize16-- > 0) {
		SPI_READ(s1d13522_spidev, ptr16++, sizeof(u16));
	}
	mutex_unlock(&ghwMutex);
}

void s1d13522if_BurstWrite16fb_Area(int left, int top,  int mWidth, int mHeight, u16 *ptr16, dma_addr_t handle)
{
	u8  *pfb = (u8 *)ptr16;
	u8 *ps;
	u16 y = 0;
	u16 copy_length = mWidth;
	u32 xpos = left;
	u32 copy_location = 0;
	u32 totalbuflen = 259200;
	u64 mask;
	void *buf_for_tx;
	dma_addr_t dma_handle;
	struct device *dev = &(s1d13522_spidev->master->dev);
	u32 cnt = 0, cnti = 0;

	dbg_info("\n%s():\n", __FUNCTION__);
	mutex_lock(&ghwMutex);
	totalbuflen = copy_length * mHeight;
	/* align to 16 bit word */
	totalbuflen = (totalbuflen & 0x00001) ? totalbuflen+1 : totalbuflen;

	dbg_info("[%s():%d] copy_length:%d totalbuflen:%d left: %d top: %d s1d13522_fb.fix.line_length: %d\n", __FUNCTION__, __LINE__, copy_length, totalbuflen, left, top, s1d13522_fb.fix.line_length);
	dbg_info("[%s():%d] xpos:%d mWidth:%d mHeight:%d copy_length:%d \n",__FUNCTION__, __LINE__, xpos, mWidth, mHeight, copy_length);

	mask = DMA_BIT_MASK(32);
	dev->dma_mask = &mask;
	dev->coherent_dma_mask = DMA_BIT_MASK(32);
	buf_for_tx = dma_alloc_coherent(dev, PAGE_ALIGN((totalbuflen)), &dma_handle, GFP_KERNEL);
	if (!buf_for_tx) {
		printk("Error allocating memory\n");
		return;
	}
	ps = (u8*) buf_for_tx;
//	if (s1d13522_fb.var.bits_per_pixel == 4) {
		for (y = 0; y < mHeight; y++) {
			copy_location = ((top+y) * S1D_DISPLAY_WIDTH) + xpos;

			for(cnt = 0 ; cnt < copy_length; cnt++) {
				ps[cnti] = pfb[copy_location + cnt];
				cnti++;
			}
			//dbg_info("~~~> cnti: %d cnt: %d\n", cnti, cnt);
		}
		GPIO_SET(GPIO_HDC, 1);
		spi_write_dma(s1d13522_spidev,(u16*)buf_for_tx, dma_handle, (totalbuflen));
#if 0
	mutex_lock(&ghwMutex);
		GPIO_SET(GPIO_HDC, 1);
		spi_write_dma(s1d13522_spidev, ptr16, handle, 259200);
#endif
/*	} else {
		printk("Error, s1d13522if_BurstWrite16fb_Area bitdepth not support.\n");
	}*/

	dma_free_coherent(dev, PAGE_ALIGN((totalbuflen)), buf_for_tx, dma_handle);
	mutex_unlock(&ghwMutex);
}

void s1d13522if_BurstWrite16fb(u16 *ptr16, dma_addr_t handle, unsigned copysize16)
{
#ifdef USE_DMA
	u16 *pSource = ptr16;
	dma_addr_t dma_handle = handle;

	mutex_lock(&ghwMutex);
	GPIO_SET(GPIO_HDC, 1);
	spi_write_dma(s1d13522_spidev, pSource, dma_handle, (copysize16<<1));
	mutex_unlock(&ghwMutex);
#else
	printk("[%s:%d] start\n", __func__, __LINE__);
	GPIO_SET(GPIO_HDC, 1);
	SPI_WRITE_DMA(s1d13522_spidev, ptr16, (copysize16<<1));
	printk("[%s:%d] end\n", __func__, __LINE__);
#endif
}

#define FIRMWARE_FILE_COMMAND  "epson/cmds1d13522.fw"

int s1d13522_init_cmdfw(unsigned ioctlcmd)
{
	struct device *dev = &(s1d13522_spidev->master->dev);
	const struct firmware *firmware = NULL;
	int err;
	unsigned cmd = ioctlcmd & 0xFF;

	dbg_info("%s\n", __FUNCTION__);

	err = request_firmware(&firmware, FIRMWARE_FILE_COMMAND, dev);
	if (err) {
		printk(KERN_ERR
		"s1d13522fb: Command Firmware not available [%d].\n", err);
		return -EIO;
	}

	printk("[%s:%d] Firmware size: %d [0x%x]\n", __func__, __LINE__, firmware->size, firmware->size);
	command(cmd);
	GPIO_SET(GPIO_HDC, 1);
	s1d13522if_BurstWrite16fb((u16 *)firmware->data, 0, firmware->size/2);
	release_firmware(firmware);
	return 0;
}

int s1d13522if_Cmd_PackPara(unsigned ioctlcmd, u16 *params, int numparams)
{
#ifdef USE_DMA
	unsigned cmd = ioctlcmd & 0xFF;
	u16 *pSource = params;
	u64 mask = 0;
	void *buf_for_tx = NULL;
	dma_addr_t dma_handle = 0;
	struct device *dev = &(s1d13522_spidev->master->dev);

	mask = DMA_BIT_MASK(32);
	dev->dma_mask = &mask;
	dev->coherent_dma_mask = DMA_BIT_MASK(32);
	buf_for_tx = dma_alloc_coherent(dev, PAGE_ALIGN((numparams<<1)), &dma_handle, GFP_KERNEL);
	if (!buf_for_tx) {
		printk("Error allocating memory\n");
		return -ERRORVAL;
	}
	memset(buf_for_tx, 0x00, PAGE_ALIGN((numparams<<1)));
	memcpy(buf_for_tx, (const void *)pSource, (numparams<<1));
	dbg_info("%s(): cmd=%xh numparams=%d\n", __FUNCTION__, cmd, numparams);
	if (s1d13522if_WaitForHRDY() != 0) {
		return -ERRORVAL;
	}
	mutex_lock(&ghwMutex);
	command(cmd);
	GPIO_SET(GPIO_HDC, 1);
	spi_write_dma(s1d13522_spidev, buf_for_tx, dma_handle, (numparams<<1));
	mutex_unlock(&ghwMutex);
	dma_free_coherent(dev, PAGE_ALIGN((numparams<<1)), buf_for_tx, dma_handle);
#else
	unsigned cmd = ioctlcmd & 0xFF;
	command(cmd);
	GPIO_SET(GPIO_HDC, 1);
	SPI_WRITE(s1d13522_spidev, params, (numparams<<1));
#endif
	return 0;
}

/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Generic I2C progarmming
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static inline int s1d13522if_GenericI2C_wait_for_ready(void)
{
#ifndef CONFIG_FB_EPSON_HRDY_OK
	int cnt = HRDY_TIMEOUT;
	u16 dummyRead = 0;
#else
	int retval = 0;
#endif

#ifndef CONFIG_FB_EPSON_HRDY_OK
	ndelay(20);
	command(RD_REG);
	GPIO_SET(GPIO_HDC, 1);
	data(0x0A);
	dummyRead = data_get();
	while (data_get() & 0x20) {
		if (--cnt <= 0)
			break;
		mdelay(1);
	}
	if (cnt <= 0) {
		printk(KERN_ERR "s1d13522if_WaitForHRDY: I/F busy bit stuck\n");
		return -ERRORVAL;
	}
#else
	ndelay(20);
	retval = wait_for_ready();
	if (retval < 0) {
		return -ERRORVAL;
	}
#endif
	return 0;
}

static inline int s1d13522if_GenericI2C_Write(u16 Index, u16 Value)
{
	if (s1d13522if_GenericI2C_wait_for_ready() != 0) {
		return -1;
	}
	command(WR_REG);
	GPIO_SET(GPIO_HDC, 1);
	data(Index);
	data(Value);
	return 0;
}

static inline u16 s1d13522if_GenericI2C_Read(u16 Index)
{
	u16 dummyRead = 0;
	u16 regVal = 0;
	command(RD_REG);
	GPIO_SET(GPIO_HDC, 1);
	data(Index);
	dummyRead = data_get();
	regVal = data_get();
	return regVal;
}

static inline int s1d13522if_GenericI2C_BusyWait(void)
{
	#define WAITCNT 200
	int i = 0;

	do {
		udelay(100);
		i++;
	} while ((s1d13522if_GenericI2C_Read(0x0218)&0x0001) && (i < WAITCNT));

	if (i < WAITCNT) {
		return 0;
	} else {
		dbg_info("s1d13522 I2C wait complete fail.\n");
		return -ERRORVAL;
	}
}


int s1d13522if_PMIC_Write_Reg(u8 Index, u8 Value)
{
	dbg_info("s1d13522if_PMIC_Write_Reg Reg[%02xh]=%02xh\n", Index, Value);
	mutex_lock(&ghwMutex);

	if (s1d13522if_GenericI2C_Write(0x021E, PMIC_WRITE) == -1) goto hrdy;
	if (s1d13522if_GenericI2C_Write(0x021A, 0x31) == -1) goto hrdy;
	s1d13522if_GenericI2C_BusyWait();

	if (s1d13522if_GenericI2C_Write(0x021E, Index) == -1) goto hrdy;
	if (s1d13522if_GenericI2C_Write(0x021A, 0x01) == -1) goto hrdy;
	s1d13522if_GenericI2C_BusyWait();

	if (s1d13522if_GenericI2C_Write(0x021E, Value) == -1) goto hrdy;
	if (s1d13522if_GenericI2C_Write(0x021A, 0x11) == -1) goto hrdy;
	s1d13522if_GenericI2C_BusyWait();
	mutex_unlock(&ghwMutex);

	return 0;
hrdy:
	mutex_unlock(&ghwMutex);
	return -1;
}

u8 s1d13522if_PMIC_Read_Reg(u8 Index)
{
	u8 Value = 0;
	mutex_lock(&ghwMutex);
	s1d13522if_GenericI2C_Write(0x021E, PMIC_WRITE);
	s1d13522if_GenericI2C_Write(0x021A, 0x31);
	s1d13522if_GenericI2C_BusyWait();

	s1d13522if_GenericI2C_Write(0x021E, Index);
	s1d13522if_GenericI2C_Write(0x021A, 0x01);
	s1d13522if_GenericI2C_BusyWait();

	s1d13522if_GenericI2C_Write(0x021E, PMIC_READ);
	s1d13522if_GenericI2C_Write(0x021A, 0x31);
	s1d13522if_GenericI2C_BusyWait();

	s1d13522if_GenericI2C_Write(0x021A, 0x17);
	s1d13522if_GenericI2C_BusyWait();

	Value = s1d13522if_GenericI2C_Read(0x021C);
	mutex_unlock(&ghwMutex);
	dbg_info("s1d13522if_PMIC_Read_Reg Reg[%02xh]=%02xh\n", Index, Value);

	return Value;
}

static int __init s1d13522spi_init(void)
{
	printk("spi init\n");
	return spi_register_driver(&s1d13522spi_driver);
}

static void __exit s1d13522spi_exit(void)
{
	spi_unregister_driver(&s1d13522spi_driver);
}

MODULE_FIRMWARE(FIRMWARE_FILE_COMMAND);

module_init(s1d13522spi_init);
module_exit(s1d13522spi_exit);

MODULE_AUTHOR("Epson Research and Development");
MODULE_DESCRIPTION("framebuffer driver for Epson s1d13522 controller");
MODULE_LICENSE("GPL");

#endif
