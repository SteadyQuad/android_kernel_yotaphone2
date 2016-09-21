/*-----------------------------------------------------------------------------
linux/drivers/video/epson-13522/s1d13522fb.c

This file provides a sample implementation of an Epson S1D13522 linux frame
buffer driver using a virtual framebuffer and deferred IO. This driver is
derived from the Broadsheet driver included with some distributions of Linux.

Copyright(C) SEIKO EPSON CORPORATION 2011-2013. All rights reserved.

This driver software is distributed as is, without any warranty of any kind,
either express or implied as further specified in the GNU Public License. This
software may be used and distributed according to the terms of the GNU Public
License, version 2. See the file COPYING in the main directory of this archive
for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>
---------------------------------------------------------------------------- */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/list.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/crc32.h>

#include "s1d13522fb.h"
#include "s1d13522fb_panels.h"
#include "s1d13522ioctl.h"

#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include "s1d13522cmd.h"
#include <linux/proc_fs.h>
#include <linux/timer.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>

#include <linux/sysfs.h>
#include <asm/cacheflush.h>

#define USE_PACKED_LOGO 1

#if USE_PACKED_LOGO
#include "epdbootlogo-packed.h"
#else
#include "epdbootlogo.h"
#endif

#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_bus.h>

#define CYTTSP5_DEVICE_ACCESS_NAME "cyttsp5_device_access"
#define CY_BACK_CORE_ID                  "main_ttsp_core_back"


#ifdef CONFIG_S1D13522_INIT_WAVE_EPROM
	#error Wavefrom in EPROM not implemented yet
#endif

#ifdef CONFIG_S1D13522_IOCTL
#include "s1d13522fb_ioctl.h"
#endif

#define S1D13522FB_VERSION_STR		"EPSON13522FB: $Revision: 1.1 $"

#ifdef CONFIG_S1D13522_SHOW_SETTINGS
#ifdef CONFIG_S1D13522_VFB_COLOR565
	#warning Using color format RGB565
#endif
#ifdef CONFIG_S1D13522_VFB_MONO8
	#warning Using color format MONO8
#endif
#ifdef CONFIG_S1D13522_INIT_CMD_FIRMWARE
	#warning Driver requires CMD data file
#endif
#ifdef CONFIG_S1D13522_INIT_CMD_FLASH
	#warning CMD firmware assumed in Serial Flash
#endif
#ifdef CONFIG_S1D13522_INIT_WAVE_FIRMWARE
	#warning Driver requires Waveform data file
#endif
#ifdef CONFIG_S1D13522_INIT_WAVE_FLASH
	#warning Waveform assumed in Serial Flash
#endif
#ifdef CONFIG_S1D13522_INIT_WAVE_EPROM
	#warning Waveform assumed in EPD panel EPROM
#endif
#ifdef CONFIG_S1D13522_INIT_AUTOBOOT
	#warning Serial Flash assumed programmed for autoboot
#endif
#endif

#define CONFIG_S1D13522_IOCTL 1
#define ARGB888_TO_GRAY4	1
#define ERROR   1

#define S1D13522_PANEL_INDEX	7
#define S1D13522_EPD_BPP		8
#define S1D13522_VFB_BPP		8
#define WAVEFORM_SFLASH

#ifdef CONFIG_S1D13522_VFB_MONO8
#define S1D13522_VFB_BPP		8
#endif
#ifdef CONFIG_S1D13522_VFB_COLOR565
#define S1D13522_VFB_BPP		16
#endif

/* #define S1D13522_HEURISTICS_FULL_UPDATE	(44*S1D13522_VFB_BPP) */
#define S1D13522_HEURISTICS_FULL_UPDATE	500

/*selects WAVEFORM Storage on SDRAM */
/*#define S1D13522_WFM_SEL_SDRAM  0x00001*/
/* selects WAVEFORM Storage on External Serial Flash (Default) */
/*#define S1D13522_WFM_SEL_SFM    0x000C0*/

int  __devinit s1d13522fb_init(void);
void __devexit s1d13522fb_exit(void);
static int __devinit s1d13522fb_probe(struct platform_device *pdev);
static int __devexit s1d13522fb_remove(struct platform_device *dev);
static ssize_t s1d13522fb_write(struct fb_info *info,
		const char __user *buf, size_t count, loff_t *ppos);
static void s1d13522fb_fillrect(struct fb_info *fbp,
		const struct fb_fillrect *fbr);
static void s1d13522fb_copyarea(struct fb_info *fbp,
		const struct fb_copyarea *fbc);
static void s1d13522fb_imageblit(struct fb_info *fbp,
		const struct fb_image *fbi);
static void s1d13522fb_dpy_deferred_io(struct fb_info *info,
		struct list_head *pagelist);

static int getVcomFromSF(void);
static void getPanelInfoFromSF(void);
static u32 getCRCWaveform(void);

void update_only(void);
#ifdef CONFIG_S1D13522_INIT_CMD_FIRMWARE
static int  s1d13522_init_cmdfw(struct s1d13522fb_par *par);
#endif

#ifdef CONFIG_S1D13522_INIT_WAVE_FIRMWARE
static int  s1d13522_init_wavefw(struct s1d13522fb_par *par, int staddr);
#endif

#ifdef CONFIG_S1D13522_IOCTL
static int s1d13522fb_ioctl(struct fb_info *info, unsigned int cmd,
		unsigned long arg);
#endif

#ifndef CONFIG_S1D13522_VFB_MONO8
/*static void s1d13522fb_colorconvert_area(struct s1d13522fb_par *par); */
#endif

#if defined(ARGB888_TO_GRAY4) || defined(RGB565_TO_GRAY4)
static void s1d13522fb_colorconvert(struct s1d13522fb_par *par, int onlyWM);
#endif

int waveform_mode_location;
u32 updateMode;
u32 updateWaveform;
u32 updateModeSpecial = UPD_FULL;
u32 updateWaveformSpecial = WF_MODE_GC16;
u32 numPage;
struct delayed_work idw;
struct delayed_work dw;
u32 lastWaveform = 0;
int wakeup_count = 0;

char *s1d13522fb_version = S1D13522FB_VERSION_STR;
extern struct spi_device *s1d13522_spidev;
extern struct device *devlog;
static int vcomVal;
extern int epdCheck;
static u16 borderEnable = 1;
static int temperature1, temperature2, temperature3, temperatureW;
static int updateImage;
static int crcWaveformSize = 200;
static u32 crcWaveform;
static u16 borderColor;
static int s1d13522_programming;
enum deviceColor {
	COLOR_BLACK = 0x000000,
	COLOR_WHITE = 0xFFFFFF,
	COLOR_READ_ERROR = -1,
} devC;
enum systemState {
	noError,
	bitStuck,
	bitStuckAgain,
	permanentError,
	bitRecovered,
	retry
} state;
enum updateCheck {
	DRAW_DISABLE = 0,
	DRAW_ENABLE = 1,
} uCheck;
static int drawingEnabled = DRAW_ENABLE;
struct mutex drawingEnabledMutex;

enum updateS {
	FULLUPDATE = 1,
	PARTUPDATE = 2,
} updateState;

dma_addr_t DmaPhys;
dma_addr_t DmaPhysH;
dma_addr_t DmaPhysHC;
struct fb_info *info;

u64 bufferSize;
int device_color = COLOR_BLACK;

int update_in_progress = 0;
struct mutex vsync_mutex;

struct fb_info  s1d13522_fb;
FB_INFO_S1D13522 s1d13522fb_info;
static struct fb_fix_screeninfo s1d13522fb_fix = {
	.id		= "s1d13522fb",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.type_aux	= 0,
	.xpanstep	= 0,
	.ypanstep	= 0,
	.ywrapstep	= 0,
	.accel		= FB_ACCEL_NONE,
};
static struct fb_ops s1d13522fb_ops = {
	.owner			= THIS_MODULE,
	.fb_read		= fb_sys_read,
	.fb_write		= s1d13522fb_write,
	.fb_setcolreg	= s1d13522fb_setcolreg,
	.fb_fillrect	= s1d13522fb_fillrect,
	.fb_copyarea	= s1d13522fb_copyarea,
	.fb_imageblit	= s1d13522fb_imageblit,
#ifdef CONFIG_S1D13522_IOCTL
	.fb_ioctl		= s1d13522fb_ioctl,
#endif
};
static struct fb_var_screeninfo s1d13522fb_var __devinitdata = {
#ifdef CONFIG_S1D13522_VFB_MONO8
	.bits_per_pixel	= 8,
	.grayscale	= 1,
	.red =		{ 0, 4, 0 },
	.green =	{ 0, 4, 0 },
	.blue =		{ 0, 4, 0 },
	.transp =	{ 0, 0, 0 },
#endif
#ifdef CONFIG_S1D13522_VFB_COLOR565
	.bits_per_pixel	= 16,
	.grayscale	= 0,
	.red =		{11, 5, 0 },
	.green =	{ 5, 6, 0 },
	.blue =		{ 0, 5, 0 },
	.transp =	{ 0, 0, 0 },
#endif
};
static struct fb_deferred_io s1d13522fb_defio = {
	.delay		= HZ*60,
	.deferred_io	= s1d13522fb_dpy_deferred_io,
};

#define NUM_TM_SENSORS 3

struct s1d13522fb_tm_sensor {
	struct thermal_zone_device	*tz_dev;
	enum thermal_device_mode	mode;
	unsigned int			sensor_num;
	unsigned int			index;
};

struct s1d13522fb_tm_sensor	*tm_sensors;

void s1d13522_write_reg(struct s1d13522fb_par *par, u16 reg, u16 data)
{
	dbg_info("%s():\n", __func__);
	par->board->write_reg(par, reg, data);
}
u16 s1d13522_read_reg(struct s1d13522fb_par *par, u16 reg)
{
	dbg_info("%s():\n", __func__);
	return par->board->read_reg(par, reg);
}
int s1d13522_send_cmdargs(struct s1d13522fb_par *par,
		u16 cmd, int argc, u16 *argv)
{
	dbg_info("%s():\n", __func__);
	return par->board->send_cmdargs(par, cmd, argc, argv);
}
int s1d13522_send_command(struct s1d13522fb_par *par, u16 cmd)
{
	dbg_info("%s():\n", __func__);
	return par->board->send_cmdargs(par, cmd, 0, NULL);
}

int systemRun(void)
{
	u16 reg0A, reg230, cntr = 0;
	s1d13522_ioctl_cmd_params cmd_params;
	dbg_info("%s: wakeup_count = %d\n", __func__, wakeup_count);
	if (!wakeup_count) {
		dbg_info("vacomVal in %s() is %d\n", __func__, vcomVal);
		reg0A = s1d13522if_ReadReg16(0x0A);
		s1d13522if_WriteReg16(0x0A, (reg0A | (BIT(12))));
		mdelay(5);

		if (s1d13522if_cmd(RUN_SYS, &cmd_params, 0) == -1) {
			gpio_set_value(GPIO_HRDY, 1);
			mdelay(10);
			if (s1d13522if_cmd(RUN_SYS, &cmd_params, 0) == -1) {
				return -1;
			}
		}
		mdelay(2);
		reg230 = s1d13522if_ReadReg16(0x230);
		while (((reg230 >> 13) & 0x1) != 0x1) {
			msleep(3);
			reg230 = s1d13522if_ReadReg16(0x230);
			if (cntr++ > 50) {
				printk(KERN_ERR "RUN_SYS command failed, need to reinit the EPD panel reg230:%X cntr: %d\n", reg230, cntr);
				s1d13522fb_re_init();
				break;
			}
		}
		s1d13522if_PMIC_Write_Reg(0x03, vcomVal & 0xFF);
		s1d13522if_PMIC_Write_Reg(0x04, (vcomVal >> 8));
	}
	wakeup_count++;
	return 0;
}

int systemSleep(void)
{
	unsigned int reg0A;
	s1d13522_ioctl_cmd_params cmd_params;
	dbg_info("%s: wakeup_count = %d\n", __func__, wakeup_count);
	wakeup_count--;
	if (!wakeup_count) {
		s1d13522if_WaitForHRDY();
		s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0);
		s1d13522if_WriteReg16(0x232, 0x100);
		s1d13522if_cmd(SLP, &cmd_params, 0);
		s1d13522if_WaitForHRDY();
		reg0A = s1d13522if_ReadReg16(0x0A);
		s1d13522if_WriteReg16(0x0A, (reg0A & ~(BIT(12))));
		mdelay(10);
	}
	return 0;
}

static int s1d13522fb_EPDInit(int reinit)
{
#ifndef CONFIG_FB_EPSON_AUTOBOOT
	int retval = 0;
	s1d13522_ioctl_cmd_params cmd_params;
	dbg_info("%s()\n", __func__);
	s1d13522if_WriteReg16(0x000A, (0x01<<12));
	mdelay(5);
	cmd_params.param[0] = 0x0003;
	cmd_params.param[1] = 0x5949;
	cmd_params.param[2] = 0x0040;
	if (s1d13522if_cmd(INIT_PLL_STANDBY, &cmd_params, 3) == -ERROR)
		return -ERROR;
	mdelay(2);

	retval = s1d13522if_ReadReg16(0x000A);
	dbg_info("0x000A=%0X\n", retval);
	s1d13522_init_cmdfw(INIT_CMD_SET);

	if (s1d13522if_cmd(INIT_SYS_RUN, &cmd_params, 0) == -ERROR)
		return -ERROR;
	if (!reinit)
		wakeup_count++;

	mdelay(10);
#ifndef WAVEFORM_SFLASH
	cmd_params.param[0] = (S1D_WF_ADDR&0x0000ffff);
	cmd_params.param[1] = ((S1D_WF_ADDR>>16)&0x000003ff);
	cmd_params.param[2] = (sizeof(s1d13522_waveform)>>1)&0x0ffff;
	cmd_params.param[3] = ((sizeof(s1d13522_waveform)>>1)&0x3ff0000)>>16;
	if (s1d13522if_cmd(BST_WR_SDR, &cmd_params, 4) == -ERROR)
		return -ERROR;

	cmd_params.param[0] = 0x154;
	if (s1d13522if_cmd(WR_REG, &cmd_params, 1) == -ERROR)
		return -ERROR;

	waveformTransfer((u16 *)s1d13522_waveform, sizeof(s1d13522_waveform));
	dbg_info("%s()Transferred the waveform data bytes[%d]\n",
			__func__, sizeof(s1d13522_waveform));
	if (s1d13522if_cmd(BST_END_SDR, &cmd_params, 0) == -ERROR)
		return -ERROR;

	cmd_params.param[0] = 0x0001;
	if (s1d13522if_cmd(INIT_WAVEDEV, &cmd_params, 1) == -ERROR)
		return -ERROR;

	cmd_params.param[0] = (S1D_WF_ADDR&0x0000ffff);
	cmd_params.param[1] = ((S1D_WF_ADDR>>16)&0x000003ff);
	if (s1d13522if_cmd(RD_WFM_INFO, &cmd_params, 2) == -ERROR)
		return -ERROR;
#else
	cmd_params.param[0] = 0x0000;
	if (s1d13522if_cmd(INIT_WAVEDEV, &cmd_params, 1) == -ERROR)
		return -ERROR;

	cmd_params.param[0] = (FLASH_WF_ADDR&0x0000ffff);
	cmd_params.param[1] = ((FLASH_WF_ADDR>>16)&0x000003ff);
	if (s1d13522if_cmd(RD_WFM_INFO, &cmd_params, 2) == -ERROR)
		return -ERROR;

#endif
	if (s1d13522if_cmd(UPD_GDRV_CLR, &cmd_params, 0) == -ERROR)
		return -ERROR;

	if (s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0) == -ERROR)
		return -ERROR;
#endif
	return 0;
}

static inline int checkBorderUpdate(u16 x)
{
	dbg_info("%s() borderColor:%d x:%d\n", __func__, borderColor, x);
	borderEnable = 0;
	if (borderEnable) {
		s1d13522if_WriteReg16(0x326, (borderColor << 4));
		borderEnable = 0;
		return x |= BIT(14);
	}
	return x;
}

static int s1d13522fb_GetTemp(struct s1d13522_ioctl_temperature *t)
{
#define WAITCNT 200
	int i = 0;
	u16 data = 0;
	int cntr = 0;
	int reg252 = 0;
	int index = t->index;
	signed short negT = 0;
	struct s1d13522fb_par *par = info->par;

	mutex_lock(&(par->io_lock));

	if (systemRun() < 0) {
		printk(KERN_ERR "%s: systemRun error\n", __func__);
		mutex_unlock(&(par->io_lock));
		return -1;
	}

	dbg_info("----------> Temperature reading starts %s() index is %d\n",
		__func__, index);
	for (cntr = 0 ; cntr < 3 ; cntr++) {
		if (!(BIT(cntr) & index)) {
			t->value[cntr] = -99;
			continue;
		}

		reg252 = s1d13522if_ReadReg16(0x0252);
		cntr == 0 ? (s1d13522if_WriteReg16(0x0252, ((reg252 | BIT(3))
				& ~BIT(4)))) : 0;
		cntr == 1 ? (s1d13522if_WriteReg16(0x0252, ((reg252 | BIT(4))
				& ~BIT(3)))) : 0;
		cntr == 2 ? (s1d13522if_WriteReg16(0x0252, (reg252 | BIT(3)
				| BIT(4)))) : 0;

		if (s1d13522if_PMIC_Write_Reg(0x0D, 0x80) == -1)
			goto hrdy;

		do {
			udelay(100);
			i++;
		} while ((s1d13522if_PMIC_Read_Reg(0x0D) & 0x80) &&
				 (i < WAITCNT));

		if (i >= WAITCNT) {
			dbg_info("s1d13522 I2C read busy.\n");
			goto hrdy;
		}
		data = s1d13522if_PMIC_Read_Reg(0x00);
		if ((data & 0x80) >= 1)
			negT = data - 255 - 1;
		else
			negT = data;
		t->value[cntr] = negT;
	}
hrdy:
	if (systemSleep() < 0)
		printk(KERN_ERR "%s: systemSleep error\n", __func__);

	mutex_unlock(&(par->io_lock));
	dbg_info("[%d:%s] val of temperature t1:%d t2:%d t3:%d\n",
		 __LINE__, __func__ , t->value[0], t->value[1], t->value[2]);
	return 0;
}

static void s1d13522fb_UpdateBorder(u16 enable)
{
	borderEnable = enable;
}

static void s1d13522fb_SetBorderColor(u16 color)
{
	borderColor = color;
}

static int s1d13522fb_systemSleep(struct s1d13522fb_par *par)
{
	dbg_info("[%d]EPD Sleep Started..!!!\n", __LINE__);
	mutex_lock(&(par->io_lock));
	dbg_info("[%d]EPD Sleep Started..Lock Acquired!!!\n", __LINE__);
	if (systemSleep() < 0)
		printk(KERN_ERR "%s: systemSleep error\n", __func__);
	mutex_unlock(&(par->io_lock));
	dbg_info("[%d]EPD Sleep Successful..Lock Freed!!!\n", __LINE__);
	return 0;
}

static void s1d13522fb_systemRun(struct s1d13522fb_par *par)
{
	dbg_info("[%d]EPD Wake-up Started..!!!\n", __LINE__);
	mutex_lock(&(par->io_lock));
	dbg_info("[%d]EPD Wake-up Started..Got the lock!!!\n", __LINE__);
	if (systemRun() < 0)
		printk(KERN_ERR "%s: systemRun error\n", __func__);
	dbg_info("[%d]EPD Wake-up Successful..!!!\n", __LINE__);
	mutex_unlock(&(par->io_lock));
	dbg_info("[%d]EPD Wake-up Successful..Freed the lock!!!\n", __LINE__);
}

void waitForSPIReady(unsigned short ra, unsigned short pos, unsigned short val)
{
	unsigned short v = s1d13522if_ReadReg16(ra);
	while (((v >> pos) & 0x1) != (val & 0x1))
		v = s1d13522if_ReadReg16(ra);
}

static int s1d13522fb_PMICInit(void)
{
	dbg_info("%s()\n", __func__);
	if (s1d13522if_PMIC_Write_Reg(0x09, 0xE4) == -ERROR)
		return -ERROR;
	if (s1d13522if_PMIC_Write_Reg(0x0A, 0x55) == -ERROR)
		return -ERROR;
	if (s1d13522if_PMIC_Write_Reg(0x0B, 0x1E) == -ERROR)
		return -ERROR;
	if (s1d13522if_PMIC_Write_Reg(0x0C, 0xE0) == -ERROR)
		return -ERROR;

	s1d13522if_WriteReg16(0x0254, 0x0400);
	s1d13522if_WriteReg16(0x0244, (0x01<<3));
	s1d13522if_WriteReg16(0x0320, (0x01<<0));
	return 0;
}

static void getPanelInfoFromSF(void)
{
	unsigned reg204 = 0, reg200 = 0;
	reg204 = s1d13522if_ReadReg16(0x0204);
	s1d13522if_WriteReg16(0x0204, (reg204 & ~(BIT(7))));
	s1d13522if_WriteReg16(0x0208, 0x0001);
	s1d13522if_WriteReg16(0x0202, 0x0103);
	waitForSPIReady(0x206, 3, 0);
	s1d13522if_WriteReg16(0x0202, (SF_BORDERADDR >> 16 & 0xFF));
	waitForSPIReady(0x206, 3, 0);
	s1d13522if_WriteReg16(0x0202, (SF_BORDERADDR >> 8 & 0xFF));
	waitForSPIReady(0x206, 3, 0);
	s1d13522if_WriteReg16(0x0202, (SF_BORDERADDR & 0xFF));
	waitForSPIReady(0x206, 3, 0);
	s1d13522if_WriteReg16(0x0202, (0x0000));
	waitForSPIReady(0x206, 3, 0);
	reg200 = s1d13522if_ReadReg16(0x0200);
	s1d13522if_WriteReg16(0x0208, 0x0000);
	reg204 = s1d13522if_ReadReg16(0x0204);
	s1d13522if_WriteReg16(0x0204, (reg204 | (BIT(7))));
	if (reg200 == 0xE6) { /* E6h is B */
		printk(KERN_INFO "Device color BLACK");
		//s1d13522fb_SetBorderColor(1);
		device_color = COLOR_BLACK;
	} else if (reg200 == 0xFB) { /* FBh is W */
		printk(KERN_INFO "Device color WHITE\n");
		//s1d13522fb_SetBorderColor(15);
		device_color = COLOR_WHITE;
	} else {
		printk(KERN_ERR "%s: Invalid panel color %d Default to Black\n",
				__func__, reg200);
		//s1d13522fb_SetBorderColor(1);
		device_color = COLOR_BLACK;
	}
}

static int getVcomFromSF(void)
{
	unsigned reg204 = 0, reg200 = 0;
	int  tempVal = 0, cntr = 0, tempSign = 0;
	dbg_info("... reading the serial flash memory\n");
	reg204 = s1d13522if_ReadReg16(0x0204);
	s1d13522if_WriteReg16(0x0204, (reg204 & ~(BIT(7))));

	s1d13522if_WriteReg16(0x0208, 0x0001);
	s1d13522if_WriteReg16(0x0202, 0x0103);
	waitForSPIReady(0x206, 3, 0);

	s1d13522if_WriteReg16(0x0202, (SF_VCOMADDR >> 16 & 0xFF));
	waitForSPIReady(0x206, 3, 0);

	s1d13522if_WriteReg16(0x0202, (SF_VCOMADDR >> 8 & 0xFF));
	waitForSPIReady(0x206, 3, 0);

	s1d13522if_WriteReg16(0x0202, (SF_VCOMADDR & 0xFF));
	waitForSPIReady(0x206, 3, 0);

	for (cntr = 0; cntr < 6; cntr++) {
		s1d13522if_WriteReg16(0x0202, (0x0000));
		waitForSPIReady(0x206, 3, 0);

		reg200 = s1d13522if_ReadReg16(0x0200);
		if ((reg200 != 0x0c) && (cntr == 0)) {
			printk(KERN_ERR "-------------------No EPD panel is connected-------------------\n");
			epdCheck = -1;
			return -ERROR;
		} else
			epdCheck = 1;
		tempSign = ((cntr == 0) ? ((reg200 == 0x0c) ? 1 : 0) : tempSign);
		if (reg200 == 0xFF)
			break;
		tempVal = ((cntr == 1) ? reg200 * 100 : tempVal);
		tempVal += ((cntr == 2) ? (reg200 == 0x0b ? 0 : -(tempVal + 100)) : 0);
		tempVal += ((cntr == 3) ? reg200 * 10 : 0);
		tempVal += ((cntr == 4) ? reg200 * 1 : 0);
	}
	dbg_info("VCOM :::: %d\n", tempVal);
	vcomVal = (tempSign == 1) ? tempVal : 180;
	dbg_info("VCOM :::: %d\n", vcomVal);
	s1d13522if_WriteReg16(0x0208, 0x0000);
	reg204 = s1d13522if_ReadReg16(0x0204);
	s1d13522if_WriteReg16(0x0204, (reg204 | (BIT(7))));
	s1d13522if_PMIC_Write_Reg(0x03, vcomVal & 0xFF);
	s1d13522if_PMIC_Write_Reg(0x04, (vcomVal >> 8));
	return 0;
}

static u32 getCRCWaveform(void)
{
	unsigned reg204 = 0, reg200 = 0;
	int cntr = 0;
	u8 *data;
	struct s1d13522fb_par *par = info->par;
	u32 retCRC = 0;

	mutex_lock(&(par->io_lock));

	if (systemRun() < 0) {
		printk(KERN_ERR "%s: systemRun error\n", __func__);
		goto err;
	}

	data = (u8*) kmalloc(sizeof(u8) * crcWaveformSize , GFP_KERNEL);
	if(data == NULL)
		goto sleeperr;

	dbg_info("... reading the serial flash memory crcWaveformSize:%d \n", crcWaveformSize);
	reg204 = s1d13522if_ReadReg16(0x0204);
	s1d13522if_WriteReg16(0x0204, (reg204 & ~(BIT(7))));

	s1d13522if_WriteReg16(0x0208, 0x0001);
	s1d13522if_WriteReg16(0x0202, 0x0103);
	waitForSPIReady(0x206, 3, 0);

	s1d13522if_WriteReg16(0x0202, (FLASH_WF_ADDR >> 16 & 0xFF));
	waitForSPIReady(0x206, 3, 0);

	s1d13522if_WriteReg16(0x0202, (FLASH_WF_ADDR >> 8 & 0xFF));
	waitForSPIReady(0x206, 3, 0);

	s1d13522if_WriteReg16(0x0202, (FLASH_WF_ADDR & 0xFF));
	waitForSPIReady(0x206, 3, 0);

	for (cntr = 0; cntr < crcWaveformSize; cntr++) {
		s1d13522if_WriteReg16(0x0202, (0x0000));
		waitForSPIReady(0x206, 3, 0);
		reg200 = s1d13522if_ReadReg16(0x0200);
		data[cntr] = reg200;
	}
	s1d13522if_WriteReg16(0x0208, 0x0000);
	reg204 = s1d13522if_ReadReg16(0x0204);
	s1d13522if_WriteReg16(0x0204, (reg204 | (BIT(7))));
	retCRC = crc32(~0, data, crcWaveformSize);
	dbg_info("EPD~~~~> Checksum is %X\n", retCRC);
	kfree(data);

sleeperr:
	if (systemSleep() < 0)
		printk(KERN_ERR "%s: systemSleep error\n", __func__);

err:
	mutex_unlock(&(par->io_lock));
	return retCRC;
}

static void s1d13522fb_SetTemp(signed short temperature)
{
	u16 t;
        struct s1d13522fb_par *par = info->par;

	dbg_info("[%d:%s] val of temperature to write %d\n",
				__LINE__, __func__ , temperature);
	mutex_lock(&(par->io_lock));
        if (systemRun() < 0) {
		printk(KERN_ERR "%s: systemRun error\n", __func__);
		mutex_unlock(&(par->io_lock));
                return ;
	}
	dbg_info("[%d:%s] val of temperature to write %d\n",
				__LINE__, __func__ , temperature);
	if (temperature < 0)
		t = 255 + temperature + 1;
	else if (temperature > 50)
		t = 50;
	else
		t = temperature;

	s1d13522if_WriteReg16(0x0322, t);
	if (systemSleep() < 0)
		printk(KERN_ERR "%s: systemSleep error\n", __func__);
	mutex_unlock(&(par->io_lock));
}

static void s1d13522fb_registerInit(void)
{
	u16 data = 0;
	dbg_info("%s()\n", __func__);

	s1d13522if_WriteReg16(0x001A, 0x0005);
	data = s1d13522if_ReadReg16(0x0250);
	s1d13522if_WriteReg16(0x0250, (data | BIT(4) | BIT(3)));

	s1d13522if_WriteReg16(0x033e, 0x0080);
	s1d13522if_WriteReg16(0x0244, 0x0002);

	data = s1d13522if_ReadReg16(0x0002);
	dbg_info("Product code:0x%X\n", data);

	s1d13522if_WriteReg16(0x0140, BIT(5));
	data = s1d13522if_ReadReg16(0x0140);
	dbg_info("Host memory access config reg 0x140h = 0x%0X\n", data);

	s1d13522if_WriteReg16(0x0334, 0x4006);
}

static void s1d13522fb_refresh_global(struct fb_info *info,
					unsigned cmd, unsigned waveform)
{
	s1d13522_ioctl_cmd_params cmd_params;
	u32 size16 = bufferSize/2;
	struct s1d13522fb_par *par = info->par;
	u16 *pSource = (u16 *) par->epd_buffer_host;
	dma_addr_t handle = (dma_addr_t)DmaPhys;
	u16 lutsel = (waveform == WF_MODE_AUTO) ? 0xC4 : 0x84;

	dbg_info("%s()\n", __func__);
	mutex_lock(&s1d13522fb_info.refresh_display_mutex);
	s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0);
	s1d13522if_WriteReg16(0x330, lutsel);
	cmd_params.param[0] = (0x3<<4);
	s1d13522if_cmd(LD_IMG, &cmd_params, 1);
	cmd_params.param[0] = 0x154;
	s1d13522if_cmd(WR_REG, &cmd_params, 1);
	s1d13522if_BurstWrite16fb(pSource, handle, size16);
	s1d13522if_cmd(LD_IMG_END, &cmd_params, 0);

	s1d13522if_WriteReg16(0x33a, 0xffff);
	cmd_params.param[0] = (waveform<<8);
	cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);

	s1d13522if_WriteReg16(0x232, 0x101);
	s1d13522if_cmd(cmd, &cmd_params, (cmd == UPD_INIT) ? 0 : 1);
	s1d13522if_WriteReg16(0x232, 0x100);
	mutex_unlock(&s1d13522fb_info.refresh_display_mutex);

	dbg_info("%s() exit\n", __func__);
}

static void s1d13522fb_refresh_area(struct fb_info *info,
					unsigned cmd, unsigned waveform,
					u16 left, u16 top,  u16 mWidth,
					u16 mHeight)
{
	s1d13522_ioctl_cmd_params cmd_params;
	struct s1d13522fb_par *par = info->par;
	u16 *pSource = (u16 *) par->epd_buffer_host;
	dma_addr_t handle = (dma_addr_t)DmaPhys;
	u16 lutsel = (waveform == WF_MODE_AUTO) ? 0xC4 : 0x84;

	dbg_info("%s()\n", __func__);

	mutex_lock(&s1d13522fb_info.refresh_display_mutex);
	s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0);
	s1d13522if_WriteReg16(0x330, lutsel);
	cmd_params.param[0] = (0x3<<4);
	cmd_params.param[1] = left;
	cmd_params.param[2] = top;
	cmd_params.param[3] = mWidth;
	cmd_params.param[4] = mHeight;
	s1d13522if_cmd(LD_IMG_AREA, &cmd_params, 5);

	cmd_params.param[0] = 0x154;
	s1d13522if_cmd(WR_REG, &cmd_params, 1);
	s1d13522if_BurstWrite16fb_Area(left, top, mWidth,
			mHeight, pSource, handle);
	s1d13522if_cmd(LD_IMG_END, &cmd_params, 0);
	s1d13522if_WriteReg16(0x33a, 0xffff);
	cmd_params.param[0] = (waveform<<8);
	cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);

	cmd_params.param[1] = left;
	cmd_params.param[2] = top;
	cmd_params.param[3] = mWidth;
	cmd_params.param[4] = mHeight;

	s1d13522if_WriteReg16(0x232, 0x101);
	s1d13522if_cmd(cmd, &cmd_params, (cmd == UPD_INIT) ? 0 : 5);
	s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0);
	s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0);
	s1d13522if_WaitForHRDY();

	s1d13522if_WriteReg16(0x232, 0x100);

	mutex_unlock(&s1d13522fb_info.refresh_display_mutex);
	dbg_info("%s() exit\n", __func__);

}

static int s1d13522fb_update_area(struct fb_info *info,
				   unsigned int cmd, unsigned int waveform,
				   u16 left, u16 top,  u16 width, u16 height,
				   u_int8_t *imgdata)
{
	struct s1d13522fb_par *par = info->par;
	dma_addr_t handle = (dma_addr_t)DmaPhys;
	u16 lutsel;
	size_t data_len;
	s1d13522_ioctl_cmd_params cmd_params;

	dbg_info("%s(): cmd=%d, waveform=%d, width=%d, height=%d, left=%d, top=%d\n",
		 __func__, cmd, waveform, width, height, left, top);

	mutex_lock(&(par->io_lock));

	s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0);

	lutsel = (waveform == WF_MODE_AUTO) ? 0xC4 : 0x84;
	s1d13522if_WriteReg16(0x330, lutsel);

	cmd_params.param[0] = (0x3<<4);
	cmd_params.param[1] = left;
	cmd_params.param[2] = top;
	cmd_params.param[3] = width;
	cmd_params.param[4] = height;
	s1d13522if_cmd(LD_IMG_AREA, &cmd_params, 5);

	cmd_params.param[0] = 0x154;
	s1d13522if_cmd(WR_REG, &cmd_params, 1);

	/* roundup to double pixel value */
	data_len = width * height;
	data_len++;

	s1d13522if_BurstWrite16fb((u16 *)imgdata, handle, data_len >> 1);
	s1d13522if_cmd(LD_IMG_END, &cmd_params, 0);

	s1d13522if_WriteReg16(0x33a, 0xffff);

	cmd_params.param[0] = (waveform<<8);
	cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);
	cmd_params.param[1] = left;
	cmd_params.param[2] = top;
	cmd_params.param[3] = width;
	cmd_params.param[4] = height;

	// Use automatic VCOM management
	s1d13522if_WriteReg16(0x232, 0x0);
	s1d13522if_cmd(cmd, &cmd_params, (cmd == UPD_INIT) ? 0 : 5);

	mutex_unlock(&(par->io_lock));
	return 0;
}

static void s1d13522fb_cursor_enable(struct fb_info *info,
					unsigned cmd, unsigned waveform,
					u16 alpha, u16 left, u16 top,
					u16 mWidth, u16 mHeight)
{
	s1d13522_ioctl_cmd_params cmd_params;
	struct s1d13522fb_par *par = info->par;
	u32 size16 = (mWidth * mHeight) / 4;

	u8 *pSource = (u8 *) par->epd_buffer_host;
	dma_addr_t handle = (dma_addr_t)DmaPhys;

	mutex_lock(&s1d13522fb_info.refresh_display_mutex);
	dbg_info("[%s] entering\n", __func__);

	s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0);
	cmd_params.param[0] = 0x21;
	cmd_params.param[1] = mWidth;
	cmd_params.param[2] = mHeight;
	cmd_params.param[3] = 0x0000;
	s1d13522if_cmd(CSR_MAINCFG, &cmd_params, 4);

	cmd_params.param[0] = (S1D_CSR_ADDR & 0x0000ffff);
	s1d13522if_cmd(CSR_ADRCFG, &cmd_params, 1);

	cmd_params.param[0] = 0x3DE;
	s1d13522if_cmd(WR_REG, &cmd_params, 1);
	s1d13522if_BurstWrite16fb((u16 *)pSource, handle, size16);
	cmd_params.param[0] = left;
	cmd_params.param[1] = top;

	s1d13522if_cmd(CSR_XYSETUP, &cmd_params, 2);

	cmd_params.param[0] = (S1D_CSR_ADDR & 0x0000ffff);
	s1d13522if_cmd(CSR_ADRCFG, &cmd_params, 1);

	mutex_unlock(&s1d13522fb_info.refresh_display_mutex);
	dbg_info("%s() exit\n", __func__);
}

static void s1d13522fb_cursor_move(u16 left, u16 top)
{
	s1d13522_ioctl_cmd_params cmd_params;
	mutex_lock(&s1d13522fb_info.refresh_display_mutex);
	cmd_params.param[0] = left;
	cmd_params.param[1] = top;
	s1d13522if_cmd(S1D13522_CSR_XYSETUP, &cmd_params, 2);
	mutex_unlock(&s1d13522fb_info.refresh_display_mutex);
	dbg_info("%s() exit\n", __func__);
}

static void s1d13522fb_cursor_disable(unsigned cmd)
{
	s1d13522_ioctl_cmd_params cmd_params;
	mutex_lock(&s1d13522fb_info.refresh_display_mutex);
	cmd_params.param[0] = 0x20;
	cmd_params.param[1] = 0x00;
	cmd_params.param[2] = 0x00;
	cmd_params.param[3] = 0x00;
	s1d13522if_cmd(S1D13522_CSR_MAINCFG, &cmd_params, 4);
	mutex_unlock(&s1d13522fb_info.refresh_display_mutex);
	dbg_info("%s() exit\n", __func__);
}

static void s1d13522fb_refresh_PIP_global(struct fb_info *info,
				unsigned cmd, unsigned waveform,
				u16 alpha, u16 left, u16 top,
				u16 mWidth, u16 mHeight)
{
	s1d13522_ioctl_cmd_params cmd_params;
	struct s1d13522fb_par *par = info->par;
	u32 size16 = (mWidth*mHeight)/2 + 1;

	u8 *pSource = (u8 *) (par->epd_buffer_host);
	dma_addr_t handle = (dma_addr_t)(DmaPhys);

	u16 lutsel = (waveform == WF_MODE_AUTO) ? 0xC4 : 0x84;

	dbg_info("%s()\n", __func__);

	mutex_lock(&s1d13522fb_info.refresh_display_mutex);

	s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0);
	s1d13522if_WriteReg16(0x330, lutsel);
	cmd_params.param[0] = (alpha<<12)|(0x02<<4);
	cmd_params.param[1] = mWidth;
	cmd_params.param[2] = mHeight;
	cmd_params.param[3] = 0x0000;
	s1d13522if_cmd(PIP_ENABLE, &cmd_params, 4);
	cmd_params.param[0] = (S1D_PIP_ADDR&0x0000ffff);
	cmd_params.param[1] = ((S1D_PIP_ADDR>>16)&0x000003ff);
	s1d13522if_cmd(PIP_ADRCFG, &cmd_params, 2);

	cmd_params.param[0] = (0x3<<4)|(1<<10);
	s1d13522if_cmd(LD_IMG, &cmd_params, 1);
	cmd_params.param[0] = 0x154;
	s1d13522if_cmd(WR_REG, &cmd_params, 1);
	s1d13522if_BurstWrite16fb((u16 *)pSource, handle, size16);
	s1d13522if_cmd(LD_IMG_END, &cmd_params, 0);

	cmd_params.param[0] = left;
	cmd_params.param[1] = top;
	s1d13522if_cmd(PIP_XYSETUP, &cmd_params, 2);

	s1d13522if_WriteReg16(0x33a, 0xffff);
	cmd_params.param[0] = (waveform<<8);
	cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);
	cmd_params.param[1] = left;
	cmd_params.param[2] = top;
	cmd_params.param[3] = mWidth;
	cmd_params.param[4] = mHeight;

	s1d13522if_WriteReg16(0x232, 0x101);
	s1d13522if_cmd(cmd, &cmd_params, (cmd == UPD_INIT) ? 0 : 5);

	s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0);
	s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0);
	s1d13522if_WaitForHRDY();
	s1d13522if_WriteReg16(0x232, 0x100);
	mutex_unlock(&s1d13522fb_info.refresh_display_mutex);
	dbg_info("%s() exit\n", __func__);

}

static void s1d13522fb_refresh_PIP_area(struct fb_info *info,
					unsigned cmd, unsigned waveform,
					u16 alpha, u16 left, u16 top,
					u16 mWidth, u16 mHeight)
{
	s1d13522_ioctl_cmd_params cmd_params;
	struct s1d13522fb_par *par = info->par;

	u8 *pSource = (u8 *) (par->epd_buffer_host + bufferSize);
	dma_addr_t handle = (dma_addr_t)(DmaPhys + bufferSize);

	u16 lutsel = (waveform == WF_MODE_AUTO) ? 0xC4 : 0x84;

	dbg_info("%s()\n", __func__);

	mutex_lock(&s1d13522fb_info.refresh_display_mutex);

	s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0);
	s1d13522if_WriteReg16(0x330, lutsel);
	cmd_params.param[0] = (alpha<<12)|(0x02<<4)|(1<<8);
	cmd_params.param[1] = (mWidth+0x03)&~0x03;
	cmd_params.param[2] = mHeight;
	cmd_params.param[3] = 0x0000;
	s1d13522if_cmd(PIP_ENABLE, &cmd_params, 4);
	cmd_params.param[0] = (S1D_PIP_ADDR&0x0000ffff);
	cmd_params.param[1] = ((S1D_PIP_ADDR>>16)&0x000003ff);
	s1d13522if_cmd(PIP_ADRCFG, &cmd_params, 2);

	cmd_params.param[0] = (0x3<<4)|(1<<10);
	cmd_params.param[1] = 0x0000;
	cmd_params.param[2] = 0x0000;
	cmd_params.param[3] = (mWidth+0x03)&~0x03;
	cmd_params.param[4] = mHeight;
	s1d13522if_cmd(LD_IMG_AREA, &cmd_params, 5);
	cmd_params.param[0] = 0x154;
	s1d13522if_cmd(WR_REG, &cmd_params, 1);
	s1d13522if_BurstWrite16fb_Area(left, top, mWidth, mHeight,
					(u16 *)pSource, handle);
	s1d13522if_cmd(LD_IMG_END, &cmd_params, 0);

	cmd_params.param[0] = (alpha<<12)|(0x02<<4)|(1<<8);
	cmd_params.param[1] = mWidth;
	cmd_params.param[2] = mHeight;
	cmd_params.param[3] = 0x0000;
	s1d13522if_cmd(PIP_ENABLE, &cmd_params, 4);

	cmd_params.param[0] = left;
	cmd_params.param[1] = top;
	s1d13522if_cmd(PIP_XYSETUP, &cmd_params, 2);

	s1d13522if_WriteReg16(0x33a, 0xffff);
	cmd_params.param[0] = (waveform<<8);
	cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);
	cmd_params.param[1] = left;
	cmd_params.param[2] = top;
	cmd_params.param[3] = mWidth;
	cmd_params.param[4] = mHeight;

	s1d13522if_WriteReg16(0x232, 0x101);
	s1d13522if_cmd(cmd, &cmd_params, (cmd == UPD_INIT) ? 0 : 5);
	s1d13522if_WriteReg16(0x232, 0x100);

	mutex_unlock(&s1d13522fb_info.refresh_display_mutex);
	dbg_info("%s() exit\n", __func__);

}

static void s1d13522fb_movePIP_Position(unsigned cmd,
					unsigned waveform, u16 left, u16 top)
{
	s1d13522_ioctl_cmd_params cmd_params;
	u16 lutsel = (waveform == WF_MODE_AUTO) ? 0xC4 : 0x84;

	dbg_info("%s()\n", __func__);

	mutex_lock(&s1d13522fb_info.refresh_display_mutex);
	s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0);
	s1d13522if_WriteReg16(0x330, lutsel);
	cmd_params.param[0] = left;
	cmd_params.param[1] = top;
	s1d13522if_cmd(PIP_XYSETUP, &cmd_params, 2);

	s1d13522if_WriteReg16(0x33a, 0xffff);
	cmd_params.param[0] = (waveform<<8);
	cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);

	s1d13522if_WriteReg16(0x232, 0x101);
	s1d13522if_cmd(cmd, &cmd_params, (cmd == UPD_INIT) ? 0 : 1);
	s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0);
	s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0);
	s1d13522if_WaitForHRDY();
	s1d13522if_WriteReg16(0x232, 0x100);

	mutex_unlock(&s1d13522fb_info.refresh_display_mutex);
	dbg_info("%s() exit\n", __func__);
}

static void s1d13522fb_removePIP(unsigned waveform)
{
	s1d13522_ioctl_cmd_params cmd_params;
	u16 lutsel = (waveform == WF_MODE_AUTO) ? 0xC4 : 0x84;

	dbg_info("%s()\n", __func__);

	mutex_lock(&s1d13522fb_info.refresh_display_mutex);
	s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0);
	s1d13522if_WriteReg16(0x330, lutsel);
	s1d13522if_cmd(PIP_DISABLE, &cmd_params, 0);
	s1d13522if_WriteReg16(0x33a, 0xffff);
	cmd_params.param[0] = (waveform<<8);
	cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);

	s1d13522if_WriteReg16(0x232, 0x101);
	s1d13522if_cmd(UPD_PART, &cmd_params, 1);
	s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0);
	s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0);
	s1d13522if_WaitForHRDY();
	s1d13522if_WriteReg16(0x232, 0x100);

	mutex_unlock(&s1d13522fb_info.refresh_display_mutex);
	dbg_info("%s() exit\n", __func__);
}

static void s1d13522fb_display_init_wq(struct work_struct *work)
{
	u32 start, end;
	dbg_info("calling a work queue to initize display\n");
	start = jiffies_to_msecs(jiffies);
	s1d13522fb_display_init(UPD_FULL, WF_MODE_INIT, info->par);
	end = jiffies_to_msecs(jiffies);
	dbg_info("[%s()] Time taken for update: %d\n",
						__func__, (end-start));
}

static void s1d13522fb_display_bootlogo_wq(struct work_struct *work)
{
	u32 start, end;
	dbg_info("calling a work queue to display the image\n");
	start = jiffies_to_msecs(jiffies);
	s1d13522fb_display_init(UPD_FULL, WF_MODE_GC16, info->par);
	end = jiffies_to_msecs(jiffies);
	dbg_info("[%s()] Time taken for update: %d\n",
						__func__, (end-start));
}

void s1d13522fb_defaultDisplay(struct s1d13522fb_par *par)
{
        INIT_DELAYED_WORK(&(idw), s1d13522fb_display_init_wq);
        dbg_info("Staring the delayed display init seq %d\n", __LINE__);
        schedule_delayed_work(&(idw), msecs_to_jiffies(200));

        INIT_DELAYED_WORK(&(dw), s1d13522fb_display_bootlogo_wq);
        dbg_info("Starting the delayed display bootlogo seq %d\n", __LINE__);
        schedule_delayed_work(&(dw), msecs_to_jiffies(2*1000));
}

#if USE_PACKED_LOGO

int unpack_logo(unsigned char *src, size_t srcLen, unsigned char *dst, size_t dstLen) {
	size_t si=0, di=0;
	bool invert = (device_color == COLOR_WHITE) ? true : false;
	while (si<srcLen && di<dstLen) {
		int b=src[si];
		if (b==0) {
			size_t l=src[si+1]+(src[si+2]<<8)+(src[si+3]<<16);
			if (l>dstLen-di) l=dstLen-di;
			memset(&dst[di],invert ? 0xFF:0x00,l);
			di+=l; si+=4;
		} else {
			dst[di++]=invert ? ~b : b;
			si++;
		}
	}
	return di;
}

#endif


void s1d13522fb_display_init(unsigned cmd, unsigned mode, struct s1d13522fb_par *par)
{
	s1d13522_ioctl_cmd_params cmd_params;
	u32	reg330, size16 = par->epd_buffer_host_memorysize/2;
	u16	*pSource = (u16 *) par->epd_buffer_host;
	dma_addr_t handle = (dma_addr_t)DmaPhysH;

	dbg_info("[%s:%d] Inside s1d13522fb_display_init\n", __func__, __LINE__);
	mutex_lock(&(par->io_lock));
	/*#if defined(ARGB888_TO_GRAY4) || defined(RGB565_TO_GRAY4)
	s1d13522fb_colorconvert(par, 0);
	#endif
	*/

#if USE_PACKED_LOGO
	unpack_logo(epdbootlogo, sizeof(epdbootlogo), (unsigned char*)pSource, (S1D_DISPLAY_WIDTH * S1D_DISPLAY_HEIGHT * 4)/8);
#else
	memcpy(pSource, epdbootlogo, (epdbootlogo_len > ((S1D_DISPLAY_WIDTH * S1D_DISPLAY_HEIGHT * 4)/8) ? ((S1D_DISPLAY_WIDTH * S1D_DISPLAY_HEIGHT*4)/8) : epdbootlogo_len));
#endif
	if (systemRun() < 0) {
		printk(KERN_ERR "%s: systemRun error\n", __func__);
		mutex_unlock(&(par->io_lock));
		return;
	}
	dbg_info("[%s:%d] Start the update\n", __func__, __LINE__);
	reg330 = s1d13522if_ReadReg16(0x330);
	s1d13522if_WriteReg16(0x330, 0x84);
	cmd_params.param[0] = (0x2<<4);
	if (s1d13522if_cmd(LD_IMG, &cmd_params, 1) == -1) goto hrdyErr;
	cmd_params.param[0] = 0x154;
	if (s1d13522if_cmd(WR_REG, &cmd_params, 1) == -1) goto hrdyErr;
	s1d13522if_BurstWrite16fb(pSource, handle, size16/2);
	dbg_info("[%s:%d] Burst write the buffer\n", __func__, __LINE__);
	if (s1d13522if_cmd(LD_IMG_END, &cmd_params, 0) == -1) goto hrdyErr;
	memcpy(par->epd_buffer_host_current, par->epd_buffer_host, par->epd_buffer_host_memorysize);
	if (s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0) == -1) goto hrdyErr;
	if (s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0) == -1) goto hrdyErr;

	cmd_params.param[0] = (mode<<8);
	s1d13522if_WriteReg16(0x232, 0x101);
	if (s1d13522if_cmd(cmd, &cmd_params, 1) == -1) goto hrdyErr;
	//s1d13522if_WaitForHRDY();
	if (s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0) == -1) goto hrdyErr;
	if (s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0) == -1) goto hrdyErr;
	s1d13522if_WriteReg16(0x232, 0x100);

	reg330 &= 0xF0;
	reg330 |= 0x04;
	s1d13522if_WriteReg16(0x330, reg330);
	dbg_info("[%s:%d] Image update completed\n", __func__, __LINE__);
	s1d13522fb_info.display_changed = 0;
hrdyErr:
	if (systemSleep() < 0)
		printk(KERN_ERR "%s: systemSleep error\n", __func__);

	mutex_unlock(&(par->io_lock));
	return;
}

static ssize_t devColor_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return sprintf(buf, "0x%06X\n", device_color);
}

static ssize_t devColor_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	printk(KERN_ERR "Device color store is not supported\n");
	return count;
}

static ssize_t update_mode_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", updateModeSpecial);
}

static ssize_t update_mode_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	u32 mode;
	sscanf(buf, "%du", &mode);
	if ((mode >= UPD_INIT) && (mode <= UPD_PART_AREA)) {
		updateModeSpecial = mode;
		dbg_info("%s(): set updateMode to %d\n", __func__, mode);
	} else {
		updateModeSpecial = UPD_FULL;
		dbg_info("%s(): Invalid updateMode:%d\n", __func__, mode);
	}
	return count;
}

static ssize_t waveform_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", updateWaveformSpecial);
}

static ssize_t waveform_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	u32 wf;
	sscanf(buf, "%du", &wf);
	if ((wf >= WF_MODE_INIT) && (wf <= WF_MODE_AUTO)) {
		updateWaveformSpecial = wf;
		dbg_info("%s(): set updateWaveform to %d\n", __func__, wf);
	} else {
		updateWaveformSpecial = WF_MODE_GC16;
		dbg_info("%s(): Invalid updateWaveform:%d\n", __func__, wf);
	}
	return count;
}


static ssize_t temperature1_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	struct s1d13522_ioctl_temperature t;
	dbg_info("%s(): Get temperature for index: 1\n", __func__);
	t.index = 1;
	s1d13522fb_GetTemp(&t);
	dbg_info("%s(): Temperature retrieved for index:1 is %d\n",
					__func__, t.value[0]);

	return sprintf(buf, "%d\n", t.value[0]);
}

static ssize_t temperature2_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	struct s1d13522_ioctl_temperature t;
	dbg_info("%s(): Get temperature for index: 2\n", __func__);
	t.index = 2;
	s1d13522fb_GetTemp(&t);
	dbg_info("%s(): Temperature retrieved for index:2 is %d\n",
					__func__, t.value[1]);

	return sprintf(buf, "%d\n", t.value[1]);
}

static ssize_t temperature3_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	struct s1d13522_ioctl_temperature t;
	dbg_info("%s(): Get temperature for index: 3\n", __func__);
	t.index = 4;
	s1d13522fb_GetTemp(&t);
	dbg_info("%s(): Temperature retrieved for index:3 is %d\n",
					__func__, t.value[2]);

	return sprintf(buf, "%d\n", t.value[2]);
}

static ssize_t temperature_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	sscanf(buf, "%du", &temperatureW);
	s1d13522fb_SetTemp(temperatureW);
	return count;
}

static ssize_t temperatureD_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	temperature1 = temperature1 + temperature2 + temperature3;
	return 1;
}

static ssize_t temperatureD_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	return 1;
}

static ssize_t drawing_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	int drawing = 0;
	mutex_lock(&drawingEnabledMutex);
	drawing = drawingEnabled;
	mutex_unlock(&drawingEnabledMutex);

	dbg_info("%s: drawingEnabled=%d\n", __func__, drawing);

	return sprintf(buf, "%d\n", drawing);
}

static ssize_t drawing_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int drawing = 0;
	sscanf(buf, "%du", &drawing);

	mutex_lock(&drawingEnabledMutex);
	if (drawing == DRAW_ENABLE || drawing == DRAW_DISABLE) {
		drawingEnabled = drawing;
	}
	dbg_info("%s: drawingEnabled=%d\n", __func__, drawingEnabled);
	mutex_unlock(&drawingEnabledMutex);
	return count;
}

static ssize_t updateImage_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", updateImage);
}

static ssize_t updateImage_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	sscanf(buf, "%du", &updateImage);
	if (updateImage == 1) {
		update_only();
	}
	return count;
}

static ssize_t crcWaveform_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	crcWaveform = getCRCWaveform();
	return sprintf(buf, "%X\n", crcWaveform);
}

static ssize_t crcWaveform_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	sscanf(buf, "%du", &crcWaveformSize);
	if (crcWaveformSize < 0 || crcWaveformSize > 456568)
		crcWaveformSize = 200;
	return count;
}

static int s1d13522fb_tm_get_temp(struct thermal_zone_device *thermal,
				  unsigned long *temp)
{
	struct s1d13522fb_tm_sensor *tm_sensor = thermal->devdata;
	struct s1d13522_ioctl_temperature t;

	if (!tm_sensor || !temp)
	{
		return -40;
	}

	dbg_info("%s(): Get temperature for index: 1\n", __func__);
	t.index = tm_sensor->sensor_num;
	s1d13522fb_GetTemp(&t);

	*temp = t.value[tm_sensor->index] * 1000;

	return 0;
}

static int s1d13522fb_tm_get_mode(struct thermal_zone_device *thermal,
				  enum thermal_device_mode *mode)
{
	struct s1d13522fb_tm_sensor *tm_sensor = thermal->devdata;

	if (!tm_sensor || !mode)
		return -EINVAL;

	*mode = tm_sensor->mode;

	return 0;
}

static int s1d13522fb_tm_get_trip_type(struct thermal_zone_device *thermal,
				       int trip, enum thermal_trip_type *type)
{
	*type = THERMAL_TRIP_PASSIVE;
	return 0;
}

static int s1d13522fb_tm_activate_trip_type(struct thermal_zone_device *thermal,
			int trip, enum thermal_trip_activation_mode mode)
{
	return 0;
}

static int s1d13522fb_tm_get_trip_temp(struct thermal_zone_device *thermal,
				       int trip, unsigned long *temp)
{
	return 0;
}

static int s1d13522fb_tm_set_trip_temp(struct thermal_zone_device *thermal,
				       int trip, long temp)
{
	return 0;
}

static struct thermal_zone_device_ops epd_tm_thermal_ops = {
	.get_temp = s1d13522fb_tm_get_temp,
	.get_mode = s1d13522fb_tm_get_mode,
	.get_trip_type = s1d13522fb_tm_get_trip_type,
	.activate_trip_type = s1d13522fb_tm_activate_trip_type,
	.get_trip_temp = s1d13522fb_tm_get_trip_temp,
	.set_trip_temp = s1d13522fb_tm_set_trip_temp,
};

static void s1d13522fb_register_thermal_zones(void)
{
	int i;
	int counter = 1;

	tm_sensors = kzalloc(NUM_TM_SENSORS *
			sizeof(struct s1d13522fb_tm_sensor),
			GFP_ATOMIC);
	if (tm_sensors == NULL) {
		pr_err("%s: kzalloc() failed.\n", __func__);
		return;
	}

	for (i = 0; i < NUM_TM_SENSORS; i++) {
		char name[17];
		snprintf(name, sizeof(name), "s1d13522_sensor%d", i);
		tm_sensors[i].mode = THERMAL_DEVICE_ENABLED;
		tm_sensors[i].index = i;
		tm_sensors[i].sensor_num = counter;
		counter = counter * 2;
		tm_sensors[i].tz_dev = thermal_zone_device_register(name,
				0, &tm_sensors[i],
				&epd_tm_thermal_ops, 0, 0, 0, 0);
		if (IS_ERR(tm_sensors[i].tz_dev)) {
			pr_err("%s: thermal_zone_device_register() failed.\n", __func__);
			break;
		}
	}

	return;
}

static void s1d13522fb_unregister_thermal_zones(void)
{
	int i;

	if (tm_sensors != NULL) {
		for (i = 0; i < NUM_TM_SENSORS; i++) {
			thermal_zone_device_unregister(tm_sensors[i].tz_dev);
		}
		kfree(tm_sensors);
		tm_sensors = NULL;
	}
}

static struct kobject *epd_kobj;
static struct kobj_attribute devColor_attribute =
	__ATTR(devColor, 0666, devColor_show, devColor_store);

static struct kobj_attribute temperature1_attribute =
	__ATTR(temperature1, 0666, temperature1_show, temperatureD_store);
static struct kobj_attribute temperature2_attribute =
	__ATTR(temperature2, 0666, temperature2_show, temperatureD_store);
static struct kobj_attribute temperature3_attribute =
	__ATTR(temperature3, 0666, temperature3_show, temperatureD_store);
static struct kobj_attribute temperatureW_attribute =
	__ATTR(temperatureW, 0666, temperatureD_show, temperature_store);
static struct kobj_attribute drawing_attribute =
	__ATTR(drawing, 0666, drawing_show, drawing_store);
static struct kobj_attribute update_mode_attribute =
	__ATTR(update_mode, 0666, update_mode_show, update_mode_store);
static struct kobj_attribute waveform_attribute =
	__ATTR(waveform, 0666, waveform_show, waveform_store);


static struct kobj_attribute updateImage_attribute =
	__ATTR(updateImage, 0666, updateImage_show, updateImage_store);
static struct kobj_attribute crcWaveform_attribute =
	__ATTR(crcWaveform, 0666, crcWaveform_show, crcWaveform_store);


static struct attribute *epd_attrs[] = {
	&devColor_attribute.attr,
	&temperature1_attribute.attr,
	&temperature2_attribute.attr,
	&temperature3_attribute.attr,
	&temperatureW_attribute.attr,
	&drawing_attribute.attr,
	&update_mode_attribute.attr,
	&waveform_attribute.attr,
	&updateImage_attribute.attr,
	&crcWaveform_attribute.attr,
	NULL,
};
static struct attribute_group epd_attr_group = {
	.attrs = epd_attrs,
};

static int __devinit s1d13522_init(struct s1d13522fb_par *par)
{
	s1d13522_ioctl_cmd_params cmd_params;
	int ret;

	if (s1d13522if_InterfaceInit(&s1d13522fb_info) != 0) {
		printk(KERN_ERR "s1d13522fb_init: InterfaceInit error\n");
		return -EINVAL;
	}
	dbg_info("Calling s1d13522fb_EPDInit\n");

	if (s1d13522fb_EPDInit(0) == -1)
		return -ERROR;

	if (s1d13522fb_PMICInit() == -1)
		return -ERROR;
	par->regal_on = false;
	par->regal_d_on = false;
	par->panelw = S1D_DISPLAY_WIDTH;
	par->panelh = S1D_DISPLAY_HEIGHT;
	getVcomFromSF();
	getPanelInfoFromSF();
	if (epdCheck == -1)
		return -ERROR;
	s1d13522fb_registerInit();
	s1d13522fb_SetTemp(25);
	s1d13522if_WaitForHRDY();
	dbg_info("s1d13522 init done....\n");

	epd_kobj = kobject_create_and_add("epd", kernel_kobj);
	if (!epd_kobj)
		return -ENOMEM;
	ret = sysfs_create_group(epd_kobj, &epd_attr_group);
	if (ret)
		kobject_put(epd_kobj);
	drawingEnabled = DRAW_ENABLE;

#ifndef CONFIG_S1D13522_INIT_AUTOBOOT
	dbg_info("[%s:%d] Calling INIT_ROTMODE\n", __func__, __LINE__);
	cmd_params.param[0] = 0x100;
	if (s1d13522if_cmd(INIT_ROTMODE, &cmd_params, 1) == -1) return -ERROR;

	dbg_info("[%s:%d] Calling s1d13522fb_defaultDisplay\n",
						 __func__, __LINE__);
	s1d13522fb_defaultDisplay(par);
	s1d13522fb_register_thermal_zones();
	s1d13522fb_systemSleep(par);
	return 0;
#else
	return 0;
#endif
}

int s1d13522fb_re_init(void)
{
	s1d13522_ioctl_cmd_params cmd_params;
	poweron(0);
	mdelay(10);
	poweron(1);

	if (s1d13522if_InterfaceReInit(&s1d13522fb_info) != 0) {
		printk(KERN_ERR "s1d13522fb_re_init: InterfaceInit error\n");
		state = permanentError;
		return -1;
	}

	if (s1d13522fb_EPDInit(1) == -ERROR) {
		state = bitRecovered;
		return -2;
	}

	if (s1d13522fb_PMICInit() == -ERROR) {
		state = bitRecovered;
		return -2;
	}
	s1d13522fb_registerInit();
	s1d13522if_WriteReg16(0x0322, 25);

	cmd_params.param[0] = 0x100;
	if (s1d13522if_cmd(INIT_ROTMODE, &cmd_params, 1) == -1) return -ERROR;

	state = bitRecovered;
	return 0;
}


int s1d13522fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			 unsigned blue, unsigned transp, struct fb_info *info)
{
	dbg_info("%s(): regno=%02Xh red=%04Xh green=%04Xh blue=%04Xh transp=%04Xh\n",
			__func__, regno, red, green, blue, transp);

	if (regno >= 256) {
		printk(KERN_WARNING "%s(): reg index too large (%Xh)\n",
						 __func__, regno);
		return 1;
	}
	/* Make the first 16 LUT entries available to the console.*/
	if (regno < 16) {
#ifdef CONFIG_S1D13522_VFB_MONO8
		/* black on white method (G= 30%R + 59%G + 11%B) */
		unsigned gray = (red*30 + green*59 + blue*11)/100;
		gray = (gray>>8) & 0xFF;

		/* invert: black on white */
		gray = 0xFF-gray;
		((u32 *)info->pseudo_palette)[regno] = gray;

		dbg_info("%s(): regno=%02Xh red=%04Xh green=%04Xh blue=%04Xh transp=%04Xh ->gray=%02Xh\n",
				__func__, regno, red, green, blue, transp, gray);
#endif
#ifdef CONFIG_S1D13522_VFB_COLOR565
		unsigned color;
		color = (red & 0xf800)|((green & 0xfc00) >> 5)|((blue & 0xf800) >> 11);
		((u32 *)info->pseudo_palette)[regno] = 0xFFFF-color;
		dbg_info("%s(): regno=%02Xh red=%04Xh green=%04Xh blue=%04Xh transp=%04Xh ->color=%04Xh\n",
				__func__, regno, red, green, blue,
				transp, ((u32 *)info->pseudo_palette)[regno]);
#endif
	}
	return 0;
}

#ifdef ARGB888_TO_GRAY4
u8 getPixel(u32 val)
{
	u8 grayscale;
	u8 r, g, b;
	r = (u8)((0x00FF0000 & val) >> 16);
	g = (u8)((0x0000FF00 & val) >> 8);
	b = (u8)(0x000000FF & val);
	grayscale = (((r*30 + g*59 + b*11)/100));
	return grayscale>>4;
}

void s1d13522fb_colorconvert(struct s1d13522fb_par *par, int onlyWM)
{
	u32 strideSrc, strideDst;
	u32 *pSrc;
	//u8 *pDst;
	u16 width, height;
	int bufSize, grayscaleBuffer;
	u32 intlocation, mode, waveform;

	dbg_info("%s(): onlyWM: %d\n", __func__, onlyWM);
	strideSrc = par->info->fix.line_length;
	strideDst = strideSrc/2;
	pSrc = (u32 *) par->epd_buffer_host; //+ (par->dy1 * strideDst) + par->dx1;
	width  = par->dx2-par->dx1 + 1;
	height = par->dy2-par->dy1 + 1;

	width = S1D_DISPLAY_WIDTH;
	height = S1D_DISPLAY_HEIGHT;
	bufSize = width*height*4;
	grayscaleBuffer = width * height * 4 / 8;
	dmac_flush_range((void *)pSrc, (void *)pSrc + bufferSize);

	intlocation = waveform_mode_location/4;
	mode = pSrc[intlocation];
	waveform = pSrc[intlocation+1];
	dbg_info("Mode %d Waveform %d\n", mode, waveform);
	if ((mode == UPD_INIT) ||
		(mode == UPD_FULL) ||
		(mode == UPD_FULL_AREA) ||
		(mode == UPD_PART) ||
		(mode == UPD_PART_AREA))
		updateMode = mode;

	if ((waveform >= WF_MODE_INIT) && (waveform <= WF_MODE_AUTO))
		updateWaveform = waveform;
}
#endif

#ifdef RGB565_TO_GRAY4
u8 getPixel(u16 val16)
{
	u8 grayscale;
	u16 r, g, b;
	r = val16  & 0xF800;
	g = (val16 << 5) & 0xFC00;
	b = (val16 << 11) & 0xF800;
	grayscale = (((r*30 + g*59 + b*11)/100)>>12);

	return grayscale;
}

void s1d13522fb_colorconvert(struct s1d13522fb_par *par, int onlyWM)
{
	u32 strideSrc, strideDst;
	u16 *pSrc;
	u8 *pDst;
	u16 width, height;
	int bufSize, x, y;

	dbg_info("%s():\n", __func__);

	strideSrc = par->info->fix.line_length;
	strideDst = strideSrc/2;
	pSrc = (u16 *)(par->info->screen_base + (par->dy1 * strideSrc) + (par->dx1*2));
	pDst = (u8 *) par->epd_buffer_host + (par->dy1 * strideDst) + par->dx1;
	width  = par->dx2-par->dx1 + 1;
	height = par->dy2-par->dy1 + 1;
	width = S1D_DISPLAY_WIDTH;
	height = S1D_DISPLAY_HEIGHT;
	bufSize = width*height*4;

	dbg_info("dx1=%d,dy1=%d,width=%d,height=%d\n",
				par->dx1, par->dy1, width, height);

	for (y = 0, x = 0; y < bufSize; y += 2, x++)
			pDst[x] = getPixel(pSrc[y]) | (getPixel(pSrc[y+1]) << 4);
}
#endif

#ifdef CONFIG_S1D13522_VFB_COLOR565
void s1d13522fb_colorconvert_area(struct s1d13522fb_par *par)
{
	u32 strideSrc, strideDst, r, g, b;
	u16 val16, *pSrc;
	u8 *pDst;
	u16 width, height;
	int i, j;

	dbg_info("%s():\n", __func__);

	strideSrc = par->info->fix.line_length;
	strideDst = strideSrc/2;
	pSrc = (u16 *)(par->info->screen_base + (par->dy1 * strideSrc) + (par->dx1*2));
	pDst = (u8 *) par->epd_buffer_host + (par->dy1 * strideDst) + par->dx1;
	width  = par->dx2-par->dx1 + 1;
	height = par->dy2-par->dy1 + 1;
	width = S1D_DISPLAY_WIDTH;
	height = S1D_DISPLAY_HEIGHT;
	dbg_info("dx1=%d,dy1=%d,width=%d,height=%d\n",
				par->dx1, par->dy1, width, height);

	for (i = 0; i < height; i++) {
		for (j = 0; j < width; j++) {
			val16 = pSrc[j];
			r = val16  & 0xF800;
			g = (val16 << 5) & 0xFC00;
			b = (val16 << 11) & 0xF800;
			pDst[j] = (((r*30 + g*59 + b*11)/100)>>8);
		}

		pSrc += strideSrc/2;
		pDst += strideDst;
	}
}
#endif

void update_only(void)
{
	s1d13522_ioctl_cmd_params cmd_params;
	struct s1d13522fb_par *par = info->par;

	mutex_lock(&(par->io_lock));
	if (systemRun() < 0) {
		printk(KERN_ERR "%s: systemRun error\n", __func__);
		mutex_unlock(&(par->io_lock));
		return;
	}
	if (updateWaveformSpecial == WF_MODE_GLR16) {
		if (par->regal_on != true) {
			par->regal_on = true;
			s1d13522fb_regal_on(par);
		}
		s1d13522fb_regal_gb_process(par);
	} else if (updateWaveformSpecial == WF_MODE_GLD16) {
		if (par->regal_d_on != true) {
			par->regal_d_on = true;
			s1d13522fb_regal_d_on(par);
		}
		s1d13522fb_regal_d_gb_process(par);
	}

	if (s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0)  == -1) goto hrdyErr;
	if (s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0) == -1) goto hrdyErr;

	if (updateModeSpecial == UPD_FULL ||
	    updateModeSpecial == UPD_FULL_AREA) {
		if (!cyttsp5_is_suspended(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID))
			cyttsp5_suspend_scanning(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID, 1);
	}

	s1d13522if_WriteReg16(0x33a, 0xffff);
	cmd_params.param[0] = (updateWaveformSpecial<<8);

	s1d13522if_WriteReg16(0x232, 0x101);

	if (updateWaveformSpecial == WF_MODE_GLR16) {
		updateModeSpecial = UPD_FULL;
		cmd_params.param[0] = 0x0400;
		if (s1d13522if_cmd(updateModeSpecial, &cmd_params, 1) == -1) goto hrdyErr;
	} else if (updateWaveformSpecial == WF_MODE_GLD16) {
		updateModeSpecial = UPD_FULL;
		cmd_params.param[0] = 0x0500;
		if (s1d13522if_cmd(updateModeSpecial, &cmd_params, 1) == -1) goto hrdyErr;
	} else {
			if (s1d13522if_cmd(updateModeSpecial, &cmd_params, 1) == -1) goto hrdyErr;
	}
	if (s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0)  == -1) goto hrdyErr;
	if (s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0) == -1) goto hrdyErr;
	s1d13522if_WaitForHRDY();
hrdyErr:
	s1d13522if_WriteReg16(0x232, 0x100);
	if (updateModeSpecial == UPD_FULL ||
	    updateModeSpecial == UPD_FULL_AREA) {
		if (!cyttsp5_is_suspended(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID))
			cyttsp5_resume_scanning(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID, 1);
	}
	if (systemSleep() < 0)
		printk(KERN_ERR "%s: systemSleep error\n", __func__);
	mutex_unlock(&(par->io_lock));
	return;
}
static void s1d13522fb_dpy_update_pages(struct s1d13522fb_par *par,
					int offset, int numpages)
{
	u16 y, height;
	u32 stride = (par->panelw*S1D13522_VFB_BPP)/8;
	s1d13522_ioctl_cmd_params cmd_params;
	//u16     *pSource = (u16 *) par->info->screen_base;
	u16     *pSource = (u16 *) par->epd_buffer_host;
	dma_addr_t handle = (dma_addr_t)DmaPhys;
	u32 size16 = (par->panelw * par->panelh * S1D13522_EPD_BPP / 8);

	dbg_info("%s(): pages to update(%d), buf:0x%X,stride:%d offset:%d\n",
		 __func__, numpages, (u32)par->epd_buffer_host, stride, offset);


	mutex_lock(&(par->io_lock));
	if (systemRun() < 0) {
		printk(KERN_ERR "%s: systemRun error\n", __func__);
		mutex_unlock(&(par->io_lock));
		return;
	}

#if defined(ARGB888_TO_GRAY4) || defined(RGB565_TO_GRAY4)
	if (numpages == 1 && offset >= waveform_mode_location) {
		s1d13522fb_colorconvert(par, 1);
		dbg_info("%s [No image data] offset:%d wf_mode_location:%d",
				__func__, offset, waveform_mode_location);
		goto updatedone;
	} else {
		s1d13522fb_colorconvert(par, 0);
	}
#endif

        if ((lastWaveform == WF_MODE_GLR16) && (updateWaveform != WF_MODE_GLR16)) {
                s1d13522fb_regal_off(par);
                par->regal_on = false;
        }
        else if ((lastWaveform == WF_MODE_GLD16) && (updateWaveform != WF_MODE_GLD16)) {
                s1d13522fb_regal_d_off(par);
                par->regal_d_on = false;
        }
        lastWaveform = updateWaveform;

	if (updateWaveform == WF_MODE_GLR16) {
		dbg_info("%s() par->regal_on is true!\n", __func__);
		if (par->regal_on != true) {
			s1d13522fb_regal_on(par);
			par->regal_on = true;
		}
		s1d13522fb_regal_gb_process(par);
		pSource = (u16*) par->epd_buffer_host;
	} else if (updateWaveform == WF_MODE_GLD16) {
		dbg_info("s1d13522fb_display_update(): par->regal_d_on is true!\n");
		if (par->regal_d_on != true) {
			s1d13522fb_regal_d_on(par);
			par->regal_d_on = true;
		}
		s1d13522fb_regal_d_gb_process(par);
		pSource = (u16*) par->epd_buffer_host;
	}

	y = offset/stride;
	height = ((numpages*PAGE_SIZE) / stride);

	if (y + height > par->panelh)
		height = par->panelh - y;

	dbg_info("%s(): y: %d height:%d par->panelw:%d\n", __func__, y, height, par->panelw);

	if (offset == 0 && numpages >= numPage) {
		cmd_params.param[0] = (0x3<<4);
		if (s1d13522if_cmd(LD_IMG, &cmd_params, 1) == -1) goto hrdyErr;
		cmd_params.param[0] = 0x154;
		if (s1d13522if_cmd(WR_REG, &cmd_params, 1) == -1) goto hrdyErr;
		s1d13522if_BurstWrite16fb(pSource, handle, size16/2);
	} else {
		cmd_params.param[0] = (0x3<<4);
		cmd_params.param[1] = 0;
		cmd_params.param[2] = y;
		cmd_params.param[3] = par->panelw;
		cmd_params.param[4] = height;
		if (s1d13522if_cmd(LD_IMG_AREA, &cmd_params, 5) == -1) goto hrdyErr;
		cmd_params.param[0] = 0x154;
		if (s1d13522if_cmd(WR_REG, &cmd_params, 1) == -1) goto hrdyErr;
		s1d13522if_BurstWrite16fb_Area(0, y, par->panelw, height,
				pSource, handle);
	}
	if (s1d13522if_cmd(LD_IMG_END, &cmd_params, 0) == -1) goto hrdyErr;
	if (s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0)  == -1) goto hrdyErr;
	if (s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0) == -1) goto hrdyErr;

	if (updateMode == UPD_FULL ||
	    updateMode == UPD_FULL_AREA) {
		if (!cyttsp5_is_suspended(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID))
			cyttsp5_suspend_scanning(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID, 1);
	}

	s1d13522if_WriteReg16(0x33a, 0xffff);
	cmd_params.param[0] = (updateWaveform<<8);
	cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);

	s1d13522if_WriteReg16(0x232, 0x101);

	if (updateWaveform == WF_MODE_GLR16) {
		updateMode = UPD_FULL;
		cmd_params.param[0] = 0x0400;
		cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);
		if (s1d13522if_cmd(updateMode, &cmd_params, 1) == -1) goto hrdyErr;
	} else if (updateWaveform == WF_MODE_GLD16) {
		updateMode = UPD_FULL;
		cmd_params.param[0] = 0x0500;
		cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);
		if (s1d13522if_cmd(updateMode, &cmd_params, 1) == -1) goto hrdyErr;
	} else {
		dbg_info("%s(): par->regal_on is false\n", __func__);
		if (updateMode == UPD_FULL_AREA ||
			updateMode == UPD_PART_AREA) {
			cmd_params.param[1] = 0;
			cmd_params.param[2] = y;
			cmd_params.param[3] = par->panelw;
			cmd_params.param[4] = height;
			dbg_info("%s x=0 y=%d width=%d height=%d Mode:%d\n",
					__func__, y, par->panelw,
					height, updateMode);
			if (s1d13522if_cmd(updateMode, &cmd_params, 5) == -1) goto hrdyErr;
		} else {
			dbg_info("~~~> %s Mode: %d waveform %d\n", __func__,
					updateMode, updateWaveform);
			if (s1d13522if_cmd(updateMode, &cmd_params, 1) == -1) goto hrdyErr;
		}
	}
#if 0
	if (numpages < S1D13522_HEURISTICS_FULL_UPDATE) {
		dbg_info("%s(): UPD_PART screen update\n", __func__);
		if (s1d13522if_cmd(UPD_PART, &cmd_params, 5) == -1) goto hrdyErr;
	} else {
		dbg_info("%s(): UPD_FULL screen update\n", __func__);
		if (s1d13522if_cmd(UPD_PART, &cmd_params, 1) == -1) goto hrdyErr;
	}
#endif
	if (s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0)  == -1) goto hrdyErr;
	memcpy(par->epd_buffer_host_current, par->epd_buffer_host, par->epd_buffer_host_memorysize);

updatedone:
hrdyErr:
	if (updateMode == UPD_FULL ||
	    updateMode == UPD_FULL_AREA) {
		s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0);
		s1d13522if_WaitForHRDY();
		if (!cyttsp5_is_suspended(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID))
			cyttsp5_resume_scanning(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID, 1);
	}
	if (systemSleep() < 0) {
		printk(KERN_ERR "%s: systemSleep error\n", __func__);
	}

	mutex_unlock(&(par->io_lock));
	return;
}


void s1d13522fb_dpy_update(struct s1d13522fb_par *par)
{
	u32 size16;
	s1d13522_ioctl_cmd_params cmd_params;
	u16	*pSource = (u16 *) par->epd_buffer_host;
	dma_addr_t handle = (dma_addr_t)DmaPhysH;
	u32 start, end;

	size16 = ((S1D_DISPLAY_WIDTH * S1D_DISPLAY_HEIGHT * S1D13522_EPD_BPP)/8);
	dbg_info("%s: wakeup_count=%d\n", __func__, wakeup_count);

	mutex_lock(&(par->io_lock));

	start = jiffies_to_msecs(jiffies);
	if (systemRun() < 0) {
		printk(KERN_ERR "%s: systemRun error\n", __func__);
		mutex_unlock(&(par->io_lock));
		return;
	}

#if defined(ARGB888_TO_GRAY4) || defined(RGB565_TO_GRAY4)
	s1d13522fb_colorconvert(par, 0);
#endif

	if (updateWaveform == WF_MODE_GLR16) {
		dbg_info("s1d13522fb_display_update:par->regal_on is true!\n");
		if (par->regal_on != true) {
			par->regal_on = true;
			s1d13522fb_regal_on(par);
		}
		s1d13522fb_regal_gb_process(par);
	} else if (updateWaveform == WF_MODE_GLD16) {
		dbg_info("s1d13522fb_display_update par->regal_d_on is true\n");
		if (par->regal_d_on != true) {
			par->regal_d_on = true;
			s1d13522fb_regal_d_on(par);
		}
		s1d13522fb_regal_d_gb_process(par);
	}

	if (s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0) == -1) goto hrdyErr;
	cmd_params.param[0] = (0x3<<4);
	if (s1d13522if_cmd(LD_IMG, &cmd_params, 1) == -1) goto hrdyErr;
	cmd_params.param[0] = 0x154;
	if (s1d13522if_cmd(WR_REG, &cmd_params, 1) == -1) goto hrdyErr;
	s1d13522if_BurstWrite16fb(pSource, handle, size16);
	if (s1d13522if_cmd(LD_IMG_END, &cmd_params, 0) == -1) goto hrdyErr;

	if (updateMode == UPD_FULL ||
	    updateMode == UPD_FULL_AREA) {
		if (!cyttsp5_is_suspended(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID))
			cyttsp5_suspend_scanning(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID, 1);
	}

	s1d13522if_WriteReg16(0x33a, 0xffff);
	cmd_params.param[0] = (updateWaveform<<8);
	cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);

	s1d13522if_WriteReg16(0x232, 0x101);

	if (updateWaveform == WF_MODE_GLR16) {
		updateMode = UPD_FULL;
		cmd_params.param[0] = 0x0400;
		cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);
		s1d13522if_cmd(updateMode, &cmd_params, 1);
	} else if (updateWaveform == WF_MODE_GLD16) {
		updateMode = UPD_FULL;
		cmd_params.param[0] = 0x0500;
		cmd_params.param[0] = checkBorderUpdate(cmd_params.param[0]);
		s1d13522if_cmd(updateMode, &cmd_params, 1);
	} else {
		dbg_info("s1d13522fb_display_update: par->regal_on is false\n");
		s1d13522if_cmd(updateMode, &cmd_params, 1);
	}

	if (s1d13522if_cmd(WAIT_DSPE_TRG, &cmd_params, 0)  == -1) goto hrdyErr;
	if (s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0) == -1) goto hrdyErr;
	s1d13522if_WaitForHRDY();
	s1d13522if_WriteReg16(0x232, 0x100);

	memcpy(par->epd_buffer_host_current, par->epd_buffer_host, par->epd_buffer_host_memorysize);

hrdyErr:
	if (updateMode == UPD_FULL ||
	    updateMode == UPD_FULL_AREA) {
		s1d13522if_cmd(WAIT_DSPE_FREND, &cmd_params, 0);
		s1d13522if_WaitForHRDY();
		if (!cyttsp5_is_suspended(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID))
			cyttsp5_resume_scanning(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID, 1);
	}
	if (systemSleep() < 0) {
		printk(KERN_ERR "%s: systemSleep error\n", __func__);
	}

	end = jiffies_to_msecs(jiffies);
	dbg_info("[%s()] Time taken to update screen: %d\n",
					__func__, (end-start));
	mutex_unlock(&(par->io_lock));
	return;
}

void s1d13522fb_dpy_update_area(struct s1d13522fb_par *par)
{
	u32 offset;
	u32 stride = par->panelw;
	u8 *pSource;
	u16 args[5];
	u16 width, height;
	int i;

	dbg_info("%s(): starting update...\n", __func__);

#ifndef CONFIG_S1D13522_VFB_MONO8
/*	s1d13522fb_colorconvert_area(par); */
#endif

	offset = (par->dy1*stride) + ((par->dx1) * (S1D13522_EPD_BPP/8));
	pSource = (u8 *) par->epd_buffer_host + offset;
	width = (par->dx2-par->dx1 + 2) & 0xFFFE;
	height = par->dy2-par->dy1 + 1;

	mutex_lock(&(par->io_lock));
	par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_TRG);
	par->board->send_command(par, S1D13522_CMD_WAIT_DSPE_FREND);
	args[0] = 0x30;		/*1 byte per pixel;*/
	args[1] = par->dx1;
	args[2] = par->dy1;
	args[3] = width;
	args[4] = height;
	par->board->send_cmdargs(par, S1D13522_CMD_LD_IMG_AREA, 5, args);

	args[0] = 0x154;
	par->board->send_cmdargs(par, S1D13522_CMD_WR_REG, 1, args);
	for (i = 0; i < height; i++) {
		par->board->burst_write(par, ((width*S1D13522_EPD_BPP)/8)/2, (u16 *) pSource);
		pSource = pSource + stride;
	}
	par->board->send_command(par, S1D13522_CMD_LD_IMG_END);

	args[0] = 0x4300;
	par->board->send_cmdargs(par, S1D13522_CMD_UPD_FULL_AREA, 5, args);
	memcpy(par->epd_buffer_host_current, par->epd_buffer_host, par->epd_buffer_host_memorysize);
	mutex_unlock(&(par->io_lock));
}

static void s1d13522fb_flush_dirty_rect(struct s1d13522fb_par *par)
{
	/* Optimization: detect the whole screen update */
	if (par->dx1 == 0 && par->dx2 == par->panelw &&
		par->dy1 == 0 && par->dy2 == par->panelh) {
		s1d13522fb_dpy_update(par);
	} else {
		s1d13522fb_dpy_update_area(par);
	}
}

static void s1d13522fb_dpy_deferred_io(struct fb_info *info,
					struct list_head *pagelist)
{
	/* It's possible to get here without anything on the pagelist via
	 s1d13522_deferred_io_touch() or via a userspace fsync() invocation. */
	struct s1d13522fb_par *par = info->par;
	u32 start, end;

	start = jiffies_to_msecs(jiffies);

	mutex_lock(&vsync_mutex);
	update_in_progress = 1;
	mutex_unlock(&vsync_mutex);

	dbg_info("~~~!!!%s(): fb_deferred_io initialized\n", __func__);

	mutex_lock(&drawingEnabledMutex);
	if (drawingEnabled == DRAW_DISABLE) {
		mutex_unlock(&drawingEnabledMutex);
		printk(KERN_ERR "Drawing disabled through sysfs entry\n");
		return;
	}
	mutex_unlock(&drawingEnabledMutex);

	if (epdCheck == -1) {
		printk(KERN_ERR "EPD Panel not Connected or Serial Flash corrupted\n");
		return;
	}

	if (!list_empty(pagelist)) {
		int consecutive_pages = 0, dirty_pages = 0;
		int prev_index = -1;
		struct page *cur, *first = NULL;
		struct fb_deferred_io *fbdefio = info->fbdefio;

		list_for_each_entry(cur, &fbdefio->pagelist, lru) {
			dirty_pages++;

			if (prev_index == -1) {
				first = cur;
				consecutive_pages = 1;
			} else if (prev_index == cur->index - 1) {
				consecutive_pages++;
			} else {
				/*Found a non-consecutive page.
				Update all consecutive pages first*/
				s1d13522fb_dpy_update_pages(par,
					(first->index << PAGE_SHIFT),
					consecutive_pages);
				first = cur;
				consecutive_pages = 1;
			}

			prev_index = cur->index;
		}
		/* Last chunk of consecutive pages */
		if (consecutive_pages) {
			s1d13522fb_dpy_update_pages(par,
				(first->index << PAGE_SHIFT),
				consecutive_pages);
		}
	} else {
		if (par->dirtyrange) {
			/* Process memory region created by io_touch()*/
			s1d13522fb_flush_dirty_rect(par);
			par->dirtyrange = 0;
		} else {
			dbg_info("s1d13522fb_deferred_io: NO PAGES\n");
			s1d13522fb_dpy_update(par);
		}
	}

	mutex_lock(&vsync_mutex);
	update_in_progress = 0;
	par->vsync_flag = 1;
	wake_up_interruptible(&par->vsync_wait);
	mutex_unlock(&vsync_mutex);

	end = jiffies_to_msecs(jiffies);
	dbg_info("[%s()] Time taken to update screen: %d\n",
						__func__, (end-start));
}

struct fb_rect {
	__u32 dx;	/* screen-relative */
	__u32 dy;
	__u32 width;
	__u32 height;
};


static void s1d13522fb_deferred_io_touch(struct s1d13522fb_par *par,
						const struct fb_rect *rect)
{
	struct fb_deferred_io *fbdefio = par->info->fbdefio;

	if (fbdefio) {
		if (par->dirtyrange == 0) {
			par->dx1 = rect->dx;
			par->dy1 = rect->dy;
			par->dx2 = rect->dx+rect->width;
			par->dy2 = rect->dy+rect->height;
		} else {
			/* Merge the new dirty range with the current one */
			if (rect->dx < par->dx1)
				par->dx1 = rect->dx;
			if (rect->dy < par->dy1)
				par->dy1 = rect->dy;
			if (rect->dx+rect->width > par->dx2)
				par->dx2 = rect->dx+rect->width;
			if (rect->dy+rect->height > par->dy2)
				par->dy2 = rect->dy+rect->height;
		}
		par->dirtyrange = 1;
		schedule_delayed_work(&par->info->deferred_work, HZ/2);
	}
}

void s1d13522fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct s1d13522fb_par *par = info->par;
	sys_fillrect(info, rect);
	s1d13522fb_deferred_io_touch(par, (const struct fb_rect *) rect);
}

void s1d13522fb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	struct s1d13522fb_par *par = info->par;
	sys_copyarea(info, area);
	s1d13522fb_deferred_io_touch(par, (const struct fb_rect *) area);
}

void s1d13522fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct s1d13522fb_par *par = info->par;
	dbg_info("%s() x:%d y:%d, [%d x %d]\n", __func__, image->dx,
				image->dy, image->width, image->height);
	sys_imageblit(info, image);
	s1d13522fb_deferred_io_touch(par, (const struct fb_rect *) image);
}

static ssize_t s1d13522fb_write(struct fb_info *info, const char __user *buf,
					size_t count, loff_t *ppos)
{
	struct s1d13522fb_par *par = info->par;
	unsigned long p = *ppos;
	void *dst;
	int err = 0;
	unsigned long total_size;
	if (info->state != FBINFO_STATE_RUNNING)
		return -EPERM;
	total_size = info->fix.smem_len;
	if (p > total_size)
		return -EFBIG;
	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}
	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;

		count = total_size - p;
	}
	dst = (void *)(info->screen_base + p);
	if (copy_from_user(dst, buf, count))
		err = -EFAULT;
	if  (!err)
		*ppos += count;
	dbg_info("~~~> %s(): count:%d p:%ld total_size:%ld\n", __func__,
						count, p, total_size);
	s1d13522fb_display_init(UPD_FULL, WF_MODE_GC16, par);
	return (err) ? err : count;
}
/*
 * Function to wait for vertical sync which for this EPD peripheral
 * translates into waiting for the current raster frame to complete.
 */
static int fb_wait_for_vsync(struct fb_info *info)
{
	struct s1d13522fb_par *par = info->par;
	int ret;

	mutex_lock(&drawingEnabledMutex);
	if (drawingEnabled == DRAW_DISABLE) {
		mutex_unlock(&drawingEnabledMutex);
		printk(KERN_ERR "fb_wait_for_vsync() disabled through sysfs entry\n");
		return -EINVAL;
	}
	mutex_unlock(&drawingEnabledMutex);

	mutex_lock(&vsync_mutex);
	if (!update_in_progress) {
		mutex_unlock(&vsync_mutex);
		return 1;
	}
	par->vsync_flag = 0;
	mutex_unlock(&vsync_mutex);

	dbg_info("%s: start wait for vsync\n", __func__);
	ret = wait_event_interruptible_timeout(par->vsync_wait,
					       par->vsync_flag != 0,
					       par->vsync_timeout);
	dbg_info("%s: end wait for vsync: ret=%d\n", __func__, ret);

	if (ret < 0)
		return ret;
	if (ret == 0) {
		printk(KERN_ERR "%s: VSYNC timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}

#ifdef CONFIG_S1D13522_IOCTL
static int s1d13522fb_ioctl(struct fb_info *info, unsigned int cmd,
			    unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct s1d13522_ioctl_hwc ioctl_hwc = {0, 0, NULL};
	struct s1d13522_ioctl_temperature ioctl_t;
	s1d13522_ioctl_cmd_params cmd_params;
	ioctl_cmd_par_signed signed_params;
	int retVal = 0;
	struct s1d13522fb_par *par = info->par;

	dbg_info("%s:cmd=%04Xh wakeup_count:%d HRDYState:%d\n",
		 __func__, cmd, wakeup_count, state);

	if (s1d13522_programming && ((cmd != S1D13522_PRG_INIT) &&
		(cmd != S1D13522_PRG_FLASH) && (cmd != S1D13522_PRG_EXIT))) {
		return 0;
	}

	if (state == bitStuckAgain || state == permanentError) {
		state = retry;
		/*retVal = s1d13522fb_re_init(); */
		if (retVal == -1 || retVal == -2) {
			state = permanentError;
			return -EINVAL;
		}
	}

	if (cmd == S1D13522_GET_PHONE_COLOR)
		return device_color;

	if ((cmd == S1D13522_RUN_SYS && (!wakeup_count)) ||
	    (cmd == S1D13522_SLP && wakeup_count))
		goto L1;
	else if (!wakeup_count) {
		dbg_info("%s()[%d]: Command[%04Xh] Not allowed, wakeup_count=%d\n",
			 __func__, __LINE__, cmd, wakeup_count);
		return -EINVAL;
	}

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
               return fb_wait_for_vsync(info);
	case S1D13522_WR_REG:
	case S1D13522_RD_REG:
		break;
	case S1D13522_GET_TEMP:
	case S1D13522_SET_TEMP:
		break;

	default:
		if (epdCheck == -1)
			return -EINVAL;
	}

L1:
	switch (cmd) {

	case S1D13522_VBUF_REFRESH_GLOBAL:
		if (copy_from_user(&cmd_params, argp, 2*sizeof(u16)))
			return -EFAULT;
		s1d13522fb_refresh_global(info, cmd_params.param[0],
			cmd_params.param[1]);
		break;

	case S1D13522_VBUF_REFRESH_AREA:
		if (copy_from_user(&cmd_params, argp, 6*sizeof(u16)))
			return -EFAULT;

		s1d13522fb_refresh_area(info, cmd_params.param[0],
			cmd_params.param[1], cmd_params.param[2],
			cmd_params.param[3], cmd_params.param[4],
			cmd_params.param[5]);
		break;
	case S1D13522_UPDATE_AREA: {
		S1D13522_UPDATE_AREA_params_t p;
		u_int8_t *img = NULL;

		if (copy_from_user(&p, argp, sizeof(p))) return -EFAULT;

		img = s1d13522_get_mem_obj(par->malloc_cache, (p.width*p.height));
		if (!img)
			return -ENOMEM;
		if (copy_from_user(img, p.imgdata, p.width*p.height)) {
			s1d13522_put_mem_obj(par->malloc_cache, img);
			return -EFAULT;
		}
		retVal = s1d13522fb_update_area(info, p.cmd, p.waveform,
					        p.left, p.top,
					        p.width, p.height, img);
		s1d13522_put_mem_obj(par->malloc_cache, img);

		return retVal;
	}

	case S1D13522_VBUF_REFRESH_PIP_GLOBAL:
		if (copy_from_user(&cmd_params, argp, 7*sizeof(u16)))
			return -EFAULT;
		s1d13522fb_refresh_PIP_global(info, cmd_params.param[0],
			cmd_params.param[1], cmd_params.param[2],
			cmd_params.param[3], cmd_params.param[4],
			cmd_params.param[5], cmd_params.param[6]);
		break;

	case S1D13522_VBUF_REFRESH_PIP_AREA:
		if (copy_from_user(&cmd_params, argp, 7*sizeof(u16)))
			return -EFAULT;
		s1d13522fb_refresh_PIP_area(info, cmd_params.param[0],
			cmd_params.param[1], cmd_params.param[2],
			cmd_params.param[3], cmd_params.param[4],
			cmd_params.param[5], cmd_params.param[6]);
		break;

	case S1D13522_MOVE_PIP_POSITION:
		if (copy_from_user(&cmd_params, argp, 4*sizeof(u16)))
			return -EFAULT;
		s1d13522fb_movePIP_Position(cmd_params.param[0],
			cmd_params.param[1], cmd_params.param[2],
			cmd_params.param[3]);
			break;

	case S1D13522_REMOVE_PIP:
		if (copy_from_user(&cmd_params, argp, sizeof(u16)))
			return -EFAULT;
		s1d13522fb_removePIP(cmd_params.param[0]);
		break;

	case S1D13522_CURSOR_ENABLE:
		if (copy_from_user(&cmd_params, argp, 7*sizeof(u16)))
			return -EFAULT;
		dbg_info("--> Got the data for S1D13522_CURSOR_ENABLE\n");
		s1d13522fb_cursor_enable(info, cmd_params.param[0],
				cmd_params.param[1], cmd_params.param[2],
				cmd_params.param[3], cmd_params.param[4],
				cmd_params.param[5], cmd_params.param[6]);
		break;

	case S1D13522_CURSOR_MOVE:
		if (copy_from_user(&cmd_params, argp, 2*sizeof(u16)))
			return -EFAULT;
		dbg_info("--> Got the data for S1D13522_CURSOR_MOVE\n");
		s1d13522fb_cursor_move(cmd_params.param[0],
					cmd_params.param[1]);
		break;

	case S1D13522_CURSOR_DISABLE:
		if (copy_from_user(&cmd_params, argp, 1*sizeof(u16)))
			return -EFAULT;
		dbg_info("--> Got the data for S1D13522_CURSOR_DISABLE\n");
		s1d13522fb_cursor_disable(cmd_params.param[0]);
		break;

	case S1D13522_GET_TEMP:
		if (copy_from_user(&ioctl_t, argp, sizeof(ioctl_t)))
			return -EFAULT;
		s1d13522fb_GetTemp(&ioctl_t);
		return copy_to_user(argp, &ioctl_t, sizeof(ioctl_t)) ? -EFAULT : 0;

	case S1D13522_SET_TEMP:
		if (copy_from_user(&signed_params, argp, sizeof(u16)))
			return -EFAULT;
		s1d13522fb_SetTemp(signed_params.param[0]);
		break;

	case S1D13522_VBUF_REFRESH:
		if (copy_from_user(&cmd_params, argp, 2*sizeof(u16)))
			return -EFAULT;

		s1d13522fb_dpy_update(par);
		break;

	case S1D13522_SET_BORDER_COLOR:
		if (copy_from_user(&cmd_params, argp, sizeof(u16)))
			return -EFAULT;
		s1d13522fb_SetBorderColor(cmd_params.param[0]);
		break;

	case S1D13522_UPDATE_BORDER:
		if (copy_from_user(&cmd_params, argp, sizeof(u16)))
			return -EFAULT;
		s1d13522fb_UpdateBorder(cmd_params.param[0]);
		break;

		/* zero parameter commands: */
	case S1D13522_RUN_SYS:
		s1d13522fb_systemRun(par);
		break;
	case S1D13522_STBY:
	case S1D13522_SLP:
		s1d13522fb_systemSleep(par);
		break;

	case S1D13522_INIT_SYS_RUN:
		mutex_lock(&(par->io_lock));
		s1d13522if_cmd(cmd, &cmd_params, 0);
		wakeup_count++;
		mutex_unlock(&(par->io_lock));
		break;

	case S1D13522_BST_END_SDR:
	case S1D13522_LD_IMG_END:
	case S1D13522_LD_IMG_WAIT:
	case S1D13522_LD_IMG_DSPEADR:
	case S1D13522_WAIT_DSPE_TRG:
	case S1D13522_WAIT_DSPE_FREND:
	case S1D13522_WAIT_DSPE_LUTFREE:
	case S1D13522_UPD_INIT:
	case S1D13522_UPD_GDRV_CLR:
	case S1D13522_PIP_DISABLE:
		s1d13522if_cmd(cmd, &cmd_params, 0);
		break;

	/* one parameter commands */
	case S1D13522_INIT_ROTMODE:
	case S1D13522_INIT_WAVEDEV:
	case S1D13522_LD_IMG:
	case S1D13522_WAIT_DSPE_MLUTFREE:
	case S1D13522_UPD_FULL:
	case S1D13522_UPD_PART:
	case S1D13522_CSR_ADRCFG:
		if (copy_from_user(&cmd_params, argp, 1*sizeof(u16)))
			return -EFAULT;

		s1d13522if_cmd(cmd, &cmd_params, 1);
		break;

	/* two parameter commands */
	case S1D13522_WR_REG:
	case S1D13522_LD_IMG_SETADR:
	case S1D13522_RD_WFM_INFO:
	case S1D13522_UPD_SET_IMGADR:
	case S1D13522_PIP_ADRCFG:
	case S1D13522_PIP_XYSETUP:
	case S1D13522_CSR_XYSETUP:
		if (copy_from_user(&cmd_params, argp, 2*sizeof(u16)))
			return -EFAULT;

		if (cmd == S1D13522_WR_REG && cmd_params.param[0] == 0x154)
			s1d13522if_cmd(cmd, &cmd_params, 1);
		else
			s1d13522if_cmd(cmd, &cmd_params, 2);
		break;

	/* four parameter commands */
	case S1D13522_PIP_ENABLE:
	case S1D13522_BST_RD_SDR:
	case S1D13522_BST_WR_SDR:
	case S1D13522_CSR_MAINCFG:
		if (copy_from_user(&cmd_params, argp, 4*sizeof(u16)))
			return -EFAULT;

		s1d13522if_cmd(cmd, &cmd_params, 4);
		break;

	/* five parameter commands */
	case S1D13522_INIT_DSPE_CFG:
	case S1D13522_INIT_DSPE_TMG:
	case S1D13522_LD_IMG_AREA:
	case S1D13522_UPD_FULL_AREA:
	case S1D13522_UPD_PART_AREA:
		if (copy_from_user(&cmd_params, argp, 5*sizeof(u16)))
			return -EFAULT;

		s1d13522if_cmd(cmd, &cmd_params, 5);
		break;

	case S1D13522_RD_REG:
			if (copy_from_user(&cmd_params, argp, 2*sizeof(u16)))
				return -EFAULT;

			cmd_params.param[1] = s1d13522if_ReadReg16(cmd_params.param[1]);
			return copy_to_user(argp, &cmd_params, 2*sizeof(u16)) ? -EFAULT : 0;

	case S1D13522_REGREAD:
		if (copy_from_user(&ioctl_hwc, argp, sizeof(ioctl_hwc)))
			return -EFAULT;

		ioctl_hwc.value = s1d13522if_ReadReg16(ioctl_hwc.addr);
		return copy_to_user(argp, &ioctl_hwc, sizeof(ioctl_hwc)) ? -EFAULT : 0;

	case S1D13522_REGWRITE:
		if (copy_from_user(&ioctl_hwc, argp, sizeof(ioctl_hwc)))
			return -EFAULT;

		s1d13522if_WriteReg16(ioctl_hwc.addr, ioctl_hwc.value);
		return 0;

	case S1D13522_MEMBURSTWRITE:
	{
		u8 buffer[512];
		unsigned user_buflen, copysize, copysize16;
		u16 *ptr16;
		u8 *user_buffer;

		if (copy_from_user(&ioctl_hwc, argp, sizeof(ioctl_hwc)))
			return -EFAULT;

		user_buflen = ioctl_hwc.value;
		user_buffer = (u8 *) ioctl_hwc.buffer;

		while (user_buflen != 0) {
			copysize = user_buflen;

			if (user_buflen > sizeof(buffer))
				copysize = sizeof(buffer);

			if (copy_from_user(buffer, user_buffer, copysize))
				return -EFAULT;

			copysize16 = (copysize + 1)/2;
			ptr16 = (u16 *) buffer;
			s1d13522if_BurstWrite16(ptr16, copysize16);
			user_buflen -= copysize;
			user_buffer += copysize;
		}

		return 0;
	}

	case S1D13522_MEMBURSTREAD:
	{
		u8 buffer[512];
		unsigned user_buflen, copysize, copysize16;
		u16 *ptr16;
		u8 *user_buffer;

		if (copy_from_user(&ioctl_hwc, argp, sizeof(ioctl_hwc)))
			return -EFAULT;

		user_buflen = ioctl_hwc.value;
		user_buffer = (u8 *) ioctl_hwc.buffer;

		while (user_buflen != 0) {
			copysize = user_buflen;

			if (user_buflen > sizeof(buffer))
				copysize = sizeof(buffer);

			copysize16 = (copysize + 1)/2;
			ptr16 = (u16 *) buffer;
			s1d13522if_BurstRead16(ptr16, copysize16);

			if (copy_to_user(user_buffer, buffer, copysize))
				return -EFAULT;

			user_buflen -= copysize;
			user_buffer += copysize;
		}

		return 0;
	}

	case S1D13522_SET_REGAL_ON:
		mutex_lock(&(par->io_lock));
		if (s1d13522fb_regal_on(par))
			par->regal_on = true;
		mutex_unlock(&(par->io_lock));
		break;

	case S1D13522_SET_REGAL_OFF:
		mutex_lock(&(par->io_lock));
		if (s1d13522fb_regal_off(par))
			par->regal_on = false;
		mutex_unlock(&(par->io_lock));
		break;

	case S1D13522_GET_REGAL_STATE:
		if (copy_from_user(&ioctl_hwc, argp, sizeof(ioctl_hwc)))
			return -EFAULT;

		mutex_lock(&(par->io_lock));
		ioctl_hwc.value = par->regal_on ? 1 : 0;
		mutex_unlock(&(par->io_lock));
		return copy_to_user(argp, &ioctl_hwc, sizeof(ioctl_hwc)) ? -EFAULT : 0;

	case S1D13522_SET_REGAL_D_ON:
		mutex_lock(&(par->io_lock));
		if (s1d13522fb_regal_d_on(par))
			par->regal_d_on = true;
		mutex_unlock(&(par->io_lock));
		return 0;

	case S1D13522_SET_REGAL_D_OFF:
		mutex_lock(&(par->io_lock));
		if (s1d13522fb_regal_d_off(par))
			par->regal_d_on = false;
		mutex_unlock(&(par->io_lock));
		return 0;

	case S1D13522_GET_REGAL_D_STATE:
		if (copy_from_user(&ioctl_hwc, argp, sizeof(ioctl_hwc)))
			return -EFAULT;

		mutex_lock(&(par->io_lock));
		ioctl_hwc.value = par->regal_d_on ? 1 : 0;
		mutex_unlock(&(par->io_lock));
		return copy_to_user(argp, &ioctl_hwc, sizeof(ioctl_hwc)) ? -EFAULT : 0;

	default:
			return -EINVAL;
	}

	return 0;
}
#endif

#ifdef CONFIG_S1D13522_INIT_CMD_FIRMWARE
static int s1d13522_init_cmdfw(struct s1d13522fb_par *par)
{
	struct device *dev = par->info->dev;
	const struct firmware *firmware = NULL;
	int err;

	dbg_info("%s\n", __func__);

	err = request_firmware(&firmware, FIRMWARE_FILE_COMMAND, dev);
	if (err) {
		printk(KERN_ERR
		"s1d13522fb: Command Firmware not available [%d].\n", err);
		return -EIO;
	}

	dbg_info("Firmware size: %d [0x%x]\n", firmware->size, firmware->size);

	par->board->send_command(par, S1D13522_CMD_INIT_CMD_SET);
	par->board->burst_write(par, (firmware->size+1)/2,
				(u16 *)firmware->data);
	par->board->wait_for_rdy(par, 1000);
	release_firmware(firmware);
	return 0;
}
#endif

#ifdef CONFIG_S1D13522_INIT_WAVE_FIRMWARE
static int  s1d13522_init_wavefw(struct s1d13522fb_par *par, int addr)
{
	struct device *dev = par->info->dev;
	const struct firmware *firmware = NULL;
	int err, bc16;
	u16 args[5];

	dbg_info("%s addr:0x%x\n", __func__, (u32)addr);

	err = request_firmware(&firmware, FIRMWARE_FILE_WAVEFORM, dev);
	if (err) {
		printk(KERN_ERR
		"s1d13522fb: Waveform file not available [%d].\n", err);
		return -EIO;
	}

	dbg_info("Firmware size: %d [0x%x]\n", firmware->size, firmware->size);

	bc16 = (firmware->size+1)/2;
	args[0] = (u16)addr;
	args[1] = (u16)(addr>>16);
	args[2] = (u16) (bc16);
	args[3] = (u16) (bc16 >> 16);
	par->board->send_cmdargs(par, S1D13522_CMD_BST_WR_SDR, 4, args);
	args[0] = 0x154;
	par->board->send_cmdargs(par, S1D13522_CMD_WR_REG, 1, args);
	par->board->burst_write(par, bc16, (u16 *)firmware->data);
	par->board->send_command(par, S1D13522_CMD_BST_END_SDR);

	release_firmware(firmware);
	return 0;
}
#endif

/* #ifndef CONFIG_S1D13522_USB_INTERFACE */
#ifndef CONFIG_FB_EPSON_SPI
static int __devinit s1d13522fb_probe(struct platform_device *pdev)
#else
static int s1d13522fb_probe(struct platform_device *pdev)
#endif
{
	int retval = -ENOMEM;
	int videomemorysize;
	unsigned char *videomemory;
	struct s1d13522fb_par *par;
	struct s1d13522fb_board *board = kmalloc(sizeof *board, GFP_KERNEL);
	int dpyw, dpyh;
	u64 mask = 0;
	struct device *dev = &(s1d13522_spidev->master->dev);
	devlog = dev;
	dbg_info("%s(): Probing S1D13522fb\n", __func__);
	info = framebuffer_alloc(sizeof(struct s1d13522fb_par), &pdev->dev);
	if (!info) {
		printk(KERN_ERR "s1d13522fb_probe: framebuffer_alloc error\n");
		goto err;
	}

	dbg_info("%s(): framebuffer allocated\n", __func__);
	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);
	par = info->par;

	par->panel_index = S1D13522_PANEL_INDEX;
	dpyw = panel_table[S1D13522_PANEL_INDEX].w;
	dpyh = panel_table[S1D13522_PANEL_INDEX].h;
	par->panelw = dpyw;
	par->panelh = dpyh;

#ifdef CONFIG_S1D13522_I80_INTERFACE
	board->init = s1d13522_I80_init;
	board->wait_for_rdy = s1d13522_I80_wait_for_HRDY;
	board->cleanup = s1d13522_I80_terminate;
	board->write_reg = s1d13522_I80_write_reg;
	board->read_reg = s1d13522_I80_read_reg;
	board->send_command = s1d13522_send_command;
	board->send_cmdargs = s1d13522_I80_send_cmdargs;
	board->burst_write = s1d13522_I80_burst_write;
	board->suspend = NULL;
	board->resume = NULL;
	par->board = board;
#endif

#ifdef CONFIG_S1D13522_USB_INTERFACE
	board->init = s1d13522_usb_InterfaceInit;
	board->wait_for_rdy = s1d13522_usb_WaitForHRDY;
	board->cleanup = s1d13522_usb_InterfaceTerminate;
	board->write_reg = s1d13522_usb_write_reg;
	/*par->board->read_reg = s1d13522_usb_read_reg;
	par->board->read_reg = s1d13522_read_reg;*/
	board->read_reg = s1d13522_usb_read_reg;
	board->send_command = s1d13522_send_command;
	board->send_cmdargs = s1d13522_usb_send_cmdargs;
	board->burst_write = s1d13522_usb_burst_write;
	board->suspend = NULL;
	board->resume = NULL;
	par->board = board;
#endif

#if 0
#ifdef CONFIG_FB_EPSON_SPI
	board->init = s1d13522if_InterfaceInit_defio;
	board->wait_for_rdy = s1d13522if_WaitForHRDY_defio;
	board->write_reg = s1d13522if_WriteReg16_defio;
	board->read_reg = s1d13522if_ReadReg16_defio;
	board->send_command = s1d13522if_cmd_defio;
	board->send_cmdargs = s1d13522if_cmd_args_defio;

	board->burst_write = s1d13522if_BurstWrite16fb_defio;
	board->suspend = s1d13522_suspend_defio;
	board->resume = s1d13522_resume_defio;

	board->cleanup = s1d13522_InterfaceTerminate_defio;
	board->burst_read = s1d13522_burst_read_defio;
	board->setup_irq = s1d13522_setupirq_defio;
	par->board = board;
#endif
#endif
	mask = DMA_BIT_MASK(32);
	dev->dma_mask = &mask;
	dev->coherent_dma_mask = DMA_BIT_MASK(32);

	waveform_mode_location = roundup((dpyw*dpyh*S1D13522_EPD_BPP/8),
						 PAGE_SIZE);
	videomemorysize = roundup(waveform_mode_location+8, PAGE_SIZE);
	bufferSize = videomemorysize;
	videomemory = dma_alloc_coherent(dev, videomemorysize,
				 (dma_addr_t *) &DmaPhys, GFP_KERNEL);
	if (!videomemory) {
		printk(KERN_WARNING "s1d13522fb_probe: vmalloc failed.\n");
		goto err_fb_rel;
	}
	dbg_info("~~~>dpyw:%d dpyh:%d S1D13522_VFB_BPP:%d videomemory is %d\n",
			dpyw, dpyh, S1D13522_VFB_BPP, videomemorysize);
	dbg_info("~~~> waveform_mode_location: %d\n", waveform_mode_location);
	par->epd_buffer_host_memorysize = ALIGN(((S1D_DISPLAY_WIDTH * S1D_DISPLAY_HEIGHT * S1D13522_EPD_BPP) / 8), PAGE_SIZE);
	par->epd_buffer_host_stride = S1D_DISPLAY_WIDTH/2;
	numPage =  par->epd_buffer_host_memorysize/PAGE_SIZE;
	printk(KERN_ERR "%s number of pages: %d\n", __func__, numPage);

	par->epd_buffer_host = dma_alloc_coherent(dev,
				par->epd_buffer_host_memorysize,
				(dma_addr_t *) &DmaPhysH, GFP_KERNEL);
	par->epd_buffer_host_current = dma_alloc_coherent(dev,
				par->epd_buffer_host_memorysize,
				(dma_addr_t *) &DmaPhysHC, GFP_KERNEL);

	par->epd_buffer_vfb = videomemory;
	dbg_info("%s(): videomemory allocated\n", __func__);

	memset(videomemory, 0xFF, videomemorysize);
	info->screen_base = (char *)videomemory;
	par->epd_buffer_host = (char *)videomemory;
	info->screen_size = (dpyw*dpyh*S1D13522_VFB_BPP/8);
	dbg_info("%s(): videomemory=%lXh, screen_base=%lXh, screensize=%lXh\n",
		__func__, (unsigned long)videomemory,
		(unsigned long)info->screen_base,
		(unsigned long)info->screen_size);

	s1d13522fb_fix.line_length = (dpyw*S1D13522_VFB_BPP/8);
	info->fix = s1d13522fb_fix;
	info->fix.smem_start = virt_to_phys((void *)info->screen_base);
	info->fix.smem_len = videomemorysize;
	dbg_info("%s(): videomemorysize=%x, info->fix.smem_len=%x\n",
		__func__, videomemorysize, info->fix.smem_len);

	/* set flags for supported features */
	info->flags = FBINFO_FLAG_DEFAULT;
	info->pseudo_palette = par->pseudo_palette;

	s1d13522fb_var.xres = dpyw;
	s1d13522fb_var.yres = dpyh;
	s1d13522fb_var.xres_virtual = dpyw;
	s1d13522fb_var.yres_virtual = dpyh;

	info->var = s1d13522fb_var;
	info->fbops = &s1d13522fb_ops;
	info->node = -1;

	info->fbdefio = &s1d13522fb_defio;
	fb_deferred_io_init(info);
	dbg_info("%s(): fb_deferred_io initialized\n", __func__);

	par->info = info;
	init_waitqueue_head(&par->waitq);
	mutex_init(&par->io_lock);
	mutex_init(&vsync_mutex);
	mutex_init(&drawingEnabledMutex);

	/* initialize the vsync wait queue */
       init_waitqueue_head(&par->vsync_wait);
       par->vsync_timeout = HZ;

	retval = s1d13522_init(par);
	if (retval < 0) {
		printk(KERN_ERR "s1d13522fb_probe: Display Init error\n");
		goto err_cleanup;
	}

	par->malloc_cache = s1d13522_init_mem_chache();
	if (!par->malloc_cache) {
		printk(KERN_ERR "s1d13522fb_probe: Couldn't allocate memcache\n");
		goto err_unregister;
	}

	retval = register_framebuffer(info);
	if (retval < 0)
		goto err_cleanup;
	dbg_info("%s(): framebuffer registered = %x\n", __func__, retval);

	dbg_info("fb%d: s1d13522 frame buffer, using %dK of video memory\n",
					info->node, videomemorysize >> 10);
	if (par->regal_on == true)
		s1d13522fb_regal_on(par);
	else if (par->regal_d_on == true)
		s1d13522fb_regal_d_on(par);

	updateMode = UPD_PART;
	updateWaveform = WF_MODE_GC16;

	printk(KERN_INFO "fb%d: s1d13522 framebuf, using %dK of video memory\n",
					info->node, videomemorysize >> 10);
#ifdef CONFIG_S1D13522_PROC
	dbg_info("%s(): Setting up PROC functions\n", __func__);
	s1d13522fb_proc_setup(info);
#endif
	kfree(board);
	return 0;
err_unregister:
	unregister_framebuffer(info);
err_cleanup:
	dma_free_coherent(dev, videomemorysize, videomemory, DmaPhys);
	dma_free_coherent(dev, par->epd_buffer_host_memorysize,
				par->epd_buffer_host, DmaPhysH);
	dma_free_coherent(dev, par->epd_buffer_host_memorysize,
				par->epd_buffer_host_current, DmaPhysHC);

err_fb_rel:
	framebuffer_release(info);
	info = NULL;
err:
	kfree(board);
	return retval;
}

/*#ifndef CONFIG_S1D13522_USB_INTERFACE*/
#ifndef CONFIG_FB_EPSON_SPI
static int __devexit s1d13522fb_remove(struct platform_device *dev)
#else
static int  s1d13522fb_remove(struct platform_device *dev)
#endif
{
	struct fb_info *info = platform_get_drvdata(dev);
	dbg_info("%s(): info:%p\n", __func__, info);

#ifdef CONFIG_S1D13522_PROC
	s1d13522fb_proc_terminate();
#endif
	if (info) {
		struct s1d13522fb_par *par = info->par;
		unregister_framebuffer(info);
		dbg_info("%s(): Unregistered framebuffer\n", __func__);
		fb_deferred_io_cleanup(info);
		s1d13522_destroy_mem_cache(par->malloc_cache);
		par->board->cleanup(par);
		vfree((void *)info->screen_base);
		framebuffer_release(info);
		platform_set_drvdata(dev, NULL);

		s1d13522fb_unregister_thermal_zones();
	}
	return 0;
}

#ifdef CONFIG_S1D13522_POWER_MANAGEMENT
static int s1d13522fb_suspend(struct platform_device *dev, pm_message_t msg)
{
	int retval = 0;
	struct fb_info *info = platform_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;

	par->board->send_command(par, S1D13522_CMD_STBY);

	if (par->board->suspend)
		retval = par->board->suspend(par);

	printk(KERN_INFO "S1D13522 frame buffer suspended\n");
	return retval;
}

static int s1d13522fb_resume(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct s1d13522fb_par *par = info->par;
	int retval = 0;

	par->board->send_command(par, S1D13522_CMD_RUN_SYS);
	if (par->board->resume)
		retval = par->board->resume(par);

	return retval;
}
#else
#define s1d13522fb_suspend NULL
#define s1d13522fb_resume NULL
#endif

static struct of_device_id vfb_match_table[] = {
	{ .compatible = "s1d13522vfb",},
	{},
};

static struct platform_driver s1d13522fb_driver = {
	.probe		= s1d13522fb_probe,
	.remove		= s1d13522fb_remove,
	.suspend	= s1d13522fb_suspend,
	.resume		= s1d13522fb_resume,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "s1d13522fb",
		.of_match_table = vfb_match_table,
	},
};

static struct platform_device *s1d13522fb_device;

#ifndef CONFIG_FB_EPSON_SPI
int __init s1d13522fb_init(void)
#else
int s1d13522fb_init(void)
#endif
{
	int ret;
	dbg_info("~~~> %s():\n", __func__);
	ret = platform_driver_register(&s1d13522fb_driver);
	dbg_info("~~~> %s(): registered driver\n", __func__);
	return ret;
}

/*#ifndef CONFIG_S1D13522_USB_INTERFACE*/
#ifndef CONFIG_FB_EPSON_SPI
void __exit s1d13522fb_exit(void)
#else
void s1d13522fb_exit(void)
#endif
{
	dbg_info("%s():\n", __func__);
	platform_driver_unregister(&s1d13522fb_driver);
	dbg_info("%s():Unregisterted driver\n", __func__);
	platform_device_unregister(s1d13522fb_device);
	dbg_info("%s():Unregistered device\n", __func__);
}

#ifndef CONFIG_S1D13522_USB_INTERFACE
#ifndef CONFIG_FB_EPSON_SPI
/*module_init(s1d13522fb_init);
module_exit(s1d13522fb_exit);*/
#endif
#endif

#ifdef CONFIG_S1D13522_INIT_CMD_FIRMWARE
MODULE_FIRMWARE(FIRMWARE_FILE_COMMAND);
MODULE_FIRMWARE(FIRMWARE_FILE_WAVEFORM);
#endif

MODULE_AUTHOR("SEIKO EPSON CORPORATION");
MODULE_DESCRIPTION("EPSON S1D13522 frame buffer driver");
MODULE_LICENSE("GPL v2");
