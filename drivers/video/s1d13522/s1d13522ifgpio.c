//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson-13522/s1d13522ifgpio.c -- Gumstix specific GPIO
// interface code for Epson S1D13522 LCD controllerframe buffer driver.
//
// Code based on E-Ink demo source code.
//
// Copyright(c) Seiko Epson Corporation 2009.
// All rights reserved.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------

#ifdef CONFIG_FB_EPSON_GPIO_GUMSTIX
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include "s1d13522fb.h"

//---------------------------------------------------------------------------
//
// Local Definitions
//
//---------------------------------------------------------------------------

#define GPIO_CNFX	16
#define GPIO_CNF1	17

#ifdef CONFIG_FB_EPSON_HRDY_OK
#define GPIO_HRDY	32
#endif

#define GPIO_HDC	48
#define GPIO_RESET_L	49
#define GPIO_HRD_L	74
#define GPIO_HWE_L	75
#define GPIO_HCS_L	76
#define GPIO_HIRQ	77
#define GPIO_HDB	58 // 58-73
#define MAP_SIZE	4096

// gpio registers

#define REG(r) (*(volatile unsigned int *)((char*)_map+(r)))

#define GPIO_ADDR	0x40E00000
#define GPIO_LR		0x0000
#define GPIO_DR		0x000C
#define GPIO_SR		0x0018
#define GPIO_CR		0x0024
#define GPIO_RER	0x0030
#define GPIO_FER	0x003C
#define GPIO_EDR	0x0048
#define GPIO_AFR	0x0054	//GPIO Alternate Function Register


//---------------------------------------------------------------------------
//
// Local Globals
//
//---------------------------------------------------------------------------
static void * _map;
static struct mutex ghwMutex;

//---------------------------------------------------------------------------
//
// Local Function Prototypes
//
//---------------------------------------------------------------------------
void init_gpio(void);
static void set_gpio_val(int pin, int val);
static void set_gpio_mode(int pin);
static void set_gpio_dir(int pin, int val);
static int  get_gpio_val(int pin);
static void gpio_hdb_dir(int v);
static void command(int v);
static void data(int v);
static int  data_get(void);
static int  gpio_hdb_get(void);
static void gpio_hdb(int val);
static void set_cmd_mode(void);

#if defined(CONFIG_FB_EPSON_DEBUG_PRINTK) || defined (CONFIG_FB_EPSON_PROC)
static int get_gpio_mode(int pin);
static int get_gpio_dir(int pin);
#endif

#ifdef CONFIG_FB_EPSON_HRDY_OK
static int  wait_for_ready(void);
#endif

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
int s1d13522if_InterfaceInit(FB_INFO_S1D13522 *info)
{
	u8* RegAddr;
	RegAddr  = (unsigned char*) ioremap_nocache(GPIO_ADDR,MAP_SIZE);

	if (RegAddr == NULL)
	{
		printk(KERN_ERR "s1d13522if_InterfaceInit (Gumstix): ioremap_nocache failed\n");
		return -EINVAL;
	}

	dbg_info("s1d13522ifgpio: %s():: RegAddr %x\n", __FUNCTION__,(unsigned int)RegAddr);

	info->RegAddr = RegAddr;
	info->RegAddrMappedSize = MAP_SIZE;
	_map = (void*)info->RegAddr;
	mutex_init(&ghwMutex);
	init_gpio();
	set_cmd_mode(); // command mode, reset the chip
	return 0;
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void s1d13522if_InterfaceTerminate(FB_INFO_S1D13522 *info)
{
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
int s1d13522if_WaitForHRDY(void)
{
#ifndef CONFIG_FB_EPSON_HRDY_OK
	int cnt = HRDY_TIMEOUT_MS;

	// The host must not issue any new command until HRDY is asserted.
	// If HRDY is not used, the host can poll the Sequence Controller Busy Status
	// bit in REG[000Ah] using the RD_REG command

	// If the HRDY is not used, poll Sequence Controller Busy Status
	mutex_lock(&ghwMutex);
	gpio_hdb_dir(1);
	set_gpio_val(GPIO_HCS_L, 0);
	command(RD_REG);
	set_gpio_val(GPIO_HDC, 1);
	data(0x0A);		     // Register address
	gpio_hdb_dir(0);

	// Loop while host interface busy bit is set...
	while (data_get() & 0x20) {
		if (--cnt <= 0)	 // Avoid endless loop
			break;

		mdelay(1);
	}

	set_gpio_val(GPIO_HCS_L, 1);
	mutex_unlock(&ghwMutex);

	if (cnt <= 0) {
		printk(KERN_ERR "s1d13522if_WaitForHRDY: I/F busy bit stuck\n");
		return -1;
	}

	return 0;
#else
	int retval;
	mutex_lock(&ghwMutex);
	retval = wait_for_ready();
	mutex_unlock(&ghwMutex);
	return retval;
#endif
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
u16 s1d13522if_ReadReg16(u16 Index)
{
	u16 Value;

	mutex_lock(&ghwMutex);
	gpio_hdb_dir(1);
	set_gpio_val(GPIO_HCS_L, 0);
	command(RD_REG);
	set_gpio_val(GPIO_HDC, 1);
	data(Index);
	gpio_hdb_dir(0);
	Value = data_get();
	set_gpio_val(GPIO_HCS_L, 1);
	mutex_unlock(&ghwMutex);
	dbg_info("s1d13522ifgpio: %s(): Reg[%02xh]=%02xh\n",__FUNCTION__, Index,Value);
	return Value;
}

//---------------------------------------------------------------------------
//
//
//---------------------------------------------------------------------------
void s1d13522if_WriteReg16(u16 Index, u16 Value)
{
	dbg_info("%s(): %02x,%02x\n",__FUNCTION__,Index,Value);
	s1d13522if_WaitForHRDY();
	mutex_lock(&ghwMutex);
	gpio_hdb_dir(1);
	set_gpio_val(GPIO_HCS_L, 0);
	command(WR_REG);
	set_gpio_val(GPIO_HDC, 1);
	data(Index);		    // Register address
	data(Value);		    // Register value
	set_gpio_val(GPIO_HCS_L, 1);
	mutex_unlock(&ghwMutex);
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
int s1d13522if_cmd(unsigned ioctlcmd,s1d13522_ioctl_cmd_params *params,int numparams)
{
	int i;
	unsigned cmd = ioctlcmd & 0xFF;

	dbg_info("s1d13522ifgpio: %s(): cmd=%xh numparams=%d\n",__FUNCTION__, cmd,numparams);

	if (s1d13522if_WaitForHRDY() != 0) {
		return -1;
	}

	mutex_lock(&ghwMutex);
	gpio_hdb_dir(1);
	set_gpio_val(GPIO_HCS_L, 0);
	command(cmd);
	set_gpio_val(GPIO_HDC, 1);

	for (i = 0; i < numparams; i++)
		data(params->param[i]);

	set_gpio_val(GPIO_HCS_L, 1);
	mutex_unlock(&ghwMutex);
	return 0;
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void s1d13522if_BurstWrite16(u16 *ptr16, unsigned copysize16)
{
	mutex_lock(&ghwMutex);
	gpio_hdb_dir(1);
	set_gpio_val(GPIO_HCS_L, 0);
	set_gpio_val(GPIO_HDC, 1);

	while (copysize16-- > 0)
	{
		set_gpio_val(GPIO_HWE_L, 0);
		gpio_hdb(*ptr16++);
		set_gpio_val(GPIO_HWE_L, 1);
	}

	set_gpio_val(GPIO_HCS_L, 1);
	mutex_unlock(&ghwMutex);
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void s1d13522if_BurstRead16(u16 *ptr16, unsigned copysize16)
{
	mutex_lock(&ghwMutex);
	gpio_hdb_dir(0);
	set_gpio_val(GPIO_HCS_L, 0);
	set_gpio_val(GPIO_HDC, 1);

	while (copysize16-- > 0) {
		set_gpio_val(GPIO_HRD_L, 0);
		*ptr16++ = gpio_hdb_get();
		set_gpio_val(GPIO_HRD_L, 1);
	}

	set_gpio_val(GPIO_HCS_L, 1);
	mutex_unlock(&ghwMutex);
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void init_gpio(void)
{
	int i;
	dbg_info("s1d13522ifgpio: %s():\n",__FUNCTION__);

	assert(GPIO_HDB == 58);

	set_gpio_mode(GPIO_CNFX);
	set_gpio_mode(GPIO_CNF1);

	set_gpio_mode(GPIO_HDC);
	set_gpio_mode(GPIO_RESET_L);
	set_gpio_mode(GPIO_HRD_L);
	set_gpio_mode(GPIO_HWE_L);
	set_gpio_mode(GPIO_HCS_L);
	set_gpio_mode(GPIO_HIRQ);

	for (i = 0; i < 16; i++)
		set_gpio_mode(GPIO_HDB + i);

	assert(get_gpio_mode(GPIO_CNFX) == 0);
	assert(get_gpio_mode(GPIO_CNF1) == 0);
	assert(get_gpio_mode(GPIO_HDC) == 0);
	assert(get_gpio_mode(GPIO_RESET_L) == 0);
	assert(get_gpio_mode(GPIO_HRD_L) == 0);
	assert(get_gpio_mode(GPIO_HWE_L) == 0);
	assert(get_gpio_mode(GPIO_HCS_L) == 0);
	assert(get_gpio_mode(GPIO_HIRQ) == 0);

	for (i = 0; i < 16; i++)
		assert(get_gpio_mode(GPIO_HDB + i) == 0);

	set_gpio_dir(GPIO_CNFX, 1);
	set_gpio_dir(GPIO_CNF1, 1);
	set_gpio_dir(GPIO_HDC, 1);
	set_gpio_dir(GPIO_RESET_L, 1);
	set_gpio_dir(GPIO_HRD_L, 1);
	set_gpio_dir(GPIO_HWE_L, 1);
	set_gpio_dir(GPIO_HCS_L, 1);
	set_gpio_dir(GPIO_HIRQ, 0);

	for (i = 0; i < 16; i++)
		set_gpio_dir(GPIO_HDB + i, 1);

	assert(get_gpio_dir(GPIO_CNF1) == 1);

#ifdef CONFIG_FB_EPSON_HRDY_OK
	set_gpio_mode(GPIO_HRDY);
	assert(get_gpio_mode(GPIO_HRDY) == 0);
	set_gpio_dir(GPIO_HRDY, 0);
	assert(get_gpio_dir(GPIO_HRDY) == 0);
#endif
	assert(get_gpio_dir(GPIO_HDC) == 1);
	assert(get_gpio_dir(GPIO_RESET_L) == 1);
	assert(get_gpio_dir(GPIO_HRD_L) == 1);
	assert(get_gpio_dir(GPIO_HWE_L) == 1);
	assert(get_gpio_dir(GPIO_HCS_L) == 1);
	assert(get_gpio_dir(GPIO_HIRQ) == 0);

	for (i = 0; i < 16; i++)
		assert(get_gpio_dir(GPIO_HDB + i) == 1);

	set_gpio_val(GPIO_CNFX, 0);
	set_gpio_val(GPIO_CNF1, 1);		// command mode
	set_gpio_val(GPIO_HDC, 0);
	set_gpio_val(GPIO_HRD_L, 1);
	set_gpio_val(GPIO_HWE_L, 1);
	set_gpio_val(GPIO_HCS_L, 1);
	set_gpio_val(GPIO_RESET_L, 1);

	assert(get_gpio_val(GPIO_CNF1) == 1);
	assert(get_gpio_val(GPIO_HDC) == 0);
	assert(get_gpio_val(GPIO_CNFX) == 0);
	assert(get_gpio_val(GPIO_HRD_L) == 1);
	assert(get_gpio_val(GPIO_HWE_L) == 1);
	assert(get_gpio_val(GPIO_HCS_L) == 1);
	assert(get_gpio_val(GPIO_RESET_L) == 1);
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void command(int v)
{
	assert(get_gpio_val(GPIO_HCS_L) == 0);
	set_gpio_val(GPIO_HDC, 0);
	set_gpio_val(GPIO_HWE_L, 0);
	gpio_hdb(v);
	set_gpio_val(GPIO_HWE_L, 1);
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void data(int v)
{
	dbg_info("s1d13522ifgpio: %s(): %02xh\n",__FUNCTION__, (unsigned)v);
	assert(get_gpio_val(GPIO_HCS_L) == 0);
	assert(get_gpio_val(GPIO_HDC) == 1);

	set_gpio_val(GPIO_HWE_L, 0);
	gpio_hdb(v);
	set_gpio_val(GPIO_HWE_L, 1);
}

//---------------------------------------------------------------------------
// The data bus direction is already set to input
//---------------------------------------------------------------------------
int data_get(void)
{
	int d;

	assert(get_gpio_val(GPIO_HCS_L) == 0);
	assert(get_gpio_val(GPIO_HDC) == 1);

	set_gpio_val(GPIO_HRD_L, 0);
	d = gpio_hdb_get();
	set_gpio_val(GPIO_HRD_L, 1);
	return d;
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
#ifdef CONFIG_FB_EPSON_HRDY_OK
int wait_for_ready(void)
{
	int cnt = HRDY_TIMEOUT_MS;
	int d = get_gpio_val(GPIO_HRDY);

	while (d == 0) 	{
		mdelay(1);

		// Avoid endless loop
		if (--cnt <= 0)	{
			printk(KERN_ERR "s1d13522if_cmd: wait_for_ready: I/F busy bit stuck\n");
			return -1;
		}
		d = get_gpio_val(GPIO_HRDY);
	}

	return 0;
}
#endif

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void set_gpio_val(int pin, int val)
{
	int r = (pin >> 5) << 2;
	int b = pin & 0x1F;
	unsigned int v = 1 << b;

	if (val & 0x1)
		REG(GPIO_SR + r) = v;
	else
		REG(GPIO_CR + r) = v;
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void gpio_hdb(int val)
{
	unsigned int v;

	v = val & 0x3F;

	if (v != 0)
		REG(GPIO_SR + 4) = v << 26;

	v = (~v) & 0x3F;

	if (v != 0)
		REG(GPIO_CR + 4) = v << 26;

	v = (val >> 6) & 0x3FF;

	if (v != 0)
		REG(GPIO_SR + 8) = v;

	v = (~v) & 0x3FF;

	if (v != 0)
		REG(GPIO_CR + 8) = v;
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
int gpio_hdb_get(void)
{
	unsigned int v;
	int d;

	v = REG(GPIO_LR + 4);
	d = (v >> 26) & 0x3F;
	v = REG(GPIO_LR + 8);
	d |= (v & 0x3FF) << 6;
	return d;
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void set_gpio_dir(int pin, int val)
{
	int b, r;

	r = (pin >> 5) << 2;
	r += GPIO_DR;
	b = pin & 0x1F;

	if (val & 0x1)
		REG(r) |= (1 << b);
	else
		REG(r) &= ~(1 << b);
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void set_gpio_mode(int pin)
{
	int r,b;

//      dbg_info("s1d13522ifgpio: %s(): %d\n",__FUNCTION__,pin);

	r = (pin >> 4) << 2;
	b = (pin & 0xF) << 1;

#ifdef CONFIG_FB_EPSON_DEBUG_PRINTK
	if (pin == 49)
		assert((r == 12) && (b == 2));

	if (pin == 75)
		assert((r == 16) && (b == 22));
#endif
	r += GPIO_AFR;
	REG(r) &= ~(3 << b);
}

#if defined(CONFIG_FB_EPSON_DEBUG_PRINTK) || defined (CONFIG_FB_EPSON_PROC)
//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
int get_gpio_mode(int pin)
{
	int r,b,v;

	r = (pin >> 4) << 2;
	r += GPIO_AFR;
	b = (pin & 0xF) << 1;
	v = REG(r);
	return (v >> b) & 0x3;
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
int get_gpio_dir(int pin)
{
	int r,b,v;

	r = (pin >> 5) << 2;
	r += GPIO_DR;
	b = pin & 0x1F;
	v = REG(r);
	return (v >> b) & 0x1;
}
#endif

//---------------------------------------------------------------------------
// Set the data bus direction:
//      val = 1 -> output
//      val = 0 -> input
//---------------------------------------------------------------------------
void gpio_hdb_dir(int val)
{
	if (val & 0x1) {
		REG(GPIO_DR + 4) |= 0xFC000000;
		REG(GPIO_DR + 8) |= 0x000003FF;
	} else {
		REG(GPIO_DR + 4) &= 0x03FFFFFF;
		REG(GPIO_DR + 8) &= 0xFFFFFC00;
	}
}

int get_gpio_val(int pin)
{
	int r,b;
	unsigned int v;

	r = (pin >> 5) << 2;
	b = pin & 0x1F;
	v = REG(GPIO_LR + r);
	return (v >> b) & 0x1;
}


void set_cmd_mode(void)
{
	dbg_info("s1d13522ifgpio: %s():\n",__FUNCTION__);

	set_gpio_val(GPIO_CNF1, 1);

	// reset pulse
	set_gpio_val(GPIO_RESET_L, 0);
	mdelay(10);
	set_gpio_val(GPIO_RESET_L, 1);

#ifdef CONFIG_FB_EPSON_HRDY_OK
	wait_for_ready();
#endif
}

#ifdef CONFIG_FB_EPSON_PROC
typedef struct
{
	const int gpio;
	const char *gpiostr;
}GPIONAMEST;

int dump_gpio(char *buf)
{
static GPIONAMEST aGpio[] =
	{
	{GPIO_CNFX,     "GPIO_CNFX   "},
	{GPIO_CNF1,     "GPIO_CNF1   "},
#ifdef CONFIG_FB_EPSON_HRDY_OK
	{GPIO_HRDY,     "GPIO_HRDY   "},
#endif
	{GPIO_HDC,      "GPIO_HDC    "},
	{GPIO_RESET_L,  "GPIO_RESET_L"},
	{GPIO_HRD_L,    "GPIO_HRD_L  "},
	{GPIO_HWE_L,    "GPIO_HWE_L  "},
	{GPIO_HCS_L,    "GPIO_HCS_L  "},
	{GPIO_HIRQ,     "GPIO_HIRQ   "},
	{GPIO_HDB+0,    "GPIO_HDB_0  "},
	{GPIO_HDB+1,    "GPIO_HDB_1  "},
	{GPIO_HDB+2,    "GPIO_HDB_2  "},
	{GPIO_HDB+3,    "GPIO_HDB_3  "},
	{GPIO_HDB+4,    "GPIO_HDB_4  "},
	{GPIO_HDB+5,    "GPIO_HDB_5  "},
	{GPIO_HDB+6,    "GPIO_HDB_6  "},
	{GPIO_HDB+7,    "GPIO_HDB_7  "},
	{GPIO_HDB+8,    "GPIO_HDB_8  "},
	{GPIO_HDB+9,    "GPIO_HDB_9  "},
	{GPIO_HDB+10,   "GPIO_HDB_A  "},
	{GPIO_HDB+11,   "GPIO_HDB_B  "},
	{GPIO_HDB+12,   "GPIO_HDB_C  "},
	{GPIO_HDB+13,   "GPIO_HDB_D  "},
	{GPIO_HDB+14,   "GPIO_HDB_E  "},
	{GPIO_HDB+15,   "GPIO_HDB_F  "}
	};

	int i;
	char *bufin = buf;

	buf += sprintf(buf,"GPIO	MODE  DIR  VAL\n");
	buf += sprintf(buf,"--------------------------\n");

	for (i = 0; i < sizeof(aGpio)/sizeof(aGpio[0]); i++) {
		buf += sprintf(buf,"%s  %d    %d    %d\n",aGpio[i].gpiostr,
			get_gpio_mode(aGpio[i].gpio),
			get_gpio_dir(aGpio[i].gpio),
			get_gpio_val(aGpio[i].gpio));
	}

	return strlen(bufin);
}
#endif //CONFIG_FB_EPSON_PROC
#endif // CONFIG_FB_EPSON_GPIO_GUMSTIX

