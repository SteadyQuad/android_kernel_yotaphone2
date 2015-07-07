//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson-13522/s1d13522if.c -- Frame buffer driver for Epson
// S1D13522 LCD controller.
//
// Copyright(c) Seiko Epson Corporation 2009.
// All rights reserved.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//
// NOTE: This file provides indirect interface functionality
//
//-----------------------------------------------------------------------------
#ifdef CONFIG_FB_EPSON_INDIRECT_INTERFACE

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>

#ifdef CONFIG_FB_EPSON_PCI
    #include <linux/pci.h>
#endif

#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>

#include "s1d13522fb.h"

//-----------------------------------------------------------------------------
//
// Local Definitions
//
//---------------------------------------------------------------------------

typedef struct
{
        volatile u16*   cmd;
        volatile u16*   param;
} S1D13522FB_IND_STRUCT;

#define S1D_MMAP_PHYSICAL_REG_SIZE (sizeof(S1D13522FB_IND_STRUCT))

//-----------------------------------------------------------------------------
//
// Function Prototypes
//
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//
// Globals
//
//-----------------------------------------------------------------------------
static struct mutex ghwMutex;
static S1D13522FB_IND_STRUCT S1DIndInfo;        // indirect pointers structure.

int s1d13522if_InterfaceInit(FB_INFO_S1D13522 *info)
{
	u8 *RegAddr;
#ifdef CONFIG_FB_EPSON_PCI
	long physicalAddress;

	if (s1d13522pci_init(&physicalAddress) != 0) {
		printk(KERN_ERR "s1d13522if_InterfaceInit: PCI init failed\n");
		return -EINVAL;
	}

	RegAddr = (u8*) ioremap_nocache(physicalAddress,S1D_MMAP_PHYSICAL_REG_SIZE);
#else
	RegAddr = (u8*) ioremap_nocache(S1D_PHYSICAL_REG_ADDR,S1D_MMAP_PHYSICAL_REG_SIZE);
#endif
	if (RegAddr == NULL)
		return -EINVAL;

	dbg_info("%s(): RegAddr %x\n", __FUNCTION__,(unsigned int)RegAddr);
	mutex_init(&ghwMutex);
	S1DIndInfo.cmd   = (u16*) (RegAddr+0);
	S1DIndInfo.param = (u16*) (RegAddr+2);

	info->RegAddr = RegAddr;
	info->RegAddrMappedSize = S1D_MMAP_PHYSICAL_REG_SIZE;
	return 0;
}

//---------------------------------------------------------------------------
//
//
//---------------------------------------------------------------------------
void s1d13522if_InterfaceTerminate(FB_INFO_S1D13522 *info)
{
#ifdef CONFIG_FB_EPSON_PCI
	s1d13522pci_terminate();
#endif
}

//---------------------------------------------------------------------------
//
//
//---------------------------------------------------------------------------
u16 s1d13522if_ReadReg16(u16 Index)
{
	u16 Value;

	mutex_lock(&ghwMutex);
	*S1DIndInfo.cmd = RD_REG;
	*S1DIndInfo.param = Index;      // Register address

	// If HRDY is not used, we assume some host I/F minimal timings are met
	// as we cannot check for host interface busy.
	Value = *S1DIndInfo.param;
	mutex_unlock(&ghwMutex);
	return Value;
}

//---------------------------------------------------------------------------
//
//
//---------------------------------------------------------------------------
void s1d13522if_BurstRead16(u16 *ptr16, unsigned copysize16)
{
	mutex_lock(&ghwMutex);
	while (copysize16-- > 0)
		*ptr16++ = *S1DIndInfo.param;
	mutex_unlock(&ghwMutex);
}

int s1d13522if_WaitForHRDY(void)
{
#ifndef CONFIG_FB_EPSON_HRDY_OK
        int cnt = HRDY_TIMEOUT_MS;

	// The host must not issue any new command until HRDY is asserted.
	// If HRDY is not used, the host can poll the Sequence Controller Busy Status
	// bit in REG[000Ah] using the RD_REG command

	// If the HRDY is not used, poll Sequence Controller Busy Status
	mutex_lock(&ghwMutex);
	*S1DIndInfo.cmd = RD_REG;
	*S1DIndInfo.param = 0x0A;       // Register address

	// Loop while host interface busy bit is set...
	while (*S1DIndInfo.param & 0x20) {
		if (--cnt <= 0)         // Avoid endless loop
			break;

		mdelay(1);
	}

	mutex_unlock(&ghwMutex);
	if (cnt <= 0) {
		printk(KERN_ERR "s1d13522if_WaitForHRDY: I/F busy bit stuck\n");
		return -1;
	}
#endif
	return 0;
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
	*S1DIndInfo.cmd = WR_REG;
	*S1DIndInfo.param = Index;      // Register address
	*S1DIndInfo.param = Value;      // Register value
	mutex_unlock(&ghwMutex);
}

//---------------------------------------------------------------------------
//
//
//---------------------------------------------------------------------------
void s1d13522if_BurstWrite16(u16 *ptr16, unsigned copysize16)
{
	mutex_lock(&ghwMutex);
	while (copysize16-- > 0)
		*S1DIndInfo.param = *ptr16++;
	mutex_unlock(&ghwMutex);
}

//---------------------------------------------------------------------------
//
//
//---------------------------------------------------------------------------
int s1d13522if_cmd(unsigned ioctlcmd,s1d13522_ioctl_cmd_params *params,int numparams)
{
	int i;
	unsigned cmd = ioctlcmd & 0xFF;

	mutex_lock(&ghwMutex);

	if (s1d13522if_WaitForHRDY() != 0) {
		mutex_unlock(&ghwMutex);
		return -1;
	}

	*S1DIndInfo.cmd = cmd;

	for (i = 0; i < numparams; i++)
		*S1DIndInfo.param = params->param[i];

	mutex_unlock(&ghwMutex);
	return 0;
}

#endif //CONFIG_FB_EPSON_INDIRECT_INTERFACE
