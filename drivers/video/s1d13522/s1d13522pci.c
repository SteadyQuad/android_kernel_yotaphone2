//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson/s1d13522pci.c -- Optional PCI specific code for
// frame buffer driver for Epson S1D13522 controllers.
//
// Copyright(c) Seiko Epson Corporation 2009.
// All rights reserved.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------

#ifdef CONFIG_FB_EPSON_PCI

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include "s1d13522fb.h"

//-----------------------------------------------------------------------------
//
// Local Definitions
//
//---------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//
// Function Prototypes
//
//-----------------------------------------------------------------------------

static int  __devinit epsonfb_pci_probe (struct pci_dev *pdev, const struct pci_device_id *ent);
static void __devexit epsonfb_pci_unregister (struct pci_dev *pdev);

//-----------------------------------------------------------------------------
//
// Local Globals
//
//-----------------------------------------------------------------------------

static struct pci_dev   *s1d13521_pcidev;

enum epson_bems {
        EPSON_1300,
        EPSON_0000,     EPSON_0001,     EPSON_0002,     EPSON_0003,
        EPSON_0004,     EPSON_0005,     EPSON_0006,     EPSON_0007,
        EPSON_0008,     EPSON_0009,     EPSON_000A,     EPSON_000B,
        EPSON_000C,     EPSON_000D,     EPSON_000E,     EPSON_000F,
        EPSON_0010,     EPSON_0011,     EPSON_0012,     EPSON_0013,
        EPSON_0014,     EPSON_0015,     EPSON_0016,     EPSON_0017,
        EPSON_0018,     EPSON_0019,     EPSON_001A,     EPSON_001B,
        EPSON_001C,     EPSON_001D,     EPSON_001E,     EPSON_001F,
        EPSON_0020,     EPSON_0021,     EPSON_0022,     EPSON_0023,
        EPSON_0024,     EPSON_0025,     EPSON_0026,     EPSON_0027,
        EPSON_0028,     EPSON_0029,     EPSON_002A,     EPSON_002B,
        EPSON_002C,     EPSON_002D,     EPSON_002E,     EPSON_002F,
        EPSON_0030,     EPSON_0031
};

#define PCI_VENDOR_ID_EPSON_OLD         0x10F4
#define PCI_VENDOR_ID_EPSON             0x14EB

static struct pci_device_id epsonfb_pci_table[] =
{
        { PCI_VENDOR_ID_EPSON_OLD,      0x1300, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_1300},
        { PCI_VENDOR_ID_EPSON,          0x0000, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0000},
        { PCI_VENDOR_ID_EPSON,          0x0001, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0001},
        { PCI_VENDOR_ID_EPSON,          0x0002, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0002},
        { PCI_VENDOR_ID_EPSON,          0x0003, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0003},
        { PCI_VENDOR_ID_EPSON,          0x0004, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0004},
        { PCI_VENDOR_ID_EPSON,          0x0005, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0005},
        { PCI_VENDOR_ID_EPSON,          0x0006, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0006},
        { PCI_VENDOR_ID_EPSON,          0x0007, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0007},
        { PCI_VENDOR_ID_EPSON,          0x0008, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0008},
        { PCI_VENDOR_ID_EPSON,          0x0009, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0009},
        { PCI_VENDOR_ID_EPSON,          0x000A, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_000A},
        { PCI_VENDOR_ID_EPSON,          0x000B, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_000B},
        { PCI_VENDOR_ID_EPSON,          0x000C, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_000C},
        { PCI_VENDOR_ID_EPSON,          0x000D, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_000D},
        { PCI_VENDOR_ID_EPSON,          0x000E, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_000E},
        { PCI_VENDOR_ID_EPSON,          0x000F, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_000F},
        { PCI_VENDOR_ID_EPSON,          0x0010, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0010},
        { PCI_VENDOR_ID_EPSON,          0x0011, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0011},
        { PCI_VENDOR_ID_EPSON,          0x0012, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0012},
        { PCI_VENDOR_ID_EPSON,          0x0013, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0013},
        { PCI_VENDOR_ID_EPSON,          0x0014, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0014},
        { PCI_VENDOR_ID_EPSON,          0x0015, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0015},
        { PCI_VENDOR_ID_EPSON,          0x0016, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0016},
        { PCI_VENDOR_ID_EPSON,          0x0017, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0017},
        { PCI_VENDOR_ID_EPSON,          0x0018, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0018},
        { PCI_VENDOR_ID_EPSON,          0x0019, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0019},
        { PCI_VENDOR_ID_EPSON,          0x001A, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_001A},
        { PCI_VENDOR_ID_EPSON,          0x001B, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_001B},
        { PCI_VENDOR_ID_EPSON,          0x001C, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_001C},
        { PCI_VENDOR_ID_EPSON,          0x001D, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_001D},
        { PCI_VENDOR_ID_EPSON,          0x001E, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_001E},
        { PCI_VENDOR_ID_EPSON,          0x001F, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_001F},
        { PCI_VENDOR_ID_EPSON,          0x0020, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0020},
        { PCI_VENDOR_ID_EPSON,          0x0021, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0021},
        { PCI_VENDOR_ID_EPSON,          0x0022, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0022},
        { PCI_VENDOR_ID_EPSON,          0x0023, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0023},
        { PCI_VENDOR_ID_EPSON,          0x0024, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0024},
        { PCI_VENDOR_ID_EPSON,          0x0025, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0025},
        { PCI_VENDOR_ID_EPSON,          0x0026, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0026},
        { PCI_VENDOR_ID_EPSON,          0x0027, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0027},
        { PCI_VENDOR_ID_EPSON,          0x0028, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0028},
        { PCI_VENDOR_ID_EPSON,          0x0029, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0029},
        { PCI_VENDOR_ID_EPSON,          0x002A, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_002A},
        { PCI_VENDOR_ID_EPSON,          0x002B, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_002B},
        { PCI_VENDOR_ID_EPSON,          0x002C, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_002C},
        { PCI_VENDOR_ID_EPSON,          0x002D, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_002D},
        { PCI_VENDOR_ID_EPSON,          0x002E, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_002E},
        { PCI_VENDOR_ID_EPSON,          0x002F, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_002F},
        { PCI_VENDOR_ID_EPSON,          0x0030, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0030},
        { PCI_VENDOR_ID_EPSON,          0x0031, PCI_ANY_ID, PCI_ANY_ID, 0, 0, EPSON_0031},
        { 0, }
};
MODULE_DEVICE_TABLE(pci, epsonfb_pci_table);

static struct pci_driver epsonfb_driver =
{
        .name           = "s1d13522fb",
        .id_table       = epsonfb_pci_table,
        .probe          = epsonfb_pci_probe,
        .remove         = __devexit_p(epsonfb_pci_unregister),
};

static int __devinit epsonfb_pci_probe (struct pci_dev *pdev, const struct pci_device_id *ent)
{
        s1d13521_pcidev = pdev;
        return 0;
}

static void __devexit epsonfb_pci_unregister(struct pci_dev *pdev)
{
}

//-----------------------------------------------------------------------------
//
// Detect Epson PCI bridge adapter card, get the BAR[0].
//
//-----------------------------------------------------------------------------
int  __devinit s1d13522pci_init(long *physicalAddress)
{
        int ret;

        dbg_info("%s():\n",__FUNCTION__);

        ret = pci_register_driver(&epsonfb_driver);

        if (ret != 0) {
                printk(KERN_ERR "s1d13521pci_init: Failed to register PCI device\n");
                return ret;
        }

        ret = pci_enable_device(s1d13521_pcidev);

        if (ret != 0) {
                printk(KERN_ERR "s1d13521pci_init: Failed to enable PCI device\n");
                return ret;
        }

        *physicalAddress = (pci_resource_start(s1d13521_pcidev, 0) & PCI_BASE_ADDRESS_MEM_MASK);
        return 0;
}


void __devexit s1d13522pci_terminate(void)
{
        pci_unregister_driver (&epsonfb_driver);
}

#endif

