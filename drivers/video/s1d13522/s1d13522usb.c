//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson-13522/s1d13522usb.c -- USB interface code for
// Epson S1D13522 LCD controllerframe buffer driver.
//
// Copyright(c) Seiko Epson Corporation 2009.
// All rights reserved.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------

#ifdef CONFIG_FB_EPSON_USB

#ifdef CONFIG_FB_EPSON_SHOW_SETTINGS
	#warning Using USB interface
#endif

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/smp_lock.h>
#include <linux/mutex.h>
#include <linux/usb.h>
#include <asm/system.h>
#include <asm/uaccess.h> /* copy from/to user */

#include "s1d13522fb.h"

//---------------------------------------------------------------------------
//
// Local Definitions
//
//---------------------------------------------------------------------------
#define SEND_CMD(CODE) seCsclWrMge(seMgeDesc.MgeRegAddr+0, 2, seCSCL_MGE_REGISTERS | seCSCL_MGE_DIRECT, (CODE))
#define SEND_PAR(DATA) seCsclWrMge(seMgeDesc.MgeRegAddr+6, 2, seCSCL_MGE_REGISTERS | seCSCL_MGE_DIRECT, (DATA))
#define READ_DATA()    seCsclRdMge(seMgeDesc.MgeRegAddr+6, 2, seCSCL_MGE_REGISTERS | seCSCL_MGE_DIRECT)
#define BURST_WRITE(CNT,DATA) seCsclBurstWrMge(seMgeDesc.MgeRegAddr+6,(CNT),seCSCL_MGE_REGISTERS|seCSCL_MGE_DIRECT,(u8*)(DATA))
#define BURST_READ(CNT, DATA) seCsclBurstRdMge(seMgeDesc.MgeRegAddr+6,(CNT),seCSCL_MGE_REGISTERS|seCSCL_MGE_DIRECT,(u8*)(DATA))

#ifndef info
  #define info(format, arg...) printk(KERN_INFO KBUILD_MODNAME ": " format "\n" , ## arg)
#endif

//---------------------------------------------------------------------------
//
// Local Globals
//
//---------------------------------------------------------------------------

static struct mutex gMgeMutex;

/* table of devices that work with this driver */
static struct usb_device_id dev_table [] = {
	{ USB_DEVICE(0x0547, 0x1002) },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, dev_table);


/* Get a minor range for your devices from the usb maintainer */
#define USB_PROMETHEUS_MINOR_BASE	0
#define MAX_TRANSFER			(PAGE_SIZE/2)
#define WRITES_IN_FLIGHT		8 /* Prometheus driver will not use simultaneous writes */

/* Structure to hold all of our device specific stuff */
struct usb_prometheus {
	struct usb_device	*udev;			/* the usb device for this device */
	struct usb_interface	*interface;		/* the interface for this device */
	struct semaphore	limit_sem;		/* limiting the number of writes in progress */
	struct usb_anchor	submitted;		/* in case we need to retract our submissions */
	unsigned char		*bulk_in_buffer;	/* the buffer to receive data */
	size_t			bulk_in_size;		/* the size of the receive buffer */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	int			errors;			/* the last request tanked */
	int			open_count;		/* count the number of openers */
	spinlock_t		err_lock;		/* lock for errors */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
};

struct usb_prometheus *private_data;

#define to_prometheus_dev(d) container_of(d, struct usb_prometheus, kref)

static struct usb_driver prometheus_driver;
static void prometheus_draw_down(struct usb_prometheus *dev);

static void prometheus_delete(struct kref *kref)
{
	struct usb_prometheus *dev = to_prometheus_dev(kref);

	usb_put_dev(dev->udev);
	kfree(dev->bulk_in_buffer);
	kfree(dev);
}

static ssize_t prometheus_read(char *buffer, size_t count)
{
	struct usb_prometheus *dev = private_data;
	int retval, bytes_read;

//	dbg_info("s1d13522: %s(): Start blocking read, count: %d buffer: %x %d\n",
//		__FUNCTION__, count, dev->bulk_in_buffer, dev->bulk_in_size);

	mutex_lock(&dev->io_mutex);
	if (!dev->interface) {		/* disconnect() was called */
		retval = -ENODEV;
		goto exit;
	}

	/* do a blocking bulk read to get data from the device */
	retval = usb_bulk_msg(dev->udev,
			      usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
			      dev->bulk_in_buffer,
			      min(dev->bulk_in_size, count),
			      &bytes_read, 10000);

	/* if the read was successful, copy the data to userspace */
	if (!retval) {
		if (copy_to_user(buffer, dev->bulk_in_buffer, bytes_read))
			retval = -EFAULT;
		else
			retval = bytes_read;
	}

exit:
	mutex_unlock(&dev->io_mutex);
	//printk("stop blocking read bytes read %d retval %d\n", bytes_read, retval );
	return retval;
}

static void prometheus_write_bulk_callback(struct urb *urb)
{
	struct usb_prometheus *dev;

	dev = urb->context;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if(!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			err("%s - nonzero write bulk status received: %d",
			    __func__, urb->status);

		spin_lock(&dev->err_lock);
		dev->errors = urb->status;
		spin_unlock(&dev->err_lock);
	}

	/* free up our allocated buffer */
	usb_buffer_free(urb->dev, urb->transfer_buffer_length,
			urb->transfer_buffer, urb->transfer_dma);
	up(&dev->limit_sem);
	//printk("end non blocking write buf 0x%x len %d\n", urb->transfer_buffer, urb->transfer_buffer_length);
}

static ssize_t prometheus_write(const char *user_buffer, size_t count)
{
	struct usb_prometheus *dev;
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	size_t writesize = min(count, (size_t)MAX_TRANSFER);

//	dbg_info("s1d13522: %s(): Start non blocking write, count: %d writesize: %d\n",__FUNCTION__,count, writesize);

	dev = private_data;

	/* verify that we actually have some data to write */
	if (count == 0)
		goto exit;

	/* limit the number of URBs in flight to stop a user from using up all RAM */
	if (down_interruptible(&dev->limit_sem)) {
		retval = -ERESTARTSYS;
		goto exit;
	}

	spin_lock_irq(&dev->err_lock);
	if ((retval = dev->errors) < 0) {
		/* any error is reported once */
		dev->errors = 0;
		/* to preserve notifications about reset */
		retval = (retval == -EPIPE) ? retval : -EIO;
	}
	spin_unlock_irq(&dev->err_lock);
	if (retval < 0)
		goto error;

	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}

	buf = usb_buffer_alloc(dev->udev, writesize, GFP_KERNEL, &urb->transfer_dma);
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}

	if (copy_from_user(buf, user_buffer, writesize)) {
		retval = -EFAULT;
		goto error;
	}

	/* this lock makes sure we don't submit URBs to gone devices */
	mutex_lock(&dev->io_mutex);
	if (!dev->interface) {		/* disconnect() was called */
		mutex_unlock(&dev->io_mutex);
		retval = -ENODEV;
		goto error;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
			  buf, writesize, prometheus_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	usb_anchor_urb(urb, &dev->submitted);

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	mutex_unlock(&dev->io_mutex);
	if (retval) {
		err("%s - failed submitting write urb, error %d", __func__, retval);
		goto error_unanchor;
	}

	/* release our reference to this urb, the USB core will eventually free it entirely */
	usb_free_urb(urb);
	return writesize;

error_unanchor:
	usb_unanchor_urb(urb);
error:
	if (urb) {
		usb_buffer_free(dev->udev, writesize, buf, urb->transfer_dma);
		usb_free_urb(urb);
	}
	up(&dev->limit_sem);

exit:
	return retval;
}

static const struct file_operations prometheus_fops = {
	.owner =	THIS_MODULE,
//	.read =		prometheus_read,
//	.write =	prometheus_write,
//	.open =		prometheus_open,
//	.release =	prometheus_release,
//	.flush =	prometheus_flush,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver prometheus_class = {
	.name =		"prometheus%d",
	.fops =		&prometheus_fops,
	.minor_base =	USB_PROMETHEUS_MINOR_BASE,
};

//extern int s1d13522fb_init(void);

static int prometheus_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_prometheus *dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;

	dbg_info("s1d13522: %s():\n",__FUNCTION__);

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		err("Out of memory");
		goto error;
	}

	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
	mutex_init(&dev->io_mutex);
	spin_lock_init(&dev->err_lock);
	init_usb_anchor(&dev->submitted);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	/* tpk. The choice for endpoints is good fpr the MGE read
	 * MGE write. For the MGE interrupt read another In endpoint to
	 * be added */
	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dev->bulk_in_endpointAddr &&
		    usb_endpoint_is_bulk_in(endpoint)) {
			/* we found a bulk in endpoint */
			buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
			printk("%s(): bulk_in_size: %d\n",__FUNCTION__,(int)buffer_size);
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
			if (!dev->bulk_in_buffer) {
				err("Could not allocate bulk_in_buffer");
				goto error;
			}
		}

		if (!dev->bulk_out_endpointAddr &&
		    usb_endpoint_is_bulk_out(endpoint)) {
			/* we found a bulk out endpoint */
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
		}
	}
	if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		err("Could not find both bulk-in and bulk-out endpoints");
		goto error;
	}

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &prometheus_class);
	if (retval) {
		/* something prevented us from registering this driver */
		err("Not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	private_data = dev;
//	s1d13522fb_init();

#ifdef CONFIG_FB_EPSON_PROC
	s1d13522proc_init();
#endif
	/* let the user know what node this device is now attached to */
	info("USB Prometheus device now attached to USB Prometheus-%d", interface->minor);
	return 0;

error:
	if (dev)
		/* this frees allocated memory */
		kref_put(&dev->kref, prometheus_delete);
	return retval;
}

static void prometheus_disconnect(struct usb_interface *interface)
{
	struct usb_prometheus *dev;
	int minor = interface->minor;

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* give back our minor */
	usb_deregister_dev(interface, &prometheus_class);

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->interface = NULL;
	mutex_unlock(&dev->io_mutex);

	usb_kill_anchored_urbs(&dev->submitted);

	/* decrement our usage count */
	kref_put(&dev->kref, prometheus_delete);
	s1d13522fb_exit();
	info("USB Prometheus #%d now disconnected", minor);
}

static void prometheus_draw_down(struct usb_prometheus *dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->submitted);
}

static int prometheus_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_prometheus *dev = usb_get_intfdata(intf);

	if (!dev)
		return 0;
	prometheus_draw_down(dev);
	return 0;
}

static int prometheus_resume (struct usb_interface *intf)
{
	return 0;
}

static int prometheus_pre_reset(struct usb_interface *intf)
{
	struct usb_prometheus *dev = usb_get_intfdata(intf);
	dbg_info("%s(): \n",__FUNCTION__);
	mutex_lock(&dev->io_mutex);
	prometheus_draw_down(dev);
	return 0;
}

static int prometheus_post_reset(struct usb_interface *intf)
{
	struct usb_prometheus *dev = usb_get_intfdata(intf);
	dbg_info("%s(): \n",__FUNCTION__);
	/* we are sure no URBs are active - no locking needed */
	dev->errors = -EPIPE;
	mutex_unlock(&dev->io_mutex);
	return 0;
}

static struct usb_driver prometheus_driver = {
	.name =		"prometheus",
	.probe =	prometheus_probe,
	.disconnect =	prometheus_disconnect,
	.suspend =	prometheus_suspend,
	.resume =	prometheus_resume,
	.pre_reset =	prometheus_pre_reset,
	.post_reset =	prometheus_post_reset,
	.id_table =	dev_table,
	.supports_autosuspend = 1,
};

static int __init usb_prometheus_init(void)
{
	int result;

	result = usb_register(&prometheus_driver);
	if (result)
		err("usb_register failed. Error number %d", result);

	return result;
}

static void __exit usb_prometheus_exit(void)
{
	s1d13522fb_exit();
	usb_deregister(&prometheus_driver);
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define seMaxMgeNum 4			/* Number of supported MGEs */
#define seMGE_REG_ADDR_OFFSET		0//(0x00105500)
#define seMGE_MEM_ADDR_OFFSET		0//(0x00105500)

/* Bit assignment for the Read MGE/ Write MGE packets */
#define seCSCL_MGE_MEMORY		1
#define seCSCL_MGE_REGISTERS		0
#define seCSCL_MGE_NORMAL		2
#define seCSCL_MGE_BYPASS		0
#define seCSCL_MGE_ADDR_INC		4
#define seCSCL_MGE_ADDR_SAME		0
#define seCSCL_MGE_DIRECT		8
#define seCSCL_MGE_INDIRECT		0

/* Data Formats */
typedef enum{
	seCSCL_READ_MGE = 		0,	//0x000
	seCSCL_MGE_READ_DATA = 		1,	//0x001
	seCSCL_WRITE_MGE = 		2,	//0x002
	seCSCL_CMD_DONE = 		3,	//0x003
	seCSCL_WRITE_CONFIG_REGS = 	4,	//0x004
	seCSCL_READ_CONFIG_REGS = 	5,	//0x005
	seCSCL_CONFIG_REGS_DATA = 	6,	//0x006
	seCSCL_READ_MGE_STATUS = 	7,	//0x007
	seCSCL_MGE_STATUS = 		8,	//0x008
	seCSCL_RELOAD_FPGA = 		9,	//0x009
	seCSCL_PROGRAM_FPGA = 		0x00a,	//0x00a
	seCSCL_CMD_NAK = 		0x00b,	//0x00b
	seCSCL_FPGA_DATA = 		0x00c,	//0x00c
	seCSCL_JTAG_ENABLE = 		0x00d,	//0x00d
	seCSCL_JTAG_DISABLE = 		0x00e,	//0x00e
	seCSCL_JTAG_SYNC = 		0x00f,	//0x00f
	seCSCL_JTAG_PREAMBLE = 		0x010,	//0x010
	seCSCL_JTAG_POSTAMBLE = 	0x011,	//0x011
	seCSCL_JTAG_HDR_TRLR = 		0x012,	//0x012
	seCSCL_JTAG_DATA_SHIFT = 	0x013,	//0x013
	seCSCL_JTAG_TDO_DATA = 		0x014,	//0x014
	seCSCL_WRONG = 			0xFF
} seCSCL_PACKET_TYPE;

// Prometheus Protocol Packets

typedef struct seCsclInterrupt {
	u8 byte0;
	u8 byte1;
} seCsclInterrupt;

typedef struct seCsclHeader {
	u8 byte0;
	u8 byte1;
} seCsclHeader;

typedef struct seCsclRdMgePkt {
	seCsclHeader hdr;
	u8 Addr0;
	u8 Addr1;
	u8 Addr2;
	u8 Param;
	u8 N0;
	u8 N1;
} seCsclRdMgePkt;

typedef struct seCsclMgeRdDataPkt {
	seCsclHeader hdr;
	u8 N0;
	u8 N1;
} seCsclMgeRdDataPkt; // Variable lenght packet

typedef struct seCsclWrMgeDataPkt {
	seCsclHeader hdr;
	u8 Addr0;
	u8 Addr1;
	u8 Addr2;
	u8 Param;
	u8 N0;
	u8 N1;
} seCsclWrMgeDataPkt; // Variable lenght packet

//#define seCSCLPACKET_MAX_DATA_SIZE	(0xfff0)
#define seCSCLPACKET_MAX_DATA_SIZE	(2048 - sizeof(seCsclWrMgeDataPkt))

typedef struct seCsclCmdDonePkt {
	seCsclHeader hdr;
} seCsclCmdDonePkt; // Header Only packet

typedef struct seCsclWrConfigRegsPkt {
	seCsclHeader hdr;
	u8 Mge0_ConfigA;
	u8 Mge0_ConfigB;
	u8 Mge0_ConfigC;
	u8 Mge0_ConfigD;
	u8 Mge0_ConfigE;
	u8 Mge0_ConfigF;
	u8 Mge1_ConfigA;
	u8 Mge1_ConfigB;
	u8 Mge1_ConfigC;
	u8 Mge1_ConfigD;
	u8 Mge1_ConfigE;
	u8 Mge1_ConfigF;
	u8 Mge2_ConfigA;
	u8 Mge2_ConfigB;
	u8 Mge2_ConfigC;
	u8 Mge2_ConfigD;
	u8 Mge2_ConfigE;
	u8 Mge2_ConfigF;
	u8 Mge3_ConfigA;
	u8 Mge3_ConfigB;
	u8 Mge3_ConfigC;
	u8 Mge3_ConfigD;
	u8 Mge3_ConfigE;
	u8 Mge3_ConfigF;
} seCsclWrConfigRegsPkt; // Fixedlenght packet

/* Private data */
// The Prometheus Configuration Info is alocated as a private structure for all mges.
//  This structure is a shadow for the hw configuration registers.

/* Global MGE Configuration */
typedef struct seCsclConfigRegs {
	u8 ConfigRegA;
	u8 ConfigRegB;
	u8 ConfigRegC;
	u8 ConfigRegD;
	u8 ConfigRegE;
	u8 ConfigRegF;
}seCsclConfigRegs;

static seCsclConfigRegs segCsclConfigRegsInfo[seMaxMgeNum] ={
	{0x41,0x50,0x44,0xd1,0x00,0x00},
	{0x40,0x55,0x00,0x50,0xbb,0xbb},
	{0x40,0x55,0x00,0x50,0xcc,0xcc},
	{0x40,0x55,0x00,0x50,0xdd,0xdd},};

typedef struct seMgeDescriptor {
	u32 MgeNum;
	u32 InterfaceFlags;
	u32 MgeMemAddr;
	u32 MgeRegAddr;
	u32 HalConfigFlags;
	seCsclConfigRegs *RegsInfo;
}seMgeDescriptor;

// Mge Descriptor keeps the host interface parameters, memory map information.
// The descriptor is declated as external in the cscl.h. The HCL files
// can use the descriptor to access CSCL layer data.
static seMgeDescriptor seMgeDesc = {0,};

static void seCsclWrConfigRegs(void);
static void seCsclDispConfigRegs(seCsclConfigRegs *pRegInfo);
static bool csclWaitForDone(bool fWait);
static seCSCL_PACKET_TYPE csclParsePktHdr(u8 *pBuf);

static u32 seUsbTransportPktOut(seCSCL_PACKET_TYPE type, u8* pBuffer, int length, bool lock);
static u32 seUsbTransportPktIn (u8 *pBuffer, int length, bool lock);

/*-----------------------------------------------------------------------------
	seCsclInitGlobals( UInt32 InterfaceFlags )
------------------------------------------------------------------------------*/
static void seCsclInitGlobals(u32 Mge, u32 InterfaceFlags, seCsclConfigRegs *pConfigRegs)
{
	dbg_info("%s(): flags: %x\n",__FUNCTION__, InterfaceFlags);

	seMgeDesc.MgeNum = 		Mge;
	seMgeDesc.InterfaceFlags =	InterfaceFlags;
	seMgeDesc.MgeMemAddr =		seMGE_MEM_ADDR_OFFSET;
	seMgeDesc.MgeRegAddr =		seMGE_REG_ADDR_OFFSET;
	seMgeDesc.RegsInfo =		&segCsclConfigRegsInfo[seMgeDesc.MgeNum];

	if (pConfigRegs != NULL) {
		segCsclConfigRegsInfo[seMgeDesc.MgeNum].ConfigRegA = pConfigRegs->ConfigRegA;
		segCsclConfigRegsInfo[seMgeDesc.MgeNum].ConfigRegB = pConfigRegs->ConfigRegB;
		segCsclConfigRegsInfo[seMgeDesc.MgeNum].ConfigRegC = pConfigRegs->ConfigRegC;
		segCsclConfigRegsInfo[seMgeDesc.MgeNum].ConfigRegD = pConfigRegs->ConfigRegD;
		segCsclConfigRegsInfo[seMgeDesc.MgeNum].ConfigRegE = pConfigRegs->ConfigRegE;
		segCsclConfigRegsInfo[seMgeDesc.MgeNum].ConfigRegF = pConfigRegs->ConfigRegF;
	}
}

/*-----------------------------------------------------------------------------
	seCsclWrConfigRegs()
------------------------------------------------------------------------------*/
static void seCsclWrConfigRegs(void)
{
	seCsclConfigRegs *pRegInfo;
	seCsclWrConfigRegsPkt pPkt;
	u8 *pBuf;
	int i = seMaxMgeNum;

	dbg_info("s1d13522: %s(): \n",__FUNCTION__);

	pRegInfo = &segCsclConfigRegsInfo[0];
	seCsclDispConfigRegs(&segCsclConfigRegsInfo[0]);

	pPkt.hdr.byte0 = seCSCL_WRITE_CONFIG_REGS;
	pPkt.hdr.byte1 = 0;

	pBuf = (u8*)&pPkt + 2;

	do {
		*pBuf++ = pRegInfo->ConfigRegA;
		*pBuf++ = pRegInfo->ConfigRegB;
		*pBuf++ = pRegInfo->ConfigRegC;
		*pBuf++ = pRegInfo->ConfigRegD;
		*pBuf++ = pRegInfo->ConfigRegE;
		*pBuf++ = pRegInfo->ConfigRegF;
		pRegInfo++;
	} while (--i > 0);

	seUsbTransportPktOut(seCSCL_WRITE_CONFIG_REGS, (u8*)&pPkt, sizeof(seCsclWrConfigRegsPkt), FALSE);
	csclWaitForDone(TRUE);
}

/*-----------------------------------------------------------------------------
    seCsclSetup(u32 Mge, u32 InterfaceFlags, seCsclConfigRegs *pConfigRegs)
------------------------------------------------------------------------------*/
static bool seCsclSetup(u32 Mge, u32 InterfaceFlags, seCsclConfigRegs *pConfigRegs )
{
	dbg_info("%s():\n",__FUNCTION__);

	seCsclInitGlobals(Mge, InterfaceFlags, pConfigRegs);
	seCsclWrConfigRegs();
	return TRUE;
}

/*-----------------------------------------------------------------------------
	Private csclParsePktHdr() returns a CSCL packet type
------------------------------------------------------------------------------*/
static seCSCL_PACKET_TYPE csclParsePktHdr(u8 *pBuf)
{
	return (pBuf[0] & 0x3F);
}

/*-----------------------------------------------------------------------------
	seCsclWrMge()
------------------------------------------------------------------------------*/
static bool seCsclWrMge(u32 address, u32 bytes, u32 param, u32 value)
{
	seCsclWrMgeDataPkt * pPkt;
	int bufferSize;
	u8* pData;

	bufferSize = sizeof(seCsclWrMgeDataPkt) + bytes;

	pPkt = (seCsclWrMgeDataPkt *)kmalloc(bufferSize, GFP_KERNEL);
	if (!pPkt) {
		printk("s1d13522: %s(): Unable to allocate transfer buffer\n",__FUNCTION__);
		return FALSE;
	}

	memset((u8*)pPkt, 0, bufferSize);

	pPkt->hdr.byte0	= seCSCL_WRITE_MGE; // MGE TBD
	pPkt->Addr0	= address ;
	pPkt->Addr1	= (address >> 8);
	pPkt->Addr2	= (address >> 16);
	pPkt->Param	= param;
	pPkt->N0	= bytes;
	pPkt->N1	= (bytes >> 8);
	pData = (u8*) (&(pPkt->N1));
	pData++;

	if (bytes == 1){
		* pData++ = value;
	} else if (bytes == 2)	{
		* pData++ = value;
		* pData++ = (value >>  8);
	} else if (bytes == 3)	{
		* pData++ = value ;
		* pData++ = (value >>  8);
		* pData++ = (value >> 16);
	} else if (bytes == 4)	{
		* pData++ = value;
		* pData++ = (value >>  8);
		* pData++ = (value >> 16);
		* pData++ = (value >> 24);
	} else {

	}

	seUsbTransportPktOut(seCSCL_WRITE_MGE, (u8*) pPkt, bufferSize, FALSE);
	kfree((u8*) pPkt);
	return csclWaitForDone(FALSE);
}

/*-----------------------------------------------------------------------------
	seCsclRdMge()
------------------------------------------------------------------------------*/
static u32 seCsclRdMge(u32 address, u32 bytes, u32 param)
{
	seCsclRdMgePkt * pPkt;
	seCsclMgeRdDataPkt * pData;
	int bufferSize = 0;
	u32 regValue = 0;
	int TimeoutMS = 5; // asume 5 frames for the Full speed devices (1.0 USB spec)

	bufferSize = sizeof(seCsclRdMgePkt);
	pPkt = (seCsclRdMgePkt*)kmalloc(bufferSize, GFP_KERNEL);

	if (!pPkt) {
		printk("s1d13522: %s(): Unable to allocate transfer buffer\n",__FUNCTION__);
		return 0;
	}

	memset((u8*)pPkt, 0, bufferSize);
	pPkt->hdr.byte0 = seCSCL_READ_MGE; // MGE TBD
	pPkt->Addr0	= address;
	pPkt->Addr1	= (address >>  8);
	pPkt->Addr2	= (address >> 16);
	pPkt->Param	= param;
	pPkt->N0	= bytes;
	pPkt->N1	= (bytes >> 8);

	seUsbTransportPktOut(seCSCL_READ_MGE, (u8*) pPkt, bufferSize, TRUE);
	bufferSize = (sizeof(seCsclMgeRdDataPkt) + bytes + 1) & ~1;
	pData = (seCsclMgeRdDataPkt *)krealloc((u8*)pPkt, bufferSize, GFP_KERNEL);

	if (!pData) {
		printk("s1d13522: %s(): Unable to re allocate transfer buffer\n",__FUNCTION__);
		return 0;
	}

	do {
		seCSCL_PACKET_TYPE pktType;
		u8 *pBuf = 0;
		int packetLength = 0;
		pktType = seCSCL_WRONG;
		memset((u8*)pData, 0, bufferSize);

		if (seUsbTransportPktIn((u8*) pData, bufferSize, FALSE) == 0) {
			TimeoutMS--;
			msleep(1);
			continue;  // in case of a glitch give it another try
		}
		if ((pktType = csclParsePktHdr((u8*) pData)) == seCSCL_MGE_READ_DATA) {
			pBuf = (u8*)pData;
			pBuf += 2;
			packetLength = pBuf[0] + (pBuf[1] << 8);
			pBuf += 2;
			switch (bytes)	{
				case 4: regValue = pBuf[0] + (pBuf[1] << 8) + (pBuf[2] << 16) + (pBuf[3] << 24); break;
				case 2: regValue = pBuf[0] + (pBuf[1] << 8); break;
				case 1: regValue = pBuf[0]; break;
				default: break;
			}
			break;
		} else	{
			printk("s1d13522: %s(): seCsclRdMge:Bogus packet received 0x%x, expected 0x%x\n",
				__FUNCTION__,pktType, seCSCL_MGE_READ_DATA);
			TimeoutMS--;
			msleep(1);
		}
	} while (TimeoutMS > 0);

	kfree((u8*)pData);
	return regValue;
}

/*-----------------------------------------------------------------------------
	seCsclBurstRdMge() -- There is no limitation (u32) on the number of
	bytes requested by application.
	Packet fragmentation shall be done since Prometheus CSCL protocol has
	16 bit limitation for the length of the data.
------------------------------------------------------------------------------*/
static u32 seCsclBurstRdMge(u32 address, u32 bytes, u32 param, u8 *pData)
{
	seCsclRdMgePkt * pPkt;
	seCsclMgeRdDataPkt * pDataPkt;
	u32 packetLength = 0, totalLength = 0;
	int bufferSize = 0;
	seCSCL_PACKET_TYPE pktType = seCSCL_WRONG;
	u32 bytesLeft = bytes;
	u32 lengthReceived = 0;
	u32 length2Expect = 0;
	bool lock = FALSE;

	if (!pData)
		return 0;

	dbg_info("s1d13522: %s(): read %d \n", __FUNCTION__,(int)bytes);

	/* Do Prometheus CSCL packet fragmentation if nessesary */

	do {
		bytes = (bytesLeft > seCSCLPACKET_MAX_DATA_SIZE) ? (seCSCLPACKET_MAX_DATA_SIZE) : (bytesLeft);
		bytesLeft -= bytes;
		lock = (bytesLeft != 0) ? TRUE : FALSE;

		/*Send a request packet. Requesting <bytes> of data from the Prometheus Server */

		bufferSize = sizeof(seCsclRdMgePkt);
		pPkt = (seCsclRdMgePkt *)kmalloc( bufferSize, GFP_KERNEL );
		if (!pPkt) {
			printk("s1d13522 %s(): Unable to allocate transfer buffer\n",__FUNCTION__);
			return 0;
		}

		memset((u8*)pPkt, 0, bufferSize);

		pPkt->hdr.byte0 = seCSCL_READ_MGE;
		pPkt->Addr0	= address;
		pPkt->Addr1	= (address >>  8);
		pPkt->Addr2	= (address >> 16);
		pPkt->Param	= param;
		pPkt->N0	= bytes;
		pPkt->N1	= (bytes >> 8);

		seUsbTransportPktOut(seCSCL_READ_MGE, (u8*)pPkt, bufferSize, TRUE);
		kfree(pPkt);

		/* Prepare to receive requested data */
		lengthReceived = 0;

		// tpk Commented out just in case if Prometheus state machine works
		// with exact number of odd bytes. Then waiting for <lengthReceived < bytes> could fail.
		//bytes = (bytes & 1) ? (bytes+1) : bytes; // round up
		bufferSize = sizeof( seCsclMgeRdDataPkt ) + ((bytes+1) & ~1);
		pDataPkt = (seCsclMgeRdDataPkt *)kmalloc( bufferSize, GFP_KERNEL );

		if (!pDataPkt) {
			printk("s1d13522 %s(): Unable to allocate a transfer buffer\n",__FUNCTION__);
			return 0;
		}

		do {
			u8 *pBuf = NULL;
			pktType = seCSCL_WRONG;
			memset((u8*)pDataPkt, 0, bufferSize);

			// Keep reading untill a desired number (Prometheus protocol) of data received.
			packetLength = seUsbTransportPktIn((u8*)pDataPkt,
					(lengthReceived ==0)?bufferSize:bufferSize-lengthReceived-4,lock);

			if (packetLength == 0)
				continue; // in case of the glitch give it another try

			pBuf = (u8*)pDataPkt;

			if (((pktType = csclParsePktHdr((u8*)pDataPkt)) == seCSCL_MGE_READ_DATA) && lengthReceived ==0) {
				// First Packet
				pBuf += 2;
				length2Expect = pBuf[0] + (pBuf[1]<<8);
				if ((length2Expect&(~1)) != (bytes&(~1))) {
					continue;
				}

				pBuf += 2;
				packetLength -= 4; // adjust for the CSCL header
			} else if (lengthReceived == 0) {
				printk("s1d13522: %s(): seCsclBurstRdMge:Bogus packet received 0x%x, expected 0x%x (len rcvd %d)\n",
						__FUNCTION__,pktType, seCSCL_MGE_READ_DATA, lengthReceived);
			}

			// Following packets
			lengthReceived += packetLength;
			if (lengthReceived == length2Expect+1)
				memcpy( pData, pBuf, packetLength-1);
			else
				memcpy( pData, pBuf, packetLength);

			pData += packetLength;
		} while (lengthReceived < bytes);


		// round down for the odd /even number of bytes.
		lengthReceived = (length2Expect < lengthReceived) ? length2Expect : lengthReceived;
		kfree((u8*)pDataPkt);
		totalLength += lengthReceived;
		address = ((seMgeDesc.InterfaceFlags & seCSCL_MGE_DIRECT)==seCSCL_MGE_DIRECT) ? (address + bytes) : address;
	} while (bytesLeft > 0);

	return totalLength;
}


/*-----------------------------------------------------------------------------
	seCsclDispConfigRegs(
------------------------------------------------------------------------------*/
static void seCsclDispConfigRegs(seCsclConfigRegs *pRegInfo)
{
	int i = seMaxMgeNum;

	do {
		printk( "CSCL:MGE %d Regs: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
			(seMaxMgeNum -i),pRegInfo->ConfigRegA,pRegInfo->ConfigRegB,
			pRegInfo->ConfigRegC, pRegInfo->ConfigRegD,
			pRegInfo->ConfigRegE, pRegInfo->ConfigRegF);
		 pRegInfo++;
	} while (--i > 0);

	printk("\n");
}

/*-----------------------------------------------------------------------------
	seCsclBurstWrMge()
------------------------------------------------------------------------------*/
static bool seCsclBurstWrMge(u32 address, u32 bytes, u32 param, u8 *pBuffer)
{
	seCsclWrMgeDataPkt * pPkt;
	int bufferSize = 0;
	u8 *pData = NULL;
	u32 bytesLeft = bytes;
	u32 bytesSent = 0;
	bool lock = FALSE;

	if (bytes == 0 || pBuffer == NULL)
		return FALSE;

//	dbg_info("s1d13522: %s(): bytes:0x%x pBuffer:%x\n",__FUNCTION__,bytes,(unsigned)pBuffer);

	do {
		bytes = (bytesLeft > seCSCLPACKET_MAX_DATA_SIZE)?(seCSCLPACKET_MAX_DATA_SIZE) : (bytesLeft);
		bytesLeft -= bytes;
		lock = (bytesLeft > 0) ? TRUE : FALSE;

		bufferSize = sizeof(seCsclWrMgeDataPkt) + bytes;
		pPkt = (seCsclWrMgeDataPkt *)kmalloc(bufferSize, GFP_KERNEL);

		if (!pPkt) {
			printk("%s(): Unable to allocate transfer buffer\n",__FUNCTION__);
			return FALSE;
		}

		memset((u8*) pPkt, 0, bufferSize);

		pPkt->hdr.byte0 = seCSCL_WRITE_MGE;
		pPkt->Addr0	= address;
		pPkt->Addr1	= (address >>  8);
		pPkt->Addr2	= (address >> 16);
		pPkt->Param	= param;
		pPkt->N0	= bytes;
		pPkt->N1	= (bytes >> 8);
		pData = (u8*) &(pPkt->N1);
		pData ++;
		memcpy(pData, pBuffer, bytes);
		bytesSent = seUsbTransportPktOut(seCSCL_WRITE_MGE, (u8*)pPkt, bufferSize, lock);
		pBuffer += bytes;
		address = ((seMgeDesc.InterfaceFlags & seCSCL_MGE_DIRECT)==seCSCL_MGE_DIRECT) ? (address + bytes) : address;
		kfree((u8*) pPkt);
	} while (bytesLeft > 0);

	return csclWaitForDone(FALSE);
}

/*-----------------------------------------------------------------------------

------------------------------------------------------------------------------*/
static bool csclWaitForDone(bool fWait)
{
	u8* pBuf;
	int packetLength;
	bool fResult = FALSE;
	seCSCL_PACKET_TYPE pktType;
	int TimeoutMS = 5; // assume 5 frames for the Full speed devices (1.0 USB spec)

//	dbg_info("s1d13522: %s():\n",__FUNCTION__);

	if (!fWait)
		return TRUE;

	pBuf = kmalloc(26 /*sizeof(seCsclCmdDonePkt)*/, GFP_KERNEL );

	do {
		pktType = seCSCL_WRONG;
		memset((u8*)pBuf, 0, 26);

		if ((packetLength = seUsbTransportPktIn(pBuf, sizeof(seCsclCmdDonePkt), FALSE)) == 0) {
			fResult = FALSE;
			break;
		} else if ((pktType = csclParsePktHdr(pBuf)) == seCSCL_CMD_NAK) {
			fResult = FALSE;
			break;
		}

		if ((pktType = csclParsePktHdr(pBuf)) == seCSCL_CMD_DONE) {
			fResult = TRUE;
			break;
		} else	{
			printk("s1d13522: csclWaitForDone:Bogus packet received 0x%x, expected 0x%x\n", pktType, seCSCL_CMD_DONE);
			fResult = FALSE;
		}

		msleep(1);
		TimeoutMS--;

	} while (TimeoutMS > 0);

	kfree((u8*)pBuf);
	return fResult;
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#define seCSCL_MGE_DATAOUT_PIPE	0
#define seCSCL_MGE_DATAIN_PIPE	2

static u32 seUsbTransportIn(int pipeNum, u8 *pbuffer, int length, bool lock);
static u32 seUsbTransportOut(int pipeNum, u8 *pbuffer, int length, bool lock);

/*----------------------------------------------------------------------------
	seUsbTransportPktOut()
------------------------------------------------------------------------------*/
static u32 seUsbTransportPktOut(seCSCL_PACKET_TYPE type, u8* pBuffer, int length, bool lock)
{
//	dbg_info("%s(): pBuffer: %x\n",__FUNCTION__, pBuffer);
	return seUsbTransportOut(seCSCL_MGE_DATAOUT_PIPE, pBuffer,length, lock);
}

/*----------------------------------------------------------------------------
	seUsbTransportPktIn ()
------------------------------------------------------------------------------*/
static u32 seUsbTransportPktIn (u8 *pBuffer, int length, bool lock)
{
	return seUsbTransportIn(seCSCL_MGE_DATAIN_PIPE, pBuffer,length, lock);
}

/*----------------------------------------------------------------------------
	u32 seUsbTransportIn()
------------------------------------------------------------------------------*/
static u32 seUsbTransportIn(int pipeNum, u8 *pbuffer, int length, bool lock)
{
	int nBytes;
	mm_segment_t old_fs;

//	dbg_info("%s(): length: %d\n",__FUNCTION__, length);

	old_fs = get_fs();
	set_fs(get_ds());
	nBytes = prometheus_read(pbuffer,length);
	set_fs(old_fs);

	if (nBytes != length) {
		printk("s1d13522: %s(): read: %d expct: %d \n", __FUNCTION__,nBytes, length);
		nBytes = 0;
	}

	return nBytes;
}

/*----------------------------------------------------------------------------
	seUsbTransportOut()
------------------------------------------------------------------------------*/
static u32 seUsbTransportOut(int pipeNum, u8 *pbuffer, int length, bool lock)
{
	int nBytes;
	mm_segment_t old_fs;

//	dbg_info("%s(): length: %d\n",__FUNCTION__, length);

	old_fs = get_fs();
	set_fs(get_ds());
	nBytes = prometheus_write(pbuffer, length);
	set_fs(old_fs);

	if (nBytes != length) {
		printk("s1d13522: %s(): wrote:%d  expct: %d\n",__FUNCTION__,nBytes,length);
		if (nBytes < 0)
			nBytes = 0;
	}

	return nBytes;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
int s1d13522if_InterfaceInit(FB_INFO_S1D13522 *info)
{
	printk("s1d13522if_InterfaceInit (USB)\n");

	mutex_init(&gMgeMutex);
	mutex_lock(&gMgeMutex);

	/* Configure Client Server Communication Layer */
	if (!seCsclSetup(0, seCSCL_MGE_REGISTERS | seCSCL_MGE_NORMAL | seCSCL_MGE_ADDR_INC, NULL)) {
		mutex_unlock(&gMgeMutex);
		printk("Error: Failed to setup USB communication.\n");
		return 1;
	}

	mutex_unlock(&gMgeMutex);
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
	mutex_lock(&gMgeMutex);
	if (!SEND_CMD(RD_REG))	printk("%s(): CSCL Error 1\n",__FUNCTION__);
	if (!SEND_PAR(0x0A))	printk("%s(): CSCL Error 2\n",__FUNCTION__);

	// Loop while host interface busy bit is set...
	while (READ_DATA() & 0x20) {
		if (--cnt <= 0)		// Avoid endless loop
			break;

		mdelay(1);
	}

	mutex_unlock(&gMgeMutex);

	if (cnt <= 0) {
		printk(KERN_ERR "s1d13522if_WaitForHRDY: I/F busy bit stuck\n");
		return -1;
	}
#endif
	return 0;
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
u16 s1d13522if_ReadReg16(u16 Index)
{
	u16 Value;

	mutex_lock(&gMgeMutex);
	if (!SEND_CMD(RD_REG))	printk("%s(): CSCL Error 1\n",__FUNCTION__);
	if (!SEND_PAR(Index))	printk("%s(): CSCL Error 2\n",__FUNCTION__);
	Value = READ_DATA();
	mutex_unlock(&gMgeMutex);
//	dbg_info("%s(): Reg[%02xh]=%02xh\n",__FUNCTION__, Index,Value);
	return Value;
}

//---------------------------------------------------------------------------
//
//
//---------------------------------------------------------------------------
void s1d13522if_WriteReg16(u16 Index, u16 Value)
{
//	dbg_info("%s(): %02x,%02x\n",__FUNCTION__,Index,Value);

	s1d13522if_WaitForHRDY();
	mutex_lock(&gMgeMutex);
	if (!SEND_CMD(WR_REG))	printk("%s() CSCL Error 1\n",__FUNCTION__);
	if (!SEND_PAR(Index))	printk("%s() CSCL Error 2\n",__FUNCTION__);
	if (!SEND_PAR(Value))	printk("%s() CSCL Error 3\n",__FUNCTION__);
	mutex_unlock(&gMgeMutex);
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
int s1d13522if_cmd(unsigned ioctlcmd,s1d13522_ioctl_cmd_params *params,int numparams)
{
	int i;
	unsigned cmd = ioctlcmd & 0xFF;

//	dbg_info("%s(): cmd=%xh numparams=%d\n",__FUNCTION__, cmd,numparams);
#if 0
	printk("%s(): cmd=0x%x ",__FUNCTION__, cmd);
	for (i = 0; i < numparams; i++)
		printk("0x%04x ",params->param[i]);
	printk("\n");
#endif

	if (s1d13522if_WaitForHRDY() != 0)
		return -1;

	mutex_lock(&gMgeMutex);
	if (!SEND_CMD(cmd))
		printk("%s() CSCL Error 1\n",__FUNCTION__);

	for (i = 0; i < numparams; i++) {
		if (!SEND_PAR(params->param[i])) {
			printk("%s() CSCL Error 2\n",__FUNCTION__);
		}
	}

	mutex_unlock(&gMgeMutex);
	return 0;
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void s1d13522if_BurstWrite16(u16 *ptr16, unsigned copysize16)
{
//	dbg_info("%s(): ptr16: 0x%x size16: %u\n",__FUNCTION__, (unsigned)ptr16,copysize16);

	mutex_lock(&gMgeMutex);
	if (!BURST_WRITE(copysize16*2, ptr16)) {
		printk("%s(): CSCL Error\n",__FUNCTION__);
	}

	mutex_unlock(&gMgeMutex);
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
void s1d13522if_BurstRead16(u16 *ptr16, unsigned copysize16)
{
	mutex_lock(&gMgeMutex);
#if 0
	while (copysize16-- > 0) {
		*ptr16++ = READ_DATA();
	}
#else
	if (!BURST_READ(copysize16*2, ptr16)) {
		printk("%s(): CSCL Error\n",__FUNCTION__);
	}
#endif
	mutex_unlock(&gMgeMutex);
}

module_init(usb_prometheus_init);
module_exit(usb_prometheus_exit);

MODULE_LICENSE("Dual BSD/GPL");
#endif // CONFIG_FB_EPSON_USB

