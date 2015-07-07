/*
 * Header file for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) touchscreen drivers.
 * For use with Cypress Txx2xx and Txx3xx parts.
 * Supported parts include:
 * CY8CTST242
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009-2012 Cypress Semiconductor, Inc.
 * Copyright (C) 2010-2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */

#ifndef __CYTTSP_CORE_H__
#define __CYTTSP_CORE_H__

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/err.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define CY_I2C_NAME                 "cyttsp3-i2c"
#define CY_SPI_NAME                 "cyttsp3-spi"
#define CY_DRIVER_VERSION           "Rev3-2M-28"
#define CY_DRIVER_DATE              "2012-02-07"

#define CY_NUM_RETRY                10 /* max retries for rd/wr ops */

/* Required firmware settings for the Cypress Touchscreen Driver */
/*
 * Set COM_ENABLE_IN_DEEP_SLEEP to 1 if using wake on bus activity
 * Set COM_ENABLE_IN_DEEP_SLEEP to 0 if using host strobe on int pin
 *
 * Set COM_HANDSHAKE_OP_MODE to 1 to use handshake (use handshake in all modes)
 *
 * Set LARGE_OBJECT_SEND_ONCE to 1
 *
 * Set COM_INT_PULSE_FIRST to 0 if firmware uses HOST_CHANGE_MODE_BIT
 *
 * Set APPLICATION_DEVELOPMENT_MODE: equ 0 to execute bootloader on startup
 *
 * Set appropriate i2c pins as required (P1.0/P1.1 or P1.5/P1.7)
 *
 * Verify SPI MISO pin is strong drive
 *
 * When generating the IIC file:
 *   For all TMA340 devices: the first image block = 0x16
 *   For all TMG240/TST241 devices: the first image block is 0x11
 *
 *   For all TMA340, TMG240, TST241 devices: the checksum block is 0x0F
 */

// Use the following define if the device is Txx2xx
#define CY_USE_GEN2

/* Use the following define if the device is Txx3xx
#define CY_USE_GEN3
 */

/* default is to build for Txx3xx */
#if !defined(CY_USE_GEN2) && !defined(CY_USE_GEN3)
#define CY_USE_GEN3
#elif defined(CY_USE_GEN2) && defined(CY_USE_GEN3)
#undef CY_USE_GEN2
#endif

/* use the following define to enable driver watchdog timer
#define CY_USE_WATCHDOG
 */

/* Use the following define if the firmware does not include bootloader
#define CY_NO_BOOTLOADER
 */

/* Use the following define to enable any difference in the auto load
 * image firmware application version number and the device image
 * firmware application version to allow auto load at startup
#define CY_ANY_DIFF_NEW_VER
 */

/* use the following define to enable register peak/poke capability*/
#define CY_USE_REG_ACCESS


/* Use the following define to use level interrupt method (else falling edge)
 * this method should only be used if the host processor misses edge interrupts
#define CY_USE_LEVEL_IRQ
 */

/*
 * Use the following define to enable manual controls of Charge Armor(tm)
 * mode and support query status;  this define will make sure
 * either enable CY_USE_REG_ACCESS or CONFIG_TOUCHSCREEN_DEBUG
#define CY_ENABLE_CA
 */
#ifdef CY_ENABLE_CA
#if !defined(CY_USE_REG_ACCESS) && !defined(CONFIG_TOUCHSCREEN_DEBUG)
#define CONFIG_TOUCHSCREEN_DEBUG
#endif
#endif

/*Use the following define to enable special debug tools for test only*/
#define CY_USE_DEBUG_TOOLS

#define CONFIG_TOUCHSCREEN_DEBUG

#ifdef CONFIG_TOUCHSCREEN_DEBUG
#define DEBUG = y                       /* enable dev_dbg() */
/* Use the following defines for dynamic debug printing */
/*
 * Level 0: Default Level
 * All debug (cyttsp_dbg) prints turned off
 */
#define CY_DBG_LVL_0			0
/*
 * Level 1:  Used to verify driver and IC are working
 *    Input from IC, output to event queue
 */
#define CY_DBG_LVL_1			1
/*
 * Level 2:  Used to further verify/debug the IC
 *    Output to IC
 */
#define CY_DBG_LVL_2			2
/*
 * Level 3:  Used to further verify/debug the driver
 *    Driver internals
 */
#define CY_DBG_LVL_3			3
#define CY_DBG_LVL_MAX			CY_DBG_LVL_3


#define cyttsp_dbg(ts, l, f, a...) {\
	if (ts->bus_ops->tsdebug >= l) \
		dev_dbg(ts->dev, f, ## a);\
}
#else
#define cyttsp_dbg(ts, l, f, a...)
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

#ifdef CY_USE_DEBUG_TOOLS
#define CY_DBG_SUSPEND                  4
#define CY_DBG_RESUME                   5
#define CY_DBG_RESTART                  98
#endif



struct cyttsp_bus_ops {
	int (*write)(void *handle, u8 addr, u8 length, const void *values);
	int (*read)(void *handle, u8 addr, u8 length, void *values);
	struct device *dev;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	u8 tsdebug;
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
};

void *cyttsp_core_init(struct cyttsp_bus_ops *bus_ops,
	struct device *dev, int irq, char *name,
	const struct of_device_id *match);

void cyttsp_core_release(void *handle);
#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
int cyttsp_resume(void *handle);
int cyttsp_suspend(void *handle);
#endif

extern int driver_fail;

#endif /* __CYTTSP_CORE_H__ */
