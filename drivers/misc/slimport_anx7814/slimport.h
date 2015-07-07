/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _SLIMPORT_H
#define _SLIMPORT_H

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/async.h>
//#include <linux/slimport.h>

#include "slimport_tx_drv.h"
#include "slimport_tx_reg.h"

#include <linux/of_gpio.h>
#include <linux/of_platform.h>
//#include <mach/board_lge.h>
#include <mach/msm_smem.h>

#define SSC_EN
 

#define AUX_ERR  1
#define AUX_OK   0

#define LOG_TAG "SlimPort Colorado3"
#define __func__  ""

struct anx7816_data {
	struct anx7816_platform_data    *pdata;
	struct delayed_work    work;
	struct workqueue_struct    *workqueue;
	struct mutex    lock;
	struct wake_lock slimport_lock;
	struct msm_hdmi_sp_ops *hdmi_sp_ops; // slimport changes
	struct platform_device *hdmi_pdev; // slimport changes
};
void slimport_main_process(struct anx7816_data *);
struct msm_hdmi_sp_ops {
        int (*set_upstream_hpd)(struct platform_device *pdev, uint8_t on);
};

extern enum SP_TX_System_State sp_tx_system_state;
extern enum RX_CBL_TYPE sp_tx_rx_type;

extern bool is_slimport_vga(void);
extern bool is_slimport_dp(void);

int sp_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf);
int sp_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value);
void sp_tx_hardware_poweron(void);
void sp_tx_hardware_powerdown(void);
int slimport_read_edid_block(int block, uint8_t *edid_buf);
unchar sp_get_link_bw(void);
void sp_set_link_bw(unchar link_bw);
 
bool slimport_is_connected(void);

 #endif
