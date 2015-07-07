/*
 * Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) platform module.
 * For use with Cypress Txx2xx and Txx3xx parts.
 * Supported parts include:
 * CY8CTST242
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009-2012 Cypress Semiconductor, Inc.
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

#include <linux/input/cyttsp3_touch_platform.h>

extern struct cyttsp3_touch_firmware cyttsp3_firmware;

int cyttsp3_hw_reset(const struct cyttsp3_touch_platform_data *pdata);
int cyttsp3_hw_recov(const struct cyttsp3_touch_platform_data *pdata, int on);
int cyttsp3_irq_stat(const struct cyttsp3_touch_platform_data *pdata);
