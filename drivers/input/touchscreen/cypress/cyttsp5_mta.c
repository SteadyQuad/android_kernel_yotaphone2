/*
 * cyttsp5_mta.c
 * Cypress TrueTouch(TM) Standard Product V5 Multi-Touch Protocol A Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2013 Cypress Semiconductor
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
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/input.h>
#include <linux/cyttsp5_core.h>

#include "cyttsp5_mt_common.h"

static void cyttsp5_final_sync(struct input_dev *input, int max_slots,
		int mt_sync_count, unsigned long *ids)
{
	if (mt_sync_count)
		input_sync(input);
}

static void cyttsp5_input_sync(struct input_dev *input)
{
	input_mt_sync(input);
}

static void cyttsp5_input_report(struct input_dev *input, int sig,
		int t, int type)
{
	if (type == CY_OBJ_STANDARD_FINGER || type == CY_OBJ_GLOVE
			|| type == CY_OBJ_HOVER) {
		input_report_key(input, BTN_TOOL_FINGER, CY_BTN_PRESSED);
		input_report_key(input, BTN_TOOL_PEN, CY_BTN_RELEASED);
	} else if (type == CY_OBJ_STYLUS) {
		input_report_key(input, BTN_TOOL_PEN, CY_BTN_PRESSED);
		input_report_key(input, BTN_TOOL_FINGER, CY_BTN_RELEASED);
	}

	if (type != CY_OBJ_HOVER)
		input_report_key(input, BTN_TOUCH, CY_BTN_PRESSED);

	input_report_abs(input, sig, t);
}

static void cyttsp5_report_slot_liftoff(struct cyttsp5_mt_data *md,
		int max_slots)
{
	input_report_key(md->input, BTN_TOUCH, CY_BTN_RELEASED);
	input_report_key(md->input, BTN_TOOL_FINGER, CY_BTN_RELEASED);
	input_report_key(md->input, BTN_TOOL_PEN, CY_BTN_RELEASED);

}

static int cyttsp5_input_register_device(struct input_dev *input, int max_slots)
{
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(BTN_TOOL_FINGER, input->keybit);
	__set_bit(BTN_TOOL_PEN, input->keybit);
	return input_register_device(input);
}

void cyttsp5_init_function_ptrs(struct cyttsp5_mt_data *md)
{
	md->mt_function.report_slot_liftoff = cyttsp5_report_slot_liftoff;
	md->mt_function.final_sync = cyttsp5_final_sync;
	md->mt_function.input_sync = cyttsp5_input_sync;
	md->mt_function.input_report = cyttsp5_input_report;
	md->mt_function.input_register_device = cyttsp5_input_register_device;
}

static int __init cyttsp5_mt_init(void)
{
	int rc;
	cyttsp5_mt_driver.driver.owner = THIS_MODULE;
	rc = cyttsp5_register_driver(&cyttsp5_mt_driver);
	pr_info("%s: Cypress TTSP v5 Multi-Touch A Driver (Built %s), rc=%d\n",
		 __func__, CY_DRIVER_DATE, rc);
	return rc;
}
//module_init(cyttsp5_mt_init);
deferred_module_init_0(cyttsp5_mt_init);

static void __exit cyttsp5_mt_exit(void)
{
	cyttsp5_unregister_driver(&cyttsp5_mt_driver);
}
module_exit(cyttsp5_mt_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product Multi-Touch A Driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
