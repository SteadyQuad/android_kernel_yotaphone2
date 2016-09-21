/*
 * Gesture recognition driver
 *
 * Copyright (c) 2013,2014,2015 YotaDevices.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/init.h>
#include <linux/device.h>
#include "gesture_config.h"
#include "gesture_driver.h"

#define FS_INPUT_DRIVER_NAME "cyttsp5_mt_front"
#define BS_INPUT_DRIVER_NAME "cyttsp5_mt_back"
#define CAPS_INPUT_DRIVER_NAME "cyttsp3-i2c"

static bool gesture_driver_event(struct input_handle *handle, unsigned int type, unsigned int code,
                                 int value);
static bool gesture_driver_match(struct input_handler *handler, struct input_dev *dev);
static int gesture_driver_connect(struct input_handler *handler, struct input_dev *dev,
                                  const struct input_device_id *id);
static void gesture_driver_disconnect(struct input_handle *handle);
static int gesture_ts_init(void);
static void gesture_ts_exit(void);
void gesture_input_dev_fill(void);
static int virtual_input_device_init(struct input_dev **dev, const char *name, int xmax, int ymax);
static void virtual_input_device_exit(struct input_dev **dev);

struct input_dev *gestureInputDev = NULL;
struct input_dev *frontVirtualTPInputDev = NULL;
struct input_dev *backVirtualTPInputDev = NULL;

log_level_t log_level = LOG_LEVEL_ERROR;
atomic_t front_touch_locked = ATOMIC_INIT(0);
atomic_t back_touch_locked = ATOMIC_INIT(0);

const struct input_device_id gesture_driver_ids[] = {
	{ .driver_info = 1 },
	{},         /* Terminating zero entry */
};

struct input_handler gesture_driver_handler = {
	.filter = gesture_driver_event,
	.match = gesture_driver_match,
	.connect =  gesture_driver_connect,
	.disconnect =   gesture_driver_disconnect,
	.name =     "gesture_filter",
	.id_table = gesture_driver_ids,
};

/*
 * Gesture driver event handler callback
 */
static bool gesture_driver_event(struct input_handle *handle, unsigned int type, unsigned int code,
                                 int value)
{
	//LOG_V("%s: type: %d, code: %d, value: %d\n", handle->dev->name, type, code, value);

	if (strcmp(FS_INPUT_DRIVER_NAME,handle->dev->name) == 0) {
		if (frontVirtualTPInputDev != NULL) {
			input_event(frontVirtualTPInputDev, type, code, value);
		}
	}
	else if (strcmp(BS_INPUT_DRIVER_NAME,handle->dev->name) == 0) {
		if (backVirtualTPInputDev != NULL) {
			input_event(backVirtualTPInputDev, type, code, value);
		}
	}
	else if (strcmp(CAPS_INPUT_DRIVER_NAME,handle->dev->name) == 0) {
		// don't send event
	}
	return true;
}

/*
 * Gesture driver match callback function
 */
static bool gesture_driver_match(struct input_handler *handler, struct input_dev *dev)
{
	if ((strcmp(FS_INPUT_DRIVER_NAME,dev->name) == 0) || (strcmp(BS_INPUT_DRIVER_NAME,dev->name) == 0) || (strcmp(CAPS_INPUT_DRIVER_NAME,dev->name) == 0)) {
		return true;
	}
	return false;
}

/*
 * Gesture driver connect callback function.
 */
static int gesture_driver_connect(struct input_handler *handler, struct input_dev *dev,
                                  const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;
	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "Gesture driver";

	error = input_register_handle(handle);
	if (error)
		goto err_free_handle;

	error = input_open_device(handle);
	if (error)
		goto err_unregister_handle;

	LOG_V("Connected device: %s (%s at %s)\n",
	      dev_name(&dev->dev),
	      dev->name ?: "unknown",
	      dev->phys ?: "unknown");

	return 0;

 err_unregister_handle: input_unregister_handle(handle);
 err_free_handle: kfree(handle);
	return error;
}

/*
 * Gesture driver disconnect callback function
 */
static void gesture_driver_disconnect(struct input_handle *handle) {
	LOG_V("gesture_driver_disconnect: %s\n", dev_name(&handle->dev->dev));

	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

/*
 * Gesture driver initialization
 */
static int __init gesture_driver_init(void)
{
	gesture_ts_init();
	virtual_input_device_init(&frontVirtualTPInputDev, "fs_virtual_ts",
				  FRONT_RIGHT_BOTTOM_X, FRONT_RIGHT_BOTTOM_Y);
	virtual_input_device_init(&backVirtualTPInputDev, "bs_virtual_ts",
				  BACK_RIGHT_BOTTOM_X, BACK_RIGHT_BOTTOM_Y);
	gesture_config_init();
	return input_register_handler(&gesture_driver_handler);
}

/*
 * Gesture driver exit function
 */
static void __exit gesture_driver_exit(void)
{
	gesture_config_terminate();
	gesture_ts_exit();
	virtual_input_device_exit(&frontVirtualTPInputDev);
	virtual_input_device_exit(&backVirtualTPInputDev);
	input_unregister_handler(&gesture_driver_handler);
}

/*
 * Register gesture input device
 */
static int gesture_ts_init(void)
{
	int error;

	LOG_V("gesture_ts_init\n");
	gestureInputDev = input_allocate_device();
	if (!gestureInputDev) {
		LOG_E("Failed to allocate memory\n");
		return -ENOMEM;
	}

	gestureInputDev->name = "gestures";
	gestureInputDev->id.bustype = BUS_VIRTUAL;
	gestureInputDev->dev.parent = NULL;

	//TODO get key IDs from gestures
	__set_bit(EV_KEY, gestureInputDev->evbit);

	gesture_input_dev_fill();

	error = input_register_device(gestureInputDev);
	if (error) {
		input_free_device(gestureInputDev);
		gestureInputDev = NULL;
		LOG_E("Failed to register device\n");
	}

	return error;
}

/*
 * Unregister gesture input device
 */
static void gesture_ts_exit(void)
{
	LOG_V("gesture_ts_exit\n");
	if (gestureInputDev != NULL) {
		input_unregister_device(gestureInputDev);
		gestureInputDev = NULL;
	}
}

/*
 * Fill gestureInputDev fields from gesture data
 */
void gesture_input_dev_fill(void)
{
	memset(gestureInputDev->keybit, 0, BITS_TO_LONGS(KEY_CNT));

	__set_bit(KEY_FS_GESTURE_LOCK, gestureInputDev->keybit);
	__set_bit(KEY_FS_GESTURE_UNLOCK, gestureInputDev->keybit);
	__set_bit(KEY_BS_GESTURE_LOCK, gestureInputDev->keybit);
	__set_bit(KEY_BS_GESTURE_UNLOCK, gestureInputDev->keybit);
}

/*
 * Virtual input device init
 */
static int virtual_input_device_init(struct input_dev **dev, const char *name, int xmax, int ymax)
{
	int error;

	LOG_V("virtual_input_device_init, name=%s\n", name);
	*dev =  (struct input_dev *)input_allocate_device();
	if (!*dev) {
		LOG_E("Failed to allocate memory\n");
		return -ENOMEM;

	}

	((struct input_dev *)*dev)->name = name;
	((struct input_dev *)*dev)->id.bustype = BUS_VIRTUAL;
	((struct input_dev *)*dev)->dev.parent = NULL;
	set_bit(EV_SYN, ((struct input_dev *)*dev)->evbit);
	set_bit(EV_KEY, ((struct input_dev *)*dev)->evbit);
	set_bit(EV_ABS, ((struct input_dev *)*dev)->evbit);

	set_bit(INPUT_PROP_DIRECT, ((struct input_dev *)*dev)->propbit);

	/* For multi touch */
	input_mt_init_slots(*dev, 10);

	input_set_abs_params((struct input_dev *)*dev, ABS_MT_POSITION_X, 0, xmax, 0, 0);
	input_set_abs_params((struct input_dev *)*dev, ABS_MT_POSITION_Y, 0, ymax, 0, 0);

	input_set_abs_params((struct input_dev *)*dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params((struct input_dev *)*dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params((struct input_dev *)*dev, ABS_MT_PRESSURE, 0, 255, 0, 0);

	error = input_register_device((struct input_dev *)*dev);
	if (error) {
		input_free_device((struct input_dev *)*dev);
		*dev = NULL;
		LOG_E("Failed to register device\n");
	}

	return error;
}

/*
 * Unregister virtual input device
 */
static void virtual_input_device_exit(struct input_dev **dev)
{
	if (*dev != NULL) {
		input_unregister_device((struct input_dev *)*dev);
		*dev = NULL;
	}
}

void send_gesture_event(unsigned int key)
{
	LOG_V("send_gesture_event: 0x%x\n", key);
	if (gestureInputDev != NULL) {
		input_event(gestureInputDev, EV_KEY, key, 1);
		input_event(gestureInputDev, EV_SYN, SYN_REPORT, 0);
		input_event(gestureInputDev, EV_KEY, key, 0);
		input_event(gestureInputDev, EV_SYN, SYN_REPORT, 0);
	}
}

void driver_touch_lock(int touch_id)
{
	if (touch_id == FRONT_TOUCH_ID)
		atomic_set(&front_touch_locked, 1);
	else if (touch_id == BACK_TOUCH_ID)
		atomic_set(&back_touch_locked, 1);
}

void driver_touch_unlock(int touch_id)
{
	if (touch_id == FRONT_TOUCH_ID)
		atomic_set(&front_touch_locked, 0);
	else if (touch_id == BACK_TOUCH_ID)
		atomic_set(&back_touch_locked, 0);
}

int driver_is_touch_locked(int touch_id)
{
	if (touch_id == FRONT_TOUCH_ID)
		return atomic_read(&front_touch_locked);
	else if (touch_id == BACK_TOUCH_ID)
		return atomic_read(&back_touch_locked);
	else
		return 0;
}

MODULE_AUTHOR("Alexander Dubinin <Alexander.Dubinin@Symphonyteleca.com>");
MODULE_DESCRIPTION("Gesture recognition driver");
MODULE_LICENSE("GPL");

MODULE_DEVICE_TABLE(input, gesture_driver_ids);

module_init(gesture_driver_init);
module_exit(gesture_driver_exit);
