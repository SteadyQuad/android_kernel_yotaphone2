/*
 * Gesture recognition driver: configuration module
 *
 * Copyright (c) 2013,2014,2015 YotaDevices.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/slab.h>
#include <linux/input.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include "gesture_driver.h"
#include "gesture_config.h"

#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_bus.h>
#include "cyttsp5_device_access.h"
#include "cyttsp5_regs.h"

#define CONFIG_NODE_BUF_SIZE (512)

static gesture_config_node_t gesture_config_nodes[CONFIG_NODES_COUNT] = {
    {{{0}, 0, 0}, "gestures"},
    {{{0}, 0, 0}, "enabled"},
    {{{0}, 0, 0}, "loglvl"},
    {{{0}, 0, 0}, "front_sensors"},
    {{{0}, 0, 0}, "back_sensors"},
    {{{0}, 0, 0}, "keys_enabled"},
    {{{0}, 0, 0}, "keys_disabled"},
    {{{0}, 0, 0}, "palmed_screen"}
};

static struct kobject *gesture_config_kobj;

/* Gesture names, keep in sync with gesture_type_t */
static const char *gesture_names[] = {
    "FRONT_GESTURE_LOCK_3FINGER",
    "FRONT_GESTURE_UNLOCK_SWIPE",
    "FRONT_GESTURE_UNLOCK_TAPTAP",
    "BACK_GESTURE_LOCK_3FINGER",
    "BACK_GESTURE_UNLOCK_SWIPE",
    "BACK_GESTURE_UNLOCK_TAPTAP",
    "BACK_DEEPSLEEP",
    "BACK_GESTURE_UNLOCK_TAPTAP_TOUCHDOWN"
};

atomic_t enabled_gestures = ATOMIC_INIT(0);
struct mutex config_mutex;

/* Enable/disable switch */
int gesture_driver_enabled = 1;

int palmed_algorithm = 0;

extern log_level_t log_level;

extern const struct input_device_id gesture_driver_ids[];
extern struct input_handler gesture_driver_handler;

u8 fs_sensor_data[CY_MAX_PRBUF_SIZE/2];
u8 bs_sensor_data[CY_MAX_PRBUF_SIZE/2];

/*
 * Gesture recognition enable/disable node functions
 */
static ssize_t onoff_show(struct kobject *kobj, struct kobj_attribute *attr,
                          char *buf) {
    return sprintf(buf, "%d\n", gesture_driver_enabled);
}

static ssize_t onoff_store(struct kobject *kobj, struct kobj_attribute *attr,
                           const char *buf, size_t count) {
    if (sscanf(buf, "%d", &gesture_driver_enabled)) {
        if (gesture_driver_enabled == 0) {
            input_unregister_handler(&gesture_driver_handler);
        } else if (gesture_driver_enabled == 1) {
            int result;
            result = input_register_handler(&gesture_driver_handler);
        }
    }
    return count;
}

/*
 * Log level select node functions
 */
static ssize_t loglvl_show(struct kobject *kobj, struct kobj_attribute *attr,
                           char *buf) {
    return sprintf(buf, "%d\n", log_level);
}

static ssize_t loglvl_store(struct kobject *kobj, struct kobj_attribute *attr,
                            const char *buf, size_t count) {
    log_level_t val;

    if (sscanf(buf, "%d", (int*) &val)) {
        if (val >= LOG_LEVEL_ERROR && val <= LOG_LEVEL_VERBOSE) {
            log_level = val;
        }
    }
    return count;
}

/*
 * Gesture configuration data node functions
 */
static size_t sensor_print(s16 *pdata16, u8 *buf, int rows, int cols)
{
    int i;
    size_t count = 0;
    for (i = 0; i < cols*rows; i++) {
        if ((i % cols) == 0)
            count += sprintf(buf+count, "\n");
        count += sprintf(buf+count, "%03d ", pdata16[i]);
    }
    count += sprintf(buf+count, "\n");
    return count;
}

static ssize_t front_sensors_show(struct kobject *kobj, struct kobj_attribute *attr,
                                  char *buf)
{
    u16 read_offset = 0, read_count = 0;
    u16 actual_read_len;
    u8 config = 0;
    size_t count = 0;

    config = 0;
    read_offset = 0;
    read_count = CY_FS_MUT_SIZE;
    cyttsp5_read_sensor_data(CYTTSP5_DEVICE_ACCESS_NAME, CY_FRONT_CORE_ID,
                             1, read_offset, read_count, CY_MUT_DIFF, NULL,
                             &config, &actual_read_len, fs_sensor_data);
    count += sprintf(buf+count, "\nMutual diff data: rows=%d, cols=%d\n", CY_FS_ROW, CY_FS_COLUMN);
    count += sensor_print((s16 *)fs_sensor_data, buf+count, CY_FS_ROW, CY_FS_COLUMN);

    config = 0;
    read_offset = 0;
    read_count = CY_FS_SELFCAP_SIZE;
    cyttsp5_read_sensor_data(CYTTSP5_DEVICE_ACCESS_NAME, CY_FRONT_CORE_ID,
                             1, read_offset, read_count, CY_SELF_DIFF, NULL,
                             &config, &actual_read_len, fs_sensor_data);
    count += sprintf(buf+count, "\nSelfcap diff data:\n");
    count += sensor_print((s16 *)fs_sensor_data, buf+count, 1, read_count);
    return count;
}

static ssize_t front_sensors_store(struct kobject *kobj, struct kobj_attribute *attr,
                                   const char *buf, size_t count)
{
    return count;
}

static ssize_t back_sensors_show(struct kobject *kobj, struct kobj_attribute *attr,
                                 char *buf)
{
    u16 read_offset = 0, read_count = 0;
    u16 actual_read_len;
    u8 config = 0;
    size_t count = 0;

    config = 0;
    read_offset = 0;
    read_count = CY_BS_MUT_SIZE;
    cyttsp5_read_sensor_data(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID,
                             1, read_offset, read_count, CY_MUT_DIFF, NULL,
                             &config, &actual_read_len, bs_sensor_data);
    count += sprintf(buf+count, "\nMutual diff data: rows=%d, cols=%d\n", CY_BS_ROW, CY_BS_COLUMN);
    count += sensor_print((s16 *)bs_sensor_data, buf+count, CY_BS_ROW, CY_BS_COLUMN);

    config = 0;
    read_offset = 0;
    read_count = CY_BS_SELFCAP_SIZE;
    cyttsp5_read_sensor_data(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID,
                             1, read_offset, read_count, CY_SELF_DIFF, NULL,
                             &config, &actual_read_len, bs_sensor_data);
    count += sprintf(buf+count, "\nSelfcap diff data:\n");
    count += sensor_print((s16 *)bs_sensor_data, buf+count, 1, read_count);
    return count;
}

static ssize_t back_sensors_store(struct kobject *kobj, struct kobj_attribute *attr,
                                  const char *buf, size_t count)
{
    return count;
}

/*
 *  Enabled Key IDs node functionality
 */
static ssize_t key_enabled_show(struct kobject *kobj, struct kobj_attribute *attr,
                                char *buf)
{
    int i, j, offset = 0;
    mutex_lock(&config_mutex);
    for (i = 1, j = 0; i <= GESTURE_TYPE_LAST; i <<= 1, j++) {
        if (atomic_read(&enabled_gestures) & i) {
            offset += sprintf(buf + offset, "%s=0x%x\n", gesture_names[j], i);
        }
    }
    mutex_unlock(&config_mutex);
    return offset;
}

static ssize_t key_enabled_store (struct kobject *kobj, struct kobj_attribute *attr,
                                  const char *buf, size_t count)
{
    int gesture = 0, current_gesture = 0;
    mutex_lock(&config_mutex);
    sscanf(buf, "%x", &gesture);
    LOG_V("key_enabled_store: 0x%x\n", gesture);
    current_gesture = atomic_read(&enabled_gestures);
    // Need to handle concurent gestures
    switch(gesture) {
    case FRONT_GESTURE_UNLOCK_SWIPE:
    case FRONT_GESTURE_UNLOCK_TAPTAP:
        current_gesture &= ~FRONT_UNLOCK_GESTURE_MASK;
        break;
    case BACK_GESTURE_UNLOCK_SWIPE:
    case BACK_GESTURE_UNLOCK_TAPTAP:
    case BACK_GESTURE_UNLOCK_TAPTAP_TOUCHDOWN:
    case BACK_DEEPSLEEP:
        current_gesture &= ~BACK_GESTURE_MASK;
        break;
    default:
        break;
    }
    atomic_set(&enabled_gestures, current_gesture | (gesture & GESTURE_TYPE_MASK));
    mutex_unlock(&config_mutex);
    return count;
}

/*
 *  Disabled Key IDs node functionality
 */
static ssize_t key_disabled_show(struct kobject *kobj, struct kobj_attribute *attr,
                                 char *buf)
{
    int i, j, offset = 0;
    mutex_lock(&config_mutex);
    for (i = 1, j = 0; i <= GESTURE_TYPE_LAST; i <<= 1, j++) {
        if (!(atomic_read(&enabled_gestures) & i)) {
            offset += sprintf(buf + offset, "%s=0x%x\n", gesture_names[j], i);
        }
    }
    mutex_unlock(&config_mutex);
    return offset;
}

static ssize_t key_disabled_store(struct kobject *kobj, struct kobj_attribute *attr,
                                  const char *buf, size_t count)
{
    int gesture = 0, current_gesture = 0;
    mutex_lock(&config_mutex);
    sscanf(buf, "%x", &gesture);
    LOG_V("key_disabled_store: 0x%x", gesture);
    current_gesture = atomic_read(&enabled_gestures);
    atomic_set(&enabled_gestures, current_gesture & (~gesture & GESTURE_TYPE_MASK));
    mutex_unlock(&config_mutex);
    return count;
}

static ssize_t palmed_screen_show(struct kobject *kobj, struct kobj_attribute *attr,
                                  char *buf)
{
    return sprintf(buf,"%d", get_palmed_screen(palmed_algorithm));
}

static ssize_t palmed_screen_store(struct kobject *kobj, struct kobj_attribute *attr,
                                   const char *buf, size_t count)
{
    int val;

    if (sscanf(buf, "%d", (int*) &val)) {
        if (val > 0) {
            palmed_algorithm = val;
        }
    }
    return count;
}

unsigned int get_unlock_gesture_enabled(int touch_id)
{
    unsigned int ret = 0;
    if (touch_id == FRONT_TOUCH_ID)
        ret = atomic_read(&enabled_gestures) & FRONT_UNLOCK_GESTURE_MASK;
    else if (touch_id == BACK_TOUCH_ID)
        ret = atomic_read(&enabled_gestures) & BACK_UNLOCK_GESTURE_MASK;
    return ret;
}

unsigned int get_lock_gesture_enabled(int touch_id)
{
    unsigned int ret = 0;
    if (touch_id == FRONT_TOUCH_ID)
        ret = atomic_read(&enabled_gestures) & FRONT_GESTURE_LOCK_3FINGER;
    else if (touch_id == BACK_TOUCH_ID)
        ret = atomic_read(&enabled_gestures) & BACK_GESTURE_LOCK_3FINGER;
    return ret;
}

unsigned int is_deepsleep_enabled(int touch_id)
{
    unsigned int ret = 0;
    if (touch_id == FRONT_TOUCH_ID)
        ret = !(atomic_read(&enabled_gestures) & (FRONT_GESTURE_UNLOCK_SWIPE | FRONT_GESTURE_UNLOCK_TAPTAP));
    else if (touch_id == BACK_TOUCH_ID)
        ret = atomic_read(&enabled_gestures) & BACK_DEEPSLEEP;
    return ret;
}

/*
 * config nodes file operations data init function
 */
static void init_node_data(void) {
    gesture_config_node_t *ptr;
    int i;

    ptr = &gesture_config_nodes[CONFIG_NODE_ENABLED];
    ptr->attribute.show = onoff_show;
    ptr->attribute.store = onoff_store;

    ptr = &gesture_config_nodes[CONFIG_NODE_LOGLVL];
    ptr->attribute.show = loglvl_show;
    ptr->attribute.store = loglvl_store;

    ptr = &gesture_config_nodes[CONFIG_NODE_FRONT_SENSORS];
    ptr->attribute.show = front_sensors_show;
    ptr->attribute.store = front_sensors_store;

    ptr = &gesture_config_nodes[CONFIG_NODE_BACK_SENSORS];
    ptr->attribute.show = back_sensors_show;
    ptr->attribute.store = back_sensors_store;

    ptr = &gesture_config_nodes[CONFIG_NODE_KEYS_ENABLED];
    ptr->attribute.show = key_enabled_show;
    ptr->attribute.store = key_enabled_store;

    ptr = &gesture_config_nodes[CONFIG_NODE_KEYS_DISABLED];
    ptr->attribute.show = key_disabled_show;
    ptr->attribute.store = key_disabled_store;

    ptr = &gesture_config_nodes[CONFIG_NODE_PALMED_SCREEN];
    ptr->attribute.show = palmed_screen_show;
    ptr->attribute.store = palmed_screen_store;

    for (i = CONFIG_NODE_ENABLED; i < CONFIG_NODES_COUNT; i++) {
        gesture_config_nodes[i].attribute.attr.name = gesture_config_nodes[i].name;
        gesture_config_nodes[i].attribute.attr.mode = CONFIG_NODE_PERMISSIONS;
    }
}

/*
 * config subsystem setup function
 */
int gesture_config_init(void) {
    int status = -ENOMEM, i = 0;

    mutex_init(&config_mutex);
    init_node_data();

    do {
        /* First setup a sysfs kobject for front screen gesture driver */
        gesture_config_kobj =\
            kobject_create_and_add(gesture_config_nodes[CONFIG_NODE_DIRECTORY].name,  kernel_kobj);

        if (!gesture_config_kobj) {
            break;
        }

        for (i = CONFIG_NODE_ENABLED; i < CONFIG_NODES_COUNT; i++) {
            if (sysfs_create_file(gesture_config_kobj,                  \
                                  &gesture_config_nodes[i].attribute.attr)) {
                break;
            }
        }
        status = 0;
    } while (0);

    if (status) {
        LOG_E("%s: %d:%s init failed.\n", __func__, i, gesture_config_nodes[i].name);
        gesture_config_terminate();
    }

    return status;
}

/*
 * config subsystem shutdown function
 */
void gesture_config_terminate(void) {
    kobject_put(gesture_config_kobj);
}
