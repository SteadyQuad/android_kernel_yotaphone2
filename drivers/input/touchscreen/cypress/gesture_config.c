/*
 * Gesture recognition driver: configuration module
 *
 * Copyright (c) 2013,2014 YotaDevices.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#define DEBUG

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
#include "gesture_front.h"
#include "gesture_back.h"
#include "gesture_config.h"

#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_bus.h>
#include "cyttsp5_device_access.h"
#include "cyttsp5_regs.h"

#define CONFIG_NODE_BUF_SIZE (512)

#define BASE_DECIMAL (10)

#define GESTURES_TOTAL_COUNT (SINGLE_TOUCH_MAX_GESTURES_COUNT + DOUBLE_TOUCH_MAX_GESTURES_COUNT)

/* hom much space do values take in string that is written to data node */
#define CHARS_PER_DATA_VALUE (6)

#define CY_FS_ROW    28
#define CY_FS_COLUMN 16
#define CY_BS_ROW    13
#define CY_BS_COLUMN 22

static gesture_config_node_t gesture_config_nodes[CONFIG_NODES_COUNT] = {
    {{{0}, 0, 0}, "gestures"},
    {{{0}, 0, 0}, "enabled"},
    {{{0}, 0, 0}, "orientation"},
    {{{0}, 0, 0}, "loglvl"},
    {{{0}, 0, 0}, "data"},
    {{{0}, 0, 0}, "keys_enabled"},
    {{{0}, 0, 0}, "keys_disabled"},
    {{{0}, 0, 0}, "filter"},
    {{{0}, 0, 0}, "palmed_screen"}
};

static struct kobject *gesture_config_kobj;

char *configDataBuffer = 0;
unsigned int configDataBufferSize = 0;

extern log_level_t log_level;
extern screen_orientation_t orient_state;
extern int gestFsEnabled;
extern touch_accumulator_t tempTouchAcc;

extern single_gesture_t singleTouchGestures[SINGLE_TOUCH_MAX_GESTURES_COUNT];
extern int singleTouchGesturesCount;

extern double_touch_gesture_t doubleTouchGestures[DOUBLE_TOUCH_MAX_GESTURES_COUNT];
extern int doubleTouchGesturesCount;

extern gesture_t gestures[SINGLE_TOUCH_MAX_GESTURES_COUNT + DOUBLE_TOUCH_MAX_GESTURES_COUNT];
extern int gesturesCount;

extern const struct input_device_id gesture_driver_ids[];
extern struct input_handler gesture_driver_handler;

extern int filterEnabled;

extern bs_gesture_t bs_gestures[];
extern bs_gesture_status_t bsStatus;

static void bs_key_id_enable(int key_id);
static void bs_key_id_disable(int key_id);

static u8 fs_sensor_data[CY_MAX_PRBUF_SIZE/2];
static u8 bs_sensor_data[CY_MAX_PRBUF_SIZE/2];

/* Helper functions */

/*
 * Print single gesture data
 */
static int print_single_gesture_config(char *buffer, single_gesture_t *gesture,\
    data_table_id_t type) {
    int j, offset;

    offset = 0;
    offset += sprintf(buffer + offset, "%6d", type);
    offset += sprintf(buffer + offset, "%6d", gesture->startPointArea.leftUpX);
    offset += sprintf(buffer + offset, "%6d",gesture->startPointArea.leftUpY);
    offset += sprintf(buffer + offset, "%6d",gesture->startPointArea.rightBottomX);
    offset += sprintf(buffer + offset, "%6d",gesture->startPointArea.rightBottomY);
    offset += sprintf(buffer + offset, "%6d",gesture->endPointArea.leftUpX);
    offset += sprintf(buffer + offset, "%6d",gesture->endPointArea.leftUpY);
    offset += sprintf(buffer + offset, "%6d",gesture->endPointArea.rightBottomX);
    offset += sprintf(buffer + offset, "%6d",gesture->endPointArea.rightBottomY);
    offset += sprintf(buffer + offset, "%6d",gesture->minDx);
    offset += sprintf(buffer + offset, "%6d",gesture->minDy);
    offset += sprintf(buffer + offset, "%6d",gesture->maxDx);
    offset += sprintf(buffer + offset, "%6d",gesture->maxDy);
    offset += sprintf(buffer + offset, "%6d",gesture->minTime);
    offset += sprintf(buffer + offset, "%6d",gesture->maxTime);
    offset += sprintf(buffer + offset, "%6d",gesture->keyId);
    offset += sprintf(buffer + offset, "%6d",gesture->needSendKeyDown);
    offset += sprintf(buffer + offset, "%6d",gesture->isLongTapGesture);
    offset += sprintf(buffer + offset, "%6d",gesture->isDetectOnTouchUp);
    offset += sprintf(buffer + offset, "%6d",gesture->touchAccPointsCount);
    for (j = 0; j < gesture->touchAccPointsCount; j++) {
        offset += sprintf(buffer + offset, "%6ld",gesture->touchAccPoints[j].x);
        offset += sprintf(buffer + offset, "%6ld",gesture->touchAccPoints[j].y);
    }

    buffer[offset++] = LINE_FEED;
    return offset;
}

static bool remove_spaces(char *str) {
    bool num_occured = false;
    bool is_negative = false;

    while (*str) {
        if (*str >= '0' && *str <= '9') {
            num_occured = true;
        }
        else {
            if (*str == '-') {
                is_negative = true;
            }
            if (num_occured) {
                *str = '\0';
                break;
            }
            else {
                *str = '0';
            }
        }
        str++;
    }
    return is_negative;
}

/*
 * Load single gesture data from parameter string
 */
static int gesture_from_string(char *buf, single_gesture_t *gesture) {
    char value[CHARS_PER_DATA_VALUE+1] = {0};
    int i,j,status,sign = -EINVAL;
    unsigned char tmp_bool;

    do {
    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_START_POINT_AREA_LEFT_UP_X],\
        CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->startPointArea.leftUpX))) {
        LOG_E("%s parse failed for: startPointArea.leftUpX\n",__func__);
        break;
    }

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_START_POINT_AREA_LEFT_UP_Y],\
        CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->startPointArea.leftUpY))) {
        LOG_E("%s parse failed for: startPointArea.leftUpY\n",__func__);
        break;
    }

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_START_POINT_AREA_RIGHT_BOTTOM_X],\
        CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->startPointArea.rightBottomX))) {
        LOG_E("%s parse failed for: startPointArea.rightBottomX\n",__func__);
        break;
    }

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_START_POINT_AREA_RIGHT_BOTTOM_Y],\
        CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->startPointArea.rightBottomY))) {
        LOG_E("%s parse failed for: startPointArea.rightBottomY\n",__func__);
        break;
    }

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_END_POINT_AREA_LEFT_UP_X],\
        CHARS_PER_DATA_VALUE);
    sign = (remove_spaces(value)) ? -1 : 1;
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->endPointArea.leftUpX))) {
        LOG_E("%s parse failed for: endPointArea.leftUpX\n",__func__);
        break;
    }
    gesture->endPointArea.leftUpX *= sign;

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_END_POINT_AREA_LEFT_UP_Y],\
        CHARS_PER_DATA_VALUE);
    sign = (remove_spaces(value)) ? -1 : 1;
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->endPointArea.leftUpY))) {
        LOG_E("%s parse failed for: endPointArea.leftUpY\n",__func__);
        break;
    }
    gesture->endPointArea.leftUpY *= sign;

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_END_POINT_AREA_RIGHT_BOTTOM_X],\
        CHARS_PER_DATA_VALUE);
    sign = (remove_spaces(value)) ? -1 : 1;
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->endPointArea.rightBottomX))) {
        LOG_E("%s parse failed for: endPointArea.rightBottomX\n",__func__);
        break;
    }
    gesture->endPointArea.rightBottomX *= sign;

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_END_POINT_AREA_RIGHT_BOTTOM_Y],\
        CHARS_PER_DATA_VALUE);
    sign = (remove_spaces(value)) ? -1 : 1;
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->endPointArea.rightBottomY))) {
        LOG_E("%s parse failed for: endPointArea.rightBottomY\n",__func__);
        break;
    }
    gesture->endPointArea.rightBottomY *= sign;

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_MIN_DX], CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->minDx))) {
        LOG_E("%s parse failed for: minDx\n",__func__);
        break;
    }

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_MIN_DY], CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->minDy))) {
        LOG_E("%s parse failed for: minDy\n",__func__);
        break;
    }

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_MAX_DX], CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->maxDx))) {
        LOG_E("%s parse failed for: maxDx\n",__func__);
        break;
    }

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_MAX_DY], CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->maxDy))) {
        LOG_E("%s parse failed for: maxDy\n",__func__);
        break;
    }

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_MIN_TIME], CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->minTime))) {
        LOG_E("%s parse failed for: minTime\n",__func__);
        break;
    }

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_MAX_TIME], CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->maxTime))) {
        LOG_E("%s parse failed for: maxTime\n",__func__);
        break;
    }

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_KEY_ID], CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &gesture->keyId))) {
        LOG_E("%s parse failed for: keyId\n",__func__);
        break;
    }

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_NEED_SEND_KEY_DOWN],\
        CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtou8(value, BASE_DECIMAL, &tmp_bool))) {
        LOG_E("%s parse failed for: needSendKeyDown\n",__func__);
        break;
    }
    gesture->needSendKeyDown = (bool)tmp_bool;

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_IS_TAP_GESTURE], CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtou8(value, BASE_DECIMAL, &tmp_bool))) {
        LOG_E("%s parse failed for: isTapGesture\n",__func__);
        break;
    }
    gesture->isLongTapGesture = (bool)tmp_bool;

    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_IS_DETECT_ON_TOUCH_UP],\
        CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtou8(value, BASE_DECIMAL, &tmp_bool))) {
        LOG_E("%s parse failed for: isDetectOnTouchUp\n",__func__);
        break;
    }
    gesture->isDetectOnTouchUp = (bool)tmp_bool;

    tempTouchAcc.pointsCount = 0;
    memcpy(value, &buf[CHARS_PER_DATA_VALUE * PARAM_INDEX_TOUCH_ACC_POINTS_COUNT],\
        CHARS_PER_DATA_VALUE);
    remove_spaces(value);
    if (0 != (status = kstrtoint(value, BASE_DECIMAL, &tempTouchAcc.pointsCount))) {
        LOG_E("%s parse failed for: tempTouchAcc.pointsCount\n",__func__);
        break;
    }

    for (i = 0, j = 1; i < tempTouchAcc.pointsCount; i++, j +=2) {
        memcpy(value, &buf[CHARS_PER_DATA_VALUE * (PARAM_INDEX_TOUCH_ACC_POINTS_COUNT + j)],\
            CHARS_PER_DATA_VALUE);
        remove_spaces(value);
        printk(KERN_ERR "%s _%s_\n",__func__,value);
        if (0 != (status = kstrtol(value, BASE_DECIMAL, &tempTouchAcc.points[i].x))) {
            LOG_E("%s parse failed for: tempTouchAcc.points[%d].x\n",__func__, i);
            break;
        }
        memcpy(value, &buf[CHARS_PER_DATA_VALUE * (PARAM_INDEX_TOUCH_ACC_POINTS_COUNT + j + 1)],\
            CHARS_PER_DATA_VALUE);
        remove_spaces(value);
        if (0 != (status = kstrtol(value, BASE_DECIMAL, &tempTouchAcc.points[i].y))) {
            LOG_E("%s parse failed for: tempTouchAcc.points[%d].y\n",__func__, i);
            break;
        }
        printk(KERN_ERR "%s _%ld_\n",__func__,tempTouchAcc.points[i].y);
    }

    gesture->touchAccPointsCount = tempTouchAcc.pointsCount;
    memcpy(gesture->touchAccPoints, tempTouchAcc.points,\
    tempTouchAcc.pointsCount * sizeof(input_point_t));

    normalize_size(&tempTouchAcc);
    normalize_spacing(gesture, &tempTouchAcc);
    normalize_center(gesture);

    } while (0);

    return status;
}

static void bs_key_id_enable(int key_id) {
    int i;
    do {
        if (key_id == KEY_FS_GESTURE_LOCK) {
            bsStatus.fs_lock_enabled = true;
            break;
        }
        if (key_id == KEY_FS_GESTURE_UNLOCK) {
            bsStatus.fs_unlock_enabled = true;
            break;
        }
        if (key_id == KEY_BS_GESTURE_LOCK) {
            bsStatus.bs_lock_enabled = true;
            break;
        }
        if (key_id == KEY_BS_GESTURE_UNLOCK) {
            bsStatus.bs_unlock_enabled = true;
            break;
        }
        if (key_id == KEY_BS_GESTURE_DOUBLE_TAP) {
            bsStatus.dt_status.is_dt_enabled = true;
            break;
        }
        if (key_id == KEY_BS_GESTURE_SCROLL_RIGHT\
          || key_id == KEY_BS_GESTURE_SCROLL_LEFT) {
            bsStatus.is_scroll_enabled = true;
            break;
        }
        for(i = 0; i < BS_MAX_GESTURES_COUNT; i++) {
            if (key_id == bs_gestures[i].key_id) {
                bs_gestures[i].enabled = true;
                break;
            }
        }
    }while (0);
}

static void bs_key_id_disable(int key_id) {
    int i;
    do {
        if (key_id == KEY_FS_GESTURE_LOCK) {
            bsStatus.fs_lock_enabled = false;
            break;
        }
        if (key_id == KEY_FS_GESTURE_UNLOCK) {
            bsStatus.fs_unlock_enabled = false;
            break;
        }
        if (key_id == KEY_BS_GESTURE_LOCK) {
            bsStatus.bs_lock_enabled = false;
            break;
        }
        if (key_id == KEY_BS_GESTURE_UNLOCK) {
            bsStatus.bs_unlock_enabled = false;
            break;
        }
        if (key_id == KEY_BS_GESTURE_DOUBLE_TAP) {
            bsStatus.dt_status.is_dt_enabled = false;
            break;
        }
        if (key_id == KEY_BS_GESTURE_SCROLL_RIGHT\
          || key_id == KEY_BS_GESTURE_SCROLL_LEFT) {
            bsStatus.is_scroll_enabled = false;
            break;
        }
        for(i = 0; i < BS_MAX_GESTURES_COUNT; i++) {
            if (key_id == bs_gestures[i].key_id) {
                bs_gestures[i].enabled = false;
                break;
            }
        }
    } while (0);
}

/* sys fs definitions */

/*
 * Device orientation node functions
 */
static ssize_t orientation_show(struct kobject *kobj, struct kobj_attribute *attr,
                        char *buf) {
        return sprintf(buf, "%d\n", orient_state);
}

static ssize_t orientation_store(struct kobject *kobj, struct kobj_attribute *attr,
                         const char *buf, size_t count) {
    screen_orientation_t val;

    if (sscanf(buf, "%d", (int*) &val)) {
        switch (val) {
        case SCREEN_ORIENTATION_ROTATE_0:
        case SCREEN_ORIENTATION_ROTATE_180:
            orient_state = val;
            break;
        default:
            break;
        }
    }

    return count;
}

/*
 * Gesture recognition enable/disable node functions
 */
static ssize_t onoff_show(struct kobject *kobj, struct kobj_attribute *attr,
                        char *buf) {
        return sprintf(buf, "%d\n", gestFsEnabled);
}

static ssize_t onoff_store(struct kobject *kobj, struct kobj_attribute *attr,
                         const char *buf, size_t count) {
    if (sscanf(buf, "%d", &gestFsEnabled)) {
        if (gestFsEnabled == 0) {
            input_unregister_handler(&gesture_driver_handler);
        } else if (gestFsEnabled == 1) {
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
static ssize_t data_show(struct kobject *kobj, struct kobj_attribute *attr,
                        char *buf) {
    int i,elements, offset = 0;

    elements = FIXED_PARAMS_PER_GESTURE * (SINGLE_TOUCH_MAX_GESTURES_COUNT +\
                DOUBLE_TOUCH_MAX_GESTURES_COUNT * 2);
    for (i = 0; i < SINGLE_TOUCH_MAX_GESTURES_COUNT; i++) {
        elements += 2 * singleTouchGestures[i].touchAccPointsCount;//x and y
    }
    for (i = 0; i < DOUBLE_TOUCH_MAX_GESTURES_COUNT; i++) {
        elements += 2 * doubleTouchGestures[i].gesture1.touchAccPointsCount;//x and y
        elements += 2 * doubleTouchGestures[i].gesture2.touchAccPointsCount;//x and y
    }
    configDataBufferSize = CHARS_PER_DATA_VALUE * elements;
    configDataBufferSize += (SINGLE_TOUCH_MAX_GESTURES_COUNT +\
        DOUBLE_TOUCH_MAX_GESTURES_COUNT * 2);// line endings
    configDataBuffer = (char*)kzalloc(configDataBufferSize, GFP_KERNEL);

    for (i = 0; i < SINGLE_TOUCH_MAX_GESTURES_COUNT; i++) {
        offset += print_single_gesture_config(configDataBuffer + offset,\
            &singleTouchGestures[i], SINGLE_GESTURE_DATA);
    }
    for (i = 0; i < DOUBLE_TOUCH_MAX_GESTURES_COUNT; i++) {
        offset += print_single_gesture_config(configDataBuffer + offset,\
            &doubleTouchGestures[i].gesture1, DOUBLE_GESTURE_DATA);
        offset += print_single_gesture_config(configDataBuffer + offset,\
            &doubleTouchGestures[i].gesture2, DOUBLE_GESTURE_DATA);
    }
    memset(configDataBuffer + offset,0,configDataBufferSize - offset);

    memcpy(buf, configDataBuffer, configDataBufferSize);

    return configDataBufferSize;
}

static ssize_t data_store(struct kobject *kobj, struct kobj_attribute *attr,
                         const char *buf, size_t count) {
    int i, offset, status = -EINVAL;
    char *ptr = (char*)buf;

    do {
        offset = 0;
        singleTouchGesturesCount = 0;
        for (i =0; i < SINGLE_TOUCH_MAX_GESTURES_COUNT; i++) {
            if (0 != (status = gesture_from_string(ptr,\
                &singleTouchGestures[i]))) {
                break;
            }
            while (*ptr != LINE_FEED) {
                if ((*ptr == 0) || (ptr - buf > count)) {
                    status = -EFAULT;
                    break;
                }
                ptr++;
            }
            ptr++;

            if (singleTouchGestures[i].keyId) {
                singleTouchGesturesCount++;
            }

        }
        doubleTouchGesturesCount = 0;
        for (i = 0; i < DOUBLE_TOUCH_MAX_GESTURES_COUNT; i++) {
            if (0 != (status = gesture_from_string(ptr, &doubleTouchGestures[i].gesture1))){
                break;
            }
            while (*ptr != LINE_FEED) {
                if ((*ptr == 0) || (ptr - buf > count)) {
                    status = -EFAULT;
                    break;
                }
                ptr++;
            }
            ptr++;
            if (0 != (status = gesture_from_string(ptr, &doubleTouchGestures[i].gesture2))){
                break;
            }
            while (*ptr != LINE_FEED) {
                if ((*ptr == 0) || (ptr - buf > count)) {
                    status = -EFAULT;
                    break;
                }
                ptr++;
            }
            ptr++;

            /* to preserve table linearity, Double Touch KeyID is stored in sub-gestures. */
            if (doubleTouchGestures[i].gesture2.keyId == doubleTouchGestures[i].gesture1.keyId){
                doubleTouchGestures[i].keyId = doubleTouchGestures[i].gesture1.keyId;
            }
            else {
                LOG_E("%s bad KeyID for DoubleGesture[%d]\n", __func__, i);
                doubleTouchGestures[i].keyId = 0;
            }

            if (doubleTouchGestures[i].keyId) {
                doubleTouchGesturesCount++;
            }
        }

        if (status) {
            break;
        }

        gesturesCount = 0;
        for (i = 0; i < singleTouchGesturesCount; i++,gesturesCount++) {
            gestures[gesturesCount].status = GESTURE_STATUS_VALID;
            gestures[gesturesCount].singleGesture = &singleTouchGestures[i];
            gestures[gesturesCount].dtGesture = 0;
        }
        for (i = 0; i < doubleTouchGesturesCount; i++,gesturesCount++) {
            gestures[gesturesCount].status = GESTURE_STATUS_VALID;
            gestures[gesturesCount].singleGesture = 0;
            gestures[gesturesCount].dtGesture = &doubleTouchGestures[i];
        }

        gesture_input_dev_fill();

        status = count;
    } while (0);
    return status;
}

/*
 *  Enabled Key IDs node functionality
 */
static ssize_t key_enabled_show(struct kobject *kobj, struct kobj_attribute *attr,
                        char *buf) {
    int i, offset = 0;

    for (i = 0; i < BS_MAX_GESTURES_COUNT; i++) {
        if (bs_gestures[i].enabled && bs_gestures[i].key_id != 0) {
            offset += sprintf(buf + offset, "%d ", bs_gestures[i].key_id);
        }
    }
    if (bsStatus.is_scroll_enabled) {
        offset += sprintf(buf + offset, "%d %d ", KEY_BS_GESTURE_SCROLL_LEFT,\
          KEY_BS_GESTURE_SCROLL_RIGHT);
    }
    if (bsStatus.dt_status.is_dt_enabled) {
        offset += sprintf(buf + offset, "%d ", KEY_BS_GESTURE_DOUBLE_TAP);
    }
    if (bsStatus.fs_lock_enabled) {
        offset += sprintf(buf + offset, "%d ", KEY_FS_GESTURE_LOCK);
    }
    if (bsStatus.fs_unlock_enabled) {
        offset += sprintf(buf + offset, "%d ", KEY_FS_GESTURE_UNLOCK);
    }
    if (bsStatus.bs_lock_enabled) {
        offset += sprintf(buf + offset, "%d ", KEY_BS_GESTURE_LOCK);
    }
    if (bsStatus.bs_unlock_enabled) {
        offset += sprintf(buf + offset, "%d ", KEY_BS_GESTURE_UNLOCK);
    }

    offset += sprintf(buf + offset, "\n");

    return offset;
}

static ssize_t key_enabled_store(struct kobject *kobj, struct kobj_attribute *attr,
                         const char *buf, size_t count) {
    int status = -EINVAL;
    char *ptr_beg, *ptr_end;
    char i;
    char numbuf[CHARS_PER_DATA_VALUE + 1];
    int key_id;

    if (buf) {
        /* parse the line containing bs gestures key ids*/
        i = 0;
        ptr_end = ptr_beg = (char*)buf;
        while (*ptr_beg && i < BS_MAX_GESTURES_COUNT) {
            while (*ptr_end && *ptr_end != ' ') {
                ptr_end++;
            }
            memset(numbuf,0,sizeof(numbuf));
            memcpy(numbuf,ptr_beg,ptr_end - ptr_beg);
            key_id = 0;
            if (0 != (status = kstrtoint(numbuf, BASE_DECIMAL, &key_id))) {
                LOG_E("%s parse failed for %d : \"%s\"\n",__func__, i, numbuf);
                break;
            }
            bs_key_id_enable(key_id);
            ptr_beg = ++ptr_end;
            i++;
        }
    }

    return count;
}

/*
 *  Disabled Key IDs node functionality
 */
static ssize_t key_disabled_show(struct kobject *kobj, struct kobj_attribute *attr,
                        char *buf) {
    int i, offset = 0;

    for (i = 0; i < BS_MAX_GESTURES_COUNT; i++) {
        if (!bs_gestures[i].enabled && bs_gestures[i].key_id != 0) {
            offset += sprintf(buf + offset, "%d ", bs_gestures[i].key_id);
        }
    }
    if (!bsStatus.is_scroll_enabled) {
        offset += sprintf(buf + offset, "%d %d ", KEY_BS_GESTURE_SCROLL_LEFT,\
          KEY_BS_GESTURE_SCROLL_RIGHT);
    }
    if (!bsStatus.dt_status.is_dt_enabled) {
        offset += sprintf(buf + offset, "%d ", KEY_BS_GESTURE_DOUBLE_TAP);
    }
    if (!bsStatus.fs_lock_enabled) {
        offset += sprintf(buf + offset, "%d ", KEY_FS_GESTURE_LOCK);
    }
    if (!bsStatus.fs_unlock_enabled) {
        offset += sprintf(buf + offset, "%d ", KEY_FS_GESTURE_UNLOCK);
    }
    if (!bsStatus.bs_lock_enabled) {
        offset += sprintf(buf + offset, "%d ", KEY_BS_GESTURE_LOCK);
    }
    if (!bsStatus.bs_unlock_enabled) {
        offset += sprintf(buf + offset, "%d ", KEY_BS_GESTURE_UNLOCK);
    }
    offset += sprintf(buf + offset, "\n");

    return offset;
}

static ssize_t key_disabled_store(struct kobject *kobj, struct kobj_attribute *attr,
                         const char *buf, size_t count) {
    int status = -EINVAL;
    char *ptr_beg, *ptr_end;
    char i;
    char numbuf[CHARS_PER_DATA_VALUE + 1];
    int key_id;

    if (buf) {
        /* parse the line containing bs gestures key ids*/
        i = 0;
        ptr_end = ptr_beg = (char*)buf;
        while (*ptr_beg && i < BS_MAX_GESTURES_COUNT) {
            while (*ptr_end && *ptr_end != ' ') {
                ptr_end++;
            }
            memset(numbuf,0,sizeof(numbuf));
            memcpy(numbuf,ptr_beg,ptr_end - ptr_beg);
            key_id = 0;
            if (0 != (status = kstrtoint(numbuf, BASE_DECIMAL, &key_id))) {
                LOG_E("%s parse failed for %d : \"%s\"\n",__func__, i, numbuf);
                break;
            }
            bs_key_id_disable(key_id);
            ptr_beg = ++ptr_end;
            i++;
        }
    }

    return count;
}

/*
 * Gesturefilter enable/disable node functions
 */
static ssize_t filter_show(struct kobject *kobj, struct kobj_attribute *attr,
                        char *buf) {
    return sprintf(buf, "%d\n", filterEnabled);
}

static ssize_t filter_store(struct kobject *kobj, struct kobj_attribute *attr,
                         const char *buf, size_t count) {
    sscanf(buf, "%d", &filterEnabled);
    return count;
}

static inline int sensor_threshold(s16 data, int threshold)
{
	return data > threshold ? data : 0;
}

static int sensor_calc_fs_sum(s16 *pdata16, int len, int threshold)
{
	int row;
	s16 *p;
	int sum = 0;

	for (row = 14; row < CY_FS_ROW-5; row++) {
		p = pdata16+row*CY_FS_COLUMN;
		sum += sensor_threshold(*p, threshold);
		*p= -5;

		p = pdata16+row*CY_FS_COLUMN + 1;
		sum += sensor_threshold(*p, threshold);
		*p = -5;

		p = pdata16+row*CY_FS_COLUMN + CY_FS_COLUMN-1;
		sum += sensor_threshold(*p, threshold);
		*p = -5;

		p = pdata16+row*CY_FS_COLUMN + CY_FS_COLUMN-2;
		sum += sensor_threshold(*p, threshold);
		*p= -5;
	}
	return sum;
}

static int sensor_calc_bs_sum(s16 *pdata16, int len, int threshold)
{
	int sum = 0;
	int offset;
	s16 *p;

	for (p = pdata16 + CY_BS_COLUMN, offset = 9; offset < CY_BS_COLUMN; offset++) {
		sum += sensor_threshold(*(p+offset), threshold);
		*(p+offset) = 0;
	}

	for (p = pdata16 + CY_BS_COLUMN*(CY_BS_ROW - 2), offset = 7; offset < CY_BS_COLUMN; offset++) {
		sum += sensor_threshold(*(p+offset), threshold);
		*(p+offset) = -1;
		sum += sensor_threshold(*(p+offset + CY_BS_COLUMN), threshold);
		*(p+offset + CY_BS_COLUMN) = -2;
	}

	return sum;
}

static int sensor_calc_fs_middle_area(s16 *pdata16, int len, int threshold)
{
	int sum = 0;
	int row, column;
	s16 *p = pdata16;

	for (row = 2; row < CY_FS_ROW/2; row++) {
		p = pdata16 + row*CY_FS_COLUMN;
		for (column = 2; column < CY_FS_COLUMN-2; column++) {
			sum += sensor_threshold(*(p + column), threshold);
			*(p+column) = -7;
		}
	}
	return sum;
}

static int sensor_calc_bs_middle_area(s16 *pdata16, int len, int threshold)
{
	int sum = 0;
	int row, column;
	s16 *p = pdata16;

	for (row = 2; row < CY_BS_ROW - 2; row++) {
		p = pdata16 + row*CY_BS_COLUMN;
		for (column = 2; column < CY_BS_COLUMN/2; column++) {
			sum += sensor_threshold(*(p + column), threshold);
			*(p+column) = -5;
		}
	}
	return sum;
}
#if 0
static int sensor_dump(s16 *pdata16, int rows, int cols)
{
	int i;
	printk("\nrows=%d, cols=%d\n", rows, cols);
	for (i = 0; i < cols*rows; i++) {
		if ((i % cols) == 0)
			printk("\n");
		printk("%03d ", pdata16[i]);
	}
	return 0;
}
#endif

static void sensor_normalize_sum(int *front_sum, int *back_sum, int fs_len, int bs_len)
{
	//	printk("\nfront_sum=%d, back_sum=%d\n", *front_sum, *back_sum);
	// Back touch has less touch sensors -> need to correct ratio
	*back_sum = *back_sum*fs_len/bs_len;
	// Normalisation because of non-equal sensor data for front and back:
        // max bs data~2300, fs data=600, ratio=4
	*front_sum = *front_sum * 3;
}

int palmed_screen(int front_sum, int back_sum) {
	if (abs(front_sum - back_sum) < 500)
		return -1;
	if (abs(front_sum) >= abs(back_sum))
		return 0;
	else
		return 1;
	return -1;
}

/*
 * Palmed screen detection
 */
static ssize_t palmed_screen_show(struct kobject *kobj, struct kobj_attribute *attr,
                        char *buf) {
	u16 read_offset, read_count;
	u8 config;
	u16 actual_read_len;
	int screen = -1;
	int front_sum = 0, back_sum = 0;
	const int fs_data_len = CY_FS_ROW*CY_FS_COLUMN;
	const int bs_data_len = CY_BS_ROW*CY_BS_COLUMN;

	read_offset = 0;
	read_count = fs_data_len;
	cyttsp5_read_sensor_data(CYTTSP5_DEVICE_ACCESS_NAME, CY_FRONT_CORE_ID,
				 1, read_offset, read_count, CY_MUT_DIFF, NULL,
				 &config, &actual_read_len, fs_sensor_data);

	//	sensor_dump((s16 *)fs_sensor_data, CY_FS_ROW, CY_FS_COLUMN);
	front_sum = sensor_calc_fs_sum((s16 *)fs_sensor_data, fs_data_len, 50);
	//	sensor_dump((s16 *)fs_sensor_data, CY_FS_ROW, CY_FS_COLUMN);

	read_offset = 0;
	read_count = bs_data_len;
	cyttsp5_read_sensor_data(CYTTSP5_DEVICE_ACCESS_NAME, CY_BACK_CORE_ID,
				 1, read_offset, read_count, CY_MUT_DIFF, NULL,
				 &config, &actual_read_len, bs_sensor_data);

	//	sensor_dump((s16 *)bs_sensor_data, CY_BS_ROW, CY_BS_COLUMN);
	back_sum = sensor_calc_bs_sum((s16 *)bs_sensor_data, bs_data_len, 150);
	//	sensor_dump((s16 *)bs_sensor_data, CY_BS_ROW, CY_BS_COLUMN);

	sensor_normalize_sum(&front_sum, &back_sum, fs_data_len, bs_data_len);
	screen = palmed_screen(front_sum, back_sum);

	//printk(KERN_INFO "%s: Palmed screen - 1st alg=%d\n", __func__, screen);

	if (screen == -1) {
		front_sum = sensor_calc_fs_middle_area((s16 *)fs_sensor_data, fs_data_len, 50);
		back_sum = sensor_calc_bs_middle_area((s16 *)bs_sensor_data, bs_data_len, 150);

		//sensor_dump((s16 *)fs_sensor_data, CY_FS_ROW, CY_FS_COLUMN);
		//sensor_dump((s16 *)bs_sensor_data, CY_BS_ROW, CY_BS_COLUMN);

		sensor_normalize_sum(&front_sum, &back_sum, fs_data_len, bs_data_len);
		screen = palmed_screen(front_sum, back_sum);

		//printk(KERN_INFO "%s: Palmed screen - 2nd alg=%d\n", __func__, screen);
	}
	printk(KERN_INFO "%s: Palmed screen=%d\n", __func__, screen);
	return sprintf(buf,"%d", screen);
}

static ssize_t palmed_screen_store(struct kobject *kobj, struct kobj_attribute *attr,
                         const char *buf, size_t count) {
    return count;
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

    ptr = &gesture_config_nodes[CONFIG_NODE_ORIENTATION];
    ptr->attribute.show = orientation_show;
    ptr->attribute.store = orientation_store;

    ptr = &gesture_config_nodes[CONFIG_NODE_LOGLVL];
    ptr->attribute.show = loglvl_show;
    ptr->attribute.store = loglvl_store;

    ptr = &gesture_config_nodes[CONFIG_NODE_DATA];
    ptr->attribute.show = data_show;
    ptr->attribute.store = data_store;

    ptr = &gesture_config_nodes[CONFIG_NODE_KEYS_ENABLED];
    ptr->attribute.show = key_enabled_show;
    ptr->attribute.store = key_enabled_store;

    ptr = &gesture_config_nodes[CONFIG_NODE_KEYS_DISABLED];
    ptr->attribute.show = key_disabled_show;
    ptr->attribute.store = key_disabled_store;

    ptr = &gesture_config_nodes[CONFIG_NODE_FILTER];
    ptr->attribute.show = filter_show;
    ptr->attribute.store = filter_store;

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

    init_node_data();

    do {
        /* First setup a sysfs kobject for front screen gesture driver */
        gesture_config_kobj =\
            kobject_create_and_add(gesture_config_nodes[CONFIG_NODE_DIRECTORY].name,  kernel_kobj);

        if (!gesture_config_kobj) {
            break;
        }

        for (i = CONFIG_NODE_ENABLED; i < CONFIG_NODES_COUNT; i++) {
            if (sysfs_create_file(gesture_config_kobj,\
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
