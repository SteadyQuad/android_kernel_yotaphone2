/*
 * Gesture recognition driver
 *
 * Copyright (c) 2013,2014, 2015 YotaDevices.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#ifndef GESTURE_DRIVER_H_FILE
#define GESTURE_DRIVER_H_FILE

#include "gesture_config.h"

#define FRONT_TOUCH_ID (0)
#define BACK_TOUCH_ID (1)

/* Touch zones definition  */
#define FRONT_LEFT_TOP_X (0)
#define FRONT_LEFT_TOP_Y (0)
#define FRONT_RIGHT_BOTTOM_X (1080)
#define FRONT_RIGHT_BOTTOM_Y (1920)

#define BACK_LEFT_TOP_X (0)
#define BACK_LEFT_TOP_Y (0)
#define BACK_RIGHT_BOTTOM_X (540)
#define BACK_RIGHT_BOTTOM_Y (960)

#define CY_FS_ROW    (28)
#define CY_FS_COLUMN (16)
#define CY_BS_ROW    (13)
#define CY_BS_COLUMN (22)
#define CY_FS_MUT_SIZE (CY_FS_ROW * CY_FS_COLUMN)
#define CY_BS_MUT_SIZE (CY_BS_ROW * CY_BS_COLUMN)
#define CY_FS_SELFCAP_SIZE CY_FS_COLUMN
#define CY_BS_SELFCAP_SIZE CY_BS_COLUMN

/* Timer value to send LOCK event */
#define LOCK_EVENT_TIMER (200)

/* Log levels */
typedef enum log_level {
    LOG_LEVEL_ERROR = 0,
    LOG_LEVEL_DEBUG = 1,
    LOG_LEVEL_VERBOSE = 2,
} log_level_t;

/* Runtime logging defines */
extern log_level_t log_level;
extern atomic_t front_touch_locked;
extern atomic_t back_touch_locked;

#ifdef DEBUG
#define LOG_V(fmt, args...) if(log_level == LOG_LEVEL_VERBOSE ) \
    pr_info(KBUILD_MODNAME ": " fmt, ## args)
#define LOG_D(fmt, args...) if(log_level >= LOG_LEVEL_DEBUG ) \
    pr_info(KBUILD_MODNAME ": " fmt, ## args)
#define LOG_E(fmt, args...) pr_err(KBUILD_MODNAME ": " fmt, ## args)
#else
#define LOG_V(fmt, args...)
#define LOG_D(fmt, args...)
#define LOG_E(fmt, args...)
#endif

extern void send_gesture_event(unsigned int key);
extern void driver_touch_lock(int touch_id);
extern void driver_touch_unlock(int touch_id);
extern int driver_is_touch_locked(int touch_id);
extern int get_palmed_screen(int algorithm);
#endif
