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

#ifndef GESTURE_CONFIG_H_FILE
#define GESTURE_CONFIG_H_FILE

#define GESTURE_PROCFS_NODE_NAME_LEN (25)

#define CONFIG_NODE_PERMISSIONS (0644)

/* Proc FS data node table IDs [0..2] */
typedef enum {
    SINGLE_GESTURE_DATA,
    DOUBLE_GESTURE_DATA,
    COMMON_VALUES_DATA,
} data_table_id_t;

/* Proc FS node information */
typedef struct {
    struct kobj_attribute attribute;
    const char name[GESTURE_PROCFS_NODE_NAME_LEN];
} gesture_config_node_t;

/* Indexes of proc node entries in proc node array [0..7] */
typedef enum {
    CONFIG_NODE_DIRECTORY,
    CONFIG_NODE_ENABLED,
    CONFIG_NODE_LOGLVL,
    CONFIG_NODE_FRONT_SENSORS,
    CONFIG_NODE_BACK_SENSORS,
    CONFIG_NODE_KEYS_ENABLED,
    CONFIG_NODE_KEYS_DISABLED,
    CONFIG_NODE_PALMED_SCREEN,
    CONFIG_NODES_COUNT
} gesture_config_node_index_t;

typedef enum {
    FRONT_GESTURE_LOCK_3FINGER = 0x1,
    FRONT_GESTURE_UNLOCK_SWIPE = 0x2,
    FRONT_GESTURE_UNLOCK_TAPTAP = 0x4,
    BACK_GESTURE_LOCK_3FINGER = 0x8,
    BACK_GESTURE_UNLOCK_SWIPE = 0x10,
    BACK_GESTURE_UNLOCK_TAPTAP = 0x20,
    BACK_DEEPSLEEP = 0x40,
    BACK_GESTURE_UNLOCK_TAPTAP_TOUCHDOWN = 0x80,
    GESTURE_TYPE_LAST = BACK_GESTURE_UNLOCK_TAPTAP_TOUCHDOWN,
} gesture_type_t;
#define GESTURE_TYPE_MASK 0xFF
#define FRONT_UNLOCK_GESTURE_MASK (FRONT_GESTURE_UNLOCK_SWIPE | FRONT_GESTURE_UNLOCK_TAPTAP)
#define BACK_UNLOCK_GESTURE_MASK (BACK_GESTURE_UNLOCK_SWIPE | BACK_GESTURE_UNLOCK_TAPTAP \
                                   | BACK_GESTURE_UNLOCK_TAPTAP_TOUCHDOWN)
#define BACK_GESTURE_MASK (BACK_UNLOCK_GESTURE_MASK | BACK_DEEPSLEEP)


int gesture_config_init(void);
void gesture_config_terminate(void);
unsigned int get_unlock_gesture_enabled(int touch_id);
unsigned int get_lock_gesture_enabled(int touch_id);
unsigned int is_deepsleep_enabled(int touch_id);

#endif /* GESTURE_CONFIG_H_FILE */
