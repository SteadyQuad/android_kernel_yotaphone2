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

#ifndef GESTURE_CONFIG_H_FILE
#define GESTURE_CONFIG_H_FILE

#define GESTURE_PROCFS_NODE_NAME_LEN (25)

#define LINE_FEED (0x0A)

#define CONFIG_NODE_PERMISSIONS (0644)

/* Proc FS data node table IDs [0..2] */
typedef enum data_table_id {
    SINGLE_GESTURE_DATA,
    DOUBLE_GESTURE_DATA,
    COMMON_VALUES_DATA,
} data_table_id_t;

/* Indexes of gesture config parameters [0..20] */
typedef enum gesture_data_index_table {
    PARAM_INDEX_DATA_TYPE,
    PARAM_INDEX_START_POINT_AREA_LEFT_UP_X,
    PARAM_INDEX_START_POINT_AREA_LEFT_UP_Y,
    PARAM_INDEX_START_POINT_AREA_RIGHT_BOTTOM_X,
    PARAM_INDEX_START_POINT_AREA_RIGHT_BOTTOM_Y,
    PARAM_INDEX_END_POINT_AREA_LEFT_UP_X,
    PARAM_INDEX_END_POINT_AREA_LEFT_UP_Y,
    PARAM_INDEX_END_POINT_AREA_RIGHT_BOTTOM_X,
    PARAM_INDEX_END_POINT_AREA_RIGHT_BOTTOM_Y,
    PARAM_INDEX_MIN_DX,
    PARAM_INDEX_MIN_DY,
    PARAM_INDEX_MAX_DX,
    PARAM_INDEX_MAX_DY,
    PARAM_INDEX_MIN_TIME,
    PARAM_INDEX_MAX_TIME,
    PARAM_INDEX_KEY_ID,
    PARAM_INDEX_NEED_SEND_KEY_DOWN,
    PARAM_INDEX_IS_TAP_GESTURE,
    PARAM_INDEX_IS_DETECT_ON_TOUCH_UP,
    PARAM_INDEX_TOUCH_ACC_POINTS_COUNT,
    FIXED_PARAMS_PER_GESTURE,
} gesture_data_index_table_t;

/* Proc FS node information */
typedef struct gesture_config_node {
    struct kobj_attribute attribute;
    const char name[GESTURE_PROCFS_NODE_NAME_LEN];
} gesture_config_node_t;

/* Indexes of proc node entries in proc node array [0..7] */
typedef enum gesture_config_node_index {
    CONFIG_NODE_DIRECTORY,
    CONFIG_NODE_ENABLED,
    CONFIG_NODE_ORIENTATION,
    CONFIG_NODE_LOGLVL,
    CONFIG_NODE_DATA,
    CONFIG_NODE_KEYS_ENABLED,
    CONFIG_NODE_KEYS_DISABLED,
    CONFIG_NODE_FILTER,
    CONFIG_NODE_PALMED_SCREEN,
    CONFIG_NODES_COUNT,
} gesture_config_node_index_t;

int gesture_config_init(void);
void gesture_config_terminate(void);

#endif /* GESTURE_CONFIG_H_FILE */
