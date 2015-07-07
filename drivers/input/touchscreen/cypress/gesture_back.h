/*
 * Gesture recognition driver
 *
 * Copyright (c) 2013,2014 YotaDevices.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#ifndef GESTURE_BACK_H_FILE
#define GESTURE_BACK_H_FILE

//#define BS_FORWARD_RAW_KEY_EVENTS

/* Titanium start */
#define KEY_BS_GESTURE_MAXIMUM KEY_BS_GESTURE_SCROLL_RIGHT
#define KEY_BS_GESTURE_MINIMUM KEY_BS_GESTURE_SWIPE_RIGHT

/* Back Touch  area  */
#define BS_ZONE_LEFT_TOP_X (0)
#define BS_ZONE_LEFT_TOP_Y (0)
#define BS_ZONE_RIGHT_BOTTOM_X (540)
#define BS_ZONE_RIGHT_BOTTOM_Y (960)
//#define BS_ZONE_RIGHT_BOTTOM_X (127)
//#define BS_ZONE_RIGHT_BOTTOM_Y (295)


/* BS Long tap timer values */
#define BS_LONG_TOUCH_MIN_TIME (500)
#define BS_LONG_TOUCH_MAX_COUNT (3)

/*
 * BS key info container
 */
typedef struct key_info {
    int code;               // key code
    int value;              // key value
    bool changed;           // info changed flag
} key_info_t;

/*
 * Gesture start point
 */
typedef struct Gesture_st_point {
    long x ;
    long y ;
} Gesture_st_point_t;

bool back_gesture_driver_event(unsigned int, unsigned int, int);
void bs_load_gestures(void);
void bs_init_driver(void);
 int bs_virtual_input_device_init(void);
void bs_virtual_input_device_exit(void);

/* Titanium end */


/*
 * BS key bits
 */
#define KEY7 (1)
#define KEY6 (2)
#define KEY5 (4)
#define KEY4 (8)
#define KEY3 (16)
#define KEY2 (32)
#define KEY1 (64)
#define KEY0 (128)

/*
 * BS maximum parts in gesture
 */
#define BS_MAX_GESTURE_PARTS (2)

/*
 * BS maximum gesture count
 */
#define BS_MAX_GESTURES_COUNT (7)

/*
 * BS long touch time
 */
#define BS_LONG_TOUCH_TIME (500)

/*
 * Double click delay between first and second tap
 */
#define BS_DC_TIME (400)

/*
 * Maximum parts count for user input
 */
#define BS_INPUT_PARTS_MAX_COUNT (2)

/*
 * Join timer delay
 */
#define BS_JOIN_TIMER_DELAY (200)

/*
 * Join timer delay for swipe scroll
 */
#define BS_SWIPE_SCROLL_JOIN_TIMER_DELAY (50)

/*
 * TAP gesture max size
 */
#define BS_TAP_GESTURE_MAX_SIZE (3)

/*
 * Interval between first and second tap in double Tap gesture
 */
#define BS_DT_INTERVAL (2)

/*
 * Maximum key count for gesture
 */
#define BS_GESTURE_MAXIMUM_KEY_COUNT (6)

/*
 * BS gesture recognition state
 */
typedef enum bs_gesture_state {
    BS_GESTURE_STATE_NOT_RECOGNIZED = 0,
    BS_GESTURE_STATE_NOT_GESTURE = 1,
    BS_GESTURE_STATE_GESTURE = 2,
    BS_GESTURE_STATE_NOT_ACTIVE =3,
    BS_GESTURE_STATE_SCROLL = 4,
} bs_gesture_state_t;

/*
 * Back screen combined key state
 */
typedef enum bs_keys_state {
    BS_KEYS_STATE_DOWN = 0,
    BS_KEYS_STATE_CHANGED = 1,
    BS_KEYS_STATE_UP = 2,
    BS_KEYS_STATE_NOT_PRESSED = 3,
} bs_keys_state_t;

/*
 * Back screen move directions
 */
typedef enum direction {
    BS_DIR_RIGHT = 0,
    BS_DIR_LEFT = 1,
    BS_DIR_NOT_CHANGED = 2,
} direction_t;

/*
 * Double tap recognition state
 */
typedef enum double_tap_state {
    BS_DT_STATE_START = 0,
    BS_DT_STATE_FIRST_TAP = 1,
} double_tap_state_t;

/*
 * Swipe scroll gesture state
 */
typedef enum bs_scroll_state {
    BS_SCROLL_STATE_START = 0,
    BS_SCROLL_STATE_LEFT = 1,
    BS_SCROLL_STATE_RIGHT= 2,
    BS_SCROLL_STATE_NOT_GESTURE = 3,
} bs_scroll_state_t;

/*
 * User input part
 */
typedef struct bs_input_part {
    direction_t dir;    // part direction
    char status_start;  // part start status
    char status_end;    // part end status
} bs_input_part_t;

/*
 * Double tap status
 */
typedef struct bs_double_tap_status {
    double_tap_state_t dt_state;   // double click state
    bool is_dt_enabled;            // Is double tap gesture enabled
    int first_tap_center;          // First tap center
}bs_double_tap_status_t;

/*
 * Back screen gesture engine state structure
 */
typedef struct bs_gesture_status {
    char status;                            // current touch status
    char prev_status;                       // previous touch status
    char start_status;                      // start touch status
    bs_gesture_state_t recognition_state;   // gesture recognition state
    bs_keys_state_t key_state;              // current combined key state
    direction_t dir;                        // current movement direction
    struct timeval startTime;               // start time
    struct timeval endTime;                 // end time
    bool needSendKeyDown;            // do we need send key down at the gesture recognition moment
    int delayedKeyUP;                // save key Id here after sent key down
    int action;                      // recognized gesture key Id
    unsigned int tracking_id;        // tracking id
    bs_input_part_t input_parts[BS_INPUT_PARTS_MAX_COUNT]; // user input parts
    int current_input_part;                                // current input part
    bs_double_tap_status_t dt_status;                      // double tap status
    bs_scroll_state_t scroll_state;                        // swipe scroll state
    bool is_scroll_enabled;                                // is swipe scroll enabled or not
    bool fs_lock_enabled;            //fs lock gesture
    bool bs_lock_enabled;            //bs lock gesture
    bool bs_unlock_enabled;          //bs unlock gesture
    bool fs_locked;                  // fs unlock gesture
    bool fs_unlock_enabled;       // fs unlock gesture
    bool sendLockEvent;           //fs lock event
} bs_gesture_status_t;

/*
 * BS tocuh area
 */

typedef struct bs_gesture_touch_area {
    char h_bit;                     // high bit of touch area
    char l_bit;                     // low bit of touch area
} bs_gesture_touch_area_t;

/*
 * BS gesture part (for multipart gestures ex. left-right-left)
 */
typedef struct bs_gesture_part {
    direction_t dir;            // gesture part direction
    char min_size;              // part min size
    char max_size;              // part max size
} bs_gesture_part;


/*
 * BS gesture definition structure
 */
typedef struct bs_gesture {
    bs_gesture_touch_area_t start_area;             // start area
    bs_gesture_part parts[BS_MAX_GESTURE_PARTS];    // gesture parts
    char parts_count;                               // parts count
    int key_id;                                     // gesture key Id
    bool isLongTapGesture;                          // long tap gesture flag
    bool isTapGesture;               // tap gesture flag
    int minTime;                     // min time value for gesture in ms
    int maxTime;                     // max time value for gesture in ms
    bool needSendKeyDown;            // do we need send key down at the gesture recognition moment
    bool enabled;                    // gesture enabled or not
} bs_gesture_t;


#endif
